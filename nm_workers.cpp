// nm_workers.cpp — NavMesh worker pool, hooks, job pipeline

#include "nm_workers.h"

#if NMCACHE_STEP >= 1


// Worker pool state
HANDLE          g_workerHandles[NAVMESH_WORKER_COUNT] = {};
volatile long   g_workerShutdown     = 0;
HANDLE          g_jobEvent           = NULL;
uintptr_t       g_navMeshGen         = 0;
int             g_workerSavedPriority[NAVMESH_WORKER_COUNT] = {};

// Original workBuffer pointer (set once, used to restore NMG+256 after clone swap)
static uintptr_t g_originalWorkBuf = 0;


// TLS indices (file-static)
static DWORD g_scratchTlsIndex     = TLS_OUT_OF_INDEXES;
static DWORD g_releaseFlagTlsIndex = TLS_OUT_OF_INDEXES;
static DWORD g_clonedWBTlsIndex    = TLS_OUT_OF_INDEXES;
static DWORD g_deepCopyTlsIndex    = TLS_OUT_OF_INDEXES;
static DWORD g_isWorkerTlsIndex    = TLS_OUT_OF_INDEXES;

static inline bool IsWorkerThread()
{
	if (g_isWorkerTlsIndex == TLS_OUT_OF_INDEXES) return false;
	return TlsGetValue(g_isWorkerTlsIndex) != NULL;
}


#if NMCACHE_STEP >= 4
void InitScratchTLS()
{
	g_scratchTlsIndex = TlsAlloc();
	g_releaseFlagTlsIndex = TlsAlloc();
	g_deepCopyTlsIndex = TlsAlloc();
	g_isWorkerTlsIndex = TlsAlloc();
	g_clonedWBTlsIndex = TlsAlloc();
	LogDebug("[ZoneOpt] Scratch TLS allocated");
}
#endif // NMCACHE_STEP >= 4

static void EnsureScratchTLS()
{
	if (g_scratchTlsIndex == TLS_OUT_OF_INDEXES) return;
	if (TlsGetValue(g_scratchTlsIndex)) return;

	WorkerScratch* ws = (WorkerScratch*)fn_gameNew(sizeof(WorkerScratch));
	if (!ws) return;

	ws->buffer = (void**)fn_gameNewArr(4096 * 8);
	ws->capacity = 4096;
	TlsSetValue(g_scratchTlsIndex, ws);
}


// ---- Havok FLA allocator thread-safety: 4 MinHook CS patches ----
// hkFreeListAllocator compiled with HK_CONFIG_SINGLE_THREADED (CS enter/leave stripped).
// Restore thread-safety by hooking 4 unlocked methods that mutate shared state,
// wrapping them with the FLA's own built-in CS at FLA+0x10.
// All threads, uniform locking (no IsWorkerThread gating — avoids asymmetric deadlocks).
// CS is reentrant: nested calls from hooked→locked methods safe.

static CRITICAL_SECTION* g_flaCS = NULL;

typedef void    (__fastcall *flaResetPeak_t)(__int64 thisAlloc);
typedef __int64 (__fastcall *flaCanAlloc_t)(__int64 thisAlloc, __int64 numBytes);
typedef __int64 (__fastcall *flaBufRealloc_t)(__int64 thisAlloc, __int64 pold, int oldNumBytes, int* reqInOut);
typedef __int64 (__fastcall *flaGC_t)(__int64 thisAlloc);

static flaResetPeak_t  orig_flaResetPeak  = NULL;
static flaCanAlloc_t   orig_flaCanAlloc   = NULL;
static flaBufRealloc_t orig_flaBufRealloc = NULL;
static flaGC_t         orig_flaGC         = NULL;

static void __fastcall hook_flaResetPeak(__int64 thisAlloc)
{
	EnterCriticalSection(g_flaCS);
	orig_flaResetPeak(thisAlloc);
	LeaveCriticalSection(g_flaCS);
}

static __int64 __fastcall hook_flaCanAlloc(__int64 thisAlloc, __int64 numBytes)
{
	EnterCriticalSection(g_flaCS);
	__int64 r = orig_flaCanAlloc(thisAlloc, numBytes);
	LeaveCriticalSection(g_flaCS);
	return r;
}

static __int64 __fastcall hook_flaBufRealloc(__int64 thisAlloc, __int64 pold, int oldNumBytes, int* reqInOut)
{
	EnterCriticalSection(g_flaCS);
	__int64 r = orig_flaBufRealloc(thisAlloc, pold, oldNumBytes, reqInOut);
	LeaveCriticalSection(g_flaCS);
	return r;
}

static __int64 __fastcall hook_flaGC(__int64 thisAlloc)
{
	EnterCriticalSection(g_flaCS);
	__int64 r = orig_flaGC(thisAlloc);
	LeaveCriticalSection(g_flaCS);
	return r;
}

// ---- edgeProcess guard (clone finalize) ----
// finalize iterates wb+520 (overrideSettings) entries, calling edgeProcess per
// entry. Each 240-byte entry has:
//   +0:  hkRefPtr to section object (refcount decrement)
//   +80: embedded SimplificationSettings (dtor frees hkStringPtr + m_userVertices)
//
// SimplificationSettings is a plain struct (no RTTI, no hkReferencedObject).
// Havok allocator doesn't zero memory. OverrideSettings entries allocated during
// cloned processJobAlt may have uninitialized fields — garbage pointers that
// cause blockFree/bufFree on invalid addresses, and garbage hkRefPtrs that
// trigger refcount operations on unmapped memory.
//
// Fix: hook edgeProcess and skip the entire per-entry cleanup during clone
// processing. Valid entries' allocations leak (~100 bytes/entry, ~1KB/MISS,
// bounded). Refcount skip means section objects keep +1 ref until zone unload
// (harmless). processJobCS serializes MISSes so the global flag is safe.

typedef void (__fastcall *edgeProcess_t)(void* entry);
static edgeProcess_t orig_edgeProcess = NULL;
static volatile long g_cloneProcessing = 0;

static void __fastcall hook_edgeProcess(void* entry)
{
	if (InterlockedCompareExchange(&g_cloneProcessing, 0, 0))
		return;  // clone context: skip all cleanup for this entry
	orig_edgeProcess(entry);
}


// SEH-safe FLA probe (separate function — MSVC 2010 can't mix __try with C++ destructors).
// Finds hkFreeListAllocator via TLS router[9]->+0x08, validates vtable + CS.
static uintptr_t ProbeFLAInstance()
{
	DWORD tlsIdx = *(DWORD*)(gameBase + RVA_HAVOK_TLS_INDEX);
	uintptr_t* router = (uintptr_t*)TlsGetValue(tlsIdx);
	if (!router) return 0;

	uintptr_t fla = 0;
	__try
	{
		// router[9] = hkThreadMemory (m_temp), +0x08 = m_memory → hkFreeListAllocator
		uintptr_t tm = router[9];
		if (!tm || tm < 0x10000) return 0;
		fla = *(uintptr_t*)(tm + 0x08);
		if (!fla || fla < 0x10000) return 0;

		// Validate: primary vtable must match known FLA vtable RVA
		uintptr_t vt = *(uintptr_t*)fla;
		if (vt != gameBase + RVA_FLA_VTABLE) return 0;

		// Validate: CS at FLA+0x10 is functional
		if (!TryEnterCriticalSection((CRITICAL_SECTION*)(fla + 0x10)))
			return 0;
		LeaveCriticalSection((CRITICAL_SECTION*)(fla + 0x10));
	}
	__except(EXCEPTION_EXECUTE_HANDLER) { return 0; }

	return fla;
}

static void InstallHavokHeapHooks()
{
	if (InterlockedCompareExchange(&flaHooksInstalled, 1, 0) != 0)
		return;

	uintptr_t fla = ProbeFLAInstance();
	if (!fla)
	{
		LogMsg("[ZoneOpt] FLA probe failed — allocator hooks NOT installed");
		InterlockedExchange(&flaHooksInstalled, 0);
		return;
	}

	g_flaCS = (CRITICAL_SECTION*)(fla + 0x10);
	InterlockedExchange(&csProbeFLAPtrLo, (long)(fla & 0xFFFFFFFF));
	InterlockedExchange(&csProbeFLAPtrHi, (long)((fla >> 32) & 0xFFFFFFFF));

	// FLA hooks and edgeProcess hook DISABLED — both cause latent CRT heap corruption
	// via MinHook trampoline issues. The 6 main FLA methods (blockAlloc, blockFree, etc.)
	// already have built-in CS. The 4 unprotected methods (resetPeak, canAlloc, bufRealloc,
	// GC) are low-frequency and not called during HIT processing.
	int installed = 0;
	{
		std::ostringstream ss;
		ss << "[ZoneOpt] FLA CS hooks: DISABLED (trampoline safety)"
		   << " FLA=" << (void*)fla << " CS=" << (void*)g_flaCS;
		LogMsg(ss.str());
	}

	// DIAG: edgeProcess hook disabled — not needed without cloning
	LogMsg("[ZoneOpt] edgeProcess guard: DISABLED (no cloning)");

	InterlockedExchange(&flaHooksInstalled, 2);
}


// DIAG: bypass WB cloning to isolate clone vs threading crashes
static bool nmSkipClone = false;

// ---- WorkBuffer cloning ----

// Construct a fresh hkaiNavMeshGenerationSettings via the real constructor,
// then copy only scalar config from the original. No memcpy of the full object.
//
// The constructor (0xDD99D0) properly initializes:
//   - hkReferencedObject header: vtable, m_memSizeAndFlags=1, m_referenceCount=0xFFFF (-1, immortal)
//   - All hkArrays: {NULL, 0, DONT_DEALLOCATE} (empty, non-owning)
//   - Sub-structs: edgeMatchingParams (+76), sub-structs (+144, +192), SimplificationSettings (+336)
//   - hkStringPtr (+512): initialized via sub_BCC9B210
//
// We then copy scalar config regions from the original (generation parameters,
// thresholds, tuning values). Arrays stay at constructor defaults — processJobAlt
// rebuilds carvers/painters/overrideSettings during generation. materialMap is
// shared from the original with DONT_DEALLOCATE.
//
// This eliminates ALL stale-metadata issues: no mystery bytes, no copied refcounts,
// no shared hkStringPtr/hkArray ownership, no adjacent heap data.
static void* ConstructFreshSettings(uintptr_t origWB)
{
	void* mem = HavokTlsAlloc(WB_OBJECT_SIZE);
	if (!mem) return NULL;

	void* fresh = fn_settingsCtor(mem);
	if (!fresh) { HavokTlsFree(mem, WB_OBJECT_SIZE); return NULL; }
	char* f = (char*)fresh;
	char* o = (char*)origWB;

	// Copy scalar config regions (skip hkReferencedObject header, all hkArrays,
	// hkStringPtr, and sub-struct internal state that constructor handles)

	// +16..+75: generation config (quantizationGridSize, up vector, agent dims, etc.)
	memcpy(f + 16, o + 16, 60);

	// +76..+131: edgeMatchingParameters (56 bytes of floats — quality-tuned)
	memcpy(f + 76, o + 76, 56);

	// +132..+239: remaining config scalars (up to carvers array)
	memcpy(f + 132, o + 132, 108);

	// +240..+271: carvers + painters arrays — constructor sets {NULL, 0, DONT_DEALLOCATE}.
	// Clear DONT_DEALLOCATE so processJobAlt's array growth works normally.
	// Original's empty arrays have capFlags=0 (no DONT_DEALLOCATE).
	*(int*)(f + 252) = 0;  // carvers capFlags: DONT_DEALLOCATE → 0
	*(int*)(f + 268) = 0;  // painters capFlags: DONT_DEALLOCATE → 0

	// +272..+287: config between painters and materialMap
	memcpy(f + 272, o + 272, 16);

	// +288..+303: materialMap — share original's data with DONT_DEALLOCATE
	*(void**)(f + 288) = *(void**)(o + 288);    // data pointer (shared)
	*(int*)(f + 296) = *(int*)(o + 296);         // count
	*(int*)(f + 300) = *(int*)(o + 300) | HKARRAY_DONT_DEALLOCATE;  // cap + non-owning

	// +304..+335: config before SimplificationSettings
	memcpy(f + 304, o + 304, 32);

	// +336..+463: SimplificationSettings scalars (before m_userVertices at +464)
	// Constructor initialized SimplSettings via sub_BCED20D0 — we overwrite scalar
	// config but leave hkArray/hkStringPtr at constructor defaults (empty/NULL/safe)
	memcpy(f + 336, o + 336, 128);

	// Skip +464..+479: m_userVertices hkArray (constructor's empty DONT_DEALLOCATE is correct)

	// +480..+487: SimplSettings scalars between m_userVertices and hkStringPtr
	memcpy(f + 480, o + 480, 8);

	// Skip +488..+495: hkStringPtr m_snapshotFilename (constructor's NULL is correct)

	// +496..+519: config after SimplificationSettings, before overrideSettings
	memcpy(f + 496, o + 496, 24);

	// +520..+535: overrideSettings array — clear DONT_DEALLOCATE (same as carvers/painters)
	*(int*)(f + 532) = 0;  // overrideSettings capFlags: DONT_DEALLOCATE → 0

	return fresh;
}

static void RebaseNMGPointers(void* clonedNMG, uintptr_t realWB, void* clonedWB, long wbSize)
{
	uintptr_t realWBEnd = realWB + wbSize;
	uintptr_t cloneWBAddr = (uintptr_t)clonedWB;

	for (int off = 0; off + 8 <= NMG_STRUCT_SIZE; off += 8)
	{
		if (off == 256) continue;  // already set to clonedWB by caller

		uintptr_t* slot = (uintptr_t*)((char*)clonedNMG + off);
		uintptr_t val = *slot;
		if (val >= realWB && val < realWBEnd)
			*slot = cloneWBAddr + (val - realWB);
	}
}


// ---- Worker dequeue ----

#if NMCACHE_STEP >= 4

static uintptr_t WorkerTryDequeue()
{
	uintptr_t nmg = g_navMeshGen;
	if (!nmg) return 0;

	// Cheap unlocked check: is the queue non-empty?
	// Only read the head POINTER (atomic on x86-64), do NOT dereference it.
	if (!*(uintptr_t*)(nmg + 136)) return 0;

	// Lock, then peek and dequeue safely
	char initBuf[16];
	void* initResult = fn_pathBuilderInit(initBuf);
	fn_pathBuilderFinalize((void*)(nmg + 152), initResult);

	uintptr_t job = *(uintptr_t*)(nmg + 136);
	if (!job)
	{
		fn_readerUnlock((void*)(nmg + 152));
		return 0;
	}

	int jobType = *(int*)(job + 88) & 7;
	if (jobType != 0 && jobType != 1)
	{
		fn_readerUnlock((void*)(nmg + 152));
		return 0;
	}

	uintptr_t nextJob = *(uintptr_t*)(job + 96);
	*(uintptr_t*)(nmg + 136) = nextJob;
	if (!nextJob)
		*(uintptr_t*)(nmg + 144) = nmg + 136;

	fn_readerUnlock((void*)(nmg + 152));

	uintptr_t zone = *(uintptr_t*)job;
	if (!zone || !*(uintptr_t*)zone)
		return 0;

	InterlockedIncrement(&nmJobCount);
	return job;
}

DWORD WINAPI NavMeshWorkerProc(LPVOID param)
{
	int workerId = (int)(uintptr_t)param;

	// Havok TLS init (5-step sequence)
	char ctx128[128];
	memset(ctx128, 0, sizeof(ctx128));
	fn_havokContextInit(ctx128);

	void* mgr = fn_havokGetManager(0);
	if (!mgr)
	{
		LogMsg("[ZoneOpt] Worker: HavokGetManager returned NULL, aborting");
		return 1;
	}

	uintptr_t mgrVtable = *(uintptr_t*)mgr;
	typedef void (*havokRegister_t)(void*, void*, const char*, int, int);
	havokRegister_t fn_reg = (havokRegister_t)(*(uintptr_t*)(mgrVtable + 24));

	char name[32];
	sprintf_s(name, sizeof(name), "ZoneOpt_W%d", workerId);
	fn_reg(mgr, ctx128, name, 3, -2);

	char buf8[8];
	memset(buf8, 0, sizeof(buf8));
	fn_havokPostRegInit(buf8, ctx128);
	_mm_setcsr(_mm_getcsr() | 0x8000);  // flush denormals

	if (g_isWorkerTlsIndex != TLS_OUT_OF_INDEXES)
		TlsSetValue(g_isWorkerTlsIndex, (void*)1);

	EnsureScratchTLS();
	InstallHavokHeapHooks();

	{
		std::ostringstream ss;
		ss << "[ZoneOpt] Worker " << workerId << " started, Havok TLS + scratch initialized";
		LogMsg(ss.str());
	}

	while (!g_workerShutdown)
	{
		WaitForSingleObject(g_jobEvent, 500);
		if (g_workerShutdown) break;

		uintptr_t job = WorkerTryDequeue();
		if (!job) continue;

		int jobType = *(int*)(job + 88) & 7;

		// HITs run concurrently (buildCollision has internal lock).
		// MISSes serialize via processJobCS inside ProcessNavMeshJob.
		ProcessNavMeshJob((void*)g_navMeshGen, (void*)g_navMeshGen, job, jobType);
	}

	fn_havokCleanup(buf8);
	fn_havokCtxCleanup(ctx128);

	{
		std::ostringstream ss;
		ss << "[ZoneOpt] Worker " << workerId << " exiting";
		LogMsg(ss.str());
	}
	return 0;
}

void CreateNavMeshWorkers()
{
	if (fn_havokContextInit && fn_havokGetManager && fn_havokPostRegInit)
	{
		int created = 0;
		for (int i = 0; i < NAVMESH_WORKER_COUNT; ++i)
		{
			g_workerHandles[i] = CreateThread(NULL, 0, NavMeshWorkerProc,
			                                   (LPVOID)(uintptr_t)i, 0, NULL);
			if (g_workerHandles[i]) created++;
		}
		std::ostringstream ws;
		ws << "[ZoneOpt] NavMesh workers: " << created << "/" << NAVMESH_WORKER_COUNT
		   << " created (lazy, from first dispatchJob)";
		LogMsg(ws.str());
	}
	else
	{
		LogMsg("[ZoneOpt] NavMesh workers: SKIPPED (Havok fn ptrs missing)");
	}
}

#endif // NMCACHE_STEP >= 4


// ---- Job processing pipeline ----

void ProcessNavMeshJob(void* realNMG, void* workNMG, uintptr_t job, int jobType)
{
	uintptr_t nmg = g_navMeshGen;

	InterlockedIncrement(&workerBusyCount);
	*(unsigned char*)(nmg + 265) = 1;  // bridge: game's isContentPending reads this byte

	uintptr_t jobZone = *(uintptr_t*)job;
	int gridX = *(int*)(jobZone + OFF_ZONE_COORDS_X);
	int gridY = *(int*)(jobZone + OFF_ZONE_COORDS_Y);
	int tileId = *(int*)(job + 32);

	InterlockedExchange(&nmDiagLastGridX, gridX);
	InterlockedExchange(&nmDiagLastGridY, gridY);
	InterlockedExchange(&nmDiagLastType, jobType);

	NavMeshCacheKey key;
	key.gridX = gridX;
	key.gridY = gridY;
	key.sectionTileId = tileId;
	key.jobType = jobType;
	key.aabbHash = HashAABB((float*)(job + 48));
	key.buildingHash = ComputeBuildingHash(jobZone);

	InterlockedExchange(&nmDiagStep, 10);

	int hitIdx = -1;
	bool isHit = false;
	bool isL2Hit = false;
	if (nmDiagStage >= 2 && !InterlockedCompareExchange(&nmCacheDisabled, 0, 0))
	{
		EnterCriticalSection(&nmCacheCS);

		hitIdx = FindCacheEntry(key);
		isHit = (hitIdx >= 0 && nmCache[hitIdx].cachedFaces != NULL && fn_navMeshCtor != NULL);

#if NMCACHE_STEP >= 2
		if (!isHit && fn_navMeshCtor != NULL)
		{
			LeaveCriticalSection(&nmCacheCS);

			LARGE_INTEGER tR0, tR1;
			QueryPerformanceCounter(&tR0);

			NavMeshCacheEntry diskEntry;
			memset(&diskEntry, 0, sizeof(diskEntry));
			if (ReadDiskCache(key, diskEntry))
			{
				EnterCriticalSection(&nmCacheCS);
				if (nmCache[nmCacheWriteIdx].valid)
					EvictCacheEntry(nmCacheWriteIdx);

				nmCache[nmCacheWriteIdx] = diskEntry;
				hitIdx = nmCacheWriteIdx;
				nmCacheWriteIdx = (nmCacheWriteIdx + 1) % NM_CACHE_SIZE;
				if (nmCacheFill < NM_CACHE_SIZE)
					nmCacheFill++;

				isHit = true;
				isL2Hit = true;
				LeaveCriticalSection(&nmCacheCS);
				InterlockedIncrement(&nmDiskHitCount);
			}
			else
			{
				InterlockedIncrement(&nmDiskMissCount);

				long idx = InterlockedIncrement(&l2MissLogCount) - 1;
				if (idx < L2_MISS_LOG_MAX)
				{
					l2MissLog[idx].gridX = key.gridX;
					l2MissLog[idx].gridY = key.gridY;
					l2MissLog[idx].tileId = key.sectionTileId;
					l2MissLog[idx].jobType = key.jobType;
					l2MissLog[idx].aabbHash = key.aabbHash;
					l2MissLog[idx].buildingHash = key.buildingHash;
					uintptr_t content = *(uintptr_t*)(jobZone + OFF_ZONE_CONTENT);
					l2MissLog[idx].thingsCount = content ? *(int*)(content + OFF_ZMC_THINGS_COUNT) : -1;
				}
			}

			QueryPerformanceCounter(&tR1);
			long readUs = (long)(QPCToMs(tR0, tR1) * 1000.0);
			InterlockedExchangeAdd(&nmDiskReadUsTimes1, readUs);
		}
		else
#endif // NMCACHE_STEP >= 2
		{
			if (isHit)
			{
				void* freshNavMesh = ReconstructNavMesh(nmCache[hitIdx]);
				LeaveCriticalSection(&nmCacheCS);

				if (freshNavMesh)
				{
					*(void**)(job + 72) = freshNavMesh;
					*(void**)(job + 80) = NULL;
				}
				else
				{
					isHit = false;
				}
			}
			else
			{
				LeaveCriticalSection(&nmCacheCS);
			}
		}
	}

	InterlockedExchange(&nmDiagStep, 11);

#if NMCACHE_STEP >= 4
	if (g_isWorkerTlsIndex != TLS_OUT_OF_INDEXES && !TlsGetValue(g_isWorkerTlsIndex))
		TlsSetValue(g_isWorkerTlsIndex, (void*)1);

	EnsureScratchTLS();
	InstallHavokHeapHooks();
#endif

	if (isHit)
	{
		if (!isL2Hit)
			InterlockedIncrement(&nmCacheHitCount);
		InterlockedExchange(&nmDiagStep, 20);
		InterlockedExchange(&nmDiagHitGrid, gridX * 100 + gridY);

		LARGE_INTEGER t0, t1;
		QueryPerformanceCounter(&t0);

		if (isL2Hit)
		{
			EnterCriticalSection(&nmCacheCS);
			void* freshNavMesh = ReconstructNavMesh(nmCache[hitIdx]);
			LeaveCriticalSection(&nmCacheCS);

			if (freshNavMesh)
			{
				*(void**)(job + 72) = freshNavMesh;
				*(void**)(job + 80) = NULL;
			}
			else
			{
				isHit = false;
			}
		}

		if (isHit)
		{
			InterlockedExchange(&nmDiagStep, 23);
			fn_buildCollision(realNMG, (void*)job);
			InterlockedExchange(&nmDiagStep, 24);
		}

		QueryPerformanceCounter(&t1);
		if (isL2Hit)
		{
			long reconUs = (long)(QPCToMs(t0, t1) * 1000.0);
			InterlockedExchangeAdd(&nmDiskReadUsTimes1, reconUs);
		}
		else if (isHit)
		{
			long ms10 = (long)(QPCToMs(t0, t1) * 10.0);
			InterlockedExchangeAdd(&nmSavedMsTimes10, ms10);
		}
	}

	if (!isHit)
	{
		InterlockedIncrement(&nmCacheMissCount);
		InterlockedExchange(&nmDiagStep, 30);

		LARGE_INTEGER t0, t1;
		QueryPerformanceCounter(&t0);

		InterlockedExchange(&nmDiagStep, 31);

		EnterCriticalSection(&processJobCS);
		{
			if (workNMG != realNMG)
			{
				uintptr_t realWB = *(uintptr_t*)((uintptr_t)realNMG + 256);
				// Free old settings and construct fresh for each MISS
				uintptr_t oldWB = *(uintptr_t*)((uintptr_t)workNMG + 256);
				if (oldWB) HavokTlsFree((void*)oldWB, WB_OBJECT_SIZE);

				void* freshWB = ConstructFreshSettings(realWB);
				*(uintptr_t*)((uintptr_t)workNMG + 256) = (uintptr_t)freshWB;

				{
					std::ostringstream ss;
					ss << "[ZoneOpt] MISS fresh: realNMG=" << realNMG << " workNMG=" << workNMG
					   << " realWB=" << (void*)realWB << " freshWB=" << freshWB
					   << " grid=(" << gridX << "," << gridY << ") type=" << jobType;
					LogMsg(ss.str());
				}
			}
			else
			{
				LogMsg("[ZoneOpt] MISS no-clone: using real NMG");
			}

			WorkerScratch* ws = (WorkerScratch*)TlsGetValue(g_scratchTlsIndex);
			if (ws)
			{
				*(void***)(gameBase + RVA_SCRATCH_BUFFER) = ws->buffer;
				*(int*)(gameBase + RVA_SCRATCH_SIZE) = ws->capacity;
			}

			uintptr_t* scratchPtr = (uintptr_t*)(gameBase + RVA_SCRATCH_BUFFER);
			if (!*scratchPtr)
			{
				unsigned int scratchCount = *(unsigned int*)(gameBase + RVA_SCRATCH_SIZE);
				if (scratchCount == 0) scratchCount = 4096;
				*scratchPtr = (uintptr_t)fn_gameNewArr((size_t)scratchCount * 8);
				if (ws)
				{
					ws->buffer = (void**)*scratchPtr;
					ws->capacity = (int)scratchCount;
				}
			}

			// processJobCS held for entire processJobAlt (parallel MISSes blocked)
			// To re-attempt parallel MISSes, uncomment:
			// TlsSetValue(g_releaseFlagTlsIndex, (void*)1);
		}

		LogMsg("[ZoneOpt] MISS: entering processJobAlt");

		if (workNMG != realNMG)
			InterlockedExchange(&g_cloneProcessing, 1);

		fn_processJobAlt(workNMG, (void*)job);

		InterlockedExchange(&g_cloneProcessing, 0);
		InterlockedExchange(&nmDiagStep, 32);

		if (jobType == 1)
			fn_partialFixup(workNMG, (void*)job);

		LeaveCriticalSection(&processJobCS);

		InterlockedExchange(&nmDiagStep, 33);

		fn_buildCollision(realNMG, (void*)job);

		InterlockedExchange(&nmDiagStep, 34);

		QueryPerformanceCounter(&t1);
		long ms10 = (long)(QPCToMs(t0, t1) * 10.0);
		InterlockedExchangeAdd(&nmTotalMsTimes10, ms10);

		if (nmDiagStage >= 2 && !InterlockedCompareExchange(&nmCacheDisabled, 0, 0))
		{
			uintptr_t navMeshResult = *(uintptr_t*)(job + 72);
			if (navMeshResult)
			{
				EnterCriticalSection(&nmCacheCS);
				StoreCacheEntry(key, navMeshResult);
				LeaveCriticalSection(&nmCacheCS);

#if NMCACHE_STEP >= 2
				LARGE_INTEGER tW0, tW1;
				QueryPerformanceCounter(&tW0);
				WriteDiskCache(key, navMeshResult);
				QueryPerformanceCounter(&tW1);
				long writeUs = (long)(QPCToMs(tW0, tW1) * 1000.0);
				InterlockedExchangeAdd(&nmDiskWriteUsTimes1, writeUs);
#endif
			}
		}

		InterlockedExchange(&nmDiagStep, 35);
	}

	InterlockedExchange(&nmDiagStep, 40);

	void* label29NavInst = *(void**)(job + 80);
	if (label29NavInst)
		*(int*)((uintptr_t)label29NavInst + 64) = *(int*)(job + 32);

	InterlockedExchange(&nmDiagStep, 41);

	uintptr_t navMeshResult = *(uintptr_t*)(job + 72);
	int faceCount = navMeshResult ? *(int*)(navMeshResult + 24) : 0;
	if (navMeshResult && faceCount > 0)
	{
		InterlockedExchange(&nmDiagStep, 42);
		fn_enqueueToProcQueue((void*)(nmg + 184), (void*)job);
	}
	else
	{
		InterlockedExchange(&nmDiagStep, 43);
		void* delNavInst = *(void**)(job + 80);
		if (delNavInst)
			fn_gameDelete(delNavInst);

		void* buildingRef = *(void**)(job + 24);
		if (buildingRef)
			fn_gameDelArr(buildingRef);

		fn_gameDelete((void*)job);
	}

	InterlockedExchange(&nmDiagStep, 44);

	if (InterlockedDecrement(&workerBusyCount) == 0)
		*(unsigned char*)(nmg + 265) = 0;

	InterlockedExchange(&nmDiagStep, 50);
}


// ---- Hooks ----

void hook_realGenerate(void* workBuffer, void* localData, void* hkaiNavMesh, int param, int timeLowPart)
{
	// DISABLED: trampoline can't relocate first instructions of 61K function
	orig_realGenerate(workBuffer, localData, hkaiNavMesh, param, timeLowPart);
}

#if NMCACHE_STEP >= 5
void hook_nmResultPopulate_diag(void* navData, void* localData, void* result, int param, int lowPart)
{
	if (TlsGetValue(g_releaseFlagTlsIndex))
	{
		WorkerScratch* ws = (WorkerScratch*)TlsGetValue(g_scratchTlsIndex);
		if (ws)
		{
			ws->buffer = *(void***)(gameBase + RVA_SCRATCH_BUFFER);
			ws->capacity = *(int*)(gameBase + RVA_SCRATCH_SIZE);
		}

		LeaveCriticalSection(&processJobCS);
		TlsSetValue(g_releaseFlagTlsIndex, NULL);
		LogMsg("[ZoneOpt] populate hook: released processJobCS, entering realGenerate");
	}

	orig_nmResultPopulate(navData, localData, result, param, lowPart);
}
#else
void hook_nmResultPopulate_diag(void* navData, void* localData, void* result, int param, int lowPart)
{
	orig_nmResultPopulate(navData, localData, result, param, lowPart);
}
#endif


static bool nmBypassHook = false;

char hook_dispatchJob(void* thisNMG)
{
	if (nmBypassHook)
		return orig_dispatchJob(thisNMG);

	uintptr_t nmg = (uintptr_t)thisNMG;

	if (!g_navMeshGen)
		g_navMeshGen = nmg;

#if NMCACHE_STEP >= 4
	// Install FLA CS hooks BEFORE creating workers (prevents unprotected concurrent allocations)
	InstallHavokHeapHooks();
	{
		static volatile long workersCreated = 0;
		if (!InterlockedCompareExchange(&workersCreated, 1, 0))
			CreateNavMeshWorkers();
	}
#endif

#if NMCACHE_STEP >= 3
	ProbeNavMeshSettings(nmg);
	ApplyNavMeshQualityTuning(nmg);
	VerifyNavMeshSettings(nmg);
	ProbeWorkBufferSize(nmg);
#endif

#if NMCACHE_STEP >= 4
	// Multi-consumer path: lock before peek (workers may free jobs concurrently)
	if (!*(uintptr_t*)(nmg + 136))
	{
		if (InterlockedCompareExchange(&workerBusyCount, 0, 0) == 0)
			*(unsigned char*)(nmg + 265) = 0;
		return 0;
	}

	char initBuf[16];
	void* initResult = fn_pathBuilderInit(initBuf);
	fn_pathBuilderFinalize((void*)(nmg + 152), initResult);

	uintptr_t job = *(uintptr_t*)(nmg + 136);
	if (!job)
	{
		fn_readerUnlock((void*)(nmg + 152));
		if (InterlockedCompareExchange(&workerBusyCount, 0, 0) == 0)
			*(unsigned char*)(nmg + 265) = 0;
		return 0;
	}

	uintptr_t peekZone = *(uintptr_t*)job;
	if (!peekZone || !*(uintptr_t*)peekZone)
	{
		fn_readerUnlock((void*)(nmg + 152));
		return orig_dispatchJob(thisNMG);
	}

	int jobType = *(int*)(job + 88) & 7;

	if (jobType != 0 && jobType != 1)
	{
		fn_readerUnlock((void*)(nmg + 152));
		InterlockedIncrement(&nmJobCount);
		InterlockedIncrement(&nmCacheSkipCount);
		return orig_dispatchJob(thisNMG);
	}

	InterlockedIncrement(&nmJobCount);

	uintptr_t nextJob = *(uintptr_t*)(job + 96);
	*(uintptr_t*)(nmg + 136) = nextJob;
	if (!nextJob)
		*(uintptr_t*)(nmg + 144) = nmg + 136;

	fn_readerUnlock((void*)(nmg + 152));
#else
	// Single-consumer path: unlocked peek is safe (no workers)
	uintptr_t head = *(uintptr_t*)(nmg + 136);
	if (!head)
	{
		if (InterlockedCompareExchange(&workerBusyCount, 0, 0) == 0)
			*(unsigned char*)(nmg + 265) = 0;
		return 0;
	}

	uintptr_t peekZone = *(uintptr_t*)head;
	if (!peekZone || !*(uintptr_t*)peekZone)
		return orig_dispatchJob(thisNMG);

	int jobType = *(int*)(head + 88) & 7;

	if (jobType != 0 && jobType != 1)
	{
		InterlockedIncrement(&nmJobCount);
		InterlockedIncrement(&nmCacheSkipCount);
		return orig_dispatchJob(thisNMG);
	}

	InterlockedIncrement(&nmJobCount);

	char initBuf[16];
	void* initResult = fn_pathBuilderInit(initBuf);
	fn_pathBuilderFinalize((void*)(nmg + 152), initResult);

	uintptr_t job = *(uintptr_t*)(nmg + 136);
	uintptr_t nextJob = *(uintptr_t*)(job + 96);
	*(uintptr_t*)(nmg + 136) = nextJob;
	if (!nextJob)
		*(uintptr_t*)(nmg + 144) = nmg + 136;

	fn_readerUnlock((void*)(nmg + 152));
#endif

	if (!*(uintptr_t*)(*(uintptr_t*)job))
		return 1;

#if NMCACHE_STEP >= 4
	if (g_jobEvent && *(uintptr_t*)(nmg + 136))
		SetEvent(g_jobEvent);
#endif

	// HITs run concurrently, MISSes serialize via processJobCS inside ProcessNavMeshJob.
	ProcessNavMeshJob(thisNMG, thisNMG, job, jobType);
	return 1;
}


#endif // NMCACHE_STEP >= 1
