// nm_workers.cpp — NavMesh worker pool, cloning, and job pipeline
//
// 3 worker threads parallelize NavMesh generation alongside Kenshi's
// single-threaded NavMesh bg thread. Workers dequeue jobs from the shared
// input queue, reconstruct cached results for L1/L2 HITs (concurrent), and
// steal MISSes under processJobCS (serialized with bg thread).
//
// Full design history in research/worker_clone_investigation.md.
// Stage 4d (parallel MISSes, processJobCS released) is structurally infeasible
// under Kenshi's HK_CONFIG_SINGLE_THREADED Havok build — see
// research/extra/stage4d_infeasibility.md.

#include "nm_workers.h"

#if NMCACHE_STEP >= 1

// --------------------------------------------------------------------
// Worker pool state
// --------------------------------------------------------------------

HANDLE          g_workerHandles[NAVMESH_WORKER_COUNT] = {};
volatile long   g_workerShutdown     = 0;
HANDLE          g_jobEvent           = NULL;
uintptr_t       g_navMeshGen         = 0;
int             g_workerSavedPriority[NAVMESH_WORKER_COUNT] = {};

// MISSes serialize via processJobCS, so a global flag is sufficient to tell
// hook_edgeProcess we're finalizing on a clone.
static volatile long g_cloneProcessing = 0;


// --------------------------------------------------------------------
// Scratch buffer lazy-init
// --------------------------------------------------------------------
//
// Mirrors orig_dispatchJob (0x3CE030). processJob (0x3C8520) is one of 8 scratch
// consumers and NULL-derefs at +0xA7 without it. Must run before every
// fn_processJobAlt. Safe under processJobCS.

static inline void EnsureGlobalScratchBuffer()
{
	uintptr_t* scratchPtr = (uintptr_t*)(gameBase + RVA_SCRATCH_BUFFER);
	if (!*scratchPtr)
	{
		unsigned int n = *(unsigned int*)(gameBase + RVA_SCRATCH_SIZE);
		if (n == 0) n = 4096;
		*scratchPtr = (uintptr_t)fn_gameNewArr((size_t)n * 8);
	}
}


// --------------------------------------------------------------------
// hkFreeListAllocator CS hooks
// --------------------------------------------------------------------
//
// Kenshi's Havok is compiled with HK_CONFIG_SINGLE_THREADED — four FLA methods
// have their internal CS calls stripped. We wrap them with the allocator's own
// CS at FLA+0x10 so concurrent callers (bg thread + workers) serialize
// correctly. Coverage is partial — dozens of other entry points exist but many
// are devirtualized/inlined and can't be externally hooked. See
// research/extra/stage4d_infeasibility.md.

static CRITICAL_SECTION* g_flaCS = NULL;

typedef void    (__fastcall *flaResetPeak_t)(__int64 thisAlloc);
typedef __int64 (__fastcall *flaCanAlloc_t)(__int64 thisAlloc, __int64 numBytes);
typedef __int64 (__fastcall *flaBufRealloc_t)(__int64 thisAlloc, __int64 pold, int oldNumBytes, int* reqInOut);
typedef __int64 (__fastcall *flaGC_t)(__int64 thisAlloc);
typedef void    (__fastcall *edgeProcess_t)(void* entry);

static flaResetPeak_t  orig_flaResetPeak  = NULL;
static flaCanAlloc_t   orig_flaCanAlloc   = NULL;
static flaBufRealloc_t orig_flaBufRealloc = NULL;
static flaGC_t         orig_flaGC         = NULL;

static volatile long g_flaResetPeakLogged  = 0;
static volatile long g_flaCanAllocLogged   = 0;
static volatile long g_flaBufReallocLogged = 0;
static volatile long g_flaGCLogged         = 0;

static void __fastcall hook_flaResetPeak(__int64 thisAlloc)
{
	if (InterlockedCompareExchange(&g_flaResetPeakLogged, 1, 0) == 0)
		LogMsg("[ZoneOpt] FLA hook fired: resetPeak");
	EnterCriticalSection(g_flaCS);
	InterlockedIncrement(&g_flaCSAcquisitions);
	orig_flaResetPeak(thisAlloc);
	LeaveCriticalSection(g_flaCS);
}

static __int64 __fastcall hook_flaCanAlloc(__int64 thisAlloc, __int64 numBytes)
{
	if (InterlockedCompareExchange(&g_flaCanAllocLogged, 1, 0) == 0)
		LogMsg("[ZoneOpt] FLA hook fired: canAlloc");
	EnterCriticalSection(g_flaCS);
	InterlockedIncrement(&g_flaCSAcquisitions);
	__int64 r = orig_flaCanAlloc(thisAlloc, numBytes);
	LeaveCriticalSection(g_flaCS);
	return r;
}

static __int64 __fastcall hook_flaBufRealloc(__int64 thisAlloc, __int64 pold, int oldNumBytes, int* reqInOut)
{
	if (InterlockedCompareExchange(&g_flaBufReallocLogged, 1, 0) == 0)
		LogMsg("[ZoneOpt] FLA hook fired: bufRealloc");
	EnterCriticalSection(g_flaCS);
	InterlockedIncrement(&g_flaCSAcquisitions);
	__int64 r = orig_flaBufRealloc(thisAlloc, pold, oldNumBytes, reqInOut);
	LeaveCriticalSection(g_flaCS);
	return r;
}

static __int64 __fastcall hook_flaGC(__int64 thisAlloc)
{
	if (InterlockedCompareExchange(&g_flaGCLogged, 1, 0) == 0)
		LogMsg("[ZoneOpt] FLA hook fired: GC");
	EnterCriticalSection(g_flaCS);
	InterlockedIncrement(&g_flaCSAcquisitions);
	__int64 r = orig_flaGC(thisAlloc);
	LeaveCriticalSection(g_flaCS);
	return r;
}


// --------------------------------------------------------------------
// edgeProcess clone-guard
// --------------------------------------------------------------------
//
// finalize (0x3C2300) iterates wb+520 (overrideSettings) entries, calling
// edgeProcess per entry. For entries appended by processJobAlt during a
// clone's generation, entry+0 (hkRefPtr) is uninitialized — the normal
// CAS-decrement in edgeProcess would read garbage. When g_cloneProcessing is
// set we skip the whole per-entry cleanup. Net effect: a bounded refcount/alloc
// leak (~100 bytes per cloned MISS) released on zone unload.

static edgeProcess_t orig_edgeProcess = NULL;
static volatile long g_edgeProcessLogged = 0;
volatile long g_edgeProcessArmedCount = 0;
volatile long g_edgeProcessUnarmedCount = 0;

static void __fastcall hook_edgeProcess(void* entry)
{
	if (InterlockedCompareExchange(&g_edgeProcessLogged, 1, 0) == 0)
		LogMsg("[ZoneOpt] edgeProcess hook fired (clone-guard armed)");

	if (InterlockedCompareExchange(&g_cloneProcessing, 0, 0) != 0)
	{
		InterlockedIncrement(&g_edgeProcessArmedCount);
		return;
	}
	InterlockedIncrement(&g_edgeProcessUnarmedCount);
	orig_edgeProcess(entry);
}


// --------------------------------------------------------------------
// FLA probe + hook install
// --------------------------------------------------------------------
//
// Finds hkFreeListAllocator via the Havok TLS router: router[9] = hkThreadMemory
// (m_temp); +0x08 = m_memory = hkFreeListAllocator. Validates primary vtable +
// CS at FLA+0x10 before capturing the pointer. SEH-wrapped because the router
// layout is only guaranteed valid after Havok world init — an early call would
// AV otherwise.

static uintptr_t ProbeFLAInstance()
{
	DWORD tlsIdx = *(DWORD*)(gameBase + RVA_HAVOK_TLS_INDEX);
	uintptr_t* router = (uintptr_t*)TlsGetValue(tlsIdx);
	if (!router) return 0;

	uintptr_t fla = 0;
	__try
	{
		uintptr_t tm = router[9];
		if (!tm || tm < 0x10000) return 0;
		fla = *(uintptr_t*)(tm + 0x08);
		if (!fla || fla < 0x10000) return 0;

		uintptr_t vt = *(uintptr_t*)fla;
		if (vt != gameBase + RVA_FLA_VTABLE) return 0;

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

	int installed = 0;
	if (KenshiLib::SUCCESS == KenshiLib::AddHook(GameAddr(RVA_FLA_RESET_PEAK),
			(void*)hook_flaResetPeak, (void**)&orig_flaResetPeak))    installed++;
	if (KenshiLib::SUCCESS == KenshiLib::AddHook(GameAddr(RVA_FLA_CAN_ALLOC),
			(void*)hook_flaCanAlloc, (void**)&orig_flaCanAlloc))      installed++;
	if (KenshiLib::SUCCESS == KenshiLib::AddHook(GameAddr(RVA_FLA_BUF_REALLOC),
			(void*)hook_flaBufRealloc, (void**)&orig_flaBufRealloc))  installed++;
	if (KenshiLib::SUCCESS == KenshiLib::AddHook(GameAddr(RVA_FLA_GC),
			(void*)hook_flaGC, (void**)&orig_flaGC))                  installed++;

	{
		std::ostringstream ss;
		ss << "[ZoneOpt] FLA CS hooks: " << installed << "/4 installed"
		   << " FLA=" << (void*)fla << " CS=" << (void*)g_flaCS;
		LogMsg(ss.str());
	}

	if (KenshiLib::SUCCESS == KenshiLib::AddHook(GameAddr(RVA_EDGE_PROCESS),
			(void*)hook_edgeProcess, (void**)&orig_edgeProcess))
		LogMsg("[ZoneOpt] edgeProcess clone-guard: installed");
	else
		LogMsg("[ZoneOpt] edgeProcess clone-guard: install FAILED");

	InterlockedExchange(&flaHooksInstalled, 2);
}


// --------------------------------------------------------------------
// WorkBuffer (hkaiNavMeshGenerationSettings) construction
// --------------------------------------------------------------------
//
// Build a fresh 544-byte settings object via the real constructor (0xDD99D0),
// then copy ONLY scalar config regions from the original. Sub-structs at +144
// (48B), +192 (48B), +308, +336 (SimplificationSettings), +424
// (ExtraVertexSettings) stay at ctor defaults — shallow-copying them shares
// internally-allocated pointers with realWB, so processJobAlt's subsequent
// hkArray growth would free buffers realWB still owns.
//
// The hkArrays (carvers +240, painters +256, materialMap +288,
// overrideSettings +520) are handled specially:
//   - carvers/painters: left at the ctor state (empty, DONT_DEALLOCATE).
//     SortedArray__grow (0xBA3AA0) takes the flag path on the first push
//     (bufAlloc + copy, the same path the real WB's arrays take) and writes
//     the new capacity without the flag. Until 2026-09-11 the flag was
//     cleared here; an array the job never grew then sat at capacity 0
//     without the flag, so a settings dtor would call bufFree(NULL, 0).
//     The heap's blockFree ignores NULL, but the ctor state keeps the
//     fresh WB on the vanilla path (research/navmesh_miss_leaks.md, Fix 1).
//   - overrideSettings: guard slot + fixed capacity, DONT_DEALLOCATE (below).
//   - materialMap: per-clone deep-copy of the pointer array into a clone-owned
//     buffer. Eliminates the last DONT_DEALLOCATE shared buffer from realWB.
//
// Constructor writes +8 = 0xFFFF0001: m_referenceCount (low word at +8) = 1,
// m_memSizeAndFlags (word at +10) = 0xFFFF = class-default size. Refcounting
// is enabled (not immortal); nothing releases the WB, FreeFreshSettings frees
// it directly.
static const int OVR_ENTRY_SIZE  = 240;                             // OverrideSettings entry (SortedArray__grow elemSize)
static const int OVR_GUARD_CAP   = 8;                               // capacity; one append per job
static const int OVR_GUARD_BYTES = OVR_ENTRY_SIZE * (OVR_GUARD_CAP + 1);  // + the guard slot at entry[-1]

static void* ConstructFreshSettings(uintptr_t origWB)
{
	void* mem = HavokTlsAlloc(WB_OBJECT_SIZE);
	if (!mem) return NULL;

	void* fresh = fn_settingsCtor(mem);
	if (!fresh) { HavokTlsFree(mem, WB_OBJECT_SIZE); return NULL; }
	char* f = (char*)fresh;
	char* o = (char*)origWB;

	// +16..+131: generation config + edgeMatchingParameters (quality-tuned)
	memcpy(f + 16, o + 16, 60);
	memcpy(f + 76, o + 76, 56);

	// +132..+143: scalar ints before +144 sub-struct
	memcpy(f + 132, o + 132, 12);

	// +240/+256 carvers + painters: stay at the ctor state (empty,
	// DONT_DEALLOCATE). SortedArray__grow handles the flag on the first push.

	// +272..+287: scalar config between painters and materialMap
	memcpy(f + 272, o + 272, 16);

	// +288 materialMap: per-clone deep-copy. Raw pointer memcpy (not
	// hkRefPtr::operator=) — scene objects behind the pointers stay shared
	// read-only. Snapshot happens under processJobCS so any writer is serialized.
	{
		int mmCount  = *(int*)(o + 296);
		int mmCapLow = *(int*)(o + 300) & 0x3FFFFFFF;
		int mmCap    = mmCapLow > 0 ? mmCapLow : mmCount;
		if (mmCount > 0 && mmCap > 0)
		{
			void* mmData = HavokTlsAlloc((size_t)mmCap * 8);
			if (mmData)
			{
				memcpy(mmData, *(void**)(o + 288), (size_t)mmCount * 8);
				*(void**)(f + 288) = mmData;
				*(int*)(f + 296)   = mmCount;
				*(int*)(f + 300)   = mmCap | HKARRAY_DONT_DEALLOCATE;
			}
		}
	}

	// +320..+335: byte flags + qword after the +308 sub-struct
	memcpy(f + 320, o + 320, 16);

	// +336..+423: SimplificationSettings scalars (SimplificationSettings::copy
	// at 0x3DA000 shallow-copies this region).
	memcpy(f + 336, o + 336, 88);

	// +480..+487: SimplSettings byte + padding
	memcpy(f + 480, o + 480, 8);

	// +496..+511: config between SimplSettings and top-level hkStringPtr
	memcpy(f + 496, o + 496, 16);

	// +520 overrideSettings. processJobAlt appends exactly ONE entry per job
	// (0x3CD3EF) and finalizeDeep (0x3C2300+0x18C) then pops entries from the
	// end until one satisfies (entry+8 != -1 && entry+0 == NULL) — with NO
	// count > 0 check. The real WB owns four base entries pushed by the NMG
	// ctor helper (0x3C48D0) that stop that loop; a ctor-fresh WB has none, so
	// after popping the appended entry the loop reads entry[-1] BEFORE the
	// array and only stops if the neighbouring heap bytes happen to look like
	// a base entry (Step 2A crash 2026-09-10: array at page offset 0x160,
	// entry[-2] fell into an unmapped page; production telemetry edge=a1060
	// over 124 clones = ~8.5 garbage pops per MISS).
	// Fix: a zeroed guard slot in front of a capacity the job can never
	// outgrow, DONT_DEALLOCATE so the game neither frees nor reallocs it.
	// FreeFreshSettings releases it.
	{
		char* ovr = (char*)HavokTlsAlloc(OVR_GUARD_BYTES);
		if (!ovr)
		{
			// A fresh WB without a guard is unsafe: fall back to the real WB
			// ("MISS no-swap"). Release the materialMap copy made above.
			void* mmData = *(void**)(f + 288);
			int mmCapFlags = *(int*)(f + 300);
			if (mmData && (mmCapFlags & HKARRAY_DONT_DEALLOCATE) && (mmCapFlags & 0x3FFFFFFF) > 0)
				HavokTlsFree(mmData, (size_t)(mmCapFlags & 0x3FFFFFFF) * 8);
			HavokTlsFree(mem, WB_OBJECT_SIZE);
			return NULL;
		}
		memset(ovr, 0, OVR_GUARD_BYTES);
		*(void**)(f + 520) = ovr + OVR_ENTRY_SIZE;   // entry[-1] = zeroed guard
		*(int*)(f + 528)   = 0;
		*(int*)(f + 532)   = OVR_GUARD_CAP | HKARRAY_DONT_DEALLOCATE;
	}

	return fresh;
}

// Release a WB from ConstructFreshSettings. No dtor: sub-structs were never
// initialised beyond ctor defaults. The override-settings guard buffer is
// still ours only while DONT_DEALLOCATE is set with our capacity —
// SortedArray__grow (0xBA3AA0) replaces the buffer and rewrites the capacity
// if a job ever appended more than OVR_GUARD_CAP entries (never observed).
static void FreeFreshSettings(void* wb)
{
	if (!wb) return;
	char* f = (char*)wb;
	char* ovr = *(char**)(f + 520);
	int capFlags = *(int*)(f + 532);
	if (ovr && (capFlags & HKARRAY_DONT_DEALLOCATE)
	    && (capFlags & 0x3FFFFFFF) == OVR_GUARD_CAP)
		HavokTlsFree(ovr - OVR_ENTRY_SIZE, OVR_GUARD_BYTES);

	// +288 materialMap deep copy (ConstructFreshSettings): ours while
	// DONT_DEALLOCATE is still set with a non-zero capacity — SortedArray__grow
	// replaces the buffer and rewrites the capacity if the game ever grows it.
	void* mm = *(void**)(f + 288);
	int mmCapFlags = *(int*)(f + 300);
	if (mm && (mmCapFlags & HKARRAY_DONT_DEALLOCATE) && (mmCapFlags & 0x3FFFFFFF) > 0)
		HavokTlsFree(mm, (size_t)(mmCapFlags & 0x3FFFFFFF) * 8);

	HavokTlsFree(wb, WB_OBJECT_SIZE);
}


// --------------------------------------------------------------------
// NavMeshGenerator clone (352 bytes)
// --------------------------------------------------------------------
//
// Workers need their own NMG so processJobAlt doesn't race on realNMG+256
// (the shared workBuffer pointer). A bare memcpy crashes — the ctor sets up
// five critical sections and two self-referencing queue sentinels that a raw
// copy would corrupt. Protocol, one step per hazard:
//
//   +136/+144: input queue head/sentinel-tail. Tail points at &head (self-ref);
//              after memcpy both slots must reference the CLONE's head, not
//              realNMG's. Clear head and point tail at &clone+136.
//   +184/+192: output queue — same pattern.
//   +72 +104 +152 +200 +272: five critical sections. Byte-state is owned by
//              whoever entered them; reusing would silently share lock
//              ownership with realNMG. fn_queueLockInit (RVA 0x25F350) is the
//              game's CS init routine — call it on each slot.
//   +312..+327: ThreadClass substructure. processJobAlt reads +320 as a
//              pointer three times — zeroing it NULL-derefs at processJobAlt
//              +0xD25. Leave it memcpy'd; the reads are side-effect-free.
//              Do NOT call ThreadClass::init/start/finalize on the clone
//              (would spawn an extra OS thread).
//   +232: current-work-item pointer. Zero so isBusy's +232 path returns false
//         for clone-in-flight jobs; queue-walk path still catches queued jobs.
//   +256: settings pointer (workBuffer). Override with a freshly-constructed
//         settings object owned by the clone.
static void* CloneNMG(void* realNMG)
{
	if (!realNMG || !fn_queueLockInit || !fn_settingsCtor) return NULL;

	void* clone = HavokTlsAlloc(NMG_STRUCT_SIZE);
	if (!clone) return NULL;

	memcpy(clone, realNMG, NMG_STRUCT_SIZE);
	char* c = (char*)clone;

	*(uintptr_t*)(c + 136) = 0;
	*(uintptr_t*)(c + 144) = (uintptr_t)(c + 136);
	*(uintptr_t*)(c + 184) = 0;
	*(uintptr_t*)(c + 192) = (uintptr_t)(c + 184);

	fn_queueLockInit(c + 72);
	fn_queueLockInit(c + 104);
	fn_queueLockInit(c + 152);
	fn_queueLockInit(c + 200);
	fn_queueLockInit(c + 272);

	*(uintptr_t*)(c + 232) = 0;

	uintptr_t realWB = *(uintptr_t*)((uintptr_t)realNMG + 256);
	void* freshWB = ConstructFreshSettings(realWB);
	if (!freshWB)
	{
		HavokTlsFree(clone, NMG_STRUCT_SIZE);
		InterlockedIncrement(&nmCloneConstructFailCount);
		return NULL;
	}
	*(uintptr_t*)(c + 256) = (uintptr_t)freshWB;

	InterlockedIncrement(&nmCloneConstructCount);
	return clone;
}

static void FreeClonedNMG(void* clone)
{
	if (!clone) return;
	uintptr_t wb = *(uintptr_t*)((char*)clone + 256);
	FreeFreshSettings((void*)wb);
	HavokTlsFree(clone, NMG_STRUCT_SIZE);
}


#if NMCACHE_STEP >= 4

// --------------------------------------------------------------------
// Worker dequeue (3-pass optimistic concurrency)
// --------------------------------------------------------------------
//
// Pass 1: acquire queue lock, peek head, compute cache key, release.
// Pass 2: L1 lookup under nmCacheCS; on L1 miss, L2 disk read (no lock)
//         promotes to L1 on HIT.
// Pass 3: re-acquire queue lock, verify head unchanged, dequeue.
//
// Pass-3's head check catches the rare case where another thread took the job
// between passes 2 and 3; we bail and retry on the next iteration.
//
// Lock ordering: nmCacheCS (outer) > queue lock (inner). bg thread's
// hook_dispatchJob releases the queue lock before calling ProcessNavMeshJob,
// so lock-order inversion with workers can't happen.
static uintptr_t WorkerTryDequeueAny(int* hitIdxOut, bool* isMissOut)
{
	*hitIdxOut = -1;
	*isMissOut = false;

	uintptr_t nmg = g_navMeshGen;
	if (!nmg) return 0;
	if (!*(uintptr_t*)(nmg + 136)) return 0;

	// Pass 1: peek under queue lock
	char initBuf[16];
	void* initResult = fn_pathBuilderInit(initBuf);
	fn_pathBuilderFinalize((void*)(nmg + 152), initResult);

	uintptr_t peekedJob = *(uintptr_t*)(nmg + 136);
	if (!peekedJob)
	{
		fn_readerUnlock((void*)(nmg + 152));
		return 0;
	}

	int jobType = *(int*)(peekedJob + 88) & 7;
	if (jobType != 0 && jobType != 1)
	{
		fn_readerUnlock((void*)(nmg + 152));
		return 0;  // types 2/3/4 stay on bg thread (stitching modifies shared edge data)
	}

	uintptr_t zone = *(uintptr_t*)peekedJob;
	if (!zone || !*(uintptr_t*)zone)
	{
		fn_readerUnlock((void*)(nmg + 152));
		return 0;
	}

	NavMeshCacheKey key;
	key.gridX = *(int*)(zone + OFF_ZONE_COORDS_X);
	key.gridY = *(int*)(zone + OFF_ZONE_COORDS_Y);
	key.sectionTileId = *(int*)(peekedJob + 32);
	key.jobType = jobType;
	key.aabbHash = HashAABB((float*)(peekedJob + 48));
	key.buildingHash = ComputeBuildingHash(zone);

	fn_readerUnlock((void*)(nmg + 152));

	// Pass 2: cache lookup (L1, then L2 fallback)
	int hitIdx = -1;

	EnterCriticalSection(&nmCacheCS);
	int found = FindCacheEntry(key);
	if (found >= 0 && nmCache[found].cachedFaces != NULL && fn_navMeshCtor != NULL)
		hitIdx = found;
	LeaveCriticalSection(&nmCacheCS);

#if NMCACHE_STEP >= 2
	if (hitIdx < 0 && fn_navMeshCtor != NULL)
	{
		NavMeshCacheEntry diskEntry;
		memset(&diskEntry, 0, sizeof(diskEntry));
		LARGE_INTEGER tR0, tR1;
		QueryPerformanceCounter(&tR0);
		bool l2Hit = ReadDiskCache(key, diskEntry);
		QueryPerformanceCounter(&tR1);
		long readUs = (long)(QPCToMs(tR0, tR1) * 1000.0);
		InterlockedExchangeAdd(&nmDiskReadUsTimes1, readUs);

		if (l2Hit)
		{
			EnterCriticalSection(&nmCacheCS);
			if (nmCache[nmCacheWriteIdx].valid)
				EvictCacheEntry(nmCacheWriteIdx);
			nmCache[nmCacheWriteIdx] = diskEntry;
			hitIdx = nmCacheWriteIdx;
			nmCacheWriteIdx = (nmCacheWriteIdx + 1) % NM_CACHE_SIZE;
			if (nmCacheFill < NM_CACHE_SIZE) nmCacheFill++;
			LeaveCriticalSection(&nmCacheCS);
			InterlockedIncrement(&nmDiskHitCount);
		}
		else
		{
			InterlockedIncrement(&nmDiskMissCount);
		}
	}
#endif

	// Pass 3: re-acquire queue lock, verify head, dequeue
	initResult = fn_pathBuilderInit(initBuf);
	fn_pathBuilderFinalize((void*)(nmg + 152), initResult);

	uintptr_t currentHead = *(uintptr_t*)(nmg + 136);
	if (currentHead != peekedJob)
	{
		fn_readerUnlock((void*)(nmg + 152));
		return 0;
	}

	uintptr_t nextJob = *(uintptr_t*)(peekedJob + 96);
	*(uintptr_t*)(nmg + 136) = nextJob;
	if (!nextJob)
		*(uintptr_t*)(nmg + 144) = nmg + 136;

	fn_readerUnlock((void*)(nmg + 152));

	InterlockedIncrement(&nmJobCount);
	*hitIdxOut = hitIdx;
	*isMissOut = (hitIdx < 0);
	return peekedJob;
}


// --------------------------------------------------------------------
// Worker HIT processing
// --------------------------------------------------------------------
//
// WorkerTryDequeueAny already looked up the cache entry. Here we just
// reconstruct the cached hkaiNavMesh and run buildCollision + job finalize.
// No processJobAlt. If the cache entry was evicted between dequeue and
// reconstruct (rare, only under cache pressure), fall through to the
// null-result finalize path.
static void WorkerProcessHit(void* nmg, uintptr_t job, int jobType, int hitIdx)
{
	InterlockedIncrement(&workerBusyCount);
	*(unsigned char*)((uintptr_t)nmg + 265) = 1;

	void* freshNavMesh = NULL;
	EnterCriticalSection(&nmCacheCS);
	if (nmCache[hitIdx].valid && nmCache[hitIdx].cachedFaces != NULL)
	{
		LARGE_INTEGER t0, t1;
		QueryPerformanceCounter(&t0);
		freshNavMesh = ReconstructNavMesh(nmCache[hitIdx]);
		QueryPerformanceCounter(&t1);
		long ms10 = (long)(QPCToMs(t0, t1) * 10.0);
		InterlockedExchangeAdd(&nmSavedMsTimes10, ms10);
	}
	LeaveCriticalSection(&nmCacheCS);

	if (freshNavMesh)
	{
		*(void**)(job + 72) = freshNavMesh;
		*(void**)(job + 80) = NULL;
		InterlockedIncrement(&nmCacheHitCount);
		fn_buildCollision(nmg, (void*)job);
	}

	void* label29NavInst = *(void**)(job + 80);
	if (label29NavInst)
		*(int*)((uintptr_t)label29NavInst + 64) = *(int*)(job + 32);

	uintptr_t navMeshResult = *(uintptr_t*)(job + 72);
	int faceCount = navMeshResult ? *(int*)(navMeshResult + 24) : 0;
	if (navMeshResult && faceCount > 0)
	{
		fn_enqueueToProcQueue((void*)((uintptr_t)nmg + 184), (void*)job);
	}
	else
	{
		void* delNavInst = *(void**)(job + 80);
		if (delNavInst) fn_gameDelete(delNavInst);
		void* buildingRef = *(void**)(job + 24);
		if (buildingRef) fn_gameDelArr(buildingRef);
		fn_gameDelete((void*)job);
	}

	if (InterlockedDecrement(&workerBusyCount) == 0)
		*(unsigned char*)((uintptr_t)nmg + 265) = 0;
}


// --------------------------------------------------------------------
// Worker thread entry
// --------------------------------------------------------------------

DWORD WINAPI NavMeshWorkerProc(LPVOID param)
{
	int workerId = (int)(uintptr_t)param;

	// Havok thread init (5-step sequence, matches Kenshi's 4 game threads):
	//   contextInit → getManager → manager->vt+24(ctx, name, 3)
	//     → postRegInit → _mm_setcsr denormal flush
	// Step 3 populates both Havok TLS slots — workers can't call the
	// router-dependent allocator without it.
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
	_mm_setcsr(_mm_getcsr() | 0x8000);

	{
		std::ostringstream ss;
		ss << "[ZoneOpt] Worker " << workerId << " started, Havok TLS initialized";
		LogMsg(ss.str());
	}

	while (!g_workerShutdown)
	{
		WaitForSingleObject(g_jobEvent, 500);
		if (g_workerShutdown) break;

		int hitIdx = -1;
		bool isMiss = false;
		uintptr_t job = WorkerTryDequeueAny(&hitIdx, &isMiss);
		if (!job) continue;

		int jobType = *(int*)(job + 88) & 7;

		if (isMiss)
		{
			// Worker MISS: clone the NMG so processJobAlt operates on our own
			// workBuffer + queue state. On clone-alloc failure, fall back to
			// running on realNMG under processJobCS — same path the bg thread
			// uses.
			void* clonedNMG = CloneNMG((void*)g_navMeshGen);
			if (!clonedNMG)
			{
				LogMsg("[ZoneOpt] Worker: CloneNMG failed, fallback to realNMG");
				ProcessNavMeshJob((void*)g_navMeshGen, (void*)g_navMeshGen, job, jobType);
			}
			else
			{
				ProcessNavMeshJob((void*)g_navMeshGen, clonedNMG, job, jobType);
				FreeClonedNMG(clonedNMG);
			}
		}
		else
		{
			WorkerProcessHit((void*)g_navMeshGen, job, jobType, hitIdx);
		}
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
	if (!fn_havokContextInit || !fn_havokGetManager || !fn_havokPostRegInit)
	{
		LogMsg("[ZoneOpt] NavMesh workers: SKIPPED (Havok fn ptrs missing)");
		return;
	}

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

#endif // NMCACHE_STEP >= 4


// --------------------------------------------------------------------
// Job processing pipeline
// --------------------------------------------------------------------
//
// Called by both the NavMesh bg thread (via hook_dispatchJob) and workers
// (for MISSes, via NavMeshWorkerProc).
//
//   realNMG = game's actual NavMeshGenerator. Always used for buildCollision
//             and result enqueue (section BST + output queue are shared).
//   workNMG = either realNMG (bg thread, or worker when CloneNMG failed) or a
//             per-worker clone (worker happy path).
//
// Cache HITs: reconstruct from L1/L2. No processJobCS — ReconstructNavMesh
// and buildCollision are thread-safe.
//
// Cache MISSes: processJobCS serializes fn_processJobAlt with the bg thread
// and any other worker. When workNMG==realNMG we use a swap-settings trick —
// temporarily override realNMG+256 with a freshly-built settings block,
// restore it before LeaveCS so buildCollision sees the original. When
// workNMG!=realNMG the clone already has its own settings installed.

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
		InterlockedExchange(&nmDiagStep, 30);

		LARGE_INTEGER t0, t1;
		QueryPerformanceCounter(&t0);

		InterlockedExchange(&nmDiagStep, 31);

		// Prepare the workBuffer fn_processJobAlt will see.
		//   workNMG!=realNMG → CloneNMG already installed freshWB at workNMG+256.
		//   workNMG==realNMG → allocate freshWB and swap realNMG+256 for the
		//     duration of processJobAlt. Restore before LeaveCS so
		//     buildCollision sees the original.
		void* localFreshWB = NULL;
		uintptr_t localOrigWB = 0;
		bool usingNMGClone = (workNMG != realNMG);
		if (!usingNMGClone)
		{
			localOrigWB = *(uintptr_t*)((uintptr_t)realNMG + 256);
			localFreshWB = ConstructFreshSettings(localOrigWB);
			if (localFreshWB)
				InterlockedIncrement(&nmCloneConstructCount);
			else
				InterlockedIncrement(&nmCloneConstructFailCount);
		}
		bool cloneActive = usingNMGClone || (localFreshWB != NULL);

		EnterCriticalSection(&processJobCS);

		// Duplicate-job check. The game submits the same zone more than once
		// (it re-registers sections; Step2A-fixed 2026-09-10: a worker and then
		// the bg thread both generated (21,41), 14 s + 17 s back-to-back while
		// the game waited to exit). If another thread finished this exact key
		// while we waited for processJobCS, L1 has it (stored under the lock):
		// take the HIT instead of generating again.
		void* lateMesh = NULL;
		if (nmDiagStage >= 2 && fn_navMeshCtor != NULL
		    && !InterlockedCompareExchange(&nmCacheDisabled, 0, 0))
		{
			EnterCriticalSection(&nmCacheCS);
			int lateIdx = FindCacheEntry(key);
			if (lateIdx >= 0 && nmCache[lateIdx].cachedFaces != NULL)
				lateMesh = ReconstructNavMesh(nmCache[lateIdx]);
			LeaveCriticalSection(&nmCacheCS);
		}

		if (lateMesh)
		{
			LeaveCriticalSection(&processJobCS);
			if (localFreshWB)
				FreeFreshSettings(localFreshWB);

			*(void**)(job + 72) = lateMesh;
			*(void**)(job + 80) = NULL;
			InterlockedIncrement(&nmCacheHitCount);
			InterlockedIncrement(&nmLateHitCount);
			{
				std::ostringstream ss;
				ss << "[ZoneOpt] Late HIT (duplicate job): grid=(" << gridX << "," << gridY
				   << ") type=" << jobType << (usingNMGClone ? " [worker]" : " [bg]");
				LogMsg(ss.str());
			}

			InterlockedExchange(&nmDiagStep, 23);
			fn_buildCollision(realNMG, (void*)job);
			InterlockedExchange(&nmDiagStep, 24);

			QueryPerformanceCounter(&t1);
			long ms10 = (long)(QPCToMs(t0, t1) * 10.0);
			InterlockedExchangeAdd(&nmSavedMsTimes10, ms10);
		}
		else
		{
			InterlockedIncrement(&nmCacheMissCount);
#if NMCACHE_STEP >= 4
			if (workNMG != realNMG)
				InterlockedIncrement(&nmWorkerMissCount);
			else
				InterlockedIncrement(&nmBgMissCount);
#endif

			{
				if (usingNMGClone)
				{
					std::ostringstream ss;
					ss << "[ZoneOpt] MISS NMG-clone: realNMG=" << realNMG
					   << " workNMG=" << workNMG
					   << " workWB=" << *(void**)((uintptr_t)workNMG + 256)
					   << " grid=(" << gridX << "," << gridY << ") type=" << jobType;
					LogMsg(ss.str());
				}
				else if (localFreshWB)
				{
					*(uintptr_t*)((uintptr_t)realNMG + 256) = (uintptr_t)localFreshWB;

					std::ostringstream ss;
					ss << "[ZoneOpt] MISS swap: realNMG=" << realNMG
					   << " origWB=" << (void*)localOrigWB << " freshWB=" << localFreshWB
					   << " grid=(" << gridX << "," << gridY << ") type=" << jobType;
					LogMsg(ss.str());
				}
				else
				{
					LogMsg("[ZoneOpt] MISS no-swap: using real WB (ConstructFreshSettings failed)");
				}

				EnsureGlobalScratchBuffer();
			}

			LogMsg("[ZoneOpt] MISS: entering processJobAlt");

			if (cloneActive)
				InterlockedExchange(&g_cloneProcessing, 1);

			fn_processJobAlt(workNMG, (void*)job);

			InterlockedExchange(&g_cloneProcessing, 0);

			InterlockedExchange(&nmDiagStep, 32);

			if (jobType == 1)
				fn_partialFixup(workNMG, (void*)job);

			// Swap-path: restore realNMG+256 before releasing processJobCS so
			// buildCollision and later code see the original.
			if (localFreshWB)
				*(uintptr_t*)((uintptr_t)realNMG + 256) = localOrigWB;

			// L1 store while still holding processJobCS: a duplicate job waiting
			// on the lock then finds the result (late HIT) instead of generating
			// it again. The cached copy is the pre-buildCollision mesh, which is
			// exactly what the HIT path feeds into buildCollision.
			uintptr_t storedResult = 0;
			if (nmDiagStage >= 2 && !InterlockedCompareExchange(&nmCacheDisabled, 0, 0))
			{
				storedResult = *(uintptr_t*)(job + 72);
				if (storedResult)
				{
					EnterCriticalSection(&nmCacheCS);
					StoreCacheEntry(key, storedResult);
					LeaveCriticalSection(&nmCacheCS);
				}
			}

			LeaveCriticalSection(&processJobCS);

			InterlockedExchange(&nmDiagStep, 33);

			fn_buildCollision(realNMG, (void*)job);

			InterlockedExchange(&nmDiagStep, 34);

			QueryPerformanceCounter(&t1);
			long ms10 = (long)(QPCToMs(t0, t1) * 10.0);
			InterlockedExchangeAdd(&nmTotalMsTimes10, ms10);

#if NMCACHE_STEP >= 2
			if (storedResult)
			{
				LARGE_INTEGER tW0, tW1;
				QueryPerformanceCounter(&tW0);
				WriteDiskCache(key, storedResult);
				QueryPerformanceCounter(&tW1);
				long writeUs = (long)(QPCToMs(tW0, tW1) * 1000.0);
				InterlockedExchangeAdd(&nmDiskWriteUsTimes1, writeUs);
			}
#endif

			// Free the swap-path freshWB (no dtor — we skipped sub-struct init so a
			// dtor would double-free whatever shared state might exist) plus its
			// override-settings guard buffer. Bounded per-MISS leak: the hkArray
			// data buffers appended during processJobAlt are released when realWB
			// is torn down at zone unload.
			if (localFreshWB)
				FreeFreshSettings(localFreshWB);

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


// --------------------------------------------------------------------
// Hooks
// --------------------------------------------------------------------

void hook_realGenerate(void* workBuffer, void* localData, void* hkaiNavMesh, int param, int timeLowPart)
{
	// MinHook can't relocate the first instructions of the 61K-byte realGenerate,
	// so this hook is never actually installed. Body kept as a passthrough for
	// the function-pointer slot.
	orig_realGenerate(workBuffer, localData, hkaiNavMesh, param, timeLowPart);
}

void hook_nmResultPopulate_diag(void* navData, void* localData, void* result, int param, int lowPart)
{
	orig_nmResultPopulate(navData, localData, result, param, lowPart);
}


static bool nmBypassHook = false;

// NavMesh bg thread entry. Runs the step-3 probes and tuning, then dequeues
// one job and processes it (HIT via cache reconstruction, MISS via
// ProcessNavMeshJob's swap-settings path). At step >= 4, workers also dequeue
// from this queue, so the peek runs under the queue lock.
char hook_dispatchJob(void* thisNMG)
{
	if (!g_navMeshBgThreadId)
	{
		if (InterlockedCompareExchange((volatile LONG*)&g_navMeshBgThreadId,
		                               (LONG)GetCurrentThreadId(), 0) == 0)
		{
			std::ostringstream ts;
			ts << "[ZoneOpt] NavMesh bg thread TID=" << g_navMeshBgThreadId;
			LogMsg(ts.str());
		}
	}

	if (nmBypassHook)
		return orig_dispatchJob(thisNMG);

	uintptr_t nmg = (uintptr_t)thisNMG;

	if (!g_navMeshGen)
		g_navMeshGen = nmg;

#if NMCACHE_STEP >= 3
	// FLA CS hooks install lazily on first dispatch — Havok world is guaranteed
	// live here so the probe's TlsGetValue returns a valid router.
	InstallHavokHeapHooks();

	ProbeNavMeshSettings(nmg);
	ApplyNavMeshQualityTuning(nmg);
	VerifyNavMeshSettings(nmg);
	ProbeWorkBufferSize(nmg);
#endif

#if NMCACHE_STEP >= 4
	{
		static volatile long workersCreated = 0;
		if (!InterlockedCompareExchange(&workersCreated, 1, 0))
			CreateNavMeshWorkers();
	}
#endif

#if NMCACHE_STEP >= 4
	// Multi-consumer dequeue: workers may free jobs concurrently, so peek
	// under the queue lock.
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
	// Single-consumer path (workers absent): unlocked peek is safe.
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
	// Signal workers that a new job is available.
	if (g_jobEvent && *(uintptr_t*)(nmg + 136))
		SetEvent(g_jobEvent);
#endif

	// HITs process concurrently with workers; MISSes serialize via processJobCS.
	ProcessNavMeshJob(thisNMG, thisNMG, job, jobType);
	return 1;
}


#endif // NMCACHE_STEP >= 1
