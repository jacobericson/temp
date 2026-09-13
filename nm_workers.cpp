// nm_workers.cpp — NavMesh worker pool, cloning, and job pipeline
//
// 3 worker threads parallelize NavMesh generation alongside Kenshi's
// single-threaded NavMesh bg thread. Workers dequeue jobs from the shared
// input queue, reconstruct cached results for L1/L2 HITs (concurrent), and
// steal MISSes under processJobCS (serialized with bg thread).
//
// Full design history in research/archive/worker_clone_investigation.md.
// Stage 4d (parallel MISSes, processJobCS released) is structurally infeasible
// under Kenshi's HK_CONFIG_SINGLE_THREADED Havok build — see
// research/archive/stage4d_infeasibility.md.

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
volatile long   g_workerInitFailed[NAVMESH_WORKER_COUNT] = {};

HANDLE WorkerSlotHandle(int i)
{
	if (i < 0 || i >= NAVMESH_WORKER_COUNT) return NULL;
	if (InterlockedCompareExchange(&g_workerInitFailed[i], 0, 0)) return NULL;
	HANDLE h = g_workerHandles[i];
	if (h == INVALID_HANDLE_VALUE) return NULL;
	return h;
}

// MISSes serialize via processJobCS, so a global flag is sufficient to tell
// hook_edgeProcess we're finalizing on a clone.
static volatile long g_cloneProcessing = 0;

// Worker calls into ProcessNavMeshJob already hold the busy bridge from claim
// time at NMFIX 8; below that gate the function still does its own accounting.
#if NMFIX_STEP >= 8
  #define WORKER_BUSY_HELD true
#else
  #define WORKER_BUSY_HELD false
#endif

// --------------------------------------------------------------------
// processJobCS ownership
// --------------------------------------------------------------------
//
// Every Enter/Leave of processJobCS goes through this pair so the thread id of
// the current owner is always known. The tripwire on processJobAlt (NMFIX 5)
// reads it to tell "called under the lock" from "called without it".
//
// The critical section is recursive, but nothing here recurses on it: CloneNMG
// leaves before ProcessNavMeshJob enters, and the first-dispatch latch block
// and the orig_dispatchJob wrap are each a single flat region. A plain owner
// slot is therefore enough; if recursion is ever introduced this needs a depth
// count instead.
static volatile long g_processJobOwnerTid = 0;

static inline void EnterProcessJobCS()
{
	EnterCriticalSection(&processJobCS);
	InterlockedExchange(&g_processJobOwnerTid, (long)GetCurrentThreadId());
}

static inline void LeaveProcessJobCS()
{
	InterlockedExchange(&g_processJobOwnerTid, 0);
	LeaveCriticalSection(&processJobCS);
}

// RAII over the pair above, the same shape BuildCollisionScope gives
// buildCollisionCS. Every region that holds processJobCS calls into game code
// with C++ unwind state (processJobAlt, partialGeneration, the settings ctor
// and dtor, orig_dispatchJob), and a processJobCS left held by an unwinding
// frame parks every MISS, every type 2/3/4 dispatch and every worker clone
// behind it for the rest of the session. The destructor releases on that path.
//
// Release() is for the one region whose normal paths let go before the scope
// ends: the MISS block in ProcessNavMeshJob drops the lock at the top of the
// late-HIT branch and after the L1 store on the generate branch, both inside
// the scope that owns the guard. Calling it keeps those exact release points;
// the destructor then has nothing left to do. It is idempotent.
//
// The owner-tid bookkeeping stays in EnterProcessJobCS/LeaveProcessJobCS, so the
// processJobAlt tripwire reads the same thing whether a region uses the guard
// or not.
struct ProcessJobLock
{
	bool held;

	ProcessJobLock() : held(true) { EnterProcessJobCS(); }
	~ProcessJobLock() { Release(); }

	void Release()
	{
		if (!held) return;
		held = false;
		LeaveProcessJobCS();
	}

private:
	ProcessJobLock(const ProcessJobLock&);
	ProcessJobLock& operator=(const ProcessJobLock&);
};

// --------------------------------------------------------------------
// Claimed-job zone re-check (Round 2 session 1 crash)
// --------------------------------------------------------------------
//
// dispatchJob_orig (0x3CE030) pops a job and, immediately before processJobAlt,
// returns 1 when the job's zone has no mapContent (`if (!**job) return 1;`),
// leaving the node neither freed nor enqueued. ZoneManager__unloadSingleZone
// (0xA09620, main thread) NULLs mapContent (+0) first and then frees and NULLs
// terrainCollision (+0xB8), and nothing in the unload path waits on the
// generator. So vanilla drops every job whose zone was unloaded before the bg
// thread reached it.
//
// The mod's claim sites make the same test at claim time, but a claimed job can
// then wait on processJobCS for seconds (a cold-cache MISS holds it 2-10 s) and
// the zone can be unloaded in between. processJobAlt reads
// *(zone+0xB8)+8 unconditionally for type 0/1 jobs (0x3C1580 via 0xA07B50, from
// 0x3CBE60+0xB3C), which is the Round 2 session 1 access violation at rva
// 0x3C1600. This re-check restores vanilla's test after the last lock wait.
//
// terrainCollision is tested as well as mapContent: every job this code runs is
// type 0/1, where vanilla's processJobAlt dereferences it with no NULL check, so
// a job with content but no terrain would crash vanilla too.
//
// ZoneMap entries live in the ZoneManager's fixed array and are never freed, so
// job+0 and the zone's fields stay readable after an unload. Plain reads, no
// allocation: runs on the NavMesh bg thread and the workers.
static bool JobZoneStillLoaded(uintptr_t job, int* reasonOut)
{
	uintptr_t zone = *(uintptr_t*)job;
	int reason = STALE_REASON_NONE;
	if (!zone)
		reason = STALE_REASON_NO_ZONE;
	else if (!*(uintptr_t*)(zone + OFF_ZONE_CONTENT))
		reason = STALE_REASON_NO_CONTENT;
	else if (!*(uintptr_t*)(zone + OFF_ZONE_TERRAIN_COLLISION))
		reason = STALE_REASON_NO_TERRAIN;
	if (reasonOut) *reasonOut = reason;
	return reason == STALE_REASON_NONE;
}

// Measurement helpers for the stats line's pjWait / claimAge / stale tokens.
// Interlocked only; safe on any thread.
static inline void NoteMaxUs(volatile long* slot, long us)
{
	for (;;)
	{
		long prev = InterlockedCompareExchange(slot, 0, 0);
		if (us <= prev) break;
		if (InterlockedCompareExchange(slot, us, prev) == prev) break;
	}
}

// Microseconds between two QPC values, clamped to what a 32-bit max slot holds
// (about 35 minutes).
static inline long QpcDeltaUs(LONGLONG from, LONGLONG to)
{
	if (to <= from || qpcFrequency.QuadPart <= 0) return 0;
	LONGLONG us = (to - from) * 1000000 / qpcFrequency.QuadPart;
	return us > 0x7FFFFFFF ? 0x7FFFFFFF : (long)us;
}

static inline LONGLONG QpcNow()
{
	LARGE_INTEGER t;
	QueryPerformanceCounter(&t);
	return t.QuadPart;
}

static inline void NotePjWait(int site, LONGLONG before, LONGLONG after)
{
	long us = QpcDeltaUs(before, after);
	InterlockedIncrement(&nmPjWaitCount[site]);
	InterlockedExchangeAdd64(&nmPjWaitTotalUs[site], (LONGLONG)us);
	NoteMaxUs(&nmPjWaitMaxUs[site], us);
}

// claimQpc 0 means the caller had no claim time; nothing is recorded.
static inline void NoteClaimAge(bool isMiss, LONGLONG claimQpc, LONGLONG now)
{
	if (!claimQpc) return;
	long us = QpcDeltaUs(claimQpc, now);
	if (isMiss)
	{
		InterlockedIncrement(&nmClaimAgeMissCount);
		InterlockedExchangeAdd64(&nmClaimAgeMissTotalUs, (LONGLONG)us);
		NoteMaxUs(&nmClaimAgeMissMaxUs, us);
		int b = (us < 10000) ? 0 : (us < 100000) ? 1 : (us < 1000000) ? 2 : (us < 5000000) ? 3 : 4;
		InterlockedIncrement(&nmClaimAgeMissBucket[b]);
	}
	else
	{
		InterlockedIncrement(&nmClaimAgeHitCount);
		InterlockedExchangeAdd64(&nmClaimAgeHitTotalUs, (LONGLONG)us);
		NoteMaxUs(&nmClaimAgeHitMaxUs, us);
	}
}

static void NoteStaleDrop(int site, uintptr_t job, int jobType, int reason, LONGLONG claimQpc)
{
	InterlockedIncrement(&nmStaleCount[site]);

	uintptr_t zone = *(uintptr_t*)job;
	long gx = zone ? *(int*)(zone + OFF_ZONE_COORDS_X) : -1;
	long gy = zone ? *(int*)(zone + OFF_ZONE_COORDS_Y) : -1;
	long ageUs = claimQpc ? QpcDeltaUs(claimQpc, QpcNow()) : 0;

	// Five separate writes: a racing reader may see a torn last event, which
	// the stats line accepts (nm_cache_core.h).
	InterlockedExchange(&nmStaleLastGridX, gx);
	InterlockedExchange(&nmStaleLastGridY, gy);
	InterlockedExchange(&nmStaleLastType, (long)jobType);
	InterlockedExchange(&nmStaleLastReason, (long)reason);
	InterlockedExchange(&nmStaleLastAgeUs, ageUs);
}

// --------------------------------------------------------------------
// processJobCS for the save-load reset (Round 2 fix 2b review)
// --------------------------------------------------------------------
//
// nm_workers.h has the contract. The try loop never blocks for longer than
// timeoutMs, so a holder that is itself waiting on the main thread (whatever
// the reason) costs the reset at most the bound, never a hang. On success the
// owner tid is set exactly as EnterProcessJobCS sets it, and the release goes
// through LeaveProcessJobCS, so the processJobAlt tripwire (NMFIX 5) reads the
// same bookkeeping as for every other holder.
//
// Readiness is an explicit flag set by startPlugin right after
// InitNavMeshCacheCS, rather than a guess from the CRITICAL_SECTION's internals.
static volatile long g_pjLockReady = 0;

void NavMeshMarkProcessJobLockReady()
{
	InterlockedExchange(&g_pjLockReady, 1);
}

NavMeshPjLockResult NavMeshTryLockProcessJobFor(DWORD timeoutMs, DWORD* waitedMs)
{
	if (waitedMs) *waitedMs = 0;
	if (!InterlockedCompareExchange(&g_pjLockReady, 0, 0))
		return NM_PJLOCK_NONE;

	LONGLONG start = QpcNow();
	for (;;)
	{
		if (TryEnterCriticalSection(&processJobCS))
		{
			InterlockedExchange(&g_processJobOwnerTid, (long)GetCurrentThreadId());
			if (waitedMs) *waitedMs = (DWORD)(QpcDeltaUs(start, QpcNow()) / 1000);
			return NM_PJLOCK_HELD;
		}
		DWORD elapsedMs = (DWORD)(QpcDeltaUs(start, QpcNow()) / 1000);
		if (elapsedMs >= timeoutMs)
		{
			if (waitedMs) *waitedMs = elapsedMs;
			return NM_PJLOCK_TIMEOUT;
		}
		Sleep(1);
	}
}

void NavMeshUnlockProcessJob()
{
	LeaveProcessJobCS();
}

#if NMFIX_STEP >= 6
// --------------------------------------------------------------------
// buildCollisionCS
// --------------------------------------------------------------------
//
// Serializes the three regions that build section collision:
//   - buildCollision (0x3CB700), hooked
//   - buildCollisionInterior (0x3CBAB0), hooked
//   - the partialGeneration call in ProcessNavMeshJob's MISS tail
// The hooks cover the game's own calls from dispatchJob_orig and the mod's
// fn_buildCollision calls alike, since they patch the function entry.
//
// Lock order is processJobCS -> buildCollisionCS, never the reverse.
// buildCollisionCS is a leaf: nothing of ours runs inside it, the hook bodies
// only forward to the original, and it is never held while waiting on
// processJobCS or nmCacheCS.
//
// g_inBuild is bumped BEFORE the lock, so bcOverlap= counts build regions in
// flight > 1 rather than lock waits. That is the necessary condition for the
// unguarded tail races, not proof one happened: two threads can both be in a
// build region and still be serialized by the game's own build mutex inside
// buildSectionCollision.
static volatile long g_inBuild = 0;

static inline void NoteBcMax(volatile long* slot, long us)
{
	for (;;)
	{
		long prev = InterlockedCompareExchange(slot, 0, 0);
		if (us <= prev) break;
		if (InterlockedCompareExchange(slot, us, prev) == prev) break;
	}
}

// RAII, because buildCollision and partialGeneration carry C++ unwind state
// (both decompile with "Hidden C++ exception states"). A plain Enter / call /
// Leave would skip the Leave if anything unwound through this frame, and a
// buildCollisionCS left held hangs every NavMesh thread. The destructor runs on
// that path.
//
// Timing lives here so the wait and the hold are measured across one object:
//   wait = time blocked in EnterCriticalSection
//   hold = time from acquiring to releasing
// Totals are 64-bit microseconds (a 32-bit microsecond total overflows after
// about six hours of 100 ms builds); the maxima are 32-bit, which is 35 minutes
// of single hold and cannot realistically overflow.
struct BuildCollisionScope
{
	LARGE_INTEGER holdStart;

	BuildCollisionScope()
	{
		// Bumped before the lock, so the count reflects regions in flight
		// rather than lock waits.
		if (InterlockedIncrement(&g_inBuild) > 1)
			InterlockedIncrement(&g_buildOverlapSeen);

		LARGE_INTEGER w0;
		QueryPerformanceCounter(&w0);
		EnterCriticalSection(&buildCollisionCS);
		QueryPerformanceCounter(&holdStart);

		long waitUs = (long)(QPCToMs(w0, holdStart) * 1000.0);
		InterlockedIncrement(&nmBcCount);
		InterlockedExchangeAdd64(&nmBcWaitTotalUs, (LONGLONG)waitUs);
		NoteBcMax(&nmBcWaitMaxUs, waitUs);
	}

	~BuildCollisionScope()
	{
		LARGE_INTEGER h1;
		QueryPerformanceCounter(&h1);
		long holdUs = (long)(QPCToMs(holdStart, h1) * 1000.0);
		InterlockedExchangeAdd64(&nmBcHoldTotalUs, (LONGLONG)holdUs);
		NoteBcMax(&nmBcHoldMaxUs, holdUs);

		LeaveCriticalSection(&buildCollisionCS);
		InterlockedDecrement(&g_inBuild);
	}

private:
	BuildCollisionScope(const BuildCollisionScope&);
	BuildCollisionScope& operator=(const BuildCollisionScope&);
};

// The real signatures, from the decompiles. buildCollision takes four
// arguments; a3 is unused in its body and a4 reaches stitch_buildCollision
// (0x3C5C10), which overwrites it before use — but the hook forwards all four
// so the trampoline call is faithful whatever the caller passed.
typedef __int64 (*buildCollisionFull_t)(void* thisNMG, void* job, __int64 a3, double a4);
typedef __int64 (*buildCollisionInterior_t)(void* thisNMG, void* job);

static buildCollisionFull_t     orig_buildCollisionHook  = NULL;
static buildCollisionInterior_t orig_buildInteriorHook   = NULL;

static __int64 hook_buildCollision(void* thisNMG, void* job, __int64 a3, double a4)
{
	BuildCollisionScope guard;
	return orig_buildCollisionHook(thisNMG, job, a3, a4);
}

static __int64 hook_buildCollisionInterior(void* thisNMG, void* job)
{
	BuildCollisionScope guard;
	return orig_buildInteriorHook(thisNMG, job);
}
#endif // NMFIX_STEP >= 6

#if NMFIX_STEP >= 5
// --------------------------------------------------------------------
// processJobAlt ownership tripwire
// --------------------------------------------------------------------
//
// Every caller of processJobAlt (0x3CBE60) must hold processJobCS: it works on
// the generator's shared work buffer, the global scratch buffer and the seed
// map. This pass-through counts any call from a thread that does not own the
// lock, which covers both the mod's own calls and the game's (orig_dispatchJob
// runs processJobAlt on the real work buffer for type 3).
//
// The hook patches the function entry, so the mod's fn_processJobAlt calls go
// through it too. That is deliberate: the point is to catch a violation
// wherever it comes from.
static processJobAlt_t orig_processJobAltTrip = NULL;

static void hook_processJobAltTrip(void* thisNMG, void* job)
{
	if (InterlockedCompareExchange(&g_processJobOwnerTid, 0, 0) != (long)GetCurrentThreadId())
		InterlockedIncrement(&nmTripCount);
	orig_processJobAltTrip(thisNMG, job);
}

// --------------------------------------------------------------------
// orig_dispatchJob under processJobCS
// --------------------------------------------------------------------
//
// dispatchJob_orig's type-3 branch runs processJobAlt on the real generator and
// work buffer, and it frees the global scratch buffer when the queue empties
// (0x3CE2A9). Both are exactly the state processJobCS serializes, so the
// original has to run under it.
//
// measure=true records the hold for the type 2/3/4 path, the one this wrap
// exists for; the bad-zone and bypass forwards are wrapped for correctness but
// are not part of that measurement.
static char CallOrigDispatchLocked(void* thisNMG, bool measure)
{
	ProcessJobLock lock;   // held for the whole function, released on return

	LARGE_INTEGER t0, t1;
	if (measure) QueryPerformanceCounter(&t0);

	char r = orig_dispatchJob(thisNMG);

	if (measure)
	{
		QueryPerformanceCounter(&t1);
		long ms10 = (long)(QPCToMs(t0, t1) * 10.0);
		InterlockedIncrement(&nmT234Count);
		InterlockedExchangeAdd(&nmT234TotalMsTimes10, ms10);
		for (;;)
		{
			long prev = InterlockedCompareExchange(&nmT234MaxMsTimes10, 0, 0);
			if (ms10 <= prev) break;
			if (InterlockedCompareExchange(&nmT234MaxMsTimes10, ms10, prev) == prev) break;
		}
	}

	return r;   // ~ProcessJobLock: same release point as the old trailing Leave
}
#endif // NMFIX_STEP >= 5

#if NMFIX_STEP >= 3
// wb+328 as written on the last fresh work buffer, raw float bits (NMFIX 3).
volatile long g_wbQualityLast = 0;

// Set once the first-dispatch probes have run against a real work buffer, so
// hook_dispatchJob stops taking processJobCS on every dispatch.
static volatile long g_firstDispatchDone = 0;
#endif

// Input geometry size of the generation running on this thread, captured by
// hook_nmResultPopulate_diag and read back at the store site. -1 means the hook
// did not run for this job (not installed, or the job never reached populate).
static __declspec(thread) int t_lastInputTriCount  = -1;
static __declspec(thread) int t_lastInputVertCount = -1;


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


typedef void (__fastcall *edgeProcess_t)(void* entry);


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


#if NMFIX_STEP >= 8
// Retires the worker pool before the game tears the NavMesh down.
// NavMesh::stop (0x3AAE90) clears +0x1C8, joins the path thread, deletes the
// manager and shuts the Havok memory system down. A worker still generating at
// that point reads freed state, and one still registered with Havok outlives
// the allocator it allocates from.
//
// Bounded on purpose: this runs on a game thread during shutdown, so it waits
// at most WORKER_RETIRE_TIMEOUT_MS and then proceeds regardless. Proceeding
// with a worker still running is bad, but hanging the shutdown is worse and
// more visible.
//
// The bound is 15 s, not the 2 s it started at: a worker in a MISS spends
// 1.7-2.2 s on average inside generation and can exceed 2 s, so the old bound
// timed out on a perfectly healthy worker and let NavMesh::stop free the state
// underneath it — the `joined=TIMEOUT live=1` line before the 0xE67E9A crash
// record in BugReports/3-warm-cache-stuck. 15 s clears one generation with room
// to spare while still refusing to hang the quit indefinitely. The line reports
// the wait actually spent, so a real hang is still visible as a long wait.
void RetireNavMeshWorkers()
{
	if (InterlockedExchange(&g_workerShutdown, 1) != 0)
		return;   // already retired

	// Manual-reset: one set releases every waiter and stays signalled, and the
	// workers' clear path skips the reset while shutdown is set.
	if (g_jobEvent)
		SetEvent(g_jobEvent);

	int activeCount = 0;
	HANDLE active[NAVMESH_WORKER_COUNT];
	for (int i = 0; i < NAVMESH_WORKER_COUNT; ++i)
	{
		// One snapshot per slot, and never a NULL into active[]:
		// WaitForMultipleObjects fails the whole array on one invalid handle,
		// which would return WAIT_FAILED immediately and log a join that never
		// happened while the surviving workers ran on.
		HANDLE h = WorkerSlotHandle(i);
		if (h)
			active[activeCount++] = h;
	}

	// See the header comment: one MISS averages 1.7-2.2 s, so the bound has to
	// clear a whole generation.
	const DWORD WORKER_RETIRE_TIMEOUT_MS = 15000;

	DWORD wait = WAIT_OBJECT_0;
	DWORD waitErr = 0;
	double waitMs = 0.0;
	if (activeCount > 0)
	{
		double t0 = ElapsedSec();
		wait = WaitForMultipleObjects(activeCount, active, TRUE, WORKER_RETIRE_TIMEOUT_MS);
		// Captured immediately: anything in between would overwrite it.
		if (wait == WAIT_FAILED)
			waitErr = GetLastError();
		waitMs = (ElapsedSec() - t0) * 1000.0;
	}

	// Three outcomes, not two: WAIT_FAILED means the wait never happened (a
	// closed or invalid handle in the array), so the workers may still be
	// running — reporting that as "ok" hid exactly the case worth seeing.
	const char* joined = "ok";
	if (wait == WAIT_TIMEOUT)      joined = "TIMEOUT";
	else if (wait == WAIT_FAILED)  joined = "FAILED";

	std::ostringstream ss;
	ss << "[ZoneOpt] NavMesh workers retired at NavMesh::stop: " << activeCount
	   << " joined=" << joined
	   << " waitMs=" << std::fixed << std::setprecision(0) << waitMs
	   << "/" << WORKER_RETIRE_TIMEOUT_MS;
	if (wait == WAIT_FAILED)
		ss << " gle=" << waitErr;
	ss << " live=" << InterlockedCompareExchange(&g_navMeshWorkersLive, 0, 0);
	LogMsg(ss.str());
}

typedef void (*navMeshStop_t)(void* navMesh);
static navMeshStop_t orig_navMeshStop = NULL;

static void hook_navMeshStop(void* navMesh)
{
	// First, before the retire: from here on the game is tearing the navmesh
	// system down, and the crash recorder tags any record written after this
	// point (core.h). Set before the worker wait, because that wait is up to
	// 15 s and a fault during it belongs to the teardown too.
	InterlockedExchange(&g_navMeshStopSeen, 1);
	RetireNavMeshWorkers();
	orig_navMeshStop(navMesh);
}
#endif // NMFIX_STEP >= 8

// --------------------------------------------------------------------
// Lazy hook install (first dispatch)
// --------------------------------------------------------------------
//
// Hooks that need the Havok world live, installed on the first dispatchJob
// rather than from startPlugin: the edgeProcess clone-guard, the populate
// pass-through that reads each generation's input triangle count, and the
// processJobAlt ownership tripwire.
//
// The four hkFreeListAllocator wrappers that used to be installed here are
// gone (NMFIX 1 / B5). The Havok heap is per thread in front and locked behind
// its own critical section already, so the wrappers protected nothing — and one
// of them, RVA 0xBCCCB0, is the allocator's destructor, not garbageCollect, so
// wrapping it was a defect in its own right. See navmesh_miss_split.md §6 #1.
//
// A failed install disables nothing: each one logs once and the rest carry on.

static void InstallNavMeshLazyHooks()
{
	if (InterlockedCompareExchange(&lazyHooksInstalled, 1, 0) != 0)
		return;

	// B1: VerifyPrologue
	if (VerifyPrologueByRva(RVA_EDGE_PROCESS)
	    && KenshiLib::SUCCESS == KenshiLib::AddHook(GameAddr(RVA_EDGE_PROCESS),
			(void*)hook_edgeProcess, (void**)&orig_edgeProcess))
		LogMsg("[ZoneOpt] edgeProcess clone-guard: installed");
	else
		LogMsg("[ZoneOpt] edgeProcess clone-guard: install FAILED");

#if NMFIX_STEP >= 1
	// Pass-through on NavMeshResult__populate, purely to read each generation's
	// input triangle count for the zero-face rule. Installed here rather than in
	// startPlugin so it shares the first-dispatch timing of the guard above.
	// B1: VerifyPrologue
	if (VerifyPrologueByRva(RVA_NM_RESULT_POPULATE)
	    && KenshiLib::SUCCESS == KenshiLib::AddHook(GameAddr(RVA_NM_RESULT_POPULATE),
			(void*)hook_nmResultPopulate_diag, (void**)&orig_nmResultPopulate))
		LogMsg("[ZoneOpt] populate hook: installed (input triangle counts)");
	else
		LogMsg("[ZoneOpt] populate hook: install FAILED (tri counts unavailable)");
#endif

#if NMFIX_STEP >= 5
	// Ownership tripwire on processJobAlt. Its prologue is
	// `mov rax, rsp` / `push rbp` / `push rsi` = exactly 5 relocatable bytes at
	// instruction boundaries, with no RIP-relative operand, so it is hookable.
	// B1: VerifyPrologue
	if (VerifyPrologueByRva(RVA_PROCESS_JOB_ALT)
	    && KenshiLib::SUCCESS == KenshiLib::AddHook(GameAddr(RVA_PROCESS_JOB_ALT),
			(void*)hook_processJobAltTrip, (void**)&orig_processJobAltTrip))
	{
		InterlockedExchange(&nmTripInstalled, 1);
		LogMsg("[ZoneOpt] processJobAlt tripwire: installed");
	}
	else
	{
		LogMsg("[ZoneOpt] processJobAlt tripwire: install FAILED (trip= reads off)");
	}
#endif

#if NMFIX_STEP >= 8
	// Retire the workers before the game tears the NavMesh down. Its prologue
	// is `test rcx, rcx` + a short `jz` into the body, so the 5 bytes contain a
	// relative branch that the trampoline has to relocate rather than copy —
	// the one hook in this file where that is true, and the reason the marker
	// below matters more here than elsewhere. A failed install logs once and
	// leaves the pre-existing DllMain behaviour.
	// B1: VerifyPrologue
	if (VerifyPrologueByRva(RVA_NAVMESH_STOP)
	    && KenshiLib::SUCCESS == KenshiLib::AddHook(GameAddr(RVA_NAVMESH_STOP),
			(void*)hook_navMeshStop, (void**)&orig_navMeshStop))
		LogMsg("[ZoneOpt] NavMesh::stop hook: installed (worker retirement)");
	else
		LogMsg("[ZoneOpt] NavMesh::stop hook: install FAILED (workers not retired early)");
#endif

#if NMFIX_STEP >= 6
	// Collision builders. Both are reached only from dispatchJob_orig, and the
	// hooks also cover the mod's own fn_buildCollision calls.
	// B1: VerifyPrologue
	if (VerifyPrologueByRva(RVA_BUILD_COLLISION_IMPL)
	    && KenshiLib::SUCCESS == KenshiLib::AddHook(GameAddr(RVA_BUILD_COLLISION_IMPL),
			(void*)hook_buildCollision, (void**)&orig_buildCollisionHook))
		LogMsg("[ZoneOpt] buildCollision serializer: installed");
	else
		LogMsg("[ZoneOpt] buildCollision serializer: install FAILED");

	// B1: VerifyPrologue
	if (VerifyPrologueByRva(RVA_BUILD_COLLISION_INTERIOR_IMPL)
	    && KenshiLib::SUCCESS == KenshiLib::AddHook(GameAddr(RVA_BUILD_COLLISION_INTERIOR_IMPL),
			(void*)hook_buildCollisionInterior, (void**)&orig_buildInteriorHook))
		LogMsg("[ZoneOpt] buildCollisionInterior serializer: installed");
	else
		LogMsg("[ZoneOpt] buildCollisionInterior serializer: install FAILED");
#endif

	InterlockedExchange(&lazyHooksInstalled, 2);
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

#if NMFIX_STEP >= 2
// DEV probe state for the material overrides (reported as wbOv= / wbOvSlope=).
volatile long g_wbOverrideInstalled = 0;   // entries appended by the last construct
volatile long g_wbOverrideAfterPop  = -1;  // count read back after processJobAlt
volatile long g_wbOverrideSlopeBad  = 0;   // table slope != the real WB's slope
volatile long g_wbOverrideSkipped   = 0;   // teardown left entries alone (count > capacity)

// OverrideSettings::dtor is at 0xDD92F0, the address hook_edgeProcess patches.
// Call the unhooked body through the trampoline so our teardown neither trips
// the clone-guard nor moves the edge=a/u counters, which count the game's own
// calls (two per MISS) and are a diagnostic in their own right.
static void CallOverrideSettingsDtor(void* entry)
{
	if (orig_edgeProcess)
		orig_edgeProcess(entry);
	else
		((edgeProcess_t)GameAddr(RVA_EDGE_PROCESS))(entry);
}

// Duplicates the real WB's four material-override entries onto a fresh WB.
// `dst` and `src` are work buffers; the fresh array is the guard-slot buffer
// installed just above, empty and with room for OVR_GUARD_CAP entries.
static void AppendMaterialOverrides(char* dst, const char* src)
{
	char* srcArr = *(char**)(src + 520);
	int   srcCnt = *(int*)(src + 528);
	char* dstArr = *(char**)(dst + 520);

	// Report 0 unless the whole run is installed below, so wbOv= describes this
	// construct rather than the last successful one.
	InterlockedExchange(&g_wbOverrideInstalled, 0);

	if (!srcArr || !dstArr || srcCnt < NM_MATERIAL_OVERRIDE_COUNT)
		return;
	if (!fn_simplSettingsCopy)
		return;

	// Validate the whole run before copying any of it, so a layout that has
	// moved leaves the fresh WB with an empty array (the guard slot alone,
	// i.e. the pre-NMFIX-2 behaviour) instead of a half-built one.
	//   +0 must be NULL: OverrideSettings__initFromSettings writes NULL there
	//     and initWorkBuffer never replaces it, which is also what makes these
	//     entries stop finalizeDeep's pop loop.
	//   +8 must be the material id 1..4, in order.
	int i;
	for (i = 0; i < NM_MATERIAL_OVERRIDE_COUNT; ++i)
	{
		const char* se = srcArr + (size_t)OVR_ENTRY_SIZE * i;
		if (*(void* const*)(se + OVR_OFF_VOLUME) != NULL) return;
		if (*(const int*)(se + OVR_OFF_MATERIAL) != i + 1) return;
	}

	for (i = 0; i < NM_MATERIAL_OVERRIDE_COUNT; ++i)
	{
		const char* se = srcArr + (size_t)OVR_ENTRY_SIZE * i;
		char*       de = dstArr + (size_t)OVR_ENTRY_SIZE * i;

		memset(de, 0, OVR_ENTRY_SIZE);
		*(void**)(de + OVR_OFF_VOLUME) = NULL;              // volume-less, nothing to addref
		*(int*)(de + OVR_OFF_MATERIAL) = i + 1;
		*(char*)(de + OVR_OFF_FLAG)    = *(const char*)(se + OVR_OFF_FLAG);
		*(float*)(de + OVR_OFF_SLOPE)  = NmMaterialSlope(i);
		memcpy(de + OVR_OFF_EMP, se + OVR_OFF_EMP, 56);
		fn_simplSettingsCopy(de + OVR_OFF_SIMPL, (void*)(se + OVR_OFF_SIMPL));

		// The table is authoritative, but it is transcribed from the decompile,
		// so check it against what the game actually installed.
		if (*(const float*)(se + OVR_OFF_SLOPE) != NmMaterialSlope(i))
			InterlockedIncrement(&g_wbOverrideSlopeBad);
	}

	*(int*)(dst + 528) = NM_MATERIAL_OVERRIDE_COUNT;
	InterlockedExchange(&g_wbOverrideInstalled, NM_MATERIAL_OVERRIDE_COUNT);
}
#endif // NMFIX_STEP >= 2

static void FreeFreshSettings(void* wb);   // below; also the failure-path teardown

static void* ConstructFreshSettings(uintptr_t origWB)
{
	// Every line below reads the original through `o`. Without this the failure
	// mode is a memcpy from address 16, so refuse and let the caller fall back
	// to the real work buffer ("MISS no-swap") the way an allocation failure
	// already does.
	if (!origWB)
	{
		InterlockedIncrement(&nmCloneConstructFailCount);
		return NULL;
	}

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
			// ("MISS no-swap"). Release what has been built so far through the
			// same teardown a complete fresh WB gets, so the two cannot drift:
			//   - the object the ctor built: at NMFIX 7 FreeFreshSettings runs
			//     the settings dtor body on it (the +144 sub-struct, +336, +512,
			//     the carver and painter arrays), which the old failure path
			//     skipped by freeing the raw block; below NMFIX 7 it skips the
			//     body exactly as it does for a complete fresh WB;
			//   - the +288 material-map copy, when one was made: still ours
			//     (DONT_DEALLOCATE, non-zero capacity), so the body leaves it
			//     and FreeFreshSettings frees it from the pointer it captured
			//     first;
			//   - the 544-byte block.
			// The +520 array is still at the ctor state here (NULL, count 0,
			// DONT_DEALLOCATE with capacity 0), so FreeFreshSettings neither
			// runs an override dtor nor treats it as our guard buffer. Nothing
			// after this point has run yet: no material overrides, no quality
			// write, so there is nothing else to undo.
			FreeFreshSettings(fresh);
			return NULL;
		}
		memset(ovr, 0, OVR_GUARD_BYTES);
		*(void**)(f + 520) = ovr + OVR_ENTRY_SIZE;   // entry[-1] = zeroed guard
		*(int*)(f + 528)   = 0;
		*(int*)(f + 532)   = OVR_GUARD_CAP | HKARRAY_DONT_DEALLOCATE;
	}

#if NMFIX_STEP >= 2
	// The four per-material walkable-slope overrides the real WB carries.
	// NavMeshGenerator__initWorkBuffer (0x3C48D0) pushes them once at NMG
	// construction: a staging entry from OverrideSettings__initFromSettings
	// (0xDD9280), then four copies with the material id at +8 and the slope at
	// +16 varied. Without them every fresh-WB MISS generated materials 1-4 at
	// the base slope from +52 instead of 60/60/90/60, because 0xDD95D0 falls
	// back to +52 when no override matches.
	//
	// The entries are duplicated from the real WB rather than rebuilt from the
	// staging sequence: that reproduces the fields this code does not model
	// (the flag byte, the 56-byte EdgeMatchingParameters block the game patches
	// before pushing, and the SimplificationSettings sub-object) without
	// re-deriving them. Only the material id and the slope are written from
	// NM_MATERIAL_SLOPE_BITS, so the table stays the single authority that the
	// L2 settings hash also reads.
	AppendMaterialOverrides(f, o);
#endif

#if NMFIX_STEP >= 3 && NMCACHE_STEP >= 3
	// The scalar copies above bring the tuned values across only because the
	// real work buffer was tuned before the first dispatch. Write them
	// explicitly so a fresh work buffer carries the mod's generation settings
	// whatever order things happened in, and so the L2 settings hash describes
	// what the worker path actually generated with.
	ApplyNavMeshQualitySettings((uintptr_t)fresh);
	InterlockedExchange(&g_wbQualityLast, *(long*)(f + 328));
#endif

	return fresh;
}

// Release a WB from ConstructFreshSettings. At NMFIX 7 this runs the game's own
// settings dtor body on it (leak Fix 1), which is what releases the carvers,
// the painters, +160/+176, +336 and +512; below that gate those stay orphaned.
// Either way the two DONT_DEALLOCATE buffers are ours to free: the +520 guard
// while +532 still carries the flag with our capacity, and the +288 material-map
// copy while +300 does — SortedArray__grow (0xBA3AA0) replaces the buffer and
// rewrites the capacity if a job ever outgrew it (never observed).
static void FreeFreshSettings(void* wb)
{
	if (!wb) return;
	char* f = (char*)wb;

#if NMFIX_STEP >= 2
	// Destroy whatever entries are still in the override array, back to front,
	// exactly as the settings dtor body (0xDD9BC0) does:
	//     v2 = count - 1; v3 = data + 240*v2;
	//     do { OverrideSettings__dtor(v3); v3 -= 240; --v2; } while (v2 >= 0);
	// Each entry owns a SimplificationSettings whose hkStringPtr may have
	// allocated during the copy, so a bare free would leak it. The count is
	// then zeroed, so the dtor body called below at NMFIX 7 finds count 0, takes
	// `count - 1 < 0` and destroys nothing: no double-destroy. Keeping the loop
	// here rather than letting the body do it is deliberate — the body reaches
	// OverrideSettings::dtor at its real address, which hook_edgeProcess
	// patches, so four extra calls per MISS would land in the edge=a/u counters
	// that A7 reads as exactly 2 per MISS.
	{
		char* entries = *(char**)(f + 520);
		int count = *(int*)(f + 528);
		if (entries && count > 0 && count <= OVR_GUARD_CAP)
		{
			for (int i = count - 1; i >= 0; --i)
				CallOverrideSettingsDtor(entries + (size_t)OVR_ENTRY_SIZE * i);
			*(int*)(f + 528) = 0;
		}
		else if (entries && count > OVR_GUARD_CAP)
		{
			// SortedArray__grow replaced the buffer, so it is no longer ours and
			// the entry count is not one this code put there. Leaving it alone
			// leaks those entries, which is the safe direction, but it should
			// never happen: the capacity is 8 and a job appends one.
			InterlockedIncrement(&g_wbOverrideSkipped);
		}
	}
#endif

	// Capture our two DONT_DEALLOCATE buffers BEFORE anything else: the dtor
	// body NULLs both fields (+520/+532 and +288/+300), so reading them
	// afterwards would lose the pointers.
	//   +520 guard: ours while +532 still carries DONT_DEALLOCATE with our
	//     capacity. SortedArray__grow would have replaced the buffer and
	//     rewritten the capacity if the game ever outgrew it.
	//   +288 material map: ours while +300 carries DONT_DEALLOCATE with a
	//     non-zero capacity.
	char* ovr = *(char**)(f + 520);
	int capFlags = *(int*)(f + 532);
	bool ovrIsOurs = (ovr != NULL) && (capFlags & HKARRAY_DONT_DEALLOCATE)
	              && ((capFlags & 0x3FFFFFFF) == OVR_GUARD_CAP);

	void* mm = *(void**)(f + 288);
	int mmCapFlags = *(int*)(f + 300);
	bool mmIsOurs = (mm != NULL) && (mmCapFlags & HKARRAY_DONT_DEALLOCATE)
	             && ((mmCapFlags & 0x3FFFFFFF) > 0);

#if NMFIX_STEP >= 7
	// Leak Fix 1 (H2). The dtor BODY, never the deleting dtor 0xDDABE0: this
	// object came from HavokTlsAlloc and is freed below, not through the class
	// allocator.
	//
	// It releases the +240 carvers and +256 painters element by element, frees
	// their buffers, destroys +336 and +512, and runs the +144 sub-struct
	// destructor which frees +160 and +176 — every one of those was orphaned
	// before, because finalizeDeep empties the arrays but keeps the buffers.
	// It skips our two buffers, because it frees +288 and +520 only when their
	// capacity word is non-negative, i.e. DONT_DEALLOCATE clear.
	//
	// Safe on a fresh buffer: the regions ConstructFreshSettings copies from
	// the real one are scalars the dtor never reads, and the sub-structs whose
	// shallow copy crashed at Stage 4b are still at ctor defaults. Runs on the
	// thread that ran processJobAlt, after it returned, so the buffers go back
	// to the allocator they came from.
	if (fn_settingsDtorBody)
		fn_settingsDtorBody(wb);
#endif

	size_t freedBytes = 0;

	if (ovrIsOurs)
	{
		HavokTlsFree(ovr - OVR_ENTRY_SIZE, OVR_GUARD_BYTES);
		freedBytes += OVR_GUARD_BYTES;
	}

	if (mmIsOurs)
	{
		size_t mmBytes = (size_t)(mmCapFlags & 0x3FFFFFFF) * 8;
		HavokTlsFree(mm, mmBytes);
		freedBytes += mmBytes;
	}

	HavokTlsFree(wb, WB_OBJECT_SIZE);
	freedBytes += WB_OBJECT_SIZE;

#if NMFIX_STEP >= 7 && defined(ZONEOPT_DEBUG)
	InterlockedExchangeAdd64(&nmWbFreedBytes, (LONGLONG)freedBytes);
#else
	(void)freedBytes;
#endif
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

	char* c = (char*)clone;
	void* freshWB = NULL;

	// NMFIX 3: every read of the real NMG and its work buffer happens under
	// processJobCS. Two races close here.
	//   - The bg thread's own MISS swaps realNMG+256 to a temporary fresh work
	//     buffer for the duration of processJobAlt, installing and restoring it
	//     inside this lock. Reading the pointer outside could snapshot the
	//     temporary instead of the real one (ZO-05).
	//   - The real work buffer's +520 override array sits at count 4 with
	//     capacity 4, so the game's next append reallocates it. Copying those
	//     entries outside the lock could read a freed buffer.
	// The lock is not held across the clone-local work below (self-reference
	// rebasing, the five lock inits), only across the reads: the guard's scope
	// is exactly this block.
	{
#if NMFIX_STEP >= 3
		// pjWait clone=: QPC either side of the guard's construction, so the
		// lock is taken exactly where it was and the owner-tid bookkeeping in
		// EnterProcessJobCS is untouched.
		LONGLONG waitStart = QpcNow();
		ProcessJobLock lock;
		NotePjWait(PJWAIT_CLONE, waitStart, QpcNow());
#endif
		memcpy(clone, realNMG, NMG_STRUCT_SIZE);
		uintptr_t realWB = *(uintptr_t*)((uintptr_t)realNMG + 256);
		freshWB = ConstructFreshSettings(realWB);
	}

	if (!freshWB)
	{
		HavokTlsFree(clone, NMG_STRUCT_SIZE);
		InterlockedIncrement(&nmCloneConstructFailCount);
		return NULL;
	}

	// Clone-local from here: nothing below reads the real NMG.
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
	*(uintptr_t*)(c + 256) = (uintptr_t)freshWB;

	InterlockedIncrement(&nmCloneConstructCount);
	return clone;
}

#if NMFIX_STEP >= 7
// The five boost::shared_mutex members of a NavMeshGenerator. CloneNMG
// re-creates all five with fn_queueLockInit, which is boost_shared_mutex__ctor
// (0x25F350): it writes three semaphore handles per lock, at +8 (an anonymous
// semaphore) and +16 / +24 (CreateSemaphoreA). The game closes exactly these
// fifteen — NavMeshGenerator__dtor (0x3C8A60) closes the nine belonging to
// +152, +200 and +272, then tail-calls ThreadClass__dtor (0x25F9C0) for the six
// belonging to +72 and +104 — and nothing ever destroys a clone, so every
// worker MISS leaked fifteen handles.
static const int NMG_LOCK_OFFSETS[5] = { 72, 104, 152, 200, 272 };
static const int NMG_LOCK_HANDLE_OFFSETS[3] = { 8, 16, 24 };

// Closes the clone's fifteen. The clone is a memcpy of the real generator, so a
// handle that is still identical to the real one was never re-created and must
// not be closed: closing it would destroy a handle the game is still using.
// That cannot happen on the path CloneNMG takes today, and the comparison is
// fifteen loads, so it stays in as a guard rather than an assumption.
static void CloseClonedNMGLocks(void* clone)
{
	uintptr_t real = g_navMeshGen;
	for (int i = 0; i < 5; ++i)
	{
		for (int h = 0; h < 3; ++h)
		{
			int off = NMG_LOCK_OFFSETS[i] + NMG_LOCK_HANDLE_OFFSETS[h];
			HANDLE ch = *(HANDLE*)((char*)clone + off);
			if (!ch) continue;
			if (real && ch == *(HANDLE*)(real + off))
			{
				InterlockedIncrement(&nmCloneHandleSkipped);
				continue;
			}
			CloseHandle(ch);
			*(HANDLE*)((char*)clone + off) = NULL;
			InterlockedIncrement(&nmCloneHandleClosed);
		}
	}
}
#endif // NMFIX_STEP >= 7

static void FreeClonedNMG(void* clone)
{
	if (!clone) return;
#if NMFIX_STEP >= 7
	// Leak Fix 3. Safe here: processJobAlt has returned, the clone's locks are
	// idle and nothing else holds the clone.
	CloseClonedNMGLocks(clone);
#endif
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
#if NMFIX_STEP >= 8
// Signals "the queue may be non-empty". Every observer of a non-empty queue
// sets it; a worker clears it when the queue holds nothing it can take. See the
// wake protocol note above the worker loop.
static inline void SignalJobAvailable()
{
	if (g_jobEvent)
		SetEvent(g_jobEvent);
}

// Clears the wake event, unless we are shutting down: DllMain signals the event
// once to release every worker, and a worker clearing it there would leave the
// others to wait out the 500 ms timeout before noticing.
static inline void ClearJobAvailable()
{
	if (g_jobEvent && !g_workerShutdown)
		ResetEvent(g_jobEvent);
}

// The generator's "busy" bridge. NavMeshGenerator::isBusy (0x3BF360) inspects
// only +232 (the current work item) and +136 (the input queue), so a job this
// mod has unlinked but not yet finished is invisible to it: the generator looks
// idle for a zone that is mid-regeneration, and isContentPending's
// generator-idle fallback can answer "ready" for a zone with no mesh. +265 is
// the byte the mod keeps for that, so it has to be raised at claim time and
// held until the job is completely done — across the L2 read, which is 31-46 ms
// on its own.
// The generator this thread raised the bridge on, so the release writes the
// same byte the claim did rather than re-reading a global that a future change
// could move underneath it.
static __declspec(thread) uintptr_t t_busyNmg = 0;

static inline void WorkerBusyEnter(uintptr_t nmg)
{
	t_busyNmg = nmg;
	InterlockedIncrement(&workerBusyCount);
	*(unsigned char*)(nmg + 265) = 1;
}

static inline void WorkerBusyLeave()
{
	uintptr_t nmg = t_busyNmg;
	t_busyNmg = 0;
	if (InterlockedDecrement(&workerBusyCount) == 0 && nmg)
		*(unsigned char*)(nmg + 265) = 0;
}

// The bg thread's backstop clear of the same bridge, run when it finds nothing
// to dispatch. Read-and-clear happen under the generator's queue lock (+152),
// the lock WorkerBusyEnter holds when it increments workerBusyCount and raises
// the byte. Without it the two are separate: the bg thread reads the count as 0,
// a worker claims a job and raises the bridge, and the bg thread's clear then
// erases a live worker's bridge — the generator looks idle for a zone that is
// mid-regeneration and isContentPending's generator-idle fallback can answer
// "ready" for a zone with no mesh.
//
// Taking the lock is preferred over the other shape, re-reading the count after
// the clear and re-raising when it is non-zero: that leaves a window where the
// re-raise lands after the owning worker has already decremented and cleared,
// pinning the byte at 1 with no worker busy until some later dispatch clears it.
// The lock is taken only when the byte is actually set, so an idle bg poll over
// an empty queue still costs nothing and never contends with a worker dequeue.
//
// lockHeld: the caller already holds +152 (the site that clears after its own
// peek), so the read-and-clear just runs in place.
static void ClearBusyBridgeIfIdle(uintptr_t nmg, bool lockHeld)
{
	if (!lockHeld)
	{
		if (!*(unsigned char*)(nmg + 265))
			return;
		char initBuf[16];
		void* initResult = fn_pathBuilderInit(initBuf);
		fn_pathBuilderFinalize((void*)(nmg + 152), initResult);
	}

	if (InterlockedCompareExchange(&workerBusyCount, 0, 0) == 0)
		*(unsigned char*)(nmg + 265) = 0;

	if (!lockHeld)
		fn_readerUnlock((void*)(nmg + 152));
}

// L2 reads in flight, by key, so two workers handed duplicate jobs for the same
// zone do not read the same file at once. Guarded by nmCacheCS. Small and
// fixed: at most one entry per worker plus the bg thread.
static NavMeshCacheKey g_l2InFlight[NAVMESH_WORKER_COUNT + 1];
static bool            g_l2InFlightUsed[NAVMESH_WORKER_COUNT + 1] = {};

// Caller holds nmCacheCS. Returns the slot taken, or:
//   L2FLIGHT_BUSY  — another thread is already reading this exact key, so the
//                    caller should skip its own read
//   L2FLIGHT_FULL  — no free slot. Not the same thing: nobody is reading this
//                    key, so the caller SHOULD read it, just without
//                    registering. Counted separately rather than reported as a
//                    duplicate, which would overstate dupL2.
static const int L2FLIGHT_BUSY = -1;
static const int L2FLIGHT_FULL = -2;

static int L2InFlightAcquire(const NavMeshCacheKey& key)
{
	int free = -1;
	for (int i = 0; i < NAVMESH_WORKER_COUNT + 1; ++i)
	{
		if (g_l2InFlightUsed[i])
		{
			if (KeysMatch(g_l2InFlight[i], key))
				return L2FLIGHT_BUSY;
		}
		else if (free < 0)
		{
			free = i;
		}
	}
	if (free < 0)
	{
		InterlockedIncrement(&nmL2FlightFull);
		return L2FLIGHT_FULL;
	}
	g_l2InFlight[free] = key;
	g_l2InFlightUsed[free] = true;
	return free;
}

static void L2InFlightRelease(int slot)
{
	if (slot >= 0 && slot < NAVMESH_WORKER_COUNT + 1)
		g_l2InFlightUsed[slot] = false;
}

// Claims one job for this worker, then looks the cache up for it.
//
// Two changes from the pre-NMFIX-8 shape, both in the brief.
//
// Claim before lookup. The old code peeked the head, released the queue lock,
// did the whole L1/L2 lookup, then re-took the lock and bailed if the head had
// moved. Phase-aligned workers therefore all looked up the SAME head, read the
// same L2 file, promoted duplicate L1 entries, and all but one threw the work
// away. Now the job is unlinked first, so each worker owns a distinct job
// before it looks anything up, and there is no bail-out path at all.
//
// Non-destructive scan. The old code gave up when the head was type 2/3/4 or
// had a bad zone, which is why workers starved behind a single stitching job.
// It now walks the list for the first type 0/1 job and unlinks only that node.
//
// The scan MUST leave the head and the order of every other node intact. The bg
// thread's hook_dispatchJob peeks the head, releases the queue lock and hands
// to orig_dispatchJob, which re-pops the head itself; A5's wrap depends on the
// head it peeked still being the head the original pops. This scan preserves
// that in the stronger form: a worker only ever takes the head when the head is
// the job it processes, and otherwise leaves it untouched.
//
// Lock ordering: the queue lock is taken alone here and released before
// nmCacheCS, so it never nests with the cache lock in either direction.
//
// claimQpcOut: QPC taken right after the job is unlinked and the queue lock
// released, before the cache lookup (whose L2 read is part of the exposure the
// claim age measures). Passed by value to WorkerProcessHit / ProcessNavMeshJob.
static uintptr_t WorkerTryDequeueAny(int* hitIdxOut, bool* isMissOut, NavMeshCacheKey* keyOut,
                                     LONGLONG* claimQpcOut)
{
	*hitIdxOut = -1;
	*isMissOut = false;
	*claimQpcOut = 0;

	uintptr_t nmg = g_navMeshGen;
	if (!nmg) return 0;

	// Phase 1: claim a job under the queue lock.
	char initBuf[16];
	void* initResult = fn_pathBuilderInit(initBuf);
	fn_pathBuilderFinalize((void*)(nmg + 152), initResult);

	uintptr_t head = *(uintptr_t*)(nmg + 136);
	if (!head)
	{
		// Empty under the lock: the event is only ever cleared while holding
		// this lock, so a worker cannot miss a job queued after the check.
		ClearJobAvailable();
		fn_readerUnlock((void*)(nmg + 152));
		return 0;
	}

	uintptr_t prev = 0;     // predecessor of `job`, 0 when job is the head
	uintptr_t job  = 0;
	int jobType = -1;

	for (uintptr_t node = head; node; node = *(uintptr_t*)(node + 96))
	{
		int t = *(int*)(node + 88) & 7;
		uintptr_t zone = *(uintptr_t*)node;
		// Types 2/3/4 stay on the bg thread (stitching mutates shared edge
		// data), and a job with no loaded zone is the bg thread's to forward.
		if ((t == 0 || t == 1) && zone && *(uintptr_t*)zone)
		{
			job = node;
			jobType = t;
			break;
		}
		prev = node;
	}

	if (!job)
	{
		// The queue is not empty but holds nothing a worker may take: a run of
		// type 2/3/4 jobs, or jobs whose zone is not loaded. Clear the event
		// anyway. Leaving it set made every worker spin — the manual-reset wait
		// returns immediately, each worker re-takes the +152 lock the bg thread
		// needs, and none of them can make progress. The bg thread re-sets the
		// event after its own dequeue whenever the queue is still non-empty, so
		// the first eligible job wakes everyone; the 500 ms wait is the
		// backstop. The list is not touched.
		ClearJobAvailable();
		fn_readerUnlock((void*)(nmg + 152));
		return 0;
	}

	// Unlink exactly this node. Head and relative order of everything else are
	// unchanged; +144 is the address of the last node's next-pointer, so it only
	// moves when the node being removed was the tail.
	uintptr_t next = *(uintptr_t*)(job + 96);
	if (prev)
		*(uintptr_t*)(prev + 96) = next;
	else
		*(uintptr_t*)(nmg + 136) = next;

	if (!next)
		*(uintptr_t*)(nmg + 144) = prev ? (prev + 96) : (nmg + 136);

	bool queueStillHasWork = (*(uintptr_t*)(nmg + 136) != 0);

	// Busy from the moment the job leaves the queue, not from the moment
	// processing starts: between those two points the job is in neither +136
	// nor +232 and isBusy would report idle. Released once, at the end of the
	// worker loop body, on every exit path.
	WorkerBusyEnter(nmg);

	fn_readerUnlock((void*)(nmg + 152));

	// Claim time: the job left the queue under the lock just released.
	*claimQpcOut = QpcNow();

	// The job is ours now: nothing else can take or free it.
	if (queueStillHasWork)
		SignalJobAvailable();

	uintptr_t zone = *(uintptr_t*)job;
	NavMeshCacheKey key;
	key.gridX = *(int*)(zone + OFF_ZONE_COORDS_X);
	key.gridY = *(int*)(zone + OFF_ZONE_COORDS_Y);
	key.sectionTileId = *(int*)(job + 32);
	key.jobType = jobType;
	key.aabbHash = HashAABB((float*)(job + 48));
	key.buildingHash = ComputeBuildingHash(zone);

	// Phase 2: cache lookup for the claimed job.
	int hitIdx = -1;

	EnterCriticalSection(&nmCacheCS);
	int found = FindCacheEntry(key);
	if (found >= 0 && nmCache[found].cachedFaces != NULL && fn_navMeshCtor != NULL)
		hitIdx = found;
	LeaveCriticalSection(&nmCacheCS);

#if NMCACHE_STEP >= 2
	if (hitIdx < 0 && fn_navMeshCtor != NULL)
	{
		// Duplicate jobs for one zone do exist, so two workers can hold
		// different jobs with the same key. Only one of them reads the file.
		EnterCriticalSection(&nmCacheCS);
		int flight = L2InFlightAcquire(key);
		LeaveCriticalSection(&nmCacheCS);

		if (flight == L2FLIGHT_BUSY)
		{
			// Another worker is reading this exact key. Skip the read and take
			// the MISS path, where the late-HIT re-check under processJobCS
			// picks up its result rather than generating again.
			InterlockedIncrement(&nmDupL2Avoided);
		}
		else
		{
			NavMeshCacheEntry diskEntry;
			memset(&diskEntry, 0, sizeof(diskEntry));
			LARGE_INTEGER tR0, tR1;
			QueryPerformanceCounter(&tR0);
			bool l2Hit = ReadDiskCache(key, diskEntry);
			QueryPerformanceCounter(&tR1);
			long readUs = (long)(QPCToMs(tR0, tR1) * 1000.0);
			InterlockedExchangeAdd(&nmDiskReadUsTimes1, readUs);

			EnterCriticalSection(&nmCacheCS);
			if (l2Hit)
				hitIdx = PromoteDiskEntryToL1(diskEntry);
			if (flight >= 0)
				L2InFlightRelease(flight);
			LeaveCriticalSection(&nmCacheCS);

			if (l2Hit && hitIdx >= 0)
				InterlockedIncrement(&nmDiskHitCount);
			else
				InterlockedIncrement(&nmDiskMissCount);
		}
	}
#endif

	InterlockedIncrement(&nmJobCount);
	*hitIdxOut = hitIdx;
	*isMissOut = (hitIdx < 0);
	*keyOut = key;
	return job;
}
#else
// claimQpcOut: QPC right after the pass-3 unlink (this shape claims after the
// lookup, so the claim age starts later than at NMFIX 8).
static uintptr_t WorkerTryDequeueAny(int* hitIdxOut, bool* isMissOut, LONGLONG* claimQpcOut)
{
	*hitIdxOut = -1;
	*isMissOut = false;
	*claimQpcOut = 0;

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
			hitIdx = PromoteDiskEntryToL1(diskEntry);
			LeaveCriticalSection(&nmCacheCS);
			if (hitIdx >= 0)
				InterlockedIncrement(&nmDiskHitCount);
			else
				InterlockedIncrement(&nmDiskMissCount);
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

	*claimQpcOut = QpcNow();

	InterlockedIncrement(&nmJobCount);
	*hitIdxOut = hitIdx;
	*isMissOut = (hitIdx < 0);
	return peekedJob;
}


#endif // NMFIX_STEP >= 8


// --------------------------------------------------------------------
// Worker HIT processing
// --------------------------------------------------------------------
//
// WorkerTryDequeueAny already looked up the cache entry. Here we just
// reconstruct the cached hkaiNavMesh and run buildCollision + job finalize.
// No processJobAlt. If the cache entry was evicted between dequeue and
// reconstruct (rare, only under cache pressure), fall through to the
// null-result finalize path.
//
// Zone re-check first (Round 2 session 1 crash): time has passed since the claim
// (the L1 lookup and the L2 read in WorkerTryDequeueAny, 31-46 ms per read), and
// a zone the game unloaded in between is dropped the way dispatchJob_orig drops
// it: not freed, not enqueued. The window left after the check is the
// reconstruct plus the buildCollisionCS wait inside the builder hook; it is
// measured (claimAge hit=) rather than checked. Not to be confused with
// hitStale=, which counts a stale cache slot, not a stale zone.
#if NMFIX_STEP >= 8
// Returns false when the slot could not be used, leaving the job untouched for
// the caller to regenerate. Never finalizes or deletes the job in that case:
// before NMFIX 8 an evicted or reused slot dropped straight to the null-result
// path, which deleted the job and left the zone with no mesh at all (B8/ZO-04).
// Returns true when the job is finished with: served, or dropped because its
// zone was unloaded (the caller must not regenerate a dropped job). Either way
// the caller's single WorkerBusyLeave releases the bridge.
static bool WorkerProcessHit(void* nmg, uintptr_t job, int jobType, int hitIdx,
                             const NavMeshCacheKey& key, LONGLONG claimQpc)
#else
static void WorkerProcessHit(void* nmg, uintptr_t job, int jobType, int hitIdx,
                             LONGLONG claimQpc)
#endif
{
	// Before the NMFIX<8 busy raise below, so the drop has nothing to release.
	{
		NoteClaimAge(false, claimQpc, QpcNow());
		int staleReason = STALE_REASON_NONE;
		if (!JobZoneStillLoaded(job, &staleReason))
		{
			NoteStaleDrop(STALE_SITE_HIT, job, jobType, staleReason, claimQpc);
#if NMFIX_STEP >= 8
			return true;
#else
			return;
#endif
		}
	}

#if NMFIX_STEP < 8
	InterlockedIncrement(&workerBusyCount);
	*(unsigned char*)((uintptr_t)nmg + 265) = 1;
#endif

	void* freshNavMesh = NULL;
	EnterCriticalSection(&nmCacheCS);
#if NMFIX_STEP >= 8
	// The ring buffer can wrap between the lookup in WorkerTryDequeueAny and
	// this reconstruct, so the index alone means nothing. Re-compare the key
	// under the lock: identity, not just validity.
	bool slotOk = (hitIdx >= 0 && hitIdx < NM_CACHE_SIZE)
	           && nmCache[hitIdx].valid
	           && nmCache[hitIdx].cachedFaces != NULL
	           && KeysMatch(nmCache[hitIdx].key, key);
	if (slotOk)
#else
	if (nmCache[hitIdx].valid && nmCache[hitIdx].cachedFaces != NULL)
#endif
	{
		LARGE_INTEGER t0, t1;
		QueryPerformanceCounter(&t0);
		freshNavMesh = ReconstructNavMesh(nmCache[hitIdx]);
		QueryPerformanceCounter(&t1);
		long ms10 = (long)(QPCToMs(t0, t1) * 10.0);
		InterlockedExchangeAdd(&nmSavedMsTimes10, ms10);
	}
	LeaveCriticalSection(&nmCacheCS);

#if NMFIX_STEP >= 8
	if (!freshNavMesh)
	{
		// Stale slot, or the reconstruct failed. Hand the job back whole; the
		// worker loop keeps the busy bridge raised and releases it once.
		InterlockedIncrement(&nmHitStaleCount);
		return false;
	}
#endif

	if (freshNavMesh)
	{
		*(void**)(job + 72) = freshNavMesh;
		*(void**)(job + 80) = NULL;
		InterlockedIncrement(&nmCacheHitCount);
		fn_buildCollision(nmg, (void*)job, 0, 0.0);
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

#if NMFIX_STEP < 8
	if (InterlockedDecrement(&workerBusyCount) == 0)
		*(unsigned char*)((uintptr_t)nmg + 265) = 0;
#endif

#if NMFIX_STEP >= 8
	return true;
#endif
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
	// H11: threadInit (0xBCA8D0) is
	//   void* __fastcall(void* memSystem, void** router, const char* name, char flags)
	// Four arguments, the last a char. The old 5-argument typedef passed a
	// trailing -2 that landed in shadow space and was ignored, so the call
	// happened to work, but the declaration was wrong.
	typedef void* (*havokRegister_t)(void*, void*, const char*, char);
	havokRegister_t fn_reg = (havokRegister_t)(*(uintptr_t*)(mgrVtable + 24));

	char name[32];
	sprintf_s(name, sizeof(name), "ZoneOpt_W%d", workerId);
	fn_reg(mgr, ctx128, name, 3);

	char buf8[8];
	memset(buf8, 0, sizeof(buf8));
	fn_havokPostRegInit(buf8, ctx128);
	_mm_setcsr(_mm_getcsr() | 0x8000);

	// Did the registration actually take?
	//
	// Neither the return value nor the TLS slot answers that. threadInit
	// (0xBCA8D0) returns memSystem+40 on every path, and postRegInit (0xBAECF0)
	// sets the TLS slot unconditionally just above, so both are non-NULL even
	// when registration failed.
	//
	// The real signal is the thread-table slot index. threadInit scans for a
	// free slot and gives up at 64 (`if (v9 >= 64) goto LABEL_7`), reports "Too
	// many threads", and then falls through and writes that index to router[14]
	// anyway (`router[14] = v11`). So router[14] >= 64 is exactly the failure
	// case, and it is the only unambiguous one: router[11] is written on the
	// same path whether or not a slot was found, so it says only that the
	// flags & 1 branch ran, which it always does for us.
	//
	// Reading router[14] is only meaningful because flags = 3 guarantees that
	// branch writes it. contextInit (0xBA4770) zeroes qword indices 10..13 and
	// dword 30, but NOT index 14, and the memset above leaves it 0 — which
	// would read as a valid slot index. With a flags value that omitted bit 0
	// this check would silently always pass.
	{
		uintptr_t slotIndex = *(uintptr_t*)((char*)ctx128 + 14 * sizeof(void*));
		if (slotIndex >= 64)
		{
			std::ostringstream ss;
			ss << "[ZoneOpt] Worker " << workerId
			   << ": Havok registration failed (thread table full, slot="
			   << (unsigned long long)slotIndex << "), exiting";
			LogMsg(ss.str());

			// Both cleanups are safe here, and the first is necessary.
			// HavokThread__cleanup (0xBAED40) only clears the two TLS slots;
			// postRegInit set one of them to point at ctx128, which dies with
			// this frame, so it has to be cleared. HavokThread__contextCleanup
			// (0xBA9580) only rewrites the context's vtable pointer. Neither
			// touches the thread table or reads router[14], so the
			// out-of-range index cannot propagate through them.
			fn_havokCleanup(buf8);
			fn_havokCtxCleanup(ctx128);

			// Flag first, then clear the handle: CreateThread may not have
			// stored it yet, and the flag is what the loops actually test.
			InterlockedExchange(&g_workerInitFailed[workerId], 1);
			g_workerHandles[workerId] = NULL;

			return 1;   // never counted live, so nothing to decrement
		}
	}

	InterlockedIncrement(&g_navMeshWorkersLive);

	{
		std::ostringstream ss;
		ss << "[ZoneOpt] Worker " << workerId << " started, Havok TLS initialized";
		LogMsg(ss.str());
	}

	while (!g_workerShutdown)
	{
		// Wake protocol (NMFIX 8): g_jobEvent is manual-reset and means "the
		// queue may be non-empty", not "one job is waiting". Anything that
		// observes NMG+136 non-empty sets it — the bg thread after its own
		// dequeue, and a worker that leaves work behind after claiming. Only a
		// worker holding the queue lock clears it, at either of two points: the
		// queue is empty, or it holds nothing a worker may take (a run of type
		// 2/3/4 jobs, or jobs whose zone is not loaded). The second case matters
		// as much as the first — leaving the event set there makes every worker
		// spin on an immediately-returning wait, each re-taking the +152 lock
		// the bg thread needs.
		//
		// No wakeup is lost, because the clear only ever happens under the queue
		// lock: the setter runs after the insert, and the clearer holds the lock
		// the inserter needs. A burst therefore wakes every idle worker instead
		// of exactly one, which is the starvation O3 describes. The 500 ms
		// timeout stays as a backstop and as the shutdown poll.
		WaitForSingleObject(g_jobEvent, 500);
		if (g_workerShutdown) break;

		int hitIdx = -1;
		bool isMiss = false;
		// QPC at the job's unlink, carried by value to WorkerProcessHit and
		// ProcessNavMeshJob for the claim-age measurement.
		LONGLONG claimQpc = 0;
#if NMFIX_STEP >= 8
		NavMeshCacheKey key;
		memset(&key, 0, sizeof(key));
		uintptr_t job = WorkerTryDequeueAny(&hitIdx, &isMiss, &key, &claimQpc);
#else
		uintptr_t job = WorkerTryDequeueAny(&hitIdx, &isMiss, &claimQpc);
#endif
		if (!job) continue;

		int jobType = *(int*)(job + 88) & 7;

#if NMFIX_STEP >= 8
		// A HIT whose slot turned out to be stale falls through to the MISS
		// path and regenerates, rather than deleting the job. A HIT whose zone
		// was unloaded returns true (dropped) and does not.
		if (!isMiss && !WorkerProcessHit((void*)g_navMeshGen, job, jobType, hitIdx, key, claimQpc))
			isMiss = true;

		if (isMiss)
#else
		if (isMiss)
#endif
		{
			// Early zone re-check, before paying for a clone: a job already
			// stale here is dropped the way dispatchJob_orig drops it (not
			// freed, not enqueued). The re-check after missLock inside
			// ProcessNavMeshJob stays the authoritative one: CloneNMG and the
			// missLock both wait on processJobCS. Nothing to release here: at
			// NMFIX 8 the loop's single WorkerBusyLeave below releases the
			// bridge, and below NMFIX 8 no bridge was raised for a MISS yet.
			int earlyReason = STALE_REASON_NONE;
			if (!JobZoneStillLoaded(job, &earlyReason))
			{
				NoteStaleDrop(STALE_SITE_EARLY, job, jobType, earlyReason, claimQpc);
			}
			else
			{
				// Worker MISS: clone the NMG so processJobAlt operates on our own
				// workBuffer + queue state. On clone-alloc failure, fall back to
				// running on realNMG under processJobCS — same path the bg thread
				// uses.
				void* clonedNMG = CloneNMG((void*)g_navMeshGen);
				if (!clonedNMG)
				{
					LogMsg("[ZoneOpt] Worker: CloneNMG failed, fallback to realNMG");
					ProcessNavMeshJob((void*)g_navMeshGen, (void*)g_navMeshGen, job, jobType,
					                  WORKER_BUSY_HELD, claimQpc);
				}
				else
				{
					// ProcessNavMeshJob returns normally on its stale-drop exit
					// too, so the clone is freed on every path.
					ProcessNavMeshJob((void*)g_navMeshGen, clonedNMG, job, jobType,
					                  WORKER_BUSY_HELD, claimQpc);
					FreeClonedNMG(clonedNMG);
				}
			}
		}
#if NMFIX_STEP < 8
		else
		{
			WorkerProcessHit((void*)g_navMeshGen, job, jobType, hitIdx, claimQpc);
		}
#endif

#if NMFIX_STEP >= 8
		// One release per claimed job, on every exit path above: HIT, stale
		// HIT that regenerated, MISS, the clone-failure fallback, and the three
		// zone-unloaded drops (HIT, early, after missLock).
		WorkerBusyLeave();
#endif
	}

	fn_havokCleanup(buf8);
	fn_havokCtxCleanup(ctx128);

	InterlockedDecrement(&g_navMeshWorkersLive);

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

	// The live count, not the capacity: NAVMESH_WORKER_COUNT only sizes the
	// arrays and bounds the INI value.
	int want = g_navMeshWorkerCount;
	if (want < 1) want = 1;
	if (want > NAVMESH_WORKER_COUNT) want = NAVMESH_WORKER_COUNT;

	int created = 0;
	for (int i = 0; i < want; ++i)
	{
		g_workerHandles[i] = CreateThread(NULL, 0, NavMeshWorkerProc,
		                                  (LPVOID)(uintptr_t)i, 0, NULL);
		if (g_workerHandles[i]) created++;
	}
	std::ostringstream ws;
	ws << "[ZoneOpt] NavMesh workers: " << created << "/" << want
	   << " created (capacity " << NAVMESH_WORKER_COUNT
	   << ", lazy, from first dispatchJob)";
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
//
// claimQpc: QPC at the job's unlink from the queue (the worker's
// WorkerTryDequeueAny or the bg thread's pop in hook_dispatchJob), 0 if
// unknown. Used only for the claim-age measurement and the stale-drop record.
//
// Zone unloaded while claimed (Round 2 session 1 crash): the MISS path re-checks
// the job's zone right after acquiring processJobCS and, when the game has
// unloaded it, returns without generating, leaving the job node as
// dispatchJob_orig's early return leaves it. The bg-thread caller then returns 1,
// the same value vanilla returns for a dropped job.
//
// Exits, and what each releases:
//   1. HIT in this function's own L1/L2 lookup (bg thread, or a worker MISS
//      whose key hit here after all): no processJobCS. Tail: result enqueued
//      (or job deleted on a null/zero-face result), bridge released when
//      !busyHeld, no L2 write. A failed reconstruct takes the MISS path.
//   2. MISS, late HIT: missLock released at the top of the branch, rungs 0-2
//      free their pre-lock fresh WB; buildCollision; then the tail as in 1.
//   3. MISS, generate: fresh WB installed at realNMG+256 and restored before the
//      release, L1 store under the lock, missLock released, buildCollision,
//      fresh WB freed; then the tail, and the L2 write last.
//   4. MISS, zone unloaded: missLock released right after it was acquired,
//      rungs 0-2 free their pre-lock fresh WB (never installed), bridge released
//      when !busyHeld, early return. Job untouched; nothing enqueued, deleted or
//      written. A worker caller frees its clone after the return as always.

void ProcessNavMeshJob(void* realNMG, void* workNMG, uintptr_t job, int jobType,
                       bool busyHeld, LONGLONG claimQpc)
{
	uintptr_t nmg = g_navMeshGen;

	// Worker vs bg thread for the pjWait / stale sites. By thread rather than
	// by workNMG, so a worker whose CloneNMG failed (workNMG == realNMG) still
	// counts as a worker.
	bool onBgThread = (GetCurrentThreadId() == (DWORD)g_navMeshBgThreadId);

	// bridge: game's isContentPending reads this byte. Skipped when the caller
	// already raised it at claim time (NMFIX 8's worker path).
	if (!busyHeld)
	{
		InterlockedIncrement(&workerBusyCount);
		*(unsigned char*)(nmg + 265) = 1;
	}

	uintptr_t jobZone = *(uintptr_t*)job;
	int gridX = *(int*)(jobZone + OFF_ZONE_COORDS_X);
	int gridY = *(int*)(jobZone + OFF_ZONE_COORDS_Y);
	int tileId = *(int*)(job + 32);

#if NMCACHE_STEP >= 2
	// O13: filled from the L1 deep copy on a MISS, written to disk at the very
	// end of the job — after the game has the mesh.
	L2WriteBlob pendingWrite;
	memset(&pendingWrite, 0, sizeof(pendingWrite));
#endif

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
			bool l2Read = ReadDiskCache(key, diskEntry);
			if (l2Read)
			{
				EnterCriticalSection(&nmCacheCS);
				hitIdx = PromoteDiskEntryToL1(diskEntry);
				LeaveCriticalSection(&nmCacheCS);

				if (hitIdx >= 0)
				{
					isHit = true;
					isL2Hit = true;
					InterlockedIncrement(&nmDiskHitCount);
				}
				else
				{
					l2Read = false;   // promotion refused it: treat as a miss
				}
			}
			if (!l2Read)
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
			fn_buildCollision(realNMG, (void*)job, 0, 0.0);
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

		// The workBuffer fn_processJobAlt will see.
		//   workNMG!=realNMG → CloneNMG already installed freshWB at workNMG+256,
		//     built there under processJobCS.
		//   workNMG==realNMG → build freshWB below, inside the lock, and swap
		//     realNMG+256 for the duration of processJobAlt. Restore before
		//     LeaveCS so buildCollision sees the original.
		void* localFreshWB = NULL;
		uintptr_t localOrigWB = 0;
		bool usingNMGClone = (workNMG != realNMG);

#if NMFIX_STEP < 3
		// Rungs 0-2: the pre-NMFIX-3 placement, before the lock. Kept so the
		// ladder can bisect the lock-scope change; see the NMFIX_STEP >= 3 arm
		// after the late-HIT check below.
		if (!usingNMGClone)
		{
			localOrigWB = *(uintptr_t*)((uintptr_t)realNMG + 256);
			localFreshWB = ConstructFreshSettings(localOrigWB);
			if (localFreshWB)
				InterlockedIncrement(&nmCloneConstructCount);
			else
				InterlockedIncrement(&nmCloneConstructFailCount);
		}
#endif

		// Held from here to one of two explicit Release() calls, both inside
		// this block: at the top of the late-HIT branch, and after the L1 store
		// on the generate branch (the late-HIT invariant needs the store inside
		// the lock). Neither branch calls buildCollision with it held. The
		// guard's destructor only acts if something unwinds out of the block
		// before a Release(). On that path the swap below is not undone — a
		// fresh work buffer would stay at realNMG+256 — but the lock is free,
		// where before it stayed held and hung every NavMesh thread.
		//
		// pjWait wMiss= / bgMiss=: QPC either side of the guard's construction,
		// so the lock is taken exactly where it was and the owner-tid
		// bookkeeping in EnterProcessJobCS is untouched.
		LONGLONG missWaitStart = QpcNow();
		ProcessJobLock missLock;
		LONGLONG missLockAt = QpcNow();
		NotePjWait(onBgThread ? PJWAIT_BGMISS : PJWAIT_WMISS, missWaitStart, missLockAt);

		// Authoritative zone re-check, after the last processJobCS wait and
		// before anything below reads the zone or touches realNMG+256: the
		// late-HIT check, ConstructFreshSettings, the swap, processJobAlt (which
		// reads *(zone+0xB8)+8 — the Round 2 session 1 crash, rva 0x3C1600, a
		// worker that waited here 9.4 s behind another generation while the
		// game unloaded (27,42)). claimAge miss= is measured here, for every
		// job that reaches this point, dropped or not.
		NoteClaimAge(true, claimQpc, missLockAt);
		{
			int staleReason = STALE_REASON_NONE;
			if (!JobZoneStillLoaded(job, &staleReason))
			{
				// The function's one early return (exit 4 in the header list).
				// The job is left exactly as dispatchJob_orig's early return
				// leaves it: not freed, not enqueued, +72/+80 untouched. Nothing was
				// generated, so nothing is counted as a miss (miss=, miss=w/bg,
				// avgMiss are unaffected) and no disk write is pending.
				missLock.Release();
#if NMFIX_STEP < 3
				// Rungs 0-2 built the swap work buffer before the lock; it was
				// never installed at realNMG+256, so it is only freed.
				if (localFreshWB)
					FreeFreshSettings(localFreshWB);
#endif
				NoteStaleDrop(onBgThread ? STALE_SITE_BGMISS : STALE_SITE_WMISS,
				              job, jobType, staleReason, claimQpc);
				InterlockedExchange(&nmDiagStep, 36);   // 36 = dropped, zone unloaded

				// The bridge this function raised (busyHeld false: bg thread,
				// and workers below NMFIX 8), released exactly as the normal
				// tail does. With busyHeld the caller's WorkerBusyLeave does it.
				if (!busyHeld)
				{
					if (InterlockedDecrement(&workerBusyCount) == 0)
						*(unsigned char*)(nmg + 265) = 0;
				}

				InterlockedExchange(&nmDiagStep, 50);
				return;   // the worker caller still frees its clone
			}
		}

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
			missLock.Release();
#if NMFIX_STEP < 3
			// Only rungs 0-2 can have built one before the lock; at NMFIX 3 the
			// build happens after this check, so there is nothing to release.
			if (localFreshWB)
				FreeFreshSettings(localFreshWB);
#endif

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
			fn_buildCollision(realNMG, (void*)job, 0, 0.0);
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

#if NMFIX_STEP >= 3
			// The real work buffer is read only under processJobCS, so the
			// fresh copy is built here rather than before the Enter above.
			// Doing it after the late-HIT check also means a duplicate job no
			// longer allocates a work buffer just to free it again. The worker
			// path built its copy inside CloneNMG, under this same lock.
			if (!usingNMGClone)
			{
				localOrigWB = *(uintptr_t*)((uintptr_t)realNMG + 256);
				localFreshWB = ConstructFreshSettings(localOrigWB);
				if (localFreshWB)
					InterlockedIncrement(&nmCloneConstructCount);
				else
					InterlockedIncrement(&nmCloneConstructFailCount);
			}
#endif
			bool cloneActive = usingNMGClone || (localFreshWB != NULL);

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

			// Clear before the run so the store site cannot read the previous
			// job's counts if this one never reaches populate.
			t_lastInputTriCount  = -1;
			t_lastInputVertCount = -1;

#if NMFIX_STEP >= 7
			// Leak Fix 2: the clone-guard is no longer armed. The two entries it
			// used to skip per MISS are fully built by then, and finalizeDeep
			// stops at the zeroed guard slot, so letting edgeProcess run its
			// normal cleanup brings the leaked volume reference back to
			// vanilla's one. The hook and its counters stay for validation:
			// edge= should read a0/uN with N = 2 per MISS.
			(void)cloneActive;
#else
			if (cloneActive)
				InterlockedExchange(&g_cloneProcessing, 1);
#endif

			fn_processJobAlt(workNMG, (void*)job);

#if NMFIX_STEP < 7
			InterlockedExchange(&g_cloneProcessing, 0);
#endif

#if NMFIX_STEP >= 2 && defined(ZONEOPT_DEBUG)
			// finalizeDeep has run inside processJobAlt and popped the single
			// entry the job appended. What is left should be exactly the four
			// material overrides; anything else means the pop loop walked past
			// them or the append count changed.
			{
				uintptr_t probeWB = localFreshWB
				                  ? (uintptr_t)localFreshWB
				                  : (usingNMGClone ? *(uintptr_t*)((uintptr_t)workNMG + 256) : 0);
				if (probeWB)
					InterlockedExchange(&g_wbOverrideAfterPop, *(int*)(probeWB + 528));
			}
#endif

			InterlockedExchange(&nmDiagStep, 32);

			if (jobType == 1)
			{
#if NMFIX_STEP >= 4
				// partialGeneration (0x3CA290) needs the real generator, not a
				// clone. It reads three `this` fields and every one of them is
				// wrong on a clone:
				//   +184  the processing queue, walked by lookupSection
				//         (0x3C75D0) to find this tile's section. CloneNMG
				//         zeroes it, so the walk always misses and the call
				//         silently drops to the +240 fallback scan.
				//   +240  the section manager, memcpy'd from the real NMG, so
				//         the fallback scan does reach real data — but pinned
				//         and read under the wrong lock.
				//   +272  the boost::shared_mutex that lookupSection takes
				//         around setting the pin bit, and that partialGeneration
				//         takes again at its tail. fn_queueLockInit re-created
				//         the clone's, so it serializes against nothing.
				// This call is inside processJobCS (entered before the
				// duplicate-job check above, left after the L1 store below), so
				// passing realNMG is safe against the bg thread and the other
				// workers.
#if NMFIX_STEP >= 6
				// Third collision-build region: partialGeneration stitches and
				// inserts sections the same way the builders do. Taken while
				// processJobCS is held, which is the declared order.
				{
					BuildCollisionScope guard;
					fn_partialFixup(realNMG, (void*)job);
				}
#else
				fn_partialFixup(realNMG, (void*)job);
#endif
				InterlockedIncrement(&nmPartialRealCount);
#else
				fn_partialFixup(workNMG, (void*)job);
#endif
			}

			// Swap-path: restore realNMG+256 before releasing processJobCS so
			// buildCollision and later code see the original.
			if (localFreshWB)
				*(uintptr_t*)((uintptr_t)realNMG + 256) = localOrigWB;

			// L1 store while still holding processJobCS: a duplicate job waiting
			// on the lock then finds the result (late HIT) instead of generating
			// it again. The cached copy is the pre-buildCollision mesh, which is
			// exactly what the HIT path feeds into buildCollision.
			uintptr_t storedResult = 0;
			int storeIdx = -1;
			if (nmDiagStage >= 2 && !InterlockedCompareExchange(&nmCacheDisabled, 0, 0))
			{
				storedResult = *(uintptr_t*)(job + 72);
				if (storedResult)
				{
					// A generation that came back with no faces: either the tile
					// is genuinely empty or the run aborted (a Havok keycode,
					// out of memory). The populate hook left this job's input
					// triangle count in thread-local storage, which tells the
					// two apart. StoreCacheEntry refuses the mesh at NMFIX 1,
					// and with it the disk blob below, which is only built from
					// a published slot.
					if (hkArrayGetCount(storedResult, NMOFF_FACES) == 0)
					{
						uintptr_t zc = *(uintptr_t*)(jobZone + OFF_ZONE_CONTENT);
						NoteZeroFaceMesh(key, t_lastInputTriCount, t_lastInputVertCount,
						                 zc ? *(int*)(zc + OFF_ZMC_THINGS_COUNT) : -1);
					}

					EnterCriticalSection(&nmCacheCS);
					storeIdx = StoreCacheEntry(key, storedResult);
					LeaveCriticalSection(&nmCacheCS);
				}
			}

			missLock.Release();

#if NMCACHE_STEP >= 2
			// O13: serialize the L1 deep copy now, write the file after the
			// result is enqueued. The bytes come from the cache entry, not from
			// the game's arrays, which buildCollision is about to touch.
			if (storeIdx >= 0)
			{
				EnterCriticalSection(&nmCacheCS);
				if (nmCache[storeIdx].valid && KeysMatch(nmCache[storeIdx].key, key))
					BuildDiskCacheBlob(key, nmCache[storeIdx], &pendingWrite);
				LeaveCriticalSection(&nmCacheCS);
			}
#else
			(void)storeIdx;
#endif

			InterlockedExchange(&nmDiagStep, 33);

			fn_buildCollision(realNMG, (void*)job, 0, 0.0);

			InterlockedExchange(&nmDiagStep, 34);

			QueryPerformanceCounter(&t1);
			long ms10 = (long)(QPCToMs(t0, t1) * 10.0);
			InterlockedExchangeAdd(&nmTotalMsTimes10, ms10);

			// Free the swap-path freshWB. At NMFIX 7 FreeFreshSettings runs the
			// game's settings dtor body on it, so the arrays processJobAlt grew
			// are released here; below that gate they stay orphaned until realWB
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

	if (!busyHeld)
	{
		if (InterlockedDecrement(&workerBusyCount) == 0)
			*(unsigned char*)(nmg + 265) = 0;
	}

#if NMCACHE_STEP >= 2
	// O13: the file write is the last thing this job does. The game already has
	// the mesh, so a slow disk no longer delays the result.
	if (pendingWrite.data)
	{
		LARGE_INTEGER tW0, tW1;
		QueryPerformanceCounter(&tW0);
		WriteDiskCacheBlob(&pendingWrite);
		QueryPerformanceCounter(&tW1);
		long writeUs = (long)(QPCToMs(tW0, tW1) * 1000.0);
		InterlockedExchangeAdd(&nmDiskWriteUsTimes1, writeUs);
	}
#endif

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

// Pass-through over the 49-byte wrapper that calls realGenerate. Its second
// argument is the input geometry processJobAlt built for this job — the only
// place the generator's input size is visible, because the geometry is a stack
// local of processJobAlt and nothing copies it onto the job or the result.
//
// The counts go into thread-local storage: the job's store site runs on the
// same thread, later in the same call, so it can read them back without any
// synchronization. A job that never reaches populate leaves the previous
// values, which is why ProcessNavMeshJob clears them before processJobAlt.
void hook_nmResultPopulate_diag(void* navData, void* localData, void* result, int param)
{
	if (localData)
	{
		t_lastInputTriCount  = *(int*)((uintptr_t)localData + OFF_GEOM_TRIANGLE_COUNT);
		t_lastInputVertCount = *(int*)((uintptr_t)localData + OFF_GEOM_VERTEX_COUNT);
	}
	orig_nmResultPopulate(navData, localData, result, param);
}


// Below NMFIX 5 the original is called exactly as before, unlocked.
#if NMFIX_STEP >= 5
  #define CALL_ORIG_DISPATCH(nmg_, measure_) CallOrigDispatchLocked((nmg_), (measure_))
#else
  #define CALL_ORIG_DISPATCH(nmg_, measure_) orig_dispatchJob(nmg_)
#endif

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
		return CALL_ORIG_DISPATCH(thisNMG, false);

	uintptr_t nmg = (uintptr_t)thisNMG;

	if (!g_navMeshGen)
		g_navMeshGen = nmg;

#if NMCACHE_STEP >= 3
	// The lazy hooks install on the first dispatch, where the Havok world is
	// guaranteed live.
	InstallNavMeshLazyHooks();

	// All four dereference realNMG+256 and ApplyNavMeshQualityTuning writes
	// through it, so they race the bg thread's own swap and any worker's clone
	// snapshot and need processJobCS.
	//
	// They must not take it on every dispatch. The bg thread's HIT path is
	// deliberately lock-free so it keeps serving cached meshes while a worker
	// holds processJobCS for a ~1.5 s MISS; an Enter here would block the bg
	// thread for that whole MISS and close exactly the overlap the worker pool
	// exists for. So the block runs once, behind a latch checked without the
	// lock.
	//
	// Running once is enough. The three probes already latch internally on
	// their own flags, and since NMFIX 3 ConstructFreshSettings writes the
	// quality table onto every fresh work buffer, so re-applying it to the real
	// one on each dispatch was redundant.
	//
	// The latch is set only when the work buffer was actually present. All
	// three probes re-arm themselves when they find realNMG+256 NULL, and
	// ApplyNavMeshQualityTuning simply does nothing; leaving the latch clear in
	// that case gives the next dispatch the retry they expect.
#if NMFIX_STEP >= 3
	if (!InterlockedCompareExchange(&g_firstDispatchDone, 0, 0))
	{
		ProcessJobLock lock;   // this block only
		if (!InterlockedCompareExchange(&g_firstDispatchDone, 0, 0))
		{
			ProbeNavMeshSettings(nmg);
			ApplyNavMeshQualityTuning(nmg);
			VerifyNavMeshSettings(nmg);
			ProbeWorkBufferSize(nmg);

			if (*(uintptr_t*)(nmg + 256))
				InterlockedExchange(&g_firstDispatchDone, 1);
		}
	}
#else
	ProbeNavMeshSettings(nmg);
	ApplyNavMeshQualityTuning(nmg);
	VerifyNavMeshSettings(nmg);
	ProbeWorkBufferSize(nmg);
#endif
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
#if NMFIX_STEP >= 8
		ClearBusyBridgeIfIdle(nmg, false);
#else
		if (InterlockedCompareExchange(&workerBusyCount, 0, 0) == 0)
			*(unsigned char*)(nmg + 265) = 0;
#endif
		return 0;
	}

	char initBuf[16];
	void* initResult = fn_pathBuilderInit(initBuf);
	fn_pathBuilderFinalize((void*)(nmg + 152), initResult);

	uintptr_t job = *(uintptr_t*)(nmg + 136);
	if (!job)
	{
#if NMFIX_STEP >= 8
		// Still under +152 here, so the read-and-clear is atomic against a
		// worker's WorkerBusyEnter without taking the lock a second time.
		ClearBusyBridgeIfIdle(nmg, true);
		fn_readerUnlock((void*)(nmg + 152));
#else
		fn_readerUnlock((void*)(nmg + 152));
		if (InterlockedCompareExchange(&workerBusyCount, 0, 0) == 0)
			*(unsigned char*)(nmg + 265) = 0;
#endif
		return 0;
	}

	uintptr_t peekZone = *(uintptr_t*)job;
	if (!peekZone || !*(uintptr_t*)peekZone)
	{
		fn_readerUnlock((void*)(nmg + 152));
#if NMFIX_STEP >= 8
		// Wake the workers once this forward has consumed the head. A worker
		// that found the queue all-ineligible cleared the event, and nothing
		// else re-sets it on this path, so eligible jobs queued behind the head
		// would sit until the 500 ms backstop expired.
		{
			char r = CALL_ORIG_DISPATCH(thisNMG, false);
			if (*(uintptr_t*)(nmg + 136))
				SignalJobAvailable();
			return r;
		}
#else
		return CALL_ORIG_DISPATCH(thisNMG, false);
#endif
	}

	int jobType = *(int*)(job + 88) & 7;

	if (jobType != 0 && jobType != 1)
	{
		// No consumer can take this head across the unlock: workers only ever
		// pop type 0/1 heads, and appends go to the tail while the head is
		// non-NULL. The main thread's PrioritizeNavMeshQueue can still reorder
		// the queue and put a different job at the head, so the original may
		// re-pop something other than what was peeked here. That is benign: the
		// original processes whatever it pops, under processJobCS, and the only
		// consequence is that one job bypasses our cache. It is not corruption,
		// and it is why the Enter below stays after the queue-lock release
		// rather than being widened to cover the peek.
		fn_readerUnlock((void*)(nmg + 152));
		InterlockedIncrement(&nmJobCount);
		InterlockedIncrement(&nmCacheSkipCount);
#if NMFIX_STEP >= 8
		// Same wake as the bad-zone forward: a stitching job at the head is
		// exactly the case that made workers clear the event and sleep on a
		// queue that still held type 0/1 work behind it.
		{
			char r = CALL_ORIG_DISPATCH(thisNMG, true);
			if (*(uintptr_t*)(nmg + 136))
				SignalJobAvailable();
			return r;
		}
#else
		return CALL_ORIG_DISPATCH(thisNMG, true);
#endif
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
		return CALL_ORIG_DISPATCH(thisNMG, false);

	int jobType = *(int*)(head + 88) & 7;

	if (jobType != 0 && jobType != 1)
	{
		// Single-consumer build: the bg thread is the only one that pops, so the
		// head the original re-pops is the one peeked just above.
		InterlockedIncrement(&nmJobCount);
		InterlockedIncrement(&nmCacheSkipCount);
		return CALL_ORIG_DISPATCH(thisNMG, true);
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

	// Claim time: the bg thread took ownership of `job` in the unlink just
	// above (either arm). Passed by value to ProcessNavMeshJob.
	LONGLONG claimQpc = QpcNow();

	// Vanilla's own claim-time check (dispatchJob_orig 0x3CE030:
	// `if (!**job) return 1;`), unchanged. ProcessNavMeshJob re-checks after
	// its processJobCS wait.
	if (!*(uintptr_t*)(*(uintptr_t*)job))
		return 1;

#if NMCACHE_STEP >= 4
	// Signal workers that a new job is available.
	if (g_jobEvent && *(uintptr_t*)(nmg + 136))
		SetEvent(g_jobEvent);
#endif

	// HITs process concurrently with workers; MISSes serialize via processJobCS.
	// Returns 1 on every path, including ProcessNavMeshJob's drop of a job whose
	// zone was unloaded during the processJobCS wait: the value vanilla returns
	// for a dropped job.
	ProcessNavMeshJob(thisNMG, thisNMG, job, jobType, false, claimQpc);
	return 1;
}


#endif // NMCACHE_STEP >= 1
