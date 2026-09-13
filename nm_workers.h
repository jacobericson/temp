// nm_workers.h — NavMesh worker pool, hooks, job pipeline (Layer 3)
// Depends on: nm_cache_core.h, nm_disk_cache.h, nm_quality.h

#ifndef KENSHI_ZONE_OPT_NM_WORKERS_H
#define KENSHI_ZONE_OPT_NM_WORKERS_H

#include "nm_cache_core.h"
#include "nm_disk_cache.h"
#include "nm_quality.h"


// Worker pool state
extern HANDLE          g_workerHandles[NAVMESH_WORKER_COUNT];
extern volatile long   g_workerShutdown;
extern HANDLE          g_jobEvent;
// Set by a worker whose Havok registration failed, before it clears its handle
// slot. The flag, not the handle, is the race-free signal: CreateThread may not
// have stored the handle yet when the worker gives up.
extern volatile long   g_workerInitFailed[NAVMESH_WORKER_COUNT];
// The handle of slot i when it belongs to a worker that registered and may
// still be running, or NULL. Returns the handle rather than a bool so callers
// use one snapshot: a worker whose registration fails clears its own slot, and
// re-reading the array after the test can hand back a NULL that has already
// passed it.
HANDLE WorkerSlotHandle(int i);
#if NMFIX_STEP >= 8
// Signals shutdown, wakes the workers and waits up to 15 s for them to finish.
// Idempotent. Called from the NavMesh::stop hook, which first sets
// g_navMeshStopSeen (core.h).
void RetireNavMeshWorkers();
#endif
extern uintptr_t       g_navMeshGen;
extern int             g_workerSavedPriority[NAVMESH_WORKER_COUNT];

// Hook functions
char hook_dispatchJob(void* thisNMG);
void hook_nmResultPopulate_diag(void* navData, void* localData, void* result, int param);
void hook_realGenerate(void* workBuffer, void* localData, void* hkaiNavMesh, int param, int timeLowPart);

// Worker infrastructure
// busyHeld: the caller already raised the worker-busy bridge for this job and
// will release it (the worker path does, from claim time). The bg thread passes
// false and lets this function do its own accounting.
// claimQpc: QueryPerformanceCounter value taken when the job was unlinked from
// the generator queue (0 = unknown), for the claim-age measurement. The function
// re-checks the job's zone after its processJobCS wait and returns without
// generating (job left as dispatchJob_orig's early return leaves it) when the
// game unloaded it meanwhile.
void ProcessNavMeshJob(void* realNMG, void* workNMG, uintptr_t job, int jobType,
                       bool busyHeld, LONGLONG claimQpc);
DWORD WINAPI NavMeshWorkerProc(LPVOID param);
void CreateNavMeshWorkers();


// --------------------------------------------------------------------
// processJobCS from outside the NavMesh pipeline (Round 2 fix 2b review)
// --------------------------------------------------------------------
//
// The save-load reset hook (preload.cpp) unloads the mod's surviving zones, and
// unloadSingleZone frees zone+0xB8 (terrain collision) and the content. A MISS
// that already passed its post-wait zone re-check is running processJobAlt,
// which reads *(zone+0xB8) (0x3CC99C). Holding processJobCS across the unloads
// lets every MISS in flight finish first; every job claimed meanwhile then
// fails its re-check after the wait. Main thread only.
//
// NM_PJLOCK_NONE    no lock taken: this build has no processJobCS
//                   (NMCACHE_STEP < 1), or startPlugin never initialised it.
// NM_PJLOCK_HELD    taken (TryEnterCriticalSection polled with Sleep(1)); the
//                   owner-tid bookkeeping is the same as every other holder's,
//                   so the processJobAlt tripwire stays exact. Release with
//                   NavMeshUnlockProcessJob.
// NM_PJLOCK_TIMEOUT not taken within timeoutMs; the caller proceeds unlocked.
// *waitedMs (optional) receives the time spent trying, in milliseconds.
enum NavMeshPjLockResult
{
	NM_PJLOCK_NONE = 0,
	NM_PJLOCK_HELD,
	NM_PJLOCK_TIMEOUT
};

#if NMCACHE_STEP >= 1
NavMeshPjLockResult NavMeshTryLockProcessJobFor(DWORD timeoutMs, DWORD* waitedMs);
// Only after NM_PJLOCK_HELD.
void NavMeshUnlockProcessJob();
// Called by startPlugin right after InitNavMeshCacheCS: processJobCS exists
// from here on. Before it, NavMeshTryLockProcessJobFor answers NM_PJLOCK_NONE.
void NavMeshMarkProcessJobLockReady();
#else
inline NavMeshPjLockResult NavMeshTryLockProcessJobFor(DWORD, DWORD* waitedMs)
{
	if (waitedMs) *waitedMs = 0;
	return NM_PJLOCK_NONE;
}
inline void NavMeshUnlockProcessJob() {}
inline void NavMeshMarkProcessJobLockReady() {}
#endif


#endif // KENSHI_ZONE_OPT_NM_WORKERS_H
