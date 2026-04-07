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
extern uintptr_t       g_navMeshGen;
extern int             g_workerSavedPriority[NAVMESH_WORKER_COUNT];

// Hook functions
char hook_dispatchJob(void* thisNMG);
void hook_nmResultPopulate_diag(void* navData, void* localData, void* result, int param, int lowPart);
void hook_realGenerate(void* workBuffer, void* localData, void* hkaiNavMesh, int param, int timeLowPart);

// Worker infrastructure
void ProcessNavMeshJob(void* realNMG, void* workNMG, uintptr_t job, int jobType);
DWORD WINAPI NavMeshWorkerProc(LPVOID param);
void CreateNavMeshWorkers();
void InitScratchTLS();


#endif // KENSHI_ZONE_OPT_NM_WORKERS_H
