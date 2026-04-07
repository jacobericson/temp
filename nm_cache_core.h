// nm_cache_core.h — L1 in-memory navmesh cache + stats reporter (Layer 3)
// Depends on: nm_cache_types.h

#ifndef KENSHI_ZONE_OPT_NM_CACHE_CORE_H
#define KENSHI_ZONE_OPT_NM_CACHE_CORE_H

#include "nm_cache_types.h"


// Critical sections (shared across navmesh cache modules)
extern CRITICAL_SECTION nmCacheCS;
extern CRITICAL_SECTION processJobCS;
extern CRITICAL_SECTION buildCollisionCS;
extern volatile long    nmCacheDisabled;

// Diagnostic counters (written by bg threads via Interlocked, read by main thread)
extern volatile long nmJobCount;
extern volatile long nmCacheHitCount;
extern volatile long nmCacheMissCount;
extern volatile long nmCacheSkipCount;
extern volatile long nmTotalMsTimes10;
extern volatile long nmSavedMsTimes10;
extern volatile long nmDiagLastGridX;
extern volatile long nmDiagLastGridY;
extern volatile long nmDiagLastType;
extern volatile long nmDiagStep;
extern volatile long nmDiagHitGrid;

// Settings probe buffers (written once by bg thread, dumped by stats reporter)
extern volatile long  nmSettingsDumped;
extern volatile long  nmSettingsVerified;
extern volatile float probeEMP[14];
extern volatile float probeGen[24];
extern volatile float probeMisc[8];
extern volatile float verifyEMP[7];

// WorkBuffer probe state
extern volatile long wbProbeDone;
extern volatile long probeWBPtrLo, probeWBPtrHi;
extern volatile long probeHavokPtrLo, probeHavokPtrHi;
extern volatile long probeWBMatch;
extern volatile long probeNMGPtrLo, probeNMGPtrHi;
extern volatile long probeWBMsize, probeWBHeapSize, probeWBFieldScan;

// hkArray scanner results
extern volatile long wbArrayScanDone;
extern int           wbArrayOffsets[WB_MAX_ARRAYS];
extern volatile long wbArrayCount;
extern int           wbWritableOffsets[WB_MAX_ARRAYS];
extern volatile long wbWritableCount;
extern WBArrayProbe  wbArrayProbes[WB_MAX_ARRAYS];
extern int           wbSdkArrayOffsets[WB_MAX_SDK_ARRAYS];
extern int           wbSdkArrayCount;

// L2 disk cache counters
extern volatile long nmDiskHitCount;
extern volatile long nmDiskMissCount;
extern volatile long nmDiskWriteCount;
extern volatile long nmDiskReadUsTimes1;
extern volatile long nmDiskWriteUsTimes1;

// L2 miss log
extern L2MissEntry   l2MissLog[L2_MISS_LOG_MAX];
extern volatile long l2MissLogCount;
extern volatile long l2MissLogReported;

// Worker counters (defined here, written by workers via Interlocked, read by stats reporter)
extern volatile long workerBusyCount;
extern volatile long g_slabAllocHits;
extern volatile long g_slabAllocWorkerHits;

// WorkBuffer clone size (set by quality probe, read by workers/dispatch)
extern volatile long g_workBufAllocSize;

// FLA allocator CS probe state (written by probe, read by stats reporter)
extern volatile long flaHooksInstalled;   // 0=pending, 1=installing, 2=done
extern volatile long csProbeFLAPtrLo;
extern volatile long csProbeFLAPtrHi;

// Disk cache directory (initialized by InitNavMeshCacheCS, used by nm_disk_cache)
extern std::string   nmDiskCacheDir;
extern bool          nmDiskCacheDirChecked;


// L1 ring buffer state (shared with nm_workers for ProcessNavMeshJob access)
extern NavMeshCacheEntry nmCache[NM_CACHE_SIZE];
extern int               nmCacheWriteIdx;
extern int               nmCacheFill;
extern int               nmDiagStage;

// Cache operations
void           InitNavMeshCacheCS();
void           ClearNavMeshCache();
int            FindCacheEntry(const NavMeshCacheKey& key);
void           EvictCacheEntry(int idx);
void           StoreCacheEntry(const NavMeshCacheKey& key, uintptr_t navMeshPtr);
void*          ReconstructNavMesh(const NavMeshCacheEntry& entry);
unsigned int   HashAABB(const float* aabb6);
unsigned int   ComputeBuildingHash(uintptr_t jobZone);
bool           KeysMatch(const NavMeshCacheKey& a, const NavMeshCacheKey& b);

// Stats reporter (called from main thread)
void LogNavMeshCacheStats(double now);


#endif // KENSHI_ZONE_OPT_NM_CACHE_CORE_H
