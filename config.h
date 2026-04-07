// config.h — Feature flags, tuning parameters, shared structs (Layer 2)
// Depends on: core.h, game.h

#ifndef KENSHI_ZONE_OPT_CONFIG_H
#define KENSHI_ZONE_OPT_CONFIG_H

#include "core.h"
#include "game.h"

// TESTING implies ZONEONLY (everything ZONEONLY excludes, TESTING also excludes)
#ifdef ZONEOPT_TESTING
#ifndef ZONEOPT_ZONEONLY
#define ZONEOPT_ZONEONLY
#endif
#endif

// Incremental navmesh cache enablement. Override via /DNMCACHE_STEP=N.
// Each step implies all previous. 0=none, 1=L1 cache, 2=+L2 disk, 3=+quality,
// 4=+workers (BLOCKED: CRT heap corruption), 5=+populate hook.
// Production default: 3 (step 4+ blocked — see worker_clone_investigation.md).
#ifndef NMCACHE_STEP
  #define NMCACHE_STEP 3
#endif

// Incremental pathfinding feature enablement. Set via /DPATHFIND_STEP=N on cl line.
// Each step implies all previous. Only meaningful in FULL builds (!ZONEOPT_ZONEONLY).
// 0=none (hooks not installed), 1=diagnostics, 2=+priority boost, 3=+bypass, 4=+cache+probe.
// Temporary gate — removed after validation.
#ifndef PATHFIND_STEP
  #define PATHFIND_STEP 0
#endif


// =========================================================================
// Feature flags (runtime, default true)
// =========================================================================

extern bool deferralEnabled;
extern bool priorityBoostEnabled;
extern bool preloadEnabled;
extern bool movementAwareEnabled;
extern bool cachingEnabled;
#ifndef ZONEOPT_ZONEONLY
extern bool groupCohesionEnabled;
#endif
#if PATHFIND_STEP >= 1
extern bool pathfindDiagEnabled;
extern bool squadPathCacheEnabled;
#endif


// =========================================================================
// NavMesh worker pool constants
// =========================================================================

const int NAVMESH_WORKER_COUNT = 3;
const int WORKBUF_SIZE = 65536;  // workBuffer clone size (runtime probe: allocSize=65536)


// =========================================================================
// Compile-time hard limits (array sizing upper bounds)
// =========================================================================

const int MAX_FORMATION_MEMBERS_LIMIT = 64;  // hard cap for embedded FormationMember arrays


// =========================================================================
// Runtime-tunable parameters (cfg_ prefix, loaded from INI)
// =========================================================================

// Zone loading
extern float  cfg_preloadThreshold;      // units from zone center (transition at 4147.2)
extern int    cfg_cameraReserved;        // camera queue slots

// Character tracking
extern float  cfg_edgeThreshold;         // distance from zone edge for mover detection
extern double cfg_charScanInterval;      // seconds between fallback scans
extern double cfg_baselineScanInterval;  // seconds between baseline scans
extern double cfg_activePollInterval;    // seconds between active mover polls

// Formation
extern double cfg_gatherTimeout;         // seconds before skipping gather phase
extern double cfg_formationTimeout;      // seconds before auto-cleanup
extern float  cfg_scatterApproachDistSq; // squared distance for scatter trigger
extern float  cfg_gatherRadiusSq;        // squared radius for gather detection

// NavMesh workers
extern int    cfg_navmeshWorkerCount;    // worker thread count (capped at NAVMESH_WORKER_COUNT)

// Hook orchestration
extern double cfg_camLogInterval;        // debug camera log interval
extern double cfg_reprioritizeInterval;  // seconds between navmesh queue reprioritization
extern double cfg_evictInterval;         // seconds between stale zone eviction checks

// Capacity (set once at startup, treated as immutable)
extern int    cfg_maxPreloaded;
extern int    cfg_maxWatched;
extern int    cfg_maxFormationGroups;
extern int    cfg_maxFormationMembers;   // soft cap, checked in code
extern int    cfg_maxCharZones;
extern int    cfg_maxPendingOrder;


// =========================================================================
// Preloading enqueue order constants
// =========================================================================

const int OWNER_CAMERA    = 0;
const int OWNER_CHARACTER = 1;

// Preload order: center first, then cardinals, then diagonals
const int ORDER_DX[9] = { 0, -1, 1, 0, 0, -1, 1, -1, 1 };
const int ORDER_DY[9] = { 0, 0, 0, -1, 1, -1, -1, 1, 1 };

// 2x2 grid matching the pause mechanism's positive-offset pattern
const int SMALL_DX[4] = { 0, 1, 0, 1 };
const int SMALL_DY[4] = { 0, 0, 1, 1 };


// =========================================================================
// Shared struct definitions (used by multiple modules)
// =========================================================================

// PreloadedZone and QueuedZone structs moved to preload.h

// WatchedCharacter struct moved to tracking.h

// Formation structs (FormationMember, FormationGroup, SquadPathSignalEntry) moved to formation.h

// PathProbeEntry and PATH_PROBE_SIZE moved to pathfind_diag.h


// =========================================================================
// ZONEONLY inline stubs for excluded features
// =========================================================================

#ifdef ZONEOPT_ZONEONLY
// Formation stubs (always disabled in ZONEONLY)
inline bool ApplyScatterPatch() { return false; }
inline void CreateFormationGroup(const float*, uintptr_t*, int) {}
inline void PollFormationGroups() {}
inline void ClearFormationGroups() {}
#endif // ZONEOPT_ZONEONLY

// Pathfinding stubs: disabled features get no-ops (works in both ZONEONLY and FULL)
#if PATHFIND_STEP < 1
inline void LogPathfindDiagStats(double) {}
inline void DumpPathProbe(double) {}
inline void ArmPathProbe() {}
#endif
#if PATHFIND_STEP < 2
inline void LogPhase12Stats(double) {}
#endif
#if PATHFIND_STEP < 4
inline void LogSquadPathCacheStats(double) {}
#endif

// NavMesh cache: stubs when step not enabled, forward declarations otherwise
#if NMCACHE_STEP < 1
inline void ClearNavMeshCache() {}
inline void InitNavMeshCacheCS() {}
inline void LogNavMeshCacheStats(double) {}
#else
void ClearNavMeshCache();
void InitNavMeshCacheCS();
void LogNavMeshCacheStats(double now);
#endif
#if NMCACHE_STEP < 4
inline void InitScratchTLS() {}
#else
void InitScratchTLS();
#endif


// =========================================================================
// Config loading
// =========================================================================

void LoadConfig(const std::string& dllDir);


#endif // KENSHI_ZONE_OPT_CONFIG_H
