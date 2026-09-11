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

// NavMesh cache feature gate. Each step implies all previous. Production ships at 4.
//   0 = no cache
//   1 = L1 in-memory ring buffer (deep-copy + reconstruct)
//   2 = +L2 disk cache (flat binary files, order-independent building hash)
//   3 = +generation quality tuning
//   4 = +worker pool (Stage 4c: workers parallelize L1/L2 HITs, steal MISSes under processJobCS)
// Retained as a compile-time gate so earlier steps can be rebuilt for regression bisection.
#ifndef NMCACHE_STEP
  #define NMCACHE_STEP 4
#endif

// Incremental pathfinding feature enablement. Set via /DPATHFIND_STEP=N on cl line.
// Each step implies all previous. Only meaningful in FULL builds (!ZONEOPT_ZONEONLY).
// 0=none (hooks not installed), 1=diagnostics, 2=+priority boost, 3=+bypass, 4=+cache+probe.
// Temporary gate — removed after validation.
#ifndef PATHFIND_STEP
  #define PATHFIND_STEP 0
#endif

// Island routing staging gate (Phase 15). Set via /DISLAND_STEP=N on the cl line
// (build_opt_step4_dev.bat). Step 0 (readiness-gate side effects on the main
// thread only + per-thread counters) is always compiled.
//   1 = builder + snapshot + marks + emulation + PLAYER STUCK fields; hooks pass through
//   2 = both hooks live (the fix; islandFix INI key can turn it off at runtime)
//   3 = parked-squad re-issue + tier-ordered registration/promotion
// Temporary gate — removed after validation (the fix then ships in all 6 variants).
#ifndef ISLAND_STEP
  #define ISLAND_STEP 0
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
#if PATHFIND_STEP >= 2
extern bool stuckRetryEnabled;
#endif
extern bool islandFixEnabled;            // islandFix: overlay answers the island hooks (ISLAND_STEP >= 2)


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

// Island routing
extern int    cfg_islandModRadius;       // Chebyshev radius (zones) around camera/player chars
                                         // within which marked mod zones stay eligible; 0 = unlimited
#ifdef ZONEOPT_DEBUG
extern double cfg_islandTestPromoteDelay; // DEV: hold promotion N seconds after registration (forces the race)
#endif

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
// Island re-issue helpers (formation groups do not exist in ZONEONLY)
inline int       FormationSlotForCharacter(uintptr_t) { return -1; }
inline uintptr_t FormationFirstAliveMember(int) { return 0; }
inline bool      FormationReissueTravel(int, double) { return false; }
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


// =========================================================================
// Config loading
// =========================================================================

void LoadConfig(const std::string& dllDir);


#endif // KENSHI_ZONE_OPT_CONFIG_H
