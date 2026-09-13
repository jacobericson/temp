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

// Round 1 navmesh MISS-path fixes staging gate (research/navmesh_miss_split.md
// §7.8). Set via /DNMFIX_STEP=N on the cl line. Steps 1..8; 8 is the worker
// dequeue and HIT-identity rewrite (B8/ZO-04 + O3), kept above A8's teardown so
// the two stay separately bisectable.
// Default 8 since Round 1 (2026-09-12): steps 1..8 all shipped, so every variant
// carries them. The ladder still builds any lower rung via /DNMFIX_STEP=N for
// regression bisection.
// Temporary gate — removed after validation.
#ifndef NMFIX_STEP
  #define NMFIX_STEP 8
#endif

// Phase 18 preload pipeline staging gate (research/preload_pipeline.md). Set via
// /DPRELOAD_STEP=N on the cl line. Steps 1..6. Step 1 (per-zone pipeline
// latencies, the transition drop count, the H15 benchmark) shipped in Round 2;
// build_opt_step4.bat defaults this to 1. Steps 2-6 are still unused.
// Temporary gate — removed after validation.
#ifndef PRELOAD_STEP
  #define PRELOAD_STEP 0
#endif

// Phase 17 path worker pool staging gate (research/path_worker_pool.md). Set via
// /DPATHPOOL_STEP=N on the cl line. Steps 1..5. Step 1 (the four pass-through
// hooks, PathQueue/GateRate/PathSlow/AstarCost/NpcPathWait) shipped in Round 2;
// build_opt_step4.bat defaults this to 1. Steps 2-5 are still unused.
// Temporary gate — removed after validation.
#ifndef PATHPOOL_STEP
  #define PATHPOOL_STEP 0
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
// H5: the squad path cache injection (Report 4 CTD, research/squad_path_cache_crash.md)
// is compiled out of every shipped variant. Only a build that defines
// ZONEOPT_SQUAD_CACHE (Phase 16 staging) gets the mutable flag; every other
// build sees a compile-time false, so every assignment site outside that
// macro fails to compile and every read folds to dead code.
#ifdef ZONEOPT_SQUAD_CACHE
extern bool squadPathCacheEnabled;
#else
static const bool squadPathCacheEnabled = false;
#endif
#endif
#if PATHFIND_STEP >= 2
extern bool stuckRetryEnabled;
#endif
extern bool islandFixEnabled;            // islandFix: overlay answers the island hooks (ISLAND_STEP >= 2)

// destroyListDiag: install the pass-through hook on GameWorld::destroyListOE's
// sole inserter (crash-3 diagnostic, core.h). It only records the calling
// thread, but it is still a 5-byte patch into a hot engine function, so PROD
// leaves it off and DEV turns it on. The invariant probe itself is always on in
// every build and is not gated by this key.
extern bool destroyListDiagEnabled;

// destroyListDefer: the crash-3 mitigation (core.h). Off-main-thread inserts
// into GameWorld::destroyListOE are queued and replayed on the main thread, so
// the unsynchronised container has a single writer. On by default in every
// build; setting it false leaves the hook installed as the plain diagnostic,
// which is the A/B control. The hook is installed when either this or
// destroyListDiag is on.
extern bool destroyListDeferEnabled;

// saveLoadUnload: the Round 2 save-load crash fix (preload.cpp,
// hook_resetUnloadZones). At the game's save-load reset, after it unloads the
// zones in Set A and Set B, unload every zone still holding a content (the
// mod's, which are in neither set) and clear the mod's state there. On by
// default in every build, PROD included. false = today's behaviour for
// bisection: nothing unloaded and the state clear left to the ZM+8 edge; the
// survivors are still counted and logged ("would unload").
extern bool saveLoadUnloadEnabled;

// islandReadinessRule: per-caller readiness rule (Phase 15 item (d), H4) in
// hook_isContentPending; A/B key for Round 2 (H2). Off by default; nothing
// reads it yet.
extern bool islandReadinessRuleEnabled;

// readinessOverrides: false = the H15 control -- every readiness override off
// (the isContentPending deferral and the promotion fallback) (H2, Z). On by
// default; nothing reads it yet.
extern bool readinessOverridesEnabled;

// npcWaitDiag: Phase 17 Step 1 NPC path-wait diagnostic (P). DEV default on,
// PROD default off; nothing reads it yet.
extern bool npcWaitDiagEnabled;

// gatePassDiag: Phase 17 Step 1 per-pass gate-code timing (P). DEV default
// on, PROD default off; nothing reads it yet.
extern bool gatePassDiagEnabled;


// =========================================================================
// NavMesh worker pool constants
// =========================================================================

// Capacity, not the live count: it sizes the worker handle arrays and the L2
// in-flight table, and bounds what navmeshWorkerCount may be set to. The number
// of workers actually created is g_navMeshWorkerCount (default 3).
const int NAVMESH_WORKER_COUNT = 6;
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
// loadSingleZone's 4th argument (xmm3) for the zones the mod preloads. The game
// writes it to zoneEntry + 4*(timerIndex + 48) and it decides when that zone is
// unloaded again. 0 takes the game's own per-timer default, which is what
// processState2 passes; a positive value overrides it. Exists so the surviving
// half of crash-3 hypothesis H2 can be A/B'd (0 vs 3600) without a rebuild.
extern float  cfg_preloadKeepAliveSeconds;
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
// The clamped worker count, i.e. how many threads CreateNavMeshWorkers starts
// and how many the priority boost/restore loops walk. Set once from
// cfg_navmeshWorkerCount after clamping.
extern int    g_navMeshWorkerCount;

// NavMesh L2 disk cache
extern int    cfg_navmeshDiskCacheMaxMB; // navmesh_cache\ size cap in MB; past it the
                                         // oldest files are deleted down to 75% of the cap

// FNV-1a over the game's active mod list (mods.cfg), read once on the main
// thread in LoadConfig. Part of every L2 disk cache filename: a changed mod set
// can move terrain and buildings, so its meshes must not be reused (ZO-11).
// 0 means "mod list unavailable".
extern unsigned int g_modSetHash;

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
