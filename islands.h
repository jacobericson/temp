// islands.h — Island routing overlay (Phase 15): mod-promoted zones join the
// game's routing islands without touching ZoneMap+0x20 or _calculateIslands.
// Depends on: config.h
//
// Threading contract:
//   - Builder (IslandTick, IslandMarkModZone, IslandRequestRebuild, IslandNoteOrder,
//     IslandDescribeStuck, IslandEmulateCrossing): MAIN THREAD ONLY.
//   - Hooks (hook_isInIsland, hook_getIsland): any thread that calls
//     CharMovement::setDestination or the smell picker (main, AI, render back
//     thread). They take no locks, allocate nothing, log nothing; they read the
//     published snapshot through a seqlock and bump Interlocked counters.
//   - IslandCountReadiness: any thread (hook_isContentPending).
//
// ISLAND_STEP (temporary staging gate, see config.h):
//   0 = readiness-gate side effects on the main thread only + per-thread counters
//   1 = builder, snapshot, marks, load reset, emulation, PLAYER STUCK fields;
//       both hooks installed but PASS THROUGH (count would-flip / would-append)
//   2 = both hooks LIVE (subject to the islandFix INI key)
//   3 = parked-squad re-issue (a) + tier-ordered registration/promotion (b)

#ifndef KENSHI_ZONE_OPT_ISLANDS_H
#define KENSHI_ZONE_OPT_ISLANDS_H

#include "config.h"


// =========================================================================
// Step 0: readiness-gate per-thread counters (always compiled)
// =========================================================================

// Called from hook_isContentPending on whichever thread the game uses.
// notReady = original returned 0; sectionsPending = notReady with sections > 0.
void IslandCountReadiness(bool notReady, bool sectionsPending);

// Main thread, every frame from hook_updateCameraZone (before the stats reporters).
// At ISLAND_STEP 0 it only logs the readiness counters.
void IslandTick(void* zoneMgr, double now);


#if ISLAND_STEP >= 1

// =========================================================================
// Hooks (installed by main.cpp when preloadEnabled; both or neither go live)
// =========================================================================

bool  hook_isInIsland(void* zoneA, void* zoneB);
void* hook_getIsland(void* zoneMgr, void* zone, void* lektorOut);

// main.cpp: called once after both hooks installed successfully.
void IslandSetHooksInstalled(bool installed);

// True when the overlay answers the hooks (installed && islandFix && ISLAND_STEP >= 2).
bool IslandHooksLive();


// =========================================================================
// Marks + rebuild requests (main thread)
// =========================================================================

// preload.cpp: called just before the WORD write that makes a promoted zone
// accessible. Marks survive ClearPreloadZones.
void IslandMarkModZone(void* zoneMgr, int gx, int gy);

// preload.cpp: called after a promotion completes (forces a rebuild this frame).
void IslandRequestRebuild();

// preload.cpp (ClearPreloadState): full reset of marks, snapshot and tracker.
void IslandReset();


// =========================================================================
// Router emulation + diagnostics (main thread)
// =========================================================================

// Emulate CharMovement::computeProjectedDest for a character standing in
// charZone, along the ray from (destX,destZ) toward (posX,posZ), using the
// island list the router receives in the CURRENT configuration (vanilla when
// the hooks pass through, overlay when live). Returns the RAW first crossing
// (before the 300-unit navmesh snap). false = no island zone crossed.
bool IslandEmulateCrossing(void* zoneMgr, void* charZone,
                           float destX, float destZ, float posX, float posZ,
                           float* outX, float* outZ);

// Extra fields for the PLAYER STUCK diagnostic line (pathfind_diag.cpp).
struct IslandStuckInfo {
	float wpX, wpZ;          // CharMovement::pathDestination (+0xE8)
	int   movingToEdge;      // +0x370
	int   edgeCounter;       // +0x368
	int   selfComp;          // liveComp of the character's zone (-1 none)
	bool  haveCrossing;      // emulation found a crossing
	float xd;                // |pos - raw crossing| when haveCrossing
	int   nextGX, nextGY;    // zone just beyond the crossing (or grid walk fallback)
	int   nextComp;          // liveComp(next)
	int   nextLabel;         // next zone +0x20
	int   nextLoading;       // next zone +176
	int   nextAccess;        // next zone +177
};
bool IslandDescribeStuck(void* zoneMgr, uintptr_t charMov,
                         float posX, float posZ, float destX, float destZ,
                         IslandStuckInfo* out);

#else // ISLAND_STEP < 1

inline void IslandSetHooksInstalled(bool) {}
inline bool IslandHooksLive() { return false; }
inline void IslandMarkModZone(void*, int, int) {}
inline void IslandRequestRebuild() {}
inline void IslandReset() {}

#endif // ISLAND_STEP >= 1


#if ISLAND_STEP >= 3
// hooks.cpp: called for every selected character of a task-29 (move) order.
void IslandNoteOrder(uintptr_t character, const float* location);
#else
inline void IslandNoteOrder(uintptr_t, const float*) {}
#endif


#endif // KENSHI_ZONE_OPT_ISLANDS_H
