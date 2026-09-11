// navmesh_sched.h — NavMesh thread priority boost and job queue prioritization
// Self-contained: no dependency on preload/tracking/worker_pool globals.
// Depends on: core.h, game.h (for gameBase, RVAs, queue lock fns)

#ifndef KENSHI_ZONE_OPT_NAVMESH_SCHED_H
#define KENSHI_ZONE_OPT_NAVMESH_SCHED_H

#include "config.h"


// =========================================================================
// Priority context structs (lightweight, no preload/tracking types)
// =========================================================================

struct SchedMoverInfo {
	int  currentX, currentY;
	int  destX, destY;
#if PATHFIND_STEP >= 5
	bool hasMoveOrder;
#endif
#if PATHFIND_STEP >= 6
	int  exitZoneGX, exitZoneGY;  // INT_MIN if no ExitFace decoded
#endif
};

struct SchedZoneInfo {
	int gridX, gridY;
};


// =========================================================================
// NavMesh scheduling functions (impl in navmesh_sched.cpp)
// =========================================================================

// NMG background thread priority boost (game.h globals only, no worker pool)
void BoostNavMeshThread();
void RestoreNavMeshThread();

// 5-tier priority for one zone (1 = most urgent .. 5 = least). Shared by
// PrioritizeNavMeshQueue and the tier-ordered registration/promotion in
// preload.cpp (ISLAND_STEP >= 3).
int ComputeZonePriority(int gridX, int gridY,
                        int camGridX, int camGridY,
                        const SchedMoverInfo* movers, int moverCount,
                        const SchedZoneInfo* preloaded, int preloadedCount);

// 5-tier job queue reordering — classification data passed explicitly.
// camGridX/Y: resolved camera position (caller handles transition-target override).
// movers/moverCount: watched characters with current/dest zones (can be NULL/0).
// preloaded/preloadedCount: preloaded zone coordinates (can be NULL/0).
void PrioritizeNavMeshQueue(int camGridX, int camGridY,
                            const SchedMoverInfo* movers, int moverCount,
                            const SchedZoneInfo* preloaded, int preloadedCount);


#if PATHFIND_STEP >= 5
// Reprio request flag. Writer (STEP 6+): hook_findPathFull on contentStream bg
// thread when ExitFace decoded -> InterlockedExchange(&g_reprioRequested, 1).
// Reader: hook_updateCameraZone on main thread, InterlockedExchange(&g, 0)
// reads + clears atomically. Lost-update window: if writer fires between
// reader's clear and the subsequent reprio call, the next backstop tick
// catches it (single-bit set semantic — multiple sets coalesce into one fire).
extern volatile long g_reprioRequested;

extern volatile long reprioFlagFires;
extern volatile long reprioTimerFires;
extern volatile long reprioOrderFires;
#endif


#endif // KENSHI_ZONE_OPT_NAVMESH_SCHED_H
