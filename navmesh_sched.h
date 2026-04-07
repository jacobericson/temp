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
	int currentX, currentY;
	int destX, destY;
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

// 5-tier job queue reordering — classification data passed explicitly.
// camGridX/Y: resolved camera position (caller handles transition-target override).
// movers/moverCount: watched characters with current/dest zones (can be NULL/0).
// preloaded/preloadedCount: preloaded zone coordinates (can be NULL/0).
void PrioritizeNavMeshQueue(int camGridX, int camGridY,
                            const SchedMoverInfo* movers, int moverCount,
                            const SchedZoneInfo* preloaded, int preloadedCount);


#endif // KENSHI_ZONE_OPT_NAVMESH_SCHED_H
