// hooks.h — Main hook function declarations (Layer 3)
// Depends on: config.h, navmesh_sched.h, preload.h, tracking.h

#ifndef KENSHI_ZONE_OPT_HOOKS_H
#define KENSHI_ZONE_OPT_HOOKS_H

#include "config.h"
#include "navmesh_sched.h"
#include "preload.h"
#include "tracking.h"


// =========================================================================
// Hook functions (impl in hooks.cpp)
// =========================================================================

void hook_showLoadingMessage(void* thisPtr, bool on);
bool hook_isContentPending(void* manager, void* zonePos);
void hook_addOrderSelected(void* thisPI, void* destIndoors, int task,
                            void* subject, bool shift, bool addDontClear,
                            const float* location);
void hook_updateCameraZone(void* zoneMgr, void* cameraPos);


// =========================================================================
// NavMesh scheduling context (shared by PrioritizeNavMeshQueue callers and
// the tier-ordered registration/promotion in preload.cpp). Main thread only.
// =========================================================================

struct SchedContext {
	int camX, camY;                      // transition target if active, else last camera zone
	SchedMoverInfo movers[MAX_WATCHED];
	int moverCount;
	SchedZoneInfo zones[MAX_PRELOADED];
	int zoneCount;
};

void BuildSchedContext(SchedContext* ctx);


#endif // KENSHI_ZONE_OPT_HOOKS_H
