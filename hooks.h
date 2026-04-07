// hooks.h — Main hook function declarations (Layer 3)
// Depends on: config.h

#ifndef KENSHI_ZONE_OPT_HOOKS_H
#define KENSHI_ZONE_OPT_HOOKS_H

#include "config.h"


// =========================================================================
// Hook functions (impl in hooks.cpp)
// =========================================================================

void hook_showLoadingMessage(void* thisPtr, bool on);
bool hook_isContentPending(void* manager, void* zonePos);
void hook_addOrderSelected(void* thisPI, void* destIndoors, int task,
                            void* subject, bool shift, bool addDontClear,
                            const float* location);
void hook_updateCameraZone(void* zoneMgr, void* cameraPos);


#endif // KENSHI_ZONE_OPT_HOOKS_H
