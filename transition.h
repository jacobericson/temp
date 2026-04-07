// transition.h — Zone transition bracket state (Layer 3)
// isTransitionActive/deferredFrameCount/transitionStartTime defined in hooks.cpp.
// navMeshThreadHandle/savedThreadPriority are file-static in preload.cpp.
// Depends on: config.h (for Windows.h types)

#ifndef KENSHI_ZONE_OPT_TRANSITION_H
#define KENSHI_ZONE_OPT_TRANSITION_H

#include "config.h"


// =========================================================================
// Transition state (defined in hooks.cpp, read by preload.cpp)
// =========================================================================

extern bool           isTransitionActive;
extern int            deferredFrameCount;
extern LARGE_INTEGER  transitionStartTime;


#endif // KENSHI_ZONE_OPT_TRANSITION_H
