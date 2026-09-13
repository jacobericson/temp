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

// Deferred transition completion (B6). The dismissal call to showLoadingMessage
// can arrive on the contentStream (path) thread, concurrently with the main
// thread, so that hook does no CRT work at all: it stamps the end time and the
// calling thread id, then raises transitionEndPending. hook_updateCameraZone
// claims the flag with InterlockedCompareExchange and runs the completion body
// (thread restore, both log lines, preload reset) on the main thread.
// transitionEndQpc and transitionEndTid are written before the flag is raised
// and read after it is claimed; the interlocked pair on the flag orders them.
extern volatile LONG  transitionEndPending;
extern LARGE_INTEGER  transitionEndQpc;
extern DWORD          transitionEndTid;

// Runs the deferred completion if one is pending. Main thread only.
// Bracket instrumentation (O11) is file-local to hooks.cpp: the target 3x3
// line at bracket open ("Transition target:") and, for transitions over
// 300 ms, " frames=s<state>:<n>,..." appended to the "Transition:" line.
void TransitionCompleteIfPending();


#endif // KENSHI_ZONE_OPT_TRANSITION_H
