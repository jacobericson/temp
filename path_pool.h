// path_pool.h — Phase 17 Step 1 instrumentation (research/path_worker_pool.md §7)
// Declarations below are available when PATHPOOL_STEP >= 1; below that,
// inline no-op stubs with the same signatures so callers compile in every
// variant without their own #if.

#ifndef KENSHI_ZONE_OPT_PATH_POOL_H
#define KENSHI_ZONE_OPT_PATH_POOL_H

#include "core.h"
#include "config.h"

// One path search, recorded by hook_findPathFull on whatever thread ran it
// (the path thread, a NavMesh thread, or the main thread). Implementations must
// be lock-free, allocation-free and must not log: this is called on threads
// where CRT strings are forbidden.
struct PathSearchSample
{
	LONGLONG ticks;       // QPC ticks around orig_findPathFull
	int      iterations;  // FindPathOutput +48
	int      status;      // FindPathOutput +60 (1 = SUCCESS)
	int      cause;       // FindPathOutput +61 (termination cause)
	int      boosted;     // 1 if the player search budget was written for this search
	int      playerByTag; // the current player tag (currentRequestIsPlayer): 0 or 1
	int      playerByReq; // *(int*)(req + 0x2C) >= 20: 0 or 1; -1 when the request is unknown
};

#if PATHPOOL_STEP >= 1

// Any thread; see the comment on PathSearchSample above.
void PathPoolNoteSearch(const PathSearchSample* s);

// Main thread only, called every frame from hook_updateCameraZone; will own
// the 1 Hz NPC wait walk and the periodic PathQueue: / GateRate: / PathSlow: /
// AstarCost: lines.
void PathPoolTickMain(double now);

// =========================================================================
// Four pass-through hooks (research/path_worker_pool.md §7). Typedefs, RVA
// constants and orig_ pointer storage live in game.h/game.cpp's "Round 2 P"
// blocks; the bodies are in path_pool.cpp. main.cpp installs them (gated on
// PATHPOOL_STEP >= 1) through VerifyPrologueByRva + KenshiLib::AddHook, same
// as the destroyList inserter.
// =========================================================================

// SectionManager::contentStream (0x3AE350), path thread. QPC around the
// original; return value (1 = a request was served this pass). Latches the
// first caller's thread id as the path thread id (below) and the section
// manager pointer (for the dequeueWork/enqueue queue filters), and marks a
// thread-local for the duration of the pass so the Gates__updateCodes hook
// can tell it is running inside one.
char hook_contentStream(void* sectionMgr);

// SectionManager::dequeueWork_threadSafe (0x3BEE40), path thread. After the
// original returns a request from the input queue, stamps the drain QPC into
// the request's first 8 bytes (req+0x00, never otherwise written). Filtered
// to calls whose queueBase is the latched section manager's input queue
// (mgr+0xB8); the sole caller (contentStream) always passes that queue, so
// this is a defensive check, not a live filter.
void* hook_dequeueWork(void* queueBase);

// PathRequestQueue::enqueue_threadSafe (0x3B6110), main thread (submit, input
// queue) and path thread (contentStream, result queue). Only when queueBase
// is the latched section manager's result queue (mgr+0xF8): records the
// completion latency (now - req+0x00) and the per-request service time (now
// - the pass-start QPC latched by hook_contentStream) for a stamped,
// non-sentinel request, feeding the PathQueue:/PathSlow: window stats.
void hook_enqueueThreadSafe(void* queueBase, void** itemPtr);

// Gates__updateCodes (0x2EF460), path thread. QPC from function entry (so
// zero-gate passes count), whether a transition bracket is open
// (isTransitionActive), and, through a thread-local flag PathPoolNoteSearch
// reads, attributes the searches/terminations that run inside the pass to
// the gate counters instead of the path-thread queue counters. Gated by
// gatePassDiagEnabled (main.cpp installs it only when that INI key is on).
__int64 hook_gatesUpdateCodes(void* gatesObj);

// Thread id of the first hook_contentStream caller, i.e. the path thread.
// Other code classifies threads by comparing GetCurrentThreadId() against
// this (alongside g_navMeshBgThreadId and IsMainThread()).
extern volatile DWORD g_pathThreadId;

#else

inline void PathPoolNoteSearch(const PathSearchSample* s) { (void)s; }
inline void PathPoolTickMain(double now) { (void)now; }

#endif // PATHPOOL_STEP >= 1

#endif // KENSHI_ZONE_OPT_PATH_POOL_H
