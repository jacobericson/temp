// pathfinding.h -- Pathfinding hook declarations + public function declarations (Layer 3)
// Gated by PATHFIND_STEP (0=none, 1=diag, 2=+boost, 3=+bypass, 4=+cache).
// Depends on: config.h

#ifndef KENSHI_ZONE_OPT_PATHFINDING_H
#define KENSHI_ZONE_OPT_PATHFINDING_H

#include "config.h"

// Step 1+: Diagnostic hooks (bg thread) + requestPath (main thread)
#if PATHFIND_STEP >= 1
char hook_csFindPath(void* manager, unsigned int startFaceKey, void* startPos,
                      void* destPos, float radius, char param5, void* resultBuf);
char hook_csCheckFaceConn(void* manager, unsigned int startFace, unsigned int destFace);
void hook_findPathFull(void* streamingCollection, void* searchState, void* findPathOutput);
void hook_requestPath(void* havokChar, float* destination, int priority);

void LogPathfindDiagStats(double now);
void ArmPathProbe();
void DumpPathProbe(double now);
#endif

// Step 2+: Priority boost hooks + stuck detection (main thread)
#if PATHFIND_STEP >= 2
void hook_pathReqSubmit(void* sectionMgr, void* requestObj, bool highPriority);
void LogPhase12Stats(double now);
void PollPlayerMovementState(double now);
void StorePlayerClickDest(uintptr_t character, const float* dest, double now);
#else
inline void PollPlayerMovementState(double) {}
#endif

// Step 4+: Cache hooks (bg thread)
#if PATHFIND_STEP >= 4
char hook_csFindPathFallback(void* manager, unsigned int startFaceKey, void* startPos,
                              unsigned int destFaceKey, void* destPos, float radius,
                              float param6, char param7, void* resultBuf);
void LogSquadPathCacheStats(double now);
#endif

#endif // KENSHI_ZONE_OPT_PATHFINDING_H
