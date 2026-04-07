// pathfind_cache.h -- Squad path cache + Phase 12 priority boost state (Layer 3)
// FULL builds only (excluded by ZONEOPT_ZONEONLY via config.h inline stubs).
// Depends on: config.h, formation.h

#ifndef KENSHI_ZONE_OPT_PATHFIND_CACHE_H
#define KENSHI_ZONE_OPT_PATHFIND_CACHE_H

#include "config.h"
#include "formation.h"

#if PATHFIND_STEP >= 1


// =========================================================================
// Phase 12: priority boost state (Step 2+, main thread only)
// =========================================================================

#if PATHFIND_STEP >= 2

// Set by hook_requestPath, read by hook_pathReqSubmit (both main thread)
extern int squadBoostTier;       // 0=none, 45=formation, 50=user no-formation, 60=user+formation
extern int squadBoostGroupIdx;   // matched formation group index, -1=no match

// Phase 12 diagnostic counters
extern volatile long p12DiagRequestPathHits;
extern volatile long p12DiagTier60;
extern volatile long p12DiagTier50;
extern volatile long p12DiagTier40;
extern volatile long p12DiagTier30;
extern volatile long p12DiagSubmitBoosts;
extern volatile long p12DiagFallbackTags;
extern volatile long p12DiagRepathLeaders;
extern volatile long p12DiagRepathInjections;
extern volatile long p12DiagSCGuardSkips;

void LogPhase12Stats(double now);

#endif // PATHFIND_STEP >= 2


// =========================================================================
// Squad path cache (Step 4+)
// =========================================================================

#if PATHFIND_STEP >= 4

struct SquadPathCacheSlot {
	int    state;            // 0=IDLE..6=REPATH_INJECT
	float  expGoalX, expGoalY, expGoalZ;  // expected goal (raw Havok, no offset)
	unsigned int goalFaceKey; // resolved dest face key
	void*  edgeData;         // cached m_visitedEdges (CRT-owned)
	int    edgeCount;
	int    numIter;
	int    goalIdx;
	float  pathCost;
	double activatedTime;
	int    memberCount;
	int    servedCount;
	int    cachedSCSize;     // streaming collection size at cache time
	double lastInjectionTime;
	int    burstInjected;
};

extern SquadPathCacheSlot spcSlots[MAX_FORMATION_GROUPS];

// Per-slot candidate offsets (computed by hook_csFindPath per request)
extern float spcSlotCandOffX[MAX_FORMATION_GROUPS];
extern float spcSlotCandOffZ[MAX_FORMATION_GROUPS];

// Cache constants
extern const float  SPC_GOAL_TOL;
extern const double SPC_TIMEOUT;
extern const int    SPC_BUDGET_MULT;
extern const double SPC_BURST_TIMEOUT;
extern const double SPC_REPATH_TIMEOUT;

// Shared pass-through (same bg thread, set by csCheckFaceConn for current request)
extern unsigned int spcLastConnStartFace;
extern unsigned int spcLastConnDestFace;

// contentStream bg thread: tag set by hook_findPathFallback, read by hook_findPathFull
extern int spcFallbackTag;

// Streaming collection size at cache creation (for invalidation guard)
extern int spcCachedSCSize;

// Burst tracking
extern double spcLastInjectionTime;
extern int    spcBurstInjected;

// SPC diagnostic counters
extern volatile long spcDiagSignals;
extern volatile long spcDiagLeaderOK;
extern volatile long spcDiagLeaderFail;
extern volatile long spcDiagInjected;
extern volatile long spcDiagExpired;
extern volatile long spcDiagBoosted;
extern volatile long spcDiagActiveSlots;

// Slot management
void SpcFreeSlot(int s);
void SpcResetSlot(int s);
void SpcResetAll();

void LogSquadPathCacheStats(double now);

#endif // PATHFIND_STEP >= 4


#endif // PATHFIND_STEP >= 1

#endif // KENSHI_ZONE_OPT_PATHFIND_CACHE_H
