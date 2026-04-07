// pathfind_diag.h -- Pathfinding diagnostic state, probes, player failure tracking (Layer 3)
// FULL builds only (excluded by ZONEOPT_ZONEONLY via config.h inline stubs).
// Depends on: config.h

#ifndef KENSHI_ZONE_OPT_PATHFIND_DIAG_H
#define KENSHI_ZONE_OPT_PATHFIND_DIAG_H

#include "config.h"

#if PATHFIND_STEP >= 1


// =========================================================================
// PathProbeEntry (moved from config.h)
// =========================================================================

const int PATH_PROBE_SIZE = 24;

struct PathProbeEntry {
	float startX, startY, startZ;
	float destX, destY, destZ;
	int   hookType;   // 1=csFindPath, 2=findPathFull, 3=csCheckFaceConn
	int   result;     // search status (1=OK, 2=unreachable, 3=terminated, 5=invalid)
	unsigned int faceKey;
};


// =========================================================================
// Diagnostic counters (volatile, Interlocked from bg thread)
// =========================================================================

// Hook 1: Primary path (ContentStream::findPath)
extern volatile long diagPrimaryAttempts;
extern volatile long diagPrimarySuccess;
extern volatile long diagPrimaryFail;

// Hook 2: Face connectivity check
extern volatile long diagConnAttempts;
extern volatile long diagConnFail;

// Hook 3: Full A* search
extern volatile long diagAstarAttempts;
extern volatile long diagAstarSuccess;
extern volatile long diagAstarUnreachable;
extern volatile long diagAstarTerminated;
extern volatile long diagAstarTruncated;
extern volatile long diagAstarInvalid;
extern volatile long diagAstarOther;

// Termination cause breakdown (when status == 3)
extern volatile long diagTermIterLimit;
extern volatile long diagTermOpenSetFull;
extern volatile long diagTermStatesFull;
extern volatile long diagTermOtherCause;

// Iteration tracking
extern volatile long diagMaxIterUsed;
extern volatile long diagLastTermIter;

// Player vs NPC request counters (main thread, from hook_requestPath priority param)
extern volatile long diagPlayerRequests;
extern volatile long diagNPCRequests;


// =========================================================================
// FindPathInput layout probe (one-time, bg thread writes, main thread reads)
// =========================================================================

extern volatile long  probeFPIDumped;
extern volatile float probeStartPos[4];
extern volatile float probeGoalPos[4];
extern volatile long  probeGoalPtrValid;
extern volatile long  probeFields[14];


// =========================================================================
// Multi-call path probe state
// =========================================================================

extern volatile long  pathProbeArmed;
extern volatile long  pathProbeWriteIdx;
extern PathProbeEntry pathProbeBuf[PATH_PROBE_SIZE];
extern double         pathProbeArmTime;


// =========================================================================
// Player failure tracking
// =========================================================================

// Last A* failure details — any request (bg thread writes, main thread reads)
struct AstarFailDetail {
	volatile float goalX, goalY, goalZ;
	volatile long  status;
	volatile long  cause;
	volatile long  iterCount;
	volatile long  sequence;  // monotonic for freshness
};

extern AstarFailDetail lastAstarFail;

#if PATHFIND_STEP >= 2
// Per-player failure ring buffer (bg thread writes, main thread drains)
// Uses priority-based tagging: boosted player requests (pri 50+) are at the
// top of the contentStream queue, so we count them down in hook_csFindPath.
struct PlayerFailEntry {
	volatile float goalX, goalZ;
	volatile long  status;
	volatile long  cause;
	volatile long  iterCount;
	volatile long  valid;  // 1=written by bg, 0=consumed by main
};

const int PLAYER_FAIL_RING = 16;
extern PlayerFailEntry playerFailRing[PLAYER_FAIL_RING];
extern volatile long   playerFailWriteIdx;
extern volatile long   playerRequestsInFlight;  // main increments, bg decrements
#endif // PATHFIND_STEP >= 2


// =========================================================================
// Diagnostic function declarations
// =========================================================================

void LogPathfindDiagStats(double now);
void ArmPathProbe();
void DumpPathProbe(double now);

#if PATHFIND_STEP >= 2

// Per-character stuck recovery tracking (main thread only)
struct TrackedPlayerDest {
	uintptr_t character;        // Character* pointer (0 = empty slot)
	float destX, destY, destZ;  // world-space click destination
	float prevPosX, prevPosZ;   // previous poll position (for velocity)
	double clickTime;           // when the order was issued
	double lastRetryTime;       // last recovery attempt time
	int  retryCount;            // recovery attempts this cycle
	int  zeroVelocityPolls;     // consecutive polls with no movement
	bool active;                // slot in use
};

const int MAX_TRACKED_PLAYERS = 256;
extern TrackedPlayerDest trackedPlayers[MAX_TRACKED_PLAYERS];
extern int trackedPlayerCount;

void StorePlayerClickDest(uintptr_t character, const float* dest, double now);
void PollPlayerMovementState(double now);

#else
inline void PollPlayerMovementState(double) {}
#endif


#endif // PATHFIND_STEP >= 1

#endif // KENSHI_ZONE_OPT_PATHFIND_DIAG_H
