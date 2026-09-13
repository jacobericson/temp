// pathfind_diag.cpp -- Diagnostic counters, reporters, probes, player failure tracking
// FULL builds only (excluded by ZONEOPT_ZONEONLY via config.h inline stubs).

#include "pathfind_diag.h"
#include "grid.h"
#include "islands.h"

#if PATHFIND_STEP >= 1


// =========================================================================
// Diagnostic counter definitions
// =========================================================================

volatile long diagPrimaryAttempts  = 0;
volatile long diagPrimarySuccess   = 0;
volatile long diagPrimaryFail      = 0;

volatile long diagConnAttempts     = 0;
volatile long diagConnFail         = 0;

volatile long diagAstarAttempts    = 0;
volatile long diagAstarSuccess     = 0;
volatile long diagAstarUnreachable = 0;
volatile long diagAstarTerminated  = 0;
volatile long diagAstarTruncated   = 0;
volatile long diagAstarInvalid     = 0;
volatile long diagAstarOther       = 0;

volatile long diagTermIterLimit    = 0;
volatile long diagTermOpenSetFull  = 0;
volatile long diagTermStatesFull   = 0;
volatile long diagTermOtherCause   = 0;

volatile long diagMaxIterUsed      = 0;
volatile long diagLastTermIter     = 0;

volatile long diagPlayerRequests   = 0;
volatile long diagNPCRequests      = 0;


// =========================================================================
// FindPathInput layout probe state
// =========================================================================

volatile long  probeFPIDumped      = 0;
volatile float probeStartPos[4]    = {};
volatile float probeGoalPos[4]     = {};
volatile long  probeGoalPtrValid   = 0;
volatile long  probeFields[14]     = {};


// =========================================================================
// Multi-call path probe state
// =========================================================================

volatile long  pathProbeArmed    = 0;
volatile long  pathProbeWriteIdx = 0;
PathProbeEntry pathProbeBuf[PATH_PROBE_SIZE];
double         pathProbeArmTime  = 0.0;


// =========================================================================
// Player failure tracking state
// =========================================================================

AstarFailDetail lastAstarFail = {};

#if PATHFIND_STEP >= 2
PlayerFailEntry playerFailRing[PLAYER_FAIL_RING] = {};
volatile long   playerFailWriteIdx    = 0;
volatile long   playerRequestsInFlight = 0;

TrackedPlayerDest trackedPlayers[MAX_TRACKED_PLAYERS] = {};
int trackedPlayerCount = 0;
#endif


// =========================================================================
// Reporter timing + per-window snapshots (main thread only)
// =========================================================================

static double lastPathDiagLogTime = 0.0;

static long prevPrimaryAttempts = 0;
static long prevPrimarySuccess  = 0;
static long prevPrimaryFail     = 0;
static long prevAstarAttempts   = 0;
static long prevAstarSuccess    = 0;
static long prevAstarUnreach    = 0;
static long prevAstarTerminated = 0;
static long prevAstarInvalid    = 0;
static long prevConnAttempts    = 0;
static long prevPlayerRequests  = 0;
static long prevNPCRequests     = 0;
static long prevFailSequence    = 0;


// =========================================================================
// LogPathfindDiagStats (called from hook_updateCameraZone)
// =========================================================================

void LogPathfindDiagStats(double now)
{
	if (!pathfindDiagEnabled)
		return;

#ifdef ZONEOPT_DEBUG
	if (now - lastPathDiagLogTime < 10.0)
#else
	if (now - lastPathDiagLogTime < 30.0)
#endif
		return;
	lastPathDiagLogTime = now;

	// Snapshot all counters (non-destructive read)
	long pAttempts = InterlockedCompareExchange(&diagPrimaryAttempts, 0, 0);
	long pSuccess  = InterlockedCompareExchange(&diagPrimarySuccess, 0, 0);
	long pFail     = InterlockedCompareExchange(&diagPrimaryFail, 0, 0);

	long cAttempts = InterlockedCompareExchange(&diagConnAttempts, 0, 0);
	long cFail     = InterlockedCompareExchange(&diagConnFail, 0, 0);

	long aAttempts  = InterlockedCompareExchange(&diagAstarAttempts, 0, 0);
	long aSuccess   = InterlockedCompareExchange(&diagAstarSuccess, 0, 0);
	long aUnreach   = InterlockedCompareExchange(&diagAstarUnreachable, 0, 0);
	long aTermed    = InterlockedCompareExchange(&diagAstarTerminated, 0, 0);
	long aTruncated = InterlockedCompareExchange(&diagAstarTruncated, 0, 0);
	long aInvalid   = InterlockedCompareExchange(&diagAstarInvalid, 0, 0);
	long aOther     = InterlockedCompareExchange(&diagAstarOther, 0, 0);

	long tIter     = InterlockedCompareExchange(&diagTermIterLimit, 0, 0);
	long tOpen     = InterlockedCompareExchange(&diagTermOpenSetFull, 0, 0);
	long tState    = InterlockedCompareExchange(&diagTermStatesFull, 0, 0);
	long tOther    = InterlockedCompareExchange(&diagTermOtherCause, 0, 0);

	long maxIter   = InterlockedCompareExchange(&diagMaxIterUsed, 0, 0);
	long lastTIter = InterlockedCompareExchange(&diagLastTermIter, 0, 0);

	if (pAttempts == 0 && aAttempts == 0)
		return;

	std::ostringstream ss;
	ss << "[ZoneOpt] PathDiag:"
	   << " primary=" << pAttempts << "/" << pSuccess << "/" << pFail
	   << " conn=" << cAttempts
#if PATHFIND_STEP >= 3
	   << "/bypass"
#else
	   << "/fail=" << cFail
#endif
	   << " astar=" << aAttempts << "/" << aSuccess
	   << "/" << aUnreach << "unreach"
	   << "/" << aTermed << "term"
	   << "/" << aInvalid << "inv";

	if (aTruncated > 0)
		ss << "/" << aTruncated << "trunc";
	if (aOther > 0)
		ss << "/" << aOther << "oth";

	if (aTermed > 0)
		ss << " term=" << tIter << "iter/" << tOpen << "open/" << tState << "state";
	if (tOther > 0)
		ss << "/" << tOther << "oth";

	ss << " maxIter=" << maxIter;
	if (lastTIter > 0)
		ss << " lastTermIter=" << lastTIter;

	if (aTermed > 0 || aUnreach > 0)
		LogMsg(ss.str());
	else
		LogDebug(ss.str());

	// Per-window throughput (delta since last report)
	{
		long dPrimary  = pAttempts - prevPrimaryAttempts;
		long dPSuccess = pSuccess  - prevPrimarySuccess;
		long dPFail    = pFail     - prevPrimaryFail;
		long dAstar    = aAttempts - prevAstarAttempts;
		long dASuccess = aSuccess  - prevAstarSuccess;
		long dAUnreach = aUnreach  - prevAstarUnreach;
		long dATermed  = aTermed   - prevAstarTerminated;
		long dAInvalid = aInvalid  - prevAstarInvalid;
		long dConn     = cAttempts - prevConnAttempts;

		long playerReqs = InterlockedCompareExchange(&diagPlayerRequests, 0, 0);
		long npcReqs    = InterlockedCompareExchange(&diagNPCRequests, 0, 0);
		long dPlayer = playerReqs - prevPlayerRequests;
		long dNPC    = npcReqs    - prevNPCRequests;

		prevPrimaryAttempts = pAttempts;
		prevPrimarySuccess  = pSuccess;
		prevPrimaryFail     = pFail;
		prevAstarAttempts   = aAttempts;
		prevAstarSuccess    = aSuccess;
		prevAstarUnreach    = aUnreach;
		prevAstarTerminated = aTermed;
		prevAstarInvalid    = aInvalid;
		prevConnAttempts    = cAttempts;
		prevPlayerRequests  = playerReqs;
		prevNPCRequests     = npcReqs;

#ifdef ZONEOPT_DEBUG
		double interval = 10.0;
#else
		double interval = 30.0;
#endif

		if (dPrimary > 0 || dAstar > 0)
		{
			long dAFail = dAstar - dASuccess;
			int successPct = (dAstar > 0) ? (int)(100 * dASuccess / dAstar) : 0;

			std::ostringstream ts;
			ts << std::fixed << std::setprecision(1);
			int playerPct = (dPlayer + dNPC > 0) ? (int)(100 * dPlayer / (dPlayer + dNPC)) : 0;

			ts << "[ZoneOpt] PathRate: "
			   << dPrimary << " req/" << (int)interval << "s"
			   << " (" << (dPrimary / interval) << "/s)"
			   << " player=" << dPlayer << "(" << playerPct << "%)"
			   << " npc=" << dNPC
			   << " direct=" << dPSuccess
			   << " fallback=" << dPFail
			   << " A*=" << dAstar
			   << " (" << (dAstar / interval) << "/s)"
			   << " ok=" << dASuccess << "(" << successPct << "%)"
			   << " fail=" << dAFail;
			if (dAFail > 0)
			{
				ts << "[unreach=" << dAUnreach;
				if (dATermed > 0)
					ts << " term=" << dATermed;
				if (dAInvalid > 0)
					ts << " inv=" << dAInvalid;
				ts << "]";
			}

			// Player failure exposure estimate
			if (dAFail > 0 && dPlayer > 0 && (dPlayer + dNPC) > 0)
			{
				int estPlayerFails = (int)((double)dAFail * dPlayer / (dPlayer + dNPC));
				ts << " playerExposure=~" << estPlayerFails;
			}

			LogMsg(ts.str());
		}

		// Last failure detail (when new failures since last report)
		long failSeq = InterlockedCompareExchange(&lastAstarFail.sequence, 0, 0);
		if (failSeq > prevFailSequence)
		{
			float gx = lastAstarFail.goalX * 10.0f;
			float gz = lastAstarFail.goalZ * 10.0f;
			long st = InterlockedCompareExchange(&lastAstarFail.status, 0, 0);
			long ca = InterlockedCompareExchange(&lastAstarFail.cause, 0, 0);
			long it = InterlockedCompareExchange(&lastAstarFail.iterCount, 0, 0);

			std::ostringstream fs;
			fs << std::fixed << std::setprecision(0);
			fs << "[ZoneOpt] LastFail: status=" << st << " cause=" << ca
			   << " iter=" << it
			   << " worldGoal=(" << gx << "," << gz << ")";
			LogMsg(fs.str());
			prevFailSequence = failSeq;
		}
	}

#if PATHFIND_STEP >= 2
	// Drain player failure ring buffer
	for (int i = 0; i < PLAYER_FAIL_RING; ++i)
	{
		if (InterlockedCompareExchange(&playerFailRing[i].valid, 0, 1) == 1)
		{
			float gx = playerFailRing[i].goalX * 10.0f;
			float gz = playerFailRing[i].goalZ * 10.0f;
			long st = playerFailRing[i].status;
			long ca = playerFailRing[i].cause;
			long it = playerFailRing[i].iterCount;

			const char* reason = "unknown";
			if (st == 99) reason = "CLUSTER_GRAPH_REJECTED";
			else if (st == 2) reason = "UNREACHABLE";
			else if (st == 3 && ca == 1) reason = "ITER_LIMIT";
			else if (st == 3 && ca == 2) reason = "OPEN_SET_FULL";
			else if (st == 3 && ca == 3) reason = "SEARCH_STATE_FULL";
			else if (st == 3) reason = "TERMINATED";
			else if (st == 5) reason = "INVALID_START";

			std::ostringstream pf;
			pf << std::fixed << std::setprecision(0);
			pf << "[ZoneOpt] PLAYER PATH FAIL: " << reason
			   << " status=" << st << " cause=" << ca
			   << " iter=" << it
			   << " worldGoal=(" << gx << "," << gz << ")";
			LogMsg(pf.str());
		}
	}
#endif

	// One-time FindPathInput layout probe report
	if (InterlockedCompareExchange(&probeFPIDumped, 2, 2) == 2)
	{
		InterlockedExchange(&probeFPIDumped, 3);

		std::ostringstream ps;
		ps << std::fixed << std::setprecision(2);
		ps << "[ZoneOpt] FindPathInput probe:"
		   << " startPos=(" << probeStartPos[0] << "," << probeStartPos[1]
		   << "," << probeStartPos[2] << "," << probeStartPos[3] << ")";

		if (probeGoalPtrValid)
			ps << " goalPos=(" << probeGoalPos[0] << "," << probeGoalPos[1]
			   << "," << probeGoalPos[2] << "," << probeGoalPos[3] << ")";
		else
			ps << " goalPtr=NULL";

		ps << " +40=" << probeFields[0]
		   << " +44=" << probeFields[1]
		   << " +48=" << probeFields[2]
		   << " +52=" << probeFields[3];

		ps << " +64=" << probeFields[6]
		   << " +68=" << probeFields[7]
		   << " +72=" << probeFields[8]
		   << " +76=" << probeFields[9];

		ps << " +128=" << probeFields[10]
		   << " +136=" << probeFields[11];

		ps << " +156=" << probeFields[12]
		   << " +160=" << probeFields[13];

		ps << " | +40f=" << std::setprecision(4) << *(float*)&probeFields[0]
		   << " +76f=" << *(float*)&probeFields[9];

		LogMsg(ps.str());
	}
}


// =========================================================================
// Multi-call path probe: arm + dump
// =========================================================================

void ArmPathProbe()
{
	for (int i = 0; i < PATH_PROBE_SIZE; ++i)
	{
		pathProbeBuf[i].hookType = 0;
		pathProbeBuf[i].result = 0;
	}
	InterlockedExchange(&pathProbeWriteIdx, 0);
	pathProbeArmTime = ElapsedSec();
	InterlockedExchange(&pathProbeArmed, 1);
	LogMsg("[ZoneOpt] PathProbe: armed");
}

void DumpPathProbe(double now)
{
	if (InterlockedCompareExchange(&pathProbeArmed, 0, 0) != 1)
		return;

	if (now - pathProbeArmTime < 8.0)
		return;

	InterlockedExchange(&pathProbeArmed, 0);

	long count = InterlockedCompareExchange(&pathProbeWriteIdx, 0, 0);
	if (count > PATH_PROBE_SIZE)
		count = PATH_PROBE_SIZE;

	{
		std::ostringstream ss;
		ss << "[ZoneOpt] PathProbe: " << count << " entries in "
		   << std::fixed << std::setprecision(1)
		   << (now - pathProbeArmTime) << "s";
		LogMsg(ss.str());
	}

	for (long i = 0; i < count; ++i)
	{
		const PathProbeEntry& e = pathProbeBuf[i];
		if (e.hookType == 0 || e.hookType == 3) continue;

		std::ostringstream ss;
		ss << std::fixed << std::setprecision(0);
		ss << "[ZoneOpt] PP[" << i << "] ";

		if (e.hookType == 1)
		{
			ss << "csFP (" << e.startX*10 << "," << e.startZ*10 << ")->("
			   << e.destX*10 << "," << e.destZ*10 << ") f=" << e.faceKey
			   << (e.result ? " OK" : " FAIL");
		}
		else if (e.hookType == 2)
		{
			const char* st = "?";
			if (e.result == 1) st = "OK";
			else if (e.result == 2) st = "UNRCH";
			else if (e.result == 3) st = "TERM";
			else if (e.result == 5) st = "INV";

			unsigned int dFace = 0;
			if (i > 0 && pathProbeBuf[i-1].hookType == 3)
				dFace = pathProbeBuf[i-1].faceKey;

			ss << "A* (" << e.startX*10 << "," << e.startZ*10 << ")->("
			   << e.destX*10 << "," << e.destZ*10 << ") sf=" << e.faceKey
			   << " " << st;
			if (dFace) ss << " df=" << dFace;
		}

		LogMsg(ss.str());
	}
}


// =========================================================================
// Player movement state poller (Step 2+)
// =========================================================================
// Traces the movement execution side when pathfinding succeeds but
// the character won't move.  Calls CharMovement methods through vtable
// (virtual) and KenshiLib stubs (non-virtual).

#if PATHFIND_STEP >= 2

// Minimal forward declaration — produces correct MSVC mangled symbols
// for linking against KenshiLib.lib stubs (patched by RE_Kenshi at runtime).
// Avoids heavy Ogre/Boost includes from the full CharMovement.h.
namespace Ogre { class Vector3 { public: float x, y, z; }; }

class AbstractMovementBase {
public:
	bool isProbablyStuck() const;
	Ogre::Vector3 getDestination() const;
};

// CharMovement vtable offsets (from CharMovement.h)
const size_t VT_GET_POSITION    = 0x40;
const size_t VT_PATH_OK         = 0x48;
const size_t VT_PATH_FAILED     = 0x50;
const size_t VT_IS_DEST_REACHED = 0x60;

typedef bool (__fastcall *BoolMethodFn)(void*);
typedef const float* (__fastcall *GetPosFn)(void*);

// Character::playerMoveOrderDefault (vtable+792 = 0x318)
typedef void (*moveOrderFn_t)(uintptr_t, void*, void*, const float*);

static const int    MAX_STUCK_RETRIES   = 5;   // 90%, 80%, 70%, 60%, 50%
static const double STUCK_RETRY_COOLDOWN = 5.0;
static const float  STUCK_PCT_START     = 0.9f; // first retry targets 90% of way to zone exit
static const float  STUCK_PCT_STEP      = 0.1f; // each retry reduces by 10%
static double lastStuckPollTime = 0.0;


#ifdef ZONEOPT_DEBUG
// =========================================================================
// PLAYER TASK diagnostic (DEV only; Round 2 fix 2a part C)
// =========================================================================
// Decides between the two ways the engine stops a moving player character
// (research r2-hostile-stop-research.md sections 1.6 and 4):
//   - the move order is deleted: t 29 -> -1 with hc136=1 and edge 1 -> 0
//     (the isDestinationReached shortcut), or with ps=3 (path failure);
//   - a threat preempts it: t 29 -> 32 (SELF_PRESERVATION) or a combat
//     action with thr>0, near<182, lvl=3; t -> 62 with bbReq>0 (STAND_STILL).
// One line per tracked player character on every change of
// (t, stopped, edge, hc136), and alongside every PLAYER STUCK line.
// Key a deletion on t and goal/lvl (0x50CD40 zeroes ts+0x1C0 and ts+0x20C)
// and on dq= (the order deque 0x50CD40 pops), not on ord= (the ts+0x88
// lektor 0x50DB20 scores, which the deletion does not pop).
//
// Every offset is a plain load (no game function is called), local to this
// file (game.h belongs to another change), with the RVA that proves it.
// Task selection, order pops and task deletion run on the game's threaded
// update, not the main thread, so every pointer chase is inside the one
// guarded helper below. threats hands are NOT resolved (that needs
// hand::getObject); the count, near and tp are enough.

// Character
static const size_t PT_OFF_CHAR_HIT       = 0x2B0; // u8 under melee attack now: 0x435430 reads (AI+0x2F8 = Character)+688
static const size_t PT_OFF_CHAR_MOVEMENT  = 0x640; // CharMovement*: 0x510460 l.51 calls *(Character+1600) vt+0x98 (stop)
static const size_t PT_OFF_CHAR_BODY      = 0x648; // CharBody*: 0x5C8820 reads *(Character+1608)
static const size_t PT_OFF_CHAR_AI        = 0x650; // AI*: 0x5C7C70 getSensoryData = *(Character+1616)+0x28
// CharBody
static const size_t PT_OFF_BODY_COMBAT    = 0x08;  // CombatClass*: 0x5C8820 returns *(CharBody+8)
static const size_t PT_OFF_BODY_ACTION    = 0x68;  // current action Tasker*: 0x5C6430 setCurrentTask, 0x5D1820
// Tasker / TaskData
static const size_t PT_OFF_TASKER_DATA    = 0x70;  // TaskData*: 0x50DB20 *(tasker+112)
static const size_t PT_OFF_TASKDATA_TYPE  = 0x44;  // int TaskType: 0x50DB20 *(TaskData+68) == 187
static const int    PT_TYPE_LIMIT         = 300;   // task/goal types outside [0, 300) are rejected
// AI (SensoryData embedded at AI+0x28, 0x5C7C70)
static const size_t PT_OFF_AI_PLATOON     = 0x10;  // 0x5065E0: *(AI+16), its +0xF8 is the Blackboard
static const size_t PT_OFF_AI_TASKSYS     = 0x20;  // AITaskSystem*: 0x510820 passes AI[4] down the think chain
static const size_t PT_OFF_AI_NEAREST_SQ  = 0x28;  // float SensoryData.nearestEnemy (squared): 0x858500 keeps the minimum
static const size_t PT_OFF_AI_THREATS     = 0x80;  // int threats count: 0x599290 *(AI+128); 0x8534A0 pushes at Sensory+0x50
static const size_t PT_OFF_AI_THREAT_PERS = 0xB0;  // float totalThreatLevelPersonal: 0x8534A0 Sensory+136
static const size_t PT_OFF_AI_NUM_ENEMIES = 0xBC;  // int numEnemies: 0x596BA0 NO_ENEMIES_IN_VICINITY reads AI+188
// AITaskSystem
static const size_t PT_OFF_TS_ORDER_COUNT = 0x90;  // int player order count: 0x50DB20 *(ts+144)
static const size_t PT_OFF_TS_ORDER_ARRAY = 0x98;  // Tasker** player orders: 0x50DB20 *(ts+152)
// u64 size of the player-order std::deque that the order-deletion function
// pops: 0x50CD40 calls 0x518210 (thunk 0x1A181) on ts+0x38; 0x518210 returns
// 0 when *(obj+40) == 0, else pops the deque at obj+8 and decrements its size
// v1[4] = obj+8+32 = obj+40 -> ts+0x60.
static const size_t PT_OFF_TS_DEQUE_SIZE  = 0x60;
static const size_t PT_OFF_TS_GOAL        = 0x1C0; // current goal record, first qword read as a Tasker*: 0x510460 ts+448 (INFERRED)
static const size_t PT_OFF_TS_GOAL_LEVEL  = 0x20C; // int level of the current goal: 0x510460 ts+524 gates the cascade
static const size_t PT_OFF_TS_FINISHED    = 0x26C; // u8 current action finished: 0x50BCA0 writes ts+620 = 1
static const size_t PT_OFF_TS_RETHINK     = 0x26D; // u8 re-think: 0x50BE00 writes ts+621 = 1
static const int    PT_ORDER_COUNT_LIMIT  = 1000;  // order counts outside [0, 1000) are rejected
// CharMovement
static const size_t PT_OFF_CMOV_STOPPED   = 0x08;  // u8 officiallyStopped: stop() 0x65F1E0 writes 1
static const size_t PT_OFF_CMOV_MOVING    = 0x24;  // u8 currentlyMoving: 0x65E320 this+36
static const size_t PT_OFF_CMOV_HC        = 0x320; // HavokCharacter*: 0x65E320 this+800
static const size_t PT_OFF_CMOV_EDGE_CTR  = 0x368; // int edge retry counter: 0x65DDA0 pathFailed (< 16)
static const size_t PT_OFF_CMOV_EDGE      = 0x370; // u8 movingToEdge: 0x65E320 this+880
// HavokCharacter
static const size_t PT_OFF_HC_ARRIVAL     = 0x88;  // int arrival code (1 = arrived): 0x65E320 hc+136 == 1
static const size_t PT_OFF_HC_PATH_STATE  = 0x90;  // int path state (3 = failed): 0x65DDA0 hc+144 == 3
// CombatClass
static const size_t PT_OFF_CC_STATE       = 0x1F0; // int combat state: 0x60C3A0 CombatClass::update this+496
static const size_t PT_OFF_CC_ATTACKERS   = 0x200; // int attackersH count (KenshiLib layout, INFERRED)
// Blackboard
static const size_t PT_OFF_PLATOON_BB     = 0xF8;  // Blackboard*: 0x5065E0 *(*(AI+16)+248)
static const size_t PT_OFF_BB_REQ_SIZE    = 0x178; // u64 TaskRequest map size: 0x269200, ctor 0x26BCB0

static const int PT_NA = (-2147483647 - 1);        // field unreadable (null link / rejected): prints "-"

// POD snapshot filled by ReadPlayerTaskSnap. Integer fields use PT_NA for
// "unreadable" (printed "-"); -1 in t / goal / ordHead means a null link
// (no current action, no goal, no order), as the research table prints it.
struct PlayerTaskSnap
{
	int       fault;       // 1 = a read faulted: every other field prints "-"
	int       t;           // current action type (CharBody+0x68 -> +0x70 -> +0x44)
	int       goal;        // current goal type (INFERRED: ts+0x1C0 -> +0x70 -> +0x44)
	int       lvl;         // ts+0x20C
	int       ordN;        // ts+0x90
	int       ordHead;     // (*(ts+0x98))[0] -> +0x70 -> +0x44
	long long dq;          // ts+0x60 (order deque size, the one 0x50CD40 pops), -1 = unreadable
	int       fin;         // ts+0x26C
	int       rethink;     // ts+0x26D
	int       stopped;     // CharMovement+0x08
	int       moving;      // CharMovement+0x24
	int       edge;        // CharMovement+0x370
	int       ctr;         // CharMovement+0x368
	int       hc136;       // HavokCharacter+0x88
	int       ps;          // HavokCharacter+0x90
	int       reached;     // computed, mirrors CharMovement::isDestinationReached 0x65E320
	int       en;          // AI+0xBC
	int       thr;         // AI+0x80
	int       haveAiF;     // 1 when nearSq / tp were read
	float     nearSq;      // AI+0x28 (squared)
	float     tp;          // AI+0xB0
	int       hit;         // Character+0x2B0
	int       cst;         // CombatClass+0x1F0
	int       atk;         // CombatClass+0x200 (INFERRED)
	long long bbReq;       // Blackboard+0x178, -1 = unreadable
};

// Type read through a Tasker* (inside ReadPlayerTaskSnap's __try only):
// -1 when the tasker or its TaskData is null, PT_NA when the type is outside
// [0, PT_TYPE_LIMIT). Kept a macro so every pointer chase stays lexically in
// the one guarded helper.
#define PT_TASKER_TYPE(taskerExpr, outField)                                   \
	do {                                                                       \
		uintptr_t ptTk_ = (taskerExpr);                                        \
		(outField) = -1;                                                       \
		if (ptTk_) {                                                           \
			uintptr_t ptTd_ = *(uintptr_t*)(ptTk_ + PT_OFF_TASKER_DATA);       \
			if (ptTd_) {                                                       \
				int ptTy_ = *(int*)(ptTd_ + PT_OFF_TASKDATA_TYPE);             \
				(outField) = (ptTy_ < 0 || ptTy_ >= PT_TYPE_LIMIT) ? PT_NA : ptTy_; \
			}                                                                  \
		}                                                                      \
	} while (0)

// The one guarded helper. Plain C: POD only, no C++ object in scope (MSVC
// 2010 rejects __try in a function with objects needing unwinding), no game
// function call, no allocation, no lock. Each pointer is read once into a
// local and null-checked before it is followed. A fault (a Tasker, order
// array or goal record deleted on the AI thread mid-read) sets fault=1; the
// fault never reaches the crash recorder (GuardEnter/GuardLeave, core.h).
// `character` has already passed PollPlayerMovementState's squad-list test.
static void ReadPlayerTaskSnap(uintptr_t character, PlayerTaskSnap* out)
{
	out->fault   = 0;
	out->t       = PT_NA;  out->goal    = PT_NA;  out->lvl     = PT_NA;
	out->ordN    = PT_NA;  out->ordHead = PT_NA;  out->fin     = PT_NA;
	out->rethink = PT_NA;  out->stopped = PT_NA;  out->moving  = PT_NA;
	out->edge    = PT_NA;  out->ctr     = PT_NA;  out->hc136   = PT_NA;
	out->ps      = PT_NA;  out->reached = PT_NA;  out->en      = PT_NA;
	out->thr     = PT_NA;  out->haveAiF = 0;      out->nearSq  = 0.0f;
	out->tp      = 0.0f;   out->hit     = PT_NA;  out->cst     = PT_NA;
	out->atk     = PT_NA;  out->bbReq   = -1;     out->dq      = -1;

	GuardEnter();
	__try
	{
		out->hit = *(unsigned char*)(character + PT_OFF_CHAR_HIT);

		uintptr_t body = *(uintptr_t*)(character + PT_OFF_CHAR_BODY);
		if (body)
		{
			uintptr_t action = *(uintptr_t*)(body + PT_OFF_BODY_ACTION);
			PT_TASKER_TYPE(action, out->t);
			uintptr_t cc = *(uintptr_t*)(body + PT_OFF_BODY_COMBAT);
			if (cc)
			{
				out->cst = *(int*)(cc + PT_OFF_CC_STATE);
				out->atk = *(int*)(cc + PT_OFF_CC_ATTACKERS);
			}
		}

		uintptr_t cm = *(uintptr_t*)(character + PT_OFF_CHAR_MOVEMENT);
		if (cm)
		{
			out->stopped = *(unsigned char*)(cm + PT_OFF_CMOV_STOPPED);
			out->moving  = *(unsigned char*)(cm + PT_OFF_CMOV_MOVING);
			out->edge    = *(unsigned char*)(cm + PT_OFF_CMOV_EDGE);
			out->ctr     = *(int*)(cm + PT_OFF_CMOV_EDGE_CTR);
			uintptr_t hc = *(uintptr_t*)(cm + PT_OFF_CMOV_HC);
			if (hc)
			{
				out->hc136 = *(int*)(hc + PT_OFF_HC_ARRIVAL);
				out->ps    = *(int*)(hc + PT_OFF_HC_PATH_STATE);
				out->reached = (out->hc136 == 1 && !out->moving && !out->edge) ? 1 : 0;
			}
			else
			{
				out->reached = 1;   // 0x65E320 returns true when there is no HavokCharacter
			}
		}

		uintptr_t ai = *(uintptr_t*)(character + PT_OFF_CHAR_AI);
		if (ai)
		{
			out->en      = *(int*)(ai + PT_OFF_AI_NUM_ENEMIES);
			out->thr     = *(int*)(ai + PT_OFF_AI_THREATS);
			out->nearSq  = *(float*)(ai + PT_OFF_AI_NEAREST_SQ);
			out->tp      = *(float*)(ai + PT_OFF_AI_THREAT_PERS);
			out->haveAiF = 1;

			uintptr_t ts = *(uintptr_t*)(ai + PT_OFF_AI_TASKSYS);
			if (ts)
			{
				uintptr_t goalRec = *(uintptr_t*)(ts + PT_OFF_TS_GOAL);
				PT_TASKER_TYPE(goalRec, out->goal);
				out->lvl     = *(int*)(ts + PT_OFF_TS_GOAL_LEVEL);
				out->fin     = *(unsigned char*)(ts + PT_OFF_TS_FINISHED);
				out->rethink = *(unsigned char*)(ts + PT_OFF_TS_RETHINK);
				out->dq      = *(long long*)(ts + PT_OFF_TS_DEQUE_SIZE);
				int n = *(int*)(ts + PT_OFF_TS_ORDER_COUNT);
				if (n >= 0 && n < PT_ORDER_COUNT_LIMIT)
				{
					out->ordN = n;
					out->ordHead = -1;
					if (n > 0)
					{
						uintptr_t arr = *(uintptr_t*)(ts + PT_OFF_TS_ORDER_ARRAY);
						if (arr)
						{
							uintptr_t head = *(uintptr_t*)arr;
							PT_TASKER_TYPE(head, out->ordHead);
						}
					}
				}
			}

			uintptr_t platoon = *(uintptr_t*)(ai + PT_OFF_AI_PLATOON);
			if (platoon)
			{
				uintptr_t bb = *(uintptr_t*)(platoon + PT_OFF_PLATOON_BB);
				if (bb)
					out->bbReq = *(long long*)(bb + PT_OFF_BB_REQ_SIZE);
			}
		}
	}
	__except (EXCEPTION_EXECUTE_HANDLER)
	{
		out->fault = 1;
	}
	GuardLeave();
}
#undef PT_TASKER_TYPE

// Per-character "last printed state" for the change detection: a small
// fixed array parallel to trackedPlayers[] (same index), keyed by the
// character pointer it was recorded for, so a slot handed to another
// character reads as "never printed". PollPlayerMovementState forgets an
// entry whenever it drops the tracked character (not in the player squad any
// more -- which is also what happens to every tracked character at a save
// load -- or arrived).
struct PlayerTaskLast
{
	uintptr_t character;   // 0 = nothing printed for this slot
	int       fault, t, stopped, edge, hc136;
};
static PlayerTaskLast g_playerTaskLast[MAX_TRACKED_PLAYERS];

static void PlayerTaskForget(int slot)
{
	if (slot >= 0 && slot < MAX_TRACKED_PLAYERS)
		g_playerTaskLast[slot].character = 0;
}

// true when (fault, t, stopped, edge, hc136) differs from the last line
// printed for this slot's character (or nothing was printed yet).
static bool PlayerTaskChanged(int slot, uintptr_t character, const PlayerTaskSnap& s)
{
	const PlayerTaskLast& l = g_playerTaskLast[slot];
	if (l.character != character) return true;
	return l.fault != s.fault || l.t != s.t || l.stopped != s.stopped
	    || l.edge != s.edge || l.hc136 != s.hc136;
}

static void PtAppendInt(std::ostringstream& ss, int v)
{
	if (v == PT_NA) ss << "-";
	else            ss << v;
}

// Formats and logs one PLAYER TASK line (main thread) and remembers it as
// this slot's last printed state.
static void LogPlayerTask(int slot, uintptr_t character, const PlayerTaskSnap& s)
{
	std::ostringstream ss;
	ss << "[ZoneOpt] PLAYER TASK: char=@" << std::hex << (character & 0xFFFF) << std::dec;
	if (s.fault)
	{
		ss << " fault=1 t=- goal=-/- ord=-:- dq=- fin=- rethink=- stopped=- moving=-"
		   << " edge=-/- hc136=- ps=- reached=- en=- thr=- near=- tp=- hit=-"
		   << " cst=- atk=- bbReq=-";
	}
	else
	{
		ss << " t=";        PtAppendInt(ss, s.t);
		ss << " goal=";     PtAppendInt(ss, s.goal);
		ss << "/";          PtAppendInt(ss, s.lvl);
		ss << " ord=";      PtAppendInt(ss, s.ordN);
		ss << ":";          PtAppendInt(ss, s.ordHead);
		ss << " dq=";
		if (s.dq < 0) ss << "-";
		else          ss << s.dq;
		ss << " fin=";      PtAppendInt(ss, s.fin);
		ss << " rethink=";  PtAppendInt(ss, s.rethink);
		ss << " stopped=";  PtAppendInt(ss, s.stopped);
		ss << " moving=";   PtAppendInt(ss, s.moving);
		ss << " edge=";     PtAppendInt(ss, s.edge);
		ss << "/";          PtAppendInt(ss, s.ctr);
		ss << " hc136=";    PtAppendInt(ss, s.hc136);
		ss << " ps=";       PtAppendInt(ss, s.ps);
		ss << " reached=";  PtAppendInt(ss, s.reached);
		ss << " en=";       PtAppendInt(ss, s.en);
		ss << " thr=";      PtAppendInt(ss, s.thr);
		ss << std::fixed << std::setprecision(0);
		ss << " near=";
		// nearestEnemy is a running minimum of squared distances (0x858500);
		// with no enemy seen it holds a large reset value, printed as "inf".
		if (!s.haveAiF || !(s.nearSq >= 0.0f)) ss << "-";
		else if (s.nearSq > 1.0e12f)           ss << "inf";
		else                                   ss << sqrtf(s.nearSq);
		ss << std::setprecision(1);
		ss << " tp=";
		if (s.haveAiF) ss << s.tp;
		else           ss << "-";
		ss << " hit=";      PtAppendInt(ss, s.hit);
		ss << " cst=";      PtAppendInt(ss, s.cst);
		ss << " atk=";      PtAppendInt(ss, s.atk);
		ss << " bbReq=";
		if (s.bbReq < 0) ss << "-";
		else             ss << s.bbReq;
	}
	LogMsg(ss.str());

	PlayerTaskLast& l = g_playerTaskLast[slot];
	l.character = character;
	l.fault   = s.fault;
	l.t       = s.t;
	l.stopped = s.stopped;
	l.edge    = s.edge;
	l.hc136   = s.hc136;
}
#endif // ZONEOPT_DEBUG


void StorePlayerClickDest(uintptr_t character, const float* dest, double now)
{
	// Update existing entry or find empty slot
	int emptySlot = -1;
	for (int i = 0; i < trackedPlayerCount; ++i)
	{
		if (trackedPlayers[i].character == character)
		{
			// Fix round 1 (review Important #2): the match is by pointer
			// whether the entry is active or not, and PollPlayerMovementState
			// deactivates an entry on arrival (or squad removal). The match
			// branch used to leave such an entry inactive, so PLAYER STUCK /
			// PLAYER TASK went silent after a character's first arrival. A
			// re-click now resets the entry exactly like the add branch below
			// (dest, prevPos, clickTime, lastRetryTime, retryCount,
			// zeroVelocityPolls, active) plus, for a reactivated entry, the
			// PLAYER TASK last-printed state, so it behaves like a new one.
#ifdef ZONEOPT_DEBUG
			if (!trackedPlayers[i].active)
				PlayerTaskForget(i);
#endif
			trackedPlayers[i].destX = dest[0];
			trackedPlayers[i].destY = dest[1];
			trackedPlayers[i].destZ = dest[2];
			trackedPlayers[i].prevPosX = 0;
			trackedPlayers[i].prevPosZ = 0;
			trackedPlayers[i].clickTime = now;
			trackedPlayers[i].lastRetryTime = 0.0;
			trackedPlayers[i].retryCount = 0;
			trackedPlayers[i].zeroVelocityPolls = 0;
			trackedPlayers[i].active = true;
			return;
		}
		if (!trackedPlayers[i].active && emptySlot < 0)
			emptySlot = i;
	}

	// Add new entry
	int slot = emptySlot;
	if (slot < 0)
	{
		if (trackedPlayerCount >= MAX_TRACKED_PLAYERS)
			return;
		slot = trackedPlayerCount++;
	}

#ifdef ZONEOPT_DEBUG
	PlayerTaskForget(slot);   // a new entry has printed nothing yet
#endif
	trackedPlayers[slot].character = character;
	trackedPlayers[slot].destX = dest[0];
	trackedPlayers[slot].destY = dest[1];
	trackedPlayers[slot].destZ = dest[2];
	trackedPlayers[slot].prevPosX = 0;
	trackedPlayers[slot].prevPosZ = 0;
	trackedPlayers[slot].clickTime = now;
	trackedPlayers[slot].lastRetryTime = 0.0;
	trackedPlayers[slot].retryCount = 0;
	trackedPlayers[slot].zeroVelocityPolls = 0;
	trackedPlayers[slot].active = true;
}


void PollPlayerMovementState(double now)
{
	if (now - lastStuckPollTime < 1.0)
		return;
	lastStuckPollTime = now;

	if (trackedPlayerCount == 0)
		return;

	// Validate tracked characters still exist in player squad
	uintptr_t playerIntf = *(uintptr_t*)(gameBase + RVA_GLOBAL_PLAYER);
	if (!playerIntf)
		return;

	unsigned int scCount = GetPlayerCharCount(playerIntf);
	uintptr_t* scStuff = GetPlayerCharStuff(playerIntf);
	if (!scStuff || scCount == 0 || scCount > 256)
		return;

	for (int i = 0; i < trackedPlayerCount; ++i)
	{
		TrackedPlayerDest& tp = trackedPlayers[i];
		if (!tp.active)
			continue;

		// Verify character still in squad
		bool found = false;
		for (unsigned int j = 0; j < scCount; ++j)
		{
			if (scStuff[j] == tp.character)
			{ found = true; break; }
		}
		if (!found)
		{
			tp.active = false;
#ifdef ZONEOPT_DEBUG
			PlayerTaskForget(i);
#endif
			continue;
		}

#ifdef ZONEOPT_DEBUG
		// PLAYER TASK (DEV): one guarded read per poll, printed on a change of
		// (t, stopped, edge, hc136) here, or next to the PLAYER STUCK line below.
		PlayerTaskSnap taskSnap;
		ReadPlayerTaskSnap(tp.character, &taskSnap);
		bool taskPrinted = false;
		if (PlayerTaskChanged(i, tp.character, taskSnap))
		{
			LogPlayerTask(i, tp.character, taskSnap);
			taskPrinted = true;
		}
#endif

		uintptr_t charMov = *(uintptr_t*)(tp.character + OFF_CHAR_MOVEMENT);
		if (!charMov)
			continue;

		uintptr_t vt = *(uintptr_t*)charMov;
		if (!vt)
			continue;

		// Read position
		const float* pos = ((GetPosFn)(*(uintptr_t*)(vt + VT_GET_POSITION)))((void*)charMov);
		if (!pos)
			continue;

		float posX = pos[0];
		float posZ = pos[2];

		// Velocity check: compare with previous poll position
		float dx = posX - tp.prevPosX;
		float dz = posZ - tp.prevPosZ;
		bool moving = (dx * dx + dz * dz) > 1.0f;

		tp.prevPosX = posX;
		tp.prevPosZ = posZ;

		if (moving)
		{
			tp.zeroVelocityPolls = 0;
			tp.retryCount = 0;  // moving again, reset retries
			continue;
		}

		tp.zeroVelocityPolls++;

		// Need two consecutive zero-velocity polls (~2s) before checking
		if (tp.zeroVelocityPolls < 2)
			continue;

		// Max retries exhausted
		if (tp.retryCount >= MAX_STUCK_RETRIES)
			continue;

		// Retry cooldown
		if (tp.lastRetryTime > 0.0 && now - tp.lastRetryTime < STUCK_RETRY_COOLDOWN)
			continue;

		// Read movement state
		bool pathOk      = ((BoolMethodFn)(*(uintptr_t*)(vt + VT_PATH_OK)))((void*)charMov);
		bool destReached  = ((BoolMethodFn)(*(uintptr_t*)(vt + VT_IS_DEST_REACHED)))((void*)charMov);

		AbstractMovementBase* amb = (AbstractMovementBase*)charMov;
		Ogre::Vector3 dest = amb->getDestination();

		// Deadlock check: dest collapsed to pos, pathOk (trivially), not arrived
		float ddx = dest.x - posX;
		float ddz = dest.z - posZ;
		bool destIsPos = (ddx * ddx + ddz * ddz) < 100.0f;  // < 10 units

		// Log state for diagnostics regardless of whether we recover
		{
			bool pathFailed = ((BoolMethodFn)(*(uintptr_t*)(vt + VT_PATH_FAILED)))((void*)charMov);
			std::ostringstream ss;
			ss << std::fixed << std::setprecision(0);
			ss << "[ZoneOpt] PLAYER STUCK: "
			   << tp.zeroVelocityPolls << " polls"
			   << " pos=(" << posX << "," << posZ << ")"
			   << " dest=(" << dest.x << "," << dest.z << ")"
			   << " pathOk=" << (pathOk ? 1 : 0)
			   << " pathFail=" << (pathFailed ? 1 : 0)
			   << " destReach=" << (destReached ? 1 : 0)
			   << " destIsPos=" << (destIsPos ? 1 : 0)
			   << " retryEnabled=" << (stuckRetryEnabled ? 1 : 0);
#if ISLAND_STEP >= 1
			// Island routing view of the park (see islands.h / island_routing.md):
			//   wp   = pathDestination, edge = movingToEdge/edgeCounter
			//   self = liveComp(char zone), xd = |pos - raw emulated crossing|
			//   next = zone beyond the crossing: c=liveComp l=label ld=+176 a=+177
			IslandStuckInfo isi;
			if (g_cachedZoneMgr
			    && IslandDescribeStuck(g_cachedZoneMgr, charMov, posX, posZ,
			                           tp.destX, tp.destZ, &isi))
			{
				ss << " wp=(" << isi.wpX << "," << isi.wpZ << ")"
				   << " edge=" << isi.movingToEdge << "/" << isi.edgeCounter
				   << " self=" << isi.selfComp;
				if (isi.haveCrossing)
					ss << " xd=" << isi.xd;
				else
					ss << " xd=none";
				ss << " next=(" << isi.nextGX << "," << isi.nextGY << ")"
				   << " c=" << isi.nextComp
				   << " l=" << isi.nextLabel
				   << " ld=" << isi.nextLoading
				   << " a=" << isi.nextAccess;
			}
#endif
			LogMsg(ss.str());
		}
#ifdef ZONEOPT_DEBUG
		// Every PLAYER STUCK line gets a PLAYER TASK line (unless this poll's
		// change already printed one, just above it).
		if (!taskPrinted)
			LogPlayerTask(i, tp.character, taskSnap);
#endif

		if (destReached)
			continue;

		// Confirm stored dest is actually far away (not already there)
		float sdx = tp.destX - posX;
		float sdz = tp.destZ - posZ;
		if (sdx * sdx + sdz * sdz < 10000.0f)  // < 100 units = already close
		{
			tp.active = false;  // arrived close enough, stop tracking
#ifdef ZONEOPT_DEBUG
			PlayerTaskForget(i);
#endif
			continue;
		}

		// Keep diagnostics active without issuing replacement movement orders.
		if (!stuckRetryEnabled)
			continue;

		// Optional recovery: move toward zone exit at decreasing percentages.
		uintptr_t charVtable = *(uintptr_t*)tp.character;
		if (!charVtable)
			continue;

		moveOrderFn_t fn_moveOrder = (moveOrderFn_t)(*(uintptr_t*)(charVtable + 0x318));
		if (!fn_moveOrder)
			continue;

		if (!gridCalibrated)
			continue;

		// Find zone boundary exit point: ray from pos toward dest, intersect zone AABB.
		int gx, gy;
		if (!WorldToZoneGrid(posX, posZ, &gx, &gy))
			continue;

		float zoneMinX = zoneOriginX + gx * zoneStepX;
		float zoneMaxX = zoneMinX + zoneStepX;
		float zoneMinZ = zoneOriginZ + gy * zoneStepZ;
		float zoneMaxZ = zoneMinZ + zoneStepZ;

		float dirX = tp.destX - posX;
		float dirZ = tp.destZ - posZ;

		// Ray-AABB: find parameter t where ray exits the zone
		float tExit = 1e30f;
		if (dirX > 0.001f)  { float t = (zoneMaxX - posX) / dirX; if (t < tExit) tExit = t; }
		if (dirX < -0.001f) { float t = (zoneMinX - posX) / dirX; if (t < tExit) tExit = t; }
		if (dirZ > 0.001f)  { float t = (zoneMaxZ - posZ) / dirZ; if (t < tExit) tExit = t; }
		if (dirZ < -0.001f) { float t = (zoneMinZ - posZ) / dirZ; if (t < tExit) tExit = t; }

		if (tExit <= 0.0f || tExit > 1e20f)
			continue;  // dest is in same zone or ray math failed

		// Target a percentage of the way to the zone exit
		float pct = STUCK_PCT_START - (tp.retryCount * STUCK_PCT_STEP);
		if (pct < 0.1f) pct = 0.1f;

		float targetX = posX + dirX * tExit * pct;
		float targetZ = posZ + dirZ * tExit * pct;
		float destVec[3] = { targetX, pos[1], targetZ };

		fn_moveOrder(tp.character, NULL, NULL, destVec);
		// Fix round 1 (review Minor #4): an island re-issue check pending for
		// this character now carries retry=1 (no-op below ISLAND_STEP 3).
		IslandFlagReissueOvertaken(tp.character, ISLAND_OVERTAKEN_RETRY);
		tp.retryCount++;

		tp.lastRetryTime = now;
		tp.zeroVelocityPolls = 0;

		{
			std::ostringstream ss;
			ss << std::fixed << std::setprecision(0);
			ss << "[ZoneOpt] STUCK RECOVERY: retry=" << tp.retryCount
			   << " pct=" << (int)(pct * 100) << "%"
			   << " pos=(" << posX << "," << posZ << ")"
			   << " target=(" << targetX << "," << targetZ << ")"
			   << " dest=(" << tp.destX << "," << tp.destZ << ")"
			   << " retryEnabled=" << (stuckRetryEnabled ? 1 : 0)
			   << " char=" << (void*)tp.character;
			LogMsg(ss.str());
		}
	}
}

#endif // PATHFIND_STEP >= 2


#endif // PATHFIND_STEP >= 1
