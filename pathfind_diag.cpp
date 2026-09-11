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


void StorePlayerClickDest(uintptr_t character, const float* dest, double now)
{
	// Update existing entry or find empty slot
	int emptySlot = -1;
	for (int i = 0; i < trackedPlayerCount; ++i)
	{
		if (trackedPlayers[i].character == character)
		{
			trackedPlayers[i].destX = dest[0];
			trackedPlayers[i].destY = dest[1];
			trackedPlayers[i].destZ = dest[2];
			trackedPlayers[i].clickTime = now;
			trackedPlayers[i].retryCount = 0;
			trackedPlayers[i].zeroVelocityPolls = 0;
			trackedPlayers[i].lastRetryTime = 0.0;
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
			continue;
		}

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

		if (destReached)
			continue;

		// Confirm stored dest is actually far away (not already there)
		float sdx = tp.destX - posX;
		float sdz = tp.destZ - posZ;
		if (sdx * sdx + sdz * sdz < 10000.0f)  // < 100 units = already close
		{
			tp.active = false;  // arrived close enough, stop tracking
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
