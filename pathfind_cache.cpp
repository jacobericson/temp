// pathfind_cache.cpp -- Squad path cache slot management + Phase 12/SPC reporters
// FULL builds only (excluded by ZONEOPT_ZONEONLY via config.h inline stubs).

#include "pathfind_cache.h"
#if PATHFIND_STEP >= 5
#include "navmesh_sched.h"
#endif
#if PATHFIND_STEP >= 6
#include "pathfinding.h"
#endif

#if PATHFIND_STEP >= 1


// =========================================================================
// Phase 12 state + reporter (Step 2+)
// =========================================================================

#if PATHFIND_STEP >= 2

int squadBoostTier     = 0;
int squadBoostGroupIdx = -1;

volatile long p12DiagRequestPathHits  = 0;
volatile long p12DiagTier60           = 0;
volatile long p12DiagTier50           = 0;
volatile long p12DiagTier40           = 0;
volatile long p12DiagTier30           = 0;
volatile long p12DiagSubmitBoosts     = 0;
volatile long p12DiagFallbackTags     = 0;
volatile long p12DiagRepathLeaders    = 0;
volatile long p12DiagRepathInjections = 0;
volatile long p12DiagSCGuardSkips     = 0;

static double lastP12LogTime = 0.0;

void LogPhase12Stats(double now)
{
	if (!squadPathCacheEnabled)
		return;
	if (now - lastP12LogTime < 30.0)
		return;
	lastP12LogTime = now;

	long rpHits    = InterlockedCompareExchange(&p12DiagRequestPathHits, 0, 0);
	long tier60    = InterlockedCompareExchange(&p12DiagTier60, 0, 0);
	long tier50    = InterlockedCompareExchange(&p12DiagTier50, 0, 0);
	long tier40    = InterlockedCompareExchange(&p12DiagTier40, 0, 0);
	long tier30    = InterlockedCompareExchange(&p12DiagTier30, 0, 0);
	long subBoosts = InterlockedCompareExchange(&p12DiagSubmitBoosts, 0, 0);
	long fbTags    = InterlockedCompareExchange(&p12DiagFallbackTags, 0, 0);
	long rpLeaders = InterlockedCompareExchange(&p12DiagRepathLeaders, 0, 0);
	long rpInjects = InterlockedCompareExchange(&p12DiagRepathInjections, 0, 0);
	long scSkips   = InterlockedCompareExchange(&p12DiagSCGuardSkips, 0, 0);

#if PATHFIND_STEP >= 5
	long rpFlag  = InterlockedCompareExchange(&reprioFlagFires, 0, 0);
	long rpTimer = InterlockedCompareExchange(&reprioTimerFires, 0, 0);
	long rpOrder = InterlockedCompareExchange(&reprioOrderFires, 0, 0);
#endif

#if PATHFIND_STEP >= 6
	long efDec   = InterlockedCompareExchange(&exitFaceDecodes, 0, 0);
	long efSame  = InterlockedCompareExchange(&exitFaceDecodesUnchanged, 0, 0);
	long efFail  = InterlockedCompareExchange(&exitFaceFailures, 0, 0);
	long efErr   = InterlockedCompareExchange(&exitFaceDecodeErrors, 0, 0);
	long efRace  = InterlockedCompareExchange(&exitFaceSCRace, 0, 0);
	long rcmIns  = InterlockedCompareExchange(&reqCharMapInserts, 0, 0);
	long rcmHit  = InterlockedCompareExchange(&reqCharMapLookupHits, 0, 0);
	long rcmMiss = InterlockedCompareExchange(&reqCharMapLookupMiss, 0, 0);
	long rcmOver = InterlockedCompareExchange(&reqCharMapOverflows, 0, 0);
	long rcmPeak = InterlockedCompareExchange(&reqCharMapHighWater, 0, 0);
	long rcmPrune = InterlockedCompareExchange(&reqCharMapDirectPrune, 0, 0);
	long rcmSups  = InterlockedCompareExchange(&reqCharMapSuperseded, 0, 0);
	long npcSkip = InterlockedCompareExchange(&npcRequestsSkipped, 0, 0);
#endif

#if PATHFIND_STEP >= 7
	long fdHits = InterlockedCompareExchange(&formationDedupHits, 0, 0);
	long fdProp = InterlockedCompareExchange(&formationPropagations, 0, 0);
	long paEnq  = InterlockedCompareExchange(&preloadAheadEnqueued, 0, 0);
	long evicts = InterlockedCompareExchange(&watchedEvictions, 0, 0);
#endif

#if PATHFIND_STEP >= 9
	long ecr    = InterlockedCompareExchange(&extractionCrashRescue, 0, 0);
	long ssHold = InterlockedCompareExchange(&spcStabilityHold, 0, 0);
	long aiHook = InterlockedCompareExchange(&addInstanceHookCalls, 0, 0);
#endif

	if (rpHits == 0 && subBoosts == 0 && fbTags == 0
#if PATHFIND_STEP >= 6
	    && efDec == 0 && rcmIns == 0
#endif
#if PATHFIND_STEP >= 7
	    && fdHits == 0 && paEnq == 0 && evicts == 0
#endif
#if PATHFIND_STEP >= 9
	    && ecr == 0 && ssHold == 0 && aiHook == 0
#endif
	   )
		return;

	std::ostringstream ss;
	ss << "[ZoneOpt] Phase12:"
	   << " rpMatch=" << rpHits
	   << " t60=" << tier60 << "/t50=" << tier50 << "/t40=" << tier40 << "/t30=" << tier30
	   << " boosts=" << subBoosts
	   << " fbTags=" << fbTags
	   << " rpLeaders=" << rpLeaders
	   << " rpInjects=" << rpInjects
	   << " scSkips=" << scSkips
	   << " pendOrd=" << pendingOrderCount
#if PATHFIND_STEP >= 5
	   << " reprio=" << rpOrder << "ord/" << rpFlag << "flg/" << rpTimer << "tmr"
#endif
#if PATHFIND_STEP >= 6
	   << " exitFace=" << efDec << "dec/" << efSame << "same/"
	   << efFail << "noCross/" << efErr << "err/" << efRace << "race"
	   << " reqMap=" << rcmIns << "ins/" << rcmHit << "hit/" << rcmMiss << "miss"
	   << "/peak" << rcmPeak
	   << "/prune" << rcmPrune
	   << "/sup" << rcmSups
	   << " npcSkip=" << npcSkip
#endif
#if PATHFIND_STEP >= 7
	   << " fmtDedup=" << fdHits << "hits/" << fdProp << "prop"
	   << " preAhead=" << paEnq
	   << " evict=" << evicts
#endif
#if PATHFIND_STEP >= 9
	   << " extractAV=" << ecr
	   << " stabHold=" << ssHold
	   << " addInst=" << aiHook
#endif
	   ;
#if PATHFIND_STEP >= 6
	if (rcmOver > 0)
		ss << " OVF=" << rcmOver;
#endif
	LogMsg(ss.str());
}

#endif // PATHFIND_STEP >= 2


// =========================================================================
// Squad path cache state + slot management + reporter (Step 4+)
// =========================================================================

#if PATHFIND_STEP >= 4

SquadPathCacheSlot spcSlots[MAX_FORMATION_GROUPS];

float spcSlotCandOffX[MAX_FORMATION_GROUPS];
float spcSlotCandOffZ[MAX_FORMATION_GROUPS];

const float  SPC_GOAL_TOL      = 5.0f;
const double SPC_TIMEOUT       = 10.0;
const int    SPC_BUDGET_MULT   = 4;
const double SPC_BURST_TIMEOUT = 2.0;
const double SPC_REPATH_TIMEOUT = 90.0;

unsigned int spcLastConnStartFace = 0;
unsigned int spcLastConnDestFace  = 0;

int    spcFallbackTag        = 0;
int    spcCachedSCSize       = 0;
double spcLastInjectionTime  = 0.0;
int    spcBurstInjected      = 0;

volatile long spcDiagSignals    = 0;
volatile long spcDiagLeaderOK   = 0;
volatile long spcDiagLeaderFail = 0;
volatile long spcDiagInjected   = 0;
volatile long spcDiagExpired    = 0;
volatile long spcDiagBoosted    = 0;
volatile long spcDiagActiveSlots = 0;

static double lastSpcLogTime = 0.0;


void SpcFreeSlot(int s)
{
	if (spcSlots[s].edgeData)
	{
		fn_gameDelArr(spcSlots[s].edgeData);
		spcSlots[s].edgeData = NULL;
	}
	spcSlots[s].edgeCount = 0;
}

void SpcResetSlot(int s)
{
	SpcFreeSlot(s);
	spcSlots[s].state = 0;
	spcSlots[s].servedCount = 0;
	spcSlots[s].goalFaceKey = 0;
#if PATHFIND_STEP >= 7
	spcSlots[s].formationExitGX         = -1;
	spcSlots[s].formationExitGY         = -1;
	spcSlots[s].formationExitUpdateTime = 0.0;
#endif
}

void SpcResetAll()
{
	for (int s = 0; s < MAX_FORMATION_GROUPS; ++s)
		SpcResetSlot(s);
}


void LogSquadPathCacheStats(double now)
{
	if (!squadPathCacheEnabled)
		return;

	if (now - lastSpcLogTime < 30.0)
		return;
	lastSpcLogTime = now;

	long signals   = InterlockedCompareExchange(&spcDiagSignals, 0, 0);
	long leaderOK  = InterlockedCompareExchange(&spcDiagLeaderOK, 0, 0);
	long leaderFail = InterlockedCompareExchange(&spcDiagLeaderFail, 0, 0);
	long injected  = InterlockedCompareExchange(&spcDiagInjected, 0, 0);
	long expired   = InterlockedCompareExchange(&spcDiagExpired, 0, 0);
	long boosted   = InterlockedCompareExchange(&spcDiagBoosted, 0, 0);

	if (signals == 0 && injected == 0)
		return;

	std::ostringstream ss;
	int activeSlots = 0;
	for (int s = 0; s < MAX_FORMATION_GROUPS; ++s)
		if (spcSlots[s].state > 0) activeSlots++;

	ss << "[ZoneOpt] SquadPathCache:"
	   << " signals=" << signals
	   << " leaderOK=" << leaderOK << "/" << leaderFail << "fail"
	   << " injected=" << injected
	   << " expired=" << expired
	   << " boosted=" << boosted
	   << " slots=" << activeSlots;

	for (int s = 0; s < MAX_FORMATION_GROUPS; ++s)
	{
		if (spcSlots[s].state > 0)
			ss << " [" << s << "]=" << spcSlots[s].state
			   << "/fk" << (spcSlots[s].goalFaceKey >> 22);
	}

	LogMsg(ss.str());
}

#endif // PATHFIND_STEP >= 4


#endif // PATHFIND_STEP >= 1
