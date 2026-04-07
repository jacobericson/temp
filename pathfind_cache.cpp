// pathfind_cache.cpp -- Squad path cache slot management + Phase 12/SPC reporters
// FULL builds only (excluded by ZONEOPT_ZONEONLY via config.h inline stubs).

#include "pathfind_cache.h"

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

	if (rpHits == 0 && subBoosts == 0 && fbTags == 0)
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
	   << " pendOrd=" << pendingOrderCount;
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
