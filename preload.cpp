#include "preload.h"
#include "transition.h"
#include "nm_workers.h"
#include "tracking.h"
#include "formation.h"
#include "islands.h"
#include "hooks.h"
#include "navmesh_sched.h"
#include <cstdio>     // _snprintf_s (hook_resetUnloadZones builds its line without CRT streams)


PreloadedZone preloadedZones[MAX_PRELOADED];
int numPreloaded     = 0;
int pendingCount     = 0;
int predictedCenterX = -1;
int predictedCenterY = -1;

QueuedZone cameraQueue[CAMERA_RESERVED];
int cameraQueueCount = 0;
int cameraQueueNext  = 0;

QueuedZone charQueue[MAX_PRELOADED];
int charQueueCount = 0;
int charQueueNext  = 0;

double lastCharScanTime = 0.0;
int charZonesQueued     = 0;
double lastCamLogTime   = 0.0;
int promotedCount       = 0;
int preloadSkipLoaded   = 0;
int lastCameraGX         = -1;
int lastCameraGY         = -1;
int regSkipCount         = 0;


// =========================================================================
// Registry guard (Round 2 fix 2b, defence in depth)
// =========================================================================
//
// Every handle issued for a zone's objects resolves through the zone's slot in
// the ZoneMap handle registry (game.h, RVA_ZONEMAP_HANDLE_*): slot
// gx + 64*gy + 1 must hold this content's HandleDummy. The game's save-load
// reset wipes the registry after unloading only Set A and Set B, so a zone the
// mod loaded before a load could keep a content whose slot is gone, and every
// object the game then instantiates in it resolves to NULL (the Round 2
// session 3 crash in Building::createPhysical). hook_resetUnloadZones below
// removes those zones at the reset; this check is the backstop on every path
// that adopts a zone or hands its content to the game, and regSkip= says
// whether it ever had to act.
//
// Main thread only (every caller is). Plain reads; the slot array only grows
// or is rewritten by main-thread code (content ctor, prepareUnload, the reset).

enum RegGuardSite
{
	REG_SITE_ADOPT = 0,
	REG_SITE_REGISTER,
	REG_SITE_PROCESS,
	REG_SITE_PROMOTE,
	REG_SITE_COUNT
};

static const char* const kRegSiteName[REG_SITE_COUNT] =
	{ "adopt", "register", "process", "promote" };

// One PROD line per zone per site until the next load: the adopt site in
// particular would otherwise log every time the preload queue revisits the
// same orphan. regSkipCount still counts every refusal.
static unsigned char g_regGuardLogged[REG_SITE_COUNT][ZONE_GRID_COUNT];

// True when the zone's content is non-NULL, its registry index is inside the
// slot array, and the slot holds this content's HandleDummy. *slotOut and
// *dummyOut receive what was read (NULL where the walk stopped first).
static bool ZoneRegistrationOk(void* zoneEntry, void** slotOut, void** dummyOut)
{
	*slotOut  = NULL;
	*dummyOut = NULL;
	if (!zoneEntry)
		return false;

	void* content = *(void**)((uintptr_t)zoneEntry + OFF_ZONE_CONTENT);
	if (!content)
		return false;

	void* dummy = *(void**)((uintptr_t)content + OFF_ZMC_HANDLE_DUMMY);
	*dummyOut = dummy;
	if (!dummy)
		return false;

	// Same fields and order as the content ctor's container id (0xA008C8):
	// ZoneMap+24 + (ZoneMap+28 << 6) + 1.
	int gx = GetZoneGridX(zoneEntry);
	int gy = GetZoneGridY(zoneEntry);
	if (gx < 0 || gx > ZONE_GRID_MAX || gy < 0 || gy > ZONE_GRID_MAX)
		return false;
	unsigned int idx = (unsigned int)(gx + 64 * gy + 1);

	void** slots = *(void***)(gameBase + RVA_ZONEMAP_HANDLE_SLOTS);
	unsigned int count = *(unsigned int*)(gameBase + RVA_ZONEMAP_HANDLE_COUNT);
	if (!slots || idx >= count)
		return false;

	void* slot = slots[idx];
	*slotOut = slot;
	return slot == dummy;
}

// Returns true when the zone may be handed to the game. On a refusal it counts
// and logs; the caller does not call into the game for this zone and drops it
// from the mod's tracking.
static bool RegistryGuardPasses(void* zoneEntry, int site)
{
	void* slot  = NULL;
	void* dummy = NULL;
	if (ZoneRegistrationOk(zoneEntry, &slot, &dummy))
		return true;

	regSkipCount++;

	int gx = zoneEntry ? GetZoneGridX(zoneEntry) : -1;
	int gy = zoneEntry ? GetZoneGridY(zoneEntry) : -1;
	int cell = (gx >= 0 && gx <= ZONE_GRID_MAX && gy >= 0 && gy <= ZONE_GRID_MAX)
	           ? gy * (ZONE_GRID_MAX + 1) + gx : -1;
	if (cell < 0 || !g_regGuardLogged[site][cell])
	{
		if (cell >= 0)
			g_regGuardLogged[site][cell] = 1;
		std::ostringstream ss;
		ss << "[ZoneOpt] Registry guard: zone (" << gx << "," << gy << ") regOk=0 at "
		   << kRegSiteName[site] << " slot=" << slot << " dummy=" << dummy
		   << " \xE2\x80\x94 skipped";
		LogMsg(ss.str());
	}
	return false;
}

// Drop a tracked zone after a guard refusal. The slot is left in place with a
// NULL zoneEntry and no stage flags, so the order[] indices the caller is
// iterating stay valid; EvictStaleZones compacts NULL entries on its next run.
// Coordinates go to -1 so IsZoneQueued no longer matches it.
static void DropTrackedZone(int i)
{
	if (preloadedZones[i].pending && pendingCount > 0)
		pendingCount--;
	preloadedZones[i].zoneEntry        = NULL;
	preloadedZones[i].gridX            = -1;
	preloadedZones[i].gridY            = -1;
	preloadedZones[i].promoted         = false;
	preloadedZones[i].pending          = false;
	preloadedZones[i].registered       = false;
	preloadedZones[i].registeredEmpty  = false;
	preloadedZones[i].pipelineHandoff  = false;
	preloadedZones[i].contentProcessed = false;
}


#if PRELOAD_STEP >= 1
// =========================================================================
// Phase 18 Step 1 instrumentation (research/preload_pipeline.md §5 Step 1)
// =========================================================================
// Per-window sample sets for the 5 latency series in the "PreloadPipe:" line.
// Fixed capacity, main-thread only, no STL: a window prints and resets every
// PIPE_STATS_INTERVAL seconds. Bounded by preload throughput (~1 zone/2s per
// the design doc), so 64 is generous headroom for a 10-30s window; a window
// that saturates it just caps the sample (percentiles then read low -- this
// is diagnostic-only, not a release gate).
static const int PIPE_SAMPLE_CAP = 64;

struct PipeSampleSet
{
	double toReadyMs[PIPE_SAMPLE_CAP];     int toReadyN;
	double to264Ms[PIPE_SAMPLE_CAP];       int to264N;
	double toRegMs[PIPE_SAMPLE_CAP];       int toRegN;
	double toPromoMs[PIPE_SAMPLE_CAP];     int toPromoN;
	double procContentMs[PIPE_SAMPLE_CAP]; int procContentN;
	int loaded;
	int registered;
	int promoted;
	int readyEmpty;
	int notReadyAtCall;
	int thingsAfter0;
	int thingsAfter1;
};
static PipeSampleSet g_pipe;
static double g_pipeWindowStart = 0.0;

#ifdef ZONEOPT_DEBUG
static const double PIPE_STATS_INTERVAL = 10.0;  // DEV
#else
static const double PIPE_STATS_INTERVAL = 30.0;  // PROD
#endif

static void PipeSampleAdd(double* arr, int* n, double ms)
{
	if (*n < PIPE_SAMPLE_CAP)
		arr[(*n)++] = ms;
}

static void PipeSortAscending(double* arr, int n)
{
	for (int i = 1; i < n; ++i)
	{
		double v = arr[i];
		int j = i - 1;
		while (j >= 0 && arr[j] > v) { arr[j + 1] = arr[j]; --j; }
		arr[j + 1] = v;
	}
}

// Floor-rank percentiles (matches the pairing-rule note in
// research/preload_pipeline.md §1): the value at sorted rank floor(frac*n).
static void PipeAppendPercentiles(std::ostringstream& ss, double* arr, int n)
{
	if (n <= 0) { ss << "-/-/-"; return; }
	double sorted[PIPE_SAMPLE_CAP];
	for (int i = 0; i < n; ++i) sorted[i] = arr[i];
	PipeSortAscending(sorted, n);
	int p50i = (int)(0.5 * n); if (p50i >= n) p50i = n - 1;
	int p90i = (int)(0.9 * n); if (p90i >= n) p90i = n - 1;
	ss << std::fixed << std::setprecision(1)
	   << sorted[p50i] << "/" << sorted[p90i] << "/" << sorted[n - 1];
}

static void PrintPipeStats()
{
	// Round 2 final review cheap minor: skip the line entirely when every
	// counter in the window is zero (preload idle this window) -- a
	// PIPE_STATS_INTERVAL-cadence line of all zeros/dashes was pure log
	// volume with nothing to grep.
	if (g_pipe.loaded == 0 && g_pipe.registered == 0 && g_pipe.promoted == 0 &&
	    g_pipe.readyEmpty == 0 && g_pipe.notReadyAtCall == 0 &&
	    g_pipe.thingsAfter0 == 0 && g_pipe.thingsAfter1 == 0 &&
	    g_pipe.toReadyN == 0 && g_pipe.to264N == 0 && g_pipe.toRegN == 0 &&
	    g_pipe.toPromoN == 0 && g_pipe.procContentN == 0)
		return;

	std::ostringstream ss;
	ss << "[ZoneOpt] PreloadPipe: loaded=" << g_pipe.loaded
	   << " registered=" << g_pipe.registered
	   << " promoted=" << g_pipe.promoted
	   << " toReady "; PipeAppendPercentiles(ss, g_pipe.toReadyMs, g_pipe.toReadyN);
	ss << " ms to264 "; PipeAppendPercentiles(ss, g_pipe.to264Ms, g_pipe.to264N);
	ss << " ms toReg "; PipeAppendPercentiles(ss, g_pipe.toRegMs, g_pipe.toRegN);
	ss << " ms toPromo "; PipeAppendPercentiles(ss, g_pipe.toPromoMs, g_pipe.toPromoN);
	ss << " ms procContent " << g_pipe.procContentN << " ";
	PipeAppendPercentiles(ss, g_pipe.procContentMs, g_pipe.procContentN);
	ss << " ms readyEmpty=" << g_pipe.readyEmpty
	   << " notReadyAtCall=" << g_pipe.notReadyAtCall
	   << " thingsAfter0=" << g_pipe.thingsAfter0
	   << " thingsAfter1=" << g_pipe.thingsAfter1;
	LogMsg(ss.str());
}


// =========================================================================
// H15: per-transition benchmark (screen / navReady / firstMove)
// =========================================================================
// docs/release_readiness.md H15; research/preload_pipeline.md §5 Step 1.
// preload.cpp detects the loading-screen dismissal (isTransitionActive going
// false between frames, transition.h) and polls navReady; tracking.cpp's
// PollActiveMovers reports firstMove back via H15ReportFirstMove. gen
// disambiguates a report from a benchmark that has already closed.
static bool   h15Active              = false;
static int    h15Gen                 = 0;
static double h15DismissTime         = 0.0;
static double h15ScreenMs            = -1.0;
static double h15NavReadyMs          = -1.0;
static double h15FirstMoveMs         = -1.0;
static bool   h15NavReadyDone        = false;
static bool   h15FirstMoveDone       = false;
static bool   h15WasTransitionActive = false;

static void H15EmitLog()
{
	std::ostringstream ss;
	ss << "[ZoneOpt] H15: screen=" << std::fixed << std::setprecision(1) << h15ScreenMs << "ms"
	   << " navReady=";
	if (h15NavReadyMs >= 0.0) ss << std::fixed << std::setprecision(1) << h15NavReadyMs << "ms";
	else ss << "-";
	ss << " firstMove=";
	if (h15FirstMoveMs >= 0.0) ss << std::fixed << std::setprecision(1) << h15FirstMoveMs << "ms";
	else ss << "-";
	LogMsg(ss.str());
	h15Active = false;
}

static void H15MaybeEmit()
{
	if (h15Active && h15NavReadyDone && h15FirstMoveDone)
		H15EmitLog();
}

static void H15OnDismiss(double screenMs, double now)
{
	if (h15Active)
	{
		// A new transition dismissed before the previous benchmark finished
		// (a travelling squad can cross zones faster than the 60s cap). Log
		// what is known now -- "-" for the rest -- instead of dropping it.
		H15EmitLog();
	}
	h15Gen++;
	h15Active        = true;
	h15DismissTime   = now;
	h15ScreenMs      = screenMs;
	h15NavReadyMs    = -1.0;
	h15FirstMoveMs   = -1.0;
	h15NavReadyDone  = false;
	h15FirstMoveDone = false;
	H15ArmFirstMoveCheck(h15Gen, now);
}

void H15ReportFirstMove(int gen, double firstMoveMs)
{
	if (!h15Active || gen != h15Gen || h15FirstMoveDone)
		return;  // stale report: benchmark already closed or superseded
	h15FirstMoveMs   = firstMoveMs;
	h15FirstMoveDone = true;
	H15MaybeEmit();
}

static void H15PollNavReady(void* zoneMgr, double now)
{
	if (!h15Active)
		return;

	if (!h15NavReadyDone)
	{
		if (now - h15DismissTime > 60.0)
		{
			h15NavReadyMs   = -1.0;
			h15NavReadyDone = true;
		}
		else if (zoneMgr && orig_isContentPending)
		{
			uintptr_t sectionMgr = *(uintptr_t*)(gameBase + RVA_GLOBAL_SECTION_MGR);
			void* targetZone = *(void**)((uintptr_t)zoneMgr + OFF_ZM_CURRENT_ZONE);
			if (sectionMgr && targetZone)
			{
				int gridCoords[2] = { GetZoneGridX(targetZone), GetZoneGridY(targetZone) };
				if (orig_isContentPending((void*)sectionMgr, (void*)gridCoords))
				{
					h15NavReadyMs   = (now - h15DismissTime) * 1000.0;
					h15NavReadyDone = true;
				}
			}
		}
	}

	// firstMove has its own 60s cap here too: if nothing is ever watched after
	// this dismissal, PollActiveMovers never runs (TieredCharacterPoll gates it
	// on numWatched > 0) and tracking.cpp would never report at all.
	if (!h15FirstMoveDone && (now - h15DismissTime) > 60.0)
	{
		h15FirstMoveMs   = -1.0;
		h15FirstMoveDone = true;
	}

	H15MaybeEmit();
}

static void H15Tick(void* zoneMgr, double now)
{
	bool active = isTransitionActive;
	if (h15WasTransitionActive && !active)
		H15OnDismiss(QPCToMs(transitionStartTime, transitionEndQpc), now);
	h15WasTransitionActive = active;

	H15PollNavReady(zoneMgr, now);
}

static void H15Reset()
{
	// Full/save-load reset: drop any in-flight benchmark rather than let a
	// stale one log against the wrong session, and resync the edge detector
	// to the flag's current value.
	h15Active        = false;
	h15NavReadyDone  = false;
	h15FirstMoveDone = false;
	h15WasTransitionActive = isTransitionActive;
}

// Called every main-thread frame from PreloadCheckSaveLoad (the only
// preload.cpp entry hooks.cpp calls unconditionally every frame), so the
// periodic print and the H15 dismissal edge detector do not need a new call
// site in hooks.cpp.
static void PreloadStep1Tick(void* zoneMgr)
{
	double now = ElapsedSec();
	if (now - g_pipeWindowStart >= PIPE_STATS_INTERVAL)
	{
		PrintPipeStats();
		memset(&g_pipe, 0, sizeof(g_pipe));
		g_pipeWindowStart = now;
	}
	H15Tick(zoneMgr, now);
}
#endif // PRELOAD_STEP >= 1


void ClearPreloadZones()
{
#if PRELOAD_STEP >= 1
	{
		// O5: what the scans queued gets thrown away here. Count by furthest
		// lifecycle stage reached (mutually exclusive) plus both queues.
		// stalled = tracked with no stage flag (stall-timed-out, never
		// registered), the same class and order the "Save load reset:" line
		// uses (fix 2b), so pending= means the same on both lines. A slot the
		// registry guard dropped (zoneEntry NULL, awaiting compaction) is not
		// a zone and is not counted.
		int dPending = 0, dStalled = 0, dRegistered = 0, dPromoted = 0, dHandoff = 0;
		for (int i = 0; i < numPreloaded; ++i)
		{
			if (!preloadedZones[i].zoneEntry)           continue;
			if (preloadedZones[i].pipelineHandoff)      dHandoff++;
			else if (preloadedZones[i].promoted)        dPromoted++;
			else if (preloadedZones[i].registered)      dRegistered++;
			else if (preloadedZones[i].pending)         dPending++;
			else                                        dStalled++;
		}
		int qCam  = cameraQueueCount - cameraQueueNext;
		int qChar = charQueueCount - charQueueNext;
		if (dPending || dStalled || dRegistered || dPromoted || dHandoff || qCam || qChar)
		{
			std::ostringstream ss;
			ss << "[ZoneOpt] Preload clear: dropped pending=" << dPending
			   << " stalled=" << dStalled
			   << " registered=" << dRegistered
			   << " promoted=" << dPromoted
			   << " handoff=" << dHandoff
			   << " queued=" << qCam << "/" << qChar;
			LogMsg(ss.str());
		}
	}
#endif

	for (int i = 0; i < MAX_PRELOADED; ++i)
	{
		preloadedZones[i].zoneEntry = NULL;
		preloadedZones[i].gridX = -1;
		preloadedZones[i].gridY = -1;
		preloadedZones[i].promoted = false;
		preloadedZones[i].pending = false;
		preloadedZones[i].registered = false;
		preloadedZones[i].registeredEmpty = false;
		preloadedZones[i].pipelineHandoff = false;
		preloadedZones[i].contentProcessed = false;
		preloadedZones[i].loadTimeSec = 0.0;
		preloadedZones[i].owner = OWNER_CHARACTER;
#if PRELOAD_STEP >= 1
		preloadedZones[i].pipeLoadSec = -1.0;
		preloadedZones[i].pipeIsReadySeen = false;
		preloadedZones[i].pipe264Seen = false;
#endif
	}
	numPreloaded = 0;
	pendingCount = 0;
	predictedCenterX = -1;
	predictedCenterY = -1;
	cameraQueueCount = 0;
	cameraQueueNext = 0;
	charQueueCount = 0;
	charQueueNext = 0;
	lastCharScanTime = 0.0;
	lastCameraGX = -1;
	lastCameraGY = -1;
	// Reset polling timers so scans fire immediately after transition end
	lastBaselineScan = 0.0;
	lastActivePoll = 0.0;
}

// Full mod-state reset. clearNavMeshCache=false keeps the L1 ring buffer:
// navmesh entries are keyed by zone geometry and building layout, so they stay
// valid across a save load of the same world, and dropping them would force
// every revisited tile back through an L2 read (31-46 ms) or a generation.
// Only a startup reset drops them.
static void ClearPreloadStateImpl(bool clearNavMeshCache)
{
	ClearPreloadZones();

	// Only on full reset, not transition end
	for (int i = 0; i < MAX_WATCHED; ++i)
	{
		watchedChars[i].character = 0;
		watchedChars[i].charMovement = 0;
#if PATHFIND_STEP >= 5
		watchedChars[i].hasMoveOrder = false;
#endif
		watchedChars[i].destZoneX = -1;
		watchedChars[i].destZoneY = -1;
		watchedChars[i].currentZoneX = -1;
		watchedChars[i].currentZoneY = -1;
		watchedChars[i].addedTime = 0.0;
#if PATHFIND_STEP >= 6
		watchedChars[i].exitZonePacked     = EXIT_ZONE_NONE;
		watchedChars[i].exitFaceUpdateTime = 0.0;
#endif
#if PATHFIND_STEP >= 7
		watchedChars[i].formationGroupId       = -1;
		watchedChars[i].preloadAheadGX         = PRELOAD_AHEAD_NONE;
		watchedChars[i].preloadAheadGY         = PRELOAD_AHEAD_NONE;
		watchedChars[i].preloadAheadUpdateTime = 0.0;
#endif
	}
	numWatched = 0;
	if (clearNavMeshCache)
		ClearNavMeshCache();
	ClearFormationGroups();
	IslandReset();
#if PRELOAD_STEP >= 1
	H15Reset();
#endif
	// Registry guard: a new session logs its first refusal per zone again.
	// regSkipCount itself stays cumulative (the Transition line reports it).
	memset(g_regGuardLogged, 0, sizeof(g_regGuardLogged));
}

void ClearPreloadState()
{
	ClearPreloadStateImpl(/*clearNavMeshCache=*/true);
}

void ClearPreloadStateForLoad()
{
	ClearPreloadStateImpl(/*clearNavMeshCache=*/false);
	// The world clear a load performs (sub_1407A82C0) frees every Ogre movable
	// and empties destroyListOE itself, so the deferred insert queue goes with
	// the rest of the state rather than being replayed onto freed objects.
	DestroyListDropDeferred();
}


// =========================================================================
// Save-load detection (B9) — main thread, independent of ISLAND_STEP
// =========================================================================
//
// SaveManager::loadGame sets the ZoneManager loading byte (ZM+8) and the game
// clears it with state 5. Everything the mod tracks (preloaded ZoneMap*,
// watched characters, formation groups, island components) names objects that
// the load destroys, so the whole state is dropped on the rising edge of that
// byte, and again if the ZoneManager pointer itself changes. While the byte is
// set, preload processing is skipped: nothing we could preload survives.
static uintptr_t g_saveLoadZm      = 0;
static bool      g_saveLoadWasSet  = false;

// Set by hook_resetUnloadZones when it has already cleared the mod's state at
// the game's reset (main thread, saveLoadUnload on); consumed by the next
// PreloadCheckSaveLoad. The reset runs inside SaveManager::loadGame, which is
// called from GameWorld__mainLoop_GPUSensitiveStuff (+0x241) before
// updateCameraZone (+0x385) in the same frame, and loadGame sets ZM+8 (0x374002)
// after the reset, so the edge below is normally the very next check.
static bool      g_resetStateCleared = false;

// The save-load state clear, shared by the reset hook and the ZM+8 edge. Every
// part is idempotent: ClearPreloadZones logs its "Preload clear:" line only
// when something was tracked, and a second run finds nothing tracked.
static void ClearModStateForLoad()
{
	ClearPreloadStateForLoad();  // also clears formation groups, the island overlay
	                             // and the deferred destroy-list queue
	promotedCount   = 0;
	charZonesQueued = 0;
}

bool PreloadCheckSaveLoad(void* zoneMgr)
{
	uintptr_t zm = (uintptr_t)zoneMgr;
	if (!zm)
		return false;

	bool loading = *(unsigned char*)(zm + OFF_ZM_LOADING) != 0;

	// Consumed on every check, edge or not: it only speaks for the reset that
	// immediately precedes this frame's check.
	bool resetCleared = g_resetStateCleared;
	g_resetStateCleared = false;

	if (zm != g_saveLoadZm || (loading && !g_saveLoadWasSet))
	{
		// Rising edge of ZM+8, or a brand-new ZoneManager: drop everything.
		// Keeps the navmesh caches: they are world-keyed and survive the load.
		// After a reset hook that already cleared, this is the backstop: it
		// runs again (harmless, see ClearModStateForLoad) and catches anything
		// queued since, e.g. off-main destroy-list inserts.
		ClearModStateForLoad();
		// The first bind of a session is a pointer change with the byte clear;
		// it gets its own line so it is not read as a mid-session load. A load
		// whose reset already cleared says so, so the two lines of one load are
		// not read as two loads.
		if (!loading)
			LogMsg("[ZoneOpt] Zone manager bound: preload state cleared");
		else if (resetCleared)
			LogMsg("[ZoneOpt] Save load detected: preload state already cleared at the reset "
			       "(backstop clear re-run)");
		else
			LogMsg("[ZoneOpt] Save load detected: preload state cleared");
		g_saveLoadZm = zm;
	}
	g_saveLoadWasSet = loading;

#if PRELOAD_STEP >= 1
	// PreloadCheckSaveLoad is the only preload.cpp function hooks.cpp calls on
	// every main-thread frame regardless of preload/zone state, so the Step 1
	// periodic print and the H15 dismissal edge detector are ticked from here
	// instead of a new call site in hooks.cpp (H2 owns hooks.cpp). Skipped
	// while a save load is in progress: the world (and the camera focus zone)
	// is being torn down or rebuilt.
	if (!loading)
		PreloadStep1Tick(zoneMgr);
#endif

	return loading;
}


// =========================================================================
// Save-load reset: unload the zones the reset leaves alive (Round 2 fix 2b)
// =========================================================================
//
// The game's "Reset game" step (sub_14036CA40) calls sub_14036C1E0 to unload
// every zone in Set A and Set B, runs the world clear, and then wipes the
// ZoneMap handle registry. The mod's zones are in neither set (it loads them
// with loadSingleZone directly), so before this hook they survived the reset
// with a live content and no registration, and the mod later adopted and
// processed one — the Round 2 session 3 crash. game.h has the addresses.
//
// After the original has unloaded its own zones, every ZoneMap that still has
// a content is outside both sets by construction (the original also empties
// Set A and erases each zone from Set B), so each one is unloaded here through
// the original's own per-zone call, with its arguments (zm, zone, 0).
//
// What is and is not intact at this point: the ZoneMap handle registry and the
// world's object set still are -- the world clear (sub_1407A82C0, 0x36CF2B) and
// the registry memset (0x36CF60) both come after this function returns. The
// reset has already run GameWorld__populateMapArea_nonPermanent (0x36CB6A),
// the sub_140786790 purge (0x36CB96), FactionManager__clearAndDestroy
// (0x36CBBA) and the sub_1408F89A0 section-manager pass (0x36CBD7). The game's
// own Set A/B unloads run in exactly that state, so the mod's zones are
// unloaded in the state vanilla unloads its own.
//
// NavMesh generation (review fix): unloadSingleZone frees zone+0xB8 (terrain
// collision) and the content. A MISS that already passed its post-wait zone
// re-check is inside processJobAlt, which reads *(zone+0xB8) (0x3CC99C), so
// the survivor loop holds processJobCS (bounded try, nm_workers.h): every MISS
// in flight finishes first, and every job claimed meanwhile fails its re-check
// after the wait. Taken lazily at the first survivor, so a reset with nothing
// to unload never waits. The deadlock audit is in r2-fix2b-report.md ("Fix
// round 1"): no thread ever waits for processJobCS while holding a lock, so no
// game lock the unload takes can close a cycle through it.
static const DWORD RESET_PJ_LOCK_TIMEOUT_MS = 10000;

// Releases processJobCS if something unwinds out of the survivor loop (the
// unload is game code with C++ unwind state); a processJobCS left held would
// park every NavMesh MISS for the rest of the session.
struct ResetPjLockGuard
{
	bool held;
	ResetPjLockGuard() : held(false) {}
	~ResetPjLockGuard() { Release(); }
	void Release()
	{
		if (!held) return;
		held = false;
		NavMeshUnlockProcessJob();
	}
private:
	ResetPjLockGuard(const ResetPjLockGuard&);
	ResetPjLockGuard& operator=(const ResetPjLockGuard&);
};

// Furthest lifecycle stage the mod's tracking records for a zone, in the same
// order and with the same meaning as ClearPreloadZones' "Preload clear:" line:
// handoff > promoted > registered > pending, and "stalled" for a tracked zone
// with none of those flags (a stall-timed-out, never-registered load, or one
// waiting for the zombie handler).
enum ResetClass
{
	RESET_PENDING = 0,
	RESET_STALLED,
	RESET_REGISTERED,
	RESET_PROMOTED,
	RESET_HANDOFF,
	RESET_UNTRACKED,
	RESET_CLASS_COUNT
};

static int ClassifyTrackedZone(int i)
{
	if (preloadedZones[i].pipelineHandoff) return RESET_HANDOFF;
	if (preloadedZones[i].promoted)        return RESET_PROMOTED;
	if (preloadedZones[i].registered)      return RESET_REGISTERED;
	if (preloadedZones[i].pending)         return RESET_PENDING;
	return RESET_STALLED;
}

// Main thread only.
static int ClassifyResetSurvivor(void* zoneEntry)
{
	for (int i = 0; i < numPreloaded; ++i)
	{
		if (preloadedZones[i].zoneEntry == zoneEntry)
			return ClassifyTrackedZone(i);
	}
	return RESET_UNTRACKED;
}

void __fastcall hook_resetUnloadZones(void* zoneMgr)
{
	// The game unloads its own Set A/B zones exactly as before.
	orig_resetUnloadZones(zoneMgr);

	uintptr_t zm = (uintptr_t)zoneMgr;
	if (!zm)
		return;

	DWORD tid     = GetCurrentThreadId();
	bool  onMain  = IsMainThread();
	bool  keyOn   = saveLoadUnloadEnabled;
	bool  fnBound = (fn_unloadZoneFromReset != NULL);
	bool  unload  = keyOn && fnBound;

	// NavMesh jobs in flight when the reset began: workerBusyCount is the
	// busy bridge, raised by the worker path at claim time and by the bg
	// thread's ProcessNavMeshJob, so it counts MISSes and HITs on both.
#if NMCACHE_STEP >= 1
	long nmBusy = InterlockedCompareExchange(&workerBusyCount, 0, 0);
#else
	long nmBusy = 0;
#endif
	NavMeshPjLockResult pj = NM_PJLOCK_NONE;
	DWORD pjWaitMs = 0;
	bool  pjTried  = false;
	ResetPjLockGuard pjGuard;

	// The mod's tracking (preloadedZones etc.) is main-thread state; off the
	// main thread it is neither read nor cleared here.
	int counts[RESET_CLASS_COUNT] = { 0, 0, 0, 0, 0, 0 };
	int survivors = 0;

	for (int i = 0; i < ZONE_GRID_COUNT; ++i)
	{
		// ZoneManager+200, 4096 ZoneMaps of 360 bytes (ctor sub_140A0BE00's
		// vector constructor; Set A starts right after, at +1474760). Index
		// order is x*64 + y (ZoneManager__lookupZone 0xA07C10); it does not
		// matter here, since every entry is visited.
		void* ze = (void*)(zm + OFF_ZM_ZONE_BASE + (size_t)ZONE_ENTRY_SIZE * (size_t)i);
		if (*(void**)((uintptr_t)ze + OFF_ZONE_CONTENT) == NULL)
			continue;

		survivors++;
		if (onMain)
		{
			int cls = ClassifyResetSurvivor(ze);
			counts[cls]++;
#ifdef ZONEOPT_DEBUG
			static const char* const kClassName[RESET_CLASS_COUNT] =
				{ "pending", "stalled", "registered", "promoted", "handoff", "untracked" };
			std::ostringstream ss;
			ss << "[ZoneOpt] Save load reset: zone (" << GetZoneGridX(ze) << ","
			   << GetZoneGridY(ze) << ") " << kClassName[cls]
			   << " load=" << (IsZoneLoading(ze) ? 1 : 0)
			   << " access=" << (IsZoneAccessible(ze) ? 1 : 0)
			   << (unload ? " unloading" : " left loaded");
			LogDebug(ss.str());
#endif
		}

		if (unload)
		{
			// processJobCS before the first unload, held to the end of the
			// loop. On timeout the unloads go ahead unlocked (the behaviour
			// before this fix): a load must never hang on the generator.
			if (!pjTried)
			{
				pjTried = true;
				pj = NavMeshTryLockProcessJobFor(RESET_PJ_LOCK_TIMEOUT_MS, &pjWaitMs);
				pjGuard.held = (pj == NM_PJLOCK_HELD);
			}
			fn_unloadZoneFromReset(zoneMgr, ze, 0);
		}
	}

	// Released right after the last unload, before any logging or state clear.
	pjGuard.Release();

	// The mod's state is cleared below, after the line, only on the main thread
	// and only when the zones were unloaded; otherwise the ZM+8 edge in
	// PreloadCheckSaveLoad does it, as before this hook existed.
	bool cleared = unload && onMain;

	// One PROD line per reset, built without CRT streams so it is safe on any
	// thread the reset might run on. It comes before the clear so that the
	// clear's own "Preload clear: dropped ..." line reads as its consequence.
	char buf[512];
	char classes[192];
	if (onMain)
		_snprintf_s(classes, sizeof(classes), _TRUNCATE,
			"pending=%d stalled=%d registered=%d promoted=%d handoff=%d untracked=%d",
			counts[RESET_PENDING], counts[RESET_STALLED], counts[RESET_REGISTERED],
			counts[RESET_PROMOTED], counts[RESET_HANDOFF], counts[RESET_UNTRACKED]);
	else
		_snprintf_s(classes, sizeof(classes), _TRUNCATE,
			"pending=- stalled=- registered=- promoted=- handoff=- untracked=-");

	// pj=none: no lock taken -- no survivor, nothing unloaded (key off or no
	// unload function), or no processJobCS in this build.
	const char* pjTok = (pj == NM_PJLOCK_HELD)    ? "held"
	                  : (pj == NM_PJLOCK_TIMEOUT) ? "timeout"
	                  :                             "none";

	const char* tail = "";
	if (!keyOn)
		tail = " (saveLoadUnload=off)";
	else if (!fnBound)
		tail = " (unload function not bound: nothing unloaded)";
	else if (!cleared)
		tail = " (off main thread: mod state not read or cleared here; left to the ZM+8 edge)";

	_snprintf_s(buf, sizeof(buf), _TRUNCATE,
		"[ZoneOpt] Save load reset: %s %d zone(s) outside Set A/B (%s) tid=%lu main=%d"
		" nmBusy=%ld pj=%s pjWaitMs=%lu%s",
		unload ? "unloaded" : "would unload", survivors, classes,
		(unsigned long)tid, onMain ? 1 : 0,
		nmBusy, pjTok, (unsigned long)pjWaitMs, tail);
	LogMsg(buf);

	// Drop the mod's state at the same moment the game drops its own, so no
	// pointer into a zone unloaded above is used again. With the key off
	// nothing was unloaded, so the clear stays where it is today (the edge).
	if (cleared)
	{
		ClearModStateForLoad();
		g_resetStateCleared = true;
	}
}


// Iteration order for registration/promotion.
//   ISLAND_STEP >= 3: navmesh tiers 1-3 first (camera grid, movers' current and
//   next zones, mover clusters), then camera-owned before character-owned, then
//   index — so the zones a travelling squad is about to need are registered
//   and promoted before speculative ones (shrinks the parked-at-edge window).
//   Otherwise: the historical order (ownerPasses: camera-owned then
//   character-owned; else plain index order).
static int BuildPreloadOrder(int* order, bool ownerPasses)
{
	int n = 0;
#if ISLAND_STEP >= 3
	(void)ownerPasses;
	static SchedContext ctx;   // main thread only; ~1.3 KB kept off the stack
	BuildSchedContext(&ctx);
	int key[MAX_PRELOADED];
	for (int i = 0; i < numPreloaded; ++i)
	{
		int tier = ComputeZonePriority(preloadedZones[i].gridX, preloadedZones[i].gridY,
		                               ctx.camX, ctx.camY, ctx.movers, ctx.moverCount,
		                               ctx.zones, ctx.zoneCount);
		if (tier < 1) tier = 1;
		if (tier > 5) tier = 5;
		int ownerRank = (preloadedZones[i].owner == OWNER_CAMERA) ? 0 : 1;
		key[i] = tier * 2 + ownerRank;
		// Stable insertion sort by key (numPreloaded <= 45)
		int k = n++;
		while (k > 0 && key[order[k - 1]] > key[i])
		{
			order[k] = order[k - 1];
			--k;
		}
		order[k] = i;
	}
#else
	if (ownerPasses)
	{
		for (int pass = 0; pass < 2; ++pass)
		{
			int targetOwner = (pass == 0) ? OWNER_CAMERA : OWNER_CHARACTER;
			for (int i = 0; i < numPreloaded; ++i)
				if (preloadedZones[i].owner == targetOwner)
					order[n++] = i;
		}
	}
	else
	{
		for (int i = 0; i < numPreloaded; ++i)
			order[n++] = i;
	}
#endif
	return n;
}


void PreparePreloadedZonesForTransition(void* zoneMgr)
{
	void* targetZone = *(void**)((uintptr_t)zoneMgr + OFF_ZM_CURRENT_ZONE);
	if (!targetZone)
		return;

	int tgtX = GetZoneGridX(targetZone);
	int tgtY = GetZoneGridY(targetZone);
	if (tgtX < 0 || tgtX > ZONE_GRID_MAX || tgtY < 0 || tgtY > ZONE_GRID_MAX)
		return;

	double now = ElapsedSec();
	int cleared = 0;
	for (int i = 0; i < numPreloaded; ++i)
	{
		if (preloadedZones[i].pipelineHandoff || preloadedZones[i].promoted)
			continue;
		void* ze = preloadedZones[i].zoneEntry;
		if (!ze)
			continue;

		int dx = preloadedZones[i].gridX - tgtX;
		int dy = preloadedZones[i].gridY - tgtY;
		if (dx < 0) dx = -dx;
		if (dy < 0) dy = -dy;
		if (dx > 1 || dy > 1)
			continue;

		// Clearing +176 lets loadSingleZone re-enter and loadZoneData add to Set A
		if (!IsZoneLoading(ze) || IsZoneAccessible(ze))
			continue;

		double age = now - preloadedZones[i].loadTimeSec;
		if (age > 60.0)
			continue;

		*(unsigned char*)((uintptr_t)ze + OFF_ZONE_IS_LOADING) = 0;
		preloadedZones[i].pipelineHandoff = true;
		preloadedZones[i].pending = false;
		if (pendingCount > 0) pendingCount--;
		cleared++;
	}

	if (cleared > 0)
	{
		std::ostringstream ss;
		ss << "[ZoneOpt] P3 handoff: cleared +176 on " << cleared
		   << " zones in grid (" << tgtX << "," << tgtY << ")";
		LogMsg(ss.str());
	}
}


static bool IsZoneQueued(int gx, int gy)
{
	for (int i = cameraQueueNext; i < cameraQueueCount; ++i)
		if (cameraQueue[i].gridX == gx && cameraQueue[i].gridY == gy)
			return true;
	for (int i = charQueueNext; i < charQueueCount; ++i)
		if (charQueue[i].gridX == gx && charQueue[i].gridY == gy)
			return true;
	for (int i = 0; i < numPreloaded; ++i)
		if (preloadedZones[i].gridX == gx && preloadedZones[i].gridY == gy)
			return true;
	return false;
}

bool EnqueueCameraZone(int gx, int gy)
{
	if (gx < 0 || gx > 63 || gy < 0 || gy > 63)
		return false;
	if (cameraQueueCount >= CAMERA_RESERVED)
		return false;
	if (IsZoneQueued(gx, gy))
		return false;

	cameraQueue[cameraQueueCount].gridX = gx;
	cameraQueue[cameraQueueCount].gridY = gy;
	cameraQueueCount++;
	return true;
}

bool EnqueueCharacterZone(int gx, int gy)
{
	if (gx < 0 || gx > 63 || gy < 0 || gy > 63)
		return false;
	if (charQueueCount >= MAX_PRELOADED)
	{
		if (charQueueNext > 0)
		{
			int remaining = charQueueCount - charQueueNext;
			for (int i = 0; i < remaining; ++i)
				charQueue[i] = charQueue[charQueueNext + i];
			charQueueCount = remaining;
			charQueueNext = 0;
		}
		if (charQueueCount >= MAX_PRELOADED)
			return false;
	}
	if (IsZoneQueued(gx, gy))
		return false;

	charQueue[charQueueCount].gridX = gx;
	charQueue[charQueueCount].gridY = gy;
	charQueueCount++;
	return true;
}

void FlushCameraQueue()
{
	cameraQueueCount = 0;
	cameraQueueNext = 0;
}

void EnqueueCameraGrid(int centerX, int centerY)
{
#if PATHFIND_STEP >= 8
	// STEP 8: 2x2 pattern (positive offsets) matches SMALL_DX/DY pause mechanism.
	// Was 3x3 (9 zones) + EnqueueAheadZones (3 zones) = 12 zones/prediction; now 4.
	for (int i = 0; i < 4; ++i)
	{
		int zx = centerX + SMALL_DX[i];
		int zy = centerY + SMALL_DY[i];
		EnqueueCameraZone(zx, zy);
	}
#else
	for (int i = 0; i < 9; ++i)
	{
		int zx = centerX + ORDER_DX[i];
		int zy = centerY + ORDER_DY[i];
		EnqueueCameraZone(zx, zy);
	}
#endif
}

static bool EnqueueZoneByOwner(int gx, int gy, int owner)
{
	if (owner == OWNER_CAMERA)
		return EnqueueCameraZone(gx, gy);
	else
		return EnqueueCharacterZone(gx, gy);
}

void EnqueueAheadZones(int centerX, int centerY, int fromX, int fromY, int owner)
{
	int ddx = centerX - fromX;
	int ddy = centerY - fromY;
	if (ddx > 1) ddx = 1; else if (ddx < -1) ddx = -1;
	if (ddy > 1) ddy = 1; else if (ddy < -1) ddy = -1;

	if (ddx == 0 && ddy == 0)
		return;

	if (ddx != 0 && ddy == 0)
	{
		int ax = centerX + 2 * ddx;
		EnqueueZoneByOwner(ax, centerY - 1, owner);
		EnqueueZoneByOwner(ax, centerY, owner);
		EnqueueZoneByOwner(ax, centerY + 1, owner);
	}
	else if (ddx == 0 && ddy != 0)
	{
		int ay = centerY + 2 * ddy;
		EnqueueZoneByOwner(centerX - 1, ay, owner);
		EnqueueZoneByOwner(centerX, ay, owner);
		EnqueueZoneByOwner(centerX + 1, ay, owner);
	}
	else
	{
		EnqueueZoneByOwner(centerX + 2 * ddx, centerY + ddy, owner);
		EnqueueZoneByOwner(centerX + ddx, centerY + 2 * ddy, owner);
		EnqueueZoneByOwner(centerX + 2 * ddx, centerY + 2 * ddy, owner);
	}
}

void ProcessPreloadQueue(void* zoneMgr)
{
	if (numPreloaded > 0 && preloadedZones[numPreloaded - 1].pending)
		return;
	if (numPreloaded >= MAX_PRELOADED)
		return;

	int gx, gy, owner;
	if (cameraQueueNext < cameraQueueCount)
	{
		gx = cameraQueue[cameraQueueNext].gridX;
		gy = cameraQueue[cameraQueueNext].gridY;
		cameraQueueNext++;
		owner = OWNER_CAMERA;
	}
	else if (charQueueNext < charQueueCount)
	{
		gx = charQueue[charQueueNext].gridX;
		gy = charQueue[charQueueNext].gridY;
		charQueueNext++;
		owner = OWNER_CHARACTER;
	}
	else
	{
		return;
	}

	void* zoneEntry = GetZoneEntry(zoneMgr, gx, gy);
	if (!zoneEntry)
		return;

	// Common fields for all three paths
	PreloadedZone& pz = preloadedZones[numPreloaded];
	pz.zoneEntry = zoneEntry;
	pz.gridX = gx;
	pz.gridY = gy;
	pz.promoted = false;
	pz.pending = false;
	pz.registered = false;
	pz.registeredEmpty = false;
	pz.pipelineHandoff = false;
	pz.contentProcessed = false;
	pz.loadTimeSec = ElapsedSec();
	pz.owner = owner;
#if PRELOAD_STEP >= 1
	// Default: this slot did not come from our own loadSingleZone call this
	// pass. Overwritten below only on the real load branch.
	pz.pipeLoadSec = -1.0;
	pz.pipeIsReadySeen = false;
	pz.pipe264Seen = false;
#endif

	if (IsZoneAccessible(zoneEntry))
	{
		pz.promoted = true;
		numPreloaded++;
		return;
	}

	if (IsZoneLoading(zoneEntry))
	{
		// Registry guard (fix 2b): adopting a loading zone leads straight to
		// processContent on its content, so refuse one whose registration is
		// not this content's (a save-load survivor). The slot is not committed.
		if (!RegistryGuardPasses(zoneEntry, REG_SITE_ADOPT))
		{
			pz.zoneEntry = NULL;
			pz.gridX = -1;
			pz.gridY = -1;
			return;
		}
		{
			std::ostringstream ss;
			ss << "[ZoneOpt] Tracking loading zone (" << gx << "," << gy << ")"
			   << (owner == OWNER_CAMERA ? " [cam]" : " [char]")
			   << " content=" << (*(void**)zoneEntry ? "yes" : "NULL");
			LogDebug(ss.str());
		}
		pz.pending = true;
		pendingCount++;
		numPreloaded++;
		return;
	}

	// Save/restore ready flag: loadSingleZone clears PhysicsInterface::_queuesClear
	// (physics+0x320), which makes NavMeshGenerator sleep and causes ~750ms stalls.
	// The read stays a plain load (single byte, no lock); the restore goes through
	// the game's setter, which takes queuesClearMuto (+0x328) around the write (H13).
	uintptr_t physics = *(uintptr_t*)(gameBase + RVA_PAUSESTATE_PHYSICS);
	unsigned char savedReady = 0;
	if (physics)
		savedReady = *(unsigned char*)(physics + 800);

	// Crash-3 guard: never let a preload attempt touch a zone the game already
	// owns. loadSingleZone (0xA0D6A0) writes the keep-alive timer at
	// zoneEntry + 4*(timerIndex + 48) BEFORE its already-loaded early return:
	//
	//     if ( keepAliveSeconds > 0.0 )
	//       *((float *)zoneEntry + v4 + 48) = keepAliveSeconds;      // A0D6F4
	//     else
	//       *((_DWORD *)zoneEntry + v4 + 48) = g_zoneKeepAliveDefaultSeconds[v4];
	//     if ( zoneEntry && (*((_BYTE *)zoneEntry + 176)
	//                     || *((_BYTE *)zoneEntry + 177)) )
	//       return 0;                                                // A0D70F
	//
	// So a call on an already-loaded zone does nothing except reset that zone's
	// unload timer — which changes when the game unloads sectors, and sector
	// unloading was in flight on the path thread in the crash second.
	//
	// The two IsZone* returns above already cover exactly this condition (+176 is
	// OFF_ZONE_IS_LOADING, +177 is OFF_ZONE_IS_ACCESS), so this test should never
	// fire. It is here as the explicit, named guarantee, and preloadSkipLoaded on
	// the transition line is the evidence: a non-zero count would mean the two
	// early returns above have a hole in them.
	if (IsZoneLoading(zoneEntry) || IsZoneAccessible(zoneEntry))
	{
		preloadSkipLoaded++;
#ifdef ZONEOPT_DEBUG
		// One line per zone: this runs in the preload queue, which revisits the
		// same coordinates every transition.
		static unsigned char skipLogged[(ZONE_GRID_MAX + 1) * (ZONE_GRID_MAX + 1)] = { 0 };
		int slot = gy * (ZONE_GRID_MAX + 1) + gx;
		if (slot >= 0 && slot < (int)sizeof(skipLogged) && !skipLogged[slot])
		{
			skipLogged[slot] = 1;
			std::ostringstream ss;
			ss << "[ZoneOpt] Preload skip: zone (" << gx << "," << gy
			   << ") already loaded by the game";
			LogDebug(ss.str());
		}
#endif
		// Record the slot the way the two branches above would have: a zone the
		// game has finished loading is promoted, one still loading is not. If this
		// guard ever does fire, it must not leave behind a third kind of slot
		// (an unpromoted, unpending entry for a zone the game owns) that no other
		// code path knows how to reason about.
		pz.promoted = IsZoneAccessible(zoneEntry);
		numPreloaded++;
		return;
	}

	// loadSingleZone takes four arguments and the last is a float in xmm3
	// (game.h): the keep-alive seconds written at zoneEntry + 4*(timerIndex + 48),
	// which decide when the game unloads this zone again. 0 is what the game's own
	// processState2 passes (xorps xmm6,xmm6 / movaps xmm3,xmm6 before the call at
	// 0xA0D928), i.e. take the per-timer default. The zones reaching this line are
	// ones the game does not have, so the default is the value the game itself
	// would have written for them.
	//
	// preloadKeepAliveSeconds makes it tunable so the surviving half of crash-3
	// hypothesis H2 — that Round 1 changed preloaded zones' unload timing — can be
	// A/B'd (0 vs 3600) without a rebuild. It does not restore the pre-Round-1
	// behaviour: that call passed three arguments and left xmm3 undefined, so no
	// fixed value reproduces it.
	bool result = fn_loadSingleZone(zoneEntry, 0, 0, cfg_preloadKeepAliveSeconds);
	if (physics && savedReady && fn_setQueuesAreClear)
		fn_setQueuesAreClear((void*)physics, true);

	pz.pending = (result != 0);
	if (result != 0)
	{
		pendingCount++;
#if PRELOAD_STEP >= 1
		pz.pipeLoadSec = pz.loadTimeSec;  // stable base; loadTimeSec itself is reused later
		g_pipe.loaded++;
#endif
	}
	numPreloaded++;

	if (result)
	{
		std::ostringstream ss;
		ss << "[ZoneOpt] Preloaded zone (" << gx << "," << gy << ")"
		   << (owner == OWNER_CAMERA ? " [cam]" : " [char]");
		LogDebug(ss.str());
	}
}


// Calls content->vtable[4](content) — the game's processContent function.
// Populates things, objects, and buildings from the zone data file.
// Same call that processState2 makes, but invoked directly.
static void CallProcessContent(void* content)
{
	uintptr_t vtable = *(uintptr_t*)((uintptr_t)content);
	uintptr_t fn = *(uintptr_t*)(vtable + 32);
	typedef void (__fastcall *ProcessContentFn)(void*);
	((ProcessContentFn)fn)(content);
}

void TryRegisterPreloadedZones(void* zoneMgr, double now)
{
	int zmState = GetZoneState(zoneMgr);
	if (zmState != 0)
		return;

	// isReadyForSections: 4 work queues empty + _queuesClear (physics+0x320) set
	uintptr_t physics = *(uintptr_t*)(gameBase + RVA_PAUSESTATE_PHYSICS);
	if (!physics)
		return;
	if (*(int*)(physics + 432) > 0) return;
	if (*(int*)(physics + 680) > 0) return;
	if (*(int*)(physics + 760) > 0) return;
	if (*(int*)(physics + 512) > 0) return;
	if (*(unsigned char*)(physics + 800) == 0) return;

	uintptr_t sectionMgr = *(uintptr_t*)(gameBase + RVA_GLOBAL_SECTION_MGR);
	if (!sectionMgr)
		return;
	if (!fn_registerZoneSections)
		return;

	int order[MAX_PRELOADED];
	int orderCount = BuildPreloadOrder(order, /*ownerPasses=*/false);

	for (int k = 0; k < orderCount; ++k)
	{
		int i = order[k];
		if (!preloadedZones[i].pending)
			continue;
		if (preloadedZones[i].registered)
			continue;
		if (preloadedZones[i].promoted || preloadedZones[i].pipelineHandoff)
			continue;

		void* ze = preloadedZones[i].zoneEntry;
		if (!ze)
			continue;

		if (!IsZoneLoading(ze) || IsZoneAccessible(ze))
			continue;

		void* content = *(void**)ze;
		if (!content)
			continue;

#if PRELOAD_STEP >= 1
		// Step 1: poll isZoneReady and content+264 every frame for every
		// pending, unregistered zone examined here -- cheap and non-blocking
		// (research/preload_pipeline.md §2), independent of the age gates
		// below, which decide only when CallProcessContent is actually called.
		if (preloadedZones[i].pipeLoadSec >= 0.0)
		{
			if (!preloadedZones[i].pipeIsReadySeen && fn_isZoneReady && fn_isZoneReady(ze))
			{
				preloadedZones[i].pipeIsReadySeen = true;
				PipeSampleAdd(g_pipe.toReadyMs, &g_pipe.toReadyN,
				              (now - preloadedZones[i].pipeLoadSec) * 1000.0);
			}
			if (!preloadedZones[i].pipe264Seen &&
			    *(unsigned char*)((uintptr_t)content + OFF_ZMC_READY_FLAG) == 0)
			{
				preloadedZones[i].pipe264Seen = true;
				PipeSampleAdd(g_pipe.to264Ms, &g_pipe.to264N,
				              (now - preloadedZones[i].pipeLoadSec) * 1000.0);
			}
		}
#endif

		int thingsCount = *(int*)((uintptr_t)content + OFF_ZMC_THINGS_COUNT);

		// processContent (vtable[4]): populates things/objects from zone data.
		// Called once per zone, after content streaming has had time to finish.
		// things<=1 means no real content yet (0=unprocessed, 1=processed-empty).
		if (thingsCount <= 1 && !preloadedZones[i].contentProcessed)
		{
			double age = now - preloadedZones[i].loadTimeSec;
			if (age < 2.0)
				continue;  // give content streaming time to finish

			// Registry guard (fix 2b): processContent instantiates the zone's
			// saved objects, and each resolves its handle through this slot.
			if (!RegistryGuardPasses(ze, REG_SITE_PROCESS))
			{
				DropTrackedZone(i);
				continue;
			}

#if PRELOAD_STEP >= 1
			// isZoneReady's state right at the moment the age gate fires the
			// call -- answers "notReadyAtCall" below.
			bool wasReadyAtCall = preloadedZones[i].pipeIsReadySeen;
			LARGE_INTEGER pcStart, pcEnd;
			QueryPerformanceCounter(&pcStart);
#endif

			CallProcessContent(content);
			preloadedZones[i].contentProcessed = true;
			thingsCount = *(int*)((uintptr_t)content + OFF_ZMC_THINGS_COUNT);

#if PRELOAD_STEP >= 1
			QueryPerformanceCounter(&pcEnd);
			PipeSampleAdd(g_pipe.procContentMs, &g_pipe.procContentN, QPCToMs(pcStart, pcEnd));

			bool clear264After = (*(unsigned char*)((uintptr_t)content + OFF_ZMC_READY_FLAG) == 0);
			if (!preloadedZones[i].pipe264Seen && clear264After && preloadedZones[i].pipeLoadSec >= 0.0)
			{
				preloadedZones[i].pipe264Seen = true;
				PipeSampleAdd(g_pipe.to264Ms, &g_pipe.to264N,
				              (now - preloadedZones[i].pipeLoadSec) * 1000.0);
			}

			if (thingsCount == 0)      g_pipe.thingsAfter0++;
			else if (thingsCount == 1) g_pipe.thingsAfter1++;

			// Attempt-3 question (research/preload_pipeline.md §7): does
			// isZoneReady pass while things==0 after finalizeContent?
			// readyEmpty answers "yes, and it still came back empty";
			// notReadyAtCall counts calls the age gate fired despite
			// isZoneReady still being false.
			if (wasReadyAtCall && clear264After && thingsCount <= 1)
				g_pipe.readyEmpty++;
			if (!wasReadyAtCall)
				g_pipe.notReadyAtCall++;
#endif
		}

		// Empty zones: things<=1 after processContent means no real content.
		// things=0: no data file processed. things=1: data file processed, nothing to load.
		if (thingsCount <= 1)
		{
			double age = now - preloadedZones[i].loadTimeSec;
			if (age < 3.0)
				continue;
		}

		// Registry guard (fix 2b): never hand the game's section registration
		// a zone whose content is not the one registered for it.
		if (!RegistryGuardPasses(ze, REG_SITE_REGISTER))
		{
			DropTrackedZone(i);
			continue;
		}

		fn_registerZoneSections((void*)sectionMgr, ze);
		// Post-condition: every game caller of registerZoneSections clears
		// ZoneMap+0xCC afterwards, and each writes a single byte. Writing an
		// int here would zero the three bytes that follow it (H12).
		*(unsigned char*)((uintptr_t)ze + 204) = 0;

		preloadedZones[i].registered = true;
		preloadedZones[i].registeredEmpty = (thingsCount <= 1);

#if PRELOAD_STEP >= 1
		g_pipe.registered++;
		if (preloadedZones[i].pipeLoadSec >= 0.0)
			PipeSampleAdd(g_pipe.toRegMs, &g_pipe.toRegN,
			              (now - preloadedZones[i].pipeLoadSec) * 1000.0);
#endif

		preloadedZones[i].loadTimeSec = now;  // reset for registration-to-promotion timer

		{
			std::ostringstream ss;
			ss << "[ZoneOpt] Registered zone ("
			   << preloadedZones[i].gridX << ","
			   << preloadedZones[i].gridY << ") things="
			   << thingsCount
			   << (preloadedZones[i].registeredEmpty ? " (empty)" : "");
			LogMsg(ss.str());
		}

		return;  // one per frame
	}
}

void TryPromotePreloadedZones(void* zoneMgr, double now)
{
	uintptr_t sectionMgr = *(uintptr_t*)(gameBase + RVA_GLOBAL_SECTION_MGR);
	if (!sectionMgr)
		return;

	// state==0 only: notifyAccessible is NOT idempotent (allocates at +1680)
	int zmState = GetZoneState(zoneMgr);
	if (zmState != 0)
	{
		static double lastStateBlockLog = 0.0;
		if (now - lastStateBlockLog > 5.0)
		{
			std::ostringstream ss;
			ss << "[ZoneOpt] Promote blocked: state=" << zmState
			   << " pending=" << pendingCount;
			LogDebug(ss.str());
			lastStateBlockLog = now;
		}
		return;
	}

	bool didFullPromotion = false;

	// Camera-owned first (tier 1-3 zones first at ISLAND_STEP >= 3), one full
	// promotion per frame (notifyAccessible is expensive)
	int order[MAX_PRELOADED];
	int orderCount = BuildPreloadOrder(order, /*ownerPasses=*/true);

	{
		for (int k = 0; k < orderCount; ++k)
		{
			int i = order[k];
			if (!preloadedZones[i].pending || preloadedZones[i].promoted)
				continue;
			if (preloadedZones[i].pipelineHandoff)
				continue;

			if (!preloadedZones[i].registered)
				continue;

			void* ze = preloadedZones[i].zoneEntry;
			if (!ze)
				continue;

			if (!IsZoneLoading(ze) || IsZoneAccessible(ze))
			{
				preloadedZones[i].pending = false;
				preloadedZones[i].promoted = true;
				if (pendingCount > 0) pendingCount--;
#if PRELOAD_STEP >= 1
				g_pipe.promoted++;
				if (preloadedZones[i].pipeLoadSec >= 0.0)
					PipeSampleAdd(g_pipe.toPromoMs, &g_pipe.toPromoN,
					              (now - preloadedZones[i].pipeLoadSec) * 1000.0);
#endif
				continue;
			}

			if (didFullPromotion)
				continue;

			bool ready = false;
			if (orig_isContentPending)
			{
				int gridCoords[2] = { preloadedZones[i].gridX, preloadedZones[i].gridY };
				ready = orig_isContentPending((void*)sectionMgr, (void*)gridCoords);
			}

			// Fallback: sectionCount==0 means contentStream finished all sections.
			// H15 control (docs/release_readiness.md): readinessOverrides=false
			// disables this fallback -- only the original's answer promotes.
			// hook_isContentPending's deferral is disabled the same way (H2).
			if (!ready && !readinessOverridesEnabled)
				continue;
			if (!ready)
			{
				double age = now - preloadedZones[i].loadTimeSec;
				if (age < 2.0)
					continue;  // Give contentStream time after registration

				int secCount = *(int*)((uintptr_t)sectionMgr + 632);
				ready = (secCount == 0);

				if (!ready)
				{
					static double lastNotReadyLog = 0.0;
					if (now - lastNotReadyLog > 3.0)
					{
						std::ostringstream ss;
						ss << "[ZoneOpt] Promote not ready (" << preloadedZones[i].gridX
						   << "," << preloadedZones[i].gridY << "): sec=" << secCount
						   << " age=" << std::fixed << std::setprecision(1) << age << "s";
						LogDebug(ss.str());
						lastNotReadyLog = now;
					}
					continue;
				}
			}

			void* zoneMapContent = *(void**)(ze);
			if (!zoneMapContent)
				continue;

			// Wait for object hash table (+304) — notifyAccessible iterates it.
			// Promoting before populated = NPCs get no pathfinding setup.
			int objCount = *(int*)((uintptr_t)zoneMapContent + 304);
			if (objCount <= 0 && !preloadedZones[i].registeredEmpty
			    && !preloadedZones[i].contentProcessed)
			{
				static double lastEmptyLog = 0.0;
				if (now - lastEmptyLog > 3.0)
				{
					std::ostringstream ss;
					ss << "[ZoneOpt] Promote waiting (" << preloadedZones[i].gridX
					   << "," << preloadedZones[i].gridY
					   << "): objs=" << objCount
					   << " age=" << std::fixed << std::setprecision(1)
					   << (now - preloadedZones[i].loadTimeSec) << "s";
					LogDebug(ss.str());
					lastEmptyLog = now;
				}
				continue;
			}

#if ISLAND_STEP >= 3 && defined(ZONEOPT_DEBUG)
			// Test hook: hold promotion N seconds after registration so a
			// travelling squad reaches the island edge before the zone joins
			// (forces the Step 3 race). Must stay below the 10 s stall timeout.
			if (cfg_islandTestPromoteDelay > 0.0
			    && now - preloadedZones[i].loadTimeSec < cfg_islandTestPromoteDelay)
				continue;
#endif

			// Registry guard (fix 2b): before anything is written or called for
			// this zone -- the island mark, the +176/+177 word, notifyZoneReady
			// and notifyAccessible.
			if (!RegistryGuardPasses(ze, REG_SITE_PROMOTE))
			{
				DropTrackedZone(i);
				continue;
			}

			// Island overlay: mark before the zone becomes accessible so the
			// next rebuild (requested below) sees it as a mod zone.
			IslandMarkModZone(zoneMgr, preloadedZones[i].gridX, preloadedZones[i].gridY);

			// WORD write: atomically clear +176, set +177
			*(unsigned short*)((uintptr_t)ze + OFF_ZONE_IS_LOADING) = 0x0100;

			void* sectionEntry = fn_notifyZoneReady(ze);
			if (sectionEntry)
				fn_finalizeZoneResources(sectionEntry);

			// Safe sole invocation: our zones bypass Set A/B (direct loadSingleZone)
			fn_notifyAccessible(zoneMapContent);

			IslandRequestRebuild();

			preloadedZones[i].promoted = true;
			preloadedZones[i].pending = false;
			if (pendingCount > 0) pendingCount--;
			promotedCount++;
			didFullPromotion = true;

#if PRELOAD_STEP >= 1
			g_pipe.promoted++;
			if (preloadedZones[i].pipeLoadSec >= 0.0)
				PipeSampleAdd(g_pipe.toPromoMs, &g_pipe.toPromoN,
				              (now - preloadedZones[i].pipeLoadSec) * 1000.0);
#endif

			{
				// Read objCount AFTER notifyAccessible (it populates the hash table)
				int objCount = *(int*)((uintptr_t)zoneMapContent + 304);
				double delayMs = (now - preloadedZones[i].loadTimeSec) * 1000.0;
				std::ostringstream ss;
				ss << "[ZoneOpt] Promoted zone (" << preloadedZones[i].gridX
				   << "," << preloadedZones[i].gridY << ") "
				   << std::fixed << std::setprecision(0) << delayMs << "ms after register"
				   << " objs=" << objCount;
				LogMsg(ss.str());
			}
		}
	}
}


void EvictStaleZones(void* zoneMgr, double now)
{
	for (int i = numPreloaded - 1; i >= 0; --i)
	{
		void* ze = preloadedZones[i].zoneEntry;
		if (!ze)
		{
			numPreloaded--;
			if (i < numPreloaded)
			{
				preloadedZones[i] = preloadedZones[numPreloaded];
				++i;
			}
			continue;
		}

		bool loading = IsZoneLoading(ze);
		bool accessible = IsZoneAccessible(ze);

		if (!loading && !accessible && !preloadedZones[i].pending)
		{
			numPreloaded--;
			if (i < numPreloaded)
			{
				preloadedZones[i] = preloadedZones[numPreloaded];
				++i;
			}
			continue;
		}

		if (preloadedZones[i].pipelineHandoff)
		{
			if (accessible)
			{
				preloadedZones[i].promoted = true;
				preloadedZones[i].pipelineHandoff = false;
				promotedCount++;
			}
			continue;
		}

		if (preloadedZones[i].pending)
		{
			double timeout = preloadedZones[i].registered ? 10.0 : 5.0;
			if (now - preloadedZones[i].loadTimeSec > timeout)
			{
				{
					std::ostringstream ss;
					ss << "[ZoneOpt] Stall timeout: zone (" << preloadedZones[i].gridX
					   << "," << preloadedZones[i].gridY << ") pending "
					   << std::fixed << std::setprecision(1)
					   << (now - preloadedZones[i].loadTimeSec) << "s"
					   << " registered=" << (preloadedZones[i].registered ? 1 : 0)
					   << " load=" << (IsZoneLoading(preloadedZones[i].zoneEntry) ? 1 : 0)
					   << " access=" << (IsZoneAccessible(preloadedZones[i].zoneEntry) ? 1 : 0);
					LogMsg(ss.str());
				}
				preloadedZones[i].pending = false;
				if (pendingCount > 0) pendingCount--;
				continue;
			}
		}

		// Zombie: clear +176 so game sees it as unloaded
		if (loading && !accessible
		    && !preloadedZones[i].pending
		    && !preloadedZones[i].promoted
		    && !preloadedZones[i].pipelineHandoff)
		{
			double age = now - preloadedZones[i].loadTimeSec;
			if (age > 10.0)
			{
				{
					std::ostringstream ss;
					ss << "[ZoneOpt] Zombie eviction: zone ("
					   << preloadedZones[i].gridX << ","
					   << preloadedZones[i].gridY << ") age="
					   << std::fixed << std::setprecision(1) << age << "s"
					   << " reg=" << (preloadedZones[i].registered ? 1 : 0);
					LogMsg(ss.str());
				}
				*(unsigned char*)((uintptr_t)ze + OFF_ZONE_IS_LOADING) = 0;
				numPreloaded--;
				if (i < numPreloaded)
				{
					preloadedZones[i] = preloadedZones[numPreloaded];
					++i;
				}
				continue;
			}
		}

		if (preloadedZones[i].promoted && (now - preloadedZones[i].loadTimeSec > 30.0))
		{
			int zx = preloadedZones[i].gridX;
			int zy = preloadedZones[i].gridY;

			if (preloadedZones[i].owner == OWNER_CAMERA && lastCameraGX >= 0)
			{
				int cdx = (zx > lastCameraGX) ? (zx - lastCameraGX) : (lastCameraGX - zx);
				int cdy = (zy > lastCameraGY) ? (zy - lastCameraGY) : (lastCameraGY - zy);
				if (cdx <= 1 && cdy <= 1)
					continue;
			}

			bool nearPlayer = false;
			uintptr_t playerIntf = *(uintptr_t*)(gameBase + RVA_GLOBAL_PLAYER);
			if (playerIntf)
			{
				unsigned int scCount = GetPlayerCharCount(playerIntf);
				uintptr_t* scStuff = GetPlayerCharStuff(playerIntf);
				if (scStuff && scCount > 0 && scCount <= 200)
				{
					for (unsigned int j = 0; j < scCount; ++j)
					{
						if (!scStuff[j])
							continue;
						float cx = GetCharPosX(scStuff[j]);
						float cz = GetCharPosZ(scStuff[j]);
						int cgx, cgy;
						if (WorldToZoneGrid(cx, cz, &cgx, &cgy))
						{
							int dx = (zx > cgx) ? (zx - cgx) : (cgx - zx);
							int dy = (zy > cgy) ? (zy - cgy) : (cgy - zy);
							if (dx <= 1 && dy <= 1)
							{
								nearPlayer = true;
								break;
							}
						}
					}
				}
			}
			if (!nearPlayer)
			{
				numPreloaded--;
				if (i < numPreloaded)
				{
					preloadedZones[i] = preloadedZones[numPreloaded];
					++i;
				}
			}
			continue;
		}
	}

	if (cameraQueueNext >= cameraQueueCount)
		FlushCameraQueue();
	if (charQueueNext >= charQueueCount)
	{
		charQueueNext = 0;
		charQueueCount = 0;
	}
}


bool TrySquadSwitchSwap(void* zoneMgr, int camGX, int camGY)
{
	int matches = 0;
	for (int i = 0; i < numPreloaded; ++i)
	{
		if (preloadedZones[i].owner != OWNER_CHARACTER)
			continue;
		int dx = preloadedZones[i].gridX - camGX;
		int dy = preloadedZones[i].gridY - camGY;
		if (dx < 0) dx = -dx;
		if (dy < 0) dy = -dy;
		if (dx <= 1 && dy <= 1)
			matches++;
	}

	if (matches < 5)
		return false;

	for (int i = 0; i < numPreloaded; ++i)
	{
		if (preloadedZones[i].owner == OWNER_CAMERA)
			preloadedZones[i].owner = OWNER_CHARACTER;
	}

	int swapped = 0;
	for (int i = 0; i < numPreloaded; ++i)
	{
		if (preloadedZones[i].owner != OWNER_CHARACTER)
			continue;
		int dx = preloadedZones[i].gridX - camGX;
		int dy = preloadedZones[i].gridY - camGY;
		if (dx < 0) dx = -dx;
		if (dy < 0) dy = -dy;
		if (dx <= 1 && dy <= 1)
		{
			preloadedZones[i].owner = OWNER_CAMERA;
			swapped++;
		}
	}

	FlushCameraQueue();
	predictedCenterX = camGX;
	predictedCenterY = camGY;

	int missing = 0;
#if PATHFIND_STEP >= 8
	// STEP 8: match the new camera 2x2 pattern so a squad-switch jump
	// doesn't re-queue an exception-to-the-rule 3x3 grid.
	for (int i = 0; i < 4; ++i)
	{
		if (EnqueueCameraZone(camGX + SMALL_DX[i], camGY + SMALL_DY[i]))
			missing++;
	}
#else
	for (int i = 0; i < 9; ++i)
	{
		if (EnqueueCameraZone(camGX + ORDER_DX[i], camGY + ORDER_DY[i]))
			missing++;
	}
#endif

	std::ostringstream ss;
	ss << "[ZoneOpt] Squad switch swap: (" << camGX << "," << camGY << ")"
	   << " " << swapped << " zones retagged"
	   << ", " << missing << " queued";
	LogMsg(ss.str());

	return true;
}
