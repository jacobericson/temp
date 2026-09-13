#include "preload.h"
#include "transition.h"
#include "nm_workers.h"
#include "hooks.h"
#include "tracking.h"
#include "formation.h"
#include "pathfinding.h"
#include "navmesh_sched.h"
#include "islands.h"
#include "path_pool.h"

// Round 2 H2 bindings, declared in game.h's H2 block and resolved in
// InitGameBindings. Defined here because game.cpp's only H2-owned lines are
// the ones inside InitGameBindings.
lookupSection_t      fn_lookupSection      = NULL;
boostUnlock_t        fn_boostUnlock        = NULL;
boostUnlockShared_t  fn_boostUnlockShared  = NULL;

// Transition state (defined here, declared in transition.h)
bool           isTransitionActive   = false;
int            deferredFrameCount   = 0;
LARGE_INTEGER  transitionStartTime;
volatile LONG  transitionEndPending  = 0;
LARGE_INTEGER  transitionEndQpc;
DWORD          transitionEndTid      = 0;

static bool prioritizedThisTransition = false;

// Bracket generation. Every transition start increments it; the dismissal
// stamps the generation its bracket belongs to. The deferred completion runs
// its body only while the two still agree, so a bracket that started before
// the main thread got round to the previous completion cannot be torn down
// by it.
static volatile LONG transitionGen    = 0;
static LONG          transitionEndGen = 0;


// =========================================================================
// Bracket instrumentation (O11 / Phase 18 Step 1)
// =========================================================================

// Target 3x3 at transition start. Written by CaptureTransitionTarget on the
// thread that opens the bracket (normally the main thread, see B6), read by
// LogTransitionTarget on the main thread. When the start ran off the main
// thread, g_tgtLogPending is raised after the buffer is written and the next
// main-thread hook_updateCameraZone claims it (interlocked pair orders them).
static char          g_tgtLetters[10];
static int           g_tgtX          = -1;
static int           g_tgtY          = -1;
static volatile LONG g_tgtLogPending = 0;

// Main-thread hook_updateCameraZone frames per zone-manager state while a
// bracket is open. Main thread only. g_bracketFramesGen is the transitionGen
// the counts belong to (0 = none yet): a new bracket is noticed by its
// generation and the counts restart, so the start branch, which can run off
// the main thread, never has to touch them.
const int            BRACKET_STATES    = 6;       // ZoneManager states 0..5
static int           g_bracketFrames[BRACKET_STATES];
static LONG          g_bracketFramesGen = 0;

// One zone of the target 3x3. Allocation-free and lock-free: it may run on
// whichever thread opened the bracket. Reads the preload arrays read-only (the
// same racy read PreparePreloadedZonesForTransition already does there) and
// the zone's +176/+177 bytes, which live in the ZoneManager's static array.
//   U unloaded, P ours in flight (loading, or tracked and not yet registered),
//   R ours registered, M ours promoted, A accessible (+177, not ours),
//   G game-loaded (+176, not ours), '-' off the grid.
static char TargetZoneLetter(void* zm, int gx, int gy)
{
	if (gx < 0 || gx > ZONE_GRID_MAX || gy < 0 || gy > ZONE_GRID_MAX)
		return '-';

	int n = numPreloaded;
	if (n > MAX_PRELOADED) n = MAX_PRELOADED;
	if (n < 0) n = 0;
	for (int i = 0; i < n; ++i)
	{
		if (preloadedZones[i].gridX != gx || preloadedZones[i].gridY != gy)
			continue;
		if (preloadedZones[i].promoted)   return 'M';
		if (preloadedZones[i].registered) return 'R';
		return 'P';
	}

	void* ze = GetZoneEntry(zm, gx, gy);
	if (!ze)                   return '-';
	if (IsZoneAccessible(ze))  return 'A';
	if (IsZoneLoading(ze))     return 'G';
	return 'U';
}

// Fills g_tgtLetters for the 3x3 around the transition target: the zone the
// P3 handoff (PreparePreloadedZonesForTransition) uses, ZoneManager's current
// zone. Rows are gy-1..gy+1, columns gx-1..gx+1, so index 4 is the target.
// Any thread; returns false when there is no target to report.
static bool CaptureTransitionTarget()
{
	void* zm = g_cachedZoneMgr;
	if (!zm)
		return false;
	void* tz = *(void**)((uintptr_t)zm + OFF_ZM_CURRENT_ZONE);
	if (!tz)
		return false;
	int tx = GetZoneGridX(tz);
	int ty = GetZoneGridY(tz);
	if (tx < 0 || tx > ZONE_GRID_MAX || ty < 0 || ty > ZONE_GRID_MAX)
		return false;

	int k = 0;
	for (int dy = -1; dy <= 1; ++dy)
		for (int dx = -1; dx <= 1; ++dx)
			g_tgtLetters[k++] = TargetZoneLetter(zm, tx + dx, ty + dy);
	g_tgtLetters[9] = '\0';
	g_tgtX = tx;
	g_tgtY = ty;
	return true;
}

// Main thread only (CRT string work).
static void LogTransitionTarget()
{
	std::ostringstream ss;
	ss << "[ZoneOpt] Transition target: (" << g_tgtX << "," << g_tgtY << ") 3x3="
	   << g_tgtLetters << " focus=" << g_tgtLetters[4];
	LogMsg(ss.str());
}

// Main thread, every hook_updateCameraZone frame.
static void CountBracketFrame(void* zm)
{
	if (!isTransitionActive || !zm)
		return;
	LONG gen = InterlockedCompareExchange(&transitionGen, 0, 0);
	if (gen != g_bracketFramesGen)
	{
		for (int s = 0; s < BRACKET_STATES; ++s)
			g_bracketFrames[s] = 0;
		g_bracketFramesGen = gen;
	}
	int state = GetZoneState(zm);
	if (state >= 0 && state < BRACKET_STATES)
		g_bracketFrames[state]++;
}


// =========================================================================
// Readiness classification + per-caller rule (Round 2 H2, H4)
// =========================================================================
//
// Section-state read design (IDA evidence in game.h's H2 block):
//
// The rule needs one fact per zone: is the zone's outdoor navmesh instance in
// the world? The walk in contentStream (0x3AEA42) tests it as state+0x28 != NULL
// with +0x1A4 >= 0. Reading it through state+0x28 from another thread is not
// safe: removeNavInstance (0x3AB8A0, path thread) removes the instance from the
// world under +0x200, RELEASES +0x200, then deletes the instance and only then
// NULLs +0x28. And the map at +0x220 is not only written under +0x1E0: the
// path thread's addZoneSection (0x3AD800, from processZoneWorkItem) inserts
// with operator[] after processZoneWorkItem has released +0x1E0.
//
// So the outdoor test goes through the world instead, the way the game's own
// navmesh queries do: try-lock +0x200 SHARED with the exact inline sequence of
// NavMesh__snapToFace (0x3A1F70), give up if it fails, and scan the main
// world's streaming collection for the instance whose section uid is
// gridX | (gridY << 8) (hkaiStreamingCollection's own lookup, 0xD0CD40). Every
// add and every world removal takes +0x200 exclusively, and the delete comes
// after the removal, so while we hold it shared each instance in the
// collection is alive and its +0x1A4 is stable. Found with +0x1A4 >= 0 is
// exactly "outdoor instance in the world". Held for the scan only.
//
// Uid collisions. Building sub-instances share the collection and its uid
// space. A runtime building uid is container | (index+1) << 16 | serial << 26
// (0x3A1440, over the building's hand: container +0xC, index +0x14, serial
// +0x18); file-loaded ones are parsed from the navmesh file's info string
// (insertSection 0x3A822A), which carries the same value. A building can
// equal a zone key gx | gy << 8 (<= 0x3F3F) only if ((index+1) & 0xFFFF) == 0
// AND (serial & 0x3F) == 0 AND its container's low 16 bits happen to be that
// zone key: a handle index of exactly 65535 + k*65536, a serial divisible by
// 64 and a zone-shaped container, all at once, for a building whose navmesh is
// in the world. The game makes the same assumption itself: contentStream
// treats a section uid below 0xFFFF as a zone (0x3AE6AD, the NavBuildLock
// key). No cheaper per-instance discriminator exists: outdoor and building
// instances are created by the same function (0x3ACD20 via 0xD09EB0) and
// carry no type field; the only other tie, section state +0x28, is exactly
// the pointer that cannot be read safely (above).
//
// Only when the scan says "not in the world" is the map consulted, to tell
// "no section" from "section, outdoor mesh missing" (diagnostic only; the rule
// answers 0 for both). That takes +0x1E0 with a try-exclusive (the inline
// sequence the original uses on +0x200 at 0x3AB59D; the original itself takes
// +0x1E0 with a blocking timed lock), calls the game's lookupSection -- the
// same lookup the original has just done -- and compares pointers only. The
// state object is never dereferenced. If that try fails, or the split is
// skipped, the class is "not in world" (RZ_NOT_IN_WORLD): the fact the rule
// needs was read, only the split is missing, so the rule still answers 0.
// Off the main thread with the rule on the split is skipped altogether, so the
// rule path adds no contention against the blocking map lookups of the game's
// threads; it runs on the main thread and in the sampled rule-off calls.
//
// The two locks are never held together. Both are try-locks, so no thread ever
// blocks here. Releases go through the game's own unlock/unlock_shared.
//
// Rejected: a per-zone table built on the main thread and published by
// seqlock. It would read the same world under the same lock, one frame stale,
// and its build would walk the whole section map every frame, which widens the
// exposure to the path thread's unlocked map insert from one lookup to a full
// traversal.

namespace {

enum ReadyCaller { RC_POLL4 = 0, RC_MAIN = 1, RC_OFF = 2, RC_COUNT = 3 };
// RZ_NOT_IN_WORLD: the scan read "outdoor instance not in the world", but the
// noSection/outdoorMissing split was skipped or its map try-lock failed.
// RZ_UNKNOWN: the +0x200 try failed (or the collection was unreadable), so
// the outdoor fact itself was not read.
enum ReadyClass  { RZ_NO_SECTION = 0, RZ_OUTDOOR_MISSING = 1, RZ_BUILDINGS_PENDING = 2,
                   RZ_NOT_IN_WORLD = 3, RZ_UNKNOWN = 4, RZ_COUNT = 5 };

// Cumulative for the session. Incremented on any thread (Interlocked only),
// read on the main thread by ReadinessReportTick.
volatile LONG g_rdyCls[RC_COUNT][RZ_COUNT];
volatile LONG g_rdyBypass    = 0;   // ready answers from the sections == 0 bypass
volatile LONG g_rdyRuleReady = 0;   // rule answers, ready
volatile LONG g_rdyRuleWait  = 0;   // rule answers, not ready

// Cost of the collection scan (QPC ticks around the scan loop, all threads).
// Session totals; the max is kept with a CAS loop.
volatile LONG   g_rdyScans     = 0;
volatile LONG64 g_rdyScanTicks = 0;
volatile LONG64 g_rdyScanMax   = 0;
volatile LONG64 g_rdyScanSlots = 0;   // collection size at each scan, summed

// Off-main sampling while islandReadinessRule is off: only 1 call in
// RDY_OFF_SAMPLE_MASK + 1 is classified (both try-locks and the scan); the
// others answer as today without touching either lock. With the rule on every
// call is classified, since its answer depends on the class.
volatile LONG g_rdyOffSample   = 0;
const LONG    RDY_OFF_SAMPLE_MASK = 15;   // 1 in 16

// Sanity cap on the collection scan. The collection grows by one slot per
// concurrently loaded section and reuses empty slots; a count past this means
// we are not reading what we think we are.
const int RDY_SC_MAX_SCAN = 8192;

// boost::shared_mutex state word (boost 1.4x win32 layout, confirmed by the
// game's unlock 0x25C3D0 and unlock_shared 0x168E10): bits 0-10 shared count,
// bit 22 exclusive, bit 31 exclusive_waiting_blocked.
const LONG BOOST_SHARED_MASK = 0x7FF;
const LONG BOOST_EXCLUSIVE   = 0x400000;

// try_lock_shared, as inlined in NavMesh__snapToFace 0x3A1F70: fails when an
// exclusive owner holds it or one is waiting (sign bit), else adds a reader.
bool BoostTryLockShared(volatile LONG* state)
{
	LONG cur = *state;
	for (;;)
	{
		if ((cur & BOOST_EXCLUSIVE) != 0 || cur < 0)
			return false;
		LONG next = (cur & ~BOOST_SHARED_MASK) | ((cur + 1) & BOOST_SHARED_MASK);
		if ((next & BOOST_SHARED_MASK) == 0)
			return false;                           // reader count would overflow
		LONG prev = InterlockedCompareExchange(state, next, cur);
		if (prev == cur)
			return true;
		cur = prev;
	}
}

// try_lock, as inlined in SectionManager__isContentPending 0x3AB59D: fails
// while any reader or the exclusive owner holds it.
bool BoostTryLockExclusive(volatile LONG* state)
{
	LONG cur = *state;
	for (;;)
	{
		if ((cur & BOOST_SHARED_MASK) != 0 || (cur & BOOST_EXCLUSIVE) != 0)
			return false;
		LONG prev = InterlockedCompareExchange(state, cur | BOOST_EXCLUSIVE, cur);
		if (prev == cur)
			return true;
		cur = prev;
	}
}

// Any thread: records one scan's cost (Interlocked only).
void RecordScanCost(LONG64 ticks, int slots)
{
	if (ticks < 0)
		ticks = 0;
	InterlockedIncrement(&g_rdyScans);
	InterlockedExchangeAdd64(&g_rdyScanTicks, ticks);
	InterlockedExchangeAdd64(&g_rdyScanSlots, (LONG64)slots);
	LONG64 cur = InterlockedCompareExchange64(&g_rdyScanMax, 0, 0);
	while (ticks > cur)
	{
		LONG64 prev = InterlockedCompareExchange64(&g_rdyScanMax, ticks, cur);
		if (prev == cur)
			break;
		cur = prev;
	}
}

// Any thread: no allocation, no logging, never blocks. splitMap = false skips
// the +0x1E0 noSection/outdoorMissing split (returns RZ_NOT_IN_WORLD instead).
int ClassifyZoneReadiness(uintptr_t mgr, const int* pos, bool splitMap)
{
	if (!mgr || !pos || !fn_lookupSection || !fn_boostUnlock || !fn_boostUnlockShared)
		return RZ_UNKNOWN;
	int gx = pos[0];
	int gy = pos[1];
	if (gx < 0 || gx > ZONE_GRID_MAX || gy < 0 || gy > ZONE_GRID_MAX)
		return RZ_UNKNOWN;
	unsigned int uid = (unsigned int)gx | ((unsigned int)gy << 8);

	// 1. Outdoor instance in the world (under +0x200 shared).
	volatile LONG* worldLock = (volatile LONG*)(mgr + OFF_RDY_SM_WORLD_LOCK);
	if (!BoostTryLockShared(worldLock))
		return RZ_UNKNOWN;
	int outdoor = -1;                              // -1 unreadable, 0 not in, 1 in
	int scanSlots = -1;                            // >= 0 once a scan ran
	LARGE_INTEGER t0, t1;
	t0.QuadPart = 0;
	t1.QuadPart = 0;
	uintptr_t world = *(uintptr_t*)(mgr + OFF_RDY_SM_WORLD);
	uintptr_t coll  = world ? *(uintptr_t*)(world + OFF_RDY_WORLD_COLLECTION) : 0;
	if (coll)
	{
		int count      = *(int*)(coll + OFF_RDY_SC_COUNT);
		uintptr_t data = *(uintptr_t*)(coll + OFF_RDY_SC_DATA);
		if (count >= 0 && count <= RDY_SC_MAX_SCAN && (count == 0 || data))
		{
			QueryPerformanceCounter(&t0);
			outdoor = 0;
			for (int k = 0; k < count; ++k)
			{
				uintptr_t inst = *(uintptr_t*)(data + (uintptr_t)k * RDY_SC_ENTRY_SIZE);
				if (!inst || *(unsigned int*)(inst + OFF_RDY_NMI_SECTION_UID) != uid)
					continue;
				if (*(int*)(inst + OFF_RDY_NMI_RUNTIME_INDEX) >= 0)
					outdoor = 1;
				break;                             // first match, as 0xD0CD40
			}
			QueryPerformanceCounter(&t1);
			scanSlots = count;
		}
	}
	fn_boostUnlockShared((void*)worldLock);
	if (scanSlots >= 0)
		RecordScanCost(t1.QuadPart - t0.QuadPart, scanSlots);   // after the release
	if (outdoor < 0)
		return RZ_UNKNOWN;
	if (outdoor > 0)
		return RZ_BUILDINGS_PENDING;   // the original said 0, so the ready byte is clear

	// 2. Section entry present? (under +0x1E0 exclusive; diagnostic split only)
	// The outdoor fact is known from here on: a skipped split or a lost try
	// is "not in world", never "unknown".
	if (!splitMap)
		return RZ_NOT_IN_WORLD;
	volatile LONG* mapLock = (volatile LONG*)(mgr + OFF_RDY_SM_MAP_LOCK);
	if (!BoostTryLockExclusive(mapLock))
		return RZ_NOT_IN_WORLD;
	void* node = NULL;
	fn_lookupSection((void*)(mgr + OFF_RDY_SM_SECTION_MAP), &node, pos);
	void* head = *(void**)(mgr + OFF_RDY_SM_SECTION_MAP_HEAD);
	bool hasSection = node && node != head
	               && *(void**)((uintptr_t)node + OFF_RDY_MAPNODE_VALUE) != NULL;
	fn_boostUnlock((void*)mapLock);
	return hasSection ? RZ_OUTDOOR_MISSING : RZ_NO_SECTION;
}

// Main thread only. The "state-4 poll" class: main thread and zone manager in
// state 4, for ANY zone (controller ruling, Round 2). The zone's accessibility
// is deliberately not tested: the state machine (0xA0E950) can poll a zone
// that is already accessible, and giving that call the rule's per-zone answer
// would put an uncapped wait into the state-4 answer, which can freeze the
// game (the pause near inaccessible zones while the state is not 0). The
// accepted cost: a main-thread setDestination during state 4 keeps today's
// lenient sections == 0 answer. A flag set by a hook around
// ZoneManager__stateMachineDriver 0xA0E950 would tell the poll apart exactly
// (not done now).
bool IsStatePoll4()
{
	void* zm = g_cachedZoneMgr;
	return zm && GetZoneState(zm) == 4;
}

inline LONG ReadCounter(volatile LONG* c)
{
	return InterlockedCompareExchange(c, 0, 0);
}

} // namespace

// Main thread, every hook_updateCameraZone frame: the one-time config line,
// the deferred target line, and the periodic Readiness line (10 s DEV, 30 s
// PROD, only when a counter moved).
static void ReadinessReportTick(double now)
{
	static bool configLogged = false;
	if (!configLogged)
	{
		configLogged = true;
		std::ostringstream cs;
		cs << "[ZoneOpt] Readiness config: readinessOverrides="
		   << (readinessOverridesEnabled ? "ON" : "OFF")
		   << " islandReadinessRule=" << (islandReadinessRuleEnabled ? "ON" : "OFF")
		   << " deferral=" << (deferralEnabled ? "ON" : "OFF");
		LogMsg(cs.str());
	}

	if (InterlockedCompareExchange(&g_tgtLogPending, 0, 1) == 1)
		LogTransitionTarget();

#ifdef ZONEOPT_DEBUG
	const double kReadinessLogIntervalSec = 10.0;
#else
	const double kReadinessLogIntervalSec = 30.0;
#endif
	static double        lastLog = 0.0;
	static unsigned long lastSig = 0;
	if (now - lastLog < kReadinessLogIntervalSec)
		return;

	// Every counter only grows, so the (wrapping, unsigned) sum changes
	// whenever one of them moved.
	LONG v[RC_COUNT][RZ_COUNT];
	unsigned long sig = 0;
	for (int c = 0; c < RC_COUNT; ++c)
		for (int z = 0; z < RZ_COUNT; ++z)
		{
			v[c][z] = ReadCounter(&g_rdyCls[c][z]);
			sig += (unsigned long)v[c][z];
		}
	LONG bypass = ReadCounter(&g_rdyBypass);
	LONG rReady = ReadCounter(&g_rdyRuleReady);
	LONG rWait  = ReadCounter(&g_rdyRuleWait);
	LONG   scans     = ReadCounter(&g_rdyScans);
	LONG64 scanTicks = InterlockedCompareExchange64(&g_rdyScanTicks, 0, 0);
	LONG64 scanMax   = InterlockedCompareExchange64(&g_rdyScanMax, 0, 0);
	LONG64 scanSlots = InterlockedCompareExchange64(&g_rdyScanSlots, 0, 0);
	sig += (unsigned long)bypass + (unsigned long)rReady + (unsigned long)rWait
	     + (unsigned long)scans;
	if (sig == lastSig)
		return;                                     // nothing new to report
	lastSig = sig;
	lastLog = now;

	// Off-main calls are sampled 1 in 16 while the rule is off (see
	// g_rdyOffSample); the label says so.
	const char* offName = islandReadinessRuleEnabled ? "off" : "off(1/16)";
	const char* callerName[RC_COUNT] = { "poll4", "main", offName };

	double freq     = (double)qpcFrequency.QuadPart;
	double scanAvg  = (scans > 0 && freq > 0.0) ? (double)scanTicks * 1e6 / freq / (double)scans : 0.0;
	double scanMaxU = (freq > 0.0) ? (double)scanMax * 1e6 / freq : 0.0;
	double slotsAvg = (scans > 0) ? (double)scanSlots / (double)scans : 0.0;

	std::ostringstream ss;
	ss << "[ZoneOpt] Readiness:";
	for (int c = 0; c < RC_COUNT; ++c)
		ss << " " << callerName[c] << " ns/om/bp="
		   << v[c][RZ_NO_SECTION] << "/" << v[c][RZ_OUTDOOR_MISSING] << "/"
		   << v[c][RZ_BUILDINGS_PENDING];
	ss << " bypass=" << bypass
	   << " rule=" << ((islandReadinessRuleEnabled && readinessOverridesEnabled) ? "on" : "off")
	   << " ruleReady=" << rReady << " ruleWait=" << rWait
	   << " unk=" << v[RC_POLL4][RZ_UNKNOWN] << "/" << v[RC_MAIN][RZ_UNKNOWN] << "/"
	   << v[RC_OFF][RZ_UNKNOWN]
	   << " niw=" << v[RC_POLL4][RZ_NOT_IN_WORLD] << "/" << v[RC_MAIN][RZ_NOT_IN_WORLD] << "/"
	   << v[RC_OFF][RZ_NOT_IN_WORLD];
	ss << std::fixed << std::setprecision(1)
	   << " scan=" << scanAvg << "/" << scanMaxU << "us"
	   << std::setprecision(0) << " slots=" << slotsAvg;
	if (!readinessOverridesEnabled)
		ss << " overrides=off";
	LogMsg(ss.str());
}


// =========================================================================
// NavMesh scheduling helpers (builds context from preload/tracking state)
// =========================================================================

void BuildSchedContext(SchedContext* ctx)
{
	// Resolve camera: use transition target if active, else lastCameraGX/GY
	ctx->camX = lastCameraGX;
	ctx->camY = lastCameraGY;
	if (isTransitionActive && g_cachedZoneMgr)
	{
		void* tz = *(void**)((uintptr_t)g_cachedZoneMgr + OFF_ZM_CURRENT_ZONE);
		if (tz) { ctx->camX = GetZoneGridX(tz); ctx->camY = GetZoneGridY(tz); }
	}

	int moverCount = numWatched;
	if (moverCount > MAX_WATCHED) moverCount = MAX_WATCHED;
	for (int i = 0; i < moverCount; ++i)
	{
		ctx->movers[i].currentX = watchedChars[i].currentZoneX;
		ctx->movers[i].currentY = watchedChars[i].currentZoneY;
		ctx->movers[i].destX    = watchedChars[i].destZoneX;
		ctx->movers[i].destY    = watchedChars[i].destZoneY;
#if PATHFIND_STEP >= 5
		ctx->movers[i].hasMoveOrder = watchedChars[i].hasMoveOrder;
#endif
#if PATHFIND_STEP >= 6
		// Atomic 64-bit read of packed (gx, gy). Bg thread (findPathFull) writes
		// via InterlockedExchange64 on the same field — no torn pair.
		LONG64 packed = InterlockedCompareExchange64(
			(volatile LONG64*)&watchedChars[i].exitZonePacked, 0, 0);
		UnpackExitZone(packed, &ctx->movers[i].exitZoneGX, &ctx->movers[i].exitZoneGY);
#endif
	}
	ctx->moverCount = moverCount;

	int zoneCount = numPreloaded;
	if (zoneCount > MAX_PRELOADED) zoneCount = MAX_PRELOADED;
	for (int i = 0; i < zoneCount; ++i)
	{
		ctx->zones[i].gridX = preloadedZones[i].gridX;
		ctx->zones[i].gridY = preloadedZones[i].gridY;
	}
	ctx->zoneCount = zoneCount;
}

static void CallPrioritizeNavMeshQueue()
{
	static SchedContext ctx;   // main thread only; kept off the stack (~1.3 KB)
	BuildSchedContext(&ctx);
	PrioritizeNavMeshQueue(ctx.camX, ctx.camY, ctx.movers, ctx.moverCount,
	                       ctx.zones, ctx.zoneCount);
}

#if NMCACHE_STEP >= 4
static void BoostWorkerThreads()
{
	// Live count, not capacity: slots above it were never created.
	for (int i = 0; i < g_navMeshWorkerCount && i < NAVMESH_WORKER_COUNT; ++i)
	{
		HANDLE h = WorkerSlotHandle(i);
		if (h)
		{
			g_workerSavedPriority[i] = GetThreadPriority(h);
			SetThreadPriority(h, THREAD_PRIORITY_HIGHEST);
		}
	}
}

static void RestoreWorkerThreads()
{
	for (int i = 0; i < g_navMeshWorkerCount && i < NAVMESH_WORKER_COUNT; ++i)
	{
		HANDLE h = WorkerSlotHandle(i);
		if (h)
			SetThreadPriority(h, g_workerSavedPriority[i]);
		g_workerSavedPriority[i] = THREAD_PRIORITY_NORMAL;
	}
}
#endif // NMCACHE_STEP >= 4


// =========================================================================
// Hook 1: showLoadingMessage -- transition start/end bracket
// =========================================================================

void hook_showLoadingMessage(void* thisPtr, bool on)
{
	// A previous bracket's dismissal may still be waiting for the main thread
	// (B6). Retire it here, before the new bracket is set up, or the start
	// below would be swallowed by the still-true isTransitionActive and the
	// deferred completion would later clear the new bracket's state.
	if (on && InterlockedCompareExchange(&transitionEndPending, 0, 0) != 0)
	{
		if (IsMainThread())
		{
			// In order and complete: stats line, thread restore, preload reset.
			TransitionCompleteIfPending();
		}
		else
		{
			// Off the main thread, only allocation-free Win32 work is allowed.
			// Undo the priority boost (so the BoostNavMeshThread below does not
			// latch HIGHEST as the saved priority) and open the bracket. The
			// pending flag stays raised: the main thread will still report the
			// arrival, see the newer generation, and skip its body.
			RestoreNavMeshThread();
#if NMCACHE_STEP >= 4
			RestoreWorkerThreads();
#endif
			isTransitionActive        = false;
			prioritizedThisTransition = false;
		}
	}

	if (on && !isTransitionActive)
	{
		InterlockedIncrement(&transitionGen);
		isTransitionActive = true;
		deferredFrameCount = 0;
		QueryPerformanceCounter(&transitionStartTime);
		BoostNavMeshThread();
#if NMCACHE_STEP >= 4
		BoostWorkerThreads();
#endif

		// O11: the target 3x3 as the bracket opens, before the P3 handoff
		// below changes any of it. The capture is allocation-free. The line is
		// logged here on the main thread; off it (B6: CRT string work is not
		// allowed there) it waits for the next main-thread hook_updateCameraZone.
		if (CaptureTransitionTarget())
		{
			if (IsMainThread())
			{
				InterlockedExchange(&g_tgtLogPending, 0);
				LogTransitionTarget();
			}
			else
			{
				InterlockedExchange(&g_tgtLogPending, 1);
			}
		}

		// P3 fix: clear +176 on preloaded zones in transition grid
		// so loadZoneData re-enters them and adds to Set A
		if (preloadEnabled && numPreloaded > 0 && g_cachedZoneMgr)
			PreparePreloadedZonesForTransition(g_cachedZoneMgr);

		// Phase 2: log pre-promotion status
		if (numPreloaded > 0)
		{
			int prePromoted = 0;
			int handedOff = 0;
			for (int i = 0; i < numPreloaded; ++i)
			{
				if (preloadedZones[i].promoted)
					prePromoted++;
				if (preloadedZones[i].pipelineHandoff)
					handedOff++;
			}
			std::ostringstream ss;
			ss << "[ZoneOpt] Transition start: " << prePromoted << "/"
			   << numPreloaded << " pre-promoted, "
			   << handedOff << " handed to pipeline";
			LogMsg(ss.str());
		}
	}
	else if (!on && isTransitionActive)
	{
		// B6: the dismissal can arrive on the contentStream (path) thread, which
		// runs concurrently with the main thread. Nothing here may touch the
		// preload arrays, the thread-priority state or the logger -- LogMsg
		// builds a std::string and an ostringstream, which is CRT work this
		// thread must not do. Stamp the end time, the calling thread and the
		// bracket generation, then raise the flag; the next main-thread
		// updateCameraZone reports the arrival and runs the completion body.
		QueryPerformanceCounter(&transitionEndQpc);
		transitionEndTid = GetCurrentThreadId();
		transitionEndGen = InterlockedCompareExchange(&transitionGen, 0, 0);
		InterlockedExchange(&transitionEndPending, 1);
	}

	orig_showLoadingMessage(thisPtr, on);
}


// Deferred transition completion (B6). Main thread only; called from the top of
// hook_updateCameraZone. Claims transitionEndPending, then runs the body that
// used to sit in the dismissal branch of hook_showLoadingMessage.
void TransitionCompleteIfPending()
{
	if (InterlockedCompareExchange(&transitionEndPending, 0, 1) != 1)
		return;

	// Close the bracket before doing anything that can take time. While
	// isTransitionActive is still true, a path-thread showLoadingMessage(false)
	// takes the dismissal branch again, re-stamps the end state and re-raises
	// the flag, and the next frame would report a second "Transition:" line for
	// the same start and call ClearPreloadZones() a second time. The only work
	// ahead of the write is one interlocked read: the generation has to be
	// known first, because on the superseded path the flag belongs to the newer
	// bracket and must not be cleared at all.
	bool superseded = (transitionEndGen != InterlockedCompareExchange(&transitionGen, 0, 0));
	if (!superseded)
		isTransitionActive = false;

	// Report the arrival here rather than in the hook: this is the main thread,
	// so the CRT work in LogMsg is safe. main=1 means the dismissal itself was
	// delivered on this thread.
	{
		std::ostringstream ss;
		ss << "[ZoneOpt] Transition end arrived on tid=" << (unsigned long)transitionEndTid
		   << " main=" << ((transitionEndTid == GetCurrentThreadId()) ? 1 : 0);
		LogMsg(ss.str());
	}

	// A newer bracket opened before this completion ran (the off-main-thread
	// path in hook_showLoadingMessage). Its start already restored the thread
	// priorities and re-armed the bracket, and transitionStartTime now belongs
	// to it, so there is nothing left here to finish and nothing safe to reset.
	if (superseded)
	{
		LogMsg("[ZoneOpt] Transition: superseded by a new transition before the "
		       "completion ran; stats dropped");
		return;
	}

	RestoreNavMeshThread();
#if NMCACHE_STEP >= 4
	RestoreWorkerThreads();
#endif
	prioritizedThisTransition = false;

	double totalMs = QPCToMs(transitionStartTime, transitionEndQpc);

	std::ostringstream ss;
	ss << std::fixed << std::setprecision(1);
	ss << "[ZoneOpt] Transition: " << totalMs << " ms, "
	   << deferredFrameCount << " frames deferred"
	   << (deferralEnabled ? "" : " (deferral OFF)")
	   << (priorityBoostEnabled ? ", priority boosted" : "");
	if (preloadEnabled)
	{
		int registeredCount = 0;
		int handoffCount = 0;
		for (int i = 0; i < numPreloaded; ++i)
		{
			if (preloadedZones[i].registered)
				registeredCount++;
			if (preloadedZones[i].pipelineHandoff)
				handoffCount++;
		}
		ss << ", preloaded=" << numPreloaded
		   << ", registered=" << registeredCount
		   << ", handoff=" << handoffCount
		   << ", promoted=" << promotedCount
		   << ", charZones=" << charZonesQueued
		   << ", preloadSkipLoaded=" << preloadSkipLoaded
		   << ", regSkip=" << regSkipCount;  // fix 2b registry guard (preload.cpp), session total
		if (movementAwareEnabled)
			ss << ", orders=" << hookOrderCount
			   << ", watched=" << numWatched;
	}
	// Crash-3 diagnostic: dlIns=<main>/<other> from the inserter hook, and the
	// off-main-thread call sites the first time any are recorded.
	ss << DestroyListStatsSuffix();

	// O11: main-thread frames per zone-manager state for slow transitions.
	// The counts belong to this bracket only if their generation matches the
	// one the dismissal was stamped with.
	if (totalMs > 300.0 && g_bracketFramesGen == transitionEndGen)
	{
		bool first = true;
		for (int s = 0; s < BRACKET_STATES; ++s)
		{
			if (g_bracketFrames[s] == 0)
				continue;
			ss << (first ? " frames=" : ",") << "s" << s << ":" << g_bracketFrames[s];
			first = false;
		}
	}
	LogMsg(ss.str());

	// Reset Phase 2 preload state; preserve watchedChars for in-transit squads
	ClearPreloadZones();
	promotedCount = 0;
	charZonesQueued = 0;
	hookOrderCount = 0;
}


// =========================================================================
// Hook 2: isContentPending -- THE navmesh deferral (Phase 1)
// =========================================================================
//
// The original returns the zone's per-zone ready byte (section state +0x50,
// set by contentStream once the outdoor instance and every building
// sub-instance are in the world); its global fallback sets the byte when
// nothing is pending anywhere (sections +632 == 0, generator idle). Returns
// 1 = ready, 0 = still loading. research/navmesh_generator.md
// ("isContentPending Gate").
//
// When the original says 0 (and deferral is on):
//   - readinessOverrides=false (H15 control): 0, for every caller.
//   - islandReadinessRule=true, any caller but the state-4 poll (main thread
//     with the zone manager in state 4, any zone), outdoor fact read (class
//     not RZ_UNKNOWN): 1 iff the zone's outdoor instance is in the world (H4).
//   - otherwise (today's rule): 1 once sections == 0. The navmesh generator
//     continues in the background.
// The ready byte is never written.

// Threading: isContentPending is also reached through isZoneStillLoading
// (0x3AC810) from CharMovement::setDestination (AI evaluation), the
// spawn-check thread and character creation. The return value is computed
// identically on every thread; the side effects (queue reprioritization,
// deferred-frame accounting, LogDebug's ostringstream) run only on the main
// thread. Per-thread call/not-ready counters feed the Islands diagnostic line.

bool hook_isContentPending(void* manager, void* zonePos)
{
	bool onMainThread = IsMainThread();

	// Prioritize navmesh queue once per transition, at start of state 4.
	// By state 4, processState3 has finished registerZoneSections ->
	// contentStream has submitted all navmesh jobs for this transition.
	// Reordering now promotes the transition zone's jobs to the front.
	if (onMainThread && isTransitionActive && !prioritizedThisTransition)
	{
		CallPrioritizeNavMeshQueue();
		prioritizedThisTransition = true;
	}

	bool result = orig_isContentPending(manager, zonePos);
	int sectionCount = *(int*)((uintptr_t)manager + OFF_RDY_SM_PENDING_SECTIONS);
	IslandCountReadiness(!result, !result && sectionCount > 0);
	if (result)
		return true;

	if (!deferralEnabled)
		return false;

	// §2 classification. The caller class uses the thread and, on the main
	// thread, the zone manager state (never read off it, where the class is
	// always "off"). Main-thread calls are always classified, and so is every
	// call while the rule is on (its answer depends on the class). Off the
	// main thread with the rule off, only 1 call in 16 is classified; the other
	// 15 take neither lock and are not counted, and their answer (today's
	// rule, below) does not depend on the class.
	const int* pos = (const int*)zonePos;
	int caller = RC_OFF;
	if (onMainThread)
		caller = IsStatePoll4() ? RC_POLL4 : RC_MAIN;
	bool classify = onMainThread || islandReadinessRuleEnabled
	             || (InterlockedIncrement(&g_rdyOffSample) & RDY_OFF_SAMPLE_MASK) == 0;
	// The +0x1E0 split (diagnostic) runs on the main thread and in the sampled
	// rule-off calls; off the main thread with the rule on it is skipped, so
	// the rule path adds no contention against the game's blocking map lookups.
	bool splitMap = onMainThread || !islandReadinessRuleEnabled;
	int cls = RZ_UNKNOWN;
	if (classify)
	{
		cls = ClassifyZoneReadiness((uintptr_t)manager, pos, splitMap);
		InterlockedIncrement(&g_rdyCls[caller][cls]);
	}

	// H15 control: the original's answer for every caller. The queue
	// reprioritization above has already run.
	if (!readinessOverridesEnabled)
		return false;

	// H4 per-caller rule: every caller but the state-4 poll (main thread in
	// state 4, any zone) is ready exactly when the zone's outdoor instance is
	// in the world, whatever the global section count. noSection,
	// outdoorMissing and notInWorld all answer 0; only RZ_UNKNOWN (the outdoor
	// fact was not read) falls through to today's rule.
	if (islandReadinessRuleEnabled && classify && caller != RC_POLL4 && cls != RZ_UNKNOWN)
	{
		bool ready = (cls == RZ_BUILDINGS_PENDING);
		InterlockedIncrement(ready ? &g_rdyRuleReady : &g_rdyRuleWait);
		return ready;
	}

	if (sectionCount > 0)
		return false;
	InterlockedIncrement(&g_rdyBypass);

	// Sections drained: return ready regardless of navmesh state.
	// Macro-transitions: original Phase 1 deferral (95% faster state 4).
	// Micro-transitions (platoon activation bumps state 0->1 without
	// showLoadingMessage): prevents NavMeshGenerator idle stall from
	// causing state 4 pause. NPCs use fallback pathfinding briefly.
	if (!onMainThread)
		return true;

	if (isTransitionActive)
		deferredFrameCount++;
	else
	{
		static double lastMicroDeferLog = 0.0;
		double t = ElapsedSec();
		if (t - lastMicroDeferLog > 5.0)
		{
			LogDebug("[ZoneOpt] Micro-transition deferral: sections=0, skipping navmesh wait");
			lastMicroDeferLog = t;
		}
	}
	return true;
}


// =========================================================================
// Hook 4: addOrderSelectedCharacters -- player move order capture (Phase 3)
// =========================================================================
//
// x64 MSVC thiscall: RCX=this, RDX=destIndoors, R8=task, R9=subject,
// stack: shift, addDontClear, location (const Vector3& = float*)
//
// For task==29 (MOVE_TO), we iterate the selected characters linked list
// (same structure the original function uses), register cross-zone movers
// into the watched array with the destination zone, then call the original.
// Edge detection (~1s poll) handles actual zone preloading as characters
// approach boundaries.
//
// Selected characters linked list layout (from decompiled addOrderSelectedCharacters):
//   count:    *(uint64_t*)(thisPI + 552)
//   arrayPtr: *(uint64_t*)(thisPI + 576)
//   index:    *(uint64_t*)(thisPI + 544)
//   head:     *(node**)(arrayPtr + 8 * index)
//   iterate:  node = *(node*)*node  (next pointer at +0)
//   type:     *(int*)(node + 24)    (1 = character handle)
//   handle:   (void*)(node + 16)    (passed to resolveHandle)

void hook_addOrderSelected(void* thisPI, void* destIndoors, int task,
                            void* subject, bool shift, bool addDontClear,
                            const float* location)
{
	// Capture group BEFORE calling original
	int charsAdded = 0;

#if !defined(ZONEOPT_ZONEONLY) || PATHFIND_STEP >= 2
	// Collect all resolved characters for formation group + pending-order buffer
	uintptr_t collectedChars[MAX_FORMATION_MEMBERS];
	int collectedCount = 0;
#endif

	if (task == 29 && location && fn_resolveHandle && g_handleTable)
	{
		float destX = location[0];
		float destZ = location[2];  // Ogre: Y is up, Z is horizontal

		int destGX = -1, destGY = -1;
		bool haveDest = gridCalibrated && WorldToZoneGrid(destX, destZ, &destGX, &destGY);

		// Iterate the selected characters linked list
		uintptr_t piAddr = (uintptr_t)thisPI;
		uintptr_t count = *(uintptr_t*)(piAddr + OFF_PI_SEL_COUNT);

		if (count > 0)
		{
			uintptr_t arrayPtr = *(uintptr_t*)(piAddr + OFF_PI_SEL_ARRAY);
			uintptr_t index    = *(uintptr_t*)(piAddr + OFF_PI_SEL_INDEX);

			if (arrayPtr && index < 1024)
			{
			uintptr_t* node    = *(uintptr_t**)(arrayPtr + 8 * index);
			void* sentinel = *(void**)(gameBase + RVA_HANDLE_SENTINEL);

			int maxIter = (int)count + 16;
			int iter = 0;
			while (node)
			{
				if (++iter > maxIter) break;
				int nodeType = *(int*)((uintptr_t)node + OFF_SEL_NODE_TYPE);
				if (nodeType == 1)
				{
					// Resolve handle at node+16 to get Character*
					void* resolved = fn_resolveHandle(g_handleTable, (void*)((uintptr_t)node + OFF_SEL_NODE_HANDLE));
					uintptr_t character = (uintptr_t)resolved;

					if (character && (void*)character != sentinel)
					{
#if !defined(ZONEOPT_ZONEONLY) || PATHFIND_STEP >= 2
						// Collect for formation group + pending-order buffer
						if (collectedCount < MAX_FORMATION_MEMBERS)
							collectedChars[collectedCount++] = character;
#endif
						// Island re-issue tracker (ISLAND_STEP >= 3; stub otherwise)
						IslandNoteOrder(character, location);

						// Phase 3 preload tracking: cross-zone movers
						if (preloadEnabled && movementAwareEnabled && haveDest)
						{
							float charX = GetCharPosX(character);
							float charZ = GetCharPosZ(character);

							int curGX, curGY;
							if (WorldToZoneGrid(charX, charZ, &curGX, &curGY))
							{
								if (curGX != destGX || curGY != destGY)
								{
									uintptr_t charMov = *(uintptr_t*)(character + OFF_CHAR_MOVEMENT);
									if (charMov)
									{
										if (AddWatchedCharacter(character, charMov,
										                        destGX, destGY, curGX, curGY
#if PATHFIND_STEP >= 5
										                        , /*hasMoveOrder=*/true
#endif
										                        ))
										{
											charsAdded++;
										}
									}
								}
							}
						}
					}
				}

				// Follow next pointer at node+0
				node = (uintptr_t*)*node;
			}
			}
		}

		hookOrderCount++;

#ifndef ZONEOPT_ZONEONLY
		// One-time confirmation that scatter patch is exercised on multi-char orders
		if (scatterPatchApplied && count > 1)
		{
			static bool scatterLogOnce = false;
			if (!scatterLogOnce)
			{
				scatterLogOnce = true;
				std::ostringstream ss;
				ss << "[ZoneOpt] Scatter patch active: " << count
				   << " chars ordered to (" << std::fixed << std::setprecision(0)
				   << destX << "," << destZ << ") — all receive exact dest";
				LogMsg(ss.str());
			}
		}
#endif

		if (charsAdded > 0)
		{
			std::ostringstream ss;
			ss << "[ZoneOpt] Order captured: dest zone (" << destGX << "," << destGY
			   << ") " << charsAdded << " chars tracked";
			LogMsg(ss.str());
		}

#if PATHFIND_STEP >= 2
		// Phase 12: populate pending-order buffer with HavokCharacter* pointers.
		if (squadPathCacheEnabled && collectedCount > 0)
		{
			pendingOrderCount = 0;
			for (int c = 0; c < collectedCount && pendingOrderCount < MAX_PENDING_ORDER; ++c)
			{
				uintptr_t cm = *(uintptr_t*)(collectedChars[c] + OFF_CHAR_MOVEMENT);
				if (!cm) continue;
				uintptr_t hc = *(uintptr_t*)(cm + OFF_CMOV_HAVOK_CHAR);
				if (hc)
					pendingOrderHC[pendingOrderCount++] = hc;
			}
		}

		// Stuck recovery: store click destination per character
		if (location && collectedCount > 0)
		{
			double clickNow = ElapsedSec();
			for (int c = 0; c < collectedCount; ++c)
				StorePlayerClickDest(collectedChars[c], location, clickNow);
		}
#endif // PATHFIND_STEP >= 2

#ifndef ZONEOPT_ZONEONLY

		// Group cohesion: create formation group for arrival scatter.
		if (groupCohesionEnabled && scatterPatchApplied && collectedCount > 1)
		{
			bool allGrouped = true;
			for (int c = 0; c < collectedCount; ++c)
			{
				uintptr_t cm = *(uintptr_t*)(collectedChars[c] + OFF_CHAR_MOVEMENT);
				if (!cm || *(int*)(cm + OFF_CMOV_SPEED_MODE) != MOVESPEED_GROUPED)
				{ allGrouped = false; break; }
			}
			if (allGrouped)
				CreateFormationGroup(location, collectedChars, collectedCount);
		}
#endif // !ZONEOPT_ZONEONLY
	}
#if ISLAND_STEP >= 3
	else if (task != 29 && fn_resolveHandle && g_handleTable)
	{
		// (a) Round 2 final review, Important #1: any non-move player order
		// (attack, job, pick-up, talk, ...) drops the IslandOrder of every
		// selected character, the same way IslandNoteOrder registers one for
		// task 29 above. An IslandOrder otherwise only clears on arrival,
		// squad removal, a save load or a new MOVE order, so a later non-move
		// order (or AI combat / a knockout replacing the order underneath the
		// player) left a stale move destination active for the stopped-park
		// test (item (e)) to eventually re-issue to a character the player
		// deliberately redirected. Same selected-characters walk as the task
		// 29 branch above, gated on ISLAND_STEP so non-island builds spend
		// nothing on it.
		uintptr_t piAddr = (uintptr_t)thisPI;
		uintptr_t count = *(uintptr_t*)(piAddr + OFF_PI_SEL_COUNT);
		if (count > 0)
		{
			uintptr_t arrayPtr = *(uintptr_t*)(piAddr + OFF_PI_SEL_ARRAY);
			uintptr_t index    = *(uintptr_t*)(piAddr + OFF_PI_SEL_INDEX);
			if (arrayPtr && index < 1024)
			{
				uintptr_t* node = *(uintptr_t**)(arrayPtr + 8 * index);
				void* sentinel = *(void**)(gameBase + RVA_HANDLE_SENTINEL);
				int maxIter = (int)count + 16;
				int iter = 0;
				while (node)
				{
					if (++iter > maxIter) break;
					int nodeType = *(int*)((uintptr_t)node + OFF_SEL_NODE_TYPE);
					if (nodeType == 1)
					{
						void* resolved = fn_resolveHandle(g_handleTable, (void*)((uintptr_t)node + OFF_SEL_NODE_HANDLE));
						uintptr_t character = (uintptr_t)resolved;
						if (character && (void*)character != sentinel)
							IslandDropOrder(character);
					}
					node = (uintptr_t*)*node;
				}
			}
		}
	}
#endif // ISLAND_STEP >= 3

	orig_addOrderSelected(thisPI, destIndoors, task, subject, shift, addDontClear, location);

#if PATHFIND_STEP >= 5
	// Immediate reprio: the player just added one or more T1 destinations.
	// Existing queued jobs at those zones float to top now. New requests from
	// the AI task system land on later ticks and ride the backstop.
	if (charsAdded > 0)
	{
		CallPrioritizeNavMeshQueue();
		InterlockedIncrement(&reprioOrderFires);
	}
#endif
}


// =========================================================================
// Hook 3: updateCameraZone -- preloading + promotion trigger (Phase 2)
// =========================================================================

void hook_updateCameraZone(void* zoneMgr, void* cameraPos)
{
	// Crash-3 diagnostic (core.h). The game calls the destroy-list drain earlier
	// in this same frame (GameWorld__mainLoop_GPUSensitiveStuff calls it at
	// +0x1B0 and updateCameraZone at +0x385), so this samples the container the
	// drain has just left, one frame before a stale count would fault.
	DestroyListProbeTick();

	// B6: finish a transition whose dismissal arrived on the path thread.
	// Runs before anything else so the stats line and the preload reset happen
	// on the first main-thread frame after the loading screen goes away.
	TransitionCompleteIfPending();

	// Always call original first (visual activate/deactivate)
	orig_updateCameraZone(zoneMgr, cameraPos);

	g_cachedZoneMgr = zoneMgr;

	// O11: per-state frame count for the open bracket (main thread).
	CountBracketFrame(zoneMgr);

	// B9: drop all mod state across a save load, and do no preload work while
	// the game is loading one (every pointer we would cache is about to die).
	bool saveLoading = PreloadCheckSaveLoad(zoneMgr);

	// Crash-3 mitigation (core.h): perform the inserts that background threads
	// queued instead of writing the container themselves. It runs after the
	// probe, so the probe still reads the state the frame's drain left behind;
	// after the frame's drain (GameWorld__mainLoop_GPUSensitiveStuff calls the
	// drain at +0x1B0 and updateCameraZone at +0x385), so a deferred object is
	// destroyed by the next frame's drain; and after the save-load check, whose
	// rising edge calls DestroyListDropDeferred — a world clear must empty the
	// queue before the flush could replay a pointer the clear freed. It runs
	// even while a load is in progress, so the queue cannot sit at its cap.
	DestroyListFlushDeferred();

	// Every frame, save load included: Phase 17 Step 1 instrumentation (a
	// no-op below PATHPOOL_STEP 1; it detects save loads itself), then the
	// readiness config/target/Readiness lines (cumulative session counters,
	// no game pointers kept).
	double tickNow = ElapsedSec();
	PathPoolTickMain(tickNow);
	ReadinessReportTick(tickNow);

	if (saveLoading)
		return;

	if (!preloadEnabled)
		return;

	if (!cameraPos)
		return;

	double now = ElapsedSec();

	// One-time grid calibration for world-to-zone conversion
	if (!gridCalibrated)
		CalibrateZoneGrid(zoneMgr);

	// Compute camera grid coords once for jump detection + debug logging
	int camGX = -1, camGY = -1;
	bool haveCamGrid = false;
	if (gridCalibrated)
	{
		float camX = *(float*)cameraPos;
		float camZ = *((float*)cameraPos + 2);
		haveCamGrid = WorldToZoneGrid(camX, camZ, &camGX, &camGY);
	}

	// Squad-switch detection: sudden camera zone jump (Manhattan > 2)
	bool swapped = false;
	if (haveCamGrid && lastCameraGX >= 0 && lastCameraGY >= 0)
	{
		int jumpX = camGX - lastCameraGX;
		int jumpY = camGY - lastCameraGY;
		if (jumpX < 0) jumpX = -jumpX;
		if (jumpY < 0) jumpY = -jumpY;
		if (jumpX + jumpY > 2)
			swapped = TrySquadSwitchSwap(zoneMgr, camGX, camGY);
	}
	if (haveCamGrid)
	{
		lastCameraGX = camGX;
		lastCameraGY = camGY;
	}

	// Register + promote pending zones only outside transitions
	if (pendingCount > 0 && !isTransitionActive)
	{
		TryRegisterPreloadedZones(zoneMgr, now);
		TryPromotePreloadedZones(zoneMgr, now);
	}

	// Evict stale/unloaded zones periodically to free preload slots
	static double lastEvictTime = 0.0;
	if (numPreloaded > 0 && now - lastEvictTime > 2.0)
	{
		EvictStaleZones(zoneMgr, now);
		lastEvictTime = now;
	}

	// Periodic navmesh queue reprioritization (5-tier system).
	// Ensures character path zones and camera zones are always
	// processed first by the NavMesh background thread.
	static double lastReprioritizeTime = 0.0;
#if PATHFIND_STEP >= 5
	{
		// Flag-driven (set by bg thread STEP 6+) takes precedence; otherwise
		// fall back to cfg_reprioritizeInterval (default 1.0s, INI-tunable).
		long flag = InterlockedExchange(&g_reprioRequested, 0);
		if (flag != 0)
		{
			CallPrioritizeNavMeshQueue();
			lastReprioritizeTime = now;
			InterlockedIncrement(&reprioFlagFires);
		}
		else if (now - lastReprioritizeTime > cfg_reprioritizeInterval)
		{
			CallPrioritizeNavMeshQueue();
			lastReprioritizeTime = now;
			InterlockedIncrement(&reprioTimerFires);
		}
	}
#else
	if (now - lastReprioritizeTime > 3.0 && (numWatched > 0 || pendingCount > 0))
	{
		CallPrioritizeNavMeshQueue();
		lastReprioritizeTime = now;
	}
#endif

#if PATHFIND_STEP >= 7
	// STEP 7: 1Hz drain of preloadAhead zones stashed by bg-thread findPathFull.
	// Atomic read-and-clear via InterlockedExchange avoids races with concurrent
	// bg decode. Clears to PRELOAD_AHEAD_NONE (=-1) so next decode re-fires.
	{
		static double lastPreloadAheadDrain = 0.0;
		if (now - lastPreloadAheadDrain > 1.0)
		{
			int drained = 0;
			for (int i = 0; i < numWatched; ++i)
			{
				long gx = InterlockedExchange((volatile long*)&watchedChars[i].preloadAheadGX,
				                               PRELOAD_AHEAD_NONE);
				long gy = InterlockedExchange((volatile long*)&watchedChars[i].preloadAheadGY,
				                               PRELOAD_AHEAD_NONE);
				if (gx == PRELOAD_AHEAD_NONE || gy == PRELOAD_AHEAD_NONE) continue;
				if (gx < 0 || gx > ZONE_GRID_MAX || gy < 0 || gy > ZONE_GRID_MAX) continue;

				if (EnqueueCharacterZone((int)gx, (int)gy))
				{
					InterlockedIncrement(&preloadAheadEnqueued);
					drained++;
				}
			}
			lastPreloadAheadDrain = now;
			if (drained > 0)
			{
				std::ostringstream ss;
				ss << "[ZoneOpt] PreloadAhead drain: " << drained << " zones";
				LogDebug(ss.str());
			}
		}
	}
#endif

	// Island routing overlay: Set B signature, component rebuild, parked-squad
	// re-issue, diagnostics (main thread; hooks read the published snapshot)
	IslandTick(zoneMgr, now);

	// NavMesh cache diagnostic: report background thread job count
	LogNavMeshCacheStats(now);

	// Pathfinding diagnostic: report A* search budget stats
	LogPathfindDiagStats(now);
	LogSquadPathCacheStats(now);
	LogPhase12Stats(now);

	// Player movement state: detect stuck characters
	PollPlayerMovementState(now);

	// Multi-call path probe: dump captured entries after window expires
	DumpPathProbe(now);

	// Only preload during idle state
	int state = GetZoneState(zoneMgr);
	if (state != 0)
		return;

	// Process one zone from the priority queue per frame
	ProcessPreloadQueue(zoneMgr);

	// --- Debug: log zone under camera every ~10 seconds ---
	if (haveCamGrid && now - lastCamLogTime > CAM_LOG_INTERVAL)
	{
		lastCamLogTime = now;
		void* camZone = GetZoneEntry(zoneMgr, camGX, camGY);
		if (camZone)
		{
			int loading = IsZoneLoading(camZone) ? 1 : 0;
			int accessible = IsZoneAccessible(camZone) ? 1 : 0;
			void* content = *(void**)camZone;

			const char* ourStatus = "not-ours";
			for (int i = 0; i < numPreloaded; ++i)
			{
				if (preloadedZones[i].gridX == camGX && preloadedZones[i].gridY == camGY)
				{
					ourStatus = preloadedZones[i].pipelineHandoff ? "handoff" :
					            preloadedZones[i].promoted ? "promoted" :
					            preloadedZones[i].pending  ? "pending" : "tracked";
					break;
				}
			}

			std::ostringstream ss;
			ss << "[ZoneOpt] CamZone (" << camGX << "," << camGY << ")"
			   << " load=" << loading << " access=" << accessible
			   << " content=" << (content ? "yes" : "NULL")
			   << " status=" << ourStatus;
			LogMsg(ss.str());
		}
	}

	// --- Camera-based prediction (skip if swap just handled it) ---
	if (!swapped)
	{
		void* currentZone = *(void**)((uintptr_t)zoneMgr + OFF_ZM_CURRENT_ZONE);
		if (currentZone)
		{
			float playerX = *(float*)cameraPos;
			float playerZ = *((float*)cameraPos + 2);

			float centerX = GetZoneCenterX(currentZone);
			float centerZ = GetZoneCenterZ(currentZone);

			float dx = playerX - centerX;
			float dz = playerZ - centerZ;
			float adx = (dx < 0.0f) ? -dx : dx;
			float adz = (dz < 0.0f) ? -dz : dz;

			if (adx < PRELOAD_THRESHOLD && adz < PRELOAD_THRESHOLD)
			{
				predictedCenterX = -1;
				predictedCenterY = -1;
			}
			else
			{
				int currentX = GetZoneGridX(currentZone);
				int currentY = GetZoneGridY(currentZone);

				int targetX = currentX;
				int targetY = currentY;
				if (dx > PRELOAD_THRESHOLD) targetX = currentX + 1;
				else if (-dx > PRELOAD_THRESHOLD) targetX = currentX - 1;
				if (dz > PRELOAD_THRESHOLD) targetY = currentY + 1;
				else if (-dz > PRELOAD_THRESHOLD) targetY = currentY - 1;

				if (targetX < 0) targetX = 0;
				if (targetX > 63) targetX = 63;
				if (targetY < 0) targetY = 0;
				if (targetY > 63) targetY = 63;

				if (targetX != predictedCenterX || targetY != predictedCenterY)
				{
					predictedCenterX = targetX;
					predictedCenterY = targetY;

					// Flush stale camera predictions before enqueueing new grid
					FlushCameraQueue();

					std::ostringstream ss;
					ss << "[ZoneOpt] Preload triggered: predicting (" << targetX << "," << targetY
					   << ") from (" << currentX << "," << currentY << ")";
					LogDebug(ss.str());

					EnqueueCameraGrid(targetX, targetY);
#if PATHFIND_STEP < 8
					EnqueueAheadZones(targetX, targetY, currentX, currentY, OWNER_CAMERA);
#endif
				}
			}
		}
	}

	// --- Character polling (Phase 3 tiered or Phase 2B fallback) ---
	if (gridCalibrated)
	{
		if (movementAwareEnabled)
			TieredCharacterPoll(zoneMgr, now);
		else if (now - lastCharScanTime > CHAR_SCAN_INTERVAL)
		{
			ScanCharacterZones(zoneMgr);
			lastCharScanTime = now;
		}
	}

#ifndef ZONEOPT_ZONEONLY
	// --- Group cohesion: poll formation groups for arrival scatter ---
	if (groupCohesionEnabled && scatterPatchApplied)
		PollFormationGroups();
#endif
}
