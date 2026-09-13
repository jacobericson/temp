// pathfind_hooks.cpp -- 6 pathfinding hook implementations with PATHFIND_STEP gating
//
// Step 1: csFindPath (count), csCheckFaceConn (count+orig), findPathFull (count+status),
//         requestPath (player/NPC count)
// Step 2: +requestPath (formation match+tier), +pathReqSubmit (priority override)
// Step 3: +csCheckFaceConn (bypass: return 1)
// Step 4: +csFindPath (offset derivation+probe), +csCheckFaceConn (face key+probe),
//         +findPathFull (cache logic+probe), +csFindPathFallback (slot tagging)
//
// FULL builds only (excluded by ZONEOPT_ZONEONLY via config.h inline stubs).

#include "pathfind_diag.h"
#include "pathfind_cache.h"
#include "path_pool.h"

#if PATHFIND_STEP >= 6
#include "tracking.h"
#include "navmesh_sched.h"
#include "grid.h"       // WorldToZoneGrid
#include <xmmintrin.h>  // __m128
#endif

#if PATHFIND_STEP >= 1

// Phase 17 Step 1 (research/path_worker_pool.md §7): request pointer recovered
// by hook_csFindPath, for whichever request hook_findPathFull runs next on
// this thread, so its PathSearchSample can carry playerByReq (*(int*)(req+0x2C)
// >= 20) without waiting for Step 2's request-derived tag rework. Round 2
// final review Important #2: OFF_REQ_RESULTBUF_SLOT (game.h) moved out of the
// PATHFIND_STEP >= 6 block it used to live in -- hook_csFindPath installs at
// step >= 1 (main.cpp) and the dispatcher passes req+128 as resultBuf at
// every step, so this tag (and the playerByReq/disagree= diagnostics it
// feeds) is live from step 1 too; previously every boosted sample below step
// 6 read back as "unk" and disagree= was 0 by construction. Thread-local for
// the same reason as currentRequestIsPlayer below: findPathFull has 6 callers
// and only the csFindPath -> fallback -> findPathFull chain shares a thread.
// hook_findPathFull consumes and clears it, so a call that did not come
// through hook_csFindPath first (the gate pass calls findPathFull directly)
// reports playerByReq = -1.
static __declspec(thread) void* g_pathPoolLastCsFindPathReq = NULL;

#if PATHFIND_STEP >= 2
// BG thread tag: set by hook_csFindPath, read by hook_findPathFull.
// Thread-local: findPathFull has 6 callers; only the csFindPathFallback chain runs
// on the same contentStream bg thread as csFindPath. Game-internal findPathFull calls
// on other bg threads would otherwise see stale flags from another thread.
static __declspec(thread) bool currentRequestIsPlayer = false;
#endif


// =========================================================================
// STEP 6: HavokCharacter* identification chain + ExitFace decode state
// =========================================================================
#if PATHFIND_STEP >= 6

struct ReqCharMapEntry {
	void*     reqObj;     // NULL = empty slot
	uintptr_t havokChar;
	double    insertTime;
};

static const int REQ_CHAR_MAP_CAPACITY = 128;
static ReqCharMapEntry reqCharMap[REQ_CHAR_MAP_CAPACITY];
static CRITICAL_SECTION reqCharMapCS;
static bool reqCharMapCSInit = false;

// Main thread only: set by hook_requestPath, consumed by hook_pathReqSubmit.
// Both run sequentially on the main thread in the same call stack -- plain static safe.
static uintptr_t currentReqChar = 0;

// Bg thread: set by hook_csFindPathFallback, consumed by hook_findPathFull.
// findPathFull has 6 callers; only the csFindPathFallback chain runs inline on
// the same thread. Game-internal findPathFull calls on other bg threads would
// otherwise consume reqCharMap entries intended for the contentStream bg thread.
// Thread-local storage ensures each thread sees its own NULL unless that thread's
// own csFindPathFallback set it.
static __declspec(thread) void* currentBgReq = NULL;

// One-time thread-ID assertion flags (CAS-gated, fire once per hook per session)
static volatile long threadAssertReqPath  = 0;
static volatile long threadAssertSubmit   = 0;
static volatile long threadAssertFallback = 0;
static volatile long threadAssertFindFull = 0;
static volatile long resultBufOffsetValidated = 0;

// STEP 6 counters (folded into LogPhase12Stats)
volatile long exitFaceDecodes          = 0;
volatile long exitFaceDecodesUnchanged = 0;
volatile long exitFaceFailures         = 0;
volatile long exitFaceDecodeErrors     = 0;
volatile long exitFaceSCRace           = 0;
volatile long reqCharMapInserts        = 0;
volatile long reqCharMapLookupHits     = 0;
volatile long reqCharMapLookupMiss     = 0;
volatile long reqCharMapOverflows      = 0;
volatile long reqCharMapHighWater      = 0;
volatile long reqCharMapDirectPrune    = 0;
volatile long reqCharMapSuperseded     = 0;
volatile long npcRequestsSkipped       = 0;

#if PATHFIND_STEP >= 7
// STEP 7: Formation dedup + preload drain + eviction counters
volatile long formationDedupHits    = 0;
volatile long formationPropagations = 0;
volatile long preloadAheadEnqueued  = 0;
volatile long watchedEvictions      = 0;
#endif

#if PATHFIND_STEP >= 9
// STEP 9 (final): extraction-SEH + stability-gate counters
volatile long extractionCrashRescue = 0;  // SEH caught AV in contentStreamCallee_0x8869
volatile long spcStabilityHold      = 0;  // injection skipped because addInstance recent
volatile long addInstanceHookCalls  = 0;  // sanity: our addInstance hook is firing

// Last addInstance timestamp as raw QPC ticks (atomic on x64 for 8-byte aligned).
// Written by navmesh bg thread (hook_addInstance), read by contentStream bg thread
// (hook_findPathFull injection gate).
volatile LONG64 g_lastAddInstanceQPC = 0;
#endif


void InitReqCharMap()
{
	if (reqCharMapCSInit) return;
	InitializeCriticalSection(&reqCharMapCS);
	for (int i = 0; i < REQ_CHAR_MAP_CAPACITY; ++i) {
		reqCharMap[i].reqObj     = NULL;
		reqCharMap[i].havokChar  = 0;
		reqCharMap[i].insertTime = 0.0;
	}
	reqCharMapCSInit = true;
}

// Linear scan -- cache-friendly, simpler than open-addressing + tombstones
// (which break probe chains on delete).
//
// Per-havokChar dedup: before inserting a new request, evict any existing
// entries for the same havokChar. Older in-flight requests are superseded by
// the new one (AI setDestination replaces path). Only the latest request's
// ExitFace decode matters (writes to watchedChars[i].exitZonePacked, a single
// slot). Without this, rapid re-paths or formation bursts leave stale entries
// competing for map capacity.
static void ReqCharMapInsert(void* reqObj, uintptr_t havokChar)
{
	if (!reqCharMapCSInit || !reqObj || !havokChar) return;
	EnterCriticalSection(&reqCharMapCS);
	int firstFree = -1;
	int oldestIdx = 0;
	int occupied  = 0;
	double t = ElapsedSec();

	// Pass 1: evict stale entries for this havokChar (any entry with matching
	// havokChar but a different reqObj pointer is superseded by the new insert).
	for (int i = 0; i < REQ_CHAR_MAP_CAPACITY; ++i) {
		if (reqCharMap[i].reqObj != NULL
		    && reqCharMap[i].reqObj != reqObj
		    && reqCharMap[i].havokChar == havokChar) {
			reqCharMap[i].reqObj     = NULL;
			reqCharMap[i].havokChar  = 0;
			reqCharMap[i].insertTime = 0.0;
			InterlockedIncrement(&reqCharMapSuperseded);
		}
	}

	// Pass 2: dedup on reqObj, find firstFree + oldest for placement.
	for (int i = 0; i < REQ_CHAR_MAP_CAPACITY; ++i) {
		if (reqCharMap[i].reqObj == reqObj) {
			// Dedup -- overwrite and exit
			reqCharMap[i].havokChar  = havokChar;
			reqCharMap[i].insertTime = t;
			LeaveCriticalSection(&reqCharMapCS);
			return;
		}
		if (reqCharMap[i].reqObj == NULL && firstFree < 0) firstFree = i;
		else if (reqCharMap[i].reqObj != NULL) {
			occupied++;
			if (reqCharMap[i].insertTime < reqCharMap[oldestIdx].insertTime)
				oldestIdx = i;
		}
	}
	int slot = (firstFree >= 0) ? firstFree : oldestIdx;
	if (firstFree < 0) InterlockedIncrement(&reqCharMapOverflows);
	reqCharMap[slot].reqObj     = reqObj;
	reqCharMap[slot].havokChar  = havokChar;
	reqCharMap[slot].insertTime = t;
	int newOccupied = occupied + (firstFree >= 0 ? 1 : 0);
	LeaveCriticalSection(&reqCharMapCS);
	InterlockedIncrement(&reqCharMapInserts);
	long prev;
	do {
		prev = InterlockedCompareExchange(&reqCharMapHighWater, 0, 0);
		if (newOccupied <= prev) break;
	} while (InterlockedCompareExchange(&reqCharMapHighWater, newOccupied, prev) != prev);
}

static uintptr_t ReqCharMapLookupAndDelete(void* reqObj)
{
	if (!reqCharMapCSInit || !reqObj) return 0;
	EnterCriticalSection(&reqCharMapCS);
	for (int i = 0; i < REQ_CHAR_MAP_CAPACITY; ++i) {
		if (reqCharMap[i].reqObj == reqObj) {
			uintptr_t result = reqCharMap[i].havokChar;
			reqCharMap[i].reqObj     = NULL;
			reqCharMap[i].havokChar  = 0;
			reqCharMap[i].insertTime = 0.0;
			LeaveCriticalSection(&reqCharMapCS);
			InterlockedIncrement(&reqCharMapLookupHits);
			return result;
		}
	}
	LeaveCriticalSection(&reqCharMapCS);
	InterlockedIncrement(&reqCharMapLookupMiss);
	return 0;
}

// Silent delete: removes entry if present. Used when csFindPath succeeds
// (direct path, findPathFull won't fire, so the entry would otherwise orphan).
// Counted separately from hit/miss since it's a proactive cleanup, not a lookup.
static void ReqCharMapPrune(void* reqObj)
{
	if (!reqCharMapCSInit || !reqObj) return;
	EnterCriticalSection(&reqCharMapCS);
	for (int i = 0; i < REQ_CHAR_MAP_CAPACITY; ++i) {
		if (reqCharMap[i].reqObj == reqObj) {
			reqCharMap[i].reqObj     = NULL;
			reqCharMap[i].havokChar  = 0;
			reqCharMap[i].insertTime = 0.0;
			LeaveCriticalSection(&reqCharMapCS);
			InterlockedIncrement(&reqCharMapDirectPrune);
			return;
		}
	}
	LeaveCriticalSection(&reqCharMapCS);
}


// ExitFace decode + watchedChars[] write. Called on bg thread from
// hook_findPathFull after orig A* completes successfully.
//
// STEP 7 restructure: watched-entry lookup moved to PHASE 1 so formation dedup
// gate (PHASE 2) can skip the walker when slot cache is fresh. Walker (PHASE 3)
// only runs on non-dedup path. PHASE 4 writes this entry + preloadAhead.
// PHASE 5 propagates to slot + formation members (timestamp-compared to avoid
// regressing followers whose own walk beat this one).
static void DecodeExitFaceAndUpdate(void* streamingCollection,
                                    void* searchState,
                                    void* findPathOutput,
                                    uintptr_t havokChar)
{
	if (!fn_faceToVertices) return;

	// ===== PHASE 1: locate watched entry =====
	// Snapshot numWatched once to bound scan against concurrent main-thread Remove.
	int localNum = numWatched;
	if (localNum > MAX_WATCHED) localNum = MAX_WATCHED;

	int watchedIdx = -1;
	for (int i = 0; i < localNum; ++i) {
		uintptr_t cm = watchedChars[i].charMovement;
		if (!cm) continue;
		uintptr_t hc = *(uintptr_t*)(cm + OFF_CMOV_HAVOK_CHAR);
		if (hc == havokChar) { watchedIdx = i; break; }
	}
	if (watchedIdx < 0) return;  // char not in watched array (downgrade race)

	double now = ElapsedSec();

#if PATHFIND_STEP >= 7
	// ===== PHASE 2: formation dedup gate =====
	int gid = watchedChars[watchedIdx].formationGroupId;  // volatile atomic load
	if (gid >= 0 && gid < MAX_FORMATION_GROUPS) {
		double slotTime = spcSlots[gid].formationExitUpdateTime;  // volatile load
		if (slotTime > 0.0 && now - slotTime < 0.1) {
			int cachedGX = spcSlots[gid].formationExitGX;
			int cachedGY = spcSlots[gid].formationExitGY;
			if (cachedGX >= 0) {
				// Apply cached decode to this follower; skip the walker.
				LONG64 newPacked = PackExitZone(cachedGX, cachedGY);
				LONG64 oldPacked = InterlockedExchange64(
					(volatile LONG64*)&watchedChars[watchedIdx].exitZonePacked,
					newPacked);
				watchedChars[watchedIdx].exitFaceUpdateTime = now;

				// Preload the zone past ExitFace for this follower.
				int curGX = watchedChars[watchedIdx].currentZoneX;
				int curGY = watchedChars[watchedIdx].currentZoneY;
				int dirX = (cachedGX > curGX) ? 1 : ((cachedGX < curGX) ? -1 : 0);
				int dirY = (cachedGY > curGY) ? 1 : ((cachedGY < curGY) ? -1 : 0);
				if (dirX != 0 || dirY != 0) {
					watchedChars[watchedIdx].preloadAheadGX = cachedGX + dirX;
					watchedChars[watchedIdx].preloadAheadGY = cachedGY + dirY;
					watchedChars[watchedIdx].preloadAheadUpdateTime = now;
				}

				InterlockedIncrement(&formationDedupHits);
				if (oldPacked != newPacked)
					InterlockedExchange(&g_reprioRequested, 1);
				return;
			}
		}
	}
#endif

	// ===== PHASE 3: walker (STEP 6 decode logic) =====
	unsigned char status = *(unsigned char*)((uintptr_t)findPathOutput + 60);
	if (status != 1) {
		InterlockedIncrement(&exitFaceFailures);
		return;
	}

	void* edgeData = *(void**)((uintptr_t)findPathOutput + 16);
	int   edgeCnt  = *(int*)((uintptr_t)findPathOutput + 24);
	if (!edgeData || edgeCnt <= 0 || edgeCnt > 200000) {
		InterlockedIncrement(&exitFaceFailures);
		return;
	}

	// Start face section comes from FindPathInput (searchState+48 = m_startFaceKey).
	// The edges array contains faces traversed THROUGH (not including start face).
	unsigned int startFaceKey = *(unsigned int*)((uintptr_t)searchState + 48);
	unsigned int startSection = startFaceKey >> 22;

	int   scCount = *(int*)((uintptr_t)streamingCollection + OFF_SC_INSTANCES_COUNT);
	void* scBase  = *(void**)((uintptr_t)streamingCollection + OFF_SC_INSTANCES_BASE);
	if (!scBase || scCount <= 0) {
		InterlockedIncrement(&exitFaceDecodeErrors);
		return;
	}

	unsigned int* edges = (unsigned int*)edgeData;
	unsigned int exitKey = 0;
	for (int e = 0; e < edgeCnt; ++e) {
		if ((edges[e] >> 22) != startSection) {
			exitKey = edges[e];
			break;
		}
	}
	if (exitKey == 0) {
		InterlockedIncrement(&exitFaceFailures);  // single-zone path
		return;
	}

	unsigned int sectionId = exitKey >> 22;
	unsigned int faceIdx   = exitKey & 0x3FFFFF;
	if ((int)sectionId >= scCount) {
		InterlockedIncrement(&exitFaceDecodeErrors);
		return;
	}

	// Race guard: scInstancesCount changed mid-walk = section table mutated.
	int scCount2 = *(int*)((uintptr_t)streamingCollection + OFF_SC_INSTANCES_COUNT);
	if (scCount2 != scCount) {
		InterlockedIncrement(&exitFaceSCRace);
		return;
	}

	void* instance = *(void**)((char*)scBase + INSTANCEINFO_SIZE * sectionId);
	if (!instance) {
		InterlockedIncrement(&exitFaceDecodeErrors);
		return;
	}

	// Resolve face -> two edge endpoints (worldspace __m128). Scratch arg is
	// uninitialized qword in observed call sites -- pass 0 explicitly.
	__m128 vA, vB;
	fn_faceToVertices(instance, (int)faceIdx, &vA, &vB, (uintptr_t)0);

	// Edge midpoint -- close enough for zone-grid mapping (zones are huge vs faces).
	float cx = (vA.m128_f32[0] + vB.m128_f32[0]) * 0.5f;
	float cz = (vA.m128_f32[2] + vB.m128_f32[2]) * 0.5f;

	int gx, gy;
	if (!WorldToZoneGrid(cx, cz, &gx, &gy)) {
		InterlockedIncrement(&exitFaceDecodeErrors);
		return;
	}

	// ===== PHASE 4: write this entry + preloadAhead =====
	LONG64 newPacked = PackExitZone(gx, gy);
	LONG64 oldPacked = InterlockedExchange64(
		(volatile LONG64*)&watchedChars[watchedIdx].exitZonePacked, newPacked);
	watchedChars[watchedIdx].exitFaceUpdateTime = now;
	InterlockedIncrement(&exitFaceDecodes);

#if PATHFIND_STEP >= 7
	{
		int curGX = watchedChars[watchedIdx].currentZoneX;
		int curGY = watchedChars[watchedIdx].currentZoneY;
		int dirX = (gx > curGX) ? 1 : ((gx < curGX) ? -1 : 0);
		int dirY = (gy > curGY) ? 1 : ((gy < curGY) ? -1 : 0);
		if (dirX != 0 || dirY != 0) {
			watchedChars[watchedIdx].preloadAheadGX = gx + dirX;
			watchedChars[watchedIdx].preloadAheadGY = gy + dirY;
			watchedChars[watchedIdx].preloadAheadUpdateTime = now;
		}
	}

	// ===== PHASE 5: formation propagation =====
	if (gid >= 0 && gid < MAX_FORMATION_GROUPS) {
		// Write leader's decode to slot cache. Followers will pick it up via
		// PHASE 2 dedup gate when their own findPathFull runs. We do NOT
		// propagate to other watched chars' fields here — that extends the
		// bg-thread work window enough to race with navmesh workers'
		// addInstance/SortedArray__grow, which reallocates m_instances and
		// causes game code to crash reading stale InstanceInfo slots.
		// The dedup gate is equivalent functionally (followers get the data
		// when their request arrives), without the extra bg write pressure.
		spcSlots[gid].formationExitGX         = gx;
		spcSlots[gid].formationExitGY         = gy;
		spcSlots[gid].formationExitUpdateTime = now;
	}
#endif

	// Set reprio flag if this char's exit zone changed. STEP 7 squad cache
	// injection writes same packed to N members; only value-change fires the flag.
	if (oldPacked != newPacked) {
		InterlockedExchange(&g_reprioRequested, 1);
	} else {
		InterlockedIncrement(&exitFaceDecodesUnchanged);
	}
}

#endif // PATHFIND_STEP >= 6


// =========================================================================
// Hook 1: ContentStream::findPath (primary "direct path" check)
// =========================================================================
// RVA 0x3AA950. Called for every path request. Returns non-zero on success.
// Runs on contentStream bg thread.

char hook_csFindPath(void* manager, unsigned int startFaceKey, void* startPos,
                      void* destPos, float radius, char param5, void* resultBuf)
{
	InterlockedIncrement(&diagPrimaryAttempts);

	char result = orig_csFindPath(manager, startFaceKey, startPos, destPos,
	                               radius, param5, resultBuf);

	if (result)
		InterlockedIncrement(&diagPrimarySuccess);
	else
		InterlockedIncrement(&diagPrimaryFail);

	// Phase 17 Step 1: tag this request for whichever hook_findPathFull call
	// runs next on this thread (playerByReq). Set on every call, success or
	// failure -- on success no findPathFull call follows for this request, so
	// the tag is simply overwritten by the next request's csFindPath call (or
	// consumed and cleared by an unrelated findPathFull call in between).
	// Round 2 final review Important #2: live from step >= 1 now (game.h moved
	// OFF_REQ_RESULTBUF_SLOT out of its old PATHFIND_STEP >= 6 block); the game
	// passes req+128 as resultBuf at every step hook_csFindPath is installed.
	g_pathPoolLastCsFindPathReq = resultBuf ? (void*)((char*)resultBuf - OFF_REQ_RESULTBUF_SLOT) : NULL;

#if PATHFIND_STEP >= 6
	// Direct-path success: findPathFull won't run for this request, so any
	// reqCharMap entry for it would orphan. Prune now. Same resultBuf->requestObj
	// recovery as hook_csFindPathFallback (dispatcher passes req+128 as resultBuf).
	// reqCharMap itself is step >= 6 only (ExitFace identification chain), so
	// this part stays gated even though the tag above no longer is.
	if (result && resultBuf) {
		void* reqObj = (void*)((char*)resultBuf - OFF_REQ_RESULTBUF_SLOT);
		ReqCharMapPrune(reqObj);
	}
#endif

#if PATHFIND_STEP >= 2
	// Player request tagging: boosted requests (pri 50+) sort to top of queue.
	// Decrement counter to tag sequential requests as player-owned.
	currentRequestIsPlayer = false;
	if (InterlockedCompareExchange(&playerRequestsInFlight, 0, 0) > 0)
	{
		InterlockedDecrement(&playerRequestsInFlight);
		currentRequestIsPlayer = true;
	}
#endif

#if PATHFIND_STEP >= 4
	// Squad path cache: derive streaming offset from destPos per slot.
	// destPos is offset-adjusted by contentStream. candidateOffset = destPos - signalDest.
	if (!result)
	{
		float* dp = (float*)destPos;
		for (int s = 0; s < MAX_FORMATION_GROUPS; ++s)
		{
			if (spcSlots[s].state == 1 || spcSlots[s].state == 4)
			{
				spcSlotCandOffX[s] = dp[0] - spcSlots[s].expGoalX;
				spcSlotCandOffZ[s] = dp[2] - spcSlots[s].expGoalZ;
			}
		}
	}

	// Multi-call probe: capture csFindPath positions when armed
	if (InterlockedCompareExchange(&pathProbeArmed, 0, 0) == 1)
	{
		long idx = InterlockedIncrement(&pathProbeWriteIdx) - 1;
		if (idx < PATH_PROBE_SIZE)
		{
			float* sp = (float*)startPos;
			float* dp = (float*)destPos;
			pathProbeBuf[idx].startX = sp[0];
			pathProbeBuf[idx].startY = sp[1];
			pathProbeBuf[idx].startZ = sp[2];
			pathProbeBuf[idx].destX  = dp[0];
			pathProbeBuf[idx].destY  = dp[1];
			pathProbeBuf[idx].destZ  = dp[2];
			pathProbeBuf[idx].hookType = 1;
			pathProbeBuf[idx].result   = result ? 1 : 0;
			pathProbeBuf[idx].faceKey  = startFaceKey;
		}
	}
#endif // PATHFIND_STEP >= 4

	return result;
}


// =========================================================================
// Hook 2: ContentStream::checkFaceConnectivity
// =========================================================================
// RVA 0x3A5B00. Called when primary findPath fails. Returns true if start
// and dest faces are in connected navmesh sections.
// Runs on contentStream bg thread.

char hook_csCheckFaceConn(void* manager, unsigned int startFace, unsigned int destFace)
{
	InterlockedIncrement(&diagConnAttempts);

#if PATHFIND_STEP >= 4
	// Multi-call probe
	if (InterlockedCompareExchange(&pathProbeArmed, 0, 0) == 1)
	{
		long idx = InterlockedIncrement(&pathProbeWriteIdx) - 1;
		if (idx < PATH_PROBE_SIZE)
		{
			pathProbeBuf[idx].startX = 0; pathProbeBuf[idx].startY = 0; pathProbeBuf[idx].startZ = 0;
			pathProbeBuf[idx].destX  = 0; pathProbeBuf[idx].destY  = 0; pathProbeBuf[idx].destZ  = 0;
			pathProbeBuf[idx].hookType = 3;
			pathProbeBuf[idx].result   = 1;
			pathProbeBuf[idx].faceKey  = destFace;
		}
	}

	// Face key pass-through for squad path cache
	spcLastConnStartFace = startFace;
	spcLastConnDestFace  = destFace;
#endif // PATHFIND_STEP >= 4

#if PATHFIND_STEP >= 3
	// Bypass the cluster graph connectivity pre-check entirely.
	// NPCs going through cluster-graph traversal hit sub_140DA4470 which derefs
	// m_instances[sec]+16 and races with addInstance (observed crash at 0xDA44D5).
	// Unconditional bypass closes that vector. Cluster graph is an optimization
	// (A* would reach the same conclusion) so correctness is preserved.
	return 1;
#else
	// Steps 1-2: call original, count failures
	char result = orig_csCheckFaceConn(manager, startFace, destFace);
	if (!result)
	{
		InterlockedIncrement(&diagConnFail);
#if PATHFIND_STEP >= 2
		if (currentRequestIsPlayer)
		{
			long idx = InterlockedIncrement(&playerFailWriteIdx) - 1;
			int slot = (int)(idx % PLAYER_FAIL_RING);
			playerFailRing[slot].goalX = 0;
			playerFailRing[slot].goalZ = 0;
			InterlockedExchange(&playerFailRing[slot].status, 99);  // 99 = connectivity rejection
			InterlockedExchange(&playerFailRing[slot].cause, 0);
			InterlockedExchange(&playerFailRing[slot].iterCount, 0);
			InterlockedExchange(&playerFailRing[slot].valid, 1);
		}
#endif
	}
	return result;
#endif
}


// =========================================================================
// Hook 3: Havok::findPathFull (hkaiPathfindingUtil::findPath)
// =========================================================================
// RVA 0xCE56D0. Full A* search. Output: +48=iterations, +60=status, +61=cause.
// Runs on contentStream bg thread.

void hook_findPathFull(void* streamingCollection, void* searchState, void* findPathOutput)
{
	InterlockedIncrement(&diagAstarAttempts);

	// Phase 17 Step 1 (research/path_worker_pool.md §7): resolve playerByReq
	// once per call from whichever request hook_csFindPath tagged just before
	// on this thread (cleared so an unrelated call, e.g. the gate pass calling
	// findPathFull directly, reports -1), track whether this call's search
	// budget gets boosted, and take the "before" QPC immediately ahead of
	// whichever branch below calls orig_findPathFull. All three converge on
	// the shared status/cause/iterCount block below, which takes the "after"
	// QPC and calls PathPoolNoteSearch -- exactly once per invocation, on
	// every thread, no allocation, no logging.
	// Round 2 final review Important #2: live from step >= 1 (see
	// g_pathPoolLastCsFindPathReq's declaration comment above).
	int pathPoolPlayerByReq = -1;
	if (g_pathPoolLastCsFindPathReq) {
		pathPoolPlayerByReq = (*(int*)((uintptr_t)g_pathPoolLastCsFindPathReq + 0x2C) >= 20) ? 1 : 0;
		g_pathPoolLastCsFindPathReq = NULL;
	}
	int pathPoolBoosted = 0;
	LARGE_INTEGER pathPoolQpcBefore;
	pathPoolQpcBefore.QuadPart = 0;

	// One-time FindPathInput layout probe
	if (searchState && !InterlockedCompareExchange(&probeFPIDumped, 1, 0))
	{
		uintptr_t ss = (uintptr_t)searchState;

		float* sp = (float*)(ss + 16);
		for (int i = 0; i < 4; ++i)
			probeStartPos[i] = sp[i];

		float** goalPtrAddr = (float**)(ss + 32);
		float* goalPtr = *goalPtrAddr;
		if (goalPtr)
		{
			for (int i = 0; i < 4; ++i)
				probeGoalPos[i] = goalPtr[i];
			InterlockedExchange(&probeGoalPtrValid, 1);
		}
		else
		{
			for (int i = 0; i < 4; ++i)
				probeGoalPos[i] = 0.0f;
			InterlockedExchange(&probeGoalPtrValid, 0);
		}

		int offsets[14] = { 40, 44, 48, 52, 56, 60, 64, 68,
		                    72, 76, 128, 136, 156, 160 };
		for (int i = 0; i < 14; ++i)
			probeFields[i] = *(int*)(ss + offsets[i]);

		InterlockedExchange(&probeFPIDumped, 2);
	}

#if PATHFIND_STEP >= 2
	// Boost A* budget for player characters to prevent SEARCH_STATE_FULL stalls.
	// Player requests tagged by hook_csFindPath via playerRequestsInFlight counter.
	if (currentRequestIsPlayer && searchState)
	{
		uintptr_t ss = (uintptr_t)searchState;
		*(int*)(ss + 156) = 131072 * 4;   // open set: 512KB (default 128KB)
		*(int*)(ss + 160) = 590336 * 4;   // search state: ~2.3MB (default 590KB)
		pathPoolBoosted = 1;
	}
#endif

#if PATHFIND_STEP >= 4
	// Multi-call probe: capture FindPathInput positions before calling orig
	long probeSlot = -1;
	if (InterlockedCompareExchange(&pathProbeArmed, 0, 0) == 1 && searchState)
	{
		probeSlot = InterlockedIncrement(&pathProbeWriteIdx) - 1;
		if (probeSlot < PATH_PROBE_SIZE)
		{
			uintptr_t ss = (uintptr_t)searchState;
			float* sp = (float*)(ss + 16);
			float** gpa = (float**)(ss + 32);
			float* gp = *gpa;

			pathProbeBuf[probeSlot].startX = sp[0];
			pathProbeBuf[probeSlot].startY = sp[1];
			pathProbeBuf[probeSlot].startZ = sp[2];
			if (gp)
			{
				pathProbeBuf[probeSlot].destX = gp[0];
				pathProbeBuf[probeSlot].destY = gp[1];
				pathProbeBuf[probeSlot].destZ = gp[2];
			}
			pathProbeBuf[probeSlot].hookType = 2;
			pathProbeBuf[probeSlot].result   = 0;
			pathProbeBuf[probeSlot].faceKey  = *(unsigned int*)(ss + 48);
		}
	}

#ifdef ZONEOPT_SQUAD_CACHE
	// =================================================================
	// Squad path cache: multi-slot leader detection, A* boost, caching, injection
	// H5: this whole block (leader detection, caching, injection) only exists
	// when ZONEOPT_SQUAD_CACHE is defined -- no shipped build defines it. The
	// Report 4 CTD lives in here (research/squad_path_cache_crash.md).
	// =================================================================
	if (squadPathCacheEnabled && searchState)
	{
		uintptr_t ss = (uintptr_t)searchState;
		uintptr_t fpo = (uintptr_t)findPathOutput;

		float* goalPtr = *(float**)(ss + 32);
		float goalX = 0, goalY = 0, goalZ = 0;
		if (goalPtr) { goalX = goalPtr[0]; goalY = goalPtr[1]; goalZ = goalPtr[2]; }

		double now = ElapsedSec();

		// Check for new signals (scan all slots)
		for (int s = 0; s < MAX_FORMATION_GROUPS; ++s)
		{
			if (InterlockedCompareExchange(&squadPathSignals[s].active, 0, 0) == 1)
			{
				SpcFreeSlot(s);
				spcSlots[s].state = 1;
				spcSlots[s].expGoalX = squadPathSignals[s].destHavokX;
				spcSlots[s].expGoalY = squadPathSignals[s].destHavokY;
				spcSlots[s].expGoalZ = squadPathSignals[s].destHavokZ;
				spcSlots[s].memberCount = squadPathSignals[s].memberCount;
				spcSlots[s].activatedTime = squadPathSignals[s].signalTime;
				spcSlots[s].servedCount = 0;
				spcSlots[s].goalFaceKey = 0;
				InterlockedExchange(&squadPathSignals[s].active, 0);
				InterlockedIncrement(&spcDiagSignals);
			}
		}

		// Timeouts + burst timeout (scan all slots)
		for (int s = 0; s < MAX_FORMATION_GROUPS; ++s)
		{
			if (spcSlots[s].state == 0) continue;
			double age = now - spcSlots[s].activatedTime;
			double timeout = (spcSlots[s].state >= 4) ? SPC_REPATH_TIMEOUT : SPC_TIMEOUT;
			if (age > timeout)
			{
				InterlockedIncrement(&spcDiagExpired);
				SpcResetSlot(s);
				continue;
			}
			if (spcSlots[s].state == 6 && now - spcSlots[s].lastInjectionTime > SPC_BURST_TIMEOUT)
			{
				SpcFreeSlot(s);
				spcSlots[s].state = 4;
				spcSlots[s].burstInjected = 0;
			}
		}

		// Read Phase 12 fallback tag (set by hook_findPathFallback, encodes slot+1)
		int taggedSlot = spcFallbackTag - 1;

		// === Process the tagged slot ===
		if (taggedSlot >= 0 && taggedSlot < MAX_FORMATION_GROUPS)
		{
			SquadPathCacheSlot& slot = spcSlots[taggedSlot];

			// AWAITING_LEADER: leader detection + A* boost + cache
			if (slot.state == 1 && goalPtr)
			{
				slot.goalFaceKey = spcLastConnDestFace;

				// Boost A* budget: 4x default
				*(int*)(ss + 156) = 131072 * SPC_BUDGET_MULT;
				*(int*)(ss + 160) = 590336 * SPC_BUDGET_MULT;
				pathPoolBoosted = 1;
				InterlockedIncrement(&spcDiagBoosted);

				QueryPerformanceCounter(&pathPoolQpcBefore);
				orig_findPathFull(streamingCollection, searchState, findPathOutput);
				unsigned char leaderStatus = *(unsigned char*)(fpo + 60);

				if (probeSlot >= 0 && probeSlot < PATH_PROBE_SIZE)
					pathProbeBuf[probeSlot].result = (int)leaderStatus;

				if (leaderStatus == 1)
				{
					void* edgeData = *(void**)(fpo + 16);
					int   edgeCnt  = *(int*)(fpo + 24);
					if (edgeData && edgeCnt > 0 && edgeCnt < 200000)
					{
						slot.edgeData = fn_gameNewArr((size_t)edgeCnt * 4);
						if (slot.edgeData)
						{
							memcpy(slot.edgeData, edgeData, (size_t)edgeCnt * 4);
							slot.edgeCount = edgeCnt;
							slot.numIter   = *(int*)(fpo + 48);
							slot.goalIdx   = *(int*)(fpo + 52);
							slot.pathCost  = *(float*)(fpo + 56);
							slot.cachedSCSize = *(int*)((uintptr_t)streamingCollection + OFF_SC_INSTANCES_COUNT);
							slot.state = 2;
							InterlockedIncrement(&spcDiagLeaderOK);
						}
						else
						{
							slot.state = 3;
							slot.goalFaceKey = 0;
							InterlockedIncrement(&spcDiagLeaderFail);
						}
					}
					else
					{
						slot.state = 3;
						slot.goalFaceKey = 0;
						InterlockedIncrement(&spcDiagLeaderFail);
					}
				}
				else
				{
					slot.state = 3;
					slot.goalFaceKey = 0;
					InterlockedIncrement(&spcDiagLeaderFail);
				}

				slot.servedCount++;
				goto diag_count;
			}

			// CACHE_ACTIVE or REPATH_INJECT: inject cached result
			if ((slot.state == 2 || slot.state == 6) && slot.edgeData)
			{
				int curSCSize = *(int*)((uintptr_t)streamingCollection + OFF_SC_INSTANCES_COUNT);
				if (curSCSize < slot.cachedSCSize)
				{
					InterlockedIncrement(&p12DiagSCGuardSkips);
					if (slot.state == 2 || slot.state == 6)
					{
						SpcFreeSlot(taggedSlot);
						slot.state = 4;
						slot.goalFaceKey = 0;
					}
					goto default_astar;
				}

#if PATHFIND_STEP >= 9
				// Stability gate: if addInstance ran in the last 500ms, the
				// streaming collection is actively mutating and our cached
				// section references may race with in-flight evictions during
				// the post-return extraction. Skip injection, let the game's
				// real A* run against current m_instances (safe by construction).
				{
					LONG64 lastAddQPC = InterlockedCompareExchange64(&g_lastAddInstanceQPC, 0, 0);
					if (lastAddQPC != 0) {
						LARGE_INTEGER nowLi;
						QueryPerformanceCounter(&nowLi);
						double deltaSec = (double)(nowLi.QuadPart - lastAddQPC)
						                  / (double)qpcFrequency.QuadPart;
						if (deltaSec < 0.5) {
							InterlockedIncrement(&spcStabilityHold);
							if (slot.state == 2 || slot.state == 6) {
								SpcFreeSlot(taggedSlot);
								slot.state = 4;
								slot.goalFaceKey = 0;
							}
							goto default_astar;
						}
					}
				}
#endif

				int byteCount = slot.edgeCount * 4;
				void* edgeCopy = HavokTlsAlloc((size_t)byteCount);
				if (edgeCopy)
				{
					memcpy(edgeCopy, slot.edgeData, byteCount);

					*(void**)(fpo + 16) = edgeCopy;
					*(int*)(fpo + 24)   = slot.edgeCount;
					*(int*)(fpo + 28)   = slot.edgeCount;
					*(int*)(fpo + 48)   = slot.numIter;
					*(int*)(fpo + 52)   = slot.goalIdx;
					*(float*)(fpo + 56) = slot.pathCost;
					*(unsigned char*)(fpo + 60) = 1;
					*(unsigned char*)(fpo + 61) = 0;

					slot.servedCount++;
					slot.lastInjectionTime = now;

					if (slot.state == 2 && slot.servedCount >= slot.memberCount)
					{
						SpcFreeSlot(taggedSlot);
						slot.state = 4;
						slot.burstInjected = 0;
					}
					else if (slot.state == 6)
					{
						slot.burstInjected++;
						InterlockedIncrement(&p12DiagRepathInjections);
					}

					InterlockedIncrement(&spcDiagInjected);
					InterlockedIncrement(&diagAstarSuccess);

					if (probeSlot >= 0 && probeSlot < PATH_PROBE_SIZE)
						pathProbeBuf[probeSlot].result = 1;

					return;
				}
				if (slot.state == 2 && slot.servedCount >= slot.memberCount - 1)
				{
					SpcFreeSlot(taggedSlot);
					slot.state = 4;
				}
			}

			// REPATH_ACTIVE: re-path leader detection
			if (slot.state == 4 && goalPtr)
			{
				slot.goalFaceKey = spcLastConnDestFace;

				QueryPerformanceCounter(&pathPoolQpcBefore);
				orig_findPathFull(streamingCollection, searchState, findPathOutput);
				unsigned char leaderStatus = *(unsigned char*)(fpo + 60);

				if (probeSlot >= 0 && probeSlot < PATH_PROBE_SIZE)
					pathProbeBuf[probeSlot].result = (int)leaderStatus;

				if (leaderStatus == 1)
				{
					void* edgeData = *(void**)(fpo + 16);
					int   edgeCnt  = *(int*)(fpo + 24);
					if (edgeData && edgeCnt > 0 && edgeCnt < 200000)
					{
						SpcFreeSlot(taggedSlot);
						slot.edgeData = fn_gameNewArr((size_t)edgeCnt * 4);
						if (slot.edgeData)
						{
							memcpy(slot.edgeData, edgeData, (size_t)edgeCnt * 4);
							slot.edgeCount = edgeCnt;
							slot.numIter   = *(int*)(fpo + 48);
							slot.goalIdx   = *(int*)(fpo + 52);
							slot.pathCost  = *(float*)(fpo + 56);
							slot.cachedSCSize = *(int*)((uintptr_t)streamingCollection + OFF_SC_INSTANCES_COUNT);
							slot.state = 6;
							slot.burstInjected = 0;
							slot.lastInjectionTime = now;
							InterlockedIncrement(&p12DiagRepathLeaders);
						}
					}
				}

				slot.servedCount++;
				goto diag_count;
			}
		}
	}
#endif // ZONEOPT_SQUAD_CACHE
#endif // PATHFIND_STEP >= 4

#if PATHFIND_STEP >= 4
#ifdef ZONEOPT_SQUAD_CACHE
default_astar:
#endif
#endif

	// Default path: call original A*
	QueryPerformanceCounter(&pathPoolQpcBefore);
	orig_findPathFull(streamingCollection, searchState, findPathOutput);

#if PATHFIND_STEP >= 4
#ifdef ZONEOPT_SQUAD_CACHE
diag_count:
#endif
#endif
	{
		unsigned char status = *(unsigned char*)((uintptr_t)findPathOutput + 60);
		unsigned char cause  = *(unsigned char*)((uintptr_t)findPathOutput + 61);
		int iterCount        = *(int*)((uintptr_t)findPathOutput + 48);

		// Phase 17 Step 1: every branch above that calls orig_findPathFull
		// (default, and -- ZONEOPT_SQUAD_CACHE only -- AWAITING_LEADER and
		// REPATH_ACTIVE) converges here exactly once per hook_findPathFull
		// call, whether by fallthrough or by goto. Take the "after" QPC and
		// hand the sample to the pool on every thread, every call.
		{
			LARGE_INTEGER pathPoolQpcAfter;
			QueryPerformanceCounter(&pathPoolQpcAfter);

			PathSearchSample pathPoolSample;
			pathPoolSample.ticks      = pathPoolQpcAfter.QuadPart - pathPoolQpcBefore.QuadPart;
			pathPoolSample.iterations = iterCount;
			pathPoolSample.status     = (int)status;
			pathPoolSample.cause      = (int)cause;
			pathPoolSample.boosted    = pathPoolBoosted;
#if PATHFIND_STEP >= 2
			pathPoolSample.playerByTag = currentRequestIsPlayer ? 1 : 0;
#else
			pathPoolSample.playerByTag = 0;
#endif
			pathPoolSample.playerByReq = pathPoolPlayerByReq;
			PathPoolNoteSearch(&pathPoolSample);
		}

#if PATHFIND_STEP >= 4
		if (probeSlot >= 0 && probeSlot < PATH_PROBE_SIZE)
			pathProbeBuf[probeSlot].result = (int)status;
#endif

		if (status == 1)
			InterlockedIncrement(&diagAstarSuccess);
		else if (status == 2)
			InterlockedIncrement(&diagAstarUnreachable);
		else if (status == 3)
		{
			InterlockedIncrement(&diagAstarTerminated);

			if (cause == 1)
				InterlockedIncrement(&diagTermIterLimit);
			else if (cause == 2)
				InterlockedIncrement(&diagTermOpenSetFull);
			else if (cause == 3)
				InterlockedIncrement(&diagTermStatesFull);
			else
				InterlockedIncrement(&diagTermOtherCause);

			InterlockedExchange(&diagLastTermIter, (long)iterCount);
		}
		else if (status == 4)
			InterlockedIncrement(&diagAstarTruncated);
		else if (status == 5)
			InterlockedIncrement(&diagAstarInvalid);
		else
			InterlockedIncrement(&diagAstarOther);

		// Update max iterations high-water mark (lock-free CAS loop)
		long prev;
		do {
			prev = InterlockedCompareExchange(&diagMaxIterUsed, 0, 0);
			if ((long)iterCount <= prev)
				break;
		} while (InterlockedCompareExchange(&diagMaxIterUsed, (long)iterCount, prev) != prev);

		// Record last failure detail for player exposure reporting
		if (status != 1 && status != 2)
		{
			float* goalPtr = searchState ? *(float**)((uintptr_t)searchState + 32) : NULL;
			if (goalPtr)
			{
				lastAstarFail.goalX = goalPtr[0];
				lastAstarFail.goalY = goalPtr[1];
				lastAstarFail.goalZ = goalPtr[2];
			}
			InterlockedExchange(&lastAstarFail.status, (long)status);
			InterlockedExchange(&lastAstarFail.cause, (long)cause);
			InterlockedExchange(&lastAstarFail.iterCount, (long)iterCount);
			InterlockedIncrement(&lastAstarFail.sequence);
		}

#if PATHFIND_STEP >= 2
		// Per-player failure: record to ring buffer for main-thread logging
		if (status != 1 && currentRequestIsPlayer)
		{
			float* goalPtr = searchState ? *(float**)((uintptr_t)searchState + 32) : NULL;
			long idx = InterlockedIncrement(&playerFailWriteIdx) - 1;
			int slot = (int)(idx % PLAYER_FAIL_RING);
			playerFailRing[slot].goalX = goalPtr ? goalPtr[0] : 0;
			playerFailRing[slot].goalZ = goalPtr ? goalPtr[2] : 0;
			InterlockedExchange(&playerFailRing[slot].status, (long)status);
			InterlockedExchange(&playerFailRing[slot].cause, (long)cause);
			InterlockedExchange(&playerFailRing[slot].iterCount, (long)iterCount);
			InterlockedExchange(&playerFailRing[slot].valid, 1);
		}
#endif
	}

#if PATHFIND_STEP >= 6
	// ExitFace decode: dispatch to helper if we have a reqCharMap match.
	// currentBgReq is set by hook_csFindPathFallback (called immediately above us
	// on the same bg thread, same call stack -- findPathFull is called inline from
	// findPathFallback). Cleared by hook_csFindPathFallback after we return.
	if (currentBgReq) {
		uintptr_t havokChar = ReqCharMapLookupAndDelete(currentBgReq);
		if (havokChar) {
			DecodeExitFaceAndUpdate(streamingCollection, searchState,
			                        findPathOutput, havokChar);
		}
	}

	if (!InterlockedCompareExchange(&threadAssertFindFull, 1, 0)) {
		std::ostringstream ss;
		ss << "[ZoneOpt] PATHFIND_STEP=6 hook_findPathFull tid=" << GetCurrentThreadId();
		LogMsg(ss.str());
	}
#endif
}


// =========================================================================
// Hook 4: HavokCharacter::requestPath (main thread)
// =========================================================================
// RVA 0x145CB0. RCX=HavokCharacter*, RDX=dest(float*), R8=priority(int)
// priority==2: player character, priority==0: NPC

#if PATHFIND_STEP >= 1

void hook_requestPath(void* havokChar, float* destination, int priority)
{
	// Player vs NPC tracking
	if (priority >= 2)
		InterlockedIncrement(&diagPlayerRequests);
	else
		InterlockedIncrement(&diagNPCRequests);

#if PATHFIND_STEP >= 6
	// NPC filter: priority>=2 = player-owned (per setDestination_Vec3 contract).
	// NPCs would flood the 64-entry map and never match a watched char anyway.
	if (priority >= 2) {
		currentReqChar = (uintptr_t)havokChar;
	} else {
		currentReqChar = 0;
		InterlockedIncrement(&npcRequestsSkipped);
	}

	if (!InterlockedCompareExchange(&threadAssertReqPath, 1, 0)) {
		std::ostringstream ss;
		ss << "[ZoneOpt] PATHFIND_STEP=6 hook_requestPath tid=" << GetCurrentThreadId();
		LogMsg(ss.str());
	}
#endif

#if PATHFIND_STEP >= 2
	squadBoostTier = 0;
	squadBoostGroupIdx = -1;

	// All player requests get baseline boost (priority >= 2 = player-owned)
	if (priority >= 2)
		squadBoostTier = 45;

	if (squadPathCacheEnabled)
	{
		uintptr_t hc = (uintptr_t)havokChar;

		// Check 1: pending-order buffer (tier 60, user just clicked)
		bool isFreshOrder = false;
		for (int p = 0; p < pendingOrderCount; ++p)
		{
			if (pendingOrderHC[p] == hc)
			{
				isFreshOrder = true;
				pendingOrderCount--;
				if (p < pendingOrderCount)
					pendingOrderHC[p] = pendingOrderHC[pendingOrderCount];
				break;
			}
		}

		// Check 2: formation group membership
		for (int g = 0; g < MAX_FORMATION_GROUPS; ++g)
		{
			if (!formationGroups[g].active)
				continue;

			for (int m = 0; m < formationGroups[g].count; ++m)
			{
				uintptr_t cm = formationGroups[g].members[m].charMovement;
				if (!cm) continue;
				uintptr_t memberHC = *(uintptr_t*)(cm + OFF_CMOV_HAVOK_CHAR);
				if (memberHC == hc)
				{
					squadBoostGroupIdx = g;

					if (isFreshOrder)
					{
						squadBoostTier = 60;
						InterlockedIncrement(&p12DiagTier60);
					}
					else if (priority == 2)
					{
						InterlockedIncrement(&p12DiagTier40);
					}
					else
					{
						InterlockedIncrement(&p12DiagTier30);
					}

					static long p12FirstMatchLogged = 0;
					if (!InterlockedCompareExchange(&p12FirstMatchLogged, 1, 0))
					{
						std::ostringstream ss;
						ss << "[ZoneOpt] P12 firstMatch: pri=" << priority
						   << " tier=" << squadBoostTier
						   << " fresh=" << (isFreshOrder ? 1 : 0)
						   << " grp=" << g << " mem=" << m
						   << " hc=" << (void*)hc
						   << " pendOrd=" << pendingOrderCount;
						LogMsg(ss.str());
					}

					InterlockedIncrement(&p12DiagRequestPathHits);
					goto matched;
				}
			}
		}

		if (isFreshOrder)
		{
			squadBoostTier = 50;
			InterlockedIncrement(&p12DiagTier50);
			InterlockedIncrement(&p12DiagRequestPathHits);
		}
	}
matched:
#endif // PATHFIND_STEP >= 2

	orig_requestPath(havokChar, destination, priority);

#if PATHFIND_STEP >= 2
	squadBoostTier = 0;
	squadBoostGroupIdx = -1;
#endif

#if PATHFIND_STEP >= 6
	currentReqChar = 0;
#endif
}

#endif // PATHFIND_STEP >= 1


// =========================================================================
// Hook 5: PathRequestQueue::submit (main thread)
// =========================================================================
// RVA 0x3AAEF0. Calls orig, then overwrites req+44 with boosted priority.

#if PATHFIND_STEP >= 2

void hook_pathReqSubmit(void* sectionMgr, void* requestObj, bool highPriority)
{
	orig_pathReqSubmit(sectionMgr, requestObj, highPriority);

	if (squadBoostTier > 0)
	{
		int gamePri = *(int*)((uintptr_t)requestObj + 44);
		if (squadBoostTier > gamePri)
		{
			*(int*)((uintptr_t)requestObj + 44) = squadBoostTier;
			InterlockedIncrement(&p12DiagSubmitBoosts);
			InterlockedIncrement(&playerRequestsInFlight);
		}
	}

#if PATHFIND_STEP >= 6
	if (currentReqChar && requestObj) {
		ReqCharMapInsert(requestObj, currentReqChar);
	}

	if (!InterlockedCompareExchange(&threadAssertSubmit, 1, 0)) {
		std::ostringstream ss;
		ss << "[ZoneOpt] PATHFIND_STEP=6 hook_pathReqSubmit tid=" << GetCurrentThreadId()
		   << " requestObj=" << requestObj;
		LogMsg(ss.str());
	}

	// One-shot offset cross-reference: emit requestObj+128 so hook_csFindPathFallback's
	// recoveredReq=Y line can be cross-checked against requestObj=R from this log.
	if (requestObj && !InterlockedCompareExchange(&resultBufOffsetValidated, 1, 0)) {
		std::ostringstream ss;
		ss << "[ZoneOpt] PATHFIND_STEP=6 first request: requestObj=" << requestObj
		   << " resultBufSlot=" << (void*)((char*)requestObj + OFF_REQ_RESULTBUF_SLOT);
		LogMsg(ss.str());
	}
#endif

	squadBoostTier = 0;
	squadBoostGroupIdx = -1;
}

#endif // PATHFIND_STEP >= 2


// =========================================================================
// Hook 6: ContentStream::findPathFallback (contentStream bg thread)
// =========================================================================
// RVA 0x3AABF0. Called when csFindPath fails. Tags cache slots for findPathFull.

#if PATHFIND_STEP >= 4

char hook_csFindPathFallback(void* manager, unsigned int startFaceKey, void* startPos,
                              unsigned int destFaceKey, void* destPos, float radius,
                              float param6, char param7, void* resultBuf)
{
#if PATHFIND_STEP >= 6
	// Recover request pointer via dispatcher offset.
	// SectionManager::contentStream (0x3AE350) calls findPathFallback with &v80[8]
	// (= req+128). PathRequest__ctor inits req+128 to NULL (buffer slot).
	if (resultBuf) {
		currentBgReq = (void*)((char*)resultBuf - OFF_REQ_RESULTBUF_SLOT);
	} else {
		currentBgReq = NULL;
	}

	if (!InterlockedCompareExchange(&threadAssertFallback, 1, 0)) {
		std::ostringstream ss;
		ss << "[ZoneOpt] PATHFIND_STEP=6 hook_csFindPathFallback tid="
		   << GetCurrentThreadId()
		   << " resultBuf=" << resultBuf
		   << " recoveredReq=" << currentBgReq;
		LogMsg(ss.str());
	}
#endif

#ifdef ZONEOPT_SQUAD_CACHE
	// H5: tagging a request for the squad path cache only exists when
	// ZONEOPT_SQUAD_CACHE is defined. currentBgReq (above) stays live in every
	// build -- the ExitFace chain reads it.
	spcFallbackTag = 0;

	if (squadPathCacheEnabled)
	{
		for (int s = 0; s < MAX_FORMATION_GROUPS; ++s)
		{
			if (spcSlots[s].state < 1) continue;

			if (spcSlots[s].goalFaceKey != 0 && destFaceKey == spcSlots[s].goalFaceKey)
			{
				spcFallbackTag = s + 1;
				InterlockedIncrement(&p12DiagFallbackTags);
				break;
			}

			if (spcSlots[s].state == 1 || spcSlots[s].state == 4)
			{
				float offX = spcSlotCandOffX[s];
				float offZ = spcSlotCandOffZ[s];
				if (offX == 0.0f && offZ == 0.0f) continue;
				float expX = spcSlots[s].expGoalX + offX;
				float expZ = spcSlots[s].expGoalZ + offZ;
				float* dp = (float*)destPos;
				float dx = dp[0] - expX;
				float dz = dp[2] - expZ;
				float adx = (dx < 0) ? -dx : dx;
				float adz = (dz < 0) ? -dz : dz;
				if (adx < SPC_GOAL_TOL && adz < SPC_GOAL_TOL)
				{
					spcFallbackTag = s + 1;
					InterlockedIncrement(&p12DiagFallbackTags);
					break;
				}
			}
		}
	}
#endif // ZONEOPT_SQUAD_CACHE

	char result = orig_csFindPathFallback(manager, startFaceKey, startPos,
	                                       destFaceKey, destPos, radius,
	                                       param6, param7, resultBuf);
#ifdef ZONEOPT_SQUAD_CACHE
	spcFallbackTag = 0;
#endif
#if PATHFIND_STEP >= 6
	currentBgReq = NULL;
#endif
	return result;
}

#endif // PATHFIND_STEP >= 4


// =========================================================================
// STEP 9 hooks: extraction SEH wrap + addInstance timestamp gate
// =========================================================================
#if PATHFIND_STEP >= 9

// Hook 7: hkaiStreamingCollection::addInstance (navmesh bg thread).
// Wraps orig and stamps g_lastAddInstanceQPC so our cache injection can detect
// whether streaming state was recently mutated. No blocking, no locks.
void hook_addInstance(void* collection, __int64 sectionData,
                      __int64 param3, __int64 param4, int param5)
{
	orig_addInstance(collection, sectionData, param3, param4, param5);
	LARGE_INTEGER now;
	QueryPerformanceCounter(&now);
	InterlockedExchange64(&g_lastAddInstanceQPC, now.QuadPart);
	InterlockedIncrement(&addInstanceHookCalls);
}

// Hook 8: Havok::contentStreamCallee_0x8869 (contentStream bg thread).
// SEH wraps the path-result extraction loop. On AV (from the race between
// our cached edges + game's addInstance evicting their sections), rolls back
// resultBuf[2] to its pre-call value so the partial writes into resultBuf's
// tail are logically erased. Game treats as "no new edges produced" -> clean
// no-path -> character re-paths on next tick instead of bugging out with
// corrupted mid-loop data.
//
// NOTE: SEH + /EHsc requires no C++ destructors in scope. This function uses
// only POD types + Win32 atomics, so it's safe.
unsigned __int64 hook_contentStreamCallee0x8869(void* manager,
                                                 unsigned int faceKey,
                                                 void* searchOutput,
                                                 unsigned int* resultBuf)
{
	unsigned int origCount = resultBuf ? resultBuf[2] : 0;
	unsigned __int64 result = origCount;
	// B7: a fault in the extraction chain is what this guard exists for, and
	// the rescue counter records it. Keep it out of the crash recorder.
	GuardEnter();
	__try {
		result = orig_contentStreamCallee0x8869(manager, faceKey, searchOutput, resultBuf);
	} __except (EXCEPTION_EXECUTE_HANDLER) {
		InterlockedIncrement(&extractionCrashRescue);
		if (resultBuf) resultBuf[2] = origCount;
		result = origCount;
	}
	GuardLeave();
	return result;
}

#endif // PATHFIND_STEP >= 9 (final)


#endif // PATHFIND_STEP >= 1
