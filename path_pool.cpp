// path_pool.cpp — Phase 17 Step 1 instrumentation (research/path_worker_pool.md §7)
//
// Four timed/counting pass-through hooks on the path thread's own functions
// (contentStream, dequeueWork, enqueue_threadSafe, Gates__updateCodes), plus
// the per-search classifier (PathPoolNoteSearch, called by hook_findPathFull)
// and the main-thread window reporter (PathPoolTickMain). No behaviour
// change: every hook here calls the original and returns exactly what it
// returned; the mod's own state is diagnostic-only.
//
// Thread rules (CLAUDE.md, path_worker_pool.md §7): everything that can run
// off the main thread is lock-free and allocation-free -- Interlocked*
// counters, fixed arrays, thread-locals, no std::string/ostringstream, no
// LogMsg from the four hooks themselves. The NPC wait walk and the window
// printers run on the main thread only and may use the CRT freely.

#include "path_pool.h"
#include "transition.h"   // isTransitionActive
#include "grid.h"         // WorldToZoneGrid
#include <intrin.h>        // _ReadWriteBarrier
#include <cmath>

#if PATHPOOL_STEP >= 1

// =========================================================================
// Shared layout constants (path_request_queue.md, path_thread_audit.md §3)
// =========================================================================

static const size_t PATHQ_INPUT_OFFSET  = 0xB8;   // mgr+0xB8: input queue (submit)
static const size_t PATHQ_RESULT_OFFSET = 0xF8;   // mgr+0xF8: result queue (contentStream)
static const size_t PATHQ_DEPTH_OFFSET  = 0x1C0;  // mgr+0x1C0: sorted array count

static const size_t REQ_STAMP_OFFSET    = 0x00;   // never written by the game (audit §3)
static const size_t REQ_PRIORITY_OFFSET = 0x2C;   // 10 NPC, 20 player, 45/50/60 mod tiers
static const size_t REQ_STATUS_OFFSET   = 0x90;   // -10 sentinel
static const size_t REQ_START_OFFSET    = 0x40;   // start position (3 floats)
static const size_t REQ_GOAL_OFFSET     = 0x50;   // goal position, offset-free once resolved

static const int REQ_STATUS_SENTINEL = -10;


// =========================================================================
// Thread identity + section manager latch (§1)
// =========================================================================

volatile DWORD g_pathThreadId = 0;

// Latched by hook_contentStream on every pass so the dequeueWork/enqueue
// filters always compare against the SectionManager currently in use (a new
// game or a save load can replace it: path_worker_pool.md §11). Single
// writer (the path thread, from inside contentStream); read on the path
// thread (same call stack) and, in principle, could be read elsewhere, so
// published through InterlockedExchangePointer for a clean 8-byte publish.
static void* volatile g_sectionMgrPtr = NULL;

// Set for the duration of hook_contentStream so hook_gatesUpdateCodes (its
// sole caller, IDA-verified) can confirm it is really running inside a pass.
static __declspec(thread) int t_inContentStreamPass = 0;

// Set for the duration of hook_gatesUpdateCodes; PathPoolNoteSearch reads it
// to attribute path-thread searches to the gate counters instead of the
// queue counters (path_worker_pool.md §7).
static __declspec(thread) int t_inGatePass = 0;

// QPC at the start of the pass currently in progress on the path thread.
// Single-threaded (only ever written/read by the path thread, from nested
// calls within the same contentStream invocation), so a plain global is
// safe -- no other thread touches it.
static LARGE_INTEGER g_passStartTicks = { 0 };

// Round-1 review fix (item 8): "svc" must not include the pass preamble
// (origin shift, drainResults, work items, section adds, the gate pass,
// the request drain), or PathSlow ranks whole passes -- a gate pass can run
// up to 929 ms -- instead of searches. The tightest lower bound the four
// hooks can give for "serve one request began" is the LATEST of: pass
// start, the gate pass's own end (if one ran this pass) and the last
// dequeueWork return this pass (request drain, audit row 6, immediately
// precedes serve, row 7). Both reset to the pass start at the top of every
// pass; hook_gatesUpdateCodes and hook_dequeueWork push them later only if
// they actually run. Plain globals: path-thread-only, same reasoning as
// g_passStartTicks above.
static LARGE_INTEGER g_gatePassEndTicks  = { 0 };
static LARGE_INTEGER g_lastDequeueTicks  = { 0 };

// Set (not logged) by hook_gatesUpdateCodes on the path thread when it fires
// outside a contentStream pass; PathPoolTickMain reports it once from the
// main thread (item 6: no LogMsg from the path thread here).
static volatile LONG g_gateOutsideStreamWarned = 0;

// Latched by PathPoolNoteSearch on the path thread for the last non-gate
// search this pass; hook_enqueueThreadSafe reads it for the PathSlow
// iterations field. Path-thread-only, like the ticks above.
//
// Round-2 review fix: reset to -1 at the top of every pass (with
// g_gatePassEndTicks/g_lastDequeueTicks below), not left holding a stale
// value across passes. On the path thread outside gate passes the only
// caller of findPathFull is findPathFallback (serve step 7); a completion
// that ends at step 3 (status 1, no start face), step 4 (direct csFindPath
// success, status 0) or step 5 (status 2, no goal face) runs no search at
// all this pass, so without the reset PathSlow's iter= would silently show
// another, possibly many-passes-old, request's iteration count. -1 prints
// as "-" (no search ran for this completion).
static LONG g_lastPathIterations = -1;

// Round-2 review fix (finding 3a): "direct=" needs to know whether ANY
// path-thread, non-gate search ran during the pass that produced a given
// completion -- a status-0 completion with none is the csFindPath direct
// path (RunPathRequest step 4), not the fallback/full-search path (step 7).
// Reset alongside g_lastPathIterations; incremented by PathPoolNoteSearch.
static LONG g_passSearchCount = 0;


static LONGLONG TicksToUs(LONGLONG ticks)
{
	if (qpcFrequency.QuadPart <= 0 || ticks <= 0)
		return 0;
	return (ticks * 1000000LL) / qpcFrequency.QuadPart;
}

// Nanosecond resolution for the per-iteration cost (item 2): microseconds
// put nearly every per-iteration cost in bucket 0/1 (the histogram's log2
// buckets are in whole units), so AstarCost's p50/p90 always read ~1.0/1.0.
// Nanoseconds spread real per-iteration costs (tens to low thousands of ns)
// across enough buckets to be informative.
static LONGLONG TicksToNs(LONGLONG ticks)
{
	if (qpcFrequency.QuadPart <= 0 || ticks <= 0)
		return 0;
	return (ticks * 1000000000LL) / qpcFrequency.QuadPart;
}


// =========================================================================
// Fixed log-spaced histogram (lock-free): p50/p90/p99/max from bucket counts
// =========================================================================
//
// Bucket b covers [2^b, 2^(b+1)) microseconds (bucket 0 covers [0,2)). 40
// buckets covers up to ~18 minutes, far beyond any single search or request.
// Percentiles are reported as the bucket's lower bound -- a log2-resolution
// approximation, adequate for a Step 1 "is it queue latency or search cost"
// diagnostic, not exact statistics.

static const int PP_HIST_BUCKETS = 40;

struct PPHist
{
	volatile LONG     buckets[PP_HIST_BUCKETS];
	volatile LONG     maxUs;
	volatile LONG     count;
	volatile LONGLONG sumUs;
};

static void PPHistAdd(PPHist* h, LONGLONG us)
{
	if (us < 0) us = 0;
	int b = 0;
	LONGLONG v = us;
	while (v > 1 && b < PP_HIST_BUCKETS - 1) { v >>= 1; ++b; }
	InterlockedIncrement(&h->buckets[b]);
	InterlockedIncrement(&h->count);
	InterlockedExchangeAdd64(&h->sumUs, us);
	for (;;)
	{
		LONG cur = h->maxUs;
		if (us <= (LONGLONG)cur) break;
		if (InterlockedCompareExchange(&h->maxUs, (LONG)us, cur) == cur) break;
	}
}

// Non-destructive. Round-1 review fix (item 7): the total used to come from
// h->count, incremented by PPHistAdd AFTER the bucket increment, so a reader
// racing a writer between those two increments could see a total the bucket
// sum had not caught up to yet -- the loop then never reaches `want` and
// falls through to the last bucket (1<<39 us, ~549,755,813 ms, exactly the
// artifact the review reported). Deriving the total from the bucket sum
// itself removes the dependency on that second counter: both the total and
// the running sum below read the same 40 buckets, so `running` is
// guaranteed to reach `total` by the last bucket even if a writer is
// concurrently incrementing (buckets only ever increase between resets).
static LONGLONG PPHistPercentileUs(const PPHist* h, double frac)
{
	LONG total = 0;
	for (int b = 0; b < PP_HIST_BUCKETS; ++b)
		total += h->buckets[b];
	if (total <= 0) return 0;
	LONG want = (LONG)(frac * (double)total + 0.5);
	if (want < 1) want = 1;
	LONG running = 0;
	for (int b = 0; b < PP_HIST_BUCKETS; ++b)
	{
		running += h->buckets[b];
		if (running >= want)
			return (LONGLONG)1 << b;
	}
	return (LONGLONG)1 << (PP_HIST_BUCKETS - 1);
}

static void PPHistReset(PPHist* h)
{
	for (int b = 0; b < PP_HIST_BUCKETS; ++b)
		InterlockedExchange(&h->buckets[b], 0);
	InterlockedExchange(&h->maxUs, 0);
	InterlockedExchange(&h->count, 0);
	InterlockedExchange64(&h->sumUs, 0);
}


// =========================================================================
// §3 PathQueue: / GateRate: window state (fed by the four hooks)
// =========================================================================

static volatile LONG     g_passCount    = 0;   // contentStream passes this window
static volatile LONGLONG g_passTotalUs  = 0;   // sum of pass durations (busy%)

static volatile LONG     g_depthMax     = 0;
static volatile LONGLONG g_depthSum     = 0;
static volatile LONG     g_depthSamples = 0;

static volatile LONG     g_arrivedCount = 0;   // requests dequeued from the input queue (item 3)
static volatile LONG     g_servedCount  = 0;   // completions seen at the result queue
static PPHist             g_waitHist;          // completion - drain stamp
// "svc" is the tightened serve-start-to-completion span (item 8); see
// g_gatePassEndTicks/g_lastDequeueTicks above for how serve-start is derived.
static PPHist             g_svcHist;
static volatile LONGLONG  g_svcTotalUs  = 0;   // for preamble = passTotalUs - svcTotalUs

static volatile LONG g_priNpcCount    = 0;     // req+0x2C <= 10
static volatile LONG g_priPlayerCount = 0;     // req+0x2C == 20
static volatile LONG g_priTierCount   = 0;     // req+0x2C >= 45

// req+0x90 at completion (path_thread_audit.md §3), one counter per raw
// status: 0 path found, 1 no start face, 2 no goal face, 3 not connected,
// 4 fallback search failed. Round-2 review fix (finding 3b): an ok/unreach
// split had grouped 3 in with "unreach", but hook_csCheckFaceConn
// unconditionally returns 1 at PATHFIND_STEP >= 3 (pathfind_hooks.cpp,
// the Phase 10 cluster-graph bypass), so RunPathRequest's step 6 -- the
// only place that can write status 3 -- never runs false in either test
// build (step-4 build or the PATHFIND_STEP=9 ladder): st3 is always 0 there
// by construction, not because goals are reachable. Genuinely unreachable
// goals surface as status 4 (fallback/full A* search ran and failed) mixed
// with 1 and 2 -- print each status separately so Step 1 can see that count
// on its own instead of folding it into a euphemism.
static volatile LONG g_reqStatusCount[5] = { 0, 0, 0, 0, 0 };

// Round-2 review fix (finding 3a): completions with status 0 that ran no
// path-thread search this pass (g_passSearchCount == 0) took RunPathRequest's
// step-4 direct csFindPath success, not the step-7 fallback/full-search path.
static volatile LONG g_directCount = 0;

static volatile LONG     g_gatePassCount        = 0;
static volatile LONGLONG g_gatePassTotalUs      = 0;
static volatile LONG     g_gatePassMaxUs        = 0;
static volatile LONG     g_gatePassInTransition = 0;

// Attributed inside a gate pass (PathPoolNoteSearch), not the queue counters.
static volatile LONG g_gateSearchCount = 0;
static volatile LONG g_gateIterLimit   = 0;    // cause == 1
static volatile LONG g_gateStateFull   = 0;    // cause == 3


// --- PathSlow: top 5 by service time (lock-free seqlock double buffer) ---

struct PPSlowEntry
{
	LONGLONG svcUs;
	LONG     status;
	LONG     priority;
	LONG     iterations;   // item 4: latched from the pass's last PathPoolNoteSearch call
	// Havok units (world units x 0.1: audit's "+0x50 goal x 0.1" scale),
	// read straight from req+0x40/+0x50 -- not converted to world units.
	float    startX, startZ, goalX, goalZ;
};

static const int PP_SLOW_N = 5;

// Path-thread-owned working copy (single writer: the enqueue-result hook).
static PPSlowEntry g_slowWork[PP_SLOW_N];
static int         g_slowWorkCount = 0;

// Published snapshot + seqlock (islands.cpp's PublishSnapshot pattern).
static volatile LONG g_slowSeq = 0;
static PPSlowEntry   g_slowPublished[PP_SLOW_N];
static volatile LONG g_slowPublishedCount = 0;

// Main thread requests a reset; the path thread clears its working copy the
// next time it has something to insert (no cross-thread array write).
static volatile LONG g_slowResetRequested = 0;

static void PPSlowConsider(LONGLONG svcUs, LONG status, LONG priority, LONG iterations,
                            float sx, float sz, float gx, float gz)
{
	if (InterlockedCompareExchange(&g_slowResetRequested, 0, 1) == 1)
		g_slowWorkCount = 0;

	int insertAt = -1;
	if (g_slowWorkCount < PP_SLOW_N)
	{
		insertAt = g_slowWorkCount++;
	}
	else
	{
		int minIdx = 0;
		for (int i = 1; i < PP_SLOW_N; ++i)
			if (g_slowWork[i].svcUs < g_slowWork[minIdx].svcUs) minIdx = i;
		if (svcUs > g_slowWork[minIdx].svcUs)
			insertAt = minIdx;
	}
	if (insertAt < 0)
		return;

	g_slowWork[insertAt].svcUs      = svcUs;
	g_slowWork[insertAt].status     = status;
	g_slowWork[insertAt].priority   = priority;
	g_slowWork[insertAt].iterations = iterations;
	g_slowWork[insertAt].startX   = sx;
	g_slowWork[insertAt].startZ   = sz;
	g_slowWork[insertAt].goalX    = gx;
	g_slowWork[insertAt].goalZ    = gz;

	InterlockedIncrement(&g_slowSeq);        // odd: writing
	_ReadWriteBarrier();
	memcpy(g_slowPublished, g_slowWork, sizeof(g_slowWork));
	InterlockedExchange(&g_slowPublishedCount, g_slowWorkCount);
	_ReadWriteBarrier();
	InterlockedIncrement(&g_slowSeq);        // even: consistent
}

// Main thread only. Returns the published count (0 on a failed snapshot,
// treated as empty for that window rather than retried indefinitely).
static int PPSlowSnapshot(PPSlowEntry* out)
{
	for (int attempt = 0; attempt < 4; ++attempt)
	{
		LONG s1 = g_slowSeq;
		if (s1 & 1) continue;
		_ReadWriteBarrier();
		PPSlowEntry tmp[PP_SLOW_N];
		memcpy(tmp, g_slowPublished, sizeof(tmp));
		LONG cnt = g_slowPublishedCount;
		_ReadWriteBarrier();
		LONG s2 = g_slowSeq;
		if (s1 == s2)
		{
			memcpy(out, tmp, sizeof(tmp));
			return (int)cnt;
		}
	}
	return 0;
}


// =========================================================================
// §2 PathPoolNoteSearch state: per-thread-class stats + boost counters
// =========================================================================

enum PPThreadClass { PP_CLASS_PATH = 0, PP_CLASS_NAVMESH = 1, PP_CLASS_MAIN = 2, PP_CLASS_OTHER = 3, PP_CLASS_COUNT = 4 };

struct PPClassStats
{
	volatile LONG     count;
	volatile LONGLONG totalTicks;       // used for a mean latency (item 2: no longer discarded)
	volatile LONGLONG totalIterations;
	PPHist            latencyUs;        // per-class search wall time (brief §2's "latency histogram")
	PPHist            iterNsHist;       // per-iteration cost in ns; only path-thread's is printed (AstarCost)
};

static PPClassStats g_classStats[PP_CLASS_COUNT];

// Path-thread, non-gate A* outcome/termination accumulation (item 2's
// second half: "the §2 path-thread status and termination-cause
// accumulation outside gate passes"). Feeds PathQueue's term= field.
// cause: 1 = iteration limit, 2 = open set full, 3 = search state full
// (gate_codes.md §1's Havok__findPathFull cause mapping); 0/other = "other".
static volatile LONG g_pathSearchOk    = 0;  // status == 1
static volatile LONG g_pathSearchFail  = 0;  // status != 1
static volatile LONG g_pathTermIterLimit   = 0;
static volatile LONG g_pathTermOpenSetFull = 0;
static volatile LONG g_pathTermStateFull   = 0;
static volatile LONG g_pathTermOther       = 0;

// Boost counters (O9). Outcome index: 0 = success, iterations > 32768;
// 1 = success, iterations <= 32768; 2 = failure (status != 1).
static volatile LONG g_boostByTag[3][2];   // [outcome][playerByTag: 0 npc / 1 player]
static volatile LONG g_boostByReq[3][3];   // [outcome][playerByReq: 0 npc / 1 player / 2 unknown(-1)]
static volatile LONG g_boostDisagree = 0;  // playerByReq != -1 && playerByReq != playerByTag


void PathPoolNoteSearch(const PathSearchSample* s)
{
	if (!s)
		return;

	DWORD tid = GetCurrentThreadId();
	int cls;
	if (tid == g_pathThreadId)
		cls = PP_CLASS_PATH;
	else if (tid == g_navMeshBgThreadId)
		cls = PP_CLASS_NAVMESH;
	else if (IsMainThread())
		cls = PP_CLASS_MAIN;
	else
		cls = PP_CLASS_OTHER;

	bool inGate = (cls == PP_CLASS_PATH) && (t_inGatePass != 0);

	if (inGate)
	{
		InterlockedIncrement(&g_gateSearchCount);
		if (s->cause == 1) InterlockedIncrement(&g_gateIterLimit);
		if (s->cause == 3) InterlockedIncrement(&g_gateStateFull);
	}
	else
	{
		PPClassStats* cs = &g_classStats[cls];
		InterlockedIncrement(&cs->count);
		InterlockedExchangeAdd64(&cs->totalTicks, s->ticks);
		InterlockedExchangeAdd64(&cs->totalIterations, (LONGLONG)s->iterations);

		LONGLONG us = TicksToUs(s->ticks);
		PPHistAdd(&cs->latencyUs, us);

		LONGLONG ns = TicksToNs(s->ticks);
		LONGLONG nsPerIter = (s->iterations > 0) ? (ns / s->iterations) : ns;
		PPHistAdd(&cs->iterNsHist, nsPerIter);

		if (cls == PP_CLASS_PATH)
		{
			// Latch this pass's last path-thread iteration count and mark
			// that a search ran this pass, so hook_enqueueThreadSafe can
			// attach both to the PathSlow entry / direct= classification it
			// builds for the completion this pass produces.
			g_lastPathIterations = s->iterations;
			++g_passSearchCount;

			if (s->status == 1) InterlockedIncrement(&g_pathSearchOk);
			else                InterlockedIncrement(&g_pathSearchFail);
			switch (s->cause)
			{
				case 1:  InterlockedIncrement(&g_pathTermIterLimit);   break;
				case 2:  InterlockedIncrement(&g_pathTermOpenSetFull); break;
				case 3:  InterlockedIncrement(&g_pathTermStateFull);   break;
				default: InterlockedIncrement(&g_pathTermOther);       break;
			}
		}
	}

	if (s->boosted)
	{
		int outcome = (s->status == 1) ? ((s->iterations > 32768) ? 0 : 1) : 2;
		int tagIdx  = (s->playerByTag != 0) ? 1 : 0;
		InterlockedIncrement(&g_boostByTag[outcome][tagIdx]);

		int reqIdx = (s->playerByReq < 0) ? 2 : ((s->playerByReq != 0) ? 1 : 0);
		InterlockedIncrement(&g_boostByReq[outcome][reqIdx]);

		if (s->playerByReq != -1 && s->playerByReq != s->playerByTag)
			InterlockedIncrement(&g_boostDisagree);
	}
}


// =========================================================================
// §1 Hooks
// =========================================================================

char hook_contentStream(void* sectionMgr)
{
	// Item 10: unconditional re-latch every pass, not just the first (a
	// compare-exchange against 0 would never update after a recreated path
	// thread, e.g. after a save load that rebuilds the SectionManager). The
	// write is free next to a QueryPerformanceCounter call either way.
	InterlockedExchange((volatile LONG*)&g_pathThreadId, (LONG)GetCurrentThreadId());
	InterlockedExchangePointer(&g_sectionMgrPtr, sectionMgr);

	t_inContentStreamPass = 1;

	// Queue depth at pass start, on the path thread -- the racy main-thread
	// read path_worker_pool.md §7 mentions as an alternative is not needed
	// since we already sample here every pass.
	if (sectionMgr)
	{
		LONG depth = *(LONG*)((char*)sectionMgr + PATHQ_DEPTH_OFFSET);
		if (depth < 0) depth = 0;
		InterlockedExchangeAdd64(&g_depthSum, (LONGLONG)depth);
		InterlockedIncrement(&g_depthSamples);
		for (;;)
		{
			LONG cur = g_depthMax;
			if (depth <= cur) break;
			if (InterlockedCompareExchange(&g_depthMax, depth, cur) == cur) break;
		}
	}

	LARGE_INTEGER t0;
	QueryPerformanceCounter(&t0);
	g_passStartTicks = t0;
	// Reset this pass's tightened serve-start trackers to the pass start;
	// hook_gatesUpdateCodes / hook_dequeueWork push them later only if they
	// actually run during this pass.
	g_gatePassEndTicks = t0;
	g_lastDequeueTicks = t0;
	// Round-2 review fix: reset per pass, not left holding a stale value
	// from a previous pass that may not have run any search at all this
	// pass (see g_lastPathIterations's declaration comment above).
	g_lastPathIterations = -1;
	g_passSearchCount = 0;

	char served = orig_contentStream(sectionMgr);

	LARGE_INTEGER t1;
	QueryPerformanceCounter(&t1);

	InterlockedIncrement(&g_passCount);
	InterlockedExchangeAdd64(&g_passTotalUs, TicksToUs(t1.QuadPart - t0.QuadPart));

	t_inContentStreamPass = 0;
	return served;
}

void* hook_dequeueWork(void* queueBase)
{
	void* req = orig_dequeueWork(queueBase);

	void* mgr = g_sectionMgrPtr;
	if (req && mgr && queueBase == (char*)mgr + PATHQ_INPUT_OFFSET)
	{
		LARGE_INTEGER now;
		QueryPerformanceCounter(&now);
		*(LONGLONG*)((char*)req + REQ_STAMP_OFFSET) = now.QuadPart;
		InterlockedIncrement(&g_arrivedCount);
		// Item 8: the LAST dequeue in the drain loop is the tightest bound
		// this hook can give for "the request drain just finished" -- the
		// step immediately before serve (audit rows 6 then 7). Plain global,
		// path-thread-only, like g_passStartTicks.
		g_lastDequeueTicks = now;
	}
	return req;
}

void hook_enqueueThreadSafe(void* queueBase, void** itemPtr)
{
	void* mgr = g_sectionMgrPtr;
	void* req = itemPtr ? *itemPtr : NULL;

	if (req && mgr && queueBase == (char*)mgr + PATHQ_RESULT_OFFSET)
	{
		LONG status = *(LONG*)((char*)req + REQ_STATUS_OFFSET);
		LONGLONG stamp = *(LONGLONG*)((char*)req + REQ_STAMP_OFFSET);

		if (status != REQ_STATUS_SENTINEL && stamp != 0)
		{
			LARGE_INTEGER now;
			QueryPerformanceCounter(&now);

			LONGLONG waitUs = TicksToUs(now.QuadPart - stamp);
			PPHistAdd(&g_waitHist, waitUs);

			// Item 8: serve-start = the latest of pass start, this pass's
			// gate-pass end and this pass's last dequeue -- excludes the
			// origin shift / drainResults / work items / section adds / gate
			// pass / request drain preamble from "svc", so PathSlow ranks
			// searches, not whole passes (a gate pass alone can run to 929 ms).
			LONGLONG serveStart = g_passStartTicks.QuadPart;
			if (g_gatePassEndTicks.QuadPart > serveStart) serveStart = g_gatePassEndTicks.QuadPart;
			if (g_lastDequeueTicks.QuadPart > serveStart) serveStart = g_lastDequeueTicks.QuadPart;
			LONGLONG svcTicks = now.QuadPart - serveStart;
			if (svcTicks < 0) svcTicks = 0;
			LONGLONG svcUs = TicksToUs(svcTicks);
			PPHistAdd(&g_svcHist, svcUs);
			InterlockedExchangeAdd64(&g_svcTotalUs, svcUs);

			LONG pri = *(LONG*)((char*)req + REQ_PRIORITY_OFFSET);
			if (pri >= 45)      InterlockedIncrement(&g_priTierCount);
			else if (pri == 20) InterlockedIncrement(&g_priPlayerCount);
			else if (pri <= 10) InterlockedIncrement(&g_priNpcCount);

			// req+0x90, one counter per raw status (finding 3b).
			if (status >= 0 && status <= 4)
				InterlockedIncrement(&g_reqStatusCount[status]);

			// "direct=" (finding 3a): a status-0 completion this pass, with
			// no path-thread search having run this pass, took the step-4
			// csFindPath success path rather than the step-7 fallback/full
			// search. g_passSearchCount is path-thread-only, read here on
			// the path thread in the same pass PathPoolNoteSearch (if any)
			// already ran in, so no synchronization is needed.
			if (status == 0 && g_passSearchCount == 0)
				InterlockedIncrement(&g_directCount);

			InterlockedIncrement(&g_servedCount);

			float* startPos = (float*)((char*)req + REQ_START_OFFSET);
			float* goalPos  = (float*)((char*)req + REQ_GOAL_OFFSET);
			// g_lastPathIterations is path-thread-only (like the ticks
			// trackers above), so reading it here -- itself on the path
			// thread, in the same pass that just produced this completion
			// -- is safe without synchronization. -1 (reset every pass,
			// see its declaration) means no search ran this pass.
			PPSlowConsider(svcUs, status, pri, g_lastPathIterations,
			               startPos[0], startPos[2], goalPos[0], goalPos[2]);
		}
	}

	orig_enqueueThreadSafe(queueBase, itemPtr);
}

__int64 hook_gatesUpdateCodes(void* gatesObj)
{
	// Item 6: no LogMsg on the path thread here (reachable if contentStream's
	// own hook failed to install while this one still did). Set the flag
	// only; PathPoolTickMain reports it once from the main thread.
	if (!t_inContentStreamPass)
		InterlockedCompareExchange(&g_gateOutsideStreamWarned, 1, 0);

	bool inTx = isTransitionActive;

	LARGE_INTEGER t0;
	QueryPerformanceCounter(&t0);

	t_inGatePass = 1;
	__int64 result = orig_gatesUpdateCodes(gatesObj);
	t_inGatePass = 0;

	LARGE_INTEGER t1;
	QueryPerformanceCounter(&t1);
	LONGLONG us = TicksToUs(t1.QuadPart - t0.QuadPart);

	// Item 8: push the tightened serve-start bound past this gate pass.
	g_gatePassEndTicks = t1;

	InterlockedIncrement(&g_gatePassCount);
	InterlockedExchangeAdd64(&g_gatePassTotalUs, us);
	for (;;)
	{
		LONG cur = g_gatePassMaxUs;
		if (us <= (LONGLONG)cur) break;
		if (InterlockedCompareExchange(&g_gatePassMaxUs, (LONG)us, cur) == cur) break;
	}
	if (inTx)
		InterlockedIncrement(&g_gatePassInTransition);

	return result;
}


// =========================================================================
// §3 NPC wait diagnostic (main thread, once per second)
// =========================================================================
//
// Walks pauseState.charUpdateListMain (a boost::unordered_set<Character*>;
// same layout as ZoneManager's Set B, OFF_SET_* in game.h). The set is
// mutated by the game (character creation/destruction), so the whole walk
// runs inside one guarded, POD-only, standalone function -- MSVC 2010 forbids
// __try in a function with objects needing unwinding, and a fault here (a
// node freed mid-walk) must end the walk, not reach the crash recorder
// (core.h's GuardEnter/GuardLeave contract), mirroring core.cpp's
// ReadContainer/ProbeContainer pattern.

static const unsigned __int64 PP_MAX_SANE_BUCKETS = 0x100000;
static const int   NPC_WAIT_TABLE_SIZE = 4096;
static const int   NPC_WAIT_WALK_CAP   = 16384;   // defensive bound on list length
static const float NPC_FAR_DEST_UNITS  = 100.0f;
static const float NPC_NO_MOVE_EPS_SQ  = 0.25f;   // 0.5 units
static const double NPC_NO_MOVE_SEC    = 5.0;

struct NpcWaitEntry
{
	void*  hc;
	int    state;
	double since;      // when 'state' was last observed changing
	float  posX, posZ; // last CharMovement position sample
	double posSince;   // when the position last moved past NPC_NO_MOVE_EPS_SQ
	bool   inUse;
};

static NpcWaitEntry g_npcWaitTable[NPC_WAIT_TABLE_SIZE];

static PPHist g_finishedWaitHist;      // seconds*1e6, waits that ended this window
// Character-samples in state 6 across this window's once-per-second polls
// (NOT a count of polls: every character seen in state 6 on a given poll
// adds one sample, so N characters stuck in state 6 on the same poll add N).
// Round-2 review fix: split by player-owned so the NPC line's reissue(6)
// only counts NPCs (the increment used to run before the !isPlayer split).
static volatile LONG g_reissueSamples       = 0;  // non-player
static volatile LONG g_reissueSamplesPlayer = 0;  // player-owned

static uintptr_t g_npcZmSeen     = 0;
static bool      g_npcWasLoading = false;

static int NpcWaitHash(void* hc)
{
	unsigned __int64 v = (unsigned __int64)hc;
	v ^= v >> 15;
	v *= 0x2545F4914F6CDD1DULL;
	v ^= v >> 32;
	return (int)(v % (unsigned __int64)NPC_WAIT_TABLE_SIZE);
}

static void NpcWaitTableReset()
{
	memset(g_npcWaitTable, 0, sizeof(g_npcWaitTable));
	PPHistReset(&g_finishedWaitHist);
	InterlockedExchange(&g_reissueSamples, 0);
	InterlockedExchange(&g_reissueSamplesPlayer, 0);
}

struct NpcWaitWalkResult
{
	int waitingTotal, waiting4, waiting5;
	int failed3FarDest;
	int stoppedFarDestNoMove;
	int navWaitZoneNotReady;
	double longestWaitSec;
	int haveTop;
	float topPosX, topPosZ;
	int topGx, topGy;
	int topState;
	double topDestDist;
	int topZoneReady;      // -1 unknown, 0/1
	int playerWaitingTotal, playerWaiting4, playerWaiting5;
};

// Standalone, POD-only (MSVC 2010 __try rule). 'zoneMgr' may be NULL (no
// zone(gx,gy)/zoneReady lookups then). Everything read here is a raw offset
// off a game pointer; a fault (the set or a character being torn down mid
// walk) unwinds only this function and the walk stops where it is.
static void PPWalkCharList(void* head, double now, void* zoneMgr, NpcWaitWalkResult* r)
{
	memset(r, 0, sizeof(*r));
	r->topZoneReady = -1;

	int seen = 0;
	GuardEnter();
	__try
	{
		void* node = head;
		while (node && seen < NPC_WAIT_WALK_CAP)
		{
			void* character = *(void**)((char*)node + OFF_SET_NODE_VALUE);
			void* nextNode  = *(void**)node;
			++seen;

			if (!character)
			{
				node = nextNode;
				continue;
			}

			void* cm = *(void**)((char*)character + OFF_CHAR_MOVEMENT);
			if (!cm) { node = nextNode; continue; }
			void* hc = *(void**)((char*)cm + OFF_CMOV_HAVOK_CHAR);
			if (!hc) { node = nextNode; continue; }

			int state = *(int*)((char*)hc + 0x90);

			float posX = *(float*)((char*)cm + OFF_CMOV_POS);
			float posZ = *(float*)((char*)cm + OFF_CMOV_POS + 8);
			float destX = *(float*)((char*)cm + OFF_CMOV_LAST_DEST);
			float destZ = *(float*)((char*)cm + OFF_CMOV_LAST_DEST + 8);

			float ddx = destX - posX, ddz = destZ - posZ;
			double destDist = sqrt((double)(ddx * ddx + ddz * ddz));
			bool farDest = destDist > NPC_FAR_DEST_UNITS;

			bool isPlayer = fn_isPriorityPath && fn_isPriorityPath(character);

			int idx = NpcWaitHash(hc);
			NpcWaitEntry& e = g_npcWaitTable[idx];

			if (!e.inUse || e.hc != hc)
			{
				// New slot or collision: reset tracking for this hc (an
				// approximation under the fixed 4096-slot table -- a
				// collision loses the evicted character's "since" history
				// for this window, which is acceptable for a diagnostic).
				e.hc = hc;
				e.inUse = true;
				e.state = state;
				e.since = now;
				e.posX = posX; e.posZ = posZ; e.posSince = now;
			}
			else
			{
				if (e.state != state)
				{
					if ((e.state == 4 || e.state == 5) && !(state == 4 || state == 5))
					{
						double dur = now - e.since;
						if (dur < 0) dur = 0;
						PPHistAdd(&g_finishedWaitHist, (LONGLONG)(dur * 1000000.0));
					}
					e.state = state;
					e.since = now;
				}
				float dx = posX - e.posX, dz = posZ - e.posZ;
				if (dx * dx + dz * dz > NPC_NO_MOVE_EPS_SQ)
				{
					e.posX = posX; e.posZ = posZ; e.posSince = now;
				}
			}

			bool navWait = false;
			if ((state == 0 || state == 1) && zoneMgr)
			{
				int gx = -1, gy = -1;
				if (WorldToZoneGrid(posX, posZ, &gx, &gy))
				{
					void* zone = GetZoneEntry(zoneMgr, gx, gy);
					if (zone && *(unsigned char*)((char*)zone + OFF_ZONE_IS_ACCESS) == 0)
						navWait = true;
				}
			}

			if (!isPlayer)
			{
				if (state == 6)
					InterlockedIncrement(&g_reissueSamples);
				if (state == 4 || state == 5)
				{
					r->waitingTotal++;
					if (state == 4) r->waiting4++; else r->waiting5++;
					double waitSec = now - e.since;
					if (waitSec > r->longestWaitSec)
					{
						r->longestWaitSec = waitSec;
						r->haveTop = 1;
						r->topPosX = posX; r->topPosZ = posZ;
						r->topState = state;
						r->topDestDist = destDist;
						int gx = -1, gy = -1;
						if (WorldToZoneGrid(posX, posZ, &gx, &gy))
						{
							r->topGx = gx; r->topGy = gy;
							void* zone = zoneMgr ? GetZoneEntry(zoneMgr, gx, gy) : NULL;
							if (zone)
								r->topZoneReady = (*(unsigned char*)((char*)zone + OFF_ZONE_IS_ACCESS) != 0) ? 1 : 0;
						}
					}
				}
				if (state == 3 && farDest)
					r->failed3FarDest++;
				if ((state == 0 || state == 1) && farDest && (now - e.posSince) >= NPC_NO_MOVE_SEC)
					r->stoppedFarDestNoMove++;
				if (navWait)
					r->navWaitZoneNotReady++;
			}
			else
			{
				if (state == 6)
					InterlockedIncrement(&g_reissueSamplesPlayer);
				if (state == 4 || state == 5)
				{
					r->playerWaitingTotal++;
					if (state == 4) r->playerWaiting4++; else r->playerWaiting5++;
				}
			}

			node = nextNode;
		}
	}
	__except (EXCEPTION_EXECUTE_HANDLER)
	{
	}
	GuardLeave();
}

// Standalone, POD-only, mirrors core.cpp's ReadContainer (MSVC 2010 __try
// rule). Reads the boost::unordered_set head at pauseState+0x750.
static bool PPReadCharListHead(uintptr_t base, void** headOut)
{
	bool ok = true;
	*headOut = NULL;
	GuardEnter();
	__try
	{
		unsigned __int64 nbuckets = *(unsigned __int64*)(base + OFF_SET_BUCKET_COUNT);
		void** buckets = *(void***)(base + OFF_SET_BUCKETS);
		if (buckets && nbuckets < PP_MAX_SANE_BUCKETS)
			*headOut = buckets[nbuckets];
		else if (nbuckets >= PP_MAX_SANE_BUCKETS)
			ok = false;
	}
	__except (EXCEPTION_EXECUTE_HANDLER)
	{
		ok = false;
	}
	GuardLeave();
	return ok;
}

// pauseState.charUpdateListMain: pauseState+0x750 (IDA-confirmed 2026-09-12,
// 2 references, one of them consumePathResults' own sentinel-branch walk of
// the same list). pauseState itself is RVA_GLOBAL_GAMEWORLD (game.h).
static const size_t OFF_PAUSESTATE_CHAR_UPDATE_LIST_MAIN = 0x750;

static NpcWaitWalkResult g_lastWalk;
static bool              g_haveLastWalk = false;

static void RunNpcWaitDiagnostic(double now)
{
	void* zoneMgr = g_cachedZoneMgr;

	// Save-load / new ZoneManager reset, mirroring islands.cpp:1251-1266
	// (ZM+8 rising edge or a new ZoneManager pointer) without including
	// islands.h.
	uintptr_t zm = (uintptr_t)zoneMgr;
	bool loading = false;
	if (zm)
	{
		loading = *(unsigned char*)(zm + OFF_ZM_LOADING) != 0;
		if (zm != g_npcZmSeen || (loading && !g_npcWasLoading))
		{
			NpcWaitTableReset();
			g_npcZmSeen = zm;
			g_haveLastWalk = false;
		}
		g_npcWasLoading = loading;
	}

	// Item 9: skip the walk itself while loading (ZM+8 set), same as
	// islands.cpp's WalkSetB/IslandTick gate -- the character list and
	// CharMovement/HavokCharacter fields it reads are least stable exactly
	// while a save or zone load is in progress.
	if (loading)
		return;

	if (!gameBase)
		return;

	void* head = NULL;
	if (!PPReadCharListHead(gameBase + RVA_GLOBAL_GAMEWORLD + OFF_PAUSESTATE_CHAR_UPDATE_LIST_MAIN, &head))
		return;
	if (!head)
		return;

	NpcWaitWalkResult r;
	PPWalkCharList(head, now, zoneMgr, &r);
	g_lastWalk = r;
	g_haveLastWalk = true;
}


// =========================================================================
// §3 Window printers (main thread only, called from PathPoolTickMain)
// =========================================================================

// Item 3: PathQueue's gate= mirror and the GateRate: line must agree, so the
// gate counters are snapshotted-and-reset exactly once per window regardless
// of whether GateRate: itself is printed (gatePassDiagEnabled off just means
// every field stays 0, honestly, since the gate hook then isn't installed).
struct GateWindowStats
{
	LONG     n;
	LONGLONG totalUs;
	LONG     maxUs;
	LONG     inTransition;
	LONG     iterLimit;
	LONG     stateFull;
	LONG     searches;
};

static GateWindowStats SnapshotAndResetGateStats()
{
	GateWindowStats g;
	g.n            = InterlockedExchange(&g_gatePassCount, 0);
	g.totalUs      = InterlockedExchange64(&g_gatePassTotalUs, 0);
	g.maxUs        = InterlockedExchange(&g_gatePassMaxUs, 0);
	g.inTransition = InterlockedExchange(&g_gatePassInTransition, 0);
	g.iterLimit    = InterlockedExchange(&g_gateIterLimit, 0);
	g.stateFull    = InterlockedExchange(&g_gateStateFull, 0);
	g.searches     = InterlockedExchange(&g_gateSearchCount, 0);
	return g;
}

static void PrintPathQueueLine(double windowSec, const GateWindowStats& gws, LONG* servedOut)
{
	LONG arrived = InterlockedExchange(&g_arrivedCount, 0);
	LONG served = InterlockedExchange(&g_servedCount, 0);
	*servedOut = served;
	LONG passes = InterlockedExchange(&g_passCount, 0);
	LONGLONG passUs = InterlockedExchange64(&g_passTotalUs, 0);

	LONG depthMax = InterlockedExchange(&g_depthMax, 0);
	LONGLONG depthSum = InterlockedExchange64(&g_depthSum, 0);
	LONG depthSamples = InterlockedExchange(&g_depthSamples, 0);
	double depthAvg = (depthSamples > 0) ? ((double)depthSum / (double)depthSamples) : 0.0;

	double waitP50 = PPHistPercentileUs(&g_waitHist, 0.50) / 1000.0;
	double waitP90 = PPHistPercentileUs(&g_waitHist, 0.90) / 1000.0;
	double waitP99 = PPHistPercentileUs(&g_waitHist, 0.99) / 1000.0;
	double waitMax = InterlockedExchange(&g_waitHist.maxUs, 0) / 1000.0;

	double svcP50 = PPHistPercentileUs(&g_svcHist, 0.50) / 1000.0;
	double svcP90 = PPHistPercentileUs(&g_svcHist, 0.90) / 1000.0;
	double svcP99 = PPHistPercentileUs(&g_svcHist, 0.99) / 1000.0;
	double svcMax = InterlockedExchange(&g_svcHist.maxUs, 0) / 1000.0;
	LONGLONG svcTotalUs = InterlockedExchange64(&g_svcTotalUs, 0);
	PPHistReset(&g_waitHist);
	PPHistReset(&g_svcHist);

	LONG npcN    = InterlockedExchange(&g_priNpcCount, 0);
	LONG playerN = InterlockedExchange(&g_priPlayerCount, 0);
	LONG tierN   = InterlockedExchange(&g_priTierCount, 0);

	LONG st0 = InterlockedExchange(&g_reqStatusCount[0], 0);
	LONG st1 = InterlockedExchange(&g_reqStatusCount[1], 0);
	LONG st2 = InterlockedExchange(&g_reqStatusCount[2], 0);
	LONG st3 = InterlockedExchange(&g_reqStatusCount[3], 0);
	LONG st4 = InterlockedExchange(&g_reqStatusCount[4], 0);
	LONG direct = InterlockedExchange(&g_directCount, 0);

	LONG termIterLimit   = InterlockedExchange(&g_pathTermIterLimit, 0);
	LONG termOpenSetFull = InterlockedExchange(&g_pathTermOpenSetFull, 0);
	LONG termStateFull   = InterlockedExchange(&g_pathTermStateFull, 0);
	LONG termOther       = InterlockedExchange(&g_pathTermOther, 0);
	LONG astarOk   = InterlockedExchange(&g_pathSearchOk, 0);
	LONG astarFail = InterlockedExchange(&g_pathSearchFail, 0);

	// preamble = (pass total) - (the tightened svc total). svc's serve-start
	// bound (hook_enqueueThreadSafe) is max(passStart, gatePassEnd,
	// lastDequeue this pass), so svc excludes the preamble (origin shift,
	// drainResults, work items, section adds, the gate pass, the request
	// drain -- audit §2 rows 1-6) ONLY when a dequeue happened this pass
	// (IDA-confirmed: the drain loop is skipped entirely when the input
	// queue is empty, contentStream+0x848 area / 0x3AEB72-0x3AEB82) or a
	// gate pass ran (only when section adds finish, 0x3AEB30). A backlog
	// pass -- one that serves an older queued request with nothing new
	// arriving this pass -- or any PROD pass with the gate hook off
	// (gatePassDiag=false by default) still has svc == the whole pass, so
	// its preamble is folded into svc, not into this field, for that pass.
	double passMs = passUs / 1000.0;
	double preambleMs = (double)(passUs - svcTotalUs) / 1000.0;
	if (preambleMs < 0.0) preambleMs = 0.0;
	double busyPct = (windowSec > 0.0) ? (passMs / (windowSec * 1000.0) * 100.0) : 0.0;
	double preambleMsPerSec = (windowSec > 0.0) ? (preambleMs / windowSec) : 0.0;

	std::ostringstream ss;
	ss << std::fixed << std::setprecision(1);
	ss << "[ZoneOpt] PathQueue: arrived=" << arrived << " served=" << served
	   << " replaced=- direct=" << direct
	   << " passes=" << passes
	   << " depth max/avg=" << depthMax << "/" << depthAvg
	   << " wait p50/p90/p99/max=" << waitP50 << "/" << waitP90 << "/" << waitP99 << "/" << waitMax << "ms\n"
	   // Round 2 final review cheap minor: every continuation line carries the
	   // [ZoneOpt] tag too, so a grep on the tag finds all of them, not just
	   // the first line of the block.
	   << "[ZoneOpt]   npc(10)=" << npcN << " player(20)=" << playerN << " tier(45+)=" << tierN
	   << " svc p50/p90/p99/max=" << svcP50 << "/" << svcP90 << "/" << svcP99 << "/" << svcMax << "ms\n"
	   << "[ZoneOpt]   st0=" << st0 << " st1=" << st1 << " st2=" << st2 << " st3=" << st3 << " st4=" << st4
	   << " (0 path/1 noStartFace/2 noGoalFace/3 notConnected/4 fallbackFailed;"
	   << " st3 is ~0 at PATHFIND_STEP>=3, the Phase 10 bypass)\n"
	   << "[ZoneOpt]   astarOk=" << astarOk << " astarFail=" << astarFail
	   << " term=" << termIterLimit << "/" << termOpenSetFull << "/" << termStateFull << "/" << termOther
	   << " (iterLimit/openSetFull/stateFull/other, path-thread queue searches only)\n"
	   << "[ZoneOpt]   busy=" << busyPct << "% preamble=" << preambleMsPerSec << "ms/s gate=";
	if (gatePassDiagEnabled)
	{
		double gateTotalMs = gws.totalUs / 1000.0;
		double gateMaxMs = gws.maxUs / 1000.0;
		ss << gws.n << "/" << gateTotalMs << "ms/" << gateMaxMs << "ms";
	}
	else
	{
		ss << "-";
	}
	ss << " origShift=- drain=- workItems=- sections=-";
	LogMsg(ss.str());
}

static void PrintGateRateLine(const GateWindowStats& gws)
{
	double totalMs = gws.totalUs / 1000.0;
	double meanMs = (gws.n > 0) ? (totalMs / (double)gws.n) : 0.0;
	double maxMs = gws.maxUs / 1000.0;

	std::ostringstream ss;
	ss << std::fixed << std::setprecision(1);
	ss << "[ZoneOpt] GateRate: n=" << gws.n << " total=" << totalMs << "ms mean=" << meanMs
	   << "ms max=" << maxMs << "ms searches=" << gws.searches
	   << " iterLimit=" << gws.iterLimit << " stateFull=" << gws.stateFull
	   << " inTransition=" << gws.inTransition;
	LogMsg(ss.str());
}

static void PrintPathSlowLine(LONG servedThisWindow)
{
	// Item 4: the working array's reset only runs on the path thread's next
	// insert, so a window with zero completions would otherwise reprint the
	// previous window's (still-published) top 5 as if it were current.
	// servedThisWindow is this window's real completion count (from
	// PrintPathQueueLine, computed in the same tick) -- skip entirely when
	// it's 0, regardless of what is still sitting in g_slowPublished.
	if (servedThisWindow <= 0)
	{
		InterlockedExchange(&g_slowResetRequested, 1);
		LogMsg("[ZoneOpt] PathSlow: (no completions this window)");
		return;
	}

	PPSlowEntry entries[PP_SLOW_N];
	int count = PPSlowSnapshot(entries);
	InterlockedExchange(&g_slowResetRequested, 1);

	if (count <= 0)
	{
		LogMsg("[ZoneOpt] PathSlow: (no completions this window)");
		return;
	}

	// Sort descending by svcUs (insertion sort, count <= 5).
	for (int i = 1; i < count; ++i)
	{
		PPSlowEntry key = entries[i];
		int j = i - 1;
		while (j >= 0 && entries[j].svcUs < key.svcUs)
		{
			entries[j + 1] = entries[j];
			--j;
		}
		entries[j + 1] = key;
	}

	std::ostringstream ss;
	ss << std::fixed << std::setprecision(1);
	ss << "[ZoneOpt] PathSlow: top " << count << " by svc ms (start/goal in Havok units, world x 0.1):";
	for (int i = 0; i < count; ++i)
	{
		ss << " #" << (i + 1) << "[svc=" << (entries[i].svcUs / 1000.0) << "ms"
		   << " status=" << entries[i].status
		   << " pri=" << entries[i].priority
		   << " iter=";
		if (entries[i].iterations < 0)
			ss << "-";
		else
			ss << entries[i].iterations;
		ss << " startHU=(" << entries[i].startX << "," << entries[i].startZ << ")"
		   << " goalHU=(" << entries[i].goalX << "," << entries[i].goalZ << ")]";
	}
	LogMsg(ss.str());
}

static void PrintAstarCostLine()
{
	PPClassStats* path = &g_classStats[PP_CLASS_PATH];
	LONG pathN = InterlockedExchange(&path->count, 0);
	LONGLONG pathTicks = InterlockedExchange64(&path->totalTicks, 0);
	LONGLONG pathIters = InterlockedExchange64(&path->totalIterations, 0);
	double pathMeanMs = (pathN > 0) ? (TicksToUs(pathTicks) / 1000.0 / (double)pathN) : 0.0;
	double pathMeanIter = (pathN > 0) ? ((double)pathIters / (double)pathN) : 0.0;
	double pathLatP50 = PPHistPercentileUs(&path->latencyUs, 0.50) / 1000.0;
	double pathLatP90 = PPHistPercentileUs(&path->latencyUs, 0.90) / 1000.0;
	PPHistReset(&path->latencyUs);
	// Item 2: nanoseconds, not microseconds -- per-iteration cost is
	// typically tens to low thousands of ns, which used to collapse into
	// histogram bucket 0/1 (whole microseconds) and always read ~1.0/1.0.
	double pathNsP50 = (double)PPHistPercentileUs(&path->iterNsHist, 0.50);
	double pathNsP90 = (double)PPHistPercentileUs(&path->iterNsHist, 0.90);
	PPHistReset(&path->iterNsHist);

	// Item 2: nav/main/other's totals are no longer summed-then-discarded --
	// a mean latency and mean iteration count per class, so the
	// accumulation (totalTicks/totalIterations) is actually used.
	PPClassStats* nav = &g_classStats[PP_CLASS_NAVMESH];
	LONG navN = InterlockedExchange(&nav->count, 0);
	LONGLONG navTicks = InterlockedExchange64(&nav->totalTicks, 0);
	LONGLONG navIters = InterlockedExchange64(&nav->totalIterations, 0);
	double navMeanMs = (navN > 0) ? (TicksToUs(navTicks) / 1000.0 / (double)navN) : 0.0;
	double navMeanIter = (navN > 0) ? ((double)navIters / (double)navN) : 0.0;
	PPHistReset(&nav->latencyUs);
	PPHistReset(&nav->iterNsHist);

	PPClassStats* mainCls = &g_classStats[PP_CLASS_MAIN];
	LONG mainN = InterlockedExchange(&mainCls->count, 0);
	LONGLONG mainTicks = InterlockedExchange64(&mainCls->totalTicks, 0);
	LONGLONG mainIters = InterlockedExchange64(&mainCls->totalIterations, 0);
	double mainMeanMs = (mainN > 0) ? (TicksToUs(mainTicks) / 1000.0 / (double)mainN) : 0.0;
	double mainMeanIter = (mainN > 0) ? ((double)mainIters / (double)mainN) : 0.0;
	PPHistReset(&mainCls->latencyUs);
	PPHistReset(&mainCls->iterNsHist);

	PPClassStats* other = &g_classStats[PP_CLASS_OTHER];
	LONG otherN = InterlockedExchange(&other->count, 0);
	LONGLONG otherTicks = InterlockedExchange64(&other->totalTicks, 0);
	LONGLONG otherIters = InterlockedExchange64(&other->totalIterations, 0);
	double otherMeanMs = (otherN > 0) ? (TicksToUs(otherTicks) / 1000.0 / (double)otherN) : 0.0;
	double otherMeanIter = (otherN > 0) ? ((double)otherIters / (double)otherN) : 0.0;
	PPHistReset(&other->latencyUs);
	PPHistReset(&other->iterNsHist);

	std::ostringstream ss;
	ss << std::fixed << std::setprecision(1);
	ss << "[ZoneOpt] AstarCost: path-thread n=" << pathN
	   << " lat p50/p90/mean=" << pathLatP50 << "/" << pathLatP90 << "/" << pathMeanMs << "ms"
	   << " meanIter=" << pathMeanIter
	   << " ns/iter p50/p90=" << pathNsP50 << "/" << pathNsP90
	   << " ; nav n=" << navN << " meanMs=" << navMeanMs << " meanIter=" << navMeanIter
	   << " ; main n=" << mainN << " meanMs=" << mainMeanMs << " meanIter=" << mainMeanIter
	   << " ; other n=" << otherN << " meanMs=" << otherMeanMs << " meanIter=" << otherMeanIter;

	// Boost counters (O9): outcome 0=successHigh(>32768 iter) 1=successLow 2=fail.
	static const char* kOutcome[3] = { "high", "low", "fail" };
	ss << " ; boost:";
	for (int o = 0; o < 3; ++o)
	{
		LONG tagNpc    = InterlockedExchange(&g_boostByTag[o][0], 0);
		LONG tagPlayer = InterlockedExchange(&g_boostByTag[o][1], 0);
		LONG reqNpc    = InterlockedExchange(&g_boostByReq[o][0], 0);
		LONG reqPlayer = InterlockedExchange(&g_boostByReq[o][1], 0);
		LONG reqUnk    = InterlockedExchange(&g_boostByReq[o][2], 0);
		ss << " " << kOutcome[o] << "(tag npc=" << tagNpc << "/player=" << tagPlayer
		   << " req npc=" << reqNpc << "/player=" << reqPlayer << "/unk=" << reqUnk << ")";
	}
	LONG disagree = InterlockedExchange(&g_boostDisagree, 0);
	ss << " disagree=" << disagree;
	LogMsg(ss.str());
}

static void PrintNpcPathWaitLine(double windowSec)
{
	if (!g_haveLastWalk)
		return;

	NpcWaitWalkResult r = g_lastWalk;

	double finP50 = PPHistPercentileUs(&g_finishedWaitHist, 0.50) / 1000000.0;
	double finP90 = PPHistPercentileUs(&g_finishedWaitHist, 0.90) / 1000000.0;
	double finMax = InterlockedExchange(&g_finishedWaitHist.maxUs, 0) / 1000000.0;
	PPHistReset(&g_finishedWaitHist);

	LONG reissueSamples       = InterlockedExchange(&g_reissueSamples, 0);
	LONG reissueSamplesPlayer = InterlockedExchange(&g_reissueSamplesPlayer, 0);
	// reissue(6) is character-samples in state 6 across this window's
	// once-per-second polls (not a poll count: N characters seen in state 6
	// on one poll add N, not 1), normalised to a /10s rate using windowSec
	// (measured by PathPoolTickMain from the real interval between prints,
	// so this is correct in both DEV's 10s and PROD's 30s window, and on a
	// shorter first window). Split by player-owned (round-2 review fix: the
	// count used to run before the !isPlayer split and land entirely on
	// this NPC line regardless of ownership).
	double reissueRate10s       = (windowSec > 0.0) ? ((double)reissueSamples * 10.0 / windowSec) : 0.0;
	double reissueRate10sPlayer = (windowSec > 0.0) ? ((double)reissueSamplesPlayer * 10.0 / windowSec) : 0.0;

	std::ostringstream ss;
	ss << std::fixed << std::setprecision(1);
	ss << "[ZoneOpt] NpcPathWait: waiting=" << r.waitingTotal
	   << " (4:" << r.waiting4 << " 5:" << r.waiting5 << ")"
	   << " finished p50/p90/max=" << finP50 << "/" << finP90 << "/" << finMax << "s"
	   << " longest=" << r.longestWaitSec << "s\n"
	   // Round 2 final review cheap minor: [ZoneOpt] tag on every continuation
	   // line, same reasoning as PrintPathQueueLine above.
	   << "[ZoneOpt]   failed(3,farDest)=" << r.failed3FarDest
	   << " reissue(6)=" << reissueRate10s << "/10s"
	   << " stopped(farDest,st0/1,noMove5s)=" << r.stoppedFarDestNoMove
	   << " navWait(st0/1,zoneNotReady)=" << r.navWaitZoneNotReady;
	if (r.haveTop)
	{
		ss << "\n[ZoneOpt]   top: (" << r.topPosX << "," << r.topPosZ << ")"
		   << " zone(" << r.topGx << "," << r.topGy << ")"
		   << " state=" << r.topState
		   << " destDist=" << r.topDestDist
		   << " zoneReady=" << r.topZoneReady;
	}
	ss << "\n[ZoneOpt]   player: waiting=" << r.playerWaitingTotal
	   << " (4:" << r.playerWaiting4 << " 5:" << r.playerWaiting5 << ")"
	   << " reissue(6)=" << reissueRate10sPlayer << "/10s";
	LogMsg(ss.str());
}


// =========================================================================
// PathPoolTickMain (main thread; called every frame from hook_updateCameraZone)
// =========================================================================

void PathPoolTickMain(double now)
{
	static double lastWindowTime = -1.0e9;
	static double lastNpcWaitTime = -1.0e9;
	static bool   gateOutsideStreamReported = false;

	// Item 6: hook_gatesUpdateCodes only sets the flag (path thread); the
	// actual LogMsg happens here, on the main thread, once.
	if (!gateOutsideStreamReported
		&& InterlockedCompareExchange(&g_gateOutsideStreamWarned, 0, 0) != 0)
	{
		gateOutsideStreamReported = true;
		LogMsg("[ZoneOpt] PathPool: Gates__updateCodes fired outside a contentStream pass "
		       "(unexpected -- contentStream is documented as its sole caller)");
	}

#ifdef ZONEOPT_DEBUG
	const double windowInterval = 10.0;
#else
	const double windowInterval = 30.0;
#endif

	if (npcWaitDiagEnabled && (now - lastNpcWaitTime >= 1.0))
	{
		lastNpcWaitTime = now;
		RunNpcWaitDiagnostic(now);
	}

	if (now - lastWindowTime < windowInterval)
		return;
	double windowSec = (lastWindowTime > -1.0e8) ? (now - lastWindowTime) : windowInterval;
	lastWindowTime = now;

	// Item 3: snapshot-and-reset the gate counters exactly once, so
	// PathQueue's gate= mirror and GateRate: (when printed) agree.
	GateWindowStats gws = SnapshotAndResetGateStats();

	LONG servedThisWindow = 0;
	PrintPathQueueLine(windowSec, gws, &servedThisWindow);
	if (gatePassDiagEnabled)
		PrintGateRateLine(gws);
	PrintPathSlowLine(servedThisWindow);
	PrintAstarCostLine();
	if (npcWaitDiagEnabled)
		PrintNpcPathWaitLine(windowSec);
}

#endif // PATHPOOL_STEP >= 1
