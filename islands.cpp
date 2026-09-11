// islands.cpp — Island routing overlay (Phase 15)
//
// The game routes long moves through "islands": connected groups of Set B
// zones labelled by ZoneManager::_calculateIslands. Zones this mod promotes
// never enter Set B, so they keep label 0 and the router either stops at the
// island edge (symptom a) or asks for a direct path that comes back
// UNREACHABLE (symptom b). This module builds "mod components" on the main
// thread (accessible Set B zones + eligible mod zones, joined over the static
// neighbors[4] links) and answers ZoneMap::isInIsland / ZoneManager::getIsland
// from them. ZoneMap+0x20 and _calculateIslands are never touched, so spawn
// checks keep seeing today's labels.
//
// See islands.h for the threading contract and the ISLAND_STEP staging table.

#include "islands.h"
#include "preload.h"
#include "formation.h"
#include <intrin.h>


// =========================================================================
// Step 0: per-thread readiness-gate counters (any thread)
// =========================================================================

namespace {

const int    READINESS_TID_SLOTS = 8;
const double DIAG_INTERVAL_SEC   = 5.0;

struct ReadinessTid {
	volatile LONG tid;
	volatile LONG isMain;
	volatile long calls;
	volatile long notReady;
	volatile long notReadySec;
};

ReadinessTid  g_readTids[READINESS_TID_SLOTS];
volatile long g_readTidOverflow = 0;

} // namespace

void IslandCountReadiness(bool notReady, bool sectionsPending)
{
	LONG tid = (LONG)GetCurrentThreadId();
	for (int i = 0; i < READINESS_TID_SLOTS; ++i)
	{
		LONG prev = InterlockedCompareExchange(&g_readTids[i].tid, tid, 0);
		if (prev == 0)
		{
			// Claimed this slot. Record whether it is the game main thread.
			InterlockedExchange(&g_readTids[i].isMain, IsMainThread() ? 1 : 0);
			prev = tid;
		}
		if (prev == tid)
		{
			InterlockedIncrement(&g_readTids[i].calls);
			if (notReady)
			{
				InterlockedIncrement(&g_readTids[i].notReady);
				if (sectionsPending)
					InterlockedIncrement(&g_readTids[i].notReadySec);
			}
			return;
		}
	}
	InterlockedIncrement(&g_readTidOverflow);
}

static void AppendReadinessTids(std::ostringstream& ss)
{
	ss << " tids=[";
	bool first = true;
	for (int i = 0; i < READINESS_TID_SLOTS; ++i)
	{
		LONG tid = InterlockedCompareExchange(&g_readTids[i].tid, 0, 0);
		if (tid == 0) continue;
		if (!first) ss << " ";
		first = false;
		ss << tid
		   << (InterlockedCompareExchange(&g_readTids[i].isMain, 0, 0) ? "M:" : ":")
		   << InterlockedCompareExchange(&g_readTids[i].calls, 0, 0) << "/"
		   << InterlockedCompareExchange(&g_readTids[i].notReady, 0, 0) << "/"
		   << InterlockedCompareExchange(&g_readTids[i].notReadySec, 0, 0);
	}
	long ovf = InterlockedCompareExchange(&g_readTidOverflow, 0, 0);
	if (ovf > 0)
		ss << " +" << ovf << " unslotted";
	ss << "]";
}


#if ISLAND_STEP >= 1

// =========================================================================
// Constants + shared helpers
// =========================================================================

namespace {

const int MAX_COMPS       = 64;               // components published per snapshot
const int MAX_MEMBERS     = ZONE_GRID_COUNT;  // every zone at most once
const int HOOK_COPY_MAX   = 512;              // getIsland stack copy bound (plan)
const double ELIGIBILITY_INTERVAL = 0.25;

// Zone index (y + x*64) from a ZoneMap pointer, or -1 if it is not an entry of
// this ZoneManager's zone array. Pure arithmetic: safe on any thread.
inline int ZoneIndexOf(uintptr_t zm, uintptr_t z)
{
	if (!zm || !z) return -1;
	uintptr_t base = zm + OFF_ZM_ZONE_BASE;
	if (z < base) return -1;
	uintptr_t off = z - base;
	if (off % ZONE_ENTRY_SIZE) return -1;
	uintptr_t idx = off / ZONE_ENTRY_SIZE;
	if (idx >= (uintptr_t)ZONE_GRID_COUNT) return -1;
	return (int)idx;
}

inline uintptr_t ZoneAt(uintptr_t zm, int idx)
{
	return zm + OFF_ZM_ZONE_BASE + (uintptr_t)ZONE_ENTRY_SIZE * (uintptr_t)idx;
}

inline int ZoneLabel(uintptr_t z)     { return *(int*)(z + OFF_ZONE_ISLAND); }
inline bool ZoneAccess(uintptr_t z)   { return *(unsigned char*)(z + OFF_ZONE_IS_ACCESS) != 0; }
inline bool ZoneLoadingF(uintptr_t z) { return *(unsigned char*)(z + OFF_ZONE_IS_LOADING) != 0; }
inline int IdxGX(int idx) { return idx / 64; }
inline int IdxGY(int idx) { return idx % 64; }


// =========================================================================
// Published snapshot (double buffer + per-buffer seqlock)
// =========================================================================

struct IslandSnapshot {
	volatile LONG  seq;          // odd while being written
	volatile LONG  valid;        // 0 = no components (save load / no zm)
	uintptr_t      zm;           // ZoneManager the indices refer to
	unsigned int   gen;
	int            compCount;
	short          comp[ZONE_GRID_COUNT];        // component id per zone, -1 = none
	unsigned short memberStart[MAX_COMPS];
	unsigned short memberCount[MAX_COMPS];
	unsigned short members[MAX_MEMBERS];         // zone indices grouped by component
};

IslandSnapshot g_snap[2];
volatile LONG  g_activeSnap = 0;

// Hook counters
volatile long g_isInCalls      = 0;
volatile long g_isInFlips      = 0;   // answer differs from vanilla (would-flip when passing through)
volatile long g_isInSeqFail    = 0;
volatile long g_getIslCalls    = 0;
volatile long g_getIslAppended = 0;   // members appended (would-append when passing through)
volatile long g_getIslFallback = 0;   // seqlock failure / oversize -> original's answer
volatile long g_hooksInstalled = 0;

// Seqlock read of comp[] for up to two zones. b may be 0.
bool SnapReadComps(uintptr_t a, uintptr_t b, int* ca, int* cb)
{
	for (int attempt = 0; attempt < 2; ++attempt)
	{
		LONG idx = g_activeSnap;
		const IslandSnapshot* s = &g_snap[idx & 1];
		LONG seq1 = s->seq;
		if (seq1 & 1) continue;
		_ReadWriteBarrier();
		LONG valid = s->valid;
		uintptr_t zm = s->zm;
		int ra = -1, rb = -1;
		if (valid)
		{
			int ia = ZoneIndexOf(zm, a);
			int ib = ZoneIndexOf(zm, b);
			if (ia >= 0) ra = s->comp[ia];
			if (ib >= 0) rb = s->comp[ib];
		}
		_ReadWriteBarrier();
		LONG seq2 = s->seq;
		if (seq1 != seq2) continue;
		*ca = ra;
		*cb = rb;
		return true;
	}
	return false;
}

// Seqlock copy of the member list of t's component into buf.
// Returns member count, 0 with *outComp = -1 when t is in no component,
// or -1 on seqlock failure / oversize (caller falls back to the original).
int SnapCopyMembers(uintptr_t t, int* outComp, uintptr_t* outZm,
                    unsigned short* buf, int maxCount)
{
	for (int attempt = 0; attempt < 2; ++attempt)
	{
		LONG idx = g_activeSnap;
		const IslandSnapshot* s = &g_snap[idx & 1];
		LONG seq1 = s->seq;
		if (seq1 & 1) continue;
		_ReadWriteBarrier();
		LONG valid = s->valid;
		uintptr_t zm = s->zm;
		int comp = -1;
		int n = 0;
		bool oversize = false;
		if (valid)
		{
			int it = ZoneIndexOf(zm, t);
			if (it >= 0) comp = s->comp[it];
			if (comp >= 0 && comp < MAX_COMPS)
			{
				int start = s->memberStart[comp];
				n = s->memberCount[comp];
				if (n > maxCount || start + n > MAX_MEMBERS) { oversize = true; n = 0; }
				else memcpy(buf, &s->members[start], n * sizeof(unsigned short));
			}
			else if (comp >= MAX_COMPS)
				comp = -1;
		}
		_ReadWriteBarrier();
		LONG seq2 = s->seq;
		if (seq1 != seq2) continue;
		if (oversize) return -1;
		*outComp = comp;
		*outZm = zm;
		return n;
	}
	return -1;
}

// Append a ZoneMap* to a lektor<ZoneMap*> exactly as ZoneManager::getIsland does.
bool AppendLektor(uintptr_t lek, uintptr_t z)
{
	unsigned int count = *(unsigned int*)(lek + OFF_LEKTOR_COUNT);
	unsigned int cap   = *(unsigned int*)(lek + OFF_LEKTOR_CAPACITY);
	if (count >= cap)
	{
		if (!fn_lektorReserve) return false;
		fn_lektorReserve((void*)lek, 2 * cap);   // 0 -> 10 inside the game
		cap = *(unsigned int*)(lek + OFF_LEKTOR_CAPACITY);
		if (count >= cap) return false;
	}
	uintptr_t* data = *(uintptr_t**)(lek + OFF_LEKTOR_DATA);
	if (!data) return false;
	data[count] = z;
	*(unsigned int*)(lek + OFF_LEKTOR_COUNT) = count + 1;
	return true;
}


// =========================================================================
// Builder state (main thread only)
// =========================================================================

unsigned char g_marks[ZONE_GRID_COUNT];   // 1 = promoted by this mod (survives ClearPreloadZones)
int           g_markCount = 0;
uintptr_t     g_builderZm = 0;
unsigned int  g_snapGen   = 0;
unsigned int  g_setBSig   = 0;
bool          g_haveSig   = false;
bool          g_rebuildRequested = false;
double        g_lastEligibility  = -1.0;
bool          g_wasLoading       = false;

// Last Set B walk (for unexpl + the vanilla router list)
unsigned short g_setBIdx[ZONE_GRID_COUNT];
int            g_setBCount = 0;
int            g_setBAccessible = 0;
unsigned char  g_inSetB[ZONE_GRID_COUNT];

// Current published result (main-thread mirror for comparison + diagnostics)
short          g_curComp[ZONE_GRID_COUNT];
unsigned short g_curMemberStart[MAX_COMPS];
unsigned short g_curMemberCount[MAX_COMPS];
unsigned short g_curMembers[MAX_MEMBERS];
unsigned char  g_curIsMod[ZONE_GRID_COUNT];
int            g_curCompCount = 0;
bool           g_curValid = false;
int            g_curModZones = 0;
int            g_compOverflow = 0;
bool           g_inputsLogged = false;

// Scratch for a rebuild
short          g_ufParent[ZONE_GRID_COUNT];
unsigned char  g_included[ZONE_GRID_COUNT];
unsigned char  g_isMod[ZONE_GRID_COUNT];
short          g_newComp[ZONE_GRID_COUNT];
unsigned short g_newMemberStart[MAX_COMPS];
unsigned short g_newMemberCount[MAX_COMPS];
unsigned short g_newMembers[MAX_MEMBERS];
short          g_rootComp[ZONE_GRID_COUNT];

int UfFind(int i)
{
	while (g_ufParent[i] != i)
	{
		g_ufParent[i] = g_ufParent[g_ufParent[i]];
		i = g_ufParent[i];
	}
	return i;
}

void UfUnion(int a, int b)
{
	int ra = UfFind(a), rb = UfFind(b);
	if (ra == rb) return;
	if (ra < rb) g_ufParent[rb] = ra; else g_ufParent[ra] = rb;
}

// Walk Set B (main thread). Fills g_setBIdx / g_inSetB and the signature.
void WalkSetB(uintptr_t zm)
{
	g_setBCount = 0;
	g_setBAccessible = 0;
	memset(g_inSetB, 0, sizeof(g_inSetB));

	uintptr_t setB = zm + OFF_ZM_SET_B;
	unsigned long long size = *(unsigned long long*)(setB + OFF_SET_SIZE);
	unsigned int sum = 0;
	if (size > 0 && size <= (unsigned long long)ZONE_GRID_COUNT * 4)
	{
		unsigned long long bucketCount = *(unsigned long long*)(setB + OFF_SET_BUCKET_COUNT);
		uintptr_t buckets = *(uintptr_t*)(setB + OFF_SET_BUCKETS);
		if (buckets && bucketCount > 0 && bucketCount < (1ull << 24))
		{
			uintptr_t node = *(uintptr_t*)(buckets + 8 * bucketCount);
			int iter = 0;
			int maxIter = (int)size + 16;
			while (node && iter++ < maxIter)
			{
				uintptr_t z = *(uintptr_t*)(node + OFF_SET_NODE_VALUE);
				int idx = ZoneIndexOf(zm, z);
				if (idx >= 0 && !g_inSetB[idx] && g_setBCount < ZONE_GRID_COUNT)
				{
					g_inSetB[idx] = 1;
					g_setBIdx[g_setBCount++] = (unsigned short)idx;
					sum += (unsigned int)idx;
					if (ZoneAccess(z)) g_setBAccessible++;
				}
				node = *(uintptr_t*)node;
			}
		}
	}
	g_setBSig = (unsigned int)size * 0x9E3779B1u
	          ^ sum * 0x85EBCA6Bu
	          ^ (unsigned int)g_setBAccessible * 0xC2B2AE35u;
}

// Anchors for eligibility: camera zone + every player character's zone.
int CollectAnchors(uintptr_t zm, int* ax, int* ay, int maxAnchors)
{
	int n = 0;
	if (lastCameraGX >= 0 && lastCameraGY >= 0 && n < maxAnchors)
	{
		ax[n] = lastCameraGX; ay[n] = lastCameraGY; n++;
	}
	else
	{
		uintptr_t cz = *(uintptr_t*)(zm + OFF_ZM_CURRENT_ZONE);
		int idx = ZoneIndexOf(zm, cz);
		if (idx >= 0 && n < maxAnchors) { ax[n] = IdxGX(idx); ay[n] = IdxGY(idx); n++; }
	}
	uintptr_t playerIntf = *(uintptr_t*)(gameBase + RVA_GLOBAL_PLAYER);
	if (playerIntf)
	{
		unsigned int scCount = GetPlayerCharCount(playerIntf);
		uintptr_t* scStuff = GetPlayerCharStuff(playerIntf);
		if (scStuff && scCount > 0 && scCount <= 200)
		{
			for (unsigned int j = 0; j < scCount && n < maxAnchors; ++j)
			{
				if (!scStuff[j]) continue;
				int gx, gy;
				if (WorldToZoneGrid(GetCharPosX(scStuff[j]), GetCharPosZ(scStuff[j]), &gx, &gy))
				{
					ax[n] = gx; ay[n] = gy; n++;
				}
			}
		}
	}
	return n;
}

bool NearAnyAnchor(int gx, int gy, const int* ax, const int* ay, int n, int radius)
{
	if (radius <= 0) return true;
	for (int i = 0; i < n; ++i)
	{
		int dx = gx - ax[i]; if (dx < 0) dx = -dx;
		int dy = gy - ay[i]; if (dy < 0) dy = -dy;
		if (dx <= radius && dy <= radius) return true;
	}
	return false;
}

void PublishSnapshot(bool valid, uintptr_t zm)
{
	LONG cur = g_activeSnap;
	IslandSnapshot* w = &g_snap[(cur + 1) & 1];
	InterlockedIncrement(&w->seq);           // odd: writing
	_ReadWriteBarrier();
	w->valid = valid ? 1 : 0;
	w->zm = zm;
	w->gen = g_snapGen;
	w->compCount = valid ? g_curCompCount : 0;
	if (valid)
	{
		memcpy(w->comp, g_curComp, sizeof(w->comp));
		memcpy(w->memberStart, g_curMemberStart, sizeof(w->memberStart));
		memcpy(w->memberCount, g_curMemberCount, sizeof(w->memberCount));
		memcpy(w->members, g_curMembers, sizeof(w->members));
	}
	else
	{
		for (int i = 0; i < ZONE_GRID_COUNT; ++i) w->comp[i] = -1;
	}
	_ReadWriteBarrier();
	InterlockedIncrement(&w->seq);           // even: consistent
	InterlockedExchange(&g_activeSnap, (cur + 1) & 1);
}

void DumpComponents();
void LogInputsOnce(uintptr_t zm);

// Full rebuild (main thread). Publishes only when the result changes.
void Rebuild(uintptr_t zm, double now)
{
	// --- eligibility of marked zones ---
	int ax[256], ay[256];
	int anchors = CollectAnchors(zm, ax, ay, 256);
	memset(g_included, 0, sizeof(g_included));
	memset(g_isMod, 0, sizeof(g_isMod));
	int modZones = 0;
	for (int idx = 0; idx < ZONE_GRID_COUNT; ++idx)
	{
		if (!g_marks[idx]) continue;
		uintptr_t z = ZoneAt(zm, idx);
		if (!ZoneLoadingF(z) && !ZoneAccess(z))
		{
			// Fully unloaded: the mark is stale. A later game load re-enters
			// through Set B, a later mod promotion re-marks it.
			g_marks[idx] = 0;
			g_markCount--;
			continue;
		}
		if (!ZoneAccess(z)) continue;
		if (g_inSetB[idx]) continue;   // the game owns it now
		if (!NearAnyAnchor(IdxGX(idx), IdxGY(idx), ax, ay, anchors, cfg_islandModRadius)) continue;
		g_included[idx] = 1;
		g_isMod[idx] = 1;
		modZones++;
	}
	g_lastEligibility = now;

	// --- accessible Set B zones ---
	for (int i = 0; i < g_setBCount; ++i)
	{
		int idx = g_setBIdx[i];
		if (ZoneAccess(ZoneAt(zm, idx)))
			g_included[idx] = 1;
	}

	// --- union-find over neighbors[4] ---
	for (int idx = 0; idx < ZONE_GRID_COUNT; ++idx) g_ufParent[idx] = (short)idx;
	for (int idx = 0; idx < ZONE_GRID_COUNT; ++idx)
	{
		if (!g_included[idx]) continue;
		uintptr_t z = ZoneAt(zm, idx);
		for (int k = 0; k < ZONE_NEIGHBOR_COUNT; ++k)
		{
			uintptr_t nb = *(uintptr_t*)(z + OFF_ZONE_NEIGHBORS + 8 * k);
			int ni = ZoneIndexOf(zm, nb);
			if (ni < 0 || !g_included[ni]) continue;
			UfUnion(idx, ni);
		}
	}

	// --- components with at least one mod zone ---
	for (int idx = 0; idx < ZONE_GRID_COUNT; ++idx) { g_rootComp[idx] = -1; g_newComp[idx] = -1; }
	int compCount = 0;
	int overflow = 0;
	for (int idx = 0; idx < ZONE_GRID_COUNT; ++idx)
	{
		if (!g_isMod[idx]) continue;
		int r = UfFind(idx);
		if (g_rootComp[r] >= 0) continue;
		if (compCount >= MAX_COMPS) { overflow++; continue; }
		g_rootComp[r] = (short)compCount++;
	}
	for (int c = 0; c < compCount; ++c) g_newMemberCount[c] = 0;
	for (int idx = 0; idx < ZONE_GRID_COUNT; ++idx)
	{
		if (!g_included[idx]) continue;
		int c = g_rootComp[UfFind(idx)];
		if (c < 0) continue;
		g_newComp[idx] = (short)c;
		g_newMemberCount[c]++;
	}
	int pos = 0;
	for (int c = 0; c < compCount; ++c)
	{
		g_newMemberStart[c] = (unsigned short)pos;
		pos += g_newMemberCount[c];
		g_newMemberCount[c] = 0;   // reused as a fill cursor below
	}
	for (int idx = 0; idx < ZONE_GRID_COUNT; ++idx)
	{
		int c = g_newComp[idx];
		if (c < 0) continue;
		g_newMembers[g_newMemberStart[c] + g_newMemberCount[c]++] = (unsigned short)idx;
	}

	// --- publish on change ---
	bool changed = !g_curValid
	            || compCount != g_curCompCount
	            || memcmp(g_newComp, g_curComp, sizeof(g_newComp)) != 0
	            || memcmp(g_isMod, g_curIsMod, sizeof(g_isMod)) != 0;
	if (!changed && compCount > 0)
	{
		if (memcmp(g_newMemberStart, g_curMemberStart, compCount * sizeof(unsigned short)) != 0
		    || memcmp(g_newMemberCount, g_curMemberCount, compCount * sizeof(unsigned short)) != 0
		    || memcmp(g_newMembers, g_curMembers, pos * sizeof(unsigned short)) != 0)
			changed = true;
	}
	g_compOverflow = overflow;
	g_curModZones = modZones;
	if (!changed) return;

	memcpy(g_curComp, g_newComp, sizeof(g_curComp));
	memcpy(g_curIsMod, g_isMod, sizeof(g_curIsMod));
	memcpy(g_curMemberStart, g_newMemberStart, sizeof(g_curMemberStart));
	memcpy(g_curMemberCount, g_newMemberCount, sizeof(g_curMemberCount));
	memcpy(g_curMembers, g_newMembers, sizeof(g_curMembers));
	g_curCompCount = compCount;
	g_curValid = true;
	g_snapGen++;
	PublishSnapshot(true, zm);
	DumpComponents();
	if (!g_inputsLogged && pos > 0)
		LogInputsOnce(zm);
}

void ResetBuilder()
{
	memset(g_marks, 0, sizeof(g_marks));
	g_markCount = 0;
	g_haveSig = false;
	g_rebuildRequested = false;
	g_lastEligibility = -1.0;
	g_setBCount = 0;
	g_setBAccessible = 0;
	memset(g_inSetB, 0, sizeof(g_inSetB));
	for (int i = 0; i < ZONE_GRID_COUNT; ++i) g_curComp[i] = -1;
	memset(g_curIsMod, 0, sizeof(g_curIsMod));
	g_curCompCount = 0;
	g_curValid = false;
	g_curModZones = 0;
	g_compOverflow = 0;
	g_snapGen++;
	PublishSnapshot(false, 0);
}

// Component id of a zone in the CURRENT (main-thread) result, with liveness.
int CurComp(uintptr_t zm, uintptr_t z)
{
	if (!g_curValid || zm != g_builderZm) return -1;
	int idx = ZoneIndexOf(zm, z);
	if (idx < 0) return -1;
	if (!ZoneAccess(z)) return -1;
	return g_curComp[idx];
}

// The island list the router receives for zone t in the CURRENT configuration:
// vanilla (Set B zones with t's label) when the hooks pass through, the overlay
// answer when they are live. Mirrors hook_getIsland exactly.
int RouterList(uintptr_t zm, uintptr_t t, unsigned short* out, int maxOut)
{
	int n = 0;
	int tl = ZoneLabel(t);
	bool live = IslandHooksLive();
	int ct = live ? CurComp(zm, t) : -1;

	if (!(live && ct >= 0 && tl <= 0))
	{
		for (int i = 0; i < g_setBCount && n < maxOut; ++i)
		{
			int idx = g_setBIdx[i];
			if (ZoneLabel(ZoneAt(zm, idx)) == tl)
				out[n++] = (unsigned short)idx;
		}
	}
	if (live && ct >= 0)
	{
		int start = g_curMemberStart[ct];
		int cnt = g_curMemberCount[ct];
		for (int i = 0; i < cnt && n < maxOut; ++i)
		{
			int idx = g_curMembers[start + i];
			uintptr_t z = ZoneAt(zm, idx);
			if (!ZoneAccess(z)) continue;
			if (tl > 0 && ZoneLabel(z) == tl) continue;
			out[n++] = (unsigned short)idx;
		}
	}
	return n;
}

void DumpComponents()
{
#ifdef ZONEOPT_DEBUG
	{
		std::ostringstream ss;
		ss << "[ZoneOpt] Island snapshot gen=" << g_snapGen
		   << " comps=" << g_curCompCount << " mod=" << g_curModZones
		   << " setB=" << g_setBCount << "/" << g_setBAccessible << "acc";
		if (g_compOverflow) ss << " compOverflow=" << g_compOverflow;
		LogDebug(ss.str());
	}
	for (int c = 0; c < g_curCompCount; ++c)
	{
		std::ostringstream ss;
		ss << "[ZoneOpt] Island comp " << c << " (" << g_curMemberCount[c] << "):";
		int shown = 0;
		for (int i = 0; i < g_curMemberCount[c]; ++i)
		{
			int idx = g_curMembers[g_curMemberStart[c] + i];
			if (shown++ >= 40) { ss << " ..."; break; }
			ss << " (" << IdxGX(idx) << "," << IdxGY(idx) << ")";
			if (g_curIsMod[idx]) ss << "*";
			else ss << "l" << ZoneLabel(ZoneAt(g_builderZm, idx));
		}
		LogDebug(ss.str());
	}
#endif
}

// Step 1 check: the router's bounds minimum and navmesh+468 must match
// centre - zoneStep/2 and zoneStep for the zones we route through.
void LogInputsOnce(uintptr_t zm)
{
	g_inputsLogged = true;
	uintptr_t navmesh = *(uintptr_t*)(gameBase + RVA_GLOBAL_SECTION_MGR);
	float size = navmesh ? *(float*)(navmesh + OFF_NAVMESH_ZONE_SIZE) : 0.0f;
	std::ostringstream ss;
	ss << std::fixed << std::setprecision(1);
	ss << "[ZoneOpt] Island router inputs: navmesh+468=" << size
	   << " zoneStep=" << zoneStepX;
	int shown = 0;
	for (int c = 0; c < g_curCompCount && shown < 3; ++c)
	{
		for (int i = 0; i < g_curMemberCount[c] && shown < 3; ++i)
		{
			int idx = g_curMembers[g_curMemberStart[c] + i];
			uintptr_t z = ZoneAt(zm, idx);
			float minX = *(float*)(z + OFF_ZONE_AABB_CENTER)     - *(float*)(z + OFF_ZONE_AABB_HALF);
			float minZ = *(float*)(z + OFF_ZONE_AABB_CENTER + 8) - *(float*)(z + OFF_ZONE_AABB_HALF + 8);
			ss << " | (" << IdxGX(idx) << "," << IdxGY(idx) << ") min=(" << minX << "," << minZ
			   << ") expect=(" << (GetZoneCenterX((void*)z) - zoneStepX * 0.5f) << ","
			   << (GetZoneCenterZ((void*)z) - zoneStepZ * 0.5f) << ")";
			shown++;
		}
	}
	LogMsg(ss.str());
}

int CountUnexplained(uintptr_t zm)
{
	int n = 0;
	for (int idx = 0; idx < ZONE_GRID_COUNT; ++idx)
	{
		uintptr_t z = ZoneAt(zm, idx);
		if (!ZoneAccess(z)) continue;
		if (!*(void**)(z + OFF_ZONE_CONTENT)) continue;
		if (g_inSetB[idx] || g_marks[idx]) continue;
		n++;
	}
	return n;
}

} // namespace


// =========================================================================
// Hooks (any thread)
// =========================================================================

void IslandSetHooksInstalled(bool installed)
{
	InterlockedExchange(&g_hooksInstalled, installed ? 1 : 0);
}

bool IslandHooksLive()
{
#if ISLAND_STEP >= 2
	return islandFixEnabled && InterlockedCompareExchange(&g_hooksInstalled, 0, 0) != 0;
#else
	return false;
#endif
}

bool hook_isInIsland(void* zoneA, void* zoneB)
{
	InterlockedIncrement(&g_isInCalls);
	uintptr_t a = (uintptr_t)zoneA;
	uintptr_t b = (uintptr_t)zoneB;

	bool vanilla = orig_isInIsland
		? orig_isInIsland(zoneA, zoneB)
		: (b != 0 && a != 0 && ZoneLabel(a) == ZoneLabel(b));

	// 1. Vanilla NULL rule.
	if (!a || !b)
		return vanilla;

	// 2. Vanilla positive match: each vanilla island lies inside one component,
	//    so this can never contradict the overlay.
	int la = ZoneLabel(a);
	if (la > 0 && la == ZoneLabel(b))
		return vanilla;

	// 3. Component answer when either zone lives in one.
	int ca, cb;
	if (!SnapReadComps(a, b, &ca, &cb))
	{
		InterlockedIncrement(&g_isInSeqFail);
		return vanilla;
	}
	if (ca >= 0 && !ZoneAccess(a)) ca = -1;
	if (cb >= 0 && !ZoneAccess(b)) cb = -1;
	if (ca < 0 && cb < 0)
		return vanilla;    // 4. outside every component: vanilla (incl. 0 == 0)

	bool answer = (ca >= 0 && ca == cb);
	if (answer != vanilla)
		InterlockedIncrement(&g_isInFlips);
	if (!IslandHooksLive())
		return vanilla;
	return answer;
}

void* hook_getIsland(void* zoneMgr, void* zone, void* lektorOut)
{
	InterlockedIncrement(&g_getIslCalls);
	uintptr_t t = (uintptr_t)zone;
	uintptr_t out = (uintptr_t)lektorOut;
	if (!t || !out)
		return orig_getIsland(zoneMgr, zone, lektorOut);

	unsigned short members[HOOK_COPY_MAX];
	int ct = -1;
	uintptr_t snapZm = 0;
	int n = SnapCopyMembers(t, &ct, &snapZm, members, HOOK_COPY_MAX);
	if (n < 0)
	{
		InterlockedIncrement(&g_getIslFallback);
		return orig_getIsland(zoneMgr, zone, lektorOut);
	}
	if (ct < 0 || snapZm != (uintptr_t)zoneMgr || !ZoneAccess(t))
		return orig_getIsland(zoneMgr, zone, lektorOut);

	int tl = ZoneLabel(t);
	bool live = IslandHooksLive();
	void* result = NULL;

	// t.island <= 0: the original would return every Set B zone labelled 0
	// (including zones that just entered Set B), which can capture the
	// router's ray. Answer from the component only.
	if (tl > 0 || !live)
		result = orig_getIsland(zoneMgr, zone, lektorOut);

	uintptr_t zm = (uintptr_t)zoneMgr;
	long appended = 0;
	for (int i = 0; i < n; ++i)
	{
		uintptr_t z = ZoneAt(zm, members[i]);
		if (!ZoneAccess(z)) continue;
		if (tl > 0 && ZoneLabel(z) == tl) continue;   // the original already appended it
		if (live && !AppendLektor(out, z)) break;
		appended++;
	}
	if (appended)
		InterlockedExchangeAdd(&g_getIslAppended, appended);
	return result;
}


// =========================================================================
// Marks + rebuild requests (main thread)
// =========================================================================

void IslandMarkModZone(void* zoneMgr, int gx, int gy)
{
	if (!zoneMgr || gx < 0 || gx > ZONE_GRID_MAX || gy < 0 || gy > ZONE_GRID_MAX)
		return;
	int idx = gy + gx * 64;
	if (!g_marks[idx]) { g_marks[idx] = 1; g_markCount++; }
	g_rebuildRequested = true;
}

void IslandRequestRebuild()
{
	g_rebuildRequested = true;
}

void IslandReset()
{
	ResetBuilder();
	g_builderZm = 0;
}


// =========================================================================
// Router emulation + PLAYER STUCK support (main thread)
// =========================================================================

bool IslandEmulateCrossing(void* zoneMgr, void* charZone,
                           float destX, float destZ, float posX, float posZ,
                           float* outX, float* outZ)
{
	uintptr_t zm = (uintptr_t)zoneMgr;
	uintptr_t t = (uintptr_t)charZone;
	if (!zm || !t) return false;
	uintptr_t navmesh = *(uintptr_t*)(gameBase + RVA_GLOBAL_SECTION_MGR);
	if (!navmesh) return false;
	float size = *(float*)(navmesh + OFF_NAVMESH_ZONE_SIZE);

	static unsigned short list[ZONE_GRID_COUNT];   // main thread only
	int n = RouterList(zm, t, list, ZONE_GRID_COUNT);

	// Ray from the destination toward the position (pos - dest), t in (0,1).
	float dirX = posX - destX;
	float dirZ = posZ - destZ;
	float best = 1.0f;
	for (int i = 0; i < n; ++i)
	{
		uintptr_t z = ZoneAt(zm, list[i]);
		float minX = *(float*)(z + OFF_ZONE_AABB_CENTER)     - *(float*)(z + OFF_ZONE_AABB_HALF);
		float minZ = *(float*)(z + OFF_ZONE_AABB_CENTER + 8) - *(float*)(z + OFF_ZONE_AABB_HALF + 8);

		float tx;
		if (dirX <= 0.0f) tx = (dirX >= 0.0f) ? 1.0f : ((minX + size) - destX) / dirX;
		else              tx = (minX - destX) / dirX;
		float zc = dirZ * tx + destZ;
		if (minZ > zc || zc > minZ + size) tx = 1.0f;

		float tz;
		if (dirZ <= 0.0f) tz = (dirZ >= 0.0f) ? 1.0f : ((minZ + size) - destZ) / dirZ;
		else              tz = (minZ - destZ) / dirZ;
		float xc = tz * dirX + destX;
		if (minX > xc || xc > minX + size) tz = 1.0f;

		if (tx > 0.0f && best > tx) best = tx;
		if (tz > 0.0f && best > tz) best = tz;
	}
	if (best >= 1.0f) return false;
	*outX = destX + dirX * best;
	*outZ = destZ + dirZ * best;
	return true;
}

bool IslandDescribeStuck(void* zoneMgr, uintptr_t charMov,
                         float posX, float posZ, float destX, float destZ,
                         IslandStuckInfo* out)
{
	uintptr_t zm = (uintptr_t)zoneMgr;
	if (!zm || !charMov || !out || !gridCalibrated) return false;

	memset(out, 0, sizeof(*out));
	out->wpX = *(float*)(charMov + OFF_CMOV_PATH_DEST);
	out->wpZ = *(float*)(charMov + OFF_CMOV_PATH_DEST + 8);
	out->movingToEdge = *(unsigned char*)(charMov + OFF_CMOV_MOVING_TO_EDGE);
	out->edgeCounter = *(int*)(charMov + OFF_CMOV_EDGE_COUNTER);
	out->selfComp = -1;
	out->nextComp = -1;
	out->nextGX = out->nextGY = -1;

	int gx, gy;
	if (!WorldToZoneGrid(posX, posZ, &gx, &gy)) return true;
	uintptr_t charZone = (uintptr_t)GetZoneEntry(zoneMgr, gx, gy);
	if (!charZone) return true;
	out->selfComp = CurComp(zm, charZone);

	float rx, rz;
	int nx = -1, ny = -1;
	if (IslandEmulateCrossing(zoneMgr, (void*)charZone, destX, destZ, posX, posZ, &rx, &rz))
	{
		out->haveCrossing = true;
		float dx = posX - rx, dz = posZ - rz;
		out->xd = sqrtf(dx * dx + dz * dz);
		// The zone just beyond the crossing, toward the destination.
		float tx = destX - rx, tz = destZ - rz;
		float len = sqrtf(tx * tx + tz * tz);
		if (len > 0.001f)
			WorldToZoneGrid(rx + tx / len * 10.0f, rz + tz / len * 10.0f, &nx, &ny);
	}
	else
	{
		// Grid walk from the waypoint toward the destination (dominant axis).
		int wx, wy;
		if (WorldToZoneGrid(out->wpX, out->wpZ, &wx, &wy))
		{
			float dx = destX - out->wpX, dz = destZ - out->wpZ;
			float adx = dx < 0 ? -dx : dx, adz = dz < 0 ? -dz : dz;
			nx = wx; ny = wy;
			if (adx >= adz) nx += (dx >= 0) ? 1 : -1;
			else            ny += (dz >= 0) ? 1 : -1;
		}
	}
	if (nx >= 0 && ny >= 0)
	{
		uintptr_t nz = (uintptr_t)GetZoneEntry(zoneMgr, nx, ny);
		if (nz)
		{
			out->nextGX = nx; out->nextGY = ny;
			out->nextComp = CurComp(zm, nz);
			out->nextLabel = ZoneLabel(nz);
			out->nextLoading = ZoneLoadingF(nz) ? 1 : 0;
			out->nextAccess = ZoneAccess(nz) ? 1 : 0;
		}
	}
	return true;
}

#endif // ISLAND_STEP >= 1


// =========================================================================
// Step 3: parked-squad re-issue tracker (main thread)
// =========================================================================

#if ISLAND_STEP >= 3

namespace {

const int    MAX_ISLAND_ORDERS   = 64;
const double ORDER_POLL_INTERVAL = 0.25;
const float  PARK_WP_DIST        = 20.0f;   // pathDestination within this of pos
const float  PARK_MIN_DEST_DIST  = 100.0f;  // farther than this from the order destination
const float  MISSED_ADVANCE_DIST = 350.0f;  // 300-unit snap + arrival tolerance
const float  GROWTH_THRESHOLD_SQ = 40.0f;   // the router's own threshold (squared)
const float  UNPARK_DIST         = 50.0f;
const int    MAX_REISSUES        = 8;
const double REISSUE_COOLDOWN    = 2.0;
const double RETRY_DELAY         = 1.5;

struct IslandOrder {
	bool      active;
	uintptr_t character;
	float     destX, destY, destZ;
	double    orderTime;
	bool      parked;
	float     parkX, parkZ;
	int       parkGX, parkGY;
	double    parkTime;
	bool      haveX0;
	float     x0X, x0Z;
	int       reissueCount;
	int       limitGX, limitGY;    // zone where the re-issue budget started
	double    lastReissueTime;
	bool      retryArmed;
	float     retryX, retryZ;      // crossing at the last re-issue
	unsigned int seenGen;
	unsigned int seenSig;
	bool      noCrossingLogged;
};

IslandOrder   g_orders[MAX_ISLAND_ORDERS];
int           g_orderCount = 0;
double        g_lastOrderPoll = 0.0;
volatile long g_reissues = 0;

void ResetOrders()
{
	for (int i = 0; i < MAX_ISLAND_ORDERS; ++i) g_orders[i].active = false;
	g_orderCount = 0;
}

bool ReissueCharacter(uintptr_t character, float dx, float dy, float dz)
{
	uintptr_t charVtable = *(uintptr_t*)character;
	if (!charVtable) return false;
	typedef void (*moveOrderFn_t)(uintptr_t, void*, void*, const float*);
	moveOrderFn_t fn_moveOrder = (moveOrderFn_t)(*(uintptr_t*)(charVtable + 0x318));
	if (!fn_moveOrder) return false;

	float dest[3] = { dx, dy, dz };
	uintptr_t cm = *(uintptr_t*)(character + OFF_CHAR_MOVEMENT);
	if (cm)
	{
		// Edge mode drops a re-issue whose destination is within 2 units of
		// the last requested one (+0xDC). Nudge it by 3 units.
		float lx = *(float*)(cm + OFF_CMOV_LAST_DEST);
		float lz = *(float*)(cm + OFF_CMOV_LAST_DEST + 8);
		float ddx = lx - dx, ddz = lz - dz;
		if (ddx * ddx + ddz * ddz < 4.0f)
			dest[0] += 3.0f;
	}
	fn_moveOrder(character, NULL, NULL, dest);
	return true;
}

bool ReissueOrder(IslandOrder& o, double now, const char* why, bool haveCross, float cx, float cz)
{
	if (o.reissueCount >= MAX_REISSUES) return false;
	if (o.lastReissueTime > 0.0 && now - o.lastReissueTime < REISSUE_COOLDOWN) return false;

	bool sent;
	int slot = FormationSlotForCharacter(o.character);
	if (slot >= 0)
		sent = FormationReissueTravel(slot, now);
	else
		sent = ReissueCharacter(o.character, o.destX, o.destY, o.destZ);
	if (!sent) return false;

	o.reissueCount++;
	o.lastReissueTime = now;
	o.retryArmed = haveCross;
	o.retryX = cx; o.retryZ = cz;
	InterlockedIncrement(&g_reissues);

	std::ostringstream ss;
	ss << std::fixed << std::setprecision(0);
	ss << "[ZoneOpt] Island reissue (" << why << "): "
	   << (slot >= 0 ? "group " : "char ") << (slot >= 0 ? slot : 0)
	   << " park=(" << o.parkX << "," << o.parkZ << ")";
	if (haveCross) ss << " cross=(" << cx << "," << cz << ")";
	ss << " dest=(" << o.destX << "," << o.destZ << ")"
	   << " n=" << o.reissueCount << "/" << MAX_REISSUES;
	LogMsg(ss.str());
	return true;
}

inline float Dist2(float ax, float az, float bx, float bz)
{
	float dx = ax - bx, dz = az - bz;
	return dx * dx + dz * dz;
}

void PollOrders(uintptr_t zm, double now)
{
	if (now - g_lastOrderPoll < ORDER_POLL_INTERVAL) return;
	g_lastOrderPoll = now;
	if (g_orderCount == 0 || !gridCalibrated) return;

	uintptr_t playerIntf = *(uintptr_t*)(gameBase + RVA_GLOBAL_PLAYER);
	if (!playerIntf) return;
	unsigned int scCount = GetPlayerCharCount(playerIntf);
	uintptr_t* scStuff = GetPlayerCharStuff(playerIntf);
	if (!scStuff || scCount == 0 || scCount > 200) return;

	for (int i = 0; i < g_orderCount; ++i)
	{
		IslandOrder& o = g_orders[i];
		if (!o.active) continue;

		bool alive = false;
		for (unsigned int j = 0; j < scCount; ++j)
			if (scStuff[j] == o.character) { alive = true; break; }
		if (!alive) { o.active = false; continue; }

		uintptr_t cm = *(uintptr_t*)(o.character + OFF_CHAR_MOVEMENT);
		if (!cm) continue;

		float posX = *(float*)(cm + OFF_CMOV_POS);
		float posZ = *(float*)(cm + OFF_CMOV_POS + 8);
		float wpX  = *(float*)(cm + OFF_CMOV_PATH_DEST);
		float wpZ  = *(float*)(cm + OFF_CMOV_PATH_DEST + 8);
		bool edge  = *(unsigned char*)(cm + OFF_CMOV_MOVING_TO_EDGE) != 0;

		if (Dist2(posX, posZ, o.destX, o.destZ) < PARK_MIN_DEST_DIST * PARK_MIN_DEST_DIST)
		{
			o.active = false;   // arrived
			continue;
		}

		// A formation group is evaluated once, through its first alive member.
		int slot = FormationSlotForCharacter(o.character);
		if (slot >= 0)
		{
			uintptr_t rep = FormationFirstAliveMember(slot);
			if (rep && rep != o.character) continue;
		}

		int gx, gy;
		if (!WorldToZoneGrid(posX, posZ, &gx, &gy)) continue;

		// Re-issue budget resets once the squad has moved more than one zone.
		if (o.reissueCount > 0)
		{
			int ddx = gx - o.limitGX; if (ddx < 0) ddx = -ddx;
			int ddy = gy - o.limitGY; if (ddy < 0) ddy = -ddy;
			if (ddx > 1 || ddy > 1) { o.reissueCount = 0; o.limitGX = gx; o.limitGY = gy; }
		}

		bool parkedNow = edge && Dist2(wpX, wpZ, posX, posZ) < PARK_WP_DIST * PARK_WP_DIST;
		uintptr_t charZone = (uintptr_t)GetZoneEntry((void*)zm, gx, gy);
		if (!charZone) continue;

		if (!o.parked)
		{
			if (!parkedNow) continue;

			// --- park: record the fixed ray dest -> parkPos and emulate X0 ---
			o.parked = true;
			o.parkX = posX; o.parkZ = posZ;
			o.parkGX = gx; o.parkGY = gy;
			o.parkTime = now;
			o.haveX0 = false;
			o.retryArmed = false;
			o.noCrossingLogged = false;
			o.seenGen = g_snapGen;
			o.seenSig = g_setBSig;
			if (o.reissueCount == 0) { o.limitGX = gx; o.limitGY = gy; }

			float rx, rz;
			if (IslandEmulateCrossing((void*)zm, (void*)charZone, o.destX, o.destZ, o.parkX, o.parkZ, &rx, &rz))
			{
				o.haveX0 = true;
				o.x0X = rx; o.x0Z = rz;
				// Missed advance: the island already reaches past the waypoint.
				if (Dist2(rx, rz, o.parkX, o.parkZ) > MISSED_ADVANCE_DIST * MISSED_ADVANCE_DIST)
					ReissueOrder(o, now, "park", true, rx, rz);
			}
			else
			{
				o.noCrossingLogged = true;
				std::ostringstream ss;
				ss << std::fixed << std::setprecision(0);
				ss << "[ZoneOpt] Island reissue skipped: no crossing"
				   << " park=(" << o.parkX << "," << o.parkZ << ") zone=(" << gx << "," << gy << ")"
				   << " dest=(" << o.destX << "," << o.destZ << ")";
				LogMsg(ss.str());
			}
			continue;
		}

		// --- parked ---
		if (!parkedNow)
		{
			bool stillHere = Dist2(posX, posZ, o.parkX, o.parkZ) < UNPARK_DIST * UNPARK_DIST;
			if (!stillHere)
			{
				o.parked = false;      // moving again
				o.retryArmed = false;
				continue;
			}
			if (!o.retryArmed)
			{
				// Edge flag dropped (a new route was accepted) or the router
				// advanced the waypoint: wait for the character to move.
				if (!edge) o.parked = false;
				continue;
			}
			// A re-issue that the readiness gate swallowed clears movingToEdge
			// before returning, so the character sits here with edge=0 and an
			// unchanged position. Treat it as still parked so the retry fires.
		}

		bool configChanged = (o.seenGen != g_snapGen) || (o.seenSig != g_setBSig);
		bool retryDue = o.retryArmed && (now - o.lastReissueTime >= RETRY_DELAY);
		if (!configChanged && !retryDue) continue;
		o.seenGen = g_snapGen;
		o.seenSig = g_setBSig;

		// Keep the ray fixed (dest -> parkPos): only the island can move the crossing.
		float rx, rz;
		bool have = IslandEmulateCrossing((void*)zm, (void*)charZone, o.destX, o.destZ, o.parkX, o.parkZ, &rx, &rz);

		if (configChanged)
		{
			if (o.haveX0)
			{
				if (have && Dist2(rx, rz, o.x0X, o.x0Z) > GROWTH_THRESHOLD_SQ)
				{
					if (ReissueOrder(o, now, "growth", true, rx, rz))
					{
						o.x0X = rx; o.x0Z = rz;
						continue;
					}
				}
			}
			else if (have)
			{
				// A crossing appeared on the fixed ray: island growth.
				if (Dist2(rx, rz, o.parkX, o.parkZ) > GROWTH_THRESHOLD_SQ)
					ReissueOrder(o, now, "growth", true, rx, rz);
				o.haveX0 = true;
				o.x0X = rx; o.x0Z = rz;
				continue;
			}
		}

		// Retry: still parked with the same crossing after a re-issue
		// (for example, the readiness gate swallowed the order).
		if (retryDue && have && Dist2(rx, rz, o.retryX, o.retryZ) <= GROWTH_THRESHOLD_SQ)
		{
			if (!ReissueOrder(o, now, "retry", true, rx, rz))
			{
				if (o.reissueCount >= MAX_REISSUES) o.retryArmed = false;
			}
		}
		else if (retryDue && !have)
			o.retryArmed = false;
	}
}

} // namespace

void IslandNoteOrder(uintptr_t character, const float* location)
{
	if (!character || !location) return;
	int slot = -1;
	for (int i = 0; i < g_orderCount; ++i)
	{
		if (g_orders[i].character == character) { slot = i; break; }
		if (!g_orders[i].active && slot < 0) slot = i;
	}
	if (slot < 0)
	{
		if (g_orderCount >= MAX_ISLAND_ORDERS) return;
		slot = g_orderCount++;
	}
	IslandOrder& o = g_orders[slot];
	memset(&o, 0, sizeof(o));
	o.active = true;
	o.character = character;
	o.destX = location[0];
	o.destY = location[1];
	o.destZ = location[2];
	o.orderTime = ElapsedSec();
}

#endif // ISLAND_STEP >= 3


// =========================================================================
// IslandTick (main thread, every frame)
// =========================================================================

#if ISLAND_STEP >= 1
namespace { unsigned int g_lastSig = 0; }
#endif

void IslandTick(void* zoneMgr, double now)
{
	static double lastDiag = 0.0;
	uintptr_t zm = (uintptr_t)zoneMgr;

#if ISLAND_STEP >= 1
	if (zm)
	{
		bool loading = *(unsigned char*)(zm + OFF_ZM_LOADING) != 0;
		if (zm != g_builderZm || (loading && !g_wasLoading))
		{
			// Save load (ZM+8) or a new ZoneManager: drop marks, snapshot and tracker.
			ResetBuilder();
#if ISLAND_STEP >= 3
			ResetOrders();
#endif
			LogMsg(loading ? "[ZoneOpt] Islands: save load detected, overlay reset"
			               : "[ZoneOpt] Islands: zone manager bound, overlay reset");
			g_builderZm = zm;
		}
		g_wasLoading = loading;

		if (!loading)
		{
			WalkSetB(zm);
			bool sigChanged = !g_haveSig || g_setBSig != g_lastSig;
			bool eligibilityDue = (g_lastEligibility < 0.0)
			                   || (now - g_lastEligibility >= ELIGIBILITY_INTERVAL);
			if (sigChanged || g_rebuildRequested || eligibilityDue)
			{
				Rebuild(zm, now);
				g_lastSig = g_setBSig;
				g_haveSig = true;
				g_rebuildRequested = false;
			}
#if ISLAND_STEP >= 3
			PollOrders(zm, now);
#endif
		}
	}
#endif // ISLAND_STEP >= 1

	if (now - lastDiag >= DIAG_INTERVAL_SEC)
	{
		lastDiag = now;
#ifdef ZONEOPT_DEBUG
		std::ostringstream ss;
		ss << "[ZoneOpt] Islands:";
#if ISLAND_STEP >= 1
		ss << " comps=" << g_curCompCount
		   << " mod=" << g_curModZones
		   << " setBacc=" << g_setBAccessible
		   << " unexpl=" << ((zm && zm == g_builderZm) ? CountUnexplained(zm) : 0)
		   << " snapGen=" << g_snapGen
		   << " live=" << (IslandHooksLive() ? 1 : 0)
		   << " isIn(flip)=" << InterlockedCompareExchange(&g_isInCalls, 0, 0)
		   << "(" << InterlockedCompareExchange(&g_isInFlips, 0, 0) << ")"
		   << " getIsl(app/fallback)=" << InterlockedCompareExchange(&g_getIslCalls, 0, 0)
		   << "(" << InterlockedCompareExchange(&g_getIslAppended, 0, 0)
		   << "/" << InterlockedCompareExchange(&g_getIslFallback, 0, 0) << ")";
		long seqFail = InterlockedCompareExchange(&g_isInSeqFail, 0, 0);
		if (seqFail) ss << " seqFail=" << seqFail;
		if (g_markCount) ss << " marks=" << g_markCount;
#endif
#if ISLAND_STEP >= 3
		ss << " reissue=" << InterlockedCompareExchange(&g_reissues, 0, 0);
#endif
		AppendReadinessTids(ss);
		LogDebug(ss.str());
#endif // ZONEOPT_DEBUG
	}
}
