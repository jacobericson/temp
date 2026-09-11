// NavMesh thread priority boost and 5-tier job queue prioritization.
// Extracted from preload.cpp with refactored interface — no preload/tracking globals.
// All classification data passed via parameters.

#include "navmesh_sched.h"


// =========================================================================
// NMG thread priority state (file-static, NMG bg thread only)
// =========================================================================

static HANDLE navMeshThreadHandle  = NULL;
static int    savedThreadPriority  = THREAD_PRIORITY_NORMAL;


#if PATHFIND_STEP >= 5
// =========================================================================
// Reprio flag + counters (declared in navmesh_sched.h)
// =========================================================================
volatile long g_reprioRequested = 0;
volatile long reprioFlagFires   = 0;
volatile long reprioTimerFires  = 0;
volatile long reprioOrderFires  = 0;
#endif


// =========================================================================
// Thread priority boost (NMG background thread only)
// =========================================================================

void BoostNavMeshThread()
{
	if (!priorityBoostEnabled)
		return;

	uintptr_t mgr = *(uintptr_t*)(gameBase + RVA_GLOBAL_SECTION_MGR);
	if (!mgr)
		return;

	uintptr_t navMeshGen = *(uintptr_t*)(mgr + OFF_MGR_NAVMESH_GEN);
	if (!navMeshGen)
		return;

	HANDLE h = *(HANDLE*)(navMeshGen + OFF_NAVMESH_THREAD);
	if (h && h != INVALID_HANDLE_VALUE)
	{
		savedThreadPriority = GetThreadPriority(h);
		if (SetThreadPriority(h, THREAD_PRIORITY_HIGHEST))
		{
			navMeshThreadHandle = h;
		}
	}
}

void RestoreNavMeshThread()
{
	if (navMeshThreadHandle)
	{
		SetThreadPriority(navMeshThreadHandle, savedThreadPriority);
		navMeshThreadHandle = NULL;
	}
	savedThreadPriority = THREAD_PRIORITY_NORMAL;
}


// =========================================================================
// 5-tier priority computation (all inputs explicit)
// =========================================================================
//   Tier 1: Camera zones (3x3 around camera)
//   Tier 2: Character path zones (current zone + immediate next in path)
//   Tier 3: Ahead of moving characters (same direction, beyond next zone)
//   Tier 4: Other preloaded zones (stationary characters, behind)
//   Tier 5: Everything else (game's own jobs, unknown zones)

int ComputeZonePriority(int gridX, int gridY,
                        int camGridX, int camGridY,
                        const SchedMoverInfo* movers, int moverCount,
                        const SchedZoneInfo* preloaded, int preloadedCount)
{
#if PATHFIND_STEP >= 5
	// T1: mover with hasMoveOrder=true at this exact zone OR at decoded ExitFace zone.
	for (int w = 0; w < moverCount; ++w)
	{
		if (!movers[w].hasMoveOrder) continue;
		if (movers[w].currentX == gridX && movers[w].currentY == gridY)
			return 1;
#if PATHFIND_STEP >= 6
		// ExitFace match: A*-derived next-zone-out, written by hook_findPathFull.
		// Sentinel guard: INT_MIN means no decode yet (skip match).
		if (movers[w].exitZoneGX > -2000000000 &&
		    movers[w].exitZoneGX == gridX && movers[w].exitZoneGY == gridY)
			return 1;
#endif
	}

	// T2: camera 2x2 pause grid. SMALL_DX/SMALL_DY = {0,1}x{0,1} (positive offsets).
	if (camGridX >= 0 && camGridY >= 0)
	{
		int dx = gridX - camGridX;
		int dy = gridY - camGridY;
		if (dx >= 0 && dx <= 1 && dy >= 0 && dy <= 1)
			return 2;
	}

	// T3: any mover at this zone (chars with hasMoveOrder=true returned T1 above,
	//     so only baseline movers reach here on the direct-match path)
	//     OR >=3 movers within a 2x2 cluster containing this zone.
	bool isT3 = false;
	for (int w = 0; w < moverCount; ++w)
	{
		if (movers[w].currentX == gridX && movers[w].currentY == gridY)
		{ isT3 = true; break; }
	}
	if (!isT3)
	{
		// Density expansion: iterate the 4 unique 2x2 anchors that contain
		// (gridX, gridY) -- i.e. anchorX in {gridX-1, gridX}, anchorY in {gridY-1, gridY}.
		// SMALL_DX/SMALL_DY = {0,1}x{0,1}, so (gridX - SMALL_DX[sd], gridY - SMALL_DY[sd])
		// produces exactly those 4 anchors.
		for (int sd = 0; sd < 4 && !isT3; ++sd)
		{
			int anchorX = gridX - SMALL_DX[sd];
			int anchorY = gridY - SMALL_DY[sd];
			int n = 0;
			for (int w = 0; w < moverCount; ++w)
			{
				int mx = movers[w].currentX, my = movers[w].currentY;
				if (mx >= anchorX && mx <= anchorX + 1 &&
				    my >= anchorY && my <= anchorY + 1)
					n++;
			}
			if (n >= 3) isT3 = true;
		}
	}
	if (isT3) return 3;

#if PATHFIND_STEP >= 8
	// STEP 8 swap: preloaded zones are our "stale preload regret" and rank
	// BELOW game-initiated jobs. Game jobs are for zones the state machine
	// decided it needs — they deserve to outrank our pre-warm speculation.
	for (int i = 0; i < preloadedCount; ++i)
	{
		if (preloaded[i].gridX == gridX && preloaded[i].gridY == gridY)
			return 5;   // stale preload -> lowest priority
	}
	return 4;           // game/default -> above stale preloads
#else
	// T4: in preloadedZones[] (didn't qualify for T1-T3)
	for (int i = 0; i < preloadedCount; ++i)
	{
		if (preloaded[i].gridX == gridX && preloaded[i].gridY == gridY)
			return 4;
	}

	// T5: default
	return 5;
#endif
#else
	// --- Tier 1: Camera zones (3x3 around camera) ---
	if (camGridX >= 0 && camGridY >= 0)
	{
		int dx = gridX - camGridX;
		int dy = gridY - camGridY;
		if (dx < 0) dx = -dx;
		if (dy < 0) dy = -dy;
		if (dx <= 1 && dy <= 1)
			return 1;
	}

	// --- Tier 2/3: Character path analysis ---
	bool isTier2 = false;
	bool isTier3 = false;
	for (int w = 0; w < moverCount; ++w)
	{
		int cx = movers[w].currentX;
		int cy = movers[w].currentY;
		int destX = movers[w].destX;
		int destY = movers[w].destY;
		if (cx < 0 || cy < 0) continue;

		// Current zone = Tier 2
		if (gridX == cx && gridY == cy)
		{ isTier2 = true; break; }

		int dirX = 0, dirY = 0;
		if (destX > cx) dirX = 1; else if (destX < cx) dirX = -1;
		if (destY > cy) dirY = 1; else if (destY < cy) dirY = -1;

		// Stationary character: skip path analysis
		if (dirX == 0 && dirY == 0) continue;

		// Next zone in path direction = Tier 2
		if ((dirX != 0 && gridX == cx + dirX && gridY == cy) ||
		    (dirY != 0 && gridX == cx && gridY == cy + dirY) ||
		    (dirX != 0 && dirY != 0 && gridX == cx + dirX && gridY == cy + dirY))
		{ isTier2 = true; break; }

		// Ahead of character (in movement direction) = Tier 3
		if (!isTier3)
		{
			bool aheadX = (dirX == 0) ? (gridX == cx) : ((gridX - cx) * dirX >= 0);
			bool aheadY = (dirY == 0) ? (gridY == cy) : ((gridY - cy) * dirY >= 0);
			if (aheadX && aheadY)
				isTier3 = true;
		}
	}

	if (isTier2) return 2;
	if (isTier3) return 3;

	// --- Tier 4: Any zone in the preload list ---
	for (int i = 0; i < preloadedCount; ++i)
	{
		if (preloaded[i].gridX == gridX && preloaded[i].gridY == gridY)
			return 4;
	}

	return 5;
#endif
}


// =========================================================================
// Queue prioritization: 5-tier navmesh job reordering
// =========================================================================

void PrioritizeNavMeshQueue(int camGridX, int camGridY,
                            const SchedMoverInfo* movers, int moverCount,
                            const SchedZoneInfo* preloaded, int preloadedCount)
{
	if (!fn_pathBuilderInit || !fn_pathBuilderFinalize || !fn_readerUnlock)
		return;

	uintptr_t sectionMgr = *(uintptr_t*)(gameBase + RVA_GLOBAL_SECTION_MGR);
	if (!sectionMgr)
		return;
	uintptr_t navMeshGen = *(uintptr_t*)(sectionMgr + OFF_MGR_NAVMESH_GEN);
	if (!navMeshGen)
		return;

	// Quick check before acquiring lock
	uintptr_t head = *(uintptr_t*)(navMeshGen + 136);
	if (!head)
		return;

	// Acquire input queue lock at navMeshGen+152
	char initBuf[16];
	void* initResult = fn_pathBuilderInit(initBuf);
	fn_pathBuilderFinalize((void*)(navMeshGen + 152), initResult);

	head = *(uintptr_t*)(navMeshGen + 136);
	if (!head)
	{
		fn_readerUnlock((void*)(navMeshGen + 152));
		return;
	}

	// Walk queue, assign tiers via ComputeZonePriority
	const int MAX_PER_TIER = 64;
	uintptr_t tierJobs[5][MAX_PER_TIER];
	int tierCounts[5] = {0, 0, 0, 0, 0};

	uintptr_t job = head;
	while (job)
	{
		uintptr_t nextJob = *(uintptr_t*)(job + 96);

		int tier = 5;
		uintptr_t zone = *(uintptr_t*)job;
		if (zone)
		{
			int gx = *(int*)(zone + OFF_ZONE_COORDS_X);
			int gy = *(int*)(zone + OFF_ZONE_COORDS_Y);
			tier = ComputeZonePriority(gx, gy, camGridX, camGridY,
			                           movers, moverCount, preloaded, preloadedCount);
		}

		int t = tier - 1;  // 0-indexed
		if (t < 0) t = 0;
		if (t > 4) t = 4;
		if (tierCounts[t] < MAX_PER_TIER)
			tierJobs[t][tierCounts[t]++] = job;
		else if (tierCounts[4] < MAX_PER_TIER)
			tierJobs[4][tierCounts[4]++] = job;  // overflow -> lowest tier

		job = nextJob;
	}

	// Rebuild linked list: tier 1 first -> tier 5 last
	uintptr_t newHead = 0;
	uintptr_t* linkPtr = &newHead;

	for (int t = 0; t < 5; ++t)
	{
		for (int i = 0; i < tierCounts[t]; ++i)
		{
			*linkPtr = tierJobs[t][i];
			linkPtr = (uintptr_t*)(tierJobs[t][i] + 96);
		}
	}
	*linkPtr = 0;  // terminate list

	// Write head
	*(uintptr_t*)(navMeshGen + 136) = newHead;

	// Write tail: points to &lastJob->next, or &head if empty
	int totalJobs = tierCounts[0] + tierCounts[1] + tierCounts[2] + tierCounts[3] + tierCounts[4];
	if (totalJobs > 0)
		*(uintptr_t*)(navMeshGen + 144) = (uintptr_t)linkPtr;
	else
		*(uintptr_t*)(navMeshGen + 144) = navMeshGen + 136;

	// Release lock
	fn_readerUnlock((void*)(navMeshGen + 152));

#if PATHFIND_STEP >= 5
	if (totalJobs > 0)
#else
	if (tierCounts[0] + tierCounts[1] + tierCounts[2] > 0)
#endif
	{
		std::ostringstream ss;
#if PATHFIND_STEP >= 8
		ss << "[ZoneOpt] NavMesh queue prioritized (T4=game/T5=stale):"
#else
		ss << "[ZoneOpt] NavMesh queue prioritized:"
#endif
		   << " T1=" << tierCounts[0] << " T2=" << tierCounts[1]
		   << " T3=" << tierCounts[2] << " T4=" << tierCounts[3]
		   << " T5=" << tierCounts[4];
		LogMsg(ss.str());
	}
}
