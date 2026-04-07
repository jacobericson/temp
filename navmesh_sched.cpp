// NavMesh thread priority boost and 5-tier job queue prioritization.
// Extracted from preload.cpp with refactored interface — no preload/tracking globals.
// All classification data passed via parameters.

#include "navmesh_sched.h"


// =========================================================================
// NMG thread priority state (file-static, NMG bg thread only)
// =========================================================================

static HANDLE navMeshThreadHandle  = NULL;
static int    savedThreadPriority  = THREAD_PRIORITY_NORMAL;


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

static int ComputeZonePriority(int gridX, int gridY,
                               int camGridX, int camGridY,
                               const SchedMoverInfo* movers, int moverCount,
                               const SchedZoneInfo* preloaded, int preloadedCount)
{
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

	if (tierCounts[0] + tierCounts[1] + tierCounts[2] > 0)
	{
		std::ostringstream ss;
		ss << "[ZoneOpt] NavMesh queue prioritized:"
		   << " T1=" << tierCounts[0] << " T2=" << tierCounts[1]
		   << " T3=" << tierCounts[2] << " T4=" << tierCounts[3]
		   << " T5=" << tierCounts[4];
		LogMsg(ss.str());
	}
}
