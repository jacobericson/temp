#include "preload.h"
#include "transition.h"
#include "nm_workers.h"
#include "tracking.h"
#include "formation.h"
#include "islands.h"
#include "hooks.h"
#include "navmesh_sched.h"


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
int lastCameraGX         = -1;
int lastCameraGY         = -1;



void ClearPreloadZones()
{
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

void ClearPreloadState()
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
	ClearNavMeshCache();
	ClearFormationGroups();
	IslandReset();
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

	if (IsZoneAccessible(zoneEntry))
	{
		pz.promoted = true;
		numPreloaded++;
		return;
	}

	if (IsZoneLoading(zoneEntry))
	{
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

	// Save/restore ready flag: loadSingleZone clears *(gateObj+800),
	// which makes NavMeshGenerator sleep and causes ~750ms stalls.
	uintptr_t gateObj = *(uintptr_t*)(gameBase + RVA_GLOBAL_GATE_OBJ);
	unsigned char savedReady = 0;
	if (gateObj)
		savedReady = *(unsigned char*)(gateObj + 800);

	int result = fn_loadSingleZone(zoneEntry, 0, 0);
	if (gateObj && savedReady)
		*(unsigned char*)(gateObj + 800) = savedReady;

	pz.pending = (result != 0);
	if (result != 0)
		pendingCount++;
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

	// isReadyForSections: 4 work queues empty + ready flag set
	uintptr_t gateObj = *(uintptr_t*)(gameBase + RVA_GLOBAL_GATE_OBJ);
	if (!gateObj)
		return;
	if (*(int*)(gateObj + 432) > 0) return;
	if (*(int*)(gateObj + 680) > 0) return;
	if (*(int*)(gateObj + 760) > 0) return;
	if (*(int*)(gateObj + 512) > 0) return;
	if (*(unsigned char*)(gateObj + 800) == 0) return;

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

		int thingsCount = *(int*)((uintptr_t)content + OFF_ZMC_THINGS_COUNT);

		// processContent (vtable[4]): populates things/objects from zone data.
		// Called once per zone, after content streaming has had time to finish.
		// things<=1 means no real content yet (0=unprocessed, 1=processed-empty).
		if (thingsCount <= 1 && !preloadedZones[i].contentProcessed)
		{
			double age = now - preloadedZones[i].loadTimeSec;
			if (age < 2.0)
				continue;  // give content streaming time to finish

			CallProcessContent(content);
			preloadedZones[i].contentProcessed = true;
			thingsCount = *(int*)((uintptr_t)content + OFF_ZMC_THINGS_COUNT);
		}

		// Empty zones: things<=1 after processContent means no real content.
		// things=0: no data file processed. things=1: data file processed, nothing to load.
		if (thingsCount <= 1)
		{
			double age = now - preloadedZones[i].loadTimeSec;
			if (age < 3.0)
				continue;
		}

		fn_registerZoneSections((void*)sectionMgr, ze);
		*(int*)((uintptr_t)ze + 204) = 0;  // post-condition: all 4 game callers clear this

		preloadedZones[i].registered = true;
		preloadedZones[i].registeredEmpty = (thingsCount <= 1);
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

			// Fallback: sectionCount==0 means contentStream finished all sections
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
