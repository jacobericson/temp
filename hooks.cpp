#include "preload.h"
#include "transition.h"
#include "nm_workers.h"
#include "hooks.h"
#include "tracking.h"
#include "formation.h"
#include "pathfinding.h"
#include "navmesh_sched.h"

// Transition state (defined here, declared in transition.h)
bool           isTransitionActive   = false;
int            deferredFrameCount   = 0;
LARGE_INTEGER  transitionStartTime;

static bool prioritizedThisTransition = false;


// =========================================================================
// NavMesh scheduling helpers (builds context from preload/tracking state)
// =========================================================================

static void CallPrioritizeNavMeshQueue()
{
	// Resolve camera: use transition target if active, else lastCameraGX/GY
	int prioCamX = lastCameraGX, prioCamY = lastCameraGY;
	if (isTransitionActive && g_cachedZoneMgr)
	{
		void* tz = *(void**)((uintptr_t)g_cachedZoneMgr + OFF_ZM_CURRENT_ZONE);
		if (tz) { prioCamX = GetZoneGridX(tz); prioCamY = GetZoneGridY(tz); }
	}

	SchedMoverInfo movers[MAX_WATCHED];
	for (int i = 0; i < numWatched; ++i)
	{
		movers[i].currentX = watchedChars[i].currentZoneX;
		movers[i].currentY = watchedChars[i].currentZoneY;
		movers[i].destX    = watchedChars[i].destZoneX;
		movers[i].destY    = watchedChars[i].destZoneY;
	}
	SchedZoneInfo zones[MAX_PRELOADED];
	for (int i = 0; i < numPreloaded; ++i)
	{
		zones[i].gridX = preloadedZones[i].gridX;
		zones[i].gridY = preloadedZones[i].gridY;
	}
	PrioritizeNavMeshQueue(prioCamX, prioCamY, movers, numWatched, zones, numPreloaded);
}

#if NMCACHE_STEP >= 4
static void BoostWorkerThreads()
{
	for (int i = 0; i < NAVMESH_WORKER_COUNT; ++i)
	{
		if (g_workerHandles[i] && g_workerHandles[i] != INVALID_HANDLE_VALUE)
		{
			g_workerSavedPriority[i] = GetThreadPriority(g_workerHandles[i]);
			SetThreadPriority(g_workerHandles[i], THREAD_PRIORITY_HIGHEST);
		}
	}
}

static void RestoreWorkerThreads()
{
	for (int i = 0; i < NAVMESH_WORKER_COUNT; ++i)
	{
		if (g_workerHandles[i] && g_workerHandles[i] != INVALID_HANDLE_VALUE)
			SetThreadPriority(g_workerHandles[i], g_workerSavedPriority[i]);
		g_workerSavedPriority[i] = THREAD_PRIORITY_NORMAL;
	}
}
#endif // NMCACHE_STEP >= 4


// =========================================================================
// Hook 1: showLoadingMessage -- transition start/end bracket
// =========================================================================

void hook_showLoadingMessage(void* thisPtr, bool on)
{
	if (on && !isTransitionActive)
	{
		isTransitionActive = true;
		deferredFrameCount = 0;
		QueryPerformanceCounter(&transitionStartTime);
		BoostNavMeshThread();
#if NMCACHE_STEP >= 4
		BoostWorkerThreads();
#endif

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
		RestoreNavMeshThread();
#if NMCACHE_STEP >= 4
		RestoreWorkerThreads();
#endif
		prioritizedThisTransition = false;

		LARGE_INTEGER endTime;
		QueryPerformanceCounter(&endTime);
		double totalMs = QPCToMs(transitionStartTime, endTime);
		isTransitionActive = false;

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
			   << ", charZones=" << charZonesQueued;
			if (movementAwareEnabled)
				ss << ", orders=" << hookOrderCount
				   << ", watched=" << numWatched;
		}
		LogMsg(ss.str());

		// Reset Phase 2 preload state; preserve watchedChars for in-transit squads
		ClearPreloadZones();
		promotedCount = 0;
		charZonesQueued = 0;
		hookOrderCount = 0;
	}

	orig_showLoadingMessage(thisPtr, on);
}


// =========================================================================
// Hook 2: isContentPending -- THE navmesh deferral (Phase 1)
// =========================================================================
//
// Original checks 4 conditions under a reader lock:
//   1. *(manager + 632) == 0       section count drained
//   2. *(navMeshGen + 265) == 0    flag265: generator idle
//   3. *(navMeshGen + 184) == NULL  processing queue empty
//   4. *(navMeshGen + 136) == NULL  input queue empty
//
// Returns 1 = zone ready (all conditions clear), 0 = still loading.
//
// Our hook: if original returns 0 and only navmesh conditions (2-4) block,
// return 1 anyway. The navmesh generator continues in the background.

bool hook_isContentPending(void* manager, void* zonePos)
{
	// Prioritize navmesh queue once per transition, at start of state 4.
	// By state 4, processState3 has finished registerZoneSections ->
	// contentStream has submitted all navmesh jobs for this transition.
	// Reordering now promotes the transition zone's jobs to the front.
	if (isTransitionActive && !prioritizedThisTransition)
	{
		CallPrioritizeNavMeshQueue();
		prioritizedThisTransition = true;
	}

	bool result = orig_isContentPending(manager, zonePos);
	if (result)
		return true;

	if (!deferralEnabled)
		return false;

	int sectionCount = *(int*)((uintptr_t)manager + 632);
	if (sectionCount > 0)
		return false;

	// Sections drained: return ready regardless of navmesh state.
	// Macro-transitions: original Phase 1 deferral (95% faster state 4).
	// Micro-transitions (platoon activation bumps state 0->1 without
	// showLoadingMessage): prevents NavMeshGenerator idle stall from
	// causing state 4 pause. NPCs use fallback pathfinding briefly.
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
										                        destGX, destGY, curGX, curGY))
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

	orig_addOrderSelected(thisPI, destIndoors, task, subject, shift, addDontClear, location);
}


// =========================================================================
// Hook 3: updateCameraZone -- preloading + promotion trigger (Phase 2)
// =========================================================================

void hook_updateCameraZone(void* zoneMgr, void* cameraPos)
{
	// Always call original first (visual activate/deactivate)
	orig_updateCameraZone(zoneMgr, cameraPos);

	g_cachedZoneMgr = zoneMgr;

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
	if (now - lastReprioritizeTime > 3.0 && (numWatched > 0 || pendingCount > 0))
	{
		CallPrioritizeNavMeshQueue();
		lastReprioritizeTime = now;
	}

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
					EnqueueAheadZones(targetX, targetY, currentX, currentY, OWNER_CAMERA);
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
