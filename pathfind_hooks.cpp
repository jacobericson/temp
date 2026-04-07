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

#if PATHFIND_STEP >= 1

#if PATHFIND_STEP >= 2
// BG thread tag: set by hook_csFindPath, read by hook_findPathFull (same thread, sequential)
static bool currentRequestIsPlayer = false;
#endif


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
	// Bypass the cluster graph connectivity pre-check entirely
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

	// =================================================================
	// Squad path cache: multi-slot leader detection, A* boost, caching, injection
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
				InterlockedIncrement(&spcDiagBoosted);

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
							slot.cachedSCSize = *(int*)((uintptr_t)streamingCollection + OFF_SC_INSTANCES_SIZE);
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
				int curSCSize = *(int*)((uintptr_t)streamingCollection + OFF_SC_INSTANCES_SIZE);
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
							slot.cachedSCSize = *(int*)((uintptr_t)streamingCollection + OFF_SC_INSTANCES_SIZE);
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
#endif // PATHFIND_STEP >= 4

#if PATHFIND_STEP >= 4
default_astar:
#endif

	// Default path: call original A*
	orig_findPathFull(streamingCollection, searchState, findPathOutput);

#if PATHFIND_STEP >= 4
diag_count:
#endif
	{
		unsigned char status = *(unsigned char*)((uintptr_t)findPathOutput + 60);
		unsigned char cause  = *(unsigned char*)((uintptr_t)findPathOutput + 61);
		int iterCount        = *(int*)((uintptr_t)findPathOutput + 48);

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

	char result = orig_csFindPathFallback(manager, startFaceKey, startPos,
	                                       destFaceKey, destPos, radius,
	                                       param6, param7, resultBuf);
	spcFallbackTag = 0;
	return result;
}

#endif // PATHFIND_STEP >= 4


#endif // PATHFIND_STEP >= 1
