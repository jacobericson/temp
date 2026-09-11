// Phase 3: Movement-aware character tracking
// Watched character management, baseline scanning, active polling,
// and tiered character poll orchestration.

#include "preload.h"
#include "tracking.h"

#if PATHFIND_STEP >= 7
// Defined in pathfind_hooks.cpp (also gated on PATHFIND_STEP >= 7).
// Declared locally to avoid pulling pathfinding.h into this TU.
extern volatile long watchedEvictions;
#endif


// =========================================================================
// Character tracking state (defined here, declared in tracking.h)
// =========================================================================

WatchedCharacter watchedChars[MAX_WATCHED];
int numWatched           = 0;
double lastBaselineScan  = 0.0;
double lastActivePoll    = 0.0;
int hookOrderCount       = 0;


// =========================================================================
// Watched character helpers
// =========================================================================

bool AddWatchedCharacter(uintptr_t character, uintptr_t charMovement,
                         int destZX, int destZY, int curZX, int curZY
#if PATHFIND_STEP >= 5
                         , bool hasMoveOrder
#endif
                         )
{
	for (int i = 0; i < numWatched; ++i)
	{
		if (watchedChars[i].character == character)
		{
			watchedChars[i].destZoneX = destZX;
			watchedChars[i].destZoneY = destZY;
			watchedChars[i].currentZoneX = curZX;
			watchedChars[i].currentZoneY = curZY;
#if PATHFIND_STEP >= 5
			// Upgrade-only: false -> true allowed, true -> false ignored.
			// Return true on upgrade so the caller counts the state change
			// (drives "Order captured" log + immediate reprio trigger).
			// Also run the directional next-zone preload seed so upgraded
			// chars get the same cheap heuristic as fresh adds.
			if (hasMoveOrder && !watchedChars[i].hasMoveOrder)
			{
				watchedChars[i].hasMoveOrder = true;
				int dirX = 0, dirY = 0;
				if (destZX > curZX) dirX = 1; else if (destZX < curZX) dirX = -1;
				if (destZY > curZY) dirY = 1; else if (destZY < curZY) dirY = -1;
				int enqueued = 0;
				if (dirX != 0 && EnqueueCharacterZone(curZX + dirX, curZY)) enqueued++;
				if (dirY != 0 && EnqueueCharacterZone(curZX, curZY + dirY)) enqueued++;
				if (dirX != 0 && dirY != 0 && EnqueueCharacterZone(curZX + dirX, curZY + dirY)) enqueued++;
				if (enqueued > 0)
				{
					std::ostringstream ss;
					ss << "[ZoneOpt] Next-zone preload (upgrade): char at (" << curZX << "," << curZY
					   << ") dir=(" << dirX << "," << dirY << ") enqueued " << enqueued << " zones";
					LogMsg(ss.str());
				}
				return true;
			}
#endif
			return false;
		}
	}

	if (numWatched >= MAX_WATCHED)
	{
#if PATHFIND_STEP >= 7
		// Evict oldest hasMoveOrder=false entry. Never evict char-with-order.
		int evictIdx = -1;
		double oldestTime = 1e20;
		for (int i = 0; i < numWatched; ++i)
		{
			if (!watchedChars[i].hasMoveOrder && watchedChars[i].addedTime < oldestTime)
			{
				evictIdx = i;
				oldestTime = watchedChars[i].addedTime;
			}
		}
		if (evictIdx >= 0)
		{
			// Overwrite slot in place; numWatched unchanged.
			// Must init ALL fields explicitly — replacing a stale char.
			watchedChars[evictIdx].character    = character;
			watchedChars[evictIdx].charMovement = charMovement;
			watchedChars[evictIdx].hasMoveOrder = hasMoveOrder;
			watchedChars[evictIdx].destZoneX    = destZX;
			watchedChars[evictIdx].destZoneY    = destZY;
			watchedChars[evictIdx].currentZoneX = curZX;
			watchedChars[evictIdx].currentZoneY = curZY;
			watchedChars[evictIdx].addedTime    = ElapsedSec();
			InterlockedExchange64(
				(volatile LONG64*)&watchedChars[evictIdx].exitZonePacked,
				EXIT_ZONE_NONE);
			watchedChars[evictIdx].exitFaceUpdateTime = 0.0;
			// Critical: clear formationGroupId so new char doesn't inherit
			// evicted char's stale group association.
			watchedChars[evictIdx].formationGroupId       = -1;
			watchedChars[evictIdx].preloadAheadGX         = PRELOAD_AHEAD_NONE;
			watchedChars[evictIdx].preloadAheadGY         = PRELOAD_AHEAD_NONE;
			watchedChars[evictIdx].preloadAheadUpdateTime = 0.0;

			InterlockedIncrement(&watchedEvictions);

			// Fire directional-preload block (same as new-entry path).
			if (hasMoveOrder)
			{
				int dirX = 0, dirY = 0;
				if (destZX > curZX) dirX = 1; else if (destZX < curZX) dirX = -1;
				if (destZY > curZY) dirY = 1; else if (destZY < curZY) dirY = -1;
				if (dirX != 0) EnqueueCharacterZone(curZX + dirX, curZY);
				if (dirY != 0) EnqueueCharacterZone(curZX, curZY + dirY);
				if (dirX != 0 && dirY != 0) EnqueueCharacterZone(curZX + dirX, curZY + dirY);
			}
			return true;
		}
		// Fall through: no evictable entry (all have move orders).
#endif
#if PATHFIND_STEP >= 5
		if (hasMoveOrder)
		{
			static double lastFullWarn = 0.0;
			double t = ElapsedSec();
			if (t - lastFullWarn > 5.0)
			{
#if PATHFIND_STEP >= 7
				LogMsg("[ZoneOpt] WARN watchedChars full, no evictable entry — dropping move-order add");
#else
				LogMsg("[ZoneOpt] WARN watchedChars full, dropping move-order add (STEP 7 adds eviction)");
#endif
				lastFullWarn = t;
			}
		}
#endif
		return false;
	}

	watchedChars[numWatched].character = character;
	watchedChars[numWatched].charMovement = charMovement;
#if PATHFIND_STEP >= 5
	watchedChars[numWatched].hasMoveOrder = hasMoveOrder;
#endif
	watchedChars[numWatched].destZoneX = destZX;
	watchedChars[numWatched].destZoneY = destZY;
	watchedChars[numWatched].currentZoneX = curZX;
	watchedChars[numWatched].currentZoneY = curZY;
	watchedChars[numWatched].addedTime = ElapsedSec();
#if PATHFIND_STEP >= 6
	watchedChars[numWatched].exitZonePacked     = EXIT_ZONE_NONE;
	watchedChars[numWatched].exitFaceUpdateTime = 0.0;
#endif
#if PATHFIND_STEP >= 7
	watchedChars[numWatched].formationGroupId       = -1;
	watchedChars[numWatched].preloadAheadGX         = PRELOAD_AHEAD_NONE;
	watchedChars[numWatched].preloadAheadGY         = PRELOAD_AHEAD_NONE;
	watchedChars[numWatched].preloadAheadUpdateTime = 0.0;
#endif
	numWatched++;

	// Immediately preload next zone(s) in path direction.
	// Ensures the zone ahead is always in the preload queue
	// regardless of edge detection threshold. Only fires for move-order adds;
	// baseline adds (curZ==destZ) are no-ops here anyway.
#if PATHFIND_STEP >= 5
	if (hasMoveOrder)
#endif
	{
		int dirX = 0, dirY = 0;
		if (destZX > curZX) dirX = 1; else if (destZX < curZX) dirX = -1;
		if (destZY > curZY) dirY = 1; else if (destZY < curZY) dirY = -1;
		int enqueued = 0;
		if (dirX != 0 && EnqueueCharacterZone(curZX + dirX, curZY)) enqueued++;
		if (dirY != 0 && EnqueueCharacterZone(curZX, curZY + dirY)) enqueued++;
		if (dirX != 0 && dirY != 0 && EnqueueCharacterZone(curZX + dirX, curZY + dirY)) enqueued++;
		if (enqueued > 0)
		{
			std::ostringstream ss;
			ss << "[ZoneOpt] Next-zone preload: char at (" << curZX << "," << curZY
			   << ") dir=(" << dirX << "," << dirY << ") enqueued " << enqueued << " zones";
			LogMsg(ss.str());
		}
	}

	return true;
}

bool IsCharacterWatched(uintptr_t character)
{
	for (int i = 0; i < numWatched; ++i)
	{
		if (watchedChars[i].character == character)
			return true;
	}
	return false;
}

#if PATHFIND_STEP >= 5
void EnsurePlayerCharsWatched()
{
	uintptr_t playerIntf = *(uintptr_t*)(gameBase + RVA_GLOBAL_PLAYER);
	if (!playerIntf) return;

	unsigned int scCount = GetPlayerCharCount(playerIntf);
	uintptr_t* scStuff   = GetPlayerCharStuff(playerIntf);
	if (!scStuff || scCount == 0 || scCount > 200) return;

	int added = 0;
	for (unsigned int i = 0; i < scCount; ++i)
	{
		uintptr_t character = scStuff[i];
		if (!character) continue;
		if (IsCharacterWatched(character)) continue;  // upgrade-only contract

		float charX = GetCharPosX(character);
		float charZ = GetCharPosZ(character);
		int gx, gy;
		if (!WorldToZoneGrid(charX, charZ, &gx, &gy)) continue;

		uintptr_t charMov = *(uintptr_t*)(character + OFF_CHAR_MOVEMENT);
		if (!charMov) continue;

		// destZ == curZ for baseline (no move order)
		if (AddWatchedCharacter(character, charMov, gx, gy, gx, gy, /*hasMoveOrder=*/false))
			added++;
	}

	if (added > 0)
	{
		std::ostringstream ss;
		ss << "[ZoneOpt] Baseline watch: +" << added << " player chars (total " << numWatched << ")";
		LogDebug(ss.str());
	}
}
#endif

void RemoveWatchedCharacter(int index)
{
	if (index < 0 || index >= numWatched)
		return;
	numWatched--;
	if (index < numWatched)
		watchedChars[index] = watchedChars[numWatched];
	watchedChars[numWatched].character = 0;
	watchedChars[numWatched].charMovement = 0;
#if PATHFIND_STEP >= 5
	watchedChars[numWatched].hasMoveOrder = false;
#endif
#if PATHFIND_STEP >= 6
	watchedChars[numWatched].exitZonePacked     = EXIT_ZONE_NONE;
	watchedChars[numWatched].exitFaceUpdateTime = 0.0;
#endif
#if PATHFIND_STEP >= 7
	watchedChars[numWatched].formationGroupId       = -1;
	watchedChars[numWatched].preloadAheadGX         = PRELOAD_AHEAD_NONE;
	watchedChars[numWatched].preloadAheadGY         = PRELOAD_AHEAD_NONE;
	watchedChars[numWatched].preloadAheadUpdateTime = 0.0;
#endif
}


// =========================================================================
// Character scanning (baseline: current-position preloading only)
// =========================================================================

void ScanCharacterZones(void* zoneMgr)
{
#if PATHFIND_STEP >= 5
	EnsurePlayerCharsWatched();
#endif

#if PATHFIND_STEP >= 8
	// STEP 8 density-aware scan: iterate watchedChars[] (populated by
	// EnsurePlayerCharsWatched), count chars per zone. Always enqueue each
	// watched zone; if a zone has >=3 chars, also enqueue its 2x2 neighbors.
	//
	// Note on density semantic: this trigger fires on "3+ chars AT this
	// zone" (strict). The classifier T3 row uses the looser "3+ chars in
	// ANY 2x2 anchor containing this zone". Mismatches at the margin are
	// benign: missing preload zones are caught by per-char enqueue plus
	// ExitFace/camera coverage on traversal.
	struct ZoneCount { int gx, gy, count; };
	ZoneCount zones[MAX_CHAR_ZONES];
	int numZones = 0;
	int zonesOverflow = 0;  // chars dropped when numZones hits MAX_CHAR_ZONES

	for (int w = 0; w < numWatched; ++w)
	{
		int gx = watchedChars[w].currentZoneX;
		int gy = watchedChars[w].currentZoneY;
		if (gx < 0 || gy < 0) continue;

		bool found = false;
		for (int z = 0; z < numZones; ++z)
		{
			if (zones[z].gx == gx && zones[z].gy == gy)
			{
				zones[z].count++;
				found = true;
				break;
			}
		}
		if (!found)
		{
			if (numZones < MAX_CHAR_ZONES)
			{
				zones[numZones].gx = gx;
				zones[numZones].gy = gy;
				zones[numZones].count = 1;
				numZones++;
			}
			else
			{
				zonesOverflow++;
			}
		}
	}

	int zonesEnqueued = 0;
	int denseZones = 0;

	for (int z = 0; z < numZones; ++z)
	{
		// Always enqueue the current zone
		if (EnqueueCharacterZone(zones[z].gx, zones[z].gy))
			zonesEnqueued++;

		// Dense zone: also enqueue 2x2 neighbors. SMALL_DX/DY[0]=(0,0)
		// duplicates center; EnqueueCharacterZone dedups via IsZoneQueued.
		if (zones[z].count >= 3)
		{
			denseZones++;
			for (int sd = 0; sd < 4; ++sd)
			{
				int nx = zones[z].gx + SMALL_DX[sd];
				int ny = zones[z].gy + SMALL_DY[sd];
				if (EnqueueCharacterZone(nx, ny))
					zonesEnqueued++;
			}
		}
	}

	charZonesQueued += zonesEnqueued;

	// Surface overflow at most once per 10s to detect saturation cases
	// (e.g. 64 watched chars scattered across 25+ unique zones).
	if (zonesOverflow > 0)
	{
		static double lastOverflowWarn = 0.0;
		double tNow = ElapsedSec();
		if (tNow - lastOverflowWarn > 10.0)
		{
			std::ostringstream warn;
			warn << "[ZoneOpt] WARN ScanCharacterZones overflow: "
			     << zonesOverflow << " chars beyond MAX_CHAR_ZONES="
			     << MAX_CHAR_ZONES;
			LogMsg(warn.str());
			lastOverflowWarn = tNow;
		}
	}

	std::ostringstream ss;
	ss << "[ZoneOpt] Character scan (STEP8): " << numWatched << " watched, "
	   << numZones << " zones, " << denseZones << " dense (>=3), "
	   << zonesEnqueued << " new queued"
	   << (zonesOverflow > 0 ? " OVERFLOW" : "");
	LogDebug(ss.str());
	return;
#else
	// Legacy STEP 5-7 path: iterates player chars, skips watched chars,
	// 3x3 or 2x2 per center. Dead code in practice since
	// EnsurePlayerCharsWatched (STEP 5) pre-populates all player chars,
	// but kept for binary-equivalent fallback under /DPATHFIND_STEP<8.
	uintptr_t playerIntf = *(uintptr_t*)(gameBase + RVA_GLOBAL_PLAYER);
	if (!playerIntf)
		return;

	struct ZoneCenter { int gx, gy; };
	ZoneCenter centers[MAX_CHAR_ZONES];
	int numCenters = 0;
	int charCount = 0;

	unsigned int scCount = GetPlayerCharCount(playerIntf);
	uintptr_t* scStuff = GetPlayerCharStuff(playerIntf);

	if (scStuff && scCount > 0 && scCount <= 200)
	{
		for (unsigned int i = 0; i < scCount; ++i)
		{
			uintptr_t character = scStuff[i];
			if (!character)
				continue;

			// Skip characters already in active group tracking
			if (numWatched > 0 && IsCharacterWatched(character))
				continue;

			float charX = GetCharPosX(character);
			float charZ = GetCharPosZ(character);

			int gx, gy;
			if (WorldToZoneGrid(charX, charZ, &gx, &gy))
			{
				bool found = false;
				for (int c = 0; c < numCenters; ++c)
				{
					if (centers[c].gx == gx && centers[c].gy == gy)
					{ found = true; break; }
				}
				if (!found && numCenters < MAX_CHAR_ZONES)
				{
					centers[numCenters].gx = gx;
					centers[numCenters].gy = gy;
					numCenters++;
				}
			}
			charCount++;
		}
	}

	int zonesEnqueued = 0;

	if (numCenters > 0)
	{
		int available = MAX_PRELOADED - CAMERA_RESERVED;
		int perCenter = available / numCenters;
		bool useFullGrid = (perCenter >= 9);

		for (int c = 0; c < numCenters; ++c)
		{
			int cx = centers[c].gx;
			int cy = centers[c].gy;

			if (useFullGrid)
			{
				for (int i = 0; i < 9; ++i)
				{
					if (EnqueueCharacterZone(cx + ORDER_DX[i], cy + ORDER_DY[i]))
						zonesEnqueued++;
				}
			}
			else
			{
				for (int i = 0; i < 4; ++i)
				{
					if (EnqueueCharacterZone(cx + SMALL_DX[i], cy + SMALL_DY[i]))
						zonesEnqueued++;
				}
			}
		}
	}

	charZonesQueued += zonesEnqueued;

	std::ostringstream ss;
	ss << "[ZoneOpt] Character scan: " << charCount << " chars"
	   << ", " << numCenters << " centers"
	   << (numCenters > 0 && (MAX_PRELOADED - CAMERA_RESERVED) / numCenters >= 9 ? " x3x3" : " x2x2")
	   << ", " << zonesEnqueued << " new zones queued";
	LogDebug(ss.str());
#endif
}


// =========================================================================
// Tiered character polling (Phase 3)
// =========================================================================

void PollActiveMovers(void* zoneMgr, double now)
{
	int removals = 0;
	int edgePreloads = 0;

	// MH2: read playerCharacters lektor once for validation
	uintptr_t playerIntf = *(uintptr_t*)(gameBase + RVA_GLOBAL_PLAYER);
	unsigned int scCount = 0;
	uintptr_t* scStuff = NULL;
	if (playerIntf)
	{
		scCount = GetPlayerCharCount(playerIntf);
		scStuff = GetPlayerCharStuff(playerIntf);
	}

	for (int i = numWatched - 1; i >= 0; --i)
	{
		uintptr_t character = watchedChars[i].character;
		uintptr_t charMov   = watchedChars[i].charMovement;

		if (!character || !charMov)
		{
			RemoveWatchedCharacter(i);
			removals++;
			continue;
		}

		// MH2: validate character is still alive before reading offsets
		bool stillAlive = false;
		if (scStuff && scCount > 0 && scCount <= 200)
		{
			for (unsigned int j = 0; j < scCount; ++j)
			{
				if (scStuff[j] == character) { stillAlive = true; break; }
			}
		}
		if (!stillAlive)
		{
			RemoveWatchedCharacter(i);
			removals++;
			continue;
		}

		uintptr_t pathObj = *(uintptr_t*)(charMov + OFF_CMOV_HAVOK_CHAR);
		if (!pathObj)
		{
			RemoveWatchedCharacter(i);
			removals++;
			continue;
		}

		float curX = GetCharPosX(character);
		float curZ = GetCharPosZ(character);

		int curGX, curGY;
		if (!WorldToZoneGrid(curX, curZ, &curGX, &curGY))
		{
			RemoveWatchedCharacter(i);
			removals++;
			continue;
		}

#if PATHFIND_STEP >= 5
		// Baseline chars: align dest with cur so the arrival check below always
		// fires, keeping them on the early-continue path. Without this, a
		// baseline char whose AI moved it cross-zone since the last 5s scan
		// would fall through to edge-detection and burst-preload — opposite of
		// spec STEP 8's goal. Move-order chars keep their stored dest so the
		// arrival check fires meaningfully on real arrival.
		if (!watchedChars[i].hasMoveOrder)
		{
			watchedChars[i].destZoneX = curGX;
			watchedChars[i].destZoneY = curGY;
		}
#endif

		if (curGX == watchedChars[i].destZoneX && curGY == watchedChars[i].destZoneY)
		{
#if PATHFIND_STEP >= 5
			// Arrived: downgrade to baseline (T3-eligible) instead of removing.
			// Keeps the char in watchedChars so T3 keeps firing for its current
			// zone and avoids the 5s gap until the next baseline scan re-adds it.
			if (watchedChars[i].hasMoveOrder)
			{
				watchedChars[i].hasMoveOrder = false;
				watchedChars[i].destZoneX = curGX;
				watchedChars[i].destZoneY = curGY;
			}
			continue;  // do not remove; baseline scan owns lifecycle now
#else
			RemoveWatchedCharacter(i);
			removals++;
			continue;
#endif
		}

		watchedChars[i].currentZoneX = curGX;
		watchedChars[i].currentZoneY = curGY;

		// Always ensure next zone in path is preloaded (no edge threshold).
		// Characters must always have at least their current zone and next
		// zone ahead available for pathfinding.
		{
			int dirX = 0, dirY = 0;
			if (watchedChars[i].destZoneX > curGX) dirX = 1;
			else if (watchedChars[i].destZoneX < curGX) dirX = -1;
			if (watchedChars[i].destZoneY > curGY) dirY = 1;
			else if (watchedChars[i].destZoneY < curGY) dirY = -1;
			if (dirX != 0 && EnqueueCharacterZone(curGX + dirX, curGY)) edgePreloads++;
			if (dirY != 0 && EnqueueCharacterZone(curGX, curGY + dirY)) edgePreloads++;
			if (dirX != 0 && dirY != 0 && EnqueueCharacterZone(curGX + dirX, curGY + dirY)) edgePreloads++;
		}

#if PATHFIND_STEP < 7
		// Legacy edge-detection branch: preload 3x3 around near-edge position
		// plus 3 movement-ahead zones. STEP 7 drops this — ExitFace preload
		// (main-thread 1Hz drain of watchedChars[i].preloadAheadGX/Y) supersedes
		// it and self-propagates as char crosses zone boundaries.
		void* zoneEntry = GetZoneEntry(zoneMgr, curGX, curGY);
		if (!zoneEntry)
			continue;

		float zoneCenterX = GetZoneCenterX(zoneEntry);
		float zoneCenterZ = GetZoneCenterZ(zoneEntry);

		float dx = curX - zoneCenterX;
		float dz = curZ - zoneCenterZ;
		float adx = (dx < 0.0f) ? -dx : dx;
		float adz = (dz < 0.0f) ? -dz : dz;

		if (adx > EDGE_THRESHOLD || adz > EDGE_THRESHOLD)
		{
			int adjX = curGX;
			int adjY = curGY;
			if (dx > EDGE_THRESHOLD) adjX = curGX + 1;
			else if (-dx > EDGE_THRESHOLD) adjX = curGX - 1;
			if (dz > EDGE_THRESHOLD) adjY = curGY + 1;
			else if (-dz > EDGE_THRESHOLD) adjY = curGY - 1;

			for (int j = 0; j < 9; ++j)
			{
				if (EnqueueCharacterZone(adjX + ORDER_DX[j], adjY + ORDER_DY[j]))
					edgePreloads++;
			}

			// 3 movement-ahead zones beyond the 3x3 grid
			EnqueueAheadZones(adjX, adjY, curGX, curGY, OWNER_CHARACTER);
		}
#endif
	}

	if (removals > 0 || edgePreloads > 0)
	{
		std::ostringstream ss;
		ss << "[ZoneOpt] Active poll: " << numWatched << " watched"
		   << ", " << removals << " removed"
		   << ", " << edgePreloads << " edge zones";
		LogDebug(ss.str());
	}

#if PATHFIND_STEP >= 5
	{
		static double lastWatchSummary = 0.0;
		if (now - lastWatchSummary > 5.0)
		{
			int withOrder = 0;
			for (int i = 0; i < numWatched; ++i)
				if (watchedChars[i].hasMoveOrder) withOrder++;
			std::ostringstream ss;
			ss << "[ZoneOpt] Watched: " << numWatched << " total, "
			   << withOrder << " with move order";
			LogDebug(ss.str());
			lastWatchSummary = now;
		}
	}
#endif
}

void TieredCharacterPoll(void* zoneMgr, double now)
{
	// Active group poll: every ~1s (only when we have watched characters)
	if (numWatched > 0 && now - lastActivePoll > ACTIVE_POLL_INTERVAL)
	{
		PollActiveMovers(zoneMgr, now);
		lastActivePoll = now;
	}

	// Baseline scan: every ~5s (current-position preloading, skips watched chars)
	if (now - lastBaselineScan > BASELINE_SCAN_INTERVAL)
	{
		ScanCharacterZones(zoneMgr);
		lastBaselineScan = now;
	}
}


#if PATHFIND_STEP >= 7
// =========================================================================
// Formation group ID helpers (called from formation.cpp, main thread only)
// =========================================================================

void SetFormationGroupIdOnMembers(int groupSlot, uintptr_t* chars, int charCount)
{
	if (groupSlot < 0) return;
	for (int c = 0; c < charCount; ++c)
	{
		uintptr_t ch = chars[c];
		if (!ch) continue;
		for (int w = 0; w < numWatched; ++w)
		{
			if (watchedChars[w].character == ch)
			{
				watchedChars[w].formationGroupId = groupSlot;  // volatile atomic store
				break;
			}
		}
	}
}

void ClearFormationGroupIdForSlot(int groupSlot)
{
	if (groupSlot < 0) return;
	for (int w = 0; w < numWatched; ++w)
	{
		if (watchedChars[w].formationGroupId == groupSlot)
			watchedChars[w].formationGroupId = -1;
	}
}
#endif
