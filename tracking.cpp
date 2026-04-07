// Phase 3: Movement-aware character tracking
// Watched character management, baseline scanning, active polling,
// and tiered character poll orchestration.

#include "preload.h"
#include "tracking.h"


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
                         int destZX, int destZY, int curZX, int curZY)
{
	for (int i = 0; i < numWatched; ++i)
	{
		if (watchedChars[i].character == character)
		{
			watchedChars[i].destZoneX = destZX;
			watchedChars[i].destZoneY = destZY;
			watchedChars[i].currentZoneX = curZX;
			watchedChars[i].currentZoneY = curZY;
			return false;
		}
	}

	if (numWatched >= MAX_WATCHED)
		return false;

	watchedChars[numWatched].character = character;
	watchedChars[numWatched].charMovement = charMovement;
	watchedChars[numWatched].destZoneX = destZX;
	watchedChars[numWatched].destZoneY = destZY;
	watchedChars[numWatched].currentZoneX = curZX;
	watchedChars[numWatched].currentZoneY = curZY;
	watchedChars[numWatched].addedTime = ElapsedSec();
	numWatched++;

	// Immediately preload next zone(s) in path direction.
	// Ensures the zone ahead is always in the preload queue
	// regardless of edge detection threshold.
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

void RemoveWatchedCharacter(int index)
{
	if (index < 0 || index >= numWatched)
		return;
	numWatched--;
	if (index < numWatched)
		watchedChars[index] = watchedChars[numWatched];
	watchedChars[numWatched].character = 0;
	watchedChars[numWatched].charMovement = 0;
}


// =========================================================================
// Character scanning (baseline: current-position preloading only)
// =========================================================================

void ScanCharacterZones(void* zoneMgr)
{
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

		if (curGX == watchedChars[i].destZoneX && curGY == watchedChars[i].destZoneY)
		{
			RemoveWatchedCharacter(i);
			removals++;
			continue;
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
	}

	if (removals > 0 || edgePreloads > 0)
	{
		std::ostringstream ss;
		ss << "[ZoneOpt] Active poll: " << numWatched << " watched"
		   << ", " << removals << " removed"
		   << ", " << edgePreloads << " edge zones";
		LogDebug(ss.str());
	}
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
