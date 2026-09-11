// formation.cpp — Group cohesion: scatter patch, formation groups, polling
// State is available when PATHFIND_STEP >= 2 (even in ZONEONLY).
// Functions are FULL builds only (excluded by ZONEOPT_ZONEONLY via config.h inline stubs).

#include "formation.h"
#include "pathfinding.h"
#if PATHFIND_STEP >= 7
#include "tracking.h"        // watchedChars[] + gid helpers
#include "pathfind_cache.h"  // spcSlots[] formation dedup cache
#endif

// State needed by pathfinding priority boost (PATHFIND_STEP >= 2)
#if !defined(ZONEOPT_ZONEONLY) || PATHFIND_STEP >= 2
FormationGroup formationGroups[MAX_FORMATION_GROUPS];
SquadPathSignalEntry squadPathSignals[MAX_FORMATION_GROUPS];
uintptr_t pendingOrderHC[MAX_PENDING_ORDER];
int pendingOrderCount = 0;
#endif

#ifndef ZONEOPT_ZONEONLY

bool scatterPatchApplied = false;

// Binary-patch scatter branch in addOrderSelectedCharacters
//
// The function at RVA_ADD_ORDER_SELECTED has a branch:
//   if (i == *v31)    // is this the leader?
//       vtable+792(char, building, subject, exactDest);
//   else
//       scatter calculation using sqrtf...
//       vtable+792(char, building, subject, scatteredDest);
//
// We patch the conditional jump to force ALL characters through the leader
// code path, giving everyone the exact click destination (zero scatter).
// Arrival scatter is handled separately by PollFormationGroups.

bool ApplyScatterPatch()
{
	uintptr_t funcBase = gameBase + RVA_ADD_ORDER_SELECTED;
	const int funcSize = 1714;
	unsigned char* funcBytes = (unsigned char*)funcBase;
	uintptr_t sqrtfAddr = gameBase + RVA_SQRTF;

	// Step 1: Find the 'call sqrtf' instruction (E8 rel32).
	// This call appears once in the function, inside the scatter block.
	int callOffset = -1;
	for (int i = 0; i < funcSize - 5; ++i)
	{
		if (funcBytes[i] == 0xE8)
		{
			int rel32 = *(int*)(funcBytes + i + 1);
			uintptr_t target = (uintptr_t)(funcBytes + i + 5) + rel32;
			if (target == sqrtfAddr)
			{
				callOffset = i;
				break;
			}
		}
	}

	if (callOffset < 0)
	{
		LogMsg("[ZoneOpt] Scatter patch: sqrtf call not found");
		return false;
	}

	// Step 2: Walk backward from sqrtf call to find the preceding
	// conditional jump (the leader check). Look for JE/JNE in both
	// short (2-byte) and near (6-byte) forms.
	int patchOffset = -1;
	int jumpLen = 0;
	bool isJE = false;  // true = JE (leader is jump target), false = JNE (scatter is jump target)

	for (int scan = callOffset - 2; scan >= 0 && scan >= callOffset - 128; --scan)
	{
		// 6-byte near conditional: 0F 84 (JE) or 0F 85 (JNE)
		if (scan >= 1 && funcBytes[scan - 1] == 0x0F)
		{
			if (funcBytes[scan] == 0x84)
			{
				patchOffset = scan - 1;
				jumpLen = 6;
				isJE = true;
				break;
			}
			if (funcBytes[scan] == 0x85)
			{
				patchOffset = scan - 1;
				jumpLen = 6;
				isJE = false;
				break;
			}
		}
		// 2-byte short conditional: 74 (JE) or 75 (JNE)
		if (funcBytes[scan] == 0x74)
		{
			patchOffset = scan;
			jumpLen = 2;
			isJE = true;
			break;
		}
		if (funcBytes[scan] == 0x75)
		{
			patchOffset = scan;
			jumpLen = 2;
			isJE = false;
			break;
		}
	}

	if (patchOffset < 0)
	{
		LogMsg("[ZoneOpt] Scatter patch: conditional jump not found before sqrtf");
		return false;
	}

	// Step 3: Patch the conditional jump.
	unsigned char* patchAddr = funcBytes + patchOffset;

	DWORD oldProtect;
	if (!VirtualProtect(patchAddr, jumpLen, PAGE_EXECUTE_READWRITE, &oldProtect))
	{
		LogMsg("[ZoneOpt] Scatter patch: VirtualProtect failed");
		return false;
	}

	if (isJE)
	{
		// JE: leader path is the jump target. Make unconditional.
		if (jumpLen == 6)
		{
			// 0F 84 rel32 -> 90 E9 rel32 (NOP + JMP near, same target)
			int rel32 = *(int*)(patchAddr + 2);
			patchAddr[0] = 0x90;  // NOP (absorbs the extra byte)
			patchAddr[1] = 0xE9;  // JMP near
			*(int*)(patchAddr + 2) = rel32;  // rel32 is relative to end of JE (6 bytes) but
			                                  // JMP end is at patchAddr+6 too, so same offset
		}
		else
		{
			// 74 rel8 -> EB rel8 (JMP short)
			patchAddr[0] = 0xEB;
		}
	}
	else
	{
		// JNE: scatter path is the jump target. NOP it so all chars
		// fall through to the leader path.
		for (int b = 0; b < jumpLen; ++b)
			patchAddr[b] = 0x90;
	}

	VirtualProtect(patchAddr, jumpLen, oldProtect, &oldProtect);

	{
		std::ostringstream ss;
		ss << "[ZoneOpt] Scatter patch: " << (isJE ? "JE" : "JNE")
		   << " at func+" << patchOffset << " (" << jumpLen << " bytes)"
		   << ", sqrtf at func+" << callOffset;
		LogMsg(ss.str());
	}

	scatterPatchApplied = true;
	return true;
}



// =========================================================================
// Formation group creation
// =========================================================================

static unsigned int nextFormationGroupId = 0;

void ClearFormationGroups()
{
	for (int i = 0; i < MAX_FORMATION_GROUPS; ++i)
	{
		formationGroups[i].active = false;
		formationGroups[i].gathered = false;
		formationGroups[i].count = 0;
		formationGroups[i].lastReissueTime = 0.0;
#if PATHFIND_STEP >= 7
		// Defensive: reset slot dedup cache. No watched chars exist on fresh
		// game load so ClearFormationGroupIdForSlot is not needed here.
		spcSlots[i].formationExitGX         = -1;
		spcSlots[i].formationExitGY         = -1;
		spcSlots[i].formationExitUpdateTime = 0.0;
#endif
	}
}

// Deactivate any existing formation group that shares a member with chars[]
static void DeactivateOverlappingGroups(uintptr_t* chars, int charCount)
{
	for (int g = 0; g < MAX_FORMATION_GROUPS; ++g)
	{
		if (!formationGroups[g].active)
			continue;
		for (int m = 0; m < formationGroups[g].count; ++m)
		{
			if (!formationGroups[g].members[m].character)
				continue;
			for (int c = 0; c < charCount; ++c)
			{
				if (formationGroups[g].members[m].character == chars[c])
				{
					formationGroups[g].active = false;
					goto next_group;
				}
			}
		}
		next_group:;
	}
}

void CreateFormationGroup(const float* dest, uintptr_t* chars, int charCount)
{
	double now = ElapsedSec();

	// Debounce: if a recent group (< 500ms) shares members with this order,
	// update its destination in-place instead of deactivate+recreate.
	// Handles drag/shift-click paths where the game fires many rapid
	// addOrderSelectedCharacters calls — only the final destination matters
	// for arrival scatter.
	for (int g = 0; g < MAX_FORMATION_GROUPS; ++g)
	{
		if (!formationGroups[g].active)
			continue;
		if (now - formationGroups[g].createdTime > 0.5)
			continue;

		bool shared = false;
		for (int m = 0; m < formationGroups[g].count && !shared; ++m)
		{
			if (!formationGroups[g].members[m].character)
				continue;
			for (int c = 0; c < charCount; ++c)
			{
				if (formationGroups[g].members[m].character == chars[c])
				{ shared = true; break; }
			}
		}

		if (shared)
		{
			formationGroups[g].destX = dest[0];
			formationGroups[g].destY = dest[1];
			formationGroups[g].destZ = dest[2];
			formationGroups[g].createdTime = now;
			for (int m = 0; m < formationGroups[g].count; ++m)
				formationGroups[g].members[m].dispatched = false;

			// Re-fire squad path signal with updated destination.
			// For pre-gathered groups the signal was already fired at
			// creation — must update it for the new destination.
			if (squadPathCacheEnabled && formationGroups[g].gathered)
			{
				squadPathSignals[g].destHavokX  = dest[0] * 0.1f;
				squadPathSignals[g].destHavokY  = dest[1] * 0.1f;
				squadPathSignals[g].destHavokZ  = dest[2] * 0.1f;
				squadPathSignals[g].memberCount = formationGroups[g].count;
				squadPathSignals[g].signalTime  = now;
				InterlockedExchange(&squadPathSignals[g].active, 1);
			}
			return;
		}
	}

	// Deactivate groups superseded by this new order
	DeactivateOverlappingGroups(chars, charCount);

	// Find free slot or evict oldest
	int slot = -1;
	double oldestTime = 1e20;
	int oldestSlot = 0;
	for (int i = 0; i < MAX_FORMATION_GROUPS; ++i)
	{
		if (!formationGroups[i].active)
		{
			slot = i;
			break;
		}
		if (formationGroups[i].createdTime < oldestTime)
		{
			oldestTime = formationGroups[i].createdTime;
			oldestSlot = i;
		}
	}
	if (slot < 0)
		slot = oldestSlot;

	FormationGroup& grp = formationGroups[slot];
	grp.destX = dest[0];
	grp.destY = dest[1];
	grp.destZ = dest[2];
	grp.count = 0;
	grp.createdTime = now;
	grp.active = true;
	grp.gathered = false;
	grp.lastReissueTime = 0.0;

	// Capture leader position as the gather point
	grp.startX = *(float*)(chars[0] + OFF_CHAR_POS_X);
	grp.startY = *(float*)(chars[0] + OFF_CHAR_POS_Y);
	grp.startZ = *(float*)(chars[0] + OFF_CHAR_POS_Z);

	unsigned int groupId = nextFormationGroupId++;

	// Scatter radius: half of original formula for tighter arrival spread
	float scatterRadius = sqrtf((float)charCount * 60.0f) * 0.5f + 5.0f;
	int nonLeaderCount = charCount - 1;
	bool allAlreadyNear = true;

	for (int i = 0; i < charCount && grp.count < MAX_FORMATION_MEMBERS; ++i)
	{
		uintptr_t ch = chars[i];
		uintptr_t cm = *(uintptr_t*)(ch + OFF_CHAR_MOVEMENT);
		if (!cm) continue;

		// Check if this member is already near the leader (gather point)
		if (allAlreadyNear)
		{
			float gdx = *(float*)(ch + OFF_CHAR_POS_X) - grp.startX;
			float gdz = *(float*)(ch + OFF_CHAR_POS_Z) - grp.startZ;
			if (gdx * gdx + gdz * gdz > GATHER_RADIUS_SQ)
				allAlreadyNear = false;
		}

		FormationMember& m = grp.members[grp.count];
		m.character = ch;
		m.charMovement = cm;
		m.dispatched = false;
		m.gatherSent = false;

		if (grp.count == 0)
		{
			// Leader stays at exact destination
			m.scatterX = 0.0f;
			m.scatterZ = 0.0f;
		}
		else if (nonLeaderCount > 0)
		{
			// Deterministic circle: evenly spaced around destination
			float angle = 6.2831853f * (float)(grp.count - 1) / (float)nonLeaderCount;
			m.scatterX = sinf(angle) * scatterRadius;
			m.scatterZ = cosf(angle) * scatterRadius;
		}
		grp.count++;
	}

#if PATHFIND_STEP >= 7
	// Set formationGroupId on each watched entry matching a member. Bg-thread
	// (findPathFull) reads this for dedup gate; volatile int = atomic on x86-64.
	{
		uintptr_t memberChars[MAX_FORMATION_MEMBERS_LIMIT];
		int mc = 0;
		for (int m = 0; m < grp.count && mc < MAX_FORMATION_MEMBERS_LIMIT; ++m)
		{
			if (grp.members[m].character)
				memberChars[mc++] = grp.members[m].character;
		}
		SetFormationGroupIdOnMembers(slot, memberChars, mc);
	}
#endif

	// Pre-gathered: skip the gather phase entirely. orig_addOrderSelected
	// (called after this returns) issues the move orders directly — no
	// gather cancel, no triple-move-order sequence.
	if (allAlreadyNear && grp.count > 1)
	{
		grp.gathered = true;

		// Fire squad path cache signal immediately (normally fires at
		// gather->travel transition in PollFormationGroups)
		if (squadPathCacheEnabled)
		{
			squadPathSignals[slot].destHavokX  = grp.destX * 0.1f;
			squadPathSignals[slot].destHavokY  = grp.destY * 0.1f;
			squadPathSignals[slot].destHavokZ  = grp.destZ * 0.1f;
			squadPathSignals[slot].memberCount = grp.count;
			squadPathSignals[slot].signalTime  = now;
			InterlockedExchange(&squadPathSignals[slot].active, 1);
		}
	}

	{
		std::ostringstream ss;
		ss << "[ZoneOpt] Formation group " << groupId << ": "
		   << grp.count << " members, dest=(" << std::fixed << std::setprecision(0)
		   << grp.destX << "," << grp.destZ << ")"
		   << " radius=" << std::setprecision(1) << scatterRadius;
		if (grp.gathered)
			ss << " [pre-gathered]";
		LogMsg(ss.str());
	}
}



// =========================================================================
// Formation group polling
// =========================================================================

void PollFormationGroups()
{
	double now = ElapsedSec();

	// Read playerCharacters lektor once for validation
	uintptr_t playerIntf = *(uintptr_t*)(gameBase + RVA_GLOBAL_PLAYER);
	unsigned int scCount = 0;
	uintptr_t* scStuff = NULL;
	if (playerIntf)
	{
		scCount = GetPlayerCharCount(playerIntf);
		scStuff = GetPlayerCharStuff(playerIntf);
	}

	for (int g = 0; g < MAX_FORMATION_GROUPS; ++g)
	{
		if (!formationGroups[g].active)
			continue;

		FormationGroup& grp = formationGroups[g];

		// Stale timeout
		if (now - grp.createdTime > FORMATION_TIMEOUT)
		{
			int pending = 0;
			for (int m = 0; m < grp.count; ++m)
				if (grp.members[m].character && !grp.members[m].dispatched) pending++;
			{
				std::ostringstream ss;
				ss << "[ZoneOpt] Formation group timeout: " << pending
				   << "/" << grp.count << " pending after "
				   << std::fixed << std::setprecision(0) << FORMATION_TIMEOUT << "s";
				LogMsg(ss.str());
			}
#if PATHFIND_STEP >= 7
			ClearFormationGroupIdForSlot(g);
			// Clear slot dedup cache so a reused group slot doesn't see stale
			// fresh-timestamp and mis-dedupe to previous group's ExitFace.
			spcSlots[g].formationExitUpdateTime = 0.0;
			spcSlots[g].formationExitGX         = -1;
			spcSlots[g].formationExitGY         = -1;
#endif
			grp.active = false;
			continue;
		}

		// Check that all members are still set to "Running Together".
		// If any member changed speed mode, deactivate the group and
		// let the game's regular movement logic handle it.
		{
			bool stillGrouped = true;
			for (int m = 0; m < grp.count; ++m)
			{
				if (!grp.members[m].character) continue;
				uintptr_t cm = *(uintptr_t*)(grp.members[m].character + OFF_CHAR_MOVEMENT);
				if (!cm || *(int*)(cm + OFF_CMOV_SPEED_MODE) != MOVESPEED_GROUPED)
				{ stillGrouped = false; break; }
			}
			if (!stillGrouped)
			{
#if PATHFIND_STEP >= 7
				ClearFormationGroupIdForSlot(g);
				spcSlots[g].formationExitUpdateTime = 0.0;
				spcSlots[g].formationExitGX         = -1;
				spcSlots[g].formationExitGY         = -1;
#endif
				grp.active = false;
				continue;
			}
		}

		// ============================================================
		// Phase 1: Gathering — send all members to leader's position
		// ============================================================
		if (!grp.gathered)
		{
			bool allNear = true;
			bool anySent = false;

			for (int m = 0; m < grp.count; ++m)
			{
				FormationMember& mem = grp.members[m];
				if (!mem.character) continue;

				// Validate alive
				bool alive = false;
				if (scStuff && scCount > 0 && scCount <= 200)
				{
					for (unsigned int j = 0; j < scCount; ++j)
					{
						if (scStuff[j] == mem.character) { alive = true; break; }
					}
				}
				if (!alive) { mem.character = 0; continue; }

				// Send gather order once per member
				if (!mem.gatherSent)
				{
					uintptr_t charVtable = *(uintptr_t*)mem.character;
					if (charVtable)
					{
						typedef void (*moveOrderFn_t)(uintptr_t, void*, void*, const float*);
						moveOrderFn_t fn_moveOrder = (moveOrderFn_t)(*(uintptr_t*)(charVtable + 0x318));
						if (fn_moveOrder)
						{
							float gatherPos[3] = { grp.startX, grp.startY, grp.startZ };
							fn_moveOrder(mem.character, NULL, NULL, gatherPos);
						}
					}
					mem.gatherSent = true;
					anySent = true;
				}

				// Check proximity to gather point
				float dx = GetCharPosX(mem.character) - grp.startX;
				float dz = GetCharPosZ(mem.character) - grp.startZ;
				if (dx * dx + dz * dz > GATHER_RADIUS_SQ)
					allNear = false;
			}

			bool gatherTimeout = (now - grp.createdTime > GATHER_TIMEOUT);
			if ((allNear && !anySent) || gatherTimeout)
			{
				grp.gathered = true;

				// Arm multi-call path probe to capture the path request flow
				// for this formation dispatch (8s capture window)
				if (pathfindDiagEnabled)
					ArmPathProbe();

				// Signal squad path cache: all members about to pathfind to same dest.
				// Store raw Havok coords (world/10). hook_findPathFull adds the
				// current streaming offset at match time (immune to zone transitions).
				if (squadPathCacheEnabled && grp.count > 1)
				{
					squadPathSignals[g].destHavokX  = grp.destX * 0.1f;
					squadPathSignals[g].destHavokY  = grp.destY * 0.1f;
					squadPathSignals[g].destHavokZ  = grp.destZ * 0.1f;
					squadPathSignals[g].memberCount = grp.count;
					squadPathSignals[g].signalTime  = now;
					InterlockedExchange(&squadPathSignals[g].active, 1);
				}

				// Send all alive members to the destination
				int departed = 0;
				for (int m = 0; m < grp.count; ++m)
				{
					FormationMember& mem = grp.members[m];
					if (!mem.character) continue;

					uintptr_t charVtable = *(uintptr_t*)mem.character;
					if (!charVtable) continue;

					typedef void (*moveOrderFn_t)(uintptr_t, void*, void*, const float*);
					moveOrderFn_t fn_moveOrder = (moveOrderFn_t)(*(uintptr_t*)(charVtable + 0x318));
					if (fn_moveOrder)
					{
						float destPos[3] = { grp.destX, grp.destY, grp.destZ };
						fn_moveOrder(mem.character, NULL, NULL, destPos);
						departed++;
					}
				}

				{
					std::ostringstream ss;
					ss << "[ZoneOpt] Formation gathered: " << departed << " departing"
					   << (gatherTimeout ? " (timeout)" : "");
					LogMsg(ss.str());
				}
			}
			continue;  // skip scatter logic until gathered
		}

		// ============================================================
		// Phase 2: Scatter — dispatch arrival spread at destination
		// ============================================================
		int aliveCount = 0;
		int doneCount = 0;

		for (int m = 0; m < grp.count; ++m)
		{
			FormationMember& mem = grp.members[m];
			if (!mem.character)
			{
				doneCount++;
				continue;
			}

			// Validate character is still alive
			bool alive = false;
			if (scStuff && scCount > 0 && scCount <= 200)
			{
				for (unsigned int j = 0; j < scCount; ++j)
				{
					if (scStuff[j] == mem.character) { alive = true; break; }
				}
			}
			if (!alive)
			{
				mem.character = 0;
				doneCount++;
				continue;
			}
			aliveCount++;

			if (mem.dispatched)
			{
				doneCount++;
				continue;
			}

			// Check proximity to exact destination
			float charX = GetCharPosX(mem.character);
			float charZ = GetCharPosZ(mem.character);

			float dx = charX - grp.destX;
			float dz = charZ - grp.destZ;
			float distSq = dx * dx + dz * dz;

			if (distSq < SCATTER_APPROACH_DIST_SQ)
			{
				// Leader (scatter offset 0,0) needs no redirection
				if (mem.scatterX == 0.0f && mem.scatterZ == 0.0f)
				{
					mem.dispatched = true;
					doneCount++;
					continue;
				}

				// Dispatch scatter via playerMoveOrderDefault (vtable+792)
				uintptr_t charVtable = *(uintptr_t*)mem.character;
				if (!charVtable)
				{
					mem.character = 0;
					doneCount++;
					continue;
				}

				typedef void (*moveOrderFn_t)(uintptr_t, void*, void*, const float*);
				moveOrderFn_t fn_moveOrder = (moveOrderFn_t)(*(uintptr_t*)(charVtable + 0x318));
				if (!fn_moveOrder)
				{
					mem.character = 0;
					doneCount++;
					continue;
				}

				float scatterDest[3];
				scatterDest[0] = grp.destX + mem.scatterX;
				scatterDest[1] = grp.destY;
				scatterDest[2] = grp.destZ + mem.scatterZ;

				fn_moveOrder(mem.character, NULL, NULL, scatterDest);
				mem.dispatched = true;
				doneCount++;

				{
					std::ostringstream ss;
					ss << "[ZoneOpt] Formation scatter: member " << m
					   << " dist=" << std::fixed << std::setprecision(0) << sqrtf(distSq)
					   << " offset=(" << std::setprecision(1)
					   << mem.scatterX << "," << mem.scatterZ << ")";
					LogMsg(ss.str());
				}
			}
		}

		// Group complete when all members dispatched or removed
		if (doneCount >= grp.count)
		{
			std::ostringstream ss;
			ss << "[ZoneOpt] Formation group complete: "
			   << aliveCount << " alive, " << doneCount << "/" << grp.count << " done";
			LogMsg(ss.str());
#if PATHFIND_STEP >= 7
			ClearFormationGroupIdForSlot(g);
			spcSlots[g].formationExitUpdateTime = 0.0;
			spcSlots[g].formationExitGX         = -1;
			spcSlots[g].formationExitGY         = -1;
#endif
			grp.active = false;
		}
	}
}


// =========================================================================
// Island re-issue helpers (islands.cpp, ISLAND_STEP >= 3). Main thread only.
// =========================================================================

int FormationSlotForCharacter(uintptr_t character)
{
	if (!character) return -1;
	for (int g = 0; g < MAX_FORMATION_GROUPS; ++g)
	{
		// Active groups only; a still-gathering group is reported too so its
		// members are never re-issued individually (FormationReissueTravel
		// declines until the group has gathered).
		const FormationGroup& grp = formationGroups[g];
		if (!grp.active) continue;
		for (int m = 0; m < grp.count; ++m)
			if (grp.members[m].character == character)
				return g;
	}
	return -1;
}

uintptr_t FormationFirstAliveMember(int slot)
{
	if (slot < 0 || slot >= MAX_FORMATION_GROUPS) return 0;
	const FormationGroup& grp = formationGroups[slot];
	if (!grp.active) return 0;

	uintptr_t playerIntf = *(uintptr_t*)(gameBase + RVA_GLOBAL_PLAYER);
	unsigned int scCount = 0;
	uintptr_t* scStuff = NULL;
	if (playerIntf)
	{
		scCount = GetPlayerCharCount(playerIntf);
		scStuff = GetPlayerCharStuff(playerIntf);
	}
	if (!scStuff || scCount == 0 || scCount > 200) return 0;

	for (int m = 0; m < grp.count; ++m)
	{
		uintptr_t ch = grp.members[m].character;
		if (!ch) continue;
		for (unsigned int j = 0; j < scCount; ++j)
			if (scStuff[j] == ch)
				return ch;
	}
	return 0;
}

bool FormationReissueTravel(int slot, double now)
{
	if (slot < 0 || slot >= MAX_FORMATION_GROUPS) return false;
	FormationGroup& grp = formationGroups[slot];
	if (!grp.active || !grp.gathered) return false;
	if (grp.lastReissueTime > 0.0 && now - grp.lastReissueTime < 2.0) return false;

	uintptr_t playerIntf = *(uintptr_t*)(gameBase + RVA_GLOBAL_PLAYER);
	unsigned int scCount = 0;
	uintptr_t* scStuff = NULL;
	if (playerIntf)
	{
		scCount = GetPlayerCharCount(playerIntf);
		scStuff = GetPlayerCharStuff(playerIntf);
	}
	if (!scStuff || scCount == 0 || scCount > 200) return false;

	// Same dispatch as the gather->travel transition in PollFormationGroups.
	int sent = 0;
	int nudged = 0;
	for (int m = 0; m < grp.count; ++m)
	{
		FormationMember& mem = grp.members[m];
		if (!mem.character) continue;

		bool alive = false;
		for (unsigned int j = 0; j < scCount; ++j)
			if (scStuff[j] == mem.character) { alive = true; break; }
		if (!alive) { mem.character = 0; continue; }

		uintptr_t charVtable = *(uintptr_t*)mem.character;
		if (!charVtable) continue;
		typedef void (*moveOrderFn_t)(uintptr_t, void*, void*, const float*);
		moveOrderFn_t fn_moveOrder = (moveOrderFn_t)(*(uintptr_t*)(charVtable + 0x318));
		if (!fn_moveOrder) continue;

		float destPos[3] = { grp.destX, grp.destY, grp.destZ };

		// CharMovement::setDestination drops a new order within 2 units of the
		// last requested destination while it is routing to an island edge.
		// Every member received the exact grp.dest (scatter patch), so nudge.
		uintptr_t cm = *(uintptr_t*)(mem.character + OFF_CHAR_MOVEMENT);
		if (cm)
		{
			float lx = *(float*)(cm + OFF_CMOV_LAST_DEST);
			float lz = *(float*)(cm + OFF_CMOV_LAST_DEST + 8);
			float ddx = lx - grp.destX, ddz = lz - grp.destZ;
			if (ddx * ddx + ddz * ddz < 4.0f)
			{
				destPos[0] += 3.0f;
				nudged++;
			}
		}

		fn_moveOrder(mem.character, NULL, NULL, destPos);
		sent++;
	}

	if (sent == 0) return false;
	grp.lastReissueTime = now;

	std::ostringstream ss;
	ss << "[ZoneOpt] Formation reissue: group slot " << slot
	   << " " << sent << " members (" << nudged << " nudged)"
	   << " dest=(" << std::fixed << std::setprecision(0)
	   << grp.destX << "," << grp.destZ << ")";
	LogMsg(ss.str());
	return true;
}

#endif // !ZONEOPT_ZONEONLY
