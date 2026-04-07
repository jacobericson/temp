// formation.h — Group cohesion: scatter patch, formation groups, squad path signals (Layer 3)
// FULL builds only (excluded by ZONEOPT_ZONEONLY via config.h inline stubs).
// Depends on: config.h

#ifndef KENSHI_ZONE_OPT_FORMATION_H
#define KENSHI_ZONE_OPT_FORMATION_H

#include "config.h"

// Types, constants, and state needed by pathfinding (PATHFIND_STEP >= 2) even in ZONEONLY builds.
// Formation functions (scatter patch, group creation, polling) remain FULL-only.
#if !defined(ZONEOPT_ZONEONLY) || PATHFIND_STEP >= 2


// =========================================================================
// Formation constants
// =========================================================================

const int MAX_FORMATION_GROUPS       = 8;
const int MAX_FORMATION_MEMBERS      = 30;
const float SCATTER_APPROACH_DIST_SQ = 2500.0f;  // 50 world units squared
const float GATHER_RADIUS_SQ         = 1600.0f;  // 40 world units squared
const double GATHER_TIMEOUT          = 15.0;      // seconds before skipping gather
const double FORMATION_TIMEOUT       = 120.0;     // seconds before auto-cleanup
const int MAX_PENDING_ORDER          = 30;


// =========================================================================
// Formation structs
// =========================================================================

struct FormationMember {
	uintptr_t character;
	uintptr_t charMovement;
	float scatterX;
	float scatterZ;
	bool dispatched;
	bool gatherSent;
};

struct FormationGroup {
	float destX, destY, destZ;
	float startX, startY, startZ;
	FormationMember members[MAX_FORMATION_MEMBERS_LIMIT];
	int count;
	double createdTime;
	bool active;
	bool gathered;
};

struct SquadPathSignalEntry {
	volatile long active;
	float destHavokX;
	float destHavokY;
	float destHavokZ;
	int   memberCount;
	double signalTime;
};


// =========================================================================
// Formation state (defined in formation.cpp)
// =========================================================================

extern bool scatterPatchApplied;
extern FormationGroup formationGroups[MAX_FORMATION_GROUPS];
extern SquadPathSignalEntry squadPathSignals[MAX_FORMATION_GROUPS];

// Pending-order buffer (main thread only, written by hooks.cpp, read by pathfind_hooks.cpp)
extern uintptr_t pendingOrderHC[MAX_PENDING_ORDER];
extern int pendingOrderCount;


#endif // !ZONEOPT_ZONEONLY || PATHFIND_STEP >= 2


// =========================================================================
// Formation functions (impl in formation.cpp, FULL builds only)
// =========================================================================

#ifndef ZONEOPT_ZONEONLY

bool ApplyScatterPatch();
void CreateFormationGroup(const float* dest, uintptr_t* chars, int charCount);
void PollFormationGroups();
void ClearFormationGroups();

#endif // !ZONEOPT_ZONEONLY

#endif // KENSHI_ZONE_OPT_FORMATION_H
