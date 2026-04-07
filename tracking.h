// tracking.h — Character tracking: watched characters, movement polling (Layer 3)
// Depends on: config.h

#ifndef KENSHI_ZONE_OPT_TRACKING_H
#define KENSHI_ZONE_OPT_TRACKING_H

#include "config.h"


// =========================================================================
// Character tracking constants
// =========================================================================

const double BASELINE_SCAN_INTERVAL = 5.0;
const double ACTIVE_POLL_INTERVAL   = 1.0;
const int    MAX_WATCHED            = 32;
const float  EDGE_THRESHOLD         = 1800.0f;  // must be < zoneStep/2 (2304)


// =========================================================================
// Watched character struct + state (defined in tracking.cpp)
// =========================================================================

struct WatchedCharacter {
	uintptr_t character;
	uintptr_t charMovement;
	int       destZoneX;
	int       destZoneY;
	int       currentZoneX;
	int       currentZoneY;
	double    addedTime;
};

extern WatchedCharacter watchedChars[MAX_WATCHED];
extern int numWatched;
extern double lastBaselineScan;
extern double lastActivePoll;
extern int hookOrderCount;


// =========================================================================
// Tracking functions (impl in tracking.cpp)
// =========================================================================

bool AddWatchedCharacter(uintptr_t character, uintptr_t charMovement,
                         int destZX, int destZY, int curZX, int curZY);
void RemoveWatchedCharacter(int index);
bool IsCharacterWatched(uintptr_t character);
void ScanCharacterZones(void* zoneMgr);
void PollActiveMovers(void* zoneMgr, double now);
void TieredCharacterPoll(void* zoneMgr, double now);


#endif // KENSHI_ZONE_OPT_TRACKING_H
