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
#if PATHFIND_STEP >= 7
const int    MAX_WATCHED            = 64;  // STEP 7: absorbs baseline scan of full player faction
#else
const int    MAX_WATCHED            = 32;
#endif
const float  EDGE_THRESHOLD         = 1800.0f;  // must be < zoneStep/2 (2304)


// =========================================================================
// Watched character struct + state (defined in tracking.cpp)
// =========================================================================

struct WatchedCharacter {
	uintptr_t character;
	uintptr_t charMovement;
#if PATHFIND_STEP >= 5
	bool      hasMoveOrder;
#endif
	int       destZoneX;
	int       destZoneY;
	int       currentZoneX;
	int       currentZoneY;
	double    addedTime;
#if PATHFIND_STEP >= 6
	// Bg thread (findPathFull) writes; main thread (hooks.cpp) reads via packed
	// 64-bit atomic. Sentinel EXIT_ZONE_NONE = "no ExitFace decoded".
	volatile LONG64 exitZonePacked;
	volatile double exitFaceUpdateTime;
#endif
#if PATHFIND_STEP >= 7
	// Main thread (formation.cpp) writes; bg thread (findPathFull) reads for
	// dedup gate. 32-bit aligned volatile int = atomic R/W on x86-64.
	volatile int formationGroupId;          // -1 = no group
	// Bg thread writes (decode result); main thread reads + atomically clears
	// at 1Hz drain via InterlockedExchange. Sentinel -1 = no pending preload.
	volatile int preloadAheadGX;
	volatile int preloadAheadGY;
	volatile double preloadAheadUpdateTime;
#endif
};

#if PATHFIND_STEP >= 6
// Sentinel: INT_MIN packed into both halves. PackExitZone(INT_MIN, INT_MIN) = this.
const LONG64 EXIT_ZONE_NONE = (LONG64)0x8000000080000000LL;

inline LONG64 PackExitZone(int gx, int gy) {
	return ((LONG64)(unsigned int)gx << 32) | (LONG64)(unsigned int)gy;
}
inline void UnpackExitZone(LONG64 packed, int* gx, int* gy) {
	*gx = (int)(packed >> 32);
	*gy = (int)(packed & 0xFFFFFFFFLL);
}
#endif

#if PATHFIND_STEP >= 7
// Sentinel for preloadAheadGX/GY. Zone coords are 0..63; -1 is safely out of range.
const int PRELOAD_AHEAD_NONE = -1;
#endif

extern WatchedCharacter watchedChars[MAX_WATCHED];
extern int numWatched;
extern double lastBaselineScan;
extern double lastActivePoll;
extern int hookOrderCount;


// =========================================================================
// Tracking functions (impl in tracking.cpp)
// =========================================================================

bool AddWatchedCharacter(uintptr_t character, uintptr_t charMovement,
                         int destZX, int destZY, int curZX, int curZY
#if PATHFIND_STEP >= 5
                         , bool hasMoveOrder
#endif
                         );
void RemoveWatchedCharacter(int index);
bool IsCharacterWatched(uintptr_t character);
void ScanCharacterZones(void* zoneMgr);
void PollActiveMovers(void* zoneMgr, double now);
void TieredCharacterPoll(void* zoneMgr, double now);
#if PATHFIND_STEP >= 5
void EnsurePlayerCharsWatched();
#endif
#if PATHFIND_STEP >= 7
// Main-thread helpers called from formation.cpp to set/clear formationGroupId
// on watched entries by group slot. Requires #include "tracking.h" in caller.
void SetFormationGroupIdOnMembers(int groupSlot, uintptr_t* chars, int charCount);
void ClearFormationGroupIdForSlot(int groupSlot);
#endif


#endif // KENSHI_ZONE_OPT_TRACKING_H
