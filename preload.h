// preload.h — Zone preloading: queues, promotion, eviction, grid calibration (Layer 3)
// Depends on: config.h

#ifndef KENSHI_ZONE_OPT_PRELOAD_H
#define KENSHI_ZONE_OPT_PRELOAD_H

#include "config.h"
#include "grid.h"


// =========================================================================
// Preloading constants
// =========================================================================

const int MAX_PRELOADED = 45;             // camera (12) + 2 moving squads (12 each) + still (9)
const float PRELOAD_THRESHOLD = 2500.0f;  // units from zone center (transition at 4147.2)
const double CHAR_SCAN_INTERVAL = 2.0;    // seconds between character scans (Phase 2B fallback)
const int MAX_CHAR_ZONES = 24;            // max unique zones from character scan
const int CAMERA_RESERVED = 12;
const double CAM_LOG_INTERVAL = 10.0;


// =========================================================================
// Preloading structs
// =========================================================================

struct PreloadedZone {
	void*  zoneEntry;
	int    gridX;
	int    gridY;
	bool   promoted;
	bool   pending;
	bool   registered;
	bool   registeredEmpty;
	bool   pipelineHandoff;
	bool   contentProcessed;   // vtable[4] called to populate things/objects
	double loadTimeSec;
	int    owner;
#if PRELOAD_STEP >= 1
	// Phase 18 Step 1 instrumentation (research/preload_pipeline.md §5 Step 1).
	// pipeLoadSec is the stable base for every Step 1 latency: loadTimeSec
	// itself gets reset at registration (registration -> promotion timer), so
	// it cannot be reused here. -1.0 = this slot did not come from our own
	// loadSingleZone call (already-accessible / already-loading / game-owned
	// branches in ProcessPreloadQueue), so no Step 1 sample is taken for it.
	double pipeLoadSec;
	bool   pipeIsReadySeen;    // isZoneReady(zoneEntry) observed true once
	bool   pipe264Seen;        // content+264 observed cleared once
#endif
};

struct QueuedZone {
	int gridX, gridY;
};


// =========================================================================
// Preloading state (defined in preload.cpp)
// =========================================================================

extern PreloadedZone preloadedZones[MAX_PRELOADED];
extern int numPreloaded;
extern int pendingCount;
extern int predictedCenterX;
extern int predictedCenterY;

extern QueuedZone cameraQueue[CAMERA_RESERVED];
extern int cameraQueueCount;
extern int cameraQueueNext;

extern QueuedZone charQueue[MAX_PRELOADED];
extern int charQueueCount;
extern int charQueueNext;

// Character scan state
extern double lastCharScanTime;
extern int charZonesQueued;

// Debug: camera zone logging
extern double lastCamLogTime;
extern int lastCameraGX;
extern int lastCameraGY;

// Logging counters (reset per transition)
extern int promotedCount;

// Crash-3 diagnostic: how many times ProcessPreloadQueue declined to call
// loadSingleZone because the game already owned the zone. Cumulative for the
// session, not per transition. See preload.cpp for why it should stay 0.
extern int preloadSkipLoaded;

// Round 2 fix 2b registry guard: how many times a zone was refused because its
// ZoneMap handle registration is not this content's (preload.cpp,
// ZoneRegistrationOk). Cumulative for the session, like preloadSkipLoaded, and
// printed beside it on the Transition line as regSkip=. Should stay 0: with
// the save-load reset unload in place no mod zone survives a load.
extern int regSkipCount;


// =========================================================================
// Preloading functions (impl in preload.cpp)
// =========================================================================

void ClearPreloadZones();
// Full reset, including the navmesh caches. Startup only.
void ClearPreloadState();
// Save-load reset: the same, but keeps the world-keyed navmesh caches.
void ClearPreloadStateForLoad();
// Save-load detector (main thread). Returns true while the game is loading a
// save, in which case the caller must skip all preload processing this frame.
bool PreloadCheckSaveLoad(void* zoneMgr);
void CalibrateZoneGrid(void* zoneMgr);
bool WorldToZoneGrid(float worldX, float worldZ, int* outX, int* outY);
bool EnqueueCameraZone(int gx, int gy);
bool EnqueueCharacterZone(int gx, int gy);
void FlushCameraQueue();
void EnqueueCameraGrid(int centerX, int centerY);
void EnqueueAheadZones(int centerX, int centerY, int fromX, int fromY, int owner);
void ProcessPreloadQueue(void* zoneMgr);
bool TrySquadSwitchSwap(void* zoneMgr, int camGX, int camGY);
void EvictStaleZones(void* zoneMgr, double now);
void TryRegisterPreloadedZones(void* zoneMgr, double now);
void TryPromotePreloadedZones(void* zoneMgr, double now);
void PreparePreloadedZonesForTransition(void* zoneMgr);

// Round 2 fix 2b: hook on the save-load reset's Set A/B unload, sub_14036C1E0
// (game.h RVA_RESET_UNLOAD_ZONES). Runs the original, then unloads every zone
// that still holds a content (the mod's: in neither set) through the same call
// the original uses, logs "Save load reset: ...", and (main thread only)
// clears the mod's state for the load. INI saveLoadUnload=false counts only.
void __fastcall hook_resetUnloadZones(void* zoneMgr);

#if PRELOAD_STEP >= 1
// =========================================================================
// H15: per-transition benchmark (screen / navReady / firstMove)
// =========================================================================
// screen: the existing "Transition: ... ms" duration, restated so H15 stands
// alone. navReady: dismissal -> the first main-thread frame where
// orig_isContentPending answers true for the camera focus zone (the original,
// not the hook). firstMove: dismissal -> a watched player character seen more
// than 50 units from its position at the first active-mover poll after the
// dismissal, or "-" past 60s. preload.cpp detects the dismissal (isTransitionActive
// going false between frames) and drives navReady; tracking.cpp's
// PollActiveMovers reports firstMove back through H15ReportFirstMove using the
// walk it already performs, so no second pass over watchedChars is needed.
// gen guards against a stale report from a benchmark that has already been
// closed (timed out or superseded by a new dismissal).
void H15ReportFirstMove(int gen, double firstMoveMs);
#endif


#endif // KENSHI_ZONE_OPT_PRELOAD_H
