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


// =========================================================================
// Preloading functions (impl in preload.cpp)
// =========================================================================

void ClearPreloadZones();
void ClearPreloadState();
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


#endif // KENSHI_ZONE_OPT_PRELOAD_H
