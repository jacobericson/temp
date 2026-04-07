// game.h — RVA constants, game structure offsets, function pointers, inline accessors (Layer 1)
// Depends on: core.h

#ifndef KENSHI_ZONE_OPT_GAME_H
#define KENSHI_ZONE_OPT_GAME_H

#include "core.h"


// =========================================================================
// RVA constants (Kenshi Steam 1.0.65)
// =========================================================================

// From KenshiAddressLogger (runtime-resolved, authoritative)
const size_t RVA_SHOW_LOADING_MESSAGE = 0x6E9800;  // ForgottenGUI::showLoadingMessage(bool)

// From IDA Pro (same binary, not in KenshiLib exports)
const size_t RVA_IS_CONTENT_PENDING   = 0x3AB4F0;  // SectionManager::isContentPending (514 bytes)

// Hook targets
const size_t RVA_UPDATE_CAMERA_ZONE   = 0xA11DA0;  // ZoneManager::updateCameraZone (882 bytes)
const size_t RVA_STATE_MACHINE_DRIVER = 0xA0E950;  // ZoneManager::stateMachineDriver (1521 bytes)

// From KenshiAddressLogger (runtime-resolved, authoritative)
const size_t RVA_ADD_ORDER_SELECTED   = 0x7F9280;  // PlayerInterface::addOrderSelectedCharacters (1714 bytes)

// Called functions (thunk RVAs)
const size_t RVA_LOAD_SINGLE_ZONE    = 0x16243;   // thunk -> 0xA0D6A0 (405 bytes)
const size_t RVA_REGISTER_ZONE_SECTIONS = 0x2D7B8; // thunk -> 0x3ABF00 (923 bytes)
const size_t RVA_NOTIFY_ZONE_READY   = 0x26BC0;   // thunk -> 0x8F3E00 (38 bytes)
const size_t RVA_FINALIZE_ZONE_RES   = 0x32D62;   // thunk -> 0x8F5100 (278 bytes)
const size_t RVA_NOTIFY_ACCESSIBLE   = 0xD5BC;    // thunk -> 0x9FC720 (659 bytes)
const size_t RVA_FLUSH_PENDING_WORK  = 0x2E073;   // thunk -> 0x927540

// Handle resolution (called from hook to iterate selected characters)
const size_t RVA_RESOLVE_HANDLE      = 0x3E419;   // thunk to handle->object resolution
const size_t RVA_HANDLE_TABLE        = 0x2132F38; // global handle lookup table (qword_142132F38)
const size_t RVA_HANDLE_SENTINEL     = 0x2132F10; // stale handle sentinel (qword_142132F10)

// NavMesh queue lock functions (thunk RVAs)
const size_t RVA_PATH_BUILDER_INIT     = 0x3D136;
const size_t RVA_PATH_BUILDER_FINALIZE = 0x4F8AE;
const size_t RVA_READER_UNLOCK         = 0x2E843;

// NavMesh cache: function RVAs (zone optimization -- included in all builds)
const size_t RVA_PROCESS_JOB_ALT    = 0x3CBE60;  // NavMeshGenerator::processJobAlt (8645 bytes)
const size_t RVA_DISPATCH_JOB       = 0x3CE030;  // NavMeshGenerator::dispatchJob (691 bytes, impl)
const size_t RVA_BUILD_COLLISION    = 0x39810;   // thunk -> 0x3CB700 (925 bytes)
const size_t RVA_PARTIAL_FIXUP      = 0x1C418;   // sub_14001C418: type 1 partial boundary fixup
const size_t RVA_ENQUEUE_TO_PROC_QUEUE = 0x2EEBA; // enqueue job to processing queue (+184)

// Game CRT allocators (safe on bg thread)
const size_t RVA_GAME_NEW          = 0xED563A;
const size_t RVA_GAME_DELETE       = 0xED5628;
const size_t RVA_GAME_NEW_ARR      = 0xED5634;
const size_t RVA_GAME_DEL_ARR      = 0xED562E;

// Global scratch buffer (used by processJobAlt + buildCollision internally)
const size_t RVA_SCRATCH_BUFFER    = 0x212DE98;
const size_t RVA_SCRATCH_SIZE      = 0x1D2C0D0;

// Havok TLS allocator index
const size_t RVA_HAVOK_TLS_INDEX       = 0x21369C8;

// hkaiNavMesh constructor
const size_t RVA_NAVMESH_CTOR          = 0xD32E30;
// hkaiNavMeshGenerationSettings constructor (writes vtable, inits arrays/refcount)
const size_t RVA_SETTINGS_CTOR         = 0xDD99D0;

// NavMeshResult::populate -- wrapper calling realGenerate
const size_t RVA_NM_RESULT_POPULATE    = 0xE0AFF0;  // 49 bytes
// HavokNavMesh::realGenerate -- the actual heavy compute
const size_t RVA_REAL_GENERATE         = 0xDFC060;   // 61,317 bytes

// Havok thread init sequence (for worker thread TLS initialization)
const size_t RVA_HAVOK_CONTEXT_INIT    = 0xBA4770;
const size_t RVA_HAVOK_GET_MANAGER     = 0xBAF970;
const size_t RVA_HAVOK_POST_REG_INIT   = 0xBAECF0;
const size_t RVA_HAVOK_CLEANUP         = 0xBAED40;
const size_t RVA_HAVOK_CTX_CLEANUP     = 0xBA9580;

// Havok FLA unlocked methods (need CS patch for parallel workers)
const size_t RVA_FLA_RESET_PEAK  = 0xBCB760;  // 84 bytes, resetPeakMemoryStatistics
const size_t RVA_FLA_CAN_ALLOC   = 0xBCBC00;  // 90 bytes, canAllocTotal
const size_t RVA_FLA_BUF_REALLOC = 0xBA96A0;  // 122 bytes, bufRealloc (shared)
const size_t RVA_FLA_GC          = 0xBCCCB0;  // 23 bytes, destructor (also garbageCollect target)
const size_t RVA_FLA_VTABLE      = 0x177F728; // FLA primary vtable (for validation)

// Clone finalize guards (prevent operations on uninitialized OverrideSettings entries)
const size_t RVA_SIMPL_SETTINGS_DTOR = 0x3DA120;  // 119 bytes, SimplificationSettings::dtor
const size_t RVA_EDGE_PROCESS        = 0xDD92F0;  // edgeProcess: per-entry cleanup in finalize

#ifndef ZONEOPT_ZONEONLY
// Group cohesion: CharMovement::setDestination (called for arrival scatter)
const size_t RVA_CHARMOV_SET_DEST   = 0x660AF0;
const size_t RVA_SQRTF              = 0xED5F7E;  // sqrtf import (scatter patch anchor)
#endif // !ZONEOPT_ZONEONLY

#if PATHFIND_STEP >= 1
// Pathfinding diagnostics: hook targets (contentStream bg thread)
const size_t RVA_CS_FIND_PATH       = 0x3AA950;
const size_t RVA_CS_CHECK_FACE_CONN = 0x3A5B00;
const size_t RVA_FIND_PATH_FULL     = 0xCE56D0;
const size_t RVA_CS_FIND_PATH_FALLBACK = 0x3AABF0;

// Squad path priority boost (main thread hooks)
const size_t RVA_REQUEST_PATH       = 0x145CB0;
const size_t RVA_PATH_REQ_SUBMIT    = 0x3AAEF0;
const size_t RVA_ENQUEUE_PATH_REQ   = 0x3B6110;
#endif // PATHFIND_STEP >= 1

// Global data RVAs
const size_t RVA_GLOBAL_SECTION_MGR  = 0x2133560;
const size_t RVA_GLOBAL_GATE_OBJ    = 0x21330C8;
const size_t RVA_GLOBAL_PLAYER       = 0x2133630;


// =========================================================================
// Game structure offsets (from decompilation + KenshiLib headers)
// =========================================================================

// RootObjectBase / Character
const size_t OFF_CHAR_POS_X      = 0x48;
const size_t OFF_CHAR_POS_Y      = 0x4C;
const size_t OFF_CHAR_POS_Z      = 0x50;
const size_t OFF_CHAR_MOVEMENT   = 0x640;

// CharMovement
const size_t OFF_CMOV_SPEED_MODE = 0x20;   // MoveSpeed enum: 0=WALK,1=JOG,2=RUN,3=GROUPED,4=NO_CHANGE
const size_t OFF_CMOV_HAVOK_CHAR = 0x320;
const int    MOVESPEED_GROUPED   = 3;      // "Running Together" mode

// ZoneMap entry (360 bytes each in ZoneManager zone array)
const int    ZONE_ENTRY_SIZE     = 360;
const size_t OFF_ZONE_CONTENT    = 0;
const size_t OFF_ZONE_COORDS_X   = 24;
const size_t OFF_ZONE_COORDS_Y   = 28;
const size_t OFF_ZONE_IS_LOADING = 176;
const size_t OFF_ZONE_IS_ACCESS  = 177;
const size_t OFF_ZONE_CENTER_X   = 248;
const size_t OFF_ZONE_CENTER_Z   = 256;
const size_t OFF_ZMC_THINGS_COUNT = 88;

// ZoneManager
const size_t OFF_ZM_ZONE_BASE    = 200;
const size_t OFF_ZM_CURRENT_ZONE = 1475024;
const size_t OFF_ZM_STATE        = 1475032;
const int    ZONE_GRID_MAX       = 63;

// PlayerInterface (playerCharacters lektor at +0x2B0)
const size_t OFF_PI_CHAR_COUNT   = 0x2B8;
const size_t OFF_PI_CHAR_STUFF   = 0x2C0;

// Selected characters linked list
const size_t OFF_PI_SEL_INDEX    = 544;
const size_t OFF_PI_SEL_COUNT    = 552;
const size_t OFF_PI_SEL_ARRAY    = 576;
const size_t OFF_SEL_NODE_HANDLE = 16;
const size_t OFF_SEL_NODE_TYPE   = 24;

// NavMeshGenerator (via section manager)
const size_t OFF_MGR_NAVMESH_GEN = 656;
const size_t OFF_NAVMESH_THREAD  = 8;

#if PATHFIND_STEP >= 1
// hkaiStreamingCollection
const size_t OFF_SC_INSTANCES_SIZE = 40;
#endif


// =========================================================================
// Address helper
// =========================================================================

extern uintptr_t gameBase;
extern void* g_cachedZoneMgr;

inline void* GameAddr(size_t rva)
{
	return (void*)(gameBase + rva);
}


// =========================================================================
// Function pointer typedefs (called, not hooked)
// =========================================================================

typedef int   (*loadSingleZone_t)(void* zoneEntry, int radius, int param);
typedef void* (*notifyZoneReady_t)(void* zoneEntry);
typedef void  (*finalizeZoneResources_t)(void* sectionEntry);
typedef void  (*notifyAccessible_t)(void* zoneMapContent);
typedef int   (*flushPendingWork_t)();
typedef void  (*registerZoneSections_t)(void* sectionMgr, void* zoneEntry);

extern loadSingleZone_t        fn_loadSingleZone;
extern notifyZoneReady_t       fn_notifyZoneReady;
extern finalizeZoneResources_t fn_finalizeZoneResources;
extern notifyAccessible_t      fn_notifyAccessible;
extern flushPendingWork_t      fn_flushPendingWork;
extern registerZoneSections_t  fn_registerZoneSections;

// Handle resolution for selected character iteration
typedef void* (*resolveHandle_t)(void* table, void* handle);
extern resolveHandle_t fn_resolveHandle;
extern void*           g_handleTable;

// NavMesh queue lock functions (acquire/release input queue lock at navMeshGen+152)
typedef void* (*pathBuilderInit_t)(void* outBuffer);
typedef void  (*pathBuilderFinalize_t)(void* lockAddr, void* initResult);
typedef void  (*readerUnlock_t)(void* lockAddr);
extern pathBuilderInit_t      fn_pathBuilderInit;
extern pathBuilderFinalize_t  fn_pathBuilderFinalize;
extern readerUnlock_t         fn_readerUnlock;

// NavMesh active cache: called functions (zone optimization -- all builds)
typedef void (*processJobAlt_t)(void* thisNavMeshGen, void* job);
typedef void (*buildCollision_t)(void* thisNavMeshGen, void* job);
typedef void (*partialFixup_t)(void* thisNavMeshGen, void* job);
typedef void (*enqueueToProcQueue_t)(void* queueAddr, void* job);
typedef void* (*navMeshCtor_t)(void* mem);
typedef void* (*settingsCtor_t)(void* mem);  // hkaiNavMeshGenerationSettings constructor
typedef void* (*gameNew_t)(size_t size);
typedef void  (*gameDelete_t)(void* ptr);
typedef void* (*gameNewArr_t)(size_t size);
typedef void  (*gameDelArr_t)(void* ptr);

extern navMeshCtor_t        fn_navMeshCtor;
extern settingsCtor_t       fn_settingsCtor;
extern processJobAlt_t      fn_processJobAlt;
extern buildCollision_t     fn_buildCollision;
extern partialFixup_t       fn_partialFixup;
extern enqueueToProcQueue_t fn_enqueueToProcQueue;
extern gameNew_t            fn_gameNew;
extern gameDelete_t         fn_gameDelete;
extern gameNewArr_t         fn_gameNewArr;
extern gameDelArr_t         fn_gameDelArr;

// Havok thread init: called functions (for worker thread TLS initialization)
// Step 3 (register) is a virtual call resolved at runtime, not stored as a global fn ptr.
typedef void  (*havokContextInit_t)(void* ctx128);
typedef void* (*havokGetManager_t)(int param);
typedef void  (*havokPostRegInit_t)(void* buf8, void* ctx128);
typedef void  (*havokCleanup_t)(void* buf8);
typedef void  (*havokCtxCleanup_t)(void* ctx128);

extern havokContextInit_t   fn_havokContextInit;
extern havokGetManager_t    fn_havokGetManager;
extern havokPostRegInit_t   fn_havokPostRegInit;
extern havokCleanup_t       fn_havokCleanup;
extern havokCtxCleanup_t    fn_havokCtxCleanup;

#ifndef ZONEOPT_ZONEONLY
// Group cohesion: CharMovement::setDestination
typedef void (*charMovSetDest_t)(void* thisCharMov, const float* dest, int priority, bool notVertical);
extern charMovSetDest_t fn_charMovSetDest;
#endif // !ZONEOPT_ZONEONLY

#if PATHFIND_STEP >= 1
// Path request queue enqueue
typedef void (*enqueuePathReq_t)(void* queueBase, void** itemPtr);
extern enqueuePathReq_t fn_enqueuePathReq;
#endif // PATHFIND_STEP >= 1


// =========================================================================
// Hook typedefs + original function pointers
// =========================================================================

typedef void (*showLoadingMessage_t)(void* thisPtr, bool on);
typedef bool (*isContentPending_t)(void* manager, void* zonePos);
typedef void (*updateCameraZone_t)(void* zoneMgr, void* cameraPos);
typedef void (*addOrderSelected_t)(void* thisPI, void* destIndoors, int task,
                                    void* subject, bool shift, bool addDontClear,
                                    const float* location);
typedef char (*dispatchJob_t)(void* thisNMG);

extern showLoadingMessage_t    orig_showLoadingMessage;
extern isContentPending_t      orig_isContentPending;
extern updateCameraZone_t      orig_updateCameraZone;
extern addOrderSelected_t      orig_addOrderSelected;
extern dispatchJob_t           orig_dispatchJob;

typedef void (*nmResultPopulate_t)(void* navData, void* localData, void* result, int param, int lowPart);
extern nmResultPopulate_t      orig_nmResultPopulate;

typedef void (*realGenerate_t)(void* workBuffer, void* localData, void* hkaiNavMesh, int param, int timeLowPart);
extern realGenerate_t          orig_realGenerate;

#if PATHFIND_STEP >= 1
typedef char (*csFindPath_t)(void* manager, unsigned int startFaceKey, void* startPos,
                              void* destPos, float radius, char param5, void* resultBuf);
typedef char (*csCheckFaceConn_t)(void* manager, unsigned int startFace, unsigned int destFace);
typedef void (*findPathFull_t)(void* streamingCollection, void* searchState, void* findPathOutput);
typedef void (*requestPath_t)(void* havokChar, float* destination, int priority);
typedef void (*pathReqSubmit_t)(void* sectionMgr, void* requestObj, bool highPriority);
typedef char (*csFindPathFallback_t)(void* manager, unsigned int startFaceKey, void* startPos,
                                      unsigned int destFaceKey, void* destPos, float radius,
                                      float param6, char param7, void* resultBuf);

extern csFindPath_t            orig_csFindPath;
extern csCheckFaceConn_t       orig_csCheckFaceConn;
extern findPathFull_t          orig_findPathFull;
extern requestPath_t           orig_requestPath;
extern pathReqSubmit_t         orig_pathReqSubmit;
extern csFindPathFallback_t    orig_csFindPathFallback;
#endif // PATHFIND_STEP >= 1


// =========================================================================
// Zone helpers (inline)
// =========================================================================

inline void* GetZoneEntry(void* zoneMgr, int x, int y)
{
	if (x < 0 || x > ZONE_GRID_MAX || y < 0 || y > ZONE_GRID_MAX)
		return NULL;
	return (void*)((uintptr_t)zoneMgr + OFF_ZM_ZONE_BASE + ZONE_ENTRY_SIZE * (y + x * 64));
}

inline int GetZoneState(void* zoneMgr)
{
	return *(int*)((uintptr_t)zoneMgr + OFF_ZM_STATE);
}

inline bool IsZoneLoading(void* zoneEntry)
{
	return *(unsigned char*)((uintptr_t)zoneEntry + OFF_ZONE_IS_LOADING) != 0;
}

inline bool IsZoneAccessible(void* zoneEntry)
{
	return *(unsigned char*)((uintptr_t)zoneEntry + OFF_ZONE_IS_ACCESS) != 0;
}

inline float GetZoneCenterX(void* zoneEntry)
{
	return *(float*)((uintptr_t)zoneEntry + OFF_ZONE_CENTER_X);
}

inline float GetZoneCenterZ(void* zoneEntry)
{
	return *(float*)((uintptr_t)zoneEntry + OFF_ZONE_CENTER_Z);
}

inline int GetZoneGridX(void* zoneEntry)
{
	return *(int*)((uintptr_t)zoneEntry + OFF_ZONE_COORDS_X);
}

inline int GetZoneGridY(void* zoneEntry)
{
	return *(int*)((uintptr_t)zoneEntry + OFF_ZONE_COORDS_Y);
}

inline float GetCharPosX(uintptr_t character)
{
	return *(float*)(character + OFF_CHAR_POS_X);
}

inline float GetCharPosZ(uintptr_t character)
{
	return *(float*)(character + OFF_CHAR_POS_Z);
}

inline unsigned int GetPlayerCharCount(uintptr_t playerIntf)
{
	return *(unsigned int*)(playerIntf + OFF_PI_CHAR_COUNT);
}

inline uintptr_t* GetPlayerCharStuff(uintptr_t playerIntf)
{
	return *(uintptr_t**)(playerIntf + OFF_PI_CHAR_STUFF);
}


// =========================================================================
// Game bindings initialization (resolves all function pointers)
// =========================================================================

void InitGameBindings(uintptr_t base);


#endif // KENSHI_ZONE_OPT_GAME_H
