#include "game.h"


// =========================================================================
// Game base + cached zone manager
// =========================================================================

uintptr_t gameBase = 0;
void* g_cachedZoneMgr = NULL;


// =========================================================================
// Function pointers (called, not hooked)
// =========================================================================

loadSingleZone_t        fn_loadSingleZone        = NULL;
notifyZoneReady_t       fn_notifyZoneReady       = NULL;
finalizeZoneResources_t fn_finalizeZoneResources  = NULL;
notifyAccessible_t      fn_notifyAccessible       = NULL;
flushPendingWork_t      fn_flushPendingWork       = NULL;
registerZoneSections_t  fn_registerZoneSections   = NULL;

resolveHandle_t fn_resolveHandle = NULL;
void*           g_handleTable    = NULL;

pathBuilderInit_t      fn_pathBuilderInit     = NULL;
pathBuilderFinalize_t  fn_pathBuilderFinalize = NULL;
readerUnlock_t         fn_readerUnlock        = NULL;
queueLockInit_t        fn_queueLockInit       = NULL;

navMeshCtor_t        fn_navMeshCtor         = NULL;
settingsCtor_t       fn_settingsCtor        = NULL;
processJobAlt_t      fn_processJobAlt       = NULL;
buildCollision_t     fn_buildCollision       = NULL;
partialFixup_t       fn_partialFixup         = NULL;
enqueueToProcQueue_t fn_enqueueToProcQueue   = NULL;
gameNew_t            fn_gameNew              = NULL;
gameDelete_t         fn_gameDelete           = NULL;
gameNewArr_t         fn_gameNewArr           = NULL;
gameDelArr_t         fn_gameDelArr           = NULL;

havokContextInit_t   fn_havokContextInit     = NULL;
havokGetManager_t    fn_havokGetManager      = NULL;
havokPostRegInit_t   fn_havokPostRegInit     = NULL;
havokCleanup_t       fn_havokCleanup         = NULL;
havokCtxCleanup_t    fn_havokCtxCleanup      = NULL;

#ifndef ZONEOPT_ZONEONLY
charMovSetDest_t     fn_charMovSetDest       = NULL;
#endif
lektorReserve_t      fn_lektorReserve        = NULL;
#if PATHFIND_STEP >= 1
enqueuePathReq_t     fn_enqueuePathReq       = NULL;
#endif
#if PATHFIND_STEP >= 6
faceToVertices_t     fn_faceToVertices       = NULL;
#endif


// =========================================================================
// Hook original function pointers
// =========================================================================

showLoadingMessage_t    orig_showLoadingMessage    = NULL;
isContentPending_t      orig_isContentPending      = NULL;
updateCameraZone_t      orig_updateCameraZone      = NULL;
addOrderSelected_t      orig_addOrderSelected      = NULL;
dispatchJob_t           orig_dispatchJob           = NULL;
nmResultPopulate_t      orig_nmResultPopulate      = NULL;
realGenerate_t          orig_realGenerate          = NULL;
isInIsland_t            orig_isInIsland            = NULL;
getIsland_t             orig_getIsland             = NULL;
#if PATHFIND_STEP >= 1
csFindPath_t            orig_csFindPath            = NULL;
csCheckFaceConn_t       orig_csCheckFaceConn       = NULL;
findPathFull_t          orig_findPathFull          = NULL;
requestPath_t           orig_requestPath           = NULL;
pathReqSubmit_t         orig_pathReqSubmit         = NULL;
csFindPathFallback_t    orig_csFindPathFallback    = NULL;
#endif
#if PATHFIND_STEP >= 9
contentStreamCallee0x8869_t orig_contentStreamCallee0x8869 = NULL;
addInstance_t               orig_addInstance               = NULL;
#endif


// =========================================================================
// InitGameBindings — resolve all function pointers from gameBase
// =========================================================================

void InitGameBindings(uintptr_t base)
{
	gameBase = base;

	// Set up HavokTlsAlloc (core.h) without core depending on game.h
	SetHavokTlsParams(base, RVA_HAVOK_TLS_INDEX);

	// Zone loading
	fn_loadSingleZone        = (loadSingleZone_t)       GameAddr(RVA_LOAD_SINGLE_ZONE);
	fn_notifyZoneReady       = (notifyZoneReady_t)       GameAddr(RVA_NOTIFY_ZONE_READY);
	fn_finalizeZoneResources = (finalizeZoneResources_t) GameAddr(RVA_FINALIZE_ZONE_RES);
	fn_notifyAccessible      = (notifyAccessible_t)      GameAddr(RVA_NOTIFY_ACCESSIBLE);
	fn_flushPendingWork      = (flushPendingWork_t)      GameAddr(RVA_FLUSH_PENDING_WORK);
	fn_registerZoneSections  = (registerZoneSections_t)  GameAddr(RVA_REGISTER_ZONE_SECTIONS);

	// Handle resolution
	fn_resolveHandle         = (resolveHandle_t)         GameAddr(RVA_RESOLVE_HANDLE);
	g_handleTable            = (void*)(base + RVA_HANDLE_TABLE);

	// NavMesh queue locks
	fn_pathBuilderInit       = (pathBuilderInit_t)      GameAddr(RVA_PATH_BUILDER_INIT);
	fn_pathBuilderFinalize   = (pathBuilderFinalize_t)  GameAddr(RVA_PATH_BUILDER_FINALIZE);
	fn_readerUnlock          = (readerUnlock_t)         GameAddr(RVA_READER_UNLOCK);
	fn_queueLockInit         = (queueLockInit_t)        GameAddr(RVA_QUEUE_LOCK_INIT);

	// NavMesh active cache
	fn_navMeshCtor           = (navMeshCtor_t)           GameAddr(RVA_NAVMESH_CTOR);
	fn_settingsCtor          = (settingsCtor_t)          GameAddr(RVA_SETTINGS_CTOR);
	fn_processJobAlt         = (processJobAlt_t)        GameAddr(RVA_PROCESS_JOB_ALT);
	fn_buildCollision        = (buildCollision_t)        GameAddr(RVA_BUILD_COLLISION);
	fn_partialFixup          = (partialFixup_t)          GameAddr(RVA_PARTIAL_FIXUP);
	fn_enqueueToProcQueue    = (enqueueToProcQueue_t)    GameAddr(RVA_ENQUEUE_TO_PROC_QUEUE);
	fn_gameNew               = (gameNew_t)               GameAddr(RVA_GAME_NEW);
	fn_gameDelete            = (gameDelete_t)            GameAddr(RVA_GAME_DELETE);
	fn_gameNewArr            = (gameNewArr_t)            GameAddr(RVA_GAME_NEW_ARR);
	fn_gameDelArr            = (gameDelArr_t)            GameAddr(RVA_GAME_DEL_ARR);

	// Havok thread init
	fn_havokContextInit      = (havokContextInit_t)      GameAddr(RVA_HAVOK_CONTEXT_INIT);
	fn_havokGetManager       = (havokGetManager_t)       GameAddr(RVA_HAVOK_GET_MANAGER);
	fn_havokPostRegInit      = (havokPostRegInit_t)      GameAddr(RVA_HAVOK_POST_REG_INIT);
	fn_havokCleanup          = (havokCleanup_t)          GameAddr(RVA_HAVOK_CLEANUP);
	fn_havokCtxCleanup       = (havokCtxCleanup_t)       GameAddr(RVA_HAVOK_CTX_CLEANUP);

	// Island routing
	fn_lektorReserve         = (lektorReserve_t)         GameAddr(RVA_LEKTOR_RESERVE);

#ifndef ZONEOPT_ZONEONLY
	fn_charMovSetDest        = (charMovSetDest_t)        GameAddr(RVA_CHARMOV_SET_DEST);
	fn_enqueuePathReq        = (enqueuePathReq_t)        GameAddr(RVA_ENQUEUE_PATH_REQ);
#elif PATHFIND_STEP >= 1
	fn_enqueuePathReq        = (enqueuePathReq_t)        GameAddr(RVA_ENQUEUE_PATH_REQ);
#endif

#if PATHFIND_STEP >= 6
	fn_faceToVertices        = (faceToVertices_t)        GameAddr(RVA_FACE_TO_VERTICES);
#endif
}
