#include "game.h"
#include "config.h"   // step gates: the table mirrors what each build installs
#include <cstdio>     // _snprintf_s (VerifyPrologueByRva's allocation-free failure path)


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
setQueuesAreClear_t     fn_setQueuesAreClear      = NULL;

resolveHandle_t fn_resolveHandle = NULL;
void*           g_handleTable    = NULL;

pathBuilderInit_t      fn_pathBuilderInit     = NULL;
pathBuilderFinalize_t  fn_pathBuilderFinalize = NULL;
readerUnlock_t         fn_readerUnlock        = NULL;
queueLockInit_t        fn_queueLockInit       = NULL;

navMeshCtor_t        fn_navMeshCtor         = NULL;
settingsCtor_t       fn_settingsCtor        = NULL;
simplSettingsCopy_t  fn_simplSettingsCopy   = NULL;
settingsDtorBody_t   fn_settingsDtorBody    = NULL;
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

// Round 2 Z: Phase 18 Step 1 instrumentation (called, not hooked; game.h).
isZoneReady_t        fn_isZoneReady          = NULL;

// Round 2 fix 2b: the reset's own per-zone unload (called, not hooked; game.h).
unloadZoneFromReset_t fn_unloadZoneFromReset = NULL;


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
resetUnloadZones_t      orig_resetUnloadZones      = NULL;   // Round 2 fix 2b
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

// Round 2 P: Phase 17 Step 1 instrumentation hooks + isPriorityPath (game.h
// "Round 2 P" block). Storage only; the hooks' orig_ pointers are filled by
// KenshiLib::AddHook in main.cpp, not here.
#if PATHPOOL_STEP >= 1
contentStream_t      orig_contentStream      = NULL;
dequeueWork_t        orig_dequeueWork        = NULL;
enqueueThreadSafe_t  orig_enqueueThreadSafe  = NULL;
gatesUpdateCodes_t   orig_gatesUpdateCodes   = NULL;
isPriorityPath_t     fn_isPriorityPath       = NULL;
#endif


// =========================================================================
// InitGameBindings — resolve all function pointers from gameBase
// =========================================================================

void InitGameBindings(uintptr_t base)
{
	gameBase = base;

	// Set up HavokTlsAlloc (core.h) without core depending on game.h
	SetHavokTlsParams(base, RVA_HAVOK_TLS_INDEX);

	// Same arrangement for the destroyListOE probe: core gets the resolved
	// GameWorld address and keeps no game knowledge of its own.
	SetDestroyListBase(base + RVA_GLOBAL_GAMEWORLD);

	// Zone loading
	fn_loadSingleZone        = (loadSingleZone_t)       GameAddr(RVA_LOAD_SINGLE_ZONE);
	fn_notifyZoneReady       = (notifyZoneReady_t)       GameAddr(RVA_NOTIFY_ZONE_READY);
	fn_finalizeZoneResources = (finalizeZoneResources_t) GameAddr(RVA_FINALIZE_ZONE_RES);
	fn_notifyAccessible      = (notifyAccessible_t)      GameAddr(RVA_NOTIFY_ACCESSIBLE);
	fn_flushPendingWork      = (flushPendingWork_t)      GameAddr(RVA_FLUSH_PENDING_WORK);
	fn_registerZoneSections  = (registerZoneSections_t)  GameAddr(RVA_REGISTER_ZONE_SECTIONS);
	fn_setQueuesAreClear     = (setQueuesAreClear_t)     GameAddr(RVA_SET_QUEUES_CLEAR);

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
	fn_simplSettingsCopy     = (simplSettingsCopy_t)     GameAddr(RVA_SIMPL_SETTINGS_COPY);
	fn_settingsDtorBody      = (settingsDtorBody_t)      GameAddr(RVA_SETTINGS_DTOR_BODY);
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

	// ---- Round 2 P additions (only task P edits this block) ----
#if PATHPOOL_STEP >= 1
	fn_isPriorityPath        = (isPriorityPath_t)        GameAddr(RVA_IS_PRIORITY_PATH);
#endif
	// ---- end Round 2 P ----

	// ---- Round 2 H1 additions (only task H1 edits this block) ----
	// ---- end Round 2 H1 ----

	// ---- Round 2 H2 additions (only task H2 edits this block) ----
	fn_lookupSection         = (lookupSection_t)         GameAddr(RVA_LOOKUP_SECTION);
	fn_boostUnlock           = (boostUnlock_t)           GameAddr(RVA_BOOST_UNLOCK);
	fn_boostUnlockShared     = (boostUnlockShared_t)     GameAddr(RVA_BOOST_UNLOCK_SHARED);
	// ---- end Round 2 H2 ----

	// ---- Round 2 F additions (only task F edits this block) ----
	// ---- end Round 2 F ----

	// ---- Round 2 Z additions (only task Z edits this block) ----
	fn_isZoneReady           = (isZoneReady_t)           GameAddr(RVA_ZONE_IS_READY);
	// ---- end Round 2 Z ----

	// ---- Round 2 N additions (only task N edits this block) ----
	// ---- end Round 2 N ----

	// ---- Round 2 fix 2b additions (only fix 2b edits this block) ----
	fn_unloadZoneFromReset   = (unloadZoneFromReset_t)   GameAddr(RVA_UNLOAD_ZONE_FROM_RESET);
	// ---- end Round 2 fix 2b ----
}


// =========================================================================
// Build gate: hook and patch-site prologues (B1)
// =========================================================================
//
// One row per site this build can write to: every KenshiLib::AddHook in
// main.cpp, the lazy installs in nm_workers.cpp, and the scatter byte-patch in
// formation.cpp (which shares addOrderSelectedCharacters' row with the hook on
// the same function). startPlugin verifies the whole table before installing
// anything; the lazy sites verify their own row again at install time.
//
// STANDING RULE: every new AddHook or byte-patch site adds a row here and a
// VerifyPrologue call at the site. A site with no row is a site that installs
// blind on an unknown build.
//
// Every byte was read from the Steam 1.0.65 IDB (mcp__ida__read_data, 16 bytes
// at the RVA) on 2026-09-12 — never from a KenshiLib header comment, which
// carries a different build's addresses. The same bytes are the "First 16
// bytes" column of docs/kenshilib_requests.md §2.
//
// The rows are compile-gated exactly like their install sites, so the count in
// the "Build gate: ok (N sites)" line is the number of sites this build can
// actually touch.

// `extern` is required: a const object at namespace scope has internal
// linkage in C++ without it, and main.cpp needs this one.
extern const HookPrologue g_hookPrologues[] =
{
	// --- main.cpp, unconditional ---
	{ "showLoadingMessage",     RVA_SHOW_LOADING_MESSAGE,
	  { 0x40,0x53,0x56,0x57,0x48,0x83,0xEC,0x30,0x48,0xC7,0x44,0x24,0x20,0xFE,0xFF,0xFF } },
	{ "isContentPending",       RVA_IS_CONTENT_PENDING,
	  { 0x40,0x55,0x56,0x57,0x41,0x54,0x41,0x55,0x48,0x81,0xEC,0xB0,0x00,0x00,0x00,0x48 } },
	{ "updateCameraZone",       RVA_UPDATE_CAMERA_ZONE,
	  { 0x48,0x8B,0xC4,0x48,0x89,0x50,0x10,0x48,0x89,0x48,0x08,0x55,0x56,0x57,0x41,0x54 } },
	// Hook target and, in FULL builds, the scatter patch's scan base.
	{ "addOrderSelected",       RVA_ADD_ORDER_SELECTED,
	  { 0x48,0x8B,0xC4,0x48,0x89,0x48,0x08,0x55,0x41,0x56,0x48,0x8D,0x68,0x98,0x48,0x81 } },
	// destroyListOE inserter. The row is unconditional because the install is
	// gated by INI keys, not by a compile step: destroyListDefer (default on) or
	// destroyListDiag can want it in any build, so the gate has to be able to
	// verify it in any build. It is NOT marked diagnostic any more: the hook now
	// carries the crash-3 mitigation, and a build that silently ran without the
	// mitigation would produce exactly the crash the gate exists to prevent us
	// from causing. A mismatch here is fatal like every other non-diagnostic row.
	// Note the cost of that choice: the gate loop checks every row regardless of
	// whether the hook will be installed, so this row is fatal even in a session
	// that has both destroyListDefer and destroyListDiag set false and would
	// never have touched this site at all.
	// On an unrecognised binary that is the intended answer — refuse to run —
	// but it does mean the INI cannot be used to work around a mismatch here.
	{ "destroyListInsert",      RVA_DESTROYLIST_INSERT,
	  { 0x48,0x89,0x54,0x24,0x10,0x57,0x48,0x83,0xEC,0x40,0x48,0xC7,0x44,0x24,0x20,0xFE } },
	// Round 2 fix 2b: the save-load reset's Set A/B unload, sub_14036C1E0. The
	// hook carries the fix for the Round 2 save-load crash (it unloads the mod's
	// zones the reset would leave alive with a wiped registry), so the row is
	// fatal like destroyListInsert's: a build that ran without it would bring
	// that crash back. Unconditional, like its install (the INI key
	// saveLoadUnload only decides whether the hook unloads or just counts).
	// `mov r11,rsp; mov [r11+8],rcx; push rbp/rsi/rdi/r12/r13/r14`.
	{ "resetUnloadZones",       RVA_RESET_UNLOAD_ZONES,
	  { 0x4C,0x8B,0xDC,0x49,0x89,0x4B,0x08,0x55,0x56,0x57,0x41,0x54,0x41,0x55,0x41,0x56 } },

#if ISLAND_STEP >= 1
	// --- main.cpp, Phase 15 island overlay ---
	{ "isInIsland",             RVA_ISINISLAND_IMPL,
	  { 0x48,0x85,0xD2,0x74,0x0B,0x8B,0x42,0x20,0x39,0x41,0x20,0x75,0x03,0xB0,0x01,0xC3 } },
	{ "getIsland",              RVA_GETISLAND_IMPL,
	  { 0x48,0x89,0x5C,0x24,0x18,0x48,0x89,0x6C,0x24,0x20,0x57,0x48,0x83,0xEC,0x20,0x48 } },
#endif

#if NMCACHE_STEP >= 1
	// --- main.cpp, navmesh cache ---
	{ "dispatchJob",            RVA_DISPATCH_JOB,
	  { 0x48,0x89,0x6C,0x24,0x20,0x56,0x48,0x83,0xEC,0x30,0x48,0x8D,0xA9,0x88,0x00,0x00 } },
#endif

#if NMCACHE_STEP >= 3
	// --- nm_workers.cpp, installed on the first dispatch ---
	{ "edgeProcess",            RVA_EDGE_PROCESS,
	  { 0x40,0x53,0x48,0x83,0xEC,0x20,0x48,0x8B,0xD9,0x48,0x83,0xC1,0x50,0xE8,0x1F,0xF0 } },
#if NMFIX_STEP >= 1
	{ "nmResultPopulate",       RVA_NM_RESULT_POPULATE,
	  { 0x48,0x83,0xEC,0x68,0x4C,0x8B,0xD1,0x48,0x8D,0x4C,0x24,0x30,0xE8,0x1F,0xAF,0xFE } },
#endif
#if NMFIX_STEP >= 5
	{ "processJobAlt",          RVA_PROCESS_JOB_ALT,
	  { 0x48,0x8B,0xC4,0x55,0x56,0x57,0x41,0x54,0x41,0x55,0x41,0x56,0x41,0x57,0x48,0x8D } },
#endif
#if NMFIX_STEP >= 6
	// The two builders share their first 16 bytes; only the bodies differ.
	{ "buildCollision",         RVA_BUILD_COLLISION_IMPL,
	  { 0x40,0x55,0x56,0x57,0x41,0x54,0x41,0x55,0x41,0x56,0x41,0x57,0x48,0x8D,0x6C,0x24 } },
	{ "buildCollisionInterior", RVA_BUILD_COLLISION_INTERIOR_IMPL,
	  { 0x40,0x55,0x56,0x57,0x41,0x54,0x41,0x55,0x41,0x56,0x41,0x57,0x48,0x8D,0x6C,0x24 } },
#endif
#if NMFIX_STEP >= 8
	// `test rcx, rcx` + `jz` — the rel8 branch sits inside the 5 bytes MinHook
	// takes, so its trampoline relocates the branch. The gate only compares
	// bytes, so that changes nothing here, but it is the one site where the
	// trampoline does more than copy.
	{ "navMeshStop",            RVA_NAVMESH_STOP,
	  { 0x48,0x85,0xC9,0x74,0x46,0x53,0x48,0x83,0xEC,0x20,0x48,0x8B,0xD9,0xC6,0x81,0xC8 } },
#endif
#endif // NMCACHE_STEP >= 3

#if PATHFIND_STEP >= 1
	// --- main.cpp, pathfinding ---
	{ "csFindPath",             RVA_CS_FIND_PATH,
	  { 0x40,0x55,0x53,0x56,0x57,0x41,0x54,0x41,0x55,0x48,0x8D,0x6C,0x24,0x98,0x48,0x81 } },
	{ "csCheckFaceConn",        RVA_CS_CHECK_FACE_CONN,
	  { 0x48,0x8B,0xC4,0x55,0x48,0x8D,0x68,0xA1,0x48,0x81,0xEC,0xF0,0x00,0x00,0x00,0x48 } },
	{ "findPathFull",           RVA_FIND_PATH_FULL,
	  { 0x48,0x89,0x4C,0x24,0x08,0x55,0x56,0x41,0x54,0x41,0x55,0x41,0x57,0x48,0x8D,0xAC } },
	{ "requestPath",            RVA_REQUEST_PATH,
	  { 0x48,0x89,0x5C,0x24,0x08,0x48,0x89,0x6C,0x24,0x10,0x48,0x89,0x74,0x24,0x18,0x57 } },
#endif
#if PATHFIND_STEP >= 2
	{ "pathReqSubmit",          RVA_PATH_REQ_SUBMIT,
	  { 0x48,0x89,0x54,0x24,0x10,0x55,0x56,0x57,0x48,0x83,0xEC,0x30,0x48,0xC7,0x44,0x24 } },
#endif
#if PATHFIND_STEP >= 4
	{ "csFindPathFallback",     RVA_CS_FIND_PATH_FALLBACK,
	  { 0x40,0x55,0x53,0x56,0x57,0x41,0x54,0x41,0x55,0x41,0x56,0x48,0x8D,0xAC,0x24,0x60 } },
#endif
#if PATHFIND_STEP >= 9
	{ "contentStreamCallee_0x8869", RVA_CONTENT_STREAM_CALLEE_0X8869,
	  { 0x48,0x8B,0xC4,0x53,0x55,0x57,0x41,0x56,0x41,0x57,0x48,0x83,0xEC,0x70,0x0F,0x57 } },
	{ "addInstance",            RVA_ADD_INSTANCE,
	  { 0x48,0x89,0x5C,0x24,0x08,0x48,0x89,0x6C,0x24,0x10,0x48,0x89,0x74,0x24,0x18,0x57 } },
#endif

#if PATHPOOL_STEP >= 1
	// --- main.cpp, Phase 17 Step 1 instrumentation (P) ---
	// Bytes read from the IDB (ida MCP disassemble) 2026-09-12; see
	// docs/kenshilib_requests.md §2 for the same rows.
	// diagnostic = true (round-1 review fix): these four are the PathPool
	// instrumentation only, not a behaviour-affecting hook. A mismatch here
	// must turn PathPool off, not refuse the whole plugin -- each install
	// site (main.cpp) re-checks its own row via VerifyPrologueByRva anyway,
	// so this row only decides whether a mismatch is fatal to the plugin.
	{ "contentStream",          RVA_CONTENT_STREAM,
	  { 0x48,0x8B,0xC4,0x55,0x41,0x54,0x41,0x55,0x41,0x56,0x41,0x57,0x48,0x8D,0x68,0xB8 }, true },
	{ "dequeueWork",            RVA_DEQUEUE_WORK,
	  { 0x48,0x89,0x5C,0x24,0x10,0x48,0x89,0x74,0x24,0x18,0x57,0x48,0x83,0xEC,0x20,0x48 }, true },
	{ "enqueueThreadSafe",      RVA_ENQUEUE_THREAD_SAFE,
	  { 0x48,0x89,0x5C,0x24,0x08,0x57,0x48,0x83,0xEC,0x20,0x48,0x8B,0xD9,0xB9,0x10,0x00 }, true },
	{ "gatesUpdateCodes",       RVA_GATES_UPDATE_CODES,
	  { 0x48,0x8B,0xC4,0x55,0x41,0x54,0x41,0x55,0x41,0x56,0x41,0x57,0x48,0x8D,0xA8,0xA8 }, true },
#endif
};

extern const int g_hookPrologueCount = (int)(sizeof(g_hookPrologues) / sizeof(g_hookPrologues[0]));

bool VerifyPrologueByRva(uintptr_t rva)
{
	for (int i = 0; i < g_hookPrologueCount; ++i)
	{
		if (g_hookPrologues[i].rva == rva)
			return VerifyPrologue(rva, g_hookPrologues[i].bytes, g_hookPrologues[i].name);
	}

	// No row: the standing rule above was not followed. Refuse the install
	// rather than write a jump into an unverified address. This can run on the
	// NavMesh bg thread (InstallNavMeshLazyHooks calls VerifyPrologueByRva from
	// inside hook_dispatchJob on its first call), so build the line with a
	// fixed buffer + _snprintf_s instead of std::ostringstream (Things That
	// DON'T Work: CRT on non-CRT thread) before handing it to LogMsg -- the
	// same off-main logging path every other bg-thread LogMsg call in this
	// codebase already uses (e.g. nm_workers.cpp's lazy hook install lines).
	// Message content unchanged from before.
	char buf[128];
	_snprintf_s(buf, sizeof(buf), _TRUNCATE,
		"[ZoneOpt] Build gate: no prologue row for RVA 0x%llx \xE2\x80\x94 install refused",
		(unsigned long long)rva);
	LogMsg(buf);
	return false;
}
