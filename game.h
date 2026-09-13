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
// PhysicsInterface::setQueuesAreClear, 0x579460 (80 bytes): takes queuesClearMuto
// (+0x328) and writes +0x320. KenshiLib declares it, but its header RVA (0x579770)
// is from a different build, so bind this build's address directly.
const size_t RVA_SET_QUEUES_CLEAR    = 0x579460;

// Handle resolution (called from hook to iterate selected characters)
const size_t RVA_RESOLVE_HANDLE      = 0x3E419;   // thunk to handle->object resolution
const size_t RVA_HANDLE_TABLE        = 0x2132F38; // global handle lookup table (qword_142132F38)
const size_t RVA_HANDLE_SENTINEL     = 0x2132F10; // stale handle sentinel (qword_142132F10)

// NavMesh queue lock functions (thunk RVAs)
const size_t RVA_PATH_BUILDER_INIT     = 0x3D136;
const size_t RVA_PATH_BUILDER_FINALIZE = 0x4F8AE;
const size_t RVA_READER_UNLOCK         = 0x2E843;

// NavMeshGen__queueLockAcquire implementation (CS init despite the name, 401 bytes).
// Called 5x in NavMeshGen_struct_ctor to initialize the 5 CSes at +72/+104/+152/+200/+272.
// Needed at Stage 4c to reinit CSes in cloned NMG (memcpy'd CS state is undefined).
const size_t RVA_QUEUE_LOCK_INIT       = 0x25F350;

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

// Offsets into the input geometry that processJobAlt builds and passes as
// NavMeshResult__populate's second argument. Three hkArray slots; the first
// (+0) is unused by the generator. Each hkArray is {data(8), size(4), capFlags(4)}.
//   vertices: data +16, count +24   (setup 0x3C1580 writes v4[6], reserves v4+4, stride 16)
//   triangles: data +32, count +40  (setup writes v4[10], reserves v4+8, stride 16)
// realGenerate (0xDFC060) tests `cmp dword ptr [r12+28h], 0` and, when zero,
// asserts "Passed in empty triMesh to generateNavMesh" and returns without
// generating — so count 0 here is exactly "empty input".
const int OFF_GEOM_VERTEX_COUNT   = 24;
const int OFF_GEOM_TRIANGLE_COUNT = 40;
// HavokNavMesh::realGenerate -- the actual heavy compute
const size_t RVA_REAL_GENERATE         = 0xDFC060;   // 61,317 bytes

// Havok thread init sequence (for worker thread TLS initialization)
const size_t RVA_HAVOK_CONTEXT_INIT    = 0xBA4770;
const size_t RVA_HAVOK_GET_MANAGER     = 0xBAF970;
const size_t RVA_HAVOK_POST_REG_INIT   = 0xBAECF0;
const size_t RVA_HAVOK_CLEANUP         = 0xBAED40;
const size_t RVA_HAVOK_CTX_CLEANUP     = 0xBA9580;

// =========================================================================
// Structure layouts (offsets, not addresses)
// =========================================================================

// hkaiNavMeshGenerationSettings +520 override entries (SortedArray, 240 B each).
// Built by NavMeshGenerator__initWorkBuffer (0x3C48D0) from a staging entry that
// OverrideSettings__initFromSettings (0xDD9280) fills.
const int OVR_OFF_VOLUME   = 0;    // hkRefPtr; NULL on all four base entries
const int OVR_OFF_MATERIAL = 8;    // int; -1 on the staging copy, 1..4 on the base entries
const int OVR_OFF_FLAG     = 12;   // byte, from settings+332
const int OVR_OFF_SLOPE    = 16;   // float walkable slope, radians
const int OVR_OFF_EMP      = 20;   // 56 bytes copied from settings+76
const int OVR_OFF_SIMPL    = 80;   // SimplificationSettings sub-object (160 bytes)


// =========================================================================
// RVAs, continued
// =========================================================================

// Collision builders, serialized under buildCollisionCS at NMFIX_STEP >= 6.
// These are the implementations, NOT the thunks: RVA_BUILD_COLLISION above is
// the thunk fn_buildCollision calls, and hooking the implementation catches
// both that path and dispatchJob_orig's own calls. Both are reached only from
// dispatchJob_orig (0x3CE030), and both reach the game's build mutex
// (0x212DEB8) through buildSectionCollision (0x3CB140).
const size_t RVA_BUILD_COLLISION_IMPL          = 0x3CB700;  // 925 bytes
const size_t RVA_BUILD_COLLISION_INTERIOR_IMPL = 0x3CBAB0;  // 938 bytes

// Clone finalize guards (prevent operations on uninitialized OverrideSettings entries)
// hkaiNavMeshGenerationSettings destructor BODY. NOT the deleting destructor
// 0xDDABE0, which frees the object through the class allocator; ours came from
// HavokTlsAlloc and is freed by HavokTlsFree. Not 0xDD92F0 either, which is the
// unrelated per-entry OverrideSettings destructor.
const size_t RVA_SETTINGS_DTOR_BODY  = 0xDD9BC0;  // 643 bytes

// NavMesh::stop. Clears +0x1C8, joins the path thread, deletes the manager and
// shuts the Havok memory system down — so the worker pool has to be retired
// before it runs. Prologue is `test rcx,rcx` + short `jz`, i.e. the first 5
// bytes contain a relative branch.
const size_t RVA_NAVMESH_STOP        = 0x3AAE90;

const size_t RVA_SIMPL_SETTINGS_DTOR = 0x3DA120;  // 119 bytes, SimplificationSettings::dtor
const size_t RVA_SIMPL_SETTINGS_COPY = 0x3DA000;  // 224 bytes, SimplificationSettings::copy
                                                  // (scalars + ExtraVertexSettings + an
                                                  //  hkStringPtr, so never a plain memcpy)

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

// Island routing (Phase 15). Verified against the IDB (kenshi_x64.exe 1.0.65).
//   ZoneMap::isInIsland      0xA07EB0 (19 bytes): `b && a->island == b->island`.
//                            Sole caller: CharMovement::setDestination (0x6607E0)
//                            via thunk 0x2FCE8 at 0x660B70. We hook the impl —
//                            the thunk's only path leads there.
//   ZoneManager::getIsland   0xA09AE0 (169 bytes): appends every Set B zone whose
//                            label equals t->island to a lektor<ZoneMap*>.
//                            Sole caller: ZoneMap::getActiveZoneIsland (0xA09B90),
//                            used by computeProjectedDest (0x3A39C0) and the
//                            smell picker (0x8F4A10).
//   lektor reserve           thunk 0x16630 -> 0x37E3A0 (grow to n, 0 => 10).
//   _calculateIslands        0xA09520: Set B only, on updateRendertimeThread.
//   isZoneStillLoading       0x3AC810: `zone && isContentPending(mgr, zone+24)`.
const size_t RVA_ISINISLAND_IMPL      = 0xA07EB0;
const size_t RVA_ISINISLAND_THUNK     = 0x2FCE8;   // documented; not hooked
const size_t RVA_GETISLAND_IMPL       = 0xA09AE0;
const size_t RVA_LEKTOR_RESERVE       = 0x16630;   // thunk -> 0x37E3A0
const size_t RVA_CALCULATE_ISLANDS    = 0xA09520;  // documented; never called/hooked
const size_t RVA_COMPUTE_PROJECTED_DEST = 0x3A39C0; // documented; emulated in islands.cpp
const size_t RVA_IS_ZONE_STILL_LOADING  = 0x3AC810; // documented

// Global data RVAs
const size_t RVA_GLOBAL_SECTION_MGR  = 0x2133560;  // pauseState.navmesh (SectionManager*)
// pauseState.physics (PhysicsInterface*, i.e. ou->physics). Fields we touch:
//   +0x1B0/+0x200/+0x2A8/+0x2F8  work-queue counts read by isReadyForSections
//   +0x320 (800)  bool _queuesClear  -- the ready flag loadSingleZone clears
//   +0x328 (808)  boost::shared_mutex queuesClearMuto, guards +0x320.
// Write +0x320 only through fn_setQueuesAreClear (it takes that lock).
const size_t RVA_PAUSESTATE_PHYSICS  = 0x21330C8;
const size_t RVA_GLOBAL_PLAYER       = 0x2133630;
// The GameWorld singleton itself, the base the two RVAs above sit inside. The
// IDB names it `pauseState` (2248 bytes, .data, 513 references). Confirmed as
// the crash-3 fault object: the faulting frame's RSI was 0x7FF7424230B0, i.e.
// kenshi_x64 + 0x21330B0. Read-only, by the destroyListOE probe in core.cpp.
const size_t RVA_GLOBAL_GAMEWORLD    = 0x21330B0;

// GameWorld::destroyListOE's sole inserter, sub_140799BE0 (197 bytes): it hides
// the MovableObject, then inserts it into the set at GameWorld+0x680. Hooked
// pass-through, for its caller's thread id only (crash-3 diagnostic).
const size_t RVA_DESTROYLIST_INSERT  = 0x799BE0;


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
// Island routing reads (from setDestination 0x6607E0 / computeProjectedDest 0x3A39C0)
const size_t OFF_CMOV_POS            = 0xC4;   // Vector3 position used for zone lookup
const size_t OFF_CMOV_LAST_DEST      = 0xDC;   // last requested destination (edge-mode compare, 2 units)
const size_t OFF_CMOV_PATH_DEST      = 0xE8;   // pathDestination handed to HavokCharacter::requestPath
const size_t OFF_CMOV_EDGE_COUNTER   = 0x368;  // edge-target retry counter (0..17)
const size_t OFF_CMOV_MOVING_TO_EDGE = 0x370;  // BYTE: 1 while routing to an island edge

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
// Island routing (ZoneMap)
const size_t OFF_ZONE_ISLAND      = 0x20;   // int label written by _calculateIslands (0 = unlabelled)
const size_t OFF_ZONE_AABB_CENTER = 0xD0;   // Ogre::Aabb mCenter (3 floats)
const size_t OFF_ZONE_AABB_HALF   = 0xDC;   // Ogre::Aabb mHalfSize (3 floats); min = center - half
const size_t OFF_ZONE_NEIGHBORS   = 0x128;  // ZoneMap* neighbors[4] (static grid adjacency, NULL at edges)
const int    ZONE_NEIGHBOR_COUNT  = 4;

// ZoneManager
const size_t OFF_ZM_ZONE_BASE    = 200;
const size_t OFF_ZM_LOADING      = 8;        // BYTE: SaveManager::loadGame sets 1; cleared with state 5
const size_t OFF_ZM_CURRENT_ZONE = 1475024;
const size_t OFF_ZM_STATE        = 1475032;
const int    ZONE_GRID_MAX       = 63;
const int    ZONE_GRID_COUNT     = 64 * 64;

// Set B (boost::unordered_set<ZoneMap*>) at ZoneManager+1474824.
// List head = *(buckets + 8*bucketCount); node: next at +0, value at +16.
const size_t OFF_ZM_SET_B          = 1474824;
const size_t OFF_SET_BUCKET_COUNT  = 24;
const size_t OFF_SET_SIZE          = 32;
const size_t OFF_SET_BUCKETS       = 56;
const size_t OFF_SET_NODE_VALUE    = 16;

// SectionManager (pauseState.navmesh): zone extent used by computeProjectedDest
const size_t OFF_NAVMESH_ZONE_SIZE = 468;

// lektor<T*> (game dynamic array): +8 count, +12 capacity, +16 data
const size_t OFF_LEKTOR_COUNT    = 8;
const size_t OFF_LEKTOR_CAPACITY = 12;
const size_t OFF_LEKTOR_DATA     = 16;

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
const size_t OFF_SC_INSTANCES_COUNT = 40;

// PathRequest layout: dispatcher passes &requestObj[+128] as resultBuf to csFindPath chain.
// Verified from SectionManager::contentStream (0x3AE350) decompilation + PathRequest__ctor.
// Round 2 final review Important #2: moved out of the PATHFIND_STEP >= 6 block --
// hook_csFindPath installs at step >= 1 (main.cpp) and the game passes req+128
// as resultBuf at every step, so playerByReq must be recoverable that early
// too (below step 6 every boosted sample was "unk" and disagree= was 0 by
// construction). The reqCharMap/ExitFace consumers of this constant stay
// gated at PATHFIND_STEP >= 6 in pathfind_hooks.cpp; only the constant moved.
const size_t OFF_REQ_RESULTBUF_SLOT = 128;
#endif

#if PATHFIND_STEP >= 6
// hkaiStreamingCollection extended layout (STEP 6 ExitFace decode)
const size_t OFF_SC_INSTANCES_BASE  = 32;   // ptr to instances[] array
const size_t INSTANCEINFO_SIZE      = 48;   // bytes per InstanceInfo entry; first qword = m_instancePtr

// sub_140033320: resolves (instance, faceIdx) -> two edge endpoint vertices (__m128).
// Same function Havok__findPathFull (line 528-534) and ContentStream__resolveDestFace
// (line 276-287) call. 5th arg is a 64-bit scalar passed by value (uninitialized stack
// qword in all observed call sites -- pass 0).
const size_t RVA_FACE_TO_VERTICES   = 0x33320;
#endif

#if PATHFIND_STEP >= 9
// Extraction race mitigation: SEH wrap around Havok::contentStreamCallee_0x8869
// (the path-result-extraction loop). 526 bytes at impl. Called from findPath +
// findPathFallback after A* success; walks m_visitedEdges and fills Kenshi
// result buffer. Derefs m_instances[sec].m_instancePtr without NULL check,
// races with addInstance -> AV when section slot is mid-mutation.
const size_t RVA_CONTENT_STREAM_CALLEE_0X8869 = 0x3A9F20;

// Cache stability gate: timestamp addInstance calls so our cache injection can
// skip when streaming state is actively mutating. 821 bytes at impl.
const size_t RVA_ADD_INSTANCE = 0xD0D8F0;
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
// Build gate table (B1) — defined in game.cpp
// =========================================================================

extern const HookPrologue g_hookPrologues[];
extern const int          g_hookPrologueCount;

// Verify the row belonging to one RVA. Used by the lazy install sites, which
// run long after startPlugin's full pass. Returns false when no row exists.
bool VerifyPrologueByRva(uintptr_t rva);


// =========================================================================
// Function pointer typedefs (called, not hooked)
// =========================================================================

// 4 arguments, and the last is a FLOAT in xmm3, not an int: the implementation
// (0xA0D6A0) is
//   bool __fastcall(void* zoneEntry, __int64 unused, int timerIndex, float keepAliveSeconds)
// The 3-argument typedef left xmm3 undefined at every call. The game's own
// state-2 path zeroes it — processState2 does `xorps xmm6, xmm6` then
// `movaps xmm3, xmm6` before the call at 0xA0D928 — so 0.0f is the value to
// pass for "no keep-alive".
typedef bool  (*loadSingleZone_t)(void* zoneEntry, __int64 unused, int timerIndex,
                                  float keepAliveSeconds);
typedef void* (*notifyZoneReady_t)(void* zoneEntry);
typedef void  (*finalizeZoneResources_t)(void* sectionEntry);
typedef void  (*notifyAccessible_t)(void* zoneMapContent);
typedef int   (*flushPendingWork_t)();
typedef void  (*registerZoneSections_t)(void* sectionMgr, void* zoneEntry);
typedef void  (__fastcall *setQueuesAreClear_t)(void* physics, bool on);

extern loadSingleZone_t        fn_loadSingleZone;
extern notifyZoneReady_t       fn_notifyZoneReady;
extern finalizeZoneResources_t fn_finalizeZoneResources;
extern notifyAccessible_t      fn_notifyAccessible;
extern flushPendingWork_t      fn_flushPendingWork;
extern registerZoneSections_t  fn_registerZoneSections;
extern setQueuesAreClear_t     fn_setQueuesAreClear;

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

// NavMesh queue lock initializer (despite the name, this IS the CS init function —
// called 5 times from NavMeshGen_struct_ctor at +72/+104/+152/+200/+272 to initialize
// the CSes/locks embedded in the NavMeshGenerator struct). Needed for Stage 4c NMG cloning.
typedef void (*queueLockInit_t)(void* lockAddr);
extern queueLockInit_t fn_queueLockInit;

// NavMesh active cache: called functions (zone optimization -- all builds)
typedef void (*processJobAlt_t)(void* thisNavMeshGen, void* job);
// 4 arguments, from the decompile of the implementation at 0x3CB700:
//   __int64 __fastcall(void** this, char* job, __int64 a3, double a4)
// a3 is never read in its body and a4 is overwritten before use inside
// stitch_buildCollision (0x3C5C10), so the two-argument typedef was harmless in
// practice — but it left r8 and xmm3 undefined at every call. Found in A6.
typedef __int64 (*buildCollision_t)(void* thisNavMeshGen, void* job, __int64 unused,
                                    double unusedF);
typedef void (*partialFixup_t)(void* thisNavMeshGen, void* job);
typedef void (*enqueueToProcQueue_t)(void* queueAddr, void* job);
typedef void* (*navMeshCtor_t)(void* mem);
typedef void* (*settingsCtor_t)(void* mem);  // hkaiNavMeshGenerationSettings constructor
// SimplificationSettings::copy (0x3DA000). NOT a memcpy: it copies scalars, calls
// ExtraVertexSettings::copy for the +88 sub-object and hkStringPtr::copy for the
// +152 string, which allocates. Needed to duplicate an override entry's +80.
typedef void  (*simplSettingsCopy_t)(void* dst, void* src);
// hkaiNavMeshGenerationSettings dtor body (0xDD9BC0). Destroys the settings in
// place without freeing the object: it releases the +240 carvers and +256
// painters element by element, destroys +336 and +512, runs the +144 sub-struct
// destructor (which frees +160 and +176), and frees the +288 and +520 buffers
// only when they do NOT carry DONT_DEALLOCATE.
typedef void  (*settingsDtorBody_t)(void* settings);
typedef void* (*gameNew_t)(size_t size);
typedef void  (*gameDelete_t)(void* ptr);
typedef void* (*gameNewArr_t)(size_t size);
typedef void  (*gameDelArr_t)(void* ptr);

extern navMeshCtor_t        fn_navMeshCtor;
extern settingsCtor_t       fn_settingsCtor;
extern simplSettingsCopy_t  fn_simplSettingsCopy;
extern settingsDtorBody_t   fn_settingsDtorBody;
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

// Island routing: lektor<ZoneMap*> growth (thunk 0x16630 -> 0x37E3A0)
typedef void (*lektorReserve_t)(void* lektor, unsigned int newCapacity);
extern lektorReserve_t fn_lektorReserve;

#if PATHFIND_STEP >= 1
// Path request queue enqueue
typedef void (*enqueuePathReq_t)(void* queueBase, void** itemPtr);
extern enqueuePathReq_t fn_enqueuePathReq;
#endif // PATHFIND_STEP >= 1

#if PATHFIND_STEP >= 6
// sub_140033320: resolve face -> two edge endpoint vertices (worldspace __m128).
typedef void (*faceToVertices_t)(void* instance, int faceIdx,
                                 void* outEdgeVertA, void* outEdgeVertB,
                                 uintptr_t scratch);
extern faceToVertices_t fn_faceToVertices;
#endif // PATHFIND_STEP >= 6

#if PATHFIND_STEP >= 9
// Havok::contentStreamCallee_0x8869 (path-result extraction loop). 4-arg
// fastcall. Our hook SEH-wraps it and on AV rolls back a4[2] so the game sees
// "no new edges produced" -> treats as no path, retries cleanly.
typedef unsigned __int64 (*contentStreamCallee0x8869_t)(void* manager,
                                                         unsigned int faceKey,
                                                         void* searchOutput,
                                                         unsigned int* resultBuf);
// hkaiStreamingCollection::addInstance. 5-arg fastcall; mutates m_instances.
// Hooked so we can timestamp the last mutation and gate our cache injection.
typedef void (*addInstance_t)(void* collection, __int64 sectionData,
                              __int64 param3, __int64 param4, int param5);

extern contentStreamCallee0x8869_t orig_contentStreamCallee0x8869;
extern addInstance_t               orig_addInstance;
#endif // PATHFIND_STEP >= 9


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

// 4 register arguments, no stack argument: the wrapper writes its own 5th
// (timeLowPart = 0) into the shadow space before tail-calling realGenerate
// (disassembly of 0xE0AFF0). The old 5-parameter declaration made the hook read
// uninitialized shadow space. H11, pulled forward into NMFIX 1.
typedef void (*nmResultPopulate_t)(void* navData, void* localData, void* result, int param);
extern nmResultPopulate_t      orig_nmResultPopulate;

// Island routing hooks (Phase 15)
typedef bool  (*isInIsland_t)(void* zoneA, void* zoneB);
typedef void* (*getIsland_t)(void* zoneMgr, void* zone, void* lektorOut);
extern isInIsland_t            orig_isInIsland;
extern getIsland_t             orig_getIsland;

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


// ---- Round 2 P additions (only task P edits this block) ----
// Phase 17 Step 1 instrumentation (research/path_worker_pool.md §7): four
// pass-through hooks on the path thread's own functions (contentStream,
// dequeueWork, enqueue_threadSafe, Gates__updateCodes), gated on
// PATHPOOL_STEP so they add nothing to the six standard variants (still at
// 0), plus one "called, not hooked" utility (isPriorityPath) for the NPC
// wait diagnostic. Kept independent of PATHFIND_STEP's own typedefs (e.g.
// enqueuePathReq_t, PATHFIND_STEP >= 1 only) on purpose, so this block never
// depends on that gate.
#if PATHPOOL_STEP >= 1

// SectionManager::contentStream (0x3AE350): one call per path-thread pass.
// char __fastcall(SectionManager*): return 1 when the pass served a request.
// Sole caller NavMesh__threadProc (0x3AEFB0) -- IDA-verified 2026-09-12.
const size_t RVA_CONTENT_STREAM = 0x3AE350;
typedef char (*contentStream_t)(void* sectionMgr);
extern contentStream_t orig_contentStream;

// SectionManager::dequeueWork_threadSafe (0x3BEE40): pops one request from
// the queue passed in. Sole caller (IDA xref, 2026-09-12): contentStream, via
// j_SectionManager__dequeueWork, always with queueBase = mgr+0xB8 (the input
// queue). We stamp the returned request's req+0x00 with the drain QPC --
// never written by the game (ctor 0x14D240, cleanup 0x3BD3A0, submit
// 0x3AAEF0, processPathResult 0x3A27A0 and applyPathResult 0x144C90 all
// checked, none touch it: path_thread_audit.md §3).
const size_t RVA_DEQUEUE_WORK = 0x3BEE40;
typedef void* (*dequeueWork_t)(void* queueBase);
extern dequeueWork_t orig_dequeueWork;

// PathRequestQueue::enqueue_threadSafe (0x3B6110). 4 call sites (IDA xref,
// 2026-09-12): PathRequestQueue::submit (+0x6B, mgr+0xB8, main thread),
// PathRequestQueue::submitCancel (+0x50, no callers in this build) and
// SectionManager::contentStream twice (+0x5E2 the section-add sentinel,
// +0xC1A the serve-block completion -- both mgr+0xF8, the result queue, path
// thread). Filtered at runtime to the result queue; the input-queue call
// passes through untouched. Same RVA as RVA_ENQUEUE_PATH_REQ / fn_enqueuePathReq
// (PATHFIND_STEP >= 1 only), but declared again here, independently, so this
// block never depends on that gate.
const size_t RVA_ENQUEUE_THREAD_SAFE = 0x3B6110;
typedef void (*enqueueThreadSafe_t)(void* queueBase, void** itemPtr);
extern enqueueThreadSafe_t orig_enqueueThreadSafe;

// Gates__updateCodes (0x2EF460): the gate-code pass that runs on the path
// thread once section adds drain, before the loading-screen dismissal
// (gate_codes.md §1). Sole caller SectionManager::contentStream at +0x7ED.
// Timed from function entry so a zero-gate pass still counts (it still takes
// the handshake).
const size_t RVA_GATES_UPDATE_CODES = 0x2EF460;
typedef __int64 (*gatesUpdateCodes_t)(void* gatesObj);
extern gatesUpdateCodes_t orig_gatesUpdateCodes;

// CharMovement::isPriorityPath (0x790B30, 27 bytes): player-owned check --
// vtable+88(character)+592 != 0 (Faction* -> PlayerInterface*; CLAUDE.md "Key
// Architectural Facts"). Called directly, not hooked, from the NPC wait
// diagnostic to report player-owned characters separately.
const size_t RVA_IS_PRIORITY_PATH = 0x790B30;
typedef bool (*isPriorityPath_t)(void* character);
extern isPriorityPath_t fn_isPriorityPath;

#endif // PATHPOOL_STEP >= 1
// ---- end Round 2 P ----

// ---- Round 2 H1 additions (only task H1 edits this block) ----
//
// HavokCharacter fields read from CharMovement's HavokCharacter pointer
// (OFF_CMOV_HAVOK_CHAR, +0x320). Named in CLAUDE.md ("HavokCharacter (`+136`
// = arrival/idle code, `+144` = path state)") and edge_advance_feasibility.md
// section 1.1; OFF_HC_PATH_STATE is decimal 144 = 0x90.
// OFF_HC_PATH_STATE 3 is not "in progress": the engine's CharMovement::pathFailed
// (0x65DDA0) keys on it. Its raw test, on the CharMovement* cm:
//   hc = cm+0x320; return 0 if hc == NULL or the dword at cm+0x378 != 0;
//   return 0 if hc+0x90 != 3;
//   return 0 if the byte at cm+0x370 is set and the dword at cm+0x368 < 16;
//   else return 1.
// So 3 is the value pathFailed reports as failed; the +0x378 / +0x370 / +0x368
// conditions are stated as read, with no meaning assigned.
const size_t OFF_HC_PATH_STATE = 0x90;  // int: 3 = the value CharMovement::pathFailed reports as failed
const size_t OFF_HC_ARRIVAL    = 136;   // int: arrival/idle code (<=1 idle)

// Current-order-type chain from Character::playerMoveOrderDefault (0x5D1820),
// verified in IDA 2026-09-12 (see CLAUDE.md item (6) / the H1 report trace).
// Four dereferences, each must be null-checked:
//   pendingTaskListHead = *(uintptr_t*)(character + OFF_CHAR_PENDING_TASK_PTR)
//   pendingTask         = *(uintptr_t*)(pendingTaskListHead + OFF_PENDING_TASK_HEAD_OFF)
//   orderObj            = *(uintptr_t*)(pendingTask + OFF_PENDING_TASK_ORDER_OFF)
//   orderType           = *(int*)(orderObj + OFF_ORDER_TYPE)
// A null link at any step means "no cached order" -- playerMoveOrderDefault
// then always takes the fresh-AddOrder path, not the in-place rewrite.
const size_t OFF_CHAR_PENDING_TASK_PTR  = 1608;
const size_t OFF_PENDING_TASK_HEAD_OFF  = 104;
const size_t OFF_PENDING_TASK_ORDER_OFF = 112;
const size_t OFF_ORDER_TYPE             = 68;
const int    ORDER_TYPE_MOVE            = 29;
// ---- end Round 2 H1 ----

// ---- Round 2 H2 additions (only task H2 edits this block) ----
//
// Readiness classification in hook_isContentPending (hooks.cpp). Verified in
// the Steam 1.0.65 IDB on 2026-09-12. The SectionManager (pauseState.navmesh,
// the `manager` argument of isContentPending) holds:
//   +0x088  hkaiWorld* of the main navmesh world. Its +0x20 is the
//           hkaiStreamingCollection: instances data +0x20 (48-byte entries,
//           the first qword is the hkaiNavMeshInstance*), size +0x28. Same
//           reads as NavMesh__snapToFace 0x3A1F70.
//   +0x1E0  boost::shared_mutex over the section map. Taken exclusively (timed
//           lock) by isContentPending 0x3AB4F0, getOrCreateSection 0x3AB1F0,
//           the erase in 0x3AB450 and processZoneWorkItem 0x3ADF30.
//   +0x200  boost::shared_mutex over the world. Section adds (contentStream
//           0x3AE66B) and world removals (removeNavInstance 0x3AB8A0) take it
//           exclusively; the query functions (snapToFace 0x3A1F70 and 16 more)
//           try-lock it shared and give up when that fails.
//   +0x220  std::map<zone key, SectionState*> (key y + 10000*x); its header
//           node pointer is at +0x228; the value is at node +0x20.
// hkaiNavMeshInstance +0x1A0 is its section uid; an outdoor zone's is
// gridX | (gridY << 8) (the section's +60, copied by 0xD09EB0 through
// 0x3ACD20). +0x1A4 is its slot in the collection, -1 when not in the world
// (addInstance 0xD0D8F0 writes it, removeInstance 0xD0DC30 resets it).
// hkaiStreamingCollection's own uid lookup is 0xD0CD40 (a linear scan).
const size_t OFF_RDY_SM_WORLD            = 0x88;
const size_t OFF_RDY_SM_MAP_LOCK         = 0x1E0;
const size_t OFF_RDY_SM_WORLD_LOCK       = 0x200;
const size_t OFF_RDY_SM_SECTION_MAP      = 0x220;
const size_t OFF_RDY_SM_SECTION_MAP_HEAD = 0x228;
const size_t OFF_RDY_SM_PENDING_SECTIONS = 0x278;  // 632: sections not yet added
const size_t OFF_RDY_MAPNODE_VALUE       = 0x20;
const size_t OFF_RDY_WORLD_COLLECTION    = 0x20;
const size_t OFF_RDY_SC_DATA             = 0x20;
const size_t OFF_RDY_SC_COUNT            = 0x28;
const size_t RDY_SC_ENTRY_SIZE           = 48;
const size_t OFF_RDY_NMI_SECTION_UID     = 0x1A0;
const size_t OFF_RDY_NMI_RUNTIME_INDEX   = 0x1A4;

// Called, never hooked (no build-gate rows needed).
//   SectionManager__lookupSection 0x3BBA60: std::map lower_bound on the zone
//     key; writes the header node to *out when there is no entry. Pure reads.
//   boost::shared_mutex::unlock 0x25C3D0 and unlock_shared 0x168E10: the
//     game's own release paths (CAS + ReleaseSemaphore, no allocation).
const size_t RVA_LOOKUP_SECTION       = 0x3BBA60;
const size_t RVA_BOOST_UNLOCK         = 0x25C3D0;
const size_t RVA_BOOST_UNLOCK_SHARED  = 0x168E10;

typedef void* (*lookupSection_t)(void* sectionMap, void** outNode, const int* zonePos);
typedef unsigned int (*boostUnlock_t)(void* mutex);
typedef long (*boostUnlockShared_t)(void* mutex);

extern lookupSection_t      fn_lookupSection;
extern boostUnlock_t        fn_boostUnlock;
extern boostUnlockShared_t  fn_boostUnlockShared;
// ---- end Round 2 H2 ----

// ---- Round 2 F additions (only task F edits this block) ----
// ---- end Round 2 F ----

// ---- Round 2 Z additions (only task Z edits this block) ----
// Phase 18 Step 1 instrumentation (research/preload_pipeline.md §2). Called,
// not hooked: bool __fastcall(void* zoneEntry), confirmed in IDA
// (ZoneMapContent__isZoneReady, size 0x2C): `return zoneEntry && *(zoneEntry+184)
// && globalReadyCheck(...)`. No allocation, never blocks (§2 of the doc).
const size_t RVA_ZONE_IS_READY  = 0xA08E10;
// ZoneMapContent+264, BYTE: ZoneMapContent__vtable32 (0x9FF730) clears it once
// isZoneReady(zoneEntry) passes and finalizeContent has run (confirmed in IDA:
// `*((_BYTE*)this + 264) == 0` gates the call, then is itself cleared to 0).
const size_t OFF_ZMC_READY_FLAG = 264;

typedef bool (__fastcall *isZoneReady_t)(void* zoneEntry);
extern isZoneReady_t fn_isZoneReady;
// ---- end Round 2 Z ----

// ---- Round 2 N additions (only task N edits this block) ----
// ZoneMap+0xB8: TerrainSector* terrainCollision (KenshiLib ZoneManager.h).
// ZoneManager__unloadSingleZone (0xA09620) frees and NULLs it after it NULLs
// mapContent (+0). processJobAlt reads *(zone+0xB8)+8 unconditionally for
// type 0/1 jobs (0x3C1580 via 0xA07B50, called from 0x3CBE60+0xB3C).
const size_t OFF_ZONE_TERRAIN_COLLISION = 0xB8;
// ---- end Round 2 N ----

// ---- Round 2 fix 2b additions (save-load reset; only fix 2b edits this block) ----
//
// The game's "Reset game" step, sub_14036CA40 (0x36CA40), runs at the start of
// every save load (SaveManager::loadGame 0x373DC0, via thunk 0x41C31 at
// 0x373E82) and from SaveManager::execute / importGame. At 0x36CE38 it calls
// sub_14036C1E0 below; then, at 0x36CF2B, the world clear sub_1407A82C0; then,
// at 0x36CF60, the ZoneMap handle registry's vt+24 (sub_1404FDC30, a memset of
// every slot). All three run on the main thread (SaveManager::execute's callers
// are GameWorld__mainLoop_GPUSensitiveStuff +0x241 and MainListener's vtable
// slot 3); the hook still checks at runtime. So while sub_14036C1E0 runs, the
// registry and the world's object set are still intact. Not everything is:
// the reset has already run GameWorld__populateMapArea_nonPermanent (0x36CB6A),
// the sub_140786790 purge (0x36CB96), FactionManager__clearAndDestroy
// (0x36CBBA) and the sub_1408F89A0 section-manager pass (0x36CBD7) -- the same
// state the game's own Set A/B unloads run in.
//
// sub_14036C1E0 (0x36C1E0, 0x2C8 bytes), `void __fastcall(ZoneManager*)`, its
// only caller being the reset through thunk 0x513FC (call at 0x36CE38): it copies every zone in
// Set B (ZM+1474824, count +1474856) and Set A (ZM+1474760, count +1474792)
// into an array, calls sub_140A09BB0(zm, zone, 0) on each (0x36C400-0x36C409),
// empties Set A, and zeroes ZM+1475024 (OFF_ZM_CURRENT_ZONE). Zones in neither
// set, i.e. the mod's, are left loaded; the registry memset that follows then
// leaves them with a live content and a dead registration. Prologue
// `mov r11,rsp; mov [r11+8],rcx` is position-independent.
const size_t RVA_RESET_UNLOAD_ZONES = 0x36C1E0;
typedef void (__fastcall *resetUnloadZones_t)(void* zoneMgr);
extern resetUnloadZones_t orig_resetUnloadZones;

// sub_140A09BB0 (0xA09BB0, 0x4B bytes), called through its thunk 0x365ED exactly
// as the original does: `__int64 __fastcall(ZoneManager*, ZoneMap*, u8 param)`.
// It calls ZoneManager::unloadSingleZone(zone, param) (0xA09620 via thunk
// 0x440DF: a no-op unless ZoneMap+0 is set; otherwise SectionManager
// onZoneUnload, Town::notifyUnloading, ZoneContent::prepareUnload (which puts
// the registry sentinel in the zone's slot), content delete, +0 = NULL,
// +176 = +177 = 0), erases the zone from Set B (sub_1409F0100: a no-op for a
// zone not in it), then sub_1409D59C0(qword_142134BE8, zone) (a per-zone map
// keyed x + 64y, releasing its +16 resource). The reset passes param = 0.
// param is ZoneContent::prepareUnload's save-zone-state flag (its a2, passed
// through unloadSingleZone): non-zero calls sub_14036DCA0 (thunk 0x48A63, at
// 0x9FDB20), the zone-state serializer the save paths 0x36E310 / 0x36EAD0 also
// use; 0 discards the zone state, which is what a load needs -- the old world's
// zones must not be written into the save data being loaded.
const size_t RVA_UNLOAD_ZONE_FROM_RESET = 0x365ED;   // thunk -> 0xA09BB0
typedef __int64 (__fastcall *unloadZoneFromReset_t)(void* zoneMgr, void* zoneEntry,
                                                      unsigned char param);
extern unloadZoneFromReset_t fn_unloadZoneFromReset;

// ZoneMap handle registry: the ZoneMapHandleContainerList object at 0x2132F50
// (vtable 0x171F828; a member of the HandleManager at 0x2132F30).
//   +8  (0x2132F58) void** slots  -- ZoneContent::prepareUnload writes
//                                    `*(qword_142132F58 + 8*idx) = sentinel` (0x9FDE30)
//   +16 (0x2132F60) u32    count  -- registerHandle 0x380930 compares the index
//                                    against it (0x38098C); the clear
//                                    sub_1404FDC30 memsets 8*count bytes
// ZoneMapContent's ctor (0xA00720, called only from loadSingleZone) builds the
// HandleDummy, stores the dummy pointer at content+0xA0 (0xA008C1), and
// registers it with hand {type 12, container = ZoneMap+24 + (ZoneMap+28 << 6) + 1,
// serial 11111} (0xA008C8-0xA008F0). registerHandle stores the dummy pointer
// itself in slots[container] when the slot is 0 or the sentinel (0x3809C8-
// 0x3809DF); the list's vt+80 (sub_1408794D0) returns hand+12, the container.
// An empty slot is 0 (after the reset's memset) or *(u64*)RVA_HANDLE_SENTINEL
// (after prepareUnload). The lookup (vt+32, sub_140336C20) refuses both, then
// also compares the dummy's hand serial -- a constant 11111 for zones.
const size_t RVA_ZONEMAP_HANDLE_LIST  = 0x2132F50;
const size_t RVA_ZONEMAP_HANDLE_SLOTS = RVA_ZONEMAP_HANDLE_LIST + 8;    // 0x2132F58
const size_t RVA_ZONEMAP_HANDLE_COUNT = RVA_ZONEMAP_HANDLE_LIST + 16;   // 0x2132F60
const size_t OFF_ZMC_HANDLE_DUMMY     = 0xA0;   // ZoneMapContent::HandleDummy* (dummy+120 = content)
// ---- end Round 2 fix 2b ----

#endif // KENSHI_ZONE_OPT_GAME_H
