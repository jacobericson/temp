// core.h — Platform, logging, timing (Layer 0)
// No game knowledge. Included by every module.

#ifndef KENSHI_ZONE_OPT_CORE_H
#define KENSHI_ZONE_OPT_CORE_H

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <Windows.h>
#include <core/Functions.h>
#include <Debug.h>

#include <sstream>
#include <iomanip>
#include <fstream>
#include <cstring>
#include <cmath>
#include <string>


// =========================================================================
// Timing state (extern — needed by ElapsedSec/QPCToMs inlines)
// =========================================================================

extern LARGE_INTEGER qpcFrequency;
extern LARGE_INTEGER pluginStartTime;

// Captured on first hook_dispatchJob call (by nm_workers.cpp or nm_workers_stub.cpp).
// Read by NavMeshCrashHandler in main.cpp to compare against GetCurrentThreadId()
// at crash time — diagnoses whether processJob crashes on the bg thread or elsewhere.
extern volatile DWORD g_navMeshBgThreadId;

// Set to 1 by the NavMesh::stop hook when the game begins tearing the navmesh
// system down (hook_navMeshStop, nm_workers.cpp, sets it as its first statement,
// before RetireNavMeshWorkers runs). Read by the crash handler (main.cpp) to
// append "afterStop=1" to any crash record written after that point -- tagged,
// not suppressed, since a real mod crash can still happen there (session 4's
// quit-time record at rva 0xE67E9A was ours).
extern volatile LONG g_navMeshStopSeen;


// =========================================================================
// Timing utilities (inline)
// =========================================================================

inline double ElapsedSec()
{
	LARGE_INTEGER now;
	QueryPerformanceCounter(&now);
	return (double)(now.QuadPart - pluginStartTime.QuadPart) / (double)qpcFrequency.QuadPart;
}

inline double QPCToMs(const LARGE_INTEGER& start, const LARGE_INTEGER& end)
{
	return (double)(end.QuadPart - start.QuadPart) * 1000.0 / (double)qpcFrequency.QuadPart;
}


// =========================================================================
// Logging
// =========================================================================

std::string GetDLLDirectory();
void InitLogFile();
void LogMsg(const std::string& line);

// True on the thread that called InitLogFile (startPlugin -> game main thread).
bool IsMainThread();

// Verbose logging: active in all dev builds (ZONEOPT_DEBUG defined for
// both DEV and ZONEONLY_DEV). Compiled out entirely in prod builds.
#ifdef ZONEOPT_DEBUG
void LogDebug(const std::string& line);
#else
inline void LogDebug(const std::string&) {}
#endif


// =========================================================================
// Our-guard counter (B7)
// =========================================================================
//
// The mod's own __try blocks fault on purpose: the work-buffer probes walk
// until they hit the end of an allocation, and the path-result guard exists
// because the extraction chain can fault on a stale face key. Those faults
// are handled where they happen and must never reach the crash recorder,
// which would otherwise write a crash_dump.txt for a non-event and, in PROD,
// spend the one recorded fault of the process on it.
//
// Per thread, so a probe on the NavMesh bg thread never masks a real fault on
// the main thread. Nesting is counted, not flagged.

extern __declspec(thread) int g_inOurGuard;

inline void GuardEnter() { ++g_inOurGuard; }
inline void GuardLeave() { --g_inOurGuard; }
inline bool InOurGuard() { return g_inOurGuard != 0; }


// =========================================================================
// Build gate (B1)
// =========================================================================
//
// The mod writes 5-byte jumps into 20-odd game functions and byte-patches one
// more. Every one of those addresses is a hardcoded RVA for Steam/GOG 1.0.65,
// so on any other build they name whatever happens to sit there. Comparing the
// first 16 bytes of each site against bytes read from the IDB turns that from a
// silent corruption into a refusal to install.
//
// The table itself lives in game.cpp (Layer 1); core only does the compare.

struct HookPrologue
{
	const char*   name;
	uintptr_t     rva;
	unsigned char bytes[16];
	// True for a site that only carries a diagnostic. A mismatch on one of these
	// turns that diagnostic off; it must not take the whole plugin down with it,
	// because the diagnostic is not what the player installed the mod for. Last
	// member on purpose: the rows in game.cpp leave it out and it value-
	// initialises to false, so only the diagnostic rows have to say anything.
	bool          diagnostic;
};

// Called once from startPlugin, before any VerifyPrologue call.
void SetCoreGameBase(uintptr_t base);

// memcmp the 16 bytes at gameBase + rva against expect. Logs once and returns
// false on a mismatch or an unreadable address.
//
// A site another plugin already detoured (5-byte `E9 rel32` or 6-byte
// `FF 25 rel32`, optionally 0x90/0xCC padded to offset 8) passes when the bytes
// past the detour still match: the binary is the one we know, and KenshiLib's
// AddHook chains onto the existing detour. *sharedOut is set true for those, so
// the caller can report them. A tail mismatch is still a hard failure.
bool VerifyPrologue(uintptr_t rva, const unsigned char* expect, const char* name,
                    bool* sharedOut = NULL);


// =========================================================================
// Havok TLS heap allocator (used by navmesh_cache + pathfind_diag)
// =========================================================================

// Called by InitGameBindings to pass gameBase + RVA without core depending on game.h
void SetHavokTlsParams(uintptr_t base, size_t rva);
void* HavokTlsAlloc(size_t size);
void  HavokTlsFree(void* ptr, size_t size);


// =========================================================================
// destroyListOE invariant probe (crash 3 diagnostic)
// =========================================================================
//
// BugReports/3-warm-cache-crash died in the game's per-frame destroy-list
// drain (sub_14079CD00+0x1C1) because GameWorld::destroyListOE held a non-zero
// element count over an empty node list: the drain enters on the count and
// dereferences the sentinel head without a null check. The state that produces
// exactly that signature is a lost update on insert, i.e. two unsynchronised
// inserts. See .superpowers/sdd/can-you-form-a-wobbly-reef/crash3-analysis.md.
//
// Two read-only instruments, both cheap enough for PROD:
//   * DestroyListProbeTick() — once per frame on the main thread, comparing
//     element count against the sentinel head for two hash containers in the
//     GameWorld singleton. It samples just AFTER the frame's own drain: the
//     drain is called at GameWorld__mainLoop_GPUSensitiveStuff+0x1B0 and
//     updateCameraZone at +0x385, in that order. A normal drain empties the
//     container, so a hit means a drain exited with a stale count over an empty
//     list — and the NEXT frame's drain is the one that faults on it.
//   * hook_destroyListInsert — pass-through on the sole inserter, recording
//     which thread calls it. A non-main thread there proves the race outright,
//     and proves the defect is vanilla rather than mod-authored.
//
// Neither writes game memory. The layering matches SetHavokTlsParams: game.cpp
// hands core the resolved address, so core keeps no game knowledge.

// Called by InitGameBindings with gameBase + RVA_GLOBAL_GAMEWORLD.
void SetDestroyListBase(uintptr_t gameWorld);

// Main thread, top of hook_updateCameraZone. One frame of warning ahead of the
// crash; see above for why it is after the drain and not before.
void DestroyListProbeTick();

// `, dlIns=<main>/<other>` for the transition stats line, plus the recorded
// off-main-thread return addresses the first time any exist.
std::string DestroyListStatsSuffix();

// -------------------------------------------------------------------------
// Mitigation: defer off-main-thread inserts to the main thread
// -------------------------------------------------------------------------
//
// The game inserts into destroyListOE from `PhysicalEntity::~PhysicalEntity`
// (RVA 0x4CAFF0; the IDB mislabels it as a basic_filebuf dtor) while sectors
// unload on the contentStream thread — `dlIns=2721/129 dlFrom=0x4cb02a` in
// BugReports/3-warm-cache-stuck. The main thread drains the same container
// every frame with no synchronisation at all, so an insert that overlaps
// another insert or the drain loses a link while both count, leaving the
// element count above the node list. The next frame's drain enters on the
// count and dereferences a NULL sentinel head (crash 3).
//
// The mitigation takes the off-main insert out of the race: the hook queues the
// object instead of touching the container, and the main thread performs the
// real insert on the next frame. Every write to destroyListOE is then on one
// thread. The object is not destroyed until a drain reaches it, and deferring
// by one frame only postpones that; nothing else in the binary frees it.
//
// This mitigates a vanilla defect that the mod's zone churn exposes, not
// mod-authored corruption — no mod write lands in the GameWorld object.

// Set at the install site, after LoadConfig (INI `destroyListDefer`).
void SetDestroyListDefer(bool enabled);

// Main thread, in hook_updateCameraZone: empties the deferred queue and
// performs each insert through the original, at most 8 batches of 32 per frame
// (leftovers next frame). Never holds the queue lock across the original call.
void DestroyListFlushDeferred();

// Drops the queue without inserting anything, counted as `dlDropClear`. Called
// when the world is cleared: sub_1407A82C0 (reached from SaveManager__execute
// and the save-load / new-game paths) drains destroyListOE with a2=1 and tears
// the scene down, so every queued pointer is about to be freed and replaying it
// would insert a dangling pointer into a container the clear has just emptied.
// The mod's save-load edge (`PreloadCheckSaveLoad` -> `ClearPreloadStateForLoad`
// in preload.cpp) is the signal; r1-fix2-report.md "Fix round 1" records which
// clear paths that edge covers and which it cannot see.
void DestroyListDropDeferred();

// Hook on the sole destroyListOE inserter, sub_140799BE0 (RVA 0x799BE0,
// `__fastcall(GameWorld*, Ogre::MovableObject*)`). On the main thread, and
// whenever deferral is off, it is a pass-through with counters only — no CRT,
// no logging, callable on any thread, except that a main-thread insert first
// removes any queued entry for the same object (`dlDedup`), because the set's
// own "already present" no-op is the thing a queue in front of it would lose.
// Off the main thread with deferral on it queues the object under `deferCS`
// (replacing any entry it already holds for that object) and returns without
// calling the original.
//
// What the original does, and therefore what deferral defers: it calls
// `getMovableType` (vtable+32), and when the type is "Entity" it also calls
// sub_140449240 via sub_140023317 — which unlinks the object from the loader
// singleton's queued (+320/+328) and preloaded (+288/+296) lists and deletes
// the list node's payload — then `setVisible(false)` and the set insert. The
// de-registration is on the Entity branch, i.e. the branch a PhysicalEntity's
// Ogre object normally takes, so it is normally deferred with the insert. That
// is a one-frame delay, not a lost call: nothing frees the object in between,
// and the flush performs the whole original. `dlDeferEnt=<entity>/<other>`
// counts which branch each deferred insert would have taken.
//
// Discarding the original's return value is safe: it tail-returns the address
// of its own 32-byte stack temporary (the `pair<iterator,bool>` that the set
// insert at sub_140018930 writes), which is dead the moment it returns, and
// none of the 22 call sites reads it. Per-site evidence: r1-fix2-report.md.
typedef __int64 (__fastcall *destroyListInsert_t)(void* gameWorld, void* movable);
extern destroyListInsert_t orig_destroyListInsert;
__int64 __fastcall hook_destroyListInsert(void* gameWorld, void* movable);


#endif // KENSHI_ZONE_OPT_CORE_H
