// islands.h — Island routing overlay (Phase 15): mod-promoted zones join the
// game's routing islands without touching ZoneMap+0x20 or _calculateIslands.
// Depends on: config.h
//
// Threading contract:
//   - Builder (IslandTick, IslandMarkModZone, IslandRequestRebuild, IslandNoteOrder,
//     IslandDescribeStuck, IslandEmulateCrossing): MAIN THREAD ONLY.
//   - Hooks (hook_isInIsland, hook_getIsland): any thread that calls
//     CharMovement::setDestination or the smell picker (main, AI, render back
//     thread). They take no locks, allocate nothing, log nothing; they read the
//     published snapshot through a seqlock and bump Interlocked counters.
//   - IslandCountReadiness: any thread (hook_isContentPending).
//
// ISLAND_STEP (temporary staging gate, see config.h):
//   0 = readiness-gate side effects on the main thread only + per-thread counters
//   1 = builder, snapshot, marks, load reset, emulation, PLAYER STUCK fields;
//       both hooks installed but PASS THROUGH (count would-flip / would-append)
//   2 = both hooks LIVE (subject to the islandFix INI key)
//   3 = parked-squad re-issue (a) + tier-ordered registration/promotion (b)

#ifndef KENSHI_ZONE_OPT_ISLANDS_H
#define KENSHI_ZONE_OPT_ISLANDS_H

#include "config.h"


// =========================================================================
// Step 0: readiness-gate per-thread counters (always compiled)
// =========================================================================

// Called from hook_isContentPending on whichever thread the game uses.
// notReady = original returned 0; sectionsPending = notReady with sections > 0.
void IslandCountReadiness(bool notReady, bool sectionsPending);

// Main thread, every frame from hook_updateCameraZone (before the stats reporters).
// At ISLAND_STEP 0 it only logs the readiness counters.
void IslandTick(void* zoneMgr, double now);


#if ISLAND_STEP >= 1

// =========================================================================
// Hooks (installed by main.cpp when preloadEnabled; both or neither go live)
// =========================================================================

bool  hook_isInIsland(void* zoneA, void* zoneB);
void* hook_getIsland(void* zoneMgr, void* zone, void* lektorOut);

// main.cpp: called once after both hooks installed successfully.
void IslandSetHooksInstalled(bool installed);

// True when the overlay answers the hooks (installed && islandFix && ISLAND_STEP >= 2).
bool IslandHooksLive();


// =========================================================================
// Marks + rebuild requests (main thread)
// =========================================================================

// preload.cpp: called just before the WORD write that makes a promoted zone
// accessible. Marks survive ClearPreloadZones.
void IslandMarkModZone(void* zoneMgr, int gx, int gy);

// preload.cpp: called after a promotion completes (forces a rebuild this frame).
void IslandRequestRebuild();

// preload.cpp (ClearPreloadState): full reset of marks, snapshot and tracker.
void IslandReset();


// =========================================================================
// Router emulation + diagnostics (main thread)
// =========================================================================

// Emulate CharMovement::computeProjectedDest for a character standing in
// charZone, along the ray from (destX,destZ) toward (posX,posZ), using the
// island list the router receives in the CURRENT configuration (vanilla when
// the hooks pass through, overlay when live). Returns the RAW first crossing
// (before the 300-unit navmesh snap). false = no island zone crossed.
bool IslandEmulateCrossing(void* zoneMgr, void* charZone,
                           float destX, float destZ, float posX, float posZ,
                           float* outX, float* outZ);

// Extra fields for the PLAYER STUCK diagnostic line (pathfind_diag.cpp).
struct IslandStuckInfo {
	float wpX, wpZ;          // CharMovement::pathDestination (+0xE8)
	int   movingToEdge;      // +0x370
	int   edgeCounter;       // +0x368
	int   selfComp;          // liveComp of the character's zone (-1 none)
	bool  haveCrossing;      // emulation found a crossing
	float xd;                // |pos - raw crossing| when haveCrossing
	int   nextGX, nextGY;    // zone just beyond the crossing (or grid walk fallback)
	int   nextComp;          // liveComp(next)
	int   nextLabel;         // next zone +0x20
	int   nextLoading;       // next zone +176
	int   nextAccess;        // next zone +177
};
bool IslandDescribeStuck(void* zoneMgr, uintptr_t charMov,
                         float posX, float posZ, float destX, float destZ,
                         IslandStuckInfo* out);

#else // ISLAND_STEP < 1

inline void IslandSetHooksInstalled(bool) {}
inline bool IslandHooksLive() { return false; }
inline void IslandMarkModZone(void*, int, int) {}
inline void IslandRequestRebuild() {}
inline void IslandReset() {}

#endif // ISLAND_STEP >= 1


// Fix round 1 (review Minor #4): reasons an order from outside the tracker
// overtook a pending re-issue check inside its 1 s window. The resolved
// "Island reissue result:" line then carries click=1 / retry=1, because a
// post=other result may be that order, not the re-issue.
enum IslandOvertakeReason
{
	ISLAND_OVERTAKEN_CLICK = 1,   // player move order (IslandNoteOrder)
	ISLAND_OVERTAKEN_RETRY = 2    // PLAYER STUCK retry fn_moveOrder (pathfind_diag.cpp)
};

#if ISLAND_STEP >= 3
// Flags `character`'s pending re-issue check, if it has one (no early
// resolve, no dereference of `character`). Main thread only.
void IslandFlagReissueOvertaken(uintptr_t character, int reason);

// hooks.cpp: called for every selected character of a task-29 (move) order.
void IslandNoteOrder(uintptr_t character, const float* location);

// hooks.cpp: called for every selected character of a NON-move (task != 29)
// player order. Drops that character's IslandOrder if it has one -- a later
// non-move order (attack, job, pick-up, talk) ends the relevance of an old
// tracked move order the same way arrival or a new move order does.
void IslandDropOrder(uintptr_t character);


// =========================================================================
// Round 2 H1: shared re-issue helpers (main thread only)
//
// Used by both islands.cpp (ReissueCharacter, per-character re-issue) and
// formation.cpp (FormationReissueTravel, per-member re-issue). Declared here
// instead of duplicated per file, per the H1 brief.
// =========================================================================

// (b) Direction-aware nudge, replacing the fixed one-sided +3 on x.
// CharMovement::setDestination_Vec3 drops a new order within 2 units of the
// last requested destination (+0xDC) while routing to an island edge. When
// (*x,*z) is within 8 units of (lastX,lastZ), push it to
// lastDest + 8*unit(sent - lastDest); when the two coincide (length < 0.01),
// push +8 on x only (z left at lastZ). No-op when already >= 8 units away.
inline void IslandNudgeAwayFromLastDest(float lastX, float lastZ, float* x, float* z)
{
	const float NUDGE_DIST    = 8.0f;
	const float COINCIDE_DIST = 0.01f;
	float dx = *x - lastX, dz = *z - lastZ;
	float len = sqrtf(dx * dx + dz * dz);
	if (len >= NUDGE_DIST) return;
	if (len < COINCIDE_DIST)
	{
		*x = lastX + NUDGE_DIST;
		*z = lastZ;
		return;
	}
	float scale = NUDGE_DIST / len;
	*x = lastX + dx * scale;
	*z = lastZ + dz * scale;
}

// (a) Discriminator line. Fields read before fn_moveOrder (playerMoveOrderDefault,
// vtable+0x318) is called: +0xDC (last dest), +0xE8 (pathDestination), +0x370
// (movingToEdge), +0x368 (edge counter), +0xC4 (position), the HavokCharacter
// path state (+0x90) and arrival code (+136), and the character's current
// order type (0 = no cached order / not a move order). Safe with a null
// CharMovement or HavokCharacter (fields read as 0, orderType as -1); the
// caller still gets a trace with valid=false.
struct IslandReissueTrace
{
	bool  valid;
	float lastX, lastZ;   // +0xDC before the call
	float wpX, wpZ;       // +0xE8 before the call
	float posX, posZ;     // +0xC4 before the call (for moved= at resolve time)
	int   edge;           // +0x370 before the call
	int   edgeCtr;        // +0x368 before the call
	int   hcPathState;    // HavokCharacter+0x90 before the call
	int   hcArrival;      // HavokCharacter+136 before the call
	int   orderType;      // current order type before the call (-1 = none/no chain)
};

void IslandCaptureReissueTrace(uintptr_t character, IslandReissueTrace* out);

// Round 1 review, Important #3: single owner of the post=sent|last|other
// classification. Since Round 2 fix 2a it runs only at resolve time (see
// IslandRecordReissueCheck below), for both the per-member lines and the
// group summary count.
enum IslandReissuePost { ISLAND_POST_SENT, ISLAND_POST_LAST, ISLAND_POST_OTHER };
IslandReissuePost IslandClassifyReissuePost(uintptr_t character, float sentX, float sentZ,
                                            const IslandReissueTrace& pre);

// Round 2 fix 2a: deferred (a) discriminator.
//
// playerMoveOrderDefault hands the order to the AI task system, which applies
// it asynchronously, so +0xDC read in the same tick as the call always still
// held the previous destination (every Round 2 line read post=last d=8.0).
// The caller now records a pending check right after fn_moveOrder; islands.cpp
// resolves it from IslandTick at the first tick at least
// REISSUE_RESULT_DELAY (1.0 s) after `now`, re-validating the character
// against the player squad list first (dropped silently and counted as
// reissueCheckDropped= when it is gone). The resolved line is:
//   [ZoneOpt] Island reissue result: <label> post=sent|last|other d=<|last-sent|>
//     order=<type> edge=<a>/<b>-><a>/<b> wp=(x,z)->(x,z)
//     ps=<a>-><b> hc136=<a>-><b> dt=<ms> moved=<units> [click=1] [retry=1]
// where every -> pair is pre-order -> resolve time, and click=1 / retry=1
// mark an order from outside the tracker inside the window
// (IslandFlagReissueOvertaken). `label` identifies the character
// ("char@<hex>", "group N member K char@<hex>"); it is copied.
//
// `dispatch` groups the members of one FormationReissueTravel call in summary
// mode (group above 6 members): -1 = none (every result prints its own line).
void IslandRecordReissueCheck(uintptr_t character, const char* label,
                              float sentX, float sentZ,
                              const IslandReissueTrace& pre, double now, int dispatch);

// Summary-mode dispatch (FormationReissueTravel, groups above 6 members):
// per-member lines only for non-sent results, plus one
//   [ZoneOpt] Island reissue result: group <slot> summary <sent>/<total> sent
// line once every member recorded under the dispatch has resolved or been
// dropped (dropped members count toward total, not sent).
// Begin returns -1 when summaryMode is false or the dispatch table is full
// (then every member prints its own line). End closes the dispatch to new
// members; it must be called once for every Begin, even when nothing was sent.
int  IslandBeginReissueDispatch(int slot, bool summaryMode);
void IslandEndReissueDispatch(int dispatch);

// Round 1 review, Important #1 backstop: true if `character` was re-issued by
// the tracker (ReissueOrder, solo or as part of a formation group) within the
// last REISSUE_COOLDOWN seconds. FormationReissueTravel skips sending a member
// a second move order inside one cooldown window (e.g. it was already
// re-issued solo this cycle via the (c) per-member path).
bool IslandRecentlyReissued(uintptr_t character, double now);

// Round 2 review, Important #1 second residual (group-then-solo cross-tick):
// stamps `character`'s own IslandOrder.lastReissueTime (the same field
// IslandRecentlyReissued reads) to `now`, WITHOUT touching reissueCount or
// parked. FormationReissueTravel calls this for every member it actually
// sends fn_moveOrder to (representative included), so a member whose order is
// swallowed by the group blast still carries a fresh cooldown timestamp even
// though its own park state was never touched -- closing the gap where a
// first-ever solo park-transition for that member would otherwise call
// ReissueOrder immediately, which the cooldown guard should, but previously
// could not, block (lastReissueTime was still 0.0). No-op if `character` has
// no active IslandOrder entry.
void IslandMarkReissued(uintptr_t character, double now);

#else
inline void IslandNoteOrder(uintptr_t, const float*) {}
inline void IslandDropOrder(uintptr_t) {}
inline void IslandFlagReissueOvertaken(uintptr_t, int) {}
#endif


#endif // KENSHI_ZONE_OPT_ISLANDS_H
