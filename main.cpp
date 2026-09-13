#include "preload.h"
#include "nm_workers.h"
#include "hooks.h"
#include "formation.h"
#include "pathfinding.h"
#include "islands.h"
#include "path_pool.h"
// B1: GetKenshiVersion. The header's own inline bodies use `KenshiPlatform::X`
// on an unscoped enum, which /W3 reports as C4482; the warning is about
// KenshiLib's code, not ours, so it is silenced around the include only.
#pragma warning(push)
#pragma warning(disable: 4482)
#include <kenshi/Kenshi.h>
#pragma warning(pop)
#if PATHFIND_STEP >= 5
#include "tracking.h"
#include "navmesh_sched.h"
#endif

// InitLogFile is declared in core.h


// =========================================================================
// DLL entry point — graceful worker shutdown on unload
// =========================================================================

// Vectored-handler registration (B7). Defined here because DllMain, above the
// handler itself, unregisters it on process detach.
static PVOID g_vehHandle = NULL;


BOOL APIENTRY DllMain(HMODULE hModule, DWORD reason, LPVOID reserved)
{
	if (reason == DLL_PROCESS_DETACH && g_vehHandle)
	{
		// B7: unregister before the code behind the handler can go away.
		RemoveVectoredExceptionHandler(g_vehHandle);
		g_vehHandle = NULL;
	}
#if NMCACHE_STEP >= 4
	if (reason == DLL_PROCESS_DETACH)
	{
		// Signal only, never wait. The bounded join belongs in the NavMesh::stop
		// hook, which runs on a game thread well before process detach; waiting
		// here would block under the loader lock. At NMFIX 8 the event is
		// manual-reset, so one set releases every worker and stays signalled.
		g_workerShutdown = 1;
		if (g_jobEvent)
			SetEvent(g_jobEvent);

		if (g_jobEvent)
		{
			CloseHandle(g_jobEvent);
			g_jobEvent = NULL;
		}
	}
#endif // NMCACHE_STEP >= 4
	return TRUE;
}


// =========================================================================
// Crash handler — record-only (B7)
// =========================================================================
//
// A vectored handler runs in the faulting thread's context, before any frame
// unwinds, so everything it touches must be safe in that context. It therefore
// uses no CRT, no heap, no lock and no logging: the text is built with the
// small formatter below into a stack buffer and written with CreateFileA /
// WriteFile. The crash file's path is resolved once in startPlugin, so the
// handler never calls GetModuleFileNameA (loader lock) while faulting.
//
// It always returns EXCEPTION_CONTINUE_SEARCH: the record is a diagnostic, and
// RE_Kenshi's own handler still produces the minidump players send us.
//
// Faults inside the mod's own __try blocks never reach here — see
// GuardEnter/GuardLeave in core.h.

static char   g_crashFilePath[MAX_PATH] = { 0 };
static volatile LONG g_crashSeq = 0;

// --- crash-context formatter: fixed buffer, no CRT, no allocation ---

struct CrashBuf
{
	char   b[1024];
	size_t n;
};

static void cbChar(CrashBuf* o, char c)
{
	if (o->n + 1 < sizeof(o->b))
		o->b[o->n++] = c;
}

static void cbStr(CrashBuf* o, const char* s)
{
	while (s && *s)
		cbChar(o, *s++);
}

static void cbHex(CrashBuf* o, unsigned __int64 v, int digits)
{
	static const char* kHex = "0123456789ABCDEF";
	if (digits < 1) digits = 1;
	if (digits > 16) digits = 16;
	for (int i = digits - 1; i >= 0; --i)
		cbChar(o, kHex[(size_t)((v >> (i * 4)) & 0xF)]);
}

static void cbDec(CrashBuf* o, unsigned __int64 v)
{
	char tmp[24];
	int n = 0;
	if (v == 0)
		tmp[n++] = '0';
	while (v > 0 && n < 24)
	{
		tmp[n++] = (char)('0' + (int)(v % 10));
		v /= 10;
	}
	while (n > 0)
		cbChar(o, tmp[--n]);
}

static void cbReg(CrashBuf* o, const char* name, unsigned __int64 v)
{
	cbStr(o, name);
	cbStr(o, "=0x");
	cbHex(o, v, 16);
	cbChar(o, ' ');
}

static LONG WINAPI NavMeshCrashHandler(PEXCEPTION_POINTERS pExInfo)
{
	// Our own guarded faults are not crashes.
	if (InOurGuard())
		return EXCEPTION_CONTINUE_SEARCH;

	if (!pExInfo || !pExInfo->ExceptionRecord || !pExInfo->ContextRecord || !gameBase)
		return EXCEPTION_CONTINUE_SEARCH;

	DWORD code = pExInfo->ExceptionRecord->ExceptionCode;
	if (code != EXCEPTION_ACCESS_VIOLATION
	    && code != EXCEPTION_INT_DIVIDE_BY_ZERO
	    && code != EXCEPTION_STACK_OVERFLOW)
		return EXCEPTION_CONTINUE_SEARCH;

	// Every early-out before the sequence increment, so a fault before the
	// path is known (or of a code we don't record) never consumes the PROD
	// build's one-record budget (g_crashSeq > 1 refuses every later fault,
	// this one included, for the rest of the process).
	if (!g_crashFilePath[0])
		return EXCEPTION_CONTINUE_SEARCH;

	LONG seq = InterlockedIncrement(&g_crashSeq);
#ifndef ZONEOPT_DEBUG
	// PROD: one record per process. RE_Kenshi writes its own dumps, and a fault
	// storm must not turn into a write storm from a faulting thread.
	if (seq > 1)
		return EXCEPTION_CONTINUE_SEARCH;
#endif

	uintptr_t addr = (uintptr_t)pExInfo->ExceptionRecord->ExceptionAddress;
	CONTEXT* c = pExInfo->ContextRecord;

	CrashBuf o;
	o.n = 0;

	cbStr(&o, "[ZoneOpt] CRASH #");
	cbDec(&o, (unsigned __int64)(unsigned long)seq);
	cbStr(&o, ": code=0x");
	cbHex(&o, (unsigned __int64)code, 8);
	cbStr(&o, " addr=0x");
	cbHex(&o, (unsigned __int64)addr, 16);
	cbStr(&o, " rva=0x");
	cbHex(&o, (unsigned __int64)(addr - gameBase), 8);
	cbStr(&o, "\r\n  ");
	cbReg(&o, "RAX", c->Rax); cbReg(&o, "RBX", c->Rbx);
	cbReg(&o, "RCX", c->Rcx); cbReg(&o, "RDX", c->Rdx);
	cbStr(&o, "\r\n  ");
	cbReg(&o, "R8 ", c->R8);  cbReg(&o, "R9 ", c->R9);
	cbReg(&o, "R10", c->R10); cbReg(&o, "R11", c->R11);
	cbStr(&o, "\r\n  ");
	cbReg(&o, "RSP", c->Rsp); cbReg(&o, "RBP", c->Rbp);
	cbReg(&o, "RSI", c->Rsi); cbReg(&o, "RDI", c->Rdi);
	cbStr(&o, "\r\n  ");
#if NMCACHE_STEP >= 1
	cbStr(&o, "busy=");
	cbDec(&o, (unsigned __int64)(unsigned long)InterlockedCompareExchange(&workerBusyCount, 0, 0));
	cbStr(&o, " slabHook=");
	cbDec(&o, (unsigned __int64)(unsigned long)InterlockedCompareExchange(&g_slabAllocHits, 0, 0));
	cbChar(&o, '/');
	cbDec(&o, (unsigned __int64)(unsigned long)InterlockedCompareExchange(&g_slabAllocWorkerHits, 0, 0));
	cbChar(&o, ' ');
#endif
	cbStr(&o, "crashTid=");
	cbDec(&o, (unsigned __int64)GetCurrentThreadId());
	cbStr(&o, " bgTid=");
	cbDec(&o, (unsigned __int64)g_navMeshBgThreadId);
	// N sets g_navMeshStopSeen from the NavMesh::stop hook when the game
	// begins tearing the navmesh system down; tag records written after that
	// point instead of suppressing them (they are still real crashes, just in
	// a context where the navmesh system is already going away).
	if (InterlockedCompareExchange(&g_navMeshStopSeen, 0, 0) != 0)
		cbStr(&o, " afterStop=1");
	cbStr(&o, "\r\n");

	// First record truncates, later ones (DEV only) append.
	HANDLE hFile = CreateFileA(g_crashFilePath, GENERIC_WRITE, FILE_SHARE_READ, NULL,
		(seq == 1) ? CREATE_ALWAYS : OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
	if (hFile != INVALID_HANDLE_VALUE)
	{
		if (seq != 1)
			SetFilePointer(hFile, 0, NULL, FILE_END);
		DWORD written = 0;
		WriteFile(hFile, o.b, (DWORD)o.n, &written, NULL);
		CloseHandle(hFile);
	}

	return EXCEPTION_CONTINUE_SEARCH;
}


// =========================================================================
// Startup banner
// =========================================================================
//
// Emitted on both paths: after a normal install, and after a refused one, so a
// player's log always says which it was. gateTok is "ok(<n> sites, <k> shared)"
// or "FAILED"; <k> counts sites another plugin had already hooked.

static void LogInitBanner(int installed, int totalHooks, const std::string& gateTok)
{
	std::ostringstream msg;
	msg << "[ZoneOpt] Initialized - " << installed << "/" << totalHooks << " hooks installed"
	    << ", deferral=" << (deferralEnabled ? "ON" : "OFF")
	    << ", priorityBoost=" << (priorityBoostEnabled ? "ON" : "OFF")
	    << ", preload=" << (preloadEnabled ? "ON" : "OFF")
	    << ", movementAware=" << (movementAwareEnabled ? "ON" : "OFF")
	    << ", caching=" << (cachingEnabled ? "ON" : "OFF")
#ifndef ZONEOPT_ZONEONLY
	    << ", cohesion=" << (groupCohesionEnabled ? "ON" : "OFF")
#endif
#if PATHFIND_STEP >= 1
	    << ", pathDiag=" << (pathfindDiagEnabled ? "ON" : "OFF")
	    << ", squadCache=" << (squadPathCacheEnabled ? "ON" : "OFF")
	    << ", pathStep=" << PATHFIND_STEP
#endif
#if PATHFIND_STEP >= 2
	    << ", stuckRetry=" << (stuckRetryEnabled ? "ON" : "OFF")
#endif
	    << ", islandFix=" << (islandFixEnabled ? "ON" : "OFF")
	    << ", islandStep=" << ISLAND_STEP
	    << ", islandModRadius=" << cfg_islandModRadius
	    // Round 2 final review Important #3: printed unconditionally (every
	    // variant defines all three via config.h's #ifndef defaults) so the
	    // banner itself proves what a build actually carries -- a plain
	    // step-4 build with no gate variables set used to compile out P
	    // (PathPool) and Z (Preload pipeline) with nothing in the log to show it.
	    << ", preloadStep=" << PRELOAD_STEP
	    << ", pathpoolStep=" << PATHPOOL_STEP
	    << ", nmfixStep=" << NMFIX_STEP
#ifdef ZONEOPT_ZONEONLY
	    << " [ZONE-ONLY BUILD]"
#endif
	;
	msg << ", gate=" << gateTok;
	DebugLog(msg.str());
	LogMsg(msg.str());
}


// =========================================================================
// Plugin entry point
// =========================================================================

__declspec(dllexport) void startPlugin()
{
	gameBase = (uintptr_t)GetModuleHandleA(NULL);

	// B7: pin our module. A vectored handler stays registered process-wide; if
	// this DLL were ever unloaded with the registration live, the next fault
	// would jump into freed memory. Pinning makes the unload impossible.
	{
		HMODULE self = NULL;
		GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_PIN |
			GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS,
			(LPCSTR)&NavMeshCrashHandler, &self);
	}

	// Resolve the crash file path here, on the main thread, so the handler
	// never touches the loader in a fault context.
	{
		std::string crashPath = GetDLLDirectory() + "crash_dump.txt";
		if (crashPath.size() < sizeof(g_crashFilePath))
			memcpy(g_crashFilePath, crashPath.c_str(), crashPath.size() + 1);
	}

	g_vehHandle = AddVectoredExceptionHandler(1, NavMeshCrashHandler);
	QueryPerformanceFrequency(&qpcFrequency);
	QueryPerformanceCounter(&pluginStartTime);
	InitLogFile();
	LoadConfig(GetDLLDirectory());
	SetCoreGameBase(gameBase);

	// ---------------------------------------------------------------
	// Build gate (B1): nothing is installed on a binary we don't know
	// ---------------------------------------------------------------
	//
	// Two checks, in order. The version tells us whether KenshiLib's tables and
	// ours are talking about the same executable; the prologue pass proves it
	// site by site. A mod that writes 5-byte jumps into 20 hardcoded addresses
	// has to do this: on a mismatched build the jumps land mid-instruction and
	// the game dies somewhere unrelated, hours later.
	std::string gateToken = "FAILED";
	bool gateOk = true;
	bool gogUnsupported = false;
	{
		KenshiLib::BinaryVersion ver = KenshiLib::GetKenshiVersion();
		KenshiLib::BinaryVersion::KenshiPlatform plat = ver.GetPlatform();
		bool versionOk = (ver.GetVersion() == "1.0.65")
			&& (plat == KenshiLib::BinaryVersion::STEAM
			 || plat == KenshiLib::BinaryVersion::GOG);

		LogMsg("[ZoneOpt] Kenshi build: " + ver.ToString() +
			(versionOk ? " (supported)" : " (UNSUPPORTED)"));

		if (!versionOk)
			gateOk = false;
		else if (plat == KenshiLib::BinaryVersion::GOG)
		{
			// GOG 1.0.65 passes the version check above (it's a supported
			// KenshiLib build), but g_hookPrologues is Steam's table: every
			// row would mismatch and the refusal below would read as
			// corruption ("20/20 prologues mismatched") instead of naming the
			// real reason. Say the real reason and skip the (pointless, all-
			// mismatch) prologue pass entirely.
			LogMsg("[ZoneOpt] Build gate: the prologue table is Steam's; GOG "
			       "1.0.65 is not supported until KenshiLib covers every site (B2)");
			gogUnsupported = true;
			gateOk = false;
		}
	}

	int gateBad = 0;
	int gateShared = 0;
	int gateDiagBad = 0;
	std::ostringstream gateDiagNames;
	if (!gogUnsupported)
	{
		for (int i = 0; i < g_hookPrologueCount; ++i)
		{
			bool shared = false;
			if (!VerifyPrologue(g_hookPrologues[i].rva,
			                    g_hookPrologues[i].bytes,
			                    g_hookPrologues[i].name,
			                    &shared))
			{
				// A diagnostic site is not worth refusing to install over: turn that
				// one diagnostic off and carry on. Everything else is fatal.
				if (g_hookPrologues[i].diagnostic)
				{
					if (gateDiagBad > 0)
						gateDiagNames << ", ";
					gateDiagNames << g_hookPrologues[i].name;
					gateDiagBad++;
				}
				else
					gateBad++;
			}
			else if (shared)
				gateShared++;
		}
		if (gateBad > 0)
			gateOk = false;
		if (gateDiagBad > 0)
		{
			// Named, not blamed on a specific flag: today's only diagnostic
			// rows are the PathPool Step 1 hooks (game.cpp), not
			// destroyListDiag -- destroyListInsert is fatal, not diagnostic,
			// since B10's mitigation shipped. Each affected hook refuses its
			// own install at its call site below (VerifyPrologueByRva is
			// checked again there); nothing here needs to flip a flag.
			std::ostringstream ds;
			ds << "[ZoneOpt] Build gate: " << gateDiagBad
			   << " diagnostic site(s) mismatched (" << gateDiagNames.str()
			   << ") — each refuses its own install, the rest of the plugin "
			      "installs normally";
			LogMsg(ds.str());
		}
	}

	if (!gateOk)
	{
		std::ostringstream gs;
		if (gogUnsupported)
			gs << "[ZoneOpt] Build gate failed: GOG platform not supported yet (B2)";
		else
			gs << "[ZoneOpt] Build gate failed: installing nothing ("
			   << gateBad << "/" << g_hookPrologueCount << " prologues mismatched)";
		LogMsg(gs.str());

		// Every feature off, so anything that reads a flag later (and the
		// banner below) sees a plugin that does nothing at all.
		deferralEnabled      = false;
		priorityBoostEnabled = false;
		preloadEnabled       = false;
		movementAwareEnabled = false;
		cachingEnabled       = false;
		islandFixEnabled     = false;
#ifndef ZONEOPT_ZONEONLY
		groupCohesionEnabled = false;
#endif
#if PATHFIND_STEP >= 1
		pathfindDiagEnabled   = false;
#ifdef ZONEOPT_SQUAD_CACHE
		squadPathCacheEnabled = false;
#endif
#endif
#if PATHFIND_STEP >= 2
		stuckRetryEnabled = false;
#endif
		LogInitBanner(0, 0, "FAILED");
		return;
	}

	{
		// gateToken keeps the banner's grepped "gate=ok(...)" token exactly as
		// it was; only the standalone log line's wording changes (it used to
		// read "Build gate: ok, ok(...)", with "ok" twice).
		std::ostringstream detail;
		detail << g_hookPrologueCount << " sites, " << gateShared << " shared";
		gateToken = "ok(" + detail.str() + ")";
		LogMsg("[ZoneOpt] Build gate: ok (" + detail.str() + ")");
	}

#if PATHFIND_STEP >= 5
	// Cross-TU ABI sanity: WatchedCharacter / SchedMoverInfo layouts depend on
	// PATHFIND_STEP. If a future build ships TUs compiled with mismatched
	// PATHFIND_STEP values, reads/writes land on different field offsets and
	// silently corrupt. VS2010 v100 has no static_assert; emit at startup so
	// any drift surfaces in the log immediately.
	{
		std::ostringstream ss;
		ss << "[ZoneOpt] sizeof(WatchedCharacter)=" << sizeof(WatchedCharacter)
		   << " sizeof(SchedMoverInfo)=" << sizeof(SchedMoverInfo)
		   << " (PATHFIND_STEP=" << PATHFIND_STEP << ")";
		LogMsg(ss.str());
	}
#endif

	// Initialize all function pointers from game RVAs
	InitGameBindings(gameBase);

#if PATHFIND_STEP >= 6
	InitReqCharMap();
#endif

#if NMCACHE_STEP >= 1
#if NMFIX_STEP >= 8
	// Manual reset (NMFIX 8): the event means "the queue may be non-empty", not
	// "one job is waiting". Every observer of a non-empty queue sets it and only
	// a worker that finds the queue empty under the queue lock clears it, so a
	// burst of jobs wakes every idle worker instead of exactly one. DllMain's
	// shutdown SetEvent then wakes all of them and stays signalled.
	g_jobEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
#else
	g_jobEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
#endif
	InitNavMeshCacheCS();
	// processJobCS exists from here on; the save-load reset hook may take it.
	NavMeshMarkProcessJobLockReady();
#endif

	ClearPreloadState();

	int installed = 0;

	// Phase 1 hooks
	if (KenshiLib::SUCCESS == KenshiLib::AddHook(
			GameAddr(RVA_SHOW_LOADING_MESSAGE),
			hook_showLoadingMessage, &orig_showLoadingMessage))
		installed++;
	else
		ErrorLog("[ZoneOpt] FAILED to hook showLoadingMessage");

	if (KenshiLib::SUCCESS == KenshiLib::AddHook(
			GameAddr(RVA_IS_CONTENT_PENDING),
			hook_isContentPending, &orig_isContentPending))
		installed++;
	else
	{
		ErrorLog("[ZoneOpt] FAILED to hook isContentPending");
		preloadEnabled = false;  // M2: promotion requires isContentPending
	}

	// Phase 2 hooks
	if (KenshiLib::SUCCESS == KenshiLib::AddHook(
			GameAddr(RVA_UPDATE_CAMERA_ZONE),
			hook_updateCameraZone, &orig_updateCameraZone))
		installed++;
	else
		ErrorLog("[ZoneOpt] FAILED to hook updateCameraZone");

	int totalHooks = 3;

	// GameWorld::destroyListOE inserter. Two jobs on one hook: the crash-3
	// mitigation (destroyListDefer — queue off-main inserts and replay them on
	// the main thread, core.h) and the thread-id diagnostic (destroyListDiag).
	// Either key alone justifies the install; the mitigation is on by default.
	// B1: VerifyPrologue before the patch, and the table row is fatal.
	SetDestroyListDefer(destroyListDeferEnabled);
	if (destroyListDiagEnabled || destroyListDeferEnabled)
	{
		totalHooks++;
		if (VerifyPrologueByRva(RVA_DESTROYLIST_INSERT)
			&& KenshiLib::SUCCESS == KenshiLib::AddHook(
				GameAddr(RVA_DESTROYLIST_INSERT),
				hook_destroyListInsert, &orig_destroyListInsert))
		{
			installed++;
			std::ostringstream dls;
			dls << "[ZoneOpt] destroyListOE inserter hook installed: defer="
			    << (destroyListDeferEnabled ? "on" : "off")
			    << " diag=" << (destroyListDiagEnabled ? "on" : "off");
			LogMsg(dls.str());
		}
		else
		{
			orig_destroyListInsert = NULL;
			SetDestroyListDefer(false);
			ErrorLog("[ZoneOpt] FAILED to hook destroyListOE inserter (sub_140799BE0)");
		}
	}

	// Round 2 fix 2b: the save-load reset's Set A/B unload (sub_14036C1E0). The
	// hook unloads the mod's zones the reset would otherwise leave alive with a
	// wiped handle registry (the Round 2 save-load crash), then clears the
	// mod's state there. Installed in every build: the zones it unloads exist
	// only when the mod preloads, but with none it only scans and logs. The
	// INI key saveLoadUnload chooses unload vs count-only, not the install.
	// B1: VerifyPrologue before the patch, and the table row is fatal.
	totalHooks++;
	if (VerifyPrologueByRva(RVA_RESET_UNLOAD_ZONES)
		&& KenshiLib::SUCCESS == KenshiLib::AddHook(
			GameAddr(RVA_RESET_UNLOAD_ZONES),
			hook_resetUnloadZones, &orig_resetUnloadZones))
	{
		installed++;
		LogMsg(std::string("[ZoneOpt] Save-load reset hook installed: saveLoadUnload=")
		       + (saveLoadUnloadEnabled ? "on" : "off"));
	}
	else
	{
		orig_resetUnloadZones = NULL;
		ErrorLog("[ZoneOpt] FAILED to hook the save-load reset unload (sub_14036C1E0); "
		         "the ZM+8 edge still clears the mod's state, and the registry guard "
		         "refuses surviving zones");
	}

	// Phase 3 hooks
	if (movementAwareEnabled)
	{
		totalHooks++;
		if (KenshiLib::SUCCESS == KenshiLib::AddHook(
				GameAddr(RVA_ADD_ORDER_SELECTED),
				hook_addOrderSelected, &orig_addOrderSelected))
			installed++;
		else
			ErrorLog("[ZoneOpt] FAILED to hook addOrderSelectedCharacters");
	}

#if ISLAND_STEP >= 1
	// Phase 15: island routing overlay. Both hooks or neither go live —
	// either one alone leaves a stall symptom unfixed or parks a squad at the
	// destination zone's boundary. islandFix=false keeps them passing through.
	if (preloadEnabled)
	{
		totalHooks += 2;
		int islandInstalled = 0;

		if (KenshiLib::SUCCESS == KenshiLib::AddHook(
				GameAddr(RVA_ISINISLAND_IMPL),
				hook_isInIsland, &orig_isInIsland))
			{ installed++; islandInstalled++; }
		else
			ErrorLog("[ZoneOpt] FAILED to hook ZoneMap::isInIsland");

		if (KenshiLib::SUCCESS == KenshiLib::AddHook(
				GameAddr(RVA_GETISLAND_IMPL),
				hook_getIsland, &orig_getIsland))
			{ installed++; islandInstalled++; }
		else
			ErrorLog("[ZoneOpt] FAILED to hook ZoneManager::getIsland");

		IslandSetHooksInstalled(islandInstalled == 2);

		std::ostringstream is;
		is << "[ZoneOpt] Island routing: "
		   << (islandInstalled == 2 ? "both hooks installed" : "PARTIAL install, overlay disabled")
		   << (IslandHooksLive() ? " (live)" : " (pass-through)")
		   << " step=" << ISLAND_STEP;
		LogMsg(is.str());
	}
#endif // ISLAND_STEP >= 1

#if NMCACHE_STEP >= 1
	if (cachingEnabled)
	{
		totalHooks++;
		KenshiLib::HookStatus cacheHookResult = KenshiLib::AddHook(
				GameAddr(RVA_DISPATCH_JOB),
				hook_dispatchJob, &orig_dispatchJob);
		if (cacheHookResult == KenshiLib::SUCCESS)
		{
			installed++;
			std::ostringstream ds;
			ds << "[ZoneOpt] dispatchJob hook installed"
			   << " orig=" << (void*)orig_dispatchJob
			   << " detour=" << (void*)hook_dispatchJob;
			LogMsg(ds.str());
		}
		else
		{
			ErrorLog("[ZoneOpt] FAILED to hook dispatchJob");
			cachingEnabled = false;
		}

		// The populate hook is no longer installed here. At NMFIX_STEP >= 1 it
		// installs lazily beside the edgeProcess clone-guard on the first
		// dispatch (nm_workers.cpp), where it captures each job's input
		// triangle count for the zero-face rule.
	}
#endif // NMCACHE_STEP >= 1

#ifndef ZONEOPT_ZONEONLY
	// Group cohesion: binary-patch scatter branch
	if (groupCohesionEnabled && orig_addOrderSelected)
	{
		if (!ApplyScatterPatch())
		{
			LogMsg("[ZoneOpt] Scatter patch failed, cohesion disabled");
			groupCohesionEnabled = false;
		}
	}
	else if (groupCohesionEnabled)
	{
		LogMsg("[ZoneOpt] addOrderSelected hook required for cohesion");
		groupCohesionEnabled = false;
	}
#endif // !ZONEOPT_ZONEONLY

#ifndef ZONEOPT_ZONEONLY
	ClearFormationGroups();
#endif

#if PATHFIND_STEP >= 1
	// Step 1: Pathfinding diagnostics (4 hooks: 3 bg thread + requestPath)
	if (pathfindDiagEnabled)
	{
		int diagInstalled = 0;
		totalHooks += 4;

		if (KenshiLib::SUCCESS == KenshiLib::AddHook(
				GameAddr(RVA_CS_FIND_PATH),
				hook_csFindPath, &orig_csFindPath))
			{ installed++; diagInstalled++; }
		else
			ErrorLog("[ZoneOpt] FAILED to hook ContentStream::findPath");

		if (KenshiLib::SUCCESS == KenshiLib::AddHook(
				GameAddr(RVA_CS_CHECK_FACE_CONN),
				hook_csCheckFaceConn, &orig_csCheckFaceConn))
			{ installed++; diagInstalled++; }
		else
			ErrorLog("[ZoneOpt] FAILED to hook checkFaceConnectivity");

		if (KenshiLib::SUCCESS == KenshiLib::AddHook(
				GameAddr(RVA_FIND_PATH_FULL),
				hook_findPathFull, &orig_findPathFull))
			{ installed++; diagInstalled++; }
		else
			ErrorLog("[ZoneOpt] FAILED to hook findPathFull");

		if (KenshiLib::SUCCESS == KenshiLib::AddHook(
				GameAddr(RVA_REQUEST_PATH),
				hook_requestPath, &orig_requestPath))
			{ installed++; diagInstalled++; }
		else
			ErrorLog("[ZoneOpt] FAILED to hook requestPath");

		if (diagInstalled < 4)
			pathfindDiagEnabled = false;

		LogMsg("[ZoneOpt] Pathfinding step 1: " +
		       std::string(diagInstalled == 4 ? "all 4 hooks installed" : "PARTIAL install"));
	}
#endif // PATHFIND_STEP >= 1

#if PATHFIND_STEP >= 1
	// Squad path cache requires pathfinding hooks and group cohesion. H5: dead
	// in every shipped build -- squadPathCacheEnabled is compile-time false
	// without ZONEOPT_SQUAD_CACHE, so every assignment below is guarded with it.
#ifdef ZONEOPT_SQUAD_CACHE
#ifdef ZONEOPT_ZONEONLY
	squadPathCacheEnabled = false;  // no group cohesion in ZONEONLY
#else
	if (squadPathCacheEnabled && (!pathfindDiagEnabled || !groupCohesionEnabled))
	{
		LogMsg("[ZoneOpt] Squad path cache requires pathDiag + cohesion");
		squadPathCacheEnabled = false;
	}
#endif
#endif // ZONEOPT_SQUAD_CACHE
#endif

#if PATHFIND_STEP >= 2
	// Step 2: Priority boost (pathReqSubmit hook)
	if (pathfindDiagEnabled)
	{
		totalHooks += 1;

		if (KenshiLib::SUCCESS == KenshiLib::AddHook(
				GameAddr(RVA_PATH_REQ_SUBMIT),
				hook_pathReqSubmit, &orig_pathReqSubmit))
		{
			installed++;
			LogMsg("[ZoneOpt] Pathfinding step 2: submit hook installed");
		}
		else
			ErrorLog("[ZoneOpt] FAILED to hook PathRequestQueue::submit");
	}
#endif // PATHFIND_STEP >= 2

#if PATHFIND_STEP >= 4
	// Step 4: Squad path cache (findPathFallback hook)
	if (squadPathCacheEnabled && pathfindDiagEnabled)
	{
		totalHooks += 1;

		if (KenshiLib::SUCCESS == KenshiLib::AddHook(
				GameAddr(RVA_CS_FIND_PATH_FALLBACK),
				hook_csFindPathFallback, &orig_csFindPathFallback))
		{
			installed++;
			LogMsg("[ZoneOpt] Pathfinding step 4: fallback hook installed");
		}
		else
			ErrorLog("[ZoneOpt] FAILED to hook findPathFallback");
	}
#endif // PATHFIND_STEP >= 4

#if PATHFIND_STEP >= 9
	// Step 9 (final): extraction-SEH (contentStreamCallee_0x8869) + addInstance timestamp hook
	if (pathfindDiagEnabled)
	{
		totalHooks += 2;

		if (KenshiLib::SUCCESS == KenshiLib::AddHook(
				GameAddr(RVA_CONTENT_STREAM_CALLEE_0X8869),
				hook_contentStreamCallee0x8869, &orig_contentStreamCallee0x8869))
		{
			installed++;
			LogMsg("[ZoneOpt] Pathfinding step 9: contentStreamCallee_0x8869 SEH hook installed");
		}
		else
			ErrorLog("[ZoneOpt] FAILED to hook contentStreamCallee_0x8869");

		if (KenshiLib::SUCCESS == KenshiLib::AddHook(
				GameAddr(RVA_ADD_INSTANCE),
				hook_addInstance, &orig_addInstance))
		{
			installed++;
			LogMsg("[ZoneOpt] Pathfinding step 9: addInstance timestamp hook installed");
		}
		else
			ErrorLog("[ZoneOpt] FAILED to hook hkaiStreamingCollection::addInstance");
	}
#endif // PATHFIND_STEP >= 9

#if PATHPOOL_STEP >= 1
	// Phase 17 Step 1 instrumentation (P): four pass-through hooks, no
	// behaviour change (research/path_worker_pool.md §7). contentStream,
	// dequeueWork and enqueueThreadSafe install unconditionally at this
	// step; gatesUpdateCodes is gated on its own INI key (gatePassDiag)
	// since it exists purely to time gate-code passes.
	{
		bool gateHookWanted = gatePassDiagEnabled;
		int poolInstalled = 0;
		int poolTotal = 3 + (gateHookWanted ? 1 : 0);
		totalHooks += poolTotal;

		if (VerifyPrologueByRva(RVA_CONTENT_STREAM)
			&& KenshiLib::SUCCESS == KenshiLib::AddHook(
				GameAddr(RVA_CONTENT_STREAM),
				hook_contentStream, &orig_contentStream))
			{ installed++; poolInstalled++; }
		else
			ErrorLog("[ZoneOpt] FAILED to hook SectionManager::contentStream (PathPool)");

		if (VerifyPrologueByRva(RVA_DEQUEUE_WORK)
			&& KenshiLib::SUCCESS == KenshiLib::AddHook(
				GameAddr(RVA_DEQUEUE_WORK),
				hook_dequeueWork, &orig_dequeueWork))
			{ installed++; poolInstalled++; }
		else
			ErrorLog("[ZoneOpt] FAILED to hook SectionManager::dequeueWork_threadSafe (PathPool)");

		if (VerifyPrologueByRva(RVA_ENQUEUE_THREAD_SAFE)
			&& KenshiLib::SUCCESS == KenshiLib::AddHook(
				GameAddr(RVA_ENQUEUE_THREAD_SAFE),
				hook_enqueueThreadSafe, &orig_enqueueThreadSafe))
			{ installed++; poolInstalled++; }
		else
			ErrorLog("[ZoneOpt] FAILED to hook PathRequestQueue::enqueue_threadSafe (PathPool)");

		if (gateHookWanted)
		{
			if (VerifyPrologueByRva(RVA_GATES_UPDATE_CODES)
				&& KenshiLib::SUCCESS == KenshiLib::AddHook(
					GameAddr(RVA_GATES_UPDATE_CODES),
					hook_gatesUpdateCodes, &orig_gatesUpdateCodes))
				{ installed++; poolInstalled++; }
			else
			{
				ErrorLog("[ZoneOpt] FAILED to hook Gates__updateCodes (PathPool)");
				gatePassDiagEnabled = false;
			}
		}

		std::ostringstream ps;
		ps << "[ZoneOpt] PathPool Step 1: " << poolInstalled << "/" << poolTotal
		   << " hooks installed npcWaitDiag=" << (npcWaitDiagEnabled ? "on" : "off")
		   << " gatePassDiag=" << (gatePassDiagEnabled ? "on" : "off");
		LogMsg(ps.str());
	}
#endif // PATHPOOL_STEP >= 1

	LogInitBanner(installed, totalHooks, gateToken);

	// Worker threads created lazily on first hook_dispatchJob call
	// (Havok world not yet initialized at plugin load time)
}
