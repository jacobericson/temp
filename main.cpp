#include "preload.h"
#include "nm_workers.h"
#include "hooks.h"
#include "formation.h"
#include "pathfinding.h"

// InitLogFile is declared in core.h


// =========================================================================
// DLL entry point — graceful worker shutdown on unload
// =========================================================================

BOOL APIENTRY DllMain(HMODULE hModule, DWORD reason, LPVOID reserved)
{
#if NMCACHE_STEP >= 4
	if (reason == DLL_PROCESS_DETACH)
	{
		g_workerShutdown = 1;
		if (g_jobEvent)
		{
			for (int i = 0; i < NAVMESH_WORKER_COUNT; ++i)
				SetEvent(g_jobEvent);
		}

		int activeCount = 0;
		HANDLE active[NAVMESH_WORKER_COUNT];
		for (int i = 0; i < NAVMESH_WORKER_COUNT; ++i)
		{
			if (g_workerHandles[i])
				active[activeCount++] = g_workerHandles[i];
		}
		if (activeCount > 0)
			WaitForMultipleObjects(activeCount, active, TRUE, 3000);

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
// Crash handler — captures crash RVA + writes minidump
// =========================================================================

static LONG WINAPI NavMeshCrashHandler(PEXCEPTION_POINTERS pExInfo)
{
	if (pExInfo && pExInfo->ExceptionRecord && gameBase)
	{
		DWORD code = pExInfo->ExceptionRecord->ExceptionCode;
		if (code == EXCEPTION_ACCESS_VIOLATION || code == EXCEPTION_INT_DIVIDE_BY_ZERO
		    || code == EXCEPTION_STACK_OVERFLOW || code == 0xC0000005)
		{
			uintptr_t addr = (uintptr_t)pExInfo->ExceptionRecord->ExceptionAddress;
			uintptr_t rva = addr - gameBase;

			// Write crash info using bare Win32 (safe in crash context)
			char buf[512];
#if NMCACHE_STEP >= 1
			sprintf_s(buf, sizeof(buf),
				"[ZoneOpt] CRASH: code=0x%08X addr=0x%p rva=0x%llX\r\n"
				"  RAX=0x%p RBX=0x%p RCX=0x%p RDX=0x%p\r\n"
				"  R8=0x%p  R9=0x%p  R10=0x%p R11=0x%p\r\n"
				"  RSP=0x%p RBP=0x%p RSI=0x%p RDI=0x%p\r\n"
				"  busy=%ld slabHook=%ld/%ld\r\n",
				code, (void*)addr, (unsigned long long)rva,
				(void*)pExInfo->ContextRecord->Rax, (void*)pExInfo->ContextRecord->Rbx,
				(void*)pExInfo->ContextRecord->Rcx, (void*)pExInfo->ContextRecord->Rdx,
				(void*)pExInfo->ContextRecord->R8, (void*)pExInfo->ContextRecord->R9,
				(void*)pExInfo->ContextRecord->R10, (void*)pExInfo->ContextRecord->R11,
				(void*)pExInfo->ContextRecord->Rsp, (void*)pExInfo->ContextRecord->Rbp,
				(void*)pExInfo->ContextRecord->Rsi, (void*)pExInfo->ContextRecord->Rdi,
				InterlockedCompareExchange(&workerBusyCount, 0, 0),
				InterlockedCompareExchange(&g_slabAllocHits, 0, 0),
				InterlockedCompareExchange(&g_slabAllocWorkerHits, 0, 0));
#else
			sprintf_s(buf, sizeof(buf),
				"[ZoneOpt] CRASH: code=0x%08X addr=0x%p rva=0x%llX\r\n"
				"  RAX=0x%p RBX=0x%p RCX=0x%p RDX=0x%p\r\n"
				"  R8=0x%p  R9=0x%p  R10=0x%p R11=0x%p\r\n"
				"  RSP=0x%p RBP=0x%p RSI=0x%p RDI=0x%p\r\n",
				code, (void*)addr, (unsigned long long)rva,
				(void*)pExInfo->ContextRecord->Rax, (void*)pExInfo->ContextRecord->Rbx,
				(void*)pExInfo->ContextRecord->Rcx, (void*)pExInfo->ContextRecord->Rdx,
				(void*)pExInfo->ContextRecord->R8, (void*)pExInfo->ContextRecord->R9,
				(void*)pExInfo->ContextRecord->R10, (void*)pExInfo->ContextRecord->R11,
				(void*)pExInfo->ContextRecord->Rsp, (void*)pExInfo->ContextRecord->Rbp,
				(void*)pExInfo->ContextRecord->Rsi, (void*)pExInfo->ContextRecord->Rdi);
#endif

			// Append to log file
			LogMsg(buf);

			// Also write to a separate crash file for reliability
			HMODULE hm = NULL;
			GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
				GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
				(LPCSTR)&NavMeshCrashHandler, &hm);
			char path[MAX_PATH];
			GetModuleFileNameA(hm, path, sizeof(path));
			char* lastSlash = path;
			for (char* p = path; *p; ++p)
				if (*p == '\\' || *p == '/') lastSlash = p + 1;
			strcpy_s(lastSlash, sizeof(path) - (lastSlash - path), "crash_dump.txt");

			HANDLE hFile = CreateFileA(path, GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, 0, NULL);
			if (hFile != INVALID_HANDLE_VALUE)
			{
				DWORD written;
				WriteFile(hFile, buf, (DWORD)strlen(buf), &written, NULL);
				CloseHandle(hFile);
			}
		}
	}
	return EXCEPTION_CONTINUE_SEARCH;
}


// =========================================================================
// Plugin entry point
// =========================================================================

__declspec(dllexport) void startPlugin()
{
	gameBase = (uintptr_t)GetModuleHandleA(NULL);
	AddVectoredExceptionHandler(1, NavMeshCrashHandler);
	QueryPerformanceFrequency(&qpcFrequency);
	QueryPerformanceCounter(&pluginStartTime);
	InitLogFile();
	LoadConfig(GetDLLDirectory());

	// Initialize all function pointers from game RVAs
	InitGameBindings(gameBase);

#if NMCACHE_STEP >= 1
	g_jobEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
	InitNavMeshCacheCS();
#endif
#if NMCACHE_STEP >= 4
	InitScratchTLS();
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

	// Phase 3 hooks
	int totalHooks = 3;
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

#if NMCACHE_STEP >= 5
		if (cachingEnabled)
		{
			totalHooks++;
			if (KenshiLib::SUCCESS == KenshiLib::AddHook(
					GameAddr(RVA_NM_RESULT_POPULATE),
					hook_nmResultPopulate_diag, &orig_nmResultPopulate))
			{
				installed++;
				LogMsg("[ZoneOpt] populate hook installed");
			}
			else
			{
				ErrorLog("[ZoneOpt] FAILED to hook nmResultPopulate");
			}
		}
#endif // NMCACHE_STEP >= 5
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
	// Squad path cache requires pathfinding hooks and group cohesion
#ifdef ZONEOPT_ZONEONLY
	squadPathCacheEnabled = false;  // no group cohesion in ZONEONLY
#else
	if (squadPathCacheEnabled && (!pathfindDiagEnabled || !groupCohesionEnabled))
	{
		LogMsg("[ZoneOpt] Squad path cache requires pathDiag + cohesion");
		squadPathCacheEnabled = false;
	}
#endif
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
#ifdef ZONEOPT_ZONEONLY
	    << " [ZONE-ONLY BUILD]"
#endif
	;
	DebugLog(msg.str());
	LogMsg(msg.str());

	// Worker threads created lazily on first hook_dispatchJob call
	// (Havok world not yet initialized at plugin load time)
}
