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
// Havok TLS heap allocator (used by navmesh_cache + pathfind_diag)
// =========================================================================

// Called by InitGameBindings to pass gameBase + RVA without core depending on game.h
void SetHavokTlsParams(uintptr_t base, size_t rva);
void* HavokTlsAlloc(size_t size);
void  HavokTlsFree(void* ptr, size_t size);


#endif // KENSHI_ZONE_OPT_CORE_H
