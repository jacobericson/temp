#include "core.h"


// =========================================================================
// Timing state
// =========================================================================

LARGE_INTEGER qpcFrequency;
LARGE_INTEGER pluginStartTime;

// Set once on first hook_dispatchJob call via InterlockedCompareExchange.
volatile DWORD g_navMeshBgThreadId = 0;


// =========================================================================
// Log utilities
// =========================================================================

static std::string logFilePath;
static std::ofstream logFile;
static CRITICAL_SECTION logCS;
static bool logCSInitialized = false;
static DWORD mainThreadId = 0;

std::string GetDLLDirectory()
{
	char path[MAX_PATH];
	HMODULE hm = NULL;
	if (!GetModuleHandleExA(
		GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
		GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
		(LPCSTR)&GetDLLDirectory, &hm))
	{
		return std::string();
	}
	GetModuleFileNameA(hm, path, sizeof(path));
	std::string dir(path);
	size_t pos = dir.find_last_of("\\/");
	if (pos != std::string::npos)
		dir = dir.substr(0, pos + 1);
	return dir;
}

void InitLogFile()
{
	logFilePath = GetDLLDirectory() + "KenshiZoneOpt.log";
	if (!logCSInitialized)
	{
		InitializeCriticalSection(&logCS);
		logCSInitialized = true;
		mainThreadId = GetCurrentThreadId();
	}
}

bool IsMainThread()
{
	return mainThreadId != 0 && GetCurrentThreadId() == mainThreadId;
}

static void OpenLogFile()
{
	if (!logFile.is_open() && !logFilePath.empty())
		logFile.open(logFilePath.c_str(), std::ios::trunc);
}

static void LogLine(std::ofstream& file, const std::string& line)
{
	// Only call DebugLog on main thread (RE_Kenshi thread safety unknown)
	if (GetCurrentThreadId() == mainThreadId)
		DebugLog(line);

	std::ostringstream ts;
	ts << std::fixed << std::setprecision(3) << ElapsedSec() << ": " << line;
	file << ts.str() << "\n";
}

void LogMsg(const std::string& line)
{
	if (!logCSInitialized)
		return;
	EnterCriticalSection(&logCS);
	OpenLogFile();
	if (logFile.is_open())
	{
		LogLine(logFile, line);
		logFile.flush();
	}
	LeaveCriticalSection(&logCS);
}

#ifdef ZONEOPT_DEBUG
void LogDebug(const std::string& line)
{
	LogMsg(line);
}
#endif


// =========================================================================
// Havok TLS heap allocator (used by navmesh_cache + pathfind_diag)
// =========================================================================

// Forward-declared here; gameBase and RVA_HAVOK_TLS_INDEX come from game.h,
// but core.cpp cannot include game.h (Layer 0 has no game knowledge).
// Instead, these are set by game.cpp's InitGameBindings and stored here.
static uintptr_t s_gameBase = 0;
static size_t s_havokTlsRva = 0;

void SetHavokTlsParams(uintptr_t base, size_t rva)
{
	s_gameBase = base;
	s_havokTlsRva = rva;
}

void* HavokTlsAlloc(size_t size)
{
	if (!s_gameBase || !s_havokTlsRva)
		return NULL;
	DWORD tlsIdx = *(DWORD*)(s_gameBase + s_havokTlsRva);
	uintptr_t* allocs = (uintptr_t*)TlsGetValue(tlsIdx);
	if (!allocs) return NULL;
	uintptr_t allocObj = allocs[11];
	if (!allocObj) return NULL;
	uintptr_t vtable = *(uintptr_t*)allocObj;
	typedef void* (__fastcall *BlockAllocFn)(uintptr_t, int);
	BlockAllocFn fn = (BlockAllocFn)(*(uintptr_t*)(vtable + 8));
	return fn(allocObj, (int)size);
}

void HavokTlsFree(void* ptr, size_t size)
{
	if (!ptr || !s_gameBase || !s_havokTlsRva)
		return;
	DWORD tlsIdx = *(DWORD*)(s_gameBase + s_havokTlsRva);
	uintptr_t* allocs = (uintptr_t*)TlsGetValue(tlsIdx);
	if (!allocs) return;
	uintptr_t allocObj = allocs[11];
	if (!allocObj) return;
	uintptr_t vtable = *(uintptr_t*)allocObj;
	typedef void (__fastcall *BlockFreeFn)(uintptr_t, void*, int);
	BlockFreeFn fn = (BlockFreeFn)(*(uintptr_t*)(vtable + 16));
	fn(allocObj, ptr, (int)size);
}
