#include "core.h"

// _ReturnAddress: the destroyListOE inserter hook records who called it from
// off the main thread, and the call site is the whole point of the record.
#include <intrin.h>
#pragma intrinsic(_ReturnAddress)


// =========================================================================
// Timing state
// =========================================================================

LARGE_INTEGER qpcFrequency;
LARGE_INTEGER pluginStartTime;

// Set once on first hook_dispatchJob call via InterlockedCompareExchange.
volatile DWORD g_navMeshBgThreadId = 0;

// Set to 1 by the NavMesh::stop hook when the game begins tearing the navmesh
// system down (hook_navMeshStop, nm_workers.cpp, sets it as its first statement,
// before RetireNavMeshWorkers runs). Read by the crash handler (main.cpp) to
// append "afterStop=1" to any crash record written after that point -- tagged,
// not suppressed, since a real mod crash can still happen there (session 4's
// quit-time record at rva 0xE67E9A was ours).
volatile LONG g_navMeshStopSeen = 0;


// =========================================================================
// Our-guard counter (B7) — see core.h
// =========================================================================

__declspec(thread) int g_inOurGuard = 0;


// =========================================================================
// Log utilities
// =========================================================================

static std::string logFilePath;
static std::ofstream logFile;
static CRITICAL_SECTION logCS;
static bool logCSInitialized = false;
static DWORD mainThreadId = 0;

// Guards the deferred destroyListOE insert queue (the mitigation section far
// below). Declared here because InitLogFile is the earliest main-thread point
// in the plugin's life and initialises it alongside logCS; the queue itself
// lives with the rest of the destroy-list code.
static CRITICAL_SECTION deferCS;
static bool deferCSInitialized = false;

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
	if (!deferCSInitialized)
	{
		// Before any hook is installed, so no off-main thread can reach the
		// queue while the lock is still uninitialised.
		InitializeCriticalSection(&deferCS);
		deferCSInitialized = true;
	}
}

bool IsMainThread()
{
	return mainThreadId != 0 && GetCurrentThreadId() == mainThreadId;
}

static void OpenLogFile()
{
	if (logFile.is_open() || logFilePath.empty())
		return;

	logFile.open(logFilePath.c_str(), std::ios::trunc);

#ifdef ZONEOPT_DEBUG
	// Crash 3 lost the last five seconds of the log: KenshiZoneOpt.log stops at
	// 86.809 while RE_Kenshi_log.txt still carries mod lines at 91.686 — exactly
	// the window the analysis needed. LogMsg has always called flush() per line,
	// which only hands the stream buffer to the OS; making the DEV stream
	// unbuffered removes that buffer entirely, so each line reaches the OS inside
	// the write that produced it and nothing is left in user space for a fault to
	// destroy. PROD keeps the buffered stream and the per-line flush.
	//
	// This must come AFTER the open, not before it. MSVC 2010's
	// basic_filebuf::setbuf returns failure immediately when _Myfile == 0, so on
	// an unopened stream it does nothing at all; once the file is open it reaches
	// setvbuf(_Myfile, NULL, _IONBF, 0), which is the call that matters. Nothing
	// has been written to the stream yet at this point, which setvbuf requires.
	if (logFile.is_open())
		logFile.rdbuf()->pubsetbuf(0, 0);
#endif
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
// Build gate (B1) — see core.h
// =========================================================================

static uintptr_t coreGameBase = 0;

void SetCoreGameBase(uintptr_t base)
{
	coreGameBase = base;
}

// Kept standalone: MSVC 2010 rejects __try in a function that also holds
// objects needing unwinding, and everything here is POD.
static bool ReadGameBytes16(const void* addr, unsigned char* out)
{
	bool ok = true;
	GuardEnter();
	__try
	{
		memcpy(out, addr, 16);
	}
	__except (EXCEPTION_EXECUTE_HANDLER)
	{
		ok = false;
	}
	GuardLeave();
	return ok;
}

static void AppendHexBytes(std::ostringstream& ss, const unsigned char* b)
{
	for (int i = 0; i < 16; ++i)
	{
		if (i) ss << ' ';
		ss << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
		   << (unsigned int)b[i];
	}
	ss << std::dec << std::nouppercase << std::setfill(' ');
}

// A site another plugin has already detoured starts with the detour, not with
// the function's own first instruction. MinHook-style patches are either a
// 5-byte `E9 rel32` or a 6-byte `FF 25 rel32` (jmp [rip+disp32]), and MinHook
// may pad with 0x90/0xCC out to the next instruction boundary. The bytes past
// the detour still belong to the real function, so they are what we verify.
// Returns the detour length (5 or 6), or 0 if this is not a detour.
static int DetourLength(const unsigned char* b)
{
	if (b[0] == 0xE9)
		return 5;
	if (b[0] == 0xFF && b[1] == 0x25)
		return 6;
	return 0;
}

// True if the bytes past a detour still match the expected prologue, either
// directly from the detour's end or after 0x90/0xCC padding up to offset 8.
static bool PrologueTailMatches(const unsigned char* actual,
                                const unsigned char* expect, int start)
{
	if (memcmp(actual + start, expect + start, 16 - start) == 0)
		return true;

	// Padding variant: MinHook copied a longer instruction and padded the
	// remainder of it. Only the bytes between the detour and offset 8 may be
	// padding; everything from 8 on must match.
	for (int i = start; i < 8; ++i)
	{
		if (actual[i] != 0x90 && actual[i] != 0xCC)
			return false;
	}
	return memcmp(actual + 8, expect + 8, 8) == 0;
}

bool VerifyPrologue(uintptr_t rva, const unsigned char* expect, const char* name,
                    bool* sharedOut)
{
	if (sharedOut) *sharedOut = false;

	if (coreGameBase == 0 || expect == NULL)
		return false;

	unsigned char actual[16];
	if (!ReadGameBytes16((const void*)(coreGameBase + rva), actual))
	{
		std::ostringstream ss;
		ss << "[ZoneOpt] Build gate: " << (name ? name : "?")
		   << " prologue unreadable at RVA 0x" << std::hex << rva << std::dec;
		LogMsg(ss.str());
		return false;
	}

	if (memcmp(actual, expect, 16) == 0)
		return true;

	// Another plugin loaded first and hooked this site. KenshiLib's AddHook
	// (MinHook) chains onto an existing detour correctly, so this is a pass as
	// long as the untouched tail proves the binary is still the one we know.
	{
		int detour = DetourLength(actual);
		if (detour > 0 && PrologueTailMatches(actual, expect, detour))
		{
			std::ostringstream ss;
			ss << "[ZoneOpt] Build gate: " << (name ? name : "?")
			   << " already hooked by another plugin (detour at RVA 0x"
			   << std::hex << rva << std::dec << "), prologue tail matches";
			LogMsg(ss.str());
			if (sharedOut) *sharedOut = true;
			return true;
		}
	}

	std::ostringstream ss;
	ss << "[ZoneOpt] Build gate: " << (name ? name : "?")
	   << " prologue mismatch at RVA 0x" << std::hex << rva << std::dec
	   << "\n    expected ";
	AppendHexBytes(ss, expect);
	ss << "\n    found    ";
	AppendHexBytes(ss, actual);
	LogMsg(ss.str());
	return false;
}


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


// =========================================================================
// destroyListOE invariant probe (crash 3 diagnostic) — see core.h
// =========================================================================

// Offsets inside the GameWorld singleton (the IDB's `pauseState`, RVA
// 0x21330B0). Both containers are Ogre `boost::unordered` ptr-node sets with
// the same three-field shape, read straight off sub_14079CD00's drain loops
// (decompiled 2026-09-12):
//
//   destroyListOE   base +0x680: bucket_count +0x698, size +0x6A0, buckets +0x6B8
//                   (drain at 0x79CEA5, the loop that faulted at +0x1C1)
//   killListPhase0  base +0x7D0: bucket_count +0x7E8, size +0x7F0, buckets +0x808
//                   (drain at the top of the same function, lines 121-143)
//
// The sentinel node list hangs off buckets[bucket_count] in both, which is the
// pointer the drain dereferences without a null check.
static const size_t OFF_GW_DESTROYOE_NBUCKETS = 0x698;
static const size_t OFF_GW_DESTROYOE_SIZE     = 0x6A0;
static const size_t OFF_GW_DESTROYOE_BUCKETS  = 0x6B8;
static const size_t OFF_GW_KILL0_NBUCKETS     = 0x7E8;
static const size_t OFF_GW_KILL0_SIZE         = 0x7F0;
static const size_t OFF_GW_KILL0_BUCKETS      = 0x808;

// A bucket count past this is garbage, not a container we should index into.
static const unsigned __int64 MAX_SANE_BUCKETS = 0x100000;

static uintptr_t s_gameWorld = 0;

// Inserter instrumentation. Plain volatile LONGs touched only by Interlocked*,
// so the hook needs no lock and no CRT.
static volatile LONG s_lastInsertTid = 0;
static volatile LONG s_insMain       = 0;
static volatile LONG s_insOther      = 0;
static volatile LONG s_ringNext      = 0;
static const int     RING_SIZE       = 8;
static volatile uintptr_t s_ring[RING_SIZE] = { 0 };
static bool          s_ringPrinted   = false;

// Deferred-insert queue (mitigation, core.h). Fixed capacity, no allocation:
// the hook runs inside an engine destructor on a background thread, where a
// heap call is a hazard of its own. Both fields are written only under deferCS.
// The gameWorld pointer is carried per entry rather than assumed constant, so
// the flush replays exactly the call the game made.
struct DeferredInsert
{
	void* gameWorld;
	void* movable;
};
static const int    DEFER_CAP   = 1024;
static DeferredInsert s_defer[DEFER_CAP];
static int          s_deferCount = 0;
static bool         s_deferEnabled = false;

static volatile LONG s_deferred   = 0;
static volatile LONG s_flushed    = 0;
static volatile LONG s_deferFull  = 0;
static volatile LONG s_dedup      = 0;
static volatile LONG s_dropClear  = 0;
static volatile LONG s_deferEnt   = 0;
static volatile LONG s_deferNonEnt = 0;

// Bound on one frame's flush: 8 batches of FLUSH_BATCH. A queue deeper than
// that is drained over the following frames rather than in one hitch, which
// also keeps the worst case of this loop independent of DEFER_CAP.
static const int FLUSH_BATCH    = 32;
static const int FLUSH_BATCHES  = 8;

// Removes every queued entry naming `movable`. deferCS must be held. Returns
// how many were removed. Order in the queue is irrelevant (the destination is
// a hash set), so removal swaps the last entry down instead of shifting.
static int RemoveQueuedLocked(void* movable)
{
	int removed = 0;
	for (int i = 0; i < s_deferCount; )
	{
		if (s_defer[i].movable == movable)
		{
			s_defer[i] = s_defer[--s_deferCount];
			removed++;
		}
		else
		{
			++i;
		}
	}
	return removed;
}

// True if the object's Ogre movable type is "Entity", i.e. the branch of
// sub_140799BE0 that also de-registers the object from the loader's queued and
// preloaded lists (sub_140449240). Safe on any thread: vtable slot 4
// (`getMovableType`) is a const getter that returns a reference to its class's
// static type-name string, and the game itself makes exactly this virtual call
// on exactly this object on this thread at this instant — the deferred path is
// not taking a risk the unmodified binary does not already take. No allocation
// and no CRT beyond memcmp.
static bool MovableTypeIsEntity(void* movable)
{
	if (!movable)
		return false;
	typedef const void* (__fastcall *getMovableType_t)(void*);
	const void* const* vt = *(const void* const* const*)movable;
	if (!vt)
		return false;
	getMovableType_t getType = (getMovableType_t)vt[4];   // +32
	if (!getType)
		return false;

	// MSVC 2010 std::string: [0]=buffer or heap pointer, [2]=size, [3]=capacity;
	// the buffer is out of line once capacity >= 16.
	const size_t* s = (const size_t*)getType(movable);
	if (!s)
		return false;
	size_t len = s[2];
	const char* buf = (s[3] >= 16) ? *(const char* const*)s : (const char*)s;
	// Ogre::EntityFactory::FACTORY_TYPE_NAME. The game compares against the
	// imported global (IAT slot RVA 0x2247F08); comparing against its value
	// avoids an import read for a counter.
	return (len == 6 && buf && memcmp(buf, "Entity", 6) == 0);
}

destroyListInsert_t orig_destroyListInsert = NULL;

void SetDestroyListBase(uintptr_t gameWorld)
{
	s_gameWorld = gameWorld;
}

// Standalone and POD-only: MSVC 2010 rejects __try in a function that also
// holds objects needing unwinding.
static bool ReadContainer(uintptr_t base, size_t nOff, size_t sizeOff, size_t bucketsOff,
                          unsigned __int64* sizeOut, void** headOut)
{
	bool ok = true;
	GuardEnter();
	__try
	{
		unsigned __int64 nbuckets = *(unsigned __int64*)(base + nOff);
		*sizeOut  = *(unsigned __int64*)(base + sizeOff);
		void** buckets = *(void***)(base + bucketsOff);
		*headOut = NULL;
		if (buckets && nbuckets < MAX_SANE_BUCKETS)
			*headOut = buckets[nbuckets];
		else if (nbuckets >= MAX_SANE_BUCKETS)
			ok = false;
	}
	__except (EXCEPTION_EXECUTE_HANDLER)
	{
		ok = false;
	}
	GuardLeave();
	return ok;
}

// size and head are two separate loads, so a concurrent insert between them can
// fake a mismatch. The crash state is permanent (size stays high for good), so a
// second read that still disagrees is the real thing and a transient one is not.
static bool ProbeContainer(const char* name, size_t nOff, size_t sizeOff, size_t bucketsOff,
                           unsigned int frame, bool quiet)
{
	unsigned __int64 size = 0;
	void* head = NULL;
	if (!ReadContainer(s_gameWorld, nOff, sizeOff, bucketsOff, &size, &head))
		return false;

	bool broken = ((size != 0) != (head != NULL));
	if (!broken)
		return false;

	if (!ReadContainer(s_gameWorld, nOff, sizeOff, bucketsOff, &size, &head))
		return false;
	if ((size != 0) == (head != NULL))
		return false;

	if (quiet)
		return false;

	std::ostringstream ss;
	ss << "[ZoneOpt] " << name << " INVARIANT BROKEN: size=" << size
	   << " head=0x" << std::hex << (uintptr_t)head << std::dec
	   << " frame=" << frame
	   << " lastInsertTid=" << (unsigned long)InterlockedCompareExchange(&s_lastInsertTid, 0, 0)
	   << DestroyListStatsSuffix();
	LogMsg(ss.str());
	return true;
}

void DestroyListProbeTick()
{
	// Main thread only, enforced here rather than assumed of the caller: the
	// frame counter and the log latch below are plain statics, and LogMsg's CRT
	// work is only safe on this thread.
	if (!IsMainThread())
		return;
	if (!s_gameWorld)
		return;

	static unsigned int frame = 0;
	frame++;

	// The read runs every frame — the corrupt state appears within a frame of the
	// crash, so a sampled probe would miss it. Only the logging is throttled, to
	// one line a minute: the state is persistent, so the frames after the first
	// detection would all say the same thing (if the game gets that far at all).
	static double lastLogSec = -1.0e9;
	double now = ElapsedSec();
	bool quiet = (now - lastLogSec < 60.0);

	bool logged = ProbeContainer("destroyListOE", OFF_GW_DESTROYOE_NBUCKETS,
	                             OFF_GW_DESTROYOE_SIZE, OFF_GW_DESTROYOE_BUCKETS,
	                             frame, quiet);
	// Control container: the same shape, drained by the same function a few
	// hundred instructions earlier. If both break together the cause is wider
	// than one list.
	if (ProbeContainer("killListPhase0", OFF_GW_KILL0_NBUCKETS,
	                   OFF_GW_KILL0_SIZE, OFF_GW_KILL0_BUCKETS, frame, quiet))
		logged = true;

	if (logged)
		lastLogSec = now;
}

std::string DestroyListStatsSuffix()
{
	LONG main  = InterlockedCompareExchange(&s_insMain, 0, 0);
	LONG other = InterlockedCompareExchange(&s_insOther, 0, 0);

	std::ostringstream ss;
	ss << ", dlIns=" << main << "/" << other
	   << " dlDeferred=" << InterlockedCompareExchange(&s_deferred, 0, 0)
	   << " dlFlushed="  << InterlockedCompareExchange(&s_flushed, 0, 0)
	   << " dlDeferFull=" << InterlockedCompareExchange(&s_deferFull, 0, 0)
	   << " dlDedup=" << InterlockedCompareExchange(&s_dedup, 0, 0)
	   << " dlDropClear=" << InterlockedCompareExchange(&s_dropClear, 0, 0)
	   << " dlDeferEnt=" << InterlockedCompareExchange(&s_deferEnt, 0, 0)
	   << "/" << InterlockedCompareExchange(&s_deferNonEnt, 0, 0);

	// The return addresses are the evidence that settles H1, so print them the
	// first time any exist and never again.
	if (other > 0 && !s_ringPrinted)
	{
		s_ringPrinted = true;
		ss << " dlFrom=";
		LONG count = InterlockedCompareExchange(&s_ringNext, 0, 0);
		if (count > RING_SIZE)
			count = RING_SIZE;
		for (LONG i = 0; i < count; ++i)
		{
			uintptr_t ra = s_ring[i];
			if (i) ss << ",";
			ss << "0x" << std::hex
			   << (uintptr_t)((coreGameBase && ra > coreGameBase) ? ra - coreGameBase : ra)
			   << std::dec;
		}
	}
	return ss.str();
}

void SetDestroyListDefer(bool enabled)
{
	s_deferEnabled = enabled;
}

void DestroyListFlushDeferred()
{
	// Main thread only: the whole point is that the real insert happens on the
	// thread that drains the container.
	if (!IsMainThread() || !deferCSInitialized || !orig_destroyListInsert)
		return;

	// Batched so the lock is held for a pointer copy and nothing else. The
	// original is called with deferCS released, because it runs engine code
	// (a virtual type-name query, setVisible, the hash insert) that must never
	// run under a mod lock a background thread can be waiting on.
	//
	// The queue is drained from the end: order is irrelevant to a hash-set
	// insert, and popping the tail keeps the bookkeeping O(1).
	//
	// Bounded: at most FLUSH_BATCHES batches per frame, leftovers next frame.
	// Only a pathological producer can reach the bound (the worst session on
	// record queued 129 inserts in total), but the cost of this loop should not
	// scale with a queue the mod does not control.
	for (int pass = 0; pass < FLUSH_BATCHES; ++pass)
	{
		DeferredInsert batch[FLUSH_BATCH];
		int taken = 0;

		EnterCriticalSection(&deferCS);
		while (taken < FLUSH_BATCH && s_deferCount > 0)
			batch[taken++] = s_defer[--s_deferCount];
		LeaveCriticalSection(&deferCS);

		if (taken == 0)
			return;

		for (int i = 0; i < taken; ++i)
		{
			// Dedup within the batch. The push path already refuses a pointer
			// the queue holds, so this is unreachable in practice; it is here
			// because inserting one pointer twice is exactly the failure the
			// dedup exists to prevent, and 32 comparisons cost nothing.
			bool dup = false;
			for (int j = 0; j < i; ++j)
			{
				if (batch[j].movable == batch[i].movable)
				{
					dup = true;
					break;
				}
			}
			if (dup)
			{
				InterlockedIncrement(&s_dedup);
				continue;
			}
			orig_destroyListInsert(batch[i].gameWorld, batch[i].movable);
			InterlockedIncrement(&s_flushed);
		}
	}
}

void DestroyListDropDeferred()
{
	// World clear: everything queued names an object the clear is about to
	// destroy, so replaying it would insert a freed pointer into a container
	// the clear has just emptied. Dropping loses nothing — the clear tears the
	// scene down wholesale, which is what the queued inserts were asking for.
	if (!deferCSInitialized)
		return;

	EnterCriticalSection(&deferCS);
	int dropped = s_deferCount;
	s_deferCount = 0;
	LeaveCriticalSection(&deferCS);

	if (dropped > 0)
		InterlockedExchangeAdd(&s_dropClear, (LONG)dropped);
}

__int64 __fastcall hook_destroyListInsert(void* gameWorld, void* movable)
{
	InterlockedExchange(&s_lastInsertTid, (LONG)GetCurrentThreadId());

	if (IsMainThread())
	{
		InterlockedIncrement(&s_insMain);

		// Deferral must not lose the set's own dedup. Vanilla's insert of an
		// object the set already holds is a no-op; with a queue in the way, a
		// main-thread insert of an object still queued would land in the set
		// now, this frame's drain would destroy it, and the flush would then
		// replay a freed pointer. So a main-thread insert claims the object:
		// anything queued for it is removed before the original runs.
		if (s_deferEnabled && deferCSInitialized && movable)
		{
			EnterCriticalSection(&deferCS);
			int removed = RemoveQueuedLocked(movable);
			LeaveCriticalSection(&deferCS);
			if (removed > 0)
				InterlockedExchangeAdd(&s_dedup, (LONG)removed);
		}
	}
	else
	{
		InterlockedIncrement(&s_insOther);
		// The ring keeps the first RING_SIZE off-main-thread call sites; later
		// ones wrap over the oldest. Either way the slot index is unique, so two
		// threads never write the same entry.
		LONG slot = InterlockedIncrement(&s_ringNext) - 1;
		// Unsigned: slot is a LONG and wraps negative after 2^31 inserts, and a
		// negative operand to % would index before the array.
		s_ring[((unsigned long)slot) % RING_SIZE] = (uintptr_t)_ReturnAddress();

		// The mitigation (core.h): queue instead of inserting, so that every
		// write to destroyListOE happens on the thread that drains it. The main
		// thread performs this insert from DestroyListFlushDeferred on the next
		// frame; the object stays alive until a drain reaches it either way.
		if (s_deferEnabled && deferCSInitialized && orig_destroyListInsert)
		{
			// Which branch of the original this insert would have taken. The
			// "Entity" branch also calls sub_140449240, which de-registers the
			// object from the loader's queued (+320/+328) and preloaded
			// (+288/+296) lists; deferring the insert defers that too, by one
			// frame, while the object stays alive. Counted, not worked around.
			if (MovableTypeIsEntity(movable))
				InterlockedIncrement(&s_deferEnt);
			else
				InterlockedIncrement(&s_deferNonEnt);

			bool queued = false;
			bool already = false;
			EnterCriticalSection(&deferCS);
			// Dedup: the set the flush inserts into holds each pointer once, so
			// a second deferral of the same object must not become a second
			// insert. Keeping the queue duplicate-free also means no batch of
			// the flush can ever hold one pointer twice.
			int collapsed = RemoveQueuedLocked(movable);
			if (collapsed > 0)
			{
				// Put it back once: this call still wants the object destroyed.
				s_defer[s_deferCount].gameWorld = gameWorld;
				s_defer[s_deferCount].movable   = movable;
				s_deferCount++;
				queued  = true;
				already = true;
			}
			else if (s_deferCount < DEFER_CAP)
			{
				s_defer[s_deferCount].gameWorld = gameWorld;
				s_defer[s_deferCount].movable   = movable;
				s_deferCount++;
				queued = true;
			}
			LeaveCriticalSection(&deferCS);

			if (already)
				InterlockedExchangeAdd(&s_dedup, (LONG)collapsed);

			if (queued)
			{
				InterlockedIncrement(&s_deferred);
				// The original's return value is the address of its own dead
				// stack temporary and no call site reads it (core.h).
				return 0;
			}
			// Queue full: fall through to the racy original rather than drop
			// the object, which would leak it and leave it visible.
			InterlockedIncrement(&s_deferFull);
		}
	}

	if (!orig_destroyListInsert)
		return 0;
	return orig_destroyListInsert(gameWorld, movable);
}
