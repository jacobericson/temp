// nm_cache_core.cpp — L1 in-memory navmesh ring buffer cache + stats reporter

#include "nm_cache_core.h"
#if NMCACHE_STEP >= 2
#include "nm_disk_cache.h"   // L2 format constants for the startup self-check
#endif

#if NMCACHE_STEP >= 1

NavMeshCacheEntry nmCache[NM_CACHE_SIZE];
int nmCacheWriteIdx = 0;
int nmCacheFill = 0;
int nmDiagStage = 2;  // 0=bypass, 1=dequeue only, 2=full cache

static bool nmCacheCSInitialized = false;
CRITICAL_SECTION nmCacheCS;
CRITICAL_SECTION buildCollisionCS;
CRITICAL_SECTION processJobCS;
volatile long    nmCacheDisabled = 0;

volatile long nmJobCount = 0;
volatile long nmCacheHitCount = 0;
volatile long nmCacheMissCount = 0;
volatile long nmCacheSkipCount = 0;
volatile long nmTotalMsTimes10 = 0;
volatile long nmSavedMsTimes10 = 0;
volatile long nmDiagLastGridX = -99;
volatile long nmDiagLastGridY = -99;
volatile long nmDiagLastType = -1;
volatile long nmDiagStep = 0;
volatile long nmDiagHitGrid = -99;
static double lastNMLogTime = 0.0;

volatile long  nmSettingsDumped = 0;
volatile long  nmSettingsVerified = 0;
volatile float probeEMP[14];
volatile float probeGen[24];
volatile float probeMisc[8];
volatile float verifyEMP[7];

volatile long wbProbeDone = 0;
volatile long probeWBPtrLo = 0;
volatile long probeWBPtrHi = 0;
volatile long probeHavokPtrLo = 0;
volatile long probeHavokPtrHi = 0;
volatile long probeWBMatch = 0;
volatile long probeNMGPtrLo = 0;
volatile long probeNMGPtrHi = 0;
volatile long probeWBMsize = 0;
volatile long probeWBHeapSize = 0;
volatile long probeWBFieldScan = 0;

volatile long wbArrayScanDone = 0;
int           wbArrayOffsets[WB_MAX_ARRAYS];
volatile long wbArrayCount = 0;
int           wbWritableOffsets[WB_MAX_ARRAYS];
volatile long wbWritableCount = 0;
WBArrayProbe  wbArrayProbes[WB_MAX_ARRAYS];
int           wbSdkArrayOffsets[WB_MAX_SDK_ARRAYS];
int           wbSdkArrayCount = 0;

volatile long nmDiskHitCount = 0;
volatile long nmDiskMissCount = 0;
volatile long nmDiskWriteCount = 0;
volatile long nmDiskReadUsTimes1 = 0;
volatile long nmDiskWriteUsTimes1 = 0;

L2MissEntry   l2MissLog[L2_MISS_LOG_MAX];
volatile long l2MissLogCount = 0;
volatile long l2MissLogReported = 0;

volatile long workerBusyCount = 0;
volatile long g_slabAllocHits = 0;
volatile long g_slabAllocWorkerHits = 0;

volatile long g_workBufAllocSize = 0;

volatile long lazyHooksInstalled = 0;
volatile long nmCloneConstructCount = 0;
volatile long nmCloneConstructFailCount = 0;
volatile long nmWorkerMissCount = 0;
volatile long nmBgMissCount = 0;
volatile long nmLateHitCount = 0;

volatile long l2RejCount[L2REJ_REASON_COUNT] = {};
volatile long l2CapEvicted = 0;
volatile long nmL2ZeroFaceSkip = 0;
volatile long nmReconFailCount = 0;

#if NMFIX_STEP >= 4
volatile long nmPartialRealCount = 0;
#endif

#if NMFIX_STEP >= 5
volatile long nmTripCount = 0;
volatile long nmTripInstalled = 0;
volatile long nmT234Count = 0;
volatile long nmT234TotalMsTimes10 = 0;
volatile long nmT234MaxMsTimes10 = 0;
#endif

#if NMFIX_STEP >= 6
volatile long     g_buildOverlapSeen = 0;
volatile long     nmBcCount = 0;
volatile LONGLONG nmBcWaitTotalUs = 0;
volatile long     nmBcWaitMaxUs = 0;
volatile LONGLONG nmBcHoldTotalUs = 0;
volatile long     nmBcHoldMaxUs = 0;
#endif

#if NMFIX_STEP >= 7
volatile long nmCloneHandleClosed = 0;
volatile long nmCloneHandleSkipped = 0;
volatile LONGLONG nmWbFreedBytes = 0;
#endif

#if NMFIX_STEP >= 8
volatile long nmHitStaleCount = 0;
volatile long nmDupL2Avoided = 0;
volatile long nmL2FlightFull = 0;
#endif

volatile long g_navMeshWorkersLive = 0;

volatile long     nmPjWaitCount[PJWAIT_SITE_COUNT]   = {};
volatile LONGLONG nmPjWaitTotalUs[PJWAIT_SITE_COUNT] = {};
volatile long     nmPjWaitMaxUs[PJWAIT_SITE_COUNT]   = {};

volatile long     nmClaimAgeMissCount   = 0;
volatile LONGLONG nmClaimAgeMissTotalUs = 0;
volatile long     nmClaimAgeMissMaxUs   = 0;
volatile long     nmClaimAgeMissBucket[CLAIMAGE_BUCKET_COUNT] = {};
volatile long     nmClaimAgeHitCount    = 0;
volatile LONGLONG nmClaimAgeHitTotalUs  = 0;
volatile long     nmClaimAgeHitMaxUs    = 0;

volatile long nmStaleCount[STALE_SITE_COUNT] = {};
volatile long nmStaleLastGridX  = -1;
volatile long nmStaleLastGridY  = -1;
volatile long nmStaleLastType   = -1;
volatile long nmStaleLastReason = STALE_REASON_NONE;
volatile long nmStaleLastAgeUs  = 0;

volatile long nmZeroFaceCount = 0;
volatile long nmZeroFaceLastTri = -1;
volatile long nmZeroFaceLastVert = -1;
volatile long nmZeroFaceLastThings = -1;
volatile long nmZeroFaceLastGridX = -99;
volatile long nmZeroFaceLastGridY = -99;
volatile long nmZeroFaceLastType = -1;
volatile long nmZeroFaceEmptyInput = 0;
volatile long nmZeroFaceAbort = 0;

std::string   nmDiskCacheDir;
char          nmDiskCacheDirBuf[MAX_PATH] = {};
bool          nmDiskCacheDirChecked = false;

void InitNavMeshCacheCS()
{
	if (!nmCacheCSInitialized)
	{
		InitializeCriticalSection(&nmCacheCS);
		InitializeCriticalSection(&buildCollisionCS);
		InitializeCriticalSection(&processJobCS);
		nmCacheCSInitialized = true;
		LogDebug("[ZoneOpt] NavMesh cache CS initialized");
	}

	// Hoist disk cache dir init to main thread to eliminate lazy-init race
	if (!nmDiskCacheDirChecked)
	{
		nmDiskCacheDir = GetDLLDirectory() + "navmesh_cache\\";
		CreateDirectoryA(nmDiskCacheDir.c_str(), NULL);
		// char copy for the bg threads: they must not touch std::string
		if (nmDiskCacheDir.size() < sizeof(nmDiskCacheDirBuf))
			strcpy_s(nmDiskCacheDirBuf, sizeof(nmDiskCacheDirBuf), nmDiskCacheDir.c_str());
		nmDiskCacheDirChecked = true;
		LogDebug("[ZoneOpt] Disk cache dir hoisted: " + nmDiskCacheDir);
	}

#if NMCACHE_STEP >= 2
	// One-time self-check of the L2 primitives (DEV only). The CRC vector is
	// the standard "123456789" check value for CRC-32/ISO-HDLC.
	{
		std::ostringstream ss;
		ss << "[ZoneOpt] L2 format: ver=" << L2_CACHE_VERSION
		   << " settingsHash=" << std::hex << L2SettingsHash()
		   << " modSet=" << g_modSetHash << std::dec
		   << " capMB=" << cfg_navmeshDiskCacheMaxMB
		   << " crcSelfTest=" << ((L2Crc32("123456789", 9) == 0xCBF43926u) ? "ok" : "FAIL");
		LogDebug(ss.str());
	}

	// Sweep stale ".tmp" leftovers and unreachable old-format entries, and
	// measure the directory, before any navmesh job runs. Without this a
	// resting cache is never trimmed, because the cap otherwise only runs on
	// a write, and orphans wait for the next MISS.
	L2StartupScan();
	{
		std::ostringstream ss;
		ss << "[ZoneOpt] L2 startup scan: removed "
		   << InterlockedCompareExchange(&l2CapEvicted, 0, 0) << " file(s)";
		LogMsg(ss.str());
	}
#endif
}


unsigned int HashAABB(const float* aabb6)
{
	unsigned int h = 2166136261u;
	const unsigned char* p = (const unsigned char*)aabb6;
	for (int i = 0; i < 24; ++i)
	{
		h ^= p[i];
		h *= 16777619u;
	}
	return h;
}

// Order-independent building hash: per-building FNV-1a of (position, rotation,
// stringID), summed across all buildings (commutative). Position is the whole
// Vector3 at +0x48, so elevation counts, and rotation is the quaternion at
// +0xB0 (ZO-11). Do NOT use hand.index — runtime-assigned.
unsigned int ComputeBuildingHash(uintptr_t jobZone)
{
	unsigned int totalHash = 0;
	uintptr_t content = *(uintptr_t*)(jobZone + OFF_ZONE_CONTENT);
	if (!content) return totalHash;
	int tCount = *(int*)(content + OFF_ZMC_THINGS_COUNT);
	uintptr_t* stuffPtr = *(uintptr_t**)(content + 96);
	if (!stuffPtr || tCount <= 0 || tCount >= 10000) return totalHash;

	for (int t = 0; t < tCount; ++t)
	{
		uintptr_t obj = (uintptr_t)stuffPtr[t];
		if (!obj) continue;
		int handType = *(int*)(obj + 0x60);
		if (handType != 0) continue;  // BUILDING only

		// RootObjectBase::pos is an Ogre::Vector3 at +0x48 (x +0x48, y +0x4C,
		// z +0x50) and RootObject::rot an Ogre::Quaternion at +0xB0, 16 bytes
		// (KenshiLib RootObjectBase.h / RootObject.h; getOrientation 0xD1EC0
		// copies the four dwords at +0xB0). Buildings derive from RootObject,
		// whose own members start at +0xC0, so both are in range for the
		// handType == 0 objects this loop keeps. Elevation and rotation change
		// the generated mesh, so they belong in the key (ZO-11).
		unsigned int h = 2166136261u;
		const unsigned char* bp = (const unsigned char*)(obj + 0x48);
		for (int b = 0; b < 12; ++b) { h ^= bp[b]; h *= 16777619u; }   // pos x, y, z
		bp = (const unsigned char*)(obj + 0xB0);
		for (int b = 0; b < 16; ++b) { h ^= bp[b]; h *= 16777619u; }   // rot quaternion

		// Hash full stringID content (GameData+0x58, MSVC 2010 std::string)
		uintptr_t gameData = *(uintptr_t*)(obj + 0x40);
		if (gameData)
		{
			uintptr_t strObj = gameData + 0x58;
			size_t len = *(size_t*)(strObj + 16);
			size_t res = *(size_t*)(strObj + 24);
			const unsigned char* strData;
			if (res < 16)
				strData = (const unsigned char*)strObj;
			else
				strData = *(const unsigned char**)strObj;
			if (strData && len > 0 && len < 256)
			{
				for (size_t c = 0; c < len; ++c)
				{ h ^= strData[c]; h *= 16777619u; }
			}
		}

		totalHash += h;
	}
	return totalHash;
}

// inputTri comes from the populate hook, which reads the input geometry's
// triangle count (geometry+40) on its way into realGenerate. realGenerate
// itself tests that field and refuses to generate when it is zero, asserting
// "Passed in empty triMesh to generateNavMesh", so the split below is the
// generator's own notion of an empty input, not an inference:
//   tri == 0  -> nothing was fed in, the empty result is correct for the tile
//   tri  > 0  -> geometry went in and no faces came out, so the run aborted
// inputTri is -1 when the count is unavailable (the hook failed to install, or
// the mesh came from an L2 file rather than a generation); neither counter
// moves then. inputThings is the zone's things count, kept as a cross-check.
void NoteZeroFaceMesh(const NavMeshCacheKey& key, int inputTri, int inputVert, int inputThings)
{
	InterlockedIncrement(&nmZeroFaceCount);
	InterlockedExchange(&nmZeroFaceLastTri, (long)inputTri);
	InterlockedExchange(&nmZeroFaceLastVert, (long)inputVert);
	InterlockedExchange(&nmZeroFaceLastThings, (long)inputThings);
	InterlockedExchange(&nmZeroFaceLastGridX, (long)key.gridX);
	InterlockedExchange(&nmZeroFaceLastGridY, (long)key.gridY);
	InterlockedExchange(&nmZeroFaceLastType, (long)key.jobType);

	if (inputTri == 0)
		InterlockedIncrement(&nmZeroFaceEmptyInput);
	else if (inputTri > 0)
		InterlockedIncrement(&nmZeroFaceAbort);
}

bool KeysMatch(const NavMeshCacheKey& a, const NavMeshCacheKey& b)
{
	return a.gridX == b.gridX
	    && a.gridY == b.gridY
	    && a.sectionTileId == b.sectionTileId
	    && a.jobType == b.jobType
	    && a.aabbHash == b.aabbHash
	    && a.buildingHash == b.buildingHash;
}


int FindCacheEntry(const NavMeshCacheKey& key)
{
	for (int i = 0; i < nmCacheFill; ++i)
	{
		if (!nmCache[i].valid || !KeysMatch(nmCache[i].key, key))
			continue;
#if NMFIX_STEP >= 1
		// Never serve a zero-face mesh, including one a build before this rule
		// left in the ring buffer. Regenerating is right whether the tile is
		// genuinely empty or the generation aborted.
		if (nmCache[i].faceCount <= 0)
			continue;
#endif
		return i;
	}
	return -1;
}

void EvictCacheEntry(int idx)
{
	if (idx < 0 || idx >= NM_CACHE_SIZE) return;
	if (!nmCache[idx].valid) return;

	if (nmCache[idx].cachedFaces)     fn_gameDelArr(nmCache[idx].cachedFaces);
	if (nmCache[idx].cachedEdges)     fn_gameDelArr(nmCache[idx].cachedEdges);
	if (nmCache[idx].cachedVertices)  fn_gameDelArr(nmCache[idx].cachedVertices);
	if (nmCache[idx].cachedFaceData)  fn_gameDelArr(nmCache[idx].cachedFaceData);
	if (nmCache[idx].cachedEdgeData)  fn_gameDelArr(nmCache[idx].cachedEdgeData);

	nmCache[idx].cachedFaces = NULL;
	nmCache[idx].cachedEdges = NULL;
	nmCache[idx].cachedVertices = NULL;
	nmCache[idx].cachedFaceData = NULL;
	nmCache[idx].cachedEdgeData = NULL;
	nmCache[idx].valid = false;
}

// Deep-copies one array out of the generated mesh. Returns false on allocation
// failure so the caller can discard the whole entry.
static bool CopyMeshArray(uintptr_t navMeshPtr, int arrayOff, int count, int unit, void** dest)
{
	*dest = NULL;
	if (count <= 0) return true;
	const void* src = hkArrayGetPtr(navMeshPtr, arrayOff);
	if (!src) return false;
	size_t sz = (size_t)count * (size_t)unit;
	void* buf = fn_gameNewArr(sz);
	if (!buf) return false;
	memcpy(buf, src, sz);
	*dest = buf;
	return true;
}

static void FreeEntryArrays(NavMeshCacheEntry& e)
{
	if (e.cachedFaces)    { fn_gameDelArr(e.cachedFaces);    e.cachedFaces = NULL; }
	if (e.cachedEdges)    { fn_gameDelArr(e.cachedEdges);    e.cachedEdges = NULL; }
	if (e.cachedVertices) { fn_gameDelArr(e.cachedVertices); e.cachedVertices = NULL; }
	if (e.cachedFaceData) { fn_gameDelArr(e.cachedFaceData); e.cachedFaceData = NULL; }
	if (e.cachedEdgeData) { fn_gameDelArr(e.cachedEdgeData); e.cachedEdgeData = NULL; }
	e.valid = false;
}

// Publishes a fully built entry into the ring buffer. Returns its slot index.
static int PublishEntry(NavMeshCacheEntry& src)
{
	if (nmCache[nmCacheWriteIdx].valid)
		EvictCacheEntry(nmCacheWriteIdx);

	int idx = nmCacheWriteIdx;
	nmCache[idx] = src;
	nmCache[idx].valid = true;

	nmCacheWriteIdx = (nmCacheWriteIdx + 1) % NM_CACHE_SIZE;
	if (nmCacheFill < NM_CACHE_SIZE)
		nmCacheFill++;

	memset(&src, 0, sizeof(src));   // ownership moved into the ring buffer
	return idx;
}

int StoreCacheEntry(const NavMeshCacheKey& key, uintptr_t navMeshPtr)
{
	if (!navMeshPtr) return -1;

	int faceCount    = hkArrayGetCount(navMeshPtr, NMOFF_FACES);
	int edgeCount    = hkArrayGetCount(navMeshPtr, NMOFF_EDGES);
	int vertexCount  = hkArrayGetCount(navMeshPtr, NMOFF_VERTICES);
	int faceStriding = *(int*)(navMeshPtr + 112);
	int edgeStriding = *(int*)(navMeshPtr + 116);
	int faceDataCnt  = hkArrayGetCount(navMeshPtr, NMOFF_FACEDATA);
	int edgeDataCnt  = hkArrayGetCount(navMeshPtr, NMOFF_EDGEDATA);

#if NMFIX_STEP >= 1
	// Zero faces means either a generation that aborted or a genuinely empty
	// tile, and nothing here can tell those apart (NoteZeroFaceMesh explains
	// why). Caching either one makes the tile permanently empty, so neither is
	// stored. Refusing here also keeps it out of L2: the disk blob is only
	// built from a slot this function published.
	if (faceCount == 0)
		return -1;
#endif

	if (faceCount    < 0 || faceCount    > L2_MAX_FACES)    return -1;
	if (edgeCount    < 0 || edgeCount    > L2_MAX_EDGES)    return -1;
	if (vertexCount  < 0 || vertexCount  > L2_MAX_VERTICES) return -1;
	if (faceDataCnt  < 0 || faceDataCnt  > L2_MAX_FACEDATA) return -1;
	if (edgeDataCnt  < 0 || edgeDataCnt  > L2_MAX_EDGEDATA) return -1;

	// Build the entry off to the side, then publish it. A failed allocation
	// frees everything taken so far and stores nothing, so a valid slot never
	// holds a NULL array (ZO-03).
	NavMeshCacheEntry e;
	memset(&e, 0, sizeof(e));
	e.key = key;
	e.faceCount     = faceCount;
	e.edgeCount     = edgeCount;
	e.vertexCount   = vertexCount;
	e.faceDataCount = faceDataCnt;
	e.edgeDataCount = edgeDataCnt;

	bool ok = true;
	if (ok) ok = CopyMeshArray(navMeshPtr, NMOFF_FACES,    faceCount,   HKAI_FACE_SIZE,     &e.cachedFaces);
	if (ok) ok = CopyMeshArray(navMeshPtr, NMOFF_EDGES,    edgeCount,   HKAI_EDGE_SIZE,     &e.cachedEdges);
	if (ok) ok = CopyMeshArray(navMeshPtr, NMOFF_VERTICES, vertexCount, HKAI_VERTEX_SIZE,   &e.cachedVertices);
	if (ok) ok = CopyMeshArray(navMeshPtr, NMOFF_FACEDATA, faceDataCnt, HKAI_FACEDATA_UNIT, &e.cachedFaceData);
	if (ok) ok = CopyMeshArray(navMeshPtr, NMOFF_EDGEDATA, edgeDataCnt, HKAI_EDGEDATA_UNIT, &e.cachedEdgeData);

	if (!ok)
	{
		FreeEntryArrays(e);
		return -1;
	}

	e.faceDataStriding = faceStriding;
	e.edgeDataStriding = edgeStriding;
	e.navMeshFlags = *(unsigned char*)(navMeshPtr + 120);
	memcpy(e.aabb, (void*)(navMeshPtr + 128), 32);
	e.erosionRadius = *(float*)(navMeshPtr + 160);
	e.userData = *(unsigned __int64*)(navMeshPtr + 168);

	return PublishEntry(e);
}

int PromoteDiskEntryToL1(NavMeshCacheEntry& e)
{
#if NMFIX_STEP >= 1
	// An L2 file written before this rule can still hold a zero-face mesh.
	// Drop it and let the job regenerate.
	if (e.valid && e.faceCount <= 0)
	{
		NoteZeroFaceMesh(e.key, -1, -1, -1);
		FreeEntryArrays(e);
		return -1;
	}
#endif
	if (!e.valid
	    || (e.faceCount     > 0 && !e.cachedFaces)
	    || (e.edgeCount     > 0 && !e.cachedEdges)
	    || (e.vertexCount   > 0 && !e.cachedVertices)
	    || (e.faceDataCount > 0 && !e.cachedFaceData)
	    || (e.edgeDataCount > 0 && !e.cachedEdgeData))
	{
		FreeEntryArrays(e);
		return -1;
	}
	return PublishEntry(e);
}

// Copies one cached array into a fresh Havok TLS buffer and installs it in the
// navmesh. Returns false on allocation failure, leaving the array slot empty.
static bool InstallMeshArray(uintptr_t nm, int arrayOff, const void* src, int count, int unit)
{
	if (count <= 0 || !src) return true;
	size_t sz = (size_t)count * (size_t)unit;
	void* buf = HavokTlsAlloc(sz);
	if (!buf) return false;
	memcpy(buf, src, sz);
	hkArraySet(nm, arrayOff, buf, count, count);
	return true;
}

// Releases the array buffers this function installed, then the navmesh block.
// The Havok destructor is deliberately not called: nothing else has seen the
// object, its refcount has never been raised, and the only allocations on it
// are the five arrays installed here.
static void UnwindReconstruct(uintptr_t nm)
{
	const int offs[5]  = { NMOFF_FACES, NMOFF_EDGES, NMOFF_VERTICES, NMOFF_FACEDATA, NMOFF_EDGEDATA };
	const int units[5] = { HKAI_FACE_SIZE, HKAI_EDGE_SIZE, HKAI_VERTEX_SIZE, HKAI_FACEDATA_UNIT, HKAI_EDGEDATA_UNIT };
	for (int i = 0; i < 5; ++i)
	{
		void* buf = hkArrayGetPtr(nm, offs[i]);
		int count = hkArrayGetCount(nm, offs[i]);
		if (buf && count > 0)
			HavokTlsFree(buf, (size_t)count * (size_t)units[i]);
		hkArraySet(nm, offs[i], NULL, 0, 0);
	}
	HavokTlsFree((void*)nm, 176);
}

// Allocate fresh hkaiNavMesh via Havok TLS, populate arrays from cache.
// Havok destructor will correctly free everything when refcount reaches 0.
//
// All-or-nothing: a failed Havok allocation frees everything taken so far and
// returns NULL. A partly filled mesh must never reach the game — the callers
// treat a non-NULL return as a complete result and hand it straight to
// buildCollision.
void* ReconstructNavMesh(const NavMeshCacheEntry& entry)
{
	void* mem = HavokTlsAlloc(176);
	if (!mem) { InterlockedIncrement(&nmReconFailCount); return NULL; }

	void* navMesh = fn_navMeshCtor(mem);
	if (!navMesh)
	{
		HavokTlsFree(mem, 176);
		InterlockedIncrement(&nmReconFailCount);
		return NULL;
	}
	uintptr_t nm = (uintptr_t)navMesh;

	bool ok = true;
	if (ok) ok = InstallMeshArray(nm, NMOFF_FACES,    entry.cachedFaces,    entry.faceCount,     HKAI_FACE_SIZE);
	if (ok) ok = InstallMeshArray(nm, NMOFF_EDGES,    entry.cachedEdges,    entry.edgeCount,     HKAI_EDGE_SIZE);
	if (ok) ok = InstallMeshArray(nm, NMOFF_VERTICES, entry.cachedVertices, entry.vertexCount,   HKAI_VERTEX_SIZE);
	if (ok) ok = InstallMeshArray(nm, NMOFF_FACEDATA, entry.cachedFaceData, entry.faceDataCount, HKAI_FACEDATA_UNIT);
	if (ok) ok = InstallMeshArray(nm, NMOFF_EDGEDATA, entry.cachedEdgeData, entry.edgeDataCount, HKAI_EDGEDATA_UNIT);

	if (!ok)
	{
		UnwindReconstruct(nm);
		InterlockedIncrement(&nmReconFailCount);
		return NULL;
	}

	*(int*)(nm + 112) = entry.faceDataStriding;
	*(int*)(nm + 116) = entry.edgeDataStriding;
	*(unsigned char*)(nm + 120) = entry.navMeshFlags;
	memcpy((void*)(nm + 128), entry.aabb, 32);
	*(float*)(nm + 160) = entry.erosionRadius;
	*(unsigned __int64*)(nm + 168) = entry.userData;

	return navMesh;
}

void ClearNavMeshCache()
{
	InterlockedExchange(&nmCacheDisabled, 1);

	if (nmCacheCSInitialized)
		EnterCriticalSection(&nmCacheCS);

	int cleared = nmCacheFill;
	for (int i = 0; i < NM_CACHE_SIZE; ++i)
		EvictCacheEntry(i);
	nmCacheWriteIdx = 0;
	nmCacheFill = 0;

	if (nmCacheCSInitialized)
		LeaveCriticalSection(&nmCacheCS);

	InterlockedExchange(&nmCacheDisabled, 0);

	if (cleared > 0)
	{
		std::ostringstream ss;
		ss << "[ZoneOpt] NavMesh cache cleared, evicted " << cleared << " entries";
		LogDebug(ss.str());
	}
}


void LogNavMeshCacheStats(double now)
{
	if (!cachingEnabled)
		return;
#ifdef ZONEOPT_DEBUG
	if (now - lastNMLogTime < 10.0)
#else
	if (now - lastNMLogTime < 30.0)
#endif
		return;
	lastNMLogTime = now;

	long jobs    = InterlockedCompareExchange(&nmJobCount, 0, 0);
	long hits    = InterlockedCompareExchange(&nmCacheHitCount, 0, 0);
	long misses  = InterlockedCompareExchange(&nmCacheMissCount, 0, 0);
	long skips   = InterlockedCompareExchange(&nmCacheSkipCount, 0, 0);
	long totalMs = InterlockedCompareExchange(&nmTotalMsTimes10, 0, 0);
	long savedMs = InterlockedCompareExchange(&nmSavedMsTimes10, 0, 0);
	long gx      = InterlockedCompareExchange(&nmDiagLastGridX, 0, 0);
	long gy      = InterlockedCompareExchange(&nmDiagLastGridY, 0, 0);
	long jt      = InterlockedCompareExchange(&nmDiagLastType, 0, 0);
	long step    = InterlockedCompareExchange(&nmDiagStep, 0, 0);
	long hitGr   = InterlockedCompareExchange(&nmDiagHitGrid, 0, 0);

	long diskHits  = InterlockedCompareExchange(&nmDiskHitCount, 0, 0);
	long diskMiss  = InterlockedCompareExchange(&nmDiskMissCount, 0, 0);
	long diskWrites = InterlockedCompareExchange(&nmDiskWriteCount, 0, 0);
	long diskReadUs = InterlockedCompareExchange(&nmDiskReadUsTimes1, 0, 0);
	long diskWriteUs = InterlockedCompareExchange(&nmDiskWriteUsTimes1, 0, 0);

	std::ostringstream ss;
	ss << "[ZoneOpt] NM cache: jobs=" << jobs
	   << " L1hit=" << hits
	   << " L2hit=" << diskHits
	   << " miss=" << misses
	   << " skips=" << skips
	   << " fill=" << nmCacheFill << "/" << NM_CACHE_SIZE;

	if (misses > 0)
	{
		double avgMs = (double)totalMs / (10.0 * misses);
		ss << " avgMiss=" << std::fixed << std::setprecision(1) << avgMs << "ms";
	}
	if (hits > 0)
	{
		double avgHitMs = (double)savedMs / (10.0 * hits);
		ss << " avgL1Hit=" << std::fixed << std::setprecision(1) << avgHitMs << "ms";
	}
	if (diskHits > 0)
	{
		double avgL2Ms = (double)diskReadUs / (1000.0 * diskHits);
		ss << " avgL2=" << std::fixed << std::setprecision(1) << avgL2Ms << "ms";
	}
	int totalHits = hits + (int)diskHits;
	int totalAttempts = totalHits + (int)misses;
	int hitPct = (totalAttempts > 0) ? (int)(100 * totalHits / totalAttempts) : 0;
	ss << " hitRate=" << hitPct << "%";
	ss << " diskWrites=" << diskWrites;

#if NMCACHE_STEP >= 2
	{
		long rej[L2REJ_REASON_COUNT];
		long rejTotal = 0;
		for (int r = 0; r < L2REJ_REASON_COUNT; ++r)
		{
			rej[r] = InterlockedCompareExchange(&l2RejCount[r], 0, 0);
			rejTotal += rej[r];
		}
		ss << " l2Rej=" << rejTotal
		   << "(m" << rej[L2REJ_MAGIC]
		   << "/v" << rej[L2REJ_VERSION]
		   << "/h" << rej[L2REJ_SETTINGS]
		   << "/l" << rej[L2REJ_LENGTH]
		   << "/c" << rej[L2REJ_CRC]
		   << "/b" << rej[L2REJ_BOUNDS]
		   << "/i" << rej[L2REJ_IO]
		   << "/a" << rej[L2REJ_ALLOC]
		   << "/p" << rej[L2REJ_PATHLONG] << ")";
		ss << " l2Cap=" << InterlockedCompareExchange(&l2CapEvicted, 0, 0);
		// NMFIX 1b: zero-face entries the blob builder refused. L1 refuses
		// them first today, so this should never print.
		long l2Zero = InterlockedCompareExchange(&nmL2ZeroFaceSkip, 0, 0);
		if (l2Zero)
			ss << " l2ZeroSkip=" << l2Zero;
	}
#endif
	{
		long reconFail = InterlockedCompareExchange(&nmReconFailCount, 0, 0);
		if (reconFail)
			ss << " reconFail=" << reconFail;
	}

	// tri= is the last zero-face generation's input triangle count (-1 when it
	// was not captured). zfEmptyIn / zfAbort split every zero-face generation by
	// that count; see NoteZeroFaceMesh.
	{
		long zeroFace = InterlockedCompareExchange(&nmZeroFaceCount, 0, 0);
		ss << " zeroFace=" << zeroFace;
		if (zeroFace)
		{
			ss << "(tri=" << InterlockedCompareExchange(&nmZeroFaceLastTri, 0, 0)
			   << " vert=" << InterlockedCompareExchange(&nmZeroFaceLastVert, 0, 0)
			   << " things=" << InterlockedCompareExchange(&nmZeroFaceLastThings, 0, 0)
			   << " grid=" << InterlockedCompareExchange(&nmZeroFaceLastGridX, 0, 0)
			   << "," << InterlockedCompareExchange(&nmZeroFaceLastGridY, 0, 0)
			   << " type=" << InterlockedCompareExchange(&nmZeroFaceLastType, 0, 0) << ")";
		}
		ss << " zfEmptyIn=" << InterlockedCompareExchange(&nmZeroFaceEmptyInput, 0, 0)
		   << " zfAbort=" << InterlockedCompareExchange(&nmZeroFaceAbort, 0, 0);
	}

	if (jobs > 0)
		ss << " lastGrid=(" << gx << "," << gy << ") type=" << jt;

	ss << " step=" << step;
	ss << " busy=" << InterlockedCompareExchange(&workerBusyCount, 0, 0);
	long cloneOk = InterlockedCompareExchange(&nmCloneConstructCount, 0, 0);
	long cloneFail = InterlockedCompareExchange(&nmCloneConstructFailCount, 0, 0);
	if (cloneOk || cloneFail)
		ss << " clone=" << cloneOk << "/" << (cloneOk + cloneFail);

	long workerMiss = InterlockedCompareExchange(&nmWorkerMissCount, 0, 0);
	long bgMiss = InterlockedCompareExchange(&nmBgMissCount, 0, 0);
	if (workerMiss || bgMiss)
		ss << " miss=w" << workerMiss << "/bg" << bgMiss;

	long lateHits = InterlockedCompareExchange(&nmLateHitCount, 0, 0);
	if (lateHits)
		ss << " lateHit=" << lateHits;

#if NMFIX_STEP >= 2
	{
		long slopeBad = InterlockedCompareExchange(&g_wbOverrideSlopeBad, 0, 0);
		long ovSkip   = InterlockedCompareExchange(&g_wbOverrideSkipped, 0, 0);
		ss << " wbOv=" << InterlockedCompareExchange(&g_wbOverrideInstalled, 0, 0);
#ifdef ZONEOPT_DEBUG
		ss << "/" << InterlockedCompareExchange(&g_wbOverrideAfterPop, 0, 0);
#endif
		if (slopeBad)
			ss << " wbOvSlope=" << slopeBad;
		if (ovSkip)
			ss << " wbOvSkip=" << ovSkip;
	}
#endif

#if NMFIX_STEP >= 3 && defined(ZONEOPT_DEBUG)
	{
		long bits = InterlockedCompareExchange(&g_wbQualityLast, 0, 0);
		float q;
		memcpy(&q, &bits, sizeof(q));
		ss << " wbQ=" << std::fixed << std::setprecision(2) << q;
	}
#endif

#if NMFIX_STEP >= 4
	ss << " nbrLookup=" << InterlockedCompareExchange(&nmPartialRealCount, 0, 0);
#endif

#if NMFIX_STEP >= 6
	ss << " bcOverlap=" << InterlockedCompareExchange(&g_buildOverlapSeen, 0, 0);
	{
		long n = InterlockedCompareExchange(&nmBcCount, 0, 0);
		if (n > 0)
		{
			double wAvg = (double)InterlockedCompareExchange64(&nmBcWaitTotalUs, 0, 0) / (1000.0 * n);
			double wMax = (double)InterlockedCompareExchange(&nmBcWaitMaxUs, 0, 0) / 1000.0;
			double hAvg = (double)InterlockedCompareExchange64(&nmBcHoldTotalUs, 0, 0) / (1000.0 * n);
			double hMax = (double)InterlockedCompareExchange(&nmBcHoldMaxUs, 0, 0) / 1000.0;
			ss << std::fixed << std::setprecision(2)
			   << " bcWait=" << wAvg << "/" << wMax << "ms"
			   << " bcHold=" << hAvg << "/" << hMax << "ms";
		}
	}
#endif

#if NMFIX_STEP >= 5
	if (InterlockedCompareExchange(&nmTripInstalled, 0, 0))
		ss << " trip=" << InterlockedCompareExchange(&nmTripCount, 0, 0);
	else
		ss << " trip=off";
	{
		long n = InterlockedCompareExchange(&nmT234Count, 0, 0);
		if (n > 0)
		{
			double avg = (double)InterlockedCompareExchange(&nmT234TotalMsTimes10, 0, 0) / (10.0 * n);
			double mx  = (double)InterlockedCompareExchange(&nmT234MaxMsTimes10, 0, 0) / 10.0;
			ss << " t234=" << std::fixed << std::setprecision(1) << avg
			   << "/" << std::fixed << std::setprecision(1) << mx << "ms";
		}
	}
#endif

#if NMFIX_STEP >= 7
	// Leak Fix 3 evidence. handles= is the whole process's handle count, read on
	// the main thread; it grew by 15 per worker MISS before the fix and should
	// now be flat across a session.
	{
		DWORD handleCount = 0;
		if (!GetProcessHandleCount(GetCurrentProcess(), &handleCount))
			handleCount = 0;
		ss << " handles=" << handleCount;
	}
	ss << " hClosed=" << InterlockedCompareExchange(&nmCloneHandleClosed, 0, 0);
#ifdef ZONEOPT_DEBUG
	ss << " wbFreed=" << InterlockedCompareExchange64(&nmWbFreedBytes, 0, 0);
#endif
	{
		long skipped = InterlockedCompareExchange(&nmCloneHandleSkipped, 0, 0);
		if (skipped)
			ss << " hSkip=" << skipped;
	}
#endif

#if NMFIX_STEP >= 8
	ss << " hitStale=" << InterlockedCompareExchange(&nmHitStaleCount, 0, 0)
	   << " dupL2=" << InterlockedCompareExchange(&nmDupL2Avoided, 0, 0)
	   << " workers=" << InterlockedCompareExchange(&g_navMeshWorkersLive, 0, 0);
	{
		long full = InterlockedCompareExchange(&nmL2FlightFull, 0, 0);
		if (full)
			ss << " l2Full=" << full;
	}
#endif

	long edgeArmed = InterlockedCompareExchange(&g_edgeProcessArmedCount, 0, 0);
	long edgeUnarmed = InterlockedCompareExchange(&g_edgeProcessUnarmedCount, 0, 0);
	if (edgeArmed || edgeUnarmed)
		ss << " edge=a" << edgeArmed << "/u" << edgeUnarmed;

	// Round 2 session 1 crash measurement (nm_cache_core.h). Always printed,
	// zeros included. Every value is ms with one decimal: the average is the
	// microsecond total over the count, the max the largest single sample; both
	// read 0.0 while the count is 0.
	{
		ss << std::fixed << std::setprecision(1);

		static const char* PJ_NAMES[PJWAIT_SITE_COUNT] = { "clone", "wMiss", "bgMiss" };
		ss << " pjWait";
		for (int s = 0; s < PJWAIT_SITE_COUNT; ++s)
		{
			long   n   = InterlockedCompareExchange(&nmPjWaitCount[s], 0, 0);
			double avg = n > 0 ? (double)InterlockedCompareExchange64(&nmPjWaitTotalUs[s], 0, 0) / (1000.0 * n) : 0.0;
			double mx  = (double)InterlockedCompareExchange(&nmPjWaitMaxUs[s], 0, 0) / 1000.0;
			ss << " " << PJ_NAMES[s] << "=" << avg << "/" << mx;
		}
		ss << "ms";

		long   mN   = InterlockedCompareExchange(&nmClaimAgeMissCount, 0, 0);
		double mAvg = mN > 0 ? (double)InterlockedCompareExchange64(&nmClaimAgeMissTotalUs, 0, 0) / (1000.0 * mN) : 0.0;
		double mMax = (double)InterlockedCompareExchange(&nmClaimAgeMissMaxUs, 0, 0) / 1000.0;
		long   hN   = InterlockedCompareExchange(&nmClaimAgeHitCount, 0, 0);
		double hAvg = hN > 0 ? (double)InterlockedCompareExchange64(&nmClaimAgeHitTotalUs, 0, 0) / (1000.0 * hN) : 0.0;
		double hMax = (double)InterlockedCompareExchange(&nmClaimAgeHitMaxUs, 0, 0) / 1000.0;
		ss << " claimAge miss=" << mAvg << "/" << mMax
		   << " hit=" << hAvg << "/" << hMax << "ms";

		ss << " [<10:"  << InterlockedCompareExchange(&nmClaimAgeMissBucket[0], 0, 0)
		   << " <100:"  << InterlockedCompareExchange(&nmClaimAgeMissBucket[1], 0, 0)
		   << " <1s:"   << InterlockedCompareExchange(&nmClaimAgeMissBucket[2], 0, 0)
		   << " <5s:"   << InterlockedCompareExchange(&nmClaimAgeMissBucket[3], 0, 0)
		   << " >=5s:"  << InterlockedCompareExchange(&nmClaimAgeMissBucket[4], 0, 0) << "]";

		long sw = InterlockedCompareExchange(&nmStaleCount[STALE_SITE_WMISS], 0, 0);
		long sb = InterlockedCompareExchange(&nmStaleCount[STALE_SITE_BGMISS], 0, 0);
		long sh = InterlockedCompareExchange(&nmStaleCount[STALE_SITE_HIT], 0, 0);
		long se = InterlockedCompareExchange(&nmStaleCount[STALE_SITE_EARLY], 0, 0);
		ss << " stale=w" << sw << "/bg" << sb << "/hit" << sh << "/early" << se;

		if (sw + sb + sh + se == 0)
		{
			ss << " staleLast=-";
		}
		else
		{
			// Torn reads across these five are possible and acceptable (see
			// nm_cache_core.h).
			static const char* REASONS[] = { "none", "noZone", "noContent", "noTerrain" };
			long r = InterlockedCompareExchange(&nmStaleLastReason, 0, 0);
			if (r < 0 || r > STALE_REASON_NO_TERRAIN) r = STALE_REASON_NONE;
			ss << " staleLast=(" << InterlockedCompareExchange(&nmStaleLastGridX, 0, 0)
			   << "," << InterlockedCompareExchange(&nmStaleLastGridY, 0, 0)
			   << ")t" << InterlockedCompareExchange(&nmStaleLastType, 0, 0)
			   << "/" << REASONS[r]
			   << "@" << (double)InterlockedCompareExchange(&nmStaleLastAgeUs, 0, 0) / 1000.0 << "ms";
		}
	}

	// Crash-3 diagnostic (core.h). Also on the transition line, but this one is
	// periodic, so a session that dies before completing a transition still
	// leaves the inserter counters and the off-main-thread call sites behind.
	ss << DestroyListStatsSuffix();

	LogMsg(ss.str());

	// L2 miss diagnostic
	long missLogN = InterlockedCompareExchange(&l2MissLogCount, 0, 0);
	long missReported = InterlockedCompareExchange(&l2MissLogReported, 0, 0);
	if (missLogN > missReported && missReported < L2_MISS_LOG_MAX)
	{
		long end = missLogN;
		if (end > L2_MISS_LOG_MAX) end = L2_MISS_LOG_MAX;
		for (long m = missReported; m < end; ++m)
		{
			const L2MissEntry& e = l2MissLog[m];
			std::ostringstream ms;
			const char* tag = (e.thingsCount == -2) ? "L2write" : "L2miss";
			ms << "[ZoneOpt] " << tag << "[" << m << "] zone=(" << e.gridX << "," << e.gridY << ")"
			   << " tile=" << e.tileId << " type=" << e.jobType
			   << " aabb=" << std::hex << e.aabbHash
			   << " bldg=" << e.buildingHash << std::dec;
			if (e.thingsCount >= 0)
				ms << " things=" << e.thingsCount;
			LogMsg(ms.str());
		}
		InterlockedExchange(&l2MissLogReported, end);
	}

	// One-time settings dump
	if (InterlockedCompareExchange(&nmSettingsDumped, 3, 2) == 2)
	{
		{
			std::ostringstream e;
			e << std::fixed << std::setprecision(5);
			e << "[ZoneOpt] NavMesh EdgeMatchParams: "
			  << "maxStepH=" << probeEMP[0]
			  << " maxSep=" << probeEMP[1]
			  << " maxOverhang=" << probeEMP[2]
			  << " behindFaceTol=" << probeEMP[3]
			  << " cosPlanarAlign=" << probeEMP[4]
			  << " cosVertAlign=" << probeEMP[5]
			  << " minEdgeOverlap=" << probeEMP[6];
			LogMsg(e.str());
		}
		{
			std::ostringstream e;
			e << std::fixed << std::setprecision(5);
			e << "[ZoneOpt] NavMesh EdgeMatchParams2: "
			  << "travHorizEps=" << probeEMP[7]
			  << " travVertEps=" << probeEMP[8]
			  << " cosClimbFace=" << probeEMP[9]
			  << " cosClimbEdge=" << probeEMP[10]
			  << " minAngleFaces=" << probeEMP[11]
			  << " edgeParallelTol=" << probeEMP[12]
			  << " pad=" << probeEMP[13];
			LogMsg(e.str());
		}
		{
			std::ostringstream e;
			e << std::fixed << std::setprecision(5);
			e << "[ZoneOpt] NavMesh GenConfig+336: ";
			for (int i = 0; i < 12; ++i)
				e << "[" << i << "]=" << probeGen[i] << " ";
			LogMsg(e.str());
		}
		{
			std::ostringstream e;
			e << std::fixed << std::setprecision(5);
			e << "[ZoneOpt] NavMesh GenConfig+384: ";
			for (int i = 12; i < 24; ++i)
				e << "[" << i << "]=" << probeGen[i] << " ";
			LogMsg(e.str());
		}
		{
			std::ostringstream e;
			e << std::fixed << std::setprecision(4);
			e << "[ZoneOpt] NavMesh Misc: "
			  << "quantGrid=" << probeMisc[0]
			  << " degenArea=" << probeMisc[1]
			  << " charWidth=" << probeMisc[2]
			  << " edgeFilterThresh=" << probeMisc[3]
			  << " maxEdgesPerFace=" << (int)probeMisc[4]
			  << " edgeMatchMetric=" << (int)probeMisc[5]
			  << " edgeConnIter=" << (int)probeMisc[6]
			  << " maxPartSize=" << (int)probeMisc[7];
			LogMsg(e.str());
		}
	}

	// Post-patch verification dump
	if (InterlockedCompareExchange(&nmSettingsVerified, 3, 2) == 2)
	{
		std::ostringstream e;
		e << std::fixed << std::setprecision(5);
		e << "[ZoneOpt] NavMesh PATCHED: "
		  << "maxStepH=" << verifyEMP[0]
		  << " maxSep=" << verifyEMP[1]
		  << " cosPlanarAlign=" << verifyEMP[2]
		  << " minCorridorW=" << verifyEMP[3]
		  << " maxCorridorW=" << verifyEMP[4]
		  << " charWidth=" << verifyEMP[5]
		  << " edgeConnIter=" << (int)verifyEMP[6];
		LogMsg(e.str());
	}

	// WorkBuffer size probe dump
	if (InterlockedCompareExchange(&wbProbeDone, 3, 2) == 2)
	{
		uintptr_t nmgAddr = ((uintptr_t)(unsigned long)probeNMGPtrHi << 32) | (unsigned long)probeNMGPtrLo;
		uintptr_t wbAddr = ((uintptr_t)(unsigned long)probeWBPtrHi << 32) | (unsigned long)probeWBPtrLo;
		uintptr_t havokAddr = ((uintptr_t)(unsigned long)probeHavokPtrHi << 32) | (unsigned long)probeHavokPtrLo;
		long match = InterlockedCompareExchange(&probeWBMatch, 0, 0);

		long fieldScan = InterlockedCompareExchange(&probeWBFieldScan, 0, 0);

		std::ostringstream e;
		e << "[ZoneOpt] WorkBuffer probe: NMG=" << (void*)nmgAddr
		  << " wb(+256)=" << (void*)wbAddr
		  << " sectionMgr+136=" << (void*)havokAddr;
		if (match)
			e << " MATCH (size=688)";
		else if (wbAddr && havokAddr)
			e << " DIFFER offset=" << (__int64)((long long)wbAddr - (long long)havokAddr);
		long lastReadable = InterlockedCompareExchange(&probeWBHeapSize, 0, 0);
		e << " lastContent=+" << fieldScan
		  << " lastReadable=+" << lastReadable
		  << " allocSize=" << (lastReadable + 8);
		LogMsg(e.str());

		long arrCount = InterlockedCompareExchange(&wbArrayCount, 0, 0);
		long wrtCount = InterlockedCompareExchange(&wbWritableCount, 0, 0);
		if (arrCount > 0)
		{
			{
				std::ostringstream a;
				a << "[ZoneOpt] hkArray scan: " << arrCount << " arrays found, "
				  << wrtCount << " classified writable. Offsets:";
				for (int i = 0; i < arrCount && i < WB_MAX_ARRAYS; ++i)
					a << " +" << wbArrayOffsets[i];
				LogMsg(a.str());
			}
			for (int i = 0; i < arrCount && i < 32; ++i)
			{
				std::ostringstream d;
				d << "[ZoneOpt]   wb+" << wbArrayProbes[i].offset
				  << " ptr=" << (void*)wbArrayProbes[i].ptr
				  << " count=" << wbArrayProbes[i].count
				  << " cap=" << (wbArrayProbes[i].capFlags & 0x3FFFFFFF)
				  << " flags=0x" << std::hex << ((unsigned int)wbArrayProbes[i].capFlags & 0xC0000000) << std::dec;
				LogMsg(d.str());
			}
		}
		else
		{
			LogMsg("[ZoneOpt] hkArray scan: 0 arrays found (workBuffer may be uninitialized)");
		}
	}
}

#endif // NMCACHE_STEP >= 1
