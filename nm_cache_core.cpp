// nm_cache_core.cpp — L1 in-memory navmesh ring buffer cache + stats reporter

#include "nm_cache_core.h"

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

volatile long flaHooksInstalled = 0;
volatile long csProbeFLAPtrLo = 0;
volatile long csProbeFLAPtrHi = 0;

std::string   nmDiskCacheDir;
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
		nmDiskCacheDirChecked = true;
		LogDebug("[ZoneOpt] Disk cache dir hoisted: " + nmDiskCacheDir);
	}
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

// Order-independent building hash: per-building FNV-1a of (posX, posZ, stringID),
// summed across all buildings (commutative). Do NOT use hand.index — runtime-assigned.
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

		unsigned int h = 2166136261u;
		const unsigned char* bp = (const unsigned char*)(obj + 0x48);
		for (int b = 0; b < 4; ++b) { h ^= bp[b]; h *= 16777619u; }
		bp = (const unsigned char*)(obj + 0x50);
		for (int b = 0; b < 4; ++b) { h ^= bp[b]; h *= 16777619u; }

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
		if (nmCache[i].valid && KeysMatch(nmCache[i].key, key))
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

void StoreCacheEntry(const NavMeshCacheKey& key, uintptr_t navMeshPtr)
{
	if (!navMeshPtr) return;

	int faceCount    = hkArrayGetCount(navMeshPtr, NMOFF_FACES);
	int edgeCount    = hkArrayGetCount(navMeshPtr, NMOFF_EDGES);
	int vertexCount  = hkArrayGetCount(navMeshPtr, NMOFF_VERTICES);
	int faceStriding = *(int*)(navMeshPtr + 112);
	int edgeStriding = *(int*)(navMeshPtr + 116);
	int faceDataCnt  = hkArrayGetCount(navMeshPtr, NMOFF_FACEDATA);
	int edgeDataCnt  = hkArrayGetCount(navMeshPtr, NMOFF_EDGEDATA);

	if (faceCount < 0 || faceCount > 50000) return;
	if (edgeCount < 0 || edgeCount > 200000) return;
	if (vertexCount < 0 || vertexCount > 100000) return;

	if (nmCache[nmCacheWriteIdx].valid)
		EvictCacheEntry(nmCacheWriteIdx);

	NavMeshCacheEntry& e = nmCache[nmCacheWriteIdx];
	e.key = key;

	e.faceCount = faceCount;
	e.cachedFaces = NULL;
	if (faceCount > 0)
	{
		size_t sz = (size_t)faceCount * HKAI_FACE_SIZE;
		e.cachedFaces = fn_gameNewArr(sz);
		if (e.cachedFaces)
			memcpy(e.cachedFaces, hkArrayGetPtr(navMeshPtr, NMOFF_FACES), sz);
	}

	e.edgeCount = edgeCount;
	e.cachedEdges = NULL;
	if (edgeCount > 0)
	{
		size_t sz = (size_t)edgeCount * HKAI_EDGE_SIZE;
		e.cachedEdges = fn_gameNewArr(sz);
		if (e.cachedEdges)
			memcpy(e.cachedEdges, hkArrayGetPtr(navMeshPtr, NMOFF_EDGES), sz);
	}

	e.vertexCount = vertexCount;
	e.cachedVertices = NULL;
	if (vertexCount > 0)
	{
		size_t sz = (size_t)vertexCount * HKAI_VERTEX_SIZE;
		e.cachedVertices = fn_gameNewArr(sz);
		if (e.cachedVertices)
			memcpy(e.cachedVertices, hkArrayGetPtr(navMeshPtr, NMOFF_VERTICES), sz);
	}

	e.faceDataCount = faceDataCnt;
	e.cachedFaceData = NULL;
	if (faceDataCnt > 0)
	{
		size_t sz = (size_t)faceDataCnt * HKAI_FACEDATA_UNIT;
		e.cachedFaceData = fn_gameNewArr(sz);
		if (e.cachedFaceData)
			memcpy(e.cachedFaceData, hkArrayGetPtr(navMeshPtr, NMOFF_FACEDATA), sz);
	}

	e.edgeDataCount = edgeDataCnt;
	e.cachedEdgeData = NULL;
	if (edgeDataCnt > 0)
	{
		size_t sz = (size_t)edgeDataCnt * HKAI_EDGEDATA_UNIT;
		e.cachedEdgeData = fn_gameNewArr(sz);
		if (e.cachedEdgeData)
			memcpy(e.cachedEdgeData, hkArrayGetPtr(navMeshPtr, NMOFF_EDGEDATA), sz);
	}

	e.faceDataStriding = faceStriding;
	e.edgeDataStriding = edgeStriding;
	e.navMeshFlags = *(unsigned char*)(navMeshPtr + 120);
	memcpy(e.aabb, (void*)(navMeshPtr + 128), 32);
	e.erosionRadius = *(float*)(navMeshPtr + 160);
	e.userData = *(unsigned __int64*)(navMeshPtr + 168);

	e.valid = true;

	nmCacheWriteIdx = (nmCacheWriteIdx + 1) % NM_CACHE_SIZE;
	if (nmCacheFill < NM_CACHE_SIZE)
		nmCacheFill++;
}

// Allocate fresh hkaiNavMesh via Havok TLS, populate arrays from cache.
// Havok destructor will correctly free everything when refcount reaches 0.
void* ReconstructNavMesh(const NavMeshCacheEntry& entry)
{
	void* mem = HavokTlsAlloc(176);
	if (!mem) return NULL;

	void* navMesh = fn_navMeshCtor(mem);
	if (!navMesh) return NULL;
	uintptr_t nm = (uintptr_t)navMesh;

	if (entry.faceCount > 0 && entry.cachedFaces)
	{
		size_t sz = (size_t)entry.faceCount * HKAI_FACE_SIZE;
		void* buf = HavokTlsAlloc(sz);
		if (buf)
		{
			memcpy(buf, entry.cachedFaces, sz);
			hkArraySet(nm, NMOFF_FACES, buf, entry.faceCount, entry.faceCount);
		}
	}

	if (entry.edgeCount > 0 && entry.cachedEdges)
	{
		size_t sz = (size_t)entry.edgeCount * HKAI_EDGE_SIZE;
		void* buf = HavokTlsAlloc(sz);
		if (buf)
		{
			memcpy(buf, entry.cachedEdges, sz);
			hkArraySet(nm, NMOFF_EDGES, buf, entry.edgeCount, entry.edgeCount);
		}
	}

	if (entry.vertexCount > 0 && entry.cachedVertices)
	{
		size_t sz = (size_t)entry.vertexCount * HKAI_VERTEX_SIZE;
		void* buf = HavokTlsAlloc(sz);
		if (buf)
		{
			memcpy(buf, entry.cachedVertices, sz);
			hkArraySet(nm, NMOFF_VERTICES, buf, entry.vertexCount, entry.vertexCount);
		}
	}

	if (entry.faceDataCount > 0 && entry.cachedFaceData)
	{
		size_t sz = (size_t)entry.faceDataCount * HKAI_FACEDATA_UNIT;
		void* buf = HavokTlsAlloc(sz);
		if (buf)
		{
			memcpy(buf, entry.cachedFaceData, sz);
			hkArraySet(nm, NMOFF_FACEDATA, buf, entry.faceDataCount, entry.faceDataCount);
		}
	}

	if (entry.edgeDataCount > 0 && entry.cachedEdgeData)
	{
		size_t sz = (size_t)entry.edgeDataCount * HKAI_EDGEDATA_UNIT;
		void* buf = HavokTlsAlloc(sz);
		if (buf)
		{
			memcpy(buf, entry.cachedEdgeData, sz);
			hkArraySet(nm, NMOFF_EDGEDATA, buf, entry.edgeDataCount, entry.edgeDataCount);
		}
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

	if (jobs > 0)
		ss << " lastGrid=(" << gx << "," << gy << ") type=" << jt;

	ss << " step=" << step;
	ss << " busy=" << InterlockedCompareExchange(&workerBusyCount, 0, 0);
	ss << " flaCS=" << InterlockedCompareExchange(&flaHooksInstalled, 0, 0);

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
