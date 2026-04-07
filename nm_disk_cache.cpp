// nm_disk_cache.cpp — L2 persistent navmesh disk cache

#include "nm_disk_cache.h"

#if NMCACHE_STEP >= 2

std::string GetDiskCachePath(const NavMeshCacheKey& key)
{
	if (!nmDiskCacheDirChecked)
	{
		nmDiskCacheDir = GetDLLDirectory() + "navmesh_cache\\";
		CreateDirectoryA(nmDiskCacheDir.c_str(), NULL);
		nmDiskCacheDirChecked = true;
	}
	char buf[256];
	sprintf_s(buf, sizeof(buf), "%s%d_%d_%d_%d_%x_%x.bin",
	          nmDiskCacheDir.c_str(),
	          key.gridX, key.gridY, key.sectionTileId, key.jobType,
	          key.aabbHash, key.buildingHash);
	return std::string(buf);
}

void WriteDiskCache(const NavMeshCacheKey& key, uintptr_t navMeshPtr)
{
	if (!navMeshPtr) return;
	std::string path = GetDiskCachePath(key);
	FILE* f = NULL;
	if (fopen_s(&f, path.c_str(), "wb") != 0 || !f) return;

	int faceCount    = hkArrayGetCount(navMeshPtr, NMOFF_FACES);
	int edgeCount    = hkArrayGetCount(navMeshPtr, NMOFF_EDGES);
	int vertexCount  = hkArrayGetCount(navMeshPtr, NMOFF_VERTICES);
	int faceDataCnt  = hkArrayGetCount(navMeshPtr, NMOFF_FACEDATA);
	int edgeDataCnt  = hkArrayGetCount(navMeshPtr, NMOFF_EDGEDATA);

	fwrite(&faceCount, 4, 1, f);
	fwrite(&edgeCount, 4, 1, f);
	fwrite(&vertexCount, 4, 1, f);
	fwrite(&faceDataCnt, 4, 1, f);
	fwrite(&edgeDataCnt, 4, 1, f);

	int faceStriding = *(int*)(navMeshPtr + 112);
	int edgeStriding = *(int*)(navMeshPtr + 116);
	unsigned char flags = *(unsigned char*)(navMeshPtr + 120);
	fwrite(&faceStriding, 4, 1, f);
	fwrite(&edgeStriding, 4, 1, f);
	fwrite(&flags, 1, 1, f);
	char pad7[7] = {0};
	fwrite(pad7, 7, 1, f);
	fwrite((void*)(navMeshPtr + 128), 32, 1, f);
	float erosion = *(float*)(navMeshPtr + 160);
	fwrite(&erosion, 4, 1, f);
	char pad4[4] = {0};
	fwrite(pad4, 4, 1, f);
	unsigned __int64 userData = *(unsigned __int64*)(navMeshPtr + 168);
	fwrite(&userData, 8, 1, f);

	if (faceCount > 0)    fwrite(hkArrayGetPtr(navMeshPtr, NMOFF_FACES),    (size_t)faceCount * HKAI_FACE_SIZE, 1, f);
	if (edgeCount > 0)    fwrite(hkArrayGetPtr(navMeshPtr, NMOFF_EDGES),    (size_t)edgeCount * HKAI_EDGE_SIZE, 1, f);
	if (vertexCount > 0)  fwrite(hkArrayGetPtr(navMeshPtr, NMOFF_VERTICES), (size_t)vertexCount * HKAI_VERTEX_SIZE, 1, f);
	if (faceDataCnt > 0)  fwrite(hkArrayGetPtr(navMeshPtr, NMOFF_FACEDATA), (size_t)faceDataCnt * HKAI_FACEDATA_UNIT, 1, f);
	if (edgeDataCnt > 0)  fwrite(hkArrayGetPtr(navMeshPtr, NMOFF_EDGEDATA), (size_t)edgeDataCnt * HKAI_EDGEDATA_UNIT, 1, f);

	fclose(f);
	InterlockedIncrement(&nmDiskWriteCount);

	long wIdx = InterlockedCompareExchange(&nmDiskWriteCount, 0, 0);
	if (wIdx <= L2_MISS_LOG_MAX)
	{
		long idx = InterlockedIncrement(&l2MissLogCount) - 1;
		if (idx < L2_MISS_LOG_MAX)
		{
			l2MissLog[idx].gridX = key.gridX;
			l2MissLog[idx].gridY = key.gridY;
			l2MissLog[idx].tileId = key.sectionTileId;
			l2MissLog[idx].jobType = key.jobType;
			l2MissLog[idx].aabbHash = key.aabbHash;
			l2MissLog[idx].buildingHash = key.buildingHash;
			l2MissLog[idx].thingsCount = -2;  // sentinel: write entry
		}
	}
}

bool ReadDiskCache(const NavMeshCacheKey& key, NavMeshCacheEntry& out)
{
	std::string path = GetDiskCachePath(key);
	FILE* f = NULL;
	if (fopen_s(&f, path.c_str(), "rb") != 0 || !f) return false;

	int counts[5];
	if (fread(counts, 4, 5, f) != 5) { fclose(f); return false; }
	int faceCount = counts[0], edgeCount = counts[1], vertexCount = counts[2];
	int faceDataCnt = counts[3], edgeDataCnt = counts[4];

	if (faceCount < 0 || faceCount > 50000 ||
	    edgeCount < 0 || edgeCount > 200000 ||
	    vertexCount < 0 || vertexCount > 100000)
	{ fclose(f); return false; }

	int faceStriding, edgeStriding;
	unsigned char flags;
	char pad7[7], pad4[4];
	char aabb[32];
	float erosion;
	unsigned __int64 userData;

	fread(&faceStriding, 4, 1, f);
	fread(&edgeStriding, 4, 1, f);
	fread(&flags, 1, 1, f);
	fread(pad7, 7, 1, f);
	fread(aabb, 32, 1, f);
	fread(&erosion, 4, 1, f);
	fread(pad4, 4, 1, f);
	fread(&userData, 8, 1, f);

	out.key = key;
	out.faceCount = faceCount;
	out.edgeCount = edgeCount;
	out.vertexCount = vertexCount;
	out.faceDataCount = faceDataCnt;
	out.edgeDataCount = edgeDataCnt;
	out.faceDataStriding = faceStriding;
	out.edgeDataStriding = edgeStriding;
	out.navMeshFlags = flags;
	memcpy(out.aabb, aabb, 32);
	out.erosionRadius = erosion;
	out.userData = userData;
	out.cachedFaces = out.cachedEdges = out.cachedVertices = NULL;
	out.cachedFaceData = out.cachedEdgeData = NULL;

	bool ok = true;
	if (faceCount > 0) {
		size_t sz = (size_t)faceCount * HKAI_FACE_SIZE;
		out.cachedFaces = fn_gameNewArr(sz);
		if (!out.cachedFaces || fread(out.cachedFaces, sz, 1, f) != 1) ok = false;
	}
	if (ok && edgeCount > 0) {
		size_t sz = (size_t)edgeCount * HKAI_EDGE_SIZE;
		out.cachedEdges = fn_gameNewArr(sz);
		if (!out.cachedEdges || fread(out.cachedEdges, sz, 1, f) != 1) ok = false;
	}
	if (ok && vertexCount > 0) {
		size_t sz = (size_t)vertexCount * HKAI_VERTEX_SIZE;
		out.cachedVertices = fn_gameNewArr(sz);
		if (!out.cachedVertices || fread(out.cachedVertices, sz, 1, f) != 1) ok = false;
	}
	if (ok && faceDataCnt > 0) {
		size_t sz = (size_t)faceDataCnt * HKAI_FACEDATA_UNIT;
		out.cachedFaceData = fn_gameNewArr(sz);
		if (!out.cachedFaceData || fread(out.cachedFaceData, sz, 1, f) != 1) ok = false;
	}
	if (ok && edgeDataCnt > 0) {
		size_t sz = (size_t)edgeDataCnt * HKAI_EDGEDATA_UNIT;
		out.cachedEdgeData = fn_gameNewArr(sz);
		if (!out.cachedEdgeData || fread(out.cachedEdgeData, sz, 1, f) != 1) ok = false;
	}

	fclose(f);

	if (!ok) {
		if (out.cachedFaces)    { fn_gameDelArr(out.cachedFaces);    out.cachedFaces = NULL; }
		if (out.cachedEdges)    { fn_gameDelArr(out.cachedEdges);    out.cachedEdges = NULL; }
		if (out.cachedVertices) { fn_gameDelArr(out.cachedVertices); out.cachedVertices = NULL; }
		if (out.cachedFaceData) { fn_gameDelArr(out.cachedFaceData); out.cachedFaceData = NULL; }
		if (out.cachedEdgeData) { fn_gameDelArr(out.cachedEdgeData); out.cachedEdgeData = NULL; }
		return false;
	}

	out.valid = true;
	return true;
}

#endif // NMCACHE_STEP >= 2
