// nm_cache_types.h — Shared types and constants for navmesh cache modules
// Depends on: config.h

#ifndef KENSHI_ZONE_OPT_NM_CACHE_TYPES_H
#define KENSHI_ZONE_OPT_NM_CACHE_TYPES_H

#include "config.h"


// hkaiNavMesh array element sizes (from Havok SDK hkaiNavMesh.h)
const int HKAI_FACE_SIZE     = 16;
const int HKAI_EDGE_SIZE     = 20;
const int HKAI_VERTEX_SIZE   = 16;
const int HKAI_FACEDATA_UNIT = 4;
const int HKAI_EDGEDATA_UNIT = 4;

// hkArray offsets within hkaiNavMesh (176 bytes)
const int NMOFF_FACES    = 16;
const int NMOFF_EDGES    = 32;
const int NMOFF_VERTICES = 48;
const int NMOFF_FACEDATA = 80;
const int NMOFF_EDGEDATA = 96;

// hkArray field accessors
inline void* hkArrayGetPtr(uintptr_t nm, int off)   { return *(void**)(nm + off); }
inline int   hkArrayGetCount(uintptr_t nm, int off)  { return *(int*)(nm + off + 8); }
inline void  hkArraySet(uintptr_t nm, int off, void* ptr, int count, int capFlags)
{
	*(void**)(nm + off)     = ptr;
	*(int*)(nm + off + 8)   = count;
	*(int*)(nm + off + 12)  = capFlags;
}


struct NavMeshCacheKey {
	int gridX;
	int gridY;
	int sectionTileId;
	int jobType;
	unsigned int aabbHash;
	unsigned int buildingHash;
};

struct NavMeshCacheEntry {
	NavMeshCacheKey key;

	void*  cachedFaces;      int faceCount;
	void*  cachedEdges;      int edgeCount;
	void*  cachedVertices;   int vertexCount;
	void*  cachedFaceData;   int faceDataCount;
	void*  cachedEdgeData;   int edgeDataCount;

	int            faceDataStriding;   // +112
	int            edgeDataStriding;   // +116
	unsigned char  navMeshFlags;       // +120
	char           aabb[32];           // +128: 2 x __m128
	float          erosionRadius;      // +160
	unsigned __int64 userData;         // +168

	bool   valid;
};

const int NM_CACHE_SIZE = 256;


struct WorkerScratch { void** buffer; int capacity; };

const int MAX_DEEPCOPY_ARRAYS = 8;
struct DeepCopyAlloc {
	void* buffer;
	int   wbOffset;
};
struct DeepCopyTracker {
	DeepCopyAlloc allocs[MAX_DEEPCOPY_ARRAYS];
	int count;
};

const int NMG_STRUCT_SIZE = 352;  // NavMeshGenerator object size (operator new(0x160))
const int WB_OBJECT_SIZE = 0x218;  // hkaiNavMeshGenerationSettings: 0x208 + sizeof(hkArray) = 536 bytes

// hkArray flag: data is not owned by this array, do not free/realloc (hkArray.h)
const int HKARRAY_DONT_DEALLOCATE = (int)0x80000000;

const int WB_MAX_ARRAYS = 64;
const int WB_MAX_SDK_ARRAYS = 8;
const int L2_MISS_LOG_MAX = 16;

struct WBArrayProbe { int offset; uintptr_t ptr; int count; int capFlags; };

struct L2MissEntry {
	int gridX, gridY, tileId, jobType;
	unsigned int aabbHash, buildingHash;
	int thingsCount;
};


#endif // KENSHI_ZONE_OPT_NM_CACHE_TYPES_H
