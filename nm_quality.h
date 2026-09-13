// nm_quality.h — NavMesh generation quality tuning + probes (Layer 3)
// Depends on: nm_cache_core.h (for probe buffer externs)

#ifndef KENSHI_ZONE_OPT_NM_QUALITY_H
#define KENSHI_ZONE_OPT_NM_QUALITY_H

#include "nm_cache_core.h"
#include <string.h>   // memcpy, used by NmMaterialSlope below


// =========================================================================
// Generation settings table
// =========================================================================
//
// The one place the mod's generation settings live. ApplyNavMeshQualityTuning
// writes this table onto the work buffer and L2SettingsHash (nm_disk_cache.cpp)
// hashes it, so a value cannot drift away from the disk cache key: editing a
// number here invalidates every cached mesh generated with the old one. Kept
// outside the NMCACHE_STEP >= 3 guard because the L2 cache (step 2) needs the
// values even in builds that never apply them.
//
// Offsets are byte offsets into hkaiNavMeshGenerationSettings (NMG+256).
// All values are in Havok units (= Kenshi world units / 10).

struct NavMeshQualitySetting {
	int   wbOffset;
	float value;     // written as a float, or as (int)value when isInt
	bool  isInt;
};

const NavMeshQualitySetting NM_QUALITY_SETTINGS[] = {
	{  80, 0.25f,  false },  // m_maxSeparation (was 0.2)
	{  92, 0.985f, false },  // m_cosPlanarAlignmentAngle (was 0.99619, ~5deg -> ~10deg)
	{ 344, 0.15f,  false },  // m_minCorridorWidth (was 0.4)
	{ 348, 1.2f,   false },  // m_maxCorridorWidth (was 0.6)
	{ 328, 0.5f,   false },  // m_minCharacterWidth (was 0.9)
	{ 136, 3.0f,   true  }   // m_edgeConnectionIterations (was 2 -> 4 passes total)
};
const int NM_QUALITY_SETTING_COUNT = 6;

// The four per-material walkable slopes NavMeshGenerator__initWorkBuffer
// (0x3C48D0) installs for materials 1-4 in the wb+520 override array, and the
// base slope at wb+52 that 0xDD95D0 falls back to when no override matches.
// ConstructFreshSettings installs the first four on every fresh work buffer
// (NMFIX 2) and L2SettingsHash hashes all five, so the cache key and the
// generated mesh cannot describe different slopes.
//
// Stored as raw IEEE-754 bit patterns, not degrees: these are hand-entered
// constants in the game, so they are radians and they are not all clean
// conversions. 60 degrees is a true pi/3 but "90 degrees" is the literal 1.57f,
// and the base is 0.6981308 where 40 degrees would be 0.6981317. Computing them
// from degrees would silently produce different meshes, so the bits are copied
// from the decompile of 0x3C48D0 and verified against the real work buffer at
// runtime in DEV builds.
const unsigned int NM_MATERIAL_SLOPE_BITS[5] = {
	0x3F860A92u,  // material 1  1.0471976  (60 deg, = 1065749138)
	0x3F860A92u,  // material 2  1.0471976  (60 deg, = 1065749138)
	0x3FC8F5C3u,  // material 3  1.57       (~90 deg, = 1070134723)
	0x3F860A92u,  // material 4  1.0471976  (60 deg, = 1065749138)
	0x3F32B8C3u   // base wb+52  0.6981308  (~40 deg, = 1060288707)
};
const int NM_MATERIAL_SLOPE_COUNT = 5;
const int NM_MATERIAL_OVERRIDE_COUNT = 4;   // the first four: materials 1-4

inline float NmMaterialSlope(int i)
{
	float f;
	unsigned int bits = NM_MATERIAL_SLOPE_BITS[i];
	memcpy(&f, &bits, sizeof(f));
	return f;
}


// All functions take explicit nmg (NavMeshGenerator*) — no globals.
void ProbeNavMeshSettings(uintptr_t nmg);
// Writes the table onto a work buffer directly. ConstructFreshSettings calls
// this so a fresh work buffer carries the same generation settings as the real
// one instead of only whatever the scalar copies happened to bring across.
void ApplyNavMeshQualitySettings(uintptr_t wb);
void ApplyNavMeshQualityTuning(uintptr_t nmg);
void VerifyNavMeshSettings(uintptr_t nmg);
void ProbeWorkBufferSize(uintptr_t nmg);

#endif // KENSHI_ZONE_OPT_NM_QUALITY_H
