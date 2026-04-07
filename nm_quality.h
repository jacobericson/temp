// nm_quality.h — NavMesh generation quality tuning + probes (Layer 3)
// Depends on: nm_cache_core.h (for probe buffer externs)

#ifndef KENSHI_ZONE_OPT_NM_QUALITY_H
#define KENSHI_ZONE_OPT_NM_QUALITY_H

#include "nm_cache_core.h"

// All functions take explicit nmg (NavMeshGenerator*) — no globals.
void ProbeNavMeshSettings(uintptr_t nmg);
void ApplyNavMeshQualityTuning(uintptr_t nmg);
void VerifyNavMeshSettings(uintptr_t nmg);
void ProbeWorkBufferSize(uintptr_t nmg);

#endif // KENSHI_ZONE_OPT_NM_QUALITY_H
