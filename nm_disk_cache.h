// nm_disk_cache.h — L2 persistent disk cache for navmesh results (Layer 3)
// Depends on: nm_cache_core.h (for types + shared state)

#ifndef KENSHI_ZONE_OPT_NM_DISK_CACHE_H
#define KENSHI_ZONE_OPT_NM_DISK_CACHE_H

#include "nm_cache_core.h"

std::string GetDiskCachePath(const NavMeshCacheKey& key);
void        WriteDiskCache(const NavMeshCacheKey& key, uintptr_t navMeshPtr);
bool        ReadDiskCache(const NavMeshCacheKey& key, NavMeshCacheEntry& out);

#endif // KENSHI_ZONE_OPT_NM_DISK_CACHE_H
