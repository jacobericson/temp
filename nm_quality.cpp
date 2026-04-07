// nm_quality.cpp — NavMesh generation quality tuning + workBuffer probes

#include "nm_quality.h"

#if NMCACHE_STEP >= 3

void ProbeNavMeshSettings(uintptr_t nmg)
{
	if (InterlockedCompareExchange(&nmSettingsDumped, 1, 0) != 0)
		return;

	uintptr_t wb = *(uintptr_t*)(nmg + 256);
	if (!wb)
	{
		InterlockedExchange(&nmSettingsDumped, 0);
		return;
	}

	// No CRT strings on bg thread — only float writes + Interlocked
	float* emp = (float*)(wb + 76);  // EdgeMatchingParameters (56 bytes = 14 floats)
	for (int i = 0; i < 14; ++i)
		probeEMP[i] = emp[i];

	float* gen = (float*)(wb + 336);
	for (int i = 0; i < 24; ++i)
		probeGen[i] = gen[i];

	probeMisc[0] = *(float*)(wb + 16);    // m_quantizationGridSize
	probeMisc[1] = *(float*)(wb + 48);    // m_degenerateAreaThreshold
	probeMisc[2] = *(float*)(wb + 328);   // m_minCharacterWidth
	probeMisc[3] = *(float*)(wb + 400);   // m_boundaryEdgeFilterThreshold
	probeMisc[4] = (float)*(int*)(wb + 72);   // m_maxNumEdgesPerFace
	probeMisc[5] = (float)*(int*)(wb + 132);  // m_edgeMatchingMetric
	probeMisc[6] = (float)*(int*)(wb + 136);  // m_edgeConnectionIterations
	probeMisc[7] = (float)*(int*)(wb + 472);  // m_maxPartitionSize

	InterlockedExchange(&nmSettingsDumped, 2);
}

// All values in Havok units (= Kenshi world units / 10)
void ApplyNavMeshQualityTuning(uintptr_t nmg)
{
	uintptr_t wb = *(uintptr_t*)(nmg + 256);
	if (!wb) return;

	// wb+76 = EdgeMatchingParameters
	*(float*)(wb + 80) = 0.25f;   // m_maxSeparation (was 0.2)
	*(float*)(wb + 92) = 0.985f;  // m_cosPlanarAlignmentAngle (was 0.99619, ~5deg -> ~10deg)

	// wb+336 = SimplificationSettings
	*(float*)(wb + 344) = 0.15f;  // m_minCorridorWidth (was 0.4)
	*(float*)(wb + 348) = 1.2f;   // m_maxCorridorWidth (was 0.6)

	// wb+328 = m_minCharacterWidth (was 0.9)
	*(float*)(wb + 328) = 0.5f;

	// wb+136 = m_edgeConnectionIterations (was 2 -> 4 passes total)
	*(int*)(wb + 136) = 3;
}

void VerifyNavMeshSettings(uintptr_t nmg)
{
	if (InterlockedCompareExchange(&nmSettingsVerified, 1, 0) != 0)
		return;

	uintptr_t wb = *(uintptr_t*)(nmg + 256);
	if (!wb)
	{
		InterlockedExchange(&nmSettingsVerified, 0);
		return;
	}

	float* emp = (float*)(wb + 76);
	verifyEMP[0] = emp[0];                    // m_maxStepHeight (+76)
	verifyEMP[1] = emp[1];                    // m_maxSeparation (+80)
	verifyEMP[2] = emp[4];                    // m_cosPlanarAlignmentAngle (+92)
	verifyEMP[3] = *(float*)(wb + 344);       // m_minCorridorWidth
	verifyEMP[4] = *(float*)(wb + 348);       // m_maxCorridorWidth
	verifyEMP[5] = *(float*)(wb + 328);       // m_minCharacterWidth
	verifyEMP[6] = (float)*(int*)(wb + 136);  // m_edgeConnectionIterations
	InterlockedExchange(&nmSettingsVerified, 2);
}

// SEH-safe probes kept in standalone functions (MSVC 2010 can't mix __try with C++ destructors)
void ProbeWorkBufferSize(uintptr_t nmg)
{
	if (InterlockedCompareExchange(&wbProbeDone, 1, 0) != 0)
		return;

	uintptr_t wb = *(uintptr_t*)(nmg + 256);
	uintptr_t sectionMgr = *(uintptr_t*)(nmg + 240);
	uintptr_t havokObj = 0;
	if (sectionMgr)
		havokObj = *(uintptr_t*)(sectionMgr + 136);

	InterlockedExchange(&probeNMGPtrLo, (long)(nmg & 0xFFFFFFFF));
	InterlockedExchange(&probeNMGPtrHi, (long)((nmg >> 32) & 0xFFFFFFFF));
	InterlockedExchange(&probeWBPtrLo, (long)(wb & 0xFFFFFFFF));
	InterlockedExchange(&probeWBPtrHi, (long)((wb >> 32) & 0xFFFFFFFF));
	InterlockedExchange(&probeHavokPtrLo, (long)(havokObj & 0xFFFFFFFF));
	InterlockedExchange(&probeHavokPtrHi, (long)((havokObj >> 32) & 0xFFFFFFFF));
	InterlockedExchange(&probeWBMatch, (wb != 0 && wb == havokObj) ? 1 : 0);

	if (wb)
	{
		// Scan for access fault boundary (true allocation size)
		int lastNonZero = 0;
		int lastReadable = 0;
		for (int off = 0; off < 65536; off += 8)
		{
			__try
			{
				uintptr_t val = *(uintptr_t*)(wb + off);
				lastReadable = off;
				if (val != 0)
					lastNonZero = off;
			}
			__except(EXCEPTION_EXECUTE_HANDLER) { break; }
		}
		InterlockedExchange(&probeWBFieldScan, lastNonZero);
		InterlockedExchange(&probeWBHeapSize, lastReadable);

		InterlockedExchange(&g_workBufAllocSize, lastReadable + 8);

		// hkArray scanner: find all arrays in the workBuffer
		int scanLimit = lastReadable;
		int arrayCount = 0;
		for (int off = 0; off + 16 <= scanLimit && arrayCount < WB_MAX_ARRAYS; off += 8)
		{
			__try
			{
				uintptr_t ptr = *(uintptr_t*)(wb + off);
				int count     = *(int*)(wb + off + 8);
				int capFlags  = *(int*)(wb + off + 12);
				int cap       = capFlags & 0x3FFFFFFF;

				if (ptr > 0x10000 && ptr < 0x7FFFFFFFFFFFULL
				    && count > 0 && count < 100000
				    && cap >= count && cap < 500000)
				{
					wbArrayProbes[arrayCount].offset = off;
					wbArrayProbes[arrayCount].ptr = ptr;
					wbArrayProbes[arrayCount].count = count;
					wbArrayProbes[arrayCount].capFlags = capFlags;
					wbArrayOffsets[arrayCount++] = off;
				}
			}
			__except(EXCEPTION_EXECUTE_HANDLER) { break; }
		}

		// Always include known intermediate arrays (may be zeroed after finalize)
		int knownWritable[] = { 240, 256, 520 };
		for (int k = 0; k < 3; ++k)
		{
			bool found = false;
			for (int a = 0; a < arrayCount; ++a)
			{
				if (wbArrayOffsets[a] == knownWritable[k])
				{ found = true; break; }
			}
			if (!found && arrayCount < WB_MAX_ARRAYS)
				wbArrayOffsets[arrayCount++] = knownWritable[k];
		}

		InterlockedExchange(&wbArrayCount, arrayCount);

		// Classify all detected arrays as writable
		int writableCount = 0;
		for (int a = 0; a < arrayCount && writableCount < WB_MAX_ARRAYS; ++a)
			wbWritableOffsets[writableCount++] = wbArrayOffsets[a];
		InterlockedExchange(&wbWritableCount, writableCount);
	}

	InterlockedExchange(&wbProbeDone, 2);
}

#endif // NMCACHE_STEP >= 3
