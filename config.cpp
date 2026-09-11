#include "config.h"
#include <cstdio>


// =========================================================================
// Feature flags
// =========================================================================

bool deferralEnabled       = true;
bool priorityBoostEnabled  = true;
bool preloadEnabled        = true;
bool movementAwareEnabled  = true;
#if NMCACHE_STEP >= 1
bool cachingEnabled        = true;
#else
bool cachingEnabled        = false;
#endif
#ifndef ZONEOPT_ZONEONLY
bool groupCohesionEnabled  = true;
#endif
#if PATHFIND_STEP >= 1
bool pathfindDiagEnabled   = true;
bool squadPathCacheEnabled = false;  // Report 4 CTD: injection is unsafe. INI squadPathCache=true is for Phase 16 staging only.
#endif
#if PATHFIND_STEP >= 2
bool stuckRetryEnabled     = false;  // Log stalled movement without replacing orders by default.
#endif
bool islandFixEnabled      = true;   // islandFix=false keeps the island hooks passing through (A/B control)


// =========================================================================
// Runtime-tunable parameters (defaults match original compile-time constants)
// =========================================================================

// Zone loading
float  cfg_preloadThreshold      = 2500.0f;
int    cfg_cameraReserved        = 12;

// Character tracking
float  cfg_edgeThreshold         = 1800.0f;
double cfg_charScanInterval      = 2.0;
double cfg_baselineScanInterval  = 5.0;
double cfg_activePollInterval    = 1.0;

// Formation
double cfg_gatherTimeout         = 15.0;
double cfg_formationTimeout      = 120.0;
float  cfg_scatterApproachDistSq = 2500.0f;
float  cfg_gatherRadiusSq        = 1600.0f;

// NavMesh workers
int    cfg_navmeshWorkerCount    = 3;

// Hook orchestration
double cfg_camLogInterval        = 10.0;
double cfg_reprioritizeInterval  = 1.0;  // STEP 5: flag-then-backstop cadence (was 3.0)
double cfg_evictInterval         = 2.0;

// Island routing
int    cfg_islandModRadius       = 2;
#ifdef ZONEOPT_DEBUG
double cfg_islandTestPromoteDelay = 0.0;
#endif

// Capacity
int    cfg_maxPreloaded          = 45;
int    cfg_maxWatched            = 32;
int    cfg_maxFormationGroups    = 8;
int    cfg_maxFormationMembers   = 30;
int    cfg_maxCharZones          = 24;
int    cfg_maxPendingOrder       = 30;


// =========================================================================
// INI parser helpers
// =========================================================================

static std::string TrimWhitespace(const std::string& s)
{
	size_t start = s.find_first_not_of(" \t\r\n");
	if (start == std::string::npos)
		return std::string();
	size_t end = s.find_last_not_of(" \t\r\n");
	return s.substr(start, end - start + 1);
}

static bool ParseBool(const std::string& val, bool* out)
{
	if (val == "true" || val == "1" || val == "yes" || val == "on")
		{ *out = true; return true; }
	if (val == "false" || val == "0" || val == "no" || val == "off")
		{ *out = false; return true; }
	return false;
}

static bool ParseFloat(const std::string& val, float* out)
{
	char* end = NULL;
	float f = (float)strtod(val.c_str(), &end);
	if (end == val.c_str()) return false;
	*out = f;
	return true;
}

static bool ParseDouble(const std::string& val, double* out)
{
	char* end = NULL;
	double d = strtod(val.c_str(), &end);
	if (end == val.c_str()) return false;
	*out = d;
	return true;
}

static bool ParseInt(const std::string& val, int* out)
{
	char* end = NULL;
	long l = strtol(val.c_str(), &end, 10);
	if (end == val.c_str()) return false;
	*out = (int)l;
	return true;
}

// Clamp with warning log
static float ClampFloat(const char* name, float val, float lo, float hi)
{
	if (val < lo)
	{
		std::ostringstream ss;
		ss << "[ZoneOpt] Config: " << name << "=" << val << " clamped to min " << lo;
		LogMsg(ss.str());
		return lo;
	}
	if (val > hi)
	{
		std::ostringstream ss;
		ss << "[ZoneOpt] Config: " << name << "=" << val << " clamped to max " << hi;
		LogMsg(ss.str());
		return hi;
	}
	return val;
}

static double ClampDouble(const char* name, double val, double lo, double hi)
{
	if (val < lo)
	{
		std::ostringstream ss;
		ss << "[ZoneOpt] Config: " << name << "=" << val << " clamped to min " << lo;
		LogMsg(ss.str());
		return lo;
	}
	if (val > hi)
	{
		std::ostringstream ss;
		ss << "[ZoneOpt] Config: " << name << "=" << val << " clamped to max " << hi;
		LogMsg(ss.str());
		return hi;
	}
	return val;
}

static int ClampInt(const char* name, int val, int lo, int hi)
{
	if (val < lo)
	{
		std::ostringstream ss;
		ss << "[ZoneOpt] Config: " << name << "=" << val << " clamped to min " << lo;
		LogMsg(ss.str());
		return lo;
	}
	if (val > hi)
	{
		std::ostringstream ss;
		ss << "[ZoneOpt] Config: " << name << "=" << val << " clamped to max " << hi;
		LogMsg(ss.str());
		return hi;
	}
	return val;
}


// =========================================================================
// LoadConfig — reads KenshiZoneOpt.ini, applies values with validation
// =========================================================================

void LoadConfig(const std::string& dllDir)
{
	std::string iniPath = dllDir + "KenshiZoneOpt.ini";
	FILE* f = NULL;
	fopen_s(&f, iniPath.c_str(), "r");
	if (!f)
	{
		LogMsg("[ZoneOpt] No INI file found, using defaults");
		return;
	}

	LogMsg("[ZoneOpt] Loading config from " + iniPath);
	int overrides = 0;

	char lineBuf[512];
	while (fgets(lineBuf, sizeof(lineBuf), f))
	{
		std::string line = TrimWhitespace(std::string(lineBuf));
		if (line.empty() || line[0] == '#' || line[0] == ';' || line[0] == '[')
			continue;

		size_t eq = line.find('=');
		if (eq == std::string::npos)
			continue;

		std::string key = TrimWhitespace(line.substr(0, eq));
		std::string val = TrimWhitespace(line.substr(eq + 1));
		if (key.empty() || val.empty())
			continue;

		bool matched = false;

		// --- Feature flags ---
		if (key == "deferral")            { bool b; if (ParseBool(val, &b)) { deferralEnabled = b; matched = true; } }
		else if (key == "priorityBoost")  { bool b; if (ParseBool(val, &b)) { priorityBoostEnabled = b; matched = true; } }
		else if (key == "preload")        { bool b; if (ParseBool(val, &b)) { preloadEnabled = b; matched = true; } }
		else if (key == "movementAware")  { bool b; if (ParseBool(val, &b)) { movementAwareEnabled = b; matched = true; } }
		else if (key == "caching")        { bool b; if (ParseBool(val, &b)) { cachingEnabled = b; matched = true; } }
#ifndef ZONEOPT_ZONEONLY
		else if (key == "groupCohesion")  { bool b; if (ParseBool(val, &b)) { groupCohesionEnabled = b; matched = true; } }
#endif
#if PATHFIND_STEP >= 1
		else if (key == "pathfindDiag")   { bool b; if (ParseBool(val, &b)) { pathfindDiagEnabled = b; matched = true; } }
		else if (key == "squadPathCache") { bool b; if (ParseBool(val, &b)) { squadPathCacheEnabled = b; matched = true; } }
#endif
#if PATHFIND_STEP >= 2
		else if (key == "stuckRetry")     { bool b; if (ParseBool(val, &b)) { stuckRetryEnabled = b; matched = true; } }
#endif
		else if (key == "islandFix")      { bool b; if (ParseBool(val, &b)) { islandFixEnabled = b; matched = true; } }

		// --- Tuning parameters ---
		else if (key == "preloadThreshold")
			{ float v; if (ParseFloat(val, &v)) { cfg_preloadThreshold = v; matched = true; } }
		else if (key == "edgeThreshold")
			{ float v; if (ParseFloat(val, &v)) { cfg_edgeThreshold = v; matched = true; } }
		else if (key == "charScanInterval")
			{ double v; if (ParseDouble(val, &v)) { cfg_charScanInterval = v; matched = true; } }
		else if (key == "baselineScanInterval")
			{ double v; if (ParseDouble(val, &v)) { cfg_baselineScanInterval = v; matched = true; } }
		else if (key == "activePollInterval")
			{ double v; if (ParseDouble(val, &v)) { cfg_activePollInterval = v; matched = true; } }
		else if (key == "gatherTimeout")
			{ double v; if (ParseDouble(val, &v)) { cfg_gatherTimeout = v; matched = true; } }
		else if (key == "formationTimeout")
			{ double v; if (ParseDouble(val, &v)) { cfg_formationTimeout = v; matched = true; } }
		else if (key == "scatterApproachDistSq")
			{ float v; if (ParseFloat(val, &v)) { cfg_scatterApproachDistSq = v; matched = true; } }
		else if (key == "gatherRadiusSq")
			{ float v; if (ParseFloat(val, &v)) { cfg_gatherRadiusSq = v; matched = true; } }
		else if (key == "navmeshWorkerCount")
			{ int v; if (ParseInt(val, &v)) { cfg_navmeshWorkerCount = v; matched = true; } }
		else if (key == "camLogInterval")
			{ double v; if (ParseDouble(val, &v)) { cfg_camLogInterval = v; matched = true; } }
		else if (key == "reprioritizeInterval")
			{ double v; if (ParseDouble(val, &v)) { cfg_reprioritizeInterval = v; matched = true; } }
		else if (key == "evictInterval")
			{ double v; if (ParseDouble(val, &v)) { cfg_evictInterval = v; matched = true; } }
		else if (key == "islandModRadius")
			{ int v; if (ParseInt(val, &v)) { cfg_islandModRadius = v; matched = true; } }
#ifdef ZONEOPT_DEBUG
		else if (key == "islandTestPromoteDelay")
			{ double v; if (ParseDouble(val, &v)) { cfg_islandTestPromoteDelay = v; matched = true; } }
#endif

		// --- Capacity (requires restart) ---
		else if (key == "maxPreloaded")
			{ int v; if (ParseInt(val, &v)) { cfg_maxPreloaded = v; matched = true; } }
		else if (key == "maxWatched")
			{ int v; if (ParseInt(val, &v)) { cfg_maxWatched = v; matched = true; } }
		else if (key == "maxFormationGroups")
			{ int v; if (ParseInt(val, &v)) { cfg_maxFormationGroups = v; matched = true; } }
		else if (key == "maxFormationMembers")
			{ int v; if (ParseInt(val, &v)) { cfg_maxFormationMembers = v; matched = true; } }
		else if (key == "maxCharZones")
			{ int v; if (ParseInt(val, &v)) { cfg_maxCharZones = v; matched = true; } }
		else if (key == "maxPendingOrder")
			{ int v; if (ParseInt(val, &v)) { cfg_maxPendingOrder = v; matched = true; } }
		else if (key == "cameraReserved")
			{ int v; if (ParseInt(val, &v)) { cfg_cameraReserved = v; matched = true; } }

		if (matched)
		{
			overrides++;
		}
		else
		{
			LogMsg("[ZoneOpt] Config: unknown or invalid key '" + key + "'");
		}
	}

	fclose(f);

	// --- Validate and clamp ---
	cfg_preloadThreshold      = ClampFloat("preloadThreshold", cfg_preloadThreshold, 500.0f, 4000.0f);
	cfg_edgeThreshold         = ClampFloat("edgeThreshold", cfg_edgeThreshold, 200.0f, 2300.0f);
	cfg_charScanInterval      = ClampDouble("charScanInterval", cfg_charScanInterval, 0.5, 30.0);
	cfg_baselineScanInterval  = ClampDouble("baselineScanInterval", cfg_baselineScanInterval, 1.0, 60.0);
	cfg_activePollInterval    = ClampDouble("activePollInterval", cfg_activePollInterval, 0.1, 10.0);
	cfg_gatherTimeout         = ClampDouble("gatherTimeout", cfg_gatherTimeout, 2.0, 120.0);
	cfg_formationTimeout      = ClampDouble("formationTimeout", cfg_formationTimeout, 10.0, 600.0);
	cfg_scatterApproachDistSq = ClampFloat("scatterApproachDistSq", cfg_scatterApproachDistSq, 100.0f, 40000.0f);
	cfg_gatherRadiusSq        = ClampFloat("gatherRadiusSq", cfg_gatherRadiusSq, 100.0f, 40000.0f);
	cfg_navmeshWorkerCount    = ClampInt("navmeshWorkerCount", cfg_navmeshWorkerCount, 1, NAVMESH_WORKER_COUNT);
	cfg_camLogInterval        = ClampDouble("camLogInterval", cfg_camLogInterval, 1.0, 300.0);
	cfg_reprioritizeInterval  = ClampDouble("reprioritizeInterval", cfg_reprioritizeInterval, 1.0, 30.0);
	cfg_evictInterval         = ClampDouble("evictInterval", cfg_evictInterval, 0.5, 30.0);
	cfg_islandModRadius       = ClampInt("islandModRadius", cfg_islandModRadius, 0, 8);
#ifdef ZONEOPT_DEBUG
	cfg_islandTestPromoteDelay = ClampDouble("islandTestPromoteDelay", cfg_islandTestPromoteDelay, 0.0, 60.0);
#endif

	cfg_maxPreloaded          = ClampInt("maxPreloaded", cfg_maxPreloaded, 9, 128);
	cfg_maxWatched            = ClampInt("maxWatched", cfg_maxWatched, 4, 128);
	cfg_maxFormationGroups    = ClampInt("maxFormationGroups", cfg_maxFormationGroups, 1, 16);
	cfg_maxFormationMembers   = ClampInt("maxFormationMembers", cfg_maxFormationMembers, 4, MAX_FORMATION_MEMBERS_LIMIT);
	cfg_maxCharZones          = ClampInt("maxCharZones", cfg_maxCharZones, 4, 64);
	cfg_maxPendingOrder       = ClampInt("maxPendingOrder", cfg_maxPendingOrder, 4, 64);
	cfg_cameraReserved        = ClampInt("cameraReserved", cfg_cameraReserved, 4, 36);

	// Cross-parameter validation
	if (cfg_cameraReserved > cfg_maxPreloaded)
	{
		LogMsg("[ZoneOpt] Config: cameraReserved clamped to maxPreloaded");
		cfg_cameraReserved = cfg_maxPreloaded;
	}
	if (cfg_formationTimeout < cfg_gatherTimeout + 5.0)
	{
		cfg_formationTimeout = cfg_gatherTimeout + 5.0;
		std::ostringstream ss;
		ss << "[ZoneOpt] Config: formationTimeout raised to " << cfg_formationTimeout
		   << " (must exceed gatherTimeout + 5)";
		LogMsg(ss.str());
	}

	if (overrides > 0)
	{
		std::ostringstream ss;
		ss << "[ZoneOpt] Config: " << overrides << " setting(s) loaded from INI";
		LogMsg(ss.str());
	}
}
