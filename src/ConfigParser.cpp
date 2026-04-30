#include "ConfigParser.hpp"
#include <fstream>
#include <sstream>
#include <cmath>
#include <unordered_map>

namespace dm {

// ============================================================
// parseKV — read a key=value config file into a map.
//
// Rules:
//   - Blank lines and lines starting with '#' are skipped.
//   - Windows-style '\r\n' line endings are handled.
//   - Inline comments (# after value) are stripped.
//   - Leading/trailing spaces on keys and values are trimmed.
//   - Duplicate keys: last value wins.
//   - Lines without '=' are reported as recoverable errors.
// ============================================================
static std::unordered_map<std::string, std::string>
parseKV(const std::string& filename, std::string& errorsOut)
{
    std::unordered_map<std::string, std::string> kv;

    std::ifstream file(filename);
    if (!file.is_open()) {
        errorsOut += "[ConfigParser] Cannot open: " + filename + "\n";
        return kv;
    }

    auto trim = [](std::string& s) {
        while (!s.empty() && (s.front() == ' ' || s.front() == '\t')) s.erase(s.begin());
        while (!s.empty() && (s.back()  == ' ' || s.back()  == '\t')) s.pop_back();
    };

    std::string line;
    int lineNum = 0;
    while (std::getline(file, line)) {
        ++lineNum;
        // Strip trailing '\r' (Windows line endings)
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty() || line[0] == '#') continue;

        const auto eq = line.find('=');
        if (eq == std::string::npos) {
            errorsOut += "[ConfigParser] " + filename + " line "
                       + std::to_string(lineNum) + ": missing '=', skipped\n";
            continue;
        }

        std::string key = line.substr(0, eq);
        std::string val = line.substr(eq + 1);

        // Strip inline comment
        const auto hashPos = val.find('#');
        if (hashPos != std::string::npos) val = val.substr(0, hashPos);

        trim(key);
        trim(val);
        if (!key.empty()) kv[key] = val;
    }
    return kv;
}

// ---- Typed helpers with fallback and error logging ----

static double readDouble(const std::unordered_map<std::string, std::string>& kv,
                         const std::string& key, double fallback,
                         std::string& errorsOut)
{
    const auto it = kv.find(key);
    if (it == kv.end()) {
        errorsOut += "[ConfigParser] Missing key '" + key
                   + "', using default " + std::to_string(fallback) + "\n";
        return fallback;
    }
    try {
        return std::stod(it->second);
    } catch (...) {
        errorsOut += "[ConfigParser] Bad value for '" + key + "': '"
                   + it->second + "', using default "
                   + std::to_string(fallback) + "\n";
        return fallback;
    }
}

static int readInt(const std::unordered_map<std::string, std::string>& kv,
                   const std::string& key, int fallback,
                   std::string& errorsOut)
{
    const auto it = kv.find(key);
    if (it == kv.end()) {
        errorsOut += "[ConfigParser] Missing key '" + key
                   + "', using default " + std::to_string(fallback) + "\n";
        return fallback;
    }
    try {
        return std::stoi(it->second);
    } catch (...) {
        errorsOut += "[ConfigParser] Bad value for '" + key + "': '"
                   + it->second + "', using default "
                   + std::to_string(fallback) + "\n";
        return fallback;
    }
}

// Returns true if the key exists in the map (used for optional keys)
static bool hasKey(const std::unordered_map<std::string, std::string>& kv,
                   const std::string& key)
{
    return kv.find(key) != kv.end();
}

// ============================================================
// parseDroneConfig
// ============================================================
bool parseDroneConfig(const std::string& filename,
                      DroneConfig& cfg,
                      std::string& errorsOut)
{
    const auto kv = parseKV(filename, errorsOut);

    cfg.width      = readDouble(kv, "width",       30.0, errorsOut) * cm;
    cfg.length     = readDouble(kv, "length",      30.0, errorsOut) * cm;
    cfg.height     = readDouble(kv, "height",      10.0, errorsOut) * cm;
    cfg.maxRotate  = readDouble(kv, "max_rotate",  90.0, errorsOut) * deg;
    cfg.maxAdvance = readDouble(kv, "max_advance", 50.0, errorsOut) * cm;
    cfg.maxElevate = readDouble(kv, "max_elevate", 30.0, errorsOut) * cm;
    cfg.lidarZmin  = readDouble(kv, "lidar_zmin",  20.0, errorsOut) * cm;
    cfg.lidarZmax  = readDouble(kv, "lidar_zmax", 120.0, errorsOut) * cm;
    cfg.lidarD     = readDouble(kv, "lidar_d",      2.5, errorsOut);
    cfg.lidarFOVC  = readInt   (kv, "lidar_fovc",    5,  errorsOut);

    // Validate Lidar range order
    if (cfg.lidarZmax.numerical_value_in(cm) <= cfg.lidarZmin.numerical_value_in(cm)) {
        errorsOut += "[ConfigParser] lidar_zmax must be > lidar_zmin; swapping values\n";
        std::swap(cfg.lidarZmax, cfg.lidarZmin);
    }

    // FOVC must be at least 1 (circle 0 = central beam only)
    if (cfg.lidarFOVC < 1) {
        errorsOut += "[ConfigParser] lidar_fovc < 1, setting to 1\n";
        cfg.lidarFOVC = 1;
    }

    return true;  // always recoverable
}

// ============================================================
// parseMissionConfig
// ============================================================
bool parseMissionConfig(const std::string& filename,
                        MissionConfig& cfg,
                        std::string& errorsOut)
{
    const auto kv = parseKV(filename, errorsOut);

    cfg.minX      = readDouble(kv, "min_x",      -500.0, errorsOut);
    cfg.maxX      = readDouble(kv, "max_x",       500.0, errorsOut);
    cfg.minY      = readDouble(kv, "min_y",      -500.0, errorsOut);
    cfg.maxY      = readDouble(kv, "max_y",       500.0, errorsOut);
    cfg.minHeight = readDouble(kv, "min_height",    0.0, errorsOut);
    cfg.maxHeight = readDouble(kv, "max_height",  300.0, errorsOut);
    cfg.resX      = readInt   (kv, "res_x",          0,  errorsOut);
    cfg.resY      = readInt   (kv, "res_y",          0,  errorsOut);
    cfg.resHeight = readInt   (kv, "res_height",     0,  errorsOut);

    // Clamp negative resolution values
    if (cfg.resX < 0)      { errorsOut += "[ConfigParser] res_x < 0, set to 0\n"; cfg.resX = 0; }
    if (cfg.resY < 0)      { errorsOut += "[ConfigParser] res_y < 0, set to 0\n"; cfg.resY = 0; }
    if (cfg.resHeight < 0) { errorsOut += "[ConfigParser] res_height < 0, set to 0\n"; cfg.resHeight = 0; }

    cfg.computeSteps();

    // Optional starting position (default: computed later by Simulator)
    cfg.startSet = hasKey(kv, "start_x") &&
                   hasKey(kv, "start_y") &&
                   hasKey(kv, "start_z");
    if (cfg.startSet) {
        cfg.startX = readDouble(kv, "start_x", 0.0, errorsOut);
        cfg.startY = readDouble(kv, "start_y", 0.0, errorsOut);
        cfg.startZ = readDouble(kv, "start_z", 0.0, errorsOut);
    }

    return true;
}

// ============================================================
// toGrid — convert continuous coordinates to integer grid indices.
//
// Division by step size then round-to-nearest ensures that
// coordinates within ±0.5*step of a grid line map to the same
// integer, giving stable voxel identity.
// ============================================================
GridPoint toGrid(double x, double y, double z,
                 const MissionConfig& mc) noexcept
{
    return GridPoint{
        static_cast<int>(std::round(x / mc.stepX)),
        static_cast<int>(std::round(y / mc.stepY)),
        static_cast<int>(std::round(z / mc.stepZ))
    };
}

} // namespace dm
