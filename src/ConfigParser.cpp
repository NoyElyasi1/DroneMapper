#include "ConfigParser.hpp"
#include <yaml-cpp/yaml.h>
#include <cmath>

namespace dm {

// ============================================================
// YAML helper — safely extract a scalar value from a node,
// falling back to 'fallback' and logging an error on any problem.
// ============================================================

static double yamlDouble(const YAML::Node& node,
                         const std::string& path,
                         double             fallback,
                         std::string&       errorsOut)
{
    if (!node || !node.IsDefined() || node.IsNull()) {
        errorsOut += "[ConfigParser] Missing key '" + path
                   + "', using default " + std::to_string(fallback) + "\n";
        return fallback;
    }
    try {
        return node.as<double>();
    } catch (...) {
        errorsOut += "[ConfigParser] Bad value for '" + path
                   + "', using default " + std::to_string(fallback) + "\n";
        return fallback;
    }
}

static int yamlInt(const YAML::Node& node,
                   const std::string& path,
                   int                fallback,
                   std::string&       errorsOut)
{
    if (!node || !node.IsDefined() || node.IsNull()) {
        errorsOut += "[ConfigParser] Missing key '" + path
                   + "', using default " + std::to_string(fallback) + "\n";
        return fallback;
    }
    try {
        return node.as<int>();
    } catch (...) {
        errorsOut += "[ConfigParser] Bad value for '" + path
                   + "', using default " + std::to_string(fallback) + "\n";
        return fallback;
    }
}

// ============================================================
// parseDroneConfig
// ============================================================
bool parseDroneConfig(const std::string& filename,
                      DroneConfig& cfg,
                      std::string& errorsOut)
{
    YAML::Node root;
    try {
        root = YAML::LoadFile(filename);
    } catch (const YAML::Exception& e) {
        errorsOut += "[ConfigParser] Cannot parse '" + filename
                   + "': " + e.what() + "\n";
        return true;  // recoverable — caller uses defaults already in cfg
    }

    const YAML::Node dc = root["drone_config"];
    if (!dc || !dc.IsDefined()) {
        errorsOut += "[ConfigParser] Missing 'drone_config' root key in "
                   + filename + "; using all defaults\n";
        return true;
    }

    cfg.dimensions = yamlDouble(dc["dimensions_cm"],  "drone_config.dimensions_cm",  30.0, errorsOut) * cm;
    cfg.maxRotate  = yamlDouble(dc["max_rotate_deg"], "drone_config.max_rotate_deg", 45.0, errorsOut) * deg;
    cfg.maxAdvance = yamlDouble(dc["max_advance_cm"], "drone_config.max_advance_cm", 50.0, errorsOut) * cm;
    cfg.maxElevate = yamlDouble(dc["max_elevate_cm"], "drone_config.max_elevate_cm", 40.0, errorsOut) * cm;

    // Lidar parameters are optional; keep existing defaults if absent.
    if (dc["lidar_zmin_cm"])
        cfg.lidarZmin = yamlDouble(dc["lidar_zmin_cm"], "drone_config.lidar_zmin_cm", 20.0, errorsOut) * cm;
    if (dc["lidar_zmax_cm"])
        cfg.lidarZmax = yamlDouble(dc["lidar_zmax_cm"], "drone_config.lidar_zmax_cm", 120.0, errorsOut) * cm;
    if (dc["lidar_d"])
        cfg.lidarD    = yamlDouble(dc["lidar_d"],        "drone_config.lidar_d",        2.5,  errorsOut);
    if (dc["lidar_fovc"])
        cfg.lidarFOVC = yamlInt   (dc["lidar_fovc"],     "drone_config.lidar_fovc",     5,    errorsOut);

    // Validate Lidar range order
    if (cfg.lidarZmax.numerical_value_in(cm) <= cfg.lidarZmin.numerical_value_in(cm)) {
        errorsOut += "[ConfigParser] lidar_zmax_cm must be > lidar_zmin_cm; swapping values\n";
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
    YAML::Node root;
    try {
        root = YAML::LoadFile(filename);
    } catch (const YAML::Exception& e) {
        errorsOut += "[ConfigParser] Cannot parse '" + filename
                   + "': " + e.what() + "\n";
        return true;  // recoverable
    }

    const YAML::Node mc = root["mission_config"];
    if (!mc || !mc.IsDefined()) {
        errorsOut += "[ConfigParser] Missing 'mission_config' root key in "
                   + filename + "; using all defaults\n";
        return true;
    }

    cfg.maxSteps = yamlInt(mc["max_steps"], "mission_config.max_steps", 2400, errorsOut);

    const YAML::Node bounds = mc["boundaries"];
    if (bounds && bounds.IsDefined()) {
        const YAML::Node xb = bounds["x_boundary"];
        const YAML::Node yb = bounds["y_boundary"];
        const YAML::Node hb = bounds["height_boundary"];

        cfg.minX      = yamlDouble(xb ? xb["min_cm"] : YAML::Node{}, "boundaries.x_boundary.min_cm",      -500.0, errorsOut);
        cfg.maxX      = yamlDouble(xb ? xb["max_cm"] : YAML::Node{}, "boundaries.x_boundary.max_cm",       500.0, errorsOut);
        cfg.minY      = yamlDouble(yb ? yb["min_cm"] : YAML::Node{}, "boundaries.y_boundary.min_cm",      -500.0, errorsOut);
        cfg.maxY      = yamlDouble(yb ? yb["max_cm"] : YAML::Node{}, "boundaries.y_boundary.max_cm",       500.0, errorsOut);
        cfg.minHeight = yamlDouble(hb ? hb["min_cm"] : YAML::Node{}, "boundaries.height_boundary.min_cm",    0.0, errorsOut);
        cfg.maxHeight = yamlDouble(hb ? hb["max_cm"] : YAML::Node{}, "boundaries.height_boundary.max_cm",  300.0, errorsOut);
    } else {
        errorsOut += "[ConfigParser] Missing 'boundaries' section in " + filename + "; using defaults\n";
    }

    cfg.gpsResolutionCm = yamlDouble(mc["gps_resolution_cm"],
                                     "mission_config.gps_resolution_cm", 10.0, errorsOut);

    if (mc["mapping_resolution_factor"])
        cfg.mappingResolutionFactor = yamlInt(mc["mapping_resolution_factor"],
                                              "mission_config.mapping_resolution_factor", 1, errorsOut);

    if (cfg.gpsResolutionCm <= 0.0) {
        errorsOut += "[ConfigParser] gps_resolution_cm must be > 0; using 10.0\n";
        cfg.gpsResolutionCm = 10.0;
    }
    if (cfg.mappingResolutionFactor < 1) {
        errorsOut += "[ConfigParser] mapping_resolution_factor < 1; using 1\n";
        cfg.mappingResolutionFactor = 1;
    }

    cfg.computeSteps();

    // Optional starting position
    const bool hasStart = mc["start_x_cm"] && mc["start_y_cm"] && mc["start_z_cm"];
    if (hasStart) {
        cfg.startSet = true;
        cfg.startX   = yamlDouble(mc["start_x_cm"], "mission_config.start_x_cm", 0.0, errorsOut);
        cfg.startY   = yamlDouble(mc["start_y_cm"], "mission_config.start_y_cm", 0.0, errorsOut);
        cfg.startZ   = yamlDouble(mc["start_z_cm"], "mission_config.start_z_cm", 0.0, errorsOut);
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
