<<<<<<< HEAD
#include "ConfigParser.hpp"
#include <yaml-cpp/yaml.h>
#include <cmath>
=======
#include <drone_mapper/ConfigParser.h>
>>>>>>> fd647a1d3568830a42b0e5889c5b3a9021e753c3

#include <yaml-cpp/yaml.h>

<<<<<<< HEAD
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
=======
#include <filesystem>
#include <stdexcept>

namespace drone_mapper {

namespace {

double yamlDouble(const YAML::Node& node, const std::string& key,
                  double fallback, std::string& err)
{
    if (!node || !node.IsDefined() || node.IsNull()) {
        err += "[ConfigParser] Missing key '" + key + "', using " + std::to_string(fallback) + "\n";
        return fallback;
    }
    try { return node.as<double>(); }
    catch (...) {
        err += "[ConfigParser] Bad value for '" + key + "', using " + std::to_string(fallback) + "\n";
>>>>>>> fd647a1d3568830a42b0e5889c5b3a9021e753c3
        return fallback;
    }
}

<<<<<<< HEAD
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
=======
int yamlInt(const YAML::Node& node, const std::string& key,
            int fallback, std::string& err)
{
    if (!node || !node.IsDefined() || node.IsNull()) {
        err += "[ConfigParser] Missing key '" + key + "', using " + std::to_string(fallback) + "\n";
        return fallback;
    }
    try { return node.as<int>(); }
    catch (...) {
        err += "[ConfigParser] Bad value for '" + key + "', using " + std::to_string(fallback) + "\n";
>>>>>>> fd647a1d3568830a42b0e5889c5b3a9021e753c3
        return fallback;
    }
}

<<<<<<< HEAD
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
=======
std::string yamlStr(const YAML::Node& node, const std::string& key,
                    const std::string& fallback, std::string& err)
{
    if (!node || !node.IsDefined() || node.IsNull()) {
        err += "[ConfigParser] Missing key '" + key + "', using '" + fallback + "'\n";
        return fallback;
    }
    try { return node.as<std::string>(); }
    catch (...) {
        err += "[ConfigParser] Bad value for '" + key + "', using '" + fallback + "'\n";
        return fallback;
    }
}

} // namespace

types::DroneConfigData parseDroneConfig(const std::filesystem::path& path, std::string& err) {
    YAML::Node root;
    try { root = YAML::LoadFile(path.string()); }
    catch (const YAML::Exception& e) {
        err += "[ConfigParser] Cannot parse drone config '" + path.string() + "': " + e.what() + "\n";
        return {};
    }
    const YAML::Node dc = root["drone_config"];
    if (!dc) { err += "[ConfigParser] Missing 'drone_config' in " + path.string() + "\n"; return {}; }

    types::DroneConfigData cfg;
    cfg.dimensions  = yamlDouble(dc["dimensions_cm"],  "drone_config.dimensions_cm",  30.0, err) * cm;
    cfg.max_rotate  = yamlDouble(dc["max_rotate_deg"], "drone_config.max_rotate_deg", 45.0, err) * horizontal_angle[deg];
    cfg.max_advance = yamlDouble(dc["max_advance_cm"], "drone_config.max_advance_cm", 50.0, err) * cm;
    cfg.max_elevate = yamlDouble(dc["max_elevate_cm"], "drone_config.max_elevate_cm", 40.0, err) * cm;
    return cfg;
}

types::LidarConfigData parseLidarConfig(const std::filesystem::path& path, std::string& err) {
    YAML::Node root;
    try { root = YAML::LoadFile(path.string()); }
    catch (const YAML::Exception& e) {
        err += "[ConfigParser] Cannot parse lidar config '" + path.string() + "': " + e.what() + "\n";
        return {};
    }
    const YAML::Node lc = root["lidar_config"];
    if (!lc) { err += "[ConfigParser] Missing 'lidar_config' in " + path.string() + "\n"; return {}; }

    types::LidarConfigData cfg;
    cfg.z_min       = yamlDouble(lc["z_min_cm"],   "lidar_config.z_min_cm",   20.0, err) * cm;
    cfg.z_max       = yamlDouble(lc["z_max_cm"],   "lidar_config.z_max_cm",  120.0, err) * cm;
    cfg.d           = yamlDouble(lc["d_cm"],        "lidar_config.d_cm",       2.5,  err) * cm;
    cfg.fov_circles = static_cast<std::size_t>(yamlInt(lc["fov_circles"], "lidar_config.fov_circles", 5, err));
    return cfg;
}

types::MissionConfigData parseMissionConfig(const std::filesystem::path& path, std::string& err) {
    YAML::Node root;
    try { root = YAML::LoadFile(path.string()); }
    catch (const YAML::Exception& e) {
        err += "[ConfigParser] Cannot parse mission config '" + path.string() + "': " + e.what() + "\n";
        return {};
    }
    const YAML::Node mc = root["mission_config"];
    if (!mc) { err += "[ConfigParser] Missing 'mission_config' in " + path.string() + "\n"; return {}; }

    types::MissionConfigData cfg;
    cfg.max_steps = static_cast<std::size_t>(yamlInt(mc["max_steps"], "mission_config.max_steps", 2400, err));
    cfg.gps_resolution = yamlDouble(mc["gps_resolution_cm"], "mission_config.gps_resolution_cm", 10.0, err) * cm;
    cfg.output_mapping_resolution_factor = yamlDouble(mc["output_mapping_resolution_factor"],
        "mission_config.output_mapping_resolution_factor", 1.0, err);
    if (cfg.output_mapping_resolution_factor < 1.0) {
        err += "[ConfigParser] output_mapping_resolution_factor < 1, using 1\n";
        cfg.output_mapping_resolution_factor = 1.0;
    }
    return cfg;
}

types::SimulationConfigData parseSimulationConfig(const std::filesystem::path& path, std::string& err) {
    YAML::Node root;
    try { root = YAML::LoadFile(path.string()); }
    catch (const YAML::Exception& e) {
        err += "[ConfigParser] Cannot parse simulation config '" + path.string() + "': " + e.what() + "\n";
        return {};
    }
    const YAML::Node sc = root["simulation_config"];
    if (!sc) { err += "[ConfigParser] Missing 'simulation_config' in " + path.string() + "\n"; return {}; }

    const std::string map_file = yamlStr(sc["map_filename"], "simulation_config.map_filename", "maps/map.npy", err);
    const double res = yamlDouble(sc["map_resolution_cm"], "simulation_config.map_resolution_cm", 10.0, err);

    const YAML::Node pos = sc["initial_drone_position"];
    const double px = yamlDouble(pos["x_cm"],      "initial_drone_position.x_cm",      0.0, err);
    const double py = yamlDouble(pos["y_cm"],       "initial_drone_position.y_cm",      0.0, err);
    const double pz = yamlDouble(pos["height_cm"], "initial_drone_position.height_cm", 0.0, err);
    const double angle = yamlDouble(sc["initial_angle_deg"], "simulation_config.initial_angle_deg", 0.0, err);

    // map_axes_offset (optional)
    const YAML::Node off_node = sc["map_axes_offset"];
    double offX = 0.0, offY = 0.0, offZ = 0.0;
    if (off_node && off_node.IsDefined()) {
        offX = yamlDouble(off_node["x_offset"],      "map_axes_offset.x_offset",      0.0, err);
        offY = yamlDouble(off_node["y_offset"],      "map_axes_offset.y_offset",      0.0, err);
        offZ = yamlDouble(off_node["height_offset"], "map_axes_offset.height_offset", 0.0, err);
    }

    std::filesystem::path map_path{map_file};
    if (!map_path.is_absolute()) map_path = path.parent_path() / map_path;

    types::SimulationConfigData cfg;
    cfg.map_filename           = map_path;
    cfg.map_resolution         = res * cm;
    cfg.map_offset             = Position3D{offX * x_extent[cm], offY * y_extent[cm], offZ * z_extent[cm]};
    cfg.initial_drone_position = Position3D{px * x_extent[cm], py * y_extent[cm], pz * z_extent[cm]};
    cfg.initial_angle          = angle * horizontal_angle[deg];
    return cfg;
}

types::SimulationCompositionData parseCompositionConfig(const std::filesystem::path& path, std::string& err) {
    YAML::Node root;
    try { root = YAML::LoadFile(path.string()); }
    catch (const YAML::Exception& e) {
        throw std::runtime_error("[ConfigParser] Cannot parse composition '" + path.string() + "': " + e.what());
    }
    const YAML::Node comp = root["simulation_compositions"];
    if (!comp) {
        throw std::runtime_error("[ConfigParser] Missing 'simulation_compositions' in " + path.string());
    }

    const std::filesystem::path base = path.parent_path();

    std::vector<types::SimulationConfigData> simulations;
    std::vector<types::MissionConfigData> missions;

    for (const auto& sim_entry : comp["simulations"]) {
        std::filesystem::path sp{sim_entry["simulation_config"].as<std::string>()};
        if (!sp.is_absolute()) sp = base / sp;
        simulations.push_back(parseSimulationConfig(sp, err));

        for (const auto& mf : sim_entry["mission_configs"]) {
            std::filesystem::path mp{mf.as<std::string>()};
            if (!mp.is_absolute()) mp = base / mp;
            missions.push_back(parseMissionConfig(mp, err));
        }
    }

    std::vector<types::DroneConfigData> drones;
    for (const auto& df : comp["drone_configs"]) {
        std::filesystem::path dp{df.as<std::string>()};
        if (!dp.is_absolute()) dp = base / dp;
        drones.push_back(parseDroneConfig(dp, err));
    }

    std::vector<types::LidarConfigData> lidars;
    for (const auto& lf : comp["lidar_configs"]) {
        std::filesystem::path lp{lf.as<std::string>()};
        if (!lp.is_absolute()) lp = base / lp;
        lidars.push_back(parseLidarConfig(lp, err));
    }

    types::SimulationCompositionData data;
    data.composition_file = path;
    data.simulations = std::move(simulations);
    data.missions    = std::move(missions);
    data.drones      = std::move(drones);
    data.lidars      = std::move(lidars);
    return data;
}

} // namespace drone_mapper
>>>>>>> fd647a1d3568830a42b0e5889c5b3a9021e753c3
