#include <drone_mapper/ConfigParser.h>

#include <yaml-cpp/yaml.h>

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
        return fallback;
    }
}

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
        return fallback;
    }
}

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
    cfg.config_file = path;
    cfg.radius      = yamlDouble(dc["dimensions_cm"],  "drone_config.dimensions_cm",  30.0, err) * cm;
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
    cfg.config_file  = path;
    cfg.z_min        = yamlDouble(lc["z_min_cm"],   "lidar_config.z_min_cm",   20.0, err) * cm;
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
    cfg.config_file = path;
    cfg.max_steps = static_cast<std::size_t>(yamlInt(mc["max_steps"], "mission_config.max_steps", 2400, err));
    cfg.gps_resolution = yamlDouble(mc["gps_resolution_cm"], "mission_config.gps_resolution_cm", 10.0, err) * cm;
    cfg.output_mapping_resolution_factor = yamlDouble(mc["output_mapping_resolution_factor"],
        "mission_config.output_mapping_resolution_factor", 1.0, err);
    if (cfg.output_mapping_resolution_factor < 1.0) {
        err += "[ConfigParser] output_mapping_resolution_factor < 1, using 1\n";
        cfg.output_mapping_resolution_factor = 1.0;
    }

    // Optional exploration boundaries
    const YAML::Node bounds = mc["boundaries"];
    if (bounds && bounds.IsDefined()) {
        types::MappingBounds b;
        const YAML::Node xb = bounds["x_boundary"];
        const YAML::Node yb = bounds["y_boundary"];
        const YAML::Node hb = bounds["height_boundary"];
        b.min_x      = yamlDouble(xb["min_cm"],    "boundaries.x_boundary.min_cm",      -1e9, err) * x_extent[cm];
        b.max_x      = yamlDouble(xb["max_cm"],    "boundaries.x_boundary.max_cm",       1e9, err) * x_extent[cm];
        b.min_y      = yamlDouble(yb["min_cm"],    "boundaries.y_boundary.min_cm",      -1e9, err) * y_extent[cm];
        b.max_y      = yamlDouble(yb["max_cm"],    "boundaries.y_boundary.max_cm",       1e9, err) * y_extent[cm];
        b.min_height = yamlDouble(hb["min_cm"], "boundaries.height_boundary.min_cm",  -1e9, err) * z_extent[cm];
        b.max_height = yamlDouble(hb["max_cm"], "boundaries.height_boundary.max_cm",   1e9, err) * z_extent[cm];
        cfg.exploration_boundaries = b;
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
    cfg.config_file            = path;
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

namespace {

types::MapComparisonEntry parseMapComparisonEntry(const YAML::Node& node, const std::string& prefix,
                                                  std::string& err)
{
    types::MapComparisonEntry entry;
    entry.map_res = yamlDouble(node["map_res_cm"], prefix + ".map_res_cm", 1.0, err) * cm;

    const YAML::Node off = node["map_offset"];
    if (off && off.IsDefined()) {
        const double ox = yamlDouble(off["x_offset"],      prefix + ".map_offset.x_offset",      0.0, err);
        const double oy = yamlDouble(off["y_offset"],      prefix + ".map_offset.y_offset",      0.0, err);
        const double oz = yamlDouble(off["height_offset"], prefix + ".map_offset.height_offset", 0.0, err);
        entry.map_offset = Position3D{ox * x_extent[cm], oy * y_extent[cm], oz * z_extent[cm]};
    }

    const YAML::Node bounds = node["map_boundaries"];
    if (bounds && bounds.IsDefined()) {
        const YAML::Node xb = bounds["x_boundary"];
        const YAML::Node yb = bounds["y_boundary"];
        const YAML::Node hb = bounds["height_boundary"];
        types::MappingBounds b;
        b.min_x      = yamlDouble(xb["min_cm"],    prefix + ".x_boundary.min_cm",      -1e9, err) * x_extent[cm];
        b.max_x      = yamlDouble(xb["max_cm"],    prefix + ".x_boundary.max_cm",       1e9, err) * x_extent[cm];
        b.min_y      = yamlDouble(yb["min_cm"],    prefix + ".y_boundary.min_cm",      -1e9, err) * y_extent[cm];
        b.max_y      = yamlDouble(yb["max_cm"],    prefix + ".y_boundary.max_cm",       1e9, err) * y_extent[cm];
        b.min_height = yamlDouble(hb["min_cm"], prefix + ".height_boundary.min_cm",  -1e9, err) * z_extent[cm];
        b.max_height = yamlDouble(hb["max_cm"], prefix + ".height_boundary.max_cm",   1e9, err) * z_extent[cm];
        entry.map_boundaries = b;
    }
    return entry;
}

} // anonymous namespace

types::ComparisonConfigData parseComparisonConfig(const std::filesystem::path& path, std::string& err) {
    YAML::Node root;
    try { root = YAML::LoadFile(path.string()); }
    catch (const YAML::Exception& e) {
        err += "[ConfigParser] Cannot parse comparison config '" + path.string() + "': " + e.what() + "\n";
        return {};
    }
    const YAML::Node cc = root["comparison_config"];
    if (!cc) { err += "[ConfigParser] Missing 'comparison_config' in " + path.string() + "\n"; return {}; }

    types::ComparisonConfigData data;
    if (cc["original"] && cc["original"].IsDefined())
        data.original = parseMapComparisonEntry(cc["original"], "original", err);
    if (cc["target"] && cc["target"].IsDefined())
        data.target = parseMapComparisonEntry(cc["target"], "target", err);
    return data;
}

} // namespace drone_mapper
