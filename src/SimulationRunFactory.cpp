#include <drone_mapper/SimulationRunFactory.h>

#include <drone_mapper/DroneControlImpl.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MappingAlgorithmImpl.h>
#include <drone_mapper/MissionControlImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockLidar.h>
#include <drone_mapper/MockMovement.h>
#include <drone_mapper/SimulationRunImpl.h>

#include <TinyNPY.h>

#include <filesystem>
#include <memory>
#include <sstream>
#include <stdexcept>

namespace drone_mapper {

namespace {

struct ResolutionResult {
    PhysicalLength resolution;
    types::ResolutionRequestStatus status;
};

ResolutionResult computeResolution(const types::MissionConfigData& mission,
                                   PhysicalLength map_resolution)
{
    const double gps_res = mission.gps_resolution.numerical_value_in(cm);
    const double factor  = mission.output_mapping_resolution_factor;
    const double map_res = map_resolution.numerical_value_in(cm);

    if (factor < 1.0) {
        return {gps_res * cm, types::ResolutionRequestStatus::Ignored};
    }
    const double actual = gps_res * factor;
    if (actual < map_res / 2.0) {
        return {gps_res * cm, types::ResolutionRequestStatus::IgnoredTooSmall};
    }
    return {actual * cm, types::ResolutionRequestStatus::Accepted};
}

std::filesystem::path makeRunDir(const std::filesystem::path& output_path,
                                 const types::SimulationConfigData& sim,
                                 const types::MissionConfigData& mission,
                                 const types::DroneConfigData& drone,
                                 const types::LidarConfigData& lidar)
{
    std::ostringstream oss;
    oss << sim.map_filename.stem().string()
        << "_ms" << mission.max_steps
        << "_d"  << static_cast<int>(drone.radius.numerical_value_in(cm))
        << "_fovc" << lidar.fov_circles;
    return output_path / "output_results" / oss.str();
}

} // namespace

std::unique_ptr<ISimulationRun>
SimulationRunFactory::create(const types::SimulationConfigData& simulation,
                                 const types::MissionConfigData& mission,
                                 const types::DroneConfigData& drone,
                                 const types::LidarConfigData& lidar,
                                 const std::filesystem::path& output_path)
{
    // Load ground truth map from NPY file
    auto npy = std::make_shared<NpyArray>();
    const char* err = npy->LoadNPY(simulation.map_filename.string().c_str());
    if (err != nullptr) {
        throw std::runtime_error(std::string("Failed to load map: ") + err);
    }

    // Build MapConfig for the ground truth map
    const types::MapConfig hidden_map_config = [&] {
        const auto& sh = npy->Shape();
        if (sh.size() != 3) throw std::runtime_error("Map must be 3D");
        const double res = simulation.map_resolution.numerical_value_in(cm);
        const double ox  = simulation.map_offset.x.numerical_value_in(cm);
        const double oy  = simulation.map_offset.y.numerical_value_in(cm);
        const double oz  = simulation.map_offset.z.numerical_value_in(cm);
        types::MapConfig cfg;
        cfg.resolution = simulation.map_resolution;
        cfg.offset     = simulation.map_offset;
        cfg.boundaries.min_x      = ox  * x_extent[cm];
        cfg.boundaries.max_x      = (ox  + (static_cast<int>(sh[0])-1)*res) * x_extent[cm];
        cfg.boundaries.min_y      = oy  * y_extent[cm];
        cfg.boundaries.max_y      = (oy  + (static_cast<int>(sh[1])-1)*res) * y_extent[cm];
        cfg.boundaries.min_height = oz  * z_extent[cm];
        cfg.boundaries.max_height = (oz  + (static_cast<int>(sh[2])-1)*res) * z_extent[cm];
        return cfg;
    }();

    auto hidden_map = std::make_unique<Map3DImpl>(npy, hidden_map_config);

    // Compute output resolution
    const auto res_result = computeResolution(mission, simulation.map_resolution);

    // Output map uses same spatial extent as hidden map but possibly different resolution.
    // If the mission specifies exploration_boundaries, clip the output map to that zone.
    types::MapConfig output_map_config = hidden_map_config;
    output_map_config.resolution = res_result.resolution;
    if (mission.exploration_boundaries) {
        const auto& eb = *mission.exploration_boundaries;
        const auto& hb = hidden_map_config.boundaries;
        // Clamp to hidden map bounds so we never go outside the loaded NPY.
        auto clampX = [](auto v, auto lo, auto hi) { return v < lo ? lo : v > hi ? hi : v; };
        output_map_config.boundaries.min_x      = clampX(eb.min_x,      hb.min_x,      hb.max_x);
        output_map_config.boundaries.max_x      = clampX(eb.max_x,      hb.min_x,      hb.max_x);
        output_map_config.boundaries.min_y      = clampX(eb.min_y,      hb.min_y,      hb.max_y);
        output_map_config.boundaries.max_y      = clampX(eb.max_y,      hb.min_y,      hb.max_y);
        output_map_config.boundaries.min_height = clampX(eb.min_height, hb.min_height, hb.max_height);
        output_map_config.boundaries.max_height = clampX(eb.max_height, hb.min_height, hb.max_height);
    }
    auto output_map = std::make_unique<Map3DImpl>(output_map_config);

    // Per-run output directory
    const auto run_dir = makeRunDir(output_path, simulation, mission, drone, lidar);
    std::filesystem::create_directories(run_dir);
    const auto output_map_file = run_dir / "output_map.npy";

    // Components
    auto gps = std::make_unique<MockGPS>(
        simulation.initial_drone_position,
        Orientation{simulation.initial_angle, 0.0 * altitude_angle[deg]});

    auto movement = std::make_unique<MockMovement>(*gps, *hidden_map, drone);
    auto lidar_impl = std::make_unique<MockLidar>(lidar, *hidden_map, *gps);
    auto mapping_algo = std::make_unique<MappingAlgorithmImpl>(drone, *output_map);
    auto drone_ctrl = std::make_unique<DroneControlImpl>(
        drone, mission, *lidar_impl, *gps, *movement, *output_map, *mapping_algo);
    auto mission_ctrl = std::make_unique<MissionControlImpl>(mission, *drone_ctrl);

    return std::make_unique<SimulationRunImpl>(
        simulation, mission, drone, lidar,
        res_result.status,
        std::move(hidden_map), std::move(output_map),
        std::move(gps), std::move(movement), std::move(lidar_impl),
        std::move(mapping_algo), std::move(drone_ctrl),
        std::move(mission_ctrl), output_map_file);
}

} // namespace drone_mapper
