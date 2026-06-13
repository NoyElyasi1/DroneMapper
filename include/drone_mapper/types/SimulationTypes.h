#pragma once

#include <drone_mapper/types/DroneTypes.h>
#include <drone_mapper/types/LidarTypes.h>
#include <drone_mapper/types/MapTypes.h>
#include <drone_mapper/types/MissionTypes.h>

#include <filesystem>
#include <string>
#include <tuple>
#include <vector>

namespace drone_mapper::types {

struct SimulationConfigData {
    std::filesystem::path config_file{};
    std::filesystem::path map_filename{};
    PhysicalLength map_resolution{};
    // Offset: moves the (0,0,0) of the npy matrix to this world point.
    Position3D map_offset{};
    Position3D initial_drone_position{};
    HorizontalAngle initial_angle{};
};

struct SimulationCompositionData {
    std::filesystem::path composition_file{};
    std::vector<SimulationConfigData> simulations{};
    std::vector<MissionConfigData> missions{};
    std::vector<DroneConfigData> drones{};
    std::vector<LidarConfigData> lidars{};
};

enum class ResolutionRequestStatus {
    Accepted,
    Ignored,
    IgnoredTooSmall,
};

struct SimulationResult {
    SimulationConfigData simulation_config{};
    MissionConfigData mission_config{};
    DroneConfigData drone_config{};
    LidarConfigData lidar_config{};
    ResolutionRequestStatus resolution_request_status = ResolutionRequestStatus::Ignored;
    std::vector<MissionRunResult> mission_results{};
    std::filesystem::path output_map_file{};
    MapConfig output_map_config{};
    double mission_score = 0.0;
};

// Replaces old SimulationReport with a flat run list + summary.
struct SimulationManagerReport {
    std::filesystem::path composition_file{};
    std::string generated_at_utc{};
    std::string metric{"output_map_accuracy"};
    std::tuple<double, double> score_range{0.0, 100.0};
    int error_score = -1;
    std::vector<SimulationResult> runs{};
};

} // namespace drone_mapper::types
