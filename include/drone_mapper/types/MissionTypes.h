#pragma once

#include <drone_mapper/Units.h>
#include <drone_mapper/types/MapTypes.h>

#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace drone_mapper::types {

struct MissionConfigData {
    std::filesystem::path config_file{};
    std::size_t max_steps = 0;
    PhysicalLength gps_resolution{};
    double output_mapping_resolution_factor = 1.0;
    // Optional: constrain the exploration zone to a sub-region of the map.
    // When absent the full map boundaries (from MapConfig) are used.
    std::optional<MappingBounds> exploration_boundaries{};
};

enum class MissionRunStatus {
    Completed,
    MaxSteps,
    Error,
};

struct ErrorRef {
    std::string code{};
    std::string message{};
};

struct MissionRunResult {
    MissionRunStatus status = MissionRunStatus::Completed;
    std::size_t steps = 0;
    // score moved to SimulationResult; output_map_file moved to SimulationResult
    std::vector<ErrorRef> errors{};
};

} // namespace drone_mapper::types
