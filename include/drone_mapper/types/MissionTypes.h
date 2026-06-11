#pragma once

#include <drone_mapper/Units.h>

#include <cstddef>
#include <string>
#include <vector>

namespace drone_mapper::types {

// Changed: boundaries removed — map bounds now live on MapConfig/IMap3D.
struct MissionConfigData {
    std::size_t max_steps = 0;
    PhysicalLength gps_resolution{};
    double output_mapping_resolution_factor = 1.0;
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
