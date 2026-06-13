#pragma once

#include <drone_mapper/Types.h>

namespace drone_mapper {

class IMap3D;

// **Do not change this interface.**
class IMappingAlgorithm {
public:
    IMappingAlgorithm(types::DroneConfigData drone_config, const IMap3D& output_map)
        : _drone_config(drone_config), _output_map(output_map) {}

    virtual ~IMappingAlgorithm() = default;

    // latest_scan may be nullptr on the first call or when no scan was performed last step.
    [[nodiscard]] virtual types::MappingStepCommand nextStep(const types::DroneState& state,
                                                             const types::LidarScanResult* latest_scan) = 0;

protected:
    const types::DroneConfigData _drone_config;
    const IMap3D& _output_map;
};

} // namespace drone_mapper
