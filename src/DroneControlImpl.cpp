#include <drone_mapper/DroneControlImpl.h>

#include <drone_mapper/ScanResultToVoxels.h>

#include <utility>

namespace drone_mapper {

DroneControlImpl::DroneControlImpl(types::DroneConfigData drone,
                                   types::MissionConfigData mission,
                                   ILidar& lidar,
                                   IGPS& gps,
                                   IDroneMovement& movement,
                                   IMutableMap3D& output_map,
                                   IMappingAlgorithm& mapping_algorithm)
    : drone_(std::move(drone))
    , mission_(std::move(mission))
    , lidar_(lidar)
    , gps_(gps)
    , movement_(movement)
    , output_map_(output_map)
    , mapping_algorithm_(mapping_algorithm)
{}

types::DroneStepResult DroneControlImpl::step() {
    // 1. Ask the algorithm what to do next (pass last scan result if available).
    const types::LidarScanResult* scan_ptr = last_scan_ ? &*last_scan_ : nullptr;
    const auto cmd = mapping_algorithm_.nextStep(state(), scan_ptr);

    // 2. Execute movement if the algorithm requested one.
    if (cmd.movement) {
        const auto& mv = *cmd.movement;
        switch (mv.type) {
        case types::MovementCommandType::Rotate:
            movement_.rotate(mv.rotation, mv.angle);
            break;
        case types::MovementCommandType::Advance:
            movement_.advance(mv.distance);
            break;
        case types::MovementCommandType::Elevate:
            movement_.elevate(mv.distance);
            break;
        default:
            break;
        }
    }

    // 3. Execute scan if the algorithm requested one, then update the output map.
    if (cmd.scan_orientation) {
        auto hits = lidar_.scan(*cmd.scan_orientation);
        const auto voxels = ScanResultToVoxels::convert(gps_.position(), gps_.heading(), hits);
        for (const auto& v : voxels) {
            output_map_.set(v.position, v.value);
        }
        last_scan_ = std::move(hits);
    } else {
        last_scan_.reset();
    }

    ++step_index_;

    // 4. Map AlgorithmStatus to DroneStepResult.
    if (cmd.status == types::AlgorithmStatus::Finished ||
        cmd.status == types::AlgorithmStatus::FinishedWithUnmappableVoxels) {
        return {types::DroneStepStatus::Completed, "Exploration complete"};
    }
    return {types::DroneStepStatus::Continue, {}};
}

types::DroneState DroneControlImpl::state() const {
    return {gps_.position(), gps_.heading(), step_index_};
}

} // namespace drone_mapper
