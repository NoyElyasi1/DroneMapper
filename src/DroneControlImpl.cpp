#include <drone_mapper/DroneControlImpl.h>

#include <drone_mapper/ScanResultToVoxels.h>

#include <utility>

namespace drone_mapper {

DroneControlImpl::DroneControlImpl(types::DroneConfigData /*drone*/,
                                   types::MissionConfigData /*mission*/,
                                   ILidar& lidar,
                                   IGPS& gps,
                                   IDroneMovement& movement,
                                   IMutableMap3D& output_map,
                                   IMappingAlgorithm& mapping_algorithm)
    : lidar_(lidar)
    , gps_(gps)
    , movement_(movement)
    , output_map_(output_map)
    , mapping_algorithm_(mapping_algorithm)
{}

types::DroneStepResult DroneControlImpl::step() {
    // 1. Scan in 10 directions (8 horizontal at 45° increments + up + down)
    types::LidarScanResult all_hits;
    for (int i = 0; i < 8; ++i) {
        const double angle_deg = static_cast<double>(i) * 45.0;
        auto hits = lidar_.scan(Orientation{
            angle_deg * horizontal_angle[deg],
            0.0 * altitude_angle[deg],
        });
        all_hits.insert(all_hits.end(), hits.begin(), hits.end());
    }
    {
        auto up = lidar_.scan(Orientation{0.0 * horizontal_angle[deg],  90.0 * altitude_angle[deg]});
        auto dn = lidar_.scan(Orientation{0.0 * horizontal_angle[deg], -90.0 * altitude_angle[deg]});
        all_hits.insert(all_hits.end(), up.begin(), up.end());
        all_hits.insert(all_hits.end(), dn.begin(), dn.end());
    }

    // 2. Convert to voxels
    const auto voxels = ScanResultToVoxels::convert(gps_.position(), gps_.heading(), all_hits);

    // 3. Update algorithm's internal map
    mapping_algorithm_.applyVoxelUpdates(voxels);

    // 4. Update output map (Occupied never overwritten by Empty — handled inside set())
    for (const auto& v : voxels) {
        output_map_.set(v.position, v.value);
    }

    // 5. Get next movement command
    const auto cmd = mapping_algorithm_.nextMove(state(), all_hits);

    // 6. Execute command
    if (cmd.type == types::MovementCommandType::Hover) {
        // Hover means exploration is done
        ++step_index_;
        return {types::DroneStepStatus::Completed, "Exploration complete"};
    }

    types::MovementResult move_result{true};
    switch (cmd.type) {
    case types::MovementCommandType::Rotate:
        move_result = movement_.rotate(cmd.rotation, cmd.angle);
        break;
    case types::MovementCommandType::Advance:
        move_result = movement_.advance(cmd.distance);
        break;
    case types::MovementCommandType::Elevate:
        move_result = movement_.elevate(cmd.distance);
        break;
    default:
        break;
    }

    ++step_index_;

    // A blocked move is a navigation obstacle, not a fatal error.
    // The algorithm will detect the position didn't change and reroute.
    (void)move_result;
    return {types::DroneStepStatus::Continue, {}};
}

types::DroneState DroneControlImpl::state() const {
    return {gps_.position(), gps_.heading(), step_index_};
}

} // namespace drone_mapper
