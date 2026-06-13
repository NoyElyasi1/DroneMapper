#include <drone_mapper/SimulationRunImpl.h>

#include <drone_mapper/MapsComparison.h>

#include <stdexcept>
#include <utility>

namespace drone_mapper {

SimulationRunImpl::SimulationRunImpl(types::SimulationConfigData simulation_config,
                                     types::MissionConfigData mission_config,
                                     types::DroneConfigData drone_config,
                                     types::LidarConfigData lidar_config,
                                     types::ResolutionRequestStatus resolution_status,
                                     std::unique_ptr<const IMap3D> hidden_map,
                                     std::unique_ptr<IMutableMap3D> output_map,
                                     std::unique_ptr<IGPS> gps,
                                     std::unique_ptr<IDroneMovement> movement,
                                     std::unique_ptr<ILidar> lidar,
                                     std::unique_ptr<IMappingAlgorithm> mapping_algorithm,
                                     std::unique_ptr<IDroneControl> drone_control,
                                     std::unique_ptr<IMissionControl> mission_control,
                                     std::filesystem::path output_map_file)
    : simulation_config_(std::move(simulation_config))
    , mission_config_(std::move(mission_config))
    , drone_config_(std::move(drone_config))
    , lidar_config_(std::move(lidar_config))
    , resolution_status_(resolution_status)
    , hidden_map_(std::move(hidden_map))
    , output_map_(std::move(output_map))
    , gps_(std::move(gps))
    , movement_(std::move(movement))
    , lidar_(std::move(lidar))
    , mapping_algorithm_(std::move(mapping_algorithm))
    , drone_control_(std::move(drone_control))
    , mission_control_(std::move(mission_control))
    , output_map_file_(std::move(output_map_file))
{
    if (!hidden_map_ || !output_map_ || !gps_ || !movement_ ||
        !lidar_ || !mapping_algorithm_ || !drone_control_ || !mission_control_) {
        throw std::invalid_argument("SimulationRunImpl: all dependencies required");
    }
}

types::SimulationResult SimulationRunImpl::run() {
    // Run the mission
    const auto mission_result = mission_control_->runMission();

    // Save the output map
    output_map_->save(output_map_file_);

    // Score: compare ground truth vs output map
    const auto scores = MapsComparison::compare(*hidden_map_, {output_map_.get()});
    const double score = scores.empty() ? -1.0 : scores[0];

    types::SimulationResult result;
    result.simulation_config    = simulation_config_;
    result.mission_config       = mission_config_;
    result.drone_config         = drone_config_;
    result.lidar_config         = lidar_config_;
    result.resolution_request_status = resolution_status_;
    result.mission_results      = {mission_result};
    result.output_map_file      = output_map_file_;
    result.output_map_config    = output_map_->getMapConfig();
    result.mission_score        = (mission_result.status == types::MissionRunStatus::Error)
                                      ? -1.0
                                      : score;
    return result;
}

} // namespace drone_mapper
