#include <drone_mapper/SimulationManager.h>

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace drone_mapper {

SimulationManager::SimulationManager(std::unique_ptr<ISimulationRunFactory> run_factory)
    : run_factory_(std::move(run_factory))
{
    if (!run_factory_) {
        throw std::invalid_argument("SimulationManager requires a run factory.");
    }
}

types::SimulationManagerReport SimulationManager::run(
    const types::SimulationCompositionData& composition,
    const std::filesystem::path& output_path)
{
    const auto now = std::chrono::system_clock::now();
    const std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::ostringstream ts;
    ts << std::put_time(std::gmtime(&t), "%Y-%m-%dT%H:%M:%SZ");

    types::SimulationManagerReport report;
    report.composition_file  = composition.composition_file;
    report.generated_at_utc  = ts.str();
    report.metric            = "output_map_accuracy";
    report.score_range       = {0.0, 100.0};
    report.error_score       = -1;

    for (const auto& simulation : composition.simulations) {
        for (const auto& mission : composition.missions) {
            for (const auto& drone : composition.drones) {
                for (const auto& lidar_cfg : composition.lidars) {
                    try {
                        auto run_obj = run_factory_->create(
                            simulation, mission, drone, lidar_cfg, output_path);
                        report.runs.push_back(run_obj->run());
                    } catch (const std::exception& ex) {
                        types::SimulationResult failed;
                        failed.simulation_config = simulation;
                        failed.mission_config    = mission;
                        failed.drone_config      = drone;
                        failed.lidar_config      = lidar_cfg;
                        failed.mission_score     = -1.0;
                        failed.mission_results.push_back({
                            types::MissionRunStatus::Error,
                            0,
                            {types::ErrorRef{"SIM_FACTORY_ERROR", ex.what()}}
                        });
                        report.runs.push_back(std::move(failed));
                    }
                }
            }
        }
    }

    return report;
}

} // namespace drone_mapper
