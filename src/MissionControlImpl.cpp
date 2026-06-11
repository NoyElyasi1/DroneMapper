#include <drone_mapper/MissionControlImpl.h>

#include <utility>

namespace drone_mapper {

MissionControlImpl::MissionControlImpl(types::MissionConfigData mission,
                                       IDroneControl& drone_control)
    : mission_(std::move(mission))
    , drone_control_(drone_control)
{}

types::MissionRunResult MissionControlImpl::runMission() {
    std::size_t steps = 0;
    types::MissionRunStatus status = types::MissionRunStatus::MaxSteps;
    std::vector<types::ErrorRef> errors;

    for (std::size_t i = 0; i < mission_.max_steps; ++i) {
        const auto result = drone_control_.step();
        ++steps;
        if (result.status == types::DroneStepStatus::Completed) {
            status = types::MissionRunStatus::Completed;
            break;
        }
        if (result.status == types::DroneStepStatus::Error) {
            status = types::MissionRunStatus::Error;
            errors.push_back({"DRONE_ERROR", result.message});
            break;
        }
    }

    return {status, steps, errors};
}

} // namespace drone_mapper
