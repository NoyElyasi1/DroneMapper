#pragma once

#include <drone_mapper/IDroneControl.h>
#include <drone_mapper/IMissionControl.h>

namespace drone_mapper {

class MissionControlImpl final : public IMissionControl {
public:
    MissionControlImpl(types::MissionConfigData mission,
                       IDroneControl& drone_control);

    [[nodiscard]] types::MissionRunResult runMission() override;

private:
    types::MissionConfigData mission_;
    IDroneControl& drone_control_;
};

} // namespace drone_mapper
