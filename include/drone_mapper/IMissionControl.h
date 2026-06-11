#pragma once

#include <drone_mapper/Types.h>

namespace drone_mapper {

class IMissionControl {
public:
    virtual ~IMissionControl() = default;

    [[nodiscard]] virtual types::MissionRunResult runMission() = 0;
};

} // namespace drone_mapper
