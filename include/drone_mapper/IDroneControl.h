#pragma once

#include <drone_mapper/Types.h>

namespace drone_mapper {

class IDroneControl {
public:
    virtual ~IDroneControl() = default;

    [[nodiscard]] virtual types::DroneStepResult step() = 0;
    [[nodiscard]] virtual types::DroneState state() const = 0;
};

} // namespace drone_mapper
