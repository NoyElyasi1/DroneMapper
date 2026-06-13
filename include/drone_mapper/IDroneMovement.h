#pragma once

#include <drone_mapper/Types.h>

namespace drone_mapper {

class IDroneMovement {
public:
    virtual ~IDroneMovement() = default;

    virtual types::MovementResult rotate(types::RotationDirection direction, HorizontalAngle angle) = 0;
    virtual types::MovementResult advance(PhysicalLength distance) = 0;
    virtual types::MovementResult elevate(PhysicalLength distance) = 0;
};

} // namespace drone_mapper
