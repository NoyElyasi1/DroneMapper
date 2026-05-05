#pragma once

// ============================================================
// PositionSensorMock.hpp — Simulated position sensor.
//
// Tracks the drone's exact position and heading.
// Updated by MovementDriverMock after every successful move
// so the two mocks share a single source of truth.
//
// The Drone sees only the IPositionSensor interface and is
// unaware that position updates come from the movement driver.
// ============================================================

#include "Interfaces.hpp"
#include "Types.hpp"
#include <cmath>

namespace dm {

class PositionSensorMock : public IPositionSensor {
public:
    PositionSensorMock() = default;

    // IPositionSensor interface
    Position3D getCurrentPosition() override { return pos_; }
    Angle      getCurrentAngle()    override { return angle_; }

    // Called by MovementDriverMock to update state after each move
    void setPosition(const Position3D& p) { pos_ = p; }

    // Set heading, normalised to [0, 360) degrees
    void setAngle(Angle a) {
        double val = std::fmod(a.numerical_value_in(deg), 360.0);
        if (val < 0.0) val += 360.0;
        angle_ = val * deg;
    }

private:
    Position3D pos_;
    Angle      angle_ = 0.0 * deg;  // 0 = East (+X)
};

} // namespace dm
