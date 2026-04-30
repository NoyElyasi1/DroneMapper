#pragma once

// ============================================================
// Interfaces.hpp — Pure abstract interfaces for Dependency Injection.
//
// The Drone depends ONLY on these three interfaces. The mock
// implementations (PositionSensorMock, MovementDriverMock,
// LidarMock) are injected at start-up by the Simulator, so
// the Drone is completely unaware it is running in simulation.
//
// This matches the classical Dependency Inversion Principle:
// high-level policy (Drone algorithm) depends on abstractions,
// not on concrete simulation details.
// ============================================================

#include "Types.hpp"
#include <vector>

namespace dm {

// ------------------------------------------------------------
// IPositionSensor
// Provides the drone's exact current position and XY heading.
// ------------------------------------------------------------
class IPositionSensor {
public:
    virtual ~IPositionSensor() = default;

    // Returns the drone centre in world coordinates (cm).
    virtual Position3D getCurrentPosition() = 0;

    // Returns the drone's XY heading in degrees.
    // Convention: 0 = East (+X), 90 = South (+Y),
    //             180 = West (-X), 270 = North (-Y).
    virtual Angle getCurrentAngle() = 0;
};

// ------------------------------------------------------------
// IMovementDriver
// Executes movement commands. All commands return false if the
// move would cause a collision or leave mission boundaries.
// ------------------------------------------------------------
class IMovementDriver {
public:
    virtual ~IMovementDriver() = default;

    // Rotate in the XY plane. Positive = clockwise (South).
    virtual bool rotate(Angle angle) = 0;

    // Advance in the current heading direction (XY plane).
    virtual bool advance(Distance distance) = 0;

    // Elevate along the Z axis. Positive = upward.
    virtual bool elevate(Distance distance) = 0;
};

// ------------------------------------------------------------
// ILidarSensor
// Fires all beams for a single scan and returns results.
//
// Parameters (per assignment spec "Scan <X-Y angle, height angle>"):
//   xyOffset       — horizontal offset from current drone heading
//                    (degrees, clockwise positive).  Default = 0.
//   elevationAngle — vertical tilt from horizontal
//                    (degrees, positive = upward). Default = 0.
//
// The Lidar's FOV cone is centred on the direction defined by
// (heading + xyOffset, elevationAngle).  All beams in the cone
// are emitted from that same origin.
// ------------------------------------------------------------
class ILidarSensor {
public:
    virtual ~ILidarSensor() = default;

    virtual std::vector<ScanResult> scan(
        Angle xyOffset       = 0.0 * deg,
        Angle elevationAngle = 0.0 * deg) = 0;
};

} // namespace dm
