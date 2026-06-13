#pragma once

#include <drone_mapper/IDroneMovement.h>
#include <drone_mapper/IMap3D.h>
#include <drone_mapper/MockGPS.h>

namespace drone_mapper {

class MockMovement final : public IDroneMovement {
public:
    MockMovement(MockGPS& gps,
                 const IMap3D& map,
                 types::DroneConfigData drone);

    types::MovementResult rotate(types::RotationDirection direction, HorizontalAngle angle) override;
    types::MovementResult advance(PhysicalLength distance) override;
    types::MovementResult elevate(PhysicalLength distance) override;

private:
    [[nodiscard]] bool isPositionFree(double cx, double cy, double cz) const;

    MockGPS& gps_;
    const IMap3D& map_;
    types::DroneConfigData drone_;
};

} // namespace drone_mapper
