#pragma once

// ============================================================
// MovementDriverMock.hpp — Simulated movement driver.
//
// Executes Rotate / Advance / Elevate commands against the
// ground-truth map, enforcing:
//   - Mission boundary limits
//   - Collision detection (drone bounding box vs. occupied cells)
//
// Collision check strategy:
//   For each move, we sample several points on the drone's
//   bounding box at the proposed new position and query the
//   ground-truth map.  If any sampled point is occupied (= 1),
//   the move is rejected and false is returned.  The drone
//   body dimensions come from DroneConfig.
// ============================================================

#include "Interfaces.hpp"
#include "PositionSensorMock.hpp"
#include "SparseBuildingMap.hpp"
#include "ConfigParser.hpp"
#include <memory>
#include <cmath>

namespace dm {

class MovementDriverMock : public IMovementDriver {
public:
    MovementDriverMock(std::shared_ptr<SparseBuildingMap> groundTruth,
                       std::shared_ptr<PositionSensorMock> posSensor,
                       const DroneConfig& droneCfg,
                       const MissionConfig& missionCfg);

    // IMovementDriver interface
    bool rotate(Angle angle)    override;
    bool advance(Distance dist) override;
    bool elevate(Distance dist) override;

private:
    // Returns true if the drone's bounding box, centred at (cx,cy,cz),
    // does not intersect any occupied cell and is within mission bounds.
    bool isPositionFree(double cx, double cy, double cz) const;

    std::shared_ptr<SparseBuildingMap> groundTruth_;
    std::shared_ptr<PositionSensorMock> posSensor_;
    DroneConfig    droneCfg_;
    MissionConfig  missionCfg_;
};

} // namespace dm
