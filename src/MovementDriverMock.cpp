#include "MovementDriverMock.hpp"
#include <cmath>
#include <algorithm>

namespace dm {

MovementDriverMock::MovementDriverMock(
        std::shared_ptr<SparseBuildingMap> groundTruth,
        std::shared_ptr<PositionSensorMock> posSensor,
        const DroneConfig& droneCfg,
        const MissionConfig& missionCfg)
    : groundTruth_(std::move(groundTruth))
    , posSensor_(std::move(posSensor))
    , droneCfg_(droneCfg)
    , missionCfg_(missionCfg)
{}

// ============================================================
// isPositionFree — check if the drone's bounding box is clear.
//
// Samples a 3D grid of points on the drone body at the proposed
// centre (cx, cy, cz) and queries the ground-truth map for each.
// Also verifies that the bounding box stays within mission bounds.
//
// Sampling uses the mission step size so no occupied voxel inside
// the bounding box is missed (assuming the drone body ≥ one voxel).
// ============================================================
bool MovementDriverMock::isPositionFree(double cx, double cy, double cz) const
{
    const double hw = droneCfg_.width.numerical_value_in(cm)  / 2.0;
    const double hl = droneCfg_.length.numerical_value_in(cm) / 2.0;
    const double hh = droneCfg_.height.numerical_value_in(cm) / 2.0;

    // Boundary check: drone body must lie fully inside the mission area
    if (cx - hw < missionCfg_.minX || cx + hw > missionCfg_.maxX) return false;
    if (cy - hl < missionCfg_.minY || cy + hl > missionCfg_.maxY) return false;
    if (cz - hh < missionCfg_.minHeight || cz + hh > missionCfg_.maxHeight) return false;

    // Collision check: sample every voxel the drone body might overlap
    for (double dx = -hw; dx <= hw + 1e-9; dx += missionCfg_.stepX) {
        for (double dy = -hl; dy <= hl + 1e-9; dy += missionCfg_.stepY) {
            for (double dz = -hh; dz <= hh + 1e-9; dz += missionCfg_.stepZ) {
                const GridPoint gp = toGrid(cx + dx, cy + dy, cz + dz, missionCfg_);
                if (groundTruth_->getCell(gp) == static_cast<int>(CellStatus::Occupied)) {
                    return false;  // collision detected
                }
            }
        }
    }
    return true;
}

// ============================================================
// rotate — update the drone's heading. No collision needed
// (rotation happens in place). Angle is normalised to [0, 360).
// ============================================================
bool MovementDriverMock::rotate(Angle angle)
{
    const Angle cur = posSensor_->getCurrentAngle();
    const double newVal =
        cur.numerical_value_in(deg) + angle.numerical_value_in(deg);
    // setAngle in PositionSensorMock normalises to [0, 360) automatically
    posSensor_->setAngle(newVal * deg);
    return true;
}

// ============================================================
// advance — move forward in the XY plane along the current heading.
// The move is rejected (false) if it would cause a collision.
// ============================================================
bool MovementDriverMock::advance(Distance dist)
{
    const Position3D pos      = posSensor_->getCurrentPosition();
    const double     angleRad =
        posSensor_->getCurrentAngle().numerical_value_in(deg) * M_PI / 180.0;
    const double d = dist.numerical_value_in(cm);

    // 0° = East (+X), 90° = South (+Y) convention
    const double nx = pos.x.numerical_value_in(cm) + d * std::cos(angleRad);
    const double ny = pos.y.numerical_value_in(cm) + d * std::sin(angleRad);
    const double nz = pos.z.numerical_value_in(cm);

    if (!isPositionFree(nx, ny, nz)) return false;

    posSensor_->setPosition(Position3D{nx * cm, ny * cm, nz * cm});
    return true;
}

// ============================================================
// elevate — move up or down along the Z axis.
// Positive distance = upward. Rejected on collision.
// ============================================================
bool MovementDriverMock::elevate(Distance dist)
{
    const Position3D pos = posSensor_->getCurrentPosition();
    const double     nx  = pos.x.numerical_value_in(cm);
    const double     ny  = pos.y.numerical_value_in(cm);
    const double     nz  = pos.z.numerical_value_in(cm)
                         + dist.numerical_value_in(cm);

    if (!isPositionFree(nx, ny, nz)) return false;

    posSensor_->setPosition(Position3D{nx * cm, ny * cm, nz * cm});
    return true;
}

} // namespace dm
