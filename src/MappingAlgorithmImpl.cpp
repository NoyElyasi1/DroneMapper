#include <drone_mapper/MappingAlgorithmImpl.h>

#include <cmath>
#include <utility>

namespace drone_mapper {

using detail::GridPoint;

namespace {

double normalizeAngle(double a) noexcept {
    while (a >  180.0) a -= 360.0;
    while (a < -180.0) a += 360.0;
    return a;
}

} // namespace

MappingAlgorithmImpl::MappingAlgorithmImpl(types::MissionConfigData mission,
                                           types::DroneConfigData drone,
                                           types::MapConfig map_config)
    : mission_(std::move(mission))
    , drone_(std::move(drone))
    , map_config_(std::move(map_config))
    , stepCm_(mission_.gps_resolution.numerical_value_in(cm))
{}

detail::GridPoint MappingAlgorithmImpl::toGrid(const Position3D& pos) const {
    return {
        static_cast<int>(std::round(pos.x.numerical_value_in(cm) / stepCm_)),
        static_cast<int>(std::round(pos.y.numerical_value_in(cm) / stepCm_)),
        static_cast<int>(std::round(pos.z.numerical_value_in(cm) / stepCm_)),
    };
}

bool MappingAlgorithmImpl::isInBounds(const GridPoint& gp) const {
    const double cx = gp.x * stepCm_;
    const double cy = gp.y * stepCm_;
    const double cz = gp.z * stepCm_;
    return cx >= map_config_.boundaries.min_x.numerical_value_in(cm)
        && cx <= map_config_.boundaries.max_x.numerical_value_in(cm)
        && cy >= map_config_.boundaries.min_y.numerical_value_in(cm)
        && cy <= map_config_.boundaries.max_y.numerical_value_in(cm)
        && cz >= map_config_.boundaries.min_height.numerical_value_in(cm)
        && cz <= map_config_.boundaries.max_height.numerical_value_in(cm);
}

bool MappingAlgorithmImpl::isOccupied(const GridPoint& gp) const {
    auto it = internalMap_.find(gp);
    return it != internalMap_.end() && it->second == types::VoxelOccupancy::Occupied;
}

bool MappingAlgorithmImpl::findFrontier(const GridPoint& cur, GridPoint& out) const {
    const double maxAdvCm  = drone_.max_advance.numerical_value_in(cm);
    const double maxElevCm = drone_.max_elevate.numerical_value_in(cm);
    const int navXY = std::max(1, static_cast<int>(std::round(maxAdvCm  / stepCm_)));
    const int navZ  = std::max(1, static_cast<int>(std::round(maxElevCm / stepCm_)));

    const int DIRS[6][3] = {
        { navXY,  0,    0},
        {-navXY,  0,    0},
        { 0,  navXY,   0},
        { 0, -navXY,   0},
        { 0,  0,  navZ},
        { 0,  0, -navZ},
    };

    for (const auto& d : DIRS) {
        GridPoint cand{cur.x + d[0], cur.y + d[1], cur.z + d[2]};
        if (!isInBounds(cand)) continue;
        if (visited_.count(cand)) continue;
        if (isOccupied(cand)) continue;
        out = cand;
        return true;
    }
    return false;
}

void MappingAlgorithmImpl::applyVoxelUpdates(const std::vector<types::MappedVoxel>& voxels) {
    for (const auto& v : voxels) {
        GridPoint gp = toGrid(v.position);
        auto it = internalMap_.find(gp);
        if (v.value == types::VoxelOccupancy::Occupied) {
            internalMap_[gp] = types::VoxelOccupancy::Occupied;
        } else if (v.value == types::VoxelOccupancy::Empty) {
            if (it == internalMap_.end() || it->second != types::VoxelOccupancy::Occupied) {
                internalMap_[gp] = types::VoxelOccupancy::Empty;
            }
        }
    }
}

types::MovementCommand MappingAlgorithmImpl::nextMove(const types::DroneState& state,
                                                      const types::LidarScanResult& /*latest_scan*/)
{
    for (int safety = 0; safety < 20; ++safety) {
        const GridPoint curGrid = toGrid(state.position);

        switch (phase_) {

        case NavPhase::Done:
            return types::MovementCommand{types::MovementCommandType::Hover};

        case NavPhase::FindNext: {
            if (!visited_.count(curGrid)) {
                visited_.insert(curGrid);
            }
            GridPoint frontier{};
            if (findFrontier(curGrid, frontier)) {
                prevGrid_ = curGrid;
                currentTarget_ = frontier;
                navigatingBack_ = false;
                phase_ = NavPhase::Elevating;
            } else if (!backtrackStack_.empty()) {
                currentTarget_ = backtrackStack_.top();
                backtrackStack_.pop();
                navigatingBack_ = true;
                phase_ = NavPhase::Elevating;
            } else {
                phase_ = NavPhase::Done;
                return types::MovementCommand{types::MovementCommandType::Hover};
            }
            break;
        }

        case NavPhase::Elevating: {
            const double targetZ = currentTarget_.z * stepCm_;
            const double curZ    = state.position.z.numerical_value_in(cm);
            const double dz      = targetZ - curZ;
            if (std::fabs(dz) > 0.5) {
                const double maxEl = drone_.max_elevate.numerical_value_in(cm);
                const double move  = std::clamp(dz, -maxEl, maxEl);
                return types::MovementCommand{
                    types::MovementCommandType::Elevate,
                    types::RotationDirection::Left,
                    {},
                    move * cm,
                };
            }
            phase_ = NavPhase::Rotating;
            break;
        }

        case NavPhase::Rotating: {
            const double targetX = currentTarget_.x * stepCm_;
            const double targetY = currentTarget_.y * stepCm_;
            const double curX    = state.position.x.numerical_value_in(cm);
            const double curY    = state.position.y.numerical_value_in(cm);
            const double xyDist  = std::hypot(targetX - curX, targetY - curY);
            if (xyDist < 0.5) {
                if (!navigatingBack_) backtrackStack_.push(prevGrid_);
                else navigatingBack_ = false;
                phase_ = NavPhase::FindNext;
                break;
            }
            double targetAngle = std::atan2(targetY - curY, targetX - curX) * 180.0 / M_PI;
            if (targetAngle < 0.0) targetAngle += 360.0;
            const double curAngle = state.heading.horizontal.numerical_value_in(deg);
            const double diff     = normalizeAngle(targetAngle - curAngle);
            const double maxRot   = drone_.max_rotate.numerical_value_in(deg);
            if (std::fabs(diff) > 0.5) {
                const double rotAmt = std::clamp(std::fabs(diff), 0.0, maxRot);
                const auto dir = (diff > 0)
                    ? types::RotationDirection::Left
                    : types::RotationDirection::Right;
                return types::MovementCommand{
                    types::MovementCommandType::Rotate,
                    dir,
                    rotAmt * horizontal_angle[deg],
                    {},
                };
            }
            phase_ = NavPhase::Advancing;
            break;
        }

        case NavPhase::Advancing: {
            const double targetX = currentTarget_.x * stepCm_;
            const double targetY = currentTarget_.y * stepCm_;
            const double curX    = state.position.x.numerical_value_in(cm);
            const double curY    = state.position.y.numerical_value_in(cm);
            const double xyDist  = std::hypot(targetX - curX, targetY - curY);

            // Stuck detection
            if (curGrid == lastKnownGrid_) {
                ++stuckCount_;
            } else {
                stuckCount_     = 0;
                lastKnownGrid_  = curGrid;
            }
            if (stuckCount_ >= 3) {
                internalMap_[currentTarget_] = types::VoxelOccupancy::Occupied;
                visited_.insert(currentTarget_);
                stuckCount_ = 0;
                phase_ = NavPhase::FindNext;
                break;
            }

            if (xyDist < 0.5) {
                if (!navigatingBack_) backtrackStack_.push(prevGrid_);
                else navigatingBack_ = false;
                phase_ = NavPhase::FindNext;
                break;
            }
            const double maxAdv = drone_.max_advance.numerical_value_in(cm);
            const double move   = std::min(xyDist, maxAdv);
            return types::MovementCommand{
                types::MovementCommandType::Advance,
                types::RotationDirection::Left,
                {},
                move * cm,
            };
        }

        } // switch
    } // safety loop

    phase_ = NavPhase::Done;
    return types::MovementCommand{types::MovementCommandType::Hover};
}

} // namespace drone_mapper
