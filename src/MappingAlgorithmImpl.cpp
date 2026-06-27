#include <drone_mapper/MappingAlgorithmImpl.h>
#include <drone_mapper/IMap3D.h>

#include <cmath>

namespace drone_mapper {

using detail::GridPoint;

namespace {

double normalizeAngle(double a) noexcept {
    while (a >  180.0) a -= 360.0;
    while (a < -180.0) a += 360.0;
    return a;
}

} // namespace

detail::GridPoint MappingAlgorithmImpl::toGrid(const Position3D& pos) const {
    const double stepCm = output_map_.getMapConfig().resolution.numerical_value_in(cm);
    return {
        static_cast<int>(std::round(pos.x.numerical_value_in(cm) / stepCm)),
        static_cast<int>(std::round(pos.y.numerical_value_in(cm) / stepCm)),
        static_cast<int>(std::round(pos.z.numerical_value_in(cm) / stepCm)),
    };
}

bool MappingAlgorithmImpl::isInBounds(const GridPoint& gp) const {
    const double stepCm = output_map_.getMapConfig().resolution.numerical_value_in(cm);
    const auto& b = output_map_.getMapConfig().boundaries;
    const double cx = gp.x * stepCm;
    const double cy = gp.y * stepCm;
    const double cz = gp.z * stepCm;
    return cx >= b.min_x.numerical_value_in(cm)
        && cx <= b.max_x.numerical_value_in(cm)
        && cy >= b.min_y.numerical_value_in(cm)
        && cy <= b.max_y.numerical_value_in(cm)
        && cz >= b.min_height.numerical_value_in(cm)
        && cz <= b.max_height.numerical_value_in(cm);
}

bool MappingAlgorithmImpl::isOccupied(const GridPoint& gp) const {
    const double stepCm = output_map_.getMapConfig().resolution.numerical_value_in(cm);
    const Position3D pos{
        gp.x * stepCm * x_extent[cm],
        gp.y * stepCm * y_extent[cm],
        gp.z * stepCm * z_extent[cm],
    };
    return output_map_.atVoxel(pos) == types::VoxelOccupancy::Occupied;
}

bool MappingAlgorithmImpl::findFrontier(const GridPoint& cur, GridPoint& out) const {
    const double stepCm    = output_map_.getMapConfig().resolution.numerical_value_in(cm);
    const double maxAdvCm  = drone_config_.max_advance.numerical_value_in(cm);
    const double maxElevCm = drone_config_.max_elevate.numerical_value_in(cm);
    const int navXY = std::max(1, static_cast<int>(std::round(maxAdvCm  / stepCm)));
    const int navZ  = std::max(1, static_cast<int>(std::round(maxElevCm / stepCm)));

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

types::MappingStepCommand MappingAlgorithmImpl::nextStep(const types::DroneState& state,
                                                         const types::LidarScanResult* /*latest_scan*/)
{
    // Always request a forward scan so DroneControlImpl updates the output_map.
    const Orientation scan_dir{state.heading.horizontal, 0.0 * altitude_angle[deg]};

    const double stepCm = output_map_.getMapConfig().resolution.numerical_value_in(cm);

    for (int safety = 0; safety < 20; ++safety) {
        const GridPoint curGrid = toGrid(state.position);

        switch (phase_) {

        case NavPhase::Done:
            return {std::nullopt, std::nullopt, types::AlgorithmStatus::Finished};

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
                return {std::nullopt, scan_dir, types::AlgorithmStatus::Finished};
            }
            break;
        }

        case NavPhase::Elevating: {
            const double targetZ = currentTarget_.z * stepCm;
            const double curZ    = state.position.z.numerical_value_in(cm);
            const double dz      = targetZ - curZ;
            if (std::fabs(dz) > 0.5) {
                const double maxEl = drone_config_.max_elevate.numerical_value_in(cm);
                const double move  = std::clamp(dz, -maxEl, maxEl);
                types::MovementCommand cmd{
                    types::MovementCommandType::Elevate,
                    types::RotationDirection::Left,
                    {},
                    move * cm,
                };
                return {cmd, scan_dir, types::AlgorithmStatus::Working};
            }
            phase_ = NavPhase::Rotating;
            break;
        }

        case NavPhase::Rotating: {
            const double targetX = currentTarget_.x * stepCm;
            const double targetY = currentTarget_.y * stepCm;
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
            const double maxRot   = drone_config_.max_rotate.numerical_value_in(deg);
            if (std::fabs(diff) > 0.5) {
                const double rotAmt = std::clamp(std::fabs(diff), 0.0, maxRot);
                const auto dir = (diff > 0)
                    ? types::RotationDirection::Left
                    : types::RotationDirection::Right;
                types::MovementCommand cmd{
                    types::MovementCommandType::Rotate,
                    dir,
                    rotAmt * horizontal_angle[deg],
                    {},
                };
                return {cmd, scan_dir, types::AlgorithmStatus::Working};
            }
            phase_ = NavPhase::Advancing;
            break;
        }

        case NavPhase::Advancing: {
            const double targetX = currentTarget_.x * stepCm;
            const double targetY = currentTarget_.y * stepCm;
            const double curX    = state.position.x.numerical_value_in(cm);
            const double curY    = state.position.y.numerical_value_in(cm);
            const double xyDist  = std::hypot(targetX - curX, targetY - curY);

            if (curGrid == lastKnownGrid_) {
                ++stuckCount_;
            } else {
                stuckCount_    = 0;
                lastKnownGrid_ = curGrid;
            }
            if (stuckCount_ >= 3) {
                // Mark target as blocked and re-plan
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
            const double maxAdv = drone_config_.max_advance.numerical_value_in(cm);
            const double move   = std::min(xyDist, maxAdv);
            types::MovementCommand cmd{
                types::MovementCommandType::Advance,
                types::RotationDirection::Left,
                {},
                move * cm,
            };
            return {cmd, scan_dir, types::AlgorithmStatus::Working};
        }

        } // switch
    } // safety loop

    phase_ = NavPhase::Done;
    return {std::nullopt, std::nullopt, types::AlgorithmStatus::Finished};
}

} // namespace drone_mapper
