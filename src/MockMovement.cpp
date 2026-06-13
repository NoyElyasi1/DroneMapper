#include <drone_mapper/MockMovement.h>

#include <cmath>
#include <utility>

namespace drone_mapper {

MockMovement::MockMovement(MockGPS& gps,
                           const IMap3D& map,
                           types::DroneConfigData drone)
    : gps_(gps)
    , map_(map)
    , drone_(std::move(drone))
{}

bool MockMovement::isPositionFree(double cx, double cy, double cz) const {
    const auto& cfg = map_.getMapConfig();
    const double r   = drone_.radius.numerical_value_in(cm);
    const double res = cfg.resolution.numerical_value_in(cm);

    // Boundary check using map config
    const double minX = cfg.boundaries.min_x.numerical_value_in(cm);
    const double maxX = cfg.boundaries.max_x.numerical_value_in(cm);
    const double minY = cfg.boundaries.min_y.numerical_value_in(cm);
    const double maxY = cfg.boundaries.max_y.numerical_value_in(cm);
    const double minZ = cfg.boundaries.min_height.numerical_value_in(cm);
    const double maxZ = cfg.boundaries.max_height.numerical_value_in(cm);

    if (cx - r < minX || cx + r > maxX) return false;
    if (cy - r < minY || cy + r > maxY) return false;
    if (cz - r < minZ || cz + r > maxZ) return false;

    // Sphere collision sampling
    for (double dx = -r; dx <= r + 1e-9; dx += res) {
        for (double dy = -r; dy <= r + 1e-9; dy += res) {
            for (double dz = -r; dz <= r + 1e-9; dz += res) {
                if (dx*dx + dy*dy + dz*dz > r*r + 1e-6) continue;
                const Position3D sample{
                    (cx + dx) * x_extent[cm],
                    (cy + dy) * y_extent[cm],
                    (cz + dz) * z_extent[cm],
                };
                if (map_.atVoxel(sample) == types::VoxelOccupancy::Occupied) return false;
            }
        }
    }
    return true;
}

types::MovementResult MockMovement::rotate(types::RotationDirection direction, HorizontalAngle angle) {
    const double max_rot = drone_.max_rotate.numerical_value_in(deg);
    const double ang     = angle.numerical_value_in(deg);
    if (ang > max_rot + 1e-9) {
        return {false, "Rotation exceeds max_rotate"};
    }
    const Orientation cur = gps_.heading();
    const double sign = (direction == types::RotationDirection::Left) ? 1.0 : -1.0;
    double newH = cur.horizontal.numerical_value_in(deg) + sign * ang;
    while (newH >= 360.0) newH -= 360.0;
    while (newH <    0.0) newH += 360.0;
    gps_.setHeading(Orientation{newH * horizontal_angle[deg], cur.altitude});
    return {true, {}};
}

types::MovementResult MockMovement::advance(PhysicalLength distance) {
    const double max_adv = drone_.max_advance.numerical_value_in(cm);
    const double d       = distance.numerical_value_in(cm);
    if (std::fabs(d) > max_adv + 1e-9) {
        return {false, "Advance exceeds max_advance"};
    }
    const Position3D pos      = gps_.position();
    const Orientation hdg     = gps_.heading();
    const double angle_rad    = hdg.horizontal.numerical_value_in(deg) * M_PI / 180.0;
    const double nx           = pos.x.numerical_value_in(cm) + d * std::cos(angle_rad);
    const double ny           = pos.y.numerical_value_in(cm) + d * std::sin(angle_rad);
    const double nz           = pos.z.numerical_value_in(cm);
    if (!isPositionFree(nx, ny, nz)) {
        return {false, "Advance blocked: collision or boundary"};
    }
    gps_.setPosition(Position3D{nx * x_extent[cm], ny * y_extent[cm], nz * z_extent[cm]});
    return {true, {}};
}

types::MovementResult MockMovement::elevate(PhysicalLength distance) {
    const double max_el = drone_.max_elevate.numerical_value_in(cm);
    const double d      = distance.numerical_value_in(cm);
    if (std::fabs(d) > max_el + 1e-9) {
        return {false, "Elevate exceeds max_elevate"};
    }
    const Position3D pos = gps_.position();
    const double nx      = pos.x.numerical_value_in(cm);
    const double ny      = pos.y.numerical_value_in(cm);
    const double nz      = pos.z.numerical_value_in(cm) + d;
    if (!isPositionFree(nx, ny, nz)) {
        return {false, "Elevate blocked: collision or boundary"};
    }
    gps_.setPosition(Position3D{nx * x_extent[cm], ny * y_extent[cm], nz * z_extent[cm]});
    return {true, {}};
}

} // namespace drone_mapper
