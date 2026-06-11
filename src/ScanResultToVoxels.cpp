#include <drone_mapper/ScanResultToVoxels.h>

#include <mp-units/systems/si/math.h>

#include <cmath>
#include <limits>

namespace drone_mapper {

std::vector<types::MappedVoxel> ScanResultToVoxels::convert(
    const Position3D& scan_origin,
    const Orientation& drone_heading,
    const types::LidarScanResult& scan)
{
    std::vector<types::MappedVoxel> result;
    constexpr double STEP_CM = 5.0;
    constexpr double MISS_THRESHOLD = std::numeric_limits<double>::max() / 2.0;

    for (const auto& hit : scan) {
        const double dist_cm = hit.distance.force_numerical_value_in(cm);
        const bool is_real_hit = dist_cm < MISS_THRESHOLD;

        if (!is_real_hit) {
            // No hit — no range info; skip
            continue;
        }

        // Absolute beam direction = relative angle + drone heading
        const HorizontalAngle abs_h = hit.angle.horizontal + drone_heading.horizontal;
        const AltitudeAngle   abs_a = hit.angle.altitude   + drone_heading.altitude;

        const double cos_alt = si::cos(abs_a).force_numerical_value_in(mp::one);
        const double dx      = cos_alt * si::cos(abs_h).force_numerical_value_in(mp::one);
        const double dy      = cos_alt * si::sin(abs_h).force_numerical_value_in(mp::one);
        const double dz      = si::sin(abs_a).force_numerical_value_in(mp::one);

        const double ox = scan_origin.x.force_numerical_value_in(cm);
        const double oy = scan_origin.y.force_numerical_value_in(cm);
        const double oz = scan_origin.z.force_numerical_value_in(cm);

        if (dist_cm <= 0.0) {
            // Hit too close — mark immediate voxel as Occupied
            result.push_back({
                Position3D{
                    (ox + dx * STEP_CM) * x_extent[cm],
                    (oy + dy * STEP_CM) * y_extent[cm],
                    (oz + dz * STEP_CM) * z_extent[cm],
                },
                types::VoxelOccupancy::Occupied,
            });
            continue;
        }

        // Mark traversed voxels as Empty
        for (double t = STEP_CM; t < dist_cm - STEP_CM * 0.5; t += STEP_CM) {
            result.push_back({
                Position3D{
                    (ox + dx * t) * x_extent[cm],
                    (oy + dy * t) * y_extent[cm],
                    (oz + dz * t) * z_extent[cm],
                },
                types::VoxelOccupancy::Empty,
            });
        }

        // Mark hit voxel as Occupied
        result.push_back({
            Position3D{
                (ox + dx * dist_cm) * x_extent[cm],
                (oy + dy * dist_cm) * y_extent[cm],
                (oz + dz * dist_cm) * z_extent[cm],
            },
            types::VoxelOccupancy::Occupied,
        });
    }

    return result;
}

} // namespace drone_mapper
