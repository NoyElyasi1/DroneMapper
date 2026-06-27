#include <drone_mapper/ScanResultToVoxels.h>

#include <mp-units/systems/si/math.h>

#include <cmath>
#include <limits>

namespace drone_mapper {

void ScanResultToVoxels::applyToMap(
    IMutableMap3D& output_map,
    const Position3D& scan_origin,
    const Orientation& drone_heading,
    const types::LidarScanResult& scan,
    const types::LidarConfigData& lidar_config)
{
    constexpr double STEP_CM = 5.0;
    const double z_max_cm = lidar_config.z_max.numerical_value_in(cm);
    const double MISS_THRESHOLD = z_max_cm * 1.5;

    for (const auto& hit : scan) {
        const double dist_cm = hit.distance.force_numerical_value_in(cm);
        const bool is_real_hit = dist_cm < MISS_THRESHOLD;

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

        if (!is_real_hit) {
            // No real hit — mark traversed zone as PotentiallyOccupied at z_max
            const double t = z_max_cm;
            output_map.set(
                Position3D{
                    (ox + dx * t) * x_extent[cm],
                    (oy + dy * t) * y_extent[cm],
                    (oz + dz * t) * z_extent[cm],
                },
                types::VoxelOccupancy::PotentiallyOccupied);
            continue;
        }

        if (dist_cm <= 0.0) {
            output_map.set(
                Position3D{
                    (ox + dx * STEP_CM) * x_extent[cm],
                    (oy + dy * STEP_CM) * y_extent[cm],
                    (oz + dz * STEP_CM) * z_extent[cm],
                },
                types::VoxelOccupancy::Occupied);
            continue;
        }

        // Mark traversed voxels as Empty
        for (double t = STEP_CM; t < dist_cm - STEP_CM * 0.5; t += STEP_CM) {
            output_map.set(
                Position3D{
                    (ox + dx * t) * x_extent[cm],
                    (oy + dy * t) * y_extent[cm],
                    (oz + dz * t) * z_extent[cm],
                },
                types::VoxelOccupancy::Empty);
        }

        // Mark hit voxel as Occupied
        output_map.set(
            Position3D{
                (ox + dx * dist_cm) * x_extent[cm],
                (oy + dy * dist_cm) * y_extent[cm],
                (oz + dz * dist_cm) * z_extent[cm],
            },
            types::VoxelOccupancy::Occupied);
    }
}

} // namespace drone_mapper
