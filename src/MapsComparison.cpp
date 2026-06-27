#include <drone_mapper/MapsComparison.h>

#include <drone_mapper/Map3DImpl.h>

namespace drone_mapper {

namespace {
constexpr double WEIGHT_OCCUPIED = 50.0;
constexpr double WEIGHT_EMPTY    = 30.0;
constexpr double WEIGHT_COVERAGE = 20.0;

double scoreOneTarget(const IMap3D& ground_truth, const IMap3D& drone_map) {
    // Same object — trivially identical → 100
    if (&ground_truth == &drone_map) return 100.0;
    const auto* gt = dynamic_cast<const Map3DImpl*>(&ground_truth);
    const auto* dm = dynamic_cast<const Map3DImpl*>(&drone_map);
    if (!gt || !dm) return 0.0;

    const auto gt_cfg  = ground_truth.getMapConfig();
    const double res_gt = gt_cfg.resolution.numerical_value_in(cm);
    const double ox_gt  = gt_cfg.offset.x.numerical_value_in(cm);
    const double oy_gt  = gt_cfg.offset.y.numerical_value_in(cm);
    const double oz_gt  = gt_cfg.offset.z.numerical_value_in(cm);

    int gt_occupied_total      = 0;
    int drone_correct_occupied = 0;
    int drone_false_positives  = 0;

    for (int iz = 0; iz < gt->szZ(); ++iz)
    for (int iy = 0; iy < gt->szY(); ++iy)
    for (int ix = 0; ix < gt->szX(); ++ix) {
        const Position3D world{
            (ox_gt + ix * res_gt) * x_extent[cm],
            (oy_gt + iy * res_gt) * y_extent[cm],
            (oz_gt + iz * res_gt) * z_extent[cm],
        };
        if (ground_truth.atVoxel(world) == types::VoxelOccupancy::Occupied) {
            ++gt_occupied_total;
            if (drone_map.atVoxel(world) == types::VoxelOccupancy::Occupied)
                ++drone_correct_occupied;
        } else {
            if (drone_map.atVoxel(world) == types::VoxelOccupancy::Occupied)
                ++drone_false_positives;
        }
    }

    // Use F1-like score: harmonic mean of recall and precision for occupied voxels
    const int drone_occupied_total = drone_correct_occupied + drone_false_positives;
    const double recall    = (gt_occupied_total == 0) ? 1.0
                             : static_cast<double>(drone_correct_occupied) / gt_occupied_total;
    const double precision = (drone_occupied_total == 0) ? 1.0
                             : static_cast<double>(drone_correct_occupied) / drone_occupied_total;
    const double f1 = (recall + precision > 0.0)
                      ? 2.0 * recall * precision / (recall + precision)
                      : 0.0;
    const double scoreA = WEIGHT_OCCUPIED * f1;

    const auto dm_cfg  = drone_map.getMapConfig();
    const double res_dm = dm_cfg.resolution.numerical_value_in(cm);
    const double ox_dm  = dm_cfg.offset.x.numerical_value_in(cm);
    const double oy_dm  = dm_cfg.offset.y.numerical_value_in(cm);
    const double oz_dm  = dm_cfg.offset.z.numerical_value_in(cm);

    int drone_empty_total   = 0;
    int drone_empty_correct = 0;
    int mapped_cells        = 0;

    for (int iz = 0; iz < dm->szZ(); ++iz)
    for (int iy = 0; iy < dm->szY(); ++iy)
    for (int ix = 0; ix < dm->szX(); ++ix) {
        const Position3D world{
            (ox_dm + ix * res_dm) * x_extent[cm],
            (oy_dm + iy * res_dm) * y_extent[cm],
            (oz_dm + iz * res_dm) * z_extent[cm],
        };
        const auto dm_val = drone_map.atVoxel(world);
        if (dm_val == types::VoxelOccupancy::Empty) {
            ++drone_empty_total;
            if (ground_truth.atVoxel(world) != types::VoxelOccupancy::Occupied)
                ++drone_empty_correct;
        }
        if (dm_val != types::VoxelOccupancy::Unmapped && dm_val != types::VoxelOccupancy::OutOfBounds)
            ++mapped_cells;
    }

    const double scoreB = (drone_empty_total == 0)
        ? 0.0
        : (WEIGHT_EMPTY * drone_empty_correct / drone_empty_total);

    const long long total_cells =
        static_cast<long long>(dm->szX()) * dm->szY() * dm->szZ();
    const double scoreC = (total_cells <= 0)
        ? 0.0
        : (WEIGHT_COVERAGE * mapped_cells / static_cast<double>(total_cells));

    return scoreA + scoreB + scoreC;
}

} // namespace

std::vector<double> MapsComparison::compare(const IMap3D& origin,
                                            const std::vector<IMap3D*> targets)
{
    std::vector<double> scores;
    scores.reserve(targets.size());
    for (const IMap3D* target : targets) {
        if (target == nullptr) {
            scores.push_back(-1.0);
        } else {
            scores.push_back(scoreOneTarget(origin, *target));
        }
    }
    return scores;
}

} // namespace drone_mapper
