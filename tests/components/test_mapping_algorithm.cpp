#include <gtest/gtest.h>

#include <drone_mapper/MappingAlgorithmImpl.h>

using namespace drone_mapper;

namespace {

types::MissionConfigData makeMission() {
    types::MissionConfigData m;
    m.max_steps = 2400;
    m.gps_resolution = 10.0 * cm;
    m.output_mapping_resolution_factor = 1.0;
    return m;
}

types::DroneConfigData makeDrone() {
    types::DroneConfigData d;
    d.dimensions  = 30.0 * cm;
    d.max_rotate  = 45.0 * horizontal_angle[deg];
    d.max_advance = 50.0 * cm;
    d.max_elevate = 40.0 * cm;
    return d;
}

types::MapConfig makeMapConfig() {
    types::MapConfig cfg;
    cfg.resolution = 10.0 * cm;
    cfg.boundaries.min_x      = -500.0 * x_extent[cm];
    cfg.boundaries.max_x      =  500.0 * x_extent[cm];
    cfg.boundaries.min_y      = -500.0 * y_extent[cm];
    cfg.boundaries.max_y      =  500.0 * y_extent[cm];
    cfg.boundaries.min_height =    0.0 * z_extent[cm];
    cfg.boundaries.max_height =  300.0 * z_extent[cm];
    return cfg;
}

types::DroneState makeState(double x, double y, double z, double hdeg = 0.0) {
    return {
        Position3D{x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]},
        Orientation{hdeg * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
        0,
    };
}

} // namespace

TEST(MappingAlgorithm, InitiallyReturnsNonHoverOnFirstCall) {
    MappingAlgorithmImpl algo{makeMission(), makeDrone(), makeMapConfig()};
    const auto state = makeState(0, 0, 50);
    const auto cmd = algo.nextMove(state, {});
    EXPECT_NE(cmd.type, types::MovementCommandType::Hover);
}

TEST(MappingAlgorithm, ApplyVoxelUpdatesDoesNotCrash) {
    MappingAlgorithmImpl algo{makeMission(), makeDrone(), makeMapConfig()};
    std::vector<types::MappedVoxel> voxels{
        {Position3D{10.0 * x_extent[cm], 0.0 * y_extent[cm], 50.0 * z_extent[cm]},
         types::VoxelOccupancy::Empty},
        {Position3D{20.0 * x_extent[cm], 0.0 * y_extent[cm], 50.0 * z_extent[cm]},
         types::VoxelOccupancy::Occupied},
    };
    EXPECT_NO_THROW(algo.applyVoxelUpdates(voxels));
}

TEST(MappingAlgorithm, OccupiedNotOverwrittenByEmpty) {
    MappingAlgorithmImpl algo{makeMission(), makeDrone(), makeMapConfig()};
    const Position3D pos{10.0 * x_extent[cm], 0.0 * y_extent[cm], 50.0 * z_extent[cm]};
    std::vector<types::MappedVoxel> occ{{pos, types::VoxelOccupancy::Occupied}};
    std::vector<types::MappedVoxel> emp{{pos, types::VoxelOccupancy::Empty}};
    algo.applyVoxelUpdates(occ);
    algo.applyVoxelUpdates(emp);
    const auto state = makeState(0, 0, 50);
    EXPECT_NO_THROW(algo.nextMove(state, {}));
}
