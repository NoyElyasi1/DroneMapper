#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <drone_mapper/DroneControlImpl.h>
#include <drone_mapper/ILidar.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockMovement.h>

using namespace drone_mapper;
using ::testing::Return;
using ::testing::_;
using ::testing::AtLeast;

class MockLidarFast : public ILidar {
public:
    MOCK_METHOD(types::LidarScanResult, scan, (Orientation), (const, override));
};

class MockMappingAlgorithm : public IMappingAlgorithm {
public:
    MOCK_METHOD(types::MovementCommand, nextMove,
                (const types::DroneState&, const types::LidarScanResult&), (override));
    MOCK_METHOD(void, applyVoxelUpdates,
                (const std::vector<types::MappedVoxel>&), (override));
};

namespace {

types::DroneConfigData makeDrone() {
    types::DroneConfigData d;
    d.dimensions = 30.0*cm; d.max_rotate = 45.0*horizontal_angle[deg];
    d.max_advance = 50.0*cm; d.max_elevate = 40.0*cm;
    return d;
}

types::MissionConfigData makeMission() {
    types::MissionConfigData m;
    m.max_steps = 100; m.gps_resolution = 10.0*cm;
    m.output_mapping_resolution_factor = 1.0;
    return m;
}

types::MapConfig makeMapConfig() {
    types::MapConfig cfg;
    cfg.resolution = 10.0*cm;
    cfg.boundaries.min_x = -500.0*x_extent[cm]; cfg.boundaries.max_x = 500.0*x_extent[cm];
    cfg.boundaries.min_y = -500.0*y_extent[cm]; cfg.boundaries.max_y = 500.0*y_extent[cm];
    cfg.boundaries.min_height = 0.0*z_extent[cm]; cfg.boundaries.max_height = 300.0*z_extent[cm];
    return cfg;
}

} // namespace

TEST(DroneControl, StepReturnsCompletedOnHover) {
    Map3DImpl map{makeMapConfig()};
    MockGPS gps{Position3D{0.0*x_extent[cm], 0.0*y_extent[cm], 50.0*z_extent[cm]},
                Orientation{0.0*horizontal_angle[deg], 0.0*altitude_angle[deg]}};
    MockMovement movement{gps, map, makeDrone()};
    Map3DImpl output_map{makeMapConfig()};
    MockLidarFast lidar;
    MockMappingAlgorithm algo;

    EXPECT_CALL(lidar, scan(_)).WillRepeatedly(Return(types::LidarScanResult{}));
    EXPECT_CALL(algo, applyVoxelUpdates(_)).Times(AtLeast(1));
    EXPECT_CALL(algo, nextMove(_, _))
        .WillRepeatedly(Return(types::MovementCommand{types::MovementCommandType::Hover}));

    DroneControlImpl ctrl{makeDrone(), makeMission(), lidar, gps, movement, output_map, algo};
    const auto result = ctrl.step();
    EXPECT_EQ(result.status, types::DroneStepStatus::Completed);
}

TEST(DroneControl, StateReflectsGPS) {
    Map3DImpl map{makeMapConfig()};
    MockGPS gps{Position3D{10.0*x_extent[cm], 20.0*y_extent[cm], 30.0*z_extent[cm]},
                Orientation{45.0*horizontal_angle[deg], 0.0*altitude_angle[deg]}};
    MockMovement movement{gps, map, makeDrone()};
    Map3DImpl output_map{makeMapConfig()};
    MockLidarFast lidar;
    MockMappingAlgorithm algo;

    DroneControlImpl ctrl{makeDrone(), makeMission(), lidar, gps, movement, output_map, algo};
    const auto state = ctrl.state();
    EXPECT_NEAR(state.position.x.numerical_value_in(cm), 10.0, 1e-6);
    EXPECT_NEAR(state.position.y.numerical_value_in(cm), 20.0, 1e-6);
    EXPECT_NEAR(state.position.z.numerical_value_in(cm), 30.0, 1e-6);
    EXPECT_NEAR(state.heading.horizontal.numerical_value_in(deg), 45.0, 1e-6);
}
