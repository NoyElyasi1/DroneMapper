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
using ::testing::Exactly;
using ::testing::StrictMock;

class MockLidarFast : public ILidar {
public:
    MOCK_METHOD(types::LidarScanResult, scan, (Orientation), (const, override));
};

class MockMappingAlgorithm : public IMappingAlgorithm {
public:
    MockMappingAlgorithm(types::DroneConfigData d, const IMap3D& m)
        : IMappingAlgorithm(d, m) {}
    MOCK_METHOD(types::MappingStepCommand, nextStep,
                (const types::DroneState&, const types::LidarScanResult*), (override));
};

// Thin wrapper around MockMovement to spy on specific calls.
class SpyMovement : public IDroneMovement {
public:
    SpyMovement() = default;
    MOCK_METHOD(types::MovementResult, advance, (PhysicalLength), (override));
    MOCK_METHOD(types::MovementResult, elevate, (PhysicalLength), (override));
    MOCK_METHOD(types::MovementResult, rotate,
                (types::RotationDirection, HorizontalAngle), (override));
};

namespace {

types::DroneConfigData makeDrone() {
    types::DroneConfigData d;
    d.radius = 30.0*cm; d.max_rotate = 45.0*horizontal_angle[deg];
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

// ---- State / construction ----

TEST(DroneControl, StateReflectsGPS) {
    Map3DImpl map{makeMapConfig()};
    MockGPS gps{Position3D{10.0*x_extent[cm], 20.0*y_extent[cm], 30.0*z_extent[cm]},
                Orientation{45.0*horizontal_angle[deg], 0.0*altitude_angle[deg]}};
    MockMovement movement{gps, map, makeDrone()};
    Map3DImpl output_map{makeMapConfig()};
    MockLidarFast lidar;
    MockMappingAlgorithm algo{makeDrone(), output_map};

    DroneControlImpl ctrl{makeDrone(), makeMission(), lidar, gps, movement, output_map, algo};
    const auto state = ctrl.state();
    EXPECT_NEAR(state.position.x.numerical_value_in(cm), 10.0, 1e-6);
    EXPECT_NEAR(state.position.y.numerical_value_in(cm), 20.0, 1e-6);
    EXPECT_NEAR(state.position.z.numerical_value_in(cm), 30.0, 1e-6);
    EXPECT_NEAR(state.heading.horizontal.numerical_value_in(deg), 45.0, 1e-6);
}

TEST(DroneControl, StepIndexStartsAtZero) {
    Map3DImpl map{makeMapConfig()};
    MockGPS gps{Position3D{0.0*x_extent[cm], 0.0*y_extent[cm], 50.0*z_extent[cm]},
                Orientation{0.0*horizontal_angle[deg], 0.0*altitude_angle[deg]}};
    MockMovement movement{gps, map, makeDrone()};
    Map3DImpl output_map{makeMapConfig()};
    MockLidarFast lidar;
    MockMappingAlgorithm algo{makeDrone(), output_map};

    DroneControlImpl ctrl{makeDrone(), makeMission(), lidar, gps, movement, output_map, algo};
    EXPECT_EQ(ctrl.state().step_index, 0u);
}

// ---- AlgorithmStatus → DroneStepStatus ----

TEST(DroneControl, StepReturnsCompletedOnFinished) {
    Map3DImpl map{makeMapConfig()};
    MockGPS gps{Position3D{0.0*x_extent[cm], 0.0*y_extent[cm], 50.0*z_extent[cm]},
                Orientation{0.0*horizontal_angle[deg], 0.0*altitude_angle[deg]}};
    MockMovement movement{gps, map, makeDrone()};
    Map3DImpl output_map{makeMapConfig()};
    MockLidarFast lidar;
    MockMappingAlgorithm algo{makeDrone(), output_map};

    EXPECT_CALL(lidar, scan(_)).WillRepeatedly(Return(types::LidarScanResult{}));
    EXPECT_CALL(algo, nextStep(_, _))
        .WillRepeatedly(Return(types::MappingStepCommand{
            std::nullopt, std::nullopt, types::AlgorithmStatus::Finished}));

    DroneControlImpl ctrl{makeDrone(), makeMission(), lidar, gps, movement, output_map, algo};
    const auto result = ctrl.step();
    EXPECT_EQ(result.status, types::DroneStepStatus::Completed);
}

TEST(DroneControl, StepReturnsContinueOnWorking) {
    Map3DImpl map{makeMapConfig()};
    MockGPS gps{Position3D{0.0*x_extent[cm], 0.0*y_extent[cm], 50.0*z_extent[cm]},
                Orientation{0.0*horizontal_angle[deg], 0.0*altitude_angle[deg]}};
    MockMovement movement{gps, map, makeDrone()};
    Map3DImpl output_map{makeMapConfig()};
    MockLidarFast lidar;
    MockMappingAlgorithm algo{makeDrone(), output_map};

    EXPECT_CALL(algo, nextStep(_, _))
        .WillRepeatedly(Return(types::MappingStepCommand{
            std::nullopt, std::nullopt, types::AlgorithmStatus::Working}));

    DroneControlImpl ctrl{makeDrone(), makeMission(), lidar, gps, movement, output_map, algo};
    const auto result = ctrl.step();
    EXPECT_EQ(result.status, types::DroneStepStatus::Continue);
}

TEST(DroneControl, StepReturnsCompletedOnFinishedWithUnmappable) {
    Map3DImpl map{makeMapConfig()};
    MockGPS gps{Position3D{0.0*x_extent[cm], 0.0*y_extent[cm], 50.0*z_extent[cm]},
                Orientation{0.0*horizontal_angle[deg], 0.0*altitude_angle[deg]}};
    MockMovement movement{gps, map, makeDrone()};
    Map3DImpl output_map{makeMapConfig()};
    MockLidarFast lidar;
    MockMappingAlgorithm algo{makeDrone(), output_map};

    EXPECT_CALL(algo, nextStep(_, _))
        .WillRepeatedly(Return(types::MappingStepCommand{
            std::nullopt, std::nullopt, types::AlgorithmStatus::FinishedWithUnmappableVoxels}));

    DroneControlImpl ctrl{makeDrone(), makeMission(), lidar, gps, movement, output_map, algo};
    const auto result = ctrl.step();
    EXPECT_EQ(result.status, types::DroneStepStatus::Completed);
}

// ---- Scan behaviour ----

TEST(DroneControl, ScanIsCalledWhenAlgoRequests) {
    Map3DImpl map{makeMapConfig()};
    MockGPS gps{Position3D{0.0*x_extent[cm], 0.0*y_extent[cm], 50.0*z_extent[cm]},
                Orientation{0.0*horizontal_angle[deg], 0.0*altitude_angle[deg]}};
    MockMovement movement{gps, map, makeDrone()};
    Map3DImpl output_map{makeMapConfig()};
    MockLidarFast lidar;
    MockMappingAlgorithm algo{makeDrone(), output_map};

    const Orientation scan_dir{0.0*horizontal_angle[deg], 0.0*altitude_angle[deg]};
    EXPECT_CALL(lidar, scan(_)).Times(AtLeast(1)).WillRepeatedly(Return(types::LidarScanResult{}));
    EXPECT_CALL(algo, nextStep(_, _))
        .WillOnce(Return(types::MappingStepCommand{
            std::nullopt, scan_dir, types::AlgorithmStatus::Finished}));

    DroneControlImpl ctrl{makeDrone(), makeMission(), lidar, gps, movement, output_map, algo};
    ctrl.step();
}

TEST(DroneControl, ScanNotCalledWhenAlgoDoesNotRequest) {
    Map3DImpl map{makeMapConfig()};
    MockGPS gps{Position3D{0.0*x_extent[cm], 0.0*y_extent[cm], 50.0*z_extent[cm]},
                Orientation{0.0*horizontal_angle[deg], 0.0*altitude_angle[deg]}};
    MockMovement movement{gps, map, makeDrone()};
    Map3DImpl output_map{makeMapConfig()};
    StrictMock<MockLidarFast> lidar;  // will fail if scan() is called unexpectedly
    MockMappingAlgorithm algo{makeDrone(), output_map};

    EXPECT_CALL(algo, nextStep(_, _))
        .WillOnce(Return(types::MappingStepCommand{
            std::nullopt, std::nullopt, types::AlgorithmStatus::Finished}));

    DroneControlImpl ctrl{makeDrone(), makeMission(), lidar, gps, movement, output_map, algo};
    ctrl.step();  // scan() must NOT be called
}

// ---- Movement behaviour ----

TEST(DroneControl, AdvanceCommandCallsMovementAdvance) {
    Map3DImpl map{makeMapConfig()};
    MockGPS gps{Position3D{0.0*x_extent[cm], 0.0*y_extent[cm], 50.0*z_extent[cm]},
                Orientation{0.0*horizontal_angle[deg], 0.0*altitude_angle[deg]}};
    Map3DImpl output_map{makeMapConfig()};
    MockLidarFast lidar;
    MockMappingAlgorithm algo{makeDrone(), output_map};

    // Use a spy movement to verify advance is called.
    StrictMock<SpyMovement> spy;
    EXPECT_CALL(spy, advance(_))
        .WillOnce(Return(types::MovementResult{true, {}}));

    types::MovementCommand mv;
    mv.type = types::MovementCommandType::Advance;
    mv.distance = 30.0*cm;
    EXPECT_CALL(algo, nextStep(_, _))
        .WillOnce(Return(types::MappingStepCommand{mv, std::nullopt, types::AlgorithmStatus::Finished}));

    DroneControlImpl ctrl{makeDrone(), makeMission(), lidar, gps, spy, output_map, algo};
    ctrl.step();
}

TEST(DroneControl, RotateCommandCallsMovementRotate) {
    Map3DImpl map{makeMapConfig()};
    MockGPS gps{Position3D{0.0*x_extent[cm], 0.0*y_extent[cm], 50.0*z_extent[cm]},
                Orientation{0.0*horizontal_angle[deg], 0.0*altitude_angle[deg]}};
    Map3DImpl output_map{makeMapConfig()};
    MockLidarFast lidar;
    MockMappingAlgorithm algo{makeDrone(), output_map};

    StrictMock<SpyMovement> spy;
    EXPECT_CALL(spy, rotate(_, _))
        .WillOnce(Return(types::MovementResult{true, {}}));

    types::MovementCommand mv;
    mv.type      = types::MovementCommandType::Rotate;
    mv.rotation  = types::RotationDirection::Left;
    mv.angle     = 30.0*horizontal_angle[deg];
    EXPECT_CALL(algo, nextStep(_, _))
        .WillOnce(Return(types::MappingStepCommand{mv, std::nullopt, types::AlgorithmStatus::Finished}));

    DroneControlImpl ctrl{makeDrone(), makeMission(), lidar, gps, spy, output_map, algo};
    ctrl.step();
}

// ---- Step index ----

TEST(DroneControl, StepIndexIncrementsEachStep) {
    Map3DImpl map{makeMapConfig()};
    MockGPS gps{Position3D{0.0*x_extent[cm], 0.0*y_extent[cm], 50.0*z_extent[cm]},
                Orientation{0.0*horizontal_angle[deg], 0.0*altitude_angle[deg]}};
    MockMovement movement{gps, map, makeDrone()};
    Map3DImpl output_map{makeMapConfig()};
    MockLidarFast lidar;
    MockMappingAlgorithm algo{makeDrone(), output_map};

    EXPECT_CALL(algo, nextStep(_, _))
        .WillRepeatedly(Return(types::MappingStepCommand{
            std::nullopt, std::nullopt, types::AlgorithmStatus::Working}));

    DroneControlImpl ctrl{makeDrone(), makeMission(), lidar, gps, movement, output_map, algo};
    EXPECT_EQ(ctrl.state().step_index, 0u);
    ctrl.step();
    EXPECT_EQ(ctrl.state().step_index, 1u);
    ctrl.step();
    EXPECT_EQ(ctrl.state().step_index, 2u);
}

// ---- nextStep receives last scan ----

TEST(DroneControl, NextStepReceivesNullScanOnFirstCall) {
    Map3DImpl map{makeMapConfig()};
    MockGPS gps{Position3D{0.0*x_extent[cm], 0.0*y_extent[cm], 50.0*z_extent[cm]},
                Orientation{0.0*horizontal_angle[deg], 0.0*altitude_angle[deg]}};
    MockMovement movement{gps, map, makeDrone()};
    Map3DImpl output_map{makeMapConfig()};
    MockLidarFast lidar;
    MockMappingAlgorithm algo{makeDrone(), output_map};

    // First call: scan_ptr should be nullptr (no previous scan)
    EXPECT_CALL(algo, nextStep(_, nullptr))
        .WillOnce(Return(types::MappingStepCommand{
            std::nullopt, std::nullopt, types::AlgorithmStatus::Finished}));

    DroneControlImpl ctrl{makeDrone(), makeMission(), lidar, gps, movement, output_map, algo};
    ctrl.step();
}
