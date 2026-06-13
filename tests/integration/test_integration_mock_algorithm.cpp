#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <drone_mapper/DroneControlImpl.h>
#include <drone_mapper/ILidar.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MissionControlImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockMovement.h>

#include <filesystem>

using namespace drone_mapper;
using ::testing::Return;
using ::testing::_;
using ::testing::AtLeast;

class MockAlgorithm : public IMappingAlgorithm {
public:
    MockAlgorithm(types::DroneConfigData d, const IMap3D& m)
        : IMappingAlgorithm(d, m) {}
    MOCK_METHOD(types::MappingStepCommand, nextStep,
                (const types::DroneState&, const types::LidarScanResult*), (override));
};

class MockLidarInt : public ILidar {
public:
    MOCK_METHOD(types::LidarScanResult, scan, (Orientation), (const, override));
};

namespace {

types::DroneConfigData makeDrone() {
    types::DroneConfigData d;
    d.radius=30.0*cm; d.max_rotate=45.0*horizontal_angle[deg];
    d.max_advance=50.0*cm; d.max_elevate=40.0*cm;
    return d;
}

types::MapConfig makeMapConfig() {
    types::MapConfig cfg;
    cfg.resolution = 10.0*cm;
    cfg.boundaries.min_x=0.0*x_extent[cm]; cfg.boundaries.max_x=100.0*x_extent[cm];
    cfg.boundaries.min_y=0.0*y_extent[cm]; cfg.boundaries.max_y=100.0*y_extent[cm];
    cfg.boundaries.min_height=0.0*z_extent[cm]; cfg.boundaries.max_height=100.0*z_extent[cm];
    return cfg;
}

types::MissionConfigData makeMission(std::size_t steps = 5) {
    types::MissionConfigData m;
    m.max_steps=steps; m.gps_resolution=10.0*cm; m.output_mapping_resolution_factor=1.0;
    return m;
}

} // namespace

// ---- Mock algorithm completes on first step ----

TEST(Integration, MockAlgorithmDrivesCompletion) {
    auto cfg = makeMapConfig();
    auto gt = std::make_unique<Map3DImpl>(cfg);
    auto output = std::make_unique<Map3DImpl>(cfg);

    auto gps = std::make_unique<MockGPS>(
        Position3D{50.0*x_extent[cm], 50.0*y_extent[cm], 50.0*z_extent[cm]},
        Orientation{0.0*horizontal_angle[deg], 0.0*altitude_angle[deg]});

    auto movement = std::make_unique<MockMovement>(*gps, *gt, makeDrone());
    auto lidar = std::make_unique<MockLidarInt>();
    auto algo = std::make_unique<MockAlgorithm>(makeDrone(), *output);

    EXPECT_CALL(*lidar, scan(_)).WillRepeatedly(Return(types::LidarScanResult{}));
    EXPECT_CALL(*algo, nextStep(_, _))
        .WillRepeatedly(Return(types::MappingStepCommand{
            std::nullopt, std::nullopt, types::AlgorithmStatus::Finished}));

    auto ctrl = std::make_unique<DroneControlImpl>(
        makeDrone(), makeMission(), *lidar, *gps, *movement, *output, *algo);

    MissionControlImpl mc{makeMission(), *ctrl};
    const auto result = mc.runMission();
    EXPECT_EQ(result.status, types::MissionRunStatus::Completed);
    EXPECT_EQ(result.steps, 1u);
}

// ---- Mock algorithm keeps working for N steps then finishes ----

TEST(Integration, MockAlgorithmRunsMultipleStepsBeforeFinish) {
    auto cfg = makeMapConfig();
    auto gt = std::make_unique<Map3DImpl>(cfg);
    auto output = std::make_unique<Map3DImpl>(cfg);

    auto gps = std::make_unique<MockGPS>(
        Position3D{50.0*x_extent[cm], 50.0*y_extent[cm], 50.0*z_extent[cm]},
        Orientation{0.0*horizontal_angle[deg], 0.0*altitude_angle[deg]});

    auto movement = std::make_unique<MockMovement>(*gps, *gt, makeDrone());
    auto lidar = std::make_unique<MockLidarInt>();
    auto algo = std::make_unique<MockAlgorithm>(makeDrone(), *output);

    EXPECT_CALL(*lidar, scan(_)).WillRepeatedly(Return(types::LidarScanResult{}));
    {
        using ::testing::InSequence;
        InSequence seq;
        EXPECT_CALL(*algo, nextStep(_, _)).Times(4)
            .WillRepeatedly(Return(types::MappingStepCommand{
                std::nullopt, std::nullopt, types::AlgorithmStatus::Working}));
        EXPECT_CALL(*algo, nextStep(_, _))
            .WillOnce(Return(types::MappingStepCommand{
                std::nullopt, std::nullopt, types::AlgorithmStatus::Finished}));
    }

    auto ctrl = std::make_unique<DroneControlImpl>(
        makeDrone(), makeMission(10), *lidar, *gps, *movement, *output, *algo);

    MissionControlImpl mc{makeMission(10), *ctrl};
    const auto result = mc.runMission();
    EXPECT_EQ(result.status, types::MissionRunStatus::Completed);
    EXPECT_EQ(result.steps, 5u);
}

// ---- Mock algo with scan: lidar is queried ----

TEST(Integration, MockAlgorithmWithScanCallsLidar) {
    auto cfg = makeMapConfig();
    auto gt = std::make_unique<Map3DImpl>(cfg);
    auto output = std::make_unique<Map3DImpl>(cfg);

    auto gps = std::make_unique<MockGPS>(
        Position3D{50.0*x_extent[cm], 50.0*y_extent[cm], 50.0*z_extent[cm]},
        Orientation{0.0*horizontal_angle[deg], 0.0*altitude_angle[deg]});
    auto movement = std::make_unique<MockMovement>(*gps, *gt, makeDrone());
    auto lidar = std::make_unique<MockLidarInt>();
    auto algo = std::make_unique<MockAlgorithm>(makeDrone(), *output);

    // One scan then finish
    EXPECT_CALL(*lidar, scan(_)).Times(AtLeast(1))
        .WillRepeatedly(Return(types::LidarScanResult{}));
    {
        using ::testing::InSequence;
        InSequence seq;
        EXPECT_CALL(*algo, nextStep(_, _))
            .WillOnce(Return(types::MappingStepCommand{
                std::nullopt,
                Orientation{0.0*horizontal_angle[deg], 0.0*altitude_angle[deg]},
                types::AlgorithmStatus::Working}));
        EXPECT_CALL(*algo, nextStep(_, _))
            .WillOnce(Return(types::MappingStepCommand{
                std::nullopt, std::nullopt, types::AlgorithmStatus::Finished}));
    }

    auto ctrl = std::make_unique<DroneControlImpl>(
        makeDrone(), makeMission(10), *lidar, *gps, *movement, *output, *algo);

    MissionControlImpl mc{makeMission(10), *ctrl};
    const auto result = mc.runMission();
    EXPECT_EQ(result.status, types::MissionRunStatus::Completed);
}

// ---- MaxSteps reached with always-working algorithm ----

TEST(Integration, MockAlgorithmHitsMaxSteps) {
    auto cfg = makeMapConfig();
    auto gt = std::make_unique<Map3DImpl>(cfg);
    auto output = std::make_unique<Map3DImpl>(cfg);

    auto gps = std::make_unique<MockGPS>(
        Position3D{50.0*x_extent[cm], 50.0*y_extent[cm], 50.0*z_extent[cm]},
        Orientation{0.0*horizontal_angle[deg], 0.0*altitude_angle[deg]});
    auto movement = std::make_unique<MockMovement>(*gps, *gt, makeDrone());
    auto lidar = std::make_unique<MockLidarInt>();
    auto algo = std::make_unique<MockAlgorithm>(makeDrone(), *output);

    EXPECT_CALL(*algo, nextStep(_, _))
        .WillRepeatedly(Return(types::MappingStepCommand{
            std::nullopt, std::nullopt, types::AlgorithmStatus::Working}));

    auto ctrl = std::make_unique<DroneControlImpl>(
        makeDrone(), makeMission(3), *lidar, *gps, *movement, *output, *algo);

    MissionControlImpl mc{makeMission(3), *ctrl};
    const auto result = mc.runMission();
    EXPECT_EQ(result.status, types::MissionRunStatus::MaxSteps);
    EXPECT_EQ(result.steps, 3u);
}
