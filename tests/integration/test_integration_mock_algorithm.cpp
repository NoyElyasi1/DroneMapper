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
    MOCK_METHOD(types::MovementCommand, nextMove,
                (const types::DroneState&, const types::LidarScanResult&), (override));
    MOCK_METHOD(void, applyVoxelUpdates,
                (const std::vector<types::MappedVoxel>&), (override));
};

class MockLidarFast2 : public ILidar {
public:
    MOCK_METHOD(types::LidarScanResult, scan, (Orientation), (const, override));
};

namespace {

types::DroneConfigData makeDrone() {
    types::DroneConfigData d;
    d.dimensions=30.0*cm; d.max_rotate=45.0*horizontal_angle[deg];
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

TEST(Integration, MockAlgorithmDrivesCompletion) {
    auto cfg = makeMapConfig();
    auto gt = std::make_unique<Map3DImpl>(cfg);
    auto output = std::make_unique<Map3DImpl>(cfg);

    auto gps = std::make_unique<MockGPS>(
        Position3D{50.0*x_extent[cm], 50.0*y_extent[cm], 50.0*z_extent[cm]},
        Orientation{0.0*horizontal_angle[deg], 0.0*altitude_angle[deg]});

    auto movement = std::make_unique<MockMovement>(*gps, *gt, makeDrone());
    auto lidar = std::make_unique<MockLidarFast2>();
    auto algo = std::make_unique<MockAlgorithm>();

    EXPECT_CALL(*lidar, scan(_)).WillRepeatedly(Return(types::LidarScanResult{}));
    EXPECT_CALL(*algo, applyVoxelUpdates(_)).Times(AtLeast(1));
    EXPECT_CALL(*algo, nextMove(_, _))
        .WillRepeatedly(Return(types::MovementCommand{types::MovementCommandType::Hover}));

    auto ctrl = std::make_unique<DroneControlImpl>(
        makeDrone(), makeMission(), *lidar, *gps, *movement, *output, *algo);

    MissionControlImpl mc{makeMission(), *ctrl};
    const auto result = mc.runMission();
    EXPECT_EQ(result.status, types::MissionRunStatus::Completed);
    EXPECT_EQ(result.steps, 1u);
}
