#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <drone_mapper/SimulationRunImpl.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockMovement.h>
#include <drone_mapper/MockLidar.h>

using namespace drone_mapper;
using ::testing::Return;
using ::testing::_;

class MockDroneCtrl : public IDroneControl {
public:
    MOCK_METHOD(types::DroneStepResult, step, (), (override));
    MOCK_METHOD(types::DroneState, state, (), (const, override));
};

class MockMissionCtrl : public IMissionControl {
public:
    MOCK_METHOD(types::MissionRunResult, runMission, (), (override));
};

class MockMappingAlgo : public IMappingAlgorithm {
public:
    MOCK_METHOD(types::MovementCommand, nextMove,
                (const types::DroneState&, const types::LidarScanResult&), (override));
    MOCK_METHOD(void, applyVoxelUpdates,
                (const std::vector<types::MappedVoxel>&), (override));
};

namespace {
types::MapConfig makeMapConfig() {
    types::MapConfig cfg;
    cfg.resolution = 10.0*cm;
    cfg.boundaries.min_x = 0.0*x_extent[cm]; cfg.boundaries.max_x = 100.0*x_extent[cm];
    cfg.boundaries.min_y = 0.0*y_extent[cm]; cfg.boundaries.max_y = 100.0*y_extent[cm];
    cfg.boundaries.min_height = 0.0*z_extent[cm]; cfg.boundaries.max_height = 100.0*z_extent[cm];
    return cfg;
}
} // namespace

TEST(SimulationRun, DelegatesToMissionControl) {
    auto mc = std::make_unique<MockMissionCtrl>();
    const types::MissionRunResult mr{types::MissionRunStatus::Completed, 5, {}};
    EXPECT_CALL(*mc, runMission()).WillOnce(Return(mr));

    auto cfg = makeMapConfig();
    auto hidden_map = std::make_unique<Map3DImpl>(cfg);
    auto output_map = std::make_unique<Map3DImpl>(cfg);
    auto gps = std::make_unique<MockGPS>(Position3D{}, Orientation{});
    types::DroneConfigData d; d.dimensions=30.0*cm; d.max_rotate=45.0*horizontal_angle[deg]; d.max_advance=50.0*cm; d.max_elevate=40.0*cm;
    auto movement = std::make_unique<MockMovement>(*gps, *hidden_map, d);
    auto lidar = std::make_unique<MockLidar>(
        types::LidarConfigData{20.0*cm,120.0*cm,2.5*cm,1}, *hidden_map, *gps);
    auto algo = std::make_unique<MockMappingAlgo>();
    auto ctrl = std::make_unique<MockDroneCtrl>();

    types::MissionConfigData m; m.max_steps=10; m.gps_resolution=10.0*cm; m.output_mapping_resolution_factor=1.0;
    types::SimulationConfigData sim;
    const std::filesystem::path out = std::filesystem::temp_directory_path() / "test_sim_run.npy";

    SimulationRunImpl run{
        sim, m, d, types::LidarConfigData{20.0*cm,120.0*cm,2.5*cm,1},
        types::ResolutionRequestStatus::Accepted,
        std::move(hidden_map), std::move(output_map),
        std::move(gps), std::move(movement), std::move(lidar),
        std::move(algo), std::move(ctrl), std::move(mc), out};

    const auto result = run.run();
    ASSERT_EQ(result.mission_results.size(), 1u);
    EXPECT_EQ(result.mission_results[0].status, types::MissionRunStatus::Completed);
    EXPECT_GE(result.mission_score, 0.0);
    std::filesystem::remove(out);
}
