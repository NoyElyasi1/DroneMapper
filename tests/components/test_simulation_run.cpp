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
    MockMappingAlgo(types::DroneConfigData d, const IMap3D& m)
        : IMappingAlgorithm({}, {}, d, m) {}
    MOCK_METHOD(types::MappingStepCommand, nextStep,
                (const types::DroneState&, const types::LidarScanResult*), (override));
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

types::DroneConfigData makeDroneConfig() {
    types::DroneConfigData d;
    d.radius=30.0*cm; d.max_rotate=45.0*horizontal_angle[deg];
    d.max_advance=50.0*cm; d.max_elevate=40.0*cm;
    return d;
}

types::LidarConfigData makeLidarConfig() {
    return types::LidarConfigData{{}, 20.0*cm, 120.0*cm, 2.5*cm, 1};
}

// Builds a fully-wired SimulationRunImpl with a mock MissionControl that returns `mr`.
std::unique_ptr<SimulationRunImpl> buildRun(
    types::MissionRunResult mr,
    const std::filesystem::path& output_file,
    types::SimulationConfigData sim_cfg = {},
    types::MissionConfigData mis_cfg = {})
{
    auto cfg = makeMapConfig();
    auto drone = makeDroneConfig();
    auto lidar_cfg = makeLidarConfig();
    if (mis_cfg.max_steps == 0) { mis_cfg.max_steps = 10; mis_cfg.gps_resolution = 10.0*cm; mis_cfg.output_mapping_resolution_factor=1.0; }
    auto hidden_map  = std::make_unique<Map3DImpl>(cfg);
    auto output_map  = std::make_unique<Map3DImpl>(cfg);
    auto gps         = std::make_unique<MockGPS>(Position3D{}, Orientation{}, 10.0*cm);
    auto movement    = std::make_unique<MockMovement>(*gps, *hidden_map, drone);
    auto lidar       = std::make_unique<MockLidar>(lidar_cfg, *hidden_map, *gps);
    auto algo        = std::make_unique<MockMappingAlgo>(drone, *output_map);
    auto ctrl        = std::make_unique<MockDroneCtrl>();
    auto mc          = std::make_unique<MockMissionCtrl>();
    EXPECT_CALL(*mc, runMission()).WillOnce(Return(mr));

    return std::make_unique<SimulationRunImpl>(
        sim_cfg, mis_cfg, drone, lidar_cfg,
        types::ResolutionRequestStatus::Accepted,
        std::move(hidden_map), std::move(output_map),
        std::move(gps), std::move(movement), std::move(lidar),
        std::move(algo), std::move(ctrl), std::move(mc), output_file);
}

} // namespace

// ---- Happy path ----

TEST(SimulationRun, DelegatesToMissionControl) {
    const std::filesystem::path out = std::filesystem::temp_directory_path() / "sr_test1.npy";
    const types::MissionRunResult mr{types::MissionRunStatus::Completed, 5, {}};
    auto run = buildRun(mr, out);
    const auto result = run->run();

    ASSERT_EQ(result.mission_results.size(), 1u);
    EXPECT_EQ(result.mission_results[0].status, types::MissionRunStatus::Completed);
    EXPECT_GE(result.mission_score, 0.0);
    std::filesystem::remove(out);
}

TEST(SimulationRun, SavesOutputMapFile) {
    const std::filesystem::path out = std::filesystem::temp_directory_path() / "sr_test2.npy";
    std::filesystem::remove(out);  // ensure absent before run
    const types::MissionRunResult mr{types::MissionRunStatus::Completed, 3, {}};
    auto run = buildRun(mr, out);
    run->run();

    EXPECT_TRUE(std::filesystem::exists(out)) << "output_map.npy was not created";
    std::filesystem::remove(out);
}

TEST(SimulationRun, OutputMapFileInResult) {
    const std::filesystem::path out = std::filesystem::temp_directory_path() / "sr_test3.npy";
    auto run = buildRun({types::MissionRunStatus::Completed, 2, {}}, out);
    const auto result = run->run();

    EXPECT_EQ(result.output_map_file, out);
    std::filesystem::remove(out);
}

TEST(SimulationRun, ConfigDataPropagatedToResult) {
    const std::filesystem::path out = std::filesystem::temp_directory_path() / "sr_test4.npy";
    types::SimulationConfigData sim_cfg;
    sim_cfg.config_file = "my_sim.yaml";
    types::MissionConfigData mis_cfg;
    mis_cfg.config_file = "my_mission.yaml";
    mis_cfg.max_steps = 10; mis_cfg.gps_resolution = 10.0*cm;
    mis_cfg.output_mapping_resolution_factor = 1.0;

    auto run = buildRun({types::MissionRunStatus::Completed, 1, {}}, out, sim_cfg, mis_cfg);
    const auto result = run->run();

    EXPECT_EQ(result.simulation_config.config_file, std::filesystem::path{"my_sim.yaml"});
    EXPECT_EQ(result.mission_config.config_file,    std::filesystem::path{"my_mission.yaml"});
    std::filesystem::remove(out);
}

TEST(SimulationRun, ScoreIsBetween0And100OnSuccess) {
    const std::filesystem::path out = std::filesystem::temp_directory_path() / "sr_test5.npy";
    auto run = buildRun({types::MissionRunStatus::Completed, 5, {}}, out);
    const auto result = run->run();

    EXPECT_GE(result.mission_score, 0.0);
    EXPECT_LE(result.mission_score, 100.0);
    std::filesystem::remove(out);
}

// ---- Error handling ----

TEST(SimulationRun, ErrorMissionReturnsMinus1Score) {
    const std::filesystem::path out = std::filesystem::temp_directory_path() / "sr_test6.npy";
    const types::MissionRunResult mr{
        types::MissionRunStatus::Error, 2, {{"DRONE_ERROR", "boom"}}};
    auto run = buildRun(mr, out);
    const auto result = run->run();

    EXPECT_NEAR(result.mission_score, -1.0, 1e-6);
    std::filesystem::remove(out);
}

TEST(SimulationRun, ErrorMissionResultPreserved) {
    const std::filesystem::path out = std::filesystem::temp_directory_path() / "sr_test7.npy";
    const types::MissionRunResult mr{
        types::MissionRunStatus::Error, 1, {{"MY_ERROR", "details"}}};
    auto run = buildRun(mr, out);
    const auto result = run->run();

    ASSERT_FALSE(result.mission_results.empty());
    ASSERT_FALSE(result.mission_results[0].errors.empty());
    EXPECT_EQ(result.mission_results[0].errors[0].code, "MY_ERROR");
    std::filesystem::remove(out);
}

// ---- MockGPS / MockMovement interaction ----

TEST(SimulationRun, MockGPSReflectsInitialPosition) {
    auto cfg = makeMapConfig();
    Position3D init_pos{20.0*x_extent[cm], 30.0*y_extent[cm], 50.0*z_extent[cm]};
    MockGPS gps{init_pos, Orientation{}};

    EXPECT_NEAR(gps.position().x.numerical_value_in(cm), 20.0, 1e-6);
    EXPECT_NEAR(gps.position().y.numerical_value_in(cm), 30.0, 1e-6);
    EXPECT_NEAR(gps.position().z.numerical_value_in(cm), 50.0, 1e-6);
}

TEST(SimulationRun, MockGPSSetPositionUpdates) {
    MockGPS gps{Position3D{}, Orientation{}};
    Position3D new_pos{5.0*x_extent[cm], 10.0*y_extent[cm], 15.0*z_extent[cm]};
    gps.setPosition(new_pos);

    EXPECT_NEAR(gps.position().x.numerical_value_in(cm), 5.0, 1e-6);
    EXPECT_NEAR(gps.position().y.numerical_value_in(cm), 10.0, 1e-6);
}

TEST(SimulationRun, MockMovementAdvanceChangesGPS) {
    auto cfg = makeMapConfig();
    auto drone = makeDroneConfig();
    // Place drone in middle of empty map, heading east (0°)
    MockGPS gps{Position3D{50.0*x_extent[cm], 50.0*y_extent[cm], 50.0*z_extent[cm]},
                Orientation{0.0*horizontal_angle[deg], 0.0*altitude_angle[deg]}};
    Map3DImpl hidden{cfg};
    MockMovement movement{gps, hidden, drone};

    const auto before_x = gps.position().x.numerical_value_in(cm);
    movement.advance(20.0*cm);
    const auto after_x = gps.position().x.numerical_value_in(cm);

    // After advancing east by 20 cm, x should have changed.
    EXPECT_NE(before_x, after_x);
}

// ---- MockGPS with custom resolution constructs without throw ----

TEST(SimulationRun, MockGPSCustomResolutionConstructsOk) {
    MockGPS gps{Position3D{}, Orientation{}, 15.0*cm};
    EXPECT_NEAR(gps.position().x.numerical_value_in(cm), 0.0, 1e-6);
    EXPECT_NEAR(gps.position().y.numerical_value_in(cm), 0.0, 1e-6);
}

// ---- MockMovement rotate changes heading ----

TEST(SimulationRun, MockMovementRotateChangesHeading) {
    auto cfg = makeMapConfig();
    auto drone = makeDroneConfig();
    MockGPS gps{Position3D{50.0*x_extent[cm], 50.0*y_extent[cm], 50.0*z_extent[cm]},
                Orientation{0.0*horizontal_angle[deg], 0.0*altitude_angle[deg]}};
    Map3DImpl hidden{cfg};
    MockMovement movement{gps, hidden, drone};

    const double before_h = gps.heading().horizontal.numerical_value_in(deg);
    movement.rotate(types::RotationDirection::Left, 45.0*horizontal_angle[deg]);
    const double after_h = gps.heading().horizontal.numerical_value_in(deg);
    EXPECT_NE(before_h, after_h);
}

// ---- Score in result does not exceed 100 ----

TEST(SimulationRun, ScoreNeverExceeds100) {
    const std::filesystem::path out = std::filesystem::temp_directory_path() / "sr_test9.npy";
    auto run = buildRun({types::MissionRunStatus::Completed, 5, {}}, out);
    const auto result = run->run();
    EXPECT_LE(result.mission_score, 100.0);
    std::filesystem::remove(out);
}

// ---- Output map file is populated (non-zero size) ----

TEST(SimulationRun, OutputMapFileIsNonEmpty) {
    const std::filesystem::path out = std::filesystem::temp_directory_path() / "sr_test10.npy";
    auto run = buildRun({types::MissionRunStatus::Completed, 2, {}}, out);
    run->run();
    EXPECT_TRUE(std::filesystem::exists(out));
    EXPECT_GT(std::filesystem::file_size(out), 0u) << "Saved .npy should be non-empty";
    std::filesystem::remove(out);
}
