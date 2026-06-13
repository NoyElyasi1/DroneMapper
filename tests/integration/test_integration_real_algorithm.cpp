#include <gtest/gtest.h>

#include <drone_mapper/DroneControlImpl.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MappingAlgorithmImpl.h>
#include <drone_mapper/MapsComparison.h>
#include <drone_mapper/MissionControlImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockLidar.h>
#include <drone_mapper/MockMovement.h>
#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/SimulationRunImpl.h>
#include <drone_mapper/SimulationRunImpl.h>

#include <filesystem>

using namespace drone_mapper;

namespace {

types::DroneConfigData makeDrone() {
    types::DroneConfigData d;
    d.radius=30.0*cm; d.max_rotate=45.0*horizontal_angle[deg];
    d.max_advance=50.0*cm; d.max_elevate=40.0*cm;
    return d;
}

types::LidarConfigData makeLidar() {
    return types::LidarConfigData{{}, 20.0*cm, 50.0*cm, 2.5*cm, 1};
}

types::MissionConfigData makeMission(std::size_t steps = 200) {
    types::MissionConfigData m;
    m.max_steps = steps;
    m.gps_resolution = 10.0 * cm;
    m.output_mapping_resolution_factor = 1.0;
    return m;
}

types::MapConfig makeMapConfig() {
    types::MapConfig cfg;
    cfg.resolution = 10.0 * cm;
    cfg.boundaries.min_x      = 0.0 * x_extent[cm];
    cfg.boundaries.max_x      = 100.0 * x_extent[cm];
    cfg.boundaries.min_y      = 0.0 * y_extent[cm];
    cfg.boundaries.max_y      = 100.0 * y_extent[cm];
    cfg.boundaries.min_height = 0.0 * z_extent[cm];
    cfg.boundaries.max_height = 100.0 * z_extent[cm];
    return cfg;
}

} // namespace

// ---- Basic run: real algo, real lidar mock, no crash ----

TEST(Integration, RealAlgorithmRunsWithoutCrash) {
    auto cfg = makeMapConfig();
    auto gt = std::make_unique<Map3DImpl>(cfg);
    auto output = std::make_unique<Map3DImpl>(cfg);

    auto gps = std::make_unique<MockGPS>(
        Position3D{50.0*x_extent[cm], 50.0*y_extent[cm], 50.0*z_extent[cm]},
        Orientation{0.0*horizontal_angle[deg], 0.0*altitude_angle[deg]});

    auto movement = std::make_unique<MockMovement>(*gps, *gt, makeDrone());
    auto lidar = std::make_unique<MockLidar>(makeLidar(), *gt, *gps);
    auto algo = std::make_unique<MappingAlgorithmImpl>(makeDrone(), *output);
    auto ctrl = std::make_unique<DroneControlImpl>(
        makeDrone(), makeMission(), *lidar, *gps, *movement, *output, *algo);

    const std::filesystem::path out =
        std::filesystem::temp_directory_path() / "integration_real_output.npy";
    auto mc = std::make_unique<MissionControlImpl>(makeMission(), *ctrl);

    EXPECT_NO_THROW({
        const auto result = mc->runMission();
        EXPECT_NE(result.status, types::MissionRunStatus::Error);
        output->save(out);
        auto* out_ptr = output.get();
        const auto scores = MapsComparison::compare(*gt, {out_ptr});
        EXPECT_GE(scores[0], 0.0);
        EXPECT_LE(scores[0], 100.0);
    });

    std::filesystem::remove(out);
}

// ---- Mission ends with Completed or MaxSteps (never Error) ----

TEST(Integration, MissionReachesCompletedOrMaxSteps) {
    auto cfg = makeMapConfig();
    auto gt = std::make_unique<Map3DImpl>(cfg);
    auto output = std::make_unique<Map3DImpl>(cfg);

    auto gps = std::make_unique<MockGPS>(
        Position3D{50.0*x_extent[cm], 50.0*y_extent[cm], 50.0*z_extent[cm]},
        Orientation{0.0*horizontal_angle[deg], 0.0*altitude_angle[deg]});

    auto movement = std::make_unique<MockMovement>(*gps, *gt, makeDrone());
    auto lidar = std::make_unique<MockLidar>(makeLidar(), *gt, *gps);
    // Fixed constructor: (DroneConfigData, IMap3D&) — NOT (MissionConfigData, DroneConfigData, MapConfig)
    auto algo = std::make_unique<MappingAlgorithmImpl>(makeDrone(), *output);
    auto ctrl = std::make_unique<DroneControlImpl>(
        makeDrone(), makeMission(50), *lidar, *gps, *movement, *output, *algo);

    const std::filesystem::path out =
        std::filesystem::temp_directory_path() / "integration_real_output2.npy";
    auto mc = std::make_unique<MissionControlImpl>(makeMission(50), *ctrl);

    const auto result = mc->runMission();
    EXPECT_NE(result.status, types::MissionRunStatus::Error);
    EXPECT_GE(result.steps, 1u);
    EXPECT_LE(result.steps, 50u);

    std::filesystem::remove(out);
}

// ---- Score from real algorithm is in [0, 100] ----

TEST(Integration, RealAlgorithmScoreIsInValidRange) {
    auto cfg = makeMapConfig();
    // Put a few obstacles in the ground truth
    auto gt = std::make_unique<Map3DImpl>(cfg);
    gt->set(Position3D{30.0*x_extent[cm], 50.0*y_extent[cm], 50.0*z_extent[cm]},
            types::VoxelOccupancy::Occupied);
    gt->set(Position3D{70.0*x_extent[cm], 50.0*y_extent[cm], 50.0*z_extent[cm]},
            types::VoxelOccupancy::Occupied);

    auto output = std::make_unique<Map3DImpl>(cfg);
    auto gps = std::make_unique<MockGPS>(
        Position3D{50.0*x_extent[cm], 50.0*y_extent[cm], 50.0*z_extent[cm]},
        Orientation{0.0*horizontal_angle[deg], 0.0*altitude_angle[deg]});
    auto movement = std::make_unique<MockMovement>(*gps, *gt, makeDrone());
    auto lidar = std::make_unique<MockLidar>(makeLidar(), *gt, *gps);
    auto algo = std::make_unique<MappingAlgorithmImpl>(makeDrone(), *output);
    auto ctrl = std::make_unique<DroneControlImpl>(
        makeDrone(), makeMission(300), *lidar, *gps, *movement, *output, *algo);
    auto mc = std::make_unique<MissionControlImpl>(makeMission(300), *ctrl);

    mc->runMission();

    const std::filesystem::path out =
        std::filesystem::temp_directory_path() / "integration_score_test.npy";
    output->save(out);

    auto* out_ptr = output.get();
    const auto scores = MapsComparison::compare(*gt, {out_ptr});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_GE(scores[0], 0.0);
    EXPECT_LE(scores[0], 100.0);

    std::filesystem::remove(out);
}

// ---- SimulationRunImpl wire-up (end-to-end, all real) ----

TEST(Integration, SimulationRunImplEndToEnd) {
    auto cfg = makeMapConfig();
    auto drone = makeDrone();
    auto lidar_cfg = makeLidar();
    auto mission = makeMission(100);

    auto hidden_map = std::make_unique<Map3DImpl>(cfg);
    auto output_map = std::make_unique<Map3DImpl>(cfg);
    auto gps = std::make_unique<MockGPS>(
        Position3D{50.0*x_extent[cm], 50.0*y_extent[cm], 50.0*z_extent[cm]},
        Orientation{0.0*horizontal_angle[deg], 0.0*altitude_angle[deg]});
    auto movement   = std::make_unique<MockMovement>(*gps, *hidden_map, drone);
    auto lidar      = std::make_unique<MockLidar>(lidar_cfg, *hidden_map, *gps);
    auto algo       = std::make_unique<MappingAlgorithmImpl>(drone, *output_map);
    auto ctrl       = std::make_unique<DroneControlImpl>(
        drone, mission, *lidar, *gps, *movement, *output_map, *algo);
    auto mc         = std::make_unique<MissionControlImpl>(mission, *ctrl);

    const std::filesystem::path out =
        std::filesystem::temp_directory_path() / "integration_e2e.npy";

    SimulationRunImpl run{
        {}, mission, drone, lidar_cfg,
        types::ResolutionRequestStatus::Accepted,
        std::move(hidden_map), std::move(output_map),
        std::move(gps), std::move(movement), std::move(lidar),
        std::move(algo), std::move(ctrl), std::move(mc), out};

    EXPECT_NO_THROW({
        const auto result = run.run();
        EXPECT_NE(result.mission_results[0].status, types::MissionRunStatus::Error);
        EXPECT_GE(result.mission_score, 0.0);
        EXPECT_LE(result.mission_score, 100.0);
        EXPECT_TRUE(std::filesystem::exists(out));
    });

    std::filesystem::remove(out);
}
