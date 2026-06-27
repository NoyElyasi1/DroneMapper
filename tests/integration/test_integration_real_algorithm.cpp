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
#include <drone_mapper/SimulationRunFactory.h>

#include <TinyNPY.h>

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
    auto algo = std::make_unique<MappingAlgorithmImpl>(makeMission(), makeLidar(), makeDrone(), *output);
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
    auto algo = std::make_unique<MappingAlgorithmImpl>(makeMission(), makeLidar(), makeDrone(), *output);
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
    auto algo = std::make_unique<MappingAlgorithmImpl>(makeMission(), makeLidar(), makeDrone(), *output);
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
    auto algo       = std::make_unique<MappingAlgorithmImpl>(mission, makeLidar(), drone, *output_map);
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

// ============================================================
// Benchmark map integration tests
// Map: data_maps/benchmark_map.npy — 29x30x31 voxels, 10cm/voxel
// Half solid, half a house with:
//   - Main 4×4 entrance, secret 2×1 roof entrance
//   - Floor 1: 5 high, Floor 2: 6 high (4×4 interior passage)
//   - 3×3 room, 2×2 room, 1×1 inner room on roof
// ============================================================

namespace {

// Returns the absolute path to data_maps/benchmark_map.npy.
// Works whether tests are run from project root or build/.
std::filesystem::path benchmarkMapPath() {
    // Try relative to binary (build/), then project root, then common locations.
    const std::vector<std::filesystem::path> candidates = {
        "data_maps/benchmark_map.npy",
        "../data_maps/benchmark_map.npy",
        "/mnt/c/Users/roni/DroneMapper/data_maps/benchmark_map.npy",
    };
    for (const auto& p : candidates) {
        if (std::filesystem::exists(p)) return p;
    }
    return "data_maps/benchmark_map.npy"; // will fail with a clear error
}

// Loads the benchmark NPY and builds a MapConfig matching 29×30×31 at 10cm/voxel, origin at 0.
std::pair<std::shared_ptr<NpyArray>, types::MapConfig> loadBenchmarkMap() {
    auto npy = std::make_shared<NpyArray>();
    const auto path = benchmarkMapPath();
    const char* err = npy->LoadNPY(path.string().c_str());
    if (err != nullptr) throw std::runtime_error(std::string("LoadNPY: ") + err);
    const auto& sh = npy->Shape();
    if (sh.size() != 3) throw std::runtime_error("Expected 3-D npy");

    const double res = 10.0;
    types::MapConfig cfg;
    cfg.resolution     = res * cm;
    cfg.offset         = Position3D{};
    cfg.boundaries.min_x      = 0.0 * x_extent[cm];
    cfg.boundaries.max_x      = (static_cast<double>(sh[0]) - 1) * res * x_extent[cm];
    cfg.boundaries.min_y      = 0.0 * y_extent[cm];
    cfg.boundaries.max_y      = (static_cast<double>(sh[1]) - 1) * res * y_extent[cm];
    cfg.boundaries.min_height = 0.0 * z_extent[cm];
    cfg.boundaries.max_height = (static_cast<double>(sh[2]) - 1) * res * z_extent[cm];
    return {npy, cfg};
}

types::LidarConfigData benchmarkLidar() {
    // Enough range to see across the 4×4 entrance (40cm) and small rooms.
    return types::LidarConfigData{{}, 10.0*cm, 120.0*cm, 5.0*cm, 3};
}

types::DroneConfigData benchmarkDroneLarge() {
    types::DroneConfigData d;
    d.radius      = 15.0*cm;  // fits through 4×4 (40cm) entrance
    d.max_rotate  = 45.0 * horizontal_angle[deg];
    d.max_advance = 10.0*cm;
    d.max_elevate = 10.0*cm;
    return d;
}

types::DroneConfigData benchmarkDroneSmall() {
    types::DroneConfigData d;
    d.radius      = 5.0*cm;  // fits through 1×1 (10cm) inner room
    d.max_rotate  = 45.0 * horizontal_angle[deg];
    d.max_advance = 10.0*cm;
    d.max_elevate = 10.0*cm;
    return d;
}

types::MissionConfigData benchmarkMission(std::size_t steps = 6000) {
    types::MissionConfigData m;
    m.max_steps = steps;
    m.gps_resolution = 10.0 * cm;
    m.output_mapping_resolution_factor = 1.0;
    return m;
}

} // anonymous namespace

// ---- Load benchmark map without crash ----
TEST(Integration, BenchmarkMapLoadsSuccessfully) {
    auto npy = std::make_shared<NpyArray>();
    const auto path = benchmarkMapPath();
    const char* err = npy->LoadNPY(path.string().c_str());
    ASSERT_EQ(err, nullptr) << "Failed to load benchmark map: " << (err ? err : "");
    const auto& sh = npy->Shape();
    ASSERT_EQ(sh.size(), 3u);
    EXPECT_EQ(sh[0], 29u);
    EXPECT_EQ(sh[1], 30u);
    EXPECT_EQ(sh[2], 31u);
}

// ---- Simulation on benchmark map with large drone (4×4 entrance) ----
TEST(Integration, BenchmarkMap_LargeDrone_RunsWithinOneMinute) {
    const auto [npy, cfg] = loadBenchmarkMap();
    auto hidden_map = std::make_unique<Map3DImpl>(npy, cfg);
    auto output_map = std::make_unique<Map3DImpl>(cfg);

    const auto drone   = benchmarkDroneLarge();
    const auto lidar   = benchmarkLidar();
    const auto mission = benchmarkMission(6000);

    // Drone starts in the open area near entrance (voxel 14,14,3 = 140cm,140cm,30cm)
    auto gps = std::make_unique<MockGPS>(
        Position3D{140.0*x_extent[cm], 140.0*y_extent[cm], 30.0*z_extent[cm]},
        Orientation{0.0*horizontal_angle[deg], 0.0*altitude_angle[deg]},
        mission.gps_resolution);

    auto movement = std::make_unique<MockMovement>(*gps, *hidden_map, drone);
    auto lidar_impl = std::make_unique<MockLidar>(lidar, *hidden_map, *gps);
    auto algo = std::make_unique<MappingAlgorithmImpl>(mission, lidar, drone, *output_map);
    auto ctrl = std::make_unique<DroneControlImpl>(
        drone, mission, *lidar_impl, *gps, *movement, *output_map, *algo);
    auto mc = std::make_unique<MissionControlImpl>(mission, *ctrl);

    const std::filesystem::path out =
        std::filesystem::temp_directory_path() / "benchmark_large_drone_output.npy";

    const auto result = mc->runMission();
    EXPECT_NE(result.status, types::MissionRunStatus::Error);
    EXPECT_GE(result.steps, 1u);
    output_map->save(out);
    EXPECT_TRUE(std::filesystem::exists(out));

    auto* out_ptr = output_map.get();
    const auto scores = MapsComparison::compare(*hidden_map, {out_ptr});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_GE(scores[0], 0.0);
    EXPECT_LE(scores[0], 100.0);

    std::filesystem::remove(out);
}

// ---- Simulation on benchmark map with small drone (fits 1×1 room) ----
TEST(Integration, BenchmarkMap_SmallDrone_RunsWithoutError) {
    const auto [npy, cfg] = loadBenchmarkMap();
    auto hidden_map = std::make_unique<Map3DImpl>(npy, cfg);
    auto output_map = std::make_unique<Map3DImpl>(cfg);

    const auto drone   = benchmarkDroneSmall();
    const auto lidar   = benchmarkLidar();
    const auto mission = benchmarkMission(6000);

    auto gps = std::make_unique<MockGPS>(
        Position3D{140.0*x_extent[cm], 140.0*y_extent[cm], 30.0*z_extent[cm]},
        Orientation{0.0*horizontal_angle[deg], 0.0*altitude_angle[deg]},
        mission.gps_resolution);

    auto movement = std::make_unique<MockMovement>(*gps, *hidden_map, drone);
    auto lidar_impl = std::make_unique<MockLidar>(lidar, *hidden_map, *gps);
    auto algo = std::make_unique<MappingAlgorithmImpl>(mission, lidar, drone, *output_map);
    auto ctrl = std::make_unique<DroneControlImpl>(
        drone, mission, *lidar_impl, *gps, *movement, *output_map, *algo);
    auto mc = std::make_unique<MissionControlImpl>(mission, *ctrl);

    const auto result = mc->runMission();
    EXPECT_NE(result.status, types::MissionRunStatus::Error);
    EXPECT_GE(result.steps, 1u);
}

// ---- Identical benchmark maps score 100 ----
TEST(Integration, BenchmarkMap_IdenticalMapsScore100) {
    const auto [npy, cfg] = loadBenchmarkMap();
    auto map1 = std::make_unique<Map3DImpl>(npy, cfg);
    auto map2 = std::make_unique<Map3DImpl>(npy, cfg);

    auto* ptr = map2.get();
    const auto scores = MapsComparison::compare(*map1, {ptr});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_NEAR(scores[0], 100.0, 1e-6);
}

// ---- Benchmark map vs empty output scores lower than identical ----
TEST(Integration, BenchmarkMap_EmptyOutputScoresLessThanIdentical) {
    const auto [npy, cfg] = loadBenchmarkMap();
    auto ground_truth = std::make_unique<Map3DImpl>(npy, cfg);
    auto empty_output = std::make_unique<Map3DImpl>(cfg);   // all Unmapped

    auto* ptr = empty_output.get();
    const auto scores = MapsComparison::compare(*ground_truth, {ptr});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_LT(scores[0], 100.0);
    EXPECT_GE(scores[0], 0.0);
}

// ---- SimulationRunImpl end-to-end on benchmark map ----
TEST(Integration, BenchmarkMap_SimulationRunImplEndToEnd) {
    const auto [npy, cfg] = loadBenchmarkMap();
    const auto drone   = benchmarkDroneLarge();
    const auto lidar   = benchmarkLidar();
    const auto mission = benchmarkMission(4000);

    auto hidden_map = std::make_unique<Map3DImpl>(npy, cfg);
    auto output_map = std::make_unique<Map3DImpl>(cfg);
    auto gps = std::make_unique<MockGPS>(
        Position3D{140.0*x_extent[cm], 140.0*y_extent[cm], 30.0*z_extent[cm]},
        Orientation{0.0*horizontal_angle[deg], 0.0*altitude_angle[deg]},
        mission.gps_resolution);
    auto movement   = std::make_unique<MockMovement>(*gps, *hidden_map, drone);
    auto lidar_impl = std::make_unique<MockLidar>(lidar, *hidden_map, *gps);
    auto algo       = std::make_unique<MappingAlgorithmImpl>(mission, lidar, drone, *output_map);
    auto ctrl       = std::make_unique<DroneControlImpl>(
        drone, mission, *lidar_impl, *gps, *movement, *output_map, *algo);
    auto mc         = std::make_unique<MissionControlImpl>(mission, *ctrl);

    const std::filesystem::path out =
        std::filesystem::temp_directory_path() / "benchmark_e2e_output.npy";

    types::SimulationConfigData sim_cfg;
    SimulationRunImpl run{
        sim_cfg, mission, drone, lidar,
        types::ResolutionRequestStatus::Accepted,
        std::move(hidden_map), std::move(output_map),
        std::move(gps), std::move(movement), std::move(lidar_impl),
        std::move(algo), std::move(ctrl), std::move(mc), out};

    const auto result = run.run();
    ASSERT_FALSE(result.mission_results.empty());
    EXPECT_NE(result.mission_results[0].status, types::MissionRunStatus::Error);
    EXPECT_GE(result.mission_score, 0.0);
    EXPECT_LE(result.mission_score, 100.0);
    EXPECT_TRUE(std::filesystem::exists(out));

    std::filesystem::remove(out);
}

// ---- Lidar scan populates the output map (non-trivial mapping) ----
TEST(Integration, ScanPopulatesOutputMap) {
    // Place a wall in the ground-truth map and run a few steps.
    // The output map should have some non-Unmapped voxels.
    auto cfg = makeMapConfig();
    auto gt = std::make_unique<Map3DImpl>(cfg);
    // Wall directly in front of the drone at x=80
    for (int y = 0; y <= 100; y += 10)
    for (int z = 0; z <= 100; z += 10) {
        gt->set(Position3D{80.0*x_extent[cm],
                           static_cast<double>(y)*y_extent[cm],
                           static_cast<double>(z)*z_extent[cm]},
                types::VoxelOccupancy::Occupied);
    }
    auto output = std::make_unique<Map3DImpl>(cfg);
    auto gps = std::make_unique<MockGPS>(
        Position3D{50.0*x_extent[cm], 50.0*y_extent[cm], 50.0*z_extent[cm]},
        Orientation{0.0*horizontal_angle[deg], 0.0*altitude_angle[deg]});
    auto movement = std::make_unique<MockMovement>(*gps, *gt, makeDrone());
    auto lidar = std::make_unique<MockLidar>(makeLidar(), *gt, *gps);
    auto algo = std::make_unique<MappingAlgorithmImpl>(makeMission(50), makeLidar(), makeDrone(), *output);
    auto ctrl = std::make_unique<DroneControlImpl>(
        makeDrone(), makeMission(50), *lidar, *gps, *movement, *output, *algo);
    auto mc = std::make_unique<MissionControlImpl>(makeMission(50), *ctrl);

    mc->runMission();

    // Count non-Unmapped voxels in output
    int mapped_count = 0;
    for (int x = 0; x <= 100; x += 10)
    for (int y = 0; y <= 100; y += 10)
    for (int z = 0; z <= 100; z += 10) {
        const auto v = output->atVoxel(
            Position3D{static_cast<double>(x)*x_extent[cm],
                       static_cast<double>(y)*y_extent[cm],
                       static_cast<double>(z)*z_extent[cm]});
        if (v != types::VoxelOccupancy::Unmapped && v != types::VoxelOccupancy::OutOfBounds)
            ++mapped_count;
    }
    EXPECT_GT(mapped_count, 0) << "Output map should have mapped voxels after a scan";
}

// ---- Lidar detects wall and marks it Occupied in output map ----
TEST(Integration, LidarDetectedWallMarkedOccupied) {
    // GT: single occupied voxel directly ahead.
    auto cfg = makeMapConfig();
    auto gt = std::make_unique<Map3DImpl>(cfg);
    gt->set(Position3D{90.0*x_extent[cm], 50.0*y_extent[cm], 50.0*z_extent[cm]},
            types::VoxelOccupancy::Occupied);

    auto output = std::make_unique<Map3DImpl>(cfg);
    auto gps = std::make_unique<MockGPS>(
        Position3D{50.0*x_extent[cm], 50.0*y_extent[cm], 50.0*z_extent[cm]},
        Orientation{0.0*horizontal_angle[deg], 0.0*altitude_angle[deg]});
    auto movement = std::make_unique<MockMovement>(*gps, *gt, makeDrone());
    auto lidar    = std::make_unique<MockLidar>(makeLidar(), *gt, *gps);
    auto algo     = std::make_unique<MappingAlgorithmImpl>(makeMission(100), makeLidar(), makeDrone(), *output);
    auto ctrl     = std::make_unique<DroneControlImpl>(
        makeDrone(), makeMission(100), *lidar, *gps, *movement, *output, *algo);
    auto mc = std::make_unique<MissionControlImpl>(makeMission(100), *ctrl);

    mc->runMission();

    // The wall at x=90 should appear as Occupied in the output map
    const auto v = output->atVoxel(
        Position3D{90.0*x_extent[cm], 50.0*y_extent[cm], 50.0*z_extent[cm]});
    EXPECT_EQ(v, types::VoxelOccupancy::Occupied)
        << "Detected wall should be marked Occupied in output map";
}

// ---- MissionControl step count is within [1, max_steps] ----
TEST(Integration, MissionStepCountWithinBounds) {
    auto cfg = makeMapConfig();
    const std::size_t max_steps = 75;
    auto gt = std::make_unique<Map3DImpl>(cfg);
    auto output = std::make_unique<Map3DImpl>(cfg);
    auto gps = std::make_unique<MockGPS>(
        Position3D{50.0*x_extent[cm], 50.0*y_extent[cm], 50.0*z_extent[cm]},
        Orientation{0.0*horizontal_angle[deg], 0.0*altitude_angle[deg]});
    auto movement = std::make_unique<MockMovement>(*gps, *gt, makeDrone());
    auto lidar    = std::make_unique<MockLidar>(makeLidar(), *gt, *gps);
    auto algo     = std::make_unique<MappingAlgorithmImpl>(makeMission(max_steps), makeLidar(), makeDrone(), *output);
    auto ctrl     = std::make_unique<DroneControlImpl>(
        makeDrone(), makeMission(max_steps), *lidar, *gps, *movement, *output, *algo);
    auto mc = std::make_unique<MissionControlImpl>(makeMission(max_steps), *ctrl);

    const auto result = mc->runMission();
    EXPECT_GE(result.steps, 1u);
    EXPECT_LE(result.steps, max_steps);
}
