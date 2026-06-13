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
#include <drone_mapper/SimulationRunFactoryImpl.h>

#include <filesystem>

using namespace drone_mapper;

namespace {

types::DroneConfigData makeDrone() {
    types::DroneConfigData d;
    d.dimensions=30.0*cm; d.max_rotate=45.0*horizontal_angle[deg];
    d.max_advance=50.0*cm; d.max_elevate=40.0*cm;
    return d;
}

types::LidarConfigData makeLidar() {
    return types::LidarConfigData{20.0*cm, 50.0*cm, 2.5*cm, 1};
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

TEST(Integration, RealAlgorithmRunsWithoutCrash) {
    auto cfg = makeMapConfig();
    auto gt = std::make_unique<Map3DImpl>(cfg);
    auto output = std::make_unique<Map3DImpl>(cfg);

    auto gps = std::make_unique<MockGPS>(
        Position3D{50.0*x_extent[cm], 50.0*y_extent[cm], 50.0*z_extent[cm]},
        Orientation{0.0*horizontal_angle[deg], 0.0*altitude_angle[deg]});

    auto movement = std::make_unique<MockMovement>(*gps, *gt, makeDrone());
    auto lidar = std::make_unique<MockLidar>(makeLidar(), *gt, *gps);
    auto algo = std::make_unique<MappingAlgorithmImpl>(makeMission(), makeDrone(), cfg);
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

TEST(Integration, MissionReachesCompletedOrMaxSteps) {
    auto cfg = makeMapConfig();
    auto gt = std::make_unique<Map3DImpl>(cfg);
    auto output = std::make_unique<Map3DImpl>(cfg);

    auto gps = std::make_unique<MockGPS>(
        Position3D{50.0*x_extent[cm], 50.0*y_extent[cm], 50.0*z_extent[cm]},
        Orientation{0.0*horizontal_angle[deg], 0.0*altitude_angle[deg]});

    auto movement = std::make_unique<MockMovement>(*gps, *gt, makeDrone());
    auto lidar = std::make_unique<MockLidar>(makeLidar(), *gt, *gps);
    auto algo = std::make_unique<MappingAlgorithmImpl>(makeMission(50), makeDrone(), cfg);
    auto ctrl = std::make_unique<DroneControlImpl>(
        makeDrone(), makeMission(50), *lidar, *gps, *movement, *output, *algo);

    const std::filesystem::path out =
        std::filesystem::temp_directory_path() / "integration_real_output2.npy";
    auto mc = std::make_unique<MissionControlImpl>(makeMission(50), *ctrl);

    const auto result = mc->runMission();
    EXPECT_NE(result.status, types::MissionRunStatus::Error);
    EXPECT_GE(result.steps, 1u);

    std::filesystem::remove(out);
}
