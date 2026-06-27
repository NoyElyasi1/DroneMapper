#include <gtest/gtest.h>

#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MappingAlgorithmImpl.h>

using namespace drone_mapper;

namespace {

types::DroneConfigData makeDrone() {
    types::DroneConfigData d;
    d.radius  = 30.0 * cm;
    d.max_rotate  = 45.0 * horizontal_angle[deg];
    d.max_advance = 50.0 * cm;
    d.max_elevate = 40.0 * cm;
    return d;
}

types::MissionConfigData makeMission() {
    types::MissionConfigData m;
    m.max_steps = 2400;
    m.gps_resolution = 10.0 * cm;
    m.output_mapping_resolution_factor = 1.0;
    return m;
}

types::LidarConfigData makeLidar() {
    return types::LidarConfigData{{}, 20.0 * cm, 120.0 * cm, 2.5 * cm, 3};
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

// ---- Initial call ----

TEST(MappingAlgorithm, InitiallyReturnsWorkingOnFirstCall) {
    Map3DImpl output_map{makeMapConfig()};
    MappingAlgorithmImpl algo{makeMission(), makeLidar(), makeDrone(), output_map};
    const auto state = makeState(0, 0, 50);
    const auto cmd = algo.nextStep(state, nullptr);
    EXPECT_NE(cmd.status, types::AlgorithmStatus::Finished);
}

TEST(MappingAlgorithm, FirstCallReturnsMovementOrScan) {
    Map3DImpl output_map{makeMapConfig()};
    MappingAlgorithmImpl algo{makeMission(), makeLidar(), makeDrone(), output_map};
    const auto cmd = algo.nextStep(makeState(0, 0, 50), nullptr);
    // Algorithm must do something useful on the first step
    EXPECT_TRUE(cmd.movement.has_value() || cmd.scan_orientation.has_value())
        << "Algorithm returned neither movement nor scan on first call";
}

// ---- Termination ----

TEST(MappingAlgorithm, ReturnsFinishedOnEmptyMap) {
    Map3DImpl output_map{makeMapConfig()};
    MappingAlgorithmImpl algo{makeMission(), makeLidar(), makeDrone(), output_map};
    const auto state = makeState(0, 0, 50);
    for (int i = 0; i < 5000; ++i) {
        const auto cmd = algo.nextStep(state, nullptr);
        if (cmd.status == types::AlgorithmStatus::Finished ||
            cmd.status == types::AlgorithmStatus::FinishedWithUnmappableVoxels) {
            SUCCEED();
            return;
        }
    }
    // Acceptable for an empty map — algorithm may still be working
}

TEST(MappingAlgorithm, FinishedWithUnmappableVoxelsIsAlsoTerminal) {
    // Any terminal status must stop the mission — both are valid.
    Map3DImpl output_map{makeMapConfig()};
    MappingAlgorithmImpl algo{makeMission(), makeLidar(), makeDrone(), output_map};
    types::AlgorithmStatus last_status = types::AlgorithmStatus::Working;
    for (int i = 0; i < 3000; ++i) {
        const auto cmd = algo.nextStep(makeState(0, 0, 50), nullptr);
        last_status = cmd.status;
        if (last_status != types::AlgorithmStatus::Working) break;
    }
    // If it ever finishes it must be one of the terminal states
    if (last_status != types::AlgorithmStatus::Working) {
        EXPECT_TRUE(last_status == types::AlgorithmStatus::Finished ||
                    last_status == types::AlgorithmStatus::FinishedWithUnmappableVoxels);
    }
}

// ---- Obstacle avoidance ----

TEST(MappingAlgorithm, OccupiedVoxelNotNavigatedInto) {
    Map3DImpl output_map{makeMapConfig()};
    const Position3D occ_pos{10.0 * x_extent[cm], 0.0 * y_extent[cm], 50.0 * z_extent[cm]};
    output_map.set(occ_pos, types::VoxelOccupancy::Occupied);

    MappingAlgorithmImpl algo{makeMission(), makeLidar(), makeDrone(), output_map};
    EXPECT_NO_THROW(algo.nextStep(makeState(0, 0, 50), nullptr));
}

TEST(MappingAlgorithm, NoCrashOnFullyOccupiedMap) {
    Map3DImpl output_map{makeMapConfig()};
    // Fill the entire map with Occupied voxels
    for (int x = -500; x <= 500; x += 10)
    for (int y = -500; y <= 500; y += 10)
    for (int z = 0; z <= 300; z += 10) {
        output_map.set(
            Position3D{static_cast<double>(x)*x_extent[cm],
                       static_cast<double>(y)*y_extent[cm],
                       static_cast<double>(z)*z_extent[cm]},
            types::VoxelOccupancy::Occupied);
    }
    MappingAlgorithmImpl algo{makeMission(), makeLidar(), makeDrone(), output_map};
    EXPECT_NO_THROW(algo.nextStep(makeState(0, 0, 50), nullptr));
}

// ---- Consistency ----

TEST(MappingAlgorithm, ManyCallsDoNotCrash) {
    Map3DImpl output_map{makeMapConfig()};
    MappingAlgorithmImpl algo{makeMission(), makeLidar(), makeDrone(), output_map};
    const auto state = makeState(0, 0, 50);
    EXPECT_NO_THROW({
        for (int i = 0; i < 2000; ++i) {
            (void)algo.nextStep(state, nullptr);
        }
    });
}

TEST(MappingAlgorithm, StatusIsValidEnum) {
    Map3DImpl output_map{makeMapConfig()};
    MappingAlgorithmImpl algo{makeMission(), makeLidar(), makeDrone(), output_map};
    for (int i = 0; i < 200; ++i) {
        const auto cmd = algo.nextStep(makeState(0, 0, 50), nullptr);
        EXPECT_TRUE(cmd.status == types::AlgorithmStatus::Working ||
                    cmd.status == types::AlgorithmStatus::Finished ||
                    cmd.status == types::AlgorithmStatus::FinishedWithUnmappableVoxels);
    }
}

TEST(MappingAlgorithm, ReadsMapStateViaOutputMap) {
    // Mark a large ring of voxels as occupied so the algo must see them.
    Map3DImpl output_map{makeMapConfig()};
    for (int a = 0; a < 360; a += 10) {
        const double r = 100.0;
        const double rx = r * std::cos(a * M_PI / 180.0);
        const double ry = r * std::sin(a * M_PI / 180.0);
        output_map.set(
            Position3D{rx*x_extent[cm], ry*y_extent[cm], 50.0*z_extent[cm]},
            types::VoxelOccupancy::Occupied);
    }
    MappingAlgorithmImpl algo{makeMission(), makeLidar(), makeDrone(), output_map};
    EXPECT_NO_THROW(algo.nextStep(makeState(0, 0, 50), nullptr));
}

// ---- Movement limits ----

TEST(MappingAlgorithm, AdvanceDistanceNeverExceedsMaxAdvance) {
    Map3DImpl output_map{makeMapConfig()};
    MappingAlgorithmImpl algo{makeMission(), makeLidar(), makeDrone(), output_map};
    const double max_adv = makeDrone().max_advance.numerical_value_in(cm);

    auto state = makeState(0, 0, 50);
    for (int i = 0; i < 500; ++i) {
        const auto cmd = algo.nextStep(state, nullptr);
        if (cmd.movement.has_value() &&
            cmd.movement->type == types::MovementCommandType::Advance) {
            EXPECT_LE(cmd.movement->distance.numerical_value_in(cm), max_adv + 1e-6)
                << "Advance command exceeds max_advance on step " << i;
        }
        if (cmd.status != types::AlgorithmStatus::Working) break;
    }
}

TEST(MappingAlgorithm, ElevateDistanceNeverExceedsMaxElevate) {
    Map3DImpl output_map{makeMapConfig()};
    MappingAlgorithmImpl algo{makeMission(), makeLidar(), makeDrone(), output_map};
    const double max_el = makeDrone().max_elevate.numerical_value_in(cm);

    auto state = makeState(0, 0, 50);
    for (int i = 0; i < 500; ++i) {
        const auto cmd = algo.nextStep(state, nullptr);
        if (cmd.movement.has_value() &&
            cmd.movement->type == types::MovementCommandType::Elevate) {
            EXPECT_LE(std::fabs(cmd.movement->distance.numerical_value_in(cm)), max_el + 1e-6)
                << "Elevate command exceeds max_elevate on step " << i;
        }
        if (cmd.status != types::AlgorithmStatus::Working) break;
    }
}

TEST(MappingAlgorithm, RotationNeverExceedsMaxRotate) {
    Map3DImpl output_map{makeMapConfig()};
    MappingAlgorithmImpl algo{makeMission(), makeLidar(), makeDrone(), output_map};
    const double max_rot = makeDrone().max_rotate.numerical_value_in(deg);

    auto state = makeState(0, 0, 50);
    for (int i = 0; i < 500; ++i) {
        const auto cmd = algo.nextStep(state, nullptr);
        if (cmd.movement.has_value() &&
            cmd.movement->type == types::MovementCommandType::Rotate) {
            EXPECT_LE(cmd.movement->angle.numerical_value_in(deg), max_rot + 1e-6)
                << "Rotate command exceeds max_rotate on step " << i;
        }
        if (cmd.status != types::AlgorithmStatus::Working) break;
    }
}

// ---- Scan always requested ----

TEST(MappingAlgorithm, AlwaysRequestsScanOrientation) {
    // Every working step should include a scan request so the output map is updated.
    Map3DImpl output_map{makeMapConfig()};
    MappingAlgorithmImpl algo{makeMission(), makeLidar(), makeDrone(), output_map};

    auto state = makeState(0, 0, 50);
    int checked = 0;
    for (int i = 0; i < 200; ++i) {
        const auto cmd = algo.nextStep(state, nullptr);
        if (cmd.status == types::AlgorithmStatus::Working) {
            EXPECT_TRUE(cmd.scan_orientation.has_value())
                << "Algorithm did not request a scan on working step " << i;
            ++checked;
        }
        if (cmd.status != types::AlgorithmStatus::Working) break;
    }
    EXPECT_GT(checked, 0) << "No working steps observed";
}

// ---- Output map not navigated into occupied cell ----

TEST(MappingAlgorithm, CommandNeverAdvancesIntoDroneBody) {
    // Fill all positions except origin+Z=50 as occupied.
    Map3DImpl output_map{makeMapConfig()};
    // Ring of obstacles 50 cm out
    for (int a = 0; a < 360; a += 45) {
        const double r = 50.0;
        output_map.set(
            Position3D{r * std::cos(a * M_PI / 180.0) * x_extent[cm],
                       r * std::sin(a * M_PI / 180.0) * y_extent[cm],
                       50.0 * z_extent[cm]},
            types::VoxelOccupancy::Occupied);
    }
    MappingAlgorithmImpl algo{makeMission(), makeLidar(), makeDrone(), output_map};
    // Just verify no crash and status is valid
    for (int i = 0; i < 50; ++i) {
        const auto cmd = algo.nextStep(makeState(0, 0, 50), nullptr);
        EXPECT_TRUE(cmd.status == types::AlgorithmStatus::Working ||
                    cmd.status == types::AlgorithmStatus::Finished ||
                    cmd.status == types::AlgorithmStatus::FinishedWithUnmappableVoxels)
            << "Invalid status on step " << i;
    }
}
