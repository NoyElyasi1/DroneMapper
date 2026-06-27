#include <gtest/gtest.h>

#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockLidar.h>

using namespace drone_mapper;

namespace {

types::LidarConfigData makeLidar(std::size_t circles = 3) {
    return types::LidarConfigData{{}, 20.0 * cm, 120.0 * cm, 2.5 * cm, circles};
}

types::MapConfig makeBoundsConfig() {
    types::MapConfig cfg;
    cfg.resolution = 10.0 * cm;
    cfg.boundaries.min_x      = 0.0 * x_extent[cm];
    cfg.boundaries.max_x      = 500.0 * x_extent[cm];
    cfg.boundaries.min_y      = 0.0 * y_extent[cm];
    cfg.boundaries.max_y      = 500.0 * y_extent[cm];
    cfg.boundaries.min_height = 0.0 * z_extent[cm];
    cfg.boundaries.max_height = 300.0 * z_extent[cm];
    return cfg;
}

} // namespace

// ---- Empty map ----

TEST(MockLidar, ScanEmptyMapReturnsMaxDistance) {
    Map3DImpl map{makeBoundsConfig()};
    MockGPS gps{Position3D{50.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]},
                Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]}};
    MockLidar lidar{makeLidar(), map, gps};

    const auto result = lidar.scan(Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]});
    EXPECT_FALSE(result.empty());
    for (const auto& hit : result) {
        // No occupied voxel → should return max double
        EXPECT_GT(hit.distance.numerical_value_in(cm), 1e100);
    }
}

// ---- Wall detection ----

TEST(MockLidar, ScanDetectsWallAhead) {
    Map3DImpl map{makeBoundsConfig()};
    map.set(Position3D{150.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]},
            types::VoxelOccupancy::Occupied);

    MockGPS gps{Position3D{50.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]},
                Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]}};
    MockLidar lidar{makeLidar(), map, gps};

    const auto result = lidar.scan(Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]});
    ASSERT_FALSE(result.empty());
    bool found_hit = false;
    for (const auto& hit : result) {
        if (hit.distance.numerical_value_in(cm) < 110.0) {
            found_hit = true;
        }
    }
    EXPECT_TRUE(found_hit) << "Expected to detect occupied voxel at x=150";
}

TEST(MockLidar, DistanceBoundedByZmax) {
    Map3DImpl map{makeBoundsConfig()};
    MockGPS gps{Position3D{50.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]},
                Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]}};
    // Place a wall just beyond z_max = 120 cm
    map.set(Position3D{250.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]},
            types::VoxelOccupancy::Occupied);

    MockLidar lidar{makeLidar(), map, gps};
    const auto result = lidar.scan(Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]});
    for (const auto& hit : result) {
        // A hit at distance > z_max should not be detected
        const double d = hit.distance.numerical_value_in(cm);
        EXPECT_TRUE(d <= 120.0 || d > 1e100)
            << "Unexpected distance " << d << " exceeding z_max";
    }
}

// ---- Beam count ----

TEST(MockLidar, ZeroFovCirclesReturnsEmpty) {
    Map3DImpl map{makeBoundsConfig()};
    MockGPS gps{Position3D{}, Orientation{}};
    MockLidar lidar{types::LidarConfigData{{}, 20.0*cm, 120.0*cm, 2.5*cm, 0}, map, gps};
    const auto result = lidar.scan(Orientation{});
    EXPECT_TRUE(result.empty());
}

TEST(MockLidar, OneFovCircleReturnsOneCenterBeam) {
    Map3DImpl map{makeBoundsConfig()};
    MockGPS gps{Position3D{50.0*x_extent[cm], 50.0*y_extent[cm], 50.0*z_extent[cm]}, Orientation{}};
    MockLidar lidar{types::LidarConfigData{{}, 20.0*cm, 120.0*cm, 2.5*cm, 1}, map, gps};
    const auto result = lidar.scan(Orientation{});
    EXPECT_EQ(result.size(), 1u);
}

TEST(MockLidar, TwoCirclesHasCorrectBeamCount) {
    // circle 0: 1 beam; circle 1: 4 beams → total 5
    Map3DImpl map{makeBoundsConfig()};
    MockGPS gps{Position3D{50.0*x_extent[cm], 50.0*y_extent[cm], 50.0*z_extent[cm]}, Orientation{}};
    MockLidar lidar{types::LidarConfigData{{}, 20.0*cm, 120.0*cm, 2.5*cm, 2}, map, gps};
    const auto result = lidar.scan(Orientation{});
    EXPECT_EQ(result.size(), 5u);  // 1 + 4
}

TEST(MockLidar, ThreeCirclesHasCorrectBeamCount) {
    // circle 0: 1; circle 1: 4; circle 2: 16 → total 21
    Map3DImpl map{makeBoundsConfig()};
    MockGPS gps{Position3D{50.0*x_extent[cm], 50.0*y_extent[cm], 50.0*z_extent[cm]}, Orientation{}};
    MockLidar lidar{types::LidarConfigData{{}, 20.0*cm, 120.0*cm, 2.5*cm, 3}, map, gps};
    const auto result = lidar.scan(Orientation{});
    EXPECT_EQ(result.size(), 21u);  // 1 + 4 + 16
}

// ---- Orientation affects detection ----

TEST(MockLidar, WallBehindNotDetectedFromFront) {
    Map3DImpl map{makeBoundsConfig()};
    // Wall at x=400 (behind the drone scanning eastward from x=250)
    map.set(Position3D{50.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]},
            types::VoxelOccupancy::Occupied);

    MockGPS gps{Position3D{250.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]},
                Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]}};
    MockLidar lidar{makeLidar(1), map, gps};  // center beam only

    // Scan forward (east = 0°): the wall is at x=50 which is behind (west), not ahead
    const auto result = lidar.scan(Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]});
    ASSERT_FALSE(result.empty());
    // Center beam should not detect the wall behind
    EXPECT_GT(result[0].distance.numerical_value_in(cm), 1e100);
}

TEST(MockLidar, HeadingAffectsAbsoluteBeamDirection) {
    Map3DImpl map{makeBoundsConfig()};
    // Wall north of the drone (in +y direction), within z_max range (120 cm)
    map.set(Position3D{250.0 * x_extent[cm], 150.0 * y_extent[cm], 50.0 * z_extent[cm]},
            types::VoxelOccupancy::Occupied);

    // Drone facing north (90°)
    MockGPS gps{Position3D{250.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]},
                Orientation{90.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]}};
    MockLidar lidar{makeLidar(1), map, gps};

    // Scan straight ahead (relative 0°): absolute beam = 90° (north). Wall at y=150 (100cm away).
    const auto result_forward = lidar.scan(
        Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]});
    ASSERT_FALSE(result_forward.empty());
    bool found_hit = result_forward[0].distance.numerical_value_in(cm) < 120.0;
    EXPECT_TRUE(found_hit) << "Should detect northward wall when heading north";
}

// ---- Close hits ----

TEST(MockLidar, HitBelowZminReturnsZeroDistance) {
    Map3DImpl map{makeBoundsConfig()};
    // Place a voxel very close, inside z_min (z_min = 20 cm)
    map.set(Position3D{55.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]},
            types::VoxelOccupancy::Occupied);

    MockGPS gps{Position3D{50.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]},
                Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]}};
    MockLidar lidar{makeLidar(1), map, gps};

    const auto result = lidar.scan(Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]});
    ASSERT_FALSE(result.empty());
    // The voxel is at 5 cm which is within z_min=20 cm → distance should be 0
    EXPECT_NEAR(result[0].distance.numerical_value_in(cm), 0.0, 1e-6);
}

// ---- Exact distance to obstacle ----

TEST(MockLidar, ExactDistanceToObstacleAhead) {
    // Drone at (50,50,50), obstacle at (150,50,50) = 100 cm ahead (east).
    Map3DImpl map{makeBoundsConfig()};
    map.set(Position3D{150.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]},
            types::VoxelOccupancy::Occupied);

    MockGPS gps{Position3D{50.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]},
                Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]}};
    MockLidar lidar{makeLidar(1), map, gps};

    const auto result = lidar.scan(Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]});
    ASSERT_FALSE(result.empty());
    // Distance should be close to 100 cm; tolerance = half-voxel (5 cm) + 1 step margin
    EXPECT_NEAR(result[0].distance.numerical_value_in(cm), 100.0, 6.0)
        << "Expected ~100 cm to obstacle";
}

// ---- Obstacle just within z_max is detected ----

TEST(MockLidar, ObstacleJustWithinZmaxIsDetected) {
    // z_max = 120 cm, place obstacle at 110 cm (safely within range)
    Map3DImpl map{makeBoundsConfig()};
    map.set(Position3D{160.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]},
            types::VoxelOccupancy::Occupied);

    MockGPS gps{Position3D{50.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]},
                Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]}};
    MockLidar lidar{makeLidar(1), map, gps};

    const auto result = lidar.scan(Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]});
    ASSERT_FALSE(result.empty());
    const double d = result[0].distance.numerical_value_in(cm);
    EXPECT_LT(d, 120.0) << "Obstacle within z_max should be detected (distance < z_max)";
    EXPECT_GT(d, 0.0);
}

// ---- Obstacle beyond z_max is NOT detected ----

TEST(MockLidar, ObstacleBeyondZmaxNotDetected) {
    // z_max = 120 cm; obstacle at 200 cm (beyond range)
    Map3DImpl map{makeBoundsConfig()};
    map.set(Position3D{260.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]},
            types::VoxelOccupancy::Occupied);

    MockGPS gps{Position3D{50.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]},
                Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]}};
    MockLidar lidar{makeLidar(1), map, gps};

    const auto result = lidar.scan(Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]});
    ASSERT_FALSE(result.empty());
    // Nothing within z_max → max double distance
    EXPECT_GT(result[0].distance.numerical_value_in(cm), 1e100)
        << "Obstacle beyond z_max should not be detected";
}

// ---- config() returns configured data ----

TEST(MockLidar, ConfigReturnsConfiguredData) {
    Map3DImpl map{makeBoundsConfig()};
    MockGPS gps{Position3D{}, Orientation{}};
    const auto lidar_cfg = types::LidarConfigData{{}, 15.0*cm, 90.0*cm, 3.0*cm, 2};
    MockLidar lidar{lidar_cfg, map, gps};

    const auto returned = lidar.config();
    EXPECT_NEAR(returned.z_min.numerical_value_in(cm), 15.0, 1e-6);
    EXPECT_NEAR(returned.z_max.numerical_value_in(cm), 90.0, 1e-6);
    EXPECT_NEAR(returned.d.numerical_value_in(cm), 3.0, 1e-6);
    EXPECT_EQ(returned.fov_circles, 2u);
}

// ---- 4 fov_circles beam count ----

TEST(MockLidar, FourCirclesHasCorrectBeamCount) {
    // 1 + 4 + 16 + 64 = 85
    Map3DImpl map{makeBoundsConfig()};
    MockGPS gps{Position3D{50.0*x_extent[cm], 50.0*y_extent[cm], 50.0*z_extent[cm]}, Orientation{}};
    MockLidar lidar{types::LidarConfigData{{}, 20.0*cm, 120.0*cm, 2.5*cm, 4}, map, gps};
    const auto result = lidar.scan(Orientation{});
    EXPECT_EQ(result.size(), 85u);  // 1 + 4 + 16 + 64
}

// ---- Returned hit angle matches scan orientation ----

TEST(MockLidar, HitAngleMatchesScanOrientation) {
    Map3DImpl map{makeBoundsConfig()};
    MockGPS gps{Position3D{50.0*x_extent[cm], 50.0*y_extent[cm], 50.0*z_extent[cm]},
                Orientation{0.0*horizontal_angle[deg], 0.0*altitude_angle[deg]}};
    MockLidar lidar{makeLidar(1), map, gps};

    const Orientation scan_dir{30.0*horizontal_angle[deg], 5.0*altitude_angle[deg]};
    const auto result = lidar.scan(scan_dir);
    ASSERT_FALSE(result.empty());
    // Center beam's reported angle should match the requested scan orientation (relative angles)
    EXPECT_NEAR(result[0].angle.horizontal.numerical_value_in(deg), 30.0, 1e-3);
    EXPECT_NEAR(result[0].angle.altitude.numerical_value_in(deg), 5.0, 1e-3);
}

// ---- Closest of two obstacles in same direction is detected ----

TEST(MockLidar, ClosestObstacleIsDetected) {
    Map3DImpl map{makeBoundsConfig()};
    // Near obstacle at 50 cm, far obstacle at 100 cm, same direction
    map.set(Position3D{100.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]},
            types::VoxelOccupancy::Occupied);
    map.set(Position3D{150.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]},
            types::VoxelOccupancy::Occupied);

    MockGPS gps{Position3D{50.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]},
                Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]}};
    MockLidar lidar{makeLidar(1), map, gps};

    const auto result = lidar.scan(Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]});
    ASSERT_FALSE(result.empty());
    // Should detect the nearer obstacle (~50 cm), not the far one (~100 cm)
    const double d = result[0].distance.numerical_value_in(cm);
    EXPECT_LT(d, 75.0) << "Should detect closest obstacle, not a further one";
}
