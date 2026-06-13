#include <gtest/gtest.h>

#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockLidar.h>

using namespace drone_mapper;

namespace {

types::LidarConfigData makeLidar() {
    return types::LidarConfigData{20.0 * cm, 120.0 * cm, 2.5 * cm, 3};
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

TEST(MockLidar, ScanEmptyMapReturnsNoRealHits) {
    Map3DImpl map{makeBoundsConfig()};
    MockGPS gps{Position3D{50.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]},
                Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]}};
    MockLidar lidar{makeLidar(), map, gps};

    const auto result = lidar.scan(Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]});
    EXPECT_FALSE(result.empty());
    for (const auto& hit : result) {
        // No real hit: distance should be max double
        EXPECT_GT(hit.distance.numerical_value_in(cm), 1e100);
    }
}

TEST(MockLidar, ScanDetectsWall) {
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
    EXPECT_TRUE(found_hit);
}

TEST(MockLidar, ZeroFovCirclesReturnsEmpty) {
    Map3DImpl map{makeBoundsConfig()};
    MockGPS gps{Position3D{}, Orientation{}};
    MockLidar lidar{types::LidarConfigData{20.0*cm, 120.0*cm, 2.5*cm, 0}, map, gps};
    const auto result = lidar.scan(Orientation{});
    EXPECT_TRUE(result.empty());
}
