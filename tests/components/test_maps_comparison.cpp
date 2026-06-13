#include <gtest/gtest.h>

#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MapsComparison.h>

#include <TinyNPY.h>

using namespace drone_mapper;

namespace {

types::MapConfig makeConfig(double res = 10.0) {
    types::MapConfig cfg;
    cfg.resolution = res * cm;
    cfg.offset = {};
    cfg.boundaries.min_x      = 0.0 * x_extent[cm];
    cfg.boundaries.max_x      = 100.0 * x_extent[cm];
    cfg.boundaries.min_y      = 0.0 * y_extent[cm];
    cfg.boundaries.max_y      = 100.0 * y_extent[cm];
    cfg.boundaries.min_height = 0.0 * z_extent[cm];
    cfg.boundaries.max_height = 100.0 * z_extent[cm];
    return cfg;
}

} // namespace

TEST(MapsComparison, IdenticalEmptyMaps) {
    Map3DImpl map1{makeConfig()};
    Map3DImpl map2{makeConfig()};
    const auto scores = MapsComparison::compare(map1, {&map2});
    ASSERT_EQ(scores.size(), 1u);
    // No GT occupied, no drone mapped → Score_A=50, Score_B=0, Score_C=0
    EXPECT_NEAR(scores[0], 50.0, 1e-6);
}

TEST(MapsComparison, PerfectMapping) {
    Map3DImpl gt{makeConfig()};
    Map3DImpl dm{makeConfig()};
    const Position3D p{50.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]};
    gt.set(p, types::VoxelOccupancy::Occupied);
    dm.set(p, types::VoxelOccupancy::Occupied);
    const auto scores = MapsComparison::compare(gt, {&dm});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_GE(scores[0], 50.0);
}

TEST(MapsComparison, MissedOccupied) {
    Map3DImpl gt{makeConfig()};
    Map3DImpl dm{makeConfig()};
    const Position3D p{50.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]};
    gt.set(p, types::VoxelOccupancy::Occupied);
    const auto scores = MapsComparison::compare(gt, {&dm});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_NEAR(scores[0], 0.0, 1e-6);
}

TEST(MapsComparison, MultipleTargets) {
    Map3DImpl gt{makeConfig()};
    Map3DImpl dm1{makeConfig()};
    Map3DImpl dm2{makeConfig()};
    const Position3D p{50.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]};
    gt.set(p, types::VoxelOccupancy::Occupied);
    dm1.set(p, types::VoxelOccupancy::Occupied);
    // dm2 misses it
    const auto scores = MapsComparison::compare(gt, {&dm1, &dm2});
    ASSERT_EQ(scores.size(), 2u);
    EXPECT_GE(scores[0], 50.0);  // found it
    EXPECT_NEAR(scores[1], 0.0, 1e-6); // missed it
}

TEST(MapsComparison, IdenticalMapsReturn100) {
    Map3DImpl gt{makeConfig()};
    for (int x = 0; x <= 100; x += 10)
    for (int y = 0; y <= 100; y += 10)
    for (int z = 0; z <= 100; z += 10) {
        const Position3D p{static_cast<double>(x) * x_extent[cm],
                           static_cast<double>(y) * y_extent[cm],
                           static_cast<double>(z) * z_extent[cm]};
        gt.set(p, types::VoxelOccupancy::Occupied);
    }
    const auto scores = MapsComparison::compare(gt, {&gt});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_NEAR(scores[0], 100.0, 1e-3);
}
