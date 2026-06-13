#include <gtest/gtest.h>

#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MapsComparison.h>

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

// ---- Empty maps ----

TEST(MapsComparison, IdenticalEmptyMaps) {
    Map3DImpl map1{makeConfig()};
    Map3DImpl map2{makeConfig()};
    const auto scores = MapsComparison::compare(map1, {&map2});
    ASSERT_EQ(scores.size(), 1u);
    // No GT occupied, no drone mapped → Score_A=50, Score_B=0, Score_C=0
    EXPECT_NEAR(scores[0], 50.0, 1e-6);
}

TEST(MapsComparison, EmptyDroneAgainstOccupiedGTScoresZero) {
    Map3DImpl gt{makeConfig()};
    Map3DImpl dm{makeConfig()};
    const Position3D p{50.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]};
    gt.set(p, types::VoxelOccupancy::Occupied);
    // dm leaves it unmapped → score should be 0 (missed all occupied)
    const auto scores = MapsComparison::compare(gt, {&dm});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_NEAR(scores[0], 0.0, 1e-6);
}

// ---- Perfect mapping ----

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

TEST(MapsComparison, IdenticalFullMapsReturn100) {
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

// ---- Missed voxels ----

TEST(MapsComparison, MissedOccupied) {
    Map3DImpl gt{makeConfig()};
    Map3DImpl dm{makeConfig()};
    const Position3D p{50.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]};
    gt.set(p, types::VoxelOccupancy::Occupied);
    const auto scores = MapsComparison::compare(gt, {&dm});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_NEAR(scores[0], 0.0, 1e-6);
}

// ---- Score range ----

TEST(MapsComparison, ScoreAlwaysInRange0To100) {
    Map3DImpl gt{makeConfig()};
    Map3DImpl dm{makeConfig()};
    for (int x = 0; x <= 50; x += 10) {
        gt.set(Position3D{static_cast<double>(x)*x_extent[cm],
                          50.0*y_extent[cm], 50.0*z_extent[cm]},
               types::VoxelOccupancy::Occupied);
    }
    for (int x = 40; x <= 90; x += 10) {
        dm.set(Position3D{static_cast<double>(x)*x_extent[cm],
                          50.0*y_extent[cm], 50.0*z_extent[cm]},
               types::VoxelOccupancy::Occupied);
    }
    const auto scores = MapsComparison::compare(gt, {&dm});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_GE(scores[0], 0.0);
    EXPECT_LE(scores[0], 100.0);
}

// ---- Multiple targets ----

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
    EXPECT_GE(scores[0], 50.0);        // found it
    EXPECT_NEAR(scores[1], 0.0, 1e-6); // missed it
}

TEST(MapsComparison, MultipleTargetsReturnOneScoreEach) {
    Map3DImpl gt{makeConfig()};
    Map3DImpl dm1{makeConfig()};
    Map3DImpl dm2{makeConfig()};
    Map3DImpl dm3{makeConfig()};
    const auto scores = MapsComparison::compare(gt, {&dm1, &dm2, &dm3});
    EXPECT_EQ(scores.size(), 3u);
}

// ---- False positives ----

TEST(MapsComparison, FalsePositivesReduceScore) {
    // GT has one occupied voxel; drone correctly maps it but also marks extra voxels as occupied.
    // Correct + false positives should score lower than a perfect mapping.
    Map3DImpl gt{makeConfig()};
    Map3DImpl perfect{makeConfig()};
    Map3DImpl with_fp{makeConfig()};

    const Position3D p{50.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]};
    gt.set(p, types::VoxelOccupancy::Occupied);
    perfect.set(p, types::VoxelOccupancy::Occupied);
    with_fp.set(p, types::VoxelOccupancy::Occupied);

    // Add false positives
    for (int x = 0; x <= 30; x += 10) {
        with_fp.set(Position3D{static_cast<double>(x)*x_extent[cm],
                               50.0*y_extent[cm], 50.0*z_extent[cm]},
                    types::VoxelOccupancy::Occupied);
    }

    const auto scores_perfect = MapsComparison::compare(gt, {&perfect});
    const auto scores_fp      = MapsComparison::compare(gt, {&with_fp});

    ASSERT_EQ(scores_perfect.size(), 1u);
    ASSERT_EQ(scores_fp.size(), 1u);
    EXPECT_GE(scores_perfect[0], scores_fp[0])
        << "False positives should not increase score";
}
