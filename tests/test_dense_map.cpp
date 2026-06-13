// ============================================================
// test_dense_map.cpp — Tests for SparseBuildingMap in dense mode.
//
// The ground-truth map in the real simulation always uses dense
// mode (initDense is called before loadFromFile).  The tests in
// test_sparse_map.cpp only exercise sparse mode, so bugs in the
// dense array path would go undetected.  This file closes that gap.
// ============================================================

#include "test_helpers.hpp"
#include "SparseBuildingMap.hpp"

#include <gtest/gtest.h>
#include <fstream>

using namespace dm;
using namespace dm::test;

// ---- Helper ----
// Build a MissionConfig for a small 10×10×10 cm space (step = 1 cm).
static MissionConfig makeTinyMission()
{
    MissionConfig mc;
    mc.minX      = 0;  mc.maxX      = 10;
    mc.minY      = 0;  mc.maxY      = 10;
    mc.minHeight = 0;  mc.maxHeight = 10;
    mc.resX = mc.resY = mc.resHeight = 0;   // step = 1 cm
    mc.computeSteps();
    return mc;
}

// ============================================================
// Dense mode — basic cell access
// ============================================================

// After initDense, every unset cell returns UnmappedNA
TEST(SparseBuildingMap_Dense, InitDense_AllCellsUnmappedNA)
{
    auto mc = makeTinyMission();
    SparseBuildingMap map;
    map.initDense(mc);

    EXPECT_EQ(map.getCell({0, 0, 0}),
              static_cast<int>(CellStatus::UnmappedNA));
    EXPECT_EQ(map.getCell({5, 5, 5}),
              static_cast<int>(CellStatus::UnmappedNA));
    EXPECT_EQ(map.getCell({10, 10, 10}),
              static_cast<int>(CellStatus::UnmappedNA));
}

// setCell / getCell round-trip in dense mode
TEST(SparseBuildingMap_Dense, SetAndGetCell_Occupied)
{
    auto mc = makeTinyMission();
    SparseBuildingMap map;
    map.initDense(mc);

    map.setCell({3, 4, 5}, static_cast<int>(CellStatus::Occupied));
    EXPECT_EQ(map.getCell({3, 4, 5}),
              static_cast<int>(CellStatus::Occupied));
}

TEST(SparseBuildingMap_Dense, SetAndGetCell_Empty)
{
    auto mc = makeTinyMission();
    SparseBuildingMap map;
    map.initDense(mc);

    map.setCell({1, 2, 3}, static_cast<int>(CellStatus::Empty));
    EXPECT_EQ(map.getCell({1, 2, 3}),
              static_cast<int>(CellStatus::Empty));
}

// Overwrite: last write wins
TEST(SparseBuildingMap_Dense, SetCell_Overwrite)
{
    auto mc = makeTinyMission();
    SparseBuildingMap map;
    map.initDense(mc);

    map.setCell({2, 2, 2}, static_cast<int>(CellStatus::Empty));
    map.setCell({2, 2, 2}, static_cast<int>(CellStatus::Occupied));
    EXPECT_EQ(map.getCell({2, 2, 2}),
              static_cast<int>(CellStatus::Occupied));
}

// hasCell returns false before write, true after
TEST(SparseBuildingMap_Dense, HasCell_Works)
{
    auto mc = makeTinyMission();
    SparseBuildingMap map;
    map.initDense(mc);

    EXPECT_FALSE(map.hasCell({7, 7, 7}));
    map.setCell ({7, 7, 7}, static_cast<int>(CellStatus::Occupied));
    EXPECT_TRUE (map.hasCell({7, 7, 7}));
}

// Out-of-bounds coordinates return UnmappedNA, never crash
TEST(SparseBuildingMap_Dense, OutOfBounds_ReturnsUnmappedNA)
{
    auto mc = makeTinyMission();
    SparseBuildingMap map;
    map.initDense(mc);

    EXPECT_EQ(map.getCell({-1,  0,  0}),
              static_cast<int>(CellStatus::UnmappedNA));
    EXPECT_EQ(map.getCell({ 0, 99,  0}),
              static_cast<int>(CellStatus::UnmappedNA));
    EXPECT_EQ(map.getCell({ 0,  0, 11}),
              static_cast<int>(CellStatus::UnmappedNA));
}

// Writing an out-of-bounds cell is silently ignored (no crash, no effect)
TEST(SparseBuildingMap_Dense, SetCell_OutOfBounds_Ignored)
{
    auto mc = makeTinyMission();
    SparseBuildingMap map;
    map.initDense(mc);

    // Should not throw or crash
    EXPECT_NO_FATAL_FAILURE(
        map.setCell({-1, -1, -1}, static_cast<int>(CellStatus::Occupied)));
    EXPECT_EQ(map.mappedCount(), 0u);
}

// ============================================================
// Dense mode — mappedCount
// ============================================================

// Starts at 0
TEST(SparseBuildingMap_Dense, MappedCount_StartsAtZero)
{
    auto mc = makeTinyMission();
    SparseBuildingMap map;
    map.initDense(mc);
    EXPECT_EQ(map.mappedCount(), 0u);
}

// Increments once per new cell, not per overwrite
TEST(SparseBuildingMap_Dense, MappedCount_IncrementsOnWrite)
{
    auto mc = makeTinyMission();
    SparseBuildingMap map;
    map.initDense(mc);

    map.setCell({0, 0, 0}, static_cast<int>(CellStatus::Occupied));
    EXPECT_EQ(map.mappedCount(), 1u);

    map.setCell({1, 1, 1}, static_cast<int>(CellStatus::Empty));
    EXPECT_EQ(map.mappedCount(), 2u);
}

TEST(SparseBuildingMap_Dense, MappedCount_OverwriteNoDoubleCount)
{
    auto mc = makeTinyMission();
    SparseBuildingMap map;
    map.initDense(mc);

    map.setCell({3, 3, 3}, static_cast<int>(CellStatus::Occupied));
    map.setCell({3, 3, 3}, static_cast<int>(CellStatus::Empty));   // overwrite
    EXPECT_EQ(map.mappedCount(), 1u);
}

// ============================================================
// Dense mode — file I/O
// ============================================================

// loadFromFile into a dense map correctly populates the dense array
TEST(SparseBuildingMap_Dense, LoadFromFile_PopulatesDenseArray)
{
    auto mc = makeTinyMission();   // 0..10, step=1

    const std::string content =
        "# test\n"
        "3,4,5,1\n"    // occupied — inside bounds
        "1,2,3,0\n"    // empty    — inside bounds
        "99,99,99,1\n" // occupied — outside dense bounds, silently ignored
        ;

    SparseBuildingMap map;
    map.initDense(mc);
    std::string errors;
    EXPECT_TRUE(map.loadFromFile(writeTempFile(content), errors));

    EXPECT_EQ(map.getCell({3, 4, 5}),
              static_cast<int>(CellStatus::Occupied));
    EXPECT_EQ(map.getCell({1, 2, 3}),
              static_cast<int>(CellStatus::Empty));

    // Out-of-bounds cell was silently dropped
    EXPECT_EQ(map.mappedCount(), 2u);
}

// saveToFile from dense mode and reload into sparse mode — roundtrip
TEST(SparseBuildingMap_Dense, SaveAndLoad_DenseToSparse_Roundtrip)
{
    auto mc = makeTinyMission();
    SparseBuildingMap dense;
    dense.initDense(mc);
    dense.setCell({1, 2, 3}, static_cast<int>(CellStatus::Occupied));
    dense.setCell({4, 5, 6}, static_cast<int>(CellStatus::Empty));

    const std::string path = writeTempFile("", ".csv");
    ASSERT_TRUE(dense.saveToFile(path));

    // Reload into a fresh sparse map
    SparseBuildingMap sparse;
    std::string errors;
    EXPECT_TRUE(sparse.loadFromFile(path, errors));
    EXPECT_TRUE(errors.empty()) << errors;

    EXPECT_EQ(sparse.getCell({1, 2, 3}),
              static_cast<int>(CellStatus::Occupied));
    EXPECT_EQ(sparse.getCell({4, 5, 6}),
              static_cast<int>(CellStatus::Empty));
    EXPECT_EQ(sparse.getCell({9, 9, 9}),
              static_cast<int>(CellStatus::UnmappedNA));
}

// ============================================================
// Dense mode — score calculation
// ============================================================

// calculateScore works when groundTruth is in dense mode
TEST(SparseBuildingMap_Dense, CalculateScore_DenseGroundTruth_PerfectMatch)
{
    auto mc = makeTinyMission();

    // Dense ground truth with two occupied cells
    SparseBuildingMap gt;
    gt.initDense(mc);
    gt.setCell({0, 0, 0}, static_cast<int>(CellStatus::Occupied));
    gt.setCell({5, 5, 5}, static_cast<int>(CellStatus::Occupied));

    // Sparse drone map — correct on both occupied cells + marks interior empty
    SparseBuildingMap drone;
    drone.setCell({0, 0, 0}, static_cast<int>(CellStatus::Occupied));
    drone.setCell({5, 5, 5}, static_cast<int>(CellStatus::Occupied));
    drone.setCell({3, 3, 3}, static_cast<int>(CellStatus::Empty));

    const double score = drone.calculateScore(gt, mc);
    // Occupied recall = 2/2 = 100% → score_A = 50
    // Empty precision = 1 correct empty / 1 total empty → score_B = 30
    EXPECT_GE(score, 79.0);
    EXPECT_LE(score, 100.0);
}

// Missing occupied cells in drone map lowers score
TEST(SparseBuildingMap_Dense, CalculateScore_DenseGroundTruth_PartialRecall)
{
    auto mc = makeTinyMission();

    SparseBuildingMap gt;
    gt.initDense(mc);
    gt.setCell({0, 0, 0}, static_cast<int>(CellStatus::Occupied));
    gt.setCell({1, 0, 0}, static_cast<int>(CellStatus::Occupied));
    gt.setCell({2, 0, 0}, static_cast<int>(CellStatus::Occupied));
    gt.setCell({3, 0, 0}, static_cast<int>(CellStatus::Occupied));

    // Drone only finds 1 out of 4
    SparseBuildingMap drone;
    drone.setCell({0, 0, 0}, static_cast<int>(CellStatus::Occupied));

    const double score = drone.calculateScore(gt, mc);
    // Occupied recall = 1/4 = 25% → score_A = 12.5
    EXPECT_LT(score, 20.0);
}

// ============================================================
// Dense mode — LidarMock uses dense ground truth (the real sim path)
// ============================================================

#include "LidarMock.hpp"
#include "PositionSensorMock.hpp"

// The actual simulation passes a dense SparseBuildingMap to LidarMock.
// This test confirms ray-casting works correctly against a dense ground truth.
TEST(SparseBuildingMap_Dense, LidarMock_DenseGroundTruth_DetectsEastWall)
{
    auto mc = makeDefaultMission();   // 0..100 in XY, 0..50 in Z
    auto dc = makeDefaultDrone();

    // Dense ground truth with east wall at x=80
    auto gt = std::make_shared<SparseBuildingMap>();
    gt->initDense(mc);
    for (int y = 0; y <= 100; ++y)
        for (int z = 0; z <= 50; ++z)
            gt->setCell({80, y, z}, static_cast<int>(CellStatus::Occupied));

    auto pos = std::make_shared<PositionSensorMock>();
    pos->setPosition(Position3D{50.0 * cm, 50.0 * cm, 25.0 * cm});
    pos->setAngle(0.0 * deg);  // facing East

    LidarMock lidar(gt, pos, dc, mc);
    auto results = lidar.scan(0.0 * deg, 0.0 * deg);
    ASSERT_FALSE(results.empty());

    // Central beam should hit wall at ~30 cm (wall at x=80, drone at x=50)
    EXPECT_DISTANCE_NEAR(results[0].distance, 30.0, 2.0);
}

// Dense ground truth: scan in empty space returns Zmax
TEST(SparseBuildingMap_Dense, LidarMock_DenseGroundTruth_EmptyReturnsZmax)
{
    auto mc = makeDefaultMission();
    auto dc = makeDefaultDrone();

    // Dense ground truth with no obstacles
    auto gt = std::make_shared<SparseBuildingMap>();
    gt->initDense(mc);

    auto pos = std::make_shared<PositionSensorMock>();
    pos->setPosition(Position3D{50.0 * cm, 50.0 * cm, 25.0 * cm});
    pos->setAngle(0.0 * deg);

    LidarMock lidar(gt, pos, dc, mc);
    auto results = lidar.scan(0.0 * deg, 0.0 * deg);
    ASSERT_FALSE(results.empty());

    EXPECT_DISTANCE_NEAR(results[0].distance,
                         dc.lidarZmax.numerical_value_in(cm), 2.0);
}
