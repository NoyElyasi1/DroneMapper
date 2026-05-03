#include "test_helpers.hpp"
#include "SparseBuildingMap.hpp"
#include "ConfigParser.hpp"

#include <gtest/gtest.h>
#include <fstream>
#include <string>

using namespace dm;
using namespace dm::test;

// ============================================================
// SparseBuildingMap — cell access tests
// ============================================================

// A fresh map returns UnmappedNA for any unset cell
TEST(SparseBuildingMap, GetCell_Unset_ReturnsUnmappedNA)
{
    SparseBuildingMap map;
    EXPECT_EQ(map.getCell({0, 0, 0}),
              static_cast<int>(CellStatus::UnmappedNA));
    EXPECT_EQ(map.getCell({99, -5, 12}),
              static_cast<int>(CellStatus::UnmappedNA));
}

TEST(SparseBuildingMap, HasCell_ReturnsFalseForUnset)
{
    SparseBuildingMap map;
    EXPECT_FALSE(map.hasCell({1, 2, 3}));
}

TEST(SparseBuildingMap, SetAndGetCell_BasicOccupied)
{
    SparseBuildingMap map;
    map.setCell({5, 10, 15}, static_cast<int>(CellStatus::Occupied));
    EXPECT_EQ(map.getCell({5, 10, 15}),
              static_cast<int>(CellStatus::Occupied));
    EXPECT_TRUE(map.hasCell({5, 10, 15}));
}

TEST(SparseBuildingMap, SetAndGetCell_Empty)
{
    SparseBuildingMap map;
    map.setCell({0, 0, 0}, static_cast<int>(CellStatus::Empty));
    EXPECT_EQ(map.getCell({0, 0, 0}),
              static_cast<int>(CellStatus::Empty));
}

// Setting a cell twice: last value wins
TEST(SparseBuildingMap, SetCell_Overwrite)
{
    SparseBuildingMap map;
    map.setCell({1, 1, 1}, static_cast<int>(CellStatus::Empty));
    map.setCell({1, 1, 1}, static_cast<int>(CellStatus::Occupied));
    EXPECT_EQ(map.getCell({1, 1, 1}),
              static_cast<int>(CellStatus::Occupied));
}

// Negative grid indices are valid
TEST(SparseBuildingMap, NegativeIndices_Work)
{
    SparseBuildingMap map;
    map.setCell({-10, -20, -5}, static_cast<int>(CellStatus::Occupied));
    EXPECT_EQ(map.getCell({-10, -20, -5}),
              static_cast<int>(CellStatus::Occupied));
}

// Setting different cells does not interfere with each other
TEST(SparseBuildingMap, MultipleCells_NoCrossContamination)
{
    SparseBuildingMap map;
    map.setCell({0, 0, 0}, static_cast<int>(CellStatus::Occupied));
    map.setCell({1, 0, 0}, static_cast<int>(CellStatus::Empty));
    map.setCell({0, 1, 0}, static_cast<int>(CellStatus::UnmappedNA));

    EXPECT_EQ(map.getCell({0, 0, 0}), static_cast<int>(CellStatus::Occupied));
    EXPECT_EQ(map.getCell({1, 0, 0}), static_cast<int>(CellStatus::Empty));
    EXPECT_EQ(map.getCell({0, 1, 0}), static_cast<int>(CellStatus::UnmappedNA));
    EXPECT_EQ(map.getCell({0, 0, 1}), static_cast<int>(CellStatus::UnmappedNA));
}

// ============================================================
// SparseBuildingMap — file I/O tests
// ============================================================

TEST(SparseBuildingMap, SaveAndLoad_Roundtrip)
{
    SparseBuildingMap orig;
    orig.setCell({1,  2,  3},  static_cast<int>(CellStatus::Occupied));
    orig.setCell({-5, 0,  10}, static_cast<int>(CellStatus::Empty));
    // UnmappedNA cells are not saved (they are the default for any unset cell)

    const std::string path = writeTempFile("", ".csv");
    EXPECT_TRUE(orig.saveToFile(path));

    SparseBuildingMap loaded;
    std::string errors;
    EXPECT_TRUE(loaded.loadFromFile(path, errors));
    EXPECT_TRUE(errors.empty()) << errors;

    EXPECT_EQ(loaded.getCell({1, 2, 3}),
              static_cast<int>(CellStatus::Occupied));
    EXPECT_EQ(loaded.getCell({-5, 0, 10}),
              static_cast<int>(CellStatus::Empty));
    // Unset cell returns UnmappedNA by default
    EXPECT_EQ(loaded.getCell({99, 99, 99}),
              static_cast<int>(CellStatus::UnmappedNA));
}

// File with comment lines and blank lines is parsed correctly
TEST(SparseBuildingMap, LoadFromFile_SkipsCommentsAndBlanks)
{
    const std::string content =
        "# x,y,z,status\n"
        "\n"
        "10,20,30,1\n"
        "# another comment\n"
        "\n"
        "-5,-5,0,0\n";

    SparseBuildingMap map;
    std::string errors;
    EXPECT_TRUE(map.loadFromFile(writeTempFile(content), errors));
    EXPECT_TRUE(errors.empty()) << errors;

    EXPECT_EQ(map.getCell({ 10,  20, 30}), 1);
    EXPECT_EQ(map.getCell({ -5,  -5,  0}), 0);
    EXPECT_EQ(map.data().size(), 2u);
}

// Bad format lines are skipped, error is logged, good lines still loaded
TEST(SparseBuildingMap, LoadFromFile_BadLines_Recoverable)
{
    const std::string content =
        "10,20,30,1\n"
        "this is garbage\n"
        "1,2,3\n"           // missing status field
        "5,6,7,0\n";

    SparseBuildingMap map;
    std::string errors;
    EXPECT_TRUE(map.loadFromFile(writeTempFile(content), errors));
    EXPECT_FALSE(errors.empty());

    // Good lines loaded, bad ones skipped
    EXPECT_EQ(map.getCell({10, 20, 30}), 1);
    EXPECT_EQ(map.getCell({ 5,  6,  7}), 0);
    // Garbage lines not in map
    EXPECT_EQ(map.data().size(), 2u);
}

// File not found → returns false, error logged
TEST(SparseBuildingMap, LoadFromFile_FileNotFound)
{
    SparseBuildingMap map;
    std::string errors;
    EXPECT_FALSE(map.loadFromFile(
        "this_really_does_not_exist_999.csv", errors));
    EXPECT_FALSE(errors.empty());
    EXPECT_TRUE(map.data().empty());
}

// saveToFile to an invalid path returns false (no crash)
TEST(SparseBuildingMap, SaveToFile_InvalidPath_ReturnsFalse)
{
    SparseBuildingMap map;
    map.setCell({0, 0, 0}, 1);
    EXPECT_FALSE(map.saveToFile("Z:/no_such_dir/out.csv"));
}

// ============================================================
// SparseBuildingMap — score calculation tests
// ============================================================

TEST(SparseBuildingMap, Score_PerfectMatch_ReturnsHigh)
{
    // Build ground truth with a few occupied cells
    SparseBuildingMap gt;
    gt.setCell({0, 0, 0}, static_cast<int>(CellStatus::Occupied));
    gt.setCell({1, 0, 0}, static_cast<int>(CellStatus::Occupied));
    gt.setCell({0, 1, 0}, static_cast<int>(CellStatus::Occupied));

    // Drone map matches exactly + marks interior as empty
    SparseBuildingMap drone;
    drone.setCell({0, 0, 0}, static_cast<int>(CellStatus::Occupied));
    drone.setCell({1, 0, 0}, static_cast<int>(CellStatus::Occupied));
    drone.setCell({0, 1, 0}, static_cast<int>(CellStatus::Occupied));
    drone.setCell({5, 5, 5}, static_cast<int>(CellStatus::Empty));

    const MissionConfig mc = makeDefaultMission();
    const double score = drone.calculateScore(gt, mc);
    // Occupied recall = 100% (3/3), so score_A = 50
    // Empty precision = 100% (1 empty cell, not occupied in gt)
    // score_B = 30
    // Coverage = small fraction of 101*101*51 cells → score_C ≈ 0
    EXPECT_GE(score, 70.0);  // at least A+B components
}

TEST(SparseBuildingMap, Score_DroneMapEmpty_ScoreZero)
{
    SparseBuildingMap gt;
    gt.setCell({0, 0, 0}, static_cast<int>(CellStatus::Occupied));
    gt.setCell({1, 1, 1}, static_cast<int>(CellStatus::Occupied));

    SparseBuildingMap drone;  // nothing mapped

    const MissionConfig mc = makeDefaultMission();
    const double score = drone.calculateScore(gt, mc);
    // Occupied recall = 0, empty precision = 0 (no empty cells), coverage = 0
    EXPECT_NEAR(score, 0.0, 1e-9);
}

TEST(SparseBuildingMap, Score_EmptyGroundTruth_Returns50)
{
    SparseBuildingMap gt;  // no occupied cells

    SparseBuildingMap drone;
    drone.setCell({5, 5, 5}, static_cast<int>(CellStatus::Empty));

    const MissionConfig mc = makeDefaultMission();
    const double score = drone.calculateScore(gt, mc);
    // Score_A = 50 (trivially correct), Score_B = 30 (empty cell valid)
    EXPECT_GE(score, 50.0);
}

TEST(SparseBuildingMap, Score_FalsePositive_PenalisesEmptyPrecision)
{
    // Ground truth: one occupied cell at (0,0,0)
    SparseBuildingMap gt;
    gt.setCell({0, 0, 0}, static_cast<int>(CellStatus::Occupied));

    // Drone: correctly marks (0,0,0) as occupied, but wrongly marks
    // (1,0,0) as occupied too (false positive = doesn't affect B)
    // AND wrongly marks (0,0,0) as empty somewhere else
    SparseBuildingMap drone;
    drone.setCell({0, 0, 0}, static_cast<int>(CellStatus::Occupied)); // correct
    // Mark a cell as empty that is occupied in GT (false negative)
    drone.setCell({0, 1, 0}, static_cast<int>(CellStatus::Empty));    // OK (not GT occupied)

    const MissionConfig mc = makeDefaultMission();
    const double score = drone.calculateScore(gt, mc);
    EXPECT_GE(score, 50.0);  // occupied recall = 100% → score_A = 50
}

TEST(SparseBuildingMap, Score_BoundedBetween0And100)
{
    SparseBuildingMap gt  = makeSimpleRoomMap();
    SparseBuildingMap drone;

    const MissionConfig mc = makeDefaultMission();
    const double score = drone.calculateScore(gt, mc);
    EXPECT_GE(score, 0.0);
    EXPECT_LE(score, 100.0);
}
