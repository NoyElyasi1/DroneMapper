#include "test_helpers.hpp"
#include "Drone.hpp"
#include "PositionSensorMock.hpp"
#include "MovementDriverMock.hpp"
#include "LidarMock.hpp"

#include <gtest/gtest.h>
#include <memory>

using namespace dm;
using namespace dm::test;

// ============================================================
// Drone unit + integration tests
// ============================================================

// Helper: create a complete drone system over a given ground-truth map.
// Returns the Drone + a handle to the position sensor mock.
struct DroneSystem {
    std::shared_ptr<PositionSensorMock>  pos;
    std::shared_ptr<MovementDriverMock>  driver;
    std::shared_ptr<LidarMock>           lidar;
    std::shared_ptr<SparseBuildingMap>   groundTruth;
    std::unique_ptr<Drone>               drone;
};

static DroneSystem makeDroneSystem(
    const SparseBuildingMap& gt,
    const DroneConfig&       dc,
    const MissionConfig&     mc,
    double startX, double startY, double startZ)
{
    DroneSystem sys;
    sys.groundTruth = std::make_shared<SparseBuildingMap>(gt);
    sys.pos         = std::make_shared<PositionSensorMock>();
    sys.pos->setPosition(Position3D{startX * cm, startY * cm, startZ * cm});
    sys.pos->setAngle(0.0 * deg);
    sys.driver  = std::make_shared<MovementDriverMock>(
                      sys.groundTruth, sys.pos, dc, mc);
    sys.lidar   = std::make_shared<LidarMock>(
                      sys.groundTruth, sys.pos, dc, mc);
    sys.drone   = std::make_unique<Drone>(
                      sys.pos, sys.lidar, sys.driver, dc, mc);
    return sys;
}

// ---- Test: first performStep scans current position ----
// After one step the drone's internal map should contain its
// current cell marked as Empty.
TEST(Drone, FirstStep_MarksStartCellEmpty)
{
    const SparseBuildingMap gt;  // empty ground truth
    const DroneConfig  dc = makeDefaultDrone();
    const MissionConfig mc = makeDefaultMission();

    auto sys = makeDroneSystem(gt, dc, mc, 50.0, 50.0, 25.0);
    sys.drone->performStep();  // may return true or false

    const GridPoint startGrid = toGrid(50.0, 50.0, 25.0, mc);
    EXPECT_EQ(sys.drone->getMap().getCell(startGrid),
              static_cast<int>(CellStatus::Empty));
}

// ---- Test: scan detects a wall ahead ----
// Place a wall at x=70 in an otherwise empty world.
// After one step, the drone's map should contain the wall as Occupied.
TEST(Drone, FirstStep_DetectsWallAhead)
{
    SparseBuildingMap gt;
    // Solid wall at x=70 plane
    for (int y = 0; y <= 100; ++y)
        for (int z = 0; z <= 50; ++z)
            gt.setCell({70, y, z}, static_cast<int>(CellStatus::Occupied));

    const DroneConfig  dc = makeDefaultDrone();
    MissionConfig mc = makeDefaultMission();

    auto sys = makeDroneSystem(gt, dc, mc, 50.0, 50.0, 25.0);
    sys.drone->performStep();

    // The wall voxel at (70, 50, 25) should be Occupied in result map
    EXPECT_EQ(sys.drone->getMap().getCell({70, 50, 25}),
              static_cast<int>(CellStatus::Occupied));
}

// ---- Test: cells along beam path are marked Empty ----
TEST(Drone, Scan_MarksBeamPathEmpty)
{
    SparseBuildingMap gt;
    // Wall at x=80
    for (int y = 0; y <= 100; ++y)
        for (int z = 0; z <= 50; ++z)
            gt.setCell({80, y, z}, static_cast<int>(CellStatus::Occupied));

    const DroneConfig  dc = makeDefaultDrone();
    const MissionConfig mc = makeDefaultMission();

    auto sys = makeDroneSystem(gt, dc, mc, 50.0, 50.0, 25.0);
    sys.drone->performStep();

    // Cells between drone (x=50) and wall (x=80) along East beam should be Empty
    // Check a few sample points
    EXPECT_EQ(sys.drone->getMap().getCell({60, 50, 25}),
              static_cast<int>(CellStatus::Empty));
    EXPECT_EQ(sys.drone->getMap().getCell({70, 50, 25}),
              static_cast<int>(CellStatus::Empty));
}

// ---- Test: performStep returns false in a tiny bounded space ----
// A 3×3×3 space: drone starts in the centre, walls on all sides.
// After scanning all faces, every reachable cell is known → should finish.
TEST(Drone, SmallEnclosedRoom_EventuallyTerminates)
{
    SparseBuildingMap gt;
    const int OCC = static_cast<int>(CellStatus::Occupied);

    // Build a tiny 10x10x10 cm box (walls at 0 and 10 in each axis)
    for (int i = 0; i <= 10; ++i) {
        for (int j = 0; j <= 10; ++j) {
            gt.setCell({0,  i, j}, OCC);
            gt.setCell({10, i, j}, OCC);
            gt.setCell({i, 0,  j}, OCC);
            gt.setCell({i, 10, j}, OCC);
            gt.setCell({i, j,  0}, OCC);
            gt.setCell({i, j, 10}, OCC);
        }
    }

    MissionConfig mc;
    mc.minX = 0; mc.maxX = 10;
    mc.minY = 0; mc.maxY = 10;
    mc.minHeight = 0; mc.maxHeight = 10;
    mc.resX = mc.resY = mc.resHeight = 0;
    mc.computeSteps();

    DroneConfig dc = makeDefaultDrone();
    dc.lidarZmax  = 15.0 * cm;
    dc.maxAdvance = 5.0  * cm;
    dc.maxElevate = 5.0  * cm;

    auto sys = makeDroneSystem(gt, dc, mc, 5.0, 5.0, 5.0);

    int steps = 0;
    constexpr int MAX = 100000;
    while (sys.drone->performStep() && steps < MAX) { ++steps; }

    EXPECT_LT(steps, MAX) << "Drone did not finish within " << MAX << " steps";
}

// ---- Test: drone never crashes (returns false cleanly) ----
// With an obstacle directly in every direction the drone
// should mark neighbours as occupied and finish without crashing.
TEST(Drone, ObstaclesEverywhere_FinishesCleanly)
{
    SparseBuildingMap gt;
    const int OCC = static_cast<int>(CellStatus::Occupied);

    // Fill every cell except (5,5,5)
    for (int x = 0; x <= 10; ++x)
        for (int y = 0; y <= 10; ++y)
            for (int z = 0; z <= 10; ++z)
                if (!(x == 5 && y == 5 && z == 5))
                    gt.setCell({x, y, z}, OCC);

    MissionConfig mc;
    mc.minX = 0; mc.maxX = 10;
    mc.minY = 0; mc.maxY = 10;
    mc.minHeight = 0; mc.maxHeight = 10;
    mc.computeSteps();

    DroneConfig dc = makeDefaultDrone();
    auto sys = makeDroneSystem(gt, dc, mc, 5.0, 5.0, 5.0);

    // Should terminate without hanging or crashing
    int steps = 0;
    while (sys.drone->performStep() && steps < 1000) { ++steps; }

    EXPECT_LT(steps, 1000);  // must not loop forever
}

// ---- Test: result map only has Empty and Occupied (no raw -1 in interior) ----
// After mapping an open room the cells visited should be 0 or 1.
TEST(Drone, ResultMap_NoUnmappedInVisitedRegion)
{
    SparseBuildingMap gt;
    const int OCC = static_cast<int>(CellStatus::Occupied);
    // Simple 3-cell corridor: open at x=4,5,6 surrounded by walls
    for (int y = 0; y <= 10; ++y)
        for (int z = 0; z <= 10; ++z) {
            gt.setCell({3, y, z}, OCC);
            gt.setCell({7, y, z}, OCC);
        }
    for (int x = 0; x <= 10; ++x)
        for (int z = 0; z <= 10; ++z) {
            gt.setCell({x, 0,  z}, OCC);
            gt.setCell({x, 10, z}, OCC);
        }
    for (int x = 0; x <= 10; ++x)
        for (int y = 0; y <= 10; ++y) {
            gt.setCell({x, y, 0},  OCC);
            gt.setCell({x, y, 10}, OCC);
        }

    MissionConfig mc;
    mc.minX = 0; mc.maxX = 10;
    mc.minY = 0; mc.maxY = 10;
    mc.minHeight = 0; mc.maxHeight = 10;
    mc.computeSteps();

    DroneConfig dc = makeDefaultDrone();
    dc.lidarZmax  = 15.0 * cm;
    dc.maxAdvance = 3.0  * cm;
    dc.maxElevate = 3.0  * cm;

    auto sys = makeDroneSystem(gt, dc, mc, 5.0, 5.0, 5.0);

    int steps = 0;
    while (sys.drone->performStep() && steps < 200000) { ++steps; }

    // Every cell in the result map should be either Empty or Occupied
    for (const auto& [pt, status] : sys.drone->getMap().data()) {
        EXPECT_TRUE(status == static_cast<int>(CellStatus::Empty) ||
                    status == static_cast<int>(CellStatus::Occupied))
            << "Unexpected status " << status
            << " at (" << pt.x << "," << pt.y << "," << pt.z << ")";
    }
}

// ---- Integration test: score > 0 after mapping a simple room ----
// This is the full end-to-end test. We build a room with walls,
// run the drone, and verify the final score is meaningful (> 30).
TEST(Drone, Integration_SimpleRoom_ScoreAboveThreshold)
{
    // Build a 30×30×20 room (manageable size for fast test)
    SparseBuildingMap gt;
    const int OCC = static_cast<int>(CellStatus::Occupied);

    for (int i = 0; i <= 30; ++i)
        for (int j = 0; j <= 30; ++j) {
            gt.setCell({0,  i,  j}, OCC);
            gt.setCell({30, i,  j}, OCC);
            gt.setCell({i,  0,  j}, OCC);
            gt.setCell({i,  30, j}, OCC);
            gt.setCell({i,  j,  0}, OCC);
            gt.setCell({i,  j,  20}, OCC);
        }

    MissionConfig mc;
    mc.minX = 0; mc.maxX = 30;
    mc.minY = 0; mc.maxY = 30;
    mc.minHeight = 0; mc.maxHeight = 20;
    mc.computeSteps();

    DroneConfig dc = makeDefaultDrone();
    dc.lidarZmax  = 40.0 * cm;   // see across the whole room
    dc.lidarFOVC  = 3;
    dc.maxAdvance = 5.0 * cm;
    dc.maxElevate = 5.0 * cm;

    auto sys = makeDroneSystem(gt, dc, mc, 15.0, 15.0, 10.0);

    int steps = 0;
    constexpr int MAX_STEPS = 500000;
    while (sys.drone->performStep() && steps < MAX_STEPS) { ++steps; }

    const double score = sys.drone->getMap().calculateScore(gt, mc);
    std::cout << "[Integration] Steps: " << steps
              << "  Score: " << score << "/100\n";

    EXPECT_GT(score, 30.0)
        << "Score too low — drone probably failed to explore the room.";
    EXPECT_LE(score, 100.0);
}
