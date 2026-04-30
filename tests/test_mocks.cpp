#include "test_helpers.hpp"
#include "PositionSensorMock.hpp"
#include "MovementDriverMock.hpp"
#include "LidarMock.hpp"

#include <gtest/gtest.h>
#include <memory>
#include <cmath>

using namespace dm;
using namespace dm::test;

// ============================================================
// PositionSensorMock tests
// ============================================================

TEST(PositionSensorMock, InitialPosition_IsOrigin)
{
    PositionSensorMock sensor;
    const Position3D pos = sensor.getCurrentPosition();
    EXPECT_DISTANCE_NEAR(pos.x, 0.0, 1e-9);
    EXPECT_DISTANCE_NEAR(pos.y, 0.0, 1e-9);
    EXPECT_DISTANCE_NEAR(pos.z, 0.0, 1e-9);
}

TEST(PositionSensorMock, InitialAngle_IsZero)
{
    PositionSensorMock sensor;
    EXPECT_ANGLE_NEAR(sensor.getCurrentAngle(), 0.0, 1e-9);
}

TEST(PositionSensorMock, SetPosition_Roundtrip)
{
    PositionSensorMock sensor;
    sensor.setPosition(Position3D{15.0 * cm, -30.0 * cm, 7.5 * cm});
    const Position3D pos = sensor.getCurrentPosition();
    EXPECT_DISTANCE_NEAR(pos.x,  15.0, 1e-9);
    EXPECT_DISTANCE_NEAR(pos.y, -30.0, 1e-9);
    EXPECT_DISTANCE_NEAR(pos.z,   7.5, 1e-9);
}

TEST(PositionSensorMock, SetAngle_NormalisesOver360)
{
    PositionSensorMock sensor;
    sensor.setAngle(450.0 * deg);  // 450 → 90
    EXPECT_ANGLE_NEAR(sensor.getCurrentAngle(), 90.0, 1e-9);
}

TEST(PositionSensorMock, SetAngle_NormalisesNegative)
{
    PositionSensorMock sensor;
    sensor.setAngle(-90.0 * deg);  // -90 → 270
    EXPECT_ANGLE_NEAR(sensor.getCurrentAngle(), 270.0, 1e-9);
}

TEST(PositionSensorMock, SetAngle_Zero_StaysZero)
{
    PositionSensorMock sensor;
    sensor.setAngle(360.0 * deg);  // 360 → 0
    EXPECT_ANGLE_NEAR(sensor.getCurrentAngle(), 0.0, 1e-9);
}

// ============================================================
// MovementDriverMock tests
// ============================================================

// Build a driver with an empty ground-truth map
// (no obstacles → all moves should succeed inside bounds)
static auto makeFreeDriver(const DroneConfig& dc, const MissionConfig& mc)
    -> std::pair<std::shared_ptr<MovementDriverMock>,
                 std::shared_ptr<PositionSensorMock>>
{
    auto gt      = std::make_shared<SparseBuildingMap>();   // empty map
    auto posMock = std::make_shared<PositionSensorMock>();
    // Place drone at centre, at mid-height
    posMock->setPosition(Position3D{50.0 * cm, 50.0 * cm, 25.0 * cm});
    posMock->setAngle(0.0 * deg);
    auto driver  = std::make_shared<MovementDriverMock>(gt, posMock, dc, mc);
    return {driver, posMock};
}

TEST(MovementDriverMock, Rotate_UpdatesAngle)
{
    auto dc = makeDefaultDrone();
    auto mc = makeDefaultMission();
    auto [driver, pos] = makeFreeDriver(dc, mc);

    EXPECT_TRUE(driver->rotate(45.0 * deg));
    EXPECT_ANGLE_NEAR(pos->getCurrentAngle(), 45.0, 1e-9);
}

TEST(MovementDriverMock, Rotate_AccumulatesCorrectly)
{
    auto dc = makeDefaultDrone();
    auto mc = makeDefaultMission();
    auto [driver, pos] = makeFreeDriver(dc, mc);

    driver->rotate(90.0 * deg);
    driver->rotate(90.0 * deg);
    // 90+90 = 180°
    EXPECT_ANGLE_NEAR(pos->getCurrentAngle(), 180.0, 1e-9);
}

TEST(MovementDriverMock, Rotate_NormalisesOver360)
{
    auto dc = makeDefaultDrone();
    auto mc = makeDefaultMission();
    auto [driver, pos] = makeFreeDriver(dc, mc);

    driver->rotate(350.0 * deg);
    driver->rotate(20.0 * deg);
    // 350 + 20 = 370 → 10°
    EXPECT_ANGLE_NEAR(pos->getCurrentAngle(), 10.0, 1e-9);
}

// Advance east (0°) moves +X
TEST(MovementDriverMock, Advance_East_MovesX)
{
    auto dc = makeDefaultDrone();
    auto mc = makeDefaultMission();
    auto [driver, pos] = makeFreeDriver(dc, mc);

    pos->setPosition(Position3D{50.0 * cm, 50.0 * cm, 25.0 * cm});
    pos->setAngle(0.0 * deg);  // East

    EXPECT_TRUE(driver->advance(10.0 * cm));

    const Position3D p = pos->getCurrentPosition();
    EXPECT_DISTANCE_NEAR(p.x, 60.0, 1e-6);
    EXPECT_DISTANCE_NEAR(p.y, 50.0, 1e-6);
    EXPECT_DISTANCE_NEAR(p.z, 25.0, 1e-6);
}

// Advance south (90°) moves +Y
TEST(MovementDriverMock, Advance_South_MovesY)
{
    auto dc = makeDefaultDrone();
    auto mc = makeDefaultMission();
    auto [driver, pos] = makeFreeDriver(dc, mc);

    pos->setPosition(Position3D{50.0 * cm, 50.0 * cm, 25.0 * cm});
    pos->setAngle(90.0 * deg);  // South

    driver->advance(10.0 * cm);

    const Position3D p = pos->getCurrentPosition();
    EXPECT_DISTANCE_NEAR(p.x, 50.0, 1e-6);
    EXPECT_DISTANCE_NEAR(p.y, 60.0, 1e-6);
}

// Advance is blocked when destination has an obstacle
TEST(MovementDriverMock, Advance_BlockedByWall)
{
    auto dc = makeDefaultDrone();
    auto mc = makeDefaultMission();

    // Place a wall at x=65 (occupying the column the drone would enter)
    auto gt = std::make_shared<SparseBuildingMap>();
    for (int y = 0; y <= 100; ++y)
        for (int z = 0; z <= 50; ++z)
            gt->setCell({65, y, z}, static_cast<int>(CellStatus::Occupied));

    auto pos = std::make_shared<PositionSensorMock>();
    pos->setPosition(Position3D{50.0 * cm, 50.0 * cm, 25.0 * cm});
    pos->setAngle(0.0 * deg);

    auto driver = std::make_shared<MovementDriverMock>(gt, pos, dc, mc);
    const bool ok = driver->advance(20.0 * cm);  // would land at x=70
    EXPECT_FALSE(ok);

    // Position should be unchanged
    EXPECT_DISTANCE_NEAR(pos->getCurrentPosition().x, 50.0, 1e-6);
}

// Elevate up increases Z
TEST(MovementDriverMock, Elevate_Up_IncreasesZ)
{
    auto dc = makeDefaultDrone();
    auto mc = makeDefaultMission();
    auto [driver, pos] = makeFreeDriver(dc, mc);

    pos->setPosition(Position3D{50.0 * cm, 50.0 * cm, 20.0 * cm});
    EXPECT_TRUE(driver->elevate(10.0 * cm));
    EXPECT_DISTANCE_NEAR(pos->getCurrentPosition().z, 30.0, 1e-6);
}

// Elevate down decreases Z
TEST(MovementDriverMock, Elevate_Down_DecreasesZ)
{
    auto dc = makeDefaultDrone();
    auto mc = makeDefaultMission();
    auto [driver, pos] = makeFreeDriver(dc, mc);

    pos->setPosition(Position3D{50.0 * cm, 50.0 * cm, 20.0 * cm});
    EXPECT_TRUE(driver->elevate(-10.0 * cm));
    EXPECT_DISTANCE_NEAR(pos->getCurrentPosition().z, 10.0, 1e-6);
}

// Elevate is blocked by ceiling
TEST(MovementDriverMock, Elevate_BlockedByCeiling)
{
    auto dc = makeDefaultDrone();
    auto mc = makeDefaultMission();  // maxHeight = 50

    // Solid ceiling at z=50 (already exists implicitly via bounds)
    auto gt = std::make_shared<SparseBuildingMap>();
    auto pos = std::make_shared<PositionSensorMock>();
    pos->setPosition(Position3D{50.0 * cm, 50.0 * cm, 44.0 * cm});

    auto driver = std::make_shared<MovementDriverMock>(gt, pos, dc, mc);
    // drone height=5cm, so top edge at 44+2.5=46.5. After +10: top=56.5 > maxHeight=50
    EXPECT_FALSE(driver->elevate(10.0 * cm));
}

// ============================================================
// LidarMock tests
// ============================================================

// Helper: create a LidarMock with a room that has a wall at x=80
// Drone starts at (50, 50, 25) facing East (0°)
static std::shared_ptr<LidarMock> makeEastWallLidar(
    std::shared_ptr<PositionSensorMock>& outPos)
{
    auto mc = makeDefaultMission();
    auto dc = makeDefaultDrone();

    auto gt = std::make_shared<SparseBuildingMap>();
    // East wall at x=80, full Y and Z extent
    for (int y = 0; y <= 100; ++y)
        for (int z = 0; z <= 50; ++z)
            gt->setCell({80, y, z}, static_cast<int>(CellStatus::Occupied));

    outPos = std::make_shared<PositionSensorMock>();
    outPos->setPosition(Position3D{50.0 * cm, 50.0 * cm, 25.0 * cm});
    outPos->setAngle(0.0 * deg);  // facing East

    return std::make_shared<LidarMock>(gt, outPos, dc, mc);
}

// Central beam (circle 0) hits the east wall at distance ≈ 30 cm
TEST(LidarMock, CentralBeam_HitsEastWall)
{
    std::shared_ptr<PositionSensorMock> pos;
    auto lidar = makeEastWallLidar(pos);

    auto results = lidar->scan(0.0 * deg, 0.0 * deg);
    ASSERT_FALSE(results.empty());

    // Circle 0 result (first element): direction should be (1,0,0)
    const ScanResult& r = results[0];
    EXPECT_NEAR(r.dx, 1.0, 1e-6);
    EXPECT_NEAR(r.dy, 0.0, 1e-6);
    EXPECT_NEAR(r.dz, 0.0, 1e-6);

    // Wall at x=80, drone at x=50 → distance = 30 cm
    EXPECT_DISTANCE_NEAR(r.distance, 30.0, 2.0);  // ±2 cm tolerance for step rounding
}

// Scanning 180° (West) in an open field returns Z-max (no hit)
TEST(LidarMock, WestScan_OpenField_ReturnsZmax)
{
    auto mc = makeDefaultMission();
    auto dc = makeDefaultDrone();

    auto gt  = std::make_shared<SparseBuildingMap>();  // no obstacles
    auto pos = std::make_shared<PositionSensorMock>();
    pos->setPosition(Position3D{50.0 * cm, 50.0 * cm, 25.0 * cm});
    pos->setAngle(0.0 * deg);

    LidarMock lidar(gt, pos, dc, mc);
    // Scan facing West (180° offset from heading 0°)
    auto results = lidar.scan(180.0 * deg, 0.0 * deg);
    ASSERT_FALSE(results.empty());

    // Central beam should return Zmax (no obstacle)
    EXPECT_DISTANCE_NEAR(results[0].distance,
                         dc.lidarZmax.numerical_value_in(cm), 2.0);
}

// Scanning straight up sees nothing in an empty field
TEST(LidarMock, UpScan_Empty_ReturnsZmax)
{
    auto mc = makeDefaultMission();
    auto dc = makeDefaultDrone();
    auto gt  = std::make_shared<SparseBuildingMap>();
    auto pos = std::make_shared<PositionSensorMock>();
    pos->setPosition(Position3D{50.0 * cm, 50.0 * cm, 10.0 * cm});

    LidarMock lidar(gt, pos, dc, mc);
    auto results = lidar.scan(0.0 * deg, 90.0 * deg);
    ASSERT_FALSE(results.empty());

    // Central beam direction should be (0, 0, 1)
    EXPECT_NEAR(results[0].dz, 1.0, 1e-6);
    EXPECT_DISTANCE_NEAR(results[0].distance,
                         dc.lidarZmax.numerical_value_in(cm), 2.0);
}

// Total number of beams for FOVC=3: 1 + 4 + 16 = 21
TEST(LidarMock, BeamCount_FOVC3_Is21)
{
    auto mc = makeDefaultMission();
    auto dc = makeDefaultDrone();
    dc.lidarFOVC = 3;

    auto gt  = std::make_shared<SparseBuildingMap>();
    auto pos = std::make_shared<PositionSensorMock>();
    LidarMock lidar(gt, pos, dc, mc);

    auto results = lidar.scan();
    EXPECT_EQ(results.size(), 21u);  // 4^0 + 4^1 + 4^2 = 1+4+16
}

// Total number of beams for FOVC=1: 1 (central beam only)
TEST(LidarMock, BeamCount_FOVC1_Is1)
{
    auto mc = makeDefaultMission();
    auto dc = makeDefaultDrone();
    dc.lidarFOVC = 1;

    auto gt  = std::make_shared<SparseBuildingMap>();
    auto pos = std::make_shared<PositionSensorMock>();
    LidarMock lidar(gt, pos, dc, mc);

    auto results = lidar.scan();
    EXPECT_EQ(results.size(), 1u);
}

// All beam direction vectors are unit length
TEST(LidarMock, AllBeams_AreUnitVectors)
{
    auto mc = makeDefaultMission();
    auto dc = makeDefaultDrone();
    dc.lidarFOVC = 3;

    auto gt  = std::make_shared<SparseBuildingMap>();
    auto pos = std::make_shared<PositionSensorMock>();
    LidarMock lidar(gt, pos, dc, mc);

    for (double az : {0.0, 45.0, 90.0, 135.0, 180.0}) {
        for (double el : {-45.0, 0.0, 45.0}) {
            auto results = lidar.scan(az * deg, el * deg);
            for (const auto& r : results) {
                const double len = std::sqrt(r.dx*r.dx + r.dy*r.dy + r.dz*r.dz);
                EXPECT_NEAR(len, 1.0, 1e-6)
                    << "Non-unit beam at az=" << az << " el=" << el;
            }
        }
    }
}

// Obstacle closer than Zmin → distance returned is 0
TEST(LidarMock, VeryCloseObstacle_ReturnsZero)
{
    auto mc = makeDefaultMission();
    auto dc = makeDefaultDrone();
    dc.lidarZmin = 20.0 * cm;

    // Wall at x=52 (only 2 cm ahead of drone at x=50 → < Zmin=20)
    auto gt = std::make_shared<SparseBuildingMap>();
    for (int y = 0; y <= 100; ++y)
        for (int z = 0; z <= 50; ++z)
            gt->setCell({52, y, z}, static_cast<int>(CellStatus::Occupied));

    auto pos = std::make_shared<PositionSensorMock>();
    pos->setPosition(Position3D{50.0 * cm, 50.0 * cm, 25.0 * cm});
    pos->setAngle(0.0 * deg);

    LidarMock lidar(gt, pos, dc, mc);
    auto results = lidar.scan();
    ASSERT_FALSE(results.empty());
    // Central beam hit at distance 2 cm < Zmin=20 cm → should return 0
    EXPECT_DISTANCE_NEAR(results[0].distance, 0.0, 1e-9);
}
