// ============================================================
// test_boundaries.cpp — Mission boundary tests for MovementDriverMock.
//
// The existing mock tests check obstacle collisions and per-move
// limit enforcement, but none of them verify that the drone cannot
// fly outside the declared mission volume (minX/maxX/minY/maxY/
// minHeight/maxHeight).  This file closes that gap.
// ============================================================

#include "test_helpers.hpp"
#include "MovementDriverMock.hpp"
#include "PositionSensorMock.hpp"

#include <gtest/gtest.h>
#include <memory>

using namespace dm;
using namespace dm::test;

// Helper: create driver + position sensor, no obstacles, drone at given position
struct BoundaryFixture {
    std::shared_ptr<PositionSensorMock>  pos;
    std::shared_ptr<MovementDriverMock>  driver;
};

static BoundaryFixture makeAt(double x, double y, double z,
                               double angleDeg,
                               const DroneConfig&   dc,
                               const MissionConfig& mc)
{
    auto gt  = std::make_shared<SparseBuildingMap>();  // empty — no obstacles
    auto pos = std::make_shared<PositionSensorMock>();
    pos->setPosition(Position3D{x * cm, y * cm, z * cm});
    pos->setAngle(angleDeg * deg);
    auto driver = std::make_shared<MovementDriverMock>(gt, pos, dc, mc);
    return {pos, driver};
}

// ============================================================
// Horizontal boundaries (advance)
// ============================================================

// Advance East that would push X beyond maxX is rejected
TEST(MovementDriverMock_Boundary, Advance_WouldExceedMaxX_Rejected)
{
    auto dc = makeDefaultDrone();   // width=10, maxAdvance=20
    auto mc = makeDefaultMission(); // maxX=100

    // Drone centre at x=95; half-width=5 → right edge at 100 (touching boundary)
    // Advancing 10 cm East → centre x=105 → right edge=110 > maxX=100
    auto [pos, driver] = makeAt(95.0, 50.0, 25.0, 0.0, dc, mc);

    EXPECT_FALSE(driver->advance(10.0 * cm));
    EXPECT_NEAR(pos->getCurrentPosition().x.numerical_value_in(cm), 95.0, 1e-6);
}

// Advance West that would push X below minX is rejected
TEST(MovementDriverMock_Boundary, Advance_WouldGoBelowMinX_Rejected)
{
    auto dc = makeDefaultDrone();
    dc.maxAdvance = 20.0 * cm;
    auto mc = makeDefaultMission(); // minX=0

    // Drone at x=3; heading West (180°); advancing 10 → centre x=-7 < minX+halfWidth
    auto [pos, driver] = makeAt(3.0, 50.0, 25.0, 180.0, dc, mc);

    EXPECT_FALSE(driver->advance(10.0 * cm));
    EXPECT_NEAR(pos->getCurrentPosition().x.numerical_value_in(cm), 3.0, 1e-6);
}

// Advance South that would push Y beyond maxY is rejected
TEST(MovementDriverMock_Boundary, Advance_WouldExceedMaxY_Rejected)
{
    auto dc = makeDefaultDrone();
    auto mc = makeDefaultMission(); // maxY=100

    // Heading South (90°), drone near y=maxY
    auto [pos, driver] = makeAt(50.0, 95.0, 25.0, 90.0, dc, mc);

    EXPECT_FALSE(driver->advance(10.0 * cm));
    EXPECT_NEAR(pos->getCurrentPosition().y.numerical_value_in(cm), 95.0, 1e-6);
}

// Advance that stays inside bounds succeeds
TEST(MovementDriverMock_Boundary, Advance_InsideBounds_Succeeds)
{
    auto dc = makeDefaultDrone();
    auto mc = makeDefaultMission();

    // Plenty of room in every direction
    auto [pos, driver] = makeAt(50.0, 50.0, 25.0, 0.0, dc, mc);

    EXPECT_TRUE(driver->advance(10.0 * cm));
    EXPECT_NEAR(pos->getCurrentPosition().x.numerical_value_in(cm), 60.0, 1e-6);
}

// ============================================================
// Vertical boundaries (elevate)
// ============================================================

// Elevate up that would push drone top above maxHeight is rejected
TEST(MovementDriverMock_Boundary, Elevate_WouldExceedMaxHeight_Rejected)
{
    auto dc = makeDefaultDrone();   // height=5 cm, half=2.5
    auto mc = makeDefaultMission(); // maxHeight=50

    // Drone centre at z=48; top edge = 48+2.5 = 50.5 after moving up 1 cm
    // (top edge after = 48+1+2.5 = 51.5 > 50)
    auto [pos, driver] = makeAt(50.0, 50.0, 48.0, 0.0, dc, mc);

    EXPECT_FALSE(driver->elevate(5.0 * cm));
    EXPECT_NEAR(pos->getCurrentPosition().z.numerical_value_in(cm), 48.0, 1e-6);
}

// Elevate down that would push drone bottom below minHeight is rejected
TEST(MovementDriverMock_Boundary, Elevate_WouldGoBelowMinHeight_Rejected)
{
    auto dc = makeDefaultDrone();   // height=5, half=2.5
    auto mc = makeDefaultMission(); // minHeight=0

    // Drone at z=2; bottom edge = 2-2.5 = -0.5 after moving down 1 cm → < 0
    auto [pos, driver] = makeAt(50.0, 50.0, 2.0, 0.0, dc, mc);

    EXPECT_FALSE(driver->elevate(-5.0 * cm));
    EXPECT_NEAR(pos->getCurrentPosition().z.numerical_value_in(cm), 2.0, 1e-6);
}

// Elevate that stays inside bounds succeeds
TEST(MovementDriverMock_Boundary, Elevate_InsideBounds_Succeeds)
{
    auto dc = makeDefaultDrone();
    auto mc = makeDefaultMission();

    auto [pos, driver] = makeAt(50.0, 50.0, 15.0, 0.0, dc, mc);

    EXPECT_TRUE(driver->elevate(5.0 * cm));
    EXPECT_NEAR(pos->getCurrentPosition().z.numerical_value_in(cm), 20.0, 1e-6);
}

// Negative elevate (descend) that stays inside bounds succeeds
TEST(MovementDriverMock_Boundary, Elevate_Descend_InsideBounds_Succeeds)
{
    auto dc = makeDefaultDrone();
    auto mc = makeDefaultMission();

    auto [pos, driver] = makeAt(50.0, 50.0, 25.0, 0.0, dc, mc);

    EXPECT_TRUE(driver->elevate(-5.0 * cm));
    EXPECT_NEAR(pos->getCurrentPosition().z.numerical_value_in(cm), 20.0, 1e-6);
}
