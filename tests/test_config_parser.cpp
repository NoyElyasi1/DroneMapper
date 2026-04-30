#include "test_helpers.hpp"
#include "ConfigParser.hpp"

#include <gtest/gtest.h>
#include <cmath>

using namespace dm;
using namespace dm::test;

// ============================================================
// ConfigParser — DroneConfig tests
// ============================================================

// All valid keys present → values parsed correctly, no errors
TEST(DroneConfig, ValidFile_AllKeysPresent)
{
    const std::string content =
        "width=25\n"
        "length=35\n"
        "height=15\n"
        "max_rotate=45\n"
        "max_advance=20\n"
        "max_elevate=15\n"
        "lidar_zmin=10\n"
        "lidar_zmax=100\n"
        "lidar_d=3.0\n"
        "lidar_fovc=3\n";

    DroneConfig cfg;
    std::string errors;
    EXPECT_TRUE(parseDroneConfig(writeTempFile(content), cfg, errors));
    EXPECT_TRUE(errors.empty()) << "Unexpected errors: " << errors;

    EXPECT_DISTANCE_NEAR(cfg.width,      25.0, 1e-9);
    EXPECT_DISTANCE_NEAR(cfg.length,     35.0, 1e-9);
    EXPECT_DISTANCE_NEAR(cfg.height,     15.0, 1e-9);
    EXPECT_ANGLE_NEAR   (cfg.maxRotate,  45.0, 1e-9);
    EXPECT_DISTANCE_NEAR(cfg.maxAdvance, 20.0, 1e-9);
    EXPECT_DISTANCE_NEAR(cfg.maxElevate, 15.0, 1e-9);
    EXPECT_DISTANCE_NEAR(cfg.lidarZmin,  10.0, 1e-9);
    EXPECT_DISTANCE_NEAR(cfg.lidarZmax, 100.0, 1e-9);
    EXPECT_NEAR(cfg.lidarD, 3.0, 1e-9);
    EXPECT_EQ  (cfg.lidarFOVC, 3);
}

// Missing keys → defaults are used, errors are logged
TEST(DroneConfig, MissingKeys_UsesDefaults)
{
    // Completely empty file
    DroneConfig cfg;
    std::string errors;
    parseDroneConfig(writeTempFile(""), cfg, errors);

    // Defaults should be in place
    EXPECT_DISTANCE_NEAR(cfg.width,      30.0, 1e-9);
    EXPECT_DISTANCE_NEAR(cfg.lidarZmin,  20.0, 1e-9);
    EXPECT_DISTANCE_NEAR(cfg.lidarZmax, 120.0, 1e-9);
    EXPECT_EQ(cfg.lidarFOVC, 5);

    // Errors should mention missing keys
    EXPECT_FALSE(errors.empty());
}

// Bad numeric value → default used, error logged
TEST(DroneConfig, BadValue_UsesDefault)
{
    const std::string content = "width=not_a_number\n";
    DroneConfig cfg;
    std::string errors;
    parseDroneConfig(writeTempFile(content), cfg, errors);

    EXPECT_DISTANCE_NEAR(cfg.width, 30.0, 1e-9);  // default
    EXPECT_NE(errors.find("width"), std::string::npos);
}

// Inline comment after value is ignored
TEST(DroneConfig, InlineComment_Ignored)
{
    const std::string content = "width=42 # this is a comment\n";
    DroneConfig cfg;
    std::string errors;
    parseDroneConfig(writeTempFile(content), cfg, errors);

    EXPECT_DISTANCE_NEAR(cfg.width, 42.0, 1e-9);
}

// File does not exist → all defaults, file error logged
TEST(DroneConfig, FileNotFound_UsesDefaults)
{
    DroneConfig cfg;
    std::string errors;
    parseDroneConfig("/tmp/this_file_does_not_exist_abc123.txt", cfg, errors);

    EXPECT_DISTANCE_NEAR(cfg.width, 30.0, 1e-9);  // default
    EXPECT_FALSE(errors.empty());
}

// lidar_zmax <= lidar_zmin → values are swapped, error logged
TEST(DroneConfig, LidarRange_SwappedIfInverted)
{
    const std::string content =
        "lidar_zmin=100\n"
        "lidar_zmax=20\n";
    DroneConfig cfg;
    std::string errors;
    parseDroneConfig(writeTempFile(content), cfg, errors);

    // After swap: zmin=20, zmax=100
    EXPECT_LT(cfg.lidarZmin.numerical_value_in(cm),
              cfg.lidarZmax.numerical_value_in(cm));
    EXPECT_FALSE(errors.empty());
}

// lidar_fovc < 1 → clamped to 1
TEST(DroneConfig, FOVC_ClampedToOne)
{
    DroneConfig cfg;
    std::string errors;
    parseDroneConfig(writeTempFile("lidar_fovc=0\n"), cfg, errors);
    EXPECT_EQ(cfg.lidarFOVC, 1);
}

// ============================================================
// ConfigParser — MissionConfig tests
// ============================================================

TEST(MissionConfig, ValidFile_AllKeysPresent)
{
    const std::string content =
        "min_x=-100\n"
        "max_x=100\n"
        "min_y=-50\n"
        "max_y=50\n"
        "min_height=5\n"
        "max_height=200\n"
        "res_x=1\n"
        "res_y=0\n"
        "res_height=1\n"
        "start_x=10\n"
        "start_y=20\n"
        "start_z=30\n";

    MissionConfig cfg;
    std::string errors;
    EXPECT_TRUE(parseMissionConfig(writeTempFile(content), cfg, errors));
    EXPECT_TRUE(errors.empty()) << errors;

    EXPECT_NEAR(cfg.minX, -100.0, 1e-9);
    EXPECT_NEAR(cfg.maxX,  100.0, 1e-9);
    EXPECT_NEAR(cfg.minY,  -50.0, 1e-9);
    EXPECT_NEAR(cfg.maxY,   50.0, 1e-9);
    EXPECT_NEAR(cfg.minHeight,  5.0, 1e-9);
    EXPECT_NEAR(cfg.maxHeight, 200.0, 1e-9);

    EXPECT_EQ(cfg.resX, 1);
    EXPECT_EQ(cfg.resY, 0);
    EXPECT_EQ(cfg.resHeight, 1);

    // Steps: res=1 → 0.1 cm, res=0 → 1.0 cm
    EXPECT_NEAR(cfg.stepX, 0.1, 1e-9);
    EXPECT_NEAR(cfg.stepY, 1.0, 1e-9);
    EXPECT_NEAR(cfg.stepZ, 0.1, 1e-9);

    // Starting position parsed
    EXPECT_TRUE(cfg.startSet);
    EXPECT_NEAR(cfg.startX, 10.0, 1e-9);
    EXPECT_NEAR(cfg.startY, 20.0, 1e-9);
    EXPECT_NEAR(cfg.startZ, 30.0, 1e-9);
}

// start_x/y/z not present → startSet is false
TEST(MissionConfig, NoStartPosition_StartSetFalse)
{
    MissionConfig cfg;
    std::string errors;
    parseMissionConfig(writeTempFile("min_x=0\nmax_x=100\n"), cfg, errors);
    EXPECT_FALSE(cfg.startSet);
}

// Partial start (only start_x and start_y, no start_z) → startSet false
TEST(MissionConfig, PartialStartPosition_StartSetFalse)
{
    MissionConfig cfg;
    std::string errors;
    parseMissionConfig(writeTempFile("start_x=5\nstart_y=5\n"), cfg, errors);
    EXPECT_FALSE(cfg.startSet);  // all three required
}

// Negative resolution → clamped to 0
TEST(MissionConfig, NegativeResolution_ClampedToZero)
{
    MissionConfig cfg;
    std::string errors;
    parseMissionConfig(writeTempFile("res_x=-1\nres_y=-2\nres_height=-3\n"), cfg, errors);
    EXPECT_EQ(cfg.resX, 0);
    EXPECT_EQ(cfg.resY, 0);
    EXPECT_EQ(cfg.resHeight, 0);
    EXPECT_FALSE(errors.empty());
}

// Windows-style \r\n line endings are handled
TEST(MissionConfig, WindowsLineEndings)
{
    const std::string content = "min_x=-200\r\nmax_x=200\r\n";
    MissionConfig cfg;
    std::string errors;
    parseMissionConfig(writeTempFile(content), cfg, errors);
    EXPECT_NEAR(cfg.minX, -200.0, 1e-9);
    EXPECT_NEAR(cfg.maxX,  200.0, 1e-9);
}

// ============================================================
// toGrid tests
// ============================================================

TEST(ToGrid, StepOne_RoundsCorrectly)
{
    MissionConfig mc = makeDefaultMission();  // stepX=stepY=stepZ=1

    GridPoint gp = toGrid(10.0, 20.0, 30.0, mc);
    EXPECT_EQ(gp.x, 10);
    EXPECT_EQ(gp.y, 20);
    EXPECT_EQ(gp.z, 30);
}

TEST(ToGrid, StepOne_RoundingHalfUp)
{
    MissionConfig mc = makeDefaultMission();

    GridPoint gp = toGrid(10.5, 20.5, 30.5, mc);
    // std::round uses round-half-away-from-zero
    EXPECT_EQ(gp.x, 11);
    EXPECT_EQ(gp.y, 21);
    EXPECT_EQ(gp.z, 31);
}

TEST(ToGrid, FractionalStep)
{
    MissionConfig mc;
    mc.resX = 1; mc.resY = 1; mc.resHeight = 1;
    mc.computeSteps();
    // stepX = stepY = stepZ = 0.1

    // 0.15 / 0.1 = 1.5 → rounds to 2
    GridPoint gp = toGrid(0.15, 0.25, 0.35, mc);
    EXPECT_EQ(gp.x, 2);
    EXPECT_EQ(gp.y, 3);  // 0.25/0.1=2.5→3
    EXPECT_EQ(gp.z, 4);  // 0.35/0.1=3.5→4
}

TEST(ToGrid, NegativeCoordinates)
{
    MissionConfig mc = makeDefaultMission();
    mc.minX = -100.0;
    mc.computeSteps();

    GridPoint gp = toGrid(-50.0, 0.0, 0.0, mc);
    EXPECT_EQ(gp.x, -50);
    EXPECT_EQ(gp.y,   0);
    EXPECT_EQ(gp.z,   0);
}

// GridPoint equality and hash
TEST(GridPoint, EqualityAndHash)
{
    GridPoint a{1, 2, 3};
    GridPoint b{1, 2, 3};
    GridPoint c{1, 2, 4};

    EXPECT_EQ(a, b);
    EXPECT_NE(a, c);

    GridPointHash hasher;
    EXPECT_EQ(hasher(a), hasher(b));
    // Different points should (usually) have different hashes
    EXPECT_NE(hasher(a), hasher(c));
}
