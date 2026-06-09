#include "test_helpers.hpp"
#include "ConfigParser.hpp"

#include <gtest/gtest.h>
#include <cmath>

using namespace dm;
using namespace dm::test;

// ============================================================
// ConfigParser — DroneConfig tests (YAML format)
// ============================================================

// All valid keys present → values parsed correctly, no errors
TEST(DroneConfig, ValidFile_AllKeysPresent)
{
    const std::string content =
        "drone_config:\n"
        "  dimensions_cm: 25\n"
        "  max_rotate_deg: 45\n"
        "  max_advance_cm: 20\n"
        "  max_elevate_cm: 15\n"
        "  lidar_zmin_cm: 10\n"
        "  lidar_zmax_cm: 100\n"
        "  lidar_d: 3.0\n"
        "  lidar_fovc: 3\n";

    DroneConfig cfg;
    std::string errors;
    EXPECT_TRUE(parseDroneConfig(writeTempFile(content, ".yaml"), cfg, errors));
    EXPECT_TRUE(errors.empty()) << "Unexpected errors: " << errors;

    EXPECT_DISTANCE_NEAR(cfg.dimensions,  25.0, 1e-9);
    EXPECT_ANGLE_NEAR   (cfg.maxRotate,   45.0, 1e-9);
    EXPECT_DISTANCE_NEAR(cfg.maxAdvance,  20.0, 1e-9);
    EXPECT_DISTANCE_NEAR(cfg.maxElevate,  15.0, 1e-9);
    EXPECT_DISTANCE_NEAR(cfg.lidarZmin,   10.0, 1e-9);
    EXPECT_DISTANCE_NEAR(cfg.lidarZmax,  100.0, 1e-9);
    EXPECT_NEAR(cfg.lidarD,   3.0, 1e-9);
    EXPECT_EQ  (cfg.lidarFOVC, 3);
}

// Missing drone_config root key → defaults used, errors logged
TEST(DroneConfig, MissingRootKey_UsesDefaults)
{
    // Empty file (no drone_config key)
    DroneConfig cfg;
    std::string errors;
    parseDroneConfig(writeTempFile("", ".yaml"), cfg, errors);

    // Defaults should be in place
    EXPECT_DISTANCE_NEAR(cfg.dimensions, 30.0, 1e-9);
    EXPECT_DISTANCE_NEAR(cfg.lidarZmin,  20.0, 1e-9);
    EXPECT_DISTANCE_NEAR(cfg.lidarZmax, 120.0, 1e-9);
    EXPECT_EQ(cfg.lidarFOVC, 5);

    // Errors should be logged
    EXPECT_FALSE(errors.empty());
}

// Missing individual keys → defaults used, errors logged
TEST(DroneConfig, MissingKeys_UsesDefaults)
{
    const std::string content =
        "drone_config:\n"
        "  dimensions_cm: 20\n";
        // max_rotate_deg etc. omitted → defaults

    DroneConfig cfg;
    std::string errors;
    parseDroneConfig(writeTempFile(content, ".yaml"), cfg, errors);

    EXPECT_DISTANCE_NEAR(cfg.dimensions, 20.0, 1e-9);
    EXPECT_ANGLE_NEAR   (cfg.maxRotate,  45.0, 1e-9);  // default
    EXPECT_DISTANCE_NEAR(cfg.maxAdvance, 50.0, 1e-9);  // default
    EXPECT_FALSE(errors.empty());
}

// Bad numeric value → default used, error logged
TEST(DroneConfig, BadValue_UsesDefault)
{
    const std::string content =
        "drone_config:\n"
        "  dimensions_cm: not_a_number\n";
    DroneConfig cfg;
    std::string errors;
    parseDroneConfig(writeTempFile(content, ".yaml"), cfg, errors);

    EXPECT_DISTANCE_NEAR(cfg.dimensions, 30.0, 1e-9);  // default
    EXPECT_NE(errors.find("dimensions_cm"), std::string::npos);
}

// File does not exist → all defaults, file error logged
TEST(DroneConfig, FileNotFound_UsesDefaults)
{
    DroneConfig cfg;
    std::string errors;
    parseDroneConfig("this_file_does_not_exist_abc123.yaml", cfg, errors);

    EXPECT_DISTANCE_NEAR(cfg.dimensions, 30.0, 1e-9);  // default
    EXPECT_FALSE(errors.empty());
}

// lidar_zmax <= lidar_zmin → values are swapped, error logged
TEST(DroneConfig, LidarRange_SwappedIfInverted)
{
    const std::string content =
        "drone_config:\n"
        "  dimensions_cm: 30\n"
        "  max_rotate_deg: 45\n"
        "  max_advance_cm: 50\n"
        "  max_elevate_cm: 40\n"
        "  lidar_zmin_cm: 100\n"
        "  lidar_zmax_cm: 20\n";
    DroneConfig cfg;
    std::string errors;
    parseDroneConfig(writeTempFile(content, ".yaml"), cfg, errors);

    // After swap: zmin < zmax
    EXPECT_LT(cfg.lidarZmin.numerical_value_in(cm),
              cfg.lidarZmax.numerical_value_in(cm));
    EXPECT_FALSE(errors.empty());
}

// lidar_fovc < 1 → clamped to 1
TEST(DroneConfig, FOVC_ClampedToOne)
{
    const std::string content =
        "drone_config:\n"
        "  dimensions_cm: 30\n"
        "  max_rotate_deg: 45\n"
        "  max_advance_cm: 50\n"
        "  max_elevate_cm: 40\n"
        "  lidar_fovc: 0\n";
    DroneConfig cfg;
    std::string errors;
    parseDroneConfig(writeTempFile(content, ".yaml"), cfg, errors);
    EXPECT_EQ(cfg.lidarFOVC, 1);
}

// ============================================================
// ConfigParser — MissionConfig tests (YAML format)
// ============================================================

TEST(MissionConfig, ValidFile_AllKeysPresent)
{
    const std::string content =
        "mission_config:\n"
        "  max_steps: 1200\n"
        "  boundaries:\n"
        "    x_boundary:\n"
        "      min_cm: -100\n"
        "      max_cm: 100\n"
        "    y_boundary:\n"
        "      min_cm: -50\n"
        "      max_cm: 50\n"
        "    height_boundary:\n"
        "      min_cm: 5\n"
        "      max_cm: 200\n"
        "  gps_resolution_cm: 10\n"
        "  mapping_resolution_factor: 2\n"
        "  start_x_cm: 10\n"
        "  start_y_cm: 20\n"
        "  start_z_cm: 30\n";

    MissionConfig cfg;
    std::string errors;
    EXPECT_TRUE(parseMissionConfig(writeTempFile(content, ".yaml"), cfg, errors));
    EXPECT_TRUE(errors.empty()) << errors;

    EXPECT_EQ  (cfg.maxSteps, 1200);
    EXPECT_NEAR(cfg.minX,    -100.0, 1e-9);
    EXPECT_NEAR(cfg.maxX,     100.0, 1e-9);
    EXPECT_NEAR(cfg.minY,     -50.0, 1e-9);
    EXPECT_NEAR(cfg.maxY,      50.0, 1e-9);
    EXPECT_NEAR(cfg.minHeight,  5.0, 1e-9);
    EXPECT_NEAR(cfg.maxHeight, 200.0, 1e-9);

    EXPECT_NEAR(cfg.gpsResolutionCm, 10.0, 1e-9);
    EXPECT_EQ  (cfg.mappingResolutionFactor, 2);

    // step = 10 / 2 = 5 cm
    EXPECT_NEAR(cfg.stepX, 5.0, 1e-9);
    EXPECT_NEAR(cfg.stepY, 5.0, 1e-9);
    EXPECT_NEAR(cfg.stepZ, 5.0, 1e-9);

    // Starting position parsed
    EXPECT_TRUE(cfg.startSet);
    EXPECT_NEAR(cfg.startX, 10.0, 1e-9);
    EXPECT_NEAR(cfg.startY, 20.0, 1e-9);
    EXPECT_NEAR(cfg.startZ, 30.0, 1e-9);
}

// mapping_resolution_factor absent → defaults to 1
TEST(MissionConfig, MappingFactorAbsent_DefaultsToOne)
{
    const std::string content =
        "mission_config:\n"
        "  max_steps: 2400\n"
        "  boundaries:\n"
        "    x_boundary:\n"
        "      min_cm: 0\n"
        "      max_cm: 100\n"
        "    y_boundary:\n"
        "      min_cm: 0\n"
        "      max_cm: 100\n"
        "    height_boundary:\n"
        "      min_cm: 0\n"
        "      max_cm: 50\n"
        "  gps_resolution_cm: 10\n";

    MissionConfig cfg;
    std::string errors;
    parseMissionConfig(writeTempFile(content, ".yaml"), cfg, errors);

    EXPECT_EQ  (cfg.mappingResolutionFactor, 1);
    EXPECT_NEAR(cfg.stepX, 10.0, 1e-9);  // 10 / 1 = 10 cm
}

// start_x/y/z absent → startSet is false
TEST(MissionConfig, NoStartPosition_StartSetFalse)
{
    const std::string content =
        "mission_config:\n"
        "  max_steps: 2400\n"
        "  boundaries:\n"
        "    x_boundary:\n"
        "      min_cm: 0\n"
        "      max_cm: 100\n"
        "    y_boundary:\n"
        "      min_cm: 0\n"
        "      max_cm: 100\n"
        "    height_boundary:\n"
        "      min_cm: 0\n"
        "      max_cm: 50\n"
        "  gps_resolution_cm: 10\n";

    MissionConfig cfg;
    std::string errors;
    parseMissionConfig(writeTempFile(content, ".yaml"), cfg, errors);
    EXPECT_FALSE(cfg.startSet);
}

// Partial start (only start_x_cm and start_y_cm, no start_z_cm) → startSet false
TEST(MissionConfig, PartialStartPosition_StartSetFalse)
{
    const std::string content =
        "mission_config:\n"
        "  max_steps: 2400\n"
        "  boundaries:\n"
        "    x_boundary:\n"
        "      min_cm: 0\n"
        "      max_cm: 100\n"
        "    y_boundary:\n"
        "      min_cm: 0\n"
        "      max_cm: 100\n"
        "    height_boundary:\n"
        "      min_cm: 0\n"
        "      max_cm: 50\n"
        "  gps_resolution_cm: 10\n"
        "  start_x_cm: 5\n"
        "  start_y_cm: 5\n";

    MissionConfig cfg;
    std::string errors;
    parseMissionConfig(writeTempFile(content, ".yaml"), cfg, errors);
    EXPECT_FALSE(cfg.startSet);  // all three required
}

// File does not exist → defaults, error logged
TEST(MissionConfig, FileNotFound_UsesDefaults)
{
    MissionConfig cfg;
    std::string errors;
    parseMissionConfig("no_such_file_xyz.yaml", cfg, errors);
    EXPECT_FALSE(errors.empty());
}

// ============================================================
// toGrid tests
// ============================================================

TEST(ToGrid, StepOne_RoundsCorrectly)
{
    MissionConfig mc = makeDefaultMission();  // step=1 cm

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
    mc.gpsResolutionCm         = 1.0;
    mc.mappingResolutionFactor = 10;  // step = 0.1 cm
    mc.computeSteps();

    // Use exact values to avoid floating-point rounding issues
    // 0.2 / 0.1 = 2.0 exactly
    GridPoint gp = toGrid(0.2, 0.3, 0.4, mc);
    EXPECT_EQ(gp.x, 2);
    EXPECT_EQ(gp.y, 3);
    EXPECT_EQ(gp.z, 4);
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

    EXPECT_EQ(gp.y, 21);
    EXPECT_EQ(gp.z, 31);
}

TEST(ToGrid, FractionalStep)
{
    MissionConfig mc;
    mc.gpsResolutionCm         = 1.0;
    mc.mappingResolutionFactor = 10;  // step = 0.1 cm
    mc.computeSteps();

    // Use exact values to avoid floating-point rounding issues
    // 0.2 / 0.1 = 2.0 exactly
    GridPoint gp = toGrid(0.2, 0.3, 0.4, mc);
    EXPECT_EQ(gp.x, 2);
    EXPECT_EQ(gp.y, 3);
    EXPECT_EQ(gp.z, 4);
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

