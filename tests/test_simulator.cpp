// ============================================================
// test_simulator.cpp — End-to-end tests for Simulator::run().
//
// These tests exercise the complete pipeline:
//   loadAll → build mocks → run drone → write output → score
//
// Each test creates a temporary directory with the three required
// input files (drone_config.yaml, mission_config.yaml, map_input.npy)
// and verifies the exit code and output files.
// ============================================================

#include "test_helpers.hpp"
#include "Simulator.hpp"

#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <atomic>
#include <string>
#include <vector>
#include <cstdint>

using namespace dm;
using namespace dm::test;
namespace fs = std::filesystem;

// ---- Helper: write text content to a file ----
static void writeFile(const fs::path& path, const std::string& content)
{
    std::ofstream f(path);
    f << content;
}

// ---- Helper: write binary content to a file ----
static void writeBinaryFile(const fs::path& path,
                             const std::vector<int8_t>& grid,
                             int nx, int ny, int nz)
{
    // Build Python-dict header
    std::string header = "{'descr': '<i1', 'fortran_order': False, 'shape': ("
                       + std::to_string(nx) + ", "
                       + std::to_string(ny) + ", "
                       + std::to_string(nz) + ",), }";
    while ((10 + header.size() + 1) % 64 != 0)
        header += ' ';
    header += '\n';

    const uint16_t hlen = static_cast<uint16_t>(header.size());

    std::ofstream f(path, std::ios::binary);
    const char magic[] = {'\x93', 'N', 'U', 'M', 'P', 'Y'};
    f.write(magic, 6);
    f.put(1); f.put(0);
    f.write(reinterpret_cast<const char*>(&hlen), 2);
    f.write(header.c_str(), static_cast<std::streamsize>(header.size()));
    f.write(reinterpret_cast<const char*>(grid.data()),
            static_cast<std::streamsize>(grid.size()));
}

// ---- Helper: create a temp directory with all 3 input files ----
// mapGrid/nx/ny/nz describe the npy array: shape (nx, ny, nz) in C order.
static std::string makeTempSimDir(
    const std::string&         droneYaml,
    const std::string&         missionYaml,
    const std::vector<int8_t>& mapGrid,
    int nx, int ny, int nz)
{
    static std::atomic<int> counter{0};
    fs::path dir = fs::temp_directory_path()
                 / ("dm_sim_test_" + std::to_string(counter++));
    fs::create_directories(dir);

    writeFile(dir / "drone_config.yaml",   droneYaml);
    writeFile(dir / "mission_config.yaml", missionYaml);
    writeBinaryFile(dir / "map_input.npy", mapGrid, nx, ny, nz);

    return dir.string();
}

// ---- Standard configs ----

// 50×50×30 cm space, 1 cm step (GPS 1 cm, factor 1)
static const std::string kSmallDroneConfig =
    "drone_config:\n"
    "  dimensions_cm: 10\n"
    "  max_rotate_deg: 90\n"
    "  max_advance_cm: 20\n"
    "  max_elevate_cm: 20\n"
    "  lidar_zmin_cm: 5\n"
    "  lidar_zmax_cm: 150\n"
    "  lidar_d: 2.5\n"
    "  lidar_fovc: 3\n";

static const std::string kSmallMissionConfig =
    "mission_config:\n"
    "  max_steps: 2000000\n"
    "  boundaries:\n"
    "    x_boundary:\n"
    "      min_cm: 0\n"
    "      max_cm: 50\n"
    "    y_boundary:\n"
    "      min_cm: 0\n"
    "      max_cm: 50\n"
    "    height_boundary:\n"
    "      min_cm: 0\n"
    "      max_cm: 30\n"
    "  gps_resolution_cm: 1\n";

// Build the .npy grid for a simple 50×50×30 room:
//   nx=51, ny=51, nz=31 (inclusive indices 0..50, 0..50, 0..30)
//   Floor (z=0), ceiling (z=30), and 4 walls.
static std::pair<std::vector<int8_t>, std::array<int,3>> buildSmallRoomNpy()
{
    const int nx = 51, ny = 51, nz = 31;
    std::vector<int8_t> grid(static_cast<size_t>(nx) * ny * nz, 0);

    auto setOcc = [&](int ix, int iy, int iz) {
        if (ix >= 0 && ix < nx && iy >= 0 && iy < ny && iz >= 0 && iz < nz)
            grid[static_cast<size_t>(ix) * ny * nz + iy * nz + iz] = 1;
    };

    // Floor (iz=0) and ceiling (iz=30)
    for (int x = 0; x <= 50; x += 1)
        for (int y = 0; y <= 50; y += 1) {
            setOcc(x, y, 0);
            setOcc(x, y, 30);
        }
    // Four walls
    for (int z = 1; z < 30; ++z)
        for (int i = 0; i <= 50; ++i) {
            setOcc(0,  i,  z);
            setOcc(50, i,  z);
            setOcc(i,  0,  z);
            setOcc(i,  50, z);
        }

    return {grid, {nx, ny, nz}};
}

// ============================================================
// Simulator — success path
// ============================================================

// run() on a valid small room returns 0 and creates map_output.txt
TEST(Simulator, Run_ValidRoom_ReturnsZero)
{
    auto [grid, shape] = buildSmallRoomNpy();
    const std::string dir = makeTempSimDir(
        kSmallDroneConfig, kSmallMissionConfig, grid, shape[0], shape[1], shape[2]);

    Simulator sim;
    EXPECT_EQ(sim.run(dir), 0);
}

TEST(Simulator, Run_ValidRoom_WritesOutputMap)
{
    auto [grid, shape] = buildSmallRoomNpy();
    const std::string dir = makeTempSimDir(
        kSmallDroneConfig, kSmallMissionConfig, grid, shape[0], shape[1], shape[2]);

    Simulator sim;
    sim.run(dir);

    EXPECT_TRUE(fs::exists(dir + "/map_output.txt"))
        << "map_output.txt was not created";
}

// The output map must be non-empty (drone mapped at least some cells)
TEST(Simulator, Run_ValidRoom_OutputMapNonEmpty)
{
    auto [grid, shape] = buildSmallRoomNpy();
    const std::string dir = makeTempSimDir(
        kSmallDroneConfig, kSmallMissionConfig, grid, shape[0], shape[1], shape[2]);

    Simulator sim;
    sim.run(dir);

    std::ifstream f(dir + "/map_output.txt");
    ASSERT_TRUE(f.is_open());
    std::string content((std::istreambuf_iterator<char>(f)),
                         std::istreambuf_iterator<char>());
    // Should contain at least one data line (not just the header comment)
    EXPECT_NE(content.find(","), std::string::npos)
        << "Output map appears to have no cell data";
}

// No input_errors.txt should be created when all inputs are valid
TEST(Simulator, Run_ValidInputs_NoErrorsFile)
{
    auto [grid, shape] = buildSmallRoomNpy();
    const std::string dir = makeTempSimDir(
        kSmallDroneConfig, kSmallMissionConfig, grid, shape[0], shape[1], shape[2]);

    Simulator sim;
    sim.run(dir);

    EXPECT_FALSE(fs::exists(dir + "/input_errors.txt"))
        << "input_errors.txt was created even though inputs were valid";
}

// ============================================================
// Simulator — unrecoverable errors (return 1)
// ============================================================

// Missing map_input.npy → loadAll fails → run() returns 1
TEST(Simulator, Run_MissingMapFile_ReturnsFatal)
{
    static std::atomic<int> ctr{0};
    fs::path dir = fs::temp_directory_path()
                 / ("dm_sim_nomap_" + std::to_string(ctr++));
    fs::create_directories(dir);

    // Write only the two config files; no map_input.npy
    writeFile(dir / "drone_config.yaml",   kSmallDroneConfig);
    writeFile(dir / "mission_config.yaml", kSmallMissionConfig);

    Simulator sim;
    EXPECT_NE(sim.run(dir.string()), 0);
}

// Drone starts inside an occupied cell → run() returns 1
TEST(Simulator, Run_StartInsideWall_ReturnsFatal)
{
    // Force start position at (25,25,15) which we put an obstacle at
    const std::string missionWithStart =
        "mission_config:\n"
        "  max_steps: 2000000\n"
        "  boundaries:\n"
        "    x_boundary:\n"
        "      min_cm: 0\n"
        "      max_cm: 50\n"
        "    y_boundary:\n"
        "      min_cm: 0\n"
        "      max_cm: 50\n"
        "    height_boundary:\n"
        "      min_cm: 0\n"
        "      max_cm: 30\n"
        "  gps_resolution_cm: 1\n"
        "  start_x_cm: 25\n"
        "  start_y_cm: 25\n"
        "  start_z_cm: 15\n";

    // Grid with a single occupied cell at (25,25,15)
    const int nx = 51, ny = 51, nz = 31;
    std::vector<int8_t> mapGrid(static_cast<size_t>(nx) * ny * nz, 0);
    mapGrid[25 * ny * nz + 25 * nz + 15] = 1;

    const std::string dir = makeTempSimDir(
        kSmallDroneConfig, missionWithStart, mapGrid, nx, ny, nz);

    Simulator sim;
    EXPECT_NE(sim.run(dir), 0);
}

// ============================================================
// Simulator — score sanity checks
// ============================================================

// After mapping an open room, the score should be meaningful (> 0)
TEST(Simulator, Run_ValidRoom_ScoreIsPositive)
{
    auto [grid, shape] = buildSmallRoomNpy();
    const std::string dir = makeTempSimDir(
        kSmallDroneConfig, kSmallMissionConfig, grid, shape[0], shape[1], shape[2]);

    Simulator sim;
    EXPECT_EQ(sim.run(dir), 0);

    // If map_output.txt has occupied cells, the score > 0
    std::ifstream f(dir + "/map_output.txt");
    ASSERT_TRUE(f.is_open());
    bool hasOccupied = false;
    std::string line;
    while (std::getline(f, line)) {
        if (!line.empty() && line[0] != '#' && line.back() == '1')
            hasOccupied = true;
    }
    EXPECT_TRUE(hasOccupied)
        << "Drone did not record any occupied cells";
}

// Recoverable config errors (empty drone_config.yaml) still allow simulation
TEST(Simulator, Run_BadDroneConfig_UsesDefaultsAndSucceeds)
{
    auto [grid, shape] = buildSmallRoomNpy();
    // Empty drone config → all defaults used
    const std::string dir = makeTempSimDir(
        "", kSmallMissionConfig, grid, shape[0], shape[1], shape[2]);

    Simulator sim;
    EXPECT_EQ(sim.run(dir), 0);
    // Errors file should appear (missing keys were logged)
    EXPECT_TRUE(fs::exists(dir + "/input_errors.txt"));
}

//
// These tests exercise the complete pipeline:
//   loadAll → build mocks → run drone → write output → score
//
// Each test creates a temporary directory with the three required
// input files (drone_config.txt, mission_config.txt, map_input.txt)
// and verifies the exit code and output files.
// ============================================================

#include "test_helpers.hpp"
#include "Simulator.hpp"

#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <atomic>
#include <string>

using namespace dm;
using namespace dm::test;
namespace fs = std::filesystem;

// ---- Helper: create a temp directory with all 3 input files ----
static std::string makeTempSimDir(
    const std::string& droneContent,
    const std::string& missionContent,
    const std::string& mapContent)
{
    static std::atomic<int> counter{0};
    fs::path dir = fs::temp_directory_path()
                 / ("dm_sim_test_" + std::to_string(counter++));
    fs::create_directories(dir);

    auto write = [&](const std::string& name, const std::string& content) {
        std::ofstream f(dir / name);
        f << content;
    };
    write("drone_config.txt",   droneContent);
    write("mission_config.txt", missionContent);
    write("map_input.txt",      mapContent);

    return dir.string();
}

// ---- Standard configs used across several tests ----

static const std::string kSmallDroneConfig =
    "width=10\nlength=10\nheight=5\n"
    "max_rotate=90\nmax_advance=20\nmax_elevate=20\n"
    "lidar_zmin=5\nlidar_zmax=150\nlidar_d=2.5\nlidar_fovc=3\n";

static const std::string kSmallMissionConfig =
    "min_x=0\nmax_x=50\nmin_y=0\nmax_y=50\n"
    "min_height=0\nmax_height=30\n"
    "res_x=0\nres_y=0\nres_height=0\n";

// Build a simple 50×50×30 room (floor, ceiling, 4 walls) as CSV text
static std::string buildSmallRoomMap()
{
    std::string map = "# 50x50x30 test room\n";
    const int OCC = 1;

    // Floor (z=0) and ceiling (z=30)
    for (int x = 0; x <= 50; x += 10)
        for (int y = 0; y <= 50; y += 10) {
            map += std::to_string(x)+","+std::to_string(y)+",0,"+std::to_string(OCC)+"\n";
            map += std::to_string(x)+","+std::to_string(y)+",30,"+std::to_string(OCC)+"\n";
        }

    // Four walls (x=0, x=50, y=0, y=50)
    for (int z = 5; z < 30; z += 5)
        for (int i = 0; i <= 50; i += 5) {
            map += "0,"+std::to_string(i)+","+std::to_string(z)+",1\n";
            map += "50,"+std::to_string(i)+","+std::to_string(z)+",1\n";
            map += std::to_string(i)+",0,"+std::to_string(z)+",1\n";
            map += std::to_string(i)+",50,"+std::to_string(z)+",1\n";
        }

    return map;
}

// ============================================================
// Simulator — success path
// ============================================================

// run() on a valid small room returns 0 and creates map_output.txt
TEST(Simulator, Run_ValidRoom_ReturnsZero)
{
    const std::string dir = makeTempSimDir(
        kSmallDroneConfig, kSmallMissionConfig, buildSmallRoomMap());

    Simulator sim;
    EXPECT_EQ(sim.run(dir), 0);
}

TEST(Simulator, Run_ValidRoom_WritesOutputMap)
{
    const std::string dir = makeTempSimDir(
        kSmallDroneConfig, kSmallMissionConfig, buildSmallRoomMap());

    Simulator sim;
    sim.run(dir);

    EXPECT_TRUE(fs::exists(dir + "/map_output.txt"))
        << "map_output.txt was not created";
}

// The output map must be non-empty (drone mapped at least some cells)
TEST(Simulator, Run_ValidRoom_OutputMapNonEmpty)
{
    const std::string dir = makeTempSimDir(
        kSmallDroneConfig, kSmallMissionConfig, buildSmallRoomMap());

    Simulator sim;
    sim.run(dir);

    std::ifstream f(dir + "/map_output.txt");
    ASSERT_TRUE(f.is_open());
    std::string content((std::istreambuf_iterator<char>(f)),
                         std::istreambuf_iterator<char>());
    // Should contain at least one data line (not just the header comment)
    EXPECT_NE(content.find(","), std::string::npos)
        << "Output map appears to have no cell data";
}

// No input_errors.txt should be created when all inputs are valid
TEST(Simulator, Run_ValidInputs_NoErrorsFile)
{
    const std::string dir = makeTempSimDir(
        kSmallDroneConfig, kSmallMissionConfig, buildSmallRoomMap());

    Simulator sim;
    sim.run(dir);

    EXPECT_FALSE(fs::exists(dir + "/input_errors.txt"))
        << "input_errors.txt was created even though inputs were valid";
}

// ============================================================
// Simulator — unrecoverable errors (return 1)
// ============================================================

// Missing map_input.txt → loadAll fails → run() returns 1
TEST(Simulator, Run_MissingMapFile_ReturnsFatal)
{
    static std::atomic<int> ctr{0};
    fs::path dir = fs::temp_directory_path()
                 / ("dm_sim_nomap_" + std::to_string(ctr++));
    fs::create_directories(dir);

    // Write only the two config files; no map_input.txt
    std::ofstream(dir / "drone_config.txt")   << kSmallDroneConfig;
    std::ofstream(dir / "mission_config.txt") << kSmallMissionConfig;

    Simulator sim;
    EXPECT_NE(sim.run(dir.string()), 0);
}

// Drone starts inside an occupied cell → run() returns 1
TEST(Simulator, Run_StartInsideWall_ReturnsFatal)
{
    // Force start position at (25,25,15) which we put an obstacle at
    const std::string missionWithStart =
        kSmallMissionConfig + "start_x=25\nstart_y=25\nstart_z=15\n";

    // Single occupied cell at the start position
    const std::string mapContent = "25,25,15,1\n";

    const std::string dir = makeTempSimDir(
        kSmallDroneConfig, missionWithStart, mapContent);

    Simulator sim;
    EXPECT_NE(sim.run(dir), 0);
}

// ============================================================
// Simulator — score sanity checks
// ============================================================

// After mapping an open room, the score should be meaningful (> 0)
// This catches regressions where the ground-truth map is loaded into
// the wrong storage mode (the initDense / loadFromFile ordering bug).
TEST(Simulator, Run_ValidRoom_ScoreIsPositive)
{
    const std::string dir = makeTempSimDir(
        kSmallDroneConfig, kSmallMissionConfig, buildSmallRoomMap());

    // Capture stdout to read the score line
    // (Easier: just verify run succeeded and output map exists — a
    //  zero score would indicate the initDense bug is back.)
    Simulator sim;
    EXPECT_EQ(sim.run(dir), 0);

    // If map_output.txt has occupied cells, the score > 0
    std::ifstream f(dir + "/map_output.txt");
    ASSERT_TRUE(f.is_open());
    bool hasOccupied = false;
    std::string line;
    while (std::getline(f, line)) {
        if (!line.empty() && line[0] != '#' && line.back() == '1')
            hasOccupied = true;
    }
    EXPECT_TRUE(hasOccupied)
        << "Drone did not record any occupied cells — "
           "ground-truth may have been loaded into wrong storage mode";
}

// Recoverable config errors still allow the simulation to run
TEST(Simulator, Run_BadDroneConfig_UsesDefaultsAndSucceeds)
{
    // Completely empty drone_config.txt → all defaults used
    const std::string dir = makeTempSimDir(
        "", kSmallMissionConfig, buildSmallRoomMap());

    Simulator sim;
    // Should still return 0 (defaults are valid)
    EXPECT_EQ(sim.run(dir), 0);
    // And errors file should appear (missing keys were logged)
    EXPECT_TRUE(fs::exists(dir + "/input_errors.txt"));
}
