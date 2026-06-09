#pragma once

// ============================================================
// test_helpers.hpp — shared utilities for all GTest files.
//
// Provides:
//   - writeTempFile()       : write a string to a temp file
//   - writeNpyFile()        : write a binary .npy file from a cell list
//   - makeSimpleRoomMap()   : build a small ground-truth map
//   - makeDefaultMission()  : build a standard MissionConfig
//   - makeDefaultDrone()    : build a standard DroneConfig
//   - EXPECT_DISTANCE_NEAR  : compare Distance quantities
//   - EXPECT_ANGLE_NEAR     : compare Angle quantities
// ============================================================

#include "ConfigParser.hpp"
#include "SparseBuildingMap.hpp"
#include "Types.hpp"

#include <gtest/gtest.h>
#include <fstream>
#include <string>
#include <atomic>
#include <filesystem>
#include <vector>
#include <cstdint>
#include <cstring>

namespace dm::test {

// ---- Write a string to a unique temp file and return its path ----
inline std::string writeTempFile(const std::string& content,
                                  const std::string& suffix = ".txt")
{
    static std::atomic<int> counter{0};
    std::filesystem::path dir = std::filesystem::temp_directory_path();
    std::string path = (dir / ("dm_test_" + std::to_string(counter++) + suffix)).string();
    std::ofstream f(path);
    f << content;
    return path;
}

// ---- Write a 3D int8 NumPy .npy file ----
//
// grid: flat C-order array of size nx*ny*nz.
//   grid[ix*ny*nz + iy*nz + iz] = cell value (0=empty, 1=occupied).
// Returns the file path.
inline std::string writeNpyFile(const std::vector<int8_t>& grid,
                                 int nx, int ny, int nz,
                                 const std::string& suffix = ".npy")
{
    static std::atomic<int> counter{0};
    std::filesystem::path dir = std::filesystem::temp_directory_path();
    std::string path = (dir / ("dm_npy_" + std::to_string(counter++) + suffix)).string();

    // Build Python-dict header
    std::string header = "{'descr': '<i1', 'fortran_order': False, 'shape': ("
                       + std::to_string(nx) + ", "
                       + std::to_string(ny) + ", "
                       + std::to_string(nz) + ",), }";

    // Pad header so that (10 + padded_header_len) is a multiple of 64.
    // The 10 fixed bytes are: magic(6) + version(2) + header_len_field(2).
    while ((10 + header.size() + 1) % 64 != 0)
        header += ' ';
    header += '\n';

    const uint16_t hlen = static_cast<uint16_t>(header.size());

    std::ofstream f(path, std::ios::binary);
    // Magic
    const char magic[] = {'\x93', 'N', 'U', 'M', 'P', 'Y'};
    f.write(magic, 6);
    // Version 1.0
    f.put(static_cast<char>(1));
    f.put(static_cast<char>(0));
    // Header length (little-endian uint16)
    f.write(reinterpret_cast<const char*>(&hlen), 2);
    // Header
    f.write(header.c_str(), static_cast<std::streamsize>(header.size()));
    // Data
    f.write(reinterpret_cast<const char*>(grid.data()),
            static_cast<std::streamsize>(grid.size()));
    return path;
}

// ---- Default configs ----

inline DroneConfig makeDefaultDrone()
{
    DroneConfig c;
    c.dimensions = 10.0 * cm;  // sphere diameter; half-extent=5 cm
    c.maxRotate  = 90.0 * deg;
    c.maxAdvance = 20.0 * cm;
    c.maxElevate = 20.0 * cm;
    c.lidarZmin  = 5.0  * cm;
    c.lidarZmax  = 80.0 * cm;
    c.lidarD     = 2.5;
    c.lidarFOVC  = 3;   // 1+4+16 = 21 beams (fast for tests)
    return c;
}

inline MissionConfig makeDefaultMission()
{
    MissionConfig m;
    m.minX      =   0.0;
    m.maxX      = 100.0;
    m.minY      =   0.0;
    m.maxY      = 100.0;
    m.minHeight =   0.0;
    m.maxHeight =  50.0;
    m.gpsResolutionCm         = 1.0;  // 1 cm step (same precision as legacy res=0)
    m.mappingResolutionFactor = 1;
    m.maxSteps                = 2'000'000;  // large limit for tests
    m.computeSteps();
    return m;
}

// ---- Build a 100×100×50 room map ----
//
// Layout (all in grid-index / cm units, stepX=stepY=stepZ=1):
//   Floor:   z=0,  x=0..100, y=0..100 (sparse representative points)
//   Ceiling: z=50, sparse
//   Walls: x=0, x=100, y=0, y=100 at various z levels
//   Pillar: x=40..50, y=40..50, z=0..49
//
// The room interior is empty.  The drone starts at (50,50,10).
inline SparseBuildingMap makeSimpleRoomMap()
{
    SparseBuildingMap map;
    const int OCC = static_cast<int>(CellStatus::Occupied);

    // Floor (z=0) — sample every 10 cm
    for (int x = 0; x <= 100; x += 10)
        for (int y = 0; y <= 100; y += 10)
            map.setCell({x, y, 0}, OCC);

    // Ceiling (z=50)
    for (int x = 0; x <= 100; x += 10)
        for (int y = 0; y <= 100; y += 10)
            map.setCell({x, y, 50}, OCC);

    // West wall (x=0)
    for (int y = 0; y <= 100; y += 10)
        for (int z = 1; z < 50; z += 5)
            map.setCell({0, y, z}, OCC);

    // East wall (x=100)
    for (int y = 0; y <= 100; y += 10)
        for (int z = 1; z < 50; z += 5)
            map.setCell({100, y, z}, OCC);

    // South wall (y=0)
    for (int x = 0; x <= 100; x += 10)
        for (int z = 1; z < 50; z += 5)
            map.setCell({x, 0, z}, OCC);

    // North wall (y=100)
    for (int x = 0; x <= 100; x += 10)
        for (int z = 1; z < 50; z += 5)
            map.setCell({x, 100, z}, OCC);

    // Internal pillar
    for (int x = 40; x <= 50; x += 5)
        for (int y = 40; y <= 50; y += 5)
            for (int z = 0; z < 50; z += 5)
                map.setCell({x, y, z}, OCC);

    return map;
}

} // namespace dm::test

// ---- Custom EXPECT macros for strong types ----

#define EXPECT_DISTANCE_NEAR(val, expected_cm, tol) \
    EXPECT_NEAR((val).numerical_value_in(dm::cm), (expected_cm), (tol))

#define EXPECT_ANGLE_NEAR(val, expected_deg, tol) \
    EXPECT_NEAR((val).numerical_value_in(dm::deg), (expected_deg), (tol))


// ---- Build a 100×100×50 room map ----
//
// Layout (all in grid-index / cm units, stepX=stepY=stepZ=1):
//   Floor:   z=0,  x=0..100, y=0..100 (sparse representative points)
//   Ceiling: z=50, sparse
//   Walls: x=0, x=100, y=0, y=100 at various z levels
//   Pillar: x=40..50, y=40..50, z=0..49
//
// The room interior is empty.  The drone starts at (50,50,10).
inline SparseBuildingMap makeSimpleRoomMap()
{
    SparseBuildingMap map;
    const int OCC = static_cast<int>(CellStatus::Occupied);

    // Floor (z=0) — sample every 10 cm
    for (int x = 0; x <= 100; x += 10)
        for (int y = 0; y <= 100; y += 10)
            map.setCell({x, y, 0}, OCC);

    // Ceiling (z=50)
    for (int x = 0; x <= 100; x += 10)
        for (int y = 0; y <= 100; y += 10)
            map.setCell({x, y, 50}, OCC);

    // West wall (x=0)
    for (int y = 0; y <= 100; y += 10)
        for (int z = 1; z < 50; z += 5)
            map.setCell({0, y, z}, OCC);

    // East wall (x=100)
    for (int y = 0; y <= 100; y += 10)
        for (int z = 1; z < 50; z += 5)
            map.setCell({100, y, z}, OCC);

    // South wall (y=0)
    for (int x = 0; x <= 100; x += 10)
        for (int z = 1; z < 50; z += 5)
            map.setCell({x, 0, z}, OCC);

    // North wall (y=100)
    for (int x = 0; x <= 100; x += 10)
        for (int z = 1; z < 50; z += 5)
            map.setCell({x, 100, z}, OCC);

    // Internal pillar
    for (int x = 40; x <= 50; x += 5)
        for (int y = 40; y <= 50; y += 5)
            for (int z = 0; z < 50; z += 5)
                map.setCell({x, y, z}, OCC);

    return map;
}

} // namespace dm::test

// ---- Custom EXPECT macros for strong types ----

#define EXPECT_DISTANCE_NEAR(val, expected_cm, tol) \
    EXPECT_NEAR((val).numerical_value_in(dm::cm), (expected_cm), (tol))

#define EXPECT_ANGLE_NEAR(val, expected_deg, tol) \
    EXPECT_NEAR((val).numerical_value_in(dm::deg), (expected_deg), (tol))
