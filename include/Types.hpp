#pragma once

// ============================================================
// Types.hpp — Core types and units for the DroneMapper project.
//
// Design decisions:
//   - All physical quantities use the mp-units library (C++23 style)
//     to prevent unit-mismatch bugs at compile time.
//   - Distances are in centimetres, angles in degrees.
//   - GridPoint uses integer indices derived from the mission
//     resolution, so all map look-ups are exact (no floating-point
//     key comparison needed).
//   - ScanResult stores the world-space beam direction so that
//     the Drone does not need to know anything about how the Lidar
//     computes directions — it just traces along (dx, dy, dz).
// ============================================================

#include <mp-units/systems/si.h>
#include <mp-units/systems/angular.h>
#include <mp-units/framework.h>
#include <cstddef>
#include <functional>

namespace dm {

// -- Unit aliases (used everywhere in the project) --
using namespace mp_units;
using namespace mp_units::si::unit_symbols;

// Strong quantity types
using Distance = mp_units::quantity<mp_units::si::centi<mp_units::si::metre>, double>;
using Angle    = mp_units::quantity<mp_units::angular::degree, double>;

// ============================================================
// GridPoint — 3D integer voxel index.
//
// Continuous coordinates are converted to grid indices by
// dividing by the mission step size and rounding (see toGrid()
// in ConfigParser). Using integers as map keys gives O(1)
// average look-up without floating-point precision issues.
// ============================================================
struct GridPoint {
    int x = 0;
    int y = 0;
    int z = 0;

    bool operator==(const GridPoint& o) const noexcept {
        return x == o.x && y == o.y && z == o.z;
    }
};

// Hash for GridPoint so it can be used in unordered containers.
// Uses FNV-inspired mixing to spread nearby integer triples well.
struct GridPointHash {
    std::size_t operator()(const GridPoint& p) const noexcept {
        std::size_t h = 0;
        // Combine three independent hashes with large prime offsets
        h ^= std::hash<int>{}(p.x) + 0x9e3779b9u + (h << 6) + (h >> 2);
        h ^= std::hash<int>{}(p.y) + 0x9e3779b9u + (h << 6) + (h >> 2);
        h ^= std::hash<int>{}(p.z) + 0x9e3779b9u + (h << 6) + (h >> 2);
        return h;
    }
};

// ============================================================
// Position3D — continuous 3D position in centimetres.
// ============================================================
struct Position3D {
    Distance x{0.0 * cm};
    Distance y{0.0 * cm};
    Distance z{0.0 * cm};
};

// ============================================================
// CellStatus — values stored in the sparse building map.
//
//   Empty       ( 0): scanned, confirmed open
//   Occupied    ( 1): scanned, confirmed solid
//   UnmappedNA  (-1): within bounds but not reachable/mapped
//   UnmappedOOB (-2): outside the mission boundary
// ============================================================
enum class CellStatus : int {
    Empty       =  0,
    Occupied    =  1,
    UnmappedNA  = -1,
    UnmappedOOB = -2
};

// ============================================================
// ScanResult — result for one Lidar beam.
//
// Storing the world-space direction (dx, dy, dz) rather than
// local azimuth/elevation decouples the Drone from knowing
// how the Lidar constructs its FOV cone.  The Drone simply
// marches along (dx, dy, dz) to update its internal map.
//
// distance:
//   0      -> hit detected but closer than Z-min (too close
//             to measure accurately; treat immediate cell as occupied)
//   Z-max  -> no obstacle found within operational range
//   other  -> distance to the detected obstacle
// ============================================================
struct ScanResult {
    double   dx       = 0.0;   // World-space unit direction X
    double   dy       = 0.0;   // World-space unit direction Y
    double   dz       = 0.0;   // World-space unit direction Z
    Distance distance = 0.0 * cm;
};

} // namespace dm
