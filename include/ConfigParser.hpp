#pragma once

// ============================================================
// ConfigParser.hpp — Configuration structures and file parser.
//
// File formats: YAML
//
// drone_config.yaml:
//   drone_config:
//     dimensions_cm: 30      # sphere diameter the drone can pass through
//     max_rotate_deg: 45     # max rotation per command (degrees)
//     max_advance_cm: 50     # max advance distance per command (cm)
//     max_elevate_cm: 40     # max elevate distance per command (cm)
//     # Lidar parameters (optional, kept for simulation fidelity):
//     lidar_zmin_cm: 20
//     lidar_zmax_cm: 120
//     lidar_d: 2.5
//     lidar_fovc: 5
//
// mission_config.yaml:
//   mission_config:
//     max_steps: 2400
//     boundaries:
//       x_boundary:
//         min_cm: -500
//         max_cm: 500
//       y_boundary:
//         min_cm: -500
//         max_cm: 500
//       height_boundary:
//         min_cm: 0
//         max_cm: 300
//     gps_resolution_cm: 10        # GPS expected measurement precision
//     mapping_resolution_factor: 1 # optional integer; defaults to 1
//     # Optional starting position:
//     start_x_cm: 0
//     start_y_cm: 0
//     start_z_cm: 5
//
// Grid step size = gps_resolution_cm / mapping_resolution_factor
//
// Error handling: all missing or bad values fall back to the
// defaults shown above. Errors are appended to errorsOut.
// ============================================================

#include "Types.hpp"
#include <string>

namespace dm {

// ============================================================
// DroneConfig — physical capabilities of the drone.
// ============================================================
struct DroneConfig {
    // Bounding sphere diameter: the drone can pass through any
    // gap of at least this diameter (cm).
    Distance dimensions = 30.0 * cm;

    Angle    maxRotate  = 45.0 * deg;
    Distance maxAdvance = 50.0 * cm;
    Distance maxElevate = 40.0 * cm;

    // Lidar sensor parameters
    Distance lidarZmin  = 20.0 * cm;
    Distance lidarZmax  = 120.0 * cm;
    double   lidarD     = 2.5;   // beam-circle spacing at Zmin (cm)
    int      lidarFOVC  = 5;     // number of beam circles (incl. circle 0)
};

// ============================================================
// MissionConfig — spatial bounds, resolution and start pose.
// ============================================================
struct MissionConfig {
    // Maximum number of drone steps before the simulation stops.
    int maxSteps = 2400;

    // Mission boundaries (cm)
    double minX      = -500.0;
    double maxX      =  500.0;
    double minY      = -500.0;
    double maxY      =  500.0;
    double minHeight =    0.0;
    double maxHeight =  300.0;

    // GPS measurement precision (cm).
    double gpsResolutionCm = 10.0;

    // Optional integer factor: map resolution = gps_resolution / factor.
    // Defaults to 1 (map resolution == GPS resolution).
    int mappingResolutionFactor = 1;

    // Computed uniform grid step size (cm). Call computeSteps() after loading.
    double stepX = 10.0;
    double stepY = 10.0;
    double stepZ = 10.0;

    // Starting position. startSet = false means the Simulator will compute
    // a sensible default automatically.
    double startX   = 0.0;
    double startY   = 0.0;
    double startZ   = 0.0;
    bool   startSet = false;

    void computeSteps() noexcept {
        const double factor = (mappingResolutionFactor > 0)
            ? static_cast<double>(mappingResolutionFactor) : 1.0;
        stepX = stepY = stepZ = gpsResolutionCm / factor;
    }
};

// ============================================================
// Parsing functions.
// Return true even on recoverable errors (defaults used).
// Return false only if the file could not be opened at all
// AND no previous values are available — in practice always
// returns true (all errors are recoverable via defaults).
// ============================================================
bool parseDroneConfig(const std::string& filename,
                      DroneConfig& cfg,
                      std::string& errorsOut);

bool parseMissionConfig(const std::string& filename,
                        MissionConfig& cfg,
                        std::string& errorsOut);

// Convert continuous world coordinates to a grid index using
// the mission resolution step sizes.
GridPoint toGrid(double x, double y, double z,
                 const MissionConfig& mc) noexcept;

} // namespace dm
