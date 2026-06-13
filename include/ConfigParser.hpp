#pragma once

// ============================================================
// ConfigParser.hpp — Configuration structures and file parser.
//
// File formats (key=value, one per line, '#' = comment):
//
// drone_config.txt:
//   width=30          # drone body width  (cm)
//   length=30         # drone body length (cm)
//   height=10         # drone body height (cm)
//   max_rotate=90     # max rotation per command (degrees)
//   max_advance=50    # max advance per command  (cm)
//   max_elevate=30    # max elevate per command  (cm)
//   lidar_zmin=20     # min measurable distance  (cm)
//   lidar_zmax=120    # max operational range    (cm)
//   lidar_d=2.5       # inter-circle spacing at zmin (cm)
//   lidar_fovc=5      # number of beam circles (1 = central only)
//
// mission_config.txt:
//   min_x=-500        # mapping boundary (cm)
//   max_x=500
//   min_y=-500
//   max_y=500
//   min_height=0
//   max_height=300
//   res_x=0           # resolution: decimal places (0=1cm, 1=0.1cm)
//   res_y=0
//   res_height=0
//   start_x=0         # initial drone position (cm); default = map centre
//   start_y=0
//   start_z=5         # default = min_height + drone_height/2 + 1
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
    Distance width      = 30.0 * cm;
    Distance length     = 30.0 * cm;
    Distance height     = 10.0 * cm;

    Angle    maxRotate  = 90.0 * deg;
    Distance maxAdvance = 50.0 * cm;
    Distance maxElevate = 30.0 * cm;

    Distance lidarZmin  = 20.0 * cm;
    Distance lidarZmax  = 120.0 * cm;
    double   lidarD     = 2.5;   // beam-circle spacing at Zmin (cm)
    int      lidarFOVC  = 5;     // number of beam circles (incl. circle 0)
};

// ============================================================
// MissionConfig — spatial bounds, resolution and start pose.
// ============================================================
struct MissionConfig {
    // Mission boundaries (cm)
    double minX      = -500.0;
    double maxX      =  500.0;
    double minY      = -500.0;
    double maxY      =  500.0;
    double minHeight =    0.0;
    double maxHeight =  300.0;

    // Resolution: number of decimal places.
    //   0  -> grid step = 1 cm
    //   1  -> grid step = 0.1 cm  etc.
    int resX      = 0;
    int resY      = 0;
    int resHeight = 0;

    // Computed grid step sizes (cm). Call computeSteps() after loading.
    double stepX = 1.0;
    double stepY = 1.0;
    double stepZ = 1.0;

    // Starting position. Use NaN as sentinel: if the value is NaN
    // the Simulator will compute a sensible default automatically.
    double startX = 0.0;
    double startY = 0.0;
    double startZ = 0.0;   // overridden if not set by user
    bool   startSet = false; // true if start_x/y/z were read from file

    void computeSteps() noexcept {
        auto decToStep = [](int dec) noexcept -> double {
            double s = 1.0;
            for (int i = 0; i < dec; ++i) s /= 10.0;
            return s;
        };
        stepX = decToStep(resX);
        stepY = decToStep(resY);
        stepZ = decToStep(resHeight);
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
