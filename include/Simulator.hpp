#pragma once

// ============================================================
// Simulator.hpp — Top-level orchestrator.
//
// Responsibilities:
//   1. Load drone_config.txt, mission_config.txt, map_input.txt
//   2. Build mock sensor/driver objects (unknown to the Drone)
//   3. Place the drone at its starting position (validated against
//      ground truth to avoid starting inside a wall)
//   4. Run the main loop: call drone.performStep() until done
//   5. Write map_output.txt
//   6. Calculate and print the mapping score
//   7. Write input_errors.txt if any recoverable errors occurred
//
// Error policy:
//   - The program NEVER calls exit() or abort().
//   - Recoverable input errors (bad values, missing keys) produce
//     defaults and are logged to input_errors.txt.
//   - Unrecoverable errors (e.g. map file missing, drone starts
//     inside a wall) print a message to stderr and return non-zero
//     from run(), ending the program via main().
// ============================================================

#include "ConfigParser.hpp"
#include "SparseBuildingMap.hpp"
#include <string>

namespace dm {

class Simulator {
public:
    // Run the complete simulation.
    // Returns 0 on success, 1 on unrecoverable error.
    int run(const std::string& basePath);

private:
    bool loadAll(const std::string& basePath);
    void writeErrorsFile(const std::string& basePath) const;

    DroneConfig       droneCfg_;
    MissionConfig     missionCfg_;
    SparseBuildingMap groundTruth_;
    std::string       errors_;
};

} // namespace dm
