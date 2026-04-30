#include "Simulator.hpp"
#include "Drone.hpp"
#include "PositionSensorMock.hpp"
#include "MovementDriverMock.hpp"
#include "LidarMock.hpp"
#include <iostream>
#include <fstream>

namespace dm {

// ============================================================
// loadAll — load all three input files.
//
// Parsing errors are recoverable (defaults used) and accumulated
// in errors_.  Only returns false if the ground-truth map file
// cannot be opened at all (unrecoverable).
// ============================================================
bool Simulator::loadAll(const std::string& basePath)
{
    parseDroneConfig  (basePath + "/drone_config.txt",   droneCfg_,   errors_);
    parseMissionConfig(basePath + "/mission_config.txt", missionCfg_, errors_);

    if (!groundTruth_.loadFromFile(basePath + "/map_input.txt", errors_)) {
        std::cerr << "[Simulator] FATAL: cannot load ground-truth map ("
                  << basePath << "/map_input.txt)\n";
        return false;
    }

    if (groundTruth_.data().empty()) {
        errors_ += "[Simulator] Warning: ground-truth map is empty.\n";
    }

    return true;
}

// ============================================================
// writeErrorsFile — create input_errors.txt only if needed.
// ============================================================
void Simulator::writeErrorsFile(const std::string& basePath) const
{
    if (errors_.empty()) return;

    std::ofstream f(basePath + "/input_errors.txt");
    if (f.is_open()) {
        f << errors_;
        std::cout << "[Simulator] Recoverable errors written to input_errors.txt\n";
    }
}

// ============================================================
// run — full simulation entry point.
// ============================================================
int Simulator::run(const std::string& basePath)
{
    // ---- 1. Load input files ----
    if (!loadAll(basePath)) {
        writeErrorsFile(basePath);
        return 1;
    }

    // ---- 2. Set up shared ground-truth and mocks ----
    // The ground truth is shared between MovementDriverMock (collision
    // detection) and LidarMock (ray casting).  The PositionSensorMock
    // is shared between MovementDriverMock (writes) and LidarMock/
    // the Drone (reads) so all components see the same position.
    auto groundTruthPtr = std::make_shared<SparseBuildingMap>(groundTruth_);
    auto posMock        = std::make_shared<PositionSensorMock>();
    auto moveMock       = std::make_shared<MovementDriverMock>(
                              groundTruthPtr, posMock, droneCfg_, missionCfg_);
    auto lidarMock      = std::make_shared<LidarMock>(
                              groundTruthPtr, posMock, droneCfg_, missionCfg_);

    // ---- 3. Determine starting position ----
    double startX, startY, startZ;

    if (missionCfg_.startSet) {
        // Use position from mission_config.txt
        startX = missionCfg_.startX;
        startY = missionCfg_.startY;
        startZ = missionCfg_.startZ;
    } else {
        // Default: horizontal centre of the map,
        // vertical: floor level + half drone height + 1 cm clearance
        startX = (missionCfg_.minX + missionCfg_.maxX) / 2.0;
        startY = (missionCfg_.minY + missionCfg_.maxY) / 2.0;
        startZ = missionCfg_.minHeight
               + droneCfg_.height.numerical_value_in(cm) / 2.0
               + 1.0;
    }

    posMock->setPosition(Position3D{startX * cm, startY * cm, startZ * cm});
    posMock->setAngle(0.0 * deg);  // heading East at start

    // Verify the starting cell is not inside a wall
    const GridPoint startGrid = toGrid(startX, startY, startZ, missionCfg_);
    if (groundTruthPtr->getCell(startGrid) == static_cast<int>(CellStatus::Occupied)) {
        std::cerr << "[Simulator] FATAL: starting position ("
                  << startX << ", " << startY << ", " << startZ
                  << ") is inside an obstacle.\n";
        writeErrorsFile(basePath);
        return 1;
    }

    std::cout << "[Simulator] Starting at ("
              << startX << ", " << startY << ", " << startZ << ") cm\n";

    // ---- 4. Create Drone and run the main loop ----
    Drone drone(posMock, lidarMock, moveMock, droneCfg_, missionCfg_);

    std::cout << "[Simulator] Starting mapping simulation...\n";
    long long steps = 0;
    while (drone.performStep()) {
        ++steps;
        if (steps % 10000 == 0) {
            const auto& mapData = drone.getMap().data();
            std::cout << "[Simulator] Step " << steps
                      << " — cells mapped: " << mapData.size() << "\n";
        }
    }
    std::cout << "[Simulator] Mapping complete after " << steps << " steps.\n";

    // ---- 5. Write output map ----
    const std::string outPath = basePath + "/map_output.txt";
    if (!drone.getMap().saveToFile(outPath)) {
        std::cerr << "[Simulator] Warning: could not write " << outPath << "\n";
    } else {
        std::cout << "[Simulator] Output map written to " << outPath << "\n";
    }

    // ---- 6. Calculate and print score ----
    const double score = drone.getMap().calculateScore(groundTruth_, missionCfg_);
    std::cout << "[Simulator] ==============================\n";
    std::cout << "[Simulator] Final Mapping Score: "
              << score << " / 100\n";
    std::cout << "[Simulator] ==============================\n";

    // ---- 7. Write error log if any ----
    writeErrorsFile(basePath);

    return 0;
}

} // namespace dm
