#pragma once

// ============================================================
// Drone.hpp — Autonomous 3D mapping agent.
//
// ---- ALGORITHM: Deterministic 3D DFS with multi-directional scanning ----
//
// State:
//   visited_        — set of GridPoints the drone has been to.
//   backtrackStack_ — path of grid positions for DFS backtracking.
//   resultMap_      — the drone's internal map (only values set by
//                     the drone; never touches the ground truth).
//
// Main loop (performStep returns false when done):
//
//   1. SCAN ALL DIRECTIONS
//      At every new cell the drone scans in 10 directions:
//        - 8 horizontal azimuths (every 45°)
//        - straight up  (+90° elevation)
//        - straight down (-90° elevation)
//      Each scan fires the full FOVC cone of beams, marking
//      occupied cells and clearing the beam path as empty.
//      Scanning 360° before moving ensures maximum coverage
//      and avoids blind-spot artefacts.
//
//   2. FIND FRONTIER
//      Search the 6 orthogonal neighbours of the current grid
//      cell for one that is:
//        (a) within mission boundaries
//        (b) not yet in visited_
//        (c) not known to be occupied
//      The search order is fixed (deterministic): +X,-X,+Y,-Y,+Z,-Z.
//
//   3. MOVE
//      Push current position on backtrackStack_, then navigate
//      to the frontier cell (rotate + advance/elevate in sub-steps).
//      If navigation fails (unexpected obstacle), mark the target as
//      occupied and do NOT push to the stack.
//
//   4. BACKTRACK
//      If no frontier cell was found, pop the stack and return to
//      the previous position.  If the stack is empty, mapping is done.
//
// ---- DESIGN CHOICES ----
//   - The visited_ set prevents revisiting cells and eliminates
//     infinite loops in the DFS.
//   - navigateTo uses the movement driver interface in small steps
//     (respecting maxAdvance / maxElevate), so the Drone never
//     issues a command that exceeds hardware limits.
//   - Backtracking navigates directly between grid positions.
//     For complex maze-like buildings this may fail if the straight
//     line between two grid positions crosses a wall; in that case
//     the move is rejected, and the drone tries the next backtrack
//     position.  Full path-finding (A*) would be needed for
//     guaranteed backtracking in all topologies (future work).
//   - MAX_STEPS is a safety guard against any remaining edge-cases.
// ============================================================

#include "Interfaces.hpp"
#include "SparseBuildingMap.hpp"
#include "ConfigParser.hpp"
#include <memory>
#include <stack>
#include <unordered_set>

namespace dm {

class Drone {
public:
    Drone(std::shared_ptr<IPositionSensor> posSensor,
          std::shared_ptr<ILidarSensor>    lidar,
          std::shared_ptr<IMovementDriver> driver,
          const DroneConfig&    droneCfg,
          const MissionConfig&  missionCfg);

    // Perform one step of the exploration.
    // Returns true while mapping is ongoing, false when done.
    bool performStep();

    // Read-only access to the drone's internal result map.
    const SparseBuildingMap& getMap() const { return resultMap_; }

private:
    // ---- Lidar ----

    // Scan in a single direction and update resultMap_.
    void scanDirection(Angle xyOffset, Angle elevAngle);

    // Scan 10 directions covering the full sphere.
    void scanAllDirections();

    // Update resultMap_ from a batch of ScanResults.
    void processScan(const std::vector<ScanResult>& results);

    // ---- Navigation ----

    // Find an unvisited, navigable neighbour of 'current'.
    // Returns true and sets 'target' if found.
    bool findFrontierNeighbour(const GridPoint& current,
                               GridPoint& target) const;

    // Navigate to targetGrid using multiple rotate/advance/elevate steps.
    // Returns false if movement is blocked.
    bool navigateTo(const GridPoint& targetGrid);

    // Rotate drone heading to face targetAngleDeg (absolute world angle).
    void faceDirection(double targetAngleDeg);

    // ---- Members ----
    std::shared_ptr<IPositionSensor> posSensor_;
    std::shared_ptr<ILidarSensor>    lidar_;
    std::shared_ptr<IMovementDriver> driver_;
    DroneConfig    droneCfg_;
    MissionConfig  missionCfg_;

    SparseBuildingMap resultMap_;

    // DFS backtrack path
    std::stack<GridPoint> backtrackStack_;

    // Cells the drone has physically visited and scanned
    std::unordered_set<GridPoint, GridPointHash> visited_;

    // Safety counter to prevent any remaining infinite-loop edge cases
    int stepCount_ = 0;
    static constexpr int MAX_STEPS = 2'000'000;
};

} // namespace dm
