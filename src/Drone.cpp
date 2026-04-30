#include "Drone.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

namespace dm {

Drone::Drone(std::shared_ptr<IPositionSensor> posSensor,
             std::shared_ptr<ILidarSensor>    lidar,
             std::shared_ptr<IMovementDriver> driver,
             const DroneConfig&   droneCfg,
             const MissionConfig& missionCfg)
    : posSensor_(std::move(posSensor))
    , lidar_(std::move(lidar))
    , driver_(std::move(driver))
    , droneCfg_(droneCfg)
    , missionCfg_(missionCfg)
{}

// ============================================================
// processScan — update the drone's internal map from scan results.
//
// For each beam result:
//   - The ScanResult stores a world-space unit direction (dx,dy,dz)
//     and the measured distance.
//   - We march along the ray from the drone origin and mark each
//     traversed voxel as Empty (0).
//   - The voxel at the hit distance is marked Occupied (1).
//   - If distance == 0 (hit < Zmin), the nearest voxel is occupied.
//   - If distance == Zmax (no hit), the full ray is marked empty.
//
// We never overwrite Occupied with Empty — a cell known to be a
// wall stays a wall even if a later beam grazes past it.
// ============================================================
void Drone::processScan(const std::vector<ScanResult>& results)
{
    const Position3D pos = posSensor_->getCurrentPosition();
    const double ox = pos.x.numerical_value_in(cm);
    const double oy = pos.y.numerical_value_in(cm);
    const double oz = pos.z.numerical_value_in(cm);

    const double zmax = droneCfg_.lidarZmax.numerical_value_in(cm);
    const double step = std::max(0.1,
        std::min({missionCfg_.stepX, missionCfg_.stepY, missionCfg_.stepZ}));

    for (const auto& sr : results) {
        const double dist = sr.distance.numerical_value_in(cm);

        if (dist <= 0.0) {
            // Hit too close to measure — mark the immediate voxel ahead
            const GridPoint gp = toGrid(ox + sr.dx * step,
                                        oy + sr.dy * step,
                                        oz + sr.dz * step,
                                        missionCfg_);
            resultMap_.setCell(gp, static_cast<int>(CellStatus::Occupied));
            continue;
        }

        // Mark all traversed voxels as Empty (only if not already Occupied)
        for (double t = step; t < dist - step * 0.5; t += step) {
            const GridPoint gp = toGrid(ox + sr.dx * t,
                                        oy + sr.dy * t,
                                        oz + sr.dz * t,
                                        missionCfg_);
            if (resultMap_.getCell(gp) != static_cast<int>(CellStatus::Occupied)) {
                resultMap_.setCell(gp, static_cast<int>(CellStatus::Empty));
            }
        }

        // Mark the hit voxel as Occupied (if the beam actually hit something)
        if (dist < zmax - step * 0.5) {
            const GridPoint gp = toGrid(ox + sr.dx * dist,
                                        oy + sr.dy * dist,
                                        oz + sr.dz * dist,
                                        missionCfg_);
            resultMap_.setCell(gp, static_cast<int>(CellStatus::Occupied));
        }
    }

    // Always mark the drone's own current voxel as Empty
    resultMap_.setCell(toGrid(ox, oy, oz, missionCfg_),
                       static_cast<int>(CellStatus::Empty));
}

// ============================================================
// scanDirection — fire the Lidar in one specific direction and
// update the internal map.
// ============================================================
void Drone::scanDirection(Angle xyOffset, Angle elevAngle)
{
    auto results = lidar_->scan(xyOffset, elevAngle);
    processScan(results);
}

// ============================================================
// scanAllDirections — comprehensive 360° + vertical coverage.
//
// Scans in 10 directions:
//   • 8 horizontal azimuths at 0°, 45°, 90°, 135°, 180°,
//     225°, 270°, 315°  (relative to drone heading)
//   • straight up   (elevation +90°)
//   • straight down (elevation -90°)
//
// This guarantees that every solid surface within Zmax of the
// drone in any direction is eventually detected and marked,
// regardless of the drone's current heading.
// ============================================================
void Drone::scanAllDirections()
{
    // Horizontal ring — 8 directions, every 45°
    for (int i = 0; i < 8; ++i) {
        scanDirection(static_cast<double>(i * 45) * deg, 0.0 * deg);
    }
    // Vertical extremes
    scanDirection(0.0 * deg,  90.0 * deg);  // straight up
    scanDirection(0.0 * deg, -90.0 * deg);  // straight down
}

// ============================================================
// findFrontierNeighbour — look for an adjacent unexplored cell.
//
// Checks the 6 orthogonal neighbours in a fixed order for one
// that satisfies all three conditions:
//   1. Within mission boundaries
//   2. Not already in visited_  (prevents revisiting cells)
//   3. Not known to be Occupied (prevents trying to enter walls)
//
// The fixed order (+X, -X, +Y, -Y, +Z, -Z) makes the search
// fully deterministic.
// ============================================================
bool Drone::findFrontierNeighbour(const GridPoint& current,
                                   GridPoint& target) const
{
    // 6 cardinal directions in grid space
    constexpr int DIRS[6][3] = {
        { 1,  0,  0},  // East
        {-1,  0,  0},  // West
        { 0,  1,  0},  // South
        { 0, -1,  0},  // North
        { 0,  0,  1},  // Up
        { 0,  0, -1}   // Down
    };

    for (const auto& d : DIRS) {
        GridPoint candidate{
            current.x + d[0],
            current.y + d[1],
            current.z + d[2]
        };

        // ---- Boundary check (continuous coordinates) ----
        const double cx = candidate.x * missionCfg_.stepX;
        const double cy = candidate.y * missionCfg_.stepY;
        const double cz = candidate.z * missionCfg_.stepZ;

        if (cx < missionCfg_.minX || cx > missionCfg_.maxX) continue;
        if (cy < missionCfg_.minY || cy > missionCfg_.maxY) continue;
        if (cz < missionCfg_.minHeight || cz > missionCfg_.maxHeight) continue;

        // ---- Already visited? ----
        if (visited_.count(candidate)) continue;

        // ---- Known to be occupied? ----
        // (Do not attempt to navigate into a wall we've already detected.)
        if (resultMap_.getCell(candidate) == static_cast<int>(CellStatus::Occupied)) {
            continue;
        }

        target = candidate;
        return true;
    }
    return false;
}

// ============================================================
// faceDirection — rotate drone to an absolute world heading.
//
// The rotation may need multiple sub-steps if the angular
// difference exceeds maxRotate.
// ============================================================
void Drone::faceDirection(double targetAngleDeg)
{
    double curAngle = posSensor_->getCurrentAngle().numerical_value_in(deg);
    double diff = targetAngleDeg - curAngle;

    // Normalise to (-180, +180] for shortest-path rotation
    while (diff >  180.0) diff -= 360.0;
    while (diff < -180.0) diff += 360.0;

    const double maxRot = droneCfg_.maxRotate.numerical_value_in(deg);

    // Rotate in as many sub-steps as needed
    while (std::fabs(diff) > 0.01) {
        const double rotAmount = std::clamp(diff, -maxRot, maxRot);
        driver_->rotate(rotAmount * deg);
        diff -= rotAmount;
    }
}

// ============================================================
// navigateTo — move the drone to a target grid cell.
//
// Strategy:
//   1. Elevate/descend to the target Z level first so that XY
//      movement happens on the correct horizontal plane.
//   2. Rotate to face the target and advance in sub-steps
//      (each sub-step ≤ maxAdvance).
//   3. Return false immediately if any sub-step is blocked.
//
// Note: this navigates in a straight line. In complex geometries
// a wall may lie between current position and target, causing
// failure.  The caller handles this by marking the target as
// occupied.  Full path-planning (A*) is left for future work.
// ============================================================
bool Drone::navigateTo(const GridPoint& targetGrid)
{
    const double tx = targetGrid.x * missionCfg_.stepX;
    const double ty = targetGrid.y * missionCfg_.stepY;
    const double tz = targetGrid.z * missionCfg_.stepZ;

    const double maxElev = droneCfg_.maxElevate.numerical_value_in(cm);
    const double maxAdv  = droneCfg_.maxAdvance.numerical_value_in(cm);

    // ---- Step 1: Elevate to target Z ----
    {
        Position3D pos = posSensor_->getCurrentPosition();
        double dz = tz - pos.z.numerical_value_in(cm);
        while (std::fabs(dz) > 0.01) {
            const double move = std::clamp(dz, -maxElev, maxElev);
            if (!driver_->elevate(move * cm)) return false;
            pos = posSensor_->getCurrentPosition();
            dz  = tz - pos.z.numerical_value_in(cm);
        }
    }

    // ---- Step 2: Rotate and advance to target XY ----
    {
        Position3D pos = posSensor_->getCurrentPosition();
        double ddx = tx - pos.x.numerical_value_in(cm);
        double ddy = ty - pos.y.numerical_value_in(cm);
        double xyDist = std::sqrt(ddx*ddx + ddy*ddy);

        if (xyDist > 0.01) {
            // Face the target (atan2 gives angle from +X axis in degrees)
            double targetAngleDeg = std::atan2(ddy, ddx) * 180.0 / M_PI;
            if (targetAngleDeg < 0.0) targetAngleDeg += 360.0;
            faceDirection(targetAngleDeg);

            // Advance in sub-steps
            while (xyDist > 0.01) {
                const double move = std::min(xyDist, maxAdv);
                if (!driver_->advance(move * cm)) return false;

                pos    = posSensor_->getCurrentPosition();
                ddx    = tx - pos.x.numerical_value_in(cm);
                ddy    = ty - pos.y.numerical_value_in(cm);
                xyDist = std::sqrt(ddx*ddx + ddy*ddy);
            }
        }
    }

    return true;
}

// ============================================================
// performStep — one iteration of the DFS exploration loop.
//
// Returns true while mapping is ongoing; false when done.
//
// Full algorithm per step:
//   1. Get current grid position.
//   2. If first visit: add to visited_ and scan all directions.
//   3. Find an unvisited, non-occupied adjacent cell (frontier).
//   4a. If found: push current to backtrack stack and move there.
//       If move fails: mark the cell as occupied, do NOT backtrack.
//   4b. If not found: pop backtrack stack and return to previous cell.
//   5. If stack is empty: mapping complete.
// ============================================================
bool Drone::performStep()
{
    if (++stepCount_ > MAX_STEPS) return false;  // safety guard

    // ---- Current grid position ----
    const Position3D pos = posSensor_->getCurrentPosition();
    const GridPoint curGrid = toGrid(
        pos.x.numerical_value_in(cm),
        pos.y.numerical_value_in(cm),
        pos.z.numerical_value_in(cm),
        missionCfg_);

    // ---- First visit to this cell: scan all directions ----
    if (!visited_.count(curGrid)) {
        visited_.insert(curGrid);
        scanAllDirections();
    }

    // ---- Find a frontier neighbour ----
    GridPoint target{};
    if (findFrontierNeighbour(curGrid, target)) {
        backtrackStack_.push(curGrid);

        if (!navigateTo(target)) {
            // Movement blocked — this cell is a wall we didn't detect yet.
            // Mark it so we never try again.
            resultMap_.setCell(target, static_cast<int>(CellStatus::Occupied));
            visited_.insert(target);       // treat as visited so it's skipped
            backtrackStack_.pop();         // undo the push
        }
        return true;
    }

    // ---- No frontier: backtrack along the DFS path ----
    while (!backtrackStack_.empty()) {
        GridPoint prev = backtrackStack_.top();
        backtrackStack_.pop();

        if (navigateTo(prev)) {
            return true;  // successfully returned to a previous cell
        }
        // If we can't go back to 'prev' either (very rare), continue
        // popping until we reach a reachable ancestor.
    }

    // Stack empty — every reachable cell has been explored.
    return false;
}

} // namespace dm
