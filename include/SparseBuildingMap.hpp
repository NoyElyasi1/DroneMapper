#pragma once

// ============================================================
// SparseBuildingMap.hpp — 3D sparse voxel map.
//
// Backed by std::unordered_map<GridPoint, int> so only cells
// that have been explicitly written consume memory.
//
// Cell values (matches CellStatus enum in Types.hpp):
//   0  = empty      (scanned, confirmed open)
//   1  = occupied   (scanned, confirmed solid)
//  -1  = unmapped   (not yet reached / unreachable)
//  -2  = out of bounds
//
// File format (map_input.txt / map_output.txt):
//   # x,y,z,status          <- comment line
//   -10,0,5,1               <- grid-index row
//   ...
//
// Grid indices are integers: coord / step, rounded to nearest int.
// ============================================================

#include "Types.hpp"
#include "ConfigParser.hpp"
#include <unordered_map>
#include <string>

namespace dm {

class SparseBuildingMap {
public:
    SparseBuildingMap() = default;

    // ---- Cell access ----

    void setCell(const GridPoint& p, int status);

    // Returns the cell value, or UnmappedNA (-1) if never written.
    int  getCell(const GridPoint& p) const;

    // True if the cell has been written at least once.
    bool hasCell(const GridPoint& p) const;

    // ---- File I/O ----

    // Load from CSV file. Recoverable parse errors are appended
    // to errorsOut. Returns false if the file cannot be opened.
    bool loadFromFile(const std::string& filename,
                      std::string& errorsOut);

    // Write all cells to CSV. Returns false on I/O failure.
    bool saveToFile(const std::string& filename) const;

    // ---- Scoring ----

    // Compare this map (drone result) against groundTruth.
    // Score formula (0–100):
    //   correctOccupied / totalOccupied   * 50   (recall on walls)
    // + correctEmpty    / totalMapped     * 30   (empty-space accuracy)
    // + coverage        / totalCells      * 20   (fraction of space visited)
    //
    // Rationale: detecting walls (occupied cells) is the primary goal,
    // so it has the highest weight. Correctly marking empty space and
    // maximising coverage are secondary objectives.
    double calculateScore(const SparseBuildingMap& groundTruth,
                          const MissionConfig& mc) const;

    // ---- Inspection ----
    const std::unordered_map<GridPoint, int, GridPointHash>& data() const {
        return data_;
    }

private:
    std::unordered_map<GridPoint, int, GridPointHash> data_;
};

} // namespace dm
