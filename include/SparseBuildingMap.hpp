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
#include <vector>
#include <cstdint>

namespace dm {

class SparseBuildingMap {
public:
    SparseBuildingMap() = default;

    // ---- Dense mode ----
    // Call initDense() once before using as the drone's result map.
    // Pre-allocates a flat array for the full mission volume and
    // initialises every cell to UnmappedNA (-1).
    // After initDense(), setCell/getCell operate on the dense array
    // (faster than the hash-map for large maps).
    void initDense(const MissionConfig& mc);

    // ---- Cell access ----

    void setCell(const GridPoint& p, int status);

    // Returns the cell value, or UnmappedNA (-1) if never written.
    int  getCell(const GridPoint& p) const;

    // True if the cell has been written at least once.
    bool hasCell(const GridPoint& p) const;

    // Number of cells that have been written (not UnmappedNA).
    std::size_t mappedCount() const;

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

    // Returns the sparse backing store.
    // NOTE: in dense mode this map is always empty; use getCell() instead.
    const std::unordered_map<GridPoint, int, GridPointHash>& data() const {
        return data_;
    }

    bool isDenseMode() const { return isDense_; }

private:
    // ---- Sparse storage (default) ----
    std::unordered_map<GridPoint, int, GridPointHash> data_;

    // ---- Dense storage (after initDense()) ----
    bool                  isDense_     = false;
    std::vector<int8_t>   dense_;
    int                   dOffX_       = 0;
    int                   dOffY_       = 0;
    int                   dOffZ_       = 0;
    int                   dSzX_        = 0;
    int                   dSzY_        = 0;
    int                   dSzZ_        = 0;
    std::size_t           mappedCount_ = 0;

    // Returns flat index into dense_, or -1 if out of range.
    int denseIndex(const GridPoint& p) const noexcept;
};

} // namespace dm
