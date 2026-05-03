#include "SparseBuildingMap.hpp"
#include <fstream>
#include <sstream>
#include <cmath>

namespace dm {

// ---- Dense mode initialisation ----

void SparseBuildingMap::initDense(const MissionConfig& mc)
{
    dOffX_ = static_cast<int>(std::round(mc.minX      / mc.stepX));
    dOffY_ = static_cast<int>(std::round(mc.minY      / mc.stepY));
    dOffZ_ = static_cast<int>(std::round(mc.minHeight / mc.stepZ));
    dSzX_  = static_cast<int>(std::round((mc.maxX      - mc.minX)      / mc.stepX)) + 1;
    dSzY_  = static_cast<int>(std::round((mc.maxY      - mc.minY)      / mc.stepY)) + 1;
    dSzZ_  = static_cast<int>(std::round((mc.maxHeight - mc.minHeight) / mc.stepZ)) + 1;

    const std::size_t total =
        static_cast<std::size_t>(dSzX_) * dSzY_ * dSzZ_;
    dense_.assign(total, static_cast<int8_t>(CellStatus::UnmappedNA));
    isDense_     = true;
    mappedCount_ = 0;
}

int SparseBuildingMap::denseIndex(const GridPoint& p) const noexcept
{
    const int ix = p.x - dOffX_;
    const int iy = p.y - dOffY_;
    const int iz = p.z - dOffZ_;
    if (ix < 0 || ix >= dSzX_ || iy < 0 || iy >= dSzY_ || iz < 0 || iz >= dSzZ_)
        return -1;
    return ix + iy * dSzX_ + iz * dSzX_ * dSzY_;
}

// ---- Cell access ----

void SparseBuildingMap::setCell(const GridPoint& p, int status)
{
    if (isDense_) {
        const int idx = denseIndex(p);
        if (idx >= 0) {
            if (dense_[idx] == static_cast<int8_t>(CellStatus::UnmappedNA))
                ++mappedCount_;
            dense_[idx] = static_cast<int8_t>(status);
        }
        return;
    }
    data_[p] = status;
}

int SparseBuildingMap::getCell(const GridPoint& p) const
{
    if (isDense_) {
        const int idx = denseIndex(p);
        if (idx >= 0) return static_cast<int>(dense_[idx]);
        return static_cast<int>(CellStatus::UnmappedNA);
    }
    const auto it = data_.find(p);
    return (it != data_.end()) ? it->second
                               : static_cast<int>(CellStatus::UnmappedNA);
}

bool SparseBuildingMap::hasCell(const GridPoint& p) const
{
    if (isDense_) {
        const int idx = denseIndex(p);
        return idx >= 0 &&
               dense_[idx] != static_cast<int8_t>(CellStatus::UnmappedNA);
    }
    return data_.find(p) != data_.end();
}

std::size_t SparseBuildingMap::mappedCount() const
{
    return isDense_ ? mappedCount_ : data_.size();
}

// ============================================================
// loadFromFile — parse a CSV map file.
//
// Format:
//   # comment
//   x,y,z,status
//
// Grid indices are integers; status is one of {0, 1, -1, -2}.
// Recoverable parse errors (bad lines) are skipped and logged.
// Returns false only if the file cannot be opened.
// ============================================================
bool SparseBuildingMap::loadFromFile(const std::string& filename,
                                      std::string& errorsOut)
{
    std::ifstream file(filename);
    if (!file.is_open()) {
        errorsOut += "[SparseBuildingMap] Cannot open: " + filename + "\n";
        return false;
    }

    std::string line;
    int lineNum  = 0;
    int badLines = 0;

    while (std::getline(file, line)) {
        ++lineNum;
        // Strip '\r'
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty() || line[0] == '#') continue;

        std::istringstream iss(line);
        int x{}, y{}, z{}, status{};
        char c1{}, c2{}, c3{};

        if ((iss >> x >> c1 >> y >> c2 >> z >> c3 >> status)
            && c1 == ',' && c2 == ',' && c3 == ',')
        {
            data_[GridPoint{x, y, z}] = status;
        } else {
            ++badLines;
            if (badLines <= 10) {
                // Report up to 10 bad lines to avoid flooding the error log
                errorsOut += "[SparseBuildingMap] " + filename
                           + " line " + std::to_string(lineNum)
                           + ": bad format, skipped\n";
            }
        }
    }

    if (badLines > 10) {
        errorsOut += "[SparseBuildingMap] ... and "
                   + std::to_string(badLines - 10)
                   + " more bad lines in " + filename + "\n";
    }

    return true;
}

// ============================================================
// saveToFile — write all cells to CSV.
// ============================================================
bool SparseBuildingMap::saveToFile(const std::string& filename) const
{
    std::ofstream file(filename);
    if (!file.is_open()) return false;

    file << "# x,y,z,status\n";

    if (isDense_) {
        for (int iz = 0; iz < dSzZ_; ++iz)
        for (int iy = 0; iy < dSzY_; ++iy)
        for (int ix = 0; ix < dSzX_; ++ix) {
            const int idx    = ix + iy * dSzX_ + iz * dSzX_ * dSzY_;
            const int status = static_cast<int>(dense_[idx]);
            if (status != static_cast<int>(CellStatus::UnmappedNA)) {
                file << (ix + dOffX_) << ',' << (iy + dOffY_) << ','
                     << (iz + dOffZ_) << ',' << status << '\n';
            }
        }
    } else {
        for (const auto& [pt, status] : data_) {
            file << pt.x << ',' << pt.y << ',' << pt.z << ',' << status << '\n';
        }
    }
    return true;
}

// ============================================================
// calculateScore — compare this map against the ground truth.
//
// Score formula (0–100, three weighted components):
//
//  A. OCCUPIED RECALL (weight 50):
//       What fraction of the ground-truth occupied cells did the
//       drone correctly identify as occupied?
//       Score_A = correctOccupied / totalGroundTruthOccupied * 50
//       (Detects walls/obstacles — the primary mapping goal.)
//
//  B. EMPTY PRECISION (weight 30):
//       Among cells the drone marked as empty, what fraction are
//       also empty in the ground truth?
//       Score_B = (droneEmpty ∩ gtEmpty) / droneEmpty * 30
//       (Penalises false negatives — reporting free space that is
//        actually blocked, which could cause future collision.)
//
//  C. COVERAGE (weight 20):
//       What fraction of the total mission volume has the drone
//       visited (marked as either empty or occupied)?
//       Score_C = mappedCells / totalMissionCells * 20
//       (Rewards exploring the whole space.)
//
// Note: if the ground truth has no occupied cells, Score_A = 50
// (building is trivially mapped).  If the drone marked nothing,
// Score_B and Score_C are 0.
// ============================================================
double SparseBuildingMap::calculateScore(const SparseBuildingMap& groundTruth,
                                          const MissionConfig& mc) const
{
    // ---- Component A: occupied recall ----
    int gtOccupiedTotal      = 0;
    int droneCorrectOccupied = 0;

    if (groundTruth.isDense_) {
        for (int iz = 0; iz < groundTruth.dSzZ_; ++iz)
        for (int iy = 0; iy < groundTruth.dSzY_; ++iy)
        for (int ix = 0; ix < groundTruth.dSzX_; ++ix) {
            const int idx = ix + iy * groundTruth.dSzX_
                               + iz * groundTruth.dSzX_ * groundTruth.dSzY_;
            if (static_cast<int>(groundTruth.dense_[idx]) ==
                    static_cast<int>(CellStatus::Occupied)) {
                ++gtOccupiedTotal;
                const GridPoint pt{ix + groundTruth.dOffX_,
                                   iy + groundTruth.dOffY_,
                                   iz + groundTruth.dOffZ_};
                if (getCell(pt) == static_cast<int>(CellStatus::Occupied))
                    ++droneCorrectOccupied;
            }
        }
    } else {
        for (const auto& [pt, gtStatus] : groundTruth.data()) {
            if (gtStatus == static_cast<int>(CellStatus::Occupied)) {
                ++gtOccupiedTotal;
                if (getCell(pt) == static_cast<int>(CellStatus::Occupied))
                    ++droneCorrectOccupied;
            }
        }
    }

    const double scoreA = (gtOccupiedTotal == 0)
        ? 50.0
        : (50.0 * droneCorrectOccupied / gtOccupiedTotal);

    // ---- Component B: empty precision + Component C: coverage ----
    int droneEmptyTotal    = 0;
    int droneEmptyCorrect  = 0;
    int mappedCells        = 0;

    if (isDense_) {
        for (int iz = 0; iz < dSzZ_; ++iz)
        for (int iy = 0; iy < dSzY_; ++iy)
        for (int ix = 0; ix < dSzX_; ++ix) {
            const int idx      = ix + iy * dSzX_ + iz * dSzX_ * dSzY_;
            const int myStatus = static_cast<int>(dense_[idx]);
            if (myStatus == static_cast<int>(CellStatus::Empty)) {
                ++droneEmptyTotal;
                const GridPoint pt{ix + dOffX_, iy + dOffY_, iz + dOffZ_};
                if (groundTruth.getCell(pt) != static_cast<int>(CellStatus::Occupied))
                    ++droneEmptyCorrect;
            }
            if (myStatus == static_cast<int>(CellStatus::Empty) ||
                myStatus == static_cast<int>(CellStatus::Occupied))
                ++mappedCells;
        }
    } else {
        for (const auto& [pt, myStatus] : data_) {
            if (myStatus == static_cast<int>(CellStatus::Empty)) {
                ++droneEmptyTotal;
                if (groundTruth.getCell(pt) != static_cast<int>(CellStatus::Occupied))
                    ++droneEmptyCorrect;
            }
            if (myStatus == static_cast<int>(CellStatus::Empty) ||
                myStatus == static_cast<int>(CellStatus::Occupied))
                ++mappedCells;
        }
    }

    const double scoreB = (droneEmptyTotal == 0)
        ? 0.0
        : (30.0 * droneEmptyCorrect / droneEmptyTotal);

    // ---- Component C: coverage ----
    // Estimate total mission cells from boundary and step sizes
    const long long xCells = static_cast<long long>(
        std::round((mc.maxX - mc.minX) / mc.stepX)) + 1;
    const long long yCells = static_cast<long long>(
        std::round((mc.maxY - mc.minY) / mc.stepY)) + 1;
    const long long zCells = static_cast<long long>(
        std::round((mc.maxHeight - mc.minHeight) / mc.stepZ)) + 1;
    const long long totalCells = xCells * yCells * zCells;

    const double scoreC = (totalCells <= 0)
        ? 0.0
        : (20.0 * mappedCells / static_cast<double>(totalCells));

    return scoreA + scoreB + scoreC;
}

} // namespace dm
