#include "SparseBuildingMap.hpp"
#include <fstream>
#include <sstream>
#include <cmath>
#include <cstring>
#include <algorithm>

namespace {
constexpr double SCORE_WEIGHT_OCCUPIED = 50.0;
constexpr double SCORE_WEIGHT_EMPTY    = 30.0;
constexpr double SCORE_WEIGHT_COVERAGE = 20.0;
} // namespace

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
// loadFromNpy — parse a NumPy .npy file (version 1.x or 2.x).
//
// Expected format:
//   - 3-dimensional array, shape (nx, ny, nz)
//   - C order (fortran_order = False)
//   - dtype: int8 / uint8 / bool (1 byte per element) or
//             int16/int32/int64 / uint16/uint32/uint64 (multi-byte)
//
// Index mapping:
//   array[ix][iy][iz] → grid point (dOffX+ix, dOffY+iy, dOffZ+iz)
//   where dOff* = round(min* / step*)
//
// Any element with value != 0 is treated as Occupied; 0 as Empty.
// ============================================================

// ---- Simple NPY header parser ----
namespace {

struct NpyInfo {
    int     ndim      = 0;
    int     shape[3]  = {0, 0, 0};
    int     itemsize  = 1;    // bytes per element
    bool    fortran   = false;
    bool    isSigned  = true; // signed vs unsigned integer
};

// Extract a single integer from a Python tuple string like "(51, 61, 21,)"
// starting after the opening '(', reading up to ndim integers.
static bool parseShape(const std::string& hdr, NpyInfo& info)
{
    const auto shapePos = hdr.find("'shape'");
    if (shapePos == std::string::npos) return false;
    const auto lp = hdr.find('(', shapePos);
    if (lp == std::string::npos) return false;
    const auto rp = hdr.find(')', lp);
    if (rp == std::string::npos) return false;

    std::string tuple = hdr.substr(lp + 1, rp - lp - 1);

    // Count dimensions and parse values
    int dim = 0;
    std::stringstream ss(tuple);
    std::string token;
    while (std::getline(ss, token, ',') && dim < 3) {
        // trim
        token.erase(0, token.find_first_not_of(" \t"));
        token.erase(token.find_last_not_of(" \t") + 1);
        if (token.empty()) continue;
        try {
            info.shape[dim++] = std::stoi(token);
        } catch (...) {
            return false;
        }
    }
    info.ndim = dim;
    return dim == 3;
}

static bool parseDtype(const std::string& hdr, NpyInfo& info)
{
    const auto descrPos = hdr.find("'descr'");
    if (descrPos == std::string::npos) return false;
    const auto q1 = hdr.find('\'', descrPos + 7);
    if (q1 == std::string::npos) return false;
    const auto q2 = hdr.find('\'', q1 + 1);
    if (q2 == std::string::npos) return false;

    const std::string dtype = hdr.substr(q1 + 1, q2 - q1 - 1);
    // dtype examples: '<i1', '|i1', '<i4', '|b1', '<u1', '>i2', etc.
    // Last char is the size in bytes; second-to-last is the kind.
    if (dtype.size() < 2) return false;

    const char kind = dtype[dtype.size() - 2];  // 'i', 'u', 'f', 'b'
    const char sizeC = dtype[dtype.size() - 1];  // '1', '2', '4', '8'
    info.itemsize = sizeC - '0';
    info.isSigned = (kind == 'i' || kind == 'b');  // bool/int

    // We support 1, 2, 4, 8 byte integers and booleans
    if (info.itemsize < 1 || info.itemsize > 8) return false;
    return true;
}

static bool parseFortranOrder(const std::string& hdr, NpyInfo& info)
{
    const auto pos = hdr.find("'fortran_order'");
    if (pos == std::string::npos) { info.fortran = false; return true; }
    const auto truePos  = hdr.find("True",  pos);
    const auto falsePos = hdr.find("False", pos);
    const auto colon    = hdr.find(':', pos);
    if (colon == std::string::npos) return false;

    if (truePos != std::string::npos && (falsePos == std::string::npos || truePos < falsePos))
        info.fortran = true;
    else
        info.fortran = false;
    return true;
}

} // anonymous namespace

bool SparseBuildingMap::loadFromNpy(const std::string& filename,
                                    const dm::MissionConfig& mc,
                                    std::string& errorsOut)
{
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        errorsOut += "[SparseBuildingMap] Cannot open: " + filename + "\n";
        return false;
    }

    // ---- 1. Magic string ("\x93NUMPY") ----
    char magic[6];
    file.read(magic, 6);
    if (!file || std::strncmp(magic + 1, "NUMPY", 5) != 0 ||
        static_cast<unsigned char>(magic[0]) != 0x93u) {
        errorsOut += "[SparseBuildingMap] Not a valid .npy file: " + filename + "\n";
        return false;
    }

    // ---- 2. Version ----
    const uint8_t major = static_cast<uint8_t>(file.get());
    file.get();  // minor version (unused)

    // ---- 3. Header length ----
    uint32_t headerLen = 0;
    if (major == 1) {
        uint8_t lo = static_cast<uint8_t>(file.get());
        uint8_t hi = static_cast<uint8_t>(file.get());
        headerLen = static_cast<uint32_t>(lo) | (static_cast<uint32_t>(hi) << 8);
    } else {
        // Version 2+: 4-byte little-endian header length
        uint8_t b[4];
        file.read(reinterpret_cast<char*>(b), 4);
        headerLen = static_cast<uint32_t>(b[0]) |
                    (static_cast<uint32_t>(b[1]) << 8) |
                    (static_cast<uint32_t>(b[2]) << 16) |
                    (static_cast<uint32_t>(b[3]) << 24);
    }

    // ---- 4. Parse header string ----
    std::string header(headerLen, '\0');
    file.read(&header[0], headerLen);
    if (!file) {
        errorsOut += "[SparseBuildingMap] Truncated .npy header: " + filename + "\n";
        return false;
    }

    NpyInfo info;
    if (!parseShape(header, info) || !parseDtype(header, info) || !parseFortranOrder(header, info)) {
        errorsOut += "[SparseBuildingMap] Cannot parse .npy header in: " + filename + "\n";
        return false;
    }

    if (info.fortran) {
        errorsOut += "[SparseBuildingMap] Fortran-order arrays not supported: " + filename + "\n";
        return false;
    }
    if (info.ndim != 3) {
        errorsOut += "[SparseBuildingMap] Expected 3D array in: " + filename + "\n";
        return false;
    }

    const int nx = info.shape[0];
    const int ny = info.shape[1];
    const int nz = info.shape[2];

    // ---- 5. Compute grid offsets from mission config ----
    const int offX = static_cast<int>(std::round(mc.minX      / mc.stepX));
    const int offY = static_cast<int>(std::round(mc.minY      / mc.stepY));
    const int offZ = static_cast<int>(std::round(mc.minHeight / mc.stepZ));

    // ---- 6. Read data ----
    const std::size_t total = static_cast<std::size_t>(nx) * ny * nz;
    const std::size_t bytes = total * static_cast<std::size_t>(info.itemsize);
    std::vector<uint8_t> data(bytes);
    file.read(reinterpret_cast<char*>(data.data()), static_cast<std::streamsize>(bytes));
    if (!file) {
        errorsOut += "[SparseBuildingMap] Truncated data in .npy file: " + filename + "\n";
        return false;
    }

    // ---- 7. Populate map ----
    // For each cell, read the value (multi-byte ints treated as little-endian).
    // Any non-zero value → Occupied; zero → Empty.
    const int is = info.itemsize;
    for (int ix = 0; ix < nx; ++ix) {
        for (int iy = 0; iy < ny; ++iy) {
            for (int iz = 0; iz < nz; ++iz) {
                const std::size_t flatIdx = static_cast<std::size_t>(ix) * ny * nz
                                          + static_cast<std::size_t>(iy) * nz
                                          + static_cast<std::size_t>(iz);
                const uint8_t* ptr = data.data() + flatIdx * is;

                // Read as little-endian integer
                int64_t val = 0;
                for (int b = 0; b < is; ++b)
                    val |= (static_cast<int64_t>(ptr[b]) << (8 * b));

                const GridPoint gp{offX + ix, offY + iy, offZ + iz};
                if (val != 0)
                    setCell(gp, static_cast<int>(CellStatus::Occupied));
                else
                    setCell(gp, static_cast<int>(CellStatus::Empty));
            }
        }
    }

    return true;
}

// ============================================================
// loadFromFile — parse a CSV map file (legacy / output round-trip).
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
            setCell(GridPoint{x, y, z}, status);
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
        ? SCORE_WEIGHT_OCCUPIED
        : (SCORE_WEIGHT_OCCUPIED * droneCorrectOccupied / gtOccupiedTotal);

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
        : (SCORE_WEIGHT_EMPTY * droneEmptyCorrect / droneEmptyTotal);

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
        : (SCORE_WEIGHT_COVERAGE * mappedCells / static_cast<double>(totalCells));

    return scoreA + scoreB + scoreC;
}

} // namespace dm
