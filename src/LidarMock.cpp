#include "LidarMock.hpp"
#include <cmath>
#include <algorithm>

namespace dm {

LidarMock::LidarMock(
        std::shared_ptr<SparseBuildingMap> groundTruth,
        std::shared_ptr<PositionSensorMock> posSensor,
        const DroneConfig& droneCfg,
        const MissionConfig& missionCfg)
    : groundTruth_(std::move(groundTruth))
    , posSensor_(std::move(posSensor))
    , droneCfg_(droneCfg)
    , missionCfg_(missionCfg)
{}

// ============================================================
// castRay — march a single ray through the ground-truth map.
//
// Parameters:
//   (ox,oy,oz) — ray origin in cm (drone centre)
//   (dx,dy,dz) — unit direction vector
//
// Returns:
//   0 * cm     : hit something closer than Zmin (too close to measure)
//   t * cm     : distance to first occupied cell (Zmin <= t <= Zmax)
//   Zmax * cm  : nothing hit within operational range
// ============================================================
Distance LidarMock::castRay(double ox, double oy, double oz,
                            double dx, double dy, double dz) const
{
    const double zmin = droneCfg_.lidarZmin.numerical_value_in(cm);
    const double zmax = droneCfg_.lidarZmax.numerical_value_in(cm);

    // Use the smallest resolution step as the march step size.
    // Floor at 0.1 cm to avoid degenerate infinite loops.
    const double step = std::max(0.1,
        std::min({missionCfg_.stepX, missionCfg_.stepY, missionCfg_.stepZ}));

    for (double t = step; t <= zmax; t += step) {
        const double px = ox + dx * t;
        const double py = oy + dy * t;
        const double pz = oz + dz * t;

        const GridPoint gp = toGrid(px, py, pz, missionCfg_);
        if (groundTruth_->getCell(gp) == static_cast<int>(CellStatus::Occupied)) {
            // Hit — report 0 if too close to measure accurately
            return (t < zmin) ? 0.0 * cm : t * cm;
        }
    }

    return zmax * cm;  // nothing detected within range
}

// ============================================================
// scan — generate all beams and ray-cast each one.
//
// The scan direction is defined by two angles relative to the
// drone's current world-space heading:
//   xyOffset       — horizontal rotation (cw positive, degrees)
//   elevationAngle — vertical tilt from horizontal (degrees)
//
// Algorithm:
//   1. Compute the central beam direction W in world space.
//   2. Build an orthonormal frame {W, R, U} for the FOV cone.
//   3. For circle k (k = 0 .. FOVC-1):
//        halfAngle_k = atan( k*D / Zmin )
//        numBeams_k  = 4^k   (1, 4, 16, 64, 256, ...)
//        For each beam j = 0..numBeams_k-1:
//          azimuth_j = 2*pi*j / numBeams_k
//          dir = cos(halfAngle)*W
//              + sin(halfAngle)*(cos(azimuth)*R + sin(azimuth)*U)
//   4. Ray-cast each dir and record the result.
// ============================================================
std::vector<ScanResult> LidarMock::scan(Angle xyOffset, Angle elevationAngle)
{
    std::vector<ScanResult> results;

    // ---- Drone state ----
    const Position3D pos        = posSensor_->getCurrentPosition();
    const double     headingDeg = posSensor_->getCurrentAngle().numerical_value_in(deg);

    const double ox = pos.x.numerical_value_in(cm);
    const double oy = pos.y.numerical_value_in(cm);
    const double oz = pos.z.numerical_value_in(cm);

    // ---- Central beam direction W ----
    const double totalHeadingRad =
        (headingDeg + xyOffset.numerical_value_in(deg)) * M_PI / 180.0;
    const double elevRad =
        elevationAngle.numerical_value_in(deg) * M_PI / 180.0;

    const double wX = std::cos(totalHeadingRad) * std::cos(elevRad);
    const double wY = std::sin(totalHeadingRad) * std::cos(elevRad);
    const double wZ = std::sin(elevRad);

    // ---- Build orthonormal frame {W, R, U} ----
    // R = right vector: cross(W, world_up).
    // If W is nearly vertical, use world_right instead to avoid zero cross product.
    double rX, rY, rZ;
    constexpr double VERTICAL_THRESHOLD = 0.99;
    if (std::fabs(wZ) < VERTICAL_THRESHOLD) {
        // cross(W, (0,0,1)) = (Wy, -Wx, 0)
        rX = wY;   rY = -wX;  rZ = 0.0;
    } else {
        // W is near vertical — use (1,0,0) as reference instead
        // cross(W, (1,0,0)) = (0, Wz, -Wy)
        rX = 0.0;  rY = wZ;   rZ = -wY;
    }
    const double rLen = std::sqrt(rX*rX + rY*rY + rZ*rZ);
    if (rLen < 1e-9) {
        // Degenerate case (should not happen given the threshold above)
        return results;
    }
    rX /= rLen;  rY /= rLen;  rZ /= rLen;

    // U = up-in-cone = cross(W, R)  [completes the right-hand frame]
    const double uX = wY*rZ - wZ*rY;
    const double uY = wZ*rX - wX*rZ;
    const double uZ = wX*rY - wY*rX;

    // ---- Lidar parameters ----
    const double zmin = droneCfg_.lidarZmin.numerical_value_in(cm);
    const double D    = droneCfg_.lidarD;
    const int    fovc = droneCfg_.lidarFOVC;

    // Pre-allocate: total beams = sum(4^k, k=0..fovc-1)
    // = (4^fovc - 1) / 3  (geometric series).  For fovc=5 this is 341.
    results.reserve(static_cast<std::size_t>((std::pow(4, fovc) - 1) / 3 + 1));

    // ---- Circle 0: single central beam ----
    {
        const Distance dist = castRay(ox, oy, oz, wX, wY, wZ);
        results.push_back(ScanResult{wX, wY, wZ, dist});
    }

    // ---- Circles 1 .. fovc-1 ----
    for (int k = 1; k < fovc; ++k) {
        // Half-angle of this circle's cone (measured from W)
        const double halfAngle = std::atan2(static_cast<double>(k) * D, zmin);
        const double cosH = std::cos(halfAngle);
        const double sinH = std::sin(halfAngle);

        // Number of beams in circle k = 4^k
        int numBeams = 1;
        for (int i = 0; i < k; ++i) numBeams *= 4;

        for (int j = 0; j < numBeams; ++j) {
            const double azRad = (2.0 * M_PI * j) / numBeams;
            const double cosA  = std::cos(azRad);
            const double sinA  = std::sin(azRad);

            // dir = cos(halfAngle)*W + sin(halfAngle)*(cos(az)*R + sin(az)*U)
            double dxB = cosH * wX + sinH * (cosA * rX + sinA * uX);
            double dyB = cosH * wY + sinH * (cosA * rY + sinA * uY);
            double dzB = cosH * wZ + sinH * (cosA * rZ + sinA * uZ);

            // Normalise (should already be unit length, but guard for safety)
            const double len = std::sqrt(dxB*dxB + dyB*dyB + dzB*dzB);
            if (len > 1e-9) { dxB /= len; dyB /= len; dzB /= len; }

            const Distance dist = castRay(ox, oy, oz, dxB, dyB, dzB);
            results.push_back(ScanResult{dxB, dyB, dzB, dist});
        }
    }

    return results;
}

} // namespace dm
