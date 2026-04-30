#pragma once

// ============================================================
// LidarMock.hpp — Simulated Lidar sensor.
//
// Implements ILidarSensor by ray-casting against the ground-
// truth map.  The real drone does NOT see this class; it only
// uses the ILidarSensor interface.
//
// ---- LIDAR GEOMETRY ----
//
// The scan() call accepts:
//   xyOffset       — horizontal rotation from drone heading (deg)
//   elevationAngle — vertical tilt from horizontal (deg)
//
// These two angles define the central beam direction W in world
// space:
//   totalHeading = droneHeading + xyOffset
//   W = ( cos(totalHeading)*cos(elev),
//          sin(totalHeading)*cos(elev),
//          sin(elev) )
//
// Additional beam circles are then distributed in a cone around W.
// To place them correctly in 3D we build a local orthonormal
// frame {W, R, U} where R = right (horizontal perp to W) and
// U = up (perp to both):
//   R = normalise( cross(W, world_up) )   [world_up = (0,0,1)]
//   U = normalise( cross(R, W) )          [or cross(W,R) depending on convention]
//
// Beam at circle k, spoke j:
//   halfAngle_k = atan( k*D / Zmin )         [cone half-angle]
//   azimuth_j   = 2*pi*j / (4^k)             [angular position on circle]
//   dir = cos(halfAngle)*W
//       + sin(halfAngle)*( cos(azimuth)*R + sin(azimuth)*U )
//
// Each beam is then ray-marched against the ground-truth map
// at step = min(stepX, stepY, stepZ) until it hits an occupied
// cell or reaches Zmax.
//
// ---- BLIND SPOTS ----
// At range r the gap between adjacent beams in circle k is
// approximately 2*pi*r*sin(halfAngle_k) / (4^k).  The drone
// algorithm handles this by scanning from multiple positions
// (at close range, blind-spot gaps shrink to sub-resolution
// size and become negligible).
// ============================================================

#include "Interfaces.hpp"
#include "PositionSensorMock.hpp"
#include "SparseBuildingMap.hpp"
#include "ConfigParser.hpp"
#include <memory>

namespace dm {

class LidarMock : public ILidarSensor {
public:
    LidarMock(std::shared_ptr<SparseBuildingMap> groundTruth,
              std::shared_ptr<PositionSensorMock> posSensor,
              const DroneConfig& droneCfg,
              const MissionConfig& missionCfg);

    // Fire all beams and return results.
    // xyOffset / elevationAngle: per assignment spec "Scan <X-Y angle, height angle>".
    std::vector<ScanResult> scan(
        Angle xyOffset       = 0.0 * deg,
        Angle elevationAngle = 0.0 * deg) override;

private:
    // Ray-march from (ox,oy,oz) in unit direction (dx,dy,dz).
    // Returns distance to first occupied cell, or 0 if hit < Zmin,
    // or Zmax if nothing hit.
    Distance castRay(double ox, double oy, double oz,
                     double dx, double dy, double dz) const;

    std::shared_ptr<SparseBuildingMap> groundTruth_;
    std::shared_ptr<PositionSensorMock> posSensor_;
    DroneConfig   droneCfg_;
    MissionConfig missionCfg_;
};

} // namespace dm
