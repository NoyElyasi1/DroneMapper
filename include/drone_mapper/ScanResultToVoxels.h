#pragma once

#include <drone_mapper/IMutableMap3D.h>
#include <drone_mapper/Types.h>

namespace drone_mapper {

class ScanResultToVoxels {
public:
    // Applies a LiDAR scan directly to the output map.
    // Writes Occupied, Empty, and PotentiallyOccupied states.
    static void applyToMap(IMutableMap3D& output_map,
                           const Position3D& scan_origin,
                           const Orientation& drone_heading,
                           const types::LidarScanResult& scan,
                           const types::LidarConfigData& lidar_config);
};

} // namespace drone_mapper
