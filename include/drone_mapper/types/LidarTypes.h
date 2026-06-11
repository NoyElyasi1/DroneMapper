#pragma once

#include <drone_mapper/Units.h>

#include <cstddef>
#include <vector>

namespace drone_mapper::types {

struct LidarConfigData {
    PhysicalLength z_min{};
    PhysicalLength z_max{};
    PhysicalLength d{};
    std::size_t fov_circles = 0;
};

struct LidarHit {
    // Misses use max double centimeters; check distance < z_max to detect a real hit.
    PhysicalLength distance{};
    Orientation angle{};
};

using LidarScanResult = std::vector<LidarHit>;

} // namespace drone_mapper::types
