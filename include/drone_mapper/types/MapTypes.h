#pragma once

#include <drone_mapper/Units.h>

#include <optional>

namespace drone_mapper::types {

enum class VoxelOccupancy {
    PotentiallyOccupied = -3,
    OutOfBounds = -2,
    Unmapped = -1,
    Empty = 0,
    Occupied = 1,
};

// Changed: moved from mission types because map bounds now belong to map configuration.
struct MappingBounds {
    XLength min_x{};
    XLength max_x{};
    YLength min_y{};
    YLength max_y{};
    ZLength min_height{};
    ZLength max_height{};
};

struct MappedVoxel {
    Position3D position{};
    VoxelOccupancy value = VoxelOccupancy::Unmapped;
};

// Changed: added to keep boundaries, offset, and resolution together on IMap3D.
struct MapConfig {
    MappingBounds boundaries{};
    Position3D offset{};
    PhysicalLength resolution{};
};

// Used by maps_comparison_main to configure each map when comparing.
struct MapComparisonEntry {
    PhysicalLength map_res{};
    Position3D map_offset{};
    std::optional<MappingBounds> map_boundaries{};
};

struct ComparisonConfigData {
    MapComparisonEntry original{};
    MapComparisonEntry target{};
};

} // namespace drone_mapper::types
