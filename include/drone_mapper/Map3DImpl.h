#pragma once

#include <TinyNPY.h>

#include <drone_mapper/IMutableMap3D.h>

#include <filesystem>
#include <memory>

namespace drone_mapper {

class Map3DImpl final : public IMutableMap3D {
public:
    // Load from .npy file with default MapConfig (offset=0, bounds=from shape, resolution=from config).
    explicit Map3DImpl(std::shared_ptr<NpyArray> map_ptr);
    // Load from .npy file with explicit offset-aware MapConfig.
    Map3DImpl(std::shared_ptr<NpyArray> map_ptr, types::MapConfig map_config);
    // Create empty output map from MapConfig (no backing NPY file).
    explicit Map3DImpl(const types::MapConfig& config);

    [[nodiscard]] types::VoxelOccupancy atVoxel(const Position3D& pos) const override;
    [[nodiscard]] types::MapConfig getMapConfig() const override;
    [[nodiscard]] bool isInBounds(const Position3D& pos) const override;

    void set(const Position3D& pos, types::VoxelOccupancy value) override;
    void save(const std::filesystem::path& output_path) const override;

    // Extra accessors used by MapsComparison (not in IMap3D interface)
    [[nodiscard]] int szX() const noexcept;
    [[nodiscard]] int szY() const noexcept;
    [[nodiscard]] int szZ() const noexcept;

private:
    std::shared_ptr<NpyArray> map_;
    types::MapConfig config_;

    [[nodiscard]] int linearIndex(int ix, int iy, int iz) const noexcept;
};

} // namespace drone_mapper
