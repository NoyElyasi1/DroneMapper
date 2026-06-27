#include <drone_mapper/Map3DImpl.h>

#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>

namespace drone_mapper {

namespace {

// Compute grid dimensions from MapConfig.
void computeSize(const types::MapConfig& cfg, int& szX, int& szY, int& szZ) {
    const double res = cfg.resolution.numerical_value_in(cm);
    const double minX = cfg.boundaries.min_x.numerical_value_in(cm) - cfg.offset.x.numerical_value_in(cm);
    const double maxX = cfg.boundaries.max_x.numerical_value_in(cm) - cfg.offset.x.numerical_value_in(cm);
    const double minY = cfg.boundaries.min_y.numerical_value_in(cm) - cfg.offset.y.numerical_value_in(cm);
    const double maxY = cfg.boundaries.max_y.numerical_value_in(cm) - cfg.offset.y.numerical_value_in(cm);
    const double minZ = cfg.boundaries.min_height.numerical_value_in(cm) - cfg.offset.z.numerical_value_in(cm);
    const double maxZ = cfg.boundaries.max_height.numerical_value_in(cm) - cfg.offset.z.numerical_value_in(cm);
    szX = static_cast<int>(std::round((maxX - minX) / res)) + 1;
    szY = static_cast<int>(std::round((maxY - minY) / res)) + 1;
    szZ = static_cast<int>(std::round((maxZ - minZ) / res)) + 1;
}

// Build a MapConfig from a loaded NpyArray and a resolution.
types::MapConfig configFromNpy(const NpyArray& arr, PhysicalLength resolution, Position3D offset) {
    const auto& sh = arr.Shape();
    const double res = resolution.numerical_value_in(cm);
    const double ox = offset.x.numerical_value_in(cm);
    const double oy = offset.y.numerical_value_in(cm);
    const double oz = offset.z.numerical_value_in(cm);
    types::MapConfig cfg;
    cfg.resolution = resolution;
    cfg.offset = offset;
    cfg.boundaries.min_x = ox * x_extent[cm];
    cfg.boundaries.max_x = (ox + (static_cast<int>(sh[0]) - 1) * res) * x_extent[cm];
    cfg.boundaries.min_y = oy * y_extent[cm];
    cfg.boundaries.max_y = (oy + (static_cast<int>(sh[1]) - 1) * res) * y_extent[cm];
    cfg.boundaries.min_height = oz * z_extent[cm];
    cfg.boundaries.max_height = (oz + (static_cast<int>(sh[2]) - 1) * res) * z_extent[cm];
    return cfg;
}

} // namespace

// ---- Constructor: from NpyArray (default config, origin at 0,0,0) ----
Map3DImpl::Map3DImpl(std::shared_ptr<NpyArray> map_ptr)
    : map_(std::move(map_ptr))
{
    if (!map_ || map_->IsEmpty()) {
        throw std::runtime_error("Map3DImpl: null or empty NpyArray");
    }
    if (map_->Shape().size() != 3) {
        throw std::runtime_error("Map3DImpl: expected 3D array");
    }
    // Default: resolution 1cm, offset = origin, bounds from shape
    config_ = configFromNpy(*map_, 1.0 * cm, Position3D{});
}

// ---- Constructor: from NpyArray with explicit MapConfig ----
Map3DImpl::Map3DImpl(std::shared_ptr<NpyArray> map_ptr, types::MapConfig map_config)
    : map_(std::move(map_ptr))
    , config_(map_config)
{
    if (!map_ || map_->IsEmpty()) {
        throw std::runtime_error("Map3DImpl: null or empty NpyArray");
    }
    if (map_->Shape().size() != 3) {
        throw std::runtime_error("Map3DImpl: expected 3D array");
    }
}

// ---- Constructor: create empty output map from MapConfig ----
Map3DImpl::Map3DImpl(const types::MapConfig& config)
    : config_(config)
{
    int sx = 0, sy = 0, sz = 0;
    computeSize(config_, sx, sy, sz);
    if (sx <= 0 || sy <= 0 || sz <= 0) {
        throw std::runtime_error("Map3DImpl: invalid MapConfig dimensions");
    }
    const NpyArray::shape_t shape{
        static_cast<std::size_t>(sx),
        static_cast<std::size_t>(sy),
        static_cast<std::size_t>(sz),
    };
    map_ = std::make_shared<NpyArray>(shape, sizeof(int8_t), 'i');
    map_->Allocate();
    std::memset(map_->Data(), static_cast<uint8_t>(types::VoxelOccupancy::Unmapped), map_->SizeBytes());
}

int Map3DImpl::szX() const noexcept {
    return map_ ? static_cast<int>(map_->Shape()[0]) : 0;
}
int Map3DImpl::szY() const noexcept {
    return map_ ? static_cast<int>(map_->Shape()[1]) : 0;
}
int Map3DImpl::szZ() const noexcept {
    return map_ ? static_cast<int>(map_->Shape()[2]) : 0;
}

int Map3DImpl::linearIndex(int ix, int iy, int iz) const noexcept {
    if (ix < 0 || ix >= szX() || iy < 0 || iy >= szY() || iz < 0 || iz >= szZ())
        return -1;
    return ix + iy * szX() + iz * szX() * szY();
}

types::VoxelOccupancy Map3DImpl::atVoxel(const Position3D& pos) const {
    const double res = config_.resolution.numerical_value_in(cm);
    const double ox  = config_.offset.x.numerical_value_in(cm);
    const double oy  = config_.offset.y.numerical_value_in(cm);
    const double oz  = config_.offset.z.numerical_value_in(cm);
    const int ix = static_cast<int>(std::round((pos.x.numerical_value_in(cm) - ox) / res));
    const int iy = static_cast<int>(std::round((pos.y.numerical_value_in(cm) - oy) / res));
    const int iz = static_cast<int>(std::round((pos.z.numerical_value_in(cm) - oz) / res));
    const int idx = linearIndex(ix, iy, iz);
    if (idx < 0) return types::VoxelOccupancy::OutOfBounds;
    const auto* raw = map_->Data<int8_t>();
    return static_cast<types::VoxelOccupancy>(raw[idx]);
}

types::MapConfig Map3DImpl::getMapConfig() const {
    return config_;
}

bool Map3DImpl::isInBounds(const Position3D& pos) const {
    const auto& b = config_.boundaries;
    const double px = pos.x.numerical_value_in(cm);
    const double py = pos.y.numerical_value_in(cm);
    const double pz = pos.z.numerical_value_in(cm);
    return px >= b.min_x.numerical_value_in(cm)
        && px <= b.max_x.numerical_value_in(cm)
        && py >= b.min_y.numerical_value_in(cm)
        && py <= b.max_y.numerical_value_in(cm)
        && pz >= b.min_height.numerical_value_in(cm)
        && pz <= b.max_height.numerical_value_in(cm);
}

void Map3DImpl::set(const Position3D& pos, types::VoxelOccupancy value) {
    const double res = config_.resolution.numerical_value_in(cm);
    const double ox  = config_.offset.x.numerical_value_in(cm);
    const double oy  = config_.offset.y.numerical_value_in(cm);
    const double oz  = config_.offset.z.numerical_value_in(cm);
    const int ix = static_cast<int>(std::round((pos.x.numerical_value_in(cm) - ox) / res));
    const int iy = static_cast<int>(std::round((pos.y.numerical_value_in(cm) - oy) / res));
    const int iz = static_cast<int>(std::round((pos.z.numerical_value_in(cm) - oz) / res));
    const int idx = linearIndex(ix, iy, iz);
    if (idx < 0) return;
    auto* raw = map_->Data<int8_t>();
    // Don't overwrite Occupied with Empty
    if (value == types::VoxelOccupancy::Empty &&
        raw[idx] == static_cast<int8_t>(types::VoxelOccupancy::Occupied))
        return;
    raw[idx] = static_cast<int8_t>(value);
}

void Map3DImpl::save(const std::filesystem::path& output_path) const {
    const auto parent = output_path.parent_path();
    if (!parent.empty()) std::filesystem::create_directories(parent);
    const char* err = map_->SaveNPY(output_path.string());
    if (err != nullptr) {
        throw std::runtime_error(std::string("Map3DImpl: save failed: ") + err);
    }
}

} // namespace drone_mapper
