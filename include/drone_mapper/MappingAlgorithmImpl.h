#pragma once

#include <drone_mapper/IMappingAlgorithm.h>

#include <stack>
#include <unordered_set>

namespace drone_mapper {

namespace detail {
struct GridPoint {
    int x, y, z;
    bool operator==(const GridPoint& o) const noexcept {
        return x == o.x && y == o.y && z == o.z;
    }
};
struct GridPointHash {
    std::size_t operator()(const GridPoint& p) const noexcept {
        std::size_t h = 0;
        auto mix = [&h](int v) {
            h ^= std::hash<int>{}(v) + 0x9e3779b9u + (h << 6) + (h >> 2);
        };
        mix(p.x); mix(p.y); mix(p.z);
        return h;
    }
};
} // namespace detail

class MappingAlgorithmImpl final : public IMappingAlgorithm {
public:
    using IMappingAlgorithm::IMappingAlgorithm;

    [[nodiscard]] types::MappingStepCommand nextStep(const types::DroneState& state,
                                                     const types::LidarScanResult* latest_scan) override;

private:
    enum class NavPhase { FindNext, Elevating, Rotating, Advancing, Done };

    [[nodiscard]] detail::GridPoint toGrid(const Position3D& pos) const;
    [[nodiscard]] bool findFrontier(const detail::GridPoint& cur, detail::GridPoint& out) const;
    [[nodiscard]] bool isOccupied(const detail::GridPoint& gp) const;
    [[nodiscard]] bool isInBounds(const detail::GridPoint& gp) const;

    NavPhase phase_ = NavPhase::FindNext;
    detail::GridPoint currentTarget_{};
    detail::GridPoint prevGrid_{};
    detail::GridPoint lastKnownGrid_{};
    int stuckCount_ = 0;
    bool navigatingBack_ = false;

    std::stack<detail::GridPoint> backtrackStack_;
    std::unordered_set<detail::GridPoint, detail::GridPointHash> visited_;
};

} // namespace drone_mapper
