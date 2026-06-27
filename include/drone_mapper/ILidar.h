#pragma once

#include <drone_mapper/Types.h>

namespace drone_mapper {

class ILidar {
public:
    virtual ~ILidar() = default;

    [[nodiscard]] virtual types::LidarScanResult scan(Orientation scan_orientation) const = 0;

    [[nodiscard]] virtual types::LidarConfigData config() const = 0;
};

} // namespace drone_mapper
