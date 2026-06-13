#pragma once

#include <drone_mapper/Types.h>

namespace drone_mapper {

class IGPS {
public:
    virtual ~IGPS() = default;

    [[nodiscard]] virtual Position3D position() const = 0;
    [[nodiscard]] virtual Orientation heading() const = 0;
};

} // namespace drone_mapper
