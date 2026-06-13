#include <drone_mapper/ConfigParser.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MapsComparison.h>

#include <TinyNPY.h>

#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

std::string parseComparisonConfigArg(const std::string& arg) {
    const std::string prefix = "comparison_config=";
    if (arg.substr(0, prefix.size()) == prefix) {
        return arg.substr(prefix.size());
    }
    return {};
}

std::shared_ptr<NpyArray> loadNpy(const std::string& path) {
    auto arr = std::make_shared<NpyArray>();
    const char* err = arr->LoadNPY(path.c_str());
    if (err != nullptr) {
        throw std::runtime_error(std::string("Cannot load ") + path + ": " + err);
    }
    return arr;
}

// Build a MapConfig from an NPY array, optional entry overrides (offset, boundaries, resolution).
drone_mapper::types::MapConfig buildMapConfig(const NpyArray& npy,
                                               const drone_mapper::types::MapComparisonEntry& entry)
{
    const auto& sh = npy.Shape();
    if (sh.size() != 3) throw std::runtime_error("Map must be 3-dimensional");

    const double res = entry.map_res.numerical_value_in(drone_mapper::cm) > 0.0
                       ? entry.map_res.numerical_value_in(drone_mapper::cm)
                       : 1.0;
    const double ox = entry.map_offset.x.numerical_value_in(drone_mapper::cm);
    const double oy = entry.map_offset.y.numerical_value_in(drone_mapper::cm);
    const double oz = entry.map_offset.z.numerical_value_in(drone_mapper::cm);

    drone_mapper::types::MapConfig cfg;
    cfg.resolution = res * drone_mapper::cm;
    cfg.offset     = entry.map_offset;
    cfg.boundaries.min_x      = ox * drone_mapper::x_extent[drone_mapper::cm];
    cfg.boundaries.max_x      = (ox + (static_cast<int>(sh[0]) - 1) * res) * drone_mapper::x_extent[drone_mapper::cm];
    cfg.boundaries.min_y      = oy * drone_mapper::y_extent[drone_mapper::cm];
    cfg.boundaries.max_y      = (oy + (static_cast<int>(sh[1]) - 1) * res) * drone_mapper::y_extent[drone_mapper::cm];
    cfg.boundaries.min_height = oz * drone_mapper::z_extent[drone_mapper::cm];
    cfg.boundaries.max_height = (oz + (static_cast<int>(sh[2]) - 1) * res) * drone_mapper::z_extent[drone_mapper::cm];

    // If explicit boundaries provided, clamp to them.
    if (entry.map_boundaries) {
        const auto& b = *entry.map_boundaries;
        cfg.boundaries = b;
    }
    return cfg;
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 3 || argc > 4) {
        std::cout << "-1\n";
        std::cerr << "Usage: maps_comparison <origin_map> <target_map> [comparison_config=<path>]\n";
        return 1;
    }

    try {
        auto origin_npy = loadNpy(argv[1]);
        auto target_npy = loadNpy(argv[2]);

        drone_mapper::types::MapComparisonEntry origin_entry;
        drone_mapper::types::MapComparisonEntry target_entry;

        if (argc == 4) {
            const std::string cfg_path = parseComparisonConfigArg(argv[3]);
            if (!cfg_path.empty()) {
                std::string parse_err;
                const auto cmp_cfg = drone_mapper::parseComparisonConfig(cfg_path, parse_err);
                if (!parse_err.empty()) {
                    std::cout << "-1\n";
                    std::cerr << parse_err;
                    return 1;
                }
                origin_entry = cmp_cfg.original;
                target_entry = cmp_cfg.target;
            }
        }

        const auto origin_config = buildMapConfig(*origin_npy, origin_entry);
        const auto target_config = buildMapConfig(*target_npy, target_entry);

        drone_mapper::Map3DImpl origin_map{origin_npy, origin_config};
        drone_mapper::Map3DImpl target_map{target_npy, target_config};

        drone_mapper::Map3DImpl* target_ptr = &target_map;
        const auto scores = drone_mapper::MapsComparison::compare(origin_map, {target_ptr});
        std::cout << scores[0] << '\n';
    } catch (const std::exception& e) {
        std::cout << "-1\n";
        std::cerr << e.what() << '\n';
        return 1;
    }
    return 0;
}
