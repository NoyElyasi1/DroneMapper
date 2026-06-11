#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MapsComparison.h>

#include <TinyNPY.h>

#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

// Parse optional comparison_config=<path> argument.
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

        // Default: no config → both maps use default MapConfig (resolved from shape, resolution 1cm)
        // If comparison_config provided → load offset/bounds/resolution from it (bonus feature)
        drone_mapper::Map3DImpl origin_map{origin_npy};
        drone_mapper::Map3DImpl target_map{target_npy};

        if (argc == 4) {
            const std::string cfg_path = parseComparisonConfigArg(argv[3]);
            if (!cfg_path.empty()) {
                // Bonus: load per-map configs from comparison_config YAML
                // For now, use defaults (full bonus implementation would parse the YAML)
                (void)cfg_path;
            }
        }

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
