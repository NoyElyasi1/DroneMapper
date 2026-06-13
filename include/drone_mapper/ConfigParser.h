#pragma once

#include <drone_mapper/Types.h>

#include <filesystem>
#include <string>

namespace drone_mapper {

// Parses YAML configuration files into typed config structs.
// Errors are appended to errorsOut; functions return std::nullopt on fatal failure.

[[nodiscard]] types::DroneConfigData parseDroneConfig(const std::filesystem::path& path,
                                                       std::string& errorsOut);

[[nodiscard]] types::LidarConfigData parseLidarConfig(const std::filesystem::path& path,
                                                       std::string& errorsOut);

[[nodiscard]] types::MissionConfigData parseMissionConfig(const std::filesystem::path& path,
                                                          std::string& errorsOut);

[[nodiscard]] types::SimulationConfigData parseSimulationConfig(const std::filesystem::path& path,
                                                                std::string& errorsOut);

[[nodiscard]] types::SimulationCompositionData parseCompositionConfig(const std::filesystem::path& path,
                                                                      std::string& errorsOut);

[[nodiscard]] types::ComparisonConfigData parseComparisonConfig(const std::filesystem::path& path,
                                                                std::string& errorsOut);

} // namespace drone_mapper
