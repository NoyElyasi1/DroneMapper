#include <drone_mapper/ConfigParser.h>
#include <drone_mapper/ErrorLogger.h>
#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/SimulationRunFactory.h>

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>

namespace {

std::string statusStr(drone_mapper::types::MissionRunStatus s) {
    switch (s) {
    case drone_mapper::types::MissionRunStatus::Completed: return "completed";
    case drone_mapper::types::MissionRunStatus::MaxSteps:  return "max_steps";
    case drone_mapper::types::MissionRunStatus::Error:     return "error";
    }
    return "unknown";
}

std::string resStatusStr(drone_mapper::types::ResolutionRequestStatus s) {
    switch (s) {
    case drone_mapper::types::ResolutionRequestStatus::Accepted:        return "ACCEPTED";
    case drone_mapper::types::ResolutionRequestStatus::Ignored:         return "IGNORED";
    case drone_mapper::types::ResolutionRequestStatus::IgnoredTooSmall: return "IGNORED TOO SMALL";
    }
    return "UNKNOWN";
}

void writeReport(const drone_mapper::types::SimulationManagerReport& report,
                 const std::filesystem::path& output_path)
{
    const auto& runs = report.runs;
    const int total = static_cast<int>(runs.size());
    int error_count = 0;
    double score_sum = 0.0;
    double min_score = 100.0;
    double max_score = 0.0;

    for (const auto& r : runs) {
        if (r.mission_score < 0.0) {
            ++error_count;
        } else {
            score_sum += r.mission_score;
            min_score = std::min(min_score, r.mission_score);
            max_score = std::max(max_score, r.mission_score);
        }
    }
    const int scored = total - error_count;
    const double avg = (scored > 0) ? score_sum / scored : 0.0;

    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "score_report" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "composition_file" << YAML::Value << report.composition_file.string();
    out << YAML::Key << "generated_at_utc" << YAML::Value << report.generated_at_utc;
    out << YAML::Key << "metric"           << YAML::Value << report.metric;
    out << YAML::Key << "score_range"      << YAML::Value << YAML::BeginMap
        << YAML::Key << "min" << YAML::Value << std::get<0>(report.score_range)
        << YAML::Key << "max" << YAML::Value << std::get<1>(report.score_range)
        << YAML::EndMap;
    out << YAML::Key << "error_score" << YAML::Value << report.error_score;

    out << YAML::Key << "summary" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "total_runs"    << YAML::Value << total;
    out << YAML::Key << "scored_runs"   << YAML::Value << scored;
    out << YAML::Key << "error_runs"    << YAML::Value << error_count;
    out << YAML::Key << "average_score" << YAML::Value << avg;
    out << YAML::Key << "min_score"     << YAML::Value << (scored > 0 ? min_score : 0.0);
    out << YAML::Key << "max_score"     << YAML::Value << (scored > 0 ? max_score : 0.0);
    out << YAML::EndMap;

    // Group: simulation_config → mission_config → [per drone×lidar run]
    // Collect unique simulation config paths in order of first appearance.
    std::vector<std::filesystem::path> sim_paths;
    for (const auto& r : runs) {
        if (std::find(sim_paths.begin(), sim_paths.end(), r.simulation_config.config_file) == sim_paths.end())
            sim_paths.push_back(r.simulation_config.config_file);
    }

    out << YAML::Key << "simulations" << YAML::Value << YAML::BeginSeq;

    for (const auto& sim_path : sim_paths) {
        out << YAML::BeginMap;
        out << YAML::Key << "simulation_config" << YAML::Value << sim_path.string();
        out << YAML::Key << "missions"          << YAML::Value << YAML::BeginSeq;

        // Unique mission config paths for this simulation, in order.
        std::vector<std::filesystem::path> mis_paths;
        for (const auto& r : runs) {
            if (r.simulation_config.config_file != sim_path) continue;
            if (std::find(mis_paths.begin(), mis_paths.end(), r.mission_config.config_file) == mis_paths.end())
                mis_paths.push_back(r.mission_config.config_file);
        }

        for (const auto& mis_path : mis_paths) {
            out << YAML::BeginMap;
            out << YAML::Key << "mission_config" << YAML::Value << mis_path.string();

            // resolution_cm and resolution_request_status are the same for all runs
            // with this (sim, mission) pair — take from the first matching run.
            for (const auto& r : runs) {
                if (r.simulation_config.config_file != sim_path) continue;
                if (r.mission_config.config_file    != mis_path) continue;
                out << YAML::Key << "resolution_cm"
                    << YAML::Value << r.output_map_config.resolution.numerical_value_in(drone_mapper::cm);
                out << YAML::Key << "resolution_request_status"
                    << YAML::Value << resStatusStr(r.resolution_request_status);
                break;
            }

            out << YAML::Key << "runs" << YAML::Value << YAML::BeginSeq;
            for (const auto& r : runs) {
                if (r.simulation_config.config_file != sim_path) continue;
                if (r.mission_config.config_file    != mis_path) continue;

                out << YAML::BeginMap;
                out << YAML::Key << "drone_config" << YAML::Value << r.drone_config.config_file.string();
                out << YAML::Key << "lidar_config" << YAML::Value << r.lidar_config.config_file.string();

                if (!r.mission_results.empty()) {
                    const auto& mr = r.mission_results[0];
                    out << YAML::Key << "status" << YAML::Value << statusStr(mr.status);
                    out << YAML::Key << "steps"  << YAML::Value << static_cast<int>(mr.steps);
                    if (!mr.errors.empty()) {
                        out << YAML::Key << "error_ref" << YAML::Value << YAML::BeginMap
                            << YAML::Key << "code"    << YAML::Value << mr.errors[0].code
                            << YAML::EndMap;
                    }
                }
                out << YAML::Key << "score"           << YAML::Value << r.mission_score;
                out << YAML::Key << "output_map_file" << YAML::Value << r.output_map_file.string();
                out << YAML::EndMap;
            }
            out << YAML::EndSeq;  // runs

            out << YAML::EndMap;  // mission block
        }

        out << YAML::EndSeq;  // missions
        out << YAML::EndMap;  // simulation block
    }

    out << YAML::EndSeq;   // simulations
    out << YAML::EndMap;   // score_report
    out << YAML::EndMap;

    std::filesystem::create_directories(output_path);
    std::ofstream f(output_path / "simulation_output.yaml");
    if (!f) throw std::runtime_error("Cannot write simulation_output.yaml");
    f << out.c_str() << '\n';
}

} // namespace

int main(int argc, char** argv) {
    std::filesystem::path composition_file =
        (argc >= 2) ? std::filesystem::path{argv[1]} : std::filesystem::path{"simulation.yaml"};
    std::filesystem::path output_path =
        (argc >= 3) ? std::filesystem::path{argv[2]} : std::filesystem::current_path();

    if (composition_file.is_relative())
        composition_file = std::filesystem::current_path() / composition_file;
    if (output_path.is_relative())
        output_path = std::filesystem::current_path() / output_path;

    try {
        std::string parse_errors;
        const auto composition = drone_mapper::parseCompositionConfig(composition_file, parse_errors);

        // Log parse errors immediately to error log file (inside output_results/).
        const std::filesystem::path results_dir = output_path / "output_results";
        std::filesystem::create_directories(results_dir);
        drone_mapper::ErrorLogger error_logger{results_dir / "errors.log"};
        if (!parse_errors.empty()) {
            error_logger.log("PARSE_ERROR", parse_errors);
            std::cerr << parse_errors;
        }

        auto run_factory = std::make_unique<drone_mapper::SimulationRunFactory>();
        drone_mapper::SimulationManager simulation{std::move(run_factory)};

        const auto report = simulation.run(composition, output_path);
        writeReport(report, output_path);

        // Log any run-level errors.
        for (const auto& r : report.runs) {
            for (const auto& mr : r.mission_results) {
                for (const auto& e : mr.errors) {
                    error_logger.log(e.code, e.message);
                }
            }
        }

        std::cout << "Simulation complete. " << report.runs.size()
                  << " run(s). Report: " << (output_path / "simulation_output.yaml").string() << '\n';
    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << '\n';
        return 1;
    }
    return 0;
}

