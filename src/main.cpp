// ============================================================
// main.cpp — drone_mapper entry point.
//
// Usage:  drone_mapper [<input_output_files_path>]
//
// If <input_output_files_path> is omitted, the current working
// directory is used.
//
// The program reads:
//   <path>/drone_config.yaml
//   <path>/mission_config.yaml
//   <path>/map_input.npy
//
// And writes:
//   <path>/map_output.txt
//   <path>/input_errors.txt  (only if recoverable errors occurred)
//
// The program NEVER calls exit() or abort(); all control flow
// terminates by returning from main().
// ============================================================

#include "Simulator.hpp"
#include <iostream>
#include <string>

int main(int argc, char* argv[])
{
    const std::string basePath = (argc > 1) ? argv[1] : ".";

    dm::Simulator simulator;
    const int result = simulator.run(basePath);

    if (result != 0) {
        std::cerr << "[drone_mapper] Simulation ended with a fatal error.\n";
    }

    return result;
}
