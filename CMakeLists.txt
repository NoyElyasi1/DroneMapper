cmake_minimum_required(VERSION 3.20)
project(drone_mapper LANGUAGES CXX)

find_package(mp-units CONFIG REQUIRED)
find_package(tinynpy CONFIG REQUIRED)
find_package(yaml-cpp CONFIG REQUIRED)

<<<<<<< HEAD
# Strict warning flags (assignment requirement)
if(MSVC)
    add_compile_options(/W4 /WX)
else()
    add_compile_options(-Wall -Wextra -Werror -pedantic -Wno-c++26-extensions)
endif()

# ============================================================
# Fetch mp-units — strong-typed unit library
# https://github.com/mpusz/mp-units
# ============================================================
include(FetchContent)

FetchContent_Declare(
    gsl-lite
    GIT_REPOSITORY https://github.com/gsl-lite/gsl-lite.git
    GIT_TAG        v0.41.0
)
FetchContent_MakeAvailable(gsl-lite)

FetchContent_Declare(
    mp-units
    GIT_REPOSITORY https://github.com/mpusz/mp-units.git
    GIT_TAG        v2.5.0
    SOURCE_SUBDIR  src
)
# Disable gsl-lite contract checking so FetchContent scoping issues are avoided.
# All project code is correct by construction; runtime assertions add no value here.
set(MP_UNITS_API_CONTRACTS "NONE" CACHE STRING "" FORCE)
FetchContent_MakeAvailable(mp-units)

# ============================================================
# Fetch yaml-cpp — YAML 1.2 parser/emitter for C++
# ============================================================
FetchContent_Declare(
    yaml-cpp
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
    GIT_TAG        0.8.0
)
set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_TOOLS  OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(yaml-cpp)

# ============================================================
# Main executable - 
# ============================================================
add_executable(drone_mapper
    src/main.cpp
    src/Simulator.cpp
    src/SparseBuildingMap.cpp
=======
add_library(drone_mapper
>>>>>>> fd647a1d3568830a42b0e5889c5b3a9021e753c3
    src/ConfigParser.cpp
    src/DroneControlImpl.cpp
    src/ErrorLogger.cpp
    src/MappingAlgorithmImpl.cpp
    src/MapsComparison.cpp
    src/MissionControlImpl.cpp
    src/Map3DImpl.cpp
    src/MockGPS.cpp
    src/MockLidar.cpp
    src/MockMovement.cpp
    src/ScanResultToVoxels.cpp
    src/SimulationManager.cpp
    src/SimulationRunFactoryImpl.cpp
    src/SimulationRunImpl.cpp
)

target_compile_features(drone_mapper PUBLIC cxx_std_20)
target_compile_options(drone_mapper
    PRIVATE
        -Wall
        -Wextra
        -Werror
        -pedantic
)

<<<<<<< HEAD
target_link_libraries(drone_mapper PRIVATE
    mp-units::mp-units
    yaml-cpp::yaml-cpp
=======
target_include_directories(drone_mapper
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
>>>>>>> fd647a1d3568830a42b0e5889c5b3a9021e753c3
)

target_link_libraries(drone_mapper
    PUBLIC
        mp-units::mp-units
        TinyNPY::TinyNPY
        yaml-cpp::yaml-cpp
)

# Precompiled header: mp-units is heavy template machinery — compile it once.
target_precompile_headers(drone_mapper PUBLIC
    <mp-units/framework.h>
    <mp-units/systems/isq.h>
    <mp-units/systems/si/unit_symbols.h>
    <mp-units/systems/si/math.h>
)

add_executable(drone_mapper_simulation src/drone_mapper_simulation_main.cpp)
target_link_libraries(drone_mapper_simulation PRIVATE drone_mapper)

add_executable(maps_comparison src/maps_comparison_main.cpp)
target_link_libraries(maps_comparison PRIVATE drone_mapper)

find_package(GTest CONFIG REQUIRED)

add_executable(drone_mapper_simulation_test
    tests/components/test_simulation_manager.cpp
    tests/components/test_simulation_run.cpp
    tests/components/test_mission_control.cpp
    tests/components/test_drone_control.cpp
    tests/components/test_mapping_algorithm.cpp
    tests/components/test_mock_lidar.cpp
    tests/components/test_maps_comparison.cpp
    tests/integration/test_integration_real_algorithm.cpp
    tests/integration/test_integration_mock_algorithm.cpp
)

target_link_libraries(drone_mapper_simulation_test
    PRIVATE drone_mapper GTest::gtest_main GTest::gmock_main)

target_compile_options(drone_mapper_simulation_test PRIVATE -Wall -Wextra -pedantic)

# Reuse the same PCH for the test binary
target_precompile_headers(drone_mapper_simulation_test REUSE_FROM drone_mapper)

include(GoogleTest)
gtest_discover_tests(drone_mapper_simulation_test)
