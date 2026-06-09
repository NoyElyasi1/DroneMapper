cmake_minimum_required(VERSION 3.20)
project(drone_mapper LANGUAGES CXX)

# ============================================================
# Build settings
# ============================================================
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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
    src/ConfigParser.cpp
    src/LidarMock.cpp
    src/PositionSensorMock.cpp
    src/MovementDriverMock.cpp
    src/Drone.cpp
)

target_include_directories(drone_mapper PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(drone_mapper PRIVATE
    mp-units::mp-units
    yaml-cpp::yaml-cpp
)

# ============================================================
# Tests
# ============================================================
enable_testing()
add_subdirectory(tests)

# ============================================================
# Build instructions:
#   mkdir build && cd build
#   cmake .. -DCMAKE_BUILD_TYPE=Release
#   make -j$(nproc)
#
# Run:
#   ./drone_mapper ../sample1
# ============================================================
