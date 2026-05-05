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
    add_compile_options(-Wall -Wextra -Werror -pedantic)
endif()

# ============================================================
# Main executable -
# No external libraries — all unit types are self-contained in
# include/Types.hpp using standard C++20 only.
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

# No external link dependencies for the main executable.

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
