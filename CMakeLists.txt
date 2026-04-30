cmake_minimum_required(VERSION 3.20)
project(drone_mapper LANGUAGES CXX)

# ============================================================
# Build settings
# ============================================================
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Strict warning flags (assignment requirement)
add_compile_options(-Wall -Wextra -Werror -pedantic)

# ============================================================
# Fetch mp-units — strong-typed unit library
# https://github.com/mpusz/mp-units
# ============================================================
include(FetchContent)
FetchContent_Declare(
    mp-units
    GIT_REPOSITORY https://github.com/mpusz/mp-units.git
    GIT_TAG        v2.1.0
)
FetchContent_MakeAvailable(mp-units)

# ============================================================
# Main executable
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
)

# ============================================================
# Build instructions:
#   mkdir build && cd build
#   cmake .. -DCMAKE_BUILD_TYPE=Release
#   make -j$(nproc)
#
# Run:
#   ./drone_mapper ../sample
# ============================================================
