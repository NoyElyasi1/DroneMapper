# ============================================================
# tests/CMakeLists.txt
# ============================================================

# Fetch GoogleTest from GitHub
include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0
)
# Prevent GTest from overriding compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# ---- Shared library of all project source files ----
# We build a static lib so each test binary links against it
# without recompiling all sources.
add_library(drone_mapper_lib STATIC
    ${CMAKE_SOURCE_DIR}/src/Simulator.cpp
    ${CMAKE_SOURCE_DIR}/src/SparseBuildingMap.cpp
    ${CMAKE_SOURCE_DIR}/src/ConfigParser.cpp
    ${CMAKE_SOURCE_DIR}/src/LidarMock.cpp
    ${CMAKE_SOURCE_DIR}/src/PositionSensorMock.cpp
    ${CMAKE_SOURCE_DIR}/src/MovementDriverMock.cpp
    ${CMAKE_SOURCE_DIR}/src/Drone.cpp
)
target_include_directories(drone_mapper_lib PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)
target_link_libraries(drone_mapper_lib PUBLIC mp-units::mp-units)

# ---- Helper macro to register a test binary ----
macro(add_drone_test name source)
    add_executable(${name} ${source})
    target_include_directories(${name} PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}   # for test_helpers.hpp
    )
    target_link_libraries(${name} PRIVATE
        drone_mapper_lib
        GTest::gtest_main
    )
    # Register with CTest so you can run them with 'ctest'
    include(GoogleTest)
    gtest_discover_tests(${name})
endmacro()

# ---- Individual test binaries ----
add_drone_test(test_config_parser test_config_parser.cpp)
add_drone_test(test_sparse_map    test_sparse_map.cpp)
add_drone_test(test_mocks         test_mocks.cpp)
add_drone_test(test_drone         test_drone.cpp)
