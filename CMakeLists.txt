cmake_minimum_required(VERSION 3.20)
project(av VERSION 0.1.0 LANGUAGES CXX)

# C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set default build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Add cmake modules
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Conan integration (if using conan)
if(CONAN_EXPORTED)
    include(${CMAKE_BINARY_DIR}/conan_paths.cmake)
endif()

# Find packages
find_package(Bullet REQUIRED HINTS /usr/lib/cmake/bullet /usr/local/lib/cmake/bullet)
if(NOT Bullet_FOUND)
    find_package(Bullet QUIET)
endif()
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Eigen3 REQUIRED)

# Optional: Find Embree for LIDAR (added in later phases)
find_package(embree QUIET)

# Optional: Find Google Test
find_package(GTest)
if(GTest_FOUND)
    enable_testing()
endif()

# Main library
add_library(av SHARED)

# Include directories
target_include_directories(av PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Foundation module
add_subdirectory(src/foundation)
target_link_libraries(av PUBLIC
    av_foundation
)

# Physics module
add_subdirectory(src/physics)
target_link_libraries(av PUBLIC
    av_physics
    Bullet::LinearMath
    Bullet::BulletDynamics
)

# World module (depends on physics)
add_subdirectory(src/world)
target_link_libraries(av PUBLIC
    av_world
)

# Sensors module (depends on foundation, physics, world)
add_subdirectory(src/sensors)
target_link_libraries(av PUBLIC
    av_sensors
)

# Perception module (depends on sensors)
add_subdirectory(src/perception)
target_link_libraries(av PUBLIC
    av_perception
)

# Planning module (depends on perception, world)
add_subdirectory(src/planning)
target_link_libraries(av PUBLIC
    av_planning
)

# Control module (depends on planning)
add_subdirectory(src/control)
target_link_libraries(av PUBLIC
    av_control
)

# Rendering module (depends on foundation)
add_subdirectory(src/rendering)
target_link_libraries(av PUBLIC
    av_rendering
    OpenGL::GL
    glfw
)

# Simulation module (depends on all)
add_subdirectory(src/simulation)
target_link_libraries(av PUBLIC
    av_simulation
)

# Main executable
add_executable(av_exe src/main.cpp)
target_link_libraries(av_exe PRIVATE
    av
    spdlog::spdlog
    nlohmann_json::nlohmann_json
    Eigen3::Eigen
)

set_target_properties(av_exe PROPERTIES
    OUTPUT_NAME av
)

# Tests
if(GTest_FOUND)
    add_subdirectory(tests)
endif()

# Benchmarks
add_subdirectory(benchmarks)

# Installation
install(TARGETS av av_exe
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/av
    DESTINATION include
)

# Print configuration
message(STATUS "AV Build Configuration:")
message(STATUS "  Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Embree Found: ${embree_FOUND}")
message(STATUS "  Google Test Found: ${GTest_FOUND}")
