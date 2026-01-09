add_library(av_planning SHARED
    global_planner.cpp
    behavior_planner.cpp
    local_planner.cpp
    trajectory_generator.cpp
    collision_checker.cpp
)

target_include_directories(av_planning PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(av_planning PUBLIC
    av_foundation
    av_perception
    av_world
)

set_target_properties(av_planning PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)
