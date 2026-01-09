add_library(av_control SHARED
    pure_pursuit.cpp
    pid_controller.cpp
    trajectory_tracker.cpp
)

target_include_directories(av_control PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(av_control PUBLIC
    av_foundation
    av_planning
    av_physics
)

set_target_properties(av_control PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)
