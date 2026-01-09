add_library(av_simulation SHARED
    simulator.cpp
    scenario.cpp
    data_recorder.cpp
)

target_include_directories(av_simulation PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(av_simulation PUBLIC
    av_foundation
    av_physics
    av_world
    av_sensors
    av_perception
    av_planning
    av_control
    av_rendering
)

set_target_properties(av_simulation PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)
