add_library(av_perception SHARED
    perception_system.cpp
    object_detector.cpp
    object_tracker.cpp
    lane_detector.cpp
    sensor_fusion.cpp
)

target_include_directories(av_perception PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(av_perception PUBLIC
    av_foundation
    av_sensors
    av_world
)

set_target_properties(av_perception PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)
