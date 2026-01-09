add_library(av_sensors SHARED
    sensor.cpp
    lidar.cpp
    camera.cpp
    radar.cpp
    imu.cpp
    gps.cpp
    sensor_manager.cpp
)

target_include_directories(av_sensors PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(av_sensors PUBLIC
    av_foundation
    av_physics
    av_world
)

# Optional: Embree for LIDAR
if(embree_FOUND)
    target_link_libraries(av_sensors PUBLIC embree)
    target_compile_definitions(av_sensors PUBLIC AV_WITH_EMBREE)
endif()

set_target_properties(av_sensors PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)
