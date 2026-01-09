add_library(av_world SHARED
    world.cpp
    road_network.cpp
    traffic.cpp
)

target_include_directories(av_world PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(av_world PUBLIC
    av_foundation
    av_physics
)

set_target_properties(av_world PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)
