add_library(av_physics SHARED
    physics_world.cpp
    vehicle_dynamics.cpp
    vehicle_controller.cpp
)

target_include_directories(av_physics PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(av_physics PUBLIC
    av_foundation
)

if(Bullet_FOUND)
    target_link_libraries(av_physics PUBLIC
        Bullet::LinearMath
        Bullet::BulletDynamics
    )
endif()

set_target_properties(av_physics PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)
