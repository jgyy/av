enable_testing()

# Foundation tests
add_executable(test_foundation
    foundation/test_math.cpp
    foundation/test_transform.cpp
    foundation/test_geometry.cpp
)

target_link_libraries(test_foundation PRIVATE
    av_foundation
    GTest::gtest_main
)

add_test(NAME FoundationTests COMMAND test_foundation)

# Physics tests
add_executable(test_physics
    physics/test_physics.cpp
)

target_link_libraries(test_physics PRIVATE
    av_physics
    av_world
    GTest::gtest_main
)

add_test(NAME PhysicsTests COMMAND test_physics)

# Rendering tests
add_executable(test_rendering
    rendering/test_rendering.cpp
)

target_link_libraries(test_rendering PRIVATE
    av_rendering
    GTest::gtest_main
)

add_test(NAME RenderingTests COMMAND test_rendering)

# World tests (Phase 3)
add_executable(test_world
    world/test_world.cpp
)

target_link_libraries(test_world PRIVATE
    av_foundation
    av_physics
    av_world
)

set_target_properties(test_world PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)

add_test(NAME WorldTests COMMAND test_world)
