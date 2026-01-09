add_library(av_rendering SHARED
    renderer.cpp
    camera.cpp
    mesh.cpp
    shader.cpp
    window.cpp
    debug_renderer.cpp
    imgui_helper.cpp
    scene.cpp
)

target_include_directories(av_rendering PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(av_rendering PUBLIC
    av_foundation
    OpenGL::GL
    glfw
)

if(GLEW_FOUND)
    target_link_libraries(av_rendering PUBLIC GLEW::GLEW)
endif()

set_target_properties(av_rendering PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)
