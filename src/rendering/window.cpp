#include "av/rendering/window.hpp"
#include "av/foundation/logging.hpp"
#include <GLFW/glfw3.h>
#include <GL/gl.h>

namespace av {

Window::Window(int width, int height, const std::string& title)
    : width_(width), height_(height), title_(title) {
    AV_DEBUG("Window created: %dx%d '%s'", width, height, title.c_str());
}

Window::~Window() {
    shutdown();
}

bool Window::initialize() {
    if (initialized_) {
        AV_WARN("Window already initialized");
        return false;
    }

    // Initialize GLFW
    if (!glfwInit()) {
        AV_ERROR("Failed to initialize GLFW");
        return false;
    }

    AV_DEBUG("GLFW initialized");

    // Set error callback
    glfwSetErrorCallback(glfwErrorCallback);

    // Window hints
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_SAMPLES, 4); // MSAA 4x

    // Create window
    handle_ = glfwCreateWindow(width_, height_, title_.c_str(), nullptr, nullptr);
    if (!handle_) {
        AV_ERROR("Failed to create GLFW window");
        glfwTerminate();
        return false;
    }

    AV_INFO("GLFW window created: %dx%d", width_, height_);

    // Make context current
    glfwMakeContextCurrent(handle_);

    // Enable vsync
    glfwSwapInterval(1);

    // Set up OpenGL
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    AV_INFO("OpenGL initialized - Version: %s", reinterpret_cast<const char*>(glGetString(GL_VERSION)));

    initialized_ = true;
    return true;
}

bool Window::shouldClose() const {
    if (!handle_) return true;
    return glfwWindowShouldClose(handle_);
}

void Window::update() {
    if (!handle_) return;

    glfwSwapBuffers(handle_);
    glfwPollEvents();
}

void Window::shutdown() {
    if (handle_) {
        glfwDestroyWindow(handle_);
        handle_ = nullptr;
        AV_DEBUG("GLFW window destroyed");
    }

    if (initialized_) {
        glfwTerminate();
        initialized_ = false;
        AV_DEBUG("GLFW terminated");
    }
}

bool Window::isKeyPressed(int key) const {
    if (!handle_) return false;
    return glfwGetKey(handle_, key) == GLFW_PRESS;
}

bool Window::isMouseButtonPressed(int button) const {
    if (!handle_) return false;
    return glfwGetMouseButton(handle_, button) == GLFW_PRESS;
}

void Window::getMousePosition(double& x, double& y) const {
    if (!handle_) {
        x = y = 0.0;
        return;
    }
    glfwGetCursorPos(handle_, &x, &y);
}

void Window::setTitle(const std::string& title) {
    title_ = title;
    if (handle_) {
        glfwSetWindowTitle(handle_, title.c_str());
    }
}

void Window::glfwErrorCallback(int error, const char* description) {
    AV_ERROR("GLFW Error [%d]: %s", error, description);
}

} // namespace av
