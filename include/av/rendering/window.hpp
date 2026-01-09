#pragma once

#include "av/foundation/math.hpp"
#include <string>
#include <memory>

struct GLFWwindow;

namespace av {

// Window management with GLFW
class Window {
public:
    Window(int width = 1280, int height = 720, const std::string& title = "AV Simulator");
    ~Window();

    // Initialize GLFW and create window
    bool initialize();

    // Check if window should close
    bool shouldClose() const;

    // Process events and swap buffers
    void update();

    // Shutdown window
    void shutdown();

    // Getters
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    float getAspectRatio() const { return static_cast<float>(width_) / static_cast<float>(height_); }
    GLFWwindow* getHandle() const { return handle_; }
    bool isValid() const { return handle_ != nullptr; }

    // Input getters
    bool isKeyPressed(int key) const;
    bool isMouseButtonPressed(int button) const;
    void getMousePosition(double& x, double& y) const;

    // Set title
    void setTitle(const std::string& title);

private:
    int width_;
    int height_;
    std::string title_;
    GLFWwindow* handle_ = nullptr;
    bool initialized_ = false;

    static void glfwErrorCallback(int error, const char* description);
};

} // namespace av
