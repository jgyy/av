#pragma once

#include "av/foundation/math.hpp"
#include <memory>
#include <vector>

namespace av {

// Forward declarations
class Window;
class Shader;
class Mesh;
class Camera;
class VehicleDynamics;
class World;

// Main renderer class
class Renderer {
public:
    Renderer();
    ~Renderer();

    // Initialize renderer (creates window and OpenGL context)
    bool initialize(int width = 1280, int height = 720, const std::string& title = "AV Simulator");

    // Check if rendering should continue
    bool shouldContinue() const;

    // Render frame
    void beginFrame();
    void endFrame();

    // Set active camera
    void setCamera(std::shared_ptr<Camera> camera);
    std::shared_ptr<Camera> getCamera() const { return activeCamera_; }

    // Render a mesh with transform and color
    void renderMesh(const std::shared_ptr<Mesh>& mesh, const Mat4& transform, const Vec3& color = Vec3::Ones());

    // Simple debug rendering
    void renderLine(const Vec3& from, const Vec3& to, const Vec3& color = Vec3(1, 1, 0));
    void renderBox(const Vec3& center, const Vec3& size, const Vec3& color = Vec3(0, 1, 0));
    void renderSphere(const Vec3& center, float radius, const Vec3& color = Vec3(1, 0, 0));

    // Get window
    std::shared_ptr<Window> getWindow() const { return window_; }

    // Get aspect ratio
    float getAspectRatio() const;

    // Shutdown
    void shutdown();

private:
    std::shared_ptr<Window> window_;
    std::shared_ptr<Shader> basicShader_;
    std::shared_ptr<Shader> debugShader_;
    std::shared_ptr<Camera> activeCamera_;

    // Debug meshes
    std::shared_ptr<Mesh> cubeMesh_;
    std::shared_ptr<Mesh> sphereMesh_;
    std::shared_ptr<Mesh> lineMesh_;

    void initializeShaders();
    void initializeDebugMeshes();
};

} // namespace av
