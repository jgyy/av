#pragma once

#include "av/foundation/math.hpp"
#include "av/foundation/transform.hpp"
#include "av/rendering/renderer.hpp"
#include "av/rendering/mesh.hpp"
#include "av/rendering/debug_renderer.hpp"
#include <memory>
#include <vector>

namespace av {

// Forward declarations
class VehicleDynamics;

// Scene object for rendering
struct SceneObject {
    std::string name;
    std::shared_ptr<Mesh> mesh;
    Transform transform;
    Vec3 color;
    bool visible = true;
};

// Scene manager for organizing renderable objects
class Scene {
public:
    Scene(std::shared_ptr<Renderer> renderer);
    ~Scene();

    // Initialize scene with ground and basic objects
    void initialize();

    // Add objects to scene
    void addObject(const std::string& name, std::shared_ptr<Mesh> mesh,
                   const Transform& transform, const Vec3& color);
    void addVehicle(const std::string& name, const Vec3& position, const Vec3& color = Vec3(0.2f, 0.2f, 0.8f));
    void addGround(float width = 100.0f, float depth = 100.0f, const Vec3& color = Vec3(0.2f, 0.6f, 0.2f));

    // Update object transforms
    void updateObjectTransform(const std::string& name, const Transform& transform);

    // Rendering
    void render();
    void renderDebug();

    // Debug visualization
    void enableDebugGrid(bool enable) { debugGridEnabled_ = enable; }
    void enableDebugAxes(bool enable) { debugAxesEnabled_ = enable; }
    void enableVehicleDebug(bool enable) { vehicleDebugEnabled_ = enable; }

    // Getters
    std::shared_ptr<SceneObject> getObject(const std::string& name);
    std::shared_ptr<DebugRenderer> getDebugRenderer() { return debugRenderer_; }
    size_t getObjectCount() const { return objects_.size(); }

    // Clear scene
    void clear();

private:
    std::shared_ptr<Renderer> renderer_;
    std::shared_ptr<DebugRenderer> debugRenderer_;
    std::vector<std::shared_ptr<SceneObject>> objects_;

    // Debug rendering options
    bool debugGridEnabled_ = true;
    bool debugAxesEnabled_ = true;
    bool vehicleDebugEnabled_ = true;

    // Pre-created meshes
    std::shared_ptr<Mesh> vehicleMesh_;
    std::shared_ptr<Mesh> groundMesh_;
};

} // namespace av
