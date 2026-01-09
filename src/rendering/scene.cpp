#include "av/rendering/scene.hpp"
#include "av/foundation/logging.hpp"

namespace av {

Scene::Scene(std::shared_ptr<Renderer> renderer)
    : renderer_(renderer) {
    debugRenderer_ = std::make_shared<DebugRenderer>(renderer);
    AV_DEBUG("Scene created");
}

Scene::~Scene() {
    clear();
    AV_DEBUG("Scene destroyed");
}

void Scene::initialize() {
    AV_INFO("Initializing scene");

    // Create vehicle mesh (simple box for now)
    vehicleMesh_ = Mesh::createCube(2.0f);

    // Create ground mesh
    groundMesh_ = Mesh::createPlane(100.0f, 100.0f, 10);

    // Add ground to scene
    addGround(100.0f, 100.0f);

    AV_INFO("Scene initialized");
}

void Scene::addObject(const std::string& name, std::shared_ptr<Mesh> mesh,
                      const Transform& transform, const Vec3& color) {
    auto obj = std::make_shared<SceneObject>();
    obj->name = name;
    obj->mesh = mesh;
    obj->transform = transform;
    obj->color = color;

    objects_.push_back(obj);
    AV_DEBUG("Added scene object: %s", name.c_str());
}

void Scene::addVehicle(const std::string& name, const Vec3& position, const Vec3& color) {
    if (!vehicleMesh_) {
        vehicleMesh_ = Mesh::createCube(2.0f);
    }

    Transform vehicleTransform;
    vehicleTransform.setPosition(position);

    addObject(name, vehicleMesh_, vehicleTransform, color);
}

void Scene::addGround(float width, float depth, const Vec3& color) {
    if (!groundMesh_) {
        groundMesh_ = Mesh::createPlane(width, depth, 10);
    }

    Transform groundTransform;
    groundTransform.setPosition(Vec3(0, -0.5f, 0));

    addObject("ground", groundMesh_, groundTransform, color);
}

void Scene::updateObjectTransform(const std::string& name, const Transform& transform) {
    for (auto& obj : objects_) {
        if (obj->name == name) {
            obj->transform = transform;
            AV_DEBUG("Updated transform for object: %s", name.c_str());
            return;
        }
    }
    AV_WARN("Object '%s' not found in scene", name.c_str());
}

void Scene::render() {
    if (!renderer_) {
        return;
    }

    // Render all scene objects
    for (const auto& obj : objects_) {
        if (obj->visible && obj->mesh) {
            Mat4 transform = obj->transform.getMatrix();
            renderer_->renderMesh(obj->mesh, transform, obj->color);
        }
    }
}

void Scene::renderDebug() {
    if (!debugRenderer_ || !debugRenderer_->isEnabled()) {
        return;
    }

    // Render debug grid
    if (debugGridEnabled_) {
        debugRenderer_->drawGrid(50.0f, 10);
    }

    // Render coordinate axes at origin
    if (debugAxesEnabled_) {
        debugRenderer_->drawAxes(Vec3::Zero(), 5.0f);
    }

    // Render debug visualization for vehicles
    if (vehicleDebugEnabled_) {
        for (const auto& obj : objects_) {
            if (obj->name.find("vehicle") != std::string::npos) {
                // Draw bounding box for vehicle
                debugRenderer_->addBox(obj->transform.getPosition(),
                                      Vec3(2.0f, 2.0f, 4.0f),
                                      Vec3(0, 1, 1));

                // Draw direction vector
                Vec3 pos = obj->transform.getPosition();
                Vec3 forward = obj->transform.getForward() * 3.0f;
                debugRenderer_->addLine(pos, pos + forward, Vec3(1, 1, 0));
            }
        }
    }

    // Render all debug geometry
    debugRenderer_->render();
    debugRenderer_->clear();
}

std::shared_ptr<SceneObject> Scene::getObject(const std::string& name) {
    for (const auto& obj : objects_) {
        if (obj->name == name) {
            return obj;
        }
    }
    return nullptr;
}

void Scene::clear() {
    objects_.clear();
    debugRenderer_->clear();
    AV_DEBUG("Scene cleared");
}

} // namespace av
