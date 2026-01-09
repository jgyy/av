#pragma once

#include "av/foundation/math.hpp"
#include "av/foundation/geometry.hpp"
#include <memory>
#include <vector>

class btDynamicsWorld;
class btPhysicsWorld;
class btRigidBody;
class btCollisionObject;

namespace av {

// Forward declarations
class RigidBody;

// Physics world wrapper
class PhysicsWorld {
public:
    PhysicsWorld();
    ~PhysicsWorld();

    // Initialize physics world
    void initialize();

    // Shut down physics world
    void shutdown();

    // Step physics simulation
    void step(float deltaTime);

    // Create a rigid body
    std::shared_ptr<RigidBody> createRigidBody(float mass, const Vec3& shape);

    // Remove a rigid body
    void removeRigidBody(const std::shared_ptr<RigidBody>& body);

    // Set gravity
    void setGravity(const Vec3& gravity);

    // Get gravity
    Vec3 getGravity() const;

    // Ray cast query
    bool raycast(const Ray& ray, float maxDistance, Vec3& hitPoint, Vec3& hitNormal);

    // Get underlying Bullet physics world
    btDynamicsWorld* getBulletWorld() const;

private:
    std::unique_ptr<btDynamicsWorld> dynamicsWorld_;
    std::vector<std::shared_ptr<RigidBody>> bodies_;

    void setupPhysicsWorld();
};

// Rigid body wrapper
class RigidBody {
public:
    RigidBody(float mass, const Vec3& shape);
    ~RigidBody();

    // Physics properties
    void setPosition(const Vec3& position);
    Vec3 getPosition() const;

    void setRotation(const Quat& rotation);
    Quat getRotation() const;

    void setVelocity(const Vec3& velocity);
    Vec3 getVelocity() const;

    void setAngularVelocity(const Vec3& angularVel);
    Vec3 getAngularVelocity() const;

    // Apply forces
    void applyForce(const Vec3& force, const Vec3& relativePosition);
    void applyTorque(const Vec3& torque);

    // Mass and inertia
    void setMass(float mass);
    float getMass() const;

    // Get underlying Bullet rigid body
    btRigidBody* getBulletBody() const;

private:
    std::unique_ptr<btRigidBody> body_;
    float mass_;

    friend class PhysicsWorld;
};

} // namespace av
