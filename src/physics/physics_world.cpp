#include "av/physics/physics_world.hpp"
#include "av/foundation/logging.hpp"
#include <btBulletDynamicsCommon.h>

namespace av {

PhysicsWorld::PhysicsWorld() {
    AV_DEBUG("PhysicsWorld created");
}

PhysicsWorld::~PhysicsWorld() {
    shutdown();
    AV_DEBUG("PhysicsWorld destroyed");
}

void PhysicsWorld::initialize() {
    AV_INFO("Initializing physics world");

    // Create broadphase collision detection
    auto broadphase = std::make_unique<btDbvtBroadphase>();

    // Create collision dispatcher
    btCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
    auto dispatcher = std::make_unique<btCollisionDispatcher>(collisionConfiguration);

    // Create constraint solver
    auto solver = std::make_unique<btSequentialImpulseConstraintSolver>();

    // Create dynamics world
    dynamicsWorld_ = std::make_unique<btDiscreteDynamicsWorld>(
        dispatcher.get(), broadphase.get(), solver.get(), collisionConfiguration
    );

    // Release ownership from unique_ptrs since btDynamicsWorld takes ownership
    broadphase.release();
    solver.release();

    // Set default gravity
    dynamicsWorld_->setGravity(btVector3(0, -9.81f, 0));

    AV_INFO("Physics world initialized successfully");
}

void PhysicsWorld::shutdown() {
    AV_INFO("Shutting down physics world");
    bodies_.clear();
    if (dynamicsWorld_) {
        // Clean up all collision objects
        for (int i = dynamicsWorld_->getNumCollisionObjects() - 1; i >= 0; i--) {
            btCollisionObject* obj = dynamicsWorld_->getCollisionObjectArray()[i];
            btRigidBody* body = btRigidBody::upcast(obj);
            if (body && body->getMotionState()) {
                delete body->getMotionState();
            }
            dynamicsWorld_->removeCollisionObject(obj);
            delete obj;
        }
    }
    dynamicsWorld_.reset();
}

void PhysicsWorld::step(float deltaTime) {
    if (dynamicsWorld_) {
        // Fixed timestep stepping (10 substeps per frame is typical)
        dynamicsWorld_->stepSimulation(deltaTime, 10, 1.0f / 100.0f);
    }
}

std::shared_ptr<RigidBody> PhysicsWorld::createRigidBody(float mass, const Vec3& shape) {
    if (!dynamicsWorld_) {
        AV_ERROR("Physics world not initialized");
        return nullptr;
    }

    auto body = std::make_shared<RigidBody>(mass, shape);
    bodies_.push_back(body);

    // Add to physics world
    if (body->getBulletBody()) {
        dynamicsWorld_->addRigidBody(body->getBulletBody());
    }

    return body;
}

void PhysicsWorld::removeRigidBody(const std::shared_ptr<RigidBody>& body) {
    auto it = std::find(bodies_.begin(), bodies_.end(), body);
    if (it != bodies_.end()) {
        if (dynamicsWorld_ && body->getBulletBody()) {
            dynamicsWorld_->removeRigidBody(body->getBulletBody());
        }
        bodies_.erase(it);
    }
}

void PhysicsWorld::setGravity(const Vec3& gravity) {
    if (dynamicsWorld_) {
        dynamicsWorld_->setGravity(btVector3(gravity.x(), gravity.y(), gravity.z()));
        AV_DEBUG("Gravity set to: ({}, {}, {})", gravity.x(), gravity.y(), gravity.z());
    }
}

Vec3 PhysicsWorld::getGravity() const {
    if (dynamicsWorld_) {
        btVector3 g = dynamicsWorld_->getGravity();
        return Vec3(g.x(), g.y(), g.z());
    }
    return Vec3(0.0f, -9.81f, 0.0f);
}

bool PhysicsWorld::raycast(const Ray& ray, float maxDistance, Vec3& hitPoint, Vec3& hitNormal) {
    if (!dynamicsWorld_) {
        return false;
    }

    btVector3 rayFrom(ray.origin.x(), ray.origin.y(), ray.origin.z());
    btVector3 rayTo = rayFrom + btVector3(ray.direction.x(), ray.direction.y(), ray.direction.z()) * maxDistance;

    btCollisionWorld::ClosestRayResultCallback rayCallback(rayFrom, rayTo);
    dynamicsWorld_->rayTest(rayFrom, rayTo, rayCallback);

    if (rayCallback.hasHit()) {
        hitPoint = Vec3(rayCallback.m_hitPointWorld.x(), rayCallback.m_hitPointWorld.y(), rayCallback.m_hitPointWorld.z());
        hitNormal = Vec3(rayCallback.m_hitNormalWorld.x(), rayCallback.m_hitNormalWorld.y(), rayCallback.m_hitNormalWorld.z());
        return true;
    }

    return false;
}

btDynamicsWorld* PhysicsWorld::getBulletWorld() const {
    return dynamicsWorld_.get();
}

// RigidBody implementation

RigidBody::RigidBody(float mass, const Vec3& shape)
    : mass_(mass) {
    // Create collision shape (box shape for simplicity)
    btCollisionShape* collisionShape = new btBoxShape(btVector3(shape.x() / 2.0f, shape.y() / 2.0f, shape.z() / 2.0f));

    // Create motion state
    btDefaultMotionState* motionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0)));

    // Calculate inertia
    btVector3 localInertia(0, 0, 0);
    if (mass_ > 0.0f) {
        collisionShape->calculateLocalInertia(mass_, localInertia);
    }

    // Create rigid body
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass_, motionState, collisionShape, localInertia);
    body_ = std::make_unique<btRigidBody>(rbInfo);

    // Set some reasonable defaults
    body_->setLinearDamping(0.04f);
    body_->setAngularDamping(0.04f);

    AV_DEBUG("RigidBody created with mass: {}", mass);
}

RigidBody::~RigidBody() {
    if (body_) {
        if (body_->getMotionState()) {
            delete body_->getMotionState();
        }
        if (body_->getCollisionShape()) {
            delete body_->getCollisionShape();
        }
    }
    AV_DEBUG("RigidBody destroyed");
}

void RigidBody::setPosition(const Vec3& position) {
    if (body_) {
        btTransform transform = body_->getWorldTransform();
        transform.setOrigin(btVector3(position.x(), position.y(), position.z()));
        body_->setWorldTransform(transform);
    }
}

Vec3 RigidBody::getPosition() const {
    if (body_) {
        btVector3 pos = body_->getWorldTransform().getOrigin();
        return Vec3(pos.x(), pos.y(), pos.z());
    }
    return Vec3::Zero();
}

void RigidBody::setRotation(const Quat& rotation) {
    if (body_) {
        btTransform transform = body_->getWorldTransform();
        transform.setRotation(btQuaternion(rotation.x(), rotation.y(), rotation.z(), rotation.w()));
        body_->setWorldTransform(transform);
    }
}

Quat RigidBody::getRotation() const {
    if (body_) {
        btQuaternion btQuat = body_->getWorldTransform().getRotation();
        return Quat(btQuat.x(), btQuat.y(), btQuat.z(), btQuat.w());
    }
    return Quat::Identity();
}

void RigidBody::setVelocity(const Vec3& velocity) {
    if (body_) {
        body_->setLinearVelocity(btVector3(velocity.x(), velocity.y(), velocity.z()));
    }
}

Vec3 RigidBody::getVelocity() const {
    if (body_) {
        btVector3 vel = body_->getLinearVelocity();
        return Vec3(vel.x(), vel.y(), vel.z());
    }
    return Vec3::Zero();
}

void RigidBody::setAngularVelocity(const Vec3& angularVel) {
    if (body_) {
        body_->setAngularVelocity(btVector3(angularVel.x(), angularVel.y(), angularVel.z()));
    }
}

Vec3 RigidBody::getAngularVelocity() const {
    if (body_) {
        btVector3 angVel = body_->getAngularVelocity();
        return Vec3(angVel.x(), angVel.y(), angVel.z());
    }
    return Vec3::Zero();
}

void RigidBody::applyForce(const Vec3& force, const Vec3& relativePosition) {
    if (body_) {
        body_->applyForce(
            btVector3(force.x(), force.y(), force.z()),
            btVector3(relativePosition.x(), relativePosition.y(), relativePosition.z())
        );
    }
}

void RigidBody::applyTorque(const Vec3& torque) {
    if (body_) {
        body_->applyTorque(btVector3(torque.x(), torque.y(), torque.z()));
    }
}

void RigidBody::setMass(float mass) {
    mass_ = mass;
    if (body_ && body_->getCollisionShape()) {
        btVector3 localInertia(0, 0, 0);
        if (mass_ > 0.0f) {
            body_->getCollisionShape()->calculateLocalInertia(mass_, localInertia);
        }
        body_->setMassProps(mass_, localInertia);
    }
}

float RigidBody::getMass() const {
    return mass_;
}

btRigidBody* RigidBody::getBulletBody() const {
    return body_.get();
}

} // namespace av
