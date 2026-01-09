#include <gtest/gtest.h>
#include "av/physics/physics_world.hpp"
#include "av/physics/vehicle_dynamics.hpp"
#include "av/physics/vehicle_controller.hpp"

using namespace av;

class PhysicsWorldTest : public ::testing::Test {
protected:
    void SetUp() override {
        world = std::make_unique<PhysicsWorld>();
        world->initialize();
    }

    void TearDown() override {
        world->shutdown();
    }

    std::unique_ptr<PhysicsWorld> world;
};

TEST_F(PhysicsWorldTest, Initialize) {
    EXPECT_NE(world->getBulletWorld(), nullptr);
}

TEST_F(PhysicsWorldTest, SetGravity) {
    Vec3 gravity(0.0f, -20.0f, 0.0f);
    world->setGravity(gravity);
    Vec3 retrievedGravity = world->getGravity();
    EXPECT_NEAR(retrievedGravity.y(), -20.0f, 0.01f);
}

TEST_F(PhysicsWorldTest, CreateRigidBody) {
    auto body = world->createRigidBody(1000.0f, Vec3(2.0f, 1.5f, 4.0f));
    EXPECT_NE(body, nullptr);
    EXPECT_FLOAT_EQ(body->getMass(), 1000.0f);
}

TEST_F(PhysicsWorldTest, RigidBodyPosition) {
    auto body = world->createRigidBody(100.0f, Vec3(1.0f, 1.0f, 1.0f));
    Vec3 position(5.0f, 10.0f, 15.0f);
    body->setPosition(position);
    Vec3 retrieved = body->getPosition();
    EXPECT_NEAR(retrieved.x(), 5.0f, 0.01f);
    EXPECT_NEAR(retrieved.y(), 10.0f, 0.01f);
    EXPECT_NEAR(retrieved.z(), 15.0f, 0.01f);
}

TEST_F(PhysicsWorldTest, RigidBodyVelocity) {
    auto body = world->createRigidBody(100.0f, Vec3(1.0f, 1.0f, 1.0f));
    Vec3 velocity(10.0f, 0.0f, 5.0f);
    body->setVelocity(velocity);
    Vec3 retrieved = body->getVelocity();
    EXPECT_NEAR(retrieved.x(), 10.0f, 0.01f);
    EXPECT_NEAR(retrieved.y(), 0.0f, 0.01f);
    EXPECT_NEAR(retrieved.z(), 5.0f, 0.01f);
}

class VehicleDynamicsTest : public ::testing::Test {
protected:
    VehicleParams params;
    std::unique_ptr<VehicleDynamics> dynamics;

    void SetUp() override {
        params.mass = 1500.0f;
        params.wheelbase = 2.7f;
        params.maxSpeed = 50.0f;
        params.maxAcceleration = 5.0f;
        dynamics = std::make_unique<VehicleDynamics>(params);
    }
};

TEST_F(VehicleDynamicsTest, Initialization) {
    EXPECT_FLOAT_EQ(dynamics->getParams().mass, 1500.0f);
}

TEST_F(VehicleDynamicsTest, SetPosition) {
    Vec3 position(10.0f, 0.0f, 20.0f);
    dynamics->setPosition(position);
    Vec3 retrieved = dynamics->getPosition();
    EXPECT_EQ(retrieved, position);
}

TEST_F(VehicleDynamicsTest, SetRotation) {
    Quat rotation = Quat::Identity();
    dynamics->setRotation(rotation);
    Quat retrieved = dynamics->getRotation();
    EXPECT_NEAR(retrieved.w(), 1.0f, 0.01f);
}

TEST_F(VehicleDynamicsTest, Throttle) {
    dynamics->setThrottle(0.5f);
    EXPECT_FLOAT_EQ(dynamics->getThrottle(), 0.5f);
}

TEST_F(VehicleDynamicsTest, Steering) {
    dynamics->setSteeringAngle(15.0f);
    EXPECT_FLOAT_EQ(dynamics->getSteeringAngle(), 15.0f);
}

TEST_F(VehicleDynamicsTest, MaxSteeringAngle) {
    dynamics->setSteeringAngle(50.0f); // Exceed max (30°)
    EXPECT_FLOAT_EQ(dynamics->getSteeringAngle(), 30.0f); // Clamped
}

TEST_F(VehicleDynamicsTest, UpdateAcceleration) {
    dynamics->setPosition(Vec3::Zero());
    dynamics->setVelocity(Vec3::Zero());
    dynamics->setThrottle(1.0f);

    // Simulate one frame
    dynamics->update(0.01f);

    // Should have accelerated
    Vec3 velocity = dynamics->getVelocity();
    EXPECT_GT(velocity.norm(), 0.0f);
}

TEST_F(VehicleDynamicsTest, MaxSpeedLimit) {
    dynamics->setVelocity(dynamics->getParams().maxSpeed * Vec3::UnitZ());
    dynamics->setThrottle(1.0f);

    // Simulate multiple frames
    for (int i = 0; i < 100; i++) {
        dynamics->update(0.01f);
    }

    // Speed should not exceed max
    float speed = dynamics->getVelocity().norm();
    EXPECT_LE(speed, dynamics->getParams().maxSpeed * 1.01f); // Allow 1% tolerance
}

class VehicleControllerTest : public ::testing::Test {
protected:
    VehicleParams params;
    std::shared_ptr<VehicleDynamics> dynamics;
    std::unique_ptr<VehicleController> controller;

    void SetUp() override {
        params.mass = 1500.0f;
        params.wheelbase = 2.7f;
        params.maxSteeringAngle = 30.0f;
        dynamics = std::make_shared<VehicleDynamics>(params);
        controller = std::make_unique<VehicleController>(dynamics);
    }
};

TEST_F(VehicleControllerTest, SteeringInput) {
    controller->setSteeringInput(0.5f);
    EXPECT_FLOAT_EQ(controller->getSteeringInput(), 0.5f);
}

TEST_F(VehicleControllerTest, ThrottleInput) {
    controller->setThrottleInput(0.75f);
    EXPECT_FLOAT_EQ(controller->getThrottleInput(), 0.75f);
}

TEST_F(VehicleControllerTest, BrakeInput) {
    controller->setBrakeInput(0.3f);
    EXPECT_FLOAT_EQ(controller->getBrakeInput(), 0.3f);
}

TEST_F(VehicleControllerTest, InputClamping) {
    controller->setSteeringInput(2.0f);
    EXPECT_FLOAT_EQ(controller->getSteeringInput(), 1.0f);

    controller->setThrottleInput(-0.5f);
    EXPECT_FLOAT_EQ(controller->getThrottleInput(), 0.0f);
}

TEST_F(VehicleControllerTest, Update) {
    controller->setThrottleInput(1.0f);
    controller->update(0.01f);

    // Vehicle should have accelerated
    float speed = dynamics->getVelocity().norm();
    EXPECT_GT(speed, 0.0f);
}

TEST_F(VehicleControllerTest, Steering) {
    controller->setThrottleInput(0.5f);
    controller->setSteeringInput(0.5f); // Turn right

    // Update several times to apply steering
    for (int i = 0; i < 100; i++) {
        controller->update(0.01f);
    }

    // Vehicle heading should have changed
    Vec3 forward = dynamics->getRotation() * Vec3::UnitZ();
    // Should not be pointing straight ahead (Z direction)
    EXPECT_LT(std::abs(forward.x()), 1.0f); // X component should be non-zero (turning)
}
