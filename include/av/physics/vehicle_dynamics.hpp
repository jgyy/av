#pragma once

#include "av/foundation/math.hpp"
#include <memory>

namespace av {

// Forward declaration
class RigidBody;

// Vehicle parameters
struct VehicleParams {
    float mass = 1500.0f;           // kg
    float wheelbase = 2.7f;         // m (distance between front and rear axles)
    float trackWidth = 1.6f;        // m (distance between left and right wheels)
    float maxSteeringAngle = 30.0f; // degrees
    float maxAcceleration = 5.0f;   // m/s^2
    float maxBraking = -10.0f;      // m/s^2
    float maxSpeed = 50.0f;         // m/s
    float dragCoefficient = 0.3f;
    float frictionCoefficient = 0.8f;
};

// Tire model (simplified Pacejka)
struct TireModel {
    float stiffnessCoefficient = 180000.0f; // N/rad
    float peakFriction = 1.0f;
    float rigidbodyContactAreaRadius = 0.02f;

    float calculateLateralForce(float slipAngle, float normalForce) const;
    float calculateLongitudinalForce(float slipRatio, float normalForce) const;
};

// Vehicle dynamics model
class VehicleDynamics {
public:
    VehicleDynamics(const VehicleParams& params);
    ~VehicleDynamics() = default;

    // Set control inputs
    void setSteeringAngle(float angle);   // In degrees
    void setThrottle(float throttle);     // 0 to 1
    void setBrake(float brake);           // 0 to 1

    // Get state
    Vec3 getPosition() const;
    Quat getRotation() const;
    Vec3 getVelocity() const;
    Vec3 getAngularVelocity() const;

    // Set state
    void setPosition(const Vec3& position);
    void setRotation(const Quat& rotation);
    void setVelocity(const Vec3& velocity);

    // Update vehicle dynamics
    void update(float deltaTime);

    // Get vehicle parameters
    const VehicleParams& getParams() const { return params_; }

    // Get tire model
    const TireModel& getTireModel() const { return tireModel_; }

    // Get control inputs
    float getSteeringAngle() const { return steeringAngle_; }
    float getThrottle() const { return throttle_; }
    float getBrake() const { return brake_; }

private:
    VehicleParams params_;
    TireModel tireModel_;

    // Control inputs
    float steeringAngle_ = 0.0f;
    float throttle_ = 0.0f;
    float brake_ = 0.0f;

    // State
    Vec3 position_ = Vec3::Zero();
    Quat rotation_ = Quat::Identity();
    Vec3 velocity_ = Vec3::Zero();
    Vec3 angularVelocity_ = Vec3::Zero();

    // Internal calculations
    void calculateForces();
    void integrateVelocity(float deltaTime);
};

} // namespace av
