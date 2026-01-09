#include "av/physics/vehicle_dynamics.hpp"
#include "av/foundation/logging.hpp"
#include <cmath>
#include <algorithm>

namespace av {

// Simplified Pacejka tire model (Magic Formula)
float TireModel::calculateLateralForce(float slipAngle, float normalForce) const {
    // Simplified linear tire model for now
    // Full Pacejka model would use: D*sin(C*atan(B*x - E*(B*x - atan(B*x))))
    // where x is the slip angle

    // Limit slip angle to avoid numerical issues
    slipAngle = clamp(slipAngle, -PI / 2.0f, PI / 2.0f);

    // Simplified model: lateral force proportional to slip angle up to peak friction
    float lateralForce = stiffnessCoefficient * slipAngle * normalForce;

    // Peak friction limit
    float peakForce = peakFriction * normalForce;
    return clamp(lateralForce, -peakForce, peakForce);
}

float TireModel::calculateLongitudinalForce(float slipRatio, float normalForce) const {
    // Simplified longitudinal tire model
    // Slip ratio ranges from -1 (locked wheel) to 0 (no slip) to +inf (wheel spinning)

    // Limit slip ratio
    slipRatio = clamp(slipRatio, -1.0f, 2.0f);

    // Simplified force calculation
    float maxForce = peakFriction * normalForce;

    // Use tanh for smooth saturation
    float force = maxForce * std::tanh(slipRatio * 5.0f);
    return force;
}

VehicleDynamics::VehicleDynamics(const VehicleParams& params)
    : params_(params) {
    AV_INFO("VehicleDynamics initialized");
    AV_DEBUG("  Mass: {} kg", params_.mass);
    AV_DEBUG("  Wheelbase: {} m", params_.wheelbase);
    AV_DEBUG("  Track Width: {} m", params_.trackWidth);
    AV_DEBUG("  Max Speed: {} m/s", params_.maxSpeed);
}

void VehicleDynamics::setSteeringAngle(float angle) {
    steeringAngle_ = clamp(angle, -params_.maxSteeringAngle, params_.maxSteeringAngle);
}

void VehicleDynamics::setThrottle(float throttle) {
    throttle_ = clamp(throttle, 0.0f, 1.0f);
}

void VehicleDynamics::setBrake(float brake) {
    brake_ = clamp(brake, 0.0f, 1.0f);
}

Vec3 VehicleDynamics::getPosition() const {
    return position_;
}

Quat VehicleDynamics::getRotation() const {
    return rotation_;
}

Vec3 VehicleDynamics::getVelocity() const {
    return velocity_;
}

Vec3 VehicleDynamics::getAngularVelocity() const {
    return angularVelocity_;
}

void VehicleDynamics::setPosition(const Vec3& position) {
    position_ = position;
}

void VehicleDynamics::setRotation(const Quat& rotation) {
    rotation_ = rotation;
}

void VehicleDynamics::setVelocity(const Vec3& velocity) {
    velocity_ = velocity;
}

void VehicleDynamics::update(float deltaTime) {
    // Clamp deltaTime to prevent instability
    deltaTime = clamp(deltaTime, 0.0f, 0.1f);

    // Get current speed (scalar)
    float speed = velocity_.norm();

    // Get current heading (yaw angle from rotation)
    Vec3 euler = quatToEuler(rotation_);
    float yaw = euler.z();
    float yawRate = 0.0f;

    // Steering angle in radians
    float steeringRad = toRadians(steeringAngle_);

    // Bicycle model for vehicle kinematics
    // Simplified model: rear axle is origin
    if (speed > 0.1f) {
        // Calculate yaw rate using bicycle model
        // yaw_rate = (speed * sin(steering_angle)) / wheelbase
        yawRate = (speed * std::sin(steeringRad)) / params_.wheelbase;

        // Limit yaw rate to reasonable values
        yawRate = clamp(yawRate, -PI, PI);
    }

    // Longitudinal dynamics
    // Calculate acceleration based on throttle and brake
    float acceleration = 0.0f;
    if (throttle_ > 0.0f) {
        // Accelerate - but reduce acceleration at high speeds
        float speedFactor = 1.0f - (speed / params_.maxSpeed) * 0.8f;
        acceleration = throttle_ * params_.maxAcceleration * speedFactor;
    } else if (brake_ > 0.0f) {
        // Brake
        acceleration = brake_ * params_.maxBraking;
    } else {
        // Drag (friction) - natural deceleration
        acceleration = -speed * 0.1f;
    }

    // Limit speed
    if (speed > params_.maxSpeed && acceleration > 0.0f) {
        acceleration = 0.0f;
    }

    // Apply drag
    float dragForce = 0.5f * params_.dragCoefficient * speed * speed;
    if (speed > 0.1f) {
        acceleration -= dragForce / params_.mass;
    }

    // Update velocity (longitudinal direction only for now)
    speed += acceleration * deltaTime;
    speed = clamp(speed, 0.0f, params_.maxSpeed);

    // Update position and rotation using kinematic model
    Vec3 forward = rotation_ * Vec3::UnitZ();
    position_ += forward * speed * deltaTime;

    // Update yaw rotation
    yaw += yawRate * deltaTime;

    // Create new quaternion from euler angles
    rotation_ = quatFromEuler(0.0f, 0.0f, yaw);

    // Update velocity vector
    velocity_ = forward * speed;

    // Update angular velocity (yaw rate)
    angularVelocity_ = Vec3(0.0f, yawRate, 0.0f);

    AV_TRACE("Vehicle state - Pos: ({:.2f}, {:.2f}, {:.2f}) Speed: {:.2f} m/s, Yaw: {:.2f}°",
             position_.x(), position_.y(), position_.z(), speed, toDegrees(yaw));
}

void VehicleDynamics::calculateForces() {
    // Reserved for more complex force calculations
    // Currently handled in update() method with simplified model
}

void VehicleDynamics::integrateVelocity(float deltaTime) {
    // Kinematic integration is handled in update()
}

} // namespace av
