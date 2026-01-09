#include "av/physics/vehicle_controller.hpp"
#include "av/physics/vehicle_dynamics.hpp"
#include "av/foundation/logging.hpp"

namespace av {

VehicleController::VehicleController(std::shared_ptr<VehicleDynamics> dynamics)
    : dynamics_(dynamics) {
    if (!dynamics_) {
        AV_ERROR("VehicleController: dynamics is null");
    } else {
        AV_DEBUG("VehicleController created");
    }
}

void VehicleController::setSteeringInput(float steering) {
    steeringInput_ = clamp(steering, -1.0f, 1.0f);
}

void VehicleController::setThrottleInput(float throttle) {
    throttleInput_ = clamp(throttle, 0.0f, 1.0f);
}

void VehicleController::setBrakeInput(float brake) {
    brakeInput_ = clamp(brake, 0.0f, 1.0f);
}

void VehicleController::setHandbrakeInput(bool handbrake) {
    handbrakeInput_ = handbrake;
}

void VehicleController::update(float deltaTime) {
    if (!dynamics_) {
        return;
    }

    // Apply input smoothing for realistic feel
    float steeringChangeRate = 1.0f / steeringSmoothing_;
    smoothedSteering_ += (steeringInput_ - smoothedSteering_) * std::min(deltaTime * steeringChangeRate, 1.0f);

    float throttleChangeRate = 1.0f / throttleSmoothing_;
    smoothedThrottle_ += (throttleInput_ - smoothedThrottle_) * std::min(deltaTime * throttleChangeRate, 1.0f);

    float brakeChangeRate = 1.0f / brakeSmoothing_;
    smoothedBrake_ += (brakeInput_ - smoothedBrake_) * std::min(deltaTime * brakeChangeRate, 1.0f);

    // Handle handbrake (overrides brake input)
    float finalBrake = handbrakeInput_ ? 1.0f : smoothedBrake_;

    // Convert normalized inputs to vehicle parameters
    const VehicleParams& params = dynamics_->getParams();
    float steeringAngle = smoothedSteering_ * params.maxSteeringAngle;

    // Apply inputs to vehicle dynamics
    dynamics_->setSteeringAngle(steeringAngle);
    dynamics_->setThrottle(smoothedThrottle_);
    dynamics_->setBrake(finalBrake);

    // Update vehicle dynamics
    dynamics_->update(deltaTime);

    // Log inputs occasionally for debugging
    static float logTimer = 0.0f;
    logTimer += deltaTime;
    if (logTimer >= 1.0f) {
        AV_TRACE("Controller input - Steering: {:.2f}, Throttle: {:.2f}, Brake: {:.2f}",
                 smoothedSteering_, smoothedThrottle_, smoothedBrake_);
        logTimer = 0.0f;
    }
}

void VehicleController::reset() {
    steeringInput_ = 0.0f;
    throttleInput_ = 0.0f;
    brakeInput_ = 0.0f;
    handbrakeInput_ = false;

    smoothedSteering_ = 0.0f;
    smoothedThrottle_ = 0.0f;
    smoothedBrake_ = 0.0f;

    AV_DEBUG("VehicleController reset");
}

} // namespace av
