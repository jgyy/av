#pragma once

#include "av/foundation/math.hpp"
#include <memory>

namespace av {

// Forward declaration
class VehicleDynamics;

// Vehicle controller for manual input
class VehicleController {
public:
    VehicleController(std::shared_ptr<VehicleDynamics> dynamics);
    ~VehicleController() = default;

    // Input handling (normalized values 0-1)
    void setSteeringInput(float steering);     // -1 to 1 (left to right)
    void setThrottleInput(float throttle);     // 0 to 1
    void setBrakeInput(float brake);           // 0 to 1
    void setHandbrakeInput(bool handbrake);    // True/False

    // Get current inputs
    float getSteeringInput() const { return steeringInput_; }
    float getThrottleInput() const { return throttleInput_; }
    float getBrakeInput() const { return brakeInput_; }
    bool getHandbrakeInput() const { return handbrakeInput_; }

    // Update vehicle based on inputs
    void update(float deltaTime);

    // Reset all inputs
    void reset();

private:
    std::shared_ptr<VehicleDynamics> dynamics_;

    // Current inputs (normalized)
    float steeringInput_ = 0.0f;
    float throttleInput_ = 0.0f;
    float brakeInput_ = 0.0f;
    bool handbrakeInput_ = false;

    // Input smoothing (for realistic feel)
    float steeringSmoothing_ = 0.15f;
    float throttleSmoothing_ = 0.1f;
    float brakeSmoothing_ = 0.1f;

    // Current smoothed values
    float smoothedSteering_ = 0.0f;
    float smoothedThrottle_ = 0.0f;
    float smoothedBrake_ = 0.0f;
};

} // namespace av
