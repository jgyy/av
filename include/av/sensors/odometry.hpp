#pragma once

#include "sensor.hpp"
#include "av/foundation/math.hpp"

namespace av {

// Odometry measurement structure
struct OdometryMeasurement {
    float leftWheelSpeed = 0.0f;   // Left wheel speed in m/s
    float rightWheelSpeed = 0.0f;  // Right wheel speed in m/s
    float cumulativeDistance = 0.0f;  // Total distance traveled in meters
    float heading = 0.0f;          // Heading angle in radians
    float velocityX = 0.0f;        // Velocity x in m/s
    float velocityY = 0.0f;        // Velocity y in m/s
    float angularVelocity = 0.0f;  // Angular velocity in rad/s
};

// Odometry sensor measuring wheel speeds and integrated position
class OdometrySensor : public Sensor {
public:
    // Odometry configuration
    struct Config {
        float updateRate = 50.0f;      // Hz
        float wheelBase = 2.7f;        // Distance between wheels in meters
        float wheelRadius = 0.33f;     // Wheel radius in meters

        // Noise model
        float wheelSpeedNoise = 0.01f;     // Speed measurement noise (std dev) in m/s
        float slipFactor = 0.02f;          // Wheel slip simulation factor (0-1)
        float encoderResolution = 0.001f;  // Encoder resolution in meters

        // Drift
        bool enableSlipDrift = true;   // Enable wheel slip drift
        bool enableCalibrationError = true;  // Enable wheel radius calibration error
    };

    OdometrySensor(const Config& config = Config());
    ~OdometrySensor() = default;

    // Sensor interface
    bool initialize() override;
    void update(float deltaTime) override;
    std::string getName() const override { return "Odometry"; }

    // Odometry-specific data access
    const OdometryMeasurement& getMeasurement() const { return lastMeasurement_; }
    const Vec3& getEstimatedPosition() const { return estimatedPosition_; }
    float getEstimatedHeading() const { return estimatedHeading_; }

    // Configuration
    void setConfig(const Config& config) { config_ = config; }
    const Config& getConfig() const { return config_; }

    // Reset estimation
    void resetEstimate(const Vec3& position, float heading);

    // Statistics
    float getLastUpdateDuration() const { return lastUpdateDuration_; }
    float getAccumulatedError() const { return accumulatedError_; }

private:
    Config config_;
    OdometryMeasurement lastMeasurement_;

    Vec3 estimatedPosition_ = Vec3::Zero();
    float estimatedHeading_ = 0.0f;
    float accumulatedError_ = 0.0f;

    float timeSinceLastUpdate_ = 0.0f;
    float lastUpdateDuration_ = 0.0f;

    Vec3 lastRealPosition_ = Vec3::Zero();
    float lastRealHeading_ = 0.0f;
    float wheelRadiusError_ = 1.0f;  // Multiplicative error on wheel radius

    // Internal measurement pipeline
    void generateMeasurement();
    void integrateMotion(float deltaTime);
    void applyWheelSlip();
    void accumulateError();
};

} // namespace av
