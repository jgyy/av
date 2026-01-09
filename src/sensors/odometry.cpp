#include "av/sensors/odometry.hpp"
#include "av/foundation/logging.hpp"
#include <cmath>
#include <random>
#include <chrono>

namespace av {

static std::mt19937 odometryRNG(std::random_device{}());

OdometrySensor::OdometrySensor(const Config& config)
    : config_(config) {
    setStatus(Status::UNINITIALIZED);
    // Simulate wheel radius calibration error
    std::normal_distribution<float> calibError(1.0f, 0.02f);
    wheelRadiusError_ = calibError(odometryRNG);
    AV_DEBUG("OdometrySensor created, wheelbase: {} m", config.wheelBase);
}

bool OdometrySensor::initialize() {
    setStatus(Status::ACTIVE);
    timeSinceLastUpdate_ = 0.0f;
    lastRealPosition_ = position_;
    lastRealHeading_ = 0.0f;
    AV_INFO("OdometrySensor initialized successfully");
    return true;
}

void OdometrySensor::update(float deltaTime) {
    timeSinceLastUpdate_ += deltaTime;
    float updateInterval = 1.0f / config_.updateRate;

    if (timeSinceLastUpdate_ >= updateInterval) {
        auto startTime = std::chrono::high_resolution_clock::now();

        // Generate wheel speed measurements
        generateMeasurement();

        // Integrate motion model
        integrateMotion(deltaTime);

        // Apply wheel slip
        if (config_.enableSlipDrift) {
            applyWheelSlip();
        }

        // Accumulate error
        accumulateError();

        frameCount_++;
        timeSinceLastUpdate_ = 0.0f;

        auto endTime = std::chrono::high_resolution_clock::now();
        lastUpdateDuration_ = std::chrono::duration<float>(endTime - startTime).count();

        if (frameCount_ % 20 == 0) {
            AV_DEBUG("Odometry update: {:.3f}ms, position: ({:.2f}, {:.2f}, {:.2f}), error: {:.3f}m",
                     lastUpdateDuration_ * 1000.0f,
                     estimatedPosition_.x(), estimatedPosition_.y(), estimatedPosition_.z(),
                     accumulatedError_);
        }
    }
}

void OdometrySensor::generateMeasurement() {
    // Compute wheel speeds from position change (would normally come from wheel encoders)
    float wheelSpeed = 0.5f;  // m/s nominal
    float slipGain = 1.0f + (config_.slipFactor * 0.1f);

    // Add measurement noise
    std::normal_distribution<float> speedNoise(0.0f, config_.wheelSpeedNoise);
    lastMeasurement_.leftWheelSpeed = wheelSpeed * slipGain + speedNoise(odometryRNG);
    lastMeasurement_.rightWheelSpeed = wheelSpeed * slipGain + speedNoise(odometryRNG);

    // Compute integrated motion
    lastMeasurement_.cumulativeDistance += (lastMeasurement_.leftWheelSpeed + lastMeasurement_.rightWheelSpeed) / 2.0f * (1.0f / config_.updateRate);

    // Compute velocity from wheel speeds
    float avgSpeed = (lastMeasurement_.leftWheelSpeed + lastMeasurement_.rightWheelSpeed) / 2.0f;
    lastMeasurement_.velocityX = avgSpeed * std::cos(estimatedHeading_);
    lastMeasurement_.velocityY = avgSpeed * std::sin(estimatedHeading_);

    // Compute angular velocity (yaw rate from differential wheel speeds)
    float speedDiff = lastMeasurement_.rightWheelSpeed - lastMeasurement_.leftWheelSpeed;
    lastMeasurement_.angularVelocity = (speedDiff / config_.wheelBase) * config_.updateRate;
    lastMeasurement_.heading = estimatedHeading_;
}

void OdometrySensor::integrateMotion(float deltaTime) {
    // Simple kinematic model integration
    float avgSpeed = (lastMeasurement_.leftWheelSpeed + lastMeasurement_.rightWheelSpeed) / 2.0f;
    float yawRate = lastMeasurement_.angularVelocity;

    // Update estimated heading
    estimatedHeading_ += yawRate * deltaTime;

    // Update estimated position (simple unicycle model)
    estimatedPosition_.x() += avgSpeed * std::cos(estimatedHeading_) * deltaTime;
    estimatedPosition_.y() += avgSpeed * std::sin(estimatedHeading_) * deltaTime;
}

void OdometrySensor::applyWheelSlip() {
    // Apply random slip factor to simulate soft terrain or tire degradation
    std::uniform_real_distribution<float> slipDist(-config_.slipFactor, config_.slipFactor);

    float slipX = slipDist(odometryRNG);
    float slipY = slipDist(odometryRNG);

    estimatedPosition_.x() += slipX;
    estimatedPosition_.y() += slipY;
}

void OdometrySensor::accumulateError() {
    // Track error between estimated and real position
    // (In real system, this would diverge over time without correction)
    Vec3 positionError = estimatedPosition_ - position_;
    accumulatedError_ = positionError.norm();
}

void OdometrySensor::resetEstimate(const Vec3& position, float heading) {
    estimatedPosition_ = position;
    estimatedHeading_ = heading;
    accumulatedError_ = 0.0f;
    lastMeasurement_.cumulativeDistance = 0.0f;
}

} // namespace av
