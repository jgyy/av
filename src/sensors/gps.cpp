#include "av/sensors/gps.hpp"
#include "av/foundation/logging.hpp"
#include <cmath>
#include <random>
#include <chrono>

namespace av {

static std::mt19937 gpsRNG(std::random_device{}());

GPSSensor::GPSSensor(const Config& config)
    : config_(config) {
    setStatus(Status::UNINITIALIZED);
    lastMeasurement_.satelliteCount = config_.minSatellites;
    AV_DEBUG("GPSSensor created with {} satellites target", config_.minSatellites);
}

bool GPSSensor::initialize() {
    setStatus(Status::ACTIVE);
    timeSinceLastUpdate_ = 0.0f;
    hasFix_ = true;
    AV_INFO("GPSSensor initialized successfully");
    return true;
}

void GPSSensor::update(float deltaTime) {
    timeSinceLastUpdate_ += deltaTime;
    float updateInterval = 1.0f / config_.updateRate;

    if (timeSinceLastUpdate_ >= updateInterval) {
        auto startTime = std::chrono::high_resolution_clock::now();

        // Generate measurement
        generateMeasurement();

        // Apply noise
        applyNoise();

        // Simulate position drift
        simulateDrift(deltaTime);

        frameCount_++;
        timeSinceLastUpdate_ = 0.0f;

        auto endTime = std::chrono::high_resolution_clock::now();
        lastUpdateDuration_ = std::chrono::duration<float>(endTime - startTime).count();

        if (frameCount_ % 10 == 0) {
            AV_DEBUG("GPS update: {:.3f}ms, satellites: {}, fix: {}",
                     lastUpdateDuration_ * 1000.0f, lastMeasurement_.satelliteCount,
                     hasFix_ ? "yes" : "no");
        }
    }
}

void GPSSensor::generateMeasurement() {
    // Use true position from world reference
    lastMeasurement_.position = position_;
    lastMeasurement_.velocityX = 0.0f;
    lastMeasurement_.velocityY = 0.0f;
    lastMeasurement_.velocityZ = 0.0f;

    // Update satellite count
    lastMeasurement_.satelliteCount = computeSatelliteCount();

    // Determine if we have a fix
    hasFix_ = lastMeasurement_.satelliteCount >= config_.minSatellites;

    // Set accuracy based on satellite count
    float satFactor = std::min(1.0f, lastMeasurement_.satelliteCount / 12.0f);
    lastMeasurement_.accuracy = config_.horizontalAccuracy / satFactor;
    lastMeasurement_.altitudeAccuracy = config_.verticalAccuracy / satFactor;
}

void GPSSensor::applyNoise() {
    if (!hasFix_) {
        return;
    }

    // Apply horizontal accuracy noise (CEP - Circular Error Probable)
    std::normal_distribution<float> horizNoise(0.0f, config_.horizontalAccuracy);
    std::normal_distribution<float> vertNoise(0.0f, config_.verticalAccuracy);

    lastMeasurement_.position.x() += horizNoise(gpsRNG) + config_.multiPathBias;
    lastMeasurement_.position.y() += horizNoise(gpsRNG) + config_.atmosphericDelay;
    lastMeasurement_.position.z() += vertNoise(gpsRNG);

    // Apply velocity noise
    std::normal_distribution<float> velNoise(0.0f, config_.velocityAccuracy);
    lastMeasurement_.velocityX = velNoise(gpsRNG);
    lastMeasurement_.velocityY = velNoise(gpsRNG);
    lastMeasurement_.velocityZ = velNoise(gpsRNG);
}

void GPSSensor::simulateDrift(float deltaTime) {
    if (!config_.enableDrift) {
        return;
    }

    // Random walk position drift
    std::normal_distribution<float> driftDist(0.0f, config_.driftRate * deltaTime);
    currentDrift_.x() += driftDist(gpsRNG);
    currentDrift_.y() += driftDist(gpsRNG);
    currentDrift_.z() += driftDist(gpsRNG);

    // Apply accumulated drift to measurement
    lastMeasurement_.position = lastMeasurement_.position + currentDrift_;
}

int GPSSensor::computeSatelliteCount() const {
    // Simulate satellite visibility with random variation
    std::uniform_real_distribution<float> satDist(0.0f, 1.0f);

    // Start with nominal count
    int count = config_.minSatellites + 2;

    // Sometimes lose satellites (simulating occlusion)
    if (satDist(gpsRNG) < config_.signalLossChance) {
        count = std::max(1, count - 2);
    }

    // Add some extra satellites randomly
    if (satDist(gpsRNG) < 0.3f) {
        count += 2;
    }

    return std::min(count, 24);  // Max typical satellites
}

} // namespace av
