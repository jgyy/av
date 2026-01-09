#include "av/sensors/ultrasonic.hpp"
#include "av/world/world.hpp"
#include "av/foundation/logging.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>
#include <random>

namespace av {

static std::mt19937 ultrasonicRNG(std::random_device{}());

UltrasonicSensor::UltrasonicSensor() : UltrasonicSensor(Config()) {
}

UltrasonicSensor::UltrasonicSensor(const Config& config)
    : config_(config) {
    setStatus(Status::UNINITIALIZED);
    measurements_.resize(config.numSensors);
    initializeSensorDirections();
    AV_DEBUG("UltrasonicSensor created with {} transducers", config.numSensors);
}

bool UltrasonicSensor::initialize() {
    setStatus(Status::ACTIVE);
    timeSinceLastUpdate_ = 0.0f;
    AV_INFO("UltrasonicSensor initialized successfully");
    return true;
}

void UltrasonicSensor::update(float deltaTime) {
    timeSinceLastUpdate_ += deltaTime;
    float updateInterval = 1.0f / config_.updateRate;

    if (timeSinceLastUpdate_ >= updateInterval) {
        auto startTime = std::chrono::high_resolution_clock::now();

        // Perform ultrasonic scan
        performUltrasonicScan();

        frameCount_++;
        timeSinceLastUpdate_ = 0.0f;

        auto endTime = std::chrono::high_resolution_clock::now();
        lastUpdateDuration_ = std::chrono::duration<float>(endTime - startTime).count();
        lastDetectionCount_ = getDetectionCount();

        if (frameCount_ % 10 == 0) {
            AV_DEBUG("Ultrasonic update: {:.3f}ms, detections: {}",
                     lastUpdateDuration_ * 1000.0f, lastDetectionCount_);
        }
    }
}

void UltrasonicSensor::initializeSensorDirections() {
    sensorDirections_.clear();

    if (config_.enableAngularArrangement) {
        // Arrange sensors in circular pattern around vehicle
        for (int i = 0; i < config_.numSensors; ++i) {
            float angle = (2.0f * 3.14159f * i) / config_.numSensors;
            Vec3 dir(std::cos(angle), std::sin(angle), 0.0f);
            sensorDirections_.push_back(dir);
        }
    } else {
        // Forward-facing sensors
        for (int i = 0; i < config_.numSensors; ++i) {
            sensorDirections_.push_back(Vec3(1.0f, 0.0f, 0.0f));
        }
    }
}

void UltrasonicSensor::performUltrasonicScan() {
    if (!world_) {
        return;
    }

    std::uniform_real_distribution<float> falseDist(0.0f, 1.0f);

    for (int i = 0; i < config_.numSensors; ++i) {
        // Compute distance for this sensor
        float distance = computeDistance(sensorDirections_[i]);

        // Add measurement noise
        addMeasurementNoise(distance);

        // Determine if detection occurred
        bool detected = distance > 0.0f && distance <= config_.maxRange;

        // Add occasional false positives
        if (!detected && falseDist(ultrasonicRNG) < config_.falsePositiveProbability) {
            distance = 0.5f + falseDist(ultrasonicRNG) * (config_.maxRange - 0.5f);
            detected = true;
        }

        measurements_[i].distance = detected ? distance : -1.0f;
        measurements_[i].objectDetected = detected;
        measurements_[i].confidence = detected ? std::max(0.5f, 1.0f - distance / config_.maxRange) : 0.0f;
    }
}

float UltrasonicSensor::computeDistance(const Vec3& rayDirection) const {
    if (!world_) {
        return -1.0f;
    }

    const auto& vehicles = world_->getTrafficVehicles();
    float closestDistance = config_.maxRange + 1.0f;

    // Check against vehicles (simple sphere collision)
    for (const auto& vehicle : vehicles) {
        if (!vehicle) continue;

        Vec3 relPos = vehicle->getPosition() - position_;
        float projectionDist = relPos.dot(rayDirection);

        if (projectionDist > config_.minRange && projectionDist < closestDistance) {
            closestDistance = projectionDist;
        }
    }

    return closestDistance <= config_.maxRange ? closestDistance : -1.0f;
}

void UltrasonicSensor::addMeasurementNoise(float& distance) const {
    if (distance < 0.0f) return;

    std::normal_distribution<float> noiseDist(0.0f, config_.rangeNoise);
    distance += noiseDist(ultrasonicRNG);
    distance = std::max(config_.minRange, distance);
}

float UltrasonicSensor::getClosestDistance() const {
    float closest = config_.maxRange;
    for (const auto& measurement : measurements_) {
        if (measurement.objectDetected && measurement.distance > 0.0f) {
            closest = std::min(closest, measurement.distance);
        }
    }
    return closest < config_.maxRange ? closest : -1.0f;
}

int UltrasonicSensor::getDetectionCount() const {
    int count = 0;
    for (const auto& measurement : measurements_) {
        if (measurement.objectDetected) count++;
    }
    return count;
}

} // namespace av
