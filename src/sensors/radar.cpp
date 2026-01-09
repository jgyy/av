#include "av/sensors/radar.hpp"
#include "av/world/world.hpp"
#include "av/foundation/logging.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>
#include <random>

namespace av {

static std::mt19937 radarRNG(std::random_device{}());

RadarSensor::RadarSensor() : RadarSensor(Config()) {
}

RadarSensor::RadarSensor(const Config& config)
    : config_(config) {
    setStatus(Status::UNINITIALIZED);
    AV_DEBUG("RadarSensor created: {}x{} at {} Hz",
             config.width, config.height, config.updateRate);
}

bool RadarSensor::initialize() {
    setStatus(Status::ACTIVE);
    timeSinceLastUpdate_ = 0.0f;
    AV_INFO("RadarSensor initialized successfully");
    return true;
}

void RadarSensor::update(float deltaTime) {
    timeSinceLastUpdate_ += deltaTime;
    float updateInterval = 1.0f / config_.updateRate;

    if (timeSinceLastUpdate_ >= updateInterval) {
        auto startTime = std::chrono::high_resolution_clock::now();

        // Perform radar scan
        performRadarScan();

        frameCount_++;
        timeSinceLastUpdate_ = 0.0f;

        auto endTime = std::chrono::high_resolution_clock::now();
        lastUpdateDuration_ = std::chrono::duration<float>(endTime - startTime).count();
        lastDetectionCount_ = detections_.size();

        if (frameCount_ % 10 == 0) {
            AV_DEBUG("Radar update: {:.3f}ms, detections: {}",
                     lastUpdateDuration_ * 1000.0f, lastDetectionCount_);
        }
    }
}

void RadarSensor::performRadarScan() {
    detections_.clear();

    if (!world_) {
        return;
    }

    // Get traffic vehicles from world
    const auto& vehicles = world_->getTrafficVehicles();

    std::uniform_real_distribution<float> uniformDist(0.0f, 1.0f);

    for (const auto& vehicle : vehicles) {
        if (!vehicle) continue;

        // Vector from radar to vehicle
        Vec3 relPos = vehicle->getPosition() - position_;
        float range = relPos.norm();

        // Check range constraints
        if (range < config_.minRange || range > config_.maxRange) continue;

        // Compute angles
        float azimuth = std::atan2(relPos.y(), relPos.x());
        float elevation = std::asin(relPos.z() / std::max(range, 0.001f));

        // Check angular FOV
        if (std::abs(azimuth) > config_.horizontalFOV * 3.14159f / 180.0f / 2.0f) continue;
        if (std::abs(elevation) > config_.verticalFOV * 3.14159f / 180.0f / 2.0f) continue;

        // Compute RCS (radar cross section)
        float rcs = computeRCS("vehicle");

        // Compute detection amplitude
        float amplitude = computeDetectionAmplitude(range, rcs);

        // Check detection threshold
        if (amplitude < std::pow(10.0f, config_.detectionThreshold / 20.0f)) continue;

        // Compute Doppler velocity if enabled
        float velocity = 0.0f;
        if (config_.enableDoppler) {
            velocity = computeDopplerVelocity(vehicle->getPosition(), vehicle->getVelocity());
        }

        // Create detection
        RadarDetection detection;
        detection.position = vehicle->getPosition();
        detection.range = range;
        detection.azimuth = azimuth;
        detection.elevation = elevation;
        detection.velocity = velocity;
        detection.amplitude = amplitude;
        detection.trackId = static_cast<int>(detections_.size());

        detections_.push_back(detection);
    }

    // Add some false positives for realism
    std::uniform_real_distribution<float> clutter(0.0f, 1.0f);
    if (clutter(radarRNG) < 0.05f) {  // 5% chance of clutter detection
        std::uniform_real_distribution<float> rangeDist(config_.minRange, config_.maxRange);
        std::uniform_real_distribution<float> angleDist(-config_.horizontalFOV / 2.0f, config_.horizontalFOV / 2.0f);

        RadarDetection clutterDet;
        clutterDet.range = rangeDist(radarRNG);
        clutterDet.azimuth = angleDist(radarRNG) * 3.14159f / 180.0f;
        clutterDet.elevation = 0.0f;
        clutterDet.velocity = addClutterNoise(clutterDet.range);
        clutterDet.amplitude = 0.3f;
        clutterDet.position = position_ + Vec3(
            clutterDet.range * std::cos(clutterDet.azimuth),
            clutterDet.range * std::sin(clutterDet.azimuth),
            0.0f
        );

        detections_.push_back(clutterDet);
    }
}

float RadarSensor::computeRCS(const std::string& objectType) const {
    if (objectType == "vehicle") {
        return config_.vehicleRCS;
    } else if (objectType == "pedestrian") {
        return config_.pedestrianRCS;
    }
    return config_.vehicleRCS;
}

float RadarSensor::computeDetectionAmplitude(float range, float rcs) const {
    // Radar equation: SNR = RCS / (range^4)
    // Simplified with constant gain and noise floor
    float rangeAttenuation = 1.0f / (range * range);
    float rcsLinear = std::pow(10.0f, rcs / 20.0f);
    return rangeAttenuation * rcsLinear * 0.1f;  // Scale factor
}

float RadarSensor::computeDopplerVelocity(const Vec3& targetPos, const Vec3& targetVel) const {
    // Doppler: v_radial = (v · r_normalized)
    Vec3 relPos = targetPos - position_;
    if (relPos.norm() < 0.001f) return 0.0f;

    Vec3 relDir = relPos.normalized();
    float radialVelocity = targetVel.dot(relDir);

    // Add Doppler noise
    std::normal_distribution<float> noiseDist(0.0f, 0.1f);
    radialVelocity += noiseDist(radarRNG);

    return radialVelocity;
}

float RadarSensor::addClutterNoise(float range) const {
    std::normal_distribution<float> noiseDist(0.0f, 0.5f);
    return noiseDist(radarRNG);
}

} // namespace av
