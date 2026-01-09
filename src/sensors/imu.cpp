#include "av/sensors/imu.hpp"
#include "av/foundation/logging.hpp"
#include <cmath>
#include <random>
#include <chrono>

namespace av {

static std::mt19937 imuRNG(std::random_device{}());

IMUSensor::IMUSensor() : IMUSensor(Config()) {
}

IMUSensor::IMUSensor(const Config& config)
    : config_(config) {
    setStatus(Status::UNINITIALIZED);
    AV_DEBUG("IMUSensor created with {} Hz update rate", config.updateRate);
}

bool IMUSensor::initialize() {
    setStatus(Status::ACTIVE);
    timeSinceLastUpdate_ = 0.0f;
    accelBias_ = Vec3::Zero();
    gyroBias_ = Vec3::Zero();
    AV_INFO("IMUSensor initialized successfully");
    return true;
}

void IMUSensor::update(float deltaTime) {
    timeSinceLastUpdate_ += deltaTime;
    float updateInterval = 1.0f / config_.updateRate;

    if (timeSinceLastUpdate_ >= updateInterval) {
        auto startTime = std::chrono::high_resolution_clock::now();

        // Generate measurement
        generateMeasurement();

        // Simulate bias drift
        simulateBiasDrift(deltaTime);

        frameCount_++;
        timeSinceLastUpdate_ = 0.0f;

        auto endTime = std::chrono::high_resolution_clock::now();
        lastUpdateDuration_ = std::chrono::duration<float>(endTime - startTime).count();

        if (frameCount_ % 50 == 0) {
            AV_DEBUG("IMU update: {:.3f}ms, accel: ({:.3f}, {:.3f}, {:.3f})",
                     lastUpdateDuration_ * 1000.0f,
                     lastMeasurement_.acceleration.x(),
                     lastMeasurement_.acceleration.y(),
                     lastMeasurement_.acceleration.z());
        }
    }
}

void IMUSensor::generateMeasurement() {
    // Ideal acceleration from world (would require velocity differentiation)
    Vec3 idealAccel = Vec3::Zero();
    if (config_.compensateGravity) {
        idealAccel.z() = 9.81f;  // Gravity compensation
    }

    // Add noise to acceleration
    lastMeasurement_.acceleration = addAccelNoise(idealAccel) + accelBias_;

    // Ideal angular velocity (would require orientation differentiation)
    Vec3 idealGyro = Vec3::Zero();

    // Add noise to gyroscope
    lastMeasurement_.angularVelocity = addGyroNoise(idealGyro) + gyroBias_;

    // Generate magnetic field
    if (config_.enableMagnetometer) {
        lastMeasurement_.magneticField = generateMagneticField();
    }

    lastMeasurement_.temperature = currentTemperature_;
    lastMeasurement_.timeOfMeasurement = frameCount_ * (1.0f / config_.updateRate);
}

Vec3 IMUSensor::addAccelNoise(const Vec3& idealAccel) {
    std::normal_distribution<float> noiseDist(0.0f, config_.accelNoiseStdDev);
    return Vec3(
        idealAccel.x() + noiseDist(imuRNG),
        idealAccel.y() + noiseDist(imuRNG),
        idealAccel.z() + noiseDist(imuRNG)
    );
}

Vec3 IMUSensor::addGyroNoise(const Vec3& idealGyro) {
    std::normal_distribution<float> noiseDist(0.0f, config_.gyroNoiseStdDev);
    return Vec3(
        idealGyro.x() + noiseDist(imuRNG),
        idealGyro.y() + noiseDist(imuRNG),
        idealGyro.z() + noiseDist(imuRNG)
    );
}

Vec3 IMUSensor::generateMagneticField() const {
    // Simulate local magnetic field (typically points north with downward component)
    float inclination = 65.0f * 3.14159f / 180.0f;  // Typical inclination angle
    float declination = 0.0f;  // Magnetic declination

    float strength = config_.magneticFieldStrength;
    float x = strength * std::cos(inclination) * std::cos(declination);
    float y = strength * std::cos(inclination) * std::sin(declination);
    float z = strength * std::sin(inclination);

    // Add small noise to magnetic field
    std::normal_distribution<float> noiseDist(0.0f, 0.5f);
    return Vec3(x + noiseDist(imuRNG), y + noiseDist(imuRNG), z + noiseDist(imuRNG));
}

void IMUSensor::simulateBiasDrift(float deltaTime) {
    // Random walk bias drift
    std::normal_distribution<float> accelDrift(0.0f, config_.accelBiasDrift * deltaTime);
    std::normal_distribution<float> gyroDrift(0.0f, config_.gyroBiasDrift * deltaTime);

    accelBias_.x() += accelDrift(imuRNG);
    accelBias_.y() += accelDrift(imuRNG);
    accelBias_.z() += accelDrift(imuRNG);

    gyroBias_.x() += gyroDrift(imuRNG);
    gyroBias_.y() += gyroDrift(imuRNG);
    gyroBias_.z() += gyroDrift(imuRNG);

    // Simulate temperature drift
    std::normal_distribution<float> tempDrift(0.0f, 0.01f);
    currentTemperature_ += tempDrift(imuRNG);

    // Apply temperature sensitivity to bias
    float tempDeviation = currentTemperature_ - 25.0f;
    accelBias_ = accelBias_ * (1.0f + config_.temperatureSensitivity * tempDeviation);
}

} // namespace av
