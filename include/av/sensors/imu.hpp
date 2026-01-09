#pragma once

#include "sensor.hpp"
#include "av/foundation/math.hpp"

namespace av {

// IMU measurement structure
struct IMUMeasurement {
    Vec3 acceleration;      // Acceleration in m/s^2 (x, y, z)
    Vec3 angularVelocity;   // Angular velocity in rad/s (roll, pitch, yaw)
    Vec3 magneticField;     // Magnetic field in Gauss
    float temperature = 25.0f;  // Sensor temperature in Celsius
    float timeOfMeasurement = 0.0f;
};

// IMU sensor with accelerometer, gyroscope, and drift models
class IMUSensor : public Sensor {
public:
    // IMU configuration
    struct Config {
        float updateRate = 100.0f;     // Hz (typically 100-200 Hz)

        // Accelerometer noise model
        float accelNoiseStdDev = 0.01f;  // Standard deviation in m/s^2
        float accelBiasDrift = 0.0001f;  // Bias drift rate in m/s^3

        // Gyroscope noise model
        float gyroNoiseStdDev = 0.001f;  // Standard deviation in rad/s
        float gyroBiasDrift = 0.00001f;  // Bias drift rate in rad/s^2

        // Magnetic field parameters
        float magneticFieldStrength = 50.0f;  // Local magnetic field in Gauss
        bool enableMagnetometer = true;       // Enable magnetic field measurement

        // Temperature sensitivity
        float temperatureSensitivity = 0.01f; // Drift per degree Celsius

        // Gravity reference
        bool compensateGravity = true;       // Subtract gravity from acceleration
    };

    IMUSensor(const Config& config = Config());
    ~IMUSensor() = default;

    // Sensor interface
    bool initialize() override;
    void update(float deltaTime) override;
    std::string getName() const override { return "IMU"; }

    // IMU-specific data access
    const IMUMeasurement& getMeasurement() const { return lastMeasurement_; }
    const Vec3& getAccelerationBias() const { return accelBias_; }
    const Vec3& getGyroBias() const { return gyroBias_; }

    // Configuration
    void setConfig(const Config& config) { config_ = config; }
    const Config& getConfig() const { return config_; }

    // Calibration
    void resetBias() { accelBias_ = Vec3::Zero(); gyroBias_ = Vec3::Zero(); }

    // Statistics
    float getLastUpdateDuration() const { return lastUpdateDuration_; }

private:
    Config config_;
    IMUMeasurement lastMeasurement_;
    Vec3 accelBias_ = Vec3::Zero();
    Vec3 gyroBias_ = Vec3::Zero();

    float timeSinceLastUpdate_ = 0.0f;
    float lastUpdateDuration_ = 0.0f;
    float currentTemperature_ = 25.0f;

    // Internal measurement pipeline
    void generateMeasurement();
    Vec3 addAccelNoise(const Vec3& idealAccel);
    Vec3 addGyroNoise(const Vec3& idealGyro);
    Vec3 generateMagneticField() const;
    void simulateBiasDrift(float deltaTime);
};

} // namespace av
