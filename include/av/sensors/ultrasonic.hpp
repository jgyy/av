#pragma once

#include "sensor.hpp"
#include "av/foundation/math.hpp"
#include <vector>

namespace av {

// Ultrasonic measurement structure
struct UltrasonicMeasurement {
    float distance = -1.0f;  // Distance in meters (-1 if no detection)
    float confidence = 0.0f; // Confidence level (0-1)
    bool objectDetected = false;
};

// Ultrasonic sensor for short-range proximity detection
class UltrasonicSensor : public Sensor {
public:
    // Ultrasonic configuration
    struct Config {
        int numSensors = 12;            // Number of ultrasonic transducers
        float maxRange = 4.0f;          // Maximum range in meters
        float minRange = 0.05f;         // Minimum range in meters
        float updateRate = 20.0f;       // Hz
        float beamAngle = 30.0f;        // Beam cone angle in degrees

        // Noise model
        float rangeNoise = 0.05f;       // Range noise std dev in meters
        float falsePositiveProbability = 0.02f;  // Probability of false detection

        // Sensor placement (arranged around vehicle)
        bool enableAngularArrangement = true;  // Sensors arranged in circle
    };

    UltrasonicSensor(const Config& config = Config());
    ~UltrasonicSensor() = default;

    // Sensor interface
    bool initialize() override;
    void update(float deltaTime) override;
    std::string getName() const override { return "Ultrasonic"; }

    // Ultrasonic-specific data access
    const std::vector<UltrasonicMeasurement>& getMeasurements() const { return measurements_; }
    float getClosestDistance() const;
    int getDetectionCount() const;

    // Configuration
    void setConfig(const Config& config) { config_ = config; }
    const Config& getConfig() const { return config_; }

    // Statistics
    float getLastUpdateDuration() const { return lastUpdateDuration_; }
    int getLastDetectionCount() const { return lastDetectionCount_; }

private:
    Config config_;
    std::vector<UltrasonicMeasurement> measurements_;
    std::vector<Vec3> sensorDirections_;  // Direction for each sensor

    float timeSinceLastUpdate_ = 0.0f;
    float lastUpdateDuration_ = 0.0f;
    int lastDetectionCount_ = 0;

    // Internal detection pipeline
    void performUltrasonicScan();
    void initializeSensorDirections();
    float computeDistance(const Vec3& rayDirection) const;
    void addMeasurementNoise(float& distance) const;
};

} // namespace av
