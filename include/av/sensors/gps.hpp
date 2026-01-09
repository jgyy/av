#pragma once

#include "sensor.hpp"
#include "av/foundation/math.hpp"
#include <vector>

namespace av {

// GPS measurement structure
struct GPSMeasurement {
    Vec3 position;           // Latitude, Longitude in degrees, Altitude in meters
    float accuracy = 5.0f;   // Horizontal accuracy (CEP) in meters
    float altitudeAccuracy = 10.0f;  // Vertical accuracy in meters
    float velocityX = 0.0f;  // Velocity in m/s
    float velocityY = 0.0f;
    float velocityZ = 0.0f;
    float timeOfWeek = 0.0f; // GPS time of week in seconds
    int satelliteCount = 0;  // Number of visible satellites
};

// GPS sensor with realistic noise model
class GPSSensor : public Sensor {
public:
    // GPS configuration
    struct Config {
        float updateRate = 10.0f;      // Hz
        float horizontalAccuracy = 5.0f;  // CEP in meters
        float verticalAccuracy = 10.0f;   // Altitude error in meters
        float velocityAccuracy = 0.1f;    // Velocity error in m/s

        // Noise model parameters
        float multiPathBias = 0.0f;    // Systematic multipath error in meters
        float atmosphericDelay = 0.0f; // Ionospheric delay in meters
        float driftRate = 0.01f;       // Position drift rate per second in m/s

        // Acquisition parameters
        int minSatellites = 4;         // Minimum satellites for fix
        float signalLossChance = 0.01f; // Probability of signal loss
        bool enableDrift = true;       // Enable position drift over time
    };

    explicit GPSSensor(const Config& config);
    GPSSensor();
    ~GPSSensor() = default;

    // Sensor interface
    bool initialize() override;
    void update(float deltaTime) override;
    std::string getName() const override { return "GPS"; }

    // GPS-specific data access
    const GPSMeasurement& getMeasurement() const { return lastMeasurement_; }
    bool hasFix() const { return hasFix_; }
    int getSatelliteCount() const { return lastMeasurement_.satelliteCount; }

    // Configuration
    void setConfig(const Config& config) { config_ = config; }
    const Config& getConfig() const { return config_; }

    // Statistics
    float getLastUpdateDuration() const { return lastUpdateDuration_; }

private:
    Config config_;
    GPSMeasurement lastMeasurement_;
    bool hasFix_ = false;

    float timeSinceLastUpdate_ = 0.0f;
    float lastUpdateDuration_ = 0.0f;
    Vec3 currentDrift_ = Vec3::Zero();  // Accumulated position drift

    // Internal measurement pipeline
    void generateMeasurement();
    void applyNoise();
    void simulateDrift(float deltaTime);
    int computeSatelliteCount() const;
};

} // namespace av
