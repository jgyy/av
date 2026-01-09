#pragma once

#include "sensor.hpp"
#include "av/foundation/math.hpp"
#include <vector>
#include <memory>

namespace av {

// Radar detection structure
struct RadarDetection {
    Vec3 position;           // World position of detected object
    float range = 0.0f;      // Range in meters
    float azimuth = 0.0f;    // Horizontal angle in radians
    float elevation = 0.0f;  // Vertical angle in radians
    float velocity = 0.0f;   // Doppler velocity (radial) in m/s
    float amplitude = 0.0f;  // Signal amplitude (0-1)
    int trackId = -1;        // Track ID for multi-frame tracking
};

// Radar sensor with cone-based detection and RCS model
class RadarSensor : public Sensor {
public:
    // Radar configuration
    struct Config {
        int width = 128;              // Horizontal resolution
        int height = 32;              // Vertical resolution
        float horizontalFOV = 120.0f; // Degrees
        float verticalFOV = 30.0f;    // Degrees
        float maxRange = 200.0f;      // Maximum range in meters
        float minRange = 0.5f;        // Minimum range in meters
        float updateRate = 10.0f;     // Hz

        // RCS (Radar Cross Section) model
        float vehicleRCS = 10.0f;     // Typical vehicle RCS in dBsm
        float pedestrianRCS = -5.0f;  // Typical pedestrian RCS in dBsm

        // Detection parameters
        float detectionThreshold = -20.0f;  // dBsm threshold for detection
        float rangeResolution = 0.2f;       // Range bin size in meters
        bool enableDoppler = true;          // Enable radial velocity measurement
        bool enableClutterNoise = true;     // Enable ground clutter simulation
    };

    RadarSensor(const Config& config = Config());
    ~RadarSensor() = default;

    // Sensor interface
    bool initialize() override;
    void update(float deltaTime) override;
    std::string getName() const override { return "Radar"; }

    // Radar-specific data access
    const std::vector<RadarDetection>& getDetections() const { return detections_; }
    int getDetectionCount() const { return detections_.size(); }

    // Configuration
    void setConfig(const Config& config) { config_ = config; }
    const Config& getConfig() const { return config_; }

    // Statistics
    float getLastUpdateDuration() const { return lastUpdateDuration_; }
    int getLastDetectionCount() const { return lastDetectionCount_; }

private:
    Config config_;
    std::vector<RadarDetection> detections_;

    float timeSinceLastUpdate_ = 0.0f;
    float lastUpdateDuration_ = 0.0f;
    int lastDetectionCount_ = 0;

    // Internal detection pipeline
    void performRadarScan();
    float computeRCS(const std::string& objectType) const;
    float computeDetectionAmplitude(float range, float rcs) const;
    float computeDopplerVelocity(const Vec3& targetPos, const Vec3& targetVel) const;
    float addClutterNoise(float range) const;
};

} // namespace av
