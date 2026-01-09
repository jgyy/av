#pragma once

#include "sensor.hpp"
#include "av/foundation/math.hpp"
#include <vector>
#include <memory>

namespace av {

// Point cloud data structure
struct PointCloud {
    std::vector<Vec3> points;      // 3D point positions
    std::vector<float> intensity;  // Intensity/reflectivity values (0-255)
    std::vector<int> ring;         // Which vertical channel the point came from
    std::vector<float> distance;   // Distance from sensor to point

    size_t size() const { return points.size(); }
    void clear() {
        points.clear();
        intensity.clear();
        ring.clear();
        distance.clear();
    }
};

// LIDAR sensor with configurable parameters
class LidarSensor : public Sensor {
public:
    // LIDAR Configuration
    struct Config {
        int numChannels = 64;              // Vertical channels
        int pointsPerSecond = 1000000;     // Total points per second
        float maxRange = 120.0f;           // Maximum detection range (meters)
        float minRange = 0.2f;             // Minimum detection range (meters)
        float horizontalFOV = 360.0f;      // Horizontal field of view (degrees)
        float verticalFOV = 26.8f;         // Vertical field of view (degrees)
        float updateRate = 10.0f;          // Updates per second (Hz)
        float angleResolution = 0.2f;      // Horizontal angle resolution (degrees)

        // Noise parameters
        float gaussianNoiseStdDev = 0.02f; // Gaussian noise std dev (meters)
        float outlierProbability = 0.01f;  // Probability of outlier
        float outlierDistance = 50.0f;     // Outlier distance (meters)
    };

    explicit LidarSensor(const Config& config);
    LidarSensor();
    ~LidarSensor() = default;

    // Sensor interface
    bool initialize() override;
    void update(float deltaTime) override;
    std::string getName() const override { return "LIDAR"; }

    // Data access
    const PointCloud& getPointCloud() const { return pointCloud_; }
    const Config& getConfig() const { return config_; }

    // Configuration
    void setConfig(const Config& config) { config_ = config; }
    void setMaxRange(float range) { config_.maxRange = range; }
    void setNumChannels(int channels) { config_.numChannels = channels; }
    void setUpdateRate(float hz) { config_.updateRate = hz; }

    // Sensor control
    void enable() { enabled_ = true; }
    void disable() { enabled_ = false; }
    bool isEnabled() const { return enabled_; }

    // Statistics
    int getLastPointCount() const { return lastPointCount_; }
    float getLastUpdateDuration() const { return lastUpdateDuration_; }

private:
    Config config_;
    PointCloud pointCloud_;
    bool enabled_ = true;

    float timeSinceLastUpdate_ = 0.0f;
    int lastPointCount_ = 0;
    float lastUpdateDuration_ = 0.0f;

    // Ray-casting simulation
    void raycastWorld(float timestamp);
    void addNoise(PointCloud& cloud);
    void addGaussianNoise(Vec3& point);
    void addOutliers(PointCloud& cloud);

    // Helper functions
    bool rayIntersectsWorld(const Vec3& origin, const Vec3& direction,
                           float maxDist, Vec3& hitPoint, float& distance);
    int getRandomRing() const;
};

} // namespace av
