#pragma once

#include "av/foundation/math.hpp"
#include "av/sensors/lidar.hpp"
#include <memory>

namespace av {

// Forward declarations
class Scene;

// LIDAR point cloud visualization
class LidarVisualizer {
public:
    struct VisualizerConfig {
        bool showPoints = true;
        bool colorByRing = true;
        bool colorByDistance = false;
        bool colorByIntensity = false;
        float pointSize = 2.0f;
        float maxDistance = 50.0f;  // For coloring
    };

    LidarVisualizer(std::shared_ptr<Scene> scene);
    ~LidarVisualizer();

    // Visualization control
    void visualizePointCloud(const PointCloud& cloud, const Vec3& sensorPosition = Vec3::Zero());
    void setConfig(const VisualizerConfig& config) { config_ = config; }

    // Color schemes
    Vec3 getColorByRing(int ring, int totalRings) const;
    Vec3 getColorByDistance(float distance, float maxDistance) const;
    Vec3 getColorByIntensity(float intensity) const;

    // Statistics visualization
    void visualizeStatistics(const LidarSensor& sensor, const Vec3& screenPosition);

private:
    std::shared_ptr<Scene> scene_;
    VisualizerConfig config_;

    // Helper methods
    Vec3 getHeatmapColor(float value) const;
    Vec3 getViridisColor(float t) const;
};

} // namespace av
