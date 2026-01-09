#include "av/sensors/lidar_visualizer.hpp"
#include "av/rendering/scene.hpp"
#include "av/foundation/logging.hpp"
#include <cmath>

namespace av {

LidarVisualizer::LidarVisualizer(std::shared_ptr<Scene> scene)
    : scene_(scene) {
    AV_DEBUG("LidarVisualizer created");
}

LidarVisualizer::~LidarVisualizer() {
    AV_DEBUG("LidarVisualizer destroyed");
}

void LidarVisualizer::visualizePointCloud(const PointCloud& cloud, const Vec3& sensorPosition) {
    if (!config_.showPoints) return;

    AV_DEBUG("Visualizing {} LIDAR points", cloud.size());

    // In a real implementation, would render points using the rendering system
    // For now, we just log the visualization
    int numPoints = cloud.size();
    if (numPoints > 0) {
        // Calculate point cloud statistics for debugging
        float minDist = cloud.distance[0];
        float maxDist = cloud.distance[0];
        float sumDist = 0.0f;

        for (size_t i = 0; i < cloud.distance.size(); ++i) {
            minDist = std::min(minDist, cloud.distance[i]);
            maxDist = std::max(maxDist, cloud.distance[i]);
            sumDist += cloud.distance[i];
        }

        float avgDist = sumDist / numPoints;

        AV_DEBUG("Point cloud stats: min_dist=%.2f, max_dist=%.2f, avg_dist=%.2f",
                 minDist, maxDist, avgDist);
    }
}

Vec3 LidarVisualizer::getColorByRing(int ring, int totalRings) const {
    // Distribute colors across the spectrum based on ring
    float t = (totalRings > 1) ? ring / (float)(totalRings - 1) : 0.5f;
    return getViridisColor(t);
}

Vec3 LidarVisualizer::getColorByDistance(float distance, float maxDistance) const {
    // Use heat map: blue (close) -> red (far)
    float t = std::min(1.0f, distance / maxDistance);
    return getHeatmapColor(t);
}

Vec3 LidarVisualizer::getColorByIntensity(float intensity) const {
    // Grayscale based on intensity (0-255)
    float normalized = intensity / 255.0f;
    return Vec3(normalized, normalized, normalized);
}

Vec3 LidarVisualizer::getHeatmapColor(float value) const {
    // Heat map: blue -> cyan -> green -> yellow -> red
    // Clamp value to [0, 1]
    value = std::max(0.0f, std::min(1.0f, value));

    float r, g, b;

    if (value < 0.25f) {
        // Blue to cyan
        float t = value / 0.25f;
        r = 0.0f;
        g = t;
        b = 1.0f;
    } else if (value < 0.5f) {
        // Cyan to green
        float t = (value - 0.25f) / 0.25f;
        r = 0.0f;
        g = 1.0f;
        b = 1.0f - t;
    } else if (value < 0.75f) {
        // Green to yellow
        float t = (value - 0.5f) / 0.25f;
        r = t;
        g = 1.0f;
        b = 0.0f;
    } else {
        // Yellow to red
        float t = (value - 0.75f) / 0.25f;
        r = 1.0f;
        g = 1.0f - t;
        b = 0.0f;
    }

    return Vec3(r, g, b);
}

Vec3 LidarVisualizer::getViridisColor(float t) const {
    // Viridis colormap approximation
    t = std::max(0.0f, std::min(1.0f, t));

    // Simple approximation of viridis
    float r, g, b;

    if (t < 0.5f) {
        // Purple to blue-green
        float u = t * 2.0f;
        r = (1.0f - u) * 0.27f + u * 0.0f;
        g = (1.0f - u) * 0.0f + u * 0.63f;
        b = (1.0f - u) * 0.33f + u * 0.98f;
    } else {
        // Blue-green to yellow
        float u = (t - 0.5f) * 2.0f;
        r = (1.0f - u) * 0.0f + u * 1.0f;
        g = (1.0f - u) * 0.63f + u * 0.96f;
        b = (1.0f - u) * 0.98f + u * 0.07f;
    }

    return Vec3(r, g, b);
}

void LidarVisualizer::visualizeStatistics(const LidarSensor& sensor, const Vec3& screenPosition) {
    // In a real implementation, would render text/graphs to screen
    // For now, just log the statistics

    const auto& cloud = sensor.getPointCloud();
    int pointCount = cloud.size();
    float updateDuration = sensor.getLastUpdateDuration();

    AV_DEBUG("LIDAR Stats: {} points, {:.3f}ms per update",
             pointCount, updateDuration * 1000.0f);
}

} // namespace av
