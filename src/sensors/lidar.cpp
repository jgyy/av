#include "av/sensors/lidar.hpp"
#include "av/world/world.hpp"
#include "av/world/road_network.hpp"
#include "av/world/traffic.hpp"
#include "av/foundation/logging.hpp"
#include <cmath>
#include <random>
#include <chrono>

namespace av {

// Random number generation
static std::mt19937 g_rng(std::chrono::high_resolution_clock::now().time_since_epoch().count());
static std::uniform_real_distribution<float> g_uniform(0.0f, 1.0f);
static std::normal_distribution<float> g_normal(0.0f, 1.0f);

LidarSensor::LidarSensor(const Config& config)
    : config_(config) {
    setStatus(Status::UNINITIALIZED);
    AV_DEBUG("LidarSensor created: {} channels, {} Hz, max range {}m",
             config.numChannels, config.updateRate, config.maxRange);
}

bool LidarSensor::initialize() {
    setStatus(Status::ACTIVE);
    pointCloud_.clear();
    timeSinceLastUpdate_ = 0.0f;
    lastPointCount_ = 0;
    AV_INFO("LidarSensor initialized successfully");
    return true;
}

void LidarSensor::update(float deltaTime) {
    if (!enabled_ || getStatus() != Status::ACTIVE) {
        return;
    }

    timeSinceLastUpdate_ += deltaTime;
    float updateInterval = 1.0f / config_.updateRate;

    if (timeSinceLastUpdate_ >= updateInterval) {
        auto startTime = std::chrono::high_resolution_clock::now();

        // Clear previous point cloud
        pointCloud_.clear();

        // Perform ray-casting from all channels
        raycastWorld(timeSinceLastUpdate_);

        // Add realistic noise
        addNoise(pointCloud_);

        lastPointCount_ = pointCloud_.size();
        frameCount_++;
        timeSinceLastUpdate_ = 0.0f;

        auto endTime = std::chrono::high_resolution_clock::now();
        lastUpdateDuration_ = std::chrono::duration<float>(endTime - startTime).count();

        if (frameCount_ % 10 == 0) {
            AV_DEBUG("LIDAR update: {} points in {:.3f}ms",
                     lastPointCount_, lastUpdateDuration_ * 1000.0f);
        }
    }
}

void LidarSensor::raycastWorld(float timestamp) {
    if (!world_) {
        AV_WARN("LidarSensor: World not set");
        return;
    }

    auto roadNetwork = world_->getRoadNetwork();
    if (!roadNetwork) {
        AV_WARN("LidarSensor: Road network not available");
        return;
    }

    // Calculate vertical angle distribution
    float verticalFOV = config_.verticalFOV * (3.14159f / 180.0f);
    float verticalMin = -verticalFOV / 2.0f;
    float verticalMax = verticalFOV / 2.0f;

    // Cast rays for each channel
    for (int channel = 0; channel < config_.numChannels; ++channel) {
        // Vertical angle for this channel
        float alpha = verticalMin + (verticalMax - verticalMin) * (channel / (float)(config_.numChannels - 1));

        // Horizontal resolution
        float azimuthResolution = config_.angleResolution * (3.14159f / 180.0f);
        float horizontalFOV = config_.horizontalFOV * (3.14159f / 180.0f);

        // Number of rays for this channel
        int numRays = static_cast<int>(horizontalFOV / azimuthResolution);

        for (int ray = 0; ray < numRays; ++ray) {
            // Horizontal angle for this ray
            float beta = (-horizontalFOV / 2.0f) + azimuthResolution * ray;

            // Compute ray direction in sensor frame
            float x = std::cos(alpha) * std::sin(beta);
            float y = std::sin(alpha);
            float z = std::cos(alpha) * std::cos(beta);
            Vec3 rayDirection(x, y, z);

            // Transform to world frame
            // For simplicity, assume sensor is aligned with world axes
            // In real implementation, would apply rotation_ transformation

            // Trace ray and find intersection
            Vec3 hitPoint;
            float distance = 0.0f;

            if (rayIntersectsWorld(position_, rayDirection, config_.maxRange, hitPoint, distance)) {
                if (distance >= config_.minRange && distance <= config_.maxRange) {
                    // Add point to cloud
                    pointCloud_.points.push_back(hitPoint);
                    pointCloud_.distance.push_back(distance);
                    pointCloud_.ring.push_back(channel);

                    // Add intensity based on distance (inverse relationship)
                    float intensity = 255.0f * (1.0f - distance / config_.maxRange);
                    intensity = std::max(0.0f, std::min(255.0f, intensity));
                    pointCloud_.intensity.push_back(intensity);
                }
            }
        }
    }

    AV_DEBUG("LidarSensor raycasting: Generated {} points",
             pointCloud_.size());
}

void LidarSensor::addNoise(PointCloud& cloud) {
    // Add Gaussian noise
    for (auto& point : cloud.points) {
        addGaussianNoise(point);
    }

    // Add outliers
    addOutliers(cloud);
}

void LidarSensor::addGaussianNoise(Vec3& point) {
    // Add Gaussian noise with configurable std dev
    float noiseX = g_normal(g_rng) * config_.gaussianNoiseStdDev;
    float noiseY = g_normal(g_rng) * config_.gaussianNoiseStdDev;
    float noiseZ = g_normal(g_rng) * config_.gaussianNoiseStdDev;

    point = point + Vec3(noiseX, noiseY, noiseZ);
}

void LidarSensor::addOutliers(PointCloud& cloud) {
    int numOutliers = static_cast<int>(cloud.points.size() * config_.outlierProbability);

    for (int i = 0; i < numOutliers; ++i) {
        // Random point index
        int idx = static_cast<int>(g_uniform(g_rng) * cloud.points.size());
        if (idx >= static_cast<int>(cloud.points.size())) {
            idx = cloud.points.size() - 1;
        }

        // Replace with random distant point
        float randomDist = g_uniform(g_rng) * config_.outlierDistance;
        float randomAngle = g_uniform(g_rng) * 2.0f * 3.14159f;

        cloud.points[idx] = Vec3(
            position_.x() + randomDist * std::cos(randomAngle),
            position_.y() + g_uniform(g_rng) * 10.0f - 5.0f,
            position_.z() + randomDist * std::sin(randomAngle)
        );
    }
}

bool LidarSensor::rayIntersectsWorld(const Vec3& origin, const Vec3& direction,
                                     float maxDist, Vec3& hitPoint, float& distance) {
    if (!world_) return false;

    auto roadNetwork = world_->getRoadNetwork();
    if (!roadNetwork) return false;

    float closestDist = maxDist;
    Vec3 closestPoint = Vec3::Zero();
    bool hitFound = false;

    // Check intersection with lanes
    for (const auto& lane : roadNetwork->getLanes()) {
        if (!lane) continue;

        const auto& centerline = lane->getCenterline();
        if (centerline.size() < 2) continue;

        // Check ray against all lane centerline segments
        for (size_t i = 0; i + 1 < centerline.size(); ++i) {
            const Vec3& p1 = centerline[i];
            const Vec3& p2 = centerline[i + 1];

            // Simple ray-line segment intersection
            Vec3 pa = p1 - origin;
            Vec3 ba = p2 - p1;

            float denom = ba.dot(ba);
            if (denom < 0.001f) continue;

            float t = pa.dot(ba) / denom;
            if (t < 0.0f || t > 1.0f) continue;

            Vec3 closest = p1 + ba * t;
            float d = (closest - origin).norm();

            if (d > 0.01f && d < closestDist) {
                closestDist = d;
                closestPoint = closest;
                hitFound = true;
            }
        }
    }

    // Check intersection with traffic vehicles (as bounding spheres)
    for (const auto& vehicle : world_->getTrafficVehicles()) {
        if (!vehicle) continue;

        Vec3 vehiclePos = vehicle->getPosition();
        Vec3 toVehicle = vehiclePos - origin;
        float proj = toVehicle.dot(direction);

        if (proj > 0.0f && proj < maxDist) {
            Vec3 closest = origin + direction * proj;
            float dist = (closest - vehiclePos).norm();

            // Assume vehicle is ~2m in radius
            if (dist < 2.0f) {
                if (proj < closestDist) {
                    closestDist = proj;
                    closestPoint = vehiclePos;
                    hitFound = true;
                }
            }
        }
    }

    // Check intersection with pedestrians (as spheres)
    for (const auto& pedestrian : world_->getPedestrians()) {
        if (!pedestrian) continue;

        Vec3 pedPos = pedestrian->getPosition();
        Vec3 toPed = pedPos - origin;
        float proj = toPed.dot(direction);

        if (proj > 0.0f && proj < maxDist) {
            Vec3 closest = origin + direction * proj;
            float dist = (closest - pedPos).norm();

            // Assume pedestrian is ~0.5m in radius
            if (dist < 0.5f) {
                if (proj < closestDist) {
                    closestDist = proj;
                    closestPoint = pedPos;
                    hitFound = true;
                }
            }
        }
    }

    if (hitFound) {
        hitPoint = closestPoint;
        distance = closestDist;
        return true;
    }

    return false;
}

int LidarSensor::getRandomRing() const {
    return static_cast<int>(g_uniform(g_rng) * config_.numChannels);
}

} // namespace av

