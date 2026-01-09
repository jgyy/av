#include "av/sensors/camera.hpp"
#include "av/world/world.hpp"
#include "av/rendering/scene.hpp"
#include "av/foundation/logging.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>

namespace av {

CameraSensor::CameraSensor() : CameraSensor(Config()) {
}

CameraSensor::CameraSensor(const Config& config)
    : config_(config) {
    setStatus(Status::UNINITIALIZED);
    updateIntrinsics();
    image_.width = config.width;
    image_.height = config.height;
    image_.clear();
    AV_DEBUG("CameraSensor created: {}x{} at {} Hz",
             config.width, config.height, config.updateRate);
}

bool CameraSensor::initialize() {
    setStatus(Status::ACTIVE);
    image_.clear();
    timeSinceLastUpdate_ = 0.0f;
    AV_INFO("CameraSensor initialized successfully");
    return true;
}

void CameraSensor::update(float deltaTime) {
    if (!enabled_ || getStatus() != Status::ACTIVE) {
        return;
    }

    timeSinceLastUpdate_ += deltaTime;
    float updateInterval = 1.0f / config_.updateRate;

    if (timeSinceLastUpdate_ >= updateInterval) {
        auto startTime = std::chrono::high_resolution_clock::now();

        // Generate image (render or synthetic)
        if (scene_) {
            renderToImage();
        } else {
            generateSyntheticImage();
        }

        // Compute statistics
        computeMeanLuminance();

        // Apply postprocessing
        if (config_.enableHDR) {
            applyToneMapping();
        }

        frameCount_++;
        timeSinceLastUpdate_ = 0.0f;

        auto endTime = std::chrono::high_resolution_clock::now();
        lastUpdateDuration_ = std::chrono::duration<float>(endTime - startTime).count();

        if (frameCount_ % 30 == 0) {
            AV_DEBUG("Camera update: {:.3f}ms, luminance: {:.3f}",
                     lastUpdateDuration_ * 1000.0f, lastFrameMeanLuminance_);
        }
    }
}

void CameraSensor::renderToImage() {
    // In a full implementation, this would use OpenGL FBO to render the scene
    // For now, we generate a synthetic image based on world state
    generateSyntheticImage();
}

void CameraSensor::generateSyntheticImage() {
    // Generate realistic synthetic image based on world state
    image_.clear();

    // Create gradient background (simulating sky)
    for (int y = 0; y < config_.height; ++y) {
        float skyGradient = y / (float)config_.height;
        uint8_t r = static_cast<uint8_t>(135 * (1.0f - skyGradient * 0.3f));
        uint8_t g = static_cast<uint8_t>(206 * (1.0f - skyGradient * 0.2f));
        uint8_t b = static_cast<uint8_t>(235 * (1.0f - skyGradient * 0.1f));

        for (int x = 0; x < config_.width; ++x) {
            image_.setPixel(x, y, r, g, b);
        }
    }

    // Add synthetic road (middle third of image)
    int roadStartY = config_.height / 2;
    int roadEndY = config_.height;

    for (int y = roadStartY; y < roadEndY; ++y) {
        float roadProgress = (y - roadStartY) / (float)(roadEndY - roadStartY);
        uint8_t roadColor = static_cast<uint8_t>(100 + roadProgress * 30);

        for (int x = 0; x < config_.width; ++x) {
            image_.setPixel(x, y, roadColor, roadColor, roadColor);
        }
    }

    // Add lane markings (yellow/white lines)
    int laneMarking1 = config_.width / 3;
    int laneMarking2 = 2 * config_.width / 3;

    for (int y = roadStartY; y < roadEndY; ++y) {
        if ((y / 20) % 2 == 0) {  // Dashed pattern
            image_.setPixel(laneMarking1, y, 255, 255, 100);
            image_.setPixel(laneMarking2, y, 255, 255, 100);
        }
    }

    // Simulate vehicles as colored rectangles based on world state
    if (world_) {
        const auto& vehicles = world_->getTrafficVehicles();
        int vehicleCount = 0;
        for (const auto& vehicle : vehicles) {
            if (!vehicle || vehicleCount >= 5) break;  // Max 5 vehicles visible

            // Simple projection: vehicles higher up in image = closer
            float vehicleDepth = 0.2f + (vehicleCount * 0.15f);
            int vehicleY = static_cast<int>(roadStartY + (1.0f - vehicleDepth) * (roadEndY - roadStartY));
            int vehicleHeight = static_cast<int>(50 * vehicleDepth);
            int vehicleWidth = static_cast<int>(30 * vehicleDepth);
            int vehicleX = config_.width / 2 - vehicleWidth / 2 + (vehicleCount - 2) * 100;

            // Draw vehicle as rectangle
            Vec3 color = vehicle->getColor();
            uint8_t r = static_cast<uint8_t>(color.x() * 255);
            uint8_t g = static_cast<uint8_t>(color.y() * 255);
            uint8_t b = static_cast<uint8_t>(color.z() * 255);

            for (int vy = vehicleY; vy < vehicleY + vehicleHeight; ++vy) {
                for (int vx = vehicleX; vx < vehicleX + vehicleWidth; ++vx) {
                    if (vx >= 0 && vx < config_.width && vy >= 0 && vy < config_.height) {
                        image_.setPixel(vx, vy, r, g, b);
                    }
                }
            }

            vehicleCount++;
        }
    }

    // Apply lens distortion if enabled
    if (config_.enableDistortion) {
        Image distortedImage = image_;

        for (int y = 0; y < config_.height; ++y) {
            for (int x = 0; x < config_.width; ++x) {
                // Normalize coordinates to [-1, 1]
                Vec2 normalizedPoint(
                    (x - config_.intrinsics.cx) / config_.intrinsics.fx,
                    (y - config_.intrinsics.cy) / config_.intrinsics.fy
                );

                // Apply inverse distortion to find source pixel
                Vec2 distortedPoint = undoLensDistortion(normalizedPoint);

                // Convert back to pixel coordinates
                int srcX = static_cast<int>(distortedPoint.x() * config_.intrinsics.fx + config_.intrinsics.cx);
                int srcY = static_cast<int>(distortedPoint.y() * config_.intrinsics.fy + config_.intrinsics.cy);

                // Sample with bounds check
                if (srcX >= 0 && srcX < config_.width && srcY >= 0 && srcY < config_.height) {
                    uint8_t r, g, b;
                    image_.getPixel(srcX, srcY, r, g, b);
                    distortedImage.setPixel(x, y, r, g, b);
                }
            }
        }

        image_ = distortedImage;
    }
}

void CameraSensor::computeMeanLuminance() {
    if (image_.size() == 0) {
        lastFrameMeanLuminance_ = 0.5f;
        return;
    }

    double sumLuminance = 0.0;
    size_t pixelCount = 0;

    // Sample every Nth pixel for performance
    int sampleStep = std::max(1, static_cast<int>(std::sqrt(image_.width * image_.height / 1000)));

    for (int y = 0; y < config_.height; y += sampleStep) {
        for (int x = 0; x < config_.width; x += sampleStep) {
            uint8_t r, g, b;
            image_.getPixel(x, y, r, g, b);

            // Luminance: 0.299*R + 0.587*G + 0.114*B
            float luminance = (0.299f * r + 0.587f * g + 0.114f * b) / 255.0f;
            sumLuminance += luminance;
            pixelCount++;
        }
    }

    lastFrameMeanLuminance_ = pixelCount > 0 ? sumLuminance / pixelCount : 0.5f;
}

void CameraSensor::updateIntrinsics() {
    // Calculate intrinsics from FOV if needed
    if (config_.horizontalFOV > 0.0f) {
        float fovRad = config_.horizontalFOV * 3.14159f / 180.0f;
        config_.intrinsics.fx = (config_.width / 2.0f) / std::tan(fovRad / 2.0f);
        config_.intrinsics.fy = config_.intrinsics.fx;
        config_.intrinsics.cx = config_.width / 2.0f;
        config_.intrinsics.cy = config_.height / 2.0f;
    }

    AV_DEBUG("Camera intrinsics: fx={:.1f}, fy={:.1f}, cx={:.1f}, cy={:.1f}",
             config_.intrinsics.fx, config_.intrinsics.fy,
             config_.intrinsics.cx, config_.intrinsics.cy);
}

Vec2 CameraSensor::applyLensDistortion(const Vec2& normalizedPoint) const {
    // Brown-Conrady distortion model
    float r2 = normalizedPoint.x() * normalizedPoint.x() + normalizedPoint.y() * normalizedPoint.y();
    float radialFactor = 1.0f + config_.intrinsics.k1 * r2 + config_.intrinsics.k2 * r2 * r2;

    Vec2 tangential = computeTangentialDistortion(normalizedPoint);

    return Vec2(
        normalizedPoint.x() * radialFactor + tangential.x(),
        normalizedPoint.y() * radialFactor + tangential.y()
    );
}

Vec2 CameraSensor::undoLensDistortion(const Vec2& distortedPoint) const {
    // Iterative undistortion using Newton's method
    Vec2 point = distortedPoint;

    for (int iter = 0; iter < 5; ++iter) {
        Vec2 distorted = applyLensDistortion(point);
        Vec2 error = distorted - distortedPoint;

        if (error.norm() < 1e-6f) break;

        // Jacobian approximation
        const float delta = 1e-4f;
        Vec2 deltaX(delta, 0);
        Vec2 distorted_dx = applyLensDistortion(point + deltaX);
        Vec2 jacobian_x = (distorted_dx - distorted) / delta;

        Vec2 deltaY(0, delta);
        Vec2 distorted_dy = applyLensDistortion(point + deltaY);
        Vec2 jacobian_y = (distorted_dy - distorted) / delta;

        float det = jacobian_x.x() * jacobian_y.y() - jacobian_x.y() * jacobian_y.x();
        if (std::abs(det) > 1e-6f) {
            point = point - Vec2(
                (jacobian_y.y() * error.x() - jacobian_x.y() * error.y()) / det,
                (-jacobian_y.x() * error.x() + jacobian_x.x() * error.y()) / det
            );
        }
    }

    return point;
}

float CameraSensor::computeRadialDistortion(float r) const {
    float r2 = r * r;
    return 1.0f + config_.intrinsics.k1 * r2 + config_.intrinsics.k2 * r2 * r2;
}

Vec2 CameraSensor::computeTangentialDistortion(const Vec2& p) const {
    float r2 = p.x() * p.x() + p.y() * p.y();
    return Vec2(
        2.0f * config_.intrinsics.p1 * p.x() * p.y() + config_.intrinsics.p2 * (r2 + 2.0f * p.x() * p.x()),
        config_.intrinsics.p1 * (r2 + 2.0f * p.y() * p.y()) + 2.0f * config_.intrinsics.p2 * p.x() * p.y()
    );
}

void CameraSensor::applyToneMapping() {
    // Convert image to floating point, apply tone mapping, convert back
    for (size_t i = 0; i < image_.data.size(); i += 3) {
        Vec3 color(
            image_.data[i] / 255.0f,
            image_.data[i + 1] / 255.0f,
            image_.data[i + 2] / 255.0f
        );

        // Apply exposure compensation
        color = color * std::pow(2.0f, config_.exposureValue);

        // Apply Reinhard tone mapping
        color = reinhardToneMapping(color);

        // Apply gamma correction
        color = gammaCorrection(color);

        // Convert back to 8-bit
        image_.data[i] = static_cast<uint8_t>(std::clamp(color.x() * 255.0f, 0.0f, 255.0f));
        image_.data[i + 1] = static_cast<uint8_t>(std::clamp(color.y() * 255.0f, 0.0f, 255.0f));
        image_.data[i + 2] = static_cast<uint8_t>(std::clamp(color.z() * 255.0f, 0.0f, 255.0f));
    }
}

void CameraSensor::applyExposure(float ev) {
    config_.exposureValue = ev;
    if (config_.enableHDR) {
        applyToneMapping();
    }
}

Vec3 CameraSensor::reinhardToneMapping(const Vec3& color) const {
    // Reinhard tone mapping: color / (1 + color)
    float lum = 0.299f * color.x() + 0.587f * color.y() + 0.114f * color.z();
    float mappedLum = lum / (1.0f + lum);
    float scale = mappedLum / (lum + 1e-6f);

    return color * scale;
}

Vec3 CameraSensor::gammaCorrection(const Vec3& color) const {
    float invGamma = 1.0f / config_.gamma;
    return Vec3(
        std::pow(color.x(), invGamma),
        std::pow(color.y(), invGamma),
        std::pow(color.z(), invGamma)
    );
}

} // namespace av

