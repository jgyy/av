#pragma once

#include "sensor.hpp"
#include "av/foundation/math.hpp"
#include <vector>
#include <memory>

namespace av {

// Forward declarations
class Scene;

// Image data structure
struct Image {
    int width = 1920;
    int height = 1080;
    std::vector<uint8_t> data;  // RGB: 3 bytes per pixel (24-bit RGB)
    float exposureTime = 1.0f;  // Exposure time in seconds

    size_t size() const { return width * height * 3; }
    void clear() { data.clear(); data.resize(size(), 0); }

    // Pixel access (RGB)
    void setPixel(int x, int y, uint8_t r, uint8_t g, uint8_t b) {
        if (x < 0 || x >= width || y < 0 || y >= height) return;
        size_t idx = (y * width + x) * 3;
        data[idx] = r;
        data[idx + 1] = g;
        data[idx + 2] = b;
    }

    void getPixel(int x, int y, uint8_t& r, uint8_t& g, uint8_t& b) const {
        if (x < 0 || x >= width || y < 0 || y >= height) return;
        size_t idx = (y * width + x) * 3;
        r = data[idx];
        g = data[idx + 1];
        b = data[idx + 2];
    }
};

// Camera sensor with realistic image generation
class CameraSensor : public Sensor {
public:
    // Camera calibration intrinsics
    struct Intrinsics {
        float fx = 800.0f;         // Focal length in pixels (x)
        float fy = 800.0f;         // Focal length in pixels (y)
        float cx = 960.0f;         // Principal point x (image center)
        float cy = 540.0f;         // Principal point y (image center)
        float k1 = -0.2f;          // Radial distortion coefficient 1
        float k2 = 0.05f;          // Radial distortion coefficient 2
        float p1 = 0.0f;           // Tangential distortion coefficient 1
        float p2 = 0.0f;           // Tangential distortion coefficient 2
        float sensorWidth = 6.4f;  // Sensor width in mm
        float sensorHeight = 3.6f; // Sensor height in mm
    };

    // Image sensor configuration
    struct Config {
        int width = 1920;
        int height = 1080;
        float horizontalFOV = 90.0f;  // Degrees
        float verticalFOV = 53.0f;    // Degrees (calculated from width/height)
        float updateRate = 30.0f;     // Hz
        bool enableDistortion = true;
        bool enableHDR = false;
        float gamma = 2.2f;           // Gamma correction
        float exposureValue = 0.0f;   // EV (stops)
        Intrinsics intrinsics;
    };

    CameraSensor(const Config& config = Config());
    ~CameraSensor() = default;

    // Sensor interface
    bool initialize() override;
    void update(float deltaTime) override;
    std::string getName() const override { return "Camera"; }

    // Image access
    const Image& getImage() const { return image_; }
    Image& getImageMutable() { return image_; }

    // Configuration
    void setConfig(const Config& config) { config_ = config; updateIntrinsics(); }
    const Config& getConfig() const { return config_; }

    // Scene rendering
    void setScene(std::shared_ptr<Scene> scene) { scene_ = scene; }
    std::shared_ptr<Scene> getScene() const { return scene_; }

    // Camera control
    void enable() { enabled_ = true; }
    void disable() { enabled_ = false; }
    bool isEnabled() const { return enabled_; }

    // Lens distortion
    Vec2 applyLensDistortion(const Vec2& normalizedPoint) const;
    Vec2 undoLensDistortion(const Vec2& distortedPoint) const;

    // HDR and tonemapping
    void applyToneMapping();
    void applyExposure(float ev);

    // Statistics
    int getLastFrameSize() const { return image_.size(); }
    float getLastUpdateDuration() const { return lastUpdateDuration_; }
    float getLastFrameMeanLuminance() const { return lastFrameMeanLuminance_; }

private:
    Config config_;
    Image image_;
    std::shared_ptr<Scene> scene_;
    bool enabled_ = true;

    float timeSinceLastUpdate_ = 0.0f;
    float lastUpdateDuration_ = 0.0f;
    float lastFrameMeanLuminance_ = 0.5f;

    // Internal helpers
    void renderToImage();
    void generateSyntheticImage();
    void computeMeanLuminance();
    void updateIntrinsics();

    // Distortion calculation
    float computeRadialDistortion(float r) const;
    Vec2 computeTangentialDistortion(const Vec2& p) const;

    // Tonemapping operators
    Vec3 reinhardToneMapping(const Vec3& color) const;
    Vec3 gammaCorrection(const Vec3& color) const;
};

} // namespace av

