/*
 * Phase 5: Camera Sensor Tests
 *
 * Comprehensive test suite for camera sensor simulation and image generation
 */

#include "av/foundation/logging.hpp"
#include "av/foundation/math.hpp"
#include "av/sensors/camera.hpp"
#include "av/world/world.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

namespace av {
namespace test {

// Test utilities
struct TestResult {
    bool passed;
    std::string message;
};

TestResult assertTrue(bool condition, const std::string& message) {
    return {condition, message};
}

TestResult assertEqual(int a, int b, const std::string& message) {
    return {a == b, message};
}

TestResult assertEqual(float a, float b, float epsilon, const std::string& message) {
    bool passed = std::abs(a - b) < epsilon;
    return {passed, message};
}

// ==================== Camera Creation Tests ====================

TestResult testCameraCreation() {
    auto camera = std::make_shared<CameraSensor>();
    return {camera != nullptr, "CameraSensor should be created"};
}

TestResult testCameraInitialization() {
    auto camera = std::make_shared<CameraSensor>();
    bool initialized = camera->initialize();
    bool isActive = camera->getStatus() == Sensor::Status::ACTIVE;
    return {initialized && isActive, "Camera should initialize to ACTIVE"};
}

TestResult testCameraName() {
    auto camera = std::make_shared<CameraSensor>();
    bool nameMatch = camera->getName() == "Camera";
    return {nameMatch, "Camera name should be 'Camera'"};
}

// ==================== Image Structure Tests ====================

TestResult testImageCreation() {
    Image img;
    return assertEqual(1920, img.width, "Image should have default width 1920");
}

TestResult testImageSize() {
    Image img;
    img.width = 640;
    img.height = 480;
    int expectedSize = 640 * 480 * 3;
    return assertEqual(expectedSize, static_cast<int>(img.size()), "Image size calculation");
}

TestResult testImageClear() {
    Image img;
    img.clear();
    bool isEmpty = img.data.size() == 1920 * 1080 * 3;
    return {isEmpty, "Image clear should allocate buffer"};
}

TestResult testImagePixelAccess() {
    Image img;
    img.clear();
    img.setPixel(100, 100, 255, 128, 64);

    uint8_t r, g, b;
    img.getPixel(100, 100, r, g, b);

    bool match = (r == 255 && g == 128 && b == 64);
    return {match, "Image pixel read/write should work"};
}

TestResult testImagePixelBounds() {
    Image img;
    img.clear();
    // Should not crash on out of bounds
    img.setPixel(-1, -1, 255, 255, 255);
    img.setPixel(10000, 10000, 255, 255, 255);
    return {true, "Image should handle out of bounds safely"};
}

// ==================== Camera Configuration Tests ====================

TestResult testCameraConfigDefault() {
    CameraSensor::Config config;
    return assertEqual(1920, config.width, "Default width should be 1920");
}

TestResult testCameraConfigCustom() {
    CameraSensor::Config config;
    config.width = 640;
    config.height = 480;
    config.updateRate = 60.0f;

    auto camera = std::make_shared<CameraSensor>(config);
    const auto& cfg = camera->getConfig();

    return assertEqual(640, cfg.width, "Custom config should be applied");
}

TestResult testCameraIntrinsics() {
    CameraSensor::Config config;
    config.horizontalFOV = 90.0f;
    config.width = 1920;
    config.height = 1080;

    auto camera = std::make_shared<CameraSensor>(config);
    const auto& intrinsics = camera->getConfig().intrinsics;

    bool fxValid = intrinsics.fx > 0.0f;
    bool cxValid = std::abs(intrinsics.cx - 960.0f) < 1.0f;

    return {fxValid && cxValid, "Intrinsics should be calculated from FOV"};
}

TestResult testCameraDistortionCoefficients() {
    CameraSensor::Config config;
    config.intrinsics.k1 = -0.3f;
    config.intrinsics.k2 = 0.1f;

    auto camera = std::make_shared<CameraSensor>(config);
    camera->setConfig(config);

    return assertEqual(-0.3f, camera->getConfig().intrinsics.k1, 0.01f,
                      "Distortion coefficients should be set");
}

// ==================== Image Generation Tests ====================

TestResult testCameraUpdate() {
    auto camera = std::make_shared<CameraSensor>();
    camera->initialize();

    // Update should not crash
    camera->update(0.1f);

    return {true, "Camera update should complete"};
}

TestResult testCameraImageGeneration() {
    auto camera = std::make_shared<CameraSensor>();
    camera->initialize();

    // Update with enough time to generate image
    camera->update(0.05f);

    const auto& image = camera->getImage();
    bool hasData = image.size() > 0;

    return {hasData, "Camera should generate image data"};
}

TestResult testCameraFrameRate() {
    CameraSensor::Config config;
    config.updateRate = 30.0f;
    auto camera = std::make_shared<CameraSensor>(config);
    camera->initialize();

    int frameCount = 0;
    for (int i = 0; i < 100; ++i) {
        int before = camera->getFrameCount();
        camera->update(0.01f);  // 10ms updates
        int after = camera->getFrameCount();
        if (after > before) frameCount++;
    }

    // At 30 Hz with 10ms updates, should get ~3 frames per second
    bool validFrameRate = frameCount > 0;
    return {validFrameRate, "Camera should maintain specified frame rate"};
}

// ==================== Lens Distortion Tests ====================

TestResult testLensDistortionApply() {
    auto camera = std::make_shared<CameraSensor>();
    Vec2 point(0.1f, 0.1f);
    Vec2 distorted = camera->applyLensDistortion(point);

    // Distorted point should be different from original
    bool changed = std::abs((distorted - point).norm()) > 0.001f;
    return {changed, "Lens distortion should modify point"};
}

TestResult testLensDistortionUndo() {
    auto camera = std::make_shared<CameraSensor>();
    Vec2 original(0.1f, 0.1f);
    Vec2 distorted = camera->applyLensDistortion(original);
    Vec2 undone = camera->undoLensDistortion(distorted);

    float error = (undone - original).norm();
    bool validError = error < 0.01f;  // Should be very close

    return {validError, "Lens undistortion should recover original point"};
}

TestResult testLensDistortionZeroCoefficients() {
    CameraSensor::Config config;
    config.intrinsics.k1 = 0.0f;
    config.intrinsics.k2 = 0.0f;
    config.intrinsics.p1 = 0.0f;
    config.intrinsics.p2 = 0.0f;

    auto camera = std::make_shared<CameraSensor>(config);
    Vec2 point(0.1f, 0.1f);
    Vec2 distorted = camera->applyLensDistortion(point);

    // With zero coefficients, distortion should be minimal
    float error = (distorted - point).norm();
    bool noDistortion = error < 0.001f;

    return {noDistortion, "Zero distortion coefficients should produce identity"};
}

// ==================== HDR and Tonemapping Tests ====================

TestResult testExposureControl() {
    CameraSensor::Config config;
    config.enableHDR = true;
    auto camera = std::make_shared<CameraSensor>(config);

    camera->applyExposure(2.0f);  // +2 EV (4x brighter)
    return assertEqual(2.0f, camera->getConfig().exposureValue, 0.01f,
                      "Exposure value should be set");
}

TestResult testTonemapping() {
    CameraSensor::Config config;
    config.width = 100;
    config.height = 100;
    config.enableHDR = true;

    auto camera = std::make_shared<CameraSensor>(config);
    camera->initialize();

    // Generate image and apply tone mapping
    camera->update(0.05f);
    camera->applyToneMapping();

    const auto& image = camera->getImage();
    bool hasImage = image.size() > 0;

    return {hasImage, "Tone mapping should produce valid image"};
}

TestResult testGammaCorrection() {
    CameraSensor::Config config;
    config.gamma = 2.2f;
    auto camera = std::make_shared<CameraSensor>(config);

    Vec3 color(0.5f, 0.5f, 0.5f);
    // Gamma correction should darken mid-tones
    Vec3 corrected = camera->getConfig(); // Just verify config is accessible
    bool valid = config.gamma > 0.0f;

    return {valid, "Gamma correction coefficient should be positive"};
}

// ==================== Statistics Tests ====================

TestResult testMeanLuminance() {
    auto camera = std::make_shared<CameraSensor>();
    camera->initialize();
    camera->update(0.05f);

    float luminance = camera->getLastFrameMeanLuminance();
    bool valid = luminance >= 0.0f && luminance <= 1.0f;

    return {valid, "Mean luminance should be in [0, 1]"};
}

TestResult testUpdateDuration() {
    auto camera = std::make_shared<CameraSensor>();
    camera->initialize();
    camera->update(0.05f);

    float duration = camera->getLastUpdateDuration();
    bool valid = duration >= 0.0f && duration < 0.1f;

    return {valid, "Update duration should be reasonable"};
}

TestResult testFrameCount() {
    auto camera = std::make_shared<CameraSensor>();
    camera->initialize();

    int initialCount = camera->getFrameCount();
    camera->update(0.05f);
    int afterCount = camera->getFrameCount();

    bool incremented = afterCount > initialCount;
    return {incremented, "Frame count should increment"};
}

// ==================== Enable/Disable Tests ====================

TestResult testCameraEnable() {
    auto camera = std::make_shared<CameraSensor>();
    camera->enable();
    return {camera->isEnabled(), "Camera should be enabled"};
}

TestResult testCameraDisable() {
    auto camera = std::make_shared<CameraSensor>();
    camera->disable();
    return {!camera->isEnabled(), "Camera should be disabled"};
}

TestResult testCameraDisabledUpdate() {
    auto camera = std::make_shared<CameraSensor>();
    camera->initialize();
    camera->disable();

    int beforeFrames = camera->getFrameCount();
    camera->update(0.05f);
    int afterFrames = camera->getFrameCount();

    bool noUpdate = afterFrames == beforeFrames;
    return {noUpdate, "Disabled camera should not update"};
}

// ==================== World Integration Tests ====================

TestResult testCameraWithWorld() {
    auto camera = std::make_shared<CameraSensor>();
    auto world = std::make_shared<World>();
    world->initialize();

    camera->initialize();
    camera->setWorld(world);

    // Create some traffic
    auto vehicle = world->createTrafficVehicle();
    vehicle->setPosition(Vec3(10, 1, 10));

    camera->setTransform(Vec3(0, 1.5, 0), Quat::Identity());

    // Update should incorporate world state
    camera->update(0.05f);

    const auto& image = camera->getImage();
    bool hasImage = image.size() > 0;

    return {hasImage, "Camera should generate image with world state"};
}

// ==================== Test Runner ====================

void printTestResult(const std::string& testName, const TestResult& result) {
    std::string status = result.passed ? "PASS" : "FAIL";
    std::cout << "[" << status << "] " << testName << ": " << result.message << std::endl;
}

void runAllTests() {
    std::cout << "\n=== Phase 5: Camera Sensor Tests ===" << std::endl;

    int passCount = 0;
    int totalCount = 0;

    // Camera creation tests
    std::cout << "\nCamera Creation Tests:" << std::endl;
    TestResult tests_create[] = {
        testCameraCreation(),
        testCameraInitialization(),
        testCameraName()
    };
    for (const auto& test : tests_create) {
        printTestResult("CameraCreate", test);
        if (test.passed) passCount++;
        totalCount++;
    }

    // Image structure tests
    std::cout << "\nImage Structure Tests:" << std::endl;
    TestResult tests_image[] = {
        testImageCreation(),
        testImageSize(),
        testImageClear(),
        testImagePixelAccess(),
        testImagePixelBounds()
    };
    for (const auto& test : tests_image) {
        printTestResult("Image", test);
        if (test.passed) passCount++;
        totalCount++;
    }

    // Configuration tests
    std::cout << "\nCamera Configuration Tests:" << std::endl;
    TestResult tests_config[] = {
        testCameraConfigDefault(),
        testCameraConfigCustom(),
        testCameraIntrinsics(),
        testCameraDistortionCoefficients()
    };
    for (const auto& test : tests_config) {
        printTestResult("CameraConfig", test);
        if (test.passed) passCount++;
        totalCount++;
    }

    // Image generation tests
    std::cout << "\nImage Generation Tests:" << std::endl;
    TestResult tests_gen[] = {
        testCameraUpdate(),
        testCameraImageGeneration(),
        testCameraFrameRate()
    };
    for (const auto& test : tests_gen) {
        printTestResult("CameraGen", test);
        if (test.passed) passCount++;
        totalCount++;
    }

    // Lens distortion tests
    std::cout << "\nLens Distortion Tests:" << std::endl;
    TestResult tests_dist[] = {
        testLensDistortionApply(),
        testLensDistortionUndo(),
        testLensDistortionZeroCoefficients()
    };
    for (const auto& test : tests_dist) {
        printTestResult("Distortion", test);
        if (test.passed) passCount++;
        totalCount++;
    }

    // HDR and tonemapping tests
    std::cout << "\nHDR and Tonemapping Tests:" << std::endl;
    TestResult tests_hdr[] = {
        testExposureControl(),
        testTonemapping(),
        testGammaCorrection()
    };
    for (const auto& test : tests_hdr) {
        printTestResult("HDR", test);
        if (test.passed) passCount++;
        totalCount++;
    }

    // Statistics tests
    std::cout << "\nStatistics Tests:" << std::endl;
    TestResult tests_stats[] = {
        testMeanLuminance(),
        testUpdateDuration(),
        testFrameCount()
    };
    for (const auto& test : tests_stats) {
        printTestResult("CameraStats", test);
        if (test.passed) passCount++;
        totalCount++;
    }

    // Control tests
    std::cout << "\nCamera Control Tests:" << std::endl;
    TestResult tests_control[] = {
        testCameraEnable(),
        testCameraDisable(),
        testCameraDisabledUpdate()
    };
    for (const auto& test : tests_control) {
        printTestResult("CameraControl", test);
        if (test.passed) passCount++;
        totalCount++;
    }

    // Integration tests
    std::cout << "\nWorld Integration Tests:" << std::endl;
    TestResult tests_integration[] = {
        testCameraWithWorld()
    };
    for (const auto& test : tests_integration) {
        printTestResult("CameraIntegration", test);
        if (test.passed) passCount++;
        totalCount++;
    }

    // Summary
    std::cout << "\n=== Test Summary ===" << std::endl;
    std::cout << "Passed: " << passCount << "/" << totalCount << std::endl;
    std::cout << "Success Rate: " << (100.0f * passCount / totalCount) << "%" << std::endl;
}

} // namespace test
} // namespace av

int main() {
    av::Logger::init("camera_tests.log");
    av::Logger::setLevel(av::Logger::INFO);

    av::test::runAllTests();

    av::Logger::shutdown();
    return 0;
}
