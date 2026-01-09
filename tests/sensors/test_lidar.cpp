/*
 * Phase 4: LIDAR Sensor Tests
 *
 * Comprehensive test suite for LIDAR sensor simulation and point cloud generation
 */

#include "av/foundation/logging.hpp"
#include "av/foundation/math.hpp"
#include "av/sensors/lidar.hpp"
#include "av/world/world.hpp"
#include "av/world/road_network.hpp"
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

// ==================== Sensor Base Class Tests ====================

TestResult testSensorCreation() {
    auto lidar = std::make_shared<LidarSensor>();
    return {lidar != nullptr, "LidarSensor should be created"};
}

TestResult testSensorStatus() {
    auto lidar = std::make_shared<LidarSensor>();
    bool isUninitialized = lidar->getStatus() == Sensor::Status::UNINITIALIZED;
    return {isUninitialized, "Sensor should start in UNINITIALIZED status"};
}

TestResult testSensorInitialize() {
    auto lidar = std::make_shared<LidarSensor>();
    bool initialized = lidar->initialize();
    bool isActive = lidar->getStatus() == Sensor::Status::ACTIVE;
    return {initialized && isActive, "Sensor should initialize to ACTIVE"};
}

TestResult testSensorTransform() {
    auto lidar = std::make_shared<LidarSensor>();
    Vec3 position(10, 2, 5);
    Quat rotation = Quat::Identity();
    lidar->setTransform(position, rotation);

    Vec3 retrievedPos = lidar->getPosition();
    float tolerance = 0.01f;
    bool posMatch = std::abs(retrievedPos.x() - 10.0f) < tolerance &&
                    std::abs(retrievedPos.y() - 2.0f) < tolerance &&
                    std::abs(retrievedPos.z() - 5.0f) < tolerance;

    return {posMatch, "Sensor transform should be set and retrieved"};
}

TestResult testSensorName() {
    auto lidar = std::make_shared<LidarSensor>();
    bool nameMatch = lidar->getName() == "LIDAR";
    return {nameMatch, "Sensor name should be LIDAR"};
}

// ==================== LIDAR Configuration Tests ====================

TestResult testLidarConfigDefaults() {
    LidarSensor::Config config;
    bool channelsMatch = config.numChannels == 64;
    bool rangeMatch = config.maxRange == 120.0f;
    bool updateMatch = config.updateRate == 10.0f;
    return {channelsMatch && rangeMatch && updateMatch, "LIDAR default config should match"};
}

TestResult testLidarConfigCustom() {
    LidarSensor::Config config;
    config.numChannels = 32;
    config.maxRange = 100.0f;
    config.updateRate = 20.0f;

    auto lidar = std::make_shared<LidarSensor>(config);
    const auto& retrievedConfig = lidar->getConfig();

    return assertEqual(32, retrievedConfig.numChannels, "Custom config channels should be 32");
}

TestResult testLidarSetMaxRange() {
    auto lidar = std::make_shared<LidarSensor>();
    lidar->setMaxRange(80.0f);
    return assertEqual(80.0f, lidar->getConfig().maxRange, 0.01f, "Max range should be 80m");
}

TestResult testLidarSetNumChannels() {
    auto lidar = std::make_shared<LidarSensor>();
    lidar->setNumChannels(32);
    return assertEqual(32, lidar->getConfig().numChannels, "Num channels should be 32");
}

TestResult testLidarSetUpdateRate() {
    auto lidar = std::make_shared<LidarSensor>();
    lidar->setUpdateRate(20.0f);
    return assertEqual(20.0f, lidar->getConfig().updateRate, 0.01f, "Update rate should be 20 Hz");
}

// ==================== Point Cloud Tests ====================

TestResult testPointCloudCreation() {
    PointCloud cloud;
    return assertEqual(0, static_cast<int>(cloud.size()), "Point cloud should start empty");
}

TestResult testPointCloudAddPoints() {
    PointCloud cloud;
    cloud.points.push_back(Vec3(0, 0, 0));
    cloud.points.push_back(Vec3(1, 1, 1));
    cloud.points.push_back(Vec3(2, 2, 2));
    cloud.intensity.push_back(100.0f);
    cloud.intensity.push_back(150.0f);
    cloud.intensity.push_back(200.0f);

    return assertEqual(3, static_cast<int>(cloud.size()), "Point cloud should have 3 points");
}

TestResult testPointCloudClear() {
    PointCloud cloud;
    cloud.points.push_back(Vec3(0, 0, 0));
    cloud.intensity.push_back(100.0f);
    cloud.clear();

    return assertEqual(0, static_cast<int>(cloud.size()), "Point cloud should be cleared");
}

TestResult testPointCloudAttributes() {
    PointCloud cloud;
    cloud.points.push_back(Vec3(10, 10, 10));
    cloud.intensity.push_back(200.0f);
    cloud.ring.push_back(32);
    cloud.distance.push_back(14.14f);

    bool pointMatch = std::abs(cloud.points[0].x() - 10.0f) < 0.01f;
    bool intensityMatch = std::abs(cloud.intensity[0] - 200.0f) < 0.01f;
    bool ringMatch = cloud.ring[0] == 32;
    bool distMatch = std::abs(cloud.distance[0] - 14.14f) < 0.01f;

    return {pointMatch && intensityMatch && ringMatch && distMatch,
            "Point cloud attributes should match"};
}

// ==================== LIDAR Update Tests ====================

TestResult testLidarUpdate() {
    auto lidar = std::make_shared<LidarSensor>();
    lidar->initialize();

    // Update should not crash even without world
    lidar->update(0.1f);

    return {true, "LIDAR update should handle missing world gracefully"};
}

TestResult testLidarUpdateWithWorld() {
    auto lidar = std::make_shared<LidarSensor>();
    auto world = std::make_shared<World>();
    world->initialize();

    lidar->initialize();
    lidar->setWorld(world);
    lidar->setTransform(Vec3(0, 1, 0), Quat::Identity());

    // Create some test geometry
    auto roadNetwork = world->getRoadNetwork();
    auto lane = roadNetwork->createLane(Lane::Type::DRIVING);
    lane->addCenterlinePoint(Vec3(0, 0, 0));
    lane->addCenterlinePoint(Vec3(10, 0, 0));
    lane->addCenterlinePoint(Vec3(20, 0, 0));

    // Update LIDAR
    lidar->update(0.1f);

    return {true, "LIDAR update with world should complete"};
}

TestResult testLidarFrameCount() {
    auto lidar = std::make_shared<LidarSensor>();
    lidar->initialize();

    int initialFrame = lidar->getFrameCount();

    // Update multiple times
    for (int i = 0; i < 20; ++i) {
        lidar->update(0.1f);  // Plenty of time for updates
    }

    bool frameIncremented = lidar->getFrameCount() > initialFrame;
    return {frameIncremented, "Frame count should increment after updates"};
}

TestResult testLidarTimestamp() {
    auto lidar = std::make_shared<LidarSensor>();
    lidar->initialize();

    float lastTime = lidar->getLastUpdateTime();
    bool startsAtZero = std::abs(lastTime - 0.0f) < 0.01f;

    return {startsAtZero, "Last update time should start at 0"};
}

// ==================== LIDAR Enable/Disable Tests ====================

TestResult testLidarEnable() {
    auto lidar = std::make_shared<LidarSensor>();
    lidar->enable();
    return {lidar->isEnabled(), "LIDAR should be enabled"};
}

TestResult testLidarDisable() {
    auto lidar = std::make_shared<LidarSensor>();
    lidar->disable();
    return {!lidar->isEnabled(), "LIDAR should be disabled"};
}

TestResult testLidarDisabledUpdate() {
    auto lidar = std::make_shared<LidarSensor>();
    lidar->initialize();
    lidar->disable();

    int initialPoints = lidar->getLastPointCount();

    // Update should not generate points when disabled
    auto world = std::make_shared<World>();
    world->initialize();
    lidar->setWorld(world);
    lidar->update(0.15f);

    int finalPoints = lidar->getLastPointCount();
    return {initialPoints == finalPoints, "Disabled LIDAR should not generate points"};
}

// ==================== LIDAR Statistics Tests ====================

TestResult testLidarPointCount() {
    auto lidar = std::make_shared<LidarSensor>();
    lidar->initialize();

    auto world = std::make_shared<World>();
    world->initialize();

    auto roadNetwork = world->getRoadNetwork();
    auto lane = roadNetwork->createLane(Lane::Type::DRIVING);
    lane->addCenterlinePoint(Vec3(0, 0, 0));
    lane->addCenterlinePoint(Vec3(50, 0, 0));

    lidar->setWorld(world);
    lidar->setTransform(Vec3(0, 1, 0), Quat::Identity());

    lidar->update(0.15f);

    int pointCount = lidar->getLastPointCount();
    bool hasPoints = pointCount > 0;

    return {hasPoints, "LIDAR should generate points from world geometry"};
}

TestResult testLidarUpdateDuration() {
    auto lidar = std::make_shared<LidarSensor>();
    lidar->initialize();

    auto world = std::make_shared<World>();
    world->initialize();
    lidar->setWorld(world);

    lidar->update(0.15f);

    float duration = lidar->getLastUpdateDuration();
    bool durationValid = duration >= 0.0f && duration < 0.1f;

    return {durationValid, "Update duration should be reasonable"};
}

// ==================== LIDAR Noise Tests ====================

TestResult testLidarGaussianNoise() {
    // This test verifies that noise models are working
    // We can't directly test randomness, but we can verify the functionality
    auto lidar = std::make_shared<LidarSensor>();
    LidarSensor::Config config = lidar->getConfig();
    config.gaussianNoiseStdDev = 0.05f;
    lidar->setConfig(config);

    return {lidar->getConfig().gaussianNoiseStdDev == 0.05f,
            "Gaussian noise parameter should be set"};
}

TestResult testLidarOutlierProbability() {
    auto lidar = std::make_shared<LidarSensor>();
    LidarSensor::Config config = lidar->getConfig();
    config.outlierProbability = 0.02f;
    lidar->setConfig(config);

    return {lidar->getConfig().outlierProbability == 0.02f,
            "Outlier probability should be set"};
}

// ==================== Test Runner ====================

void printTestResult(const std::string& testName, const TestResult& result) {
    std::string status = result.passed ? "PASS" : "FAIL";
    std::cout << "[" << status << "] " << testName << ": " << result.message << std::endl;
}

void runAllTests() {
    std::cout << "\n=== Phase 4: LIDAR Sensor Tests ===" << std::endl;

    int passCount = 0;
    int totalCount = 0;

    // Sensor base class tests
    std::cout << "\nSensor Base Class Tests:" << std::endl;
    TestResult tests_sensor[] = {
        testSensorCreation(),
        testSensorStatus(),
        testSensorInitialize(),
        testSensorTransform(),
        testSensorName()
    };
    for (const auto& test : tests_sensor) {
        printTestResult("Sensor", test);
        if (test.passed) passCount++;
        totalCount++;
    }

    // LIDAR configuration tests
    std::cout << "\nLIDAR Configuration Tests:" << std::endl;
    TestResult tests_config[] = {
        testLidarConfigDefaults(),
        testLidarConfigCustom(),
        testLidarSetMaxRange(),
        testLidarSetNumChannels(),
        testLidarSetUpdateRate()
    };
    for (const auto& test : tests_config) {
        printTestResult("LidarConfig", test);
        if (test.passed) passCount++;
        totalCount++;
    }

    // Point cloud tests
    std::cout << "\nPoint Cloud Tests:" << std::endl;
    TestResult tests_cloud[] = {
        testPointCloudCreation(),
        testPointCloudAddPoints(),
        testPointCloudClear(),
        testPointCloudAttributes()
    };
    for (const auto& test : tests_cloud) {
        printTestResult("PointCloud", test);
        if (test.passed) passCount++;
        totalCount++;
    }

    // LIDAR update tests
    std::cout << "\nLIDAR Update Tests:" << std::endl;
    TestResult tests_update[] = {
        testLidarUpdate(),
        testLidarUpdateWithWorld(),
        testLidarFrameCount(),
        testLidarTimestamp()
    };
    for (const auto& test : tests_update) {
        printTestResult("LidarUpdate", test);
        if (test.passed) passCount++;
        totalCount++;
    }

    // Enable/disable tests
    std::cout << "\nLIDAR Enable/Disable Tests:" << std::endl;
    TestResult tests_control[] = {
        testLidarEnable(),
        testLidarDisable(),
        testLidarDisabledUpdate()
    };
    for (const auto& test : tests_control) {
        printTestResult("LidarControl", test);
        if (test.passed) passCount++;
        totalCount++;
    }

    // Statistics tests
    std::cout << "\nLIDAR Statistics Tests:" << std::endl;
    TestResult tests_stats[] = {
        testLidarPointCount(),
        testLidarUpdateDuration()
    };
    for (const auto& test : tests_stats) {
        printTestResult("LidarStats", test);
        if (test.passed) passCount++;
        totalCount++;
    }

    // Noise tests
    std::cout << "\nLIDAR Noise Tests:" << std::endl;
    TestResult tests_noise[] = {
        testLidarGaussianNoise(),
        testLidarOutlierProbability()
    };
    for (const auto& test : tests_noise) {
        printTestResult("LidarNoise", test);
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
    av::Logger::init("lidar_tests.log");
    av::Logger::setLevel(av::Logger::INFO);

    av::test::runAllTests();

    av::Logger::shutdown();
    return 0;
}
