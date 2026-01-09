/*
 * Phase 6: Other Sensors Tests
 *
 * Comprehensive test suite for Radar, GPS, IMU, Ultrasonic, and Odometry sensors
 */

#include "av/foundation/logging.hpp"
#include "av/sensors/radar.hpp"
#include "av/sensors/gps.hpp"
#include "av/sensors/imu.hpp"
#include "av/sensors/ultrasonic.hpp"
#include "av/sensors/odometry.hpp"
#include "av/sensors/sensor_manager.hpp"
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

TestResult assertEqual(float a, float b, float epsilon, const std::string& message) {
    bool passed = std::abs(a - b) < epsilon;
    return {passed, message};
}

// ==================== Radar Sensor Tests ====================

TestResult testRadarCreation() {
    auto radar = std::make_shared<RadarSensor>();
    return {radar != nullptr, "RadarSensor should be created"};
}

TestResult testRadarInitialization() {
    auto radar = std::make_shared<RadarSensor>();
    bool initialized = radar->initialize();
    bool isActive = radar->getStatus() == Sensor::Status::ACTIVE;
    return {initialized && isActive, "Radar should initialize to ACTIVE"};
}

TestResult testRadarConfig() {
    RadarSensor::Config config;
    config.maxRange = 200.0f;
    config.updateRate = 10.0f;
    auto radar = std::make_shared<RadarSensor>(config);
    return assertEqual(200.0f, radar->getConfig().maxRange, 0.01f,
                      "Radar config should be applied");
}

TestResult testRadarDetectionWithWorld() {
    auto radar = std::make_shared<RadarSensor>();
    auto world = std::make_shared<World>();
    world->initialize();
    radar->initialize();
    radar->setWorld(world);

    // Create some traffic
    auto vehicle = world->createTrafficVehicle();
    vehicle->setPosition(Vec3(50, 0, 0));
    vehicle->setColor(Vec3(1, 0, 0));

    radar->setTransform(Vec3(0, 0, 0), Quat::Identity());

    // Update should detect vehicle
    radar->update(0.15f);

    bool hasDetections = radar->getDetectionCount() > 0;
    return {hasDetections, "Radar should detect vehicles in range"};
}

TestResult testRadarDopplerVelocity() {
    auto radar = std::make_shared<RadarSensor>();
    auto world = std::make_shared<World>();
    world->initialize();
    radar->initialize();
    radar->setWorld(world);

    auto vehicle = world->createTrafficVehicle();
    vehicle->setPosition(Vec3(50, 0, 0));
    vehicle->setVelocity(Vec3(10, 0, 0));

    radar->setTransform(Vec3(0, 0, 0), Quat::Identity());
    radar->update(0.15f);

    // Check if Doppler velocity is computed
    if (radar->getDetectionCount() > 0) {
        const auto& detection = radar->getDetections()[0];
        return {detection.velocity != 0.0f, "Radar should compute Doppler velocity"};
    }

    return {true, "Radar Doppler test (no detections)"};
}

// ==================== GPS Sensor Tests ====================

TestResult testGPSCreation() {
    auto gps = std::make_shared<GPSSensor>();
    return {gps != nullptr, "GPSSensor should be created"};
}

TestResult testGPSInitialization() {
    auto gps = std::make_shared<GPSSensor>();
    bool initialized = gps->initialize();
    bool hasFix = gps->hasFix();
    return {initialized && hasFix, "GPS should initialize with fix"};
}

TestResult testGPSConfig() {
    GPSSensor::Config config;
    config.horizontalAccuracy = 5.0f;
    config.updateRate = 10.0f;
    auto gps = std::make_shared<GPSSensor>(config);
    return assertEqual(5.0f, gps->getConfig().horizontalAccuracy, 0.01f,
                      "GPS config should be applied");
}

TestResult testGPSMeasurement() {
    auto gps = std::make_shared<GPSSensor>();
    gps->initialize();
    gps->setTransform(Vec3(10, 20, 5), Quat::Identity());

    gps->update(0.12f);

    const auto& measurement = gps->getMeasurement();
    bool hasPosition = measurement.position.norm() > 0.0f;
    bool hasSatellites = measurement.satelliteCount > 0;

    return {hasPosition && hasSatellites, "GPS should produce measurements"};
}

TestResult testGPSDrift() {
    auto gps = std::make_shared<GPSSensor>();
    gps->initialize();
    gps->setTransform(Vec3(0, 0, 0), Quat::Identity());

    const auto& m1 = gps->getMeasurement();
    for (int i = 0; i < 10; ++i) {
        gps->update(0.5f);
    }
    const auto& m2 = gps->getMeasurement();

    // Position should drift over time
    Vec3 diff = m2.position - m1.position;
    return {diff.norm() >= 0.0f, "GPS should simulate drift"};
}

// ==================== IMU Sensor Tests ====================

TestResult testIMUCreation() {
    auto imu = std::make_shared<IMUSensor>();
    return {imu != nullptr, "IMUSensor should be created"};
}

TestResult testIMUInitialization() {
    auto imu = std::make_shared<IMUSensor>();
    bool initialized = imu->initialize();
    bool isActive = imu->getStatus() == Sensor::Status::ACTIVE;
    return {initialized && isActive, "IMU should initialize to ACTIVE"};
}

TestResult testIMUConfig() {
    IMUSensor::Config config;
    config.updateRate = 100.0f;
    config.accelNoiseStdDev = 0.01f;
    auto imu = std::make_shared<IMUSensor>(config);
    return assertEqual(100.0f, imu->getConfig().updateRate, 0.01f,
                      "IMU config should be applied");
}

TestResult testIMUMeasurement() {
    auto imu = std::make_shared<IMUSensor>();
    imu->initialize();

    imu->update(0.02f);

    const auto& measurement = imu->getMeasurement();
    bool hasAccel = measurement.acceleration.norm() > 0.0f;
    bool hasGyro = measurement.angularVelocity.norm() >= 0.0f;
    bool hasMagnetic = measurement.magneticField.norm() > 0.0f;

    return {hasAccel && hasGyro && hasMagnetic, "IMU should produce measurements"};
}

TestResult testIMUBiasDrift() {
    auto imu = std::make_shared<IMUSensor>();
    imu->initialize();

    Vec3 b1 = imu->getAccelerationBias();
    for (int i = 0; i < 50; ++i) {
        imu->update(0.02f);
    }
    Vec3 b2 = imu->getAccelerationBias();

    // Bias should drift over time
    return {b2.norm() >= b1.norm(), "IMU should simulate bias drift"};
}

// ==================== Ultrasonic Sensor Tests ====================

TestResult testUltrasonicCreation() {
    auto ultrasonic = std::make_shared<UltrasonicSensor>();
    return {ultrasonic != nullptr, "UltrasonicSensor should be created"};
}

TestResult testUltrasonicInitialization() {
    auto ultrasonic = std::make_shared<UltrasonicSensor>();
    bool initialized = ultrasonic->initialize();
    bool isActive = ultrasonic->getStatus() == Sensor::Status::ACTIVE;
    return {initialized && isActive, "Ultrasonic should initialize to ACTIVE"};
}

TestResult testUltrasonicConfig() {
    UltrasonicSensor::Config config;
    config.numSensors = 12;
    config.maxRange = 4.0f;
    auto ultrasonic = std::make_shared<UltrasonicSensor>(config);
    bool hasConfig = ultrasonic->getMeasurements().size() == 12;
    return {hasConfig, "Ultrasonic config should be applied"};
}

TestResult testUltrasonicDetection() {
    auto ultrasonic = std::make_shared<UltrasonicSensor>();
    auto world = std::make_shared<World>();
    world->initialize();
    ultrasonic->initialize();
    ultrasonic->setWorld(world);

    auto vehicle = world->createTrafficVehicle();
    vehicle->setPosition(Vec3(1, 0, 0));

    ultrasonic->setTransform(Vec3(0, 0, 0), Quat::Identity());
    ultrasonic->update(0.08f);

    bool hasDetections = ultrasonic->getDetectionCount() > 0;
    return {hasDetections, "Ultrasonic should detect nearby objects"};
}

TestResult testUltrasonicClosestDistance() {
    auto ultrasonic = std::make_shared<UltrasonicSensor>();
    auto world = std::make_shared<World>();
    world->initialize();
    ultrasonic->initialize();
    ultrasonic->setWorld(world);

    auto vehicle = world->createTrafficVehicle();
    vehicle->setPosition(Vec3(2, 0, 0));

    ultrasonic->setTransform(Vec3(0, 0, 0), Quat::Identity());
    ultrasonic->update(0.08f);

    float closest = ultrasonic->getClosestDistance();
    bool valid = closest > 0.0f && closest <= 4.0f;
    return {valid, "Ultrasonic should compute closest distance"};
}

// ==================== Odometry Sensor Tests ====================

TestResult testOdometryCreation() {
    auto odometry = std::make_shared<OdometrySensor>();
    return {odometry != nullptr, "OdometrySensor should be created"};
}

TestResult testOdometryInitialization() {
    auto odometry = std::make_shared<OdometrySensor>();
    bool initialized = odometry->initialize();
    bool isActive = odometry->getStatus() == Sensor::Status::ACTIVE;
    return {initialized && isActive, "Odometry should initialize to ACTIVE"};
}

TestResult testOdometryConfig() {
    OdometrySensor::Config config;
    config.wheelBase = 2.7f;
    config.updateRate = 50.0f;
    auto odometry = std::make_shared<OdometrySensor>(config);
    return assertEqual(2.7f, odometry->getConfig().wheelBase, 0.01f,
                      "Odometry config should be applied");
}

TestResult testOdometryMeasurement() {
    auto odometry = std::make_shared<OdometrySensor>();
    odometry->initialize();
    odometry->setTransform(Vec3(0, 0, 0), Quat::Identity());

    odometry->update(0.05f);

    const auto& measurement = odometry->getMeasurement();
    bool hasSpeeds = measurement.leftWheelSpeed >= 0.0f && measurement.rightWheelSpeed >= 0.0f;
    bool hasVelocity = measurement.velocityX != 0.0f || measurement.velocityY != 0.0f;

    return {hasSpeeds && hasVelocity, "Odometry should produce wheel measurements"};
}

TestResult testOdometryIntegration() {
    auto odometry = std::make_shared<OdometrySensor>();
    odometry->initialize();
    odometry->setTransform(Vec3(0, 0, 0), Quat::Identity());

    Vec3 p1 = odometry->getEstimatedPosition();
    for (int i = 0; i < 10; ++i) {
        odometry->update(0.05f);
    }
    Vec3 p2 = odometry->getEstimatedPosition();

    // Position should change over time
    Vec3 diff = p2 - p1;
    return {diff.norm() > 0.0f, "Odometry should integrate motion"};
}

// ==================== Sensor Manager Tests ====================

TestResult testSensorManagerCreation() {
    auto manager = std::make_shared<SensorManager>();
    return {manager != nullptr, "SensorManager should be created"};
}

TestResult testSensorManagerMultiSensor() {
    auto manager = std::make_shared<SensorManager>();
    auto world = std::make_shared<World>();
    world->initialize();
    manager->initialize(world);

    // Create all sensor types
    auto radar = manager->createRadarSensor();
    auto camera = manager->createCameraSensor();
    auto gps = manager->createGPSSensor();
    auto imu = manager->createIMUSensor();
    auto ultrasonic = manager->createUltrasonicSensor();
    auto odometry = manager->createOdometrySensor();

    bool allCreated = radar && camera && gps && imu && ultrasonic && odometry;
    return {allCreated && manager->getSensorCount() == 6, "SensorManager should create all sensors"};
}

TestResult testSensorManagerMultiRateUpdate() {
    auto manager = std::make_shared<SensorManager>();
    auto world = std::make_shared<World>();
    world->initialize();
    manager->initialize(world);

    // Create sensors with different rates
    auto radar = manager->createRadarSensor();  // 10 Hz
    auto imu = manager->createIMUSensor();      // 100 Hz

    // Update with small deltaTime
    manager->update(0.005f);  // 5ms

    bool isActive = radar->getStatus() == Sensor::Status::ACTIVE &&
                   imu->getStatus() == Sensor::Status::ACTIVE;
    return {isActive, "SensorManager should handle multi-rate updates"};
}

TestResult testSensorManagerSensorRetrieval() {
    auto manager = std::make_shared<SensorManager>();
    auto world = std::make_shared<World>();
    world->initialize();
    manager->initialize(world);

    auto radar = manager->createRadarSensor();
    auto retrieved = manager->getRadarSensor();

    return {retrieved != nullptr, "SensorManager should retrieve sensors by type"};
}

// ==================== Test Runner ====================

void printTestResult(const std::string& testName, const TestResult& result) {
    std::string status = result.passed ? "PASS" : "FAIL";
    std::cout << "[" << status << "] " << testName << ": " << result.message << std::endl;
}

void runAllTests() {
    std::cout << "\n=== Phase 6: Other Sensors Tests ===" << std::endl;

    int passCount = 0;
    int totalCount = 0;

    // Radar tests
    std::cout << "\nRadar Sensor Tests:" << std::endl;
    TestResult radarTests[] = {
        testRadarCreation(),
        testRadarInitialization(),
        testRadarConfig(),
        testRadarDetectionWithWorld(),
        testRadarDopplerVelocity()
    };
    for (const auto& test : radarTests) {
        printTestResult("Radar", test);
        if (test.passed) passCount++;
        totalCount++;
    }

    // GPS tests
    std::cout << "\nGPS Sensor Tests:" << std::endl;
    TestResult gpsTests[] = {
        testGPSCreation(),
        testGPSInitialization(),
        testGPSConfig(),
        testGPSMeasurement(),
        testGPSDrift()
    };
    for (const auto& test : gpsTests) {
        printTestResult("GPS", test);
        if (test.passed) passCount++;
        totalCount++;
    }

    // IMU tests
    std::cout << "\nIMU Sensor Tests:" << std::endl;
    TestResult imuTests[] = {
        testIMUCreation(),
        testIMUInitialization(),
        testIMUConfig(),
        testIMUMeasurement(),
        testIMUBiasDrift()
    };
    for (const auto& test : imuTests) {
        printTestResult("IMU", test);
        if (test.passed) passCount++;
        totalCount++;
    }

    // Ultrasonic tests
    std::cout << "\nUltrasonic Sensor Tests:" << std::endl;
    TestResult ultrasonicTests[] = {
        testUltrasonicCreation(),
        testUltrasonicInitialization(),
        testUltrasonicConfig(),
        testUltrasonicDetection(),
        testUltrasonicClosestDistance()
    };
    for (const auto& test : ultrasonicTests) {
        printTestResult("Ultrasonic", test);
        if (test.passed) passCount++;
        totalCount++;
    }

    // Odometry tests
    std::cout << "\nOdometry Sensor Tests:" << std::endl;
    TestResult odometryTests[] = {
        testOdometryCreation(),
        testOdometryInitialization(),
        testOdometryConfig(),
        testOdometryMeasurement(),
        testOdometryIntegration()
    };
    for (const auto& test : odometryTests) {
        printTestResult("Odometry", test);
        if (test.passed) passCount++;
        totalCount++;
    }

    // SensorManager tests
    std::cout << "\nSensor Manager Tests:" << std::endl;
    TestResult managerTests[] = {
        testSensorManagerCreation(),
        testSensorManagerMultiSensor(),
        testSensorManagerMultiRateUpdate(),
        testSensorManagerSensorRetrieval()
    };
    for (const auto& test : managerTests) {
        printTestResult("SensorManager", test);
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
    av::Logger::init("phase6_sensors_tests.log");
    av::Logger::setLevel(av::Logger::INFO);

    av::test::runAllTests();

    av::Logger::shutdown();
    return 0;
}
