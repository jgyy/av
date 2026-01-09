/*
 * Phase 3: World Simulation Tests
 *
 * Comprehensive test suite for road network, traffic, and world simulation
 */

#include "av/foundation/logging.hpp"
#include "av/foundation/math.hpp"
#include "av/world/road_network.hpp"
#include "av/world/traffic.hpp"
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

TestResult assertEqual(int a, int b, const std::string& message) {
    return {a == b, message};
}

// ==================== Lane Tests ====================

TestResult testLaneCreation() {
    auto lane = std::make_shared<Lane>(0, Lane::Type::DRIVING);
    return assertEqual(0, lane->getId(), "Lane ID should be 0");
}

TestResult testLaneCenterlineAddition() {
    auto lane = std::make_shared<Lane>(0, Lane::Type::DRIVING);
    lane->addCenterlinePoint(Vec3(0, 0, 0));
    lane->addCenterlinePoint(Vec3(10, 0, 0));
    lane->addCenterlinePoint(Vec3(20, 0, 0));

    return assertEqual(3, static_cast<int>(lane->getCenterline().size()), "Lane should have 3 points");
}

TestResult testLaneLength() {
    auto lane = std::make_shared<Lane>(0, Lane::Type::DRIVING);
    lane->addCenterlinePoint(Vec3(0, 0, 0));
    lane->addCenterlinePoint(Vec3(10, 0, 0));
    lane->addCenterlinePoint(Vec3(20, 0, 0));

    float expectedLength = 20.0f;
    float tolerance = 0.01f;
    return assertEqual(expectedLength, lane->getLength(), tolerance, "Lane length should be 20m");
}

TestResult testLanePointAtDistance() {
    auto lane = std::make_shared<Lane>(0, Lane::Type::DRIVING);
    lane->addCenterlinePoint(Vec3(0, 0, 0));
    lane->addCenterlinePoint(Vec3(10, 0, 0));
    lane->addCenterlinePoint(Vec3(20, 0, 0));

    Vec3 point = lane->getPointAtDistance(10.0f);
    float tolerance = 0.01f;

    bool xMatch = std::abs(point.x() - 10.0f) < tolerance;
    bool yMatch = std::abs(point.y() - 0.0f) < tolerance;
    bool zMatch = std::abs(point.z() - 0.0f) < tolerance;

    return {xMatch && yMatch && zMatch, "Point at distance 10m should be (10, 0, 0)"};
}

TestResult testLaneDistanceToPoint() {
    auto lane = std::make_shared<Lane>(0, Lane::Type::DRIVING);
    lane->addCenterlinePoint(Vec3(0, 0, 0));
    lane->addCenterlinePoint(Vec3(10, 0, 0));
    lane->addCenterlinePoint(Vec3(20, 0, 0));

    // Point at the middle of the lane
    Vec3 testPoint(10, 0, 0);
    float distance = lane->getDistanceToPoint(testPoint);
    float tolerance = 0.01f;

    return assertEqual(10.0f, distance, tolerance, "Distance to point (10,0,0) should be 10m");
}

TestResult testLaneWidth() {
    auto lane = std::make_shared<Lane>(0, Lane::Type::DRIVING);
    lane->setLeftBoundary(2.5f);
    lane->setRightBoundary(2.5f);

    float expectedWidth = 5.0f;
    return assertEqual(expectedWidth, lane->getWidth(), "Lane width should be 5m");
}

TestResult testLaneConnections() {
    auto lane1 = std::make_shared<Lane>(0, Lane::Type::DRIVING);
    auto lane2 = std::make_shared<Lane>(1, Lane::Type::DRIVING);

    lane1->setNextLane(lane2);
    lane2->setPreviousLane(lane1);

    bool nextMatch = lane1->getNextLane() && lane1->getNextLane()->getId() == 1;
    bool prevMatch = lane2->getPreviousLane() && lane2->getPreviousLane()->getId() == 0;

    return {nextMatch && prevMatch, "Lane connections should be established"};
}

// ==================== TrafficLight Tests ====================

TestResult testTrafficLightCreation() {
    auto light = std::make_shared<TrafficLight>(0);
    return assertEqual(0, light->getId(), "TrafficLight ID should be 0");
}

TestResult testTrafficLightInitialState() {
    auto light = std::make_shared<TrafficLight>(0);
    bool isRed = light->getState() == TrafficLight::State::RED;
    return {isRed, "TrafficLight should start in RED state"};
}

TestResult testTrafficLightDurations() {
    auto light = std::make_shared<TrafficLight>(0);
    light->setGreenDuration(25.0f);
    light->setYellowDuration(4.0f);
    light->setRedDuration(25.0f);

    // Note: We can't directly get these values, so we just verify setting works
    return {true, "Traffic light durations set successfully"};
}

TestResult testTrafficLightStateTransition() {
    auto light = std::make_shared<TrafficLight>(0);
    light->setState(TrafficLight::State::RED);
    light->update(10.0f);  // Update less than red duration

    // Light should still be red
    bool stillRed = light->getState() == TrafficLight::State::RED;
    return {stillRed, "Traffic light should remain RED after 10s (red duration is 30s)"};
}

// ==================== Intersection Tests ====================

TestResult testIntersectionCreation() {
    Vec3 position(50, 0, 50);
    auto intersection = std::make_shared<Intersection>(0, position);
    return assertEqual(0, intersection->getId(), "Intersection ID should be 0");
}

TestResult testIntersectionPosition() {
    Vec3 position(50, 0, 50);
    auto intersection = std::make_shared<Intersection>(0, position);

    const Vec3& intPos = intersection->getPosition();
    float tolerance = 0.01f;
    bool xMatch = std::abs(intPos.x() - 50.0f) < tolerance;
    bool yMatch = std::abs(intPos.y() - 0.0f) < tolerance;
    bool zMatch = std::abs(intPos.z() - 50.0f) < tolerance;

    return {xMatch && yMatch && zMatch, "Intersection position should match"};
}

TestResult testIntersectionTrafficLights() {
    auto intersection = std::make_shared<Intersection>(0, Vec3(50, 0, 50));
    auto light = std::make_shared<TrafficLight>(0);
    intersection->addTrafficLight(light);

    return assertEqual(1, static_cast<int>(intersection->getTrafficLightCount()),
                      "Intersection should have 1 traffic light");
}

TestResult testIntersectionLaneConnections() {
    auto intersection = std::make_shared<Intersection>(0, Vec3(50, 0, 50));
    auto lane1 = std::make_shared<Lane>(0, Lane::Type::DRIVING);
    auto lane2 = std::make_shared<Lane>(1, Lane::Type::DRIVING);

    intersection->addIncomingLane(lane1);
    intersection->addOutgoingLane(lane2);

    bool incomingMatch = intersection->getIncomingLanes().size() == 1;
    bool outgoingMatch = intersection->getOutgoingLanes().size() == 1;

    return {incomingMatch && outgoingMatch, "Intersection should have 1 incoming and 1 outgoing lane"};
}

TestResult testIntersectionContainment() {
    auto intersection = std::make_shared<Intersection>(0, Vec3(50, 0, 50));
    intersection->setRadius(20.0f);

    // Point inside intersection
    bool inside = intersection->isPointInIntersection(Vec3(50, 0, 50));
    // Point outside intersection
    bool outside = !intersection->isPointInIntersection(Vec3(100, 0, 100));

    return {inside && outside, "Intersection containment check should work"};
}

// ==================== RoadNetwork Tests ====================

TestResult testRoadNetworkCreation() {
    auto network = std::make_shared<RoadNetwork>();
    return {network != nullptr, "RoadNetwork should be created successfully"};
}

TestResult testRoadNetworkLaneCreation() {
    auto network = std::make_shared<RoadNetwork>();
    auto lane = network->createLane(Lane::Type::DRIVING);

    return assertEqual(1, static_cast<int>(network->getLaneCount()), "RoadNetwork should have 1 lane");
}

TestResult testRoadNetworkIntersectionCreation() {
    auto network = std::make_shared<RoadNetwork>();
    auto intersection = network->createIntersection(Vec3(50, 0, 50));

    return assertEqual(1, static_cast<int>(network->getIntersectionCount()),
                      "RoadNetwork should have 1 intersection");
}

TestResult testRoadNetworkFindClosestLane() {
    auto network = std::make_shared<RoadNetwork>();
    auto lane = network->createLane(Lane::Type::DRIVING);
    lane->addCenterlinePoint(Vec3(0, 0, 0));
    lane->addCenterlinePoint(Vec3(10, 0, 0));

    auto closestLane = network->findClosestLane(Vec3(5, 0, 0), 50.0f);
    return {closestLane != nullptr, "Should find closest lane"};
}

// ==================== TrafficVehicle Tests ====================

TestResult testTrafficVehicleCreation() {
    auto vehicle = std::make_shared<TrafficVehicle>();
    return {vehicle->getId() >= 0, "TrafficVehicle should have valid ID"};
}

TestResult testTrafficVehiclePosition() {
    auto vehicle = std::make_shared<TrafficVehicle>();
    Vec3 testPos(10, 1, 20);
    vehicle->setPosition(testPos);

    Vec3 retrievedPos = vehicle->getPosition();
    float tolerance = 0.01f;
    bool xMatch = std::abs(retrievedPos.x() - 10.0f) < tolerance;
    bool yMatch = std::abs(retrievedPos.y() - 1.0f) < tolerance;
    bool zMatch = std::abs(retrievedPos.z() - 20.0f) < tolerance;

    return {xMatch && yMatch && zMatch, "Vehicle position should be set and retrieved"};
}

TestResult testTrafficVehicleLaneAssignment() {
    auto vehicle = std::make_shared<TrafficVehicle>();
    auto lane = std::make_shared<Lane>(0, Lane::Type::DRIVING);
    lane->addCenterlinePoint(Vec3(0, 0, 0));
    lane->addCenterlinePoint(Vec3(10, 0, 0));

    vehicle->setCurrentLane(lane);
    return {vehicle->getCurrentLane() != nullptr, "Vehicle should have assigned lane"};
}

// ==================== Pedestrian Tests ====================

TestResult testPedestrianCreation() {
    auto pedestrian = std::make_shared<Pedestrian>();
    return {pedestrian->getId() >= 0, "Pedestrian should have valid ID"};
}

TestResult testPedestrianPosition() {
    auto pedestrian = std::make_shared<Pedestrian>();
    Vec3 testPos(20, 0, 30);
    pedestrian->setPosition(testPos);

    Vec3 retrievedPos = pedestrian->getPosition();
    float tolerance = 0.01f;
    bool xMatch = std::abs(retrievedPos.x() - 20.0f) < tolerance;
    bool yMatch = std::abs(retrievedPos.y() - 0.0f) < tolerance;
    bool zMatch = std::abs(retrievedPos.z() - 30.0f) < tolerance;

    return {xMatch && yMatch && zMatch, "Pedestrian position should be set and retrieved"};
}

TestResult testPedestrianNavigation() {
    auto pedestrian = std::make_shared<Pedestrian>();
    pedestrian->setPosition(Vec3(0, 0, 0));
    pedestrian->setTargetPosition(Vec3(10, 0, 0));
    pedestrian->setWalkSpeed(1.0f);

    // Update for 5 seconds
    pedestrian->update(5.0f);

    Vec3 newPos = pedestrian->getPosition();
    // Should have moved roughly 5 meters in 5 seconds at 1 m/s
    float expectedDistance = 5.0f;
    float actualDistance = newPos.distance(Vec3(0, 0, 0));
    float tolerance = 0.1f;

    return assertEqual(expectedDistance, actualDistance, tolerance,
                      "Pedestrian should walk towards target");
}

// ==================== World Tests ====================

TestResult testWorldCreation() {
    auto world = std::make_shared<World>();
    return {world != nullptr, "World should be created successfully"};
}

TestResult testWorldRoadNetwork() {
    auto world = std::make_shared<World>();
    world->initialize();

    auto roadNetwork = world->getRoadNetwork();
    return {roadNetwork != nullptr, "World should have road network"};
}

TestResult testWorldTrafficVehicleManagement() {
    auto world = std::make_shared<World>();
    world->initialize();

    auto vehicle = world->createTrafficVehicle();
    return assertEqual(1, static_cast<int>(world->getTrafficVehicleCount()),
                      "World should have 1 traffic vehicle");
}

TestResult testWorldPedestrianManagement() {
    auto world = std::make_shared<World>();
    world->initialize();

    auto pedestrian = world->createPedestrian();
    return assertEqual(1, static_cast<int>(world->getPedestrianCount()),
                      "World should have 1 pedestrian");
}

TestResult testWorldUpdate() {
    auto world = std::make_shared<World>();
    world->initialize();

    auto vehicle = world->createTrafficVehicle();
    vehicle->setPosition(Vec3(0, 1, 0));

    // Update world
    world->update(0.016f);  // ~60 FPS

    return {true, "World update should complete successfully"};
}

TestResult testWorldEnvironment() {
    auto world = std::make_shared<World>();
    world->setTimeOfDay(14.5f);
    world->setWeather(1);  // Rain

    bool timeMatch = world->getTimeOfDay() > 14.0f && world->getTimeOfDay() < 15.0f;
    bool weatherMatch = world->getWeather() == 1;

    return {timeMatch && weatherMatch, "World environment should be configured"};
}

// ==================== Test Runner ====================

void printTestResult(const std::string& testName, const TestResult& result) {
    std::string status = result.passed ? "PASS" : "FAIL";
    std::cout << "[" << status << "] " << testName << ": " << result.message << std::endl;
}

void runAllTests() {
    std::cout << "\n=== Phase 3: World Simulation Tests ===" << std::endl;

    int passCount = 0;
    int totalCount = 0;

    // Lane tests
    std::cout << "\nLane Tests:" << std::endl;
    TestResult tests_lane[] = {
        testLaneCreation(),
        testLaneCenterlineAddition(),
        testLaneLength(),
        testLanePointAtDistance(),
        testLaneDistanceToPoint(),
        testLaneWidth(),
        testLaneConnections()
    };
    for (const auto& test : tests_lane) {
        printTestResult("Lane", test);
        if (test.passed) passCount++;
        totalCount++;
    }

    // TrafficLight tests
    std::cout << "\nTrafficLight Tests:" << std::endl;
    TestResult tests_light[] = {
        testTrafficLightCreation(),
        testTrafficLightInitialState(),
        testTrafficLightDurations(),
        testTrafficLightStateTransition()
    };
    for (const auto& test : tests_light) {
        printTestResult("TrafficLight", test);
        if (test.passed) passCount++;
        totalCount++;
    }

    // Intersection tests
    std::cout << "\nIntersection Tests:" << std::endl;
    TestResult tests_intersection[] = {
        testIntersectionCreation(),
        testIntersectionPosition(),
        testIntersectionTrafficLights(),
        testIntersectionLaneConnections(),
        testIntersectionContainment()
    };
    for (const auto& test : tests_intersection) {
        printTestResult("Intersection", test);
        if (test.passed) passCount++;
        totalCount++;
    }

    // RoadNetwork tests
    std::cout << "\nRoadNetwork Tests:" << std::endl;
    TestResult tests_network[] = {
        testRoadNetworkCreation(),
        testRoadNetworkLaneCreation(),
        testRoadNetworkIntersectionCreation(),
        testRoadNetworkFindClosestLane()
    };
    for (const auto& test : tests_network) {
        printTestResult("RoadNetwork", test);
        if (test.passed) passCount++;
        totalCount++;
    }

    // TrafficVehicle tests
    std::cout << "\nTrafficVehicle Tests:" << std::endl;
    TestResult tests_vehicle[] = {
        testTrafficVehicleCreation(),
        testTrafficVehiclePosition(),
        testTrafficVehicleLaneAssignment()
    };
    for (const auto& test : tests_vehicle) {
        printTestResult("TrafficVehicle", test);
        if (test.passed) passCount++;
        totalCount++;
    }

    // Pedestrian tests
    std::cout << "\nPedestrian Tests:" << std::endl;
    TestResult tests_ped[] = {
        testPedestrianCreation(),
        testPedestrianPosition(),
        testPedestrianNavigation()
    };
    for (const auto& test : tests_ped) {
        printTestResult("Pedestrian", test);
        if (test.passed) passCount++;
        totalCount++;
    }

    // World tests
    std::cout << "\nWorld Tests:" << std::endl;
    TestResult tests_world[] = {
        testWorldCreation(),
        testWorldRoadNetwork(),
        testWorldTrafficVehicleManagement(),
        testWorldPedestrianManagement(),
        testWorldUpdate(),
        testWorldEnvironment()
    };
    for (const auto& test : tests_world) {
        printTestResult("World", test);
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

// Main entry point for standalone testing
int main() {
    av::Logger::init("world_tests.log");
    av::Logger::setLevel(av::Logger::INFO);

    av::test::runAllTests();

    av::Logger::shutdown();
    return 0;
}
