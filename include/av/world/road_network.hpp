#pragma once

#include "av/foundation/math.hpp"
#include <vector>
#include <memory>
#include <string>
#include <map>

namespace av {

// Forward declarations
class Lane;
class Intersection;
class TrafficLight;

// Lane centerline and boundaries
class Lane {
public:
    enum class Type {
        DRIVING,
        TURN_LEFT,
        TURN_RIGHT,
        PARKING
    };

    Lane(int id, Type type = Type::DRIVING);
    ~Lane();

    // Centerline management
    void addCenterlinePoint(const Vec3& point);
    const std::vector<Vec3>& getCenterline() const { return centerline_; }

    // Boundary management
    void setLeftBoundary(float distance) { leftBoundary_ = distance; }
    void setRightBoundary(float distance) { rightBoundary_ = distance; }
    float getLeftBoundary() const { return leftBoundary_; }
    float getRightBoundary() const { return rightBoundary_; }

    // Lane properties
    int getId() const { return id_; }
    Type getType() const { return type_; }
    float getWidth() const { return leftBoundary_ + rightBoundary_; }
    float getLength() const;

    // Navigation
    Vec3 getPointAtDistance(float distance) const;
    Vec3 getPointAtParameter(float t) const;  // t in [0, 1]
    Vec3 getLateralOffset(const Vec3& position, float lateralDistance) const;
    float getDistanceToPoint(const Vec3& position) const;

    // Connections
    void setNextLane(std::shared_ptr<Lane> lane) { nextLane_ = lane; }
    void setPreviousLane(std::shared_ptr<Lane> lane) { previousLane_ = lane; }
    std::shared_ptr<Lane> getNextLane() const { return nextLane_; }
    std::shared_ptr<Lane> getPreviousLane() const { return previousLane_; }

    // Speed limit
    void setSpeedLimit(float speed) { speedLimit_ = speed; }
    float getSpeedLimit() const { return speedLimit_; }

private:
    int id_;
    Type type_;
    std::vector<Vec3> centerline_;
    float leftBoundary_ = 2.0f;   // 2 meters to left edge
    float rightBoundary_ = 2.0f;  // 2 meters to right edge
    float speedLimit_ = 13.4f;    // m/s (about 50 km/h)

    std::shared_ptr<Lane> nextLane_;
    std::shared_ptr<Lane> previousLane_;

    Vec3 interpolateCenterline(float distance) const;
};

// Traffic light state machine
class TrafficLight {
public:
    enum class State {
        RED,
        YELLOW,
        GREEN
    };

    TrafficLight(int id);

    // Control
    void update(float deltaTime);
    void setState(State state);
    State getState() const { return currentState_; }

    // Timing
    void setGreenDuration(float duration) { greenDuration_ = duration; }
    void setYellowDuration(float duration) { yellowDuration_ = duration; }
    void setRedDuration(float duration) { redDuration_ = duration; }

    float getTimeInState() const { return timeInState_; }
    int getId() const { return id_; }

private:
    int id_;
    State currentState_ = State::RED;
    float timeInState_ = 0.0f;

    float greenDuration_ = 30.0f;   // 30 seconds
    float yellowDuration_ = 3.0f;   // 3 seconds
    float redDuration_ = 30.0f;     // 30 seconds

    void transitionState();
};

// Intersection with traffic lights
class Intersection {
public:
    Intersection(int id, const Vec3& position);

    // Properties
    int getId() const { return id_; }
    const Vec3& getPosition() const { return position_; }
    float getRadius() const { return radius_; }
    void setRadius(float radius) { radius_ = radius; }

    // Traffic lights
    void addTrafficLight(std::shared_ptr<TrafficLight> light);
    std::shared_ptr<TrafficLight> getTrafficLight(int index) const;
    size_t getTrafficLightCount() const { return trafficLights_.size(); }

    // Connected lanes
    void addIncomingLane(std::shared_ptr<Lane> lane);
    void addOutgoingLane(std::shared_ptr<Lane> lane);
    const std::vector<std::shared_ptr<Lane>>& getIncomingLanes() const { return incomingLanes_; }
    const std::vector<std::shared_ptr<Lane>>& getOutgoingLanes() const { return outgoingLanes_; }

    // Update
    void update(float deltaTime);

    // Check if point is in intersection
    bool isPointInIntersection(const Vec3& point) const;

private:
    int id_;
    Vec3 position_;
    float radius_ = 20.0f;  // 20 meter radius

    std::vector<std::shared_ptr<TrafficLight>> trafficLights_;
    std::vector<std::shared_ptr<Lane>> incomingLanes_;
    std::vector<std::shared_ptr<Lane>> outgoingLanes_;
};

// Road network manager
class RoadNetwork {
public:
    RoadNetwork();
    ~RoadNetwork();

    // Lane management
    std::shared_ptr<Lane> createLane(Lane::Type type = Lane::Type::DRIVING);
    std::shared_ptr<Lane> getLane(int id) const;
    const std::vector<std::shared_ptr<Lane>>& getLanes() const { return lanes_; }
    size_t getLaneCount() const { return lanes_.size(); }

    // Intersection management
    std::shared_ptr<Intersection> createIntersection(const Vec3& position);
    std::shared_ptr<Intersection> getIntersection(int id) const;
    const std::vector<std::shared_ptr<Intersection>>& getIntersections() const { return intersections_; }
    size_t getIntersectionCount() const { return intersections_.size(); }

    // Queries
    std::shared_ptr<Lane> findClosestLane(const Vec3& position, float maxDistance = 50.0f) const;
    std::shared_ptr<Lane> findLaneAhead(const Vec3& position, const Vec3& direction) const;

    // Update
    void update(float deltaTime);

    // Save/Load
    bool loadFromJson(const std::string& filePath);
    bool saveToJson(const std::string& filePath) const;

    // Bounds
    void setBounds(const Vec3& min, const Vec3& max) { boundsMin_ = min; boundsMax_ = max; }
    const Vec3& getBoundsMin() const { return boundsMin_; }
    const Vec3& getBoundsMax() const { return boundsMax_; }

private:
    int nextLaneId_ = 0;
    int nextIntersectionId_ = 0;

    std::vector<std::shared_ptr<Lane>> lanes_;
    std::vector<std::shared_ptr<Intersection>> intersections_;
    std::map<int, std::shared_ptr<Lane>> laneMap_;
    std::map<int, std::shared_ptr<Intersection>> intersectionMap_;

    Vec3 boundsMin_ = Vec3(-100, -10, -100);
    Vec3 boundsMax_ = Vec3(100, 10, 100);
};

} // namespace av
