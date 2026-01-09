#pragma once

#include "av/foundation/math.hpp"
#include "av/physics/vehicle_dynamics.hpp"
#include "av/world/road_network.hpp"
#include <memory>
#include <vector>

namespace av {

// AI-controlled traffic vehicle with lane following
class TrafficVehicle {
public:
    enum class State {
        IDLE,
        FOLLOWING_LANE,
        WAITING_AT_LIGHT,
        TURNING_AT_INTERSECTION
    };

    TrafficVehicle();
    ~TrafficVehicle();

    // Update vehicle AI and physics
    void update(float deltaTime, std::shared_ptr<RoadNetwork> roadNetwork);

    // Position and movement
    void setPosition(const Vec3& position);
    Vec3 getPosition() const;

    void setRotation(const Quat& rotation);
    Quat getRotation() const;

    Vec3 getVelocity() const;
    float getSpeed() const;

    // Lane assignment
    void setCurrentLane(std::shared_ptr<Lane> lane);
    std::shared_ptr<Lane> getCurrentLane() const;
    float getDistanceAlongLane() const;

    // State management
    State getState() const { return state_; }
    void setState(State state) { state_ = state; }

    // Control
    void setTargetSpeed(float speed) { targetSpeed_ = speed; }
    float getTargetSpeed() const { return targetSpeed_; }

    // Vehicle parameters
    void setColor(const Vec3& color) { color_ = color; }
    Vec3 getColor() const { return color_; }

    int getId() const { return id_; }

private:
    static int nextId_;

    int id_;
    State state_ = State::IDLE;
    Vec3 color_ = Vec3(0.8f, 0.2f, 0.2f);  // Red by default

    // Physics
    std::shared_ptr<VehicleDynamics> dynamics_;
    VehicleParams vehicleParams_;

    // Lane following
    std::shared_ptr<Lane> currentLane_;
    float distanceAlongLane_ = 0.0f;
    float targetSpeed_ = 13.4f;  // m/s (about 50 km/h)

    // AI behavior
    float steeringInput_ = 0.0f;
    float throttleInput_ = 0.0f;
    float brakeInput_ = 0.0f;

    // AI state tracking
    float waitingAtLightTime_ = 0.0f;
    bool stoppedAtLight_ = false;

    // Control loop
    void updateAI(float deltaTime, std::shared_ptr<RoadNetwork> roadNetwork);
    void updatePhysics(float deltaTime);

    // Lane following controller
    Vec3 getDesiredLanePoint() const;
    float calculateSteeringInput(const Vec3& desiredPoint);
    float calculateSpeedInput(float roadSpeed, float distanceToNextVehicle);

    // Traffic light handling
    bool isAtTrafficLight(std::shared_ptr<RoadNetwork> roadNetwork) const;
    bool shouldWaitAtLight(std::shared_ptr<RoadNetwork> roadNetwork);
};

// Pedestrian with simple navigation
class Pedestrian {
public:
    enum class State {
        IDLE,
        WALKING,
        WAITING
    };

    Pedestrian();
    ~Pedestrian();

    // Update pedestrian
    void update(float deltaTime);

    // Position and movement
    void setPosition(const Vec3& position);
    Vec3 getPosition() const;

    void setTargetPosition(const Vec3& target);
    Vec3 getTargetPosition() const;

    Vec3 getVelocity() const;
    float getSpeed() const;

    // State management
    State getState() const { return state_; }
    void setState(State state) { state_ = state; }

    // Control
    void setWalkSpeed(float speed) { walkSpeed_ = speed; }
    float getWalkSpeed() const { return walkSpeed_; }

    void setColor(const Vec3& color) { color_ = color; }
    Vec3 getColor() const { return color_; }

    int getId() const { return id_; }
    bool hasReachedTarget() const;

private:
    static int nextId_;

    int id_;
    State state_ = State::IDLE;
    Vec3 color_ = Vec3(0.2f, 0.8f, 0.2f);  // Green by default

    // Position
    Vec3 position_ = Vec3::Zero();
    Vec3 velocity_ = Vec3::Zero();
    Vec3 targetPosition_ = Vec3::Zero();

    // Movement
    float walkSpeed_ = 1.4f;  // m/s (about 5 km/h)
    float arrivalThreshold_ = 0.5f;  // meters

    // Waiting
    float waitingTime_ = 0.0f;
    float maxWaitingTime_ = 5.0f;  // seconds

    // Navigation
    void moveTowardsTarget(float deltaTime);
};

} // namespace av
