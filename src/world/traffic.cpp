#include "av/world/traffic.hpp"
#include "av/foundation/logging.hpp"
#include <cmath>
#include <algorithm>

namespace av {

// Static member initialization
int TrafficVehicle::nextId_ = 0;
int Pedestrian::nextId_ = 0;

// ==================== TrafficVehicle Implementation ====================

TrafficVehicle::TrafficVehicle()
    : id_(nextId_++), dynamics_(std::make_shared<VehicleDynamics>(vehicleParams_)) {
    AV_DEBUG("TrafficVehicle created: id={}", id_);
}

TrafficVehicle::~TrafficVehicle() {
    AV_DEBUG("TrafficVehicle destroyed: id={}", id_);
}

void TrafficVehicle::update(float deltaTime, std::shared_ptr<RoadNetwork> roadNetwork) {
    if (!dynamics_) return;

    // Update AI behavior
    updateAI(deltaTime, roadNetwork);

    // Update physics
    updatePhysics(deltaTime);
}

void TrafficVehicle::updateAI(float deltaTime, std::shared_ptr<RoadNetwork> roadNetwork) {
    // Find current lane if not set
    if (!currentLane_ && roadNetwork) {
        currentLane_ = roadNetwork->findClosestLane(getPosition());
    }

    if (!currentLane_) return;

    // Check for traffic lights
    if (isAtTrafficLight(roadNetwork)) {
        if (shouldWaitAtLight(roadNetwork)) {
            setState(State::WAITING_AT_LIGHT);
            stoppedAtLight_ = true;
            throttleInput_ = 0.0f;
            brakeInput_ = 1.0f;
            waitingAtLightTime_ += deltaTime;
            return;
        } else {
            stoppedAtLight_ = false;
            waitingAtLightTime_ = 0.0f;
            setState(State::FOLLOWING_LANE);
        }
    }

    // Lane following behavior
    setState(State::FOLLOWING_LANE);

    // Get desired point on lane ahead
    Vec3 desiredPoint = getDesiredLanePoint();

    // Calculate steering input using Pure Pursuit
    steeringInput_ = calculateSteeringInput(desiredPoint);

    // Calculate speed input
    float roadSpeedLimit = currentLane_->getSpeedLimit();
    throttleInput_ = calculateSpeedInput(roadSpeedLimit, 50.0f);
    brakeInput_ = 0.0f;

    // Apply control inputs to physics
    dynamics_->setSteeringAngle(steeringInput_);
    dynamics_->setThrottle(throttleInput_);
    dynamics_->setBrake(brakeInput_);
}

void TrafficVehicle::updatePhysics(float deltaTime) {
    if (!dynamics_) return;

    dynamics_->update(deltaTime);

    // Update distance along lane
    if (currentLane_) {
        distanceAlongLane_ = currentLane_->getDistanceToPoint(getPosition());

        // Check if we've passed the end of the lane
        if (distanceAlongLane_ > currentLane_->getLength() * 0.9f) {
            auto nextLane = currentLane_->getNextLane();
            if (nextLane) {
                currentLane_ = nextLane;
                distanceAlongLane_ = 0.0f;
            }
        }
    }
}

Vec3 TrafficVehicle::getDesiredLanePoint() const {
    if (!currentLane_) return getPosition();

    // Look ahead distance based on current speed
    float currentSpeed = getSpeed();
    float lookAheadDistance = std::max(5.0f, currentSpeed * 0.5f);  // 0.5 seconds ahead
    float targetDistance = distanceAlongLane_ + lookAheadDistance;

    return currentLane_->getPointAtDistance(targetDistance);
}

float TrafficVehicle::calculateSteeringInput(const Vec3& desiredPoint) {
    Vec3 currentPos = getPosition();
    Vec3 forward = dynamics_->getVelocity().normalized();

    if (forward.norm() < 0.1f) {
        forward = Vec3::UnitX();  // Default forward if stationary
    }

    // Vector from current to desired point
    Vec3 toDesired = (desiredPoint - currentPos).normalized();

    // Calculate cross product for steering error
    Vec3 right = Vec3(0, 1, 0).cross(forward).normalized();
    float crossError = toDesired.dot(right);

    // Pure Pursuit steering law
    float wheelbase = vehicleParams_.wheelbase;
    float currentSpeed = getSpeed();

    if (currentSpeed > 0.1f) {
        float steeringAngle = std::atan2(crossError * 2.0f * wheelbase, currentSpeed * 2.0f);
        return std::clamp(steeringAngle * 180.0f / 3.14159f, -vehicleParams_.maxSteeringAngle, vehicleParams_.maxSteeringAngle);
    }

    return 0.0f;
}

float TrafficVehicle::calculateSpeedInput(float roadSpeed, float distanceToNextVehicle) {
    float currentSpeed = getSpeed();
    float speedError = roadSpeed - currentSpeed;

    // PID-like speed control
    float throttle = 0.0f;
    if (currentSpeed < roadSpeed && speedError > 0.1f) {
        throttle = std::clamp(speedError / roadSpeed, 0.0f, 1.0f);
    } else if (currentSpeed > roadSpeed) {
        // For now, just reduce throttle; actual braking handled in update()
        throttle = 0.0f;
    }

    return throttle;
}

bool TrafficVehicle::isAtTrafficLight(std::shared_ptr<RoadNetwork> roadNetwork) const {
    if (!roadNetwork || !currentLane_) return false;

    // Check if we're near an intersection
    const auto& intersections = roadNetwork->getIntersections();
    for (const auto& intersection : intersections) {
        if (!intersection) continue;

        // Check if current lane is an incoming lane to this intersection
        const auto& incomingLanes = intersection->getIncomingLanes();
        for (const auto& incomingLane : incomingLanes) {
            if (incomingLane && incomingLane->getId() == currentLane_->getId()) {
                // Check distance to intersection
                float distToIntersection = currentLane_->getLength() - distanceAlongLane_;
                if (distToIntersection < 20.0f) {  // Within 20 meters
                    return true;
                }
            }
        }
    }

    return false;
}

bool TrafficVehicle::shouldWaitAtLight(std::shared_ptr<RoadNetwork> roadNetwork) {
    if (!roadNetwork || !currentLane_) return false;

    const auto& intersections = roadNetwork->getIntersections();
    for (const auto& intersection : intersections) {
        if (!intersection) continue;

        const auto& incomingLanes = intersection->getIncomingLanes();
        for (const auto& incomingLane : incomingLanes) {
            if (incomingLane && incomingLane->getId() == currentLane_->getId()) {
                // Check if any traffic light is RED
                for (size_t i = 0; i < intersection->getTrafficLightCount(); ++i) {
                    auto light = intersection->getTrafficLight(i);
                    if (light && light->getState() == TrafficLight::State::RED) {
                        return true;
                    }
                }
            }
        }
    }

    return false;
}

void TrafficVehicle::setPosition(const Vec3& position) {
    if (dynamics_) {
        dynamics_->setPosition(position);
    }
}

Vec3 TrafficVehicle::getPosition() const {
    if (dynamics_) {
        return dynamics_->getPosition();
    }
    return Vec3::Zero();
}

void TrafficVehicle::setRotation(const Quat& rotation) {
    if (dynamics_) {
        dynamics_->setRotation(rotation);
    }
}

Quat TrafficVehicle::getRotation() const {
    if (dynamics_) {
        return dynamics_->getRotation();
    }
    return Quat::Identity();
}

Vec3 TrafficVehicle::getVelocity() const {
    if (dynamics_) {
        return dynamics_->getVelocity();
    }
    return Vec3::Zero();
}

float TrafficVehicle::getSpeed() const {
    return getVelocity().norm();
}

void TrafficVehicle::setCurrentLane(std::shared_ptr<Lane> lane) {
    currentLane_ = lane;
    if (currentLane_) {
        distanceAlongLane_ = 0.0f;
        AV_DEBUG("TrafficVehicle {}: set to lane {}", id_, currentLane_->getId());
    }
}

std::shared_ptr<Lane> TrafficVehicle::getCurrentLane() const {
    return currentLane_;
}

float TrafficVehicle::getDistanceAlongLane() const {
    return distanceAlongLane_;
}

// ==================== Pedestrian Implementation ====================

Pedestrian::Pedestrian()
    : id_(nextId_++) {
    AV_DEBUG("Pedestrian created: id={}", id_);
}

Pedestrian::~Pedestrian() {
    AV_DEBUG("Pedestrian destroyed: id={}", id_);
}

void Pedestrian::update(float deltaTime) {
    switch (state_) {
        case State::WALKING:
            moveTowardsTarget(deltaTime);
            if (hasReachedTarget()) {
                setState(State::IDLE);
                velocity_ = Vec3::Zero();
            }
            break;

        case State::WAITING:
            waitingTime_ += deltaTime;
            if (waitingTime_ >= maxWaitingTime_) {
                waitingTime_ = 0.0f;
                setState(State::IDLE);
            }
            break;

        case State::IDLE:
        default:
            velocity_ = Vec3::Zero();
            break;
    }
}

void Pedestrian::moveTowardsTarget(float deltaTime) {
    if (position_.distance(targetPosition_) < arrivalThreshold_) {
        return;
    }

    Vec3 direction = (targetPosition_ - position_).normalized();
    position_ += direction * walkSpeed_ * deltaTime;
    velocity_ = direction * walkSpeed_;
}

void Pedestrian::setPosition(const Vec3& position) {
    position_ = position;
}

Vec3 Pedestrian::getPosition() const {
    return position_;
}

void Pedestrian::setTargetPosition(const Vec3& target) {
    targetPosition_ = target;
    setState(State::WALKING);
}

Vec3 Pedestrian::getTargetPosition() const {
    return targetPosition_;
}

Vec3 Pedestrian::getVelocity() const {
    return velocity_;
}

float Pedestrian::getSpeed() const {
    return velocity_.norm();
}

bool Pedestrian::hasReachedTarget() const {
    return position_.distance(targetPosition_) < arrivalThreshold_;
}

} // namespace av

