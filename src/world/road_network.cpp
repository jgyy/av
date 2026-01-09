#include "av/world/road_network.hpp"
#include "av/foundation/logging.hpp"
#include <cmath>
#include <algorithm>

namespace av {

// Lane Implementation
Lane::Lane(int id, Type type)
    : id_(id), type_(type) {
    AV_DEBUG("Lane created: id=%d", id);
}

Lane::~Lane() {
    AV_DEBUG("Lane destroyed: id=%d", id_);
}

void Lane::addCenterlinePoint(const Vec3& point) {
    centerline_.push_back(point);
}

float Lane::getLength() const {
    if (centerline_.size() < 2) return 0.0f;

    float length = 0.0f;
    for (size_t i = 0; i + 1 < centerline_.size(); ++i) {
        length += (centerline_[i + 1] - centerline_[i]).norm();
    }
    return length;
}

Vec3 Lane::getPointAtDistance(float distance) const {
    return interpolateCenterline(distance);
}

Vec3 Lane::getPointAtParameter(float t) const {
    if (centerline_.size() < 2) return Vec3::Zero();
    if (t <= 0.0f) return centerline_.front();
    if (t >= 1.0f) return centerline_.back();

    float totalLength = getLength();
    float targetDistance = t * totalLength;
    return interpolateCenterline(targetDistance);
}

Vec3 Lane::interpolateCenterline(float distance) const {
    if (centerline_.size() < 2) return Vec3::Zero();

    float currentDist = 0.0f;
    for (size_t i = 0; i + 1 < centerline_.size(); ++i) {
        Vec3 segmentVec = centerline_[i + 1] - centerline_[i];
        float segmentLength = segmentVec.norm();

        if (currentDist + segmentLength >= distance) {
            float t = (segmentLength > 0.0f) ? (distance - currentDist) / segmentLength : 0.0f;
            return centerline_[i] + segmentVec * t;
        }

        currentDist += segmentLength;
    }

    return centerline_.back();
}

Vec3 Lane::getLateralOffset(const Vec3& position, float lateralDistance) const {
    // Find closest point on centerline
    Vec3 closestPoint = getPointAtDistance(getDistanceToPoint(position));

    // Calculate perpendicular direction
    if (centerline_.size() < 2) return position;

    // Use next point to determine direction
    Vec3 direction = Vec3::UnitX();  // Default direction
    for (size_t i = 0; i + 1 < centerline_.size(); ++i) {
        direction = (centerline_[i + 1] - centerline_[i]).normalized();
        break;
    }

    // Right perpendicular (Y-Z plane)
    Vec3 right = Vec3(0, -direction.z(), direction.y()).normalized();

    return closestPoint + right * lateralDistance;
}

float Lane::getDistanceToPoint(const Vec3& position) const {
    if (centerline_.size() < 2) return 0.0f;

    float minDistToStart = 0.0f;
    float minDist = (position - centerline_.front()).norm();

    float currentDist = 0.0f;
    for (size_t i = 0; i + 1 < centerline_.size(); ++i) {
        Vec3 p1 = centerline_[i];
        Vec3 p2 = centerline_[i + 1];
        Vec3 segmentVec = p2 - p1;
        float segmentLength = segmentVec.norm();

        if (segmentLength > 0.0f) {
            float t = std::max(0.0f, std::min(1.0f,
                ((position - p1).dot(segmentVec)) / (segmentLength * segmentLength)));

            Vec3 closest = p1 + segmentVec * t;
            float dist = (position - closest).norm();

            if (dist < minDist) {
                minDist = dist;
                minDistToStart = currentDist + t * segmentLength;
            }
        }

        currentDist += segmentLength;
    }

    return minDistToStart;
}

// TrafficLight Implementation
TrafficLight::TrafficLight(int id)
    : id_(id) {
    AV_DEBUG("TrafficLight created: id=%d", id);
}

void TrafficLight::update(float deltaTime) {
    timeInState_ += deltaTime;

    float currentDuration = 0.0f;
    switch (currentState_) {
        case State::GREEN:
            currentDuration = greenDuration_;
            break;
        case State::YELLOW:
            currentDuration = yellowDuration_;
            break;
        case State::RED:
            currentDuration = redDuration_;
            break;
    }

    if (timeInState_ >= currentDuration) {
        transitionState();
    }
}

void TrafficLight::setState(State state) {
    currentState_ = state;
    timeInState_ = 0.0f;
}

void TrafficLight::transitionState() {
    switch (currentState_) {
        case State::GREEN:
            setState(State::YELLOW);
            break;
        case State::YELLOW:
            setState(State::RED);
            break;
        case State::RED:
            setState(State::GREEN);
            break;
    }
}

// Intersection Implementation
Intersection::Intersection(int id, const Vec3& position)
    : id_(id), position_(position) {
    AV_DEBUG("Intersection created: id=%d at (%.2f, %.2f, %.2f)", id,
             position.x(), position.y(), position.z());
}

void Intersection::addTrafficLight(std::shared_ptr<TrafficLight> light) {
    if (light) {
        trafficLights_.push_back(light);
    }
}

std::shared_ptr<TrafficLight> Intersection::getTrafficLight(int index) const {
    if (index >= 0 && index < static_cast<int>(trafficLights_.size())) {
        return trafficLights_[index];
    }
    return nullptr;
}

void Intersection::addIncomingLane(std::shared_ptr<Lane> lane) {
    if (lane) {
        incomingLanes_.push_back(lane);
    }
}

void Intersection::addOutgoingLane(std::shared_ptr<Lane> lane) {
    if (lane) {
        outgoingLanes_.push_back(lane);
    }
}

void Intersection::update(float deltaTime) {
    for (auto& light : trafficLights_) {
        if (light) {
            light->update(deltaTime);
        }
    }
}

bool Intersection::isPointInIntersection(const Vec3& point) const {
    float dist = (point - position_).norm();
    return dist <= radius_;
}

// RoadNetwork Implementation
RoadNetwork::RoadNetwork() {
    AV_DEBUG("RoadNetwork created");
}

RoadNetwork::~RoadNetwork() {
    AV_DEBUG("RoadNetwork destroyed with %zu lanes and %zu intersections",
             lanes_.size(), intersections_.size());
}

std::shared_ptr<Lane> RoadNetwork::createLane(Lane::Type type) {
    auto lane = std::make_shared<Lane>(nextLaneId_++, type);
    lanes_.push_back(lane);
    laneMap_[lane->getId()] = lane;
    return lane;
}

std::shared_ptr<Lane> RoadNetwork::getLane(int id) const {
    auto it = laneMap_.find(id);
    if (it != laneMap_.end()) {
        return it->second;
    }
    return nullptr;
}

std::shared_ptr<Intersection> RoadNetwork::createIntersection(const Vec3& position) {
    auto intersection = std::make_shared<Intersection>(nextIntersectionId_++, position);
    intersections_.push_back(intersection);
    intersectionMap_[intersection->getId()] = intersection;
    return intersection;
}

std::shared_ptr<Intersection> RoadNetwork::getIntersection(int id) const {
    auto it = intersectionMap_.find(id);
    if (it != intersectionMap_.end()) {
        return it->second;
    }
    return nullptr;
}

std::shared_ptr<Lane> RoadNetwork::findClosestLane(const Vec3& position, float maxDistance) const {
    std::shared_ptr<Lane> closestLane = nullptr;
    float minDistance = maxDistance;

    for (const auto& lane : lanes_) {
        if (!lane) continue;

        // Simplified: check distance to lane centerline points
        for (const auto& point : lane->getCenterline()) {
            float dist = (position - point).norm();
            if (dist < minDistance) {
                minDistance = dist;
                closestLane = lane;
            }
        }
    }

    return closestLane;
}

std::shared_ptr<Lane> RoadNetwork::findLaneAhead(const Vec3& position, const Vec3& direction) const {
    // Find closest lane
    auto currentLane = findClosestLane(position);
    if (!currentLane) return nullptr;

    // Check if we should transition to next lane
    float laneProgress = currentLane->getDistanceToPoint(position);
    if (laneProgress > currentLane->getLength() * 0.8f) {
        // Near end of lane, try next lane
        auto nextLane = currentLane->getNextLane();
        if (nextLane) return nextLane;
    }

    return currentLane;
}

void RoadNetwork::update(float deltaTime) {
    for (auto& intersection : intersections_) {
        if (intersection) {
            intersection->update(deltaTime);
        }
    }
}

bool RoadNetwork::loadFromJson(const std::string& filePath) {
    AV_INFO("Loading road network from: %s", filePath.c_str());

    // Note: Actual JSON loading is handled by WorldLoader class
    // This method is kept for backward compatibility

    // Try to load using WorldLoader if available
    #ifdef HAVE_WORLD_LOADER
        // Forward to WorldLoader::loadRoadNetwork(filePath, shared_from_this())
        // For now, return false as it requires shared_ptr support
    #endif

    return false;
}

bool RoadNetwork::saveToJson(const std::string& filePath) const {
    AV_INFO("Saving road network to: %s", filePath.c_str());

    // Note: Actual JSON saving is handled by WorldLoader class
    // This method is kept for backward compatibility

    #ifdef HAVE_WORLD_LOADER
        // Forward to WorldLoader::saveRoadNetwork
    #endif

    return false;
}

} // namespace av
