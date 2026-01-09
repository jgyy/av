#include "av/world/world.hpp"
#include "av/world/road_network.hpp"
#include "av/foundation/logging.hpp"

namespace av {

World::World() {
    AV_DEBUG("World created");
    roadNetwork_ = std::make_shared<RoadNetwork>();
}

World::~World() {
    AV_DEBUG("World destroyed");
}

void World::initialize() {
    AV_INFO("Initializing world");
    if (roadNetwork_) {
        // Road network initialization if needed
    }
}

void World::update(float deltaTime) {
    // Update all traffic vehicles
    for (auto& vehicle : trafficVehicles_) {
        if (vehicle) {
            vehicle->update(deltaTime);
        }
    }

    // Update all pedestrians
    for (auto& pedestrian : pedestrians_) {
        if (pedestrian) {
            pedestrian->update(deltaTime);
        }
    }
}

std::shared_ptr<RoadNetwork> World::getRoadNetwork() {
    return roadNetwork_;
}

std::shared_ptr<TrafficVehicle> World::createTrafficVehicle() {
    auto vehicle = std::make_shared<TrafficVehicle>();
    trafficVehicles_.push_back(vehicle);
    AV_DEBUG("Created traffic vehicle (total: {})", trafficVehicles_.size());
    return vehicle;
}

void World::removeTrafficVehicle(const std::shared_ptr<TrafficVehicle>& vehicle) {
    auto it = std::find(trafficVehicles_.begin(), trafficVehicles_.end(), vehicle);
    if (it != trafficVehicles_.end()) {
        trafficVehicles_.erase(it);
        AV_DEBUG("Removed traffic vehicle (remaining: {})", trafficVehicles_.size());
    }
}

std::shared_ptr<Pedestrian> World::createPedestrian() {
    auto pedestrian = std::make_shared<Pedestrian>();
    pedestrians_.push_back(pedestrian);
    AV_DEBUG("Created pedestrian (total: {})", pedestrians_.size());
    return pedestrian;
}

void World::removePedestrian(const std::shared_ptr<Pedestrian>& pedestrian) {
    auto it = std::find(pedestrians_.begin(), pedestrians_.end(), pedestrian);
    if (it != pedestrians_.end()) {
        pedestrians_.erase(it);
        AV_DEBUG("Removed pedestrian (remaining: {})", pedestrians_.size());
    }
}

void World::setTimeOfDay(float hour) {
    if (hour < 0 || hour > 24) {
        AV_WARN("Invalid time of day: {}. Expected 0-24.", hour);
        return;
    }
    AV_DEBUG("Time of day set to: {:.1f}", hour);
}

void World::setWeather(int weatherType) {
    if (weatherType < 0 || weatherType > 2) {
        AV_WARN("Invalid weather type: {}. Expected 0-2.", weatherType);
        return;
    }
    static const char* weatherNames[] = {"Clear", "Rain", "Fog"};
    AV_DEBUG("Weather set to: {}", weatherNames[weatherType]);
}

// RoadNetwork implementation

RoadNetwork::RoadNetwork() {
    AV_DEBUG("RoadNetwork created");
}

RoadNetwork::~RoadNetwork() {
    AV_DEBUG("RoadNetwork destroyed");
}

void RoadNetwork::loadFromFile(const std::string& filePath) {
    AV_INFO("Loading road network from: {}", filePath);
    // TODO: Load from JSON file
}

Vec3 RoadNetwork::getClosestLanePoint(const Vec3& position) {
    // TODO: Find closest point on any lane to given position
    return position;
}

// TrafficVehicle implementation

TrafficVehicle::TrafficVehicle() {
    AV_DEBUG("TrafficVehicle created");
}

TrafficVehicle::~TrafficVehicle() {
    AV_DEBUG("TrafficVehicle destroyed");
}

void TrafficVehicle::update(float deltaTime) {
    // TODO: Implement AI behavior for traffic vehicles
}

void TrafficVehicle::setPosition(const Vec3& position) {
    position_ = position;
}

Vec3 TrafficVehicle::getPosition() const {
    return position_;
}

// Pedestrian implementation

Pedestrian::Pedestrian() {
    AV_DEBUG("Pedestrian created");
}

Pedestrian::~Pedestrian() {
    AV_DEBUG("Pedestrian destroyed");
}

void Pedestrian::update(float deltaTime) {
    // TODO: Implement pedestrian navigation
}

void Pedestrian::setPosition(const Vec3& position) {
    position_ = position;
}

Vec3 Pedestrian::getPosition() const {
    return position_;
}

} // namespace av
