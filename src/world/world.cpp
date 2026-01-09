#include "av/world/world.hpp"
#include "av/world/road_network.hpp"
#include "av/foundation/logging.hpp"
#include <algorithm>

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
    // Update road network (traffic lights, etc.)
    if (roadNetwork_) {
        roadNetwork_->update(deltaTime);
    }

    // Update all traffic vehicles with road network reference
    for (auto& vehicle : trafficVehicles_) {
        if (vehicle) {
            vehicle->update(deltaTime, roadNetwork_);
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

const std::shared_ptr<RoadNetwork> World::getRoadNetwork() const {
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

const std::vector<std::shared_ptr<TrafficVehicle>>& World::getTrafficVehicles() const {
    return trafficVehicles_;
}

size_t World::getTrafficVehicleCount() const {
    return trafficVehicles_.size();
}

const std::vector<std::shared_ptr<Pedestrian>>& World::getPedestrians() const {
    return pedestrians_;
}

size_t World::getPedestrianCount() const {
    return pedestrians_.size();
}

void World::setTimeOfDay(float hour) {
    if (hour < 0 || hour > 24) {
        AV_WARN("Invalid time of day: {}. Expected 0-24.", hour);
        return;
    }
    timeOfDay_ = hour;
    AV_DEBUG("Time of day set to: {:.1f}", hour);
}

float World::getTimeOfDay() const {
    return timeOfDay_;
}

void World::setWeather(int weatherType) {
    if (weatherType < 0 || weatherType > 2) {
        AV_WARN("Invalid weather type: {}. Expected 0-2.", weatherType);
        return;
    }
    weatherType_ = weatherType;
    static const char* weatherNames[] = {"Clear", "Rain", "Fog"};
    AV_DEBUG("Weather set to: {}", weatherNames[weatherType]);
}

int World::getWeather() const {
    return weatherType_;
}

} // namespace av
