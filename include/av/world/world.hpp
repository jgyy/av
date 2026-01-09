#pragma once

#include "av/foundation/math.hpp"
#include "av/world/road_network.hpp"
#include "av/world/traffic.hpp"
#include <vector>
#include <memory>

namespace av {

// Main world object - orchestrates all entities
class World {
public:
    World();
    ~World();

    // Initialize world
    void initialize();

    // Update world state
    void update(float deltaTime);

    // Road network access
    std::shared_ptr<RoadNetwork> getRoadNetwork();
    const std::shared_ptr<RoadNetwork> getRoadNetwork() const;

    // Traffic vehicle management
    std::shared_ptr<TrafficVehicle> createTrafficVehicle();
    void removeTrafficVehicle(const std::shared_ptr<TrafficVehicle>& vehicle);
    const std::vector<std::shared_ptr<TrafficVehicle>>& getTrafficVehicles() const;
    size_t getTrafficVehicleCount() const;

    // Pedestrian management
    std::shared_ptr<Pedestrian> createPedestrian();
    void removePedestrian(const std::shared_ptr<Pedestrian>& pedestrian);
    const std::vector<std::shared_ptr<Pedestrian>>& getPedestrians() const;
    size_t getPedestrianCount() const;

    // Environment properties
    void setTimeOfDay(float hour);  // 0-24 hours
    float getTimeOfDay() const;

    void setWeather(int weatherType); // 0: clear, 1: rain, 2: fog
    int getWeather() const;

private:
    std::shared_ptr<RoadNetwork> roadNetwork_;
    std::vector<std::shared_ptr<TrafficVehicle>> trafficVehicles_;
    std::vector<std::shared_ptr<Pedestrian>> pedestrians_;

    float timeOfDay_ = 12.0f;  // noon
    int weatherType_ = 0;       // clear
};

} // namespace av
