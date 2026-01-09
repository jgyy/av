#pragma once

#include "av/foundation/math.hpp"
#include <vector>
#include <memory>

namespace av {

// Forward declarations
class RoadNetwork;
class TrafficVehicle;
class Pedestrian;

// Main world object
class World {
public:
    World();
    ~World();

    // Initialize world
    void initialize();

    // Update world
    void update(float deltaTime);

    // Create and manage entities
    std::shared_ptr<RoadNetwork> getRoadNetwork();

    // Traffic management
    std::shared_ptr<TrafficVehicle> createTrafficVehicle();
    void removeTrafficVehicle(const std::shared_ptr<TrafficVehicle>& vehicle);

    // Pedestrian management
    std::shared_ptr<Pedestrian> createPedestrian();
    void removePedestrian(const std::shared_ptr<Pedestrian>& pedestrian);

    // Environment properties
    void setTimeOfDay(float hour);  // 0-24
    void setWeather(int weatherType); // 0: clear, 1: rain, 2: fog

private:
    std::shared_ptr<RoadNetwork> roadNetwork_;
    std::vector<std::shared_ptr<TrafficVehicle>> trafficVehicles_;
    std::vector<std::shared_ptr<Pedestrian>> pedestrians_;
};

// Road network representation
class RoadNetwork {
public:
    RoadNetwork();
    ~RoadNetwork();

    // Load road network from file
    void loadFromFile(const std::string& filePath);

    // Road network queries
    Vec3 getClosestLanePoint(const Vec3& position);

private:
    // Internal representation
};

// Traffic vehicle (AI-controlled)
class TrafficVehicle {
public:
    TrafficVehicle();
    ~TrafficVehicle();

    void update(float deltaTime);
    void setPosition(const Vec3& position);
    Vec3 getPosition() const;

private:
    Vec3 position_ = Vec3::Zero();
};

// Pedestrian (AI-controlled)
class Pedestrian {
public:
    Pedestrian();
    ~Pedestrian();

    void update(float deltaTime);
    void setPosition(const Vec3& position);
    Vec3 getPosition() const;

private:
    Vec3 position_ = Vec3::Zero();
};

} // namespace av
