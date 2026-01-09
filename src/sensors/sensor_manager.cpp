#include "av/sensors/sensor_manager.hpp"
#include "av/world/world.hpp"
#include "av/foundation/logging.hpp"
#include <sstream>

namespace av {

SensorManager::SensorManager() {
    AV_DEBUG("SensorManager created");
}

SensorManager::~SensorManager() {
    shutdown();
    AV_DEBUG("SensorManager destroyed");
}

void SensorManager::initialize(std::shared_ptr<World> world) {
    world_ = world;
    AV_INFO("SensorManager initialized with world");

    // Initialize all sensors
    for (auto& sensor : sensors_) {
        if (sensor) {
            sensor->setWorld(world);
            sensor->initialize();
        }
    }
}

void SensorManager::shutdown() {
    // Shutdown all sensors
    for (auto& sensor : sensors_) {
        if (sensor) {
            sensor->shutdown();
        }
    }
    sensors_.clear();
    sensorMap_.clear();
    AV_DEBUG("SensorManager shutdown complete");
}

void SensorManager::addSensor(std::shared_ptr<Sensor> sensor) {
    if (!sensor) {
        AV_WARN("Cannot add null sensor");
        return;
    }

    sensors_.push_back(sensor);
    std::string name = sensor->getName();

    // Handle duplicate names with suffix
    if (sensorMap_.find(name) != sensorMap_.end()) {
        std::stringstream ss;
        ss << name << "_" << nextSensorId_++;
        name = ss.str();
    }

    sensorMap_[name] = sensor;

    // Set world if already initialized
    if (world_) {
        sensor->setWorld(world_);
        sensor->initialize();
    }

    AV_DEBUG("Added sensor: {}", name);
}

void SensorManager::removeSensor(const std::string& name) {
    auto it = sensorMap_.find(name);
    if (it == sensorMap_.end()) {
        AV_WARN("Sensor not found: {}", name);
        return;
    }

    auto sensor = it->second;
    sensor->shutdown();

    // Remove from vector
    auto vec_it = std::find(sensors_.begin(), sensors_.end(), sensor);
    if (vec_it != sensors_.end()) {
        sensors_.erase(vec_it);
    }

    sensorMap_.erase(it);
    AV_DEBUG("Removed sensor: {}", name);
}

std::shared_ptr<Sensor> SensorManager::getSensor(const std::string& name) const {
    auto it = sensorMap_.find(name);
    if (it != sensorMap_.end()) {
        return it->second;
    }
    return nullptr;
}

void SensorManager::update(float deltaTime) {
    if (!world_) {
        return;
    }

    // Update all sensors
    for (auto& sensor : sensors_) {
        if (sensor) {
            sensor->update(deltaTime);
        }
    }
}

std::shared_ptr<LidarSensor> SensorManager::createLidarSensor(const LidarSensor::Config& config) {
    auto lidar = std::make_shared<LidarSensor>(config);
    std::string name = "lidar";

    // Check if lidar sensor already exists
    if (sensorMap_.find(name) != sensorMap_.end()) {
        std::stringstream ss;
        ss << name << "_" << sensors_.size();
        name = ss.str();
    }

    addSensor(lidar);
    sensorMap_[name] = lidar;

    AV_INFO("Created LIDAR sensor: {}", name);
    return lidar;
}

std::shared_ptr<LidarSensor> SensorManager::getLidarSensor(const std::string& name) const {
    auto sensor = getSensor(name);
    if (!sensor) {
        return nullptr;
    }

    // Dynamic cast to LidarSensor
    return std::dynamic_pointer_cast<LidarSensor>(sensor);
}

} // namespace av

