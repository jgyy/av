#pragma once

#include "sensor.hpp"
#include "lidar.hpp"
#include <vector>
#include <memory>
#include <map>
#include <string>

namespace av {

// Forward declarations
class World;

// Manages multiple sensors and coordinates their updates
class SensorManager {
public:
    SensorManager();
    ~SensorManager();

    // Initialization
    void initialize(std::shared_ptr<World> world);
    void shutdown();

    // Sensor management
    void addSensor(std::shared_ptr<Sensor> sensor);
    void removeSensor(const std::string& name);
    std::shared_ptr<Sensor> getSensor(const std::string& name) const;

    // Update loop
    void update(float deltaTime);

    // Convenience methods for common sensors
    std::shared_ptr<LidarSensor> createLidarSensor(const LidarSensor::Config& config = LidarSensor::Config());
    std::shared_ptr<LidarSensor> getLidarSensor(const std::string& name = "lidar") const;

    // Queries
    size_t getSensorCount() const { return sensors_.size(); }
    const std::vector<std::shared_ptr<Sensor>>& getSensors() const { return sensors_; }

    // World access
    std::shared_ptr<World> getWorld() const { return world_; }

private:
    std::vector<std::shared_ptr<Sensor>> sensors_;
    std::map<std::string, std::shared_ptr<Sensor>> sensorMap_;
    std::shared_ptr<World> world_;

    int nextSensorId_ = 0;
};

} // namespace av
