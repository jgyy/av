#pragma once

#include "sensor.hpp"
#include "lidar.hpp"
#include "camera.hpp"
#include "radar.hpp"
#include "gps.hpp"
#include "imu.hpp"
#include "ultrasonic.hpp"
#include "odometry.hpp"
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

    // Update loop (handles multi-rate sensor updates)
    void update(float deltaTime);

    // Convenience methods for common sensors
    std::shared_ptr<LidarSensor> createLidarSensor(const LidarSensor::Config& config = LidarSensor::Config());
    std::shared_ptr<LidarSensor> getLidarSensor(const std::string& name = "lidar") const;

    std::shared_ptr<CameraSensor> createCameraSensor(const CameraSensor::Config& config = CameraSensor::Config());
    std::shared_ptr<CameraSensor> getCameraSensor(const std::string& name = "camera") const;

    std::shared_ptr<RadarSensor> createRadarSensor(const RadarSensor::Config& config = RadarSensor::Config());
    std::shared_ptr<RadarSensor> getRadarSensor(const std::string& name = "radar") const;

    std::shared_ptr<GPSSensor> createGPSSensor(const GPSSensor::Config& config = GPSSensor::Config());
    std::shared_ptr<GPSSensor> getGPSSensor(const std::string& name = "gps") const;

    std::shared_ptr<IMUSensor> createIMUSensor(const IMUSensor::Config& config = IMUSensor::Config());
    std::shared_ptr<IMUSensor> getIMUSensor(const std::string& name = "imu") const;

    std::shared_ptr<UltrasonicSensor> createUltrasonicSensor(const UltrasonicSensor::Config& config = UltrasonicSensor::Config());
    std::shared_ptr<UltrasonicSensor> getUltrasonicSensor(const std::string& name = "ultrasonic") const;

    std::shared_ptr<OdometrySensor> createOdometrySensor(const OdometrySensor::Config& config = OdometrySensor::Config());
    std::shared_ptr<OdometrySensor> getOdometrySensor(const std::string& name = "odometry") const;

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

    // Helper for creating typed sensors
    template<typename T>
    std::shared_ptr<T> createSensor(const typename T::Config& config, const std::string& baseName);
};

} // namespace av
