#pragma once

#include "sensor.hpp"

namespace av {

class LidarSensor : public Sensor {
public:
    LidarSensor();
    void update(float deltaTime) override;
    std::string getName() const override { return "LIDAR"; }
};

} // namespace av
