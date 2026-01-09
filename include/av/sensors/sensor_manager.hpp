#pragma once
#include "sensor.hpp"
#include <vector>
#include <memory>
namespace av { class SensorManager { public: void update(float deltaTime); void addSensor(std::shared_ptr<Sensor> sensor); private: std::vector<std::shared_ptr<Sensor>> sensors_; }; } // namespace av
