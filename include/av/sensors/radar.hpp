#pragma once
#include "sensor.hpp"
namespace av { class RadarSensor : public Sensor { public: RadarSensor(); void update(float deltaTime) override; std::string getName() const override { return "Radar"; } }; } // namespace av
