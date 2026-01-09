#pragma once
#include "sensor.hpp"
namespace av { class IMUSensor : public Sensor { public: IMUSensor(); void update(float deltaTime) override; std::string getName() const override { return "IMU"; } }; } // namespace av
