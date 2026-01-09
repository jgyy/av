#pragma once
#include "sensor.hpp"
namespace av { class CameraSensor : public Sensor { public: CameraSensor(); void update(float deltaTime) override; std::string getName() const override { return "Camera"; } }; } // namespace av
