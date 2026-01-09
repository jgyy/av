#pragma once
#include "sensor.hpp"
namespace av { class GPSSensor : public Sensor { public: GPSSensor(); void update(float deltaTime) override; std::string getName() const override { return "GPS"; } }; } // namespace av
