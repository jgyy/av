#pragma once
#include "av/foundation/math.hpp"
namespace av { class PIDController { public: float compute(float error, float dt); private: float kp_ = 1.0f, ki_ = 0.0f, kd_ = 0.0f; float integral_ = 0.0f; }; } // namespace av
