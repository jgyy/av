#pragma once
#include "av/foundation/math.hpp"
#include "av/foundation/transform.hpp"
namespace av { class Camera { public: Vec3 getPosition() const; void setPosition(const Vec3& pos); Mat4 getViewMatrix() const; Mat4 getProjectionMatrix() const; private: Transform transform_; }; } // namespace av
