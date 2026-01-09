#pragma once

#include "av/foundation/math.hpp"
#include "av/foundation/transform.hpp"

namespace av {

// Base camera class
class Camera {
public:
    Camera(const Vec3& position = Vec3::Zero(), const Vec3& target = Vec3::UnitZ(),
           float fov = 60.0f, float aspect = 1.777f, float nearPlane = 0.1f, float farPlane = 1000.0f);
    virtual ~Camera() = default;

    // Position and direction
    void setPosition(const Vec3& position);
    void setTarget(const Vec3& target);
    Vec3 getPosition() const { return position_; }
    Vec3 getTarget() const { return target_; }
    Vec3 getViewDirection() const;

    // Projection parameters
    void setFOV(float fov);
    void setAspectRatio(float aspect);
    float getFOV() const { return fov_; }

    // Matrices
    Mat4 getViewMatrix() const;
    Mat4 getProjectionMatrix() const;
    Mat4 getViewProjectionMatrix() const;

    // Virtual update for derived classes
    virtual void update(float deltaTime) {}

protected:
    Vec3 position_;
    Vec3 target_;
    float fov_;
    float aspect_;
    float nearPlane_;
    float farPlane_;

    Mat4 viewMatrix_;
    Mat4 projectionMatrix_;
    mutable bool viewDirty_ = true;
    mutable bool projDirty_ = true;

    void updateViewMatrix();
    void updateProjectionMatrix();
};

// Orbital camera - rotates around a point
class OrbitCamera : public Camera {
public:
    OrbitCamera(const Vec3& center = Vec3::Zero(), float distance = 10.0f,
                float fov = 60.0f, float aspect = 1.777f);

    void update(float deltaTime) override;

    void rotate(float deltaYaw, float deltaPitch);
    void zoom(float amount);
    void setCenter(const Vec3& center);

    Vec3 getCenter() const { return center_; }
    float getDistance() const { return distance_; }

private:
    Vec3 center_;
    float distance_;
    float yaw_;
    float pitch_;
};

// Follow camera - tracks a target with smooth motion
class FollowCamera : public Camera {
public:
    FollowCamera(float fov = 60.0f, float aspect = 1.777f);

    void update(float deltaTime) override;

    void setTarget(const Transform& transform);
    void setFollowDistance(float distance);
    void setFollowHeight(float height);

private:
    const Transform* targetTransform_ = nullptr;
    float followDistance_;
    float followHeight_;
    float targetLookAhead_;
};

// Free camera - can move freely in space
class FreeCamera : public Camera {
public:
    FreeCamera(const Vec3& position = Vec3::Zero(), float fov = 60.0f, float aspect = 1.777f);

    void update(float deltaTime) override;

    void move(const Vec3& direction);
    void rotate(float deltaYaw, float deltaPitch);
    void setMoveSpeed(float speed);
    void setRotateSpeed(float speed);

private:
    float yaw_;
    float pitch_;
    float moveSpeed_;
    float rotateSpeed_;
};

} // namespace av
