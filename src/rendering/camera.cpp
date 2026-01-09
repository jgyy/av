#include "av/rendering/camera.hpp"
#include "av/foundation/logging.hpp"
#include <cmath>

namespace av {

Camera::Camera(const Vec3& position, const Vec3& target, float fov, float aspect, float nearPlane, float farPlane)
    : position_(position), target_(target), fov_(fov), aspect_(aspect),
      nearPlane_(nearPlane), farPlane_(farPlane) {
    updateViewMatrix();
    updateProjectionMatrix();
    AV_DEBUG("Camera created at position (%.2f, %.2f, %.2f)", position.x(), position.y(), position.z());
}

void Camera::setPosition(const Vec3& position) {
    position_ = position;
    viewDirty_ = true;
}

void Camera::setTarget(const Vec3& target) {
    target_ = target;
    viewDirty_ = true;
}

void Camera::setFOV(float fov) {
    fov_ = clamp(fov, 10.0f, 120.0f);
    projDirty_ = true;
}

void Camera::setAspectRatio(float aspect) {
    aspect_ = aspect;
    projDirty_ = true;
}

Vec3 Camera::getViewDirection() const {
    return normalize(target_ - position_);
}

Mat4 Camera::getViewMatrix() const {
    if (viewDirty_) {
        const_cast<Camera*>(this)->updateViewMatrix();
    }
    return viewMatrix_;
}

Mat4 Camera::getProjectionMatrix() const {
    if (projDirty_) {
        const_cast<Camera*>(this)->updateProjectionMatrix();
    }
    return projectionMatrix_;
}

Mat4 Camera::getViewProjectionMatrix() const {
    return getProjectionMatrix() * getViewMatrix();
}

void Camera::updateViewMatrix() {
    viewMatrix_ = lookAt(position_, target_, Vec3::UnitY());
    viewDirty_ = false;
}

void Camera::updateProjectionMatrix() {
    projectionMatrix_ = perspective(toRadians(fov_), aspect_, nearPlane_, farPlane_);
    projDirty_ = false;
}

// OrbitCamera implementation

OrbitCamera::OrbitCamera(const Vec3& center, float distance, float fov, float aspect)
    : Camera(center + Vec3(distance, distance, distance), center, fov, aspect),
      center_(center), distance_(distance), yaw_(0.0f), pitch_(0.0f) {
    AV_DEBUG("OrbitCamera created");
}

void OrbitCamera::update(float deltaTime) {
    // Update position based on yaw and pitch
    float cosYaw = std::cos(yaw_);
    float sinYaw = std::sin(yaw_);
    float cosPitch = std::cos(pitch_);
    float sinPitch = std::sin(pitch_);

    Vec3 offset(
        distance_ * cosPitch * sinYaw,
        distance_ * sinPitch,
        distance_ * cosPitch * cosYaw
    );

    setPosition(center_ + offset);
    setTarget(center_);
}

void OrbitCamera::rotate(float deltaYaw, float deltaPitch) {
    yaw_ += deltaYaw;
    pitch_ = clamp(pitch_ + deltaPitch, -PI / 2.0f + 0.1f, PI / 2.0f - 0.1f);
}

void OrbitCamera::zoom(float amount) {
    distance_ = clamp(distance_ + amount, 2.0f, 200.0f);
    update(0.0f);
}

void OrbitCamera::setCenter(const Vec3& center) {
    center_ = center;
    update(0.0f);
}

// FollowCamera implementation

FollowCamera::FollowCamera(float fov, float aspect)
    : Camera(Vec3::Zero(), Vec3::Zero(), fov, aspect),
      followDistance_(5.0f), followHeight_(2.0f), targetLookAhead_(2.0f) {
    AV_DEBUG("FollowCamera created");
}

void FollowCamera::update(float deltaTime) {
    if (!targetTransform_) {
        return;
    }

    // Get target position and direction
    Vec3 targetPos = targetTransform_->getPosition();
    Vec3 targetDir = targetTransform_->getRotation() * Vec3::UnitZ();

    // Calculate follow position (behind and above the target)
    Vec3 followPos = targetPos - targetDir * followDistance_ + Vec3::UnitY() * followHeight_;

    // Calculate look-ahead point
    Vec3 lookAheadPos = targetPos + targetDir * targetLookAhead_ + Vec3::UnitY() * 0.5f;

    // Smooth camera movement
    float lerpFactor = 1.0f - std::exp(-5.0f * deltaTime);
    Vec3 newPos = getPosition() + (followPos - getPosition()) * lerpFactor;

    Camera::setPosition(newPos);
    Camera::setTarget(lookAheadPos);
}

void FollowCamera::setTarget(const Transform& transform) {
    targetTransform_ = &transform;
}

void FollowCamera::setFollowDistance(float distance) {
    followDistance_ = clamp(distance, 0.5f, 20.0f);
}

void FollowCamera::setFollowHeight(float height) {
    followHeight_ = clamp(height, 0.0f, 10.0f);
}

// FreeCamera implementation

FreeCamera::FreeCamera(const Vec3& position, float fov, float aspect)
    : Camera(position, position + Vec3::UnitZ(), fov, aspect),
      yaw_(0.0f), pitch_(0.0f), moveSpeed_(10.0f), rotateSpeed_(2.0f) {
    AV_DEBUG("FreeCamera created");
}

void FreeCamera::update(float deltaTime) {
    // Update direction from yaw and pitch
    Vec3 direction(
        std::cos(pitch_) * std::sin(yaw_),
        std::sin(pitch_),
        std::cos(pitch_) * std::cos(yaw_)
    );

    setTarget(getPosition() + direction);
}

void FreeCamera::move(const Vec3& direction) {
    Vec3 moveDir = normalize(direction);
    setPosition(getPosition() + moveDir * moveSpeed_);
}

void FreeCamera::rotate(float deltaYaw, float deltaPitch) {
    yaw_ += deltaYaw * rotateSpeed_;
    pitch_ = clamp(pitch_ + deltaPitch * rotateSpeed_, -PI / 2.0f + 0.1f, PI / 2.0f - 0.1f);
}

void FreeCamera::setMoveSpeed(float speed) {
    moveSpeed_ = clamp(speed, 0.1f, 100.0f);
}

void FreeCamera::setRotateSpeed(float speed) {
    rotateSpeed_ = clamp(speed, 0.1f, 10.0f);
}

} // namespace av
