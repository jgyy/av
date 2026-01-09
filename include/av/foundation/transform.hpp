#pragma once

#include "math.hpp"

namespace av {

// 3D Transform with position, rotation (quaternion), and scale
class Transform {
public:
    Transform()
        : position_(Vec3::Zero()),
          rotation_(Quat::Identity()),
          scale_(Vec3::Ones()) {
    }

    explicit Transform(const Vec3& position)
        : position_(position),
          rotation_(Quat::Identity()),
          scale_(Vec3::Ones()) {
    }

    Transform(const Vec3& position, const Quat& rotation)
        : position_(position),
          rotation_(rotation),
          scale_(Vec3::Ones()) {
    }

    Transform(const Vec3& position, const Quat& rotation, const Vec3& scale)
        : position_(position),
          rotation_(rotation),
          scale_(scale) {
    }

    // Getters
    const Vec3& getPosition() const { return position_; }
    const Quat& getRotation() const { return rotation_; }
    const Vec3& getScale() const { return scale_; }

    // Setters
    void setPosition(const Vec3& position) { position_ = position; }
    void setRotation(const Quat& rotation) { rotation_ = rotation; }
    void setScale(const Vec3& scale) { scale_ = scale; }

    // Transform a point
    Vec3 transformPoint(const Vec3& point) const {
        return rotation_ * (scale_.asDiagonal() * point) + position_;
    }

    // Transform a direction (no translation)
    Vec3 transformDirection(const Vec3& direction) const {
        return rotation_ * (scale_.asDiagonal() * direction);
    }

    // Inverse transform a point
    Vec3 inverseTransformPoint(const Vec3& point) const {
        Vec3 translated = point - position_;
        Vec3 rotated = rotation_.conjugate() * translated;
        return (scale_.asDiagonal().inverse() * rotated).eval();
    }

    // Inverse transform a direction
    Vec3 inverseTransformDirection(const Vec3& direction) const {
        Vec3 rotated = rotation_.conjugate() * direction;
        return (scale_.asDiagonal().inverse() * rotated).eval();
    }

    // Get forward direction (local Z axis in world space)
    Vec3 getForward() const {
        return rotation_ * Vec3::UnitZ();
    }

    // Get right direction (local X axis in world space)
    Vec3 getRight() const {
        return rotation_ * Vec3::UnitX();
    }

    // Get up direction (local Y axis in world space)
    Vec3 getUp() const {
        return rotation_ * Vec3::UnitY();
    }

    // Get 4x4 transformation matrix
    Mat4 getMatrix() const {
        Mat4 mat = Mat4::Identity();
        mat.topLeftCorner(3, 3) = rotation_.toRotationMatrix().cast<float>();
        // Apply scale
        mat(0, 0) *= scale_.x();
        mat(1, 0) *= scale_.x();
        mat(2, 0) *= scale_.x();

        mat(0, 1) *= scale_.y();
        mat(1, 1) *= scale_.y();
        mat(2, 1) *= scale_.y();

        mat(0, 2) *= scale_.z();
        mat(1, 2) *= scale_.z();
        mat(2, 2) *= scale_.z();

        // Apply translation
        mat(0, 3) = position_.x();
        mat(1, 3) = position_.y();
        mat(2, 3) = position_.z();

        return mat;
    }

    // Get inverse matrix
    Mat4 getInverseMatrix() const {
        Mat4 mat = Mat4::Identity();

        // Inverse rotation
        Quat invRot = rotation_.conjugate();
        Mat3 rotMat = invRot.toRotationMatrix();

        // Inverse scale
        Vec3 invScale = Vec3(1.0f / scale_.x(), 1.0f / scale_.y(), 1.0f / scale_.z());

        // Construct inverse transform
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                mat(i, j) = rotMat(i, j) * invScale(j);
            }
        }

        // Inverse translation
        Vec3 invTranslation = -rotMat * (invScale.asDiagonal() * position_);
        mat(0, 3) = invTranslation.x();
        mat(1, 3) = invTranslation.y();
        mat(2, 3) = invTranslation.z();

        return mat;
    }

    // Compose two transforms
    Transform operator*(const Transform& other) const {
        Vec3 newPosition = transformPoint(other.position_);
        Quat newRotation = rotation_ * other.rotation_;
        Vec3 newScale = scale_.asDiagonal() * other.scale_;
        return Transform(newPosition, newRotation, newScale);
    }

    // Interpolate between two transforms (simple linear interpolation)
    static Transform lerp(const Transform& a, const Transform& b, float t) {
        t = clamp(t, 0.0f, 1.0f);
        Vec3 position = a.position_ + (b.position_ - a.position_) * t;
        Quat rotation = a.rotation_.slerp(t, b.rotation_);
        Vec3 scale = a.scale_ + (b.scale_ - a.scale_) * t;
        return Transform(position, rotation, scale);
    }

    // Reset to identity
    void setIdentity() {
        position_ = Vec3::Zero();
        rotation_ = Quat::Identity();
        scale_ = Vec3::Ones();
    }

    // Check if identity
    bool isIdentity(float tolerance = EPSILON) const {
        return isNearZero(position_.norm(), tolerance) &&
               isNearEqual(rotation_.norm(), 1.0f, tolerance) &&
               isNearEqual(scale_.x(), 1.0f, tolerance) &&
               isNearEqual(scale_.y(), 1.0f, tolerance) &&
               isNearEqual(scale_.z(), 1.0f, tolerance);
    }

private:
    Vec3 position_;
    Quat rotation_;
    Vec3 scale_;
};

} // namespace av
