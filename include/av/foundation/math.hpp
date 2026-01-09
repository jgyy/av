#pragma once

#include <cmath>
#include <algorithm>
#include <limits>
#include <Eigen/Dense>

namespace av {

// Constants
constexpr float PI = 3.14159265359f;
constexpr float TWO_PI = 2.0f * PI;
constexpr float HALF_PI = PI / 2.0f;
constexpr float DEG_TO_RAD = PI / 180.0f;
constexpr float RAD_TO_DEG = 180.0f / PI;
constexpr float EPSILON = std::numeric_limits<float>::epsilon();

// Vector types using Eigen
using Vec2 = Eigen::Vector2f;
using Vec3 = Eigen::Vector3f;
using Vec4 = Eigen::Vector4f;
using DVec2 = Eigen::Vector2d;
using DVec3 = Eigen::Vector3d;
using DVec4 = Eigen::Vector4d;

// Matrix types using Eigen
using Mat2 = Eigen::Matrix2f;
using Mat3 = Eigen::Matrix3f;
using Mat4 = Eigen::Matrix4f;
using DMat2 = Eigen::Matrix2d;
using DMat3 = Eigen::Matrix3d;
using DMat4 = Eigen::Matrix4d;

// Quaternion type using Eigen
using Quat = Eigen::Quaternionf;
using DQuat = Eigen::Quaterniond;

// Utility functions
inline float toRadians(float degrees) {
    return degrees * DEG_TO_RAD;
}

inline float toDegrees(float radians) {
    return radians * RAD_TO_DEG;
}

inline float clamp(float value, float min, float max) {
    return std::max(min, std::min(max, value));
}

inline float lerp(float a, float b, float t) {
    return a + (b - a) * clamp(t, 0.0f, 1.0f);
}

inline bool isNearZero(float value, float tolerance = EPSILON) {
    return std::abs(value) < tolerance;
}

inline bool isNearEqual(float a, float b, float tolerance = EPSILON) {
    return std::abs(a - b) < tolerance;
}

// Vector utilities
inline float distance(const Vec3& a, const Vec3& b) {
    return (a - b).norm();
}

inline float distanceSquared(const Vec3& a, const Vec3& b) {
    return (a - b).squaredNorm();
}

inline Vec3 normalize(const Vec3& v) {
    float len = v.norm();
    if (len > EPSILON) {
        return v / len;
    }
    return v;
}

inline float dot(const Vec3& a, const Vec3& b) {
    return a.dot(b);
}

inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return a.cross(b);
}

// Quaternion utilities
inline Quat angleAxis(float angle, const Vec3& axis) {
    return Quat(Eigen::AngleAxisf(angle, normalize(axis)));
}

inline Quat quatFromEuler(float roll, float pitch, float yaw) {
    return Quat(Eigen::AngleAxisf(yaw, Vec3::UnitZ()) *
                Eigen::AngleAxisf(pitch, Vec3::UnitY()) *
                Eigen::AngleAxisf(roll, Vec3::UnitX()));
}

inline Vec3 quatToEuler(const Quat& q) {
    auto angles = q.toRotationMatrix().eulerAngles(2, 1, 0);
    return Vec3(angles(2), angles(1), angles(0));
}

// Matrix utilities
inline Mat4 perspective(float fovy, float aspect, float near, float far) {
    float f = 1.0f / std::tan(fovy / 2.0f);
    Mat4 mat = Mat4::Zero();
    mat(0, 0) = f / aspect;
    mat(1, 1) = f;
    mat(2, 2) = (far + near) / (near - far);
    mat(2, 3) = (2 * far * near) / (near - far);
    mat(3, 2) = -1.0f;
    return mat;
}

inline Mat4 orthogonal(float left, float right, float bottom, float top,
                       float near, float far) {
    Mat4 mat = Mat4::Identity();
    mat(0, 0) = 2.0f / (right - left);
    mat(1, 1) = 2.0f / (top - bottom);
    mat(2, 2) = -2.0f / (far - near);
    mat(0, 3) = -(right + left) / (right - left);
    mat(1, 3) = -(top + bottom) / (top - bottom);
    mat(2, 3) = -(far + near) / (far - near);
    return mat;
}

inline Mat4 lookAt(const Vec3& eye, const Vec3& center, const Vec3& up) {
    Vec3 f = normalize(center - eye);
    Vec3 s = normalize(cross(f, up));
    Vec3 u = cross(s, f);

    Mat4 mat = Mat4::Identity();
    mat(0, 0) = s.x();   mat(0, 1) = s.y();   mat(0, 2) = s.z();
    mat(1, 0) = u.x();   mat(1, 1) = u.y();   mat(1, 2) = u.z();
    mat(2, 0) = -f.x();  mat(2, 1) = -f.y();  mat(2, 2) = -f.z();

    mat(0, 3) = -dot(s, eye);
    mat(1, 3) = -dot(u, eye);
    mat(2, 3) = dot(f, eye);

    return mat;
}

inline Mat4 translate(const Mat4& mat, const Vec3& translation) {
    Mat4 result = mat;
    result.col(3).head(3) += mat.block<3, 3>(0, 0) * translation;
    return result;
}

inline Mat4 scale(const Mat4& mat, const Vec3& scale) {
    Mat4 result = mat;
    result.col(0).head(3) *= scale.x();
    result.col(1).head(3) *= scale.y();
    result.col(2).head(3) *= scale.z();
    return result;
}

inline Mat4 rotate(const Mat4& mat, const Quat& rotation) {
    Mat4 rotMat = Mat4::Identity();
    rotMat.topLeftCorner(3, 3) = rotation.toRotationMatrix().cast<float>();
    return mat * rotMat;
}

} // namespace av
