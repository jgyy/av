#pragma once

#include "math.hpp"

namespace av {

// Ray representation
struct Ray {
    Vec3 origin;
    Vec3 direction;

    Ray() : origin(Vec3::Zero()), direction(Vec3::UnitZ()) {}
    Ray(const Vec3& o, const Vec3& d) : origin(o), direction(normalize(d)) {}

    Vec3 pointAt(float t) const {
        return origin + direction * t;
    }
};

// Axis-aligned bounding box
class AABB {
public:
    AABB()
        : min_(Vec3::Zero()),
          max_(Vec3::Zero()) {
    }

    AABB(const Vec3& min, const Vec3& max)
        : min_(min), max_(max) {
        normalize();
    }

    void normalize() {
        for (int i = 0; i < 3; i++) {
            if (min_(i) > max_(i)) {
                std::swap(min_(i), max_(i));
            }
        }
    }

    const Vec3& getMin() const { return min_; }
    const Vec3& getMax() const { return max_; }
    Vec3 getCenter() const { return (min_ + max_) * 0.5f; }
    Vec3 getSize() const { return max_ - min_; }
    float getVolume() const {
        Vec3 size = getSize();
        return size.x() * size.y() * size.z();
    }

    bool contains(const Vec3& point) const {
        return (point.x() >= min_.x() && point.x() <= max_.x()) &&
               (point.y() >= min_.y() && point.y() <= max_.y()) &&
               (point.z() >= min_.z() && point.z() <= max_.z());
    }

    bool intersects(const AABB& other) const {
        return !(max_.x() < other.min_.x() || min_.x() > other.max_.x()) &&
               !(max_.y() < other.min_.y() || min_.y() > other.max_.y()) &&
               !(max_.z() < other.min_.z() || min_.z() > other.max_.z());
    }

    bool rayIntersect(const Ray& ray, float& tMin, float& tMax) const {
        tMin = -std::numeric_limits<float>::max();
        tMax = std::numeric_limits<float>::max();

        for (int i = 0; i < 3; i++) {
            if (std::abs(ray.direction(i)) < EPSILON) {
                if (ray.origin(i) < min_(i) || ray.origin(i) > max_(i)) {
                    return false;
                }
            } else {
                float t1 = (min_(i) - ray.origin(i)) / ray.direction(i);
                float t2 = (max_(i) - ray.origin(i)) / ray.direction(i);
                if (t1 > t2) std::swap(t1, t2);

                tMin = std::max(tMin, t1);
                tMax = std::min(tMax, t2);

                if (tMin > tMax) return false;
            }
        }
        return true;
    }

    void expand(const Vec3& point) {
        min_ = min_.cwiseMin(point);
        max_ = max_.cwiseMax(point);
    }

    void expand(const AABB& other) {
        min_ = min_.cwiseMin(other.min_);
        max_ = max_.cwiseMax(other.max_);
    }

    static AABB fromPoints(const std::vector<Vec3>& points) {
        if (points.empty()) {
            return AABB();
        }
        AABB result(points[0], points[0]);
        for (size_t i = 1; i < points.size(); i++) {
            result.expand(points[i]);
        }
        return result;
    }

private:
    Vec3 min_;
    Vec3 max_;
};

// Oriented bounding box
class OBB {
public:
    OBB()
        : center_(Vec3::Zero()),
          halfExtents_(Vec3::Ones()),
          rotation_(Quat::Identity()) {
    }

    OBB(const Vec3& center, const Vec3& halfExtents, const Quat& rotation)
        : center_(center),
          halfExtents_(halfExtents),
          rotation_(rotation) {
    }

    const Vec3& getCenter() const { return center_; }
    const Vec3& getHalfExtents() const { return halfExtents_; }
    const Quat& getRotation() const { return rotation_; }

    Vec3 getAxis(int index) const {
        if (index == 0) return rotation_ * Vec3::UnitX();
        if (index == 1) return rotation_ * Vec3::UnitY();
        return rotation_ * Vec3::UnitZ();
    }

    bool contains(const Vec3& point) const {
        Vec3 local = rotation_.conjugate() * (point - center_);
        return (std::abs(local.x()) <= halfExtents_.x()) &&
               (std::abs(local.y()) <= halfExtents_.y()) &&
               (std::abs(local.z()) <= halfExtents_.z());
    }

    bool intersects(const OBB& other) const {
        // Simplified SAT (Separating Axis Theorem) check
        float distance = (other.center_ - center_).norm();
        float combinedRadius = (halfExtents_ + other.halfExtents_).norm();
        return distance <= combinedRadius;
    }

    AABB getAABB() const {
        Mat3 axes;
        axes.col(0) = getAxis(0);
        axes.col(1) = getAxis(1);
        axes.col(2) = getAxis(2);

        Vec3 extents = Vec3::Zero();
        for (int i = 0; i < 3; i++) {
            extents(i) = halfExtents_.x() * std::abs(axes(i, 0)) +
                         halfExtents_.y() * std::abs(axes(i, 1)) +
                         halfExtents_.z() * std::abs(axes(i, 2));
        }

        return AABB(center_ - extents, center_ + extents);
    }

private:
    Vec3 center_;
    Vec3 halfExtents_;
    Quat rotation_;
};

// Sphere
struct Sphere {
    Vec3 center;
    float radius;

    Sphere() : center(Vec3::Zero()), radius(1.0f) {}
    Sphere(const Vec3& c, float r) : center(c), radius(r) {}

    bool contains(const Vec3& point) const {
        return (point - center).norm() <= radius;
    }

    bool intersects(const Sphere& other) const {
        return (center - other.center).norm() <= (radius + other.radius);
    }

    bool intersects(const AABB& aabb) const {
        Vec3 closest = clamp(center.x(), aabb.getMin().x(), aabb.getMax().x()),
                        clamp(center.y(), aabb.getMin().y(), aabb.getMax().y()),
                        clamp(center.z(), aabb.getMin().z(), aabb.getMax().z());
        return (closest - center).norm() <= radius;
    }
};

// Plane
struct Plane {
    Vec3 normal;
    float distance;

    Plane() : normal(Vec3::UnitZ()), distance(0.0f) {}
    Plane(const Vec3& n, float d) : normal(normalize(n)), distance(d) {}
    Plane(const Vec3& n, const Vec3& point)
        : normal(normalize(n)), distance(dot(n, point)) {}

    float signedDistance(const Vec3& point) const {
        return dot(normal, point) - distance;
    }

    Vec3 project(const Vec3& point) const {
        return point - normal * signedDistance(point);
    }

    bool rayIntersect(const Ray& ray, float& t) const {
        float denom = dot(normal, ray.direction);
        if (std::abs(denom) < EPSILON) {
            return false; // Ray is parallel to plane
        }
        t = (distance - dot(normal, ray.origin)) / denom;
        return t >= 0.0f;
    }
};

// Frustum (camera viewing frustum)
class Frustum {
public:
    enum PlaneIndex {
        NEAR = 0, FAR = 1,
        LEFT = 2, RIGHT = 3,
        TOP = 4, BOTTOM = 5
    };

    Frustum() = default;

    static Frustum fromMatrix(const Mat4& viewProj) {
        Frustum f;

        // Extract planes from view-projection matrix
        for (int i = 0; i < 3; i++) {
            // Right plane
            f.planes_[RIGHT].normal(i) = viewProj(i, 3) - viewProj(i, 0);
            // Left plane
            f.planes_[LEFT].normal(i) = viewProj(i, 3) + viewProj(i, 0);
            // Top plane
            f.planes_[TOP].normal(i) = viewProj(i, 3) - viewProj(i, 1);
            // Bottom plane
            f.planes_[BOTTOM].normal(i) = viewProj(i, 3) + viewProj(i, 1);
            // Far plane
            f.planes_[FAR].normal(i) = viewProj(i, 3) - viewProj(i, 2);
            // Near plane
            f.planes_[NEAR].normal(i) = viewProj(i, 3) + viewProj(i, 2);
        }

        for (int i = 0; i < 6; i++) {
            float length = f.planes_[i].normal.norm();
            f.planes_[i].normal /= length;
            f.planes_[i].distance /= length;
        }

        return f;
    }

    bool contains(const Vec3& point) const {
        for (int i = 0; i < 6; i++) {
            if (planes_[i].signedDistance(point) < 0.0f) {
                return false;
            }
        }
        return true;
    }

    bool intersects(const Sphere& sphere) const {
        for (int i = 0; i < 6; i++) {
            if (planes_[i].signedDistance(sphere.center) < -sphere.radius) {
                return false;
            }
        }
        return true;
    }

    bool intersects(const AABB& aabb) const {
        for (int i = 0; i < 6; i++) {
            Vec3 positive;
            if (planes_[i].normal.x() > 0) positive.x() = aabb.getMax().x();
            else positive.x() = aabb.getMin().x();
            if (planes_[i].normal.y() > 0) positive.y() = aabb.getMax().y();
            else positive.y() = aabb.getMin().y();
            if (planes_[i].normal.z() > 0) positive.z() = aabb.getMax().z();
            else positive.z() = aabb.getMin().z();

            if (planes_[i].signedDistance(positive) < 0.0f) {
                return false;
            }
        }
        return true;
    }

private:
    Plane planes_[6];
};

} // namespace av
