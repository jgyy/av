#include <gtest/gtest.h>
#include "av/foundation/geometry.hpp"

using namespace av;

TEST(GeometryTest, AABBContains) {
    AABB box(Vec3(0.0f, 0.0f, 0.0f), Vec3(10.0f, 10.0f, 10.0f));

    EXPECT_TRUE(box.contains(Vec3(5.0f, 5.0f, 5.0f)));
    EXPECT_FALSE(box.contains(Vec3(15.0f, 15.0f, 15.0f)));
}

TEST(GeometryTest, AABBIntersects) {
    AABB box1(Vec3(0.0f, 0.0f, 0.0f), Vec3(10.0f, 10.0f, 10.0f));
    AABB box2(Vec3(5.0f, 5.0f, 5.0f), Vec3(15.0f, 15.0f, 15.0f));
    AABB box3(Vec3(20.0f, 20.0f, 20.0f), Vec3(30.0f, 30.0f, 30.0f));

    EXPECT_TRUE(box1.intersects(box2));
    EXPECT_FALSE(box1.intersects(box3));
}

TEST(GeometryTest, SphereContains) {
    Sphere sphere(Vec3(0.0f, 0.0f, 0.0f), 5.0f);

    EXPECT_TRUE(sphere.contains(Vec3(0.0f, 0.0f, 0.0f)));
    EXPECT_TRUE(sphere.contains(Vec3(3.0f, 4.0f, 0.0f))); // Distance = 5.0
    EXPECT_FALSE(sphere.contains(Vec3(10.0f, 0.0f, 0.0f)));
}

TEST(GeometryTest, RayAABBIntersection) {
    Ray ray(Vec3(-5.0f, 5.0f, 5.0f), Vec3(1.0f, 0.0f, 0.0f));
    AABB box(Vec3(0.0f, 0.0f, 0.0f), Vec3(10.0f, 10.0f, 10.0f));

    float tMin, tMax;
    EXPECT_TRUE(box.rayIntersect(ray, tMin, tMax));
    EXPECT_FLOAT_EQ(tMin, 5.0f);
}

TEST(GeometryTest, PlaneDistance) {
    Plane plane(Vec3::UnitZ(), Vec3(0.0f, 0.0f, 5.0f));

    EXPECT_FLOAT_EQ(plane.signedDistance(Vec3(0.0f, 0.0f, 10.0f)), 5.0f);
    EXPECT_FLOAT_EQ(plane.signedDistance(Vec3(0.0f, 0.0f, 0.0f)), -5.0f);
}
