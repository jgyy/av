#include <gtest/gtest.h>
#include "av/foundation/transform.hpp"

using namespace av;

TEST(TransformTest, Identity) {
    Transform t;
    EXPECT_TRUE(t.isIdentity());
    EXPECT_EQ(t.getPosition(), Vec3::Zero());
    EXPECT_EQ(t.getScale(), Vec3::Ones());
}

TEST(TransformTest, TransformPoint) {
    Transform t(Vec3(1.0f, 2.0f, 3.0f));
    Vec3 point(0.0f, 0.0f, 0.0f);
    Vec3 result = t.transformPoint(point);
    EXPECT_EQ(result, Vec3(1.0f, 2.0f, 3.0f));
}

TEST(TransformTest, Composition) {
    Transform t1(Vec3(1.0f, 0.0f, 0.0f));
    Transform t2(Vec3(2.0f, 0.0f, 0.0f));
    Transform result = t1 * t2;
    EXPECT_EQ(result.getPosition(), Vec3(3.0f, 0.0f, 0.0f));
}

TEST(TransformTest, Lerp) {
    Transform t1(Vec3(0.0f, 0.0f, 0.0f));
    Transform t2(Vec3(10.0f, 0.0f, 0.0f));
    Transform result = Transform::lerp(t1, t2, 0.5f);
    EXPECT_NEAR(result.getPosition().x(), 5.0f, EPSILON);
}
