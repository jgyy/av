#include <gtest/gtest.h>
#include "av/foundation/math.hpp"

using namespace av;

TEST(MathTest, Vector3Operations) {
    Vec3 v1(1.0f, 2.0f, 3.0f);
    Vec3 v2(4.0f, 5.0f, 6.0f);

    // Addition
    Vec3 sum = v1 + v2;
    EXPECT_FLOAT_EQ(sum.x(), 5.0f);
    EXPECT_FLOAT_EQ(sum.y(), 7.0f);
    EXPECT_FLOAT_EQ(sum.z(), 9.0f);

    // Dot product
    float dotProduct = dot(v1, v2);
    EXPECT_FLOAT_EQ(dotProduct, 32.0f);

    // Cross product
    Vec3 crossProduct = cross(v1, v2);
    EXPECT_FLOAT_EQ(crossProduct.x(), -3.0f);
    EXPECT_FLOAT_EQ(crossProduct.y(), 6.0f);
    EXPECT_FLOAT_EQ(crossProduct.z(), -3.0f);

    // Distance
    float dist = distance(v1, v2);
    EXPECT_NEAR(dist, std::sqrt(27.0f), EPSILON);

    // Normalize
    Vec3 normalized = normalize(v1);
    EXPECT_NEAR(normalized.norm(), 1.0f, EPSILON);
}

TEST(MathTest, Conversions) {
    EXPECT_NEAR(toRadians(90.0f), PI / 2.0f, EPSILON);
    EXPECT_NEAR(toDegrees(PI / 2.0f), 90.0f, EPSILON);
}

TEST(MathTest, Clamp) {
    EXPECT_EQ(clamp(5.0f, 0.0f, 10.0f), 5.0f);
    EXPECT_EQ(clamp(-5.0f, 0.0f, 10.0f), 0.0f);
    EXPECT_EQ(clamp(15.0f, 0.0f, 10.0f), 10.0f);
}

TEST(MathTest, Lerp) {
    float result = lerp(0.0f, 10.0f, 0.5f);
    EXPECT_FLOAT_EQ(result, 5.0f);

    result = lerp(0.0f, 10.0f, 1.5f); // Clamped to 1.0
    EXPECT_FLOAT_EQ(result, 10.0f);
}
