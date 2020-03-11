#pragma once

#include <gtest/gtest.h>

#include "util/Math/Vector.hpp"

namespace NAV
{
TEST(VectorTest, Constructors)
{
    Vector<2, float> vec2(2);
    EXPECT_EQ(vec2.c[0], 2);
    EXPECT_EQ(vec2.c[1], 2);

    Vector<3, float> vec3(3);
    EXPECT_EQ(vec3.c[0], 3);
    EXPECT_EQ(vec3.c[1], 3);
    EXPECT_EQ(vec3.c[2], 3);
}

TEST(VectorTest, DataAccess)
{
    Vector<2, float> vec2{ 5, -20.6f };
    EXPECT_EQ(vec2[0], 5);
    EXPECT_EQ(vec2[1], -20.6f);
    EXPECT_DEATH(vec2[2], "Assertion `index < dimension' failed"); // Takes long because of REGEX search
    EXPECT_EQ(vec2.x, 5);
    EXPECT_EQ(vec2.y, -20.6f);

    Vector<3, float> vec3{ 5, -10.6f, 4 };
    EXPECT_EQ(vec3[0], 5);
    EXPECT_EQ(vec3[1], -10.6f);
    EXPECT_EQ(vec3[2], 4);
    EXPECT_EQ(vec3.x, 5);
    EXPECT_EQ(vec3.y, -10.6f);
    EXPECT_EQ(vec3.z, 4);
    EXPECT_EQ(vec3.r, 5);
    EXPECT_EQ(vec3.g, -10.6f);
    EXPECT_EQ(vec3.b, 4);
    EXPECT_EQ(vec3.xy.x, 5);
    EXPECT_EQ(vec3.xy.y, -10.6f);

    Vector<4, float> vec4{ 5, -10.6f, 4, 2 };
    EXPECT_EQ(vec4[0], 5);
    EXPECT_EQ(vec4[1], -10.6f);
    EXPECT_EQ(vec4[2], 4);
    EXPECT_EQ(vec4[3], 2);
    EXPECT_EQ(vec4.x, 5);
    EXPECT_EQ(vec4.y, -10.6f);
    EXPECT_EQ(vec4.z, 4);
    EXPECT_EQ(vec4.w, 2);
    EXPECT_EQ(vec4.r, 5);
    EXPECT_EQ(vec4.g, -10.6f);
    EXPECT_EQ(vec4.b, 4);
    EXPECT_EQ(vec4.a, 2);
    EXPECT_EQ(vec4.xy.x, 5);
    EXPECT_EQ(vec4.xy.y, -10.6f);
    EXPECT_EQ(vec4.xyz.x, 5);
    EXPECT_EQ(vec4.xyz.y, -10.6f);
    EXPECT_EQ(vec4.xyz.z, 4);
    EXPECT_EQ(vec4.rgb.r, 5);
    EXPECT_EQ(vec4.rgb.g, -10.6f);
    EXPECT_EQ(vec4.rgb.b, 4);
}

TEST(VectorTest, HelperMethods)
{
    Vector<2, float> vec2 = Vector<2, float>::zero();
    EXPECT_EQ(vec2[0], 0);
    EXPECT_EQ(vec2[1], 0);
    vec2 = Vector<2, float>::one();
    EXPECT_EQ(vec2[0], 1);
    EXPECT_EQ(vec2[1], 1);
    vec2 = Vector<2, float>::unitX();
    EXPECT_EQ(vec2[0], 1);
    EXPECT_EQ(vec2[1], 0);
    vec2 = Vector<2, float>::unitY();
    EXPECT_EQ(vec2[0], 0);
    EXPECT_EQ(vec2[1], 1);

    Vector<3, float> vec3 = Vector<3, float>::unitX();
    EXPECT_EQ(vec3[0], 1);
    EXPECT_EQ(vec3[1], 0);
    EXPECT_EQ(vec3[2], 0);
    vec3 = Vector<3, float>::unitY();
    EXPECT_EQ(vec3[0], 0);
    EXPECT_EQ(vec3[1], 1);
    EXPECT_EQ(vec3[2], 0);
    vec3 = Vector<3, float>::unitZ();
    EXPECT_EQ(vec3[0], 0);
    EXPECT_EQ(vec3[1], 0);
    EXPECT_EQ(vec3[2], 1);

    Vector<4, float> vec4 = Vector<4, float>::unitX();
    EXPECT_EQ(vec4[0], 1);
    EXPECT_EQ(vec4[1], 0);
    EXPECT_EQ(vec4[2], 0);
    EXPECT_EQ(vec4[3], 0);
    vec4 = Vector<4, float>::unitY();
    EXPECT_EQ(vec4[0], 0);
    EXPECT_EQ(vec4[1], 1);
    EXPECT_EQ(vec4[2], 0);
    EXPECT_EQ(vec4[3], 0);
    vec4 = Vector<4, float>::unitZ();
    EXPECT_EQ(vec4[0], 0);
    EXPECT_EQ(vec4[1], 0);
    EXPECT_EQ(vec4[2], 1);
    EXPECT_EQ(vec4[3], 0);
    vec4 = Vector<4, float>::unitW();
    EXPECT_EQ(vec4[0], 0);
    EXPECT_EQ(vec4[1], 0);
    EXPECT_EQ(vec4[2], 0);
    EXPECT_EQ(vec4[3], 1);
}

TEST(VectorTest, OperatorOverloads)
{
    Vector<2, float> vec{ 4, -9.9 };
    Vector<2, float> vec2 = -vec;
    EXPECT_FLOAT_EQ(vec2[0], -4);
    EXPECT_FLOAT_EQ(vec2[1], 9.9);
    Vector<2, float> vec2tmp(2);
    vec2 += vec2tmp;
    EXPECT_FLOAT_EQ(vec2[0], -2);
    EXPECT_FLOAT_EQ(vec2[1], 11.9);
    vec2tmp = Vector<2, float>(5);
    vec2 -= vec2tmp;
    EXPECT_FLOAT_EQ(vec2[0], -7);
    EXPECT_FLOAT_EQ(vec2[1], 6.9);
    vec2 *= 2;
    EXPECT_FLOAT_EQ(vec2[0], -14);
    EXPECT_FLOAT_EQ(vec2[1], 13.8);
    vec2 /= 3;
    EXPECT_FLOAT_EQ(vec2[0], -14.0 / 3);
    EXPECT_FLOAT_EQ(vec2[1], 13.8 / 3);
    EXPECT_NE(vec2, vec2tmp);
    vec2tmp = { -14.0 / 3, 13.8 / 3 };
    EXPECT_EQ(vec2, vec2tmp);
}

TEST(VectorTest, PublicMethods)
{
    Vector<2, float> vec2 = { 0, 1 };
    EXPECT_EQ(vec2.dim(), 2);

    Vector<2, float> tmp = { -0, -1 };
    EXPECT_EQ(vec2.neg(), tmp);

    tmp = { 3, -5 };
    EXPECT_FLOAT_EQ(tmp.mag(), sqrt(34.0f));

    tmp = { 5, 5 };
    Vector<2, float> tmp2 = { 1 / sqrt(2), 1 / sqrt(2) };
    EXPECT_EQ(tmp.normalized(), tmp2);

    tmp.normalize();
    EXPECT_EQ(tmp, tmp2);

    tmp = { 5, 5 };
    EXPECT_FLOAT_EQ(tmp.dot(tmp2), 5 * 1 / sqrt(2) * 2);
}

TEST(VectorTest, GlobalOperatorOverloads)
{
    Vector<2, float> vec1{ 4, 5.5 };
    Vector<2, float> vec2{ 1.5, 2 };
    Vector<2, float> vec{ 5.5, 7.5 };
    EXPECT_EQ(vec1 + vec2, vec);
    vec = { 2.5, 3.5 };
    EXPECT_EQ(vec1 - vec2, vec);
    vec = { 8, 11 };
    EXPECT_EQ(vec1 * 2, vec);
    vec = { 12, 16.5 };
    EXPECT_EQ(3 * vec1, vec);
    vec = { 2, 2.75 };
    EXPECT_EQ(vec1 / 2, vec);
}

TEST(VectorTest, GlobalCommonFunctions)
{
    Vector<2, float> vec{ 5.5, 7.5 };
    EXPECT_EQ("(5.5; 7.5)", str(vec));

    std::stringstream ss;
    ss << vec;
    EXPECT_EQ("(5.5; 7.5)", ss.str());
}

} // namespace NAV
