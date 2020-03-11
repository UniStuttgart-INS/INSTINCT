#pragma once

#include <gtest/gtest.h>

#include "util/Math/Vector.hpp"
#include "util/Math/Matrix.hpp"

namespace NAV
{
TEST(MatrixTest, Constructors)
{
    Matrix<2, 2, float> mat1(2);
    EXPECT_EQ(mat1.e[0][0], 2);
    EXPECT_EQ(mat1.e[0][1], 2);
    EXPECT_EQ(mat1.e[1][0], 2);
    EXPECT_EQ(mat1.e[1][1], 2);

    Matrix<2, 2, float> mat2(2, 3, 4, 5);
    EXPECT_EQ(mat2.e[0][0], 2);
    EXPECT_EQ(mat2.e[0][1], 3);
    EXPECT_EQ(mat2.e[1][0], 4);
    EXPECT_EQ(mat2.e[1][1], 5);

    Vector<2, float> vec1(1, 2);
    Vector<2, float> vec2(3, 4);
    Matrix<2, 2, float> mat3(vec1, vec2);
    EXPECT_EQ(mat3.e[0][0], 1);
    EXPECT_EQ(mat3.e[0][1], 3);
    EXPECT_EQ(mat3.e[1][0], 2);
    EXPECT_EQ(mat3.e[1][1], 4);

    Matrix<3, 3, float> mat4(3);
    EXPECT_EQ(mat4.e[0][0], 3);
    EXPECT_EQ(mat4.e[0][1], 3);
    EXPECT_EQ(mat4.e[0][2], 3);
    EXPECT_EQ(mat4.e[1][0], 3);
    EXPECT_EQ(mat4.e[1][1], 3);
    EXPECT_EQ(mat4.e[1][2], 3);
    EXPECT_EQ(mat4.e[2][0], 3);
    EXPECT_EQ(mat4.e[2][1], 3);
    EXPECT_EQ(mat4.e[2][2], 3);
}

TEST(MatrixTest, DataAccess)
{
    Matrix<2, 2, float> mat2(5, -20.6f,
                             1, 2);
    EXPECT_EQ(mat2(0, 0), 5);
    EXPECT_EQ(mat2(0, 1), -20.6f);
    EXPECT_EQ(mat2(1, 0), 1);
    EXPECT_EQ(mat2(1, 1), 2);
    EXPECT_DEATH(mat2(2, 1), "Assertion `row < m && col < n' failed"); // Takes long because of REGEX search
    EXPECT_EQ(mat2.e00, 5);
    EXPECT_EQ(mat2.e01, -20.6f);
    EXPECT_EQ(mat2.e10, 1);
    EXPECT_EQ(mat2.e11, 2);

    Matrix<2, 3, float> mat23;
    mat23(0, 0) = 0;
    mat23(0, 1) = 1;
    mat23(0, 2) = 2;
    mat23(1, 0) = 3;
    mat23(1, 1) = 4;
    mat23(1, 2) = 5;
    EXPECT_EQ(mat23(0, 0), 0);
    EXPECT_EQ(mat23(1, 2), 5);
}

TEST(MatrixTest, HelperMethods)
{
    Matrix<2, 2, float> mat = Matrix<2, 2, float>::zero();
    EXPECT_EQ(mat(0, 0), 0);
    EXPECT_EQ(mat(0, 1), 0);
    EXPECT_EQ(mat(1, 0), 0);
    EXPECT_EQ(mat(1, 1), 0);
    mat = Matrix<2, 2, float>::one();
    EXPECT_EQ(mat(0, 0), 1);
    EXPECT_EQ(mat(0, 1), 1);
    EXPECT_EQ(mat(1, 0), 1);
    EXPECT_EQ(mat(1, 1), 1);
    Matrix<3, 3, float> mat2 = Matrix<3, 3, float>::identity();
    EXPECT_EQ(mat2(0, 0), 1);
    EXPECT_EQ(mat2(0, 1), 0);
    EXPECT_EQ(mat2(0, 2), 0);
    EXPECT_EQ(mat2(1, 0), 0);
    EXPECT_EQ(mat2(1, 1), 1);
    EXPECT_EQ(mat2(1, 2), 0);
    EXPECT_EQ(mat2(2, 0), 0);
    EXPECT_EQ(mat2(2, 1), 0);
    EXPECT_EQ(mat2(2, 2), 1);

    Matrix<2, 2, double> mat3 = Matrix<2, 2, double>::rot(M_PI);
    Vector<2, double> vec(1, 0);
    Vector<2, double> vec2(-1, 0);
    EXPECT_FLOAT_EQ(round((mat3 * vec)(0) * 10e10) / 10e10, vec2(0));
    EXPECT_FLOAT_EQ(round((mat3 * vec)(1) * 10e10) / 10e10, vec2(1));

    mat3 = Matrix<2, 2, double>::rot_deg(90);
    vec2 = Vector<2, double>(0, 1);
    EXPECT_FLOAT_EQ(round((mat3 * vec)(0) * 10e10) / 10e10, vec2(0));
    EXPECT_FLOAT_EQ(round((mat3 * vec)(1) * 10e10) / 10e10, vec2(1));

    Matrix<3, 3, double> matRotX = Matrix<3, 3, double>::rotX(M_PI);
    Matrix<3, 3, double> matRotXd = Matrix<3, 3, double>::rotX_deg(180);
    EXPECT_EQ(matRotX, matRotXd);
    Matrix<3, 3, double> matRotY = Matrix<3, 3, double>::rotY(M_PI / 2);
    Matrix<3, 3, double> matRotYd = Matrix<3, 3, double>::rotY_deg(180 / 2);
    EXPECT_EQ(matRotY, matRotYd);
    Matrix<3, 3, double> matRotZ = Matrix<3, 3, double>::rotZ(M_PI * 3 / 2);
    Matrix<3, 3, double> matRotZd = Matrix<3, 3, double>::rotZ_deg(270);
    EXPECT_EQ(matRotZ, matRotZd);

    Vector<3, double> vec3(1, 2, 3);

    Vector<3, double> calc = matRotXd * vec3;
    EXPECT_FLOAT_EQ(calc(0), 1);
    EXPECT_FLOAT_EQ(calc(1), -2);
    EXPECT_FLOAT_EQ(calc(2), -3);

    calc = matRotYd * vec3;
    EXPECT_FLOAT_EQ(calc(0), 3);
    EXPECT_FLOAT_EQ(calc(1), 2);
    EXPECT_FLOAT_EQ(calc(2), -1);

    calc = matRotZd * vec3;
    EXPECT_FLOAT_EQ(calc(0), 2);
    EXPECT_FLOAT_EQ(calc(1), -1);
    EXPECT_FLOAT_EQ(calc(2), 3);

    Vector<3, double> f(1, 0, 0);
    Matrix<3, 3, double> rot = Matrix<3, 3, double>::rot(M_PI, f);
    EXPECT_FLOAT_EQ(round(rot(0, 0) * 10e10) / 10e10, round(matRotX(0, 0) * 10e10) / 10e10);
    EXPECT_FLOAT_EQ(round(rot(0, 1) * 10e10) / 10e10, round(matRotX(0, 1) * 10e10) / 10e10);
    EXPECT_FLOAT_EQ(round(rot(0, 2) * 10e10) / 10e10, round(matRotX(0, 2) * 10e10) / 10e10);
    EXPECT_FLOAT_EQ(round(rot(1, 0) * 10e10) / 10e10, round(matRotX(1, 0) * 10e10) / 10e10);
    EXPECT_FLOAT_EQ(round(rot(1, 1) * 10e10) / 10e10, round(matRotX(1, 1) * 10e10) / 10e10);
    EXPECT_FLOAT_EQ(round(rot(1, 2) * 10e10) / 10e10, round(matRotX(1, 2) * 10e10) / 10e10);
    EXPECT_FLOAT_EQ(round(rot(2, 0) * 10e10) / 10e10, round(matRotX(2, 0) * 10e10) / 10e10);
    EXPECT_FLOAT_EQ(round(rot(2, 1) * 10e10) / 10e10, round(matRotX(2, 1) * 10e10) / 10e10);
    EXPECT_FLOAT_EQ(round(rot(2, 2) * 10e10) / 10e10, round(matRotX(2, 2) * 10e10) / 10e10);

    f = { 0, 2, 0 };
    rot = Matrix<3, 3, double>::rot_deg(180 / 2, f);
    EXPECT_FLOAT_EQ(round(rot(0, 0) * 10e10) / 10e10, round(matRotYd(0, 0) * 10e10) / 10e10);
    EXPECT_FLOAT_EQ(round(rot(0, 1) * 10e10) / 10e10, round(matRotYd(0, 1) * 10e10) / 10e10);
    EXPECT_FLOAT_EQ(round(rot(0, 2) * 10e10) / 10e10, round(matRotYd(0, 2) * 10e10) / 10e10);
    EXPECT_FLOAT_EQ(round(rot(1, 0) * 10e10) / 10e10, round(matRotYd(1, 0) * 10e10) / 10e10);
    EXPECT_FLOAT_EQ(round(rot(1, 1) * 10e10) / 10e10, round(matRotYd(1, 1) * 10e10) / 10e10);
    EXPECT_FLOAT_EQ(round(rot(1, 2) * 10e10) / 10e10, round(matRotYd(1, 2) * 10e10) / 10e10);
    EXPECT_FLOAT_EQ(round(rot(2, 0) * 10e10) / 10e10, round(matRotYd(2, 0) * 10e10) / 10e10);
    EXPECT_FLOAT_EQ(round(rot(2, 1) * 10e10) / 10e10, round(matRotYd(2, 1) * 10e10) / 10e10);
    EXPECT_FLOAT_EQ(round(rot(2, 2) * 10e10) / 10e10, round(matRotYd(2, 2) * 10e10) / 10e10);

    f = { 0, 0, 3.5 };
    rot = Matrix<3, 3, double>::rot_deg(270, f);
    EXPECT_FLOAT_EQ(round(rot(0, 0) * 10e10) / 10e10, round(matRotZ(0, 0) * 10e10) / 10e10);
    EXPECT_FLOAT_EQ(round(rot(0, 1) * 10e10) / 10e10, round(matRotZ(0, 1) * 10e10) / 10e10);
    EXPECT_FLOAT_EQ(round(rot(0, 2) * 10e10) / 10e10, round(matRotZ(0, 2) * 10e10) / 10e10);
    EXPECT_FLOAT_EQ(round(rot(1, 0) * 10e10) / 10e10, round(matRotZ(1, 0) * 10e10) / 10e10);
    EXPECT_FLOAT_EQ(round(rot(1, 1) * 10e10) / 10e10, round(matRotZ(1, 1) * 10e10) / 10e10);
    EXPECT_FLOAT_EQ(round(rot(1, 2) * 10e10) / 10e10, round(matRotZ(1, 2) * 10e10) / 10e10);
    EXPECT_FLOAT_EQ(round(rot(2, 0) * 10e10) / 10e10, round(matRotZ(2, 0) * 10e10) / 10e10);
    EXPECT_FLOAT_EQ(round(rot(2, 1) * 10e10) / 10e10, round(matRotZ(2, 1) * 10e10) / 10e10);
    EXPECT_FLOAT_EQ(round(rot(2, 2) * 10e10) / 10e10, round(matRotZ(2, 2) * 10e10) / 10e10);

    matRotXd = Matrix<3, 3, double>::rotX_deg(90);
    matRotYd = Matrix<3, 3, double>::rotY_deg(90);
    matRotZd = Matrix<3, 3, double>::rotZ_deg(90);

    vec3 = Vector<3, double>(0, 1, 0);
    calc = matRotXd * vec3;
    EXPECT_FLOAT_EQ(round(calc(0) * 10e10) / 10e10, 0);
    EXPECT_FLOAT_EQ(round(calc(1) * 10e10) / 10e10, 0);
    EXPECT_FLOAT_EQ(round(calc(2) * 10e10) / 10e10, 1);

    vec3 = Vector<3, double>(1, 0, 0);
    calc = matRotYd * vec3;
    EXPECT_FLOAT_EQ(round(calc(0) * 10e10) / 10e10, 0);
    EXPECT_FLOAT_EQ(round(calc(1) * 10e10) / 10e10, 0);
    EXPECT_FLOAT_EQ(round(calc(2) * 10e10) / 10e10, -1);

    vec3 = Vector<3, double>(1, 0, 0);
    calc = matRotZd * vec3;
    EXPECT_FLOAT_EQ(round(calc(0) * 10e10) / 10e10, 0);
    EXPECT_FLOAT_EQ(round(calc(1) * 10e10) / 10e10, 1);
    EXPECT_FLOAT_EQ(round(calc(2) * 10e10) / 10e10, 0);

    Matrix<3, 3, double> rotXYZ = Matrix<3, 3, double>::rotXYZ(M_PI, M_PI / 2, M_PI * 3 / 2);
    Matrix<3, 3, double> rotXYZ2 = (matRotX * matRotY * matRotZ);
    EXPECT_EQ(rotXYZ, rotXYZ2);
}

TEST(MatrixTest, OperatorOverloads)
{
    Matrix<2, 2, float> mat(4, -9.9, 7, 8);
    Matrix<2, 2, float> mat2 = -mat;
    EXPECT_EQ(mat2(0, 0), -4);
    EXPECT_EQ(mat2(0, 1), 9.9f);
    EXPECT_EQ(mat2(1, 0), -7);
    EXPECT_EQ(mat2(1, 1), -8);

    Matrix<2, 2, float> mat2tmp(2);
    mat2 += mat2tmp;
    EXPECT_FLOAT_EQ(mat2(0, 0), -2);
    EXPECT_FLOAT_EQ(mat2(0, 1), 11.9f);
    EXPECT_FLOAT_EQ(mat2(1, 0), -5);
    EXPECT_FLOAT_EQ(mat2(1, 1), -6);

    mat2tmp = Matrix<2, 2, float>(5);
    mat2 -= mat2tmp;
    EXPECT_FLOAT_EQ(mat2(0, 0), -7);
    EXPECT_FLOAT_EQ(mat2(0, 1), 6.9f);
    EXPECT_FLOAT_EQ(mat2(1, 0), -10);
    EXPECT_FLOAT_EQ(mat2(1, 1), -11);

    mat2 *= 2;
    EXPECT_FLOAT_EQ(mat2(0, 0), -14);
    EXPECT_FLOAT_EQ(mat2(0, 1), 13.8f);
    EXPECT_FLOAT_EQ(mat2(1, 0), -20);
    EXPECT_FLOAT_EQ(mat2(1, 1), -22);

    mat2 /= 3;
    EXPECT_FLOAT_EQ(mat2(0, 0), -14.0 / 3);
    EXPECT_FLOAT_EQ(mat2(0, 1), 13.8 / 3);
    EXPECT_FLOAT_EQ(mat2(1, 0), -20.0 / 3);
    EXPECT_FLOAT_EQ(mat2(1, 1), -22.0 / 3);
    EXPECT_NE(mat2, mat2tmp);

    mat2tmp = Matrix<2, 2, float>(-14.0 / 3, 13.8 / 3, -20.0 / 3, -22.0 / 3);
    EXPECT_EQ(mat2, mat2tmp);

    mat = Matrix<2, 2, float>(1, 2,
                              3, 4);
    mat2 = Matrix<2, 2, float>(5, 6,
                               7, 8);
    mat2tmp = Matrix<2, 2, float>(1 * 5 + 2 * 7, 1 * 6 + 2 * 8,
                                  3 * 5 + 4 * 7, 3 * 6 + 4 * 8);
    Matrix<2, 2, float> mat3 = mat * mat2;
    EXPECT_EQ(mat3, mat2tmp);

    Matrix<2, 3, float> mat23;
    mat23(0, 0) = 1;
    mat23(0, 1) = 2;
    mat23(0, 2) = 3;
    mat23(1, 0) = 4;
    mat23(1, 1) = 5;
    mat23(1, 2) = 6;
    Matrix<3, 2, float> mat32;
    mat32(0, 0) = 7;
    mat32(0, 1) = 8;
    mat32(1, 0) = 9;
    mat32(1, 1) = 10;
    mat32(2, 0) = 11;
    mat32(2, 1) = 12;

    Matrix<3, 3, float> mat33 = mat32 * mat23;
    Matrix<3, 3, float> mat33cmp;
    mat33cmp(0, 0) = 7 * 1 + 8 * 4;
    mat33cmp(0, 1) = 7 * 2 + 8 * 5;
    mat33cmp(0, 2) = 7 * 3 + 8 * 6;
    mat33cmp(1, 0) = 9 * 1 + 10 * 4;
    mat33cmp(1, 1) = 9 * 2 + 10 * 5;
    mat33cmp(1, 2) = 9 * 3 + 10 * 6;
    mat33cmp(2, 0) = 11 * 1 + 12 * 4;
    mat33cmp(2, 1) = 11 * 2 + 12 * 5;
    mat33cmp(2, 2) = 11 * 3 + 12 * 6;
    EXPECT_EQ(mat33, mat33cmp);

    Matrix<2, 2, float> mat22 = mat23 * mat32;
    Matrix<2, 2, float> mat22cmp(1 * 7 + 2 * 9 + 3 * 11, 1 * 8 + 2 * 10 + 3 * 12,
                                 4 * 7 + 5 * 9 + 6 * 11, 4 * 8 + 5 * 10 + 6 * 12);
    EXPECT_EQ(mat22, mat22cmp);

    Matrix<3, 1, float> dyad1;
    dyad1(0, 0) = 1;
    dyad1(1, 0) = 2;
    dyad1(2, 0) = 3;
    Matrix<1, 3, float> dyad2;
    dyad2(0, 0) = 4;
    dyad2(0, 1) = 5;
    dyad2(0, 2) = 6;
    Matrix<3, 3, float> resdyad;
    resdyad(0, 0) = 4;
    resdyad(0, 1) = 5;
    resdyad(0, 2) = 6;
    resdyad(1, 0) = 8;
    resdyad(1, 1) = 10;
    resdyad(1, 2) = 12;
    resdyad(2, 0) = 12;
    resdyad(2, 1) = 15;
    resdyad(2, 2) = 18;
    EXPECT_EQ(dyad1 * dyad2, resdyad);
}

TEST(MatrixTest, PublicMethods)
{
    Matrix<2, 3, float> mat23;
    EXPECT_EQ(mat23.dimRow(), 2);
    EXPECT_EQ(mat23.dimCol(), 3);
    Matrix<2, 2, float> mat(1, -2, 3.5, -4.5);
    Matrix<2, 2, float> mat2(-1, 2, -3.5, 4.5);
    EXPECT_EQ(mat.neg(), mat2);

    mat23(0, 0) = 1;
    mat23(0, 1) = 2;
    mat23(0, 2) = 3;
    mat23(1, 0) = 4;
    mat23(1, 1) = 5;
    mat23(1, 2) = 6;
    Matrix<3, 2, float> mat32;
    mat32(0, 0) = 1;
    mat32(0, 1) = 4;
    mat32(1, 0) = 2;
    mat32(1, 1) = 5;
    mat32(2, 0) = 3;
    mat32(2, 1) = 6;
    EXPECT_EQ(mat23.transpose(), mat32);

    double alpha = 50;
    double beta = 134;
    double gamma = 265;
    Matrix<3, 3, double> mat3 = Matrix<3, 3, double>::rotXYZ_deg(alpha, beta, gamma);
    Vector<3, double> euler = mat3.toEuler() * 180 / M_PI;

    Matrix<3, 3, double> matX = Matrix<3, 3, double>::rotX_deg(euler.x);
    Matrix<3, 3, double> matY = Matrix<3, 3, double>::rotY_deg(euler.y);
    Matrix<3, 3, double> matZ = Matrix<3, 3, double>::rotZ_deg(euler.z);

    Vector<3, double> test = Vector<3, double>(1, 2, 3);
    EXPECT_DOUBLE_EQ((mat3 * test).x, (matX * matY * matZ * test).x);
    EXPECT_DOUBLE_EQ((mat3 * test).y, (matX * matY * matZ * test).y);
    EXPECT_DOUBLE_EQ((mat3 * test).z, (matX * matY * matZ * test).z);
}

TEST(MatrixTest, GlobalOperatorOverloads)
{
    Matrix<2, 2, float> mat1(1, -2, 3, -4.5);
    Matrix<2, 2, float> mat2(5, 6, 7.5, 8);

    Matrix<2, 2, float> mat(6, 4, 10.5, 3.5);
    EXPECT_EQ(mat1 + mat2, mat);

    mat = Matrix<2, 2, float>(-4, -8, -4.5, -12.5);
    EXPECT_EQ(mat1 - mat2, mat);

    mat = Matrix<2, 2, float>(2, -4, 6, -9);
    EXPECT_EQ(mat1 * 2, mat);

    mat = Matrix<2, 2, float>(3, -6, 9, -13.5);
    EXPECT_EQ(3 * mat1, mat);

    mat = Matrix<2, 2, float>(0.5, -1, 1.5, -2.25);
    EXPECT_EQ(mat1 / 2, mat);
}

TEST(MatrixTest, GlobalCommonFunctions)
{
    Matrix<2, 2, float> mat(5.5, -7.6, 1, 3.33);
    EXPECT_EQ("[(5.5; -7.6)(1; 3.33)]", str(mat));

    std::stringstream ss;
    ss << mat;
    EXPECT_EQ("[(5.5; -7.6)(1; 3.33)]", ss.str());
}

TEST(MatrixTest, MatrixVector)
{
    Matrix<2, 2, float> mat22(58, 64,
                              139, 154);
    Vector<2, float> vec(1, 2);
    Vector<2, float> vec2(186, 447);
    EXPECT_EQ(mat22 * vec, vec2);

    Vector<3, float> dyad1;
    dyad1(0) = 1;
    dyad1(1) = 2;
    dyad1(2) = 3;
    Matrix<1, 3, float> dyad2;
    dyad2(0, 0) = 4;
    dyad2(0, 1) = 5;
    dyad2(0, 2) = 6;
    Matrix<3, 3, float> resdyad;
    resdyad(0, 0) = 4;
    resdyad(0, 1) = 5;
    resdyad(0, 2) = 6;
    resdyad(1, 0) = 8;
    resdyad(1, 1) = 10;
    resdyad(1, 2) = 12;
    resdyad(2, 0) = 12;
    resdyad(2, 1) = 15;
    resdyad(2, 2) = 18;
    EXPECT_EQ(dyad1 * dyad2, resdyad);
}

} // namespace NAV
