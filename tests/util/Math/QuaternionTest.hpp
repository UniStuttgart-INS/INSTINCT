#pragma once

#include <gtest/gtest.h>

#include "util/Math/Quaternion.hpp"
#include "util/Math/Vector.hpp"
#include "util/Math/Matrix.hpp"

namespace NAV
{
TEST(QuaternionTest, Constructors)
{
    Quaternion<float> q(2);
    EXPECT_EQ(q.q[0], 2);
    EXPECT_EQ(q.q[1], 2);
    EXPECT_EQ(q.q[2], 2);
    EXPECT_EQ(q.q[3], 2);

    Quaternion<float> q1(1, 2, 3);
    EXPECT_EQ(q1.q[0], 0);
    EXPECT_EQ(q1.q[1], 1);
    EXPECT_EQ(q1.q[2], 2);
    EXPECT_EQ(q1.q[3], 3);

    Quaternion<float> q2(1, 2, 3, 4);
    EXPECT_EQ(q2.q[0], 1);
    EXPECT_EQ(q2.q[1], 2);
    EXPECT_EQ(q2.q[2], 3);
    EXPECT_EQ(q2.q[3], 4);

    Vector<3, float> f(2, 3, 4);
    Quaternion<float> q3(1, f);
    EXPECT_EQ(q3.q[0], 1);
    EXPECT_EQ(q3.q[1], 2);
    EXPECT_EQ(q3.q[2], 3);
    EXPECT_EQ(q3.q[3], 4);

    Quaternion<float> q4(f);
    EXPECT_EQ(q4.q[0], 0);
    EXPECT_EQ(q4.q[1], 2);
    EXPECT_EQ(q4.q[2], 3);
    EXPECT_EQ(q4.q[3], 4);

    Vector<4, float> f2(1, 2, 3, 4);
    Quaternion<float> q5(f2);
    EXPECT_EQ(q5.q[0], 1);
    EXPECT_EQ(q5.q[1], 2);
    EXPECT_EQ(q5.q[2], 3);
    EXPECT_EQ(q5.q[3], 4);
}

TEST(QuaternionTest, DataAccess)
{
    Quaternion<float> q1;
    q1(0) = 0;
    q1(1) = 2;
    q1(2) = 3;
    q1(3) = 4;
    EXPECT_EQ(q1.q[0], 0);
    EXPECT_EQ(q1.q[1], 2);
    EXPECT_EQ(q1.q[2], 3);
    EXPECT_EQ(q1.q[3], 4);
}

TEST(QuaternionTest, HelperMethods)
{
    Quaternion<double> qRotX = Quaternion<double>::rotX(M_PI);
    Quaternion<double> qRotXd = Quaternion<double>::rotX_deg(180);
    EXPECT_EQ(qRotX, qRotXd);
    Quaternion<double> qRotY = Quaternion<double>::rotY(M_PI / 2);
    Quaternion<double> qRotYd = Quaternion<double>::rotY_deg(180 / 2);
    EXPECT_EQ(qRotY, qRotYd);
    Quaternion<double> qRotZ = Quaternion<double>::rotZ(M_PI * 3 / 2);
    Quaternion<double> qRotZd = Quaternion<double>::rotZ_deg(270);
    EXPECT_EQ(qRotZ, qRotZd);

    Vector<3, double> vec3(1, 2, 3);
    Matrix<3, 3, double> mRotX = Matrix<3, 3, double>::rotX(M_PI);
    Matrix<3, 3, double> mRotY = Matrix<3, 3, double>::rotY(M_PI / 2);
    Matrix<3, 3, double> mRotZ = Matrix<3, 3, double>::rotZ(M_PI * 3 / 2);

    Vector<3, double> vec3cmp = mRotX * vec3;
    ;
    Vector<3, double> calc = qRotX * vec3 * qRotX.conjugate();
    EXPECT_FLOAT_EQ(round(calc(0) * 10e10) / 10e10, round(vec3cmp(0) * 10e10) / 10e10);
    EXPECT_FLOAT_EQ(round(calc(1) * 10e10) / 10e10, round(vec3cmp(1) * 10e10) / 10e10);
    EXPECT_FLOAT_EQ(round(calc(2) * 10e10) / 10e10, round(vec3cmp(2) * 10e10) / 10e10);

    vec3 = { 1, 0, 0 };
    calc = qRotY * vec3 * qRotY.conjugate();
    vec3cmp = mRotY * vec3;
    EXPECT_FLOAT_EQ(round(calc(0) * 10e10) / 10e10, round(vec3cmp(0) * 10e10) / 10e10);
    EXPECT_FLOAT_EQ(round(calc(1) * 10e10) / 10e10, round(vec3cmp(1) * 10e10) / 10e10);
    EXPECT_FLOAT_EQ(round(calc(2) * 10e10) / 10e10, round(vec3cmp(2) * 10e10) / 10e10);

    calc = qRotZ * vec3 * qRotZ.conjugate();
    vec3cmp = mRotZ * vec3;
    EXPECT_FLOAT_EQ(round(calc(0) * 10e10) / 10e10, round(vec3cmp(0) * 10e10) / 10e10);
    EXPECT_FLOAT_EQ(round(calc(1) * 10e10) / 10e10, round(vec3cmp(1) * 10e10) / 10e10);
    EXPECT_FLOAT_EQ(round(calc(2) * 10e10) / 10e10, round(vec3cmp(2) * 10e10) / 10e10);

    Quaternion<double> qRotX2 = Quaternion<double>::rot(M_PI, { 3, 0, 0 });
    Quaternion<double> qRotY2 = Quaternion<double>::rot(M_PI / 2, { 0, 0.1, 0 });
    Quaternion<double> qRotZ2 = Quaternion<double>::rot_deg(270, { 0, 0, 1 });

    EXPECT_EQ(qRotX, qRotX2);
    EXPECT_EQ(qRotY, qRotY2);
    EXPECT_EQ(qRotZ, qRotZ2);

    Vector<3, double> vec(1, 2, 3);
    Vector<3, double> vec2;
    for (size_t i = 0; i < 359; i++)
    {
        qRotX = Quaternion<double>::dcm2quat(Matrix<3, 3, double>::rotX_deg(i));
        qRotX2 = Quaternion<double>::rotX_deg(i);
        qRotY = Quaternion<double>::dcm2quat(Matrix<3, 3, double>::rotY_deg(i));
        qRotY2 = Quaternion<double>::rotY_deg(i);
        qRotZ = Quaternion<double>::dcm2quat(Matrix<3, 3, double>::rotZ_deg(i));
        qRotZ2 = Quaternion<double>::rotZ_deg(i);

        vec2 = qRotX * vec * qRotX.inverse();
        vec3 = qRotX2 * vec * qRotX2.inverse();
        ASSERT_FLOAT_EQ(vec2.x, vec3.x)
            << "; i = " << i << ", " << qRotX << ", " << qRotX2;
        ASSERT_FLOAT_EQ(vec2.y, vec3.y)
            << "; i = " << i << ", " << qRotX << ", " << qRotX2;
        ASSERT_FLOAT_EQ(vec2.z, vec3.z)
            << "; i = " << i << ", " << qRotX << ", " << qRotX2;

        vec2 = qRotY * vec * qRotY.inverse();
        vec3 = qRotY2 * vec * qRotY2.inverse();
        ASSERT_FLOAT_EQ(vec2.x, vec3.x)
            << "; i = " << i << ", " << qRotY << ", " << qRotY2;
        ASSERT_FLOAT_EQ(vec2.y, vec3.y)
            << "; i = " << i << ", " << qRotY << ", " << qRotY2;
        ASSERT_FLOAT_EQ(vec2.z, vec3.z)
            << "; i = " << i << ", " << qRotY << ", " << qRotY2;

        vec2 = qRotZ * vec * qRotZ.inverse();
        vec3 = qRotZ2 * vec * qRotZ2.inverse();
        ASSERT_FLOAT_EQ(vec2.x, vec3.x)
            << "; i = " << i << ", " << qRotZ << ", " << qRotZ2;
        ASSERT_FLOAT_EQ(vec2.y, vec3.y)
            << "; i = " << i << ", " << qRotZ << ", " << qRotZ2;
        ASSERT_FLOAT_EQ(vec2.z, vec3.z)
            << "; i = " << i << ", " << qRotZ << ", " << qRotZ2;
    }
}

TEST(QuaternionTest, OperatorOverloads)
{
    Quaternion<float> q1(0, -1, 2.5, 3);
    Vector<3, float> v3 = q1;
    EXPECT_EQ(q1(1), v3(0));
    EXPECT_EQ(q1(2), v3(1));
    EXPECT_EQ(q1(3), v3(2));
    Vector<4, float> v4 = q1;
    EXPECT_EQ(q1(0), v4(0));
    EXPECT_EQ(q1(1), v4(1));
    EXPECT_EQ(q1(2), v4(2));
    EXPECT_EQ(q1(3), v4(3));

    q1 *= 2;
    EXPECT_EQ(q1(0), 0);
    EXPECT_EQ(q1(1), -2);
    EXPECT_EQ(q1(2), 5);
    EXPECT_EQ(q1(3), 6);

    Quaternion<float> q2(1, 4, 5, -3.5);
    q1 += q2;
    EXPECT_EQ(q1(0), 1);
    EXPECT_EQ(q1(1), 2);
    EXPECT_EQ(q1(2), 10);
    EXPECT_EQ(q1(3), 2.5);

    q1 /= 3;
    EXPECT_FLOAT_EQ(q1(0), 1.0 / 3);
    EXPECT_FLOAT_EQ(q1(1), 2.0 / 3);
    EXPECT_FLOAT_EQ(q1(2), 10.0 / 3);
    EXPECT_FLOAT_EQ(q1(3), 2.5 / 3);

    q1 *= q2;
    EXPECT_FLOAT_EQ(q1(0), -16.083333);
    EXPECT_FLOAT_EQ(q1(1), -13.833333);
    EXPECT_FLOAT_EQ(q1(2), 10.666666);
    EXPECT_FLOAT_EQ(q1(3), -10.333333);

    Quaternion<float> q3(1, 4, 5, -3.5);
    EXPECT_EQ(q2, q3);
    EXPECT_NE(q1, q3);
}

TEST(QuaternionTest, PublicMethods)
{
    Quaternion<double> q1(1, 4, 5, -3.5);
    Quaternion<double> q2(1, -4, -5, 3.5);
    EXPECT_EQ(q1.conjugate(), q2);

    q2 = q1 * q1.inverse();
    EXPECT_FLOAT_EQ(q2(0), 1);
    EXPECT_FLOAT_EQ(q2(1), 0);
    EXPECT_FLOAT_EQ(q2(2), 0);
    EXPECT_FLOAT_EQ(q2(3), 0);

    double magnitude = q1.mag();
    EXPECT_FLOAT_EQ(magnitude, sqrt(1 * 1 + 4 * 4 + 5 * 5 + 3.5 * 3.5));
    EXPECT_FLOAT_EQ(q1.mag_sqr(), 1 * 1 + 4 * 4 + 5 * 5 + 3.5 * 3.5);

    q2 = q1.normalized();
    EXPECT_FLOAT_EQ(q2.mag(), 1);
    q2 *= magnitude;
    EXPECT_FLOAT_EQ(q2(0), q1(0));
    EXPECT_FLOAT_EQ(q2(1), q1(1));
    EXPECT_FLOAT_EQ(q2(2), q1(2));
    EXPECT_FLOAT_EQ(q2(3), q1(3));

    q2 = q1.normalized();
    q1.normalize();
    EXPECT_EQ(q2, q1);

    Vector<4, double> v41(1, 2, 3, 4);
    Vector<4, double> v42(5, 6, 7, 8);
    Vector<3, double> v31(2, 3, 4);
    Vector<3, double> v32(6, 7, 8);
    q1 = Quaternion<double>(v41);
    q2 = Quaternion<double>(v42);

    EXPECT_EQ(q1.dot(q2), v41.dot(v42));
    Vector<3, double> cross = q1.cross(q2);
    v31 = v31.cross(v32);
    EXPECT_EQ(cross, v31);

    EXPECT_FLOAT_EQ(Quaternion<double>::rotX_deg(45).toDCM()(0, 0), (Matrix<3, 3, double>::rotX_deg(45)(0, 0)));
    EXPECT_FLOAT_EQ(Quaternion<double>::rotX_deg(45).toDCM()(0, 1), (Matrix<3, 3, double>::rotX_deg(45)(0, 1)));
    EXPECT_FLOAT_EQ(Quaternion<double>::rotX_deg(45).toDCM()(0, 2), (Matrix<3, 3, double>::rotX_deg(45)(0, 2)));
    EXPECT_FLOAT_EQ(Quaternion<double>::rotX_deg(45).toDCM()(1, 0), (Matrix<3, 3, double>::rotX_deg(45)(1, 0)));
    EXPECT_FLOAT_EQ(Quaternion<double>::rotX_deg(45).toDCM()(1, 1), (Matrix<3, 3, double>::rotX_deg(45)(1, 1)));
    EXPECT_FLOAT_EQ(Quaternion<double>::rotX_deg(45).toDCM()(1, 2), (Matrix<3, 3, double>::rotX_deg(45)(1, 2)));
    EXPECT_FLOAT_EQ(Quaternion<double>::rotX_deg(45).toDCM()(2, 0), (Matrix<3, 3, double>::rotX_deg(45)(2, 0)));
    EXPECT_FLOAT_EQ(Quaternion<double>::rotX_deg(45).toDCM()(2, 1), (Matrix<3, 3, double>::rotX_deg(45)(2, 1)));
    EXPECT_FLOAT_EQ(Quaternion<double>::rotX_deg(45).toDCM()(2, 2), (Matrix<3, 3, double>::rotX_deg(45)(2, 2)));

    EXPECT_FLOAT_EQ(Quaternion<double>::rotY_deg(185).toDCM()(0, 0), (Matrix<3, 3, double>::rotY_deg(185)(0, 0)));
    EXPECT_FLOAT_EQ(Quaternion<double>::rotY_deg(185).toDCM()(0, 1), (Matrix<3, 3, double>::rotY_deg(185)(0, 1)));
    EXPECT_FLOAT_EQ(Quaternion<double>::rotY_deg(185).toDCM()(0, 2), (Matrix<3, 3, double>::rotY_deg(185)(0, 2)));
    EXPECT_FLOAT_EQ(Quaternion<double>::rotY_deg(185).toDCM()(1, 0), (Matrix<3, 3, double>::rotY_deg(185)(1, 0)));
    EXPECT_FLOAT_EQ(Quaternion<double>::rotY_deg(185).toDCM()(1, 1), (Matrix<3, 3, double>::rotY_deg(185)(1, 1)));
    EXPECT_FLOAT_EQ(Quaternion<double>::rotY_deg(185).toDCM()(1, 2), (Matrix<3, 3, double>::rotY_deg(185)(1, 2)));
    EXPECT_FLOAT_EQ(Quaternion<double>::rotY_deg(185).toDCM()(2, 0), (Matrix<3, 3, double>::rotY_deg(185)(2, 0)));
    EXPECT_FLOAT_EQ(Quaternion<double>::rotY_deg(185).toDCM()(2, 1), (Matrix<3, 3, double>::rotY_deg(185)(2, 1)));
    EXPECT_FLOAT_EQ(Quaternion<double>::rotY_deg(185).toDCM()(2, 2), (Matrix<3, 3, double>::rotY_deg(185)(2, 2)));

    EXPECT_FLOAT_EQ(Quaternion<double>::rotZ_deg(135).toDCM()(0, 0), (Matrix<3, 3, double>::rotZ_deg(135)(0, 0)));
    EXPECT_FLOAT_EQ(Quaternion<double>::rotZ_deg(135).toDCM()(0, 1), (Matrix<3, 3, double>::rotZ_deg(135)(0, 1)));
    EXPECT_FLOAT_EQ(Quaternion<double>::rotZ_deg(135).toDCM()(0, 2), (Matrix<3, 3, double>::rotZ_deg(135)(0, 2)));
    EXPECT_FLOAT_EQ(Quaternion<double>::rotZ_deg(135).toDCM()(1, 0), (Matrix<3, 3, double>::rotZ_deg(135)(1, 0)));
    EXPECT_FLOAT_EQ(Quaternion<double>::rotZ_deg(135).toDCM()(1, 1), (Matrix<3, 3, double>::rotZ_deg(135)(1, 1)));
    EXPECT_FLOAT_EQ(Quaternion<double>::rotZ_deg(135).toDCM()(1, 2), (Matrix<3, 3, double>::rotZ_deg(135)(1, 2)));
    EXPECT_FLOAT_EQ(Quaternion<double>::rotZ_deg(135).toDCM()(2, 0), (Matrix<3, 3, double>::rotZ_deg(135)(2, 0)));
    EXPECT_FLOAT_EQ(Quaternion<double>::rotZ_deg(135).toDCM()(2, 1), (Matrix<3, 3, double>::rotZ_deg(135)(2, 1)));
    EXPECT_FLOAT_EQ(Quaternion<double>::rotZ_deg(135).toDCM()(2, 2), (Matrix<3, 3, double>::rotZ_deg(135)(2, 2)));

    double alpha = 50;
    double beta = 134;
    double gamma = 265;
    Matrix<3, 3, double> mat3 = Matrix<3, 3, double>::rotXYZ_deg(alpha, beta, gamma);
    Quaternion<double> quat = Quaternion<double>::dcm2quat(mat3);

    Vector<3, double> test = Vector<3, double>(1, 2, 3);
    EXPECT_DOUBLE_EQ(((Vector<3, double>)(quat * test * quat.inverse())).x, (mat3 * test).x);
    EXPECT_DOUBLE_EQ(((Vector<3, double>)(quat * test * quat.inverse())).y, (mat3 * test).y);
    EXPECT_DOUBLE_EQ(((Vector<3, double>)(quat * test * quat.inverse())).z, (mat3 * test).z);

    Matrix<3, 3, double> mat31 = quat.toDCM();
    EXPECT_DOUBLE_EQ((mat31 * test).x, (mat3 * test).x);
    EXPECT_DOUBLE_EQ((mat31 * test).y, (mat3 * test).y);
    EXPECT_DOUBLE_EQ((mat31 * test).z, (mat3 * test).z);

    Vector<3, double> euler = quat.toEuler() * 180 / M_PI;

    Matrix<3, 3, double> matX = Matrix<3, 3, double>::rotX_deg(euler.x);
    Matrix<3, 3, double> matY = Matrix<3, 3, double>::rotY_deg(euler.y);
    Matrix<3, 3, double> matZ = Matrix<3, 3, double>::rotZ_deg(euler.z);

    EXPECT_DOUBLE_EQ((mat3 * test).x, (matX * matY * matZ * test).x);
    EXPECT_DOUBLE_EQ((mat3 * test).y, (matX * matY * matZ * test).y);
    EXPECT_DOUBLE_EQ((mat3 * test).z, (matX * matY * matZ * test).z);

    EXPECT_DOUBLE_EQ((quat.rotVector(test)).x, ((Vector<3, double>)(quat * test * quat.inverse())).x);
    EXPECT_DOUBLE_EQ((quat.rotVector(test)).y, ((Vector<3, double>)(quat * test * quat.inverse())).y);
    EXPECT_DOUBLE_EQ((quat.rotVector(test)).z, ((Vector<3, double>)(quat * test * quat.inverse())).z);
}

TEST(QuaternionTest, GlobalOperatorOverloads)
{
    Vector<3, double> vec(1, 2, 3);
    Quaternion<double> q1(4, 5, 6, 7);
    Quaternion<double> q2 = q1 * vec;
    EXPECT_EQ(q2, Quaternion<double>(-38, 8, 0, 16));

    EXPECT_EQ(vec * q1, Quaternion<double>(-38, 0, 16, 8));

    EXPECT_EQ(q1 * q2, Quaternion<double>(-304, -62, -252, -250));

    EXPECT_EQ(q2 * 2, Quaternion<double>(-76, 16, 0, 32));
    EXPECT_EQ(3 * q2, Quaternion<double>(-114, 24, 0, 48));
    EXPECT_EQ(q2 / 2, Quaternion<double>(-19, 4, 0, 8));
}

TEST(QuaternionTest, GlobalCommonFunctions)
{
    Quaternion<float> quat(1.2131, -2, 1232.12, -21.1211);
    EXPECT_EQ("(1.2131; -2; 1232.12; -21.1211)", str(quat));

    std::stringstream ss;
    ss << quat;
    EXPECT_EQ("(1.2131; -2; 1232.12; -21.1211)", ss.str());
}

} // namespace NAV
