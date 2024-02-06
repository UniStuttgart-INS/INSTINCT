// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include <catch2/catch_test_macros.hpp>
#include "CatchMatchers.hpp"

#include "Navigation/Transformations/CoordinateFrames.hpp"
#include "Navigation/Transformations/Units.hpp"
#include "Logger.hpp"

#include "util/Eigen.hpp"

#include <limits>

namespace NAV::TESTS::CoordinateFramesTests
{

namespace ref
{

Eigen::Vector4d qCoeffsFromDcm(const Eigen::Matrix3d& C)
{
    auto a = 0.5 * std::sqrt(1 + C(0, 0) + C(1, 1) + C(2, 2));
    if (1 + C(0, 0) + C(1, 1) + C(2, 2) < 0)
    {
        fmt::print("Negative a {}\n", 1 + C(0, 0) + C(1, 1) + C(2, 2));
    }

    auto b = 1 / (4 * a) * (C(2, 1) - C(1, 2));
    auto c = 1 / (4 * a) * (C(0, 2) - C(2, 0));
    auto d = 1 / (4 * a) * (C(1, 0) - C(0, 1));

    return { b, c, d, a };
}

Eigen::Matrix3d n_Dcm_b(double roll, double pitch, double yaw)
{
    double& R = roll;
    double& P = pitch;
    double& Y = yaw;

    Eigen::Matrix3d DCM;
    // clang-format off
    DCM << cos(Y)*cos(P), cos(Y)*sin(P)*sin(R) - sin(Y)*cos(R), cos(Y)*sin(P)*cos(R) + sin(Y)*sin(R),
           sin(Y)*cos(P), sin(Y)*sin(P)*sin(R) + cos(Y)*cos(R), sin(Y)*sin(P)*cos(R) - cos(Y)*sin(R),
              -sin(P)   ,             cos(P)*sin(R)           ,             cos(P)*cos(R)           ;

    // clang-format on
    return DCM;
}

Eigen::Matrix3d e_Dcm_n(double latitude, double longitude)
{
    Eigen::Matrix3d DCM;
    // clang-format off
    DCM << -sin(latitude)*cos(longitude), -sin(longitude), -cos(latitude)*cos(longitude),
           -sin(latitude)*sin(longitude),  cos(longitude), -cos(latitude)*sin(longitude),
                   cos(latitude)        ,        0       ,        -sin(latitude)        ;

    // clang-format on
    return DCM;
}

Eigen::Matrix3d e_Dcm_i(const double time, const double omega_ie)
{
    double a = omega_ie * time;

    Eigen::Matrix3d DCM;
    // clang-format off
    DCM <<  cos(a), sin(a), 0,
           -sin(a), cos(a), 0,
              0   ,   0   , 1;

    // clang-format on
    return DCM;
}

/// @brief Converts Earth-centered-Earth-fixed coordinates into latitude, longitude and altitude
/// @param[in] e_position Vector with coordinates in ECEF frame in [m]
/// @param[in] a Semi-major axis of the reference ellipsoid
/// @param[in] e_squared Square of the first eccentricity of the ellipsoid
/// @return Vector containing [latitude 𝜙, longitude λ, altitude]^T in [rad, rad, m]
/// @note See C. Jekeli, 2001, Inertial Navigation Systems with Geodetic Applications
Eigen::Vector3d ecef2lla_iter(const Eigen::Vector3d& e_position, double a = InsConst<>::WGS84::a, double e_squared = InsConst<>::WGS84::e_squared)
{
    // Value is used every iteration and does not change
    double sqrt_x1x1_x2x2 = std::sqrt(std::pow(e_position(0), 2) + std::pow(e_position(1), 2));

    // Latitude with initial assumption that h = 0 (eq. 1.85)
    double latitude = std::atan2(e_position(2) / (1 - e_squared), sqrt_x1x1_x2x2);

    double N{};
    size_t maxIterationCount = 6;
    for (size_t i = 0; i < maxIterationCount; i++) // Convergence should break the loop, but better limit the loop itself
    {
        // Radius of curvature of the ellipsoid in the prime vertical plane,
        // i.e., the plane containing the normal at P and perpendicular to the meridian (eq. 1.81)
        N = a / std::sqrt(1 - e_squared * std::pow(std::sin(latitude), 2));

        // Latitude (eq. 1.84)
        double newLatitude = std::atan2(e_position(2) + e_squared * N * std::sin(latitude), sqrt_x1x1_x2x2);

        // Check convergence
        if (std::abs(newLatitude - latitude) <= 1e-13)
        {
            latitude = newLatitude;
            break;
        }

        if (i == maxIterationCount - 1)
        {
            LOG_ERROR("ECEF2LLA conversion did not converge! Difference is still at {} [rad]", std::abs(newLatitude - latitude));
        }

        latitude = newLatitude;
    }

    // Longitude (eq. 1.84)
    double longitude = std::atan2(e_position(1), e_position(0));
    // Altitude (eq. 1.84)
    double altitude = sqrt_x1x1_x2x2 / std::cos(latitude);
    altitude -= N;

    return { latitude, longitude, altitude };
}

} // namespace ref

TEST_CASE("[InsTransformations] Euler to Quaternion conversion", "[InsTransformations]")
{
    auto logger = initializeTestLogger();

    double delta = deg2rad(5);
    // (-pi:pi] x (-pi/2:pi/2] x (-pi:pi]
    for (double roll = -M_PI + delta; roll < M_PI - std::numeric_limits<float>::epsilon(); roll += delta) // NOLINT(clang-analyzer-security.FloatLoopCounter,cert-flp30-c)
    {
        Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());

        auto q = Eigen::Quaterniond{ rollAngle };
        Eigen::Matrix3d C = q.toRotationMatrix();

        fmt::print("Roll: {}\n", rad2deg(roll));
        REQUIRE_THAT(ref::qCoeffsFromDcm(C), Catch::Matchers::WithinAbs(q.coeffs(), 1e-12));
    }
    // (-pi:pi] x (-pi/2:pi/2] x (-pi:pi]
    for (double roll = -M_PI + delta; roll < M_PI - std::numeric_limits<float>::epsilon(); roll += delta) // NOLINT(clang-analyzer-security.FloatLoopCounter,cert-flp30-c)
    {
        for (double pitch = -M_PI / 2.0 + delta; pitch <= M_PI / 2.0 + std::numeric_limits<float>::epsilon(); pitch += delta) // NOLINT(clang-analyzer-security.FloatLoopCounter,cert-flp30-c)
        {
            Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());

            auto q = pitchAngle * rollAngle;
            Eigen::Matrix3d C = q.toRotationMatrix();

            fmt::print("Roll, Pitch: {}, {}\n", rad2deg(roll), rad2deg(pitch));
            REQUIRE_THAT(ref::qCoeffsFromDcm(C), Catch::Matchers::WithinAbs(q.coeffs(), 1e-12));
        }
    }
    // (-pi:pi] x (-pi/2:pi/2] x (-pi:pi]
    for (double roll = -M_PI + delta; roll < M_PI - std::numeric_limits<float>::epsilon(); roll += delta) // NOLINT(clang-analyzer-security.FloatLoopCounter,cert-flp30-c)
    {
        if (std::abs(roll - (-M_PI / 2.0)) < 1e-8    // -90°
            || std::abs(roll - (M_PI / 2.0)) < 1e-8) //  90°
        {
            continue; // Scalar Quaternion becomes 0
        }
        for (double pitch = -M_PI / 2.0 + delta; pitch < M_PI / 2.0 - std::numeric_limits<float>::epsilon(); pitch += delta) // NOLINT(clang-analyzer-security.FloatLoopCounter,cert-flp30-c)
        {
            for (double yaw = -M_PI + delta; yaw < M_PI - std::numeric_limits<float>::epsilon(); yaw += delta) // NOLINT(clang-analyzer-security.FloatLoopCounter,cert-flp30-c)
            {
                if (std::abs(yaw - (-M_PI / 2.0)) < 1e-8    // -90°
                    || std::abs(yaw - (M_PI / 2.0)) < 1e-8) //  90°
                {
                    continue; // Scalar Quaternion becomes 0
                }
                Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
                Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
                Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

                auto q = yawAngle * pitchAngle * rollAngle;
                Eigen::Matrix3d C = q.toRotationMatrix();

                fmt::print("Roll, Pitch, Yaw: {}, {}, {}\n", rad2deg(roll), rad2deg(pitch), rad2deg(yaw));
                fmt::print("DCM\n{}\n", C);

                // Check if in our notation the scalar quaternion is always positive
                REQUIRE(ref::qCoeffsFromDcm(C)(3) > 0);
                if (q.w() < 0)
                {
                    Eigen::Vector4d qCoeffsNeg = -q.coeffs();
                    REQUIRE_THAT(ref::qCoeffsFromDcm(C), Catch::Matchers::WithinAbs(qCoeffsNeg, 1e-6));
                }
                else
                {
                    REQUIRE_THAT(ref::qCoeffsFromDcm(C), Catch::Matchers::WithinAbs(q.coeffs(), 1e-6));
                }
            }
        }
    }
}

TEST_CASE("[InsTransformations] Quaternion to Euler conversion", "[InsTransformations]")
{
    auto logger = initializeTestLogger();

    auto quat = [](double alpha, double beta, double gamma) {
        Eigen::AngleAxisd xAngle(alpha, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd yAngle(beta, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd zAngle(gamma, Eigen::Vector3d::UnitZ());

        return zAngle * yAngle * xAngle;
    };

    double delta = deg2rad(5);
    // (-pi:pi] x (-pi/2:pi/2] x (-pi:pi]
    for (double alpha = -M_PI + delta; alpha < M_PI - std::numeric_limits<float>::epsilon(); alpha += delta) // NOLINT(clang-analyzer-security.FloatLoopCounter,cert-flp30-c)
    {
        for (double beta = -M_PI / 2.0 + delta; beta < M_PI / 2.0 - std::numeric_limits<float>::epsilon(); beta += delta) // NOLINT(clang-analyzer-security.FloatLoopCounter,cert-flp30-c)
        {
            for (double gamma = -M_PI + delta; gamma < M_PI - std::numeric_limits<float>::epsilon(); gamma += delta) // NOLINT(clang-analyzer-security.FloatLoopCounter,cert-flp30-c)
            {
                auto q = quat(alpha, beta, gamma);
                auto ZYX = rad2deg(trafo::quat2eulerZYX(q));
                REQUIRE_THAT(ZYX, Catch::Matchers::WithinAbs(rad2deg(Eigen::Vector3d{ alpha, beta, gamma }), 1e-8));
            }
        }
    }
}

TEST_CASE("[InsTransformations] Negated Quaternion to Euler conversion", "[InsTransformations]")
{
    auto logger = initializeTestLogger();

    auto quat = [](double alpha, double beta, double gamma) {
        Eigen::AngleAxisd xAngle(alpha, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd yAngle(beta, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd zAngle(gamma, Eigen::Vector3d::UnitZ());

        return zAngle * yAngle * xAngle;
    };

    double delta = deg2rad(5);
    // (-pi:pi] x (-pi/2:pi/2] x (-pi:pi]
    for (double alpha = -M_PI + delta; alpha < M_PI - std::numeric_limits<float>::epsilon(); alpha += delta) // NOLINT(clang-analyzer-security.FloatLoopCounter,cert-flp30-c)
    {
        for (double beta = -M_PI / 2.0 + delta; beta < M_PI / 2.0 - std::numeric_limits<float>::epsilon(); beta += delta) // NOLINT(clang-analyzer-security.FloatLoopCounter,cert-flp30-c)
        {
            for (double gamma = -M_PI + delta; gamma < M_PI - std::numeric_limits<float>::epsilon(); gamma += delta) // NOLINT(clang-analyzer-security.FloatLoopCounter,cert-flp30-c)
            {
                auto q = quat(alpha, beta, gamma);
                if (q.w() < 0)
                {
                    q.coeffs() = -q.coeffs();
                }

                auto ZYX = rad2deg(trafo::quat2eulerZYX(q));
                REQUIRE_THAT(ZYX, Catch::Matchers::WithinAbs(rad2deg(Eigen::Vector3d{ alpha, beta, gamma }), 1e-8));
            }
        }
    }
}

TEST_CASE("[InsTransformations] Inertial <=> Earth-fixed frame conversion", "[InsTransformations]")
{
    auto logger = initializeTestLogger();

    double time = 86164.099 / 3.0;
    auto e_Quat_i = trafo::e_Quat_i(time, InsConst<>::omega_ie);
    CHECK_THAT(e_Quat_i.norm(), Catch::Matchers::WithinAbs(1.0, EPSILON));

    auto C_ei = e_Quat_i.toRotationMatrix();
    auto C_ei_ref = ref::e_Dcm_i(time, InsConst<>::omega_ie);

    CHECK_THAT(C_ei, Catch::Matchers::WithinAbs(C_ei_ref, EPSILON));

    /* -------------------------------------------------------------------------------------------------------- */

    // Sidereal day: 23h 56min 4.099s
    auto siderialDay4 = 86164.099 / 4.0;
    e_Quat_i = trafo::e_Quat_i(siderialDay4, InsConst<>::omega_ie);
    CHECK_THAT(e_Quat_i.norm(), Catch::Matchers::WithinAbs(1.0, EPSILON));
    // Star day: 23h 56min 4.0905s
    // auto starHalfDay = 86164.0905 / 2.0;
    // auto e_Quat_i = trafo::e_Quat_i(starHalfDay);

    Eigen::Vector3d i_x{ 1, -2.5, 22 };
    Eigen::Vector3d e_x = e_Quat_i * i_x;

    CHECK_THAT(e_x, Catch::Matchers::WithinAbs(Eigen::Vector3d{ -2.5, -1, 22 }, 1e-8));

    auto i_Quat_e = trafo::i_Quat_e(siderialDay4, InsConst<>::omega_ie);
    CHECK_THAT(i_Quat_e.norm(), Catch::Matchers::WithinAbs(1.0, EPSILON));

    auto q_identity = i_Quat_e * e_Quat_i;
    CHECK_THAT(q_identity, Catch::Matchers::WithinAbs(Eigen::Quaterniond::Identity(), EPSILON));
}

TEST_CASE("[InsTransformations] Navigation <=> Earth-fixed frame conversion", "[InsTransformations]")
{
    auto logger = initializeTestLogger();

    double latitude = deg2rad(88);
    double longitude = deg2rad(-40);

    auto e_Quat_n = trafo::e_Quat_n(latitude, longitude);
    CHECK_THAT(e_Quat_n.norm(), Catch::Matchers::WithinAbs(1.0, EPSILON));

    auto C_en = e_Quat_n.toRotationMatrix();
    auto C_en_ref = ref::e_Dcm_n(latitude, longitude);

    CHECK_THAT(C_en, Catch::Matchers::WithinAbs(C_en_ref, EPSILON));

    /* -------------------------------------------------------------------------------------------------------- */

    latitude = deg2rad(90);
    longitude = 0.0;

    auto n_Quat_e = trafo::n_Quat_e(latitude, longitude);
    CHECK_THAT(n_Quat_e.norm(), Catch::Matchers::WithinAbs(1.0, EPSILON));

    Eigen::Vector3d e_x{ 1, 2, 3 };
    auto n_x = n_Quat_e * e_x;

    CHECK_THAT(n_x, Catch::Matchers::WithinAbs(Eigen::Vector3d{ -1, 2, -3 }, EPSILON));

    /* -------------------------------------------------------------------------------------------------------- */

    latitude = deg2rad(0);
    longitude = deg2rad(0);

    n_Quat_e = trafo::n_Quat_e(latitude, longitude);
    CHECK_THAT(n_Quat_e.norm(), Catch::Matchers::WithinAbs(1.0, EPSILON));

    e_x = { 1, 2, 3 };
    n_x = n_Quat_e * e_x;

    CHECK_THAT(n_x, Catch::Matchers::WithinAbs(Eigen::Vector3d{ 3, 2, -1 }, EPSILON));

    /* -------------------------------------------------------------------------------------------------------- */

    latitude = deg2rad(0);
    longitude = deg2rad(0);

    n_Quat_e = trafo::n_Quat_e(latitude, longitude);
    CHECK_THAT(n_Quat_e.norm(), Catch::Matchers::WithinAbs(1.0, EPSILON));
    //                    (0, 0, 7.2921151467e-05)
    n_x = n_Quat_e * InsConst<>::e_omega_ie;

    CHECK_THAT(n_x, Catch::Matchers::WithinAbs(Eigen::Vector3d{ InsConst<>::omega_ie, 0, 0 }, EPSILON));

    /* -------------------------------------------------------------------------------------------------------- */

    latitude = deg2rad(45);
    longitude = deg2rad(90);

    n_Quat_e = trafo::n_Quat_e(latitude, longitude);
    CHECK_THAT(n_Quat_e.norm(), Catch::Matchers::WithinAbs(1.0, EPSILON));
    //                    (0, 0, 7.2921151467e-05)
    n_x = n_Quat_e * InsConst<>::e_omega_ie;

    CHECK_THAT(n_x, Catch::Matchers::WithinAbs(Eigen::Vector3d{ InsConst<>::omega_ie / std::sqrt(2), 0, -InsConst<>::omega_ie / std::sqrt(2) }, EPSILON));
}

TEST_CASE("[InsTransformations] NED <=> Earth-centered-earth-fixed frame conversion", "[InsTransformations]")
{
    auto logger = initializeTestLogger();

    double latitude_ref = deg2rad(88);
    double longitude_ref = deg2rad(-40);
    double altitude_ref = 500;
    auto e_position_ref = trafo::lla2ecef_WGS84(Eigen::Vector3d{ latitude_ref, longitude_ref, altitude_ref });

    auto n_position_ref = trafo::ecef2ned(e_position_ref, Eigen::Vector3d{ latitude_ref, longitude_ref, altitude_ref });

    CHECK(n_position_ref(0) == 0);
    CHECK(n_position_ref(1) == 0);
    CHECK(n_position_ref(2) == 0);

    auto e_position = trafo::lla2ecef_WGS84(Eigen::Vector3d{ latitude_ref, longitude_ref, altitude_ref + 200 });
    auto n_position = trafo::ecef2ned(e_position, Eigen::Vector3d{ latitude_ref, longitude_ref, altitude_ref });

    LOG_DEBUG("{}", (n_position - Eigen::Vector3d{ 0, 0, -200 }).transpose());
    CHECK_THAT(n_position, Catch::Matchers::WithinAbs(Eigen::Vector3d{ 0, 0, -200 }, 1e-9));

    /* -------------------------------------------------------------------------------------------------------- */

    e_position = trafo::ned2ecef(n_position_ref, Eigen::Vector3d{ latitude_ref, longitude_ref, altitude_ref });

    CHECK(e_position == e_position_ref);

    /* -------------------------------------------------------------------------------------------------------- */

    e_position_ref = trafo::lla2ecef_WGS84(Eigen::Vector3d{ deg2rad(-10), deg2rad(70), 2001 });
    e_position = e_position_ref;
    for (size_t i = 0; i < 10000; i++)
    {
        n_position = trafo::ecef2ned(e_position, Eigen::Vector3d{ latitude_ref, longitude_ref, altitude_ref });
        e_position = trafo::ned2ecef(n_position, Eigen::Vector3d{ latitude_ref, longitude_ref, altitude_ref });
    }

    LOG_DEBUG("{}", (e_position - e_position_ref).transpose());
    CHECK_THAT(e_position, Catch::Matchers::WithinAbs(e_position_ref, 7e-8));

    /* -------------------------------------------------------------------------------------------------------- */
}

TEST_CASE("[InsTransformations] Body <=> navigation DCM/Quaternion comparison", "[InsTransformations]")
{
    auto logger = initializeTestLogger();

    double delta = deg2rad(5);
    // (-pi:pi] x (-pi/2:pi/2] x (-pi:pi]
    for (double roll = -M_PI + delta; roll <= M_PI + std::numeric_limits<float>::epsilon(); roll += delta) // NOLINT(clang-analyzer-security.FloatLoopCounter,cert-flp30-c)
    {
        for (double pitch = -M_PI / 2.0 + delta; pitch <= M_PI / 2.0 + std::numeric_limits<float>::epsilon(); pitch += delta) // NOLINT(clang-analyzer-security.FloatLoopCounter,cert-flp30-c)
        {
            for (double yaw = -M_PI + delta; yaw <= M_PI + std::numeric_limits<float>::epsilon(); yaw += delta) // NOLINT(clang-analyzer-security.FloatLoopCounter,cert-flp30-c)
            {
                auto n_Quat_b = trafo::n_Quat_b(roll, pitch, yaw);
                REQUIRE_THAT(n_Quat_b.norm(), Catch::Matchers::WithinAbs(1.0, EPSILON));

                auto C_nb = n_Quat_b.toRotationMatrix();
                auto C_nb_ref = ref::n_Dcm_b(roll, pitch, yaw);

                REQUIRE_THAT(C_nb, Catch::Matchers::WithinAbs(C_nb_ref, 1e-13));
            }
        }
    }
}

TEST_CASE("[InsTransformations] Body <=> navigation frame conversion", "[InsTransformations]")
{
    auto logger = initializeTestLogger();

    /* -------------------------------------------------------------------------------------------------------- */

    double roll = deg2rad(45);
    double pitch = 0.0;
    double yaw = 0.0;
    auto b_Quat_n = trafo::b_Quat_n(roll, pitch, yaw);
    CHECK_THAT(b_Quat_n.norm(), Catch::Matchers::WithinAbs(1.0, EPSILON));

    Eigen::Vector3d n_x{ 1.0, 1.0, 1.0 };
    Eigen::Vector3d b_x = b_Quat_n * n_x;

    CHECK_THAT(b_x, Catch::Matchers::WithinAbs(Eigen::Vector3d{ 1, std::sqrt(2), 0 }, EPSILON));

    /* -------------------------------------------------------------------------------------------------------- */

    roll = 0.0;
    pitch = deg2rad(45);
    yaw = 0.0;
    b_Quat_n = trafo::b_Quat_n(roll, pitch, yaw);
    CHECK_THAT(b_Quat_n.norm(), Catch::Matchers::WithinAbs(1.0, EPSILON));

    n_x = { 1.0, 1.0, 1.0 };
    b_x = b_Quat_n * n_x;

    CHECK_THAT(b_x, Catch::Matchers::WithinAbs(Eigen::Vector3d{ 0, 1, std::sqrt(2) }, EPSILON));

    /* -------------------------------------------------------------------------------------------------------- */

    roll = 0.0;
    pitch = 0.0;
    yaw = deg2rad(-45);
    b_Quat_n = trafo::b_Quat_n(roll, pitch, yaw);
    CHECK_THAT(b_Quat_n.norm(), Catch::Matchers::WithinAbs(1.0, EPSILON));

    n_x = Eigen::Vector3d{ 1.0, 1.0, 0.0 };
    b_x = b_Quat_n * n_x;

    CHECK_THAT(b_x, Catch::Matchers::WithinAbs(Eigen::Vector3d{ 0, std::sqrt(2), 0 }, EPSILON));

    /* -------------------------------------------------------------------------------------------------------- */

    roll = 0.0;
    pitch = deg2rad(90);
    yaw = deg2rad(90);
    b_Quat_n = trafo::b_Quat_n(roll, pitch, yaw);
    CHECK_THAT(b_Quat_n.norm(), Catch::Matchers::WithinAbs(1.0, EPSILON));

    n_x = Eigen::Vector3d{ 1.0, 2.0, 3.0 };
    b_x = b_Quat_n * n_x;

    CHECK_THAT(b_x, Catch::Matchers::WithinAbs(Eigen::Vector3d{ -3, -1, 2 }, EPSILON));

    /* -------------------------------------------------------------------------------------------------------- */

    roll = deg2rad(90);
    pitch = deg2rad(90);
    yaw = 0.0;
    b_Quat_n = trafo::b_Quat_n(roll, pitch, yaw);
    CHECK_THAT(b_Quat_n.norm(), Catch::Matchers::WithinAbs(1.0, EPSILON));

    n_x = Eigen::Vector3d{ 1.0, 2.0, 3.0 };
    b_x = b_Quat_n * n_x;

    CHECK_THAT(b_x, Catch::Matchers::WithinAbs(Eigen::Vector3d{ -3, 1, -2 }, EPSILON));

    /* -------------------------------------------------------------------------------------------------------- */

    roll = deg2rad(90);
    pitch = deg2rad(180);
    yaw = deg2rad(90);
    b_Quat_n = trafo::b_Quat_n(roll, pitch, yaw);
    CHECK_THAT(b_Quat_n.norm(), Catch::Matchers::WithinAbs(1.0, EPSILON));

    n_x = { 1.0, 2.0, 3.0 };
    b_x = b_Quat_n * n_x;

    CHECK_THAT(b_x, Catch::Matchers::WithinAbs(Eigen::Vector3d{ -2, -3, 1 }, EPSILON));
}

TEST_CASE("[InsTransformations] Platform <=> body frame conversion", "[InsTransformations]")
{
    auto logger = initializeTestLogger();

    double mountingAngleX = deg2rad(90);
    double mountingAngleY = 0.0;
    double mountingAngleZ = deg2rad(-90);

    auto b_Quat_p = trafo::b_Quat_p(mountingAngleX, mountingAngleY, mountingAngleZ);
    CHECK_THAT(b_Quat_p.norm(), Catch::Matchers::WithinAbs(1.0, EPSILON));

    Eigen::Vector3d p_x{ 2.0, 0.0, 9.81 };
    Eigen::Vector3d b_x = b_Quat_p * p_x;

    CHECK_THAT(b_x, Catch::Matchers::WithinAbs(Eigen::Vector3d{ p_x(1), p_x(2), -p_x(0) }, EPSILON));
}

TEST_CASE("[InsTransformations] LLA <=> ECEF conversion", "[InsTransformations]")
{
    auto logger = initializeTestLogger();

    // Conversion with https://www.oc.nps.edu/oc2902w/coord/llhxyz.htm

    // Stuttgart, Breitscheidstraße 2
    // https://www.koordinaten-umrechner.de/decimal/48.780810,9.172012?karte=OpenStreetMap&zoom=19
    double latitude = deg2rad(48.78081);
    double longitude = deg2rad(9.172012);
    double altitude = 254;
    Eigen::Vector3d e_position_ref = Eigen::Vector3d(4157.128, 671.224, 4774.723) * 1000;
    Eigen::Vector3d e_position = trafo::lla2ecef_WGS84(Eigen::Vector3d{ latitude, longitude, altitude });
    Eigen::Vector3d lla_position = trafo::ecef2lla_WGS84(e_position_ref);
    CHECK_THAT(e_position, Catch::Matchers::WithinAbs(e_position_ref, 0.3));
    CHECK_THAT(lla_position.x(), Catch::Matchers::WithinAbs(latitude, 1e-7));
    CHECK_THAT(lla_position.y(), Catch::Matchers::WithinAbs(longitude, 1e-8));
    CHECK_THAT(lla_position.z(), Catch::Matchers::WithinAbs(altitude, 0.5));

    /* -------------------------------------------------------------------------- */

    // New York
    // https://www.koordinaten-umrechner.de/decimal/40.712728,-74.006015?karte=OpenStreetMap&zoom=4
    latitude = deg2rad(40.712728);
    longitude = deg2rad(-74.006015);
    altitude = 13;
    e_position_ref = Eigen::Vector3d(1334.001, -4654.06, 4138.303) * 1000;
    e_position = trafo::lla2ecef_WGS84(Eigen::Vector3d{ latitude, longitude, altitude });
    lla_position = trafo::ecef2lla_WGS84(e_position_ref);
    CHECK_THAT(e_position, Catch::Matchers::WithinAbs(e_position_ref, 0.5));
    CHECK_THAT(lla_position.x(), Catch::Matchers::WithinAbs(latitude, 1e-7));
    CHECK_THAT(lla_position.y(), Catch::Matchers::WithinAbs(longitude, 1e-7));
    CHECK_THAT(lla_position.z(), Catch::Matchers::WithinAbs(altitude, 0.5));

    /* -------------------------------------------------------------------------- */

    latitude = 0;
    longitude = 0;
    altitude = -3492;
    e_position_ref = Eigen::Vector3d(6374.645, 0, 0) * 1000;
    e_position = trafo::lla2ecef_WGS84(Eigen::Vector3d{ latitude, longitude, altitude });
    lla_position = trafo::ecef2lla_WGS84(e_position_ref);
    CHECK_THAT(e_position, Catch::Matchers::WithinAbs(e_position_ref, EPSILON));
    CHECK_THAT(lla_position.x(), Catch::Matchers::WithinAbs(latitude, EPSILON));
    CHECK_THAT(lla_position.y(), Catch::Matchers::WithinAbs(longitude, EPSILON));
    CHECK_THAT(lla_position.z(), Catch::Matchers::WithinAbs(altitude, 1e-9));

    /* -------------------------------------------------------------------------- */

    latitude = deg2rad(-89.9999);
    longitude = deg2rad(0);
    altitude = 2801;
    e_position_ref = Eigen::Vector3d(0.011, 0, -6359.553) * 1000;
    e_position = trafo::lla2ecef_WGS84(Eigen::Vector3d{ latitude, longitude, altitude });
    lla_position = trafo::ecef2lla_WGS84(e_position_ref);
    CHECK_THAT(e_position, Catch::Matchers::WithinAbs(e_position_ref, 0.4));
    CHECK_THAT(lla_position.x(), Catch::Matchers::WithinAbs(latitude, 1e-6));
    CHECK_THAT(lla_position.y(), Catch::Matchers::WithinAbs(longitude, EPSILON));
    CHECK_THAT(lla_position.z(), Catch::Matchers::WithinAbs(altitude, 0.5));

    /* -------------------------------------------------------------------------- */

    latitude = deg2rad(40);
    longitude = deg2rad(180);
    altitude = -5097;
    e_position_ref = Eigen::Vector3d(-4888.803, 0, 4074.709) * 1000;
    e_position = trafo::lla2ecef_WGS84(Eigen::Vector3d{ latitude, longitude, altitude });
    lla_position = trafo::ecef2lla_WGS84(e_position_ref);
    LOG_DATA("{}", (e_position - e_position_ref).transpose());
    CHECK_THAT(e_position.x(), Catch::Matchers::WithinAbs(e_position_ref.x(), 0.072));
    CHECK_THAT(e_position.y(), Catch::Matchers::WithinAbs(e_position_ref.y(), 1e-9));
    CHECK_THAT(e_position.z(), Catch::Matchers::WithinAbs(e_position_ref.z(), 0.29));
    LOG_DATA("{}", (lla_position - Eigen::Vector3d(latitude, longitude, altitude)).transpose());
    CHECK_THAT(lla_position.x(), Catch::Matchers::WithinAbs(latitude, 2.7e-8));
    CHECK_THAT(lla_position.y(), Catch::Matchers::WithinAbs(longitude, EPSILON));
    CHECK_THAT(lla_position.z(), Catch::Matchers::WithinAbs(altitude, 0.24));

    /* -------------------------------------------------------------------------- */

    latitude = deg2rad(40);
    longitude = deg2rad(180);
    altitude = -5097;
    lla_position = { latitude, longitude, altitude };
    for (size_t i = 0; i < 100000; i++)
    {
        lla_position = trafo::ecef2lla_WGS84(trafo::lla2ecef_WGS84(lla_position));
    }
    CHECK_THAT(lla_position.x(), Catch::Matchers::WithinAbs(latitude, EPSILON));
    CHECK_THAT(lla_position.y(), Catch::Matchers::WithinAbs(longitude, EPSILON));
    CHECK_THAT(lla_position.z(), Catch::Matchers::WithinAbs(altitude, 1e-8));
    LOG_DATA("{}", (lla_position - Eigen::Vector3d(latitude, longitude, altitude)).transpose());
}

TEST_CASE("[InsTransformations] LLA => ECEF => LLA conversion", "[InsTransformations]")
{
    auto logger = initializeTestLogger();

    for (int lat = -89; lat < 90; lat += 1)
    {
        for (int lon = -179; lon < 180; lon += 1)
        {
            for (int alt = -10000; alt <= 100000; alt += 10000)
            {
                double latitude = deg2rad(lat);
                double longitude = deg2rad(lon);
                double altitude = alt;
                Eigen::Vector3d lla{ latitude, longitude, altitude };
                for (size_t i = 0; i < 10; i++)
                {
                    lla = trafo::ecef2lla_WGS84(trafo::lla2ecef_WGS84(lla));
                }
                REQUIRE_THAT(lla.x(), Catch::Matchers::WithinAbs(latitude, 1e-14));
                REQUIRE_THAT(lla.y(), Catch::Matchers::WithinAbs(longitude, 1e-14));
                REQUIRE_THAT(lla.z(), Catch::Matchers::WithinAbs(altitude, 1e-7));
            }
        }
    }
}

TEST_CASE("[InsTransformations] LLA => ECEF => LLH-iterative conversion", "[InsTransformations]")
{
    auto logger = initializeTestLogger();

    for (int lat = -89; lat < 90; lat += 1)
    {
        for (int lon = -179; lon < 180; lon += 1)
        {
            for (int alt = -10000; alt <= 100000; alt += 10000)
            {
                double latitude = deg2rad(lat);
                double longitude = deg2rad(lon);
                double altitude = alt;
                Eigen::Vector3d lla{ latitude, longitude, altitude };
                for (size_t i = 0; i < 10; i++)
                {
                    lla = ref::ecef2lla_iter(trafo::lla2ecef_WGS84(lla));
                }
                REQUIRE_THAT(lla.x(), Catch::Matchers::WithinAbs(latitude, 1e-14));
                REQUIRE_THAT(lla.y(), Catch::Matchers::WithinAbs(longitude, 1e-14));
                REQUIRE_THAT(lla.z(), Catch::Matchers::WithinAbs(altitude, 1e-7));
            }
        }
    }

    /* -------------------------------------------------------------------------------------------------------- */

    double latitude = deg2rad(90);
    double longitude = deg2rad(0);
    double altitude = 0;
    auto e_position = trafo::lla2ecef_WGS84(Eigen::Vector3d{ latitude, longitude, altitude });
    auto lla_position_iter = ref::ecef2lla_iter(e_position, InsConst<>::WGS84::a, InsConst<>::WGS84::e_squared);

    CHECK_THAT(lla_position_iter.x(), Catch::Matchers::WithinAbs(latitude, EPSILON));
    CHECK_THAT(lla_position_iter.y(), Catch::Matchers::WithinAbs(longitude, EPSILON));
    CHECK_THAT(lla_position_iter.z(), Catch::Matchers::WithinAbs(altitude, 1e-9));

    /* -------------------------------------------------------------------------------------------------------- */

    latitude = deg2rad(90);
    longitude = deg2rad(180);
    altitude = 0;
    e_position = trafo::lla2ecef_WGS84(Eigen::Vector3d{ latitude, longitude, altitude });
    lla_position_iter = ref::ecef2lla_iter(e_position, InsConst<>::WGS84::a, InsConst<>::WGS84::e_squared);

    CHECK_THAT(lla_position_iter.x(), Catch::Matchers::WithinAbs(latitude, EPSILON));
    CHECK_THAT(lla_position_iter.y(), Catch::Matchers::WithinAbs(longitude, EPSILON));
    CHECK_THAT(lla_position_iter.z(), Catch::Matchers::WithinAbs(altitude, 1e-9));
}

TEST_CASE("[InsTransformations] Transformation chains", "[InsTransformations]")
{
    auto logger = initializeTestLogger();

    double roll = deg2rad(20);
    double pitch = deg2rad(50);
    double yaw = deg2rad(190);

    double latitude = deg2rad(10);
    double longitude = deg2rad(40);

    Eigen::Quaterniond b_Quat_p(Eigen::AngleAxisd(deg2rad(-90), Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond n_Quat_b = trafo::n_Quat_b(roll, pitch, yaw);
    Eigen::Quaterniond e_Quat_n = trafo::e_Quat_n(latitude, longitude);

    CHECK_THAT(b_Quat_p.norm(), Catch::Matchers::WithinAbs(1.0, EPSILON));
    CHECK_THAT(n_Quat_b.norm(), Catch::Matchers::WithinAbs(1.0, EPSILON));
    CHECK_THAT(e_Quat_n.norm(), Catch::Matchers::WithinAbs(1.0, EPSILON));

    Eigen::Vector3d v_p{ 1, 3, 5 };

    Eigen::Vector3d b_v = b_Quat_p * v_p;
    Eigen::Vector3d n_velocity = n_Quat_b * b_v;
    Eigen::Vector3d e_v = e_Quat_n * n_velocity;

    Eigen::Quaterniond n_Quat_p = n_Quat_b * b_Quat_p;
    CHECK_THAT(n_Quat_p.norm(), Catch::Matchers::WithinAbs(1.0, EPSILON));
    Eigen::Vector3d n_velocity_direct = n_Quat_p * v_p;

    Eigen::Quaterniond e_Quat_p = e_Quat_n * n_Quat_b * b_Quat_p;
    CHECK_THAT(e_Quat_p.norm(), Catch::Matchers::WithinAbs(1.0, EPSILON));
    Eigen::Vector3d v_e_direct = e_Quat_p * v_p;

    CHECK_THAT(n_velocity, Catch::Matchers::WithinAbs(n_velocity_direct, EPSILON));

    CHECK_THAT(e_v, Catch::Matchers::WithinAbs(v_e_direct, EPSILON));

    Eigen::Matrix3d e_Dcm_n = e_Quat_n.toRotationMatrix();
    Eigen::Matrix3d n_Dcm_b = n_Quat_b.toRotationMatrix();

    Eigen::Matrix3d e_Dcm_n_ref = ref::e_Dcm_n(latitude, longitude);
    Eigen::Matrix3d n_Dcm_b_ref = ref::n_Dcm_b(roll, pitch, yaw);

    CHECK_THAT(e_Dcm_n_ref, Catch::Matchers::WithinAbs(e_Dcm_n, 1e-13));
    CHECK_THAT(n_Dcm_b_ref, Catch::Matchers::WithinAbs(n_Dcm_b, 1e-13));

    Eigen::Matrix3d e_Dcm_b_ref = e_Dcm_n_ref * n_Dcm_b_ref;
    Eigen::Matrix3d e_Dcm_b = e_Dcm_n * n_Dcm_b;
    Eigen::Matrix3d e_Dcm_b_quat = (e_Quat_n * n_Quat_b).toRotationMatrix();

    CHECK_THAT(e_Dcm_b_quat, Catch::Matchers::WithinAbs(e_Dcm_b, 1e-13));
    CHECK_THAT(e_Dcm_b_ref, Catch::Matchers::WithinAbs(e_Dcm_b, 1e-13));
    CHECK_THAT(e_Dcm_b_ref, Catch::Matchers::WithinAbs(e_Dcm_b_quat, 1e-13));
}

} // namespace NAV::TESTS::CoordinateFramesTests
