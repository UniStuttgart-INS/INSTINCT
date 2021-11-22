#include <catch2/catch.hpp>
#include "EigenApprox.hpp"

#include "util/InsTransformations.hpp"
#include "util/Logger.hpp"

#include "util/Eigen.hpp"

#include <limits>

namespace NAV
{
constexpr double EPSILON = 10.0 * std::numeric_limits<double>::epsilon();

auto qCoeffsFromDcm(const Eigen::Matrix3d& C)
{
    auto a = 0.5 * std::sqrt(1 + C(0, 0) + C(1, 1) + C(2, 2));
    if (1 + C(0, 0) + C(1, 1) + C(2, 2) < 0)
    {
        fmt::print("Negative a {}\n", 1 + C(0, 0) + C(1, 1) + C(2, 2));
    }

    auto b = 1 / (4 * a) * (C(2, 1) - C(1, 2));
    auto c = 1 / (4 * a) * (C(0, 2) - C(2, 0));
    auto d = 1 / (4 * a) * (C(1, 0) - C(0, 1));

    return Eigen::Vector4d{ b, c, d, a };
}

Eigen::Matrix3d DCM_nb(double roll, double pitch, double yaw)
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

Eigen::Matrix3d DCM_en(double latitude, double longitude)
{
    Eigen::Matrix3d DCM;
    // clang-format off
    DCM << -sin(latitude)*cos(longitude), -sin(longitude), -cos(latitude)*cos(longitude),
           -sin(latitude)*sin(longitude),  cos(longitude), -cos(latitude)*sin(longitude),
                   cos(latitude)        ,        0       ,        -sin(latitude)        ;

    // clang-format on
    return DCM;
}

Eigen::Matrix3d DCM_ei(const double time, const double angularRate_ie)
{
    double a = angularRate_ie * time;

    Eigen::Matrix3d DCM;
    // clang-format off
    DCM <<  cos(a), sin(a), 0,
           -sin(a), cos(a), 0,
              0   ,   0   , 1;

    // clang-format on
    return DCM;
}

/// @brief Converts Earth-centered-Earth-fixed coordinates into latitude, longitude and altitude
/// @param[in] ecef Vector with coordinates in ECEF frame in [m]
/// @param[in] a Semi-major axis of the reference ellipsoid
/// @param[in] e_squared Square of the first eccentricity of the ellipsoid
/// @return Vector containing [latitude 𝜙, longitude λ, altitude]^T in [rad, rad, m]
/// @note See C. Jekeli, 2001, Inertial Navigation Systems with Geodetic Applications
Eigen::Vector3d ecef2lla_iter(const Eigen::Vector3d& ecef, double a = InsConst::WGS84_a, double e_squared = InsConst::WGS84_e_squared)
{
    // Value is used every iteration and does not change
    double sqrt_x1x1_x2x2 = std::sqrt(std::pow(ecef(0), 2) + std::pow(ecef(1), 2));

    // Latitude with initial assumption that h = 0 (eq. 1.85)
    double latitude = std::atan2(ecef(2) / (1 - e_squared), sqrt_x1x1_x2x2);

    double N{};
    size_t maxIterationCount = 6;
    for (size_t i = 0; i < maxIterationCount; i++) // Convergence should break the loop, but better limit the loop itself
    {
        // Radius of curvature of the ellipsoid in the prime vertical plane,
        // i.e., the plane containing the normal at P and perpendicular to the meridian (eq. 1.81)
        N = a / std::sqrt(1 - e_squared * std::pow(std::sin(latitude), 2));

        // Latitude (eq. 1.84)
        double newLatitude = std::atan2(ecef(2) + e_squared * N * std::sin(latitude), sqrt_x1x1_x2x2);

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
    double longitude = std::atan2(ecef(1), ecef(0));
    // Altitude (eq. 1.84)
    double altitude = sqrt_x1x1_x2x2 / std::cos(latitude);
    altitude -= N;

    return Eigen::Vector3d(latitude, longitude, altitude);
}

TEST_CASE("[InsTransformations] Degree to radian conversion", "[InsTransformations]")
{
    double rad_90 = trafo::deg2rad(90);
    double rad_180 = trafo::deg2rad(180.0);
    double rad_360 = trafo::deg2rad(360.0F);

    REQUIRE(rad_90 == M_PI_2);
    REQUIRE(rad_180 == M_PI);
    REQUIRE(rad_360 == M_PI * 2.0);

    Eigen::Vector3d rad3 = trafo::deg2rad3({ 90, 180, 360 });

    REQUIRE(rad3.x() == M_PI_2);
    REQUIRE(rad3.y() == M_PI);
    REQUIRE(rad3.z() == M_PI * 2.0);
}

TEST_CASE("[InsTransformations] Degree to radian conversion constexpr", "[InsTransformations]")
{
    constexpr double rad_90 = trafo::deg2rad(90);
    constexpr double rad_180 = trafo::deg2rad(180.0);
    constexpr double rad_360 = trafo::deg2rad(360.0F);

    STATIC_REQUIRE(rad_90 == M_PI_2);
    STATIC_REQUIRE(rad_180 == M_PI);
    STATIC_REQUIRE(rad_360 == M_PI * 2.0);
}

TEST_CASE("[InsTransformations] Radian to degree conversion", "[InsTransformations]")
{
    double deg_90 = trafo::rad2deg(M_PI_2);
    double deg_180 = trafo::rad2deg(M_PI);
    double deg_360 = trafo::rad2deg(M_PI * 2.0);

    REQUIRE(deg_90 == 90);
    REQUIRE(deg_180 == 180);
    REQUIRE(deg_360 == 360);

    Eigen::Vector3d deg3 = trafo::rad2deg3({ M_PI_2, M_PI, M_PI * 2.0 });

    REQUIRE(deg3.x() == 90);
    REQUIRE(deg3.y() == 180);
    REQUIRE(deg3.z() == 360);
}

TEST_CASE("[InsTransformations] Radian to degree conversion constexpr", "[InsTransformations]")
{
    constexpr double deg_90 = trafo::rad2deg(M_PI_2);
    constexpr double deg_180 = trafo::rad2deg(M_PI);
    constexpr double deg_360 = trafo::rad2deg(M_PI * 2.0);

    STATIC_REQUIRE(deg_90 == 90.0);
    STATIC_REQUIRE(deg_180 == 180.0);
    STATIC_REQUIRE(deg_360 == 360.0);
}

TEST_CASE("[InsTransformations] Euler to Quaternion conversion", "[InsTransformations]")
{
    double delta = trafo::deg2rad(5);
    // (-pi:pi] x (-pi/2:pi/2] x (-pi:pi]
    for (double roll = -M_PI + delta; roll < M_PI - std::numeric_limits<float>::epsilon(); roll += delta) // NOLINT(clang-analyzer-security.FloatLoopCounter,cert-flp30-c)
    {
        Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());

        auto q = Eigen::Quaterniond{ rollAngle };
        Eigen::Matrix3d C = q.toRotationMatrix();

        fmt::print("Roll: {}\n", trafo::rad2deg(roll));
        REQUIRE(qCoeffsFromDcm(C) == EigApprox(q.coeffs()).margin(1e-12).epsilon(0));
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

            fmt::print("Roll, Pitch: {}, {}\n", trafo::rad2deg(roll), trafo::rad2deg(pitch));
            REQUIRE(qCoeffsFromDcm(C) == EigApprox(q.coeffs()).margin(1e-12).epsilon(0));
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

                fmt::print("Roll, Pitch, Yaw: {}, {}, {}\n", trafo::rad2deg(roll), trafo::rad2deg(pitch), trafo::rad2deg(yaw));
                fmt::print("DCM\n{}\n", C);

                // Check if in our notation the scalar quaternion is always positive
                REQUIRE(qCoeffsFromDcm(C)(3) > 0);
                if (q.w() < 0)
                {
                    Eigen::Vector4d qCoeffsNeg = -q.coeffs();
                    REQUIRE(qCoeffsFromDcm(C) == EigApprox(qCoeffsNeg).margin(1e-6).epsilon(0));
                }
                else
                {
                    REQUIRE(qCoeffsFromDcm(C) == EigApprox(q.coeffs()).margin(1e-6).epsilon(0));
                }
            }
        }
    }
}

TEST_CASE("[InsTransformations] Quaternion to Euler conversion", "[InsTransformations]")
{
    auto quat = [](double alpha, double beta, double gamma) {
        Eigen::AngleAxisd xAngle(alpha, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd yAngle(beta, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd zAngle(gamma, Eigen::Vector3d::UnitZ());

        return zAngle * yAngle * xAngle;
    };

    double delta = trafo::deg2rad(5);
    // (-pi:pi] x (-pi/2:pi/2] x (-pi:pi]
    for (double alpha = -M_PI + delta; alpha <= M_PI; alpha += delta) // NOLINT(clang-analyzer-security.FloatLoopCounter,cert-flp30-c)
    {
        for (double beta = -M_PI / 2.0 + delta; beta <= M_PI / 2.0; beta += delta) // NOLINT(clang-analyzer-security.FloatLoopCounter,cert-flp30-c)
        {
            for (double gamma = -M_PI + delta; gamma <= M_PI; gamma += delta) // NOLINT(clang-analyzer-security.FloatLoopCounter,cert-flp30-c)
            {
                auto q = quat(alpha, beta, gamma);
                auto ZYX = trafo::rad2deg3(trafo::quat2eulerZYX(q));
                REQUIRE(ZYX == EigApprox(trafo::rad2deg3(Eigen::Vector3d{ alpha, beta, gamma })).margin(1e-8).epsilon(0));
            }
        }
    }
}

TEST_CASE("[InsTransformations] Negated Quaternion to Euler conversion", "[InsTransformations]")
{
    auto quat = [](double alpha, double beta, double gamma) {
        Eigen::AngleAxisd xAngle(alpha, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd yAngle(beta, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd zAngle(gamma, Eigen::Vector3d::UnitZ());

        return zAngle * yAngle * xAngle;
    };

    double delta = trafo::deg2rad(5);
    // (-pi:pi] x (-pi/2:pi/2] x (-pi:pi]
    for (double alpha = -M_PI + delta; alpha <= M_PI; alpha += delta) // NOLINT(clang-analyzer-security.FloatLoopCounter,cert-flp30-c)
    {
        for (double beta = -M_PI / 2.0 + delta; beta <= M_PI / 2.0; beta += delta) // NOLINT(clang-analyzer-security.FloatLoopCounter,cert-flp30-c)
        {
            for (double gamma = -M_PI + delta; gamma <= M_PI; gamma += delta) // NOLINT(clang-analyzer-security.FloatLoopCounter,cert-flp30-c)
            {
                auto q = quat(alpha, beta, gamma);
                if (q.w() < 0)
                {
                    q.coeffs() = -q.coeffs();
                }

                auto ZYX = trafo::rad2deg3(trafo::quat2eulerZYX(q));
                REQUIRE(ZYX == EigApprox(trafo::rad2deg3(Eigen::Vector3d{ alpha, beta, gamma })).margin(1e-8).epsilon(0));
            }
        }
    }
}

TEST_CASE("[InsTransformations] Inertial <=> Earth-fixed frame conversion", "[InsTransformations]")
{
    double time = 86164.099 / 3.0;
    auto q_ei = trafo::quat_ei(time, InsConst::angularVelocity_ie);
    CHECK(q_ei.norm() == Approx(1.0).margin(EPSILON).epsilon(0));

    auto C_ei = q_ei.toRotationMatrix();
    auto C_ei_ref = DCM_ei(time, InsConst::angularVelocity_ie);

    CHECK(C_ei == EigApprox(C_ei_ref).margin(EPSILON).epsilon(0));

    /* -------------------------------------------------------------------------------------------------------- */

    // Sidereal day: 23h 56min 4.099s
    auto siderialDay4 = 86164.099 / 4.0;
    q_ei = trafo::quat_ei(siderialDay4, InsConst::angularVelocity_ie);
    CHECK(q_ei.norm() == Approx(1.0).margin(EPSILON).epsilon(0));
    // Star day: 23h 56min 4.0905s
    // auto starHalfDay = 86164.0905 / 2.0;
    // auto q_ei = trafo::quat_ei(starHalfDay);

    Eigen::Vector3d x_i{ 1, -2.5, 22 };
    Eigen::Vector3d x_e = q_ei * x_i;

    CHECK(x_e == EigApprox(Eigen::Vector3d{ -2.5, -1, 22 }).margin(1e-8).epsilon(0));

    auto q_ie = trafo::quat_ie(siderialDay4, InsConst::angularVelocity_ie);
    CHECK(q_ie.norm() == Approx(1.0).margin(EPSILON).epsilon(0));

    auto q_identity = q_ie * q_ei;
    CHECK(q_identity == EigApproxQ(Eigen::Quaterniond::Identity()).margin(EPSILON).epsilon(0));
}

TEST_CASE("[InsTransformations] Navigation <=> Earth-fixed frame conversion", "[InsTransformations]")
{
    double latitude = trafo::deg2rad(88);
    double longitude = trafo::deg2rad(-40);

    auto q_en = trafo::quat_en(latitude, longitude);
    CHECK(q_en.norm() == Approx(1.0).margin(EPSILON).epsilon(0));

    auto C_en = q_en.toRotationMatrix();
    auto C_en_ref = DCM_en(latitude, longitude);

    CHECK(C_en == EigApprox(C_en_ref).margin(EPSILON).epsilon(0));

    /* -------------------------------------------------------------------------------------------------------- */

    latitude = trafo::deg2rad(90);
    longitude = 0.0;

    auto q_ne = trafo::quat_ne(latitude, longitude);
    CHECK(q_ne.norm() == Approx(1.0).margin(EPSILON).epsilon(0));

    Eigen::Vector3d x_e{ 1, 2, 3 };
    auto x_n = q_ne * x_e;

    CHECK(x_n == EigApprox(Eigen::Vector3d{ -1, 2, -3 }));

    /* -------------------------------------------------------------------------------------------------------- */

    latitude = trafo::deg2rad(0);
    longitude = trafo::deg2rad(0);

    q_ne = trafo::quat_ne(latitude, longitude);
    CHECK(q_ne.norm() == Approx(1.0).margin(EPSILON).epsilon(0));

    x_e = { 1, 2, 3 };
    x_n = q_ne * x_e;

    CHECK(x_n == EigApprox(Eigen::Vector3d{ 3, 2, -1 }));

    /* -------------------------------------------------------------------------------------------------------- */

    latitude = trafo::deg2rad(0);
    longitude = trafo::deg2rad(0);

    q_ne = trafo::quat_ne(latitude, longitude);
    CHECK(q_ne.norm() == Approx(1.0).margin(EPSILON).epsilon(0));
    //                    (0, 0, 7.2921151467e-05)
    x_n = q_ne * InsConst::angularVelocity_ie_e;

    CHECK(x_n == EigApprox(Eigen::Vector3d{ InsConst::angularVelocity_ie, 0, 0 }).margin(EPSILON));

    /* -------------------------------------------------------------------------------------------------------- */

    latitude = trafo::deg2rad(45);
    longitude = trafo::deg2rad(90);

    q_ne = trafo::quat_ne(latitude, longitude);
    CHECK(q_ne.norm() == Approx(1.0).margin(EPSILON).epsilon(0));
    //                    (0, 0, 7.2921151467e-05)
    x_n = q_ne * InsConst::angularVelocity_ie_e;

    CHECK(x_n == EigApprox(Eigen::Vector3d{ InsConst::angularVelocity_ie / std::sqrt(2), 0, -InsConst::angularVelocity_ie / std::sqrt(2) }).margin(EPSILON));
}

TEST_CASE("[InsTransformations] NED <=> Earth-centered-earth-fixed frame conversion", "[InsTransformations]")
{
    double latitude_ref = trafo::deg2rad(88);
    double longitude_ref = trafo::deg2rad(-40);
    double altitude_ref = 500;
    auto position_e_ref = trafo::lla2ecef_WGS84({ latitude_ref, longitude_ref, altitude_ref });

    auto position_n_ref = trafo::ecef2ned(position_e_ref, { latitude_ref, longitude_ref, altitude_ref });

    CHECK(position_n_ref(0) == 0);
    CHECK(position_n_ref(1) == 0);
    CHECK(position_n_ref(2) == 0);

    auto position_e = trafo::lla2ecef_WGS84({ latitude_ref, longitude_ref, altitude_ref + 200 });
    auto position_n = trafo::ecef2ned(position_e, { latitude_ref, longitude_ref, altitude_ref });

    CHECK(position_n == EigApprox(Eigen::Vector3d{ 0, 0, -200 }).margin(1e-9).epsilon(0));

    /* -------------------------------------------------------------------------------------------------------- */

    position_e = trafo::ned2ecef(position_n_ref, { latitude_ref, longitude_ref, altitude_ref });

    CHECK(position_e == position_e_ref);

    /* -------------------------------------------------------------------------------------------------------- */

    position_e_ref = trafo::lla2ecef_WGS84({ trafo::deg2rad(-10), trafo::deg2rad(70), 2001 });
    position_e = position_e_ref;
    for (size_t i = 0; i < 10000; i++)
    {
        position_n = trafo::ecef2ned(position_e, { latitude_ref, longitude_ref, altitude_ref });
        position_e = trafo::ned2ecef(position_n, { latitude_ref, longitude_ref, altitude_ref });
    }

    CHECK(position_e == EigApprox(position_e_ref));

    /* -------------------------------------------------------------------------------------------------------- */
}

TEST_CASE("[InsTransformations] Body <=> navigation DCM/Quaternion comparison", "[InsTransformations]")
{
    double delta = trafo::deg2rad(5);
    // (-pi:pi] x (-pi/2:pi/2] x (-pi:pi]
    for (double roll = -M_PI + delta; roll <= M_PI + std::numeric_limits<float>::epsilon(); roll += delta) // NOLINT(clang-analyzer-security.FloatLoopCounter,cert-flp30-c)
    {
        for (double pitch = -M_PI / 2.0 + delta; pitch <= M_PI / 2.0 + std::numeric_limits<float>::epsilon(); pitch += delta) // NOLINT(clang-analyzer-security.FloatLoopCounter,cert-flp30-c)
        {
            for (double yaw = -M_PI + delta; yaw <= M_PI + std::numeric_limits<float>::epsilon(); yaw += delta) // NOLINT(clang-analyzer-security.FloatLoopCounter,cert-flp30-c)
            {
                auto q_nb = trafo::quat_nb(roll, pitch, yaw);
                REQUIRE(q_nb.norm() == Approx(1.0).margin(EPSILON).epsilon(0));

                auto C_nb = q_nb.toRotationMatrix();
                auto C_nb_ref = DCM_nb(roll, pitch, yaw);

                REQUIRE(C_nb == EigApprox(C_nb_ref).margin(1e-13).epsilon(0));
            }
        }
    }
}

TEST_CASE("[InsTransformations] Body <=> navigation frame conversion", "[InsTransformations]")
{
    /* -------------------------------------------------------------------------------------------------------- */

    double roll = trafo::deg2rad(45);
    double pitch = 0.0;
    double yaw = 0.0;
    auto q_bn = trafo::quat_bn(roll, pitch, yaw);
    CHECK(q_bn.norm() == Approx(1.0).margin(EPSILON).epsilon(0));

    Eigen::Vector3d x_n{ 1.0, 1.0, 1.0 };
    Eigen::Vector3d x_b = q_bn * x_n;

    CHECK(x_b == EigApprox(Eigen::Vector3d{ 1, std::sqrt(2), 0 }).margin(EPSILON).epsilon(0));

    /* -------------------------------------------------------------------------------------------------------- */

    roll = 0.0;
    pitch = trafo::deg2rad(45);
    yaw = 0.0;
    q_bn = trafo::quat_bn(roll, pitch, yaw);
    CHECK(q_bn.norm() == Approx(1.0).margin(EPSILON).epsilon(0));

    x_n = { 1.0, 1.0, 1.0 };
    x_b = q_bn * x_n;

    CHECK(x_b == EigApprox(Eigen::Vector3d{ 0, 1, std::sqrt(2) }).margin(EPSILON).epsilon(0));

    /* -------------------------------------------------------------------------------------------------------- */

    roll = 0.0;
    pitch = 0.0;
    yaw = trafo::deg2rad(-45);
    q_bn = trafo::quat_bn(roll, pitch, yaw);
    CHECK(q_bn.norm() == Approx(1.0).margin(EPSILON).epsilon(0));

    x_n = Eigen::Vector3d{ 1.0, 1.0, 0.0 };
    x_b = q_bn * x_n;

    CHECK(x_b == EigApprox(Eigen::Vector3d{ 0, std::sqrt(2), 0 }).margin(EPSILON).epsilon(0));

    /* -------------------------------------------------------------------------------------------------------- */

    roll = 0.0;
    pitch = trafo::deg2rad(90);
    yaw = trafo::deg2rad(90);
    q_bn = trafo::quat_bn(roll, pitch, yaw);
    CHECK(q_bn.norm() == Approx(1.0).margin(EPSILON).epsilon(0));

    x_n = Eigen::Vector3d{ 1.0, 2.0, 3.0 };
    x_b = q_bn * x_n;

    CHECK(x_b == EigApprox(Eigen::Vector3d{ -3, -1, 2 }).margin(EPSILON).epsilon(0));

    /* -------------------------------------------------------------------------------------------------------- */

    roll = trafo::deg2rad(90);
    pitch = trafo::deg2rad(90);
    yaw = 0.0;
    q_bn = trafo::quat_bn(roll, pitch, yaw);
    CHECK(q_bn.norm() == Approx(1.0).margin(EPSILON).epsilon(0));

    x_n = Eigen::Vector3d{ 1.0, 2.0, 3.0 };
    x_b = q_bn * x_n;

    CHECK(x_b == EigApprox(Eigen::Vector3d{ -3, 1, -2 }).margin(EPSILON).epsilon(0));

    /* -------------------------------------------------------------------------------------------------------- */

    roll = trafo::deg2rad(90);
    pitch = trafo::deg2rad(180);
    yaw = trafo::deg2rad(90);
    q_bn = trafo::quat_bn(roll, pitch, yaw);
    CHECK(q_bn.norm() == Approx(1.0).margin(EPSILON).epsilon(0));

    x_n = { 1.0, 2.0, 3.0 };
    x_b = q_bn * x_n;

    CHECK(x_b == EigApprox(Eigen::Vector3d{ -2, -3, 1 }).margin(EPSILON).epsilon(0));
}

TEST_CASE("[InsTransformations] Platform <=> body frame conversion", "[InsTransformations]")
{
    double mountingAngleX = trafo::deg2rad(90);
    double mountingAngleY = 0.0;
    double mountingAngleZ = trafo::deg2rad(-90);

    auto q_bp = trafo::quat_bp(mountingAngleX, mountingAngleY, mountingAngleZ);
    CHECK(q_bp.norm() == Approx(1.0).margin(EPSILON).epsilon(0));

    Eigen::Vector3d x_p{ 2.0, 0.0, 9.81 };
    Eigen::Vector3d x_b = q_bp * x_p;

    CHECK(x_b == EigApprox(Eigen::Vector3d{ x_p(1), x_p(2), -x_p(0) }).margin(EPSILON).epsilon(0));
}

TEST_CASE("[InsTransformations] LLA <=> ECEF conversion", "[InsTransformations]")
{
    // Conversion with https://www.oc.nps.edu/oc2902w/coord/llhxyz.htm

    // Stuttgart, Breitscheidstraße 2
    // https://www.koordinaten-umrechner.de/decimal/48.780810,9.172012?karte=OpenStreetMap&zoom=19
    double latitude = trafo::deg2rad(48.78081);
    double longitude = trafo::deg2rad(9.172012);
    double altitude = 254;
    Eigen::Vector3d ecef_ref = Eigen::Vector3d(4157.128, 671.224, 4774.723) * 1000;
    Eigen::Vector3d ecef = trafo::lla2ecef_WGS84({ latitude, longitude, altitude });
    Eigen::Vector3d lla = trafo::ecef2lla_WGS84(ecef_ref);
    CHECK(ecef == EigApprox(ecef_ref).margin(0.3).epsilon(0));
    CHECK(lla.x() == Approx(latitude).margin(1e-7).epsilon(0));
    CHECK(lla.y() == Approx(longitude).margin(1e-8).epsilon(0));
    CHECK(lla.z() == Approx(altitude).margin(0.5).epsilon(0));

    /* -------------------------------------------------------------------------- */

    // New York
    // https://www.koordinaten-umrechner.de/decimal/40.712728,-74.006015?karte=OpenStreetMap&zoom=4
    latitude = trafo::deg2rad(40.712728);
    longitude = trafo::deg2rad(-74.006015);
    altitude = 13;
    ecef_ref = Eigen::Vector3d(1334.001, -4654.06, 4138.303) * 1000;
    ecef = trafo::lla2ecef_WGS84({ latitude, longitude, altitude });
    lla = trafo::ecef2lla_WGS84(ecef_ref);
    CHECK(ecef == EigApprox(ecef_ref).margin(0.5).epsilon(0));
    CHECK(lla.x() == Approx(latitude).margin(1e-7).epsilon(0));
    CHECK(lla.y() == Approx(longitude).margin(1e-7).epsilon(0));
    CHECK(lla.z() == Approx(altitude).margin(0.5).epsilon(0));

    /* -------------------------------------------------------------------------- */

    latitude = 0;
    longitude = 0;
    altitude = -3492;
    ecef_ref = Eigen::Vector3d(6374.645, 0, 0) * 1000;
    ecef = trafo::lla2ecef_WGS84({ latitude, longitude, altitude });
    lla = trafo::ecef2lla_WGS84(ecef_ref);
    CHECK(ecef == EigApprox(ecef_ref).margin(EPSILON).epsilon(0));
    CHECK(lla.x() == Approx(latitude).margin(EPSILON).epsilon(0));
    CHECK(lla.y() == Approx(longitude).margin(EPSILON).epsilon(0));
    CHECK(lla.z() == Approx(altitude).margin(1e-9).epsilon(0));

    /* -------------------------------------------------------------------------- */

    latitude = trafo::deg2rad(-89.9999);
    longitude = trafo::deg2rad(0);
    altitude = 2801;
    ecef_ref = Eigen::Vector3d(0.011, 0, -6359.553) * 1000;
    ecef = trafo::lla2ecef_WGS84({ latitude, longitude, altitude });
    lla = trafo::ecef2lla_WGS84(ecef_ref);
    CHECK(ecef == EigApprox(ecef_ref).margin(0.4).epsilon(0));
    CHECK(lla.x() == Approx(latitude).margin(1e-6).epsilon(0));
    CHECK(lla.y() == Approx(longitude).margin(EPSILON).epsilon(0));
    CHECK(lla.z() == Approx(altitude).margin(0.5).epsilon(0));

    /* -------------------------------------------------------------------------- */

    latitude = trafo::deg2rad(40);
    longitude = trafo::deg2rad(180);
    altitude = -5097;
    ecef_ref = Eigen::Vector3d(-4888.803, 0, 4074.709) * 1000;
    ecef = trafo::lla2ecef_WGS84({ latitude, longitude, altitude });
    lla = trafo::ecef2lla_WGS84(ecef_ref);
    CHECK(ecef.x() == Approx(ecef_ref.x()));
    CHECK(ecef.y() == Approx(ecef_ref.y()).margin(1e-9).epsilon(0));
    CHECK(ecef.z() == Approx(ecef_ref.z()));
    CHECK(lla.x() == Approx(latitude).margin(1e-6).epsilon(0));
    CHECK(lla.y() == Approx(longitude).margin(EPSILON).epsilon(0));
    CHECK(lla.z() == Approx(altitude).margin(0.5).epsilon(0));

    /* -------------------------------------------------------------------------- */

    latitude = trafo::deg2rad(40);
    longitude = trafo::deg2rad(180);
    altitude = -5097;
    lla = { latitude, longitude, altitude };
    for (size_t i = 0; i < 100000; i++)
    {
        lla = trafo::ecef2lla_WGS84(trafo::lla2ecef_WGS84(lla));
    }
    CHECK(lla.x() == Approx(latitude).margin(EPSILON).epsilon(0));
    CHECK(lla.y() == Approx(longitude).margin(EPSILON).epsilon(0));
    CHECK(lla.z() == Approx(altitude).margin(1e-8).epsilon(0));
}

TEST_CASE("[InsTransformations] LLA => ECEF => LLA conversion", "[InsTransformations]")
{
    for (int lat = -89; lat < 90; lat += 1)
    {
        for (int lon = -179; lon < 180; lon += 1)
        {
            for (int alt = -10000; alt <= 100000; alt += 10000)
            {
                double latitude = trafo::deg2rad(lat);
                double longitude = trafo::deg2rad(lon);
                double altitude = alt;
                Eigen::Vector3d lla{ latitude, longitude, altitude };
                for (size_t i = 0; i < 10; i++)
                {
                    lla = trafo::ecef2lla_WGS84(trafo::lla2ecef_WGS84(lla));
                }
                REQUIRE(lla.x() == Approx(latitude).margin(1e-14).epsilon(0));
                REQUIRE(lla.y() == Approx(longitude).margin(1e-14).epsilon(0));
                REQUIRE(lla.z() == Approx(altitude).margin(1e-7).epsilon(0));
            }
        }
    }
}

TEST_CASE("[InsTransformations] LLA => ECEF => LLH-iterative conversion", "[InsTransformations]")
{
    for (int lat = -89; lat < 90; lat += 1)
    {
        for (int lon = -179; lon < 180; lon += 1)
        {
            for (int alt = -10000; alt <= 100000; alt += 10000)
            {
                double latitude = trafo::deg2rad(lat);
                double longitude = trafo::deg2rad(lon);
                double altitude = alt;
                Eigen::Vector3d lla{ latitude, longitude, altitude };
                for (size_t i = 0; i < 10; i++)
                {
                    lla = ecef2lla_iter(trafo::lla2ecef_WGS84(lla));
                }
                REQUIRE(lla.x() == Approx(latitude).margin(1e-14).epsilon(0));
                REQUIRE(lla.y() == Approx(longitude).margin(1e-14).epsilon(0));
                REQUIRE(lla.z() == Approx(altitude).margin(1e-7).epsilon(0));
            }
        }
    }

    /* -------------------------------------------------------------------------------------------------------- */

    double latitude = trafo::deg2rad(90);
    double longitude = trafo::deg2rad(0);
    double altitude = 0;
    auto ecef = trafo::lla2ecef_WGS84({ latitude, longitude, altitude });
    auto lla_iter = ecef2lla_iter(ecef, InsConst::WGS84_a, InsConst::WGS84_e_squared);

    CHECK(lla_iter.x() == Approx(latitude).margin(EPSILON).epsilon(0));
    CHECK(lla_iter.y() == Approx(longitude).margin(EPSILON).epsilon(0));
    CHECK(lla_iter.z() == Approx(altitude).margin(1e-9).epsilon(0));

    /* -------------------------------------------------------------------------------------------------------- */

    latitude = trafo::deg2rad(90);
    longitude = trafo::deg2rad(180);
    altitude = 0;
    ecef = trafo::lla2ecef_WGS84({ latitude, longitude, altitude });
    lla_iter = ecef2lla_iter(ecef, InsConst::WGS84_a, InsConst::WGS84_e_squared);

    CHECK(lla_iter.x() == Approx(latitude).margin(EPSILON).epsilon(0));
    CHECK(lla_iter.y() == Approx(longitude).margin(EPSILON).epsilon(0));
    CHECK(lla_iter.z() == Approx(altitude).margin(1e-9).epsilon(0));
}

TEST_CASE("[InsTransformations] Transformation chains", "[InsTransformations]")
{
    double roll = trafo::deg2rad(20);
    double pitch = trafo::deg2rad(50);
    double yaw = trafo::deg2rad(190);

    double latitude = trafo::deg2rad(10);
    double longitude = trafo::deg2rad(40);

    Eigen::Quaterniond q_bp(Eigen::AngleAxisd(trafo::deg2rad(-90), Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond q_nb = trafo::quat_nb(roll, pitch, yaw);
    Eigen::Quaterniond q_en = trafo::quat_en(latitude, longitude);

    CHECK(q_bp.norm() == Approx(1.0).margin(EPSILON).epsilon(0));
    CHECK(q_nb.norm() == Approx(1.0).margin(EPSILON).epsilon(0));
    CHECK(q_en.norm() == Approx(1.0).margin(EPSILON).epsilon(0));

    Eigen::Vector3d v_p{ 1, 3, 5 };

    Eigen::Vector3d v_b = q_bp * v_p;
    Eigen::Vector3d v_n = q_nb * v_b;
    Eigen::Vector3d v_e = q_en * v_n;

    Eigen::Quaterniond q_np = q_nb * q_bp;
    CHECK(q_np.norm() == Approx(1.0).margin(EPSILON).epsilon(0));
    Eigen::Vector3d v_n_direct = q_np * v_p;

    Eigen::Quaterniond q_ep = q_en * q_nb * q_bp;
    CHECK(q_ep.norm() == Approx(1.0).margin(EPSILON).epsilon(0));
    Eigen::Vector3d v_e_direct = q_ep * v_p;

    CHECK(v_n.x() == Approx(v_n_direct.x()).margin(EPSILON).epsilon(0));
    CHECK(v_n.y() == Approx(v_n_direct.y()).margin(EPSILON).epsilon(0));
    CHECK(v_n.z() == Approx(v_n_direct.z()).margin(EPSILON).epsilon(0));

    CHECK(v_e.x() == Approx(v_e_direct.x()).margin(EPSILON).epsilon(0));
    CHECK(v_e.y() == Approx(v_e_direct.y()).margin(EPSILON).epsilon(0));
    CHECK(v_e.z() == Approx(v_e_direct.z()).margin(EPSILON).epsilon(0));

    Eigen::Matrix3d dcm_en = q_en.toRotationMatrix();
    Eigen::Matrix3d dcm_nb = q_nb.toRotationMatrix();

    Eigen::Matrix3d dcm_en_ref = DCM_en(latitude, longitude);
    Eigen::Matrix3d dcm_nb_ref = DCM_nb(roll, pitch, yaw);

    CHECK(dcm_en_ref == EigApprox(dcm_en).margin(1e-13).epsilon(0));
    CHECK(dcm_nb_ref == EigApprox(dcm_nb).margin(1e-13).epsilon(0));

    Eigen::Matrix3d dcm_eb_ref = dcm_en_ref * dcm_nb_ref;
    Eigen::Matrix3d dcm_eb = dcm_en * dcm_nb;
    Eigen::Matrix3d dcm_eb_quat = (q_en * q_nb).toRotationMatrix();

    CHECK(dcm_eb_quat == EigApprox(dcm_eb).margin(1e-13).epsilon(0));
    CHECK(dcm_eb_ref == EigApprox(dcm_eb).margin(1e-13).epsilon(0));
    CHECK(dcm_eb_ref == EigApprox(dcm_eb_quat).margin(1e-13).epsilon(0));
}

} // namespace NAV
