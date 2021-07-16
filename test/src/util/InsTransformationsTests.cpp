#include <catch2/catch.hpp>

#include "util/InsTransformations.hpp"
#include "util/Logger.hpp"

#include "util/Eigen.hpp"

#include <limits>

namespace NAV
{
constexpr double EPSILON = 10.0 * std::numeric_limits<double>::epsilon();

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
Eigen::Vector3d ecef2lla_iter(const Eigen::Vector3d& ecef, double a, double e_squared)
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

TEST_CASE("[InsTransformations] Quaternion to Euler conversion", "[InsTransformations]")
{
    auto quat = [](double alpha, double beta, double gamma) {
        Eigen::AngleAxisd xAngle(alpha, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd yAngle(beta, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd zAngle(gamma, Eigen::Vector3d::UnitZ());

        return zAngle * yAngle * xAngle;
    };

    double delta = trafo::rad2deg(0.01);
    bool error = false;
    // (-pi:pi]x(-pi/2:pi/2]x(-pi:pi]
    for (double alpha = -M_PI + delta; alpha <= M_PI && !error; alpha += delta) // NOLINT(clang-analyzer-security.FloatLoopCounter,cert-flp30-c)
    {
        for (double beta = -M_PI / 2.0 + delta; beta <= M_PI / 2.0 && !error; beta += delta) // NOLINT(clang-analyzer-security.FloatLoopCounter,cert-flp30-c)
        {
            for (double gamma = -M_PI + delta; gamma <= M_PI && !error; gamma += delta) // NOLINT(clang-analyzer-security.FloatLoopCounter,cert-flp30-c)
            {
                auto q = quat(alpha, beta, gamma);
                auto ZYX = trafo::rad2deg3(trafo::quat2eulerZYX(q));
                CHECK(ZYX.x() == Approx(trafo::rad2deg(alpha)).margin(1e-8));
                CHECK(ZYX.y() == Approx(trafo::rad2deg(beta)).margin(1e-8));
                CHECK(ZYX.z() == Approx(trafo::rad2deg(gamma)).margin(1e-8));
                if (std::abs(ZYX.x() - trafo::rad2deg(alpha)) >= 1e-8
                    || std::abs(ZYX.y() - trafo::rad2deg(beta)) >= 1e-8
                    || std::abs(ZYX.z() - trafo::rad2deg(gamma)) >= 1e-8)
                {
                    error = true;
                }
            }
        }
    }
}

TEST_CASE("[InsTransformations] Inertial <=> Earth-fixed frame conversion", "[InsTransformations]")
{
    double time = 86164.099 / 3.0;
    auto C_ei = trafo::quat_ei(time, InsConst::angularVelocity_ie).toRotationMatrix();
    auto C_ei_ref = DCM_ei(time, InsConst::angularVelocity_ie);

    CHECK(C_ei(0, 0) == Approx(C_ei_ref(0, 0)));
    CHECK(C_ei(0, 1) == Approx(C_ei_ref(0, 1)));
    CHECK(C_ei(0, 2) == Approx(C_ei_ref(0, 2)));
    CHECK(C_ei(1, 0) == Approx(C_ei_ref(1, 0)));
    CHECK(C_ei(1, 1) == Approx(C_ei_ref(1, 1)));
    CHECK(C_ei(1, 2) == Approx(C_ei_ref(1, 2)));
    CHECK(C_ei(2, 0) == Approx(C_ei_ref(2, 0)));
    CHECK(C_ei(2, 1) == Approx(C_ei_ref(2, 1)));
    CHECK(C_ei(2, 2) == Approx(C_ei_ref(2, 2)));

    /* -------------------------------------------------------------------------------------------------------- */

    // Sidereal day: 23h 56min 4.099s
    auto siderialDay4 = 86164.099 / 4.0;
    auto q_ei = trafo::quat_ei(siderialDay4, InsConst::angularVelocity_ie);
    // Star day: 23h 56min 4.0905s
    // auto starHalfDay = 86164.0905 / 2.0;
    // auto q_ei = trafo::quat_ei(starHalfDay);

    Eigen::Vector3d x_i{ 1, -2.5, 22 };
    Eigen::Vector3d x_e = q_ei * x_i;

    CHECK(x_e.x() == Approx(-2.5).margin(0.000001));
    CHECK(x_e.y() == Approx(-1).margin(0.000001));
    CHECK(x_e.z() == Approx(22.0).margin(0.000001));

    auto q_ie = trafo::quat_ie(siderialDay4, InsConst::angularVelocity_ie);

    auto q_identity = q_ie * q_ei;
    CHECK(q_identity.coeffs()(0) == Approx(Eigen::Quaterniond::Identity().coeffs()(0)).margin(EPSILON));
    CHECK(q_identity.coeffs()(1) == Approx(Eigen::Quaterniond::Identity().coeffs()(1)).margin(EPSILON));
    CHECK(q_identity.coeffs()(2) == Approx(Eigen::Quaterniond::Identity().coeffs()(2)).margin(EPSILON));
    CHECK(q_identity.coeffs()(3) == Approx(Eigen::Quaterniond::Identity().coeffs()(3)).margin(EPSILON));
}

TEST_CASE("[InsTransformations] Navigation <=> Earth-fixed frame conversion", "[InsTransformations]")
{
    double latitude = trafo::deg2rad(88);
    double longitude = trafo::deg2rad(-40);

    auto C_en = trafo::quat_en(latitude, longitude).toRotationMatrix();
    auto C_en_ref = DCM_en(latitude, longitude);

    CHECK(C_en(0, 0) == Approx(C_en_ref(0, 0)).margin(EPSILON));
    CHECK(C_en(0, 1) == Approx(C_en_ref(0, 1)).margin(EPSILON));
    CHECK(C_en(0, 2) == Approx(C_en_ref(0, 2)).margin(EPSILON));
    CHECK(C_en(1, 0) == Approx(C_en_ref(1, 0)).margin(EPSILON));
    CHECK(C_en(1, 1) == Approx(C_en_ref(1, 1)).margin(EPSILON));
    CHECK(C_en(1, 2) == Approx(C_en_ref(1, 2)).margin(EPSILON));
    CHECK(C_en(2, 0) == Approx(C_en_ref(2, 0)).margin(EPSILON));
    CHECK(C_en(2, 1) == Approx(C_en_ref(2, 1)).margin(EPSILON));
    CHECK(C_en(2, 2) == Approx(C_en_ref(2, 2)).margin(EPSILON));

    /* -------------------------------------------------------------------------------------------------------- */

    latitude = trafo::deg2rad(90);
    longitude = 0.0;

    auto q_ne = trafo::quat_ne(latitude, longitude);

    Eigen::Vector3d x_e{ 1, 2, 3 };
    auto x_n = q_ne * x_e;

    CHECK(x_n.x() == Approx(-1.0));
    CHECK(x_n.y() == Approx(2.0));
    CHECK(x_n.z() == Approx(-3.0));

    /* -------------------------------------------------------------------------------------------------------- */

    latitude = trafo::deg2rad(0);
    longitude = trafo::deg2rad(0);

    q_ne = trafo::quat_ne(latitude, longitude);

    x_e = { 1, 2, 3 };
    x_n = q_ne * x_e;

    CHECK(x_n.x() == Approx(3.0));
    CHECK(x_n.y() == Approx(2.0));
    CHECK(x_n.z() == Approx(-1.0));

    /* -------------------------------------------------------------------------------------------------------- */

    latitude = trafo::deg2rad(0);
    longitude = trafo::deg2rad(0);

    q_ne = trafo::quat_ne(latitude, longitude);
    //                    (0, 0, 7.2921151467e-05)
    x_n = q_ne * InsConst::angularVelocity_ie_e;

    CHECK(x_n.x() == Approx(InsConst::angularVelocity_ie));
    CHECK(x_n.y() == Approx(0));
    CHECK(x_n.z() == Approx(0).margin(EPSILON));

    /* -------------------------------------------------------------------------------------------------------- */

    latitude = trafo::deg2rad(45);
    longitude = trafo::deg2rad(90);

    q_ne = trafo::quat_ne(latitude, longitude);
    //                    (0, 0, 7.2921151467e-05)
    x_n = q_ne * InsConst::angularVelocity_ie_e;

    CHECK(x_n.x() == Approx(InsConst::angularVelocity_ie / std::sqrt(2)));
    CHECK(x_n.y() == Approx(0).margin(EPSILON));
    CHECK(x_n.z() == Approx(-InsConst::angularVelocity_ie / std::sqrt(2)));
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

    CHECK(position_n(0) == Approx(0).margin(1e-10));
    CHECK(position_n(1) == Approx(0).margin(1e-10));
    CHECK(position_n(2) == Approx(-200));

    /* -------------------------------------------------------------------------------------------------------- */

    position_e = trafo::ned2ecef(position_n_ref, { latitude_ref, longitude_ref, altitude_ref });

    CHECK(position_e(0) == position_e_ref(0));
    CHECK(position_e(1) == position_e_ref(1));
    CHECK(position_e(2) == position_e_ref(2));

    /* -------------------------------------------------------------------------------------------------------- */

    position_e_ref = trafo::lla2ecef_WGS84({ -10, 70, 2001 });
    position_n = trafo::ecef2ned(position_e_ref, { latitude_ref, longitude_ref, altitude_ref });
    position_e = trafo::ned2ecef(position_n, { latitude_ref, longitude_ref, altitude_ref });

    CHECK(position_e(0) == Approx(position_e_ref(0)));
    CHECK(position_e(1) == Approx(position_e_ref(1)));
    CHECK(position_e(2) == Approx(position_e_ref(2)));
}

TEST_CASE("[InsTransformations] Body <=> navigation frame conversion", "[InsTransformations]")
{
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = trafo::deg2rad(45);
    auto C_nb = trafo::quat_nb(roll, pitch, yaw).toRotationMatrix();
    auto C_nb_ref = DCM_nb(roll, pitch, yaw);

    CHECK(C_nb(0, 0) == Approx(C_nb_ref(0, 0)));
    CHECK(C_nb(0, 1) == Approx(C_nb_ref(0, 1)));
    CHECK(C_nb(0, 2) == Approx(C_nb_ref(0, 2)));
    CHECK(C_nb(1, 0) == Approx(C_nb_ref(1, 0)));
    CHECK(C_nb(1, 1) == Approx(C_nb_ref(1, 1)));
    CHECK(C_nb(1, 2) == Approx(C_nb_ref(1, 2)));
    CHECK(C_nb(2, 0) == Approx(C_nb_ref(2, 0)));
    CHECK(C_nb(2, 1) == Approx(C_nb_ref(2, 1)));
    CHECK(C_nb(2, 2) == Approx(C_nb_ref(2, 2)));

    /* -------------------------------------------------------------------------------------------------------- */

    roll = trafo::deg2rad(10);
    pitch = trafo::deg2rad(-39);
    yaw = trafo::deg2rad(170);
    C_nb = trafo::quat_nb(roll, pitch, yaw).toRotationMatrix();
    C_nb_ref = DCM_nb(roll, pitch, yaw);

    CHECK(C_nb(0, 0) == Approx(C_nb_ref(0, 0)));
    CHECK(C_nb(0, 1) == Approx(C_nb_ref(0, 1)));
    CHECK(C_nb(0, 2) == Approx(C_nb_ref(0, 2)));
    CHECK(C_nb(1, 0) == Approx(C_nb_ref(1, 0)));
    CHECK(C_nb(1, 1) == Approx(C_nb_ref(1, 1)));
    CHECK(C_nb(1, 2) == Approx(C_nb_ref(1, 2)));
    CHECK(C_nb(2, 0) == Approx(C_nb_ref(2, 0)));
    CHECK(C_nb(2, 1) == Approx(C_nb_ref(2, 1)));
    CHECK(C_nb(2, 2) == Approx(C_nb_ref(2, 2)));

    /* -------------------------------------------------------------------------------------------------------- */

    roll = 0.0;
    pitch = trafo::deg2rad(90);
    yaw = trafo::deg2rad(90);
    auto q_bn = trafo::quat_bn(roll, pitch, yaw);

    Eigen::Vector3d x_n{ 1.0, 0.0, 0.0 };
    Eigen::Vector3d x_b = q_bn * x_n;

    CHECK(x_b.x() == Approx(0.0).margin(EPSILON));
    CHECK(x_b.y() == Approx(-1.0).margin(EPSILON));
    CHECK(x_b.z() == Approx(0.0).margin(EPSILON));

    /* -------------------------------------------------------------------------------------------------------- */

    roll = 0.0;
    pitch = 0.0;
    yaw = trafo::deg2rad(-45);
    q_bn = trafo::quat_bn(roll, pitch, yaw);

    x_n = Eigen::Vector3d{ 1.0, 1.0, 0.0 };
    x_b = q_bn * x_n;

    CHECK(x_b.x() == Approx(0.0).margin(EPSILON));
    CHECK(x_b.y() == Approx(std::sqrt(2)).margin(EPSILON));
    CHECK(x_b.z() == Approx(0.0).margin(EPSILON));

    /* -------------------------------------------------------------------------------------------------------- */

    roll = trafo::deg2rad(45);
    pitch = 0.0;
    yaw = 0.0;
    q_bn = trafo::quat_bn(roll, pitch, yaw);

    x_n = { 1.0, 1.0, 1.0 };
    x_b = q_bn * x_n;

    CHECK(x_b.x() == Approx(1.0).margin(EPSILON));
    CHECK(x_b.y() == Approx(std::sqrt(2)).margin(EPSILON));
    CHECK(x_b.z() == Approx(0.0).margin(EPSILON));

    /* -------------------------------------------------------------------------------------------------------- */

    roll = 0.0;
    pitch = trafo::deg2rad(45);
    yaw = 0.0;
    q_bn = trafo::quat_bn(roll, pitch, yaw);

    x_n = { 1.0, 1.0, 1.0 };
    x_b = q_bn * x_n;

    CHECK(x_b.x() == Approx(0.0).margin(EPSILON));
    CHECK(x_b.y() == Approx(1.0).margin(EPSILON));
    CHECK(x_b.z() == Approx(std::sqrt(2)).margin(EPSILON));

    /* -------------------------------------------------------------------------------------------------------- */

    roll = trafo::deg2rad(90);
    pitch = trafo::deg2rad(180);
    yaw = trafo::deg2rad(90);
    q_bn = trafo::quat_bn(roll, pitch, yaw);

    x_n = { 1.0, 2.0, 3.0 };
    x_b = q_bn * x_n;

    CHECK(x_b.x() == Approx(-x_n(1)).margin(EPSILON));
    CHECK(x_b.y() == Approx(-x_n(2)).margin(EPSILON));
    CHECK(x_b.z() == Approx(x_n(0)).margin(EPSILON));
}

TEST_CASE("[InsTransformations] Platform <=> body frame conversion", "[InsTransformations]")
{
    double mountingAngleX = trafo::deg2rad(90);
    double mountingAngleY = 0.0;
    double mountingAngleZ = trafo::deg2rad(-90);

    auto q_bp = trafo::quat_bp(mountingAngleX, mountingAngleY, mountingAngleZ);

    Eigen::Vector3d x_p{ 2.0, 0.0, 9.81 };
    Eigen::Vector3d x_b = q_bp * x_p;

    CHECK(x_b.x() == Approx(0.0).margin(EPSILON));
    CHECK(x_b.y() == Approx(9.81).margin(EPSILON));
    CHECK(x_b.z() == Approx(-2.0).margin(EPSILON));
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
    CHECK(ecef.x() == Approx(ecef_ref.x()));
    CHECK(ecef.y() == Approx(ecef_ref.y()));
    CHECK(ecef.z() == Approx(ecef_ref.z()));
    CHECK(lla.x() == Approx(latitude));
    CHECK(lla.y() == Approx(longitude));
    CHECK(lla.z() == Approx(altitude).margin(0.5));

    /* -------------------------------------------------------------------------- */

    // New York
    // https://www.koordinaten-umrechner.de/decimal/40.712728,-74.006015?karte=OpenStreetMap&zoom=4
    latitude = trafo::deg2rad(40.712728);
    longitude = trafo::deg2rad(-74.006015);
    altitude = 13;
    ecef_ref = Eigen::Vector3d(1334.001, -4654.06, 4138.303) * 1000;
    ecef = trafo::lla2ecef_WGS84({ latitude, longitude, altitude });
    lla = trafo::ecef2lla_WGS84(ecef_ref);
    CHECK(ecef.x() == Approx(ecef_ref.x()));
    CHECK(ecef.y() == Approx(ecef_ref.y()));
    CHECK(ecef.z() == Approx(ecef_ref.z()));
    CHECK(lla.x() == Approx(latitude));
    CHECK(lla.y() == Approx(longitude));
    CHECK(lla.z() == Approx(altitude).margin(0.5));

    /* -------------------------------------------------------------------------- */

    latitude = 0;
    longitude = 0;
    altitude = -3492;
    ecef_ref = Eigen::Vector3d(6374.645, 0, 0) * 1000;
    ecef = trafo::lla2ecef_WGS84({ latitude, longitude, altitude });
    lla = trafo::ecef2lla_WGS84(ecef_ref);
    CHECK(ecef.x() == Approx(ecef_ref.x()));
    CHECK(ecef.y() == Approx(ecef_ref.y()));
    CHECK(ecef.z() == Approx(ecef_ref.z()));
    CHECK(lla.x() == Approx(latitude));
    CHECK(lla.y() == Approx(longitude));
    CHECK(lla.z() == Approx(altitude));

    /* -------------------------------------------------------------------------- */

    latitude = trafo::deg2rad(-89.9999);
    longitude = trafo::deg2rad(0);
    altitude = 2801;
    ecef_ref = Eigen::Vector3d(0.011, 0, -6359.553) * 1000;
    ecef = trafo::lla2ecef_WGS84({ latitude, longitude, altitude });
    lla = trafo::ecef2lla_WGS84(ecef_ref);
    CHECK(ecef.x() == Approx(ecef_ref.x()).margin(0.2));
    CHECK(ecef.y() == Approx(ecef_ref.y()));
    CHECK(ecef.z() == Approx(ecef_ref.z()));
    CHECK(lla.x() == Approx(latitude));
    CHECK(lla.y() == Approx(longitude));
    CHECK(lla.z() == Approx(altitude).margin(0.5));

    /* -------------------------------------------------------------------------- */

    latitude = trafo::deg2rad(40);
    longitude = trafo::deg2rad(180);
    altitude = -5097;
    ecef_ref = Eigen::Vector3d(-4888.803, 0, 4074.709) * 1000;
    ecef = trafo::lla2ecef_WGS84({ latitude, longitude, altitude });
    lla = trafo::ecef2lla_WGS84(ecef_ref);
    CHECK(ecef.x() == Approx(ecef_ref.x()));
    CHECK(ecef.y() == Approx(ecef_ref.y()).margin(1e-9));
    CHECK(ecef.z() == Approx(ecef_ref.z()));
    CHECK(lla.x() == Approx(latitude));
    CHECK(lla.y() == Approx(longitude));
    CHECK(lla.z() == Approx(altitude).margin(0.5));

    /* -------------------------------------------------------------------------- */

    latitude = trafo::deg2rad(40);
    longitude = trafo::deg2rad(180);
    altitude = -5097;
    lla = trafo::ecef2lla_WGS84(trafo::lla2ecef_WGS84({ latitude, longitude, altitude }));
    CHECK(lla.x() == Approx(latitude));
    CHECK(lla.y() == Approx(longitude));
    CHECK(lla.z() == Approx(altitude));
}

TEST_CASE("[InsTransformations] ECEF <=> LLH iterative conversion", "[InsTransformations]")
{
    // Conversion with https://www.oc.nps.edu/oc2902w/coord/llhxyz.htm

    // Stuttgart, Breitscheidstraße 2
    // https://www.koordinaten-umrechner.de/decimal/48.780810,9.172012?karte=OpenStreetMap&zoom=19
    double latitude = trafo::deg2rad(48.78081);
    double longitude = trafo::deg2rad(9.172012);
    double altitude = 254;
    Eigen::Vector3d ecef = trafo::lla2ecef_WGS84({ latitude, longitude, altitude });
    Eigen::Vector3d lla = trafo::ecef2lla_WGS84(ecef);
    Eigen::Vector3d lla_iter = ecef2lla_iter(ecef, InsConst::WGS84_a, InsConst::WGS84_e_squared);

    CHECK(lla.x() == Approx(latitude));
    CHECK(lla.y() == Approx(longitude));
    CHECK(lla.z() == Approx(altitude));
    CHECK(lla.x() == Approx(lla_iter.x()));
    CHECK(lla.y() == Approx(lla_iter.y()));
    CHECK(lla.z() == Approx(lla_iter.z()));

    /* -------------------------------------------------------------------------------------------------------- */

    latitude = trafo::deg2rad(48.78081);
    longitude = trafo::deg2rad(9.172012);
    altitude = 0;
    ecef = trafo::lla2ecef_WGS84({ latitude, longitude, altitude });
    lla = trafo::ecef2lla_WGS84(ecef);
    lla_iter = ecef2lla_iter(ecef, InsConst::WGS84_a, InsConst::WGS84_e_squared);

    CHECK(lla.x() == Approx(latitude));
    CHECK(lla.y() == Approx(longitude));
    CHECK(lla.z() == Approx(altitude));
    CHECK(lla.x() == Approx(lla_iter.x()));
    CHECK(lla.y() == Approx(lla_iter.y()));
    CHECK(lla.z() == Approx(lla_iter.z()).margin(1e-9));

    /* -------------------------------------------------------------------------------------------------------- */

    latitude = trafo::deg2rad(0);
    longitude = trafo::deg2rad(0);
    altitude = 0;
    ecef = trafo::lla2ecef_WGS84({ latitude, longitude, altitude });
    lla = trafo::ecef2lla_WGS84(ecef);
    lla_iter = ecef2lla_iter(ecef, InsConst::WGS84_a, InsConst::WGS84_e_squared);

    CHECK(lla.x() == Approx(latitude));
    CHECK(lla.y() == Approx(longitude));
    CHECK(lla.z() == Approx(altitude));
    CHECK(lla.x() == Approx(lla_iter.x()));
    CHECK(lla.y() == Approx(lla_iter.y()));
    CHECK(lla.z() == Approx(lla_iter.z()));

    /* -------------------------------------------------------------------------------------------------------- */

    latitude = trafo::deg2rad(90);
    longitude = trafo::deg2rad(0);
    altitude = 0;
    ecef = trafo::lla2ecef_WGS84({ latitude, longitude, altitude });
    lla = trafo::ecef2lla_WGS84(ecef);
    lla_iter = ecef2lla_iter(ecef, InsConst::WGS84_a, InsConst::WGS84_e_squared);

    CHECK(lla.x() == Approx(latitude));
    CHECK(lla.y() == Approx(longitude));
    CHECK(lla.z() == Approx(altitude));
    CHECK(lla.x() == Approx(lla_iter.x()));
    CHECK(lla.y() == Approx(lla_iter.y()));
    CHECK(lla.z() == Approx(lla_iter.z()));

    /* -------------------------------------------------------------------------------------------------------- */

    latitude = trafo::deg2rad(90);
    longitude = trafo::deg2rad(180);
    altitude = 0;
    ecef = trafo::lla2ecef_WGS84({ latitude, longitude, altitude });
    lla = trafo::ecef2lla_WGS84(ecef);
    lla_iter = ecef2lla_iter(ecef, InsConst::WGS84_a, InsConst::WGS84_e_squared);

    CHECK(lla.x() == Approx(latitude));
    CHECK(lla.y() == Approx(longitude));
    CHECK(lla.z() == Approx(altitude));
    CHECK(lla.x() == Approx(lla_iter.x()));
    CHECK(lla.y() == Approx(lla_iter.y()));
    CHECK(lla.z() == Approx(lla_iter.z()));
}

TEST_CASE("[InsTransformations] Transformation chains", "[InsTransformations]")
{
    Eigen::Quaterniond q_bp(Eigen::AngleAxisd(trafo::deg2rad(-90), Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond q_nb = trafo::quat_nb(trafo::deg2rad(20), trafo::deg2rad(50), trafo::deg2rad(190));
    Eigen::Quaterniond q_en = trafo::quat_en(trafo::deg2rad(10), trafo::deg2rad(40));

    Eigen::Vector3d v_p{ 1, 3, 5 };

    Eigen::Vector3d v_b = q_bp * v_p;
    Eigen::Vector3d v_n = q_nb * v_b;
    Eigen::Vector3d v_e = q_en * v_n;

    Eigen::Quaterniond q_np = q_nb * q_bp;
    Eigen::Vector3d v_n_direct = q_np * v_p;

    Eigen::Quaterniond q_ep = q_en * q_nb * q_bp;
    Eigen::Vector3d v_e_direct = q_ep * v_p;

    CHECK(v_n.x() == Approx(v_n_direct.x()));
    CHECK(v_n.y() == Approx(v_n_direct.y()));
    CHECK(v_n.z() == Approx(v_n_direct.z()));

    CHECK(v_e.x() == Approx(v_e_direct.x()));
    CHECK(v_e.y() == Approx(v_e_direct.y()));
    CHECK(v_e.z() == Approx(v_e_direct.z()));
}

} // namespace NAV
