#include <catch2/catch.hpp>

#include "util/InsTransformations.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <limits>

namespace NAV
{
constexpr double EPSILON = 2.0 * std::numeric_limits<double>::epsilon();

TEST_CASE("[InsTransformations] Degree to radian conversion", "[InsTransformations]")
{
    double rad_90 = trafo::deg2rad(90);
    double rad_180 = trafo::deg2rad(180);
    double rad_360 = trafo::deg2rad(360);

    REQUIRE(rad_90 == M_PI_2);
    REQUIRE(rad_180 == M_PI);
    REQUIRE(rad_360 == M_PI * 2.0);
}

TEST_CASE("[InsTransformations] Degree to radian conversion constexpr", "[InsTransformations]")
{
    constexpr double rad_90 = trafo::deg2rad(90);
    constexpr double rad_180 = trafo::deg2rad(180);
    constexpr double rad_360 = trafo::deg2rad(360);

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
    // Conversions with https://www.andre-gaschler.com/rotationconverter

    //                              w    ,     x    ,     y    ,      z
    auto q = Eigen::Quaterniond(0.6612731, 0.6451492, 0.2785897, -0.2624657);
    auto ZYX = trafo::quat2eulerZYX(q);

    REQUIRE(std::abs(ZYX.z() - trafo::deg2rad(89)) <= 0.000001);
    REQUIRE(std::abs(ZYX.y() - trafo::deg2rad(45.0)) <= 0.000001);
    REQUIRE(std::abs(ZYX.x() - trafo::deg2rad(1.0)) <= 0.000001);
}

TEST_CASE("[InsTransformations] Inertial <=> Earth-fixed frame conversion", "[InsTransformations]")
{
    // Sidereal day: 23h 56min 4.099s
    auto siderialHalfDay = 86164.099 / 2.0;
    auto q_i2e = trafo::quat_i2e(siderialHalfDay);
    auto dcm_i2e = trafo::DCM_i2e(siderialHalfDay);
    // Star day: 23h 56min 4.0905s
    // auto starHalfDay = 86164.0905 / 2.0;
    // auto q_i2e = trafo::quat_i2e(starHalfDay);
    // auto dcm_i2e = trafo::DCM_i2e(starHalfDay);

    auto x_e = Eigen::Vector3d(1, -2.5, 22);
    auto x_i = q_i2e * x_e;
    auto x_i2 = dcm_i2e * x_e;

    REQUIRE(std::abs(x_i.x() - -1.0) <= 0.000001);
    REQUIRE(std::abs(x_i.y() - 2.5) <= 0.000001);
    REQUIRE(std::abs(x_i.z() - 22.0) <= 0.000001);
    REQUIRE(std::abs(x_i2.x() - -1.0) <= 0.000001);
    REQUIRE(std::abs(x_i2.y() - 2.5) <= 0.000001);
    REQUIRE(std::abs(x_i2.z() - 22.0) <= 0.000001);
}

TEST_CASE("[InsTransformations] Navigation <=> Earth-fixed frame conversion", "[InsTransformations]")
{
    double latitude = trafo::deg2rad(90);
    double longitude = 0.0;

    auto q_e2n = trafo::quat_n2e(latitude, longitude).conjugate();
    auto dcm_e2n = trafo::DCM_n2e(latitude, longitude).transpose();

    auto x_e = Eigen::Vector3d(1, 2, 3);
    auto x_n = q_e2n * x_e;
    auto x_n2 = dcm_e2n * x_e;

    REQUIRE(std::abs(x_n.x() - -1.0) <= EPSILON);
    REQUIRE(std::abs(x_n.y() - 2.0) <= EPSILON);
    REQUIRE(std::abs(x_n.z() - -3.0) <= EPSILON);
    REQUIRE(std::abs(x_n2.x() - -1.0) <= EPSILON);
    REQUIRE(std::abs(x_n2.y() - 2.0) <= EPSILON);
    REQUIRE(std::abs(x_n2.z() - -3.0) <= EPSILON);
}

TEST_CASE("[InsTransformations] Body <=> navigation frame conversion", "[InsTransformations]")
{
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = trafo::deg2rad(45);
    auto q_n2b = trafo::quat_b2n(roll, pitch, yaw).conjugate();
    Eigen::Matrix3d dcm_n2b = trafo::DCM_b2n(roll, pitch, yaw).transpose();

    auto x_n = Eigen::Vector3d(1.0, 1.0, 0.0);
    Eigen::Vector3d x_b = q_n2b * x_n;
    Eigen::Vector3d x_b2 = dcm_n2b * x_n;

    REQUIRE(std::abs(x_b.x() - 0.0) <= EPSILON);
    REQUIRE(std::abs(x_b.y() - std::sqrt(2)) <= EPSILON);
    REQUIRE(std::abs(x_b.z() - 0.0) <= EPSILON);
    REQUIRE(std::abs(x_b2.x() - 0.0) <= EPSILON);
    REQUIRE(std::abs(x_b2.y() - std::sqrt(2)) <= EPSILON);
    REQUIRE(std::abs(x_b2.z() - 0.0) <= EPSILON);

    roll = trafo::deg2rad(45);
    pitch = 0.0;
    yaw = 0.0;
    q_n2b = trafo::quat_b2n(roll, pitch, yaw).conjugate();
    dcm_n2b = trafo::DCM_b2n(roll, pitch, yaw).transpose();

    x_n = Eigen::Vector3d(1.0, 1.0, 1.0);
    x_b = q_n2b * x_n;
    x_b2 = dcm_n2b * x_n;

    REQUIRE(std::abs(x_b.x() - 1.0) <= EPSILON);
    REQUIRE(std::abs(x_b.y() - 0.0) <= EPSILON);
    REQUIRE(std::abs(x_b.z() - std::sqrt(2)) <= EPSILON);
    REQUIRE(std::abs(x_b2.x() - 1.0) <= EPSILON);
    REQUIRE(std::abs(x_b2.y() - 0.0) <= EPSILON);
    REQUIRE(std::abs(x_b2.z() - std::sqrt(2)) <= EPSILON);

    roll = trafo::deg2rad(90);
    pitch = trafo::deg2rad(180);
    yaw = trafo::deg2rad(90);
    q_n2b = trafo::quat_b2n(roll, pitch, yaw).conjugate();
    dcm_n2b = trafo::DCM_b2n(roll, pitch, yaw).transpose();

    x_n = Eigen::Vector3d(1.0, 2.0, 3.0);
    x_b = q_n2b * x_n;
    x_b2 = dcm_n2b * x_n;

    REQUIRE(std::abs(x_b.x() - 2.0) <= EPSILON);
    REQUIRE(std::abs(x_b.y() - 3.0) <= EPSILON);
    REQUIRE(std::abs(x_b.z() - 1.0) <= EPSILON);
    REQUIRE(std::abs(x_b2.x() - 2.0) <= EPSILON);
    REQUIRE(std::abs(x_b2.y() - 3.0) <= EPSILON);
    REQUIRE(std::abs(x_b2.z() - 1.0) <= EPSILON);
}

TEST_CASE("[InsTransformations] LLH to ECEF", "[InsTransformations]")
{
    // Conversion with https://www.oc.nps.edu/oc2902w/coord/llhxyz.htm

    // Stuttgart, Breitscheidstraße 2
    // https://www.koordinaten-umrechner.de/decimal/48.780810,9.172012?karte=OpenStreetMap&zoom=19
    double latitude = 48.78081;
    double longitude = 9.172012;
    double height = 254;
    Eigen::Vector3d ecef_ref = Eigen::Vector3d(4157.128, 671.224, 4774.723) * 1000;

    Eigen::Vector3d ecef = trafo::llh2ecef_wgs84(latitude, longitude, height);

    REQUIRE(ecef == ecef_ref);
}

} // namespace NAV
