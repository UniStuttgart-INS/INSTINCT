#include <catch2/catch.hpp>

#include "util/InsTransformations.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <limits>

namespace NAV
{
constexpr double EPSILON = 10.0 * std::numeric_limits<double>::epsilon();

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
    // Star day: 23h 56min 4.0905s
    // auto starHalfDay = 86164.0905 / 2.0;
    // auto q_i2e = trafo::quat_i2e(starHalfDay);

    auto x_e = Eigen::Vector3d(1, -2.5, 22);
    auto x_i = q_i2e * x_e;

    REQUIRE(std::abs(x_i.x() - -1.0) <= 0.000001);
    REQUIRE(std::abs(x_i.y() - 2.5) <= 0.000001);
    REQUIRE(std::abs(x_i.z() - 22.0) <= 0.000001);
}

TEST_CASE("[InsTransformations] Navigation <=> Earth-fixed frame conversion", "[InsTransformations]")
{
    double latitude = trafo::deg2rad(90);
    double longitude = 0.0;

    auto q_e2n = trafo::quat_n2e(latitude, longitude).conjugate();

    auto x_e = Eigen::Vector3d(1, 2, 3);
    auto x_n = q_e2n * x_e;

    REQUIRE(std::abs(x_n.x() - -1.0) <= EPSILON);
    REQUIRE(std::abs(x_n.y() - 2.0) <= EPSILON);
    REQUIRE(std::abs(x_n.z() - -3.0) <= EPSILON);
}

TEST_CASE("[InsTransformations] Body <=> navigation frame conversion", "[InsTransformations]")
{
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = trafo::deg2rad(45);
    auto q_n2b = trafo::quat_b2n(roll, pitch, yaw).conjugate();

    auto x_n = Eigen::Vector3d(1.0, 1.0, 0.0);
    Eigen::Vector3d x_b = q_n2b * x_n;

    REQUIRE(std::abs(x_b.x() - 0.0) <= EPSILON);
    REQUIRE(std::abs(x_b.y() - std::sqrt(2)) <= EPSILON);
    REQUIRE(std::abs(x_b.z() - 0.0) <= EPSILON);

    /* -------------------------------------------------------------------------------------------------------- */

    roll = trafo::deg2rad(-45);
    pitch = 0.0;
    yaw = 0.0;
    q_n2b = trafo::quat_b2n(roll, pitch, yaw).conjugate();

    x_n = Eigen::Vector3d(1.0, 1.0, 1.0);
    x_b = q_n2b * x_n;

    REQUIRE(std::abs(x_b.x() - 1.0) <= EPSILON);
    REQUIRE(std::abs(x_b.y() - std::sqrt(2)) <= EPSILON);
    REQUIRE(std::abs(x_b.z() - 0.0) <= EPSILON);

    /* -------------------------------------------------------------------------------------------------------- */

    roll = 0.0;
    pitch = trafo::deg2rad(-45);
    yaw = 0.0;
    q_n2b = trafo::quat_b2n(roll, pitch, yaw).conjugate();

    x_n = Eigen::Vector3d(1.0, 1.0, 1.0);
    x_b = q_n2b * x_n;

    REQUIRE(std::abs(x_b.x() - 0.0) <= EPSILON);
    REQUIRE(std::abs(x_b.y() - 1.0) <= EPSILON);
    REQUIRE(std::abs(x_b.z() - std::sqrt(2)) <= EPSILON);

    /* -------------------------------------------------------------------------------------------------------- */

    roll = trafo::deg2rad(-90);
    pitch = trafo::deg2rad(-180);
    yaw = trafo::deg2rad(-90);
    q_n2b = trafo::quat_b2n(roll, pitch, yaw).conjugate();

    x_n = Eigen::Vector3d(1.0, 2.0, 3.0);
    x_b = q_n2b * x_n;

    REQUIRE(std::abs(x_b.x() - -x_n(1)) <= EPSILON);
    REQUIRE(std::abs(x_b.y() - -x_n(2)) <= EPSILON);
    REQUIRE(std::abs(x_b.z() - x_n(0)) <= EPSILON);
}

TEST_CASE("[InsTransformations] LLH <=> ECEF conversion", "[InsTransformations]")
{
    // Conversion with https://www.oc.nps.edu/oc2902w/coord/llhxyz.htm

    // Stuttgart, Breitscheidstraße 2
    // https://www.koordinaten-umrechner.de/decimal/48.780810,9.172012?karte=OpenStreetMap&zoom=19
    double latitude = trafo::deg2rad(48.78081);
    double longitude = trafo::deg2rad(9.172012);
    double height = 254;
    Eigen::Vector3d ecef_ref = Eigen::Vector3d(4157.128, 671.224, 4774.723) * 1000;
    Eigen::Vector3d ecef = trafo::llh2ecef_WGS84(latitude, longitude, height);
    Eigen::Vector3d llh = trafo::ecef2llh_WGS84(ecef_ref);
    REQUIRE(std::abs(ecef.x() - ecef_ref.x()) <= 0.5);
    REQUIRE(std::abs(ecef.y() - ecef_ref.y()) <= 0.5);
    REQUIRE(std::abs(ecef.z() - ecef_ref.z()) <= 0.5);
    REQUIRE(std::abs(llh.x() - latitude) <= 0.000001);
    REQUIRE(std::abs(llh.y() - longitude) <= 0.000001);
    REQUIRE(std::abs(llh.z() - height) <= 0.5);

    /* -------------------------------------------------------------------------- */

    // New York
    // https://www.koordinaten-umrechner.de/decimal/40.712728,-74.006015?karte=OpenStreetMap&zoom=4
    latitude = trafo::deg2rad(40.712728);
    longitude = trafo::deg2rad(-74.006015);
    height = 13;
    ecef_ref = Eigen::Vector3d(1334.001, -4654.06, 4138.303) * 1000;
    ecef = trafo::llh2ecef_WGS84(latitude, longitude, height);
    llh = trafo::ecef2llh_WGS84(ecef_ref);
    REQUIRE(std::abs(ecef.x() - ecef_ref.x()) <= 0.5);
    REQUIRE(std::abs(ecef.y() - ecef_ref.y()) <= 0.5);
    REQUIRE(std::abs(ecef.z() - ecef_ref.z()) <= 0.5);
    REQUIRE(std::abs(llh.x() - latitude) <= 0.000001);
    REQUIRE(std::abs(llh.y() - longitude) <= 0.000001);
    REQUIRE(std::abs(llh.z() - height) <= 0.5);

    /* -------------------------------------------------------------------------- */

    latitude = 0;
    longitude = 0;
    height = -3492;
    ecef_ref = Eigen::Vector3d(6374.645, 0, 0) * 1000;
    ecef = trafo::llh2ecef_WGS84(latitude, longitude, height);
    llh = trafo::ecef2llh_WGS84(ecef_ref);
    REQUIRE(std::abs(ecef.x() - ecef_ref.x()) <= 0.000001);
    REQUIRE(std::abs(ecef.y() - ecef_ref.y()) <= 0.000001);
    REQUIRE(std::abs(ecef.z() - ecef_ref.z()) <= 0.000001);
    REQUIRE(std::abs(llh.x() - latitude) <= 0.000001);
    REQUIRE(std::abs(llh.y() - longitude) <= 0.000001);
    REQUIRE(std::abs(llh.z() - height) <= 0.000001);

    /* -------------------------------------------------------------------------- */

    latitude = trafo::deg2rad(-89.9999);
    longitude = trafo::deg2rad(0);
    height = 2801;
    ecef_ref = Eigen::Vector3d(0.011, 0, -6359.553) * 1000;
    ecef = trafo::llh2ecef_WGS84(latitude, longitude, height);
    llh = trafo::ecef2llh_WGS84(ecef_ref);
    REQUIRE(std::abs(ecef.x() - ecef_ref.x()) <= 0.5);
    REQUIRE(std::abs(ecef.y() - ecef_ref.y()) <= 0.5);
    REQUIRE(std::abs(ecef.z() - ecef_ref.z()) <= 0.5);
    REQUIRE(std::abs(llh.x() - latitude) <= 0.000001);
    REQUIRE(std::abs(llh.y() - longitude) <= 0.000001);
    REQUIRE(std::abs(llh.z() - height) <= 0.5);

    /* -------------------------------------------------------------------------- */

    latitude = trafo::deg2rad(40);
    longitude = trafo::deg2rad(180);
    height = -5097;
    ecef_ref = Eigen::Vector3d(-4888.803, 0, 4074.709) * 1000;
    ecef = trafo::llh2ecef_WGS84(latitude, longitude, height);
    llh = trafo::ecef2llh_WGS84(ecef_ref);
    REQUIRE(std::abs(ecef.x() - ecef_ref.x()) <= 0.5);
    REQUIRE(std::abs(ecef.y() - ecef_ref.y()) <= 0.5);
    REQUIRE(std::abs(ecef.z() - ecef_ref.z()) <= 0.5);
    REQUIRE(std::abs(llh.x() - latitude) <= 0.000001);
    REQUIRE(std::abs(llh.y() - longitude) <= 0.000001);
    REQUIRE(std::abs(llh.z() - height) <= 0.5);

    /* -------------------------------------------------------------------------- */

    latitude = trafo::deg2rad(40);
    longitude = trafo::deg2rad(180);
    height = -5097;
    llh = trafo::ecef2llh_WGS84(trafo::llh2ecef_WGS84(latitude, longitude, height));
    REQUIRE(std::abs(llh.x() - latitude) <= EPSILON);
    REQUIRE(std::abs(llh.y() - longitude) <= EPSILON);
    REQUIRE(std::abs(llh.z() - height) <= EPSILON);
}

TEST_CASE("[InsTransformations] Transformation chains", "[InsTransformations]")
{
    Eigen::Quaterniond q_p2b(Eigen::AngleAxisd(trafo::deg2rad(-90), Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond q_b2n = trafo::quat_b2n(trafo::deg2rad(20), trafo::deg2rad(50), trafo::deg2rad(190));
    Eigen::Quaterniond q_n2e = trafo::quat_n2e(trafo::deg2rad(10), trafo::deg2rad(40));

    Eigen::Vector3d v_p(1, 3, 5);

    Eigen::Vector3d v_b = q_p2b * v_p;
    Eigen::Vector3d v_n = q_b2n * v_b;
    Eigen::Vector3d v_e = q_n2e * v_n;

    Eigen::Quaterniond q_p2n = q_b2n * q_p2b;
    Eigen::Vector3d v_n_direct = q_p2n * v_p;

    Eigen::Quaterniond q_p2e = q_n2e * q_b2n * q_p2b;
    Eigen::Vector3d v_e_direct = q_p2e * v_p;

    REQUIRE(std::abs(v_n.x() - v_n_direct.x()) <= EPSILON);
    REQUIRE(std::abs(v_n.y() - v_n_direct.y()) <= EPSILON);
    REQUIRE(std::abs(v_n.z() - v_n_direct.z()) <= EPSILON);

    REQUIRE(std::abs(v_e.x() - v_e_direct.x()) <= EPSILON);
    REQUIRE(std::abs(v_e.y() - v_e_direct.y()) <= EPSILON);
    REQUIRE(std::abs(v_e.z() - v_e_direct.z()) <= EPSILON);
}

} // namespace NAV
