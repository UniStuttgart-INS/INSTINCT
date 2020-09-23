#include <catch2/catch.hpp>

#include "util/InsTransformations.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <limits>

namespace NAV
{
constexpr double EPSILON = 10.0 * std::numeric_limits<double>::epsilon();

Eigen::Matrix3d DCM_b2n(double roll, double pitch, double yaw)
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

Eigen::Matrix3d DCM_n2e(double latitude, double longitude)
{
    Eigen::Matrix3d DCM;
    // clang-format off
    DCM << -sin(latitude)*cos(longitude), -sin(longitude), -cos(latitude)*cos(longitude),
           -sin(latitude)*sin(longitude),  cos(longitude), -cos(latitude)*sin(longitude),
                   cos(latitude)        ,        0       ,        -sin(latitude)        ;

    // clang-format on
    return DCM;
}

Eigen::Matrix3d DCM_i2e(const double time, const double angularRate_ie)
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

    CHECK(ZYX.z() == Approx(trafo::deg2rad(89)).margin(0.000001));
    CHECK(ZYX.y() == Approx(trafo::deg2rad(45.0)).margin(0.000001));
    CHECK(ZYX.x() == Approx(trafo::deg2rad(1.0)).margin(0.000001));
}

TEST_CASE("[InsTransformations] Inertial <=> Earth-fixed frame conversion", "[InsTransformations]")
{
    double time = 86164.099 / 3.0;
    auto C_i2e = trafo::quat_i2e(time, InsConst::angularVelocity_ie).toRotationMatrix();
    auto C_i2e_ref = DCM_i2e(time, InsConst::angularVelocity_ie);

    CHECK(C_i2e(0, 0) == Approx(C_i2e_ref(0, 0)));
    CHECK(C_i2e(0, 1) == Approx(C_i2e_ref(0, 1)));
    CHECK(C_i2e(0, 2) == Approx(C_i2e_ref(0, 2)));
    CHECK(C_i2e(1, 0) == Approx(C_i2e_ref(1, 0)));
    CHECK(C_i2e(1, 1) == Approx(C_i2e_ref(1, 1)));
    CHECK(C_i2e(1, 2) == Approx(C_i2e_ref(1, 2)));
    CHECK(C_i2e(2, 0) == Approx(C_i2e_ref(2, 0)));
    CHECK(C_i2e(2, 1) == Approx(C_i2e_ref(2, 1)));
    CHECK(C_i2e(2, 2) == Approx(C_i2e_ref(2, 2)));

    /* -------------------------------------------------------------------------------------------------------- */

    // Sidereal day: 23h 56min 4.099s
    auto siderialDay4 = 86164.099 / 4.0;
    auto q_i2e = trafo::quat_i2e(siderialDay4, InsConst::angularVelocity_ie);
    // Star day: 23h 56min 4.0905s
    // auto starHalfDay = 86164.0905 / 2.0;
    // auto q_i2e = trafo::quat_i2e(starHalfDay);

    auto x_e = Eigen::Vector3d(1, -2.5, 22);
    auto x_i = q_i2e * x_e;

    CHECK(x_i.x() == Approx(-2.5).margin(0.000001));
    CHECK(x_i.y() == Approx(-1).margin(0.000001));
    CHECK(x_i.z() == Approx(22.0).margin(0.000001));
}

TEST_CASE("[InsTransformations] Navigation <=> Earth-fixed frame conversion", "[InsTransformations]")
{
    double latitude = trafo::deg2rad(88);
    double longitude = trafo::deg2rad(-40);

    auto C_n2e = trafo::quat_n2e(latitude, longitude).toRotationMatrix();
    auto C_n2e_ref = DCM_n2e(latitude, longitude);

    CHECK(C_n2e(0, 0) == Approx(C_n2e_ref(0, 0)).margin(EPSILON));
    CHECK(C_n2e(0, 1) == Approx(C_n2e_ref(0, 1)).margin(EPSILON));
    CHECK(C_n2e(0, 2) == Approx(C_n2e_ref(0, 2)).margin(EPSILON));
    CHECK(C_n2e(1, 0) == Approx(C_n2e_ref(1, 0)).margin(EPSILON));
    CHECK(C_n2e(1, 1) == Approx(C_n2e_ref(1, 1)).margin(EPSILON));
    CHECK(C_n2e(1, 2) == Approx(C_n2e_ref(1, 2)).margin(EPSILON));
    CHECK(C_n2e(2, 0) == Approx(C_n2e_ref(2, 0)).margin(EPSILON));
    CHECK(C_n2e(2, 1) == Approx(C_n2e_ref(2, 1)).margin(EPSILON));
    CHECK(C_n2e(2, 2) == Approx(C_n2e_ref(2, 2)).margin(EPSILON));

    /* -------------------------------------------------------------------------------------------------------- */

    latitude = trafo::deg2rad(90);
    longitude = 0.0;

    auto q_e2n = trafo::quat_n2e(latitude, longitude).conjugate();

    auto x_e = Eigen::Vector3d(1, 2, 3);
    auto x_n = q_e2n * x_e;

    CHECK(x_n.x() == Approx(-1.0));
    CHECK(x_n.y() == Approx(2.0));
    CHECK(x_n.z() == Approx(-3.0));
}

TEST_CASE("[InsTransformations] Body <=> navigation frame conversion", "[InsTransformations]")
{
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = trafo::deg2rad(45);
    auto C_b2n = trafo::quat_b2n(roll, pitch, yaw).toRotationMatrix();
    auto C_b2n_ref = DCM_b2n(roll, pitch, yaw);

    CHECK(C_b2n(0, 0) == Approx(C_b2n_ref(0, 0)));
    CHECK(C_b2n(0, 1) == Approx(C_b2n_ref(0, 1)));
    CHECK(C_b2n(0, 2) == Approx(C_b2n_ref(0, 2)));
    CHECK(C_b2n(1, 0) == Approx(C_b2n_ref(1, 0)));
    CHECK(C_b2n(1, 1) == Approx(C_b2n_ref(1, 1)));
    CHECK(C_b2n(1, 2) == Approx(C_b2n_ref(1, 2)));
    CHECK(C_b2n(2, 0) == Approx(C_b2n_ref(2, 0)));
    CHECK(C_b2n(2, 1) == Approx(C_b2n_ref(2, 1)));
    CHECK(C_b2n(2, 2) == Approx(C_b2n_ref(2, 2)));

    /* -------------------------------------------------------------------------------------------------------- */

    roll = trafo::deg2rad(10);
    pitch = trafo::deg2rad(-39);
    yaw = trafo::deg2rad(170);
    C_b2n = trafo::quat_b2n(roll, pitch, yaw).toRotationMatrix();
    C_b2n_ref = DCM_b2n(roll, pitch, yaw);

    CHECK(C_b2n(0, 0) == Approx(C_b2n_ref(0, 0)));
    CHECK(C_b2n(0, 1) == Approx(C_b2n_ref(0, 1)));
    CHECK(C_b2n(0, 2) == Approx(C_b2n_ref(0, 2)));
    CHECK(C_b2n(1, 0) == Approx(C_b2n_ref(1, 0)));
    CHECK(C_b2n(1, 1) == Approx(C_b2n_ref(1, 1)));
    CHECK(C_b2n(1, 2) == Approx(C_b2n_ref(1, 2)));
    CHECK(C_b2n(2, 0) == Approx(C_b2n_ref(2, 0)));
    CHECK(C_b2n(2, 1) == Approx(C_b2n_ref(2, 1)));
    CHECK(C_b2n(2, 2) == Approx(C_b2n_ref(2, 2)));

    /* -------------------------------------------------------------------------------------------------------- */

    roll = 0.0;
    pitch = 0.0;
    yaw = trafo::deg2rad(-45);
    auto q_n2b = trafo::quat_b2n(roll, pitch, yaw).conjugate();

    auto x_n = Eigen::Vector3d(1.0, 1.0, 0.0);
    Eigen::Vector3d x_b = q_n2b * x_n;

    CHECK(x_b.x() == Approx(0.0).margin(EPSILON));
    CHECK(x_b.y() == Approx(std::sqrt(2)).margin(EPSILON));
    CHECK(x_b.z() == Approx(0.0).margin(EPSILON));

    /* -------------------------------------------------------------------------------------------------------- */

    roll = trafo::deg2rad(45);
    pitch = 0.0;
    yaw = 0.0;
    q_n2b = trafo::quat_b2n(roll, pitch, yaw).conjugate();

    x_n = Eigen::Vector3d(1.0, 1.0, 1.0);
    x_b = q_n2b * x_n;

    CHECK(x_b.x() == Approx(1.0).margin(EPSILON));
    CHECK(x_b.y() == Approx(std::sqrt(2)).margin(EPSILON));
    CHECK(x_b.z() == Approx(0.0).margin(EPSILON));

    /* -------------------------------------------------------------------------------------------------------- */

    roll = 0.0;
    pitch = trafo::deg2rad(45);
    yaw = 0.0;
    q_n2b = trafo::quat_b2n(roll, pitch, yaw).conjugate();

    x_n = Eigen::Vector3d(1.0, 1.0, 1.0);
    x_b = q_n2b * x_n;

    CHECK(x_b.x() == Approx(0.0).margin(EPSILON));
    CHECK(x_b.y() == Approx(1.0).margin(EPSILON));
    CHECK(x_b.z() == Approx(std::sqrt(2)).margin(EPSILON));

    /* -------------------------------------------------------------------------------------------------------- */

    roll = trafo::deg2rad(90);
    pitch = trafo::deg2rad(180);
    yaw = trafo::deg2rad(90);
    q_n2b = trafo::quat_b2n(roll, pitch, yaw).conjugate();

    x_n = Eigen::Vector3d(1.0, 2.0, 3.0);
    x_b = q_n2b * x_n;

    CHECK(x_b.x() == Approx(-x_n(1)).margin(EPSILON));
    CHECK(x_b.y() == Approx(-x_n(2)).margin(EPSILON));
    CHECK(x_b.z() == Approx(x_n(0)).margin(EPSILON));
}

TEST_CASE("[InsTransformations] Platform <=> body frame conversion", "[InsTransformations]")
{
    double mountingAngleX = 0.0;
    double mountingAngleY = trafo::deg2rad(180);
    double mountingAngleZ = trafo::deg2rad(45);

    auto q_p2b = trafo::quat_p2b(mountingAngleX, mountingAngleY, mountingAngleZ).conjugate();

    auto x_p = Eigen::Vector3d(2.0, 0.0, 9.81);
    Eigen::Vector3d x_b = q_p2b * x_p;

    CHECK(x_b.x() == Approx(-std::sqrt(2)));
    CHECK(x_b.y() == Approx(-std::sqrt(2)));
    CHECK(x_b.z() == Approx(-9.81));
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
    CHECK(ecef.x() == Approx(ecef_ref.x()));
    CHECK(ecef.y() == Approx(ecef_ref.y()));
    CHECK(ecef.z() == Approx(ecef_ref.z()));
    CHECK(llh.x() == Approx(latitude));
    CHECK(llh.y() == Approx(longitude));
    CHECK(llh.z() == Approx(height).margin(0.5));

    /* -------------------------------------------------------------------------- */

    // New York
    // https://www.koordinaten-umrechner.de/decimal/40.712728,-74.006015?karte=OpenStreetMap&zoom=4
    latitude = trafo::deg2rad(40.712728);
    longitude = trafo::deg2rad(-74.006015);
    height = 13;
    ecef_ref = Eigen::Vector3d(1334.001, -4654.06, 4138.303) * 1000;
    ecef = trafo::llh2ecef_WGS84(latitude, longitude, height);
    llh = trafo::ecef2llh_WGS84(ecef_ref);
    CHECK(ecef.x() == Approx(ecef_ref.x()));
    CHECK(ecef.y() == Approx(ecef_ref.y()));
    CHECK(ecef.z() == Approx(ecef_ref.z()));
    CHECK(llh.x() == Approx(latitude));
    CHECK(llh.y() == Approx(longitude));
    CHECK(llh.z() == Approx(height).margin(0.5));

    /* -------------------------------------------------------------------------- */

    latitude = 0;
    longitude = 0;
    height = -3492;
    ecef_ref = Eigen::Vector3d(6374.645, 0, 0) * 1000;
    ecef = trafo::llh2ecef_WGS84(latitude, longitude, height);
    llh = trafo::ecef2llh_WGS84(ecef_ref);
    CHECK(ecef.x() == Approx(ecef_ref.x()));
    CHECK(ecef.y() == Approx(ecef_ref.y()));
    CHECK(ecef.z() == Approx(ecef_ref.z()));
    CHECK(llh.x() == Approx(latitude));
    CHECK(llh.y() == Approx(longitude));
    CHECK(llh.z() == Approx(height));

    /* -------------------------------------------------------------------------- */

    latitude = trafo::deg2rad(-89.9999);
    longitude = trafo::deg2rad(0);
    height = 2801;
    ecef_ref = Eigen::Vector3d(0.011, 0, -6359.553) * 1000;
    ecef = trafo::llh2ecef_WGS84(latitude, longitude, height);
    llh = trafo::ecef2llh_WGS84(ecef_ref);
    CHECK(ecef.x() == Approx(ecef_ref.x()).margin(0.2));
    CHECK(ecef.y() == Approx(ecef_ref.y()));
    CHECK(ecef.z() == Approx(ecef_ref.z()));
    CHECK(llh.x() == Approx(latitude));
    CHECK(llh.y() == Approx(longitude));
    CHECK(llh.z() == Approx(height).margin(0.5));

    /* -------------------------------------------------------------------------- */

    latitude = trafo::deg2rad(40);
    longitude = trafo::deg2rad(180);
    height = -5097;
    ecef_ref = Eigen::Vector3d(-4888.803, 0, 4074.709) * 1000;
    ecef = trafo::llh2ecef_WGS84(latitude, longitude, height);
    llh = trafo::ecef2llh_WGS84(ecef_ref);
    CHECK(ecef.x() == Approx(ecef_ref.x()));
    CHECK(ecef.y() == Approx(ecef_ref.y()).margin(1e-9));
    CHECK(ecef.z() == Approx(ecef_ref.z()));
    CHECK(llh.x() == Approx(latitude));
    CHECK(llh.y() == Approx(longitude));
    CHECK(llh.z() == Approx(height).margin(0.5));

    /* -------------------------------------------------------------------------- */

    latitude = trafo::deg2rad(40);
    longitude = trafo::deg2rad(180);
    height = -5097;
    llh = trafo::ecef2llh_WGS84(trafo::llh2ecef_WGS84(latitude, longitude, height));
    CHECK(llh.x() == Approx(latitude));
    CHECK(llh.y() == Approx(longitude));
    CHECK(llh.z() == Approx(height));
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

    CHECK(v_n.x() == Approx(v_n_direct.x()));
    CHECK(v_n.y() == Approx(v_n_direct.y()));
    CHECK(v_n.z() == Approx(v_n_direct.z()));

    CHECK(v_e.x() == Approx(v_e_direct.x()));
    CHECK(v_e.y() == Approx(v_e_direct.y()));
    CHECK(v_e.z() == Approx(v_e_direct.z()));
}

} // namespace NAV
