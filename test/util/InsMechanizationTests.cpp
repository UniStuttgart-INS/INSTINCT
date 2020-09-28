#include <catch2/catch.hpp>

#include "util/InsMath.hpp"
#include "util/InsMechanization.hpp"
#include "util/InsTransformations.hpp"
#include "util/InsGravity.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <deque>

namespace NAV
{
TEST_CASE("[InsMechanization] Update Quaternions ep Runge-Kutta 3. Order", "[InsMechanization]")
{
    // Stuttgart, Breitscheidstraße 2
    // https://www.koordinaten-umrechner.de/decimal/48.780810,9.172012?karte=OpenStreetMap&zoom=19
    double latitude = trafo::deg2rad(48.78081);
    double longitude = trafo::deg2rad(9.172012);

    /// Δtₖ = (tₖ - tₖ₋₁) Time difference in [seconds]
    long double timeDifferenceSec = 0.0001L;
    /// ω_ip_p (tₖ) Angular velocity in [rad/s], of the inertial to platform system, in platform coordinates
    Eigen::Vector3d angularVelocity_ip_p(0, 0, 1.0);
    /// ω_ie_e (tₖ) Angular velocity in [rad/s], of the inertial to earth system, in earth coordinates, at the time tₖ
    Eigen::Vector3d angularVelocity_ie_e(0, 0, 0);

    /// q Quaternion, from platform to body coordinates. Depends on mounting of strap down IMU
    Eigen::Quaterniond q_bp = Eigen::Quaterniond::Identity();

    /// q Quaternion, from earth to navigation coordinates. Depends on location
    Eigen::Quaterniond q_ne = trafo::quat_ne(latitude, longitude);

    Eigen::Quaterniond q_nb__t0 = Eigen::Quaterniond::Identity();

    std::deque<Eigen::Quaterniond> quats_ep;
    quats_ep.push_back(trafo::quat_en(latitude, longitude) * q_nb__t0 * q_bp);
    quats_ep.push_back(trafo::quat_en(latitude, longitude) * q_nb__t0 * q_bp);

    size_t count = 10000;
    for (size_t i = 0; i < count; i++)
    {
        Eigen::Quaterniond q_ep = updateQuaternion_ep_RungeKutta3(timeDifferenceSec, timeDifferenceSec,
                                                                  angularVelocity_ip_p, angularVelocity_ip_p,
                                                                  angularVelocity_ie_e,
                                                                  quats_ep.at(1), quats_ep.at(0));
        quats_ep.push_back(q_ep);
        quats_ep.pop_front();
    }

    auto q_ep = quats_ep.at(quats_ep.size() - 1);
    auto q_nb = q_ne * q_ep * q_bp;
    auto yawPitchRoll = trafo::quat2eulerZYX(q_nb);

    Eigen::Vector3d expectedRollPitchYaw = angularVelocity_ip_p * (static_cast<double>(timeDifferenceSec) * static_cast<double>(count));
    Eigen::Quaterniond expectedQuat_nb = trafo::quat_nb(expectedRollPitchYaw.x(), expectedRollPitchYaw.y(), expectedRollPitchYaw.z());

    CHECK(q_nb.x() == Approx(expectedQuat_nb.x()).margin(1e-13));
    CHECK(q_nb.y() == Approx(expectedQuat_nb.y()).margin(1e-13));
    CHECK(q_nb.z() == Approx(expectedQuat_nb.z()).margin(1e-13));
    CHECK(q_nb.w() == Approx(expectedQuat_nb.w()).margin(1e-13));

    CHECK(yawPitchRoll.x() == Approx(expectedRollPitchYaw.z()).margin(1e-13));
    CHECK(yawPitchRoll.y() == Approx(expectedRollPitchYaw.y()).margin(1e-13));
    CHECK(yawPitchRoll.z() == Approx(expectedRollPitchYaw.x()).margin(1e-13));
}

TEST_CASE("[InsMechanization] Update Quaternions nb Runge-Kutta 3. Order", "[InsMechanization]")
{
    /// Δtₖ = (tₖ - tₖ₋₁) Time difference in [seconds]
    long double timeDifferenceSec = 0.0001L;
    /// ω_ip_p (tₖ) Angular velocity in [rad/s], of the inertial to platform system, in body coordinates
    Eigen::Vector3d angularVelocity_ip_b(0, 0, 1.0);
    /// ω_ie_e (tₖ) Angular velocity in [rad/s], of the inertial to earth system, in navigation coordinates, at the time tₖ
    Eigen::Vector3d angularVelocity_ie_n(0, 0, 0);
    /// ω_en_n (tₖ₋₁) Transport Rate, rotation rate of the Earth frame relative to the navigation frame in navigation coordinates
    Eigen::Vector3d angularVelocity_en_n(0, 0, 0);

    /// q Quaternion, from platform to body coordinates. Depends on mounting of strap down IMU
    Eigen::Quaterniond q_bp = Eigen::Quaterniond::Identity();

    /// q Quaternion, from earth to navigation coordinates. Depends on location
    Eigen::Quaterniond q_ne = Eigen::Quaterniond::Identity();

    std::deque<Eigen::Quaterniond> quats;
    quats.push_back(Eigen::Quaterniond::Identity());
    quats.push_back(Eigen::Quaterniond::Identity());

    size_t count = 10000;
    for (size_t i = 0; i < count; i++)
    {
        Eigen::Quaterniond q_nb = updateQuaternion_nb_RungeKutta3(timeDifferenceSec, timeDifferenceSec,
                                                                  angularVelocity_ip_b, angularVelocity_ip_b,
                                                                  angularVelocity_ie_n,
                                                                  angularVelocity_en_n,
                                                                  quats.at(1), quats.at(0));
        quats.push_back(q_nb);
        quats.pop_front();
    }

    auto q_ep = quats.at(quats.size() - 1);
    auto q_nb = q_ne * q_ep * q_bp;
    auto yawPitchRoll = trafo::quat2eulerZYX(q_nb);

    Eigen::Vector3d expectedRollPitchYaw = angularVelocity_ip_b * (static_cast<double>(timeDifferenceSec) * static_cast<double>(count));
    Eigen::Quaterniond expectedQuat_nb = trafo::quat_nb(expectedRollPitchYaw.x(), expectedRollPitchYaw.y(), expectedRollPitchYaw.z());

    CHECK(q_nb.x() == Approx(expectedQuat_nb.x()).margin(1e-13));
    CHECK(q_nb.y() == Approx(expectedQuat_nb.y()).margin(1e-13));
    CHECK(q_nb.z() == Approx(expectedQuat_nb.z()).margin(1e-13));
    CHECK(q_nb.w() == Approx(expectedQuat_nb.w()).margin(1e-13));

    CHECK(yawPitchRoll.y() == Approx(expectedRollPitchYaw.y()).margin(1e-13));
    CHECK(yawPitchRoll.x() == Approx(expectedRollPitchYaw.z()).margin(1e-13));
    CHECK(yawPitchRoll.z() == Approx(expectedRollPitchYaw.x()).margin(1e-13));
}

TEST_CASE("[InsMechanization] Update Velocity e-frame Runge-Kutta 3. Order", "[InsMechanization]")
{
    /// Δtₖ = (tₖ - tₖ₋₁) Time difference in [seconds]
    long double timeDifferenceSec = 0.0001L;

    // Stuttgart, Breitscheidstraße 2
    // https://www.koordinaten-umrechner.de/decimal/48.780810,9.172012?karte=OpenStreetMap&zoom=19
    double latitude = trafo::deg2rad(48.78081);
    double longitude = trafo::deg2rad(9.172012);
    double height = 254;

    double roll = 0;
    double pitch = 0;
    double yaw = trafo::deg2rad(45);

    double mountingAngleX = 90;
    double mountingAngleY = 180;
    double mountingAngleZ = 0;

    auto gravity = gravity::gravityMagnitude_Gleason(latitude);
    Eigen::Vector3d gravity_n = Eigen::Vector3d(0, 0, gravity);
    Eigen::Vector3d gravity_e = trafo::quat_en(latitude, longitude) * gravity_n;

    /// a_p Acceleration in [m/s^2], in navigation coordinates
    Eigen::Vector3d acceleration_n(1, -1, -gravity);
    Eigen::Vector3d acceleration_p = trafo::quat_pb(mountingAngleX, mountingAngleY, mountingAngleZ)
                                     * trafo::quat_bn(roll, pitch, yaw)
                                     * acceleration_n;

    Eigen::Vector3d position = trafo::llh2ecef_WGS84(latitude, longitude, height);

    Eigen::Quaterniond quaternion_ep = trafo::quat_en(latitude, longitude)
                                       * trafo::quat_nb(roll, pitch, yaw)
                                       * trafo::quat_bp(mountingAngleX, mountingAngleY, mountingAngleZ);

    std::deque<Eigen::Vector3d> velocities;
    velocities.emplace_back(Eigen::Vector3d::Zero());
    velocities.emplace_back(Eigen::Vector3d::Zero());

    size_t count = 10000;
    for (size_t i = 0; i <= count; i++)
    {
        Eigen::Vector3d v_e = updateVelocity_e_RungeKutta3(timeDifferenceSec, timeDifferenceSec,
                                                           acceleration_p, acceleration_p,
                                                           velocities.at(0),
                                                           position,
                                                           gravity_e,
                                                           quaternion_ep,
                                                           quaternion_ep,
                                                           quaternion_ep);
        velocities.push_back(v_e);
        velocities.pop_front();
    }

    auto v_e = velocities.at(velocities.size() - 1);

    auto v_n = trafo::quat_ne(latitude, longitude) * v_e;

    // Exact values are not achieved
    CHECK(v_n.x() == Approx(1).margin(0.03));
    CHECK(v_n.y() == Approx(-1).margin(0.01));
    CHECK(v_n.z() == Approx(0).margin(0.02));
}

TEST_CASE("[InsMechanization] Update Velocity n-frame Runge-Kutta 3. Order", "[InsMechanization]")
{
    /// Δtₖ = (tₖ - tₖ₋₁) Time difference in [seconds]
    long double timeDifferenceSec = 0.0001L;

    // Stuttgart, Breitscheidstraße 2
    // https://www.koordinaten-umrechner.de/decimal/48.780810,9.172012?karte=OpenStreetMap&zoom=19
    double latitude = trafo::deg2rad(48.78081);
    double longitude = trafo::deg2rad(9.172012);
    double height = 254;

    double roll = 0;
    double pitch = 0;
    double yaw = trafo::deg2rad(45);

    auto gravity = gravity::gravityMagnitude_Gleason(latitude);
    Eigen::Vector3d gravity_n = Eigen::Vector3d(0, 0, gravity);

    /// a_p Acceleration in [m/s^2], in navigation coordinates
    Eigen::Vector3d acceleration_n(1, 1, -gravity);
    Eigen::Vector3d acceleration_b = trafo::quat_bn(roll, pitch, yaw)
                                     * acceleration_n;

    Eigen::Quaterniond quaternion_nb = trafo::quat_nb(roll, pitch, yaw);

    /// ω_ie_n Nominal mean angular velocity of the Earth in [rad/s], in navigation coordinates
    Eigen::Vector3d angularVelocity_ie_n = trafo::quat_ne(latitude, longitude) * InsConst::angularVelocity_ie_e;

    /// North/South (meridian) earth radius [m]
    double R_N = earthRadius_N(InsConst::WGS84_a, InsConst::WGS84_e_squared, latitude);
    /// East/West (prime vertical) earth radius [m]
    double R_E = earthRadius_E(InsConst::WGS84_a, InsConst::WGS84_e_squared, latitude);

    std::deque<Eigen::Vector3d> velocities;
    velocities.emplace_back(Eigen::Vector3d::Zero());
    velocities.emplace_back(Eigen::Vector3d::Zero());

    size_t count = 10000;
    for (size_t i = 0; i < count; i++)
    {
        /// ω_en_n (tₖ₋₁) Transport Rate, rotation rate of the Earth frame relative to the navigation frame, in navigation coordinates
        Eigen::Vector3d angularVelocity_en_n = transportRate(Eigen::Vector3d(latitude, longitude, height),
                                                             velocities.at(1), R_N, R_E);

        Eigen::Vector3d v_n = updateVelocity_n_RungeKutta3(timeDifferenceSec, timeDifferenceSec,
                                                           acceleration_b,
                                                           acceleration_b,
                                                           velocities.at(1),
                                                           velocities.at(0),
                                                           gravity_n,
                                                           angularVelocity_ie_n,
                                                           angularVelocity_en_n,
                                                           quaternion_nb,
                                                           quaternion_nb,
                                                           quaternion_nb);
        velocities.push_back(v_n);
        velocities.pop_front();
    }

    auto v_n = velocities.at(velocities.size() - 1);

    // Exact values are not achieved
    CHECK(v_n.x() == Approx(1).margin(0.001));
    CHECK(v_n.y() == Approx(1).margin(0.001));
    CHECK(v_n.z() == Approx(0).margin(1e-4));
}

TEST_CASE("[InsMechanization] Update Position e-frame", "[InsMechanization]")
{
    /// Δtₖ = (tₖ - tₖ₋₁) Time difference in [seconds]
    long double timeDifferenceSec = 0.0001L;

    // Stuttgart, Breitscheidstraße 2
    // https://www.koordinaten-umrechner.de/decimal/48.780810,9.172012?karte=OpenStreetMap&zoom=19
    double latitude = trafo::deg2rad(48.78081);
    double longitude = trafo::deg2rad(9.172012);
    double height = 254;

    Eigen::Vector3d velocity_n = Eigen::Vector3d(2, 0, 0);
    Eigen::Vector3d velocity_e = trafo::quat_en(latitude, longitude) * velocity_n;

    Eigen::Vector3d position_e = trafo::llh2ecef_WGS84(latitude, longitude, height);

    size_t count = 10000;
    for (size_t i = 0; i < count; i++)
    {
        position_e = updatePosition_e(timeDifferenceSec, position_e, velocity_e);
    }
    auto llh = trafo::ecef2llh_WGS84(position_e);

    CHECK(measureDistance(latitude, longitude, llh(0), llh(1)) == Approx(2.0).margin(0.002));

    CHECK(measureDistance(latitude, longitude, llh(0), longitude) == Approx(2.0).margin(0.002));
    CHECK(longitude == Approx(llh(1)).margin(1e-13));

    CHECK(latitude < llh(0));
}

TEST_CASE("[InsMechanization] Update Position n-frame", "[InsMechanization]")
{
    /// Δtₖ = (tₖ - tₖ₋₁) Time difference in [seconds]
    long double dt = 0.001L;

    // Stuttgart, Breitscheidstraße 2
    // https://www.koordinaten-umrechner.de/decimal/48.780810,9.172012?karte=OpenStreetMap&zoom=19
    double latitude = trafo::deg2rad(48.78081);
    double longitude = trafo::deg2rad(9.172012);
    double height = 254;

    double roll = 0;
    double pitch = 0;
    double yaw = trafo::deg2rad(45);

    Eigen::Vector3d velocity_b = Eigen::Vector3d(2, 0, 0);

    Eigen::Vector3d velocity_n = trafo::quat_nb(roll, pitch, yaw) * velocity_b;

    Eigen::Vector3d latLonHeight = Eigen::Vector3d(latitude, longitude, height);

    Eigen::Vector3d pos_n = Eigen::Vector3d::Zero();

    size_t count = 4000;
    for (size_t i = 0; i < count; i++)
    {
        /// North/South (meridian) earth radius [m]
        double R_N = earthRadius_N(InsConst::WGS84_a, InsConst::WGS84_e_squared, latLonHeight(0));
        /// East/West (prime vertical) earth radius [m]
        double R_E = earthRadius_E(InsConst::WGS84_a, InsConst::WGS84_e_squared, latLonHeight(0));

        latLonHeight = updatePosition_n(dt, latLonHeight, velocity_n, R_N, R_E);

        pos_n = updatePosition_n(dt, pos_n, velocity_n);
    }
    double distance = static_cast<double>(count) * static_cast<double>(dt) * velocity_b.norm();

    CHECK(pos_n.norm() == Approx(distance));
    CHECK(pos_n(0) == Approx(distance * std::sin(yaw)));
    CHECK(pos_n(1) == Approx(distance * std::cos(yaw)));
    CHECK(pos_n(2) == 0);

    // updatePosition_n with lat lon formula shows really bad accuracy
    CHECK(measureDistance(latitude, longitude, latLonHeight(0), latLonHeight(1)) == Approx(distance).margin(0.004));
    CHECK(measureDistance(latitude, longitude, latLonHeight(0), longitude) == Approx(distance * std::sin(yaw)).margin(0.02));
    CHECK(measureDistance(latitude, longitude, latitude, latLonHeight(1)) == Approx(distance * std::cos(yaw)).margin(0.02));

    CHECK(latitude < latLonHeight(0));
    CHECK(longitude < latLonHeight(1));
}

} // namespace NAV
