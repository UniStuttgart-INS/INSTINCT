#include <catch2/catch.hpp>

#include "util/InsMechanization.hpp"
#include "util/InsTransformations.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <deque>

namespace NAV
{
TEST_CASE("[InsMechanization] Update Quaternions Runge-Kutta 3. Order", "[InsMechanization]")
{
    /// Δtₖ = (tₖ - tₖ₋₁) Time difference in [seconds]
    long double timeDifferenceSec = 0.0001L;
    /// ω_ip_p (tₖ) Angular velocity in [rad/s], of the inertial to platform system, in platform coordinates
    Eigen::Vector3d angularVelocity_ip(0, 0, 1.0);
    /// ω_ie_e (tₖ) Angular velocity in [rad/s], of the inertial to earth system, in earth coordinates, at the time tₖ
    Eigen::Vector3d angularVelocity_ie(0, 0, 0);

    /// q Quaternion, from platform to body coordinates. Depends on mounting of strap down IMU
    Eigen::Quaterniond q_p2b = Eigen::Quaterniond::Identity();

    /// q Quaternion, from earth to navigation coordinates. Depends on location
    Eigen::Quaterniond q_e2n = Eigen::Quaterniond::Identity();

    std::deque<Eigen::Quaterniond> quats;
    quats.push_back(Eigen::Quaterniond::Identity());
    quats.push_back(Eigen::Quaterniond::Identity());

    size_t count = 10000;
    for (size_t i = 0; i < count; i++)
    {
        Eigen::Quaterniond q_p2e = updateQuaternionsRungeKutta3(timeDifferenceSec, timeDifferenceSec,
                                                                angularVelocity_ip, angularVelocity_ip,
                                                                angularVelocity_ie,
                                                                quats.at(1), quats.at(0));
        quats.push_back(q_p2e);
        quats.pop_front();
    }

    auto q_p2e = quats.at(quats.size() - 1);
    auto q_b2n = q_e2n * q_p2e * q_p2b;
    auto yawPitchRoll = -trafo::quat2eulerZYX(q_b2n);

    Eigen::Vector3d expectedRollPitchYaw = -angularVelocity_ip * (static_cast<double>(timeDifferenceSec) * static_cast<double>(count));
    Eigen::Quaterniond expectedQuat_b2n = trafo::quat_b2n(expectedRollPitchYaw.x(), expectedRollPitchYaw.y(), expectedRollPitchYaw.z());

    REQUIRE(std::abs(q_b2n.x() - expectedQuat_b2n.x()) < 0.00000000000001);
    REQUIRE(std::abs(q_b2n.y() - expectedQuat_b2n.y()) < 0.00000000000001);
    REQUIRE(std::abs(q_b2n.z() - expectedQuat_b2n.z()) < 0.00000000000001);
    REQUIRE(std::abs(q_b2n.w() - expectedQuat_b2n.w()) < 0.00000000000001);

    REQUIRE(std::abs(yawPitchRoll.x() - expectedRollPitchYaw.z()) < 0.00000000000001);
    REQUIRE(std::abs(yawPitchRoll.y() - expectedRollPitchYaw.y()) < 0.00000000000001);
    REQUIRE(std::abs(yawPitchRoll.z() - expectedRollPitchYaw.x()) < 0.00000000000001);
}

} // namespace NAV
