#include <catch2/catch.hpp>

#include "util/InsMechanization.hpp"
#include "util/InsTransformations.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <deque>

namespace NAV
{
// TEST_CASE("[InsMechanization] Update Quaternions Runge-Kutta 3. Order", "[InsMechanization]")
// {
//     /// Δtₖ = (tₖ - tₖ₋₁) Time difference in [seconds]
//     long double timeDifferenceSec = 0.0001L;
//     /// ω_ip_p (tₖ) Rotation rate in [rad/s], of the inertial to platform system, in platform coordinates
//     Eigen::Vector3d rotationRate_ip(0, 0, 1.0);
//     /// ω_ie_e (tₖ) Rotation rate in [rad/s], of the inertial to earth system, in earth coordinates, at the time tₖ
//     Eigen::Vector3d rotationRate_ie(0, 0, 0);

//     std::deque<Eigen::Quaterniond> quats;
//     quats.push_back(Eigen::Quaterniond::Identity());
//     quats.push_back(Eigen::Quaterniond::Identity());

//     size_t count = 10000;
//     for (size_t i = 0; i < count; i++)
//     {
//         Eigen::Quaterniond q = updateQuaternionsRungeKutta3(timeDifferenceSec, timeDifferenceSec,
//                                                             rotationRate_ip, rotationRate_ip,
//                                                             rotationRate_ie,
//                                                             quats.at(1), quats.at(0));
//         quats.push_back(q);
//         quats.pop_front();
//     }

//     auto yawPitchRoll = trafo::quat2eulerZYX(quats.at(quats.size() - 1));

//     Eigen::Vector3d expectedRollPitchYaw = rotationRate_ip * (static_cast<double>(timeDifferenceSec) * static_cast<double>(count));
//     Eigen::Quaterniond expectedQuat = trafo::quat_b2n(expectedRollPitchYaw.x(), expectedRollPitchYaw.y(), expectedRollPitchYaw.z());

//     REQUIRE(std::abs(quats.at(quats.size() - 1).x() - expectedQuat.x()) < 0.00000000000001);
//     REQUIRE(std::abs(quats.at(quats.size() - 1).y() - expectedQuat.y()) < 0.00000000000001);
//     REQUIRE(std::abs(quats.at(quats.size() - 1).z() - expectedQuat.z()) < 0.00000000000001);
//     REQUIRE(std::abs(quats.at(quats.size() - 1).w() - expectedQuat.w()) < 0.00000000000001);

//     REQUIRE(std::abs(yawPitchRoll.x() - expectedRollPitchYaw.z()) < 0.00000000000001);
//     REQUIRE(std::abs(yawPitchRoll.y() - expectedRollPitchYaw.y()) < 0.00000000000001);
//     REQUIRE(std::abs(yawPitchRoll.z() - expectedRollPitchYaw.x()) < 0.00000000000001);
// }
} // namespace NAV
