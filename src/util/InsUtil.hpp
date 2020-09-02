/// @file InsUtil.hpp
/// @brief Collection of different utility functions
/// @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
/// @date 2020-09-01

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace NAV
{
/// @brief Converts Euler rotation angles into quaternions
/// @param[in] yaw Yaw angle (rotation around z axis) in [rad]
/// @param[in] pitch Pitch angle (rotation around y axis) in [rad]
/// @param[in] roll Roll angle (rotation around x axis) in [rad]
/// @return Quaternion of the yaw, pitch and roll rotation
[[nodiscard]] Eigen::Quaterniond euler2quaternion(double yaw, double pitch, double roll);

/// @brief Converts the quaternion to Euler rotation angles
/// @param[in] q Quaternion to convert
/// @return [yaw, pitch, roll]^T vector with angles in [rad]
[[nodiscard]] Eigen::Vector3d quaternion2euler(const Eigen::Quaterniond& q);

} // namespace NAV
