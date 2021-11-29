/// @file Attitude.hpp
/// @brief Functions determining the attitude
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-11-28

#pragma once

#include <Eigen/Core>

namespace NAV
{

/// @brief Calculates the roll angle from a static acceleration measurement
/// @param[in] accel_b Acceleration measurement in static condition in [m/s^2]
/// @return The roll angle in [rad]
///
/// @note See E.-H. Shin (2005) - Estimation Techniques for Low-Cost Inertial Navigation (Chapter 2.6)
[[nodiscard]] double rollFromStaticAccelerationObs(const Eigen::Vector3d& accel_b);

/// @brief Calculates the pitch angle from a static acceleration measurement
/// @param[in] accel_b Acceleration measurement in static condition in [m/s^2]
/// @return The pitch angle in [rad]
///
/// @note See E.-H. Shin (2005) - Estimation Techniques for Low-Cost Inertial Navigation (Chapter 2.6)
[[nodiscard]] double pitchFromStaticAccelerationObs(const Eigen::Vector3d& accel_b);

/// @brief Calculates the Yaw angle from the trajectory defined by the given velocity
/// @param[in] velocity_n Velocity in [m/s] in local-navigation frame coordinates
/// @return Yaw angle in [rad]
///
/// @note See Groves (2013) equation (6.14)
[[nodiscard]] double yawFromVelocity(const Eigen::Vector3d& velocity_n);

/// @brief Calculates the Pitch angle from the trajectory defined by the given velocity
/// @param[in] velocity_n Velocity in [m/s] in local-navigation frame coordinates
/// @return Pitch angle in [rad]
///
/// @note See Groves (2013) equation (6.17)
[[nodiscard]] double pitchFromVelocity(const Eigen::Vector3d& velocity_n);

} // namespace NAV
