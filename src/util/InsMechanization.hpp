/// @file InsMechanization.hpp
/// @brief Inertial Navigation Mechanization Functions
/// @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
/// @date 2020-09-02

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace NAV
{
/// @brief Integrates the angular velocities and calculates the new Quaternion q_p2e with Runge Kutta of 3rd Order
/// @param[in] timeDifferenceSec__t0 Δtₖ Time difference in [seconds]. This epoch to previous epoch
/// @param[in] timeDifferenceSec__t1 Δtₖ₋₁ Time difference in [seconds]. Previous epoch to twice previous epoch
/// @param[in] angularVelocity_ip__t0 ω_ip_p (tₖ) Angluar velocity in [rad/s],
///                                 of the inertial to platform system, in platform coordinates, at the time tₖ
/// @param[in] angularVelocity_ip__t1 ω_ip_p (tₖ₋₁) Angluar velocity in [rad/s],
///                                 of the inertial to platform system, in platform coordinates, at the time tₖ₋₁
/// @param[in] angularVelocity_ie__t0 ω_ie_e (tₖ) Angluar velocity in [rad/s],
///                                 of the inertial to earth system, in earth coordinates, at the time tₖ
/// @param[in] quaternion_p2e__t1 q (tₖ₋₁) Quaternion, from platform to earth coordinates, at the time tₖ₋₁
/// @param[in] quaternion_p2e__t2 q (tₖ₋₂) Quaternion, from platform to earth coordinates, at the time tₖ₋₂
/// @return The updated Quaternion q_p2e
///
/// @note See C. Jekeli (2001) - Inertial Navigation Systems with Geodetic Applications (Chapter 4.2.4.1.2)
///       See T. Hobiger - Inertialnavigation (Lecture Slides Chapter 8)
[[nodiscard]] Eigen::Quaterniond updateQuaternionsRungeKutta3(const long double& timeDifferenceSec__t0,
                                                              const long double& timeDifferenceSec__t1,
                                                              const Eigen::Vector3d& angularVelocity_ip__t0,
                                                              const Eigen::Vector3d& angularVelocity_ip__t1,
                                                              const Eigen::Vector3d& angularVelocity_ie__t0,
                                                              const Eigen::Quaterniond& quaternion_p2e__t1,
                                                              const Eigen::Quaterniond& quaternion_p2e__t2);

/// @brief Integrates the accelerations and calculates the new velocity v_e with Runge Kutta of 3rd Order
/// @param[in] timeDifferenceSec__t0 Δtₖ Time difference in [seconds]. This epoch to previous epoch
/// @param[in] timeDifferenceSec__t1 Δtₖ₋₁ Time difference in [seconds]. Previous epoch to twice previous epoch
/// @param[in] acceleration_p__t0 a_p (tₖ) Acceleration in [m/s^2], in platform coordinates, at the time tₖ
/// @param[in] acceleration_p__t1 a_p (tₖ₋₁) Acceleration in [m/s^2], in platform coordinates, at the time tₖ₋₁
/// @param[in] velocity_e__t2 v_e (tₖ₋₂) Velocity in [m/s], in earth coordinates, at the time tₖ₋₂
/// @param[in] position_e__t2 x_e (tₖ₋₂) Position in [m/s], in earth coordinates, at the time tₖ₋₂
/// @param[in] gravity_e g_e Gravity vector in [m/s^2], in earth coordinates
/// @param[in] quaternion_p2e__t0 q (tₖ) Quaternion, from platform to earth coordinates, at the time tₖ
/// @param[in] quaternion_p2e__t1 q (tₖ₋₁) Quaternion, from platform to earth coordinates, at the time tₖ₋₁
/// @param[in] quaternion_p2e__t2 q (tₖ₋₂) Quaternion, from platform to earth coordinates, at the time tₖ₋₂
/// @return The updated velocity v_e
///
/// @note See C. Jekeli (2001) - Inertial Navigation Systems with Geodetic Applications (Chapter 4.2.4.1.2)
///       See T. Hobiger - Inertialnavigation (Lecture Slides Chapter 8)
[[nodiscard]] Eigen::Vector3d updateVelocityRungeKutta3(const long double& timeDifferenceSec__t0,
                                                        const long double& timeDifferenceSec__t1,
                                                        const Eigen::Vector3d& acceleration_p__t0,
                                                        const Eigen::Vector3d& acceleration_p__t1,
                                                        const Eigen::Vector3d& velocity_e__t2,
                                                        const Eigen::Vector3d& position_e__t2,
                                                        const Eigen::Vector3d& gravity_e,
                                                        const Eigen::Quaterniond& quaternion_p2e__t0,
                                                        const Eigen::Quaterniond& quaternion_p2e__t1,
                                                        const Eigen::Quaterniond& quaternion_p2e__t2);

} // namespace NAV
