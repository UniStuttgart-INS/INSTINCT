/// @file Functions.hpp
/// @brief Inertial Navigation Helper Functions
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-09-02

#pragma once

#include <Eigen/Core>

#include "Navigation/Constants.hpp"

namespace NAV
{
/// @brief Calculates the transport rate ω_en_n (rotation rate of the Earth frame relative to the navigation frame)
///
/// \anchor eq-INS-Mechanization-TransportRate \f{equation}{ \label{eq:eq-INS-Mechanization-TransportRate}
///   \boldsymbol{\omega}_{en}^n = \begin{bmatrix} \dfrac{v_E}{R_E + h} & \dfrac{-v_N}{R_N + h} & \dfrac{-v_E \tan{\phi}}{R_E + h} \end{bmatrix}^T
/// \f}
///
/// @param[in] latLonAlt [𝜙, λ, h] Latitude, Longitude and altitude in [rad, rad, m]
/// @param[in] velocity_n v_n Velocity in [m/s], in navigation coordinate
/// @param[in] R_N North/South (meridian) earth radius [m]
/// @param[in] R_E East/West (prime vertical) earth radius [m]
/// @return ω_en_n Transport Rate in local-navigation coordinates in [rad/s]
///
/// @note See \cite Groves2013 Groves, ch. 5.4.1, eq. 5.44, p. 177
/// @note See \cite Gleason2009 Gleason, ch. 6.2.3.2, eq. 6.15, p. 155
/// @note See \cite Titterton2004 Titterton, ch. 3.7.2, eq. 3.87, p. 50 (mistake in denominator 3rd term)
///
[[nodiscard]] Eigen::Vector3d calcTransportRate_n(const Eigen::Vector3d& latLonAlt, const Eigen::Vector3d& velocity_n, const double& R_N, const double& R_E);

/// @brief Calculates the centrifugal acceleration in [m/s^2] (acceleration that makes a body follow a curved path)
///
/// \anchor eq-INS-Mechanization-CentrifugalAcceleration \f{equation}{ \label{eq:eq-INS-Mechanization-CentrifugalAcceleration}
///   \boldsymbol{\omega}_{ie}^e \times [ \boldsymbol{\omega}_{ie}^e \times \mathbf{x}^e ]
/// \f}
///
/// @param[in] x_e Position in ECEF coordinates in [m]
/// @param[in] omega_ie_e Angular rate of the Earth rotation in [rad/s] in the Earth coordinate frame
/// @return Centrifugal acceleration in the Earth coordinate frame in [m/s^2]
[[nodiscard]] Eigen::Vector3d calcCentrifugalAcceleration_e(const Eigen::Vector3d& x_e, const Eigen::Vector3d& omega_ie_e = InsConst::omega_ie_e);

/// @brief Calculates the coriolis acceleration in [m/s^2] (acceleration due to motion in rotating reference frame)
///
/// \anchor eq-INS-Mechanization-CoriolisAcceleration \f{equation}{ \label{eq:eq-INS-Mechanization-CoriolisAcceleration}
///   (2 \boldsymbol{\omega}_{ie}^n + \boldsymbol{\omega}_{en}^n) \times \boldsymbol{v}^n
/// \f}
///
/// @param[in] omega_ie_n ω_ie_n Angular rate of the Earth rotation in [rad/s] in local-navigation coordinates
/// @param[in] omega_en_n ω_en_n Transport rate in [rad/s] in local-navigation coordinates
/// @param[in] velocity_n v_n Velocity in local-navigation frame coordinates in [m/s^2]
/// @return Coriolis acceleration in local-navigation coordinates in [m/s^2]
[[nodiscard]] Eigen::Vector3d calcCoriolisAcceleration_n(const Eigen::Vector3d& omega_ie_n, const Eigen::Vector3d& omega_en_n, const Eigen::Vector3d& velocity_n);

/// @brief Calculates the roll angle from a static acceleration measurement
/// @param[in] accel_b Acceleration measurement in static condition in [m/s^2]
/// @return The roll angle in [rad]
///
/// @note See E.-H. Shin (2005) - Estimation Techniques for Low-Cost Inertial Navigation (Chapter 2.6)
[[nodiscard]] double calcRollFromStaticAcceleration(const Eigen::Vector3d& accel_b);

/// @brief Calculates the pitch angle from a static acceleration measurement
/// @param[in] accel_b Acceleration measurement in static condition in [m/s^2]
/// @return The pitch angle in [rad]
///
/// @note See E.-H. Shin (2005) - Estimation Techniques for Low-Cost Inertial Navigation (Chapter 2.6)
[[nodiscard]] double calcPitchFromStaticAcceleration(const Eigen::Vector3d& accel_b);

/// @brief Calculates the Yaw angle from the trajectory defined by the given velocity
///
/// \anchor eq-INS-Mechanization-Yaw \f{equation}{ \label{eq:eq-INS-Mechanization-Yaw}
///   Y = \tan^{-1}\left(\frac{v_E}{v_N}\right)
/// \f}
///
/// @param[in] velocity_n Velocity in [m/s] in local-navigation frame coordinates
/// @return Yaw angle in [rad]
///
/// @note See Groves (2013) equation (6.14)
[[nodiscard]] double calcYawFromVelocity(const Eigen::Vector3d& velocity_n);

/// @brief Calculates the Pitch angle from the trajectory defined by the given velocity
///
/// \anchor eq-INS-Mechanization-Pitch \f{equation}{ \label{eq:eq-INS-Mechanization-Pitch}
///   P = \tan^{-1}\left(\frac{-v_D}{\sqrt{v_N^2 + v_E^2}}\right)
/// \f}
///
/// @param[in] velocity_n Velocity in [m/s] in local-navigation frame coordinates
/// @return Pitch angle in [rad]
///
/// @note See Groves (2013) equation (6.17)
[[nodiscard]] double calcPitchFromVelocity(const Eigen::Vector3d& velocity_n);

} // namespace NAV
