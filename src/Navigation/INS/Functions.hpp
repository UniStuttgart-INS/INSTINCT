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
/// @param[in] lla_position [𝜙, λ, h] Latitude, Longitude and altitude in [rad, rad, m]
/// @param[in] n_velocity Velocity in [m/s], in navigation coordinate
/// @param[in] R_N North/South (meridian) earth radius [m]
/// @param[in] R_E East/West (prime vertical) earth radius [m]
/// @return ω_en_n Transport Rate in local-navigation coordinates in [rad/s]
///
/// @note See \cite Groves2013 Groves, ch. 5.4.1, eq. 5.44, p. 177
/// @note See \cite Gleason2009 Gleason, ch. 6.2.3.2, eq. 6.15, p. 155
/// @note See \cite Titterton2004 Titterton, ch. 3.7.2, eq. 3.87, p. 50 (mistake in denominator 3rd term)
///
[[nodiscard]] Eigen::Vector3d n_calcTransportRate(const Eigen::Vector3d& lla_position, const Eigen::Vector3d& n_velocity, const double& R_N, const double& R_E);

/// @brief Calculates the centrifugal acceleration in [m/s^2] (acceleration that makes a body follow a curved path)
///
/// \anchor eq-INS-Mechanization-CentrifugalAcceleration \f{equation}{ \label{eq:eq-INS-Mechanization-CentrifugalAcceleration}
///   \boldsymbol{\omega}_{ie}^e \times [ \boldsymbol{\omega}_{ie}^e \times \mathbf{x}^e ]
/// \f}
///
/// @param[in] e_position Position in  coordinates in [m]
/// @param[in] e_omega_ie Angular rate of the Earth rotation in [rad/s] in the Earth coordinate frame
/// @return Centrifugal acceleration in the Earth coordinate frame in [m/s^2]
[[nodiscard]] Eigen::Vector3d e_calcCentrifugalAcceleration(const Eigen::Vector3d& e_position, const Eigen::Vector3d& e_omega_ie = InsConst::e_omega_ie);

/// @brief Calculates the coriolis acceleration in [m/s^2] (acceleration due to motion in rotating reference frame)
///
/// \anchor eq-INS-Mechanization-CoriolisAcceleration \f{equation}{ \label{eq:eq-INS-Mechanization-CoriolisAcceleration}
///   (2 \boldsymbol{\omega}_{ie}^n + \boldsymbol{\omega}_{en}^n) \times \boldsymbol{v}^n
/// \f}
///
/// @param[in] n_omega_ie ω_ie_n Angular rate of the Earth rotation in [rad/s] in local-navigation coordinates
/// @param[in] n_omega_en ω_en_n Transport rate in [rad/s] in local-navigation coordinates
/// @param[in] n_velocity Velocity in local-navigation frame coordinates in [m/s^2]
/// @return Coriolis acceleration in local-navigation coordinates in [m/s^2]
[[nodiscard]] Eigen::Vector3d n_calcCoriolisAcceleration(const Eigen::Vector3d& n_omega_ie, const Eigen::Vector3d& n_omega_en, const Eigen::Vector3d& n_velocity);

/// @brief Calculates the coriolis acceleration in [m/s^2] (acceleration due to motion in rotating reference frame)
///
/// \anchor eq-INS-Mechanization-CoriolisAcceleration-e \f{equation}{ \label{eq:eq-INS-Mechanization-CoriolisAcceleration-e}
///   2 \boldsymbol{\omega}_{ie}^e \times \boldsymbol{v}^e
/// \f}
///
/// @param[in] e_omega_ie ω_ie_e Angular rate of the Earth rotation in [rad/s] in ECEF coordinates
/// @param[in] e_velocity Velocity in ECEF frame coordinates in [m/s^2]
/// @return Coriolis acceleration in ECEF coordinates in [m/s^2]
[[nodiscard]] Eigen::Vector3d e_calcCoriolisAcceleration(const Eigen::Vector3d& e_omega_ie, const Eigen::Vector3d& e_velocity);

/// @brief Calculates the roll angle from a static acceleration measurement
/// @param[in] b_accel Acceleration measurement in static condition in [m/s^2]
/// @return The roll angle in [rad]
///
/// @note See E.-H. Shin (2005) - Estimation Techniques for Low-Cost Inertial Navigation (Chapter 2.6)
[[nodiscard]] double calcRollFromStaticAcceleration(const Eigen::Vector3d& b_accel);

/// @brief Calculates the pitch angle from a static acceleration measurement
/// @param[in] b_accel Acceleration measurement in static condition in [m/s^2]
/// @return The pitch angle in [rad]
///
/// @note See E.-H. Shin (2005) - Estimation Techniques for Low-Cost Inertial Navigation (Chapter 2.6)
[[nodiscard]] double calcPitchFromStaticAcceleration(const Eigen::Vector3d& b_accel);

/// @brief Calculates the Yaw angle from the trajectory defined by the given velocity
///
/// \anchor eq-INS-Mechanization-Yaw \f{equation}{ \label{eq:eq-INS-Mechanization-Yaw}
///   Y = \tan^{-1}\left(\frac{v_E}{v_N}\right)
/// \f}
///
/// @param[in] n_velocity Velocity in [m/s] in local-navigation frame coordinates
/// @return Yaw angle in [rad]
///
/// @note See \cite Groves2013 Groves, ch. 6.1.4, eq. 6.14, p. 225
[[nodiscard]] double calcYawFromVelocity(const Eigen::Vector3d& n_velocity);

/// @brief Calculates the Pitch angle from the trajectory defined by the given velocity
///
/// \anchor eq-INS-Mechanization-Pitch \f{equation}{ \label{eq:eq-INS-Mechanization-Pitch}
///   P = \tan^{-1}\left(\frac{-v_D}{\sqrt{v_N^2 + v_E^2}}\right)
/// \f}
///
/// @param[in] n_velocity Velocity in [m/s] in local-navigation frame coordinates
/// @return Pitch angle in [rad]
///
/// @note See \cite Groves2013 Groves, ch. 6.1.4, eq. 6.17, p. 225
[[nodiscard]] double calcPitchFromVelocity(const Eigen::Vector3d& n_velocity);

} // namespace NAV
