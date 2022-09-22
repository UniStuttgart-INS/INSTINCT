// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file ErrorEquations.hpp
/// @brief Error Equations for the ECEF frame
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-06-12

#pragma once

#include <Eigen/Core>

namespace NAV
{
/// @brief Calculates the matrix 𝐅_𝜓'_𝜓
/// @param[in] omega_ie Angular velocity of the Earth in [rad/s]
/// @return 3x3 matrix in [rad / s]
/// @note See \cite Groves2013 Groves, ch. 14.2.3, eq. 14.48, p. 583 - 𝐅_11
[[nodiscard]] Eigen::Matrix3d e_F_dpsi_dpsi(double omega_ie);

/// @brief Calculates the matrix 𝐅_𝜓'_𝛿ω
/// @param[in] e_Dcm_b DCM from body to Earth frame
/// @return 3x3 matrix in [-]
/// @note See \cite Groves2013 Groves, ch. 14.2.3, eq. 14.48, p. 583 - 𝐅_15
[[nodiscard]] Eigen::Matrix3d e_F_dpsi_dw(const Eigen::Matrix3d& e_Dcm_b);

/// @brief Calculates the matrix 𝐅_𝛿v'_𝜓
/// @param[in] e_force_ib Specific force of the body with respect to inertial frame in [m / s^2], resolved in Earth frame coordinates
/// @return 3x3 matrix in [m / s^2]
/// @note See \cite Groves2013 Groves, ch. 14.2.3, eq. 14.49, p. 584 - 𝐅_21
[[nodiscard]] Eigen::Matrix3d e_F_dv_dpsi(const Eigen::Vector3d& e_force_ib);

/// @brief Calculates the matrix 𝐅_𝛿v'_𝛿v
/// @param[in] omega_ie omega_ie Angular velocity of the Earth in [rad/s]
/// @return 3x3 matrix in [1 / s]
/// @note See \cite Groves2013 Groves, ch. 14.2.3, eq. 14.48, p. 583 - 𝐅_22
[[nodiscard]] Eigen::Matrix3d e_F_dv_dv(double omega_ie);

/// @brief Calculates the matrix 𝐅_𝛿v'_𝛿r
/// @param[in] e_position Position in ECEF coordinates in [m]
/// @param[in] e_gravitation Gravitational acceleration in [m/s^2]
/// @param[in] r_eS_e Geocentric radius. The distance of a point on the Earth's surface from the center of the Earth in [m]
/// @param[in] e_omega_ie Angular velocity of Earth with respect to inertial system, represented in e-sys in [rad/s]
/// @return 3x3 matrix in [1 / s^2]
/// @note See \cite Groves2013 Groves, ch. 14.2.3, eq. 14.49, p. 584 - 𝐅_23
[[nodiscard]] Eigen::Matrix3d e_F_dv_dr(const Eigen::Vector3d& e_position, const Eigen::Vector3d& e_gravitation, double r_eS_e, const Eigen::Vector3d& e_omega_ie);

/// @brief Calculates the matrix 𝐅_𝜓'_𝛿f
/// @param[in] e_Dcm_b DCM from body to Earth frame
/// @return 3x3 matrix in [-]
/// @note See \cite Groves2013 Groves, ch. 14.2.3, eq. 14.48, p. 583 - 𝐅_24
[[nodiscard]] Eigen::Matrix3d e_F_dv_df(const Eigen::Matrix3d& e_Dcm_b);

/// @brief Calculates the matrix 𝐅_𝛿r'_𝛿v
/// @return 3x3 matrix in [-]
/// @note See \cite Groves2013 Groves, ch. 14.2.3, eq. 14.48, p. 583 - 𝐅_32
[[nodiscard]] Eigen::Matrix3d e_F_dr_dv();

/// @brief Calculates the matrix 𝐅_𝛿f'_𝛿f
/// @param[in] beta_a Gauss-Markov constant for the accelerometer 𝛽 = 1 / 𝜏 (𝜏 correlation length)
/// @return 3x3 matrix in [1 / s]
/// @note See T. Hobiger (2021) Inertialnavigation V06 - equation (6.3)
[[nodiscard]] Eigen::Matrix3d e_F_df_df(const Eigen::Vector3d& beta_a);

/// @brief Calculates the matrix 𝐅_𝛿ω'_𝛿ω
/// @param[in] beta_omega Gauss-Markov constant for the gyroscope 𝛽 = 1 / 𝜏 (𝜏 correlation length)
/// @return 3x3 matrix in [1 / s]
/// @note See T. Hobiger (2021) Inertialnavigation V06 - equation (6.3)
[[nodiscard]] Eigen::Matrix3d e_F_dw_dw(const Eigen::Vector3d& beta_omega);

} // namespace NAV