// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file ProcessNoise.hpp
/// @brief General process Noise definitions
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-01-31

#pragma once

#include <Eigen/Core>

namespace NAV
{

/// @brief Random walk noise input matrix 𝐆
/// @param[in] sigma2 Variance of the noise on the measurements
/// @note See T. Hobiger (2021) Inertialnavigation V06 - equation (6.3)
Eigen::Matrix3d G_RandomWalk(const Eigen::Vector3d& sigma2);

/// @brief Submatrix 𝐆_a of the noise input matrix 𝐆
/// @param[in] sigma2 Variance of the noise on the measurements
/// @param[in] beta Gauss-Markov constant 𝛽 = 1 / 𝜏 (𝜏 correlation length)
/// @note See T. Hobiger (2021) Inertialnavigation V06 - equation (6.3)
Eigen::Matrix3d G_GaussMarkov1(const Eigen::Vector3d& sigma2, const Eigen::Vector3d& beta);

/// @brief S_ra Power Spectral Density of the random noise
/// @param[in] sigma2_r 𝜎²_r standard deviation of the noise on the measurements
/// @param[in] tau_i 𝜏ᵢ interval between the input of successive outputs to the inertial navigation equations in [s]
/// @note See P. Groves (2013) - Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems (ch. 14.2.6)
[[nodiscard]] Eigen::Vector3d psdNoise(const Eigen::Vector3d& sigma2_r, const double& tau_i);

/// @brief S_bad Power Spectral Density of the bias variation
/// @param[in] sigma2_bd 𝜎²_bd standard deviation of the dynamic bias
/// @param[in] tau_bd 𝜏 Correlation length in [s]
/// @note See P. Groves (2013) - Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems (ch. 14.2.6)
[[nodiscard]] Eigen::Vector3d psdBiasVariation(const Eigen::Vector3d& sigma2_bd, const Eigen::Vector3d& tau_bd);

// /// @brief S_cPhi Power Spectral Density of the receiver clock phase drift
// /// @param[in] sigma2_cPhi 𝜎²_bd standard deviation of the receiver clock phase drift in [m]
// /// @param[in] tau_i 𝜏ᵢ interval between the input of successive outputs to the inertial navigation equations in [s]
// /// @note See P. Groves (2013) - Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems (ch. 9.4.2, eq. 9.157)
// double psdClockPhaseDrift(const double& sigma2_cPhi, const double& tau_i);

// /// @brief S_cf Power Spectral Density of the receiver clock frequency-drift
// /// @param sigma2_cf 𝜎²_bd standard deviation of the receiver clock frequency-drift in [m/s]
// /// @param tau_i 𝜏ᵢ interval between the input of successive outputs to the inertial navigation equations in [s]
// /// @return See P. Groves (2013) - Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems (ch. 9.4.2, eq. 9.157)
// double psdClockFreqDrift(const double& sigma2_cf, const double& tau_i);

/// @brief Submatrix 𝐐_11 of the system noise covariance matrix 𝐐
/// @param[in] S_rg Power Spectral Density of the gyroscope random noise
/// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix 𝐐_11
/// @note See Groves (2013) equation (14.81)
[[nodiscard]] Eigen::Matrix3d Q_psi_psi(const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const double& tau_s);

/// @brief Submatrix 𝐐_21 of the system noise covariance matrix 𝐐
/// @param[in] S_rg Power Spectral Density of the gyroscope random noise
/// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
/// @param[in] ien_F_21 Submatrix 𝐅_21 of the system matrix 𝐅 in {i,e,n} frame
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix 𝐐_21
/// @note See Groves (2013) equation (14.81)
[[nodiscard]] Eigen::Matrix3d ien_Q_dv_psi(const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& ien_F_21, const double& tau_s);

/// @brief Submatrix 𝐐_22 of the system noise covariance matrix 𝐐
/// @param[in] S_ra Power Spectral Density of the accelerometer random noise
/// @param[in] S_bad Power Spectral Density of the accelerometer bias variation
/// @param[in] S_rg Power Spectral Density of the gyroscope random noise
/// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
/// @param[in] ien_F_21 Submatrix 𝐅_21 of the system matrix 𝐅 in {i,e,n} frame
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix 𝐐_22
/// @note See Groves (2013) equation (14.81)
[[nodiscard]] Eigen::Matrix3d ien_Q_dv_dv(const Eigen::Vector3d& S_ra, const Eigen::Vector3d& S_bad, const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& ien_F_21, const double& tau_s);

/// @brief Submatrix 𝐐_25 of the system noise covariance matrix 𝐐
/// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
/// @param[in] ien_F_21 Submatrix 𝐅_21 of the system matrix 𝐅
/// @param[in] ien_Dcm_b Direction Cosine Matrix from body to {i,e,n} frame
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix 𝐐_25
/// @note See Groves (2013) equation (14.80)
[[nodiscard]] Eigen::Matrix3d ien_Q_dv_domega(const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& ien_F_21, const Eigen::Matrix3d& ien_Dcm_b, const double& tau_s);

/// @brief Submatrix 𝐐_31 of the system noise covariance matrix 𝐐
/// @param[in] S_rg Power Spectral Density of the gyroscope random noise
/// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
/// @param[in] n_F_21 Submatrix 𝐅_21 of the system matrix 𝐅
/// @param[in] T_rn_p Conversion matrix between cartesian and curvilinear perturbations to the position
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix 𝐐_31
/// @note See Groves (2013) equation (14.81)
[[nodiscard]] Eigen::Matrix3d n_Q_dr_psi(const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& n_F_21, const Eigen::Matrix3d& T_rn_p, const double& tau_s);

/// @brief Submatrix 𝐐_31 of the system noise covariance matrix 𝐐
/// @param[in] S_rg Power Spectral Density of the gyroscope random noise
/// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
/// @param[in] ie_F_21 Submatrix 𝐅_21 of the system matrix 𝐅 in {i,e} frame
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix 𝐐_31
/// @note See Groves (2013) equation (14.81)
[[nodiscard]] Eigen::Matrix3d ie_Q_dr_psi(const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& ie_F_21, const double& tau_s);

/// @brief Submatrix 𝐐_32 of the system noise covariance matrix 𝐐
/// @param[in] S_ra Power Spectral Density of the accelerometer random noise
/// @param[in] S_bad Power Spectral Density of the accelerometer bias variation
/// @param[in] S_rg Power Spectral Density of the gyroscope random noise
/// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
/// @param[in] n_F_21 Submatrix 𝐅_21 of the system matrix 𝐅
/// @param[in] T_rn_p Conversion matrix between cartesian and curvilinear perturbations to the position
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix 𝐐_32
/// @note See Groves (2013) equation (14.81)
[[nodiscard]] Eigen::Matrix3d n_Q_dr_dv(const Eigen::Vector3d& S_ra, const Eigen::Vector3d& S_bad, const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& n_F_21, const Eigen::Matrix3d& T_rn_p, const double& tau_s);

/// @brief Submatrix 𝐐_32 of the system noise covariance matrix 𝐐
/// @param[in] S_ra Power Spectral Density of the accelerometer random noise
/// @param[in] S_bad Power Spectral Density of the accelerometer bias variation
/// @param[in] S_rg Power Spectral Density of the gyroscope random noise
/// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
/// @param[in] ie_F_21 Submatrix 𝐅_21 of the system matrix 𝐅 in {i,e} frame
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix 𝐐_32
/// @note See Groves (2013) equation (14.81)
[[nodiscard]] Eigen::Matrix3d ie_Q_dr_dv(const Eigen::Vector3d& S_ra, const Eigen::Vector3d& S_bad, const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& ie_F_21, const double& tau_s);

/// @brief Submatrix 𝐐_33 of the system noise covariance matrix 𝐐
/// @param[in] S_ra Power Spectral Density of the accelerometer random noise
/// @param[in] S_bad Power Spectral Density of the accelerometer bias variation
/// @param[in] S_rg Power Spectral Density of the gyroscope random noise
/// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
/// @param[in] n_F_21 Submatrix 𝐅_21 of the system matrix 𝐅
/// @param[in] T_rn_p Conversion matrix between cartesian and curvilinear perturbations to the position
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix 𝐐_33
/// @note See Groves (2013) equation (14.81)
[[nodiscard]] Eigen::Matrix3d n_Q_dr_dr(const Eigen::Vector3d& S_ra, const Eigen::Vector3d& S_bad, const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& n_F_21, const Eigen::Matrix3d& T_rn_p, const double& tau_s);

/// @brief Submatrix 𝐐_33 of the system noise covariance matrix 𝐐
/// @param[in] S_ra Power Spectral Density of the accelerometer random noise
/// @param[in] S_bad Power Spectral Density of the accelerometer bias variation
/// @param[in] S_rg Power Spectral Density of the gyroscope random noise
/// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
/// @param[in] ie_F_21 Submatrix 𝐅_21 of the system matrix 𝐅 in {i,e} frame
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix 𝐐_33
/// @note See Groves (2013) equation (14.81)
[[nodiscard]] Eigen::Matrix3d ie_Q_dr_dr(const Eigen::Vector3d& S_ra, const Eigen::Vector3d& S_bad, const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& ie_F_21, const double& tau_s);

/// @brief Submatrix 𝐐_34 of the system noise covariance matrix 𝐐
/// @param[in] S_bad Power Spectral Density of the accelerometer bias variation
/// @param[in] T_rn_p Conversion matrix between cartesian and curvilinear perturbations to the position
/// @param[in] n_Dcm_b Direction Cosine Matrix from body to navigation coordinates
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix 𝐐_34
/// @note See Groves (2013) equation (14.81)
[[nodiscard]] Eigen::Matrix3d n_Q_dr_df(const Eigen::Vector3d& S_bad, const Eigen::Matrix3d& T_rn_p, const Eigen::Matrix3d& n_Dcm_b, const double& tau_s);

/// @brief Submatrix 𝐐_34 of the system noise covariance matrix 𝐐
/// @param[in] S_bad Power Spectral Density of the accelerometer bias variation
/// @param[in] ie_Dcm_b Direction Cosine Matrix from body to {i,e} coordinates
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix 𝐐_34
/// @note See Groves (2013) equation (14.81)
[[nodiscard]] Eigen::Matrix3d ie_Q_dr_df(const Eigen::Vector3d& S_bad, const Eigen::Matrix3d& ie_Dcm_b, const double& tau_s);

/// @brief Submatrix 𝐐_35 of the system noise covariance matrix 𝐐
/// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
/// @param[in] n_F_21 Submatrix 𝐅_21 of the system matrix 𝐅
/// @param[in] T_rn_p Conversion matrix between cartesian and curvilinear perturbations to the position
/// @param[in] n_Dcm_b Direction Cosine Matrix from body to navigation coordinates
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix 𝐐_35
/// @note See Groves (2013) equation (14.81)
[[nodiscard]] Eigen::Matrix3d n_Q_dr_domega(const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& n_F_21, const Eigen::Matrix3d& T_rn_p, const Eigen::Matrix3d& n_Dcm_b, const double& tau_s);

/// @brief Submatrix 𝐐_35 of the system noise covariance matrix 𝐐
/// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
/// @param[in] ie_F_21 Submatrix 𝐅_21 of the system matrix 𝐅
/// @param[in] ie_Dcm_b Direction Cosine Matrix from body to {i,e} coordinates
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix 𝐐_35
/// @note See Groves (2013) equation (14.81)
[[nodiscard]] Eigen::Matrix3d ie_Q_dr_domega(const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& ie_F_21, const Eigen::Matrix3d& ie_Dcm_b, const double& tau_s);

/// @brief Submatrix 𝐐_42 of the system noise covariance matrix 𝐐
/// @param[in] S_bad Power Spectral Density of the accelerometer bias variation
/// @param[in] b_Dcm_ien Direction Cosine Matrix from {i,e,n} to body coordinates
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix 𝐐_42
/// @note See Groves (2013) equation (14.80)
[[nodiscard]] Eigen::Matrix3d Q_df_dv(const Eigen::Vector3d& S_bad, const Eigen::Matrix3d& b_Dcm_ien, const double& tau_s);

/// @brief Submatrix 𝐐_44 of the system noise covariance matrix 𝐐
/// @param[in] S_bad Power Spectral Density of the accelerometer bias variation
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix 𝐐_44
/// @note See Groves (2013) equation (14.80)
[[nodiscard]] Eigen::Matrix3d Q_df_df(const Eigen::Vector3d& S_bad, const double& tau_s);

/// @brief Submatrix 𝐐_51 of the system noise covariance matrix 𝐐
/// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
/// @param[in] b_Dcm_ien Direction Cosine Matrix from {i,e,n} to body coordinates
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix 𝐐_51
/// @note See Groves (2013) equation (14.80)
[[nodiscard]] Eigen::Matrix3d Q_domega_psi(const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& b_Dcm_ien, const double& tau_s);

/// @brief Submatrix 𝐐_55 of the system noise covariance matrix 𝐐
/// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix 𝐐_55
/// @note See Groves (2013) equation (14.80)
[[nodiscard]] Eigen::Matrix3d Q_domega_domega(const Eigen::Vector3d& S_bgd, const double& tau_s);

/// @brief Submatrix 𝐐_66 of the system noise covariance matrix 𝐐
/// @param[in] S_cPhi Power Spectral Density of the receiver clock phase drift in [m^2 s^-1]
/// @param[in] S_cf Power Spectral Density of the receiver clock frequency-drift [m^2 s^-3]
/// @param[in] tau_s Time interval in [s]
/// @return See Groves (2013) equation (9.152)
[[nodiscard]] Eigen::Matrix2d Q_gnss(const double& S_cPhi, const double& S_cf, const double& tau_s);

} // namespace NAV