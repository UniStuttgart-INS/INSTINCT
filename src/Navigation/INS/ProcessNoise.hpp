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

/// @brief Random walk noise input matrix ๐
/// @param[in] sigma2 Variance of the noise on the measurements
/// @note See T. Hobiger (2021) Inertialnavigation V06 - equation (6.3)
Eigen::Matrix3d G_RandomWalk(const Eigen::Vector3d& sigma2);

/// @brief Submatrix ๐_a of the noise input matrix ๐
/// @param[in] sigma2 Variance of the noise on the measurements
/// @param[in] beta Gauss-Markov constant ๐ฝ = 1 / ๐ (๐ correlation length)
/// @note See T. Hobiger (2021) Inertialnavigation V06 - equation (6.3)
Eigen::Matrix3d G_GaussMarkov1(const Eigen::Vector3d& sigma2, const Eigen::Vector3d& beta);

/// @brief S_ra Power Spectral Density of the random noise
/// @param[in] sigma2_r ๐ยฒ_r standard deviation of the noise on the measurements
/// @param[in] tau_i ๐แตข interval between the input of successive outputs to the inertial navigation equations in [s]
/// @note See P. Groves (2013) - Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems (ch. 14.2.6)
[[nodiscard]] Eigen::Vector3d psdNoise(const Eigen::Vector3d& sigma2_r, const double& tau_i);

/// @brief S_bad Power Spectral Density of the bias variation
/// @param[in] sigma2_bd ๐ยฒ_bd standard deviation of the dynamic bias
/// @param[in] tau_bd ๐ Correlation length in [s]
/// @note See P. Groves (2013) - Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems (ch. 14.2.6)
[[nodiscard]] Eigen::Vector3d psdBiasVariation(const Eigen::Vector3d& sigma2_bd, const Eigen::Vector3d& tau_bd);

/// @brief Submatrix ๐_11 of the system noise covariance matrix ๐
/// @param[in] S_rg Power Spectral Density of the gyroscope random noise
/// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix ๐_11
/// @note See Groves (2013) equation (14.81)
[[nodiscard]] Eigen::Matrix3d Q_psi_psi(const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const double& tau_s);

/// @brief Submatrix ๐_21 of the system noise covariance matrix ๐
/// @param[in] S_rg Power Spectral Density of the gyroscope random noise
/// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
/// @param[in] ien_F_21 Submatrix ๐_21 of the system matrix ๐ in {i,e,n} frame
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix ๐_21
/// @note See Groves (2013) equation (14.81)
[[nodiscard]] Eigen::Matrix3d ien_Q_dv_psi(const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& ien_F_21, const double& tau_s);

/// @brief Submatrix ๐_22 of the system noise covariance matrix ๐
/// @param[in] S_ra Power Spectral Density of the accelerometer random noise
/// @param[in] S_bad Power Spectral Density of the accelerometer bias variation
/// @param[in] S_rg Power Spectral Density of the gyroscope random noise
/// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
/// @param[in] ien_F_21 Submatrix ๐_21 of the system matrix ๐ in {i,e,n} frame
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix ๐_22
/// @note See Groves (2013) equation (14.81)
[[nodiscard]] Eigen::Matrix3d ien_Q_dv_dv(const Eigen::Vector3d& S_ra, const Eigen::Vector3d& S_bad, const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& ien_F_21, const double& tau_s);

/// @brief Submatrix ๐_25 of the system noise covariance matrix ๐
/// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
/// @param[in] ien_F_21 Submatrix ๐_21 of the system matrix ๐
/// @param[in] ien_Dcm_b Direction Cosine Matrix from body to {i,e,n} frame
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix ๐_25
/// @note See Groves (2013) equation (14.80)
[[nodiscard]] Eigen::Matrix3d ien_Q_dv_domega(const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& ien_F_21, const Eigen::Matrix3d& ien_Dcm_b, const double& tau_s);

/// @brief Submatrix ๐_31 of the system noise covariance matrix ๐
/// @param[in] S_rg Power Spectral Density of the gyroscope random noise
/// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
/// @param[in] n_F_21 Submatrix ๐_21 of the system matrix ๐
/// @param[in] T_rn_p Conversion matrix between cartesian and curvilinear perturbations to the position
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix ๐_31
/// @note See Groves (2013) equation (14.81)
[[nodiscard]] Eigen::Matrix3d n_Q_dr_psi(const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& n_F_21, const Eigen::Matrix3d& T_rn_p, const double& tau_s);

/// @brief Submatrix ๐_31 of the system noise covariance matrix ๐
/// @param[in] S_rg Power Spectral Density of the gyroscope random noise
/// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
/// @param[in] ie_F_21 Submatrix ๐_21 of the system matrix ๐ in {i,e} frame
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix ๐_31
/// @note See Groves (2013) equation (14.81)
[[nodiscard]] Eigen::Matrix3d ie_Q_dr_psi(const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& ie_F_21, const double& tau_s);

/// @brief Submatrix ๐_32 of the system noise covariance matrix ๐
/// @param[in] S_ra Power Spectral Density of the accelerometer random noise
/// @param[in] S_bad Power Spectral Density of the accelerometer bias variation
/// @param[in] S_rg Power Spectral Density of the gyroscope random noise
/// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
/// @param[in] n_F_21 Submatrix ๐_21 of the system matrix ๐
/// @param[in] T_rn_p Conversion matrix between cartesian and curvilinear perturbations to the position
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix ๐_32
/// @note See Groves (2013) equation (14.81)
[[nodiscard]] Eigen::Matrix3d n_Q_dr_dv(const Eigen::Vector3d& S_ra, const Eigen::Vector3d& S_bad, const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& n_F_21, const Eigen::Matrix3d& T_rn_p, const double& tau_s);

/// @brief Submatrix ๐_32 of the system noise covariance matrix ๐
/// @param[in] S_ra Power Spectral Density of the accelerometer random noise
/// @param[in] S_bad Power Spectral Density of the accelerometer bias variation
/// @param[in] S_rg Power Spectral Density of the gyroscope random noise
/// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
/// @param[in] ie_F_21 Submatrix ๐_21 of the system matrix ๐ in {i,e} frame
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix ๐_32
/// @note See Groves (2013) equation (14.81)
[[nodiscard]] Eigen::Matrix3d ie_Q_dr_dv(const Eigen::Vector3d& S_ra, const Eigen::Vector3d& S_bad, const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& ie_F_21, const double& tau_s);

/// @brief Submatrix ๐_33 of the system noise covariance matrix ๐
/// @param[in] S_ra Power Spectral Density of the accelerometer random noise
/// @param[in] S_bad Power Spectral Density of the accelerometer bias variation
/// @param[in] S_rg Power Spectral Density of the gyroscope random noise
/// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
/// @param[in] n_F_21 Submatrix ๐_21 of the system matrix ๐
/// @param[in] T_rn_p Conversion matrix between cartesian and curvilinear perturbations to the position
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix ๐_33
/// @note See Groves (2013) equation (14.81)
[[nodiscard]] Eigen::Matrix3d n_Q_dr_dr(const Eigen::Vector3d& S_ra, const Eigen::Vector3d& S_bad, const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& n_F_21, const Eigen::Matrix3d& T_rn_p, const double& tau_s);

/// @brief Submatrix ๐_33 of the system noise covariance matrix ๐
/// @param[in] S_ra Power Spectral Density of the accelerometer random noise
/// @param[in] S_bad Power Spectral Density of the accelerometer bias variation
/// @param[in] S_rg Power Spectral Density of the gyroscope random noise
/// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
/// @param[in] ie_F_21 Submatrix ๐_21 of the system matrix ๐ in {i,e} frame
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix ๐_33
/// @note See Groves (2013) equation (14.81)
[[nodiscard]] Eigen::Matrix3d ie_Q_dr_dr(const Eigen::Vector3d& S_ra, const Eigen::Vector3d& S_bad, const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& ie_F_21, const double& tau_s);

/// @brief Submatrix ๐_34 of the system noise covariance matrix ๐
/// @param[in] S_bad Power Spectral Density of the accelerometer bias variation
/// @param[in] T_rn_p Conversion matrix between cartesian and curvilinear perturbations to the position
/// @param[in] n_Dcm_b Direction Cosine Matrix from body to navigation coordinates
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix ๐_34
/// @note See Groves (2013) equation (14.81)
[[nodiscard]] Eigen::Matrix3d n_Q_dr_df(const Eigen::Vector3d& S_bad, const Eigen::Matrix3d& T_rn_p, const Eigen::Matrix3d& n_Dcm_b, const double& tau_s);

/// @brief Submatrix ๐_34 of the system noise covariance matrix ๐
/// @param[in] S_bad Power Spectral Density of the accelerometer bias variation
/// @param[in] ie_Dcm_b Direction Cosine Matrix from body to {i,e} coordinates
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix ๐_34
/// @note See Groves (2013) equation (14.81)
[[nodiscard]] Eigen::Matrix3d ie_Q_dr_df(const Eigen::Vector3d& S_bad, const Eigen::Matrix3d& ie_Dcm_b, const double& tau_s);

/// @brief Submatrix ๐_35 of the system noise covariance matrix ๐
/// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
/// @param[in] n_F_21 Submatrix ๐_21 of the system matrix ๐
/// @param[in] T_rn_p Conversion matrix between cartesian and curvilinear perturbations to the position
/// @param[in] n_Dcm_b Direction Cosine Matrix from body to navigation coordinates
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix ๐_35
/// @note See Groves (2013) equation (14.81)
[[nodiscard]] Eigen::Matrix3d n_Q_dr_domega(const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& n_F_21, const Eigen::Matrix3d& T_rn_p, const Eigen::Matrix3d& n_Dcm_b, const double& tau_s);

/// @brief Submatrix ๐_35 of the system noise covariance matrix ๐
/// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
/// @param[in] ie_F_21 Submatrix ๐_21 of the system matrix ๐
/// @param[in] ie_Dcm_b Direction Cosine Matrix from body to {i,e} coordinates
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix ๐_35
/// @note See Groves (2013) equation (14.81)
[[nodiscard]] Eigen::Matrix3d ie_Q_dr_domega(const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& ie_F_21, const Eigen::Matrix3d& ie_Dcm_b, const double& tau_s);

/// @brief Submatrix ๐_42 of the system noise covariance matrix ๐
/// @param[in] S_bad Power Spectral Density of the accelerometer bias variation
/// @param[in] b_Dcm_ien Direction Cosine Matrix from {i,e,n} to body coordinates
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix ๐_42
/// @note See Groves (2013) equation (14.80)
[[nodiscard]] Eigen::Matrix3d Q_df_dv(const Eigen::Vector3d& S_bad, const Eigen::Matrix3d& b_Dcm_ien, const double& tau_s);

/// @brief Submatrix ๐_44 of the system noise covariance matrix ๐
/// @param[in] S_bad Power Spectral Density of the accelerometer bias variation
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix ๐_44
/// @note See Groves (2013) equation (14.80)
[[nodiscard]] Eigen::Matrix3d Q_df_df(const Eigen::Vector3d& S_bad, const double& tau_s);

/// @brief Submatrix ๐_51 of the system noise covariance matrix ๐
/// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
/// @param[in] b_Dcm_ien Direction Cosine Matrix from {i,e,n} to body coordinates
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix ๐_51
/// @note See Groves (2013) equation (14.80)
[[nodiscard]] Eigen::Matrix3d Q_domega_psi(const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& b_Dcm_ien, const double& tau_s);

/// @brief Submatrix ๐_55 of the system noise covariance matrix ๐
/// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix ๐_55
/// @note See Groves (2013) equation (14.80)
[[nodiscard]] Eigen::Matrix3d Q_domega_domega(const Eigen::Vector3d& S_bgd, const double& tau_s);

} // namespace NAV