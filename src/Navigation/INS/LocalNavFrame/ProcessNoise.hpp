/// @file ProcessNoise.hpp
/// @brief Process Noise for the local navigation frame
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
/// @param[in] i_tau 𝜏ᵢ interval between the input of successive outputs to the inertial navigation equations in [s]
/// @note See P. Groves (2013) - Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems (ch. 14.2.6)
[[nodiscard]] Eigen::Vector3d psdNoise(const Eigen::Vector3d& sigma2_r, const double& i_tau);

/// @brief S_bad Power Spectral Density of the bias variation
/// @param[in] sigma2_bd 𝜎²_bd standard deviation of the dynamic bias
/// @param[in] tau_bd 𝜏 Correlation length in [s]
/// @note See P. Groves (2013) - Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems (ch. 14.2.6)
[[nodiscard]] Eigen::Vector3d psdBiasVariation(const Eigen::Vector3d& sigma2_bd, const Eigen::Vector3d& tau_bd);

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
/// @param[in] n_F_21 Submatrix 𝐅_21 of the system matrix 𝐅
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix 𝐐_21
/// @note See Groves (2013) equation (14.81)
[[nodiscard]] Eigen::Matrix3d Q_dv_psi(const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& n_F_21, const double& tau_s);

/// @brief Submatrix 𝐐_22 of the system noise covariance matrix 𝐐
/// @param[in] S_ra Power Spectral Density of the accelerometer random noise
/// @param[in] S_bad Power Spectral Density of the accelerometer bias variation
/// @param[in] S_rg Power Spectral Density of the gyroscope random noise
/// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
/// @param[in] n_F_21 Submatrix 𝐅_21 of the system matrix 𝐅
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix 𝐐_22
/// @note See Groves (2013) equation (14.81)
[[nodiscard]] Eigen::Matrix3d Q_dv_dv(const Eigen::Vector3d& S_ra, const Eigen::Vector3d& S_bad, const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& n_F_21, const double& tau_s);

/// @brief Submatrix 𝐐_25 of the system noise covariance matrix 𝐐
/// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
/// @param[in] n_F_21 Submatrix 𝐅_21 of the system matrix 𝐅
/// @param[in] b_n_Dcm Direction Cosine Matrix from body to navigation coordinates
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix 𝐐_25
/// @note See Groves (2013) equation (14.80)
[[nodiscard]] Eigen::Matrix3d Q_dv_domega(const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& n_F_21, const Eigen::Matrix3d& b_n_Dcm, const double& tau_s);

/// @brief Submatrix 𝐐_31 of the system noise covariance matrix 𝐐
/// @param[in] S_rg Power Spectral Density of the gyroscope random noise
/// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
/// @param[in] n_F_21 Submatrix 𝐅_21 of the system matrix 𝐅
/// @param[in] T_rn_p Conversion matrix between cartesian and curvilinear perturbations to the position
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix 𝐐_31
/// @note See Groves (2013) equation (14.81)
[[nodiscard]] Eigen::Matrix3d Q_dr_psi(const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& n_F_21, const Eigen::Matrix3d& T_rn_p, const double& tau_s);

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
[[nodiscard]] Eigen::Matrix3d Q_dr_dv(const Eigen::Vector3d& S_ra, const Eigen::Vector3d& S_bad, const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& n_F_21, const Eigen::Matrix3d& T_rn_p, const double& tau_s);

/// @brief Submatrix 𝐐_33 of the system noise covariance matrix 𝐐
/// @param[in] S_ra Power Spectral Density of the accelerometer random noise
/// @param[in] S_bad Power Spectral Density of the accelerometer bias variation
/// @param[in] S_rg Power Spectral Density of the gyroscope random noise
/// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
/// @param[in] T_rn_p Conversion matrix between cartesian and curvilinear perturbations to the position
/// @param[in] n_F_21 Submatrix 𝐅_21 of the system matrix 𝐅
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix 𝐐_33
/// @note See Groves (2013) equation (14.81)
[[nodiscard]] Eigen::Matrix3d Q_dr_dr(const Eigen::Vector3d& S_ra, const Eigen::Vector3d& S_bad, const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& T_rn_p, const Eigen::Matrix3d& n_F_21, const double& tau_s);

/// @brief Submatrix 𝐐_34 of the system noise covariance matrix 𝐐
/// @param[in] S_bad Power Spectral Density of the accelerometer bias variation
/// @param[in] T_rn_p Conversion matrix between cartesian and curvilinear perturbations to the position
/// @param[in] b_n_Dcm Direction Cosine Matrix from body to navigation coordinates
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix 𝐐_34
/// @note See Groves (2013) equation (14.81)
[[nodiscard]] Eigen::Matrix3d Q_dr_df(const Eigen::Vector3d& S_bad, const Eigen::Matrix3d& T_rn_p, const Eigen::Matrix3d& b_n_Dcm, const double& tau_s);

/// @brief Submatrix 𝐐_35 of the system noise covariance matrix 𝐐
/// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
/// @param[in] n_F_21 Submatrix 𝐅_21 of the system matrix 𝐅
/// @param[in] T_rn_p Conversion matrix between cartesian and curvilinear perturbations to the position
/// @param[in] b_n_Dcm Direction Cosine Matrix from body to navigation coordinates
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix 𝐐_35
/// @note See Groves (2013) equation (14.81)
[[nodiscard]] Eigen::Matrix3d Q_dr_domega(const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& n_F_21, const Eigen::Matrix3d& T_rn_p, const Eigen::Matrix3d& b_n_Dcm, const double& tau_s);

/// @brief Submatrix 𝐐_42 of the system noise covariance matrix 𝐐
/// @param[in] S_bad Power Spectral Density of the accelerometer bias variation
/// @param[in] b_n_Dcm Direction Cosine Matrix from body to navigation coordinates
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix 𝐐_42
/// @note See Groves (2013) equation (14.80)
[[nodiscard]] Eigen::Matrix3d Q_df_dv(const Eigen::Vector3d& S_bad, const Eigen::Matrix3d& b_n_Dcm, const double& tau_s);

/// @brief Submatrix 𝐐_44 of the system noise covariance matrix 𝐐
/// @param[in] S_bad Power Spectral Density of the accelerometer bias variation
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix 𝐐_44
/// @note See Groves (2013) equation (14.80)
[[nodiscard]] Eigen::Matrix3d Q_df_df(const Eigen::Vector3d& S_bad, const double& tau_s);

/// @brief Submatrix 𝐐_51 of the system noise covariance matrix 𝐐
/// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
/// @param[in] b_n_Dcm Direction Cosine Matrix from body to navigation coordinates
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix 𝐐_51
/// @note See Groves (2013) equation (14.80)
[[nodiscard]] Eigen::Matrix3d Q_domega_psi(const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& b_n_Dcm, const double& tau_s);

/// @brief Submatrix 𝐐_55 of the system noise covariance matrix 𝐐
/// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix 𝐐_55
/// @note See Groves (2013) equation (14.80)
[[nodiscard]] Eigen::Matrix3d Q_domega_domega(const Eigen::Vector3d& S_bgd, const double& tau_s);

} // namespace NAV