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

} // namespace NAV