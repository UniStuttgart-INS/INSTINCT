/// @file ErrorEquations.hpp
/// @brief Error Equations for the local navigation frame
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-01-12

#pragma once

#include <Eigen/Core>

namespace NAV
{
/// @brief Calculates the matrix 𝐅_𝜓'_𝜓
/// @param[in] omega_in_n Angular rate vector of the n-system with respect to the i-system in [rad / s], resolved in the n-system
/// @return 3x3 matrix in [rad / s]
/// @note See T. Hobiger (2021) Inertialnavigation V07 - equation (7.22)
/// @note See Groves (2013) equation (14.64)
[[nodiscard]] Eigen::Matrix3d F_dotpsi_psi_n(const Eigen::Vector3d& omega_in_n);

/// @brief Calculates the matrix 𝐅_𝜓'_𝛿v
/// @param[in] latitude Geodetic latitude of the body in [rad]
/// @param[in] height Geodetic height of the body in [m]
/// @param[in] R_N North/South (meridian) earth radius in [m]
/// @param[in] R_E East/West (prime vertical) earth radius in [m]
/// @return 3x3 matrix in [1 / m]
/// @note See T. Hobiger (2021) Inertialnavigation V07 - equation (7.21)
/// @note See Groves (2013) equation (14.65)
[[nodiscard]] Eigen::Matrix3d F_dotpsi_dv_n(double latitude, double height, double R_N, double R_E);

/// @brief Calculates the matrix 𝐅_𝜓'_𝛿r
/// @param[in] latitude Geodetic latitude of the body in [rad]
/// @param[in] height Geodetic height of the body in [m]
/// @param[in] v_n Velocity of the body with respect to the e-system in [m / s], resolved in the n-system
/// @param[in] R_N North/South (meridian) earth radius in [m]
/// @param[in] R_E East/West (prime vertical) earth radius in [m]
/// @return 3x3 matrix in [rad / s] for latitude and [1 / (m · s)] for height
/// @note See T. Hobiger (2021) Inertialnavigation V07 - equation (7.21)
/// @note See Groves (2013) equation (14.66)
[[nodiscard]] Eigen::Matrix3d F_dotpsi_dr_n(double latitude, double height, const Eigen::Vector3d& v_n, double R_N, double R_E);

/// @brief Calculates the matrix 𝐅_𝜓'_𝛿ω
/// @param[in] C_nb DCM from body to navigation frame
/// @return 3x3 matrix in [-]
[[nodiscard]] Eigen::Matrix3d F_dotpsi_dw_n(const Eigen::Matrix3d& C_nb);

/// @brief Calculates the matrix 𝐅_𝛿v'_𝜓
/// @param[in] f_ib_n Specific force of the body with respect to inertial frame in [m / s^2], resolved in local navigation frame coordinates
/// @return 3x3 matrix in [m / s^2]
/// @note See T. Hobiger (2021) Inertialnavigation V08 - equation (8.4)
/// @note See Groves (2013) equation (14.67)
[[nodiscard]] Eigen::Matrix3d F_dotdv_psi_n(const Eigen::Vector3d& f_ib_n);

/// @brief Calculates the matrix 𝐅_𝛿v'_𝛿v
/// @param[in] v_n Velocity of the body with respect to the e-system in [m / s], resolved in the n-system
/// @param[in] latitude Geodetic latitude of the body in [rad]
/// @param[in] height Geodetic height of the body in [m]
/// @param[in] R_N North/South (meridian) earth radius in [m]
/// @param[in] R_E East/West (prime vertical) earth radius in [m]
/// @return 3x3 matrix in [1 / s]
/// @note See T. Hobiger (2021) Inertialnavigation V08 - equation (8.6, 8.15)
/// @note See Groves (2013) equation (14.68)
[[nodiscard]] Eigen::Matrix3d F_dotdv_dv_n(const Eigen::Vector3d& v_n, double latitude, double height, double R_N, double R_E);

/// @brief Calculates the matrix 𝐅_𝛿v'_𝛿r
/// @param[in] v_n Velocity of the body with respect to the e-system in [m / s], resolved in the n-system
/// @param[in] latitude Geodetic latitude of the body in [rad]
/// @param[in] height Geodetic height of the body in [m]
/// @param[in] R_N North/South (meridian) earth radius in [m]
/// @param[in] R_E East/West (prime vertical) earth radius in [m]
/// @return 3x3 matrix in [m / s^2] for latitude and [1 / s^2] for height
/// @note See T. Hobiger (2021) Inertialnavigation V08 - equation (8.14, 8.16)
/// @note See Groves (2013) equation (14.69)
[[nodiscard]] Eigen::Matrix3d F_dotdv_dr_n(const Eigen::Vector3d& v_n, double latitude, double height, double R_N, double R_E);

/// @brief Calculates the matrix 𝐅_𝜓'_𝛿f
/// @param[in] C_nb DCM from body to navigation frame
/// @return 3x3 matrix in [-]
[[nodiscard]] Eigen::Matrix3d F_dotdv_df_n(const Eigen::Matrix3d& C_nb);

/// @brief Calculates the matrix 𝐅_𝛿r'_𝛿v
/// @param[in] latitude Geodetic latitude of the body in [rad]
/// @param[in] height Geodetic height of the body in [m]
/// @param[in] R_N North/South (meridian) earth radius in [m]
/// @param[in] R_E East/West (prime vertical) earth radius in [m]
/// @return 3x3 matrix in [1 / m] for latitude and longitude and [-] for height
/// @note See T. Hobiger (2021) Inertialnavigation V07 - equation (7.5)
/// @note See Groves (2013) equation (14.70)
[[nodiscard]] Eigen::Matrix3d F_dotdr_dv_n(double latitude, double height, double R_N, double R_E);

/// @brief Calculates the matrix 𝐅_𝛿r'_𝛿r
/// @param[in] v_n Velocity of the body with respect to the e-system in [m / s], resolved in the n-system
/// @param[in] latitude Geodetic latitude of the body in [rad]
/// @param[in] height Geodetic height of the body in [m]
/// @param[in] R_N North/South (meridian) earth radius in [m]
/// @param[in] R_E East/West (prime vertical) earth radius in [m]
/// @return 3x3 matrix in [1 / s] for latitude and [1 / (m · s)] for height
/// @note See T. Hobiger (2021) Inertialnavigation V07 - equation (7.5)
/// @note See Groves (2013) equation (14.71)
[[nodiscard]] Eigen::Matrix3d F_dotdr_dr_n(const Eigen::Vector3d& v_n, double latitude, double height, double R_N, double R_E);

/// @brief Calculates the matrix 𝐅_𝛿f'_𝛿f
/// @param[in] beta_a Gauss-Markov constant for the accelerometer 𝛽 = 1 / 𝜏 (𝜏 correlation length)
/// @return 3x3 matrix in [1 / s]
/// @note See T. Hobiger (2021) Inertialnavigation V06 - equation (6.3)
[[nodiscard]] Eigen::Matrix3d F_dotdf_df_n(const Eigen::Vector3d& beta_a);

/// @brief Calculates the matrix 𝐅_𝛿ω'_𝛿ω
/// @param[in] beta_omega Gauss-Markov constant for the gyroscope 𝛽 = 1 / 𝜏 (𝜏 correlation length)
/// @return 3x3 matrix in [1 / s]
/// @note See T. Hobiger (2021) Inertialnavigation V06 - equation (6.3)
[[nodiscard]] Eigen::Matrix3d F_dotdw_dw_n(const Eigen::Vector3d& beta_omega);

} // namespace NAV