/// @file ErrorEquations.hpp
/// @brief Error Equations for the local navigation frame
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-01-12

#pragma once

#include <Eigen/Core>

namespace NAV
{
/// @brief Calculates the matrix 𝐅_𝜓'_𝜓
/// @param[in] n_omega_in Angular rate vector of the n-system with respect to the i-system in [rad / s], resolved in the n-system
/// @return 3x3 matrix in [rad / s]
/// @note See \cite Groves2013 Groves, ch. 14.2.4, eq. 14.64, p. 587 - 𝐅_11
/// @note See T. Hobiger (2021) Inertialnavigation V07 - equation (7.22)
[[nodiscard]] Eigen::Matrix3d n_F_dpsi_dpsi(const Eigen::Vector3d& n_omega_in);

/// @brief Calculates the matrix 𝐅_𝜓'_𝛿v
/// @param[in] latitude Geodetic latitude of the body in [rad]
/// @param[in] height Geodetic height of the body in [m]
/// @param[in] R_N North/South (meridian) earth radius in [m]
/// @param[in] R_E East/West (prime vertical) earth radius in [m]
/// @return 3x3 matrix in [1 / m]
/// @note See \cite Groves2013 Groves, ch. 14.2.4, eq. 14.65, p. 587 - 𝐅_12
/// @note See T. Hobiger (2021) Inertialnavigation V07 - equation (7.21)
[[nodiscard]] Eigen::Matrix3d n_F_dpsi_dv(double latitude, double height, double R_N, double R_E);

/// @brief Calculates the matrix 𝐅_𝜓'_𝛿r
/// @param[in] latitude Geodetic latitude of the body in [rad]
/// @param[in] height Geodetic height of the body in [m]
/// @param[in] n_velocity Velocity of the body with respect to the e-system in [m / s], resolved in the n-system
/// @param[in] R_N North/South (meridian) earth radius in [m]
/// @param[in] R_E East/West (prime vertical) earth radius in [m]
/// @return 3x3 matrix in [rad / s] for latitude and [1 / (m · s)] for height
/// @note See \cite Groves2013 Groves, ch. 14.2.4, eq. 14.66, p. 587 - 𝐅_13
/// @note See T. Hobiger (2021) Inertialnavigation V07 - equation (7.21)
[[nodiscard]] Eigen::Matrix3d n_F_dpsi_dr(double latitude, double height, const Eigen::Vector3d& n_velocity, double R_N, double R_E);

/// @brief Calculates the matrix 𝐅_𝜓'_𝛿ω
/// @param[in] n_Dcm_b DCM from body to navigation frame
/// @return 3x3 matrix in [-]
/// @note See \cite Groves2013 Groves, ch. 14.2.4, eq. 14.63, p. 587 - 𝐅_15
[[nodiscard]] Eigen::Matrix3d n_F_dpsi_dw(const Eigen::Matrix3d& n_Dcm_b);

/// @brief Calculates the matrix 𝐅_𝛿v'_𝜓
/// @param[in] n_force_ib Specific force of the body with respect to inertial frame in [m / s^2], resolved in local navigation frame coordinates
/// @return 3x3 matrix in [m / s^2]
/// @note See \cite Groves2013 Groves, ch. 14.2.4, eq. 14.67, p. 587 - 𝐅_21
/// @note See T. Hobiger (2021) Inertialnavigation V08 - equation (8.4)
[[nodiscard]] Eigen::Matrix3d n_F_dv_dpsi(const Eigen::Vector3d& n_force_ib);

/// @brief Calculates the matrix 𝐅_𝛿v'_𝛿v
/// @param[in] n_velocity Velocity of the body with respect to the e-system in [m / s], resolved in the n-system
/// @param[in] latitude Geodetic latitude of the body in [rad]
/// @param[in] height Geodetic height of the body in [m]
/// @param[in] R_N North/South (meridian) earth radius in [m]
/// @param[in] R_E East/West (prime vertical) earth radius in [m]
/// @return 3x3 matrix in [1 / s]
/// @note See \cite Groves2013 Groves, ch. 14.2.4, eq. 14.68, p. 587 - 𝐅_22
/// @note See T. Hobiger (2021) Inertialnavigation V08 - equation (8.6, 8.15)
[[nodiscard]] Eigen::Matrix3d n_F_dv_dv(const Eigen::Vector3d& n_velocity, double latitude, double height, double R_N, double R_E);

/// @brief Calculates the matrix 𝐅_𝛿v'_𝛿r
/// @param[in] n_velocity Velocity of the body with respect to the e-system in [m / s], resolved in the n-system
/// @param[in] latitude Geodetic latitude of the body in [rad]
/// @param[in] height Geodetic height of the body in [m]
/// @param[in] R_N North/South (meridian) earth radius in [m]
/// @param[in] R_E East/West (prime vertical) earth radius in [m]
/// @param[in] g_0 Magnitude of the gravity vector in [m/s^2] (see \cite Groves2013 Groves, ch. 2.4.7, eq. 2.135, p. 70)
/// @param[in] r_eS_e Geocentric radius. The distance of a point on the Earth's surface from the center of the Earth in [m]
/// @return 3x3 matrix in [m / s^2] for latitude and [1 / s^2] for height
/// @note See \cite Groves2013 Groves, ch. 14.2.4, eq. 14.69, p. 588 - 𝐅_23
/// @note See T. Hobiger (2021) Inertialnavigation V08 - equation (8.14, 8.16)
[[nodiscard]] Eigen::Matrix3d n_F_dv_dr(const Eigen::Vector3d& n_velocity, double latitude, double height, double R_N, double R_E, double g_0, double r_eS_e);

/// @brief Calculates the matrix 𝐅_𝜓'_𝛿f
/// @param[in] n_Dcm_b DCM from body to navigation frame
/// @return 3x3 matrix in [-]
/// @note See \cite Groves2013 Groves, ch. 14.2.4, eq. 14.63, p. 587 - 𝐅_24
[[nodiscard]] Eigen::Matrix3d n_F_dv_df(const Eigen::Matrix3d& n_Dcm_b);

/// @brief Calculates the matrix 𝐅_𝛿r'_𝛿v
/// @param[in] latitude Geodetic latitude of the body in [rad]
/// @param[in] height Geodetic height of the body in [m]
/// @param[in] R_N North/South (meridian) earth radius in [m]
/// @param[in] R_E East/West (prime vertical) earth radius in [m]
/// @return 3x3 matrix in [1 / m] for latitude and longitude and [-] for height
/// @note See \cite Groves2013 Groves, ch. 14.2.4, eq. 14.70, p. 588 - 𝐅_32
/// @note See T. Hobiger (2021) Inertialnavigation V07 - equation (7.5)
[[nodiscard]] Eigen::Matrix3d n_F_dr_dv(double latitude, double height, double R_N, double R_E);

/// @brief Calculates the matrix 𝐅_𝛿r'_𝛿r
/// @param[in] n_velocity Velocity of the body with respect to the e-system in [m / s], resolved in the n-system
/// @param[in] latitude Geodetic latitude of the body in [rad]
/// @param[in] height Geodetic height of the body in [m]
/// @param[in] R_N North/South (meridian) earth radius in [m]
/// @param[in] R_E East/West (prime vertical) earth radius in [m]
/// @return 3x3 matrix in [1 / s] for latitude and [1 / (m · s)] for height
/// @note See \cite Groves2013 Groves, ch. 14.2.4, eq. 14.71, p. 588 - 𝐅_33
/// @note See T. Hobiger (2021) Inertialnavigation V07 - equation (7.5)
[[nodiscard]] Eigen::Matrix3d n_F_dr_dr(const Eigen::Vector3d& n_velocity, double latitude, double height, double R_N, double R_E);

/// @brief Calculates the matrix 𝐅_𝛿f'_𝛿f
/// @param[in] beta_a Gauss-Markov constant for the accelerometer 𝛽 = 1 / 𝜏 (𝜏 correlation length)
/// @return 3x3 matrix in [1 / s]
/// @note See T. Hobiger (2021) Inertialnavigation V06 - equation (6.3)
[[nodiscard]] Eigen::Matrix3d n_F_df_df(const Eigen::Vector3d& beta_a);

/// @brief Calculates the matrix 𝐅_𝛿ω'_𝛿ω
/// @param[in] beta_omega Gauss-Markov constant for the gyroscope 𝛽 = 1 / 𝜏 (𝜏 correlation length)
/// @return 3x3 matrix in [1 / s]
/// @note See T. Hobiger (2021) Inertialnavigation V06 - equation (6.3)
[[nodiscard]] Eigen::Matrix3d n_F_dw_dw(const Eigen::Vector3d& beta_omega);

} // namespace NAV