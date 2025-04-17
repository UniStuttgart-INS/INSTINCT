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

#include "Navigation/Constants.hpp"
#include "Navigation/Math/Math.hpp"
#include "Navigation/GNSS/SystemModel/MotionModel.hpp"
#include "Navigation/INS/Keys.hpp"
#include "util/Container/KeyedMatrix.hpp"

namespace NAV
{
/// @brief Calculates the matrix 𝐅_𝜓'_𝜓
/// @param[in] omega_ie Angular velocity of the Earth in [rad/s]
/// @return 3x3 matrix in [rad / s]
/// @note See \cite Groves2013 Groves, ch. 14.2.3, eq. 14.48, p. 583 - 𝐅_11
template<typename T>
[[nodiscard]] Eigen::Matrix3<T> e_F_dpsi_dpsi(const T& omega_ie)
{
    // Math: \mathbf{F}_{11}^e = -\mathbf{\Omega}_{ie}^e
    Eigen::Matrix3<T> e_F_11;
    // clang-format off
    e_F_11 <<      0   , omega_ie, 0,
              -omega_ie,     0   , 0,
                   0   ,     0   , 0;
    // clang-format on
    return e_F_11;
}

/// @brief Calculates the matrix 𝐅_𝜓'_𝛿ω
/// @param[in] e_Dcm_b DCM from body to Earth frame
/// @return 3x3 matrix in [-]
/// @note See \cite Groves2013 Groves, ch. 14.2.3, eq. 14.48, p. 583 - 𝐅_15
template<typename Derived>
[[nodiscard]] Eigen::Matrix<typename Derived::Scalar, 3, 3> e_F_dpsi_dw(const Eigen::MatrixBase<Derived>& e_Dcm_b)
{
    return e_Dcm_b;
}

/// @brief Calculates the matrix 𝐅_𝛿v'_𝜓
/// @param[in] e_force_ib Specific force of the body with respect to inertial frame in [m / s^2], resolved in Earth frame coordinates
/// @return 3x3 matrix in [m / s^2]
/// @note See \cite Groves2013 Groves, ch. 14.2.3, eq. 14.49, p. 584 - 𝐅_21
template<typename Derived>
[[nodiscard]] Eigen::Matrix<typename Derived::Scalar, 3, 3> e_F_dv_dpsi(const Eigen::MatrixBase<Derived>& e_force_ib)
{
    // Math: \mathbf{F}_{21}^e = \begin{bmatrix} (\mathbf{C}_{b}^e \hat{f}_{ib}^b) \land \end{bmatrix}
    Eigen::Matrix<typename Derived::Scalar, 3, 3> e_F_21;
    // clang-format off
    e_F_21 <<        0       ,  e_force_ib.z(), -e_force_ib.y(),
              -e_force_ib.z(),        0       ,  e_force_ib.x(),
               e_force_ib.y(), -e_force_ib.x(),        0       ;
    // clang-format on
    return e_F_21;
}

/// @brief Calculates the matrix 𝐅_𝛿v'_𝛿v
/// @param[in] omega_ie omega_ie Angular velocity of the Earth in [rad/s]
/// @return 3x3 matrix in [1 / s]
/// @note See \cite Groves2013 Groves, ch. 14.2.3, eq. 14.48, p. 583 - 𝐅_22
template<typename T>
[[nodiscard]] Eigen::Matrix3<T> e_F_dv_dv(double omega_ie)
{
    // Math: \mathbf{F}_{22}^e = -2\mathbf{\Omega}_{ie}^e
    Eigen::Matrix3d e_F_22;
    // clang-format off
    e_F_22 <<        0.0     , 2.0 * omega_ie, 0.0,
              -2.0 * omega_ie,       0.0     , 0.0,
                     0.0     ,       0.0     , 0.0;
    // clang-format on
    return e_F_22.cast<T>();
}

/// @brief Calculates the matrix 𝐅_𝛿v'_𝛿r
/// @param[in] e_position Position in ECEF coordinates in [m]
/// @param[in] e_gravitation Gravitational acceleration in [m/s^2]
/// @param[in] r_eS_e Geocentric radius. The distance of a point on the Earth's surface from the center of the Earth in [m]
/// @param[in] e_omega_ie Angular velocity of Earth with respect to inertial system, represented in e-sys in [rad/s]
/// @return 3x3 matrix in [1 / s^2]
/// @note See \cite Groves2013 Groves, ch. 14.2.3, eq. 14.49, p. 584 - 𝐅_23
template<typename Derived1, typename Derived2, typename T>
[[nodiscard]] Eigen::Matrix<typename Derived1::Scalar, 3, 3> e_F_dv_dr(const Eigen::MatrixBase<Derived1>& e_position,
                                                                       const Eigen::MatrixBase<Derived2>& e_gravitation,
                                                                       const T& r_eS_e,
                                                                       const Eigen::Vector3d& e_omega_ie)
{
    Eigen::Matrix3d e_Omega_ie = math::skewSymmetricMatrix(e_omega_ie);

    // Math: \mathbf{F}_{23}^e = -( \dfrac{2 \boldsymbol{\gamma}_{ib}^e}{r_{eS}^e} \dfrac{{\mathbf{r}_{eb}^e}^T}{\left| \mathbf{r}_{eb}^e \right|} + \boldsymbol{\Omega}_{ie}^e \boldsymbol{\Omega}_{ie}^e )
    Eigen::Matrix<typename Derived1::Scalar, 3, 3> e_F_23 = -((2.0 * e_gravitation * e_position.transpose()) / (r_eS_e * e_position.norm()) + e_Omega_ie * e_Omega_ie);

    return e_F_23;
}

/// @brief Calculate the linearized velocity differential equation partially derived for the attitude
/// @param p_accelBias Acceleration bias in platform frame coordinates [m/s^2]
/// @param p_accelScale Acceleration scale factor in platform frame coordinates [m/s^2]
/// @param p_quatAccel_ps Accelerometer misalignment
/// @param ps_f_ip Accelerometer measurement [m/s^2]
/// @param b_quat_p Quaternion from IMU platform frame to body frame
/// @param e_quat_b Quaternion from body to Earth frame
template<typename Derived1, typename Derived2, typename Derived3, typename Derived4, typename Derived5, typename Derived6>
Eigen::Matrix<typename Derived6::Scalar, 3, 4> e_F_dv_dq(const Eigen::MatrixBase<Derived1>& p_accelBias,
                                                         const Eigen::MatrixBase<Derived2>& p_accelScale,
                                                         const Eigen::QuaternionBase<Derived3>& p_quatAccel_ps,
                                                         const Eigen::MatrixBase<Derived4>& ps_f_ip,
                                                         const Eigen::QuaternionBase<Derived5>& b_quat_p,
                                                         const Eigen::QuaternionBase<Derived6>& e_quat_b)
{
    Eigen::Vector3<typename Derived6::Scalar> b_f_ip = b_quat_p.template cast<typename Derived6::Scalar>()
                                                       * (p_accelScale.asDiagonal() * p_quatAccel_ps * ps_f_ip.template cast<typename Derived6::Scalar>() + p_accelBias);

    Eigen::Matrix<typename Derived6::Scalar, 3, 4> F_dv_dq;

    F_dv_dq(0, 0) = 2.0 * (b_f_ip.x() * e_quat_b.x() + b_f_ip.y() * e_quat_b.y() + b_f_ip.z() * e_quat_b.z());
    F_dv_dq(0, 1) = -2.0 * (b_f_ip.x() * e_quat_b.y() - b_f_ip.y() * e_quat_b.x() - b_f_ip.z() * e_quat_b.w());
    F_dv_dq(0, 2) = -2.0 * (b_f_ip.x() * e_quat_b.z() + b_f_ip.y() * e_quat_b.w() - b_f_ip.z() * e_quat_b.x());
    F_dv_dq(0, 3) = 2.0 * (b_f_ip.x() * e_quat_b.w() - b_f_ip.y() * e_quat_b.z() + b_f_ip.z() * e_quat_b.y());

    F_dv_dq(1, 0) = -F_dv_dq(0, 1);
    F_dv_dq(1, 1) = F_dv_dq(0, 0);
    F_dv_dq(1, 2) = F_dv_dq(0, 3);
    F_dv_dq(1, 3) = -F_dv_dq(0, 2);

    F_dv_dq(2, 0) = -F_dv_dq(0, 2);
    F_dv_dq(2, 1) = -F_dv_dq(0, 3);
    F_dv_dq(2, 2) = F_dv_dq(0, 0);
    F_dv_dq(2, 3) = F_dv_dq(0, 1);

    return F_dv_dq;
}

/// @brief Calculates the matrix 𝐅_𝜓'_𝛿f_b
/// @param[in] e_Dcm_b DCM from body to Earth frame
/// @return 3x3 matrix in [-]
/// @note See \cite Groves2013 Groves, ch. 14.2.3, eq. 14.48, p. 583 - 𝐅_24
template<typename Derived>
[[nodiscard]] Eigen::Matrix<typename Derived::Scalar, 3, 3> e_F_dv_df_b(const Eigen::MatrixBase<Derived>& e_Dcm_b)
{
    return e_Dcm_b;
}

/// @brief Calculates the matrix 𝐅_𝜓'_𝛿f_p
/// @param[in] b_quat_p Quaternion from IMU platform frame to body frame
/// @param[in] e_quat_b Quaternion from body to Earth frame
/// @return 3x3 matrix in [-]
template<typename Derived1, typename Derived2>
[[nodiscard]] Eigen::Matrix<typename Derived2::Scalar, 3, 3> e_F_dv_dAccelBias(const Eigen::QuaternionBase<Derived1>& b_quat_p, const Eigen::QuaternionBase<Derived2>& e_quat_b)
{
    return (e_quat_b * b_quat_p.template cast<typename Derived2::Scalar>()).toRotationMatrix();
}

/// @brief Calculate the linearized velocity differential equation partially derived for the acceleration bias
/// @param p_quatAccel_ps Accelerometer misalignment
/// @param ps_f_ip Accelerometer measurement [m/s^2]
/// @param b_quat_p Quaternion from IMU platform frame to body frame
/// @param e_quat_b Quaternion from body to Earth frame
template<typename Derived1, typename Derived2, typename Derived3, typename Derived4>
[[nodiscard]] Eigen::Matrix<typename Derived4::Scalar, 3, 3> e_F_dv_dAccelScale(const Eigen::QuaternionBase<Derived1>& p_quatAccel_ps,
                                                                                const Eigen::MatrixBase<Derived2>& ps_f_ip,
                                                                                const Eigen::QuaternionBase<Derived3>& b_quat_p,
                                                                                const Eigen::QuaternionBase<Derived4>& e_quat_b)
{
    return e_quat_b
           * b_quat_p.template cast<typename Derived4::Scalar>()
           * p_quatAccel_ps
           * ps_f_ip.template cast<typename Derived4::Scalar>().asDiagonal();
}

/// @brief Calculate the linearized velocity differential equation partially derived for the accelerometer misalignment
/// @param p_accelScale Acceleration scale factor in platform frame coordinates [m/s^2]
/// @param p_quatAccel_ps Accelerometer misalignment
/// @param ps_f_ip Accelerometer measurement [m/s^2]
/// @param b_quat_p Quaternion from IMU platform frame to body frame
/// @param e_quat_b Quaternion from body to Earth frame
template<typename Derived1, typename Derived2, typename Derived3, typename Derived4, typename Derived5>
[[nodiscard]] Eigen::Matrix<typename Derived5::Scalar, 3, 4> e_F_dv_dAccelMisalignment(const Eigen::MatrixBase<Derived1>& p_accelScale,
                                                                                       const Eigen::QuaternionBase<Derived2>& p_quatAccel_ps,
                                                                                       const Eigen::MatrixBase<Derived3>& ps_f_ip,
                                                                                       const Eigen::QuaternionBase<Derived4>& b_quat_p,
                                                                                       const Eigen::QuaternionBase<Derived5>& e_quat_b)
{
    Eigen::Matrix3<typename Derived5::Scalar> e_dcmAccelScaled_p = e_quat_b * b_quat_p.template cast<typename Derived5::Scalar>() * p_accelScale.asDiagonal();

    auto a = p_quatAccel_ps.w() * e_dcmAccelScaled_p(0, 2) + p_quatAccel_ps.y() * e_dcmAccelScaled_p(0, 0) - p_quatAccel_ps.x() * e_dcmAccelScaled_p(0, 1);
    auto b = p_quatAccel_ps.w() * e_dcmAccelScaled_p(1, 2) + p_quatAccel_ps.y() * e_dcmAccelScaled_p(1, 0) - p_quatAccel_ps.x() * e_dcmAccelScaled_p(1, 1);
    auto c = p_quatAccel_ps.w() * e_dcmAccelScaled_p(2, 2) + p_quatAccel_ps.y() * e_dcmAccelScaled_p(2, 0) - p_quatAccel_ps.x() * e_dcmAccelScaled_p(2, 1);
    auto d = p_quatAccel_ps.w() * e_dcmAccelScaled_p(0, 1) + p_quatAccel_ps.x() * e_dcmAccelScaled_p(0, 2) - p_quatAccel_ps.z() * e_dcmAccelScaled_p(0, 0);
    auto e = p_quatAccel_ps.w() * e_dcmAccelScaled_p(1, 1) + p_quatAccel_ps.x() * e_dcmAccelScaled_p(1, 2) - p_quatAccel_ps.z() * e_dcmAccelScaled_p(1, 0);
    auto f = p_quatAccel_ps.w() * e_dcmAccelScaled_p(2, 1) + p_quatAccel_ps.x() * e_dcmAccelScaled_p(2, 2) - p_quatAccel_ps.z() * e_dcmAccelScaled_p(2, 0);
    auto g = p_quatAccel_ps.w() * e_dcmAccelScaled_p(0, 0) + p_quatAccel_ps.z() * e_dcmAccelScaled_p(0, 1) - p_quatAccel_ps.y() * e_dcmAccelScaled_p(0, 2);
    auto h = p_quatAccel_ps.w() * e_dcmAccelScaled_p(1, 0) + p_quatAccel_ps.z() * e_dcmAccelScaled_p(1, 1) - p_quatAccel_ps.y() * e_dcmAccelScaled_p(1, 2);
    auto i = p_quatAccel_ps.w() * e_dcmAccelScaled_p(2, 0) + p_quatAccel_ps.z() * e_dcmAccelScaled_p(2, 1) - p_quatAccel_ps.y() * e_dcmAccelScaled_p(2, 2);
    auto j = p_quatAccel_ps.x() * e_dcmAccelScaled_p(0, 0) + p_quatAccel_ps.y() * e_dcmAccelScaled_p(0, 1) + p_quatAccel_ps.z() * e_dcmAccelScaled_p(0, 2);
    auto k = p_quatAccel_ps.x() * e_dcmAccelScaled_p(1, 0) + p_quatAccel_ps.y() * e_dcmAccelScaled_p(1, 1) + p_quatAccel_ps.z() * e_dcmAccelScaled_p(1, 2);
    auto l = p_quatAccel_ps.x() * e_dcmAccelScaled_p(2, 0) + p_quatAccel_ps.y() * e_dcmAccelScaled_p(2, 1) + p_quatAccel_ps.z() * e_dcmAccelScaled_p(2, 2);

    Eigen::Matrix<typename Derived5::Scalar, 3, 4> e_F_dv_dAccelMisalignment;

    e_F_dv_dAccelMisalignment(0, 0) = 2.0 * (j * ps_f_ip.x() + a * ps_f_ip.y() - d * ps_f_ip.z());
    e_F_dv_dAccelMisalignment(0, 1) = -2.0 * (a * ps_f_ip.x() - j * ps_f_ip.y() - g * ps_f_ip.z());
    e_F_dv_dAccelMisalignment(0, 2) = 2.0 * (d * ps_f_ip.x() - g * ps_f_ip.y() + j * ps_f_ip.z());
    e_F_dv_dAccelMisalignment(0, 3) = 2.0 * (g * ps_f_ip.x() + d * ps_f_ip.y() + a * ps_f_ip.z());

    e_F_dv_dAccelMisalignment(1, 0) = 2.0 * (k * ps_f_ip.x() + b * ps_f_ip.y() - e * ps_f_ip.z());
    e_F_dv_dAccelMisalignment(1, 1) = -2.0 * (b * ps_f_ip.x() - k * ps_f_ip.y() - h * ps_f_ip.z());
    e_F_dv_dAccelMisalignment(1, 2) = 2.0 * (e * ps_f_ip.x() - h * ps_f_ip.y() + k * ps_f_ip.z());
    e_F_dv_dAccelMisalignment(1, 3) = 2.0 * (h * ps_f_ip.x() + e * ps_f_ip.y() + b * ps_f_ip.z());

    e_F_dv_dAccelMisalignment(2, 0) = 2.0 * (l * ps_f_ip.x() + c * ps_f_ip.y() - f * ps_f_ip.z());
    e_F_dv_dAccelMisalignment(2, 1) = -2.0 * (c * ps_f_ip.x() - l * ps_f_ip.y() - i * ps_f_ip.z());
    e_F_dv_dAccelMisalignment(2, 2) = 2.0 * (f * ps_f_ip.x() - i * ps_f_ip.y() + l * ps_f_ip.z());
    e_F_dv_dAccelMisalignment(2, 3) = 2.0 * (i * ps_f_ip.x() + f * ps_f_ip.y() + c * ps_f_ip.z());

    return e_F_dv_dAccelMisalignment;
}

/// @brief Calculate the linearized attitude quaternion differential equation partially derived for the attitude quaternion
/// @param p_gyroBias Angular rate bias in platform frame coordinates [rad/s]
/// @param p_gyroScale Angular rate scale factor in platform frame coordinates [rad/s]
/// @param p_quatGyro_ps Gyroscope misalignment
/// @param ps_omega_ip Gyroscope measurement [m/s^2]
/// @param b_quat_p Quaternion from IMU platform frame to body frame
/// @param e_quat_b Quaternion from body to Earth frame
/// @param omega_ie Earth rotation rate [rad/s]
template<typename Derived1, typename Derived2, typename Derived3, typename Derived4, typename Derived5, typename Derived6>
[[nodiscard]] Eigen::Matrix4<typename Derived6::Scalar> e_F_dq_dq(const Eigen::MatrixBase<Derived1>& p_gyroBias,
                                                                  const Eigen::MatrixBase<Derived2>& p_gyroScale,
                                                                  const Eigen::QuaternionBase<Derived3>& p_quatGyro_ps,
                                                                  const Eigen::MatrixBase<Derived4>& ps_omega_ip,
                                                                  const Eigen::QuaternionBase<Derived5>& b_quat_p,
                                                                  const Eigen::QuaternionBase<Derived6>& e_quat_b,
                                                                  double omega_ie)
{
    Eigen::Vector3<typename Derived6::Scalar> b_omega_ib = b_quat_p.template cast<typename Derived6::Scalar>()
                                                           * (p_gyroScale.asDiagonal() * p_quatGyro_ps * ps_omega_ip.template cast<typename Derived6::Scalar>()
                                                              + p_gyroBias);

    Eigen::Matrix4<typename Derived6::Scalar> F_dq_dq;

    F_dq_dq(0, 0) = omega_ie * e_quat_b.x() * e_quat_b.y();
    F_dq_dq(0, 1) = 0.5 * b_omega_ib.z() + 0.5 * omega_ie * std::pow(e_quat_b.w(), 2) + 0.5 * omega_ie * std::pow(e_quat_b.x(), 2) + 0.5 * omega_ie * std::pow(e_quat_b.z(), 2) + 1.5 * omega_ie * std::pow(e_quat_b.y(), 2);
    F_dq_dq(0, 2) = -0.5 * b_omega_ib.y() + omega_ie * e_quat_b.y() * e_quat_b.z();
    F_dq_dq(0, 3) = 0.5 * b_omega_ib.x() + omega_ie * e_quat_b.w() * e_quat_b.y();

    F_dq_dq(1, 0) = -0.5 * b_omega_ib.z() - 0.5 * omega_ie * std::pow(e_quat_b.w(), 2) - 0.5 * omega_ie * std::pow(e_quat_b.y(), 2) - 0.5 * omega_ie * std::pow(e_quat_b.z(), 2) - 1.5 * omega_ie * std::pow(e_quat_b.x(), 2);
    F_dq_dq(1, 1) = -F_dq_dq(0, 0);
    F_dq_dq(1, 2) = 0.5 * b_omega_ib.x() - omega_ie * e_quat_b.x() * e_quat_b.z();
    F_dq_dq(1, 3) = 0.5 * b_omega_ib.y() - omega_ie * e_quat_b.w() * e_quat_b.x();

    F_dq_dq(2, 0) = F_dq_dq(1, 3);
    F_dq_dq(2, 1) = -0.5 * b_omega_ib.x() - omega_ie * e_quat_b.w() * e_quat_b.y();
    F_dq_dq(2, 2) = -omega_ie * e_quat_b.w() * e_quat_b.z();
    F_dq_dq(2, 3) = 0.5 * b_omega_ib.z() - 0.5 * omega_ie * std::pow(e_quat_b.x(), 2) - 0.5 * omega_ie * std::pow(e_quat_b.y(), 2) - 0.5 * omega_ie * std::pow(e_quat_b.z(), 2) - 1.5 * omega_ie * std::pow(e_quat_b.w(), 2);

    F_dq_dq(3, 0) = -0.5 * b_omega_ib.x() + omega_ie * e_quat_b.x() * e_quat_b.z();
    F_dq_dq(3, 1) = F_dq_dq(0, 2);
    F_dq_dq(3, 2) = -0.5 * b_omega_ib.z() + 0.5 * omega_ie * std::pow(e_quat_b.w(), 2) + 0.5 * omega_ie * std::pow(e_quat_b.x(), 2) + 0.5 * omega_ie * std::pow(e_quat_b.y(), 2) + 1.5 * omega_ie * std::pow(e_quat_b.z(), 2);
    F_dq_dq(3, 3) = -F_dq_dq(2, 2);

    return F_dq_dq;
}

/// @brief Calculate the linearized attitude quaternion differential equation partially derived for the angular rate bias
/// @param b_quat_p Quaternion from IMU platform frame to body frame
/// @param e_quat_b Quaternion from body to Earth frame
template<typename Derived1, typename Derived2>
[[nodiscard]] Eigen::Matrix<typename Derived2::Scalar, 4, 3> e_F_dq_dGyroBias(const Eigen::QuaternionBase<Derived1>& b_quat_p,
                                                                              const Eigen::QuaternionBase<Derived2>& e_quat_b)
{
    Eigen::Matrix<typename Derived2::Scalar, 4, 3> F_dq_dGyroBias;

    auto a = std::pow(b_quat_p.w(), 2) + std::pow(b_quat_p.x(), 2) - std::pow(b_quat_p.y(), 2) - std::pow(b_quat_p.z(), 2);
    auto b = std::pow(b_quat_p.w(), 2) + std::pow(b_quat_p.y(), 2) - std::pow(b_quat_p.x(), 2) - std::pow(b_quat_p.z(), 2);
    auto c = std::pow(b_quat_p.w(), 2) + std::pow(b_quat_p.z(), 2) - std::pow(b_quat_p.x(), 2) - std::pow(b_quat_p.y(), 2);
    auto d = b_quat_p.w() * b_quat_p.x() + b_quat_p.y() * b_quat_p.z();
    auto e = b_quat_p.w() * b_quat_p.x() - b_quat_p.y() * b_quat_p.z();
    auto f = b_quat_p.w() * b_quat_p.y() + b_quat_p.x() * b_quat_p.z();
    auto g = b_quat_p.w() * b_quat_p.y() - b_quat_p.x() * b_quat_p.z();
    auto h = b_quat_p.w() * b_quat_p.z() + b_quat_p.x() * b_quat_p.y();
    auto i = b_quat_p.w() * b_quat_p.z() - b_quat_p.x() * b_quat_p.y();

    F_dq_dGyroBias(0, 0) = 0.5 * a * e_quat_b.w() - g * e_quat_b.y() - h * e_quat_b.z();
    F_dq_dGyroBias(0, 1) = d * e_quat_b.y() - 0.5 * b * e_quat_b.z() - i * e_quat_b.w();
    F_dq_dGyroBias(0, 2) = 0.5 * c * e_quat_b.y() + e * e_quat_b.z() + f * e_quat_b.w();

    F_dq_dGyroBias(1, 0) = 0.5 * a * e_quat_b.z() + g * e_quat_b.x() + h * e_quat_b.w();
    F_dq_dGyroBias(1, 1) = 0.5 * b * e_quat_b.w() - d * e_quat_b.x() - i * e_quat_b.z();
    F_dq_dGyroBias(1, 2) = f * e_quat_b.z() - 0.5 * c * e_quat_b.x() - e * e_quat_b.w();

    F_dq_dGyroBias(2, 0) = h * e_quat_b.x() - 0.5 * a * e_quat_b.y() - g * e_quat_b.w();
    F_dq_dGyroBias(2, 1) = 0.5 * b * e_quat_b.x() + d * e_quat_b.w() + i * e_quat_b.y();
    F_dq_dGyroBias(2, 2) = 0.5 * c * e_quat_b.w() - e * e_quat_b.x() - f * e_quat_b.y();

    F_dq_dGyroBias(3, 0) = g * e_quat_b.z() - 0.5 * a * e_quat_b.x() - h * e_quat_b.y();
    F_dq_dGyroBias(3, 1) = i * e_quat_b.x() - 0.5 * b * e_quat_b.y() - d * e_quat_b.z();
    F_dq_dGyroBias(3, 2) = e * e_quat_b.y() - 0.5 * c * e_quat_b.z() - f * e_quat_b.x();

    return F_dq_dGyroBias;
}

/// @brief Calculate the linearized attitude quaternion differential equation partially derived for the angular rate scale factor
/// @param p_quatGyro_ps Gyroscope misalignment
/// @param ps_omega_ip Gyroscope measurement [m/s^2]
/// @param b_quat_p Quaternion from IMU platform frame to body frame
/// @param e_quat_b Quaternion from body to Earth frame
template<typename Derived1, typename Derived2, typename Derived3, typename Derived4>
[[nodiscard]] Eigen::Matrix<typename Derived4::Scalar, 4, 3> e_F_dq_dGyroScale(const Eigen::QuaternionBase<Derived1>& p_quatGyro_ps,
                                                                               const Eigen::MatrixBase<Derived2>& ps_omega_ip,
                                                                               const Eigen::QuaternionBase<Derived3>& b_quat_p,
                                                                               const Eigen::QuaternionBase<Derived4>& e_quat_b)
{
    Eigen::Vector3<typename Derived4::Scalar> p_omega_ip = p_quatGyro_ps * ps_omega_ip.template cast<typename Derived4::Scalar>();

    return e_F_dq_dGyroBias(b_quat_p, e_quat_b) * p_omega_ip.asDiagonal();
}

/// @brief Calculate the linearized attitude quaternion differential equation partially derived for the gyroscope misalignment
/// @param p_gyroScale Angular rate scale factor in platform frame coordinates [rad/s]
/// @param p_quatGyro_ps Gyroscope misalignment
/// @param ps_omega_ip Gyroscope measurement [m/s^2]
/// @param b_quat_p Quaternion from IMU platform frame to body frame
/// @param e_quat_b Quaternion from body to Earth frame
template<typename Derived1, typename Derived2, typename Derived3, typename Derived4, typename Derived5>
[[nodiscard]] Eigen::Matrix4<typename Derived5::Scalar> e_F_dq_dGyroMisalignment(const Eigen::MatrixBase<Derived1>& p_gyroScale,
                                                                                 const Eigen::QuaternionBase<Derived2>& p_quatGyro_ps,
                                                                                 const Eigen::MatrixBase<Derived3>& ps_omega_ip,
                                                                                 const Eigen::QuaternionBase<Derived4>& b_quat_p,
                                                                                 const Eigen::QuaternionBase<Derived5>& e_quat_b)
{
    Eigen::Matrix3<typename Derived1::Scalar> b_dcmGyroScaled_p = b_quat_p.template cast<typename Derived1::Scalar>() * p_gyroScale.asDiagonal();

    auto a = p_quatGyro_ps.x() * b_dcmGyroScaled_p(0, 0) + p_quatGyro_ps.y() * b_dcmGyroScaled_p(0, 1) + p_quatGyro_ps.z() * b_dcmGyroScaled_p(0, 2);
    auto b = p_quatGyro_ps.w() * b_dcmGyroScaled_p(0, 2) + p_quatGyro_ps.y() * b_dcmGyroScaled_p(0, 0) - p_quatGyro_ps.x() * b_dcmGyroScaled_p(0, 1);
    auto c = p_quatGyro_ps.w() * b_dcmGyroScaled_p(0, 1) + p_quatGyro_ps.x() * b_dcmGyroScaled_p(0, 2) - p_quatGyro_ps.z() * b_dcmGyroScaled_p(0, 0);
    auto d = p_quatGyro_ps.x() * b_dcmGyroScaled_p(2, 0) + p_quatGyro_ps.y() * b_dcmGyroScaled_p(2, 1) + p_quatGyro_ps.z() * b_dcmGyroScaled_p(2, 2);
    auto e = p_quatGyro_ps.w() * b_dcmGyroScaled_p(2, 2) + p_quatGyro_ps.y() * b_dcmGyroScaled_p(2, 0) - p_quatGyro_ps.x() * b_dcmGyroScaled_p(2, 1);
    auto f = p_quatGyro_ps.w() * b_dcmGyroScaled_p(2, 1) + p_quatGyro_ps.x() * b_dcmGyroScaled_p(2, 2) - p_quatGyro_ps.z() * b_dcmGyroScaled_p(2, 0);
    auto g = p_quatGyro_ps.x() * b_dcmGyroScaled_p(1, 0) + p_quatGyro_ps.y() * b_dcmGyroScaled_p(1, 1) + p_quatGyro_ps.z() * b_dcmGyroScaled_p(1, 2);
    auto h = p_quatGyro_ps.w() * b_dcmGyroScaled_p(1, 2) + p_quatGyro_ps.y() * b_dcmGyroScaled_p(1, 0) - p_quatGyro_ps.x() * b_dcmGyroScaled_p(1, 1);
    auto i = p_quatGyro_ps.w() * b_dcmGyroScaled_p(1, 1) + p_quatGyro_ps.x() * b_dcmGyroScaled_p(1, 2) - p_quatGyro_ps.z() * b_dcmGyroScaled_p(1, 0);
    auto j = p_quatGyro_ps.w() * b_dcmGyroScaled_p(0, 0) + p_quatGyro_ps.z() * b_dcmGyroScaled_p(0, 1) - p_quatGyro_ps.y() * b_dcmGyroScaled_p(0, 2);
    auto k = p_quatGyro_ps.w() * b_dcmGyroScaled_p(2, 0) + p_quatGyro_ps.z() * b_dcmGyroScaled_p(2, 1) - p_quatGyro_ps.y() * b_dcmGyroScaled_p(2, 2);
    auto l = p_quatGyro_ps.w() * b_dcmGyroScaled_p(1, 0) + p_quatGyro_ps.z() * b_dcmGyroScaled_p(1, 1) - p_quatGyro_ps.y() * b_dcmGyroScaled_p(1, 2);
    auto m = a * ps_omega_ip.x() + b * ps_omega_ip.y() - c * ps_omega_ip.z();
    auto n = d * ps_omega_ip.x() + e * ps_omega_ip.y() - f * ps_omega_ip.z();
    auto o = g * ps_omega_ip.x() + h * ps_omega_ip.y() - i * ps_omega_ip.z();
    auto p = a * ps_omega_ip.y() + j * ps_omega_ip.z() - b * ps_omega_ip.x();
    auto q = d * ps_omega_ip.y() + k * ps_omega_ip.z() - e * ps_omega_ip.x();
    auto r = g * ps_omega_ip.y() + l * ps_omega_ip.z() - h * ps_omega_ip.x();
    auto s = a * ps_omega_ip.z() + c * ps_omega_ip.x() - j * ps_omega_ip.y();
    auto t = d * ps_omega_ip.z() + f * ps_omega_ip.x() - k * ps_omega_ip.y();
    auto u = g * ps_omega_ip.z() + i * ps_omega_ip.x() - l * ps_omega_ip.y();
    auto v = b * ps_omega_ip.z() + c * ps_omega_ip.y() + j * ps_omega_ip.x();
    auto w = e * ps_omega_ip.z() + f * ps_omega_ip.y() + k * ps_omega_ip.x();
    auto x = h * ps_omega_ip.z() + i * ps_omega_ip.y() + l * ps_omega_ip.x();

    Eigen::Matrix4<typename Derived5::Scalar> F_dq_dGyroMisalignment;

    F_dq_dGyroMisalignment(0, 0) = m * e_quat_b.w() + n * e_quat_b.y() - o * e_quat_b.z();
    F_dq_dGyroMisalignment(0, 1) = p * e_quat_b.w() + q * e_quat_b.y() - r * e_quat_b.z();
    F_dq_dGyroMisalignment(0, 2) = s * e_quat_b.w() + t * e_quat_b.y() - u * e_quat_b.z();
    F_dq_dGyroMisalignment(0, 3) = v * e_quat_b.w() + w * e_quat_b.y() - x * e_quat_b.z();

    F_dq_dGyroMisalignment(1, 0) = m * e_quat_b.z() + o * e_quat_b.w() - n * e_quat_b.x();
    F_dq_dGyroMisalignment(1, 1) = p * e_quat_b.z() + r * e_quat_b.w() - q * e_quat_b.x();
    F_dq_dGyroMisalignment(1, 2) = s * e_quat_b.z() + u * e_quat_b.w() - t * e_quat_b.x();
    F_dq_dGyroMisalignment(1, 3) = v * e_quat_b.z() + x * e_quat_b.w() - w * e_quat_b.x();

    F_dq_dGyroMisalignment(2, 0) = n * e_quat_b.w() + o * e_quat_b.x() - m * e_quat_b.y();
    F_dq_dGyroMisalignment(2, 1) = q * e_quat_b.w() + r * e_quat_b.x() - p * e_quat_b.y();
    F_dq_dGyroMisalignment(2, 2) = t * e_quat_b.w() + u * e_quat_b.x() - s * e_quat_b.y();
    F_dq_dGyroMisalignment(2, 3) = w * e_quat_b.w() + x * e_quat_b.x() - v * e_quat_b.y();

    F_dq_dGyroMisalignment(3, 0) = -m * e_quat_b.x() - n * e_quat_b.z() - o * e_quat_b.y();
    F_dq_dGyroMisalignment(3, 1) = -p * e_quat_b.x() - q * e_quat_b.z() - r * e_quat_b.y();
    F_dq_dGyroMisalignment(3, 2) = -s * e_quat_b.x() - t * e_quat_b.z() - u * e_quat_b.y();
    F_dq_dGyroMisalignment(3, 3) = -v * e_quat_b.x() - w * e_quat_b.z() - x * e_quat_b.y();

    return F_dq_dGyroMisalignment;
}

/// @brief Calculates the matrix 𝐅_𝛿r'_𝛿v
/// @return 3x3 matrix in [-]
/// @note See \cite Groves2013 Groves, ch. 14.2.3, eq. 14.48, p. 583 - 𝐅_32
template<typename T>
[[nodiscard]] Eigen::Matrix3<T> e_F_dr_dv()
{
    return Eigen::Matrix3<T>::Identity();
}

/// @brief Calculates the matrix 𝐅_𝛿f'_𝛿f
/// @param[in] beta_a Gauss-Markov constant for the accelerometer 𝛽 = 1 / 𝜏 (𝜏 correlation length)
/// @return 3x3 matrix in [1 / s]
/// @note See T. Hobiger (2021) Inertialnavigation V06 - equation (6.3)
template<typename Derived>
[[nodiscard]] Eigen::Matrix<typename Derived::Scalar, 3, 3> e_F_df_df(const Eigen::MatrixBase<Derived>& beta_a)
{
    // Math: \mathbf{F}_{a} = - \begin{bmatrix} \beta_{a,1} & 0 & 0 \\ 0 & \beta_{a,2} & 0 \\ 0 & 0 & \beta_{a,2} \end{bmatrix} \quad \text{T. Hobiger}\,(6.3)
    return -1.0 * beta_a.asDiagonal();
}

/// @brief Calculates the matrix 𝐅_𝛿ω'_𝛿ω
/// @param[in] beta_omega Gauss-Markov constant for the gyroscope 𝛽 = 1 / 𝜏 (𝜏 correlation length)
/// @return 3x3 matrix in [1 / s]
/// @note See T. Hobiger (2021) Inertialnavigation V06 - equation (6.3)
template<typename Derived>
[[nodiscard]] Eigen::Matrix<typename Derived::Scalar, 3, 3> e_F_dw_dw(const Eigen::MatrixBase<Derived>& beta_omega)
{
    // Math: \mathbf{F}_{\omega} = - \begin{bmatrix} \beta_{\omega,1} & 0 & 0 \\ 0 & \beta_{\omega,2} & 0 \\ 0 & 0 & \beta_{\omega,2} \end{bmatrix} \quad \text{T. Hobiger}\,(6.3)
    return -1.0 * beta_omega.asDiagonal();
}

/// @brief Calculate the linearized system matrix in Earth frame coordinates
/// @param ps_f_ip Accelerometer measurement [m/s^2]
/// @param ps_omega_ip Gyroscope measurement [m/s^2]
/// @param b_quat_p Quaternion from IMU platform frame to body frame
/// @param e_gravitation Gravitation in [m/s^2] in ECEF coordinates
/// @param r_eS_e Geocentric radius in [m]
/// @param tau_bad Correlation length of the accel dynamic bias in [s]
/// @param tau_bgd Correlation length of the gyro dynamic bias in [s]
/// @param e_position Position in ECEF coordinates in [m]
/// @param e_quat_b Quaternion from body to Earth frame
/// @param p_accelBias Acceleration bias in platform frame coordinates [m/s^2]
/// @param p_gyroBias Angular rate bias in platform frame coordinates [rad/s]
/// @param p_accelScale Acceleration scale factor in platform frame coordinates [m/s^2]
/// @param p_gyroScale Angular rate scale factor in platform frame coordinates [rad/s]
/// @param p_quatAccel_ps Accelerometer misalignment
/// @param p_quatGyro_ps Gyroscope misalignment
/// @param config Integration config
/// @param accelBiases Flag wether to calculate the rows and columns for the acceleration bias
/// @param gyroBiases Flag wether to calculate the rows and columns for the angular rate bias
/// @param accelScaleFactor Flag wether to calculate the rows and columns for the acceleration scale factor
/// @param gyroScaleFactor Flag wether to calculate the rows and columns for the angular rate scale factor
/// @param accelMisalignment Flag wether to calculate the rows and columns for the accelerometer misalignment
/// @param gyroMisalignment Flag wether to calculate the rows and columns for the gyroscope misalignment
template<typename KeyType, typename T>
[[nodiscard]] KeyedMatrixX<T, KeyType, KeyType> e_systemMatrix_F(const Eigen::Vector3d& ps_f_ip,
                                                                 const Eigen::Vector3d& ps_omega_ip,
                                                                 const Eigen::Quaterniond& b_quat_p,
                                                                 const Eigen::Vector3<T>& e_gravitation,
                                                                 const T& r_eS_e,
                                                                 std::optional<double> tau_bad,
                                                                 std::optional<double> tau_bgd,
                                                                 const Eigen::Vector3<T>& e_position,
                                                                 const Eigen::Quaternion<T>& e_quat_b,
                                                                 const Eigen::Vector3<T>& p_accelBias,
                                                                 const Eigen::Vector3<T>& p_gyroBias,
                                                                 const Eigen::Vector3<T>& p_accelScale,
                                                                 const Eigen::Vector3<T>& p_gyroScale,
                                                                 const Eigen::Quaternion<T>& p_quatAccel_ps,
                                                                 const Eigen::Quaternion<T>& p_quatGyro_ps,
                                                                 const PosVelAttDerivativeConstants& config,
                                                                 bool accelBiases, bool gyroBiases,
                                                                 bool accelScaleFactor, bool gyroScaleFactor,
                                                                 bool accelMisalignment, bool gyroMisalignment)
{
    std::vector<KeyType> keys;
    keys.reserve(3 + 3 + 4
                 + 3 * (static_cast<size_t>(accelBiases) + static_cast<size_t>(gyroBiases))
                 + 3 * (static_cast<size_t>(accelScaleFactor) + static_cast<size_t>(gyroScaleFactor))
                 + 4 * (static_cast<size_t>(accelMisalignment) + static_cast<size_t>(gyroMisalignment)));

    keys.insert(keys.end(), { Keys::PosX, Keys::PosY, Keys::PosZ,
                              Keys::VelX, Keys::VelY, Keys::VelZ,
                              Keys::AttQ1, Keys::AttQ2, Keys::AttQ3, Keys::AttQ0 });

    if (accelBiases) { keys.insert(keys.end(), Keys::AccelBias<KeyType>.begin(), Keys::AccelBias<KeyType>.end()); }
    if (gyroBiases) { keys.insert(keys.end(), Keys::GyroBias<KeyType>.begin(), Keys::GyroBias<KeyType>.end()); }
    if (accelScaleFactor) { keys.insert(keys.end(), Keys::AccelScaleFactor<KeyType>.begin(), Keys::AccelScaleFactor<KeyType>.end()); }
    if (gyroScaleFactor) { keys.insert(keys.end(), Keys::GyroScaleFactor<KeyType>.begin(), Keys::GyroScaleFactor<KeyType>.end()); }
    if (accelMisalignment) { keys.insert(keys.end(), Keys::AccelMisalignment<KeyType>.begin(), Keys::AccelMisalignment<KeyType>.end()); }
    if (gyroMisalignment) { keys.insert(keys.end(), Keys::GyroMisalignment<KeyType>.begin(), Keys::GyroMisalignment<KeyType>.end()); }

    KeyedMatrixX<T, KeyType, KeyType> F(Eigen::MatrixX<T>::Zero(static_cast<int>(keys.size()), static_cast<int>(keys.size())), keys, keys);

    F(Keys::Pos<KeyType>, Keys::Vel<KeyType>) = e_F_dr_dv<T>();
    F(Keys::Vel<KeyType>, Keys::Pos<KeyType>) = e_F_dv_dr(e_position,
                                                          config.gravitationModel != GravitationModel::None ? e_gravitation : Eigen::Vector3<T>::Zero(),
                                                          r_eS_e, config.centrifgalAccelerationCompensationEnabled ? InsConst::e_omega_ie : Eigen::Vector3d::Zero());
    F(Keys::Vel<KeyType>, Keys::Vel<KeyType>) = e_F_dv_dv<T>(config.coriolisAccelerationCompensationEnabled ? InsConst::omega_ie : 0.0);
    F(Keys::Vel<KeyType>, Keys::Att<KeyType>) = e_F_dv_dq(p_accelBias, p_accelScale, p_quatAccel_ps, ps_f_ip, b_quat_p, e_quat_b);
    F(Keys::Att<KeyType>, Keys::Att<KeyType>) = e_F_dq_dq(p_gyroBias, p_gyroScale, p_quatGyro_ps, ps_omega_ip, b_quat_p, e_quat_b,
                                                          config.angularRateEarthRotationCompensationEnabled ? InsConst::omega_ie : 0.0);

    if (accelBiases)
    {
        F(Keys::Vel<KeyType>, Keys::AccelBias<KeyType>) = e_F_dv_dAccelBias(b_quat_p, e_quat_b);
        if (tau_bad) { F(Keys::AccelBias<KeyType>, Keys::AccelBias<KeyType>).diagonal().setConstant(T(-1.0 / *tau_bad)); }
    }
    if (gyroBiases)
    {
        F(Keys::Att<KeyType>, Keys::GyroBias<KeyType>) = e_F_dq_dGyroBias(b_quat_p, e_quat_b);
        if (tau_bgd) { F(Keys::GyroBias<KeyType>, Keys::GyroBias<KeyType>).diagonal().setConstant(T(-1.0 / *tau_bgd)); }
    }
    if (accelScaleFactor)
    {
        F(Keys::Vel<KeyType>, Keys::AccelScaleFactor<KeyType>) = e_F_dv_dAccelScale(p_quatAccel_ps, ps_f_ip, b_quat_p, e_quat_b);
    }
    if (gyroScaleFactor)
    {
        F(Keys::Att<KeyType>, Keys::GyroScaleFactor<KeyType>) = e_F_dq_dGyroScale(p_quatGyro_ps, ps_omega_ip, b_quat_p, e_quat_b);
    }
    if (accelMisalignment)
    {
        F(Keys::Vel<KeyType>, Keys::AccelMisalignment<KeyType>) = e_F_dv_dAccelMisalignment(p_accelScale, p_quatAccel_ps, ps_f_ip, b_quat_p, e_quat_b);
    }
    if (gyroMisalignment)
    {
        F(Keys::Att<KeyType>, Keys::GyroMisalignment<KeyType>) = e_F_dq_dGyroMisalignment(p_gyroScale, p_quatGyro_ps, ps_omega_ip, b_quat_p, e_quat_b);
    }

    return F;
}

} // namespace NAV