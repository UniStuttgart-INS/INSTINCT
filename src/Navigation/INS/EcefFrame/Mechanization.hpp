// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Mechanization.hpp
/// @brief Inertial Navigation Mechanization Functions in ECEF frame
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-06-12

#pragma once

#include "Navigation/Gravity/Gravity.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>

#include "Navigation/Constants.hpp"
#include "Navigation/Ellipsoid/Ellipsoid.hpp"
#include "Navigation/INS/Functions.hpp"
#include "Navigation/INS/Mechanization.hpp"
#include "Navigation/Math/Math.hpp"
#include "Navigation/Transformations/CoordinateFrames.hpp"
#include "util/Logger.hpp"

#include <cmath>

namespace NAV
{
/// @brief Calculates the time derivative of the quaternion e_Quat_b
///
/// \anchor eq-INS-Mechanization-e_Quat_b-dot \f{equation}{ \label{eq:eq-INS-Mechanization-e_Quat_b-dot}
///   \mathbf{\dot{q}}_b^e
///    = \begin{bmatrix} \dot{x} \\ \dot{y} \\ \dot{z} \\ \dot{w} \end{bmatrix}
///    = \frac{1}{2} \begin{bmatrix}        0         &  \omega_{eb,z}^b & -\omega_{eb,y}^b & \omega_{eb,x}^b \\
///                                  -\omega_{eb,z}^b &        0         &  \omega_{eb,x}^b & \omega_{eb,y}^b \\
///                                   \omega_{eb,y}^b & -\omega_{eb,x}^b &        0         & \omega_{eb,z}^b \\
///                                  -\omega_{eb,x}^b & -\omega_{eb,y}^b & -\omega_{eb,z}^b &        0        \end{bmatrix}
///                  \begin{bmatrix} x \\ y \\ z \\ w \end{bmatrix}
/// \f}
///
/// @param[in] b_omega_eb ω_eb_b Body rate with respect to the ECEF frame, expressed in the body frame
/// @param[in] e_Quat_b_coeffs Coefficients of the quaternion e_Quat_b in order x, y, z, w (q = w + ix + jy + kz)
/// @return The time derivative of the coefficients of the quaternion e_Quat_b in order x, y, z, w (q = w + ix + jy + kz)
///
/// @note See \ref ImuIntegrator-Mechanization-e-Attitude-Quaternion equation \eqref{eq-ImuIntegrator-Mechanization-e-Attitude-Quaternion-matrix}
template<typename DerivedA, typename DerivedB>
Eigen::Vector4<typename DerivedA::Scalar> calcTimeDerivativeFor_e_Quat_b(const Eigen::MatrixBase<DerivedA>& b_omega_eb,
                                                                         const Eigen::MatrixBase<DerivedB>& e_Quat_b_coeffs)
{
    using T = typename DerivedA::Scalar;

    // Angular rates in matrix form (Titterton (2005), eq. (11.35))
    Eigen::Matrix4<T> A;

    // clang-format off
    A <<     T(0.0)    ,  b_omega_eb(2), -b_omega_eb(1), b_omega_eb(0),
         -b_omega_eb(2),     T(0.0)    ,  b_omega_eb(0), b_omega_eb(1),
          b_omega_eb(1), -b_omega_eb(0),     T(0.0)    , b_omega_eb(2),
         -b_omega_eb(0), -b_omega_eb(1), -b_omega_eb(2),    T(0.0)    ;
    // clang-format on

    // Propagation of an attitude Quaternion with time (Titterton, ch. 11.2.5, eq. 11.33-11.35, p. 319)
    return 0.5 * A * e_Quat_b_coeffs; // (x, y, z, w)
}

/// @brief Calculates the time derivative of the velocity in ECEF frame coordinates
///
/// \anchor eq-INS-Mechanization-v_e-dot \f{equation}{ \label{eq:eq-INS-Mechanization-v_e-dot}
///   \boldsymbol{\dot{v}}^e
///       = \overbrace{\boldsymbol{f}^e}^{\hidewidth\text{measured}\hidewidth}
///         -\ \underbrace{2 \boldsymbol{\omega}_{ie}^e \times \boldsymbol{v}^e}_{\text{coriolis acceleration}}
///         +\ \overbrace{\mathbf{g}^e}^{\hidewidth\text{gravitation}\hidewidth}
///         -\ \underbrace{\left(\boldsymbol{\omega}_{ie}^e \times [ \boldsymbol{\omega}_{ie}^e \times \mathbf{x}^e ] \right)}_{\text{centrifugal acceleration}}
/// \f}
///
/// @param[in] e_measuredForce f_e Specific force vector as measured by a triad of accelerometers and resolved into ECEF frame coordinates
/// @param[in] e_coriolisAcceleration Coriolis acceleration in ECEF coordinates in [m/s^2]
/// @param[in] e_gravitation Local gravitation vector (caused by effects of mass attraction) in ECEF frame coordinates [m/s^2]
/// @param[in] e_centrifugalAcceleration Centrifugal acceleration in ECEF coordinates in [m/s^2]
/// @return The time derivative of the velocity in ECEF frame coordinates
///
/// @note See \ref ImuIntegrator-Mechanization-e-Velocity equation \eqref{eq-ImuIntegrator-Mechanization-e-Velocity}
template<typename DerivedA, typename DerivedB, typename DerivedC, typename DerivedD>
Eigen::Vector3<typename DerivedA::Scalar> e_calcTimeDerivativeForVelocity(const Eigen::MatrixBase<DerivedA>& e_measuredForce,
                                                                          const Eigen::MatrixBase<DerivedB>& e_coriolisAcceleration,
                                                                          const Eigen::MatrixBase<DerivedC>& e_gravitation,
                                                                          const Eigen::MatrixBase<DerivedD>& e_centrifugalAcceleration)
{
    return e_measuredForce
           - e_coriolisAcceleration
           + e_gravitation
           - e_centrifugalAcceleration;
}

/// @brief Calculates the time derivative of the ECEF position
///
/// \anchor eq-INS-Mechanization-x_e-dot \f{equation}{ \label{eq:eq-INS-Mechanization-x_e-dot}
///   \boldsymbol{\dot{x}}^e = \boldsymbol{v}^e
/// \f}
///
/// @param[in] e_velocity Velocity with respect to the Earth in ECEF frame coordinates [m/s]
/// @return The time derivative of the ECEF position
///
/// @note See \ref ImuIntegrator-Mechanization-e-Position equation \eqref{eq-ImuIntegrator-Mechanization-e-Position}
template<typename Derived>
Eigen::Vector3<typename Derived::Scalar> e_calcTimeDerivativeForPosition(const Eigen::MatrixBase<Derived>& e_velocity)
{
    return e_velocity;
}

/// @brief Calculates the derivative of the quaternion, velocity and position in ECEF coordinates
/// @param[in] y [x, y, z, v_x, v_y, v_z, e_q_bx, e_q_by, e_q_bz, e_q_bw]^T
/// @param[in] z [fx, fy, fz, ωx, ωy, ωz]^T
/// @param[in] c Constant values needed to calculate the derivatives
/// @return The derivative ∂/∂t [x, y, z, v_x, v_y, v_z, e_q_bx, e_q_by, e_q_bz, e_q_bw]^T
template<typename T>
Eigen::Vector<T, 10> e_calcPosVelAttDerivative(const Eigen::Vector<T, 10>& y, const Eigen::Vector<T, 6>& z, const PosVelAttDerivativeConstants& c, double /* t */ = 0.0)
{
    //        0  1  2   3    4    5     6       7       8       9
    // ∂/∂t [ x, y, z, v_x, v_y, v_z, e_q_bx, e_q_by, e_q_bz, e_q_bw]^T
    Eigen::Vector<T, 10> y_dot = Eigen::Vector<T, 10>::Zero();

    Eigen::Vector3<T> lla_position = trafo::ecef2lla_WGS84(y.template segment<3>(0));

    Eigen::Quaternion<T> e_Quat_b{ y.template segment<4>(6) };
    e_Quat_b.normalize();
    Eigen::Quaternion<T> n_Quat_e = trafo::n_Quat_e(lla_position(0), lla_position(1));

    Eigen::Quaternion<T> b_Quat_e = e_Quat_b.conjugate();
    Eigen::Quaternion<T> e_Quat_n = n_Quat_e.conjugate();

    LOG_DATA("e_velocity   = {} [m/s]", y.template segment<3>(3).transpose());
    LOG_DATA("e_position = {} [m]", y.template segment<3>(0).transpose());

    // ω_eb_b = ω_ib_b - C_be * ω_ie_e
    Eigen::Vector3<T> b_omega_eb = z.template segment<3>(3)
                                   - b_Quat_e * (c.angularRateEarthRotationCompensationEnabled ? InsConst::e_omega_ie : Eigen::Vector3d::Zero()).template cast<T>();
    LOG_DATA("b_omega_eb = {} [rad/s]", b_omega_eb.transpose());

    // Coriolis acceleration in [m/s^2] (acceleration due to motion in rotating reference frame)
    Eigen::Vector3<T> e_coriolisAcceleration = c.coriolisAccelerationCompensationEnabled
                                                   ? e_calcCoriolisAcceleration(InsConst::e_omega_ie, y.template segment<3>(3))
                                                   : Eigen::Vector3<T>::Zero();
    LOG_DATA("e_coriolisAcceleration = {} [m/s^2]", e_coriolisAcceleration.transpose());
    // Centrifugal acceleration in [m/s^2] (acceleration that makes a body follow a curved path)
    Eigen::Vector3<T> e_centrifugalAcceleration = c.centrifgalAccelerationCompensationEnabled
                                                      ? e_calcCentrifugalAcceleration(y.template segment<3>(0), InsConst::e_omega_ie)
                                                      : Eigen::Vector3<T>::Zero();
    LOG_DATA("e_centrifugalAcceleration = {} [m/s^2]", e_centrifugalAcceleration.transpose());

    Eigen::Vector3<T> e_gravitation = e_Quat_n * n_calcGravitation(lla_position, c.gravitationModel);
    LOG_DATA("e_gravitation = {} [m/s^2] ({})", e_gravitation.transpose(), to_string(c.gravitationModel));

    y_dot.template segment<3>(0) = e_calcTimeDerivativeForPosition(y.template segment<3>(3)); // Velocity with respect to the Earth in ECEF frame coordinates [m/s]

    y_dot.template segment<3>(3) = e_calcTimeDerivativeForVelocity(e_Quat_b * z.template segment<3>(0), // f_n Specific force vector as measured by a triad of accelerometers and resolved into ECEF frame coordinates
                                                                   e_coriolisAcceleration,              // Coriolis acceleration in ECEF coordinates in [m/s^2]
                                                                   e_gravitation,                       // Local gravitation vector (caused by effects of mass attraction) in ECEF frame coordinates [m/s^2]
                                                                   e_centrifugalAcceleration);          // Centrifugal acceleration in ECEF coordinates in [m/s^2]

    y_dot.template segment<4>(6) = calcTimeDerivativeFor_e_Quat_b(b_omega_eb,                // ω_eb_b Body rate with respect to the ECEF frame, expressed in the body frame
                                                                  y.template segment<4>(6)); // e_Quat_b_coeffs Coefficients of the quaternion e_Quat_b

    LOG_DATA("e_position_dot = {} [m/s]", y_dot.template segment<3>(0).transpose());
    LOG_DATA("e_velocity_dot = {} [m/s^2]", y_dot.template segment<3>(3).transpose());
    LOG_DATA("e_Quat_b_dot = {} ", y_dot.template segment<4>(6).transpose());

    return y_dot;
}

} // namespace NAV
