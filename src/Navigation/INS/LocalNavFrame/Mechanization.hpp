// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Mechanization.hpp
/// @brief Inertial Navigation Mechanization Functions in local navigation frame
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-09-02

#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>

#include "Navigation/Constants.hpp"
#include "Navigation/Ellipsoid/Ellipsoid.hpp"
#include "Navigation/INS/Functions.hpp"
#include "Navigation/INS/Mechanization.hpp"
#include "Navigation/Gravity/Gravity.hpp"
#include "Navigation/Math/Math.hpp"
#include "Navigation/Math/NumericalIntegration.hpp"
#include "Navigation/Transformations/CoordinateFrames.hpp"
#include "Navigation/Transformations/Units.hpp"
#include "util/Logger.hpp"

#include <cmath>

namespace NAV
{
/// @brief Calculates the time derivative of the quaternion n_Quat_b
///
/// \anchor eq-INS-Mechanization-n_Quat_b-dot \f{equation}{ \label{eq:eq-INS-Mechanization-n_Quat_b-dot}
///   \mathbf{\dot{q}}_b^n
///    = \begin{bmatrix} \dot{w} \\ \dot{x} \\ \dot{y} \\ \dot{z} \end{bmatrix}
///    = \frac{1}{2} \begin{bmatrix}        0        & -\omega_{nb,x}^b & -\omega_{nb,y}^b & -\omega_{nb,z}^b \\
///                                  \omega_{nb,x}^b &        0         &  \omega_{nb,z}^b & -\omega_{nb,y}^b \\
///                                  \omega_{nb,y}^b & -\omega_{nb,z}^b &        0         &  \omega_{nb,x}^b \\
///                                  \omega_{nb,z}^b &  \omega_{nb,y}^b & -\omega_{nb,x}^b &        0         \end{bmatrix}
///                  \begin{bmatrix} w \\ x \\ y \\ z \end{bmatrix}
/// \f}
///
/// @param[in] b_omega_nb ω_nb_b Body rate with respect to the navigation frame, expressed in the body frame
/// @param[in] n_Quat_b_coeffs Coefficients of the quaternion n_Quat_b in order w, x, y, z (q = w + ix + jy + kz)
/// @return The time derivative of the coefficients of the quaternion n_Quat_b in order w, x, y, z (q = w + ix + jy + kz)
///
/// @note See \ref ImuIntegrator-Mechanization-n-Attitude-Quaternion equation \ref eq-ImuIntegrator-Mechanization-n-Attitude-Quaternion-matrix-Titterton
template<typename DerivedA, typename DerivedB>
Eigen::Vector4<typename DerivedA::Scalar> calcTimeDerivativeFor_n_Quat_b(const Eigen::MatrixBase<DerivedA>& b_omega_nb,
                                                                         const Eigen::MatrixBase<DerivedB>& n_Quat_b_coeffs)
{
    // Angular rates in matrix form (Titterton (2005), eq. (11.35))
    Eigen::Matrix4<typename DerivedA::Scalar> A;

    // clang-format off
    A <<       0.0     , -b_omega_nb(0), -b_omega_nb(1), -b_omega_nb(2),
          b_omega_nb(0),       0.0     ,  b_omega_nb(2), -b_omega_nb(1),
          b_omega_nb(1), -b_omega_nb(2),       0.0     ,  b_omega_nb(0),
          b_omega_nb(2),  b_omega_nb(1), -b_omega_nb(0),       0.0     ;
    // clang-format on

    // Propagation of an attitude Quaternion with time (Titterton, ch. 11.2.5, eq. 11.33-11.35, p. 319)
    return 0.5 * A * n_Quat_b_coeffs; // (w, x, y, z)
}

/// @brief Calculates the time derivative of the velocity in local-navigation frame coordinates
///
/// \anchor eq-INS-Mechanization-v_n-dot \f{equation}{ \label{eq:eq-INS-Mechanization-v_n-dot}
///   \boldsymbol{\dot{v}}^n
///       = \overbrace{\boldsymbol{f}^n}^{\text{measured}}
///         -\ \underbrace{(2 \boldsymbol{\omega}_{ie}^n + \boldsymbol{\omega}_{en}^n) \times \boldsymbol{v}^n}_{\text{coriolis acceleration}}
///         +\ \overbrace{\mathbf{g}^n}^{\text{gravitation}}
///         -\ \mathbf{C}_e^n \cdot \underbrace{\left(\boldsymbol{\omega}_{ie}^e \times [ \boldsymbol{\omega}_{ie}^e \times \mathbf{x}^e ] \right)}_{\text{centrifugal acceleration}}
/// \f}
///
/// @param[in] n_measuredForce f_n = [f_N  f_E  f_D]^T Specific force vector as measured by a triad of accelerometers and resolved into local-navigation frame coordinates
/// @param[in] n_coriolisAcceleration Coriolis acceleration in local-navigation coordinates in [m/s^2]
/// @param[in] n_gravitation Local gravitation vector (caused by effects of mass attraction) in local-navigation frame coordinates [m/s^2]
/// @param[in] n_centrifugalAcceleration Centrifugal acceleration in local-navigation coordinates in [m/s^2]
/// @return The time derivative of the velocity in local-navigation frame coordinates
///
/// @note See \ref ImuIntegrator-Mechanization-n-Velocity equation \ref eq-ImuIntegrator-Mechanization-n-Velocity
template<typename DerivedA, typename DerivedB, typename DerivedC, typename DerivedD>
Eigen::Vector3<typename DerivedA::Scalar> n_calcTimeDerivativeForVelocity(const Eigen::MatrixBase<DerivedA>& n_measuredForce,
                                                                          const Eigen::MatrixBase<DerivedB>& n_coriolisAcceleration,
                                                                          const Eigen::MatrixBase<DerivedC>& n_gravitation,
                                                                          const Eigen::MatrixBase<DerivedD>& n_centrifugalAcceleration)
{
    return n_measuredForce
           - n_coriolisAcceleration
           + n_gravitation
           - n_centrifugalAcceleration;
}

/// @brief Calculates the time derivative of the curvilinear position
///
/// \anchor eq-INS-Mechanization-p_lla-dot \f{equation}{ \label{eq:eq-INS-Mechanization-p_lla-dot}
/// \begin{aligned}
///   \dot{\phi}    &= \frac{v_N}{R_N + h} \\
///   \dot{\lambda} &= \frac{v_E}{(R_E + h) \cos{\phi}} \\
///   \dot{h}       &= -v_D
/// \end{aligned}
/// \f}
///
/// @param[in] n_velocity [v_N  v_E  v_D]^T Velocity with respect to the Earth in local-navigation frame coordinates [m/s]
/// @param[in] phi ϕ Latitude [rad]
/// @param[in] h Altitude above the ellipsoid [m]
/// @param[in] R_N North/South (meridian) earth radius [m]
/// @param[in] R_E East/West (prime vertical) earth radius [m]
/// @return The time derivative of the curvilinear position
///
/// @note See \ref ImuIntegrator-Mechanization-n-Position equation \ref eq-ImuIntegrator-Mechanization-n-Position
template<typename Derived>
Eigen::Vector3<typename Derived::Scalar> lla_calcTimeDerivativeForPosition(const Eigen::MatrixBase<Derived>& n_velocity,
                                                                           const typename Derived::Scalar& phi, const typename Derived::Scalar& h,
                                                                           const typename Derived::Scalar& R_N, const typename Derived::Scalar& R_E)
{
    // Velocity North in [m/s]
    const auto& v_N = n_velocity(0);
    // Velocity East in [m/s]
    const auto& v_E = n_velocity(1);
    // Velocity Down in [m/s]
    const auto& v_D = n_velocity(2);

    return { v_N / (R_N + h),
             v_E / ((R_E + h) * std::cos(phi)),
             -v_D };
}

/// @brief Calculates the derivative of the quaternion, velocity and curvilinear position
/// @param[in] y [w, x, y, z, v_N, v_E, v_D, 𝜙, λ, h]^T
/// @param[in] z [fx, fy, fz, ωx, ωy, ωz]^T
/// @param[in] c Constant values needed to calculate the derivatives
/// @return The derivative ∂/∂t [w, x, y, z, v_N, v_E, v_D, 𝜙, λ, h]^T
template<typename Scalar, typename = std::enable_if_t<std::is_floating_point_v<Scalar>>>
Eigen::Vector<Scalar, 10> n_calcPosVelAttDerivative(const Eigen::Vector<Scalar, 10>& y, const Eigen::Vector<Scalar, 6>& z, const PosVelAttDerivativeConstants<Scalar>& c, Scalar /* t */ = 0.0)
{
    //  0  1  2  3   4    5    6   7  8  9
    // [w, x, y, z, v_N, v_E, v_D, 𝜙, λ, h]^T
    Eigen::Vector<Scalar, 10> y_dot = Eigen::Vector<Scalar, 10>::Zero();

    Eigen::Quaternion<Scalar> n_Quat_b{ y(0), y(1), y(2), y(3) };
    n_Quat_b.normalize();
    Eigen::Quaternion<Scalar> n_Quat_e = trafo::n_Quat_e(y(7), y(8));

    LOG_DATA("rollPitchYaw = {} [°]", rad2deg(trafo::quat2eulerZYX(n_Quat_b)).transpose());
    LOG_DATA("n_velocity   = {} [m/s]", y.template segment<3>(4).transpose());
    LOG_DATA("lla_position = {}°, {}°, {} m", rad2deg(y(7)), rad2deg(y(8)), y(9));

    auto R_N = calcEarthRadius_N(y(7));
    LOG_DATA("R_N = {} [m]", R_N);
    auto R_E = calcEarthRadius_E(y(7));
    LOG_DATA("R_E = {} [m]", R_E);

    // ω_ie_n Turn rate of the Earth expressed in local-navigation frame coordinates
    Eigen::Vector3<Scalar> n_omega_ie = n_Quat_e * InsConst::e_omega_ie;
    LOG_DATA("n_omega_ie = {} [rad/s]", n_omega_ie.transpose());
    // ω_en_n Turn rate of the local frame with respect to the Earth-fixed frame, called the transport rate, expressed in local-navigation frame coordinates
    Eigen::Vector3<Scalar> n_omega_en = n_calcTransportRate(y.template segment<3>(7), y.template segment<3>(4), R_N, R_E);
    LOG_DATA("n_omega_en = {} [rad/s]", n_omega_en.transpose());
    // ω_nb_b = ω_ib_b - C_bn * (ω_ie_n + ω_en_n)
    Eigen::Vector3<Scalar> b_omega_nb = z.template segment<3>(3)
                                        - n_Quat_b.conjugate()
                                              * ((c.angularRateEarthRotationCompensationEnabled ? n_omega_ie : Eigen::Vector3<Scalar>::Zero())
                                                 + (c.angularRateTransportRateCompensationEnabled ? n_omega_en : Eigen::Vector3<Scalar>::Zero()));
    LOG_DATA("b_omega_nb = {} [rad/s]", b_omega_nb.transpose());

    // Coriolis acceleration in [m/s^2] (acceleration due to motion in rotating reference frame)
    Eigen::Vector3<Scalar> n_coriolisAcceleration = c.coriolisAccelerationCompensationEnabled
                                                        ? n_calcCoriolisAcceleration(n_omega_ie, n_omega_en, y.template segment<3>(4))
                                                        : Eigen::Vector3<Scalar>::Zero();
    LOG_DATA("n_coriolisAcceleration = {} [m/s^2]", n_coriolisAcceleration.transpose());
    // Centrifugal acceleration in [m/s^2] (acceleration that makes a body follow a curved path)
    Eigen::Vector3<Scalar> n_centrifugalAcceleration = c.centrifgalAccelerationCompensationEnabled
                                                           ? n_Quat_e * e_calcCentrifugalAcceleration(trafo::lla2ecef_WGS84(y.template segment<3>(7)), InsConst::e_omega_ie)
                                                           : Eigen::Vector3<Scalar>::Zero();
    LOG_DATA("n_centrifugalAcceleration = {} [m/s^2]", n_centrifugalAcceleration.transpose());

    Eigen::Vector3<Scalar> n_gravitation = n_calcGravitation(y.template segment<3>(7), c.gravitationModel);
    LOG_DATA("n_gravitation = {} [m/s^2] ({})", n_gravitation.transpose(), to_string(c.gravitationModel));

    y_dot.template segment<4>(0) = calcTimeDerivativeFor_n_Quat_b(b_omega_nb,                // ω_nb_b Body rate with respect to the navigation frame, expressed in the body frame
                                                                  y.template segment<4>(0)); // n_Quat_b_coeffs Coefficients of the quaternion n_Quat_b in order w, x, y, z (q = w + ix + jy + kz)

    y_dot.template segment<3>(4) = n_calcTimeDerivativeForVelocity(n_Quat_b * z.template segment<3>(0), // f_n Specific force vector as measured by a triad of accelerometers and resolved into local-navigation frame coordinates
                                                                   n_coriolisAcceleration,              // Coriolis acceleration in local-navigation coordinates in [m/s^2]
                                                                   n_gravitation,                       // Local gravitation vector (caused by effects of mass attraction) in local-navigation frame coordinates [m/s^2]
                                                                   n_centrifugalAcceleration);          // Centrifugal acceleration in local-navigation coordinates in [m/s^2]

    y_dot.template segment<3>(7) = lla_calcTimeDerivativeForPosition(y.template segment<3>(4), // Velocity with respect to the Earth in local-navigation frame coordinates [m/s]
                                                                     y(7),                     // 𝜙 Latitude in [rad]
                                                                     y(9),                     // h Altitude in [m]
                                                                     R_N,                      // North/South (meridian) earth radius [m]
                                                                     R_E);                     // East/West (prime vertical) earth radius [m]

    LOG_DATA("n_Quat_b_dot = {} ", y_dot.template segment<4>(0).transpose());
    LOG_DATA("n_velocity_dot = {} [m/s^2]", y_dot.template segment<3>(4).transpose());
    LOG_DATA("lla_position_dot = {} [rad/s, rad/s, m/s]", y_dot.template segment<3>(7).transpose());

    return y_dot;
}

} // namespace NAV
