/// @file Mechanization.hpp
/// @brief Inertial Navigation Mechanization Functions in ECEF frame
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-06-12

#pragma once

#include "Navigation/Gravity/Gravity.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>

namespace NAV
{
/// @brief Calculates the time derivative of the quaternion e_Quat_b
///
/// \anchor eq-INS-Mechanization-e_Quat_b-dot \f{equation}{ \label{eq:eq-INS-Mechanization-e_Quat_b-dot}
///   \mathbf{\dot{q}}_b^e
///    = \begin{bmatrix} \dotup{w} \\ \dotup{x} \\ \dotup{y} \\ \dotup{z} \end{bmatrix}
///    = \frac{1}{2} \begin{bmatrix}        0        & -\omega_{eb,x}^b & -\omega_{eb,y}^b & -\omega_{eb,z}^b \\
///                                  \omega_{eb,x}^b &        0         &  \omega_{eb,z}^b & -\omega_{eb,y}^b \\
///                                  \omega_{eb,y}^b & -\omega_{eb,z}^b &        0         &  \omega_{eb,x}^b \\
///                                  \omega_{eb,z}^b &  \omega_{eb,y}^b & -\omega_{eb,x}^b &        0         \end{bmatrix}
///                  \begin{bmatrix} w \\ x \\ y \\ z \end{bmatrix}
/// \f}
///
/// @param[in] b_omega_eb ω_eb_b Body rate with respect to the ECEF frame, expressed in the body frame
/// @param[in] e_Quat_b_coeffs Coefficients of the quaternion e_Quat_b in order w, x, y, z (q = w + ix + jy + kz)
/// @return The time derivative of the coefficients of the quaternion e_Quat_b in order w, x, y, z (q = w + ix + jy + kz)
///
/// @note See \ref ImuIntegrator-Mechanization-e-Attitude-Quaternion equation \eqref{eq-ImuIntegrator-Mechanization-e-Attitude-Quaternion-matrix}
Eigen::Vector4d calcTimeDerivativeFor_e_Quat_b(const Eigen::Vector3d& b_omega_eb, const Eigen::Vector4d& e_Quat_b_coeffs);

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
Eigen::Vector3d e_calcTimeDerivativeForVelocity(const Eigen::Vector3d& e_measuredForce,
                                                const Eigen::Vector3d& e_coriolisAcceleration,
                                                const Eigen::Vector3d& e_gravitation,
                                                const Eigen::Vector3d& e_centrifugalAcceleration);

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
Eigen::Vector3d e_calcTimeDerivativeForPosition(const Eigen::Vector3d& e_velocity);

/// @brief Values needed to calculate the PosVelAttDerivative for the ECEF frame
struct PosVelAttDerivativeConstants_e
{
    Eigen::Vector3d b_omega_ib_dot;                              ///< ∂/∂t ω_ip_b Angular velocity rate in [rad/s²], of the inertial to platform system, in body coordinates
    Eigen::Vector3d b_measuredForce_dot;                         ///< ∂/∂t f_b Acceleration rate in [m/s^3], in body coordinates
    double timeDifferenceSec = 0;                                ///< Time difference Δtₖ = (tₖ - tₖ₋₁) in [seconds]
    GravitationModel gravitationModel = GravitationModel::EGM96; ///< Gravity Model to use
    bool coriolisAccelerationCompensationEnabled = true;         ///< Apply the coriolis acceleration compensation to the measured accelerations
    bool centrifgalAccelerationCompensationEnabled = true;       ///< Apply the centrifugal acceleration compensation to the measured accelerations
    bool angularRateEarthRotationCompensationEnabled = true;     ///< Apply the Earth rotation rate compensation to the measured angular rates
    bool velocityUpdateRotationCorrectionEnabled = false;        ///< Apply Zwiener's rotation correction for the velocity update
};

/// @brief Calculates the derivative of the quaternion, velocity and position in ECEF coordinates
/// @param[in] y [w, x, y, z, v_x, v_y, v_z, x, y, z, fx, fy, fz, ωx, ωy, ωz]^T
/// @param[in] c Constant values needed to calculate the derivatives
/// @return The derivative ∂/∂t [w, x, y, z, v_x, v_y, v_z, x, y, z, fx, fy, fz, ωx, ωy, ωz]^T
Eigen::Matrix<double, 16, 1> e_calcPosVelAttDerivative(const Eigen::Matrix<double, 16, 1>& y, const PosVelAttDerivativeConstants_e& c);

} // namespace NAV
