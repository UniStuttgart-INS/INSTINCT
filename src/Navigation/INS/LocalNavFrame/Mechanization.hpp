/// @file Mechanization.hpp
/// @brief Inertial Navigation Mechanization Functions in local navigation frame
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-09-02

#pragma once

#include "Navigation/Gravity/Gravity.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>

namespace NAV
{
/// @brief Calculates the time derivative of the quaternion n_Quat_b
///
/// \anchor eq-INS-Mechanization-n_Quat_b-dot \f{equation}{ \label{eq:eq-INS-Mechanization-n_Quat_b-dot}
///   \mathbf{\dot{q}}_b^n
///    = \begin{bmatrix} \dotup{w} \\ \dotup{x} \\ \dotup{y} \\ \dotup{z} \end{bmatrix}
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
/// @note See \ref ImuIntegrator-Mechanization-n-Attitude-Quaternion equation \eqref{eq-ImuIntegrator-Mechanization-n-Attitude-Quaternion-matrix-Titterton}
Eigen::Vector4d calcTimeDerivativeFor_n_Quat_b(const Eigen::Vector3d& b_omega_nb, const Eigen::Vector4d& n_Quat_b_coeffs);

/// @brief Calculates the time derivative of the velocity in local-navigation frame coordinates
///
/// \anchor eq-INS-Mechanization-v_n-dot \f{equation}{ \label{eq:eq-INS-Mechanization-v_n-dot}
///   \boldsymbol{\dot{v}}^n
///       = \overbrace{\boldsymbol{f}^n}^{\hidewidth\text{measured}\hidewidth}
///         -\ \underbrace{(2 \boldsymbol{\omega}_{ie}^n + \boldsymbol{\omega}_{en}^n) \times \boldsymbol{v}^n}_{\text{coriolis acceleration}}
///         +\ \overbrace{\mathbf{g}^n}^{\hidewidth\text{gravitation}\hidewidth}
///         -\ \mathbf{C}_e^n \cdot \underbrace{\left(\boldsymbol{\omega}_{ie}^e \times [ \boldsymbol{\omega}_{ie}^e \times \mathbf{x}^e ] \right)}_{\text{centrifugal acceleration}}
/// \f}
///
/// @param[in] n_measuredForce f_n = [f_N  f_E  f_D]^T Specific force vector as measured by a triad of accelerometers and resolved into local-navigation frame coordinates
/// @param[in] n_coriolisAcceleration Coriolis acceleration in local-navigation coordinates in [m/s^2]
/// @param[in] n_gravitation Local gravitation vector (caused by effects of mass attraction) in local-navigation frame coordinates [m/s^2]
/// @param[in] n_centrifugalAcceleration Centrifugal acceleration in local-navigation coordinates in [m/s^2]
/// @return The time derivative of the velocity in local-navigation frame coordinates
///
/// @note See \ref ImuIntegrator-Mechanization-n-Velocity equation \eqref{eq-ImuIntegrator-Mechanization-n-Velocity}
Eigen::Vector3d n_calcTimeDerivativeForVelocity(const Eigen::Vector3d& n_measuredForce,
                                                const Eigen::Vector3d& n_coriolisAcceleration,
                                                const Eigen::Vector3d& n_gravitation,
                                                const Eigen::Vector3d& n_centrifugalAcceleration);

/// @brief Calculates the time derivative of the curvilinear position
///
/// \anchor eq-INS-Mechanization-p_lla-dot \f{equation}{ \label{eq:eq-INS-Mechanization-p_lla-dot}
/// \begin{aligned}
///   \dotup{\phi}    &= \frac{v_N}{R_N + h} \\
///   \dotup{\lambda} &= \frac{v_E}{(R_E + h) \cos{\phi}} \\
///   \dotup{h}       &= -v_D
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
/// @note See \ref ImuIntegrator-Mechanization-n-Position equation \eqref{eq-ImuIntegrator-Mechanization-n-Position}
Eigen::Vector3d lla_calcTimeDerivativeForPosition(const Eigen::Vector3d& n_velocity,
                                                  const double& phi,
                                                  const double& h,
                                                  const double& R_N,
                                                  const double& R_E);

/// @brief Values needed to calculate the PosVelAttDerivative for the local-navigation frame
struct PosVelAttDerivativeConstants_n
{
    Eigen::Vector3d b_omega_ib_dot;                              ///< ∂/∂t ω_ip_b Angular velocity rate in [rad/s²], of the inertial to platform system, in body coordinates
    Eigen::Vector3d b_measuredForce_dot;                         ///< ∂/∂t f_b Acceleration rate in [m/s^3], in body coordinates
    double timeDifferenceSec = 0;                                ///< Time difference Δtₖ = (tₖ - tₖ₋₁) in [seconds]
    GravitationModel gravitationModel = GravitationModel::EGM96; ///< Gravity Model to use
    bool coriolisAccelerationCompensationEnabled = true;         ///< Apply the coriolis acceleration compensation to the measured accelerations
    bool centrifgalAccelerationCompensationEnabled = true;       ///< Apply the centrifugal acceleration compensation to the measured accelerations
    bool angularRateEarthRotationCompensationEnabled = true;     ///< Apply the Earth rotation rate compensation to the measured angular rates
    bool angularRateTransportRateCompensationEnabled = true;     ///< Apply the transport rate compensation to the measured angular rates
};

/// @brief Calculates the derivative of the quaternion, velocity and curvilinear position
/// @param[in] y [w, x, y, z, v_N, v_E, v_D, 𝜙, λ, h, fx, fy, fz, ωx, ωy, ωz]^T
/// @param[in] c Constant values needed to calculate the derivatives
/// @return The derivative ∂/∂t [w, x, y, z, v_N, v_E, v_D, 𝜙, λ, h, fx, fy, fz, ωx, ωy, ωz]^T
Eigen::Matrix<double, 16, 1> n_calcPosVelAttDerivative(const Eigen::Matrix<double, 16, 1>& y, const PosVelAttDerivativeConstants_n& c);

} // namespace NAV
