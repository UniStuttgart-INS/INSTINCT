/// @file Mechanization.hpp
/// @brief Inertial Navigation Mechanization Functions
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-09-02

#pragma once

#include "Navigation/Gravity/Gravity.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>

namespace NAV
{

/// @brief Calculates the time derivative of the quaternion q_nb
///
/// \anchor eq-INS-Mechanization-q_nb-dot \f{equation}{ \label{eq:eq-INS-Mechanization-q_nb-dot}
///   \mathbf{\dot{q}}_b^n
///    = \begin{bmatrix} \dotup{w} \\ \dotup{x} \\ \dotup{y} \\ \dotup{z} \end{bmatrix}
///    = \frac{1}{2} \begin{bmatrix}        0        & -\omega_{nb,x}^b & -\omega_{nb,y}^b & -\omega_{nb,z}^b \\
///                                  \omega_{nb,x}^b &        0         &  \omega_{nb,z}^b & -\omega_{nb,y}^b \\
///                                  \omega_{nb,y}^b & -\omega_{nb,z}^b &        0         &  \omega_{nb,x}^b \\
///                                  \omega_{nb,z}^b &  \omega_{nb,y}^b & -\omega_{nb,x}^b &        0         \end{bmatrix}
///                  \begin{bmatrix} w \\ x \\ y \\ z \end{bmatrix}
/// \f}
///
/// @param[in] omega_nb_b ω_nb_b Body rate with respect to the navigation frame, expressed in the body frame
/// @param[in] q_nb_coeffs Coefficients of the quaternion q_nb in order w, x, y, z (q = w + ix + jy + kz)
/// @return The time derivative of the coefficients of the quaternion q_nb in order w, x, y, z (q = w + ix + jy + kz)
///
/// @note See \ref ImuIntegrator-Mechanization-n-Attitude-Quaternion equation \eqref{eq-ImuIntegrator-Mechanization-n-Attitude-Quaternion-matrix-Titterton}
Eigen::Vector4d calcTimeDerivativeForQuaternion_nb(const Eigen::Vector3d& omega_nb_b, const Eigen::Vector4d& q_nb_coeffs);

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
/// @param[in] f_n f_n = [f_N  f_E  f_D]^T Specific force vector as measured by a triad of accelerometers and resolved into local-navigation frame coordinates
/// @param[in] omega_ie_e Turn rate of the Earth expressed in Earth frame coordinates
/// @param[in] omega_ie_n Turn rate of the Earth expressed in local-navigation frame coordinates
/// @param[in] omega_en_n Turn rate of the local frame with respect to the Earth-fixed frame, called the transport rate, expressed in local-navigation frame coordinates
/// @param[in] velocity_n v_n = [v_N  v_E  v_D]^T Velocity with respect to the Earth in local-navigation frame coordinates [m/s]
/// @param[in] gravitation_n Local gravitation vector (caused by effects of mass attraction) in local-navigation frame coordinates [m/s^2]
/// @param[in] q_ne Rotation quaternion which converts vectors in Earth frame to local-navigation frame coordinates (r_n = q_ne * r_e)
/// @param[in] x_e Position in ECEF coordinates
/// @param[in] coriolisAccelerationCompensationEnabled Apply the coriolis acceleration compensation to the measured accelerations
/// @param[in] centrifgalAccelerationCompensationEnabled Apply the centrifugal acceleration compensation to the measured accelerations
/// @return The time derivative of the velocity in local-navigation frame coordinates
///
/// @note See \ref ImuIntegrator-Mechanization-n-Velocity equation \eqref{eq-ImuIntegrator-Mechanization-n-Velocity}
Eigen::Vector3d calcTimeDerivativeForVelocity_n(const Eigen::Vector3d& f_n,
                                                const Eigen::Vector3d& omega_ie_e,
                                                const Eigen::Vector3d& omega_ie_n,
                                                const Eigen::Vector3d& omega_en_n,
                                                const Eigen::Vector3d& velocity_n,
                                                const Eigen::Vector3d& gravitation_n,
                                                const Eigen::Quaterniond& q_ne,
                                                const Eigen::Vector3d& x_e,
                                                bool coriolisAccelerationCompensationEnabled = true,
                                                bool centrifgalAccelerationCompensationEnabled = true);

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
/// @param[in] velocity_n v_n = [v_N  v_E  v_D]^T Velocity with respect to the Earth in local-navigation frame coordinates [m/s]
/// @param[in] phi ϕ Latitude [rad]
/// @param[in] h Altitude above the ellipsoid [m]
/// @param[in] R_N North/South (meridian) earth radius [m]
/// @param[in] R_E East/West (prime vertical) earth radius [m]
/// @return The time derivative of the curvilinear position
///
/// @note See \ref ImuIntegrator-Mechanization-n-Position equation \eqref{eq-ImuIntegrator-Mechanization-n-Position}
Eigen::Vector3d calcTimeDerivativeForPosition_lla(const Eigen::Vector3d& velocity_n,
                                                  const double& phi,
                                                  const double& h,
                                                  const double& R_N,
                                                  const double& R_E);

/// @brief Values needed to calculate the PosVelAttDerivative for the local-navigation frame
struct PosVelAttDerivativeConstants_n
{
    Eigen::Vector3d omega_ib_b;                       ///< ω_ip_b Angular velocity in [rad/s], of the inertial to platform system, in body coordinates
    Eigen::Vector3d f_b;                              ///< f_b Acceleration in [m/s^2], in body coordinates
    GravityModel gravityModel;                        ///< Gravity Model to use
    bool coriolisAccelerationCompensationEnabled;     ///< Apply the coriolis acceleration compensation to the measured accelerations
    bool centrifgalAccelerationCompensationEnabled;   ///< Apply the centrifugal acceleration compensation to the measured accelerations
    bool angularRateEarthRotationCompensationEnabled; ///< Apply the Earth rotation rate compensation to the measured angular rates
    bool angularRateTransportRateCompensationEnabled; ///< Apply the transport rate compensation to the measured angular rates
};

/// @brief Calculates the derivative of the quaternion, velocity and curvilinear position
/// @param[in] y [w, x, y, z, v_N, v_E, v_D, 𝜙, λ, h]^T
/// @param[in] c Constant values needed to calculate the derivatives
/// @return The derivative ∂/∂t [w, x, y, z, v_N, v_E, v_D, 𝜙, λ, h]^T
Eigen::Matrix<double, 10, 1> calcPosVelAttDerivative_n(const Eigen::Matrix<double, 10, 1>& y, const PosVelAttDerivativeConstants_n& c);

} // namespace NAV
