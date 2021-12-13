/// @file Mechanization.hpp
/// @brief Inertial Navigation Mechanization Functions
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-09-02

#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>

#include "NodeData/State/PosVelAtt.hpp"
#include "NodeData/State/PVAError.hpp"

#include "Functions.hpp"

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
                                                const Eigen::Vector3d& x_e);

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

// ###########################################################################################################
//                                              Attitude Update
// ###########################################################################################################

/// @brief Integrates the angular velocities and calculates the new Quaternion q_nb with Runge Kutta of 1st Order
/// @param[in] timeDifferenceSec__t0 Δtₖ Time difference in [seconds]. This epoch to previous epoch
/// @param[in] angularVelocity_ip_b__t0 ω_ip_b (tₖ) Angular velocity in [rad/s],
///                                     of the inertial to platform system, in body coordinates, at the time tₖ
/// @param[in] angularVelocity_ie_n__t1 ω_ie_n (tₖ₋₁) Angular velocity in [rad/s]
///                                     of the inertial to earth system, in navigation coordinates, at the time tₖ₋₁
/// @param[in] angularVelocity_en_n__t1 ω_en_n (tₖ₋₁) Transport Rate, rotation rate of the Earth frame relative to the navigation frame
///                                     in navigation coordinates
/// @param[in] quaternion_nb__t1 q (tₖ₋₁) Quaternion, from body to navigation coordinates, at the time tₖ₋₁
/// @return The updated Quaternion q_nb
/// @note See C. Jekeli (2001) - Inertial Navigation Systems with Geodetic Applications (Chapter 4.2.4.1.2)
[[nodiscard]] Eigen::Quaterniond updateQuaternion_nb_RungeKutta1(const long double& timeDifferenceSec__t0,
                                                                 const Eigen::Vector3d& angularVelocity_ip_b__t0,
                                                                 const Eigen::Vector3d& angularVelocity_ie_n__t1,
                                                                 const Eigen::Vector3d& angularVelocity_en_n__t1,
                                                                 const Eigen::Quaterniond& quaternion_nb__t1);

/// @brief Integrates the angular velocities and calculates the new Quaternion q_ep with Runge Kutta of 3rd Order
/// @param[in] timeDifferenceSec__t0 Δtₖ Time difference in [seconds]. This epoch to previous epoch
/// @param[in] timeDifferenceSec__t1 Δtₖ₋₁ Time difference in [seconds]. Previous epoch to twice previous epoch
/// @param[in] angularVelocity_ip_p__t0 ω_ip_p (tₖ) Angular velocity in [rad/s],
///                                     of the inertial to platform system, in platform coordinates, at the time tₖ
/// @param[in] angularVelocity_ip_p__t1 ω_ip_p (tₖ₋₁) Angular velocity in [rad/s],
///                                     of the inertial to platform system, in platform coordinates, at the time tₖ₋₁
/// @param[in] angularVelocity_ie_e__t0 ω_ie_e (tₖ) Angular velocity in [rad/s],
///                                     of the inertial to earth system, in earth coordinates, at the time tₖ
/// @param[in] quaternion_ep__t1 q (tₖ₋₁) Quaternion, from platform to earth coordinates, at the time tₖ₋₁
/// @param[in] quaternion_ep__t2 q (tₖ₋₂) Quaternion, from platform to earth coordinates, at the time tₖ₋₂
/// @return The updated Quaternion q_ep
/// @note See C. Jekeli (2001) - Inertial Navigation Systems with Geodetic Applications (Chapter 4.2.4.1.2)
///       See T. Hobiger - Inertialnavigation (Lecture Slides Chapter 8)
[[nodiscard]] Eigen::Quaterniond updateQuaternion_ep_RungeKutta3(const long double& timeDifferenceSec__t0,
                                                                 const long double& timeDifferenceSec__t1,
                                                                 const Eigen::Vector3d& angularVelocity_ip_p__t0,
                                                                 const Eigen::Vector3d& angularVelocity_ip_p__t1,
                                                                 const Eigen::Vector3d& angularVelocity_ie_e__t0,
                                                                 const Eigen::Quaterniond& quaternion_ep__t1,
                                                                 const Eigen::Quaterniond& quaternion_ep__t2);

/// @brief Integrates the angular velocities and calculates the new Quaternion q_nb with Runge Kutta of 3rd Order
/// @param[in] timeDifferenceSec__t0 Δtₖ Time difference in [seconds]. This epoch to previous epoch
/// @param[in] timeDifferenceSec__t1 Δtₖ₋₁ Time difference in [seconds]. Previous epoch to twice previous epoch
/// @param[in] angularVelocity_ip_b__t0 ω_ip_b (tₖ) Angular velocity in [rad/s],
///                                     of the inertial to platform system, in body coordinates, at the time tₖ
/// @param[in] angularVelocity_ip_b__t1 ω_ip_b (tₖ₋₁) Angular velocity in [rad/s],
///                                     of the inertial to platform system, in body coordinates, at the time tₖ₋₁
/// @param[in] angularVelocity_ie_n__t1 ω_ie_n (tₖ₋₁) Angular velocity in [rad/s]
///                                     of the inertial to earth system, in navigation coordinates, at the time tₖ₋₁
/// @param[in] angularVelocity_en_n__t1 ω_en_n (tₖ₋₁) Transport Rate, rotation rate of the Earth frame relative to the navigation frame
///                                     in navigation coordinates
/// @param[in] quaternion_nb__t1 q (tₖ₋₁) Quaternion, from body to navigation coordinates, at the time tₖ₋₁
/// @param[in] quaternion_nb__t2 q (tₖ₋₂) Quaternion, from body to navigation coordinates, at the time tₖ₋₂
/// @return The updated Quaternion q_nb
/// @note See C. Jekeli (2001) - Inertial Navigation Systems with Geodetic Applications (Chapter 4.2.4.1.2)
///       See T. Hobiger - Inertialnavigation (Lecture Slides Chapter 8)
[[nodiscard]] Eigen::Quaterniond updateQuaternion_nb_RungeKutta3(const long double& timeDifferenceSec__t0,
                                                                 const long double& timeDifferenceSec__t1,
                                                                 const Eigen::Vector3d& angularVelocity_ip_b__t0,
                                                                 const Eigen::Vector3d& angularVelocity_ip_b__t1,
                                                                 const Eigen::Vector3d& angularVelocity_ie_n__t1,
                                                                 const Eigen::Vector3d& angularVelocity_en_n__t1,
                                                                 const Eigen::Quaterniond& quaternion_nb__t1,
                                                                 const Eigen::Quaterniond& quaternion_nb__t2);

// ###########################################################################################################
//                                              Velocity Update
// ###########################################################################################################

/// @brief Integrates the accelerations and calculates the new velocity v_n with Runge Kutta of 1st Order
/// @param[in, out] timeDifferenceSec__t0 Δtₖ Time difference in [seconds]. This epoch to previous epoch
/// @param[in, out] acceleration_b__t0 a_p (tₖ) Acceleration in [m/s^2], in body coordinates, at the time tₖ
/// @param[in, out] velocity_n__t1 v_n (tₖ₋₂) Velocity in [m/s], in navigation coordinates, at the time tₖ₋₁
/// @param[in, out] gravity_n__t1 g_n (tₖ₋₁) Gravity vector in [m/s^2], in navigation coordinates, at the time tₖ₋₁
/// @param[in, out] angularVelocity_ie_n__t1 ω_ie_n (tₖ₋₁) Nominal mean angular velocity of the Earth in [rad/s], in navigation coordinates, at the time tₖ₋₁
/// @param[in, out] angularVelocity_en_n__t1 ω_ie_n (tₖ₋₁) Transport Rate in [rad/s], in navigation coordinates, at the time tₖ₋₁
/// @param[in, out] quaternion_nb__t1 q (tₖ₋₁) Quaternion, from body to navigation coordinates, at the time tₖ₋₁
/// @return The updated velocity v_n
[[nodiscard]] Eigen::Vector3d updateVelocity_n_RungeKutta1(const long double& timeDifferenceSec__t0,
                                                           const Eigen::Vector3d& acceleration_b__t0,
                                                           const Eigen::Vector3d& velocity_n__t1,
                                                           const Eigen::Vector3d& gravity_n__t1,
                                                           const Eigen::Vector3d& angularVelocity_ie_n__t1,
                                                           const Eigen::Vector3d& angularVelocity_en_n__t1,
                                                           const Eigen::Quaterniond& quaternion_nb__t1);

/// @brief Integrates the accelerations and calculates the new velocity v_e with Simpson rule
/// @param[in] timeDifferenceSec__t0 Δtₖ Time difference in [seconds]. This epoch to previous epoch
/// @param[in] timeDifferenceSec__t1 Δtₖ₋₁ Time difference in [seconds]. Previous epoch to twice previous epoch
/// @param[in] acceleration_p__t0 a_p (tₖ) Acceleration in [m/s^2], in platform coordinates, at the time tₖ
/// @param[in] acceleration_p__t1 a_p (tₖ₋₁) Acceleration in [m/s^2], in platform coordinates, at the time tₖ₋₁
/// @param[in] velocity_e__t2 v_e (tₖ₋₂) Velocity in [m/s], in earth coordinates, at the time tₖ₋₂
/// @param[in] position_e__t2 x_e (tₖ₋₂) Position in [m/s], in earth coordinates, at the time tₖ₋₂
/// @param[in] gravity_e g_e Gravity vector in [m/s^2], in earth coordinates
/// @param[in] quaternion_ep__t0 q (tₖ) Quaternion, from platform to earth coordinates, at the time tₖ
/// @param[in] quaternion_ep__t1 q (tₖ₋₁) Quaternion, from platform to earth coordinates, at the time tₖ₋₁
/// @param[in] quaternion_ep__t2 q (tₖ₋₂) Quaternion, from platform to earth coordinates, at the time tₖ₋₂
/// @param[in] suppressCoriolis If true, don't calculate the coriolis acceleration
/// @return The updated velocity v_e
///
/// @note See C. Jekeli (2001) - Inertial Navigation Systems with Geodetic Applications (Chapter 4.2.4.1.2)
///       See T. Hobiger - Inertialnavigation (Lecture Slides Chapter 8)
[[nodiscard]] Eigen::Vector3d updateVelocity_e_Simpson(const long double& timeDifferenceSec__t0,
                                                       const long double& timeDifferenceSec__t1,
                                                       const Eigen::Vector3d& acceleration_p__t0,
                                                       const Eigen::Vector3d& acceleration_p__t1,
                                                       const Eigen::Vector3d& velocity_e__t2,
                                                       const Eigen::Vector3d& position_e__t2,
                                                       const Eigen::Vector3d& gravity_e,
                                                       const Eigen::Quaterniond& quaternion_ep__t0,
                                                       const Eigen::Quaterniond& quaternion_ep__t1,
                                                       const Eigen::Quaterniond& quaternion_ep__t2,
                                                       bool suppressCoriolis);

/// @brief Integrates the accelerations and calculates the new velocity v_n with Simpson rule
/// @param[in] timeDifferenceSec__t0 Δtₖ Time difference in [seconds]. This epoch to previous epoch
/// @param[in] timeDifferenceSec__t1 Δtₖ₋₁ Time difference in [seconds]. Previous epoch to twice previous epoch
/// @param[in] acceleration_b__t0 a_p (tₖ) Acceleration in [m/s^2], in body coordinates, at the time tₖ
/// @param[in] acceleration_b__t1 a_p (tₖ₋₁) Acceleration in [m/s^2], in body coordinates, at the time tₖ₋₁
/// @param[in] velocity_n__t1 v_n (tₖ₋₁) Velocity in [m/s], in navigation coordinates, at the time tₖ₋₁
/// @param[in] velocity_n__t2 v_n (tₖ₋₂) Velocity in [m/s], in navigation coordinates, at the time tₖ₋₂
/// @param[in] gravity_n__t1 g_n (tₖ₋₁) Gravity vector in [m/s^2], in navigation coordinates, at the time tₖ₋₁
/// @param[in] angularVelocity_ie_n__t1 ω_ie_n (tₖ₋₁) Nominal mean angular velocity of the Earth in [rad/s], in navigation coordinates, at the time tₖ₋₁
/// @param[in] angularVelocity_en_n__t1 ω_ie_n (tₖ₋₁) Transport Rate in [rad/s], in navigation coordinates, at the time tₖ₋₁
/// @param[in] quaternion_nb__t0 q (tₖ) Quaternion, from body to navigation coordinates, at the time tₖ
/// @param[in] quaternion_nb__t1 q (tₖ₋₁) Quaternion, from body to navigation coordinates, at the time tₖ₋₁
/// @param[in] quaternion_nb__t2 q (tₖ₋₂) Quaternion, from body to navigation coordinates, at the time tₖ₋₂
/// @param[in] suppressCoriolis If true, don't calculate the coriolis acceleration
/// @return The updated velocity v_n
///
/// @note See S. Gleason (2009) - GNSS Applications and Methods (Chapter 6.2.3.2)
[[nodiscard]] Eigen::Vector3d updateVelocity_n_Simpson(const long double& timeDifferenceSec__t0,
                                                       const long double& timeDifferenceSec__t1,
                                                       const Eigen::Vector3d& acceleration_b__t0,
                                                       const Eigen::Vector3d& acceleration_b__t1,
                                                       const Eigen::Vector3d& velocity_n__t1,
                                                       const Eigen::Vector3d& velocity_n__t2,
                                                       const Eigen::Vector3d& gravity_n__t1,
                                                       const Eigen::Vector3d& angularVelocity_ie_n__t1,
                                                       const Eigen::Vector3d& angularVelocity_en_n__t1,
                                                       const Eigen::Quaterniond& quaternion_nb__t0,
                                                       const Eigen::Quaterniond& quaternion_nb__t1,
                                                       const Eigen::Quaterniond& quaternion_nb__t2,
                                                       bool suppressCoriolis);

/// @brief Integrates the accelerations and calculates the new velocity v_n with Runge Kutta of 3rd Order
/// @param[in] timeDifferenceSec__t0 Δtₖ Time difference in [seconds]. This epoch to previous epoch
/// @param[in] timeDifferenceSec__t1 Δtₖ₋₁ Time difference in [seconds]. Previous epoch to twice previous epoch
/// @param[in] acceleration_b__t0 a_p (tₖ) Acceleration in [m/s^2], in body coordinates, at the time tₖ
/// @param[in] acceleration_b__t1 a_p (tₖ₋₁) Acceleration in [m/s^2], in body coordinates, at the time tₖ₋₁
/// @param[in] velocity_n__t2 v_n (tₖ₋₂) Velocity in [m/s], in navigation coordinates, at the time tₖ₋₂
/// @param[in] gravity_n__t1 g_n (tₖ₋₁) Gravity vector in [m/s^2], in navigation coordinates, at the time tₖ₋₁
/// @param[in] angularVelocity_ie_n__t1 ω_ie_n (tₖ₋₁) Nominal mean angular velocity of the Earth in [rad/s], in navigation coordinates, at the time tₖ₋₁
/// @param[in] angularVelocity_en_n__t1 ω_ie_n (tₖ₋₁) Transport Rate in [rad/s], in navigation coordinates, at the time tₖ₋₁
/// @param[in] quaternion_nb__t0 q (tₖ) Quaternion, from body to navigation coordinates, at the time tₖ
/// @param[in] quaternion_nb__t1 q (tₖ₋₁) Quaternion, from body to navigation coordinates, at the time tₖ₋₁
/// @param[in] quaternion_nb__t2 q (tₖ₋₂) Quaternion, from body to navigation coordinates, at the time tₖ₋₂
/// @param[in] suppressCoriolis If true, don't calculate the coriolis acceleration
/// @return The updated velocity v_n
///
/// @note Prefer using Simpson for the velocity update as yields same result but computational more efficient (Jekeli (2001) - Chapter 4.3.6)
/// @note See C. Jekeli (2001) - Inertial Navigation Systems with Geodetic Applications (Chapter 4.3.4)
[[nodiscard]] Eigen::Vector3d updateVelocity_n_RungeKutta3(const long double& timeDifferenceSec__t0,
                                                           const long double& timeDifferenceSec__t1,
                                                           const Eigen::Vector3d& acceleration_b__t0,
                                                           const Eigen::Vector3d& acceleration_b__t1,
                                                           const Eigen::Vector3d& velocity_n__t2,
                                                           const Eigen::Vector3d& gravity_n__t1,
                                                           const Eigen::Vector3d& angularVelocity_ie_n__t1,
                                                           const Eigen::Vector3d& angularVelocity_en_n__t1,
                                                           const Eigen::Quaterniond& quaternion_nb__t0,
                                                           const Eigen::Quaterniond& quaternion_nb__t1,
                                                           const Eigen::Quaterniond& quaternion_nb__t2,
                                                           bool suppressCoriolis);

// ###########################################################################################################
//                                              Position Update
// ###########################################################################################################

/// @brief Calculates the new position x_e in earth frame
/// @param[in] timeDifferenceSec__t0 Δtₖ Time difference in [seconds]. This epoch to previous epoch
/// @param[in] position_e__t1 x_e (tₖ₋₁) Position in [m/s], in earth coordinates, at the time tₖ₋₁
/// @param[in] velocity_e__t1 v_e (tₖ₋₁) Velocity in [m/s], in earth coordinates, at the time tₖ₋₁
/// @return x_e (tₖ) Position in [m/s], in earth coordinates, at the time tₖ
///
/// @note See T. Hobiger - Inertialnavigation (Lecture Slides Chapter 9)
[[nodiscard]] Eigen::Vector3d updatePosition_e(const long double& timeDifferenceSec__t0,
                                               const Eigen::Vector3d& position_e__t1,
                                               const Eigen::Vector3d& velocity_e__t1);

/// @brief Calculates the new position [𝜙, λ, h]
/// @param[in] timeDifferenceSec__t0 Δtₖ Time difference in [seconds]. This epoch to previous epoch
/// @param[in] latLonAlt__t1 [𝜙, λ, h] (tₖ₋₁) Latitude, Longitude and altitude in [rad, rad, m] at the time tₖ₋₁
/// @param[in] velocity_n__t1 v_n (tₖ₋₁) Velocity in [m/s], in navigation coordinates, at the time tₖ₋₁
/// @param[in] R_N North/South (meridian) earth radius [m]
/// @param[in] R_E East/West (prime vertical) earth radius [m]
/// @return [𝜙, λ, h] (tₖ) Latitude, Longitude and altitude in [rad, rad, m] at the time tₖ
///
/// @note See S. Gleason (2009) - GNSS Applications and Methods (Chapter 6.2.3.3)
/// @note See P. Groves (2013) - Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems (eq. 2.111, p. 61)
[[nodiscard]] Eigen::Vector3d updatePosition_lla(const long double& timeDifferenceSec__t0,
                                                 const Eigen::Vector3d& latLonAlt__t1,
                                                 const Eigen::Vector3d& velocity_n__t1,
                                                 const double& R_N,
                                                 const double& R_E);

// ###########################################################################################################
//                                             Earth Parameters
// ###########################################################################################################

/// @brief Corrects the provided Position, Velocity and Attitude with the corrections
/// @param[in] posVelAtt PosVelAtt to correct
/// @param[in] pvaError Corrections to apply
/// @return Newly allocated pointer to the corrected posVelAtt
std::shared_ptr<const NAV::PosVelAtt> correctPosVelAtt(const std::shared_ptr<const NAV::PosVelAtt>& posVelAtt, const std::shared_ptr<const NAV::PVAError>& pvaError);

} // namespace NAV
