#include "InsMechanization.hpp"

#include "InsConstants.hpp"

#include "util/Logger.hpp"

#include <util/NumericalIntegration.hpp>

namespace NAV
{
// ###########################################################################################################
//                                             Private Functions
// ###########################################################################################################

Eigen::Vector4d quaternionUpdateModel(const Eigen::Vector3d& angularVelocity, const Eigen::Vector4d& quat)
{
    // Rearranged because Skript Quaternion has order (w,x,y,z)
    Eigen::Vector4d q = { quat(3), quat(0), quat(1), quat(2) };
    // clang-format off

    /// A Matrix at the time tₖ (Titterton p 319 e.q. 11.35)
    Eigen::Matrix4d A;
    A <<         0.0        , -angularVelocity(0), -angularVelocity(1), -angularVelocity(2),
          angularVelocity(0),         0.0        ,  angularVelocity(2), -angularVelocity(1),
          angularVelocity(1), -angularVelocity(2),         0.0        ,  angularVelocity(0),
          angularVelocity(2),  angularVelocity(1), -angularVelocity(0),         0.0        ;
    // clang-format on

    q = 0.5 * A * q; // (w,x,y,z)

    // Rearranged because Eigen::Quaternion has order (x,y,z,w)
    return { q(1), q(2), q(3), q(0) };
}

struct VelocityUpdateState
{
    /// a_n Taylor-Approximation of acceleration in [m/s^2]
    Eigen::Vector3d accel_n;
    /// ω_ie_n Nominal mean angular velocity of the Earth in [rad/s], in navigation coordinates
    Eigen::Vector3d angularVelocity_ie_n;
    /// ω_ie_n Transport Rate in [rad/s], in navigation coordinates
    Eigen::Vector3d angularVelocity_en_n;
    /// g_n (tₖ₋₁) Gravity vector in [m/s^2], in navigation coordinates (including centrifugal acceleration)
    Eigen::Vector3d gravity_n;
};

/// @note See C. Jekeli (2001) - Inertial Navigation Systems with Geodetic Applications (Chapter 4.3.4)
Eigen::Vector3d velocityUpdateModel(const VelocityUpdateState& x, const Eigen::Vector3d& velocity_n)
{
    // TODO: Consider suppressCoriolis option from the GUI here
    // The Coriolis force accounts for the fact that the NED frame is noninertial
    const Eigen::Vector3d coriolisAcceleration_n = (2 * x.angularVelocity_ie_n + x.angularVelocity_en_n).cross(velocity_n);

    // Jekeli (eq. 4.88) - g includes centrifugal acceleration
    return x.accel_n - coriolisAcceleration_n + x.gravity_n;
}

// ###########################################################################################################
//                                             Public Functions
// ###########################################################################################################

// ###########################################################################################################
//                                              Attitude Update
// ###########################################################################################################

Eigen::Quaterniond updateQuaternion_nb_RungeKutta1(const long double& timeDifferenceSec__t0,
                                                   const Eigen::Vector3d& angularVelocity_ip_b__t0,
                                                   const Eigen::Vector3d& angularVelocity_ie_n__t1,
                                                   const Eigen::Vector3d& angularVelocity_en_n__t1,
                                                   const Eigen::Quaterniond& quaternion_nb__t1)
{
    /// q (tₖ₋₁) Quaternion, from n-system to b-system, at the time tₖ₋₁
    const Eigen::Quaterniond quaternion_bn__t1 = quaternion_nb__t1.conjugate();

    /// Δβ⁠_nb_p (tₖ) The angular velocities in [rad],
    /// of the navigation to body system, in body coordinates, at the time tₖ (eq. 8.9)
    const Eigen::Vector3d angularVelocity_nb_b__t0 = angularVelocity_ip_b__t0
                                                     - quaternion_bn__t1 * (angularVelocity_ie_n__t1 + angularVelocity_en_n__t1);

    // Updated Quaternion (eq. 8.2)
    Eigen::Quaterniond q_nb__t0;
    q_nb__t0 = Integration::rungeKutta1(quaternionUpdateModel, timeDifferenceSec__t0, quaternion_nb__t1.coeffs(), angularVelocity_nb_b__t0);

    // Normalize Quaternion
    q_nb__t0.normalize();

    return q_nb__t0;
}

Eigen::Quaterniond updateQuaternion_ep_RungeKutta3(
    const long double& timeDifferenceSec__t0,        // Δtₖ = (tₖ - tₖ₋₁) Time difference in [seconds]
    const long double& timeDifferenceSec__t1,        // Δtₖ₋₁ = (tₖ₋₁ - tₖ₋₂) Time difference in [seconds]
    const Eigen::Vector3d& angularVelocity_ip_p__t0, // ω_ip_p (tₖ) Angular velocity in [rad/s], of the inertial to platform system, in platform coordinates, at the time tₖ
    const Eigen::Vector3d& angularVelocity_ip_p__t1, // ω_ip_p (tₖ₋₁) Angular velocity in [rad/s], of the inertial to platform system, in platform coordinates, at the time tₖ₋₁
    const Eigen::Vector3d& angularVelocity_ie_e__t0, // ω_ie_e (tₖ) Angular velocity in [rad/s], of the inertial to earth system, in earth coordinates, at the time tₖ
    const Eigen::Quaterniond& quaternion_ep__t1,     // q (tₖ₋₁) Quaternion, from platform to earth coordinates, at the time tₖ₋₁
    const Eigen::Quaterniond& quaternion_ep__t2)     // q (tₖ₋₂) Quaternion, from platform to earth coordinates, at the time tₖ₋₂
{
    /// q (tₖ₋₂) Quaternion, from earth to platform coordinates, at the time tₖ₋₂
    const Eigen::Quaterniond quaternion_pe__t2 = quaternion_ep__t2.conjugate();
    /// q (tₖ₋₁) Quaternion, from earth to platform coordinates, at the time tₖ₋₁
    const Eigen::Quaterniond quaternion_pe__t1 = quaternion_ep__t1.conjugate();

    /// Δα_ip_p (tₖ₋₁) The integrated angular velocities in [radian],
    /// of the inertial to platform system, in platform coordinates, at the time tₖ₋₁ (eq. 8.4)
    const Eigen::Vector3d integratedAngularVelocity_ip_p__t1 = timeDifferenceSec__t1 * angularVelocity_ip_p__t1;
    /// Δα_ip_p (tₖ) The integrated angular velocities in [radian],
    /// of the inertial to platform system, in platform coordinates, at the time tₖ (eq. 8.4)
    const Eigen::Vector3d integratedAngularVelocity_ip_p__t0 = timeDifferenceSec__t0 * angularVelocity_ip_p__t0;
    /// Δβ⁠_ep_p (tₖ₋₁) The integrated angular velocities in [radian],
    /// of the earth to platform system, in platform coordinates, at the time tₖ₋₁ (eq. 8.9)
    const Eigen::Vector3d integratedAngularVelocity_ep_p__t1 = integratedAngularVelocity_ip_p__t1
                                                               - quaternion_pe__t2 * angularVelocity_ie_e__t0 * timeDifferenceSec__t1;
    /// Δβ⁠_ep_p (tₖ) The integrated angular velocities in [radian],
    /// of the earth to platform system, in platform coordinates, at the time tₖ (eq. 8.9)
    const Eigen::Vector3d integratedAngularVelocity_ep_p__t0 = integratedAngularVelocity_ip_p__t0
                                                               - quaternion_pe__t1 * angularVelocity_ie_e__t0 * timeDifferenceSec__t0;

    /// Runge-Kutta integration step [s]
    const long double integrationStep = timeDifferenceSec__t0 + timeDifferenceSec__t1;

    /// ῶ_ep_p (tₖ₋₂) Taylor-Approximation of angular velocities in [rad/s],
    /// of the earth to platform system, in platform coordinates, at the time tₖ₋₂ (eq. 8.15)
    const Eigen::Vector3d angularVelocity_ep_p__t2 = (3 * integratedAngularVelocity_ep_p__t1 - integratedAngularVelocity_ep_p__t0) / integrationStep;
    /// ῶ_ep_p (tₖ₋₁) Taylor-Approximation of angular velocities in [rad/s],
    /// of the earth to platform system, in platform coordinates, at the time tₖ₋₁ (eq. 8.15)
    const Eigen::Vector3d angularVelocity_ep_p__t1 = (integratedAngularVelocity_ep_p__t1 + integratedAngularVelocity_ep_p__t0) / integrationStep;
    /// ῶ_ep_p (tₖ) Taylor-Approximation of angular velocities in [rad/s],
    /// of the earth to platform system, in platform coordinates, at the time tₖ (eq. 8.15)
    const Eigen::Vector3d angularVelocity_ep_p__t0 = (3 * integratedAngularVelocity_ep_p__t0 - integratedAngularVelocity_ep_p__t1) / integrationStep;

    // Updated Quaternion (eq. 8.2)
    Eigen::Quaterniond q_ep__t0;
    q_ep__t0 = Integration::rungeKutta3(quaternionUpdateModel, integrationStep, quaternion_ep__t2.coeffs(),
                                        angularVelocity_ep_p__t2, angularVelocity_ep_p__t1, angularVelocity_ep_p__t0);

    // Normalize Quaternion
    q_ep__t0.normalize();

    return q_ep__t0;
}

Eigen::Quaterniond updateQuaternion_nb_RungeKutta3(
    const long double& timeDifferenceSec__t0,        // Δtₖ = (tₖ - tₖ₋₁) Time difference in [seconds]
    const long double& timeDifferenceSec__t1,        // Δtₖ₋₁ = (tₖ₋₁ - tₖ₋₂) Time difference in [seconds]
    const Eigen::Vector3d& angularVelocity_ip_b__t0, // ω_ip_b (tₖ) Angular velocity in [rad/s], of the inertial to platform system, in body coordinates, at the time tₖ
    const Eigen::Vector3d& angularVelocity_ip_b__t1, // ω_ip_b (tₖ₋₁) Angular velocity in [rad/s], of the inertial to platform system, in body coordinates, at the time tₖ₋₁
    const Eigen::Vector3d& angularVelocity_ie_n__t1, // ω_ie_n (tₖ₋₁) Angular velocity in [rad/s], of the inertial to earth system, in navigation coordinates, at the time tₖ₋₁
    const Eigen::Vector3d& angularVelocity_en_n__t1, // ω_en_n (tₖ₋₁) Transport Rate, rotation rate of the Earth frame relative to the navigation frame, in navigation coordinates
    const Eigen::Quaterniond& quaternion_nb__t1,     // q (tₖ₋₁) Quaternion, from body to navigation coordinates, at the time tₖ₋₁
    const Eigen::Quaterniond& quaternion_nb__t2)     // q (tₖ₋₂) Quaternion, from body to navigation coordinates, at the time tₖ₋₂
{
    /// q (tₖ₋₂) Quaternion, from earth to platform coordinates, at the time tₖ₋₂
    const Eigen::Quaterniond quaternion_bn__t2 = quaternion_nb__t2.conjugate();
    /// q (tₖ₋₁) Quaternion, from earth to platform coordinates, at the time tₖ₋₁
    const Eigen::Quaterniond quaternion_bn__t1 = quaternion_nb__t1.conjugate();

    /// Δα_ip_p (tₖ₋₁) The integrated angular velocities in [radian],
    /// of the inertial to platform system, in body coordinates, at the time tₖ₋₁ (eq. 8.4)
    const Eigen::Vector3d integratedAngularVelocity_ip_b__t1 = timeDifferenceSec__t1 * angularVelocity_ip_b__t1;
    /// Δα_ip_p (tₖ) The integrated angular velocities in [radian],
    /// of the inertial to platform system, in body coordinates, at the time tₖ (eq. 8.4)
    const Eigen::Vector3d integratedAngularVelocity_ip_b__t0 = timeDifferenceSec__t0 * angularVelocity_ip_b__t0;
    /// Δβ⁠_nb_p (tₖ₋₁) The integrated angular velocities in [radian],
    /// of the navigation to body system, in body coordinates, at the time tₖ₋₁ (eq. 8.9)
    const Eigen::Vector3d integratedAngularVelocity_nb_b__t1 = integratedAngularVelocity_ip_b__t1
                                                               - quaternion_bn__t2 * (angularVelocity_ie_n__t1 + angularVelocity_en_n__t1) * timeDifferenceSec__t1;
    /// Δβ⁠_nb_p (tₖ) The integrated angular velocities in [radian],
    /// of the navigation to body system, in body coordinates, at the time tₖ (eq. 8.9)
    const Eigen::Vector3d integratedAngularVelocity_nb_b__t0 = integratedAngularVelocity_ip_b__t0
                                                               - quaternion_bn__t1 * (angularVelocity_ie_n__t1 + angularVelocity_en_n__t1) * timeDifferenceSec__t0;

    /// Runge-Kutta integration step [s]
    const long double integrationStep = timeDifferenceSec__t0 + timeDifferenceSec__t1;

    /// ῶ_nb_b (tₖ₋₂) Taylor-Approximation of angular velocities in [rad/s],
    /// of the navigation to body system, in body coordinates, at the time tₖ₋₂ (eq. 8.15)
    const Eigen::Vector3d angularVelocity_nb_b__t2 = (3 * integratedAngularVelocity_nb_b__t1 - integratedAngularVelocity_nb_b__t0) / integrationStep;
    /// ῶ_nb_b (tₖ₋₁) Taylor-Approximation of angular velocities in [rad/s],
    /// of the navigation to body system, in body coordinates, at the time tₖ₋₁ (eq. 8.15)
    const Eigen::Vector3d angularVelocity_nb_b__t1 = (integratedAngularVelocity_nb_b__t1 + integratedAngularVelocity_nb_b__t0) / integrationStep;
    /// ῶ_nb_b (tₖ) Taylor-Approximation of angular velocities in [rad/s],
    /// of the navigation to body system, in body coordinates, at the time tₖ (eq. 8.15)
    const Eigen::Vector3d angularVelocity_nb_b__t0 = (3 * integratedAngularVelocity_nb_b__t0 - integratedAngularVelocity_nb_b__t1) / integrationStep;

    // Updated Quaternion (eq. 8.2)
    Eigen::Quaterniond q_nb__t0;
    q_nb__t0 = Integration::rungeKutta3(quaternionUpdateModel, integrationStep, quaternion_nb__t2.coeffs(),
                                        angularVelocity_nb_b__t2, angularVelocity_nb_b__t1, angularVelocity_nb_b__t0);

    // Normalize Quaternion
    q_nb__t0.normalize();

    return q_nb__t0;
}

// ###########################################################################################################
//                                              Velocity Update
// ###########################################################################################################

Eigen::Vector3d updateVelocity_n_RungeKutta1(const long double& timeDifferenceSec__t0,
                                             const Eigen::Vector3d& acceleration_b__t0,
                                             const Eigen::Vector3d& velocity_n__t1,
                                             const Eigen::Vector3d& gravity_n__t1,
                                             const Eigen::Vector3d& angularVelocity_ie_n__t1,
                                             const Eigen::Vector3d& angularVelocity_en_n__t1,
                                             const Eigen::Quaterniond& quaternion_nb__t1)
{
    VelocityUpdateState state___t1;

    /// The derivative of velocity (see Jekeli (2001), eq. 4.88)
    state___t1.accel_n = quaternion_nb__t1 * acceleration_b__t0;

    state___t1.angularVelocity_ie_n = angularVelocity_ie_n__t1;
    state___t1.angularVelocity_en_n = angularVelocity_en_n__t1;
    state___t1.gravity_n = gravity_n__t1;

    /// v_n (tₖ) Velocity in [m/s], in navigation coordinates, at the time tₖ
    Eigen::Vector3d velocity_n__t0 = Integration::rungeKutta1(velocityUpdateModel, timeDifferenceSec__t0, velocity_n__t1, state___t1);

    return velocity_n__t0;
}

Eigen::Vector3d updateVelocity_e_Simpson(const long double& timeDifferenceSec__t0,    // Δtₖ Time difference in [seconds]. This epoch to previous epoch
                                         const long double& timeDifferenceSec__t1,    // Δtₖ₋₁ Time difference in [seconds]. Previous epoch to twice previous epoch
                                         const Eigen::Vector3d& acceleration_p__t0,   // a_p (tₖ) Acceleration in [m/s^2], in platform coordinates, at the time tₖ
                                         const Eigen::Vector3d& acceleration_p__t1,   // a_p (tₖ₋₁) Acceleration in [m/s^2], in platform coordinates, at the time tₖ₋₁
                                         const Eigen::Vector3d& velocity_e__t2,       // v_e (tₖ₋₂) Velocity in [m/s], in earth coordinates, at the time tₖ₋₂
                                         const Eigen::Vector3d& position_e__t2,       // x_e (tₖ₋₂) Position in [m/s], in earth coordinates, at the time tₖ₋₂
                                         const Eigen::Vector3d& gravity_e,            // g_e Gravity vector in [m/s^2], in earth coordinates
                                         const Eigen::Quaterniond& quaternion_ep__t0, // q (tₖ) Quaternion, from platform to earth coordinates, at the time tₖ
                                         const Eigen::Quaterniond& quaternion_ep__t1, // q (tₖ₋₁) Quaternion, from platform to earth coordinates, at the time tₖ₋₁
                                         const Eigen::Quaterniond& quaternion_ep__t2, // q (tₖ₋₂) Quaternion, from platform to earth coordinates, at the time tₖ₋₂
                                         bool suppressCoriolis                        // Toggles deactivation of coriolis acceleration
)
{
    /// Δv_p (tₖ) Integrated velocity in [m/s], in platform coordinates, at the time tₖ (eq. 9.3)
    const Eigen::Vector3d deltaVelocity_p__t0 = acceleration_p__t0 * timeDifferenceSec__t0;

    /// Δv_p (tₖ₋₁) Integrated velocity in [m/s], in platform coordinates, at the time tₖ₋₁ (eq. 9.3)
    const Eigen::Vector3d deltaVelocity_p__t1 = acceleration_p__t1 * timeDifferenceSec__t1;

    /// Integration step [s]
    const long double integrationStep = timeDifferenceSec__t0 + timeDifferenceSec__t1;

    /// Integration of delta velocities (eq. 9.12)
    const Eigen::Vector3d simpsonIntegration_e = (quaternion_ep__t2 * (3 * deltaVelocity_p__t1 - deltaVelocity_p__t0)
                                                  + 4 * (quaternion_ep__t1 * (deltaVelocity_p__t1 + deltaVelocity_p__t0))
                                                  + quaternion_ep__t0 * (3 * deltaVelocity_p__t0 - deltaVelocity_p__t1))
                                                 / 6.0;

    Eigen::Vector3d coriolisAcceleration_e;

    if (suppressCoriolis)
    {
        LOG_DATA("Coriolis acceleration is set to zero");
        coriolisAcceleration_e << 0.0, 0.0, 0.0;
    }
    else
    {
        /// The Coriolis force accounts for the fact that the NED frame is noninertial
        coriolisAcceleration_e = 2 * InsConst::angularVelocityCrossProduct_ie_e * velocity_e__t2
                                 + InsConst::angularVelocityCrossProduct_ie_e * InsConst::angularVelocityCrossProduct_ie_e * position_e__t2;
    }

    /// v_e (tₖ) Velocity in [m/s], in earth coordinates, at the time tₖ (eq. 9.12)
    Eigen::Vector3d velocity_e__t0 = velocity_e__t2 + simpsonIntegration_e - (coriolisAcceleration_e - gravity_e) * integrationStep;

    return velocity_e__t0;
}

Eigen::Vector3d updateVelocity_n_Simpson(const long double& timeDifferenceSec__t0,        // Δtₖ Time difference in [seconds]. This epoch to previous epoch
                                         const long double& timeDifferenceSec__t1,        // Δtₖ₋₁ Time difference in [seconds]. Previous epoch to twice previous epoch
                                         const Eigen::Vector3d& acceleration_b__t0,       // a_p (tₖ) Acceleration in [m/s^2], in body coordinates, at the time tₖ
                                         const Eigen::Vector3d& acceleration_b__t1,       // a_p (tₖ₋₁) Acceleration in [m/s^2], in body coordinates, at the time tₖ₋₁
                                         const Eigen::Vector3d& velocity_n__t1,           // v_n (tₖ₋₁) Velocity in [m/s], in navigation coordinates, at the time tₖ₋₁
                                         const Eigen::Vector3d& velocity_n__t2,           // v_n (tₖ₋₂) Velocity in [m/s], in navigation coordinates, at the time tₖ₋₂
                                         const Eigen::Vector3d& gravity_n__t1,            // g_n (tₖ₋₁) Gravity vector in [m/s^2], in navigation coordinates, at the time tₖ₋₁
                                         const Eigen::Vector3d& angularVelocity_ie_n__t1, // ω_ie_n (tₖ₋₁) Nominal mean angular velocity of the Earth in [rad/s], in navigation coordinates, at the time tₖ₋₁
                                         const Eigen::Vector3d& angularVelocity_en_n__t1, // ω_en_n (tₖ₋₁) Transport Rate in [rad/s], in navigation coordinates, at the time tₖ₋₁
                                         const Eigen::Quaterniond& quaternion_nb__t0,     // q (tₖ) Quaternion, from body to navigation coordinates, at the time tₖ
                                         const Eigen::Quaterniond& quaternion_nb__t1,     // q (tₖ₋₁) Quaternion, from body to navigation coordinates, at the time tₖ₋₁
                                         const Eigen::Quaterniond& quaternion_nb__t2,     // q (tₖ₋₂) Quaternion, from body to navigation coordinates, at the time tₖ₋₂
                                         bool suppressCoriolis                            // Toggles deactivation of coriolis acceleration
)
{
    /// Δv_p (tₖ) Integrated velocity in [m/s], in body coordinates, at the time tₖ
    const Eigen::Vector3d deltaVelocity_b__t0 = acceleration_b__t0 * timeDifferenceSec__t0;

    /// Δv_p (tₖ₋₁) Integrated velocity in [m/s], in body coordinates, at the time tₖ₋₁
    const Eigen::Vector3d deltaVelocity_b__t1 = acceleration_b__t1 * timeDifferenceSec__t1;

    /// Integration step [s]
    const long double integrationStep = timeDifferenceSec__t0 + timeDifferenceSec__t1;

    /// Integration of delta velocities
    const Eigen::Vector3d simpsonIntegration_n = (quaternion_nb__t2 * (3 * deltaVelocity_b__t1 - deltaVelocity_b__t0)
                                                  + 4 * (quaternion_nb__t1 * (deltaVelocity_b__t1 + deltaVelocity_b__t0))
                                                  + quaternion_nb__t0 * (3 * deltaVelocity_b__t0 - deltaVelocity_b__t1))
                                                 / 6.0;

    /// The Coriolis force accounts for the fact that the NED frame is noninertial
    Eigen::Vector3d coriolisAcceleration_n__t1;

    if (suppressCoriolis)
    {
        coriolisAcceleration_n__t1 << 0.0, 0.0, 0.0;
    }
    else
    {
        /// The Coriolis force accounts for the fact that the NED frame is noninertial
        coriolisAcceleration_n__t1 = (2 * angularVelocity_ie_n__t1 + angularVelocity_en_n__t1).cross(velocity_n__t1);
    }

    /// v_e (tₖ) Velocity in [m/s], in navigation coordinates, at the time tₖ (eq. 6.13)
    Eigen::Vector3d velocity_n__t0 = velocity_n__t2 + simpsonIntegration_n - (coriolisAcceleration_n__t1 - gravity_n__t1) * integrationStep;

    return velocity_n__t0;
}

Eigen::Vector3d updateVelocity_n_RungeKutta3(const long double& timeDifferenceSec__t0,        // Δtₖ Time difference in [seconds]. This epoch to previous epoch
                                             const long double& timeDifferenceSec__t1,        // Δtₖ₋₁ Time difference in [seconds]. Previous epoch to twice previous epoch
                                             const Eigen::Vector3d& acceleration_b__t0,       // a_p (tₖ) Acceleration in [m/s^2], in body coordinates, at the time tₖ
                                             const Eigen::Vector3d& acceleration_b__t1,       // a_p (tₖ₋₁) Acceleration in [m/s^2], in body coordinates, at the time tₖ₋₁
                                             const Eigen::Vector3d& velocity_n__t2,           // v_n (tₖ₋₂) Velocity in [m/s], in navigation coordinates, at the time tₖ₋₂
                                             const Eigen::Vector3d& gravity_n__t1,            // g_n (tₖ₋₁) Gravity vector in [m/s^2], in navigation coordinates, at the time tₖ₋₁
                                             const Eigen::Vector3d& angularVelocity_ie_n__t1, // ω_ie_n (tₖ₋₁) Nominal mean angular velocity of the Earth in [rad/s], in navigation coordinates, at the time tₖ₋₁
                                             const Eigen::Vector3d& angularVelocity_en_n__t1, // ω_ie_n (tₖ₋₁) Transport Rate in [rad/s], in navigation coordinates, at the time tₖ₋₁
                                             const Eigen::Quaterniond& quaternion_nb__t0,     // q (tₖ) Quaternion, from body to navigation coordinates, at the time tₖ
                                             const Eigen::Quaterniond& quaternion_nb__t1,     // q (tₖ₋₁) Quaternion, from body to navigation coordinates, at the time tₖ₋₁
                                             const Eigen::Quaterniond& quaternion_nb__t2,     // q (tₖ₋₂) Quaternion, from body to navigation coordinates, at the time tₖ₋₂
                                             bool suppressCoriolis                            // Toggles deactivation of coriolis acceleration
)
{
    /// Δv_p (tₖ) Integrated velocity in [m/s], in body coordinates, at the time tₖ
    const Eigen::Vector3d deltaVelocity_b__t0 = acceleration_b__t0 * timeDifferenceSec__t0;

    /// Δv_p (tₖ₋₁) Integrated velocity in [m/s], in body coordinates, at the time tₖ₋₁
    const Eigen::Vector3d deltaVelocity_b__t1 = acceleration_b__t1 * timeDifferenceSec__t1;

    /// Integration step [s]
    const long double integrationStep = timeDifferenceSec__t0 + timeDifferenceSec__t1;

    VelocityUpdateState state__t2;
    VelocityUpdateState state__t1;
    VelocityUpdateState state__t0;

    /// a_n (tₖ₋₂) Taylor-Approximation of acceleration in [m/s^2]
    state__t2.accel_n = quaternion_nb__t2 * (3 * deltaVelocity_b__t1 - deltaVelocity_b__t0) / integrationStep;
    /// a_n (tₖ₋₁) Taylor-Approximation of acceleration in [m/s^2]
    state__t1.accel_n = quaternion_nb__t1 * (deltaVelocity_b__t1 + deltaVelocity_b__t0) / integrationStep;
    /// a_n (tₖ) Taylor-Approximation of acceleration in [m/s^2]
    state__t0.accel_n = quaternion_nb__t0 * (3 * deltaVelocity_b__t0 - deltaVelocity_b__t1) / integrationStep;

    if (suppressCoriolis)
    {
        state__t0.angularVelocity_ie_n = Eigen::Vector3d::Zero();
        state__t0.angularVelocity_en_n = Eigen::Vector3d::Zero();
    }
    else
    {
        state__t0.angularVelocity_ie_n = angularVelocity_ie_n__t1;
        state__t0.angularVelocity_en_n = angularVelocity_en_n__t1;
    }

    state__t0.gravity_n = gravity_n__t1;

    state__t1.angularVelocity_ie_n = state__t0.angularVelocity_ie_n;
    state__t1.angularVelocity_en_n = state__t0.angularVelocity_en_n;
    state__t1.gravity_n = state__t0.gravity_n;

    state__t2.angularVelocity_ie_n = state__t0.angularVelocity_ie_n;
    state__t2.angularVelocity_en_n = state__t0.angularVelocity_en_n;
    state__t2.gravity_n = state__t0.gravity_n;

    /// v_n (tₖ) Velocity in [m/s], in navigation coordinates, at the time tₖ
    Eigen::Vector3d velocity_n__t0 = Integration::rungeKutta3(velocityUpdateModel, integrationStep, velocity_n__t2, state__t2, state__t1, state__t0);

    return velocity_n__t0;
}

// ###########################################################################################################
//                                              Position Update
// ###########################################################################################################

Eigen::Vector3d updatePosition_e(const long double& timeDifferenceSec__t0, // Δtₖ Time difference in [seconds]. This epoch to previous epoch
                                 const Eigen::Vector3d& position_e__t1,    // x_e (tₖ₋₁) Position in [m/s], in earth coordinates, at the time tₖ₋₁
                                 const Eigen::Vector3d& velocity_e__t1)    // v_e (tₖ₋₁) Velocity in [m/s], in earth coordinates, at the time tₖ₋₁
{
    /// x_e (tₖ) Position in [m/s], in earth coordinates, at the time tₖ
    Eigen::Vector3d position_e__t0 = position_e__t1 + velocity_e__t1 * timeDifferenceSec__t0;

    return position_e__t0;
}

Eigen::Vector3d updatePosition_lla(const long double& timeDifferenceSec__t0, // Δtₖ Time difference in [seconds]. This epoch to previous epoch
                                   const Eigen::Vector3d& latLonAlt__t1,     // [𝜙, λ, h] (tₖ₋₁) Latitude, Longitude and altitude in [rad, rad, m] at the time tₖ₋₁
                                   const Eigen::Vector3d& velocity_n__t1,    // v_n (tₖ₋₁) Velocity in [m/s], in navigation coordinates, at the time tₖ₋₁
                                   const double& R_N,                        // R_N North/South (meridian) earth radius [m]
                                   const double& R_E)                        // R_E East/West (prime vertical) earth radius [m]
{
    /// 𝜙 Latitude in [rad]
    const auto& latitude = latLonAlt__t1(0);
    /// λ Longitude in [rad]
    const auto& longitude = latLonAlt__t1(1);
    /// h Altitude in [m]
    const auto& altitude = latLonAlt__t1(2);

    /// Δtₖ Time difference in [seconds]
    const auto tau = static_cast<double>(timeDifferenceSec__t0);

    /// Velocity North in [m/s]
    const auto& v_N = velocity_n__t1(0);
    /// Velocity East in [m/s]
    const auto& v_E = velocity_n__t1(1);
    /// Velocity Down in [m/s]
    const auto& v_D = velocity_n__t1(2);

    /// [𝜙, λ, h] (tₖ) Latitude, Longitude and Altitude in [rad, rad, m], at the current time tₖ (see Gleason eq. 6.18 - 6.20)
    Eigen::Vector3d latLonAlt__t0{ latitude + tau * (v_N / (R_N + altitude)),
                                   longitude + tau * (v_E / ((R_E + altitude) * std::cos(latitude))),
                                   altitude - tau * v_D };

    return latLonAlt__t0;
}

Eigen::Vector3d updatePosition_n(const long double& timeDifferenceSec__t0, // Δtₖ Time difference in [seconds]. This epoch to previous epoch
                                 const Eigen::Vector3d& position_n__t1,    // [x_n, x_e, x_d] (tₖ₋₁) Position NED in [m] at the time tₖ₋₁
                                 const Eigen::Vector3d& velocity_n__t1)    // v_n (tₖ₋₁) Velocity in [m/s], in navigation coordinates, at the time tₖ₋₁
{
    auto position_n__t0 = position_n__t1 + velocity_n__t1 * static_cast<double>(timeDifferenceSec__t0);

    return position_n__t0;
}

// ###########################################################################################################
//                                             Earth Parameters
// ###########################################################################################################

double earthRadius_N(const double& latitude, const double& a, const double& e_squared)
{
    double k = std::sqrt(1 - e_squared * std::pow(std::sin(latitude), 2));

    /// North/South (meridian) earth radius [m]
    double R_N = a * (1 - e_squared) / std::pow(k, 3);

    return R_N;
}

double earthRadius_E(const double& latitude, const double& a, const double& e_squared)
{
    /// East/West (prime vertical) earth radius [m]
    double R_E = a / std::sqrt(1 - e_squared * std::pow(std::sin(latitude), 2));

    return R_E;
}

Eigen::Vector3d transportRate(const Eigen::Vector3d& latLonAlt__t1,  // [𝜙, λ, h] (tₖ₋₁) Latitude, Longitude and altitude in [rad, rad, m] at the time tₖ₋₁
                              const Eigen::Vector3d& velocity_n__t1, // v_n (tₖ₋₁) Velocity in [m/s], in navigation coordinates, at the time tₖ₋₁
                              const double& R_N,                     // R_N North/South (meridian) earth radius [m]
                              const double& R_E)                     // R_E East/West (prime vertical) earth radius [m]
{
    /// 𝜙 Latitude in [rad]
    const auto& latitude = latLonAlt__t1(0);
    /// h Altitude in [m]
    const auto& altitude = latLonAlt__t1(2);

    /// Velocity North in [m/s]
    const auto& v_N = velocity_n__t1(0);
    /// Velocity East in [m/s]
    const auto& v_E = velocity_n__t1(1);

    /// ω_en_n (tₖ₋₁) Transport Rate, rotation rate of the Earth frame relative to the navigation frame,
    /// in navigation coordinates see Gleason (eq. 6.15)
    Eigen::Vector3d angularVelocity_en_n__t1;
    angularVelocity_en_n__t1(0) = v_E / (R_E + altitude);
    angularVelocity_en_n__t1(1) = -v_N / (R_N + altitude);
    angularVelocity_en_n__t1(2) = -angularVelocity_en_n__t1(0) * std::tan(latitude);

    return angularVelocity_en_n__t1;
}

Eigen::Matrix3d AngularVelocityEarthSkew_ie_n(double latitude)
{
    // Math: \mathbf{\Omega}_{ie}^{n} = \omega_{ie} \begin{pmatrix} 0 & \sin(L_b) & 0 \\ -\sin(L_b) & 0 & -\cos(L_b) \\ 0 & \cos(L_b) & 0 \end{pmatrix} \qquad \text{P. Groves}\,(5.34)
    Eigen::Matrix3d Omega;
    Omega << 0, std::sin(latitude), 0,
        -std::sin(latitude), 0, -std::cos(latitude),
        0, std::cos(latitude), 0;

    return InsConst::angularVelocity_ie * Omega;
}

} // namespace NAV