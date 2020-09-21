#include "InsMechanization.hpp"

#include "InsConstants.hpp"

namespace NAV
{
Eigen::Quaterniond updateQuaternion_p2e_RungeKutta3(
    const long double& timeDifferenceSec__t0,        // Δtₖ = (tₖ - tₖ₋₁) Time difference in [seconds]
    const long double& timeDifferenceSec__t1,        // Δtₖ₋₁ = (tₖ₋₁ - tₖ₋₂) Time difference in [seconds]
    const Eigen::Vector3d& angularVelocity_ip_p__t0, // ω_ip_p (tₖ) Angluar velocity in [rad/s], of the inertial to platform system, in platform coordinates, at the time tₖ
    const Eigen::Vector3d& angularVelocity_ip_p__t1, // ω_ip_p (tₖ₋₁) Angluar velocity in [rad/s], of the inertial to platform system, in platform coordinates, at the time tₖ₋₁
    const Eigen::Vector3d& angularVelocity_ie_e__t0, // ω_ie_e (tₖ) Angluar velocity in [rad/s], of the inertial to earth system, in earth coordinates, at the time tₖ
    const Eigen::Quaterniond& quaternion_p2e__t1,    // q (tₖ₋₁) Quaternion, from platform to earth coordinates, at the time tₖ₋₁
    const Eigen::Quaterniond& quaternion_p2e__t2)    // q (tₖ₋₂) Quaternion, from platform to earth coordinates, at the time tₖ₋₂
{
    /// q (tₖ₋₂) Quaternion, from earth to platform coordinates, at the time tₖ₋₂
    const Eigen::Quaterniond quaternion_e2p__t2 = quaternion_p2e__t2.conjugate();
    /// q (tₖ₋₁) Quaternion, from earth to platform coordinates, at the time tₖ₋₁
    const Eigen::Quaterniond quaternion_e2p__t1 = quaternion_p2e__t1.conjugate();

    /// Δα_ip_p (tₖ₋₁) The integrated angluar velocities in [radian],
    /// of the inertial to platform system, in platform coordinates, at the time tₖ₋₁ (eq. 8.4)
    Eigen::Vector3d integratedAngularVelocity_ip_p__t1 = timeDifferenceSec__t1 * angularVelocity_ip_p__t1;
    /// Δα_ip_p (tₖ) The integrated angluar velocities in [radian],
    /// of the inertial to platform system, in platform coordinates, at the time tₖ (eq. 8.4)
    Eigen::Vector3d integratedAngularVelocity_ip_p__t0 = timeDifferenceSec__t0 * angularVelocity_ip_p__t0;
    /// Δβ⁠_ep_p (tₖ₋₁) The integrated angluar velocities in [radian],
    /// of the earth to platform system, in platform coordinates, at the time tₖ₋₁ (eq. 8.9)
    Eigen::Vector3d integratedAngularVelocity_ep_p__t1 = integratedAngularVelocity_ip_p__t1
                                                         - quaternion_e2p__t2 * angularVelocity_ie_e__t0 * timeDifferenceSec__t1;
    /// Δβ⁠_ep_p (tₖ) The integrated angluar velocities in [radian],
    /// of the earth to platform system, in platform coordinates, at the time tₖ (eq. 8.9)
    Eigen::Vector3d integratedAngularVelocity_ep_p__t0 = integratedAngularVelocity_ip_p__t0
                                                         - quaternion_e2p__t1 * angularVelocity_ie_e__t0 * timeDifferenceSec__t0;

    /// Runge-Kutta integration step [s]
    long double integrationStep = 2.0L * timeDifferenceSec__t0;

    /// ῶ_ep_p (tₖ₋₂) Taylor-Approximation of angular velocities in [rad/s],
    /// of the earth to platform system, in platform coordinates, at the time tₖ₋₂ (eq. 8.15)
    Eigen::Vector3d angularVelocity_ep_p__t2 = (3 * integratedAngularVelocity_ep_p__t1 - integratedAngularVelocity_ep_p__t0) / integrationStep;
    /// ῶ_ep_p (tₖ₋₁) Taylor-Approximation of angular velocities in [rad/s],
    /// of the earth to platform system, in platform coordinates, at the time tₖ₋₁ (eq. 8.15)
    Eigen::Vector3d angularVelocity_ep_p__t1 = (integratedAngularVelocity_ep_p__t1 + integratedAngularVelocity_ep_p__t0) / integrationStep;
    /// ῶ_ep_p (tₖ) Taylor-Approximation of angular velocities in [rad/s],
    /// of the earth to platform system, in platform coordinates, at the time tₖ (eq. 8.15)
    Eigen::Vector3d angularVelocity_ep_p__t0 = (3 * integratedAngularVelocity_ep_p__t0 - integratedAngularVelocity_ep_p__t0) / integrationStep;

    // clang-format off

    /// A Matrix at the time tₖ₋₂ (eq. 8.1 / 8.16). Reordered because Eigen::Quaternion(x,y,z,w), Skript (w,x,y,z)
    Eigen::Matrix4d A__t2;
    A__t2 <<             0.0             ,  angularVelocity_ep_p__t2(2), -angularVelocity_ep_p__t2(1),  angularVelocity_ep_p__t2(0),
             -angularVelocity_ep_p__t2(2),             0.0             ,  angularVelocity_ep_p__t2(0),  angularVelocity_ep_p__t2(1),
              angularVelocity_ep_p__t2(1), -angularVelocity_ep_p__t2(0),             0.0             ,  angularVelocity_ep_p__t2(2),
             -angularVelocity_ep_p__t2(0), -angularVelocity_ep_p__t2(1), -angularVelocity_ep_p__t2(2),             0.0             ;
    /// A Matrix at the time tₖ₋₁ (eq. 8.1 / 8.16). Reordered because Eigen::Quaternion(x,y,z,w), Skript (w,x,y,z)
    Eigen::Matrix4d A__t1;
    A__t1 <<             0.0             ,  angularVelocity_ep_p__t1(2), -angularVelocity_ep_p__t1(1),  angularVelocity_ep_p__t1(0),
             -angularVelocity_ep_p__t1(2),             0.0             ,  angularVelocity_ep_p__t1(0),  angularVelocity_ep_p__t1(1),
              angularVelocity_ep_p__t1(1), -angularVelocity_ep_p__t1(0),             0.0             ,  angularVelocity_ep_p__t1(2),
             -angularVelocity_ep_p__t1(0), -angularVelocity_ep_p__t1(1), -angularVelocity_ep_p__t1(2),             0.0             ;
    /// A Matrix at the time tₖ (eq. 8.1 / 8.16). Reordered because Eigen::Quaternion(x,y,z,w), Skript (w,x,y,z)
    Eigen::Matrix4d A__t0;
    A__t0 <<             0.0             ,  angularVelocity_ep_p__t0(2), -angularVelocity_ep_p__t0(1),  angularVelocity_ep_p__t0(0),
             -angularVelocity_ep_p__t0(2),             0.0             ,  angularVelocity_ep_p__t0(0),  angularVelocity_ep_p__t0(1),
              angularVelocity_ep_p__t0(1), -angularVelocity_ep_p__t0(0),             0.0             ,  angularVelocity_ep_p__t0(2),
             -angularVelocity_ep_p__t0(0), -angularVelocity_ep_p__t0(1), -angularVelocity_ep_p__t0(2),             0.0             ;

    // clang-format on

    /// Function calculating the Runge-Kutta coefficients
    auto f = [](const Eigen::Matrix4d& A, const Eigen::Vector4d& q) {
        Eigen::Vector4d coefficients = 0.5 * A * q;
        return Eigen::Quaterniond(coefficients);
    };

    /// Runge-Kutta coefficient k₁ (eq. 8.2)
    Eigen::Quaterniond k1 = f(A__t2, quaternion_p2e__t2.coeffs());
    /// Runge-Kutta coefficient k₂ (eq. 8.2)
    Eigen::Quaterniond k2 = f(A__t1, quaternion_p2e__t2.coeffs()
                                         + k1.coeffs() * integrationStep / 2.0);
    /// Runge-Kutta coefficient k₃ (eq. 8.2)
    Eigen::Quaterniond k3 = f(A__t0, quaternion_p2e__t2.coeffs()
                                         - k1.coeffs() * integrationStep
                                         + k2.coeffs() * 2.0 * integrationStep);

    /// Updated Quaternion (eq. 8.2)
    Eigen::Quaterniond q_p2e__t0;
    q_p2e__t0 = quaternion_p2e__t2.coeffs() + integrationStep * (k1.coeffs() + 4.0 * k2.coeffs() + k3.coeffs()) / 6.0;

    // Normalize Quaternion
    q_p2e__t0.normalize();

    return q_p2e__t0;
}

Eigen::Quaterniond updateQuaternion_b2n_RungeKutta3(
    const long double& timeDifferenceSec__t0,        // Δtₖ = (tₖ - tₖ₋₁) Time difference in [seconds]
    const long double& timeDifferenceSec__t1,        // Δtₖ₋₁ = (tₖ₋₁ - tₖ₋₂) Time difference in [seconds]
    const Eigen::Vector3d& angularVelocity_ip_b__t0, // ω_ip_b (tₖ) Angluar velocity in [rad/s], of the inertial to platform system, in body coordinates, at the time tₖ
    const Eigen::Vector3d& angularVelocity_ip_b__t1, // ω_ip_b (tₖ₋₁) Angluar velocity in [rad/s], of the inertial to platform system, in body coordinates, at the time tₖ₋₁
    const Eigen::Vector3d& angularVelocity_ie_n__t1, // ω_ie_n (tₖ₋₁) Angluar velocity in [rad/s], of the inertial to earth system, in navigation coordinates, at the time tₖ₋₁
    const Eigen::Vector3d& angularVelocity_en_n__t1, // ω_en_n (tₖ₋₁) Transport Rate, rotation rate of the Earth frame relative to the navigation frame, in navigation coordinates
    const Eigen::Quaterniond& quaternion_b2n__t1,    // q (tₖ₋₁) Quaternion, from body to navigation coordinates, at the time tₖ₋₁
    const Eigen::Quaterniond& quaternion_b2n__t2)    // q (tₖ₋₂) Quaternion, from body to navigation coordinates, at the time tₖ₋₂
{
    /// q (tₖ₋₂) Quaternion, from earth to platform coordinates, at the time tₖ₋₂
    const Eigen::Quaterniond quaternion_n2b__t2 = quaternion_b2n__t2.conjugate();
    /// q (tₖ₋₁) Quaternion, from earth to platform coordinates, at the time tₖ₋₁
    const Eigen::Quaterniond quaternion_n2b__t1 = quaternion_b2n__t1.conjugate();

    /// Δα_ip_p (tₖ₋₁) The integrated angluar velocities in [radian],
    /// of the inertial to platform system, in body coordinates, at the time tₖ₋₁ (eq. 8.4)
    Eigen::Vector3d integratedAngularVelocity_ip_b__t1 = timeDifferenceSec__t1 * angularVelocity_ip_b__t1;
    /// Δα_ip_p (tₖ) The integrated angluar velocities in [radian],
    /// of the inertial to platform system, in body coordinates, at the time tₖ (eq. 8.4)
    Eigen::Vector3d integratedAngularVelocity_ip_b__t0 = timeDifferenceSec__t0 * angularVelocity_ip_b__t0;
    /// Δβ⁠_nb_p (tₖ₋₁) The integrated angluar velocities in [radian],
    /// of the navigation to body system, in body coordinates, at the time tₖ₋₁ (eq. 8.9)
    Eigen::Vector3d integratedAngularVelocity_nb_b__t1 = integratedAngularVelocity_ip_b__t1
                                                         - quaternion_n2b__t2 * (angularVelocity_ie_n__t1 + angularVelocity_en_n__t1) * timeDifferenceSec__t1;
    /// Δβ⁠_nb_p (tₖ) The integrated angluar velocities in [radian],
    /// of the navigation to body system, in body coordinates, at the time tₖ (eq. 8.9)
    Eigen::Vector3d integratedAngularVelocity_nb_b__t0 = integratedAngularVelocity_ip_b__t0
                                                         - quaternion_n2b__t1 * (angularVelocity_ie_n__t1 + angularVelocity_en_n__t1) * timeDifferenceSec__t0;

    /// Runge-Kutta integration step [s]
    long double integrationStep = 2.0L * timeDifferenceSec__t0;

    /// ῶ_nb_p (tₖ₋₂) Taylor-Approximation of angular velocities in [rad/s],
    /// of the navigation to body system, in body coordinates, at the time tₖ₋₂ (eq. 8.15)
    Eigen::Vector3d angularVelocity_nb_b__t2 = (3 * integratedAngularVelocity_nb_b__t1 - integratedAngularVelocity_nb_b__t0) / integrationStep;
    /// ῶ_nb_p (tₖ₋₁) Taylor-Approximation of angular velocities in [rad/s],
    /// of the navigation to body system, in body coordinates, at the time tₖ₋₁ (eq. 8.15)
    Eigen::Vector3d angularVelocity_nb_b__t1 = (integratedAngularVelocity_nb_b__t1 + integratedAngularVelocity_nb_b__t0) / integrationStep;
    /// ῶ_nb_p (tₖ) Taylor-Approximation of angular velocities in [rad/s],
    /// of the navigation to body system, in body coordinates, at the time tₖ (eq. 8.15)
    Eigen::Vector3d angularVelocity_nb_b__t0 = (3 * integratedAngularVelocity_nb_b__t0 - integratedAngularVelocity_nb_b__t0) / integrationStep;

    // clang-format off

    /// A Matrix at the time tₖ₋₂ (eq. 8.1 / 8.16). Reordered because Eigen::Quaternion(x,y,z,w), Skript (w,x,y,z)
    Eigen::Matrix4d A__t2;
    A__t2 <<             0.0             ,  angularVelocity_nb_b__t2(2), -angularVelocity_nb_b__t2(1),  angularVelocity_nb_b__t2(0),
             -angularVelocity_nb_b__t2(2),             0.0             ,  angularVelocity_nb_b__t2(0),  angularVelocity_nb_b__t2(1),
              angularVelocity_nb_b__t2(1), -angularVelocity_nb_b__t2(0),             0.0             ,  angularVelocity_nb_b__t2(2),
             -angularVelocity_nb_b__t2(0), -angularVelocity_nb_b__t2(1), -angularVelocity_nb_b__t2(2),             0.0             ;
    /// A Matrix at the time tₖ₋₁ (eq. 8.1 / 8.16). Reordered because Eigen::Quaternion(x,y,z,w), Skript (w,x,y,z)
    Eigen::Matrix4d A__t1;
    A__t1 <<             0.0             ,  angularVelocity_nb_b__t1(2), -angularVelocity_nb_b__t1(1),  angularVelocity_nb_b__t1(0),
             -angularVelocity_nb_b__t1(2),             0.0             ,  angularVelocity_nb_b__t1(0),  angularVelocity_nb_b__t1(1),
              angularVelocity_nb_b__t1(1), -angularVelocity_nb_b__t1(0),             0.0             ,  angularVelocity_nb_b__t1(2),
             -angularVelocity_nb_b__t1(0), -angularVelocity_nb_b__t1(1), -angularVelocity_nb_b__t1(2),             0.0             ;
    /// A Matrix at the time tₖ (eq. 8.1 / 8.16). Reordered because Eigen::Quaternion(x,y,z,w), Skript (w,x,y,z)
    Eigen::Matrix4d A__t0;
    A__t0 <<             0.0             ,  angularVelocity_nb_b__t0(2), -angularVelocity_nb_b__t0(1),  angularVelocity_nb_b__t0(0),
             -angularVelocity_nb_b__t0(2),             0.0             ,  angularVelocity_nb_b__t0(0),  angularVelocity_nb_b__t0(1),
              angularVelocity_nb_b__t0(1), -angularVelocity_nb_b__t0(0),             0.0             ,  angularVelocity_nb_b__t0(2),
             -angularVelocity_nb_b__t0(0), -angularVelocity_nb_b__t0(1), -angularVelocity_nb_b__t0(2),             0.0             ;

    // clang-format on

    /// Function calculating the Runge-Kutta coefficients
    auto f = [](const Eigen::Matrix4d& A, const Eigen::Vector4d& q) {
        Eigen::Vector4d coefficients = 0.5 * A * q;
        return Eigen::Quaterniond(coefficients);
    };

    /// Runge-Kutta coefficient k₁ (eq. 8.2)
    Eigen::Quaterniond k1 = f(A__t2, quaternion_b2n__t2.coeffs());
    /// Runge-Kutta coefficient k₂ (eq. 8.2)
    Eigen::Quaterniond k2 = f(A__t1, quaternion_b2n__t2.coeffs()
                                         + k1.coeffs() * integrationStep / 2.0);
    /// Runge-Kutta coefficient k₃ (eq. 8.2)
    Eigen::Quaterniond k3 = f(A__t0, quaternion_b2n__t2.coeffs()
                                         - k1.coeffs() * integrationStep
                                         + k2.coeffs() * 2.0 * integrationStep);

    /// Updated Quaternion (eq. 8.2)
    Eigen::Quaterniond q_b2n__t0;
    q_b2n__t0 = quaternion_b2n__t2.coeffs() + integrationStep * (k1.coeffs() + 4.0 * k2.coeffs() + k3.coeffs()) / 6.0;

    // Normalize Quaternion
    q_b2n__t0.normalize();

    return q_b2n__t0;
}

Eigen::Vector3d updateVelocity_e_RungeKutta3(const long double& timeDifferenceSec__t0,     // Δtₖ Time difference in [seconds]. This epoch to previous epoch
                                             const long double& timeDifferenceSec__t1,     // Δtₖ₋₁ Time difference in [seconds]. Previous epoch to twice previous epoch
                                             const Eigen::Vector3d& acceleration_p__t0,    // a_p (tₖ) Acceleration in [m/s^2], in platform coordinates, at the time tₖ
                                             const Eigen::Vector3d& acceleration_p__t1,    // a_p (tₖ₋₁) Acceleration in [m/s^2], in platform coordinates, at the time tₖ₋₁
                                             const Eigen::Vector3d& velocity_e__t2,        // v_e (tₖ₋₂) Velocity in [m/s], in earth coordinates, at the time tₖ₋₂
                                             const Eigen::Vector3d& position_e__t2,        // x_e (tₖ₋₂) Position in [m/s], in earth coordinates, at the time tₖ₋₂
                                             const Eigen::Vector3d& gravity_e,             // g_e Gravity vector in [m/s^2], in earth coordinates
                                             const Eigen::Quaterniond& quaternion_p2e__t0, // q (tₖ) Quaternion, from platform to earth coordinates, at the time tₖ
                                             const Eigen::Quaterniond& quaternion_p2e__t1, // q (tₖ₋₁) Quaternion, from platform to earth coordinates, at the time tₖ₋₁
                                             const Eigen::Quaterniond& quaternion_p2e__t2) // q (tₖ₋₂) Quaternion, from platform to earth coordinates, at the time tₖ₋₂
{
    /// Δv_p (tₖ) Integrated velocity in [m/s], in platform coordinates, at the time tₖ (eq. 9.3)
    const Eigen::Vector3d deltaVelocity_p__t0 = acceleration_p__t0 * timeDifferenceSec__t0;

    /// Δv_p (tₖ₋₁) Integrated velocity in [m/s], in platform coordinates, at the time tₖ₋₁ (eq. 9.3)
    const Eigen::Vector3d deltaVelocity_p__t1 = acceleration_p__t1 * timeDifferenceSec__t1;

    /// Runge-Kutta integration step [s]
    const long double integrationStep = 2.0L * timeDifferenceSec__t0;

    /// Runge Kutta Integration of delta velocities (eq. 9.12)
    const Eigen::Vector3d rungeKuttaIntegration_e = (quaternion_p2e__t2 * (3 * deltaVelocity_p__t1 - deltaVelocity_p__t0)
                                                     + 4 * (quaternion_p2e__t1 * (deltaVelocity_p__t1 + deltaVelocity_p__t0))
                                                     + quaternion_p2e__t0 * (3 * deltaVelocity_p__t0 - deltaVelocity_p__t1))
                                                    / 6.0;

    /// The Coriolis force accounts for the fact that the NED frame is noninertial
    const Eigen::Vector3d coriolisAcceleration_e = 2 * InsConst::angularVelocityCrossProduct_ie_e * velocity_e__t2
                                                   + InsConst::angularVelocityCrossProduct_ie_e * InsConst::angularVelocityCrossProduct_ie_e * position_e__t2;

    /// v_e (tₖ) Velocity in [m/s], in earth coordinates, at the time tₖ (eq. 9.12)
    Eigen::Vector3d velocity_e__t0 = velocity_e__t2 + rungeKuttaIntegration_e - (coriolisAcceleration_e - gravity_e) * integrationStep;

    return velocity_e__t0;
}

Eigen::Vector3d updateVelocity_n_RungeKutta3(const long double& timeDifferenceSec__t0,        // Δtₖ Time difference in [seconds]. This epoch to previous epoch
                                             const long double& timeDifferenceSec__t1,        // Δtₖ₋₁ Time difference in [seconds]. Previous epoch to twice previous epoch
                                             const Eigen::Vector3d& acceleration_b__t0,       // a_p (tₖ) Acceleration in [m/s^2], in body coordinates, at the time tₖ
                                             const Eigen::Vector3d& acceleration_b__t1,       // a_p (tₖ₋₁) Acceleration in [m/s^2], in body coordinates, at the time tₖ₋₁
                                             const Eigen::Vector3d& velocity_n__t1,           // v_n (tₖ₋₁) Velocity in [m/s], in navigation coordinates, at the time tₖ₋₁
                                             const Eigen::Vector3d& velocity_n__t2,           // v_n (tₖ₋₂) Velocity in [m/s], in navigation coordinates, at the time tₖ₋₂
                                             const Eigen::Vector3d& gravity_n__t1,            // g_n (tₖ₋₁) Gravity vector in [m/s^2], in navigation coordinates, at the time tₖ₋₁
                                             const Eigen::Vector3d& angularVelocity_ie_n__t1, // ω_ie_n (tₖ₋₁) Nominal mean angular velocity of the Earth in [rad/s], in navigation coordinates, at the time tₖ₋₁
                                             const Eigen::Vector3d& angularVelocity_en_n__t1, // ω_ie_n (tₖ₋₁) Transport Rate in [rad/s], in navigation coordinates, at the time tₖ₋₁
                                             const Eigen::Quaterniond& quaternion_b2n__t0,    // q (tₖ) Quaternion, from body to navigation coordinates, at the time tₖ
                                             const Eigen::Quaterniond& quaternion_b2n__t1,    // q (tₖ₋₁) Quaternion, from body to navigation coordinates, at the time tₖ₋₁
                                             const Eigen::Quaterniond& quaternion_b2n__t2)    // q (tₖ₋₂) Quaternion, from body to navigation coordinates, at the time tₖ₋₂
{
    /// Δv_p (tₖ) Integrated velocity in [m/s], in body coordinates, at the time tₖ
    const Eigen::Vector3d deltaVelocity_b__t0 = acceleration_b__t0 * timeDifferenceSec__t0;

    /// Δv_p (tₖ₋₁) Integrated velocity in [m/s], in body coordinates, at the time tₖ₋₁
    const Eigen::Vector3d deltaVelocity_b__t1 = acceleration_b__t1 * timeDifferenceSec__t1;

    /// Runge-Kutta integration step [s]
    const long double integrationStep = 2.0L * timeDifferenceSec__t0;

    /// Runge Kutta Integration of delta velocities
    const Eigen::Vector3d rungeKuttaIntegration_n = (quaternion_b2n__t2 * (3 * deltaVelocity_b__t1 - deltaVelocity_b__t0)
                                                     + 4 * (quaternion_b2n__t1 * (deltaVelocity_b__t1 + deltaVelocity_b__t0))
                                                     + quaternion_b2n__t0 * (3 * deltaVelocity_b__t0 - deltaVelocity_b__t1))
                                                    / 6.0;

    /// The Coriolis force accounts for the fact that the NED frame is noninertial
    const Eigen::Vector3d coriolisAcceleration_n__t1 = (2 * angularVelocity_ie_n__t1 + angularVelocity_en_n__t1).cross(velocity_n__t1);

    /// v_e (tₖ) Velocity in [m/s], in navigation coordinates, at the time tₖ (eq. 6.13)
    Eigen::Vector3d velocity_n__t0 = velocity_n__t2 + rungeKuttaIntegration_n - (coriolisAcceleration_n__t1 - gravity_n__t1) * integrationStep;

    return velocity_n__t0;
}

Eigen::Vector3d updatePosition_e(const long double& timeDifferenceSec__t0, // Δtₖ Time difference in [seconds]. This epoch to previous epoch
                                 const Eigen::Vector3d& position_e__t1,    // x_e (tₖ₋₁) Position in [m/s], in earth coordinates, at the time tₖ₋₁
                                 const Eigen::Vector3d& velocity_e__t1)    // v_e (tₖ₋₁) Velocity in [m/s], in earth coordinates, at the time tₖ₋₁
{
    /// x_e (tₖ) Position in [m/s], in earth coordinates, at the time tₖ
    Eigen::Vector3d position_e__t0 = position_e__t1 + velocity_e__t1 * timeDifferenceSec__t0;

    return position_e__t0;
}

Eigen::Vector3d updatePosition_n(const long double& timeDifferenceSec__t0, // Δtₖ Time difference in [seconds]. This epoch to previous epoch
                                 const Eigen::Vector3d& latLonHeight__t1,  // [𝜙, λ, h] (tₖ₋₁) Latitude, Longitude and height in [rad, rad, m] at the time tₖ₋₁
                                 const Eigen::Vector3d& velocity_n__t1,    // v_n (tₖ₋₁) Velocity in [m/s], in navigation coordinates, at the time tₖ₋₁
                                 const double& R_N,                        // R_N North/South (meridian) earth radius [m]
                                 const double& R_E)                        // R_E East/West (prime vertical) earth radius [m]
{
    /// 𝜙 Latitude in [rad]
    const auto& latitude = latLonHeight__t1(0);
    /// λ Longitude in [rad]
    const auto& longitude = latLonHeight__t1(1);
    /// h Height in [m]
    const auto& height = latLonHeight__t1(2);

    /// Δtₖ Time difference in [seconds]
    const auto tau = static_cast<double>(timeDifferenceSec__t0);

    /// Velocity North in [m/s]
    const auto& v_N = velocity_n__t1(0);
    /// Velocity East in [m/s]
    const auto& v_E = velocity_n__t1(1);
    /// Velocity Down in [m/s]
    const auto& v_D = velocity_n__t1(2);

    /// [𝜙, λ, h] (tₖ) Latitude, Longitude and Height in [rad, rad, m], at the current time tₖ (see Gleason eq. 6.18 - 6.20)
    Eigen::Vector3d latLonHeight__t0 = {
        latitude + tau * (v_N / (R_N + height)),
        longitude + tau * (v_E / ((R_E + height) * std::cos(latitude))),
        height - tau * v_D
    };

    return latLonHeight__t0;
}

double earthRadius_N(const double& a, const double& e_squared, const double& latitude)
{
    double k = std::sqrt(1 - e_squared * std::pow(std::sin(latitude), 2));

    /// North/South (meridian) earth radius [m]
    double R_N = a * (1 - e_squared) / std::pow(k, 3);

    return R_N;
}

double earthRadius_E(const double& a, const double& e_squared, const double& latitude)
{
    /// East/West (prime vertical) earth radius [m]
    double R_E = a / std::sqrt(1 - e_squared * std::pow(std::sin(latitude), 2));

    return R_E;
}

Eigen::Vector3d transportRate(const Eigen::Vector3d& latLonHeight__t1, // [𝜙, λ, h] (tₖ₋₁) Latitude, Longitude and height in [rad, rad, m] at the time tₖ₋₁
                              const Eigen::Vector3d& velocity_n__t1,   // v_n (tₖ₋₁) Velocity in [m/s], in navigation coordinates, at the time tₖ₋₁
                              const double& R_N,                       // R_N North/South (meridian) earth radius [m]
                              const double& R_E)                       // R_E East/West (prime vertical) earth radius [m]
{
    /// 𝜙 Latitude in [rad]
    const auto& latitude = latLonHeight__t1(0);
    /// h Height in [m]
    const auto& height = latLonHeight__t1(2);

    /// Velocity North in [m/s]
    const auto& v_N = velocity_n__t1(0);
    /// Velocity East in [m/s]
    const auto& v_E = velocity_n__t1(1);

    /// ω_en_n (tₖ₋₁) Transport Rate, rotation rate of the Earth frame relative to the navigation frame,
    /// in navigation coordinates (eq. 6.15)
    Eigen::Vector3d angularVelocity_en_n__t1;
    angularVelocity_en_n__t1(0) = v_E / (R_E + height);
    angularVelocity_en_n__t1(1) = -v_N / (R_N + height);
    angularVelocity_en_n__t1(2) = -angularVelocity_en_n__t1(0) * std::tan(latitude);

    return angularVelocity_en_n__t1;
}

} // namespace NAV