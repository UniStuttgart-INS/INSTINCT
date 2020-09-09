#include "InsMechanization.hpp"

namespace NAV
{
Eigen::Quaterniond updateQuaternionsRungeKutta3(
    const long double& timeDifferenceSec__t0,      // Δtₖ = (tₖ - tₖ₋₁) Time difference in [seconds]
    const long double& timeDifferenceSec__t1,      // Δtₖ₋₁ = (tₖ₋₁ - tₖ₋₂) Time difference in [seconds]
    const Eigen::Vector3d& angularVelocity_ip__t0, // ω_ip_p (tₖ) Angluar velocity in [rad/s], of the inertial to platform system, in platform coordinates, at the time tₖ
    const Eigen::Vector3d& angularVelocity_ip__t1, // ω_ip_p (tₖ₋₁) Angluar velocity in [rad/s], of the inertial to platform system, in platform coordinates, at the time tₖ₋₁
    const Eigen::Vector3d& angularVelocity_ie__t0, // ω_ie_e (tₖ) Angluar velocity in [rad/s], of the inertial to earth system, in earth coordinates, at the time tₖ
    const Eigen::Quaterniond& quaternion_p2e__t1,  // q (tₖ₋₁) Quaternion, from platform to earth coordinates, at the time tₖ₋₁
    const Eigen::Quaterniond& quaternion_p2e__t2)  // q (tₖ₋₂) Quaternion, from platform to earth coordinates, at the time tₖ₋₂
{
    /// q (tₖ₋₂) Quaternion, from earth to platform coordinates, at the time tₖ₋₂
    const Eigen::Quaterniond quaternion_e2p__t2 = quaternion_p2e__t2.conjugate();
    /// q (tₖ₋₁) Quaternion, from earth to platform coordinates, at the time tₖ₋₁
    const Eigen::Quaterniond quaternion_e2p__t1 = quaternion_p2e__t1.conjugate();

    /// Δα_ip_p (tₖ₋₁) The integrated angluar velocities in [radian],
    /// of the inertial to platform system, in platform coordinates, at the time tₖ₋₁ (eq. 8.4)
    Eigen::Vector3d integratedAngularVelocity_ip__t1 = timeDifferenceSec__t1 * angularVelocity_ip__t1;
    /// Δα_ip_p (tₖ) The integrated angluar velocities in [radian],
    /// of the inertial to platform system, in platform coordinates, at the time tₖ (eq. 8.4)
    Eigen::Vector3d integratedAngularVelocity_ip__t0 = timeDifferenceSec__t0 * angularVelocity_ip__t0;
    /// Δβ⁠_ep_p (tₖ₋₁) The integrated angluar velocities in [radian],
    /// of the earth to platform system, in platform coordinates, at the time tₖ₋₁ (eq. 8.9)
    Eigen::Vector3d integratedAngularVelocity_ep__t1 = integratedAngularVelocity_ip__t1
                                                       - quaternion_e2p__t2 * angularVelocity_ie__t0 * timeDifferenceSec__t1;
    /// Δβ⁠_ep_p (tₖ) The integrated angluar velocities in [radian],
    /// of the earth to platform system, in platform coordinates, at the time tₖ (eq. 8.9)
    Eigen::Vector3d integratedAngularVelocity_ep__t0 = integratedAngularVelocity_ip__t0
                                                       - quaternion_e2p__t1 * angularVelocity_ie__t0 * timeDifferenceSec__t0;

    /// Runge-Kutta integration step
    long double integrationStep = 2.0L * timeDifferenceSec__t0;

    /// ῶ_ep_p (tₖ₋₂) Taylor-Approximation of angular velocities in [rad/s],
    /// of the earth to platform system, in platform coordinates, at the time tₖ₋₂ (eq. 8.15)
    Eigen::Vector3d angularVelocity_ep__t2 = (3 * integratedAngularVelocity_ep__t1 - integratedAngularVelocity_ep__t0) / integrationStep;
    /// ῶ_ep_p (tₖ₋₁) Taylor-Approximation of angular velocities in [rad/s],
    /// of the earth to platform system, in platform coordinates, at the time tₖ₋₁ (eq. 8.15)
    Eigen::Vector3d angularVelocity_ep__t1 = (integratedAngularVelocity_ep__t1 + integratedAngularVelocity_ep__t0) / integrationStep;
    /// ῶ_ep_p (tₖ) Taylor-Approximation of angular velocities in [rad/s],
    /// of the earth to platform system, in platform coordinates, at the time tₖ (eq. 8.15)
    Eigen::Vector3d angularVelocity_ep__t0 = (3 * integratedAngularVelocity_ep__t0 - integratedAngularVelocity_ep__t0) / integrationStep;

    // clang-format off

    /// A Matrix at the time tₖ₋₂ (eq. 8.1 / 8.16). Reordered because Eigen::Quaternion(x,y,z,w), Skript (w,x,y,z)
    Eigen::Matrix4d A__t2;
    A__t2 <<          0.0          ,  angularVelocity_ep__t2(2), -angularVelocity_ep__t2(1),  angularVelocity_ep__t2(0),
             -angularVelocity_ep__t2(2),          0.0          ,  angularVelocity_ep__t2(0),  angularVelocity_ep__t2(1),
              angularVelocity_ep__t2(1), -angularVelocity_ep__t2(0),          0.0          ,  angularVelocity_ep__t2(2),
             -angularVelocity_ep__t2(0), -angularVelocity_ep__t2(1), -angularVelocity_ep__t2(2),          0.0          ;
    /// A Matrix at the time tₖ₋₁ (eq. 8.1 / 8.16). Reordered because Eigen::Quaternion(x,y,z,w), Skript (w,x,y,z)
    Eigen::Matrix4d A__t1;
    A__t1 <<          0.0          ,  angularVelocity_ep__t1(2), -angularVelocity_ep__t1(1),  angularVelocity_ep__t1(0),
             -angularVelocity_ep__t1(2),          0.0          ,  angularVelocity_ep__t1(0),  angularVelocity_ep__t1(1),
              angularVelocity_ep__t1(1), -angularVelocity_ep__t1(0),          0.0          ,  angularVelocity_ep__t1(2),
             -angularVelocity_ep__t1(0), -angularVelocity_ep__t1(1), -angularVelocity_ep__t1(2),          0.0          ;
    /// A Matrix at the time tₖ (eq. 8.1 / 8.16). Reordered because Eigen::Quaternion(x,y,z,w), Skript (w,x,y,z)
    Eigen::Matrix4d A__t0;
    A__t0 <<          0.0          ,  angularVelocity_ep__t0(2), -angularVelocity_ep__t0(1),  angularVelocity_ep__t0(0),
             -angularVelocity_ep__t0(2),          0.0          ,  angularVelocity_ep__t0(0),  angularVelocity_ep__t0(1),
              angularVelocity_ep__t0(1), -angularVelocity_ep__t0(0),          0.0          ,  angularVelocity_ep__t0(2),
             -angularVelocity_ep__t0(0), -angularVelocity_ep__t0(1), -angularVelocity_ep__t0(2),          0.0          ;

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
    Eigen::Quaterniond q;
    q = quaternion_p2e__t2.coeffs() + integrationStep * (k1.coeffs() + 4.0 * k2.coeffs() + k3.coeffs()) / 6.0;

    // Normalize Quaternion
    q.normalize();

    return q;
}

} // namespace NAV