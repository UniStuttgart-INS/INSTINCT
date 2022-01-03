#include "Mechanization.hpp"

#include "Navigation/Constants.hpp"
#include "Navigation/Ellipsoid/Ellipsoid.hpp"
#include "Navigation/INS/Functions.hpp"
#include "Navigation/Math/Math.hpp"
#include "Navigation/Math/NumericalIntegration.hpp"
#include "Navigation/Transformations/CoordinateFrames.hpp"
#include "util/Logger.hpp"

#include <cmath>

namespace NAV
{
Eigen::Vector4d calcTimeDerivativeForQuaternion_nb(const Eigen::Vector3d& omega_nb_b, const Eigen::Vector4d& q_nb_coeffs)
{
    // Angular rates in matrix form (Titterton (2005), eq. (11.35))
    Eigen::Matrix4d A;

    // clang-format off
    A <<       0.0     , -omega_nb_b(0), -omega_nb_b(1), -omega_nb_b(2),
          omega_nb_b(0),       0.0     ,  omega_nb_b(2), -omega_nb_b(1),
          omega_nb_b(1), -omega_nb_b(2),       0.0     ,  omega_nb_b(0),
          omega_nb_b(2),  omega_nb_b(1), -omega_nb_b(0),       0.0     ;
    // clang-format on

    // Propagation of an attitude Quaternion with time (Titterton, ch. 11.2.5, eq. 11.33-11.35, p. 319)
    return 0.5 * A * q_nb_coeffs; // (w, x, y, z)
}

Eigen::Vector3d calcTimeDerivativeForVelocity_n(const Eigen::Vector3d& f_n,
                                                const Eigen::Vector3d& coriolisAcceleration_n,
                                                const Eigen::Vector3d& gravitation_n,
                                                const Eigen::Vector3d& centrifugalAcceleration_n)
{
    return f_n
           - coriolisAcceleration_n
           + gravitation_n
           - centrifugalAcceleration_n;
}

Eigen::Vector3d calcTimeDerivativeForVelocity_n_RotationCorrection(const Eigen::Vector3d& f_n,
                                                                   const Eigen::Vector3d& coriolisAcceleration_n,
                                                                   const Eigen::Vector3d& gravitation_n,
                                                                   const Eigen::Vector3d& centrifugalAcceleration_n,
                                                                   const Eigen::Vector3d& omega_ib_b,
                                                                   const Eigen::Vector3d& omega_ie_n,
                                                                   const Eigen::Vector3d& omega_en_n,
                                                                   const Eigen::Quaterniond& quaternion_nb,
                                                                   const double& timeDifferenceSec)
{
    // q Quaternion, from n-system to b-system
    const Eigen::Quaterniond quaternion_bn = quaternion_nb.conjugate();

    // Δβ⁠_nb_p The angular velocities in [rad], of the navigation to body system, in body coordinates (eq. 8.9)
    const Eigen::Vector3d omega_nb_b = omega_ib_b - quaternion_bn * (omega_ie_n + omega_en_n);

    // Zwiener eq. (3.37)
    Eigen::Matrix3d rotA = 2 * skewSymmetricMatrix(omega_nb_b) * std::pow(std::sin(timeDifferenceSec * omega_nb_b.norm() * 0.5) / omega_nb_b.norm(), 2);
    // Zwiener eq. (3.43)
    Eigen::Matrix3d rotB = (std::pow(timeDifferenceSec, 3) / 6.0 - std::pow(omega_nb_b.norm(), 2) / 120.0 * std::pow(timeDifferenceSec, 5)) * skewSymmetricMatrix2(omega_nb_b);

    // Rotation correction factor from Zwiener eq. (3.44)
    Eigen::Matrix3d rotCorr = Eigen::Matrix3d::Identity(3, 3) * timeDifferenceSec + rotA + rotB;
    rotCorr /= timeDifferenceSec;

    // Specific force in body coordinates
    Eigen::Vector3d f_b = quaternion_bn * f_n;

    return quaternion_nb * (rotCorr * f_b) - coriolisAcceleration_n + gravitation_n - centrifugalAcceleration_n;
}

Eigen::Vector3d calcTimeDerivativeForPosition_lla(const Eigen::Vector3d& velocity_n,
                                                  const double& phi,
                                                  const double& h,
                                                  const double& R_N,
                                                  const double& R_E)
{
    // Velocity North in [m/s]
    const auto& v_N = velocity_n(0);
    // Velocity East in [m/s]
    const auto& v_E = velocity_n(1);
    // Velocity Down in [m/s]
    const auto& v_D = velocity_n(2);

    return { v_N / (R_N + h),
             v_E / ((R_E + h) * std::cos(phi)),
             -v_D };
}

Eigen::Matrix<double, 10, 1> calcPosVelAttDerivative_n(const Eigen::Matrix<double, 10, 1>& y, const PosVelAttDerivativeConstants_n& c)
{
    //       0  1  2  3   4    5    6   7  8  9
    // ∂/∂t [w, x, y, z, v_N, v_E, v_D, 𝜙, λ, h]^T
    Eigen::Matrix<double, 10, 1> y_dot = Eigen::Matrix<double, 10, 1>::Zero();

    const Eigen::Quaterniond q_nb{ y(0), y(1), y(2), y(3) };
    const Eigen::Quaterniond q_ne = trafo::quat_ne(y(7), y(8));

    LOG_DATA("rollPitchYaw = {} [°]", trafo::rad2deg(trafo::quat2eulerZYX(q_nb)).transpose());
    LOG_DATA("velocity_n   = {} [m/s]", y.segment<3>(4).transpose());
    LOG_DATA("position_lla = {}°, {}°, {} m", trafo::rad2deg(y(7)), trafo::rad2deg(y(8)), y(9));
    LOG_DATA("f_b = {} [m/s^2]", c.f_b.transpose());
    LOG_DATA("omega_ib_b = {} [rad/s]", c.omega_ib_b.transpose());

    const auto R_N = calcEarthRadius_N(y(7));
    LOG_DATA("R_N = {} [m]", R_N);
    const auto R_E = calcEarthRadius_E(y(7));
    LOG_DATA("R_E = {} [m]", R_E);

    // ω_ie_e Turn rate of the Earth expressed in Earth frame coordinates
    const Eigen::Vector3d& omega_ie_e = InsConst::angularVelocity_ie_e;
    LOG_DATA("omega_ie_e = {} [rad/s]", omega_ie_e.transpose());
    // ω_ie_n Turn rate of the Earth expressed in local-navigation frame coordinates
    const Eigen::Vector3d omega_ie_n = q_ne * omega_ie_e;
    LOG_DATA("omega_ie_n = {} [rad/s]", omega_ie_n.transpose());
    // ω_en_n Turn rate of the local frame with respect to the Earth-fixed frame, called the transport rate, expressed in local-navigation frame coordinates
    const Eigen::Vector3d omega_en_n = calcTransportRate_n(y.segment<3>(7), y.segment<3>(4), R_N, R_E);
    LOG_DATA("omega_en_n = {} [rad/s]", omega_en_n.transpose());
    // ω_nb_b = ω_ib_b - C_bn * (ω_ie_n + ω_en_n)
    const Eigen::Vector3d omega_nb_b = c.omega_ib_b
                                       - q_nb.conjugate()
                                             * ((c.angularRateEarthRotationCompensationEnabled ? omega_ie_n : Eigen::Vector3d::Zero())
                                                + (c.angularRateTransportRateCompensationEnabled ? omega_en_n : Eigen::Vector3d::Zero()));
    LOG_DATA("omega_nb_b = {} [rad/s]", omega_nb_b.transpose());

    // Coriolis acceleration in [m/s^2] (acceleration due to motion in rotating reference frame)
    const Eigen::Vector3d coriolisAcceleration_n = c.coriolisAccelerationCompensationEnabled
                                                       ? calcCoriolisAcceleration_n(omega_ie_n, omega_en_n, y.segment<3>(4))
                                                       : Eigen::Vector3d::Zero();
    LOG_DATA("coriolisAcceleration_n = {} [m/s^2]", coriolisAcceleration_n.transpose());
    // Centrifugal acceleration in [m/s^2] (acceleration that makes a body follow a curved path)
    const Eigen::Vector3d centrifugalAcceleration_n = c.centrifgalAccelerationCompensationEnabled
                                                          ? q_ne * calcCentrifugalAcceleration_e(trafo::lla2ecef_WGS84(y.segment<3>(7)), omega_ie_e)
                                                          : Eigen::Vector3d::Zero();
    LOG_DATA("centrifugalAcceleration_n = {} [m/s^2]", centrifugalAcceleration_n.transpose());

    const Eigen::Vector3d gravitation_n = calcGravitation_n(y.segment<3>(7), c.gravityModel);
    LOG_DATA("gravitation_n = {} [m/s^2] ({})", gravitation_n.transpose(), to_string(c.gravityModel));

    y_dot.segment<4>(0) = calcTimeDerivativeForQuaternion_nb(omega_nb_b,       // ω_nb_b Body rate with respect to the navigation frame, expressed in the body frame
                                                             y.segment<4>(0)); // q_nb_coeffs Coefficients of the quaternion q_nb in order w, x, y, z (q = w + ix + jy + kz)

    if (c.velocityUpdateRotationCorrectionEnabled)
    {
        y_dot.segment<3>(4) = calcTimeDerivativeForVelocity_n_RotationCorrection(q_nb * c.f_b,              // f_n Specific force vector as measured by a triad of accelerometers and resolved into local-navigation frame coordinates
                                                                                 coriolisAcceleration_n,    // Coriolis acceleration in local-navigation coordinates in [m/s^2]
                                                                                 gravitation_n,             // gravitation_n Local gravitation vector (caused by effects of mass attraction) in local-navigation frame coordinates [m/s^2]
                                                                                 centrifugalAcceleration_n, // Centrifugal acceleration in local-navigation coordinates in [m/s^2]
                                                                                 c.omega_ib_b,
                                                                                 omega_ie_n,
                                                                                 omega_en_n,
                                                                                 q_nb,
                                                                                 c.timeDifferenceSec);
    }
    else
    {
        y_dot.segment<3>(4) = calcTimeDerivativeForVelocity_n(q_nb * c.f_b,               // f_n Specific force vector as measured by a triad of accelerometers and resolved into local-navigation frame coordinates
                                                              coriolisAcceleration_n,     // Coriolis acceleration in local-navigation coordinates in [m/s^2]
                                                              gravitation_n,              // gravitation_n Local gravitation vector (caused by effects of mass attraction) in local-navigation frame coordinates [m/s^2]
                                                              centrifugalAcceleration_n); // Centrifugal acceleration in local-navigation coordinates in [m/s^2]
    }

    y_dot.segment<3>(7) = calcTimeDerivativeForPosition_lla(y.segment<3>(4), // v_n Velocity with respect to the Earth in local-navigation frame coordinates [m/s]
                                                            y(7),            // 𝜙 Latitude in [rad]
                                                            y(9),            // h Altitude in [m]
                                                            R_N,             // North/South (meridian) earth radius [m]
                                                            R_E);            // East/West (prime vertical) earth radius [m]

    LOG_DATA("q_nb_dot = {} ", y_dot.segment<4>(0).transpose());
    LOG_DATA("v_n_dot = {} [m/s^2]", y_dot.segment<3>(4).transpose());
    LOG_DATA("latLonAlt_dot = {} [rad/s, rad/s, m/s]", y_dot.segment<3>(7).transpose());

    return y_dot;
}

} // namespace NAV