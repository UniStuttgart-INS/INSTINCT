#include "Mechanization.hpp"

#include "Navigation/Constants.hpp"
#include "Navigation/Ellipsoid/Ellipsoid.hpp"
#include "Navigation/INS/Functions.hpp"
#include "Navigation/Math/Math.hpp"
#include "Navigation/Transformations/CoordinateFrames.hpp"
#include "util/Logger.hpp"

#include <cmath>

namespace NAV
{
Eigen::Vector4d calcTimeDerivativeFor_e_Quat_b(const Eigen::Vector3d& b_omega_eb, const Eigen::Vector4d& e_Quat_b_coeffs)
{
    // Angular rates in matrix form (Titterton (2005), eq. (11.35))
    Eigen::Matrix4d A;

    // clang-format off
    A <<       0.0     , -b_omega_eb(0), -b_omega_eb(1), -b_omega_eb(2),
          b_omega_eb(0),       0.0     ,  b_omega_eb(2), -b_omega_eb(1),
          b_omega_eb(1), -b_omega_eb(2),       0.0     ,  b_omega_eb(0),
          b_omega_eb(2),  b_omega_eb(1), -b_omega_eb(0),       0.0     ;
    // clang-format on

    // Propagation of an attitude Quaternion with time (Titterton, ch. 11.2.5, eq. 11.33-11.35, p. 319)
    return 0.5 * A * e_Quat_b_coeffs; // (w, x, y, z)
}

Eigen::Vector3d e_calcTimeDerivativeForVelocity(const Eigen::Vector3d& e_measuredForce,
                                                const Eigen::Vector3d& e_coriolisAcceleration,
                                                const Eigen::Vector3d& e_gravitation,
                                                const Eigen::Vector3d& e_centrifugalAcceleration)
{
    return e_measuredForce
           - e_coriolisAcceleration
           + e_gravitation
           - e_centrifugalAcceleration;
}

Eigen::Vector3d e_calcTimeDerivativeForPosition(const Eigen::Vector3d& e_velocity)
{
    return e_velocity;
}

Eigen::Matrix<double, 10, 1> e_calcPosVelAttDerivative(const Eigen::Matrix<double, 10, 1>& y, const PosVelAttDerivativeConstants_e& c)
{
    //       0  1  2  3   4    5    6   7  8  9
    // ∂/∂t [w, x, y, z, v_x, v_y, v_z, x, y, z]^T
    Eigen::Matrix<double, 10, 1> y_dot = Eigen::Matrix<double, 10, 1>::Zero();

    Eigen::Vector3d lla_position = trafo::ecef2lla_WGS84(y.segment<3>(7));

    Eigen::Quaterniond e_Quat_b{ y(0), y(1), y(2), y(3) };
    Eigen::Quaterniond n_Quat_e = trafo::n_Quat_e(lla_position(0), lla_position(1));

    Eigen::Quaterniond b_Quat_e = e_Quat_b.conjugate();
    Eigen::Quaterniond e_Quat_n = n_Quat_e.conjugate();

    LOG_DATA("e_velocity   = {} [m/s]", y.segment<3>(4).transpose());
    LOG_DATA("e_position = {} [m]", y.segment<3>(7).transpose());
    LOG_DATA("b_measuredForce = {} [m/s^2]", c.b_measuredForce.transpose());
    LOG_DATA("b_omega_ib = {} [rad/s]", c.b_omega_ib.transpose());

    // ω_eb_b = ω_ib_b - C_be * ω_ie_e
    Eigen::Vector3d b_omega_eb = c.b_omega_ib - b_Quat_e * (c.angularRateEarthRotationCompensationEnabled ? InsConst::e_omega_ie : Eigen::Vector3d::Zero());
    LOG_DATA("b_omega_eb = {} [rad/s]", b_omega_eb.transpose());

    // Coriolis acceleration in [m/s^2] (acceleration due to motion in rotating reference frame)
    Eigen::Vector3d e_coriolisAcceleration = c.coriolisAccelerationCompensationEnabled
                                                 ? e_calcCoriolisAcceleration(InsConst::e_omega_ie, y.segment<3>(4))
                                                 : Eigen::Vector3d::Zero();
    LOG_DATA("e_coriolisAcceleration = {} [m/s^2]", e_coriolisAcceleration.transpose());
    // Centrifugal acceleration in [m/s^2] (acceleration that makes a body follow a curved path)
    Eigen::Vector3d e_centrifugalAcceleration = c.centrifgalAccelerationCompensationEnabled
                                                    ? e_calcCentrifugalAcceleration(y.segment<3>(7), InsConst::e_omega_ie)
                                                    : Eigen::Vector3d::Zero();
    LOG_DATA("e_centrifugalAcceleration = {} [m/s^2]", e_centrifugalAcceleration.transpose());

    Eigen::Vector3d e_gravitation = e_Quat_n * n_calcGravitation(lla_position, c.gravitationModel);
    LOG_DATA("e_gravitation = {} [m/s^2] ({})", e_gravitation.transpose(), to_string(c.gravitationModel));

    y_dot.segment<4>(0) = calcTimeDerivativeFor_e_Quat_b(b_omega_eb,       // ω_eb_b Body rate with respect to the ECEF frame, expressed in the body frame
                                                         y.segment<4>(0)); // e_Quat_b_coeffs Coefficients of the quaternion e_Quat_b in order w, x, y, z (q = w + ix + jy + kz)

    // TODO: Zwiener c.velocityUpdateRotationCorrectionEnabled

    y_dot.segment<3>(4) = e_calcTimeDerivativeForVelocity(e_Quat_b * c.b_measuredForce, // f_n Specific force vector as measured by a triad of accelerometers and resolved into ECEF frame coordinates
                                                          e_coriolisAcceleration,       // Coriolis acceleration in ECEF coordinates in [m/s^2]
                                                          e_gravitation,                // Local gravitation vector (caused by effects of mass attraction) in ECEF frame coordinates [m/s^2]
                                                          e_centrifugalAcceleration);   // Centrifugal acceleration in ECEF coordinates in [m/s^2]

    y_dot.segment<3>(7) = e_calcTimeDerivativeForPosition(y.segment<3>(4)); // Velocity with respect to the Earth in ECEF frame coordinates [m/s]

    LOG_DATA("e_Quat_b_dot = {} ", y_dot.segment<4>(0).transpose());
    LOG_DATA("e_velocity_dot = {} [m/s^2]", y_dot.segment<3>(4).transpose());
    LOG_DATA("e_position_dot = {} [m/s]", y_dot.segment<3>(7).transpose());

    return y_dot;
}

} // namespace NAV