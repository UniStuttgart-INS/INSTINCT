// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Mechanization.hpp"

#include "Navigation/Constants.hpp"
#include "Navigation/Ellipsoid/Ellipsoid.hpp"
#include "Navigation/INS/Functions.hpp"
#include "Navigation/Math/Math.hpp"
#include "Navigation/Math/NumericalIntegration.hpp"
#include "Navigation/Transformations/CoordinateFrames.hpp"
#include "Navigation/Transformations/Units.hpp"
#include "util/Logger.hpp"

#include <cmath>

namespace NAV
{
Eigen::Vector4d calcTimeDerivativeFor_n_Quat_b(const Eigen::Vector3d& b_omega_nb, const Eigen::Vector4d& n_Quat_b_coeffs)
{
    // Angular rates in matrix form (Titterton (2005), eq. (11.35))
    Eigen::Matrix4d A;

    // clang-format off
    A <<       0.0     , -b_omega_nb(0), -b_omega_nb(1), -b_omega_nb(2),
          b_omega_nb(0),       0.0     ,  b_omega_nb(2), -b_omega_nb(1),
          b_omega_nb(1), -b_omega_nb(2),       0.0     ,  b_omega_nb(0),
          b_omega_nb(2),  b_omega_nb(1), -b_omega_nb(0),       0.0     ;
    // clang-format on

    // Propagation of an attitude Quaternion with time (Titterton, ch. 11.2.5, eq. 11.33-11.35, p. 319)
    return 0.5 * A * n_Quat_b_coeffs; // (w, x, y, z)
}

Eigen::Vector3d n_calcTimeDerivativeForVelocity(const Eigen::Vector3d& n_measuredForce,
                                                const Eigen::Vector3d& n_coriolisAcceleration,
                                                const Eigen::Vector3d& n_gravitation,
                                                const Eigen::Vector3d& n_centrifugalAcceleration)
{
    return n_measuredForce
           - n_coriolisAcceleration
           + n_gravitation
           - n_centrifugalAcceleration;
}

Eigen::Vector3d lla_calcTimeDerivativeForPosition(const Eigen::Vector3d& n_velocity,
                                                  const double& phi,
                                                  const double& h,
                                                  const double& R_N,
                                                  const double& R_E)
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

Eigen::Matrix<double, 16, 1> n_calcPosVelAttDerivative(const Eigen::Matrix<double, 16, 1>& y, const PosVelAttDerivativeConstants_n& c)
{
    //  0  1  2  3   4    5    6   7  8  9  10  11  12  13  14  15
    // [w, x, y, z, v_N, v_E, v_D, 𝜙, λ, h, fx, fy, fz, ωx, ωy, ωz]^T
    Eigen::Matrix<double, 16, 1> y_dot = Eigen::Matrix<double, 16, 1>::Zero();

    Eigen::Quaterniond n_Quat_b{ y(0), y(1), y(2), y(3) };
    Eigen::Quaterniond n_Quat_e = trafo::n_Quat_e(y(7), y(8));

    Eigen::Vector3d b_measuredForce = y.segment<3>(10);
    Eigen::Vector3d b_omega_ib = y.segment<3>(13);

    LOG_DATA("rollPitchYaw = {} [°]", rad2deg(trafo::quat2eulerZYX(n_Quat_b)).transpose());
    LOG_DATA("n_velocity   = {} [m/s]", y.segment<3>(4).transpose());
    LOG_DATA("lla_position = {}°, {}°, {} m", rad2deg(y(7)), rad2deg(y(8)), y(9));
    LOG_DATA("b_measuredForce = {} [m/s^2]", b_measuredForce.transpose());
    LOG_DATA("b_omega_ib = {} [rad/s]", b_omega_ib.transpose());

    auto R_N = calcEarthRadius_N(y(7));
    LOG_DATA("R_N = {} [m]", R_N);
    auto R_E = calcEarthRadius_E(y(7));
    LOG_DATA("R_E = {} [m]", R_E);

    // ω_ie_n Turn rate of the Earth expressed in local-navigation frame coordinates
    Eigen::Vector3d n_omega_ie = n_Quat_e * InsConst::e_omega_ie;
    LOG_DATA("n_omega_ie = {} [rad/s]", n_omega_ie.transpose());
    // ω_en_n Turn rate of the local frame with respect to the Earth-fixed frame, called the transport rate, expressed in local-navigation frame coordinates
    Eigen::Vector3d n_omega_en = n_calcTransportRate(y.segment<3>(7), y.segment<3>(4), R_N, R_E);
    LOG_DATA("n_omega_en = {} [rad/s]", n_omega_en.transpose());
    // ω_nb_b = ω_ib_b - C_bn * (ω_ie_n + ω_en_n)
    Eigen::Vector3d b_omega_nb = b_omega_ib
                                 - n_Quat_b.conjugate()
                                       * ((c.angularRateEarthRotationCompensationEnabled ? n_omega_ie : Eigen::Vector3d::Zero())
                                          + (c.angularRateTransportRateCompensationEnabled ? n_omega_en : Eigen::Vector3d::Zero()));
    LOG_DATA("b_omega_nb = {} [rad/s]", b_omega_nb.transpose());

    // Coriolis acceleration in [m/s^2] (acceleration due to motion in rotating reference frame)
    Eigen::Vector3d n_coriolisAcceleration = c.coriolisAccelerationCompensationEnabled
                                                 ? n_calcCoriolisAcceleration(n_omega_ie, n_omega_en, y.segment<3>(4))
                                                 : Eigen::Vector3d::Zero();
    LOG_DATA("n_coriolisAcceleration = {} [m/s^2]", n_coriolisAcceleration.transpose());
    // Centrifugal acceleration in [m/s^2] (acceleration that makes a body follow a curved path)
    Eigen::Vector3d n_centrifugalAcceleration = c.centrifgalAccelerationCompensationEnabled
                                                    ? n_Quat_e * e_calcCentrifugalAcceleration(trafo::lla2ecef_WGS84(y.segment<3>(7)), InsConst::e_omega_ie)
                                                    : Eigen::Vector3d::Zero();
    LOG_DATA("n_centrifugalAcceleration = {} [m/s^2]", n_centrifugalAcceleration.transpose());

    Eigen::Vector3d n_gravitation = n_calcGravitation(y.segment<3>(7), c.gravitationModel);
    LOG_DATA("n_gravitation = {} [m/s^2] ({})", n_gravitation.transpose(), to_string(c.gravitationModel));

    y_dot.segment<4>(0) = calcTimeDerivativeFor_n_Quat_b(b_omega_nb,       // ω_nb_b Body rate with respect to the navigation frame, expressed in the body frame
                                                         y.segment<4>(0)); // n_Quat_b_coeffs Coefficients of the quaternion n_Quat_b in order w, x, y, z (q = w + ix + jy + kz)

    y_dot.segment<3>(4) = n_calcTimeDerivativeForVelocity(n_Quat_b * b_measuredForce, // f_n Specific force vector as measured by a triad of accelerometers and resolved into local-navigation frame coordinates
                                                          n_coriolisAcceleration,     // Coriolis acceleration in local-navigation coordinates in [m/s^2]
                                                          n_gravitation,              // Local gravitation vector (caused by effects of mass attraction) in local-navigation frame coordinates [m/s^2]
                                                          n_centrifugalAcceleration); // Centrifugal acceleration in local-navigation coordinates in [m/s^2]

    y_dot.segment<3>(7) = lla_calcTimeDerivativeForPosition(y.segment<3>(4), // Velocity with respect to the Earth in local-navigation frame coordinates [m/s]
                                                            y(7),            // 𝜙 Latitude in [rad]
                                                            y(9),            // h Altitude in [m]
                                                            R_N,             // North/South (meridian) earth radius [m]
                                                            R_E);            // East/West (prime vertical) earth radius [m]

    y_dot.segment<3>(10) = c.b_measuredForce_dot;
    y_dot.segment<3>(13) = c.b_omega_ib_dot;

    LOG_DATA("n_Quat_b_dot = {} ", y_dot.segment<4>(0).transpose());
    LOG_DATA("n_velocity_dot = {} [m/s^2]", y_dot.segment<3>(4).transpose());
    LOG_DATA("lla_position_dot = {} [rad/s, rad/s, m/s]", y_dot.segment<3>(7).transpose());
    LOG_DATA("b_measuredForce_dot = {} [m/s^3]", y_dot.segment<3>(10).transpose());
    LOG_DATA("b_omega_ib_dot = {} [rad/s^2]", y_dot.segment<3>(13).transpose());

    return y_dot;
}

} // namespace NAV