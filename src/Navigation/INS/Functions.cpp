// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Functions.hpp"

#include "Navigation/Math/Math.hpp"

namespace NAV
{

Eigen::Vector3d n_calcTransportRate(const Eigen::Vector3d& lla_position, const Eigen::Vector3d& n_velocity, const double& R_N, const double& R_E)
{
    // 𝜙 Latitude in [rad]
    const auto& latitude = lla_position(0);
    // h Altitude in [m]
    const auto& altitude = lla_position(2);

    // Velocity North in [m/s]
    const auto& v_N = n_velocity(0);
    // Velocity East in [m/s]
    const auto& v_E = n_velocity(1);

    Eigen::Vector3d n_omega_en__t1;
    n_omega_en__t1(0) = v_E / (R_E + altitude);
    n_omega_en__t1(1) = -v_N / (R_N + altitude);
    n_omega_en__t1(2) = -n_omega_en__t1(0) * std::tan(latitude);

    return n_omega_en__t1;
}

Eigen::Vector3d e_calcCentrifugalAcceleration(const Eigen::Vector3d& e_position, const Eigen::Vector3d& e_omega_ie)
{
    // ω_ie_e ⨯ [ω_ie_e ⨯ x_e]
    return e_omega_ie.cross(e_omega_ie.cross(e_position));
}

Eigen::Vector3d n_calcCoriolisAcceleration(const Eigen::Vector3d& n_omega_ie, const Eigen::Vector3d& n_omega_en, const Eigen::Vector3d& n_velocity)
{
    return (2 * n_omega_ie + n_omega_en).cross(n_velocity);
}

Eigen::Vector3d e_calcCoriolisAcceleration(const Eigen::Vector3d& e_omega_ie, const Eigen::Vector3d& e_velocity)
{
    return (2 * e_omega_ie).cross(e_velocity);
}

double calcRollFromStaticAcceleration(const Eigen::Vector3d& b_accel)
{
    // See E.-H. Shin (2005) - Estimation Techniques for Low-Cost Inertial Navigation (Chapter 2.6 - eq. 2.72a)
    return math::sgn(b_accel.z()) * std::asin(b_accel.y() / b_accel.norm());

    // Another possible calculation would be:
    // return std::atan2(b_accel.y(), b_accel.z());
}

double calcPitchFromStaticAcceleration(const Eigen::Vector3d& b_accel)
{
    // See E.-H. Shin (2005) - Estimation Techniques for Low-Cost Inertial Navigation (Chapter 2.6 - eq. 2.72b)
    return -math::sgn(b_accel.z()) * std::asin(b_accel.x() / b_accel.norm());

    // Another possible calculation would be:
    // return std::atan2((-b_accel.x()), sqrt(std::pow(b_accel.y(), 2) + std::pow(b_accel.z(), 2)));
}

double calcYawFromVelocity(const Eigen::Vector3d& n_velocity)
{
    return std::atan2(n_velocity(1), n_velocity(0));
}

double calcPitchFromVelocity(const Eigen::Vector3d& n_velocity)
{
    return std::atan(-n_velocity(2) / std::sqrt(std::pow(n_velocity(0), 2) + std::pow(n_velocity(1), 2)));
}

} // namespace NAV
