#include "Functions.hpp"

#include "Navigation/Math/Math.hpp"

namespace NAV
{

Eigen::Vector3d n_calcTransportRate(const Eigen::Vector3d& latLonAlt, const Eigen::Vector3d& n_velocity, const double& R_N, const double& R_E)
{
    // 𝜙 Latitude in [rad]
    const auto& latitude = latLonAlt(0);
    // h Altitude in [m]
    const auto& altitude = latLonAlt(2);

    // Velocity North in [m/s]
    const auto& v_N = n_velocity(0);
    // Velocity East in [m/s]
    const auto& v_E = n_velocity(1);

    Eigen::Vector3d omega_en_n__t1;
    omega_en_n__t1(0) = v_E / (R_E + altitude);
    omega_en_n__t1(1) = -v_N / (R_N + altitude);
    omega_en_n__t1(2) = -omega_en_n__t1(0) * std::tan(latitude);

    return omega_en_n__t1;
}

Eigen::Vector3d e_calcCentrifugalAcceleration(const Eigen::Vector3d& x_e, const Eigen::Vector3d& omega_ie_e)
{
    // ω_ie_e ⨯ [ω_ie_e ⨯ x_e]
    return omega_ie_e.cross(omega_ie_e.cross(x_e));
}

Eigen::Vector3d n_calcCoriolisAcceleration(const Eigen::Vector3d& n_omega_ie, const Eigen::Vector3d& n_omega_en, const Eigen::Vector3d& n_velocity)
{
    return (2 * n_omega_ie + n_omega_en).cross(n_velocity);
}

double calcRollFromStaticAcceleration(const Eigen::Vector3d& accel_b)
{
    // See E.-H. Shin (2005) - Estimation Techniques for Low-Cost Inertial Navigation (Chapter 2.6 - eq. 2.72a)
    return sgn(accel_b.z()) * std::asin(accel_b.y() / accel_b.norm());

    // Another possible calculation would be:
    // return std::atan2(accel_b.y(), accel_b.z());
}

double calcPitchFromStaticAcceleration(const Eigen::Vector3d& accel_b)
{
    // See E.-H. Shin (2005) - Estimation Techniques for Low-Cost Inertial Navigation (Chapter 2.6 - eq. 2.72b)
    return -sgn(accel_b.z()) * std::asin(accel_b.x() / accel_b.norm());

    // Another possible calculation would be:
    // return std::atan2((-accel_b.x()), sqrt(std::pow(accel_b.y(), 2) + std::pow(accel_b.z(), 2)));
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
