#include "Mechanization.hpp"

namespace NAV
{

Eigen::Vector3d calcTransportRate_n(const Eigen::Vector3d& latLonAlt, const Eigen::Vector3d& velocity_n, const double& R_N, const double& R_E)
{
    // 𝜙 Latitude in [rad]
    const auto& latitude = latLonAlt(0);
    // h Altitude in [m]
    const auto& altitude = latLonAlt(2);

    // Velocity North in [m/s]
    const auto& v_N = velocity_n(0);
    // Velocity East in [m/s]
    const auto& v_E = velocity_n(1);

    Eigen::Vector3d omega_en_n__t1;
    omega_en_n__t1(0) = v_E / (R_E + altitude);
    omega_en_n__t1(1) = -v_N / (R_N + altitude);
    omega_en_n__t1(2) = -omega_en_n__t1(0) * std::tan(latitude);

    return omega_en_n__t1;
}

Eigen::Vector3d calcCentripetalForce_e(const Eigen::Vector3d& x_e, const Eigen::Vector3d& omega_ie_e)
{
    // ω_ie_e ⨯ [ω_ie_e ⨯ x_e]
    return omega_ie_e.cross(omega_ie_e.cross(x_e));
}

Eigen::Vector3d calcCoriolisForce_n(const Eigen::Vector3d& omega_ie_n, const Eigen::Vector3d& omega_en_n, const Eigen::Vector3d& velocity_n)
{
    return (2 * omega_ie_n + omega_en_n).cross(velocity_n);
}

} // namespace NAV
