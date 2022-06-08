/// @file LcKfInsGnssErrors.hpp
/// @brief Loosely-coupled Kalman Filter INS/GNSS errors
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-06-08

#pragma once

#include "util/Eigen.hpp"
#include "NodeData/InsObs.hpp"

namespace NAV
{
/// Loosely-coupled Kalman Filter INS/GNSS errors
class LcKfInsGnssErrors : public InsObs
{
  public:
    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type()
    {
        return "LcKfInsGnssErrors";
    }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string> parentTypes()
    {
        return { InsObs::type() };
    }

    /// δ𝛙_nb_n = [δ𝜑 δ𝜃 δ𝜓]_nb_n The attitude error (roll, pitch, yaw) in navigation coordinates in [rad]
    Eigen::Vector3d n_attitudeError;
    /// δ𝐯_n The velocity error in navigation coordinates in [m/s]
    Eigen::Vector3d n_velocityError;
    /// δ𝐩 = [δ𝜙 δλ δ𝘩] The position error (latitude, longitude, altitude) in [rad, rad, m]
    Eigen::Vector3d lla_positionError;

    /// 𝐛_a The accelerometer bias in body frame in [m/s^2]
    Eigen::Vector3d b_biasAccel{ 0, 0, 0 };
    /// 𝐛_g The gyroscope bias in body frame in [rad/s]
    Eigen::Vector3d b_biasGyro{ 0, 0, 0 };
};

} // namespace NAV
