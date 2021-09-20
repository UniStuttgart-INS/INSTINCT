/// @file LCKFState.hpp
/// @brief Imu biases estimations to correct IMU observations
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-08-31

#pragma once

#include "util/Eigen.hpp"
#include "NodeData/InsObs.hpp"

namespace NAV
{
/// Position, Velocity and Attitude Storage Class
class ImuBiases : public InsObs
{
  public:
    /// @brief Default constructor
    ImuBiases() = default;
    /// @brief Destructor
    ~ImuBiases() override = default;
    /// @brief Copy constructor
    ImuBiases(const ImuBiases&) = delete;
    /// @brief Move constructor
    ImuBiases(ImuBiases&&) = delete;
    /// @brief Copy assignment operator
    ImuBiases& operator=(const ImuBiases&) = delete;
    /// @brief Move assignment operator
    ImuBiases& operator=(ImuBiases&&) = delete;

    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type()
    {
        return std::string("ImuBiases");
    }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string> parentTypes()
    {
        return { InsObs::type() };
    }

    /// 𝐛_a The accelerometer bias in platform frame in [m/s^2]
    Eigen::Vector3d biasAccel_p{ 0, 0, 0 };
    /// 𝐛_g The gyroscope bias in platform frame in [rad/s]
    Eigen::Vector3d biasGyro_p{ 0, 0, 0 };
};

} // namespace NAV
