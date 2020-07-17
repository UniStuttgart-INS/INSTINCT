/**
 * @file ImuObs.hpp
 * @brief Parent Class for all IMU Observations
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-03-12
 */

#pragma once

#include "NodeData/InsObs.hpp"

#include "Eigen/Dense"
#include <Eigen/Geometry>

namespace NAV
{
/// IMU Observation storage class
class ImuObs : public InsObs
{
  public:
    ImuObs() = default;                        ///< Constructor
    ~ImuObs() override = default;              ///< Destructor
    ImuObs(const ImuObs&) = delete;            ///< Copy constructor
    ImuObs(ImuObs&&) = delete;                 ///< Move constructor
    ImuObs& operator=(const ImuObs&) = delete; ///< Copy assignment operator
    ImuObs& operator=(ImuObs&&) = delete;      ///< Move assignment operator

    /**
     * @brief Returns the type of the data class
     * 
     * @retval constexpr std::string_view The data type
     */
    [[nodiscard]] constexpr std::string_view type() const override
    {
        return std::string_view("ImuObs");
    }

    /**
     * @brief Returns the parent types of the data class
     * 
     * @retval std::vector<std::string_view> The parent data types
     */
    [[nodiscard]] std::vector<std::string_view> parentTypes() const override
    {
        std::vector<std::string_view> parents{ "InsObs" };
        return parents;
    }

    /** The system time since startup measured in [nano seconds]. */
    std::optional<uint64_t> timeSinceStartup;

    /// The IMU magnetic field measured in units of [Gauss], given in the body frame.
    std::optional<Eigen::Vector3d> magUncompXYZ;
    /// The IMU acceleration measured in units of [m/s^2], given in the body frame.
    std::optional<Eigen::Vector3d> accelUncompXYZ;
    /// The IMU angular rate measured in units of [rad/s], given in the body frame.
    std::optional<Eigen::Vector3d> gyroUncompXYZ;

    /// The IMU temperature measured in units of [Celsius].
    std::optional<double> temperature = 0.0;
};

} // namespace NAV
