/// @file SkydelObs.hpp
/// @brief Parent Class for all observations simulated by Skydel
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2021-06-21

#pragma once

#include "NodeData/InsObs.hpp"

// #include "NodeData/IMU/ImuPos.hpp"
#include "util/Eigen.hpp"

namespace NAV
{
/// Skydel Observation storage class
class SkydelObs : public InsObs
{
  public:
    // explicit SkydelObs();
    // : skydelObs(nullptr) {}

    /// @brief Default constructor
    SkydelObs() = default;
    /// @brief Destructor
    ~SkydelObs() override = default;
    /// @brief Copy constructor
    SkydelObs(const SkydelObs&) = delete;
    /// @brief Move constructor
    SkydelObs(SkydelObs&&) = delete;
    /// @brief Copy assignment operator
    SkydelObs& operator=(const SkydelObs&) = delete;
    /// @brief Move assignment operator
    SkydelObs& operator=(SkydelObs&&) = delete;

    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type()
    {
        return std::string("SkydelObs");
    }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string> parentTypes()
    {
        return { InsObs::type() };
    }

    /// The system time since startup measured in [nano seconds].
    // std::optional<uint64_t> timeSinceStartup;

    /// The Skydel simulated position
    std::optional<Eigen::Vector3d> posXYZ;
    /// The Skydel simulated attitude
    std::optional<Eigen::Vector3d> attRPY;

    // /// The IMU magnetic field measured in units of [Gauss], given in the platform frame.
    // std::optional<Eigen::Vector3d> magUncompXYZ;
    // /// The IMU acceleration measured in units of [m/s^2], given in the platform frame.
    // std::optional<Eigen::Vector3d> accelUncompXYZ;
    // /// The IMU angular rate measured in units of [rad/s], given in the platform frame.
    // std::optional<Eigen::Vector3d> gyroUncompXYZ;

    // /// The compensated magnetic field measured in units of [Gauss], and given in the platform frame.
    // std::optional<Eigen::Vector3d> magCompXYZ;
    // /// The compensated acceleration measured in units of [m/s^2], and given in the platform frame.
    // std::optional<Eigen::Vector3d> accelCompXYZ;
    // /// The compensated angular rate measured in units of [rad/s], and given in the platform frame.
    // std::optional<Eigen::Vector3d> gyroCompXYZ;

    // /// The IMU temperature measured in units of [Celsius].
    // std::optional<double> temperature = 0.0;
};

} // namespace NAV
