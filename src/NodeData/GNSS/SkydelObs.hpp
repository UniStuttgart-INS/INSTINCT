/// @file SkydelObs.hpp
/// @brief Parent Class for all observations simulated by Skydel
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2021-06-21

#pragma once

#include "NodeData/InsObs.hpp"
#include "util/Eigen.hpp"

namespace NAV
{
/// Skydel Observation storage class
class SkydelObs : public InsObs
{
  public:
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
    std::optional<uint64_t> timeSinceStartup;

    /// The Skydel simulated position in ECEF coordinates in [m] ('PositionObserver Plug-in')
    std::optional<Eigen::Vector3d> posXYZ;
    /// The Skydel simulated attitude (roll, pitch, yaw) in [rad] ('PositionObserver Plug-in')
    std::optional<Eigen::Vector3d> attRPY;
};

} // namespace NAV
