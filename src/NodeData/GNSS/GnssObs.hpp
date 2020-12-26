/// @file GnssObs.hpp
/// @brief Abstract GNSS Observation Class
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-03-19

#pragma once

#include "NodeData/InsObs.hpp"

#include "util/LinearAlgebra.hpp"

namespace NAV
{
/// Abstract GNSS Observation Class
class GnssObs : public InsObs
{
  public:
    /// @brief Default constructor
    GnssObs() = default;
    /// @brief Destructor
    ~GnssObs() override = default;
    /// @brief Copy constructor
    GnssObs(const GnssObs&) = delete;
    /// @brief Move constructor
    GnssObs(GnssObs&&) = delete;
    /// @brief Copy assignment operator
    GnssObs& operator=(const GnssObs&) = delete;
    /// @brief Move assignment operator
    GnssObs& operator=(GnssObs&&) = delete;

    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static constexpr std::string_view type()
    {
        return std::string_view("GnssObs");
    }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string_view> parentTypes()
    {
        std::vector<std::string_view> parents{ "InsObs" };
        return parents;
    }

    /// ECEF position [m]
    std::optional<Vector3d<Earth>> position_ecef;
};
} // namespace NAV
