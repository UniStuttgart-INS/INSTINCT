/// @file RtklibPosObs.hpp
/// @brief RTKLIB Pos Observation Class
/// @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
/// @date 2020-06-02

#pragma once

#include "GnssObs.hpp"

namespace NAV
{
/// RTKLIB Observation Class
class RtklibPosObs : public GnssObs
{
  public:
    /// @brief Default constructor
    RtklibPosObs() = default;
    /// @brief Destructor
    ~RtklibPosObs() override = default;
    /// @brief Copy constructor
    RtklibPosObs(const RtklibPosObs&) = delete;
    /// @brief Move constructor
    RtklibPosObs(RtklibPosObs&&) = delete;
    /// @brief Copy assignment operator
    RtklibPosObs& operator=(const RtklibPosObs&) = delete;
    /// @brief Move assignment operator
    RtklibPosObs& operator=(RtklibPosObs&&) = delete;

    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] constexpr std::string_view type() const override
    {
        return std::string_view("RtklibPosObs");
    }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] std::vector<std::string_view> parentTypes() const override
    {
        std::vector<std::string_view> parents{ "GnssObs" };
        return parents;
    }
    /// 1:fix, 2:float, 3:sbas, 4:dgps, 5:single, 6:ppp
    std::optional<uint8_t> Q;
    /// Number of satellites
    std::optional<uint8_t> ns;

    /// Standard Deviation XYZ [m]
    std::optional<Eigen::Vector3d> sdXYZ;
    /// Standard Deviation North East Down [m]
    std::optional<Eigen::Vector3d> sdNEU;
    /// Standard Deviation xy [m]
    std::optional<double> sdxy;
    /// Standard Deviation yz [m]
    std::optional<double> sdyz;
    /// Standard Deviation zx [m]
    std::optional<double> sdzx;
    /// Standard Deviation ne [m]
    std::optional<double> sdne;
    /// Standard Deviation eu [m]
    std::optional<double> sdeu;
    /// Standard Deviation un [m]
    std::optional<double> sdun;
    /// Age [s]
    std::optional<double> age;
    /// Ratio
    std::optional<double> ratio;
};

} // namespace NAV
