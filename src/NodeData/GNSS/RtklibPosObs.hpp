/// @file RtklibPosObs.hpp
/// @brief RTKLIB Pos Observation Class
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-06-02

#pragma once

#include "NodeData/InsObs.hpp"
#include "util/Eigen.hpp"

namespace NAV
{
/// RTKLIB Observation Class
class RtklibPosObs : public InsObs
{
  public:
    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type()
    {
        return "RtklibPosObs";
    }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string> parentTypes()
    {
        return { InsObs::type() };
    }

    /// ECEF position [m]
    std::optional<Eigen::Vector3d> e_position;

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
