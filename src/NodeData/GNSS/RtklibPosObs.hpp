/// @file RtklibPosObs.hpp
/// @brief RTKLIB Pos Observation Class
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-06-02

#pragma once

#include "NodeData/State/PosVel.hpp"
#include "util/Eigen.hpp"

namespace NAV
{
/// RTKLIB Observation Class
class RtklibPosObs : public PosVel
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
        return { NodeData::type() };
    }

    /// 1:fix, 2:float, 3:sbas, 4:dgps, 5:single, 6:ppp
    std::optional<uint8_t> Q;
    /// Number of satellites
    std::optional<uint8_t> ns;

    /// Standard Deviation XYZ [m]
    Eigen::Vector3d sdXYZ{ std::nan(""), std::nan(""), std::nan("") };
    /// Standard Deviation North East Down [m]
    Eigen::Vector3d sdNED{ std::nan(""), std::nan(""), std::nan("") };
    /// Standard Deviation xy [m]
    double sdxy = std::nan("");
    /// Standard Deviation yz [m]
    double sdyz = std::nan("");
    /// Standard Deviation zx [m]
    double sdzx = std::nan("");
    /// Standard Deviation ne [m]
    double sdne = std::nan("");
    /// Standard Deviation ed [m]
    double sded = std::nan("");
    /// Standard Deviation dn [m]
    double sddn = std::nan("");
    /// Age [s]
    double age = std::nan("");
    /// Ratio
    double ratio = std::nan("");

    /// Standard Deviation velocity NED [m/s]
    Eigen::Vector3d sdvNED{ std::nan(""), std::nan(""), std::nan("") };
    /// Standard Deviation velocity north-east [m/s]
    double sdvne = std::nan("");
    /// Standard Deviation velocity east-down [m/s]
    double sdved = std::nan("");
    /// Standard Deviation velocity down-north [m/s]
    double sdvdn = std::nan("");
};

} // namespace NAV
