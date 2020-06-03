/**
 * @file RtklibPosObs.hpp
 * @brief RTKLIB Pos Observation Class
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-06-02
 */

#pragma once

#include "GnssObs.hpp"

#include "Eigen/Dense"
#include <Eigen/Geometry>

namespace NAV
{
/// RTKLIB Observation Class
class RtklibPosObs : public GnssObs
{
  public:
    RtklibPosObs() = default;                              ///< Constructor
    ~RtklibPosObs() override = default;                    ///< Destructor
    RtklibPosObs(const RtklibPosObs&) = delete;            ///< Copy constructor
    RtklibPosObs(RtklibPosObs&&) = delete;                 ///< Move constructor
    RtklibPosObs& operator=(const RtklibPosObs&) = delete; ///< Copy assignment operator
    RtklibPosObs& operator=(RtklibPosObs&&) = delete;      ///< Move assignment operator

    /**
     * @brief Returns the type of the data class
     * 
     * @retval constexpr std::string_view The data type
     */
    [[nodiscard]] constexpr std::string_view type() const override
    {
        return std::string_view("RtklibPosObs");
    }

    /**
     * @brief Returns the parent types of the data class
     * 
     * @retval std::vector<std::string_view> The parent data types
     */
    [[nodiscard]] std::vector<std::string_view> parentTypes() const override
    {
        std::vector<std::string_view> parents{ "GnssObs" };
        return parents;
    }

    /// ECEF position [m]
    std::optional<Eigen::Vector3d> positionXYZ;
    /// Position in Longitude [deg], Latitude [deg], height [m]
    std::optional<Eigen::Array3d> positionLLH;
    /// 1:fix, 2:float, 3:sbas, 4:dgps, 5:single, 6:ppp
    std::optional<uint8_t> Q;
    /// Number of satellites
    std::optional<uint8_t> ns;

    /// [m]
    std::optional<Eigen::Vector3d> sdXYZ;
    /// [m]
    std::optional<Eigen::Vector3d> sdNEU;
    /// [m]
    std::optional<double> sdxy;
    /// [m]
    std::optional<double> sdyz;
    /// [m]
    std::optional<double> sdzx;
    /// [m]
    std::optional<double> sdne;
    /// [m]
    std::optional<double> sdeu;
    /// [m]
    std::optional<double> sdun;
    /// [s]
    std::optional<double> age;
    ///
    std::optional<double> ratio;
};

} // namespace NAV
