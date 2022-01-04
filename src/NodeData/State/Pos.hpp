/// @file Pos.hpp
/// @brief Position Storage Class
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-10-27

#pragma once

#include "Navigation/Transformations/CoordinateFrames.hpp"

#include "util/Eigen.hpp"
#include "NodeData/InsObs.hpp"

namespace NAV
{
/// Position, Velocity and Attitude Storage Class
class Pos : public InsObs
{
  public:
    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type()
    {
        return "Pos";
    }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string> parentTypes()
    {
        return { InsObs::type() };
    }

    /* -------------------------------------------------------------------------------------------------------- */
    /*                                           Rotation Quaternions                                           */
    /* -------------------------------------------------------------------------------------------------------- */

    /// @brief Returns the Quaternion from navigation to Earth-fixed frame
    /// @return The Quaternion for the rotation from navigation to earth coordinates
    [[nodiscard]] Eigen::Quaterniond quaternion_en() const
    {
        return trafo::quat_en(latitude(), longitude());
    }

    /// @brief Returns the Quaternion from Earth-fixed frame to navigation
    /// @return The Quaternion for the rotation from earth navigation coordinates
    [[nodiscard]] Eigen::Quaterniond quaternion_ne() const
    {
        return quaternion_en().conjugate();
    }

    /* -------------------------------------------------------------------------------------------------------- */
    /*                                                 Position                                                 */
    /* -------------------------------------------------------------------------------------------------------- */

    /// Returns the latitude 𝜙, longitude λ and altitude (height above ground) in [rad, rad, m]
    [[nodiscard]] const Eigen::Vector3d& latLonAlt() const { return _p_lla; }

    /// Returns the latitude 𝜙 in [rad]
    [[nodiscard]] const double& latitude() const { return latLonAlt()(0); }

    /// Returns the longitude λ in [rad]
    [[nodiscard]] const double& longitude() const { return latLonAlt()(1); }

    /// Returns the altitude (height above ground) in [m]
    [[nodiscard]] const double& altitude() const { return latLonAlt()(2); }

    /// Returns the ECEF coordinates in [m]
    [[nodiscard]] const Eigen::Vector3d& position_ecef() const { return _p_ecef; }

    // ###########################################################################################################
    //                                                  Setter
    // ###########################################################################################################

    /// @brief Set the Position in ecef coordinates
    /// @param[in] pos_ecef New Position in ECEF coordinates
    void setPosition_e(const Eigen::Vector3d& pos_ecef)
    {
        _p_ecef = pos_ecef;
        _p_lla = trafo::ecef2lla_WGS84(pos_ecef);
    }

    /// @brief Set the Position lla object
    /// @param[in] pos_lla New Position in LatLonAlt coordinates
    void setPosition_lla(const Eigen::Vector3d& pos_lla)
    {
        _p_ecef = trafo::lla2ecef_WGS84(pos_lla);
        _p_lla = pos_lla;
    }

    /* -------------------------------------------------------------------------------------------------------- */
    /*                                             Member variables                                             */
    /* -------------------------------------------------------------------------------------------------------- */

  private:
    /// Position in ECEF coordinates [m]
    Eigen::Vector3d _p_ecef{ 0, 0, 0 };
    /// Position in LatLonAlt coordinates [rad, rad, m]
    Eigen::Vector3d _p_lla{ 0, 0, 0 };
};

} // namespace NAV
