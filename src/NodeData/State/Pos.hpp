// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Pos.hpp
/// @brief Position Storage Class
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-10-27

#pragma once

#include "Navigation/Transformations/CoordinateFrames.hpp"
#include "Navigation/Transformations/Units.hpp"

#include "util/Eigen.hpp"
#include "util/Logger/CommonLog.hpp"
#include "NodeData/NodeData.hpp"

namespace NAV
{
/// Position, Velocity and Attitude Storage Class
class Pos : public NodeData
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
        return { NodeData::type() };
    }

    /// @brief Returns a vector of data descriptors
    [[nodiscard]] static std::vector<std::string> GetStaticDataDescriptors()
    {
        return {
            "Latitude [deg]",
            "Longitude [deg]",
            "Altitude [m]",
            "North/South [m]",
            "East/West [m]",
            "X-ECEF [m]",
            "Y-ECEF [m]",
            "Z-ECEF [m]",
        };
    }

    /// @brief Get the amount of descriptors
    [[nodiscard]] static constexpr size_t GetStaticDescriptorCount() { return 8; }

    /// @brief Returns a vector of data descriptors
    [[nodiscard]] std::vector<std::string> staticDataDescriptors() const override { return GetStaticDataDescriptors(); }

    /// @brief Get the amount of descriptors
    [[nodiscard]] size_t staticDescriptorCount() const override { return GetStaticDescriptorCount(); }

    /// @brief Get the value at the index
    /// @param idx Index corresponding to data descriptor order
    /// @return Value if in the observation
    [[nodiscard]] std::optional<double> getValueAt(size_t idx) const override
    {
        INS_ASSERT(idx < GetStaticDescriptorCount());
        switch (idx)
        {
        case 0: // Latitude [deg]
            return rad2deg(latitude());
        case 1: // Longitude [deg]
            return rad2deg(longitude());
        case 2: // Altitude [m]
            return altitude();
        case 3: // North/South [m]
            return CommonLog::calcLocalPosition(lla_position()).northSouth;
        case 4: // East/West [m]
            return CommonLog::calcLocalPosition(lla_position()).eastWest;
        case 5: // X-ECEF [m]
            return e_position().x();
        case 6: // Y-ECEF [m]
            return e_position().y();
        case 7: // Z-ECEF [m]
            return e_position().z();
        default:
            return std::nullopt;
        }
    }

    /* -------------------------------------------------------------------------------------------------------- */
    /*                                           Rotation Quaternions                                           */
    /* -------------------------------------------------------------------------------------------------------- */

    /// @brief Returns the Quaternion from navigation to Earth-fixed frame
    /// @return The Quaternion for the rotation from navigation to earth coordinates
    [[nodiscard]] Eigen::Quaterniond e_Quat_n() const
    {
        return trafo::e_Quat_n(latitude(), longitude());
    }

    /// @brief Returns the Quaternion from Earth-fixed frame to navigation
    /// @return The Quaternion for the rotation from earth navigation coordinates
    [[nodiscard]] Eigen::Quaterniond n_Quat_e() const
    {
        return e_Quat_n().conjugate();
    }

    /* -------------------------------------------------------------------------------------------------------- */
    /*                                                 Position                                                 */
    /* -------------------------------------------------------------------------------------------------------- */

    /// Returns the latitude 𝜙, longitude λ and altitude (height above ground) in [rad, rad, m]
    [[nodiscard]] const Eigen::Vector3d& lla_position() const { return _lla_position; }

    /// Returns the latitude 𝜙 in [rad]
    [[nodiscard]] const double& latitude() const { return lla_position()(0); }

    /// Returns the longitude λ in [rad]
    [[nodiscard]] const double& longitude() const { return lla_position()(1); }

    /// Returns the altitude (height above ground) in [m]
    [[nodiscard]] const double& altitude() const { return lla_position()(2); }

    /// Returns the  coordinates in [m]
    [[nodiscard]] const Eigen::Vector3d& e_position() const { return _e_position; }

    // ###########################################################################################################
    //                                                  Setter
    // ###########################################################################################################

    /// @brief Set the Position in  coordinates
    /// @param[in] e_position New Position in ECEF coordinates
    void setPosition_e(const Eigen::Vector3d& e_position)
    {
        _e_position = e_position;
        _lla_position = trafo::ecef2lla_WGS84(e_position);
    }

    /// @brief Set the Position lla object
    /// @param[in] lla_position New Position in LatLonAlt coordinates
    void setPosition_lla(const Eigen::Vector3d& lla_position)
    {
        _e_position = trafo::lla2ecef_WGS84(lla_position);
        _lla_position = lla_position;
    }

    /* -------------------------------------------------------------------------------------------------------- */
    /*                                             Member variables                                             */
    /* -------------------------------------------------------------------------------------------------------- */

  private:
    /// Position in ECEF coordinates [m]
    Eigen::Vector3d _e_position{ std::nan(""), std::nan(""), std::nan("") };
    /// Position in LatLonAlt coordinates [rad, rad, m]
    Eigen::Vector3d _lla_position{ std::nan(""), std::nan(""), std::nan("") };
};

} // namespace NAV
