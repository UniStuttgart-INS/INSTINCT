// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file PositionInput.hpp
/// @brief Position Input GUI widgets
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-08-14

#pragma once

#include <cstdint>
#include <Eigen/Core>
#include <nlohmann/json.hpp>
using json = nlohmann::json; ///< json namespace

#include "Navigation/Transformations/Units.hpp"
#include "Navigation/Transformations/CoordinateFrames.hpp"

namespace NAV
{
namespace gui::widgets
{

/// Position with Reference frame, used for GUI input
struct PositionWithFrame
{
    /// Reference frames
    enum class ReferenceFrame : uint8_t
    {
        ECEF,  ///< Earth-centered Earth-fixed
        LLA,   ///< Latitude, Longitude, Altitude
        COUNT, ///< Amount of items in the enum
    };

    /// Reference frame used for the input, not for the storage of values
    ReferenceFrame frame = ReferenceFrame::LLA;
    /// Position in ECEF coordinates in [m]
    Eigen::Vector3d e_position = trafo::lla2ecef_WGS84(Eigen::Vector3d::Zero());

    /// Latitude in [rad]
    [[nodiscard]] double latitude() const { return trafo::ecef2lla_WGS84(e_position)(0); }
    /// Longitude in [rad]
    [[nodiscard]] double longitude() const { return trafo::ecef2lla_WGS84(e_position)(1); }
    /// Altitude in [m]
    [[nodiscard]] double altitude() const { return trafo::ecef2lla_WGS84(e_position)(2); }
    /// Latitude in [deg]
    [[nodiscard]] double latitude_deg() const { return trafo::ecef2lla_WGS84(e_position)(0); }
    /// Longitude in [deg]
    [[nodiscard]] double longitude_deg() const { return trafo::ecef2lla_WGS84(e_position)(1); }

    /// Latitude in [rad, rad, m]
    [[nodiscard]] Eigen::Vector3d latLonAlt() const { return trafo::ecef2lla_WGS84(e_position); }
    /// Latitude in [deg, deg, m]
    [[nodiscard]] Eigen::Vector3d latLonAlt_deg() const
    {
        auto lla = trafo::ecef2lla_WGS84(e_position);
        return { rad2deg(lla(0)), rad2deg(lla(1)), lla(2) };
    }
};

/// @brief Converts the provided Object into a json object
/// @param[out] j Return Json object
/// @param[in] refFrame Object to convert
void to_json(json& j, const PositionWithFrame::ReferenceFrame& refFrame);
/// @brief Converts the provided json object
/// @param[in] j Json object with the time system
/// @param[out] refFrame Object to return
void from_json(const json& j, PositionWithFrame::ReferenceFrame& refFrame);
/// @brief Converts the provided Object into a json object
/// @param[out] j Return Json object
/// @param[in] position Object to convert
void to_json(json& j, const PositionWithFrame& position);
/// @brief Converts the provided json object
/// @param[in] j Json object with the time system
/// @param[out] position Object to return
void from_json(const json& j, PositionWithFrame& position);

/// Layout options for the Position input
enum class PositionInputLayout : uint8_t
{
    SINGLE_COLUMN, ///< All elements in a single column
    SINGLE_ROW,    ///< All elements in a single row
    TWO_ROWS,      ///< 2 rows
};

/// @brief Inputs to edit an Position object
/// @param[in] str Text to display near the Frame selection (Unique id for the ImGui elements)
/// @param[in, out] position Position and reference frame object to modify
/// @param[in] layout Layout to use
/// @param[in] itemWidth Width of the widget items
/// @return True if changes were made to the object
bool PositionInput(const char* str, PositionWithFrame& position, PositionInputLayout layout = PositionInputLayout::SINGLE_COLUMN, float itemWidth = 140.0F);

} // namespace gui::widgets

/// @brief Converts the enum to a string
/// @param[in] refFrame Enum value to convert into text
/// @return String representation of the enum
const char* to_string(gui::widgets::PositionWithFrame::ReferenceFrame refFrame);

} // namespace NAV
