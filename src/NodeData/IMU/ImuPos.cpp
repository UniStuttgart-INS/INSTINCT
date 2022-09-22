// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ImuPos.hpp"

#include <nlohmann/json.hpp>
using json = nlohmann::json; ///< json namespace

namespace NAV
{
/// @brief Converts the provided imu position into a json object
/// @param[out] j Json object which gets filled with the info
/// @param[in] pos Imu position to convert into json
void to_json(json& j, const ImuPos& pos)
{
    j = json{
        { "b_positionAccel", pos.b_positionAccel() },
        { "b_positionGyro", pos.b_positionGyro() },
        { "b_positionMag", pos.b_positionMag() },
        { "b_quatAccel_p", pos.b_quatAccel_p().coeffs() },
        { "b_quatGyro_p", pos.b_quatGyro_p().coeffs() },
        { "b_quatMag_p", pos.b_quatMag_p().coeffs() },
    };
}
/// @brief Converts the provided json object into a imu position object
/// @param[in] j Json object with the needed values
/// @param[out] pos Object to fill from the json
void from_json(const json& j, ImuPos& pos)
{
    if (j.contains("b_positionAccel"))
    {
        j.at("b_positionAccel").get_to(pos._b_positionAccel);
    }
    if (j.contains("b_positionGyro"))
    {
        j.at("b_positionGyro").get_to(pos._b_positionGyro);
    }
    if (j.contains("b_positionMag"))
    {
        j.at("b_positionMag").get_to(pos._b_positionMag);
    }
    if (j.contains("b_quatAccel_p"))
    {
        j.at("b_quatAccel_p").get_to(pos._b_quatAccel_p.coeffs());
    }
    if (j.contains("b_quatGyro_p"))
    {
        j.at("b_quatGyro_p").get_to(pos._b_quatGyro_p.coeffs());
    }
    if (j.contains("b_quatMag_p"))
    {
        j.at("b_quatMag_p").get_to(pos._b_quatMag_p.coeffs());
    }
}

} // namespace NAV
