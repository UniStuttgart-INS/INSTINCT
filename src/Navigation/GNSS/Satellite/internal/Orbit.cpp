// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Orbit.hpp"

namespace NAV
{
Orbit::Pos Orbit::calcSatellitePos(const InsTime& transTime) const
{
    auto posVelAtt = calcSatelliteData(transTime, Calc_Position);
    return { .e_pos = posVelAtt.e_pos };
}

Orbit::PosVel Orbit::calcSatellitePosVel(const InsTime& transTime) const
{
    auto posVelAtt = calcSatelliteData(transTime, Calc_Position | Calc_Velocity);
    return { .e_pos = posVelAtt.e_pos, .e_vel = posVelAtt.e_vel };
}

Orbit::PosVelAccel Orbit::calcSatellitePosVelAccel(const InsTime& transTime) const
{
    return calcSatelliteData(transTime, Calc_Position | Calc_Velocity | Calc_Acceleration);
}

} // namespace NAV
