// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Units.hpp"

#include <cmath>
#include "Navigation/Transformations/Units.hpp"
#include "util/Logger.hpp"

namespace NAV::Units
{

void to_json(json& j, const PositionUncertaintyUnits& data)
{
    j = to_string(data);
}
void from_json(const json& j, PositionUncertaintyUnits& data)
{
    if (!j.is_string())
    {
        LOG_WARN("Could not parse '{}' into PositionUncertaintyUnits. Consider resaving the flow", j.dump());
        return;
    }
    std::string str = j.get<std::string>();
    for (size_t i = 0; i < static_cast<size_t>(PositionUncertaintyUnits::COUNT); i++)
    {
        auto enumItem = static_cast<PositionUncertaintyUnits>(i);
        if (str == to_string(enumItem))
        {
            data = enumItem;
            return;
        }
    }
}

void to_json(json& j, const VelocityUncertaintyUnits& data)
{
    j = to_string(data);
}
void from_json(const json& j, VelocityUncertaintyUnits& data)
{
    if (!j.is_string())
    {
        LOG_WARN("Could not parse '{}' into VelocityUncertaintyUnits. Consider resaving the flow", j.dump());
        return;
    }
    std::string str = j.get<std::string>();
    for (size_t i = 0; i < static_cast<size_t>(VelocityUncertaintyUnits::COUNT); i++)
    {
        auto enumItem = static_cast<VelocityUncertaintyUnits>(i);
        if (str == to_string(enumItem))
        {
            data = enumItem;
            return;
        }
    }
}

void to_json(json& j, const AttitudeUncertaintyUnits& data)
{
    j = to_string(data);
}
void from_json(const json& j, AttitudeUncertaintyUnits& data)
{
    if (!j.is_string())
    {
        LOG_WARN("Could not parse '{}' into AttitudeUncertaintyUnits. Consider resaving the flow", j.dump());
        return;
    }
    std::string str = j.get<std::string>();
    for (size_t i = 0; i < static_cast<size_t>(AttitudeUncertaintyUnits::COUNT); i++)
    {
        auto enumItem = static_cast<AttitudeUncertaintyUnits>(i);
        if (str == to_string(enumItem))
        {
            data = enumItem;
            return;
        }
    }
}

} // namespace NAV::Units

double NAV::convertUnit(const double& value, Units::PositionUncertaintyUnits unit)
{
    switch (unit)
    {
    case Units::PositionUncertaintyUnits::meter:
        return value;
    case Units::PositionUncertaintyUnits::meter2:
        return std::sqrt(value);
    case Units::PositionUncertaintyUnits::COUNT:
        break;
    }
    return value; // Position standard deviation [m]
}
Eigen::Vector3d NAV::convertUnit(const Eigen::Vector3d& value, Units::PositionUncertaintyUnits unit)
{
    switch (unit)
    {
    case Units::PositionUncertaintyUnits::meter:
        return value;
    case Units::PositionUncertaintyUnits::meter2:
        return value.array().sqrt();
    case Units::PositionUncertaintyUnits::COUNT:
        break;
    }
    return value; // Position standard deviation [m]
}

double NAV::convertUnit(const double& value, Units::VelocityUncertaintyUnits unit)
{
    switch (unit)
    {
    case Units::VelocityUncertaintyUnits::m_s:
        return value;
    case Units::VelocityUncertaintyUnits::m2_s2:
        return std::sqrt(value);
    case Units::VelocityUncertaintyUnits::COUNT:
        break;
    }
    return value; // Velocity standard deviation [m/s]
}
Eigen::Vector3d NAV::convertUnit(const Eigen::Vector3d& value, Units::VelocityUncertaintyUnits unit)
{
    switch (unit)
    {
    case Units::VelocityUncertaintyUnits::m_s:
        return value;
    case Units::VelocityUncertaintyUnits::m2_s2:
        return value.array().sqrt();
    case Units::VelocityUncertaintyUnits::COUNT:
        break;
    }
    return value; // Velocity standard deviation [m/s]
}

double NAV::convertUnit(const double& value, Units::AttitudeUncertaintyUnits unit)
{
    switch (unit)
    {
    case Units::AttitudeUncertaintyUnits::rad:
        return value;
    case Units::AttitudeUncertaintyUnits::deg:
        return deg2rad(value);
    case Units::AttitudeUncertaintyUnits::rad2:
        return std::sqrt(value);
    case Units::AttitudeUncertaintyUnits::deg2:
        return deg2rad(std::sqrt(value));
    case Units::AttitudeUncertaintyUnits::COUNT:
        break;
    }
    return value; // Attitude standard deviation [rad]
}
Eigen::Vector3d NAV::convertUnit(const Eigen::Vector3d& value, Units::AttitudeUncertaintyUnits unit)
{
    switch (unit)
    {
    case Units::AttitudeUncertaintyUnits::rad:
        return value;
    case Units::AttitudeUncertaintyUnits::deg:
        return deg2rad(value);
    case Units::AttitudeUncertaintyUnits::rad2:
        return value.array().sqrt();
    case Units::AttitudeUncertaintyUnits::deg2:
        return deg2rad(value.array().sqrt());
    case Units::AttitudeUncertaintyUnits::COUNT:
        break;
    }
    return value; // Attitude standard deviation [rad]
}

std::string NAV::to_string(Units::PositionUncertaintyUnits unit)
{
    switch (unit)
    {
    case Units::PositionUncertaintyUnits::meter2:
        return "m^2";
    case Units::PositionUncertaintyUnits::meter:
        return "m";
    case Units::PositionUncertaintyUnits::COUNT:
        break;
    }
    return "";
}

std::string NAV::to_string(Units::VelocityUncertaintyUnits unit)
{
    switch (unit)
    {
    case Units::VelocityUncertaintyUnits::m2_s2:
        return "m^2/s^2";
    case Units::VelocityUncertaintyUnits::m_s:
        return "m/s";
    case Units::VelocityUncertaintyUnits::COUNT:
        break;
    }
    return "";
}

std::string NAV::to_string(Units::AttitudeUncertaintyUnits unit)
{
    switch (unit)
    {
    case Units::AttitudeUncertaintyUnits::rad:
        return "rad";
    case Units::AttitudeUncertaintyUnits::deg:
        return "deg";
    case Units::AttitudeUncertaintyUnits::rad2:
        return "rad^2";
    case Units::AttitudeUncertaintyUnits::deg2:
        return "deg^2";
    case Units::AttitudeUncertaintyUnits::COUNT:
        break;
    }
    return "";
}