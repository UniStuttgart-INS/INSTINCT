// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Units.hpp"

#include "util/Logger.hpp"

namespace NAV::Units
{

void to_json(json& j, const ImuAccelerometerUnits& data)
{
    j = to_string(data);
}
void from_json(const json& j, ImuAccelerometerUnits& data)
{
    if (!j.is_string())
    {
        LOG_WARN("Could not parse '{}' into ImuAccelerometerUnits. Consider resaving the flow", j.dump());
        return;
    }
    std::string str = j.get<std::string>();
    for (size_t i = 0; i < static_cast<size_t>(ImuAccelerometerUnits::COUNT); i++)
    {
        auto enumItem = static_cast<ImuAccelerometerUnits>(i);
        if (str == to_string(enumItem))
        {
            data = enumItem;
            return;
        }
    }
}

void to_json(json& j, const ImuGyroscopeUnits& data)
{
    j = to_string(data);
}
void from_json(const json& j, ImuGyroscopeUnits& data)
{
    if (!j.is_string())
    {
        LOG_WARN("Could not parse '{}' into ImuGyroscopeUnits. Consider resaving the flow", j.dump());
        return;
    }
    std::string str = j.get<std::string>();
    for (size_t i = 0; i < static_cast<size_t>(ImuGyroscopeUnits::COUNT); i++)
    {
        auto enumItem = static_cast<ImuGyroscopeUnits>(i);
        if (str == to_string(enumItem))
        {
            data = enumItem;
            return;
        }
    }
}

void to_json(json& j, const ImuAccelerometerNoiseUnits& data)
{
    j = to_string(data);
}
void from_json(const json& j, ImuAccelerometerNoiseUnits& data)
{
    if (!j.is_string())
    {
        LOG_WARN("Could not parse '{}' into ImuAccelerometerNoiseUnits. Consider resaving the flow", j.dump());
        return;
    }
    std::string str = j.get<std::string>();
    for (size_t i = 0; i < static_cast<size_t>(ImuAccelerometerNoiseUnits::COUNT); i++)
    {
        auto enumItem = static_cast<ImuAccelerometerNoiseUnits>(i);
        if (str == to_string(enumItem))
        {
            data = enumItem;
            return;
        }
    }
}

void to_json(json& j, const ImuGyroscopeNoiseUnits& data)
{
    j = to_string(data);
}
void from_json(const json& j, ImuGyroscopeNoiseUnits& data)
{
    if (!j.is_string())
    {
        LOG_WARN("Could not parse '{}' into ImuGyroscopeNoiseUnits. Consider resaving the flow", j.dump());
        return;
    }
    std::string str = j.get<std::string>();
    for (size_t i = 0; i < static_cast<size_t>(ImuGyroscopeNoiseUnits::COUNT); i++)
    {
        auto enumItem = static_cast<ImuGyroscopeNoiseUnits>(i);
        if (str == to_string(enumItem))
        {
            data = enumItem;
            return;
        }
    }
}

void to_json(json& j, const ImuAccelerometerIRWUnits& data)
{
    j = to_string(data);
}
void from_json(const json& j, ImuAccelerometerIRWUnits& data)
{
    if (!j.is_string())
    {
        LOG_WARN("Could not parse '{}' into ImuAccelerometerIRWUnits. Consider resaving the flow", j.dump());
        return;
    }
    std::string str = j.get<std::string>();
    for (size_t i = 0; i < static_cast<size_t>(ImuAccelerometerIRWUnits::COUNT); i++)
    {
        auto enumItem = static_cast<ImuAccelerometerIRWUnits>(i);
        if (str == to_string(enumItem))
        {
            data = enumItem;
            return;
        }
    }
}

void to_json(json& j, const ImuGyroscopeIRWUnits& data)
{
    j = to_string(data);
}
void from_json(const json& j, ImuGyroscopeIRWUnits& data)
{
    if (!j.is_string())
    {
        LOG_WARN("Could not parse '{}' into ImuGyroscopeIRWUnits. Consider resaving the flow", j.dump());
        return;
    }
    std::string str = j.get<std::string>();
    for (size_t i = 0; i < static_cast<size_t>(ImuGyroscopeIRWUnits::COUNT); i++)
    {
        auto enumItem = static_cast<ImuGyroscopeIRWUnits>(i);
        if (str == to_string(enumItem))
        {
            data = enumItem;
            return;
        }
    }
}

} // namespace NAV::Units

std::string NAV::to_string(Units::ImuAccelerometerUnits unit)
{
    switch (unit)
    {
    case Units::ImuAccelerometerUnits::m_s2:
        return "m/s^2";
    case Units::ImuAccelerometerUnits::COUNT:
        break;
    }
    return "";
}

std::string NAV::to_string(Units::ImuGyroscopeUnits unit)
{
    switch (unit)
    {
    case Units::ImuGyroscopeUnits::rad_s:
        return "rad/s";
    case Units::ImuGyroscopeUnits::deg_s:
        return "deg/s";
    case Units::ImuGyroscopeUnits::COUNT:
        break;
    }
    return "";
}

std::string NAV::to_string(Units::ImuAccelerometerNoiseUnits unit)
{
    switch (unit)
    {
    case Units::ImuAccelerometerNoiseUnits::m_s2_sqrts:
        return "m/s^2/√(s)";
    case Units::ImuAccelerometerNoiseUnits::m_s2_sqrth:
        return "m/s^2/√(h)";
    case Units::ImuAccelerometerNoiseUnits::COUNT:
        break;
    }
    return "";
}

std::string NAV::to_string(Units::ImuGyroscopeNoiseUnits unit)
{
    switch (unit)
    {
    case Units::ImuGyroscopeNoiseUnits::rad_s_sqrts:
        return "rad/s/√(s)";
    case Units::ImuGyroscopeNoiseUnits::rad_s_sqrth:
        return "rad/s/√(h)";
    case Units::ImuGyroscopeNoiseUnits::deg_s_sqrts:
        return "deg/s/√(s)";
    case Units::ImuGyroscopeNoiseUnits::deg_s_sqrth:
        return "deg/s/√(h)";
    case Units::ImuGyroscopeNoiseUnits::COUNT:
        break;
    }
    return "";
}

std::string NAV::to_string(Units::ImuAccelerometerIRWUnits unit)
{
    switch (unit)
    {
    case Units::ImuAccelerometerIRWUnits::m_s3_sqrts:
        return "m/s^3/√(s)";
    case Units::ImuAccelerometerIRWUnits::m_s3_sqrth:
        return "m/s^3/√(h)";
    case Units::ImuAccelerometerIRWUnits::COUNT:
        break;
    }
    return "";
}

std::string NAV::to_string(Units::ImuGyroscopeIRWUnits unit)
{
    switch (unit)
    {
    case Units::ImuGyroscopeIRWUnits::rad_s2_sqrts:
        return "rad/s^2/√(s)";
    case Units::ImuGyroscopeIRWUnits::rad_s2_sqrth:
        return "rad/s^2/√(h)";
    case Units::ImuGyroscopeIRWUnits::deg_s2_sqrts:
        return "deg/s^2/√(s)";
    case Units::ImuGyroscopeIRWUnits::deg_s2_sqrth:
        return "deg/s^2/√(h)";
    case Units::ImuGyroscopeIRWUnits::COUNT:
        break;
    }
    return "";
}