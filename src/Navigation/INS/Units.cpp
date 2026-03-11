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

void to_json(json& j, const ImuAccelerometerRWUnits& data)
{
    j = to_string(data);
}
void from_json(const json& j, ImuAccelerometerRWUnits& data)
{
    if (!j.is_string())
    {
        LOG_WARN("Could not parse '{}' into ImuAccelerometerRWUnits. Consider resaving the flow", j.dump());
        return;
    }
    std::string str = j.get<std::string>();
    for (size_t i = 0; i < static_cast<size_t>(ImuAccelerometerRWUnits::COUNT); i++)
    {
        auto enumItem = static_cast<ImuAccelerometerRWUnits>(i);
        if (str == to_string(enumItem))
        {
            data = enumItem;
            return;
        }
    }
}

void to_json(json& j, const ImuGyroscopeRWUnits& data)
{
    j = to_string(data);
}
void from_json(const json& j, ImuGyroscopeRWUnits& data)
{
    if (!j.is_string())
    {
        LOG_WARN("Could not parse '{}' into ImuGyroscopeRWUnits. Consider resaving the flow", j.dump());
        return;
    }
    std::string str = j.get<std::string>();
    for (size_t i = 0; i < static_cast<size_t>(ImuGyroscopeRWUnits::COUNT); i++)
    {
        auto enumItem = static_cast<ImuGyroscopeRWUnits>(i);
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

void to_json(json& j, const ImuAccelerometerFilterNoiseUnits& data)
{
    j = to_string(data);
}
void from_json(const json& j, ImuAccelerometerFilterNoiseUnits& data)
{
    if (!j.is_string())
    {
        LOG_WARN("Could not parse '{}' into ImuAccelerometerFilterNoiseUnits. Consider resaving the flow", j.dump());
        return;
    }
    std::string str = j.get<std::string>();
    for (size_t i = 0; i < static_cast<size_t>(ImuAccelerometerFilterNoiseUnits::COUNT); i++)
    {
        auto enumItem = static_cast<ImuAccelerometerFilterNoiseUnits>(i);
        if (str == to_string(enumItem))
        {
            data = enumItem;
            return;
        }
    }
}

void to_json(json& j, const ImuGyroscopeFilterNoiseUnits& data)
{
    j = to_string(data);
}
void from_json(const json& j, ImuGyroscopeFilterNoiseUnits& data)
{
    if (!j.is_string())
    {
        LOG_WARN("Could not parse '{}' into ImuGyroscopeFilterNoiseUnits. Consider resaving the flow", j.dump());
        return;
    }
    std::string str = j.get<std::string>();
    for (size_t i = 0; i < static_cast<size_t>(ImuGyroscopeFilterNoiseUnits::COUNT); i++)
    {
        auto enumItem = static_cast<ImuGyroscopeFilterNoiseUnits>(i);
        if (str == to_string(enumItem))
        {
            data = enumItem;
            return;
        }
    }
}

void to_json(json& j, const ImuAccelerometerFilterBiasUnits& data)
{
    j = to_string(data);
}
void from_json(const json& j, ImuAccelerometerFilterBiasUnits& data)
{
    if (!j.is_string())
    {
        LOG_WARN("Could not parse '{}' into ImuAccelerometerFilterBiasUnits. Consider resaving the flow", j.dump());
        return;
    }
    std::string str = j.get<std::string>();
    for (size_t i = 0; i < static_cast<size_t>(ImuAccelerometerFilterBiasUnits::COUNT); i++)
    {
        auto enumItem = static_cast<ImuAccelerometerFilterBiasUnits>(i);
        if (str == to_string(enumItem))
        {
            data = enumItem;
            return;
        }
    }
}

void to_json(json& j, const ImuGyroscopeFilterBiasUnits& data)
{
    j = to_string(data);
}
void from_json(const json& j, ImuGyroscopeFilterBiasUnits& data)
{
    if (!j.is_string())
    {
        LOG_WARN("Could not parse '{}' into ImuGyroscopeFilterBiasUnits. Consider resaving the flow", j.dump());
        return;
    }
    std::string str = j.get<std::string>();
    for (size_t i = 0; i < static_cast<size_t>(ImuGyroscopeFilterBiasUnits::COUNT); i++)
    {
        auto enumItem = static_cast<ImuGyroscopeFilterBiasUnits>(i);
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
    case Units::ImuAccelerometerUnits::g:
        return "g";
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
    case Units::ImuAccelerometerNoiseUnits::m_s_sqrts:
        return "m/s/√(s)";
    case Units::ImuAccelerometerNoiseUnits::m_s_sqrth:
        return "m/s/√(h)";
    case Units::ImuAccelerometerNoiseUnits::COUNT:
        break;
    }
    return "";
}

std::string NAV::to_string(Units::ImuGyroscopeNoiseUnits unit)
{
    switch (unit)
    {
    case Units::ImuGyroscopeNoiseUnits::rad_sqrts:
        return "rad/√(s)";
    case Units::ImuGyroscopeNoiseUnits::rad_sqrth:
        return "rad/√(h)";
    case Units::ImuGyroscopeNoiseUnits::deg_sqrts:
        return "deg/√(s)";
    case Units::ImuGyroscopeNoiseUnits::deg_sqrth:
        return "deg/√(h)";
    case Units::ImuGyroscopeNoiseUnits::COUNT:
        break;
    }
    return "";
}

std::string NAV::to_string(Units::ImuAccelerometerRWUnits unit)
{
    switch (unit)
    {
    case Units::ImuAccelerometerRWUnits::m_s2_sqrts:
        return "m/s^2/√(s)";
    case Units::ImuAccelerometerRWUnits::m_s2_sqrth:
        return "m/s^2/√(h)";
    case Units::ImuAccelerometerRWUnits::COUNT:
        break;
    }
    return "";
}

std::string NAV::to_string(Units::ImuGyroscopeRWUnits unit)
{
    switch (unit)
    {
    case Units::ImuGyroscopeRWUnits::rad_s_sqrts:
        return "rad/s/√(s)";
    case Units::ImuGyroscopeRWUnits::rad_s_sqrth:
        return "rad/s/√(h)";
    case Units::ImuGyroscopeRWUnits::deg_s_sqrts:
        return "deg/s/√(s)";
    case Units::ImuGyroscopeRWUnits::deg_s_sqrth:
        return "deg/s/√(h)";
    case Units::ImuGyroscopeRWUnits::COUNT:
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

std::string NAV::to_string(Units::ImuAccelerometerFilterNoiseUnits unit)
{
    switch (unit)
    {
    case Units::ImuAccelerometerFilterNoiseUnits::m_s2_sqrtHz:
        return "m/s^2/√(Hz)";
    case Units::ImuAccelerometerFilterNoiseUnits::mg_sqrtHz:
        return "mg/√(Hz)";
    case Units::ImuAccelerometerFilterNoiseUnits::COUNT:
        break;
    }
    return "";
}

std::string NAV::to_string(Units::ImuGyroscopeFilterNoiseUnits unit)
{
    switch (unit)
    {
    case Units::ImuGyroscopeFilterNoiseUnits::rad_s_sqrtHz:
        return "rad/s/√(Hz)";
    case Units::ImuGyroscopeFilterNoiseUnits::rad_hr_sqrtHz:
        return "rad/hr/√(Hz)";
    case Units::ImuGyroscopeFilterNoiseUnits::deg_s_sqrtHz:
        return "deg/s/√(Hz)";
    case Units::ImuGyroscopeFilterNoiseUnits::deg_hr_sqrtHz:
        return "deg/hr/√(Hz)";
    case Units::ImuGyroscopeFilterNoiseUnits::COUNT:
        break;
    }
    return "";
}

std::string NAV::to_string(Units::ImuAccelerometerFilterBiasUnits unit)
{
    switch (unit)
    {
    case Units::ImuAccelerometerFilterBiasUnits::m_s2:
        return "m/s^2";
    case Units::ImuAccelerometerFilterBiasUnits::microg:
        return "µg";
    case Units::ImuAccelerometerFilterBiasUnits::COUNT:
        break;
    }
    return "";
}

std::string NAV::to_string(Units::ImuGyroscopeFilterBiasUnits unit)
{
    switch (unit)
    {
    case Units::ImuGyroscopeFilterBiasUnits::rad_s:
        return "1/s";
    case Units::ImuGyroscopeFilterBiasUnits::rad_h:
        return "1/h";
    case Units::ImuGyroscopeFilterBiasUnits::deg_s:
        return "°/s";
    case Units::ImuGyroscopeFilterBiasUnits::deg_h:
        return "°/h";
    case Units::ImuGyroscopeFilterBiasUnits::COUNT:
        break;
    }
    return "";
}