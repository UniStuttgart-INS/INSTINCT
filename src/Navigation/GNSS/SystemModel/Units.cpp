// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Units.hpp"

#include <cmath>

namespace NAV::Units
{

void to_json(json& j, const CovarianceAccelUnits& data)
{
    j = to_string(data);
}
void from_json(const json& j, CovarianceAccelUnits& data)
{
    std::string str = j.get<std::string>();
    for (size_t i = 0; i < static_cast<size_t>(CovarianceAccelUnits::COUNT); i++)
    {
        auto enumItem = static_cast<CovarianceAccelUnits>(i);
        if (str == to_string(enumItem))
        {
            data = enumItem;
            return;
        }
    }
}

void to_json(json& j, const CovarianceClkPhaseDriftUnits& data)
{
    j = to_string(data);
}
void from_json(const json& j, CovarianceClkPhaseDriftUnits& data)
{
    std::string str = j.get<std::string>();
    for (size_t i = 0; i < static_cast<size_t>(CovarianceClkPhaseDriftUnits::COUNT); i++)
    {
        auto enumItem = static_cast<CovarianceClkPhaseDriftUnits>(i);
        if (str == to_string(enumItem))
        {
            data = enumItem;
            return;
        }
    }
}

void to_json(json& j, const CovarianceClkFrequencyDriftUnits& data)
{
    j = to_string(data);
}
void from_json(const json& j, CovarianceClkFrequencyDriftUnits& data)
{
    std::string str = j.get<std::string>();
    for (size_t i = 0; i < static_cast<size_t>(CovarianceClkFrequencyDriftUnits::COUNT); i++)
    {
        auto enumItem = static_cast<CovarianceClkFrequencyDriftUnits>(i);
        if (str == to_string(enumItem))
        {
            data = enumItem;
            return;
        }
    }
}

} // namespace NAV::Units

double NAV::convertUnit(const double& value, Units::CovarianceAccelUnits unit)
{
    // Covariance of the acceleration 𝜎_a due to user motion in horizontal and vertical component [m²/s³]
    switch (unit)
    {
    case Units::CovarianceAccelUnits::m2_s3:
        return value;
    case Units::CovarianceAccelUnits::m_sqrts3:
        return std::pow(value, 2);
    case Units::CovarianceAccelUnits::COUNT:
        break;
    }
    return value;
}

double NAV::convertUnit(const double& value, Units::CovarianceClkPhaseDriftUnits unit)
{
    // Covariance of the clock phase drift [m²/s]
    switch (unit)
    {
    case Units::CovarianceClkPhaseDriftUnits::m2_s:
        return value;
    case Units::CovarianceClkPhaseDriftUnits::m_sqrts:
        return std::pow(value, 2);
    case Units::CovarianceClkPhaseDriftUnits::COUNT:
        break;
    }
    return value;
}

double NAV::convertUnit(const double& value, Units::CovarianceClkFrequencyDriftUnits unit)
{
    // Covariance of the frequency phase drift [m²/s³]
    switch (unit)
    {
    case Units::CovarianceClkFrequencyDriftUnits::m2_s3:
        return value;
    case Units::CovarianceClkFrequencyDriftUnits::m_sqrts3:
        return std::pow(value, 2);
    case Units::CovarianceClkFrequencyDriftUnits::COUNT:
        break;
    }
    return value;
}

std::string NAV::to_string(Units::CovarianceAccelUnits unit)
{
    switch (unit)
    {
    case Units::CovarianceAccelUnits::m2_s3:
        return "m^2/s^3";
    case Units::CovarianceAccelUnits::m_sqrts3:
        return "m/√(s^3)";
    case Units::CovarianceAccelUnits::COUNT:
        break;
    }
    return "";
}

std::string NAV::to_string(Units::CovarianceClkPhaseDriftUnits unit)
{
    switch (unit)
    {
    case Units::CovarianceClkPhaseDriftUnits::m2_s:
        return "m^2/s";
    case Units::CovarianceClkPhaseDriftUnits::m_sqrts:
        return "m/√s";
    case Units::CovarianceClkPhaseDriftUnits::COUNT:
        break;
    }
    return "";
}

std::string NAV::to_string(Units::CovarianceClkFrequencyDriftUnits unit)
{
    switch (unit)
    {
    case Units::CovarianceClkFrequencyDriftUnits::m2_s3:
        return "m^2/s^3";
    case Units::CovarianceClkFrequencyDriftUnits::m_sqrts3:
        return "m/√(s^3)";
    case Units::CovarianceClkFrequencyDriftUnits::COUNT:
        break;
    }
    return "";
}