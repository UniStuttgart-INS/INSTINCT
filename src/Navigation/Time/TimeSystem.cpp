// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file TimeSystem.cpp
/// @brief Time
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-08-10

#include "TimeSystem.hpp"

#include "internal/gui/widgets/EnumCombo.hpp"

namespace NAV
{
TimeSystem::TimeSystemEnum TimeSystem::getEnumValue() const
{
    if (value & TimeSys_None) { return TimeSystemEnum::TimeSys_None; }
    if (value & UTC) { return TimeSystemEnum::UTC; }
    if (value & GPST) { return TimeSystemEnum::GPST; }
    if (value & GLNT) { return TimeSystemEnum::GLNT; }
    if (value & GST) { return TimeSystemEnum::GST; }
    if (value & BDT) { return TimeSystemEnum::BDT; }
    if (value & QZSST) { return TimeSystemEnum::QZSST; }
    if (value & IRNSST) { return TimeSystemEnum::IRNSST; }

    return TimeSystemEnum::TimeSys_None;
}

void to_json(json& j, const TimeSystem& timeSystem)
{
    j = std::string(timeSystem);
}

void from_json(const json& j, TimeSystem& timeSystem)
{
    timeSystem = TimeSystem::fromString(j.get<std::string>());
}

bool ComboTimeSystem(const char* label, TimeSystem& timeSystem)
{
    auto enumeration = timeSystem.getEnumValue();
    bool changed = gui::widgets::EnumCombo(label, enumeration, 1);
    if (changed)
    {
        timeSystem = TimeSystem(enumeration);
    }
    return changed;
}

const char* to_string(TimeSystem::TimeSystemEnum timeSystem)
{
    return TimeSystem(timeSystem).toString();
}

} // namespace NAV