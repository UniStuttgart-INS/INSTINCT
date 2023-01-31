/// This file is part of INSTINCT, the INS Toolkit for Integrated
/// Navigation Concepts and Training by the Institute of Navigation of
/// the University of Stuttgart, Germany.
///
/// This Source Code Form is subject to the terms of the Mozilla Public
/// License, v. 2.0. If a copy of the MPL was not distributed with this
/// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Pressure.hpp"

#include "internal/gui/widgets/EnumCombo.hpp"
#include "util/Logger.hpp"

#include "Models/StandardAtmosphere.hpp"

namespace NAV
{

const char* to_string(PressureModel pressureModel)
{
    switch (pressureModel)
    {
    case PressureModel::None:
        return "None";
    case PressureModel::ConstNN:
        return "Const p0";
    case PressureModel::ISA:
        return "ISA";
    case PressureModel::COUNT:
        break;
    }
    return "";
}

bool ComboPressureModel(const char* label, PressureModel& pressureModel)
{
    return gui::widgets::EnumCombo(label, pressureModel);
}

double calcTotalPressure(double altitudeMSL, PressureModel pressureModel)
{
    switch (pressureModel)
    {
    case PressureModel::ConstNN:
        return calcTotalPressureStAtm(0);
    case PressureModel::ISA:
        return calcTotalPressureStAtm(altitudeMSL);
    case PressureModel::None:
    case PressureModel::COUNT:
        break;
    }

    return 0.0;
}

} // namespace NAV
