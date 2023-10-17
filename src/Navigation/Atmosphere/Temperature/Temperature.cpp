/// This file is part of INSTINCT, the INS Toolkit for Integrated
/// Navigation Concepts and Training by the Institute of Navigation of
/// the University of Stuttgart, Germany.
///
/// This Source Code Form is subject to the terms of the Mozilla Public
/// License, v. 2.0. If a copy of the MPL was not distributed with this
/// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Temperature.hpp"

#include "internal/gui/widgets/EnumCombo.hpp"
#include "util/Logger.hpp"
#include "util/Assert.h"

#include "Models/StandardAtmosphere.hpp"

namespace NAV
{

const char* to_string(TemperatureModel temperatureModel)
{
    switch (temperatureModel)
    {
    case TemperatureModel::None:
        return "None";
    case TemperatureModel::ConstNN:
        return "Const T0";
    case TemperatureModel::ISA:
        return "ISA";
    case TemperatureModel::GPT2:
        return "GPT2";
    case TemperatureModel::GPT3:
        return "GPT3";
    case TemperatureModel::COUNT:
        break;
    }
    return "";
}

bool ComboTemperatureModel(const char* label, TemperatureModel& temperatureModel)
{
    return gui::widgets::EnumCombo(label, temperatureModel);
}

double calcAbsoluteTemperature(double altitudeMSL, TemperatureModel temperatureModel)
{
    switch (temperatureModel)
    {
    case TemperatureModel::ConstNN:
        return calcAbsoluteTemperatureStAtm(0);
    case TemperatureModel::ISA:
        return calcAbsoluteTemperatureStAtm(altitudeMSL);
    case TemperatureModel::GPT2:
    case TemperatureModel::GPT3:
        INS_ASSERT_USER_ERROR(false, "GPT2/GPT3 Model needs to be called separately because of parameter lookup.");
        break;
    case TemperatureModel::None:
    case TemperatureModel::COUNT:
        break;
    }

    return 0.0;
}

} // namespace NAV
