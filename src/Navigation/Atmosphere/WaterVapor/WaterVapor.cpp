/// This file is part of INSTINCT, the INS Toolkit for Integrated
/// Navigation Concepts and Training by the Institute of Navigation of
/// the University of Stuttgart, Germany.
///
/// This Source Code Form is subject to the terms of the Mozilla Public
/// License, v. 2.0. If a copy of the MPL was not distributed with this
/// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "WaterVapor.hpp"

#include "internal/gui/widgets/EnumCombo.hpp"
#include "util/Logger.hpp"
#include "util/Assert.h"

#include "Models/StandardAtmosphere.hpp"

namespace NAV
{

const char* to_string(WaterVaporModel waterVaporModel)
{
    switch (waterVaporModel)
    {
    case WaterVaporModel::None:
        return "None";
    case WaterVaporModel::ISA:
        return "ISA";
    case WaterVaporModel::GPT2:
        return "GPT2";
    case WaterVaporModel::GPT3:
        return "GPT3";
    case WaterVaporModel::COUNT:
        break;
    }
    return "";
}

bool ComboWaterVaporModel(const char* label, WaterVaporModel& waterVaporModel)
{
    return gui::widgets::EnumCombo(label, waterVaporModel);
}

double calcWaterVaporPartialPressure(double temp, double humidity_rel, WaterVaporModel waterVaporModel)
{
    switch (waterVaporModel)
    {
    case WaterVaporModel::ISA:
        return calcWaterVaporPartialPressureStAtm(temp, humidity_rel);
    case WaterVaporModel::GPT2:
    case WaterVaporModel::GPT3:
        INS_ASSERT_USER_ERROR(false, "GPT2/GPT3 Model needs to be called separately because of parameter lookup.");
        break;
    case WaterVaporModel::None:
    case WaterVaporModel::COUNT:
        break;
    }

    return 0.0;
}

} // namespace NAV