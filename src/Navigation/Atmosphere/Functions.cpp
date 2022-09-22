// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Functions.hpp"

#include <cmath>

namespace NAV
{

double calcTotalPressure(double altitudeMSL)
{
    return 1013.25 * std::pow(1 - 2.2557e-5 * altitudeMSL, 5.2568);
}

double calcAbsoluteTemperature(double altitudeMSL)
{
    return 15.0 - 6.5e-3 * altitudeMSL + 273.15;
}

double calcWaterVaporPartialPressure(double temp, double humidity_rel)
{
    return 6.108 * std::exp((17.15 * temp - 4684.0) / (temp - 38.45)) * humidity_rel;
}

} // namespace NAV
