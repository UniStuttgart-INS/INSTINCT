// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "NumericalIntegration.hpp"

namespace NAV
{

const char* to_string(IntegrationAlgorithm algorithm)
{
    switch (algorithm)
    {
    // case IntegrationAlgorithm::RectangularRule:
    //     return "Rectangular Rule";
    // case IntegrationAlgorithm::Simpson:
    //     return "Simpson";
    case IntegrationAlgorithm::Heun:
        return "Heun's method";
    case IntegrationAlgorithm::RungeKutta1:
        return "Runge Kutta 1st Order";
    case IntegrationAlgorithm::RungeKutta2:
        return "Runge Kutta 2nd Order";
    case IntegrationAlgorithm::RungeKutta3:
        return "Runge Kutta 3rd Order";
    case IntegrationAlgorithm::RungeKutta4:
        return "Runge Kutta 4th Order";
    case IntegrationAlgorithm::COUNT:
        return "";
    }
    return "";
}

} // namespace NAV
