// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Mechanization.hpp"

namespace NAV
{

void to_json(json& j, const PosVelAttDerivativeConstants& data)
{
    j = json{
        { "gravitationModel", data.gravitationModel },
        { "coriolisAccelerationCompensationEnabled", data.coriolisAccelerationCompensationEnabled },
        { "centrifgalAccelerationCompensationEnabled", data.centrifgalAccelerationCompensationEnabled },
        { "angularRateEarthRotationCompensationEnabled", data.angularRateEarthRotationCompensationEnabled },
        { "angularRateTransportRateCompensationEnabled", data.angularRateTransportRateCompensationEnabled },
    };
}

void from_json(const json& j, PosVelAttDerivativeConstants& data)
{
    if (j.contains("gravitationModel")) { j.at("gravitationModel").get_to(data.gravitationModel); }
    if (j.contains("coriolisAccelerationCompensationEnabled")) { j.at("coriolisAccelerationCompensationEnabled").get_to(data.coriolisAccelerationCompensationEnabled); }
    if (j.contains("centrifgalAccelerationCompensationEnabled")) { j.at("centrifgalAccelerationCompensationEnabled").get_to(data.centrifgalAccelerationCompensationEnabled); }
    if (j.contains("angularRateEarthRotationCompensationEnabled")) { j.at("angularRateEarthRotationCompensationEnabled").get_to(data.angularRateEarthRotationCompensationEnabled); }
    if (j.contains("angularRateTransportRateCompensationEnabled")) { j.at("angularRateTransportRateCompensationEnabled").get_to(data.angularRateTransportRateCompensationEnabled); }
}

} // namespace NAV