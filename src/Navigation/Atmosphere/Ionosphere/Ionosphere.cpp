// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Ionosphere.hpp"

#include <vector>
#include <array>
#include "internal/gui/widgets/EnumCombo.hpp"
#include "util/Logger.hpp"

#include "Models/Klobuchar.hpp"

#include "Navigation/GNSS/Functions.hpp"
#include "Navigation/Constants.hpp"

namespace NAV
{

const char* to_string(IonosphereModel ionosphereModel)
{
    switch (ionosphereModel)
    {
    case IonosphereModel::None:
        return "None";
    case IonosphereModel::Klobuchar:
        return "Klobuchar / Broadcast";
    case IonosphereModel::COUNT:
        break;
    }
    return "";
}

bool ComboIonosphereModel(const char* label, IonosphereModel& ionosphereModel)
{
    return gui::widgets::EnumCombo(label, ionosphereModel);
}

double calcIonosphericDelay(double tow, Frequency freq, int8_t freqNum,
                            const Eigen::Vector3d& lla_pos,
                            double elevation, double azimuth,
                            IonosphereModel ionosphereModel,
                            const IonosphericCorrections* corrections)
{
    if (lla_pos(2) < -1000.0)
    {
        LOG_TRACE("Not calculating ionospheric delay, due to altitude being invalid: {}m", lla_pos(2));
        return 0.0;
    }

    switch (ionosphereModel)
    {
    case IonosphereModel::Klobuchar:
    {
        if (corrections)
        {
            const auto* alpha = corrections->get(GPS, IonosphericCorrections::Alpha);
            const auto* beta = corrections->get(GPS, IonosphericCorrections::Beta);
            if (alpha && beta)
            {
                return calcIonosphericTimeDelay_Klobuchar(tow, freq, freqNum, lla_pos(0), lla_pos(1), elevation, azimuth, *alpha, *beta)
                       * InsConst::C;
            }
        }

        LOG_ERROR("Ionosphere model Klobuchar/Broadcast needs correction parameters. Ionospheric time delay will be 0.");
        break;
    }
    case IonosphereModel::None:
    case IonosphereModel::COUNT:
        break;
    }

    return 0.0;
}

double ionoErrorVar(double dpsr_I, Frequency freq, int8_t num)
{
    constexpr double ERR_BRDCI = 0.5; // Broadcast iono model error factor (See GPS ICD ch. 20.3.3.5.2.5, p. 130: 50% reduction on RMS error)

    return ratioFreqSquared(freq.getL1(), freq, num, num)
           * std::pow(dpsr_I * ERR_BRDCI, 2);
}

} // namespace NAV
