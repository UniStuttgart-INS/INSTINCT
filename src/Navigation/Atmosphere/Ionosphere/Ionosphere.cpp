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
#include <imgui.h>
#include "util/Logger.hpp"

#include "Models/Klobuchar.hpp"

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
    bool clicked = false;
    if (ImGui::BeginCombo(label, NAV::to_string(ionosphereModel)))
    {
        for (size_t i = 0; i < static_cast<size_t>(IonosphereModel::COUNT); i++)
        {
            const bool is_selected = (static_cast<size_t>(ionosphereModel) == i);
            if (ImGui::Selectable(NAV::to_string(static_cast<IonosphereModel>(i)), is_selected))
            {
                ionosphereModel = static_cast<IonosphereModel>(i);
                clicked = true;
            }

            // Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
            if (is_selected)
            {
                ImGui::SetItemDefaultFocus();
            }
        }

        ImGui::EndCombo();
    }
    return clicked;
}

double calcIonosphericTimeDelay(double tow, Frequency freq,
                                const Eigen::Vector3d& lla_pos,
                                double elevation, double azimuth,
                                IonosphereModel ionosphereModel,
                                const IonosphericCorrections* corrections)
{
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
                return calcIonosphericTimeDelay_Klobuchar(tow, freq, lla_pos(0), lla_pos(1), elevation, azimuth, *alpha, *beta);
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

} // namespace NAV
