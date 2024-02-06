// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Gravity.hpp"

#include <cmath>
#include <optional>
#include <string>
#include <fstream>
#include <streambuf>

#include <imgui.h>

namespace NAV
{

const char* to_string(GravitationModel gravitationModel)
{
    switch (gravitationModel)
    {
    case GravitationModel::None:
        return "None";
    case GravitationModel::WGS84:
        return "WGS84";
    case GravitationModel::WGS84_Skydel:
        return "WGS84 (Skydel Constants)";
    case GravitationModel::Somigliana:
        return "Somigliana";
    case GravitationModel::EGM96:
        return "EGM96";
    case GravitationModel::COUNT:
        return "";
    }
    return "";
}

bool ComboGravitationModel(const char* label, GravitationModel& gravitationModel)
{
    bool clicked = false;
    if (ImGui::BeginCombo(label, NAV::to_string(gravitationModel)))
    {
        for (size_t i = 0; i < static_cast<size_t>(GravitationModel::COUNT); i++)
        {
            const bool is_selected = (static_cast<size_t>(gravitationModel) == i);
            if (ImGui::Selectable(NAV::to_string(static_cast<GravitationModel>(i)), is_selected))
            {
                gravitationModel = static_cast<GravitationModel>(i);
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

} // namespace NAV