// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Troposphere.hpp"

#include <imgui.h>
#include "util/Logger.hpp"

#include "Models/Saastamoinen.hpp"

#include "MappingFunctions/Cosecant.hpp"

namespace NAV
{

const char* to_string(TroposphereModel troposphereModel)
{
    switch (troposphereModel)
    {
    case TroposphereModel::None:
        return "None";
    case TroposphereModel::Saastamoinen:
        return "Saastamoinen";
    case TroposphereModel::COUNT:
        break;
    }
    return "";
}

const char* to_string(MappingFunction mappingFunction)
{
    switch (mappingFunction)
    {
    case MappingFunction::None:
        return "None";
    case MappingFunction::Cosecant:
        return "Cosecant(elevation)";
    case MappingFunction::COUNT:
        break;
    }
    return "";
}

bool ComboTroposphereModel(const char* label, TroposphereModel& troposphereModel)
{
    bool clicked = false;
    if (ImGui::BeginCombo(label, NAV::to_string(troposphereModel)))
    {
        for (size_t i = 0; i < static_cast<size_t>(TroposphereModel::COUNT); i++)
        {
            const bool is_selected = (static_cast<size_t>(troposphereModel) == i);
            if (ImGui::Selectable(NAV::to_string(static_cast<TroposphereModel>(i)), is_selected))
            {
                troposphereModel = static_cast<TroposphereModel>(i);
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

bool ComboMappingFunction(const char* label, MappingFunction& mappingFunction)
{
    bool clicked = false;
    if (ImGui::BeginCombo(label, NAV::to_string(mappingFunction)))
    {
        for (size_t i = 0; i < static_cast<size_t>(MappingFunction::COUNT); i++)
        {
            const bool is_selected = (static_cast<size_t>(mappingFunction) == i);
            if (ImGui::Selectable(NAV::to_string(static_cast<MappingFunction>(i)), is_selected))
            {
                mappingFunction = static_cast<MappingFunction>(i);
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

ZenithDelay calcTroposphericRangeDelay(const Eigen::Vector3d& lla_pos, TroposphereModel troposphereModel)
{
    switch (troposphereModel)
    {
    case TroposphereModel::Saastamoinen:
        return calcTroposphericRangeDelay_Saastamoinen(lla_pos);
    case TroposphereModel::None:
    case TroposphereModel::COUNT:
        break;
    }

    return { .ZHD = 0.0,
             .ZWD = 0.0 };
}

double calcTropoMapFunc(double elevation, MappingFunction mappingFunction)
{
    switch (mappingFunction)
    {
    case MappingFunction::Cosecant:
        return calcTropoMapFunc_cosecant(elevation);
    case MappingFunction::None:
    case MappingFunction::COUNT:
        break;
    }

    return 1.0;
}

} // namespace NAV
