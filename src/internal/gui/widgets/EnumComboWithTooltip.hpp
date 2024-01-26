// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file EnumComboWithTooltip.hpp
/// @brief Combo representing an enumeration with tooltip
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-01-19

#pragma once

#include <imgui.h>
#include <fmt/core.h>

namespace NAV::gui::widgets
{

/// @brief Combo representing an enumeration
/// @tparam T Enumeration Type
/// @param[in] label Label to show beside the combo box. This has to be a unique id for ImGui.
/// @param[in] enumeration Reference to the enumeration variable to select
/// @param[in] startIdx Start Index in the enum (so skip first items)
/// @return True if the value changed
/// @attention The Enum type needs a last element called 'COUNT'
template<typename T>
bool EnumComboWithToolTip(const char* label, T& enumeration, size_t startIdx = 0)
{
    bool clicked = false;
    if (ImGui::BeginCombo(label, NAV::to_string(enumeration)))
    {
        for (size_t i = startIdx; i < static_cast<size_t>(T::COUNT); i++)
        {
            const bool is_selected = (static_cast<size_t>(enumeration) == i);
            if (ImGui::Selectable(NAV::to_string(static_cast<T>(i)), is_selected))
            {
                enumeration = static_cast<T>(i);
                clicked = true;
            }
            if (ImGui::IsItemHovered())
            {
                ImGui::SetTooltip("%s", NAV::tooltip(static_cast<T>(i)));
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

} // namespace NAV::gui::widgets