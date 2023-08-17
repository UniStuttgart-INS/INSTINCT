// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file EnumCombo.hpp
/// @brief Combo representing an enumeration
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-01-31

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
bool EnumCombo(const char* label, T& enumeration, size_t startIdx = 0)
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

/// @brief Combo representing two enumerations. Values will be displayed appended and set to the same value.
/// @tparam T Enumeration Type
/// @param[in] label Label to show beside the combo box. This has to be a unique id for ImGui.
/// @param[in] enumeration1 Reference to the first enumeration variable to select
/// @param[in] enumeration2 Reference to the second enumeration variable to set to the same value as the first one
/// @return True if the value changed
/// @attention The Enum type needs a last element called 'COUNT'
template<typename T>
bool EnumCombo(const char* label, T& enumeration1, T& enumeration2)
{
    bool clicked = false;
    std::string previewText = (enumeration1 == enumeration2
                                   ? NAV::to_string(enumeration1)
                                   : fmt::format("{} | {}", NAV::to_string(enumeration1), NAV::to_string(enumeration2)));
    if (ImGui::BeginCombo(label, previewText.c_str()))
    {
        for (size_t i = 0; i < static_cast<size_t>(T::COUNT); i++)
        {
            const bool is_selected = (static_cast<size_t>(enumeration1) == i);
            if (ImGui::Selectable(NAV::to_string(static_cast<T>(i)), is_selected))
            {
                enumeration1 = static_cast<T>(i);
                enumeration2 = static_cast<T>(i);
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

} // namespace NAV::gui::widgets