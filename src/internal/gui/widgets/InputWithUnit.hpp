// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file InputWithUnit.hpp
/// @brief Defines Widgets which allow the input of values and the selection of the unit
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-10-21

#pragma once

#include <cstdint>
#include <fmt/format.h>
#include <imgui.h>
#include <string>
#include <algorithm>
#include "imgui_ex.hpp"

namespace NAV
{

/// @brief Units separated by '\0' and terminated by double '\0'
template<typename T>
[[nodiscard]] std::string MakeComboItems()
{
    std::string str;
    for (size_t i = 0; i < static_cast<size_t>(T::COUNT); i++)
    {
        if (!str.empty()) { str += '\0'; }
        str += to_string(static_cast<T>(i));
    }
    str += '\0';
    str += '\0';
    return str;
}

namespace gui::widgets
{
/// Return value signaling that the input or the unit changed
enum InputWithUnitChange : uint8_t
{
    InputWithUnitChange_None = 0, ///< Nothing changed
    InputWithUnitChange_Input,    ///< The Input changed
    InputWithUnitChange_Unit,     ///< The Unit changed
};

namespace internal
{

/// @brief Shows a Unit input combo
/// @param[in] label Label to display beside the input. Has to be unique (use # to hide text afterwards to append an uid)
/// @param unitWidth Width of the unit combo
/// @param combo_current_item The selected item in the unit combo
/// @param combo_items_separated_by_zeros Items to display in the unit combo (separated by \0 and ends with \0\0)
/// @param combo_popup_max_height_in_items Maximum height of the combo in number of items
/// @return Returns if the unit was changed
template<typename U>
InputWithUnitChange Unit(const char* label, float unitWidth,
                         U& combo_current_item, const char* combo_items_separated_by_zeros,
                         int combo_popup_max_height_in_items)
{
    InputWithUnitChange retVal = InputWithUnitChange_None;

    ImGui::SetNextItemWidth(unitWidth - ImGui::GetStyle().ItemSpacing.x);

    char first = '1';
    char second = '1';
    bool disable = true;
    for (size_t i = 0; first != '\0' || second != '\0'; i++)
    {
        first = *(combo_items_separated_by_zeros + i);
        second = *(combo_items_separated_by_zeros + i + 1);

        if (first == '\0' && second != '\0')
        {
            disable = false;
            break;
        }
    }

    if (disable) { ImGui::BeginDisabled(); }
    if (auto current_item = static_cast<int>(combo_current_item);
        ImGui::Combo(fmt::format("##{} - unit", label).c_str(), &current_item, combo_items_separated_by_zeros, combo_popup_max_height_in_items))
    {
        combo_current_item = static_cast<U>(current_item);
        retVal = InputWithUnitChange_Unit;
    }
    if (disable) { ImGui::EndDisabled(); }
    ImGui::SameLine();
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() - ImGui::GetStyle().ItemSpacing.x + ImGui::GetStyle().ItemInnerSpacing.x);
    std::string strLabel{ label };
    ImGui::TextUnformatted(strLabel.substr(0, strLabel.find('#')).c_str());

    return retVal;
}

/// @brief Shows an InputText GUI element to modify the provided value and also set its unit
/// @param[in] label Label to display beside the input. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in] itemWidth Width of the input element(s) + unit combo
/// @param[in] unitWidth Width of the unit combo
/// @param[in, out] v Pointer to the value to modify
/// @param[in, out] combo_current_item The selected item in the unit combo
/// @param[in] combo_items_separated_by_zeros Items to display in the unit combo (separated by \0 and ends with \0\0)
/// @param[in] step Step size of the InputText
/// @param[in] step_fast Fast step size of the InputText
/// @param[in] format Printf format to display the value with
/// @param[in] flags InputText flags to modify the behavior
/// @param[in] combo_popup_max_height_in_items Maximum height of the combo in number of items
/// @return Returns if the value or unit was changed
template<ImGuiDataType_ _Scalar, unsigned int _Size, typename T, typename U>
InputWithUnitChange InputWithUnit(const char* label, float itemWidth, float unitWidth,
                                  T v[_Size], U& combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                  T step, T step_fast, const char* format, ImGuiInputTextFlags flags,
                                  int combo_popup_max_height_in_items)
{
    InputWithUnitChange retVal = InputWithUnitChange_None;

    ImGui::SetNextItemWidth(itemWidth - unitWidth);
    if constexpr (_Size == 1)
    {
        flags |= ImGuiInputTextFlags_CharsScientific;
        if (ImGui::InputScalar(fmt::format("##{} - input", label).c_str(), _Scalar, static_cast<void*>(v), static_cast<void*>(step > 0.0 ? &step : nullptr), static_cast<void*>(step_fast > 0.0 ? &step_fast : nullptr), format, flags))
        {
            retVal = InputWithUnitChange_Input;
        }
    }
    else
    {
        if (ImGui::InputScalarN(fmt::format("##{} - input", label).c_str(), _Scalar, v, _Size, nullptr, nullptr, format, flags))
        {
            retVal = InputWithUnitChange_Input;
        }
    }
    ImGui::SameLine();
    retVal = static_cast<InputWithUnitChange>(retVal | Unit(label, unitWidth, combo_current_item, combo_items_separated_by_zeros, combo_popup_max_height_in_items));

    return retVal;
}

/// @brief Shows an Slider GUI element to modify the provided value and also set its unit
/// @param[in] label Label to display beside the input. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in] itemWidth Width of the input element(s) + unit combo
/// @param[in] unitWidth Width of the unit combo
/// @param[in, out] v Pointer to the value to modify
/// @param[in, out] combo_current_item The selected item in the unit combo
/// @param[in] combo_items_separated_by_zeros Items to display in the unit combo (separated by \0 and ends with \0\0)
/// @param[in] min Minimum value allowed
/// @param[in] max Maximum value allowed
/// @param[in] format Printf format to display the value with
/// @param[in] flags Slider flags to modify the behavior
/// @param[in] combo_popup_max_height_in_items Maximum height of the combo in number of items
/// @return Returns if the value or unit was changed
template<ImGuiDataType_ _Scalar, unsigned int _Size, typename T, typename U>
InputWithUnitChange SliderWithUnit(const char* label, float itemWidth, float unitWidth,
                                   T v[_Size], U& combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                   T min, T max, const char* format, ImGuiInputTextFlags flags,
                                   int combo_popup_max_height_in_items)
{
    InputWithUnitChange retVal = InputWithUnitChange_None;

    ImGui::SetNextItemWidth(itemWidth - unitWidth);
    if constexpr (_Size == 1)
    {
        flags |= ImGuiInputTextFlags_CharsScientific;
        if (ImGui::SliderScalar(fmt::format("##{} - input", label).c_str(), _Scalar, static_cast<void*>(v), &min, &max, format, flags))
        {
            retVal = InputWithUnitChange_Input;
        }
    }
    else
    {
        if (ImGui::SliderScalarN(fmt::format("##{} - input", label).c_str(), _Scalar, v, _Size, &min, &max, format, flags))
        {
            retVal = InputWithUnitChange_Input;
        }
    }
    ImGui::SameLine();
    retVal = static_cast<InputWithUnitChange>(retVal | Unit(label, unitWidth, combo_current_item, combo_items_separated_by_zeros, combo_popup_max_height_in_items));

    return retVal;
}

} // namespace internal

/// @brief Shows an InputText GUI element to modify the provided value and also set its unit
/// @param[in] label Label to display beside the input. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in] itemWidth Width of the input element(s) + unit combo
/// @param[in] unitWidth Width of the unit combo
/// @param[in, out] v Pointer to the value to modify
/// @param[in, out] combo_current_item The selected item in the unit combo
/// @param[in] combo_items_separated_by_zeros Items to display in the unit combo (separated by \0 and ends with \0\0)
/// @param[in] step Step size of the InputText
/// @param[in] step_fast Fast step size of the InputText
/// @param[in] format Printf format to display the value with
/// @param[in] flags InputText flags to modify the behavior
/// @param[in] combo_popup_max_height_in_items Maximum height of the combo in number of items
/// @return Returns if the value or unit was changed
template<typename U>
InputWithUnitChange InputFloatWithUnit(const char* label, float itemWidth, float unitWidth,
                                       float* v, U& combo_current_item, const char* combo_items_separated_by_zeros,
                                       float step = 0.0, float step_fast = 0.0, const char* format = "%.3f", ImGuiInputTextFlags flags = 0,
                                       int combo_popup_max_height_in_items = -1)
{
    return internal::InputWithUnit<ImGuiDataType_Float, 1, float>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, step, step_fast, format, flags, combo_popup_max_height_in_items);
}

/// @brief Shows an InputText GUI element to modify the provided value and also set its unit
/// @param[in] label Label to display beside the input. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in] itemWidth Width of the input element(s) + unit combo
/// @param[in] unitWidth Width of the unit combo
/// @param[in, out] v Pointer to the value to modify
/// @param[in, out] combo_current_item The selected item in the unit combo
/// @param[in] combo_items_separated_by_zeros Items to display in the unit combo (separated by \0 and ends with \0\0)
/// @param[in] format Printf format to display the value with
/// @param[in] flags InputText flags to modify the behavior
/// @param[in] combo_popup_max_height_in_items Maximum height of the combo in number of items
/// @return Returns if the value or unit was changed
template<typename U>
InputWithUnitChange InputFloat2WithUnit(const char* label, float itemWidth, float unitWidth,
                                        float v[2], U& combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                        const char* format = "%.3f", ImGuiInputTextFlags flags = 0,
                                        int combo_popup_max_height_in_items = -1)
{
    return internal::InputWithUnit<ImGuiDataType_Float, 2, float>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, 0.0, 0.0, format, flags, combo_popup_max_height_in_items);
}

/// @brief Shows an InputText GUI element to modify the provided value and also set its unit
/// @param[in] label Label to display beside the input. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in] itemWidth Width of the input element(s) + unit combo
/// @param[in] unitWidth Width of the unit combo
/// @param[in, out] v Pointer to the value to modify
/// @param[in, out] combo_current_item The selected item in the unit combo
/// @param[in] combo_items_separated_by_zeros Items to display in the unit combo (separated by \0 and ends with \0\0)
/// @param[in] format Printf format to display the value with
/// @param[in] flags InputText flags to modify the behavior
/// @param[in] combo_popup_max_height_in_items Maximum height of the combo in number of items
/// @return Returns if the value or unit was changed
template<typename U>
InputWithUnitChange InputFloat3WithUnit(const char* label, float itemWidth, float unitWidth,
                                        float v[3], U& combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                        const char* format = "%.3f", ImGuiInputTextFlags flags = 0,
                                        int combo_popup_max_height_in_items = -1)
{
    return internal::InputWithUnit<ImGuiDataType_Float, 3, float>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, 0.0, 0.0, format, flags, combo_popup_max_height_in_items);
}

/// @brief Shows an InputText GUI element to modify the provided value and also set its unit
/// @param[in] label Label to display beside the input. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in] itemWidth Width of the input element(s) + unit combo
/// @param[in] unitWidth Width of the unit combo
/// @param[in, out] v Pointer to the value to modify
/// @param[in, out] combo_current_item The selected item in the unit combo
/// @param[in] combo_items_separated_by_zeros Items to display in the unit combo (separated by \0 and ends with \0\0)
/// @param[in] format Printf format to display the value with
/// @param[in] flags InputText flags to modify the behavior
/// @param[in] combo_popup_max_height_in_items Maximum height of the combo in number of items
/// @return Returns if the value or unit was changed
template<typename U>
InputWithUnitChange InputFloat4WithUnit(const char* label, float itemWidth, float unitWidth,
                                        float v[4], U& combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                        const char* format = "%.3f", ImGuiInputTextFlags flags = 0,
                                        int combo_popup_max_height_in_items = -1)
{
    return internal::InputWithUnit<ImGuiDataType_Float, 4, float>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, 0.0, 0.0, format, flags, combo_popup_max_height_in_items);
}

/// @brief Shows an InputText GUI element to modify the provided value and also set its unit
/// @param[in] label Label to display beside the input. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in] itemWidth Width of the input element(s) + unit combo
/// @param[in] unitWidth Width of the unit combo
/// @param[in, out] v Pointer to the value to modify
/// @param[in, out] combo_current_item The selected item in the unit combo
/// @param[in] combo_items_separated_by_zeros Items to display in the unit combo (separated by \0 and ends with \0\0)
/// @param[in] step Step size of the InputText
/// @param[in] step_fast Fast step size of the InputText
/// @param[in] format Printf format to display the value with
/// @param[in] flags InputText flags to modify the behavior
/// @param[in] combo_popup_max_height_in_items Maximum height of the combo in number of items
/// @return Returns if the value or unit was changed
template<typename U>
InputWithUnitChange InputDoubleWithUnit(const char* label, float itemWidth, float unitWidth,
                                        double* v, U& combo_current_item, const char* combo_items_separated_by_zeros,
                                        double step = 0.0, double step_fast = 0.0, const char* format = "%.6f", ImGuiInputTextFlags flags = 0,
                                        int combo_popup_max_height_in_items = -1)
{
    return internal::InputWithUnit<ImGuiDataType_Double, 1, double>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, step, step_fast, format, flags, combo_popup_max_height_in_items);
}

/// @brief Shows an InputText GUI element to modify the provided value and also set its unit
/// @param[in] label Label to display beside the input. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in] itemWidth Width of the input element(s) + unit combo
/// @param[in] unitWidth Width of the unit combo
/// @param[in, out] v Pointer to the value to modify
/// @param[in, out] combo_current_item The selected item in the unit combo
/// @param[in] combo_items_separated_by_zeros Items to display in the unit combo (separated by \0 and ends with \0\0)
/// @param[in] format Printf format to display the value with
/// @param[in] flags InputText flags to modify the behavior
/// @param[in] combo_popup_max_height_in_items Maximum height of the combo in number of items
/// @return Returns if the value or unit was changed
template<typename U>
InputWithUnitChange InputDouble2WithUnit(const char* label, float itemWidth, float unitWidth,
                                         double v[2], U& combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                         const char* format = "%.6f", ImGuiInputTextFlags flags = 0,
                                         int combo_popup_max_height_in_items = -1)
{
    return internal::InputWithUnit<ImGuiDataType_Double, 2, double>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, 0.0, 0.0, format, flags, combo_popup_max_height_in_items);
}

/// @brief Shows an InputText GUI element to modify the provided value and also set its unit
/// @param[in] label Label to display beside the input. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in] itemWidth Width of the input element(s) + unit combo
/// @param[in] unitWidth Width of the unit combo
/// @param[in, out] v Pointer to the value to modify
/// @param[in, out] combo_current_item The selected item in the unit combo
/// @param[in] combo_items_separated_by_zeros Items to display in the unit combo (separated by \0 and ends with \0\0)
/// @param[in] format Printf format to display the value with
/// @param[in] flags InputText flags to modify the behavior
/// @param[in] combo_popup_max_height_in_items Maximum height of the combo in number of items
/// @return Returns if the value or unit was changed
template<typename U>
InputWithUnitChange InputDouble3WithUnit(const char* label, float itemWidth, float unitWidth,
                                         double v[3], U& combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                         const char* format = "%.6f", ImGuiInputTextFlags flags = 0,
                                         int combo_popup_max_height_in_items = -1)
{
    return internal::InputWithUnit<ImGuiDataType_Double, 3, double>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, 0.0, 0.0, format, flags, combo_popup_max_height_in_items);
}

/// @brief Shows an InputText GUI element to modify the provided value and also set its unit
/// @param[in] label Label to display beside the input. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in] itemWidth Width of the input element(s) + unit combo
/// @param[in] unitWidth Width of the unit combo
/// @param[in, out] v Pointer to the value to modify
/// @param[in, out] combo_current_item The selected item in the unit combo
/// @param[in] combo_items_separated_by_zeros Items to display in the unit combo (separated by \0 and ends with \0\0)
/// @param[in] format Printf format to display the value with
/// @param[in] flags InputText flags to modify the behavior
/// @param[in] combo_popup_max_height_in_items Maximum height of the combo in number of items
/// @return Returns if the value or unit was changed
template<typename U>
InputWithUnitChange InputDouble4WithUnit(const char* label, float itemWidth, float unitWidth,
                                         double v[4], U& combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                         const char* format = "%.6f", ImGuiInputTextFlags flags = 0,
                                         int combo_popup_max_height_in_items = -1)
{
    return internal::InputWithUnit<ImGuiDataType_Double, 4, double>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, 0.0, 0.0, format, flags, combo_popup_max_height_in_items);
}

// ###########################################################################################################

/// @brief Shows an InputText GUI element to modify the provided value and also set its unit
/// @param[in] label Label to display beside the input. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in] itemWidth Width of the input element(s) + unit combo
/// @param[in] unitWidth Width of the unit combo
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_min Minimum value allowed
/// @param[in] v_max Maximum value allowed
/// @param[in, out] combo_current_item The selected item in the unit combo
/// @param[in] combo_items_separated_by_zeros Items to display in the unit combo (separated by \0 and ends with \0\0)
/// @param[in] step Step size of the InputText
/// @param[in] step_fast Fast step size of the InputText
/// @param[in] format Printf format to display the value with
/// @param[in] flags InputText flags to modify the behavior
/// @param[in] combo_popup_max_height_in_items Maximum height of the combo in number of items
/// @return Returns if the value or unit was changed
template<typename U>
InputWithUnitChange InputFloatLWithUnit(const char* label, float itemWidth, float unitWidth,
                                        float* v, float v_min, float v_max, U& combo_current_item, const char* combo_items_separated_by_zeros,
                                        float step = 0.0, float step_fast = 0.0, const char* format = "%.3f", ImGuiInputTextFlags flags = 0,
                                        int combo_popup_max_height_in_items = -1)
{
    auto change = internal::InputWithUnit<ImGuiDataType_Float, 1, float>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, step, step_fast, format, flags, combo_popup_max_height_in_items);
    if (change == InputWithUnitChange::InputWithUnitChange_Input)
    {
        *v = std::clamp(*v, v_min, v_max);
    }
    return change;
}

/// @brief Shows an InputText GUI element to modify the provided value and also set its unit
/// @param[in] label Label to display beside the input. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in] itemWidth Width of the input element(s) + unit combo
/// @param[in] unitWidth Width of the unit combo
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_min Minimum value allowed
/// @param[in] v_max Maximum value allowed
/// @param[in, out] combo_current_item The selected item in the unit combo
/// @param[in] combo_items_separated_by_zeros Items to display in the unit combo (separated by \0 and ends with \0\0)
/// @param[in] format Printf format to display the value with
/// @param[in] flags InputText flags to modify the behavior
/// @param[in] combo_popup_max_height_in_items Maximum height of the combo in number of items
/// @return Returns if the value or unit was changed
template<typename U>
InputWithUnitChange InputFloat2LWithUnit(const char* label, float itemWidth, float unitWidth,
                                         float v[2], float v_min, float v_max, U& combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                         const char* format = "%.3f", ImGuiInputTextFlags flags = 0,
                                         int combo_popup_max_height_in_items = -1)
{
    auto change = internal::InputWithUnit<ImGuiDataType_Float, 2, float>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, 0.0, 0.0, format, flags, combo_popup_max_height_in_items);
    if (change == InputWithUnitChange::InputWithUnitChange_Input)
    {
        for (size_t i = 0; i < 2; i++)
        {
            v[i] = std::clamp(v[i], v_min, v_max);
        }
    }
    return change;
}

/// @brief Shows an InputText GUI element to modify the provided value and also set its unit
/// @param[in] label Label to display beside the input. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in] itemWidth Width of the input element(s) + unit combo
/// @param[in] unitWidth Width of the unit combo
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_min Minimum value allowed
/// @param[in] v_max Maximum value allowed
/// @param[in, out] combo_current_item The selected item in the unit combo
/// @param[in] combo_items_separated_by_zeros Items to display in the unit combo (separated by \0 and ends with \0\0)
/// @param[in] format Printf format to display the value with
/// @param[in] flags InputText flags to modify the behavior
/// @param[in] combo_popup_max_height_in_items Maximum height of the combo in number of items
/// @return Returns if the value or unit was changed
template<typename U>
InputWithUnitChange InputFloat3LWithUnit(const char* label, float itemWidth, float unitWidth,
                                         float v[3], float v_min, float v_max, U& combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                         const char* format = "%.3f", ImGuiInputTextFlags flags = 0,
                                         int combo_popup_max_height_in_items = -1)
{
    auto change = internal::InputWithUnit<ImGuiDataType_Float, 3, float>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, 0.0, 0.0, format, flags, combo_popup_max_height_in_items);
    if (change == InputWithUnitChange::InputWithUnitChange_Input)
    {
        for (size_t i = 0; i < 3; i++)
        {
            v[i] = std::clamp(v[i], v_min, v_max);
        }
    }
    return change;
}

/// @brief Shows an InputText GUI element to modify the provided value and also set its unit
/// @param[in] label Label to display beside the input. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in] itemWidth Width of the input element(s) + unit combo
/// @param[in] unitWidth Width of the unit combo
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_min Minimum value allowed
/// @param[in] v_max Maximum value allowed
/// @param[in, out] combo_current_item The selected item in the unit combo
/// @param[in] combo_items_separated_by_zeros Items to display in the unit combo (separated by \0 and ends with \0\0)
/// @param[in] format Printf format to display the value with
/// @param[in] flags InputText flags to modify the behavior
/// @param[in] combo_popup_max_height_in_items Maximum height of the combo in number of items
/// @return Returns if the value or unit was changed
template<typename U>
InputWithUnitChange InputFloat4LWithUnit(const char* label, float itemWidth, float unitWidth,
                                         float v[4], float v_min, float v_max, U& combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                         const char* format = "%.3f", ImGuiInputTextFlags flags = 0,
                                         int combo_popup_max_height_in_items = -1)
{
    auto change = internal::InputWithUnit<ImGuiDataType_Float, 4, float>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, 0.0, 0.0, format, flags, combo_popup_max_height_in_items);
    if (change == InputWithUnitChange::InputWithUnitChange_Input)
    {
        for (size_t i = 0; i < 4; i++)
        {
            v[i] = std::clamp(v[i], v_min, v_max);
        }
    }
    return change;
}

/// @brief Shows an InputText GUI element to modify the provided value and also set its unit
/// @param[in] label Label to display beside the input. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in] itemWidth Width of the input element(s) + unit combo
/// @param[in] unitWidth Width of the unit combo
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_min Minimum value allowed
/// @param[in] v_max Maximum value allowed
/// @param[in, out] combo_current_item The selected item in the unit combo
/// @param[in] combo_items_separated_by_zeros Items to display in the unit combo (separated by \0 and ends with \0\0)
/// @param[in] step Step size of the InputText
/// @param[in] step_fast Fast step size of the InputText
/// @param[in] format Printf format to display the value with
/// @param[in] flags InputText flags to modify the behavior
/// @param[in] combo_popup_max_height_in_items Maximum height of the combo in number of items
/// @return Returns if the value or unit was changed
template<typename U>
InputWithUnitChange InputDoubleLWithUnit(const char* label, float itemWidth, float unitWidth,
                                         double* v, double v_min, double v_max, U& combo_current_item, const char* combo_items_separated_by_zeros,
                                         double step = 0.0, double step_fast = 0.0, const char* format = "%.6f", ImGuiInputTextFlags flags = 0,
                                         int combo_popup_max_height_in_items = -1)
{
    auto change = internal::InputWithUnit<ImGuiDataType_Double, 1, double>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, step, step_fast, format, flags, combo_popup_max_height_in_items);
    if (change == InputWithUnitChange::InputWithUnitChange_Input)
    {
        *v = std::clamp(*v, v_min, v_max);
    }
    return change;
}

/// @brief Shows an InputText GUI element to modify the provided value and also set its unit
/// @param[in] label Label to display beside the input. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in] itemWidth Width of the input element(s) + unit combo
/// @param[in] unitWidth Width of the unit combo
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_min Minimum value allowed
/// @param[in] v_max Maximum value allowed
/// @param[in, out] combo_current_item The selected item in the unit combo
/// @param[in] combo_items_separated_by_zeros Items to display in the unit combo (separated by \0 and ends with \0\0)
/// @param[in] format Printf format to display the value with
/// @param[in] flags InputText flags to modify the behavior
/// @param[in] combo_popup_max_height_in_items Maximum height of the combo in number of items
/// @return Returns if the value or unit was changed
template<typename U>
InputWithUnitChange InputDouble2LWithUnit(const char* label, float itemWidth, float unitWidth,
                                          double v[2], double v_min, double v_max, U& combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                          const char* format = "%.6f", ImGuiInputTextFlags flags = 0,
                                          int combo_popup_max_height_in_items = -1)
{
    auto change = internal::InputWithUnit<ImGuiDataType_Double, 2, double>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, 0.0, 0.0, format, flags, combo_popup_max_height_in_items);
    if (change == InputWithUnitChange::InputWithUnitChange_Input)
    {
        for (size_t i = 0; i < 2; i++)
        {
            v[i] = std::clamp(v[i], v_min, v_max);
        }
    }
    return change;
}

/// @brief Shows an InputText GUI element to modify the provided value and also set its unit
/// @param[in] label Label to display beside the input. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in] itemWidth Width of the input element(s) + unit combo
/// @param[in] unitWidth Width of the unit combo
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_min Minimum value allowed
/// @param[in] v_max Maximum value allowed
/// @param[in, out] combo_current_item The selected item in the unit combo
/// @param[in] combo_items_separated_by_zeros Items to display in the unit combo (separated by \0 and ends with \0\0)
/// @param[in] format Printf format to display the value with
/// @param[in] flags InputText flags to modify the behavior
/// @param[in] combo_popup_max_height_in_items Maximum height of the combo in number of items
/// @return Returns if the value or unit was changed
template<typename U>
InputWithUnitChange InputDouble3LWithUnit(const char* label, float itemWidth, float unitWidth,
                                          double v[3], double v_min, double v_max, U& combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                          const char* format = "%.6f", ImGuiInputTextFlags flags = 0,
                                          int combo_popup_max_height_in_items = -1)
{
    auto change = internal::InputWithUnit<ImGuiDataType_Double, 3, double>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, 0.0, 0.0, format, flags, combo_popup_max_height_in_items);
    if (change == InputWithUnitChange::InputWithUnitChange_Input)
    {
        for (size_t i = 0; i < 3; i++)
        {
            v[i] = std::clamp(v[i], v_min, v_max);
        }
    }
    return change;
}

/// @brief Shows an InputText GUI element to modify the provided value and also set its unit
/// @param[in] label Label to display beside the input. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in] itemWidth Width of the input element(s) + unit combo
/// @param[in] unitWidth Width of the unit combo
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_min Minimum value allowed
/// @param[in] v_max Maximum value allowed
/// @param[in, out] combo_current_item The selected item in the unit combo
/// @param[in] combo_items_separated_by_zeros Items to display in the unit combo (separated by \0 and ends with \0\0)
/// @param[in] format Printf format to display the value with
/// @param[in] flags InputText flags to modify the behavior
/// @param[in] combo_popup_max_height_in_items Maximum height of the combo in number of items
/// @return Returns if the value or unit was changed
template<typename U>
InputWithUnitChange InputDouble4LWithUnit(const char* label, float itemWidth, float unitWidth,
                                          double v[4], double v_min, double v_max, U& combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                          const char* format = "%.6f", ImGuiInputTextFlags flags = 0,
                                          int combo_popup_max_height_in_items = -1)
{
    auto change = internal::InputWithUnit<ImGuiDataType_Double, 4, double>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, 0.0, 0.0, format, flags, combo_popup_max_height_in_items);
    if (change == InputWithUnitChange::InputWithUnitChange_Input)
    {
        for (size_t i = 0; i < 4; i++)
        {
            v[i] = std::clamp(v[i], v_min, v_max);
        }
    }
    return change;
}

// ###########################################################################################################

/// @brief Shows an Slider GUI element to modify the provided value and also set its unit
/// @param[in] label Label to display beside the input. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in] itemWidth Width of the input element(s) + unit combo
/// @param[in] unitWidth Width of the unit combo
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_min Minimum value allowed
/// @param[in] v_max Maximum value allowed
/// @param[in, out] combo_current_item The selected item in the unit combo
/// @param[in] combo_items_separated_by_zeros Items to display in the unit combo (separated by \0 and ends with \0\0)
/// @param[in] format Printf format to display the value with
/// @param[in] flags Slider flags to modify the behavior
/// @param[in] combo_popup_max_height_in_items Maximum height of the combo in number of items
/// @return Returns if the value or unit was changed
template<typename U>
InputWithUnitChange SliderFloatWithUnit(const char* label, float itemWidth, float unitWidth,
                                        float* v, float v_min, float v_max, U& combo_current_item, const char* combo_items_separated_by_zeros,
                                        const char* format = "%.3f", ImGuiSliderFlags flags = 0,
                                        int combo_popup_max_height_in_items = -1)
{
    return internal::SliderWithUnit<ImGuiDataType_Float, 1, float>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, v_min, v_max, format, flags, combo_popup_max_height_in_items);
}

/// @brief Shows an Slider GUI element to modify the provided value and also set its unit
/// @param[in] label Label to display beside the input. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in] itemWidth Width of the input element(s) + unit combo
/// @param[in] unitWidth Width of the unit combo
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_min Minimum value allowed
/// @param[in] v_max Maximum value allowed
/// @param[in, out] combo_current_item The selected item in the unit combo
/// @param[in] combo_items_separated_by_zeros Items to display in the unit combo (separated by \0 and ends with \0\0)
/// @param[in] format Printf format to display the value with
/// @param[in] flags Slider flags to modify the behavior
/// @param[in] combo_popup_max_height_in_items Maximum height of the combo in number of items
/// @return Returns if the value or unit was changed
template<typename U>
InputWithUnitChange SliderFloat2WithUnit(const char* label, float itemWidth, float unitWidth,
                                         float v[2], float v_min, float v_max, U& combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                         const char* format = "%.3f", ImGuiSliderFlags flags = 0,
                                         int combo_popup_max_height_in_items = -1)
{
    return internal::SliderWithUnit<ImGuiDataType_Float, 2, float>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, v_min, v_max, format, flags, combo_popup_max_height_in_items);
}

/// @brief Shows an Slider GUI element to modify the provided value and also set its unit
/// @param[in] label Label to display beside the input. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in] itemWidth Width of the input element(s) + unit combo
/// @param[in] unitWidth Width of the unit combo
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_min Minimum value allowed
/// @param[in] v_max Maximum value allowed
/// @param[in, out] combo_current_item The selected item in the unit combo
/// @param[in] combo_items_separated_by_zeros Items to display in the unit combo (separated by \0 and ends with \0\0)
/// @param[in] format Printf format to display the value with
/// @param[in] flags Slider flags to modify the behavior
/// @param[in] combo_popup_max_height_in_items Maximum height of the combo in number of items
/// @return Returns if the value or unit was changed
template<typename U>
InputWithUnitChange SliderFloat3WithUnit(const char* label, float itemWidth, float unitWidth,
                                         float v[3], float v_min, float v_max, U& combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                         const char* format = "%.3f", ImGuiSliderFlags flags = 0,
                                         int combo_popup_max_height_in_items = -1)
{
    return internal::SliderWithUnit<ImGuiDataType_Float, 3, float>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, v_min, v_max, format, flags, combo_popup_max_height_in_items);
}

/// @brief Shows an Slider GUI element to modify the provided value and also set its unit
/// @param[in] label Label to display beside the input. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in] itemWidth Width of the input element(s) + unit combo
/// @param[in] unitWidth Width of the unit combo
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_min Minimum value allowed
/// @param[in] v_max Maximum value allowed
/// @param[in, out] combo_current_item The selected item in the unit combo
/// @param[in] combo_items_separated_by_zeros Items to display in the unit combo (separated by \0 and ends with \0\0)
/// @param[in] format Printf format to display the value with
/// @param[in] flags Slider flags to modify the behavior
/// @param[in] combo_popup_max_height_in_items Maximum height of the combo in number of items
/// @return Returns if the value or unit was changed
template<typename U>
InputWithUnitChange SliderFloat4WithUnit(const char* label, float itemWidth, float unitWidth,
                                         float v[4], float v_min, float v_max, U& combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                         const char* format = "%.3f", ImGuiSliderFlags flags = 0,
                                         int combo_popup_max_height_in_items = -1)
{
    return internal::SliderWithUnit<ImGuiDataType_Float, 4, float>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, v_min, v_max, format, flags, combo_popup_max_height_in_items);
}

/// @brief Shows an Slider GUI element to modify the provided value and also set its unit
/// @param[in] label Label to display beside the input. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in] itemWidth Width of the input element(s) + unit combo
/// @param[in] unitWidth Width of the unit combo
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_min Minimum value allowed
/// @param[in] v_max Maximum value allowed
/// @param[in, out] combo_current_item The selected item in the unit combo
/// @param[in] combo_items_separated_by_zeros Items to display in the unit combo (separated by \0 and ends with \0\0)
/// @param[in] format Printf format to display the value with
/// @param[in] flags Slider flags to modify the behavior
/// @param[in] combo_popup_max_height_in_items Maximum height of the combo in number of items
/// @return Returns if the value or unit was changed
template<typename U>
InputWithUnitChange SliderDoubleWithUnit(const char* label, float itemWidth, float unitWidth,
                                         double* v, double v_min, double v_max, U& combo_current_item, const char* combo_items_separated_by_zeros,
                                         const char* format = "%.6f", ImGuiSliderFlags flags = 0,
                                         int combo_popup_max_height_in_items = -1)
{
    return internal::SliderWithUnit<ImGuiDataType_Double, 1, double>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, v_min, v_max, format, flags, combo_popup_max_height_in_items);
}

/// @brief Shows an Slider GUI element to modify the provided value and also set its unit
/// @param[in] label Label to display beside the input. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in] itemWidth Width of the input element(s) + unit combo
/// @param[in] unitWidth Width of the unit combo
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_min Minimum value allowed
/// @param[in] v_max Maximum value allowed
/// @param[in, out] combo_current_item The selected item in the unit combo
/// @param[in] combo_items_separated_by_zeros Items to display in the unit combo (separated by \0 and ends with \0\0)
/// @param[in] format Printf format to display the value with
/// @param[in] flags Slider flags to modify the behavior
/// @param[in] combo_popup_max_height_in_items Maximum height of the combo in number of items
/// @return Returns if the value or unit was changed
template<typename U>
InputWithUnitChange SliderDouble2WithUnit(const char* label, float itemWidth, float unitWidth,
                                          double v[2], double v_min, double v_max, U& combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                          const char* format = "%.6f", ImGuiSliderFlags flags = 0,
                                          int combo_popup_max_height_in_items = -1)
{
    return internal::SliderWithUnit<ImGuiDataType_Double, 2, double>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, v_min, v_max, format, flags, combo_popup_max_height_in_items);
}

/// @brief Shows an Slider GUI element to modify the provided value and also set its unit
/// @param[in] label Label to display beside the input. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in] itemWidth Width of the input element(s) + unit combo
/// @param[in] unitWidth Width of the unit combo
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_min Minimum value allowed
/// @param[in] v_max Maximum value allowed
/// @param[in, out] combo_current_item The selected item in the unit combo
/// @param[in] combo_items_separated_by_zeros Items to display in the unit combo (separated by \0 and ends with \0\0)
/// @param[in] format Printf format to display the value with
/// @param[in] flags Slider flags to modify the behavior
/// @param[in] combo_popup_max_height_in_items Maximum height of the combo in number of items
/// @return Returns if the value or unit was changed
template<typename U>
InputWithUnitChange SliderDouble3WithUnit(const char* label, float itemWidth, float unitWidth,
                                          double v[3], double v_min, double v_max, U& combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                          const char* format = "%.6f", ImGuiSliderFlags flags = 0,
                                          int combo_popup_max_height_in_items = -1)
{
    return internal::SliderWithUnit<ImGuiDataType_Double, 3, double>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, v_min, v_max, format, flags, combo_popup_max_height_in_items);
}

/// @brief Shows an Slider GUI element to modify the provided value and also set its unit
/// @param[in] label Label to display beside the input. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in] itemWidth Width of the input element(s) + unit combo
/// @param[in] unitWidth Width of the unit combo
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_min Minimum value allowed
/// @param[in] v_max Maximum value allowed
/// @param[in, out] combo_current_item The selected item in the unit combo
/// @param[in] combo_items_separated_by_zeros Items to display in the unit combo (separated by \0 and ends with \0\0)
/// @param[in] format Printf format to display the value with
/// @param[in] flags Slider flags to modify the behavior
/// @param[in] combo_popup_max_height_in_items Maximum height of the combo in number of items
/// @return Returns if the value or unit was changed
template<typename U>
InputWithUnitChange SliderDouble4WithUnit(const char* label, float itemWidth, float unitWidth,
                                          double v[4], double v_min, double v_max, U& combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                          const char* format = "%.6f", ImGuiSliderFlags flags = 0,
                                          int combo_popup_max_height_in_items = -1)
{
    return internal::SliderWithUnit<ImGuiDataType_Double, 4, double>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, v_min, v_max, format, flags, combo_popup_max_height_in_items);
}

} // namespace gui::widgets
} // namespace NAV