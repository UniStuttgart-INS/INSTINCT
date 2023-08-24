// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "InputWithUnit.hpp"

#include <fmt/format.h>
#include <algorithm>
#include "util/Logger.hpp"

#include "imgui_ex.hpp"

namespace NAV::gui::widgets
{

InputWithUnitChange Unit(const char* label, float unitWidth,
                         int* combo_current_item, const char* combo_items_separated_by_zeros,
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
    if (ImGui::Combo(fmt::format("##{} - unit", label).c_str(), combo_current_item, combo_items_separated_by_zeros, combo_popup_max_height_in_items))
    {
        retVal = InputWithUnitChange_Unit;
    }
    if (disable) { ImGui::EndDisabled(); }
    ImGui::SameLine();
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() - ImGui::GetStyle().ItemSpacing.x + ImGui::GetStyle().ItemInnerSpacing.x);
    std::string strLabel{ label };
    ImGui::TextUnformatted(strLabel.substr(0, strLabel.find('#')).c_str());

    return retVal;
}

template<ImGuiDataType_ _Scalar, unsigned int _Size, typename T>
InputWithUnitChange InputWithUnit(const char* label, float itemWidth, float unitWidth,
                                  T v[_Size], int* combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
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

template<ImGuiDataType_ _Scalar, unsigned int _Size, typename T>
InputWithUnitChange SliderWithUnit(const char* label, float itemWidth, float unitWidth,
                                   T v[_Size], int* combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
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

InputWithUnitChange InputFloatWithUnit(const char* label, float itemWidth, float unitWidth,
                                       float* v, int* combo_current_item, const char* combo_items_separated_by_zeros,
                                       float step, float step_fast, const char* format, ImGuiInputTextFlags flags,
                                       int combo_popup_max_height_in_items)
{
    return InputWithUnit<ImGuiDataType_Float, 1, float>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, step, step_fast, format, flags, combo_popup_max_height_in_items);
}

InputWithUnitChange InputFloat2WithUnit(const char* label, float itemWidth, float unitWidth,
                                        float v[2], int* combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                        const char* format, ImGuiInputTextFlags flags,
                                        int combo_popup_max_height_in_items)
{
    return InputWithUnit<ImGuiDataType_Float, 2, float>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, 0.0, 0.0, format, flags, combo_popup_max_height_in_items);
}

InputWithUnitChange InputFloat3WithUnit(const char* label, float itemWidth, float unitWidth,
                                        float v[3], int* combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                        const char* format, ImGuiInputTextFlags flags,
                                        int combo_popup_max_height_in_items)
{
    return InputWithUnit<ImGuiDataType_Float, 3, float>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, 0.0, 0.0, format, flags, combo_popup_max_height_in_items);
}

InputWithUnitChange InputFloat4WithUnit(const char* label, float itemWidth, float unitWidth,
                                        float v[4], int* combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                        const char* format, ImGuiInputTextFlags flags,
                                        int combo_popup_max_height_in_items)
{
    return InputWithUnit<ImGuiDataType_Float, 4, float>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, 0.0, 0.0, format, flags, combo_popup_max_height_in_items);
}

InputWithUnitChange InputDoubleWithUnit(const char* label, float itemWidth, float unitWidth,
                                        double* v, int* combo_current_item, const char* combo_items_separated_by_zeros,
                                        double step, double step_fast, const char* format, ImGuiInputTextFlags flags,
                                        int combo_popup_max_height_in_items)
{
    return InputWithUnit<ImGuiDataType_Double, 1, double>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, step, step_fast, format, flags, combo_popup_max_height_in_items);
}

InputWithUnitChange InputDouble2WithUnit(const char* label, float itemWidth, float unitWidth,
                                         double v[2], int* combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                         const char* format, ImGuiInputTextFlags flags,
                                         int combo_popup_max_height_in_items)
{
    return InputWithUnit<ImGuiDataType_Double, 2, double>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, 0.0, 0.0, format, flags, combo_popup_max_height_in_items);
}

InputWithUnitChange InputDouble3WithUnit(const char* label, float itemWidth, float unitWidth,
                                         double v[3], int* combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                         const char* format, ImGuiInputTextFlags flags,
                                         int combo_popup_max_height_in_items)
{
    return InputWithUnit<ImGuiDataType_Double, 3, double>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, 0.0, 0.0, format, flags, combo_popup_max_height_in_items);
}

InputWithUnitChange InputDouble4WithUnit(const char* label, float itemWidth, float unitWidth,
                                         double v[4], int* combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                         const char* format, ImGuiInputTextFlags flags,
                                         int combo_popup_max_height_in_items)
{
    return InputWithUnit<ImGuiDataType_Double, 4, double>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, 0.0, 0.0, format, flags, combo_popup_max_height_in_items);
}

// ###########################################################################################################

InputWithUnitChange InputFloatLWithUnit(const char* label, float itemWidth, float unitWidth,
                                        float* v, float v_min, float v_max, int* combo_current_item, const char* combo_items_separated_by_zeros,
                                        float step, float step_fast, const char* format, ImGuiInputTextFlags flags,
                                        int combo_popup_max_height_in_items)
{
    auto change = InputWithUnit<ImGuiDataType_Float, 1, float>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, step, step_fast, format, flags, combo_popup_max_height_in_items);
    if (change == InputWithUnitChange::InputWithUnitChange_Input)
    {
        *v = std::clamp(*v, v_min, v_max);
    }
    return change;
}

InputWithUnitChange InputFloat2LWithUnit(const char* label, float itemWidth, float unitWidth,
                                         float v[2], float v_min, float v_max, int* combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                         const char* format, ImGuiInputTextFlags flags,
                                         int combo_popup_max_height_in_items)
{
    auto change = InputWithUnit<ImGuiDataType_Float, 2, float>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, 0.0, 0.0, format, flags, combo_popup_max_height_in_items);
    if (change == InputWithUnitChange::InputWithUnitChange_Input)
    {
        for (size_t i = 0; i < 2; i++)
        {
            v[i] = std::clamp(v[i], v_min, v_max);
        }
    }
    return change;
}

InputWithUnitChange InputFloat3LWithUnit(const char* label, float itemWidth, float unitWidth,
                                         float v[3], float v_min, float v_max, int* combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                         const char* format, ImGuiInputTextFlags flags,
                                         int combo_popup_max_height_in_items)
{
    auto change = InputWithUnit<ImGuiDataType_Float, 3, float>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, 0.0, 0.0, format, flags, combo_popup_max_height_in_items);
    if (change == InputWithUnitChange::InputWithUnitChange_Input)
    {
        for (size_t i = 0; i < 3; i++)
        {
            v[i] = std::clamp(v[i], v_min, v_max);
        }
    }
    return change;
}

InputWithUnitChange InputFloat4LWithUnit(const char* label, float itemWidth, float unitWidth,
                                         float v[4], float v_min, float v_max, int* combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                         const char* format, ImGuiInputTextFlags flags,
                                         int combo_popup_max_height_in_items)
{
    auto change = InputWithUnit<ImGuiDataType_Float, 4, float>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, 0.0, 0.0, format, flags, combo_popup_max_height_in_items);
    if (change == InputWithUnitChange::InputWithUnitChange_Input)
    {
        for (size_t i = 0; i < 4; i++)
        {
            v[i] = std::clamp(v[i], v_min, v_max);
        }
    }
    return change;
}

InputWithUnitChange InputDoubleLWithUnit(const char* label, float itemWidth, float unitWidth,
                                         double* v, double v_min, double v_max, int* combo_current_item, const char* combo_items_separated_by_zeros,
                                         double step, double step_fast, const char* format, ImGuiInputTextFlags flags,
                                         int combo_popup_max_height_in_items)
{
    auto change = InputWithUnit<ImGuiDataType_Double, 1, double>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, step, step_fast, format, flags, combo_popup_max_height_in_items);
    if (change == InputWithUnitChange::InputWithUnitChange_Input)
    {
        *v = std::clamp(*v, v_min, v_max);
    }
    return change;
}

InputWithUnitChange InputDouble2LWithUnit(const char* label, float itemWidth, float unitWidth,
                                          double v[2], double v_min, double v_max, int* combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                          const char* format, ImGuiInputTextFlags flags,
                                          int combo_popup_max_height_in_items)
{
    auto change = InputWithUnit<ImGuiDataType_Double, 2, double>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, 0.0, 0.0, format, flags, combo_popup_max_height_in_items);
    if (change == InputWithUnitChange::InputWithUnitChange_Input)
    {
        for (size_t i = 0; i < 2; i++)
        {
            v[i] = std::clamp(v[i], v_min, v_max);
        }
    }
    return change;
}

InputWithUnitChange InputDouble3LWithUnit(const char* label, float itemWidth, float unitWidth,
                                          double v[3], double v_min, double v_max, int* combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                          const char* format, ImGuiInputTextFlags flags,
                                          int combo_popup_max_height_in_items)
{
    auto change = InputWithUnit<ImGuiDataType_Double, 3, double>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, 0.0, 0.0, format, flags, combo_popup_max_height_in_items);
    if (change == InputWithUnitChange::InputWithUnitChange_Input)
    {
        for (size_t i = 0; i < 3; i++)
        {
            v[i] = std::clamp(v[i], v_min, v_max);
        }
    }
    return change;
}

InputWithUnitChange InputDouble4LWithUnit(const char* label, float itemWidth, float unitWidth,
                                          double v[4], double v_min, double v_max, int* combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                          const char* format, ImGuiInputTextFlags flags,
                                          int combo_popup_max_height_in_items)
{
    auto change = InputWithUnit<ImGuiDataType_Double, 4, double>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, 0.0, 0.0, format, flags, combo_popup_max_height_in_items);
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

InputWithUnitChange SliderFloatWithUnit(const char* label, float itemWidth, float unitWidth,
                                        float* v, float v_min, float v_max, int* combo_current_item, const char* combo_items_separated_by_zeros,
                                        const char* format, ImGuiSliderFlags flags,
                                        int combo_popup_max_height_in_items)
{
    return SliderWithUnit<ImGuiDataType_Float, 1, float>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, v_min, v_max, format, flags, combo_popup_max_height_in_items);
}

InputWithUnitChange SliderFloat2WithUnit(const char* label, float itemWidth, float unitWidth,
                                         float v[2], float v_min, float v_max, int* combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                         const char* format, ImGuiSliderFlags flags,
                                         int combo_popup_max_height_in_items)
{
    return SliderWithUnit<ImGuiDataType_Float, 2, float>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, v_min, v_max, format, flags, combo_popup_max_height_in_items);
}

InputWithUnitChange SliderFloat3WithUnit(const char* label, float itemWidth, float unitWidth,
                                         float v[3], float v_min, float v_max, int* combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                         const char* format, ImGuiSliderFlags flags,
                                         int combo_popup_max_height_in_items)
{
    return SliderWithUnit<ImGuiDataType_Float, 3, float>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, v_min, v_max, format, flags, combo_popup_max_height_in_items);
}

InputWithUnitChange SliderFloat4WithUnit(const char* label, float itemWidth, float unitWidth,
                                         float v[4], float v_min, float v_max, int* combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                         const char* format, ImGuiSliderFlags flags,
                                         int combo_popup_max_height_in_items)
{
    return SliderWithUnit<ImGuiDataType_Float, 4, float>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, v_min, v_max, format, flags, combo_popup_max_height_in_items);
}

InputWithUnitChange SliderDoubleWithUnit(const char* label, float itemWidth, float unitWidth,
                                         double* v, double v_min, double v_max, int* combo_current_item, const char* combo_items_separated_by_zeros,
                                         const char* format, ImGuiSliderFlags flags,
                                         int combo_popup_max_height_in_items)
{
    return SliderWithUnit<ImGuiDataType_Double, 1, double>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, v_min, v_max, format, flags, combo_popup_max_height_in_items);
}

InputWithUnitChange SliderDouble2WithUnit(const char* label, float itemWidth, float unitWidth,
                                          double v[2], double v_min, double v_max, int* combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                          const char* format, ImGuiSliderFlags flags,
                                          int combo_popup_max_height_in_items)
{
    return SliderWithUnit<ImGuiDataType_Double, 2, double>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, v_min, v_max, format, flags, combo_popup_max_height_in_items);
}

InputWithUnitChange SliderDouble3WithUnit(const char* label, float itemWidth, float unitWidth,
                                          double v[3], double v_min, double v_max, int* combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                          const char* format, ImGuiSliderFlags flags,
                                          int combo_popup_max_height_in_items)
{
    return SliderWithUnit<ImGuiDataType_Double, 3, double>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, v_min, v_max, format, flags, combo_popup_max_height_in_items);
}

InputWithUnitChange SliderDouble4WithUnit(const char* label, float itemWidth, float unitWidth,
                                          double v[4], double v_min, double v_max, int* combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                          const char* format, ImGuiSliderFlags flags,
                                          int combo_popup_max_height_in_items)
{
    return SliderWithUnit<ImGuiDataType_Double, 4, double>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, v_min, v_max, format, flags, combo_popup_max_height_in_items);
}

} // namespace NAV::gui::widgets