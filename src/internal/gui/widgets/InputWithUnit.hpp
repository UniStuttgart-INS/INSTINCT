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

#include <imgui.h>

namespace NAV::gui::widgets
{
/// Return value signaling that the input or the unit changed
enum InputWithUnitChange : int
{
    InputWithUnitChange_None = 0, ///< Nothing changed
    InputWithUnitChange_Input,    ///< The Input changed
    InputWithUnitChange_Unit,     ///< The Unit changed
};

/// @brief Shows an InputText GUI element to modify the provided value and also set its unit
/// @param[in] label Label to display beside the input. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in] itemWidth Width of the input element(s) + unit combo
/// @param[in] unitWidth Width of the unit combo
/// @param[in, out] v Pointer to the value to modify
/// @param[in, out] combo_current_item Pointer to the selected item in the unit combo
/// @param[in] combo_items_separated_by_zeros Items to display in the unit combo (separated by \0 and ends with \0\0)
/// @param[in] step Step size of the InputText
/// @param[in] step_fast Fast step size of the InputText
/// @param[in] format Printf format to display the value with
/// @param[in] flags InputText flags to modify the behavior
/// @param[in] combo_popup_max_height_in_items Maximum height of the combo in number of items
/// @return Returns if the value or unit was changed
InputWithUnitChange InputFloatWithUnit(const char* label, float itemWidth, float unitWidth,
                                       float* v, int* combo_current_item, const char* combo_items_separated_by_zeros,
                                       float step = 0.0, float step_fast = 0.0, const char* format = "%.3f", ImGuiInputTextFlags flags = 0,
                                       int combo_popup_max_height_in_items = -1);

/// @brief Shows an InputText GUI element to modify the provided value and also set its unit
/// @param[in] label Label to display beside the input. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in] itemWidth Width of the input element(s) + unit combo
/// @param[in] unitWidth Width of the unit combo
/// @param[in, out] v Pointer to the value to modify
/// @param[in, out] combo_current_item Pointer to the selected item in the unit combo
/// @param[in] combo_items_separated_by_zeros Items to display in the unit combo (separated by \0 and ends with \0\0)
/// @param[in] format Printf format to display the value with
/// @param[in] flags InputText flags to modify the behavior
/// @param[in] combo_popup_max_height_in_items Maximum height of the combo in number of items
/// @return Returns if the value or unit was changed
InputWithUnitChange InputFloat2WithUnit(const char* label, float itemWidth, float unitWidth,
                                        float v[2], int* combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                        const char* format = "%.3f", ImGuiInputTextFlags flags = 0,
                                        int combo_popup_max_height_in_items = -1);

/// @brief Shows an InputText GUI element to modify the provided value and also set its unit
/// @param[in] label Label to display beside the input. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in] itemWidth Width of the input element(s) + unit combo
/// @param[in] unitWidth Width of the unit combo
/// @param[in, out] v Pointer to the value to modify
/// @param[in, out] combo_current_item Pointer to the selected item in the unit combo
/// @param[in] combo_items_separated_by_zeros Items to display in the unit combo (separated by \0 and ends with \0\0)
/// @param[in] format Printf format to display the value with
/// @param[in] flags InputText flags to modify the behavior
/// @param[in] combo_popup_max_height_in_items Maximum height of the combo in number of items
/// @return Returns if the value or unit was changed
InputWithUnitChange InputFloat3WithUnit(const char* label, float itemWidth, float unitWidth,
                                        float v[3], int* combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                        const char* format = "%.3f", ImGuiInputTextFlags flags = 0,
                                        int combo_popup_max_height_in_items = -1);

/// @brief Shows an InputText GUI element to modify the provided value and also set its unit
/// @param[in] label Label to display beside the input. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in] itemWidth Width of the input element(s) + unit combo
/// @param[in] unitWidth Width of the unit combo
/// @param[in, out] v Pointer to the value to modify
/// @param[in, out] combo_current_item Pointer to the selected item in the unit combo
/// @param[in] combo_items_separated_by_zeros Items to display in the unit combo (separated by \0 and ends with \0\0)
/// @param[in] format Printf format to display the value with
/// @param[in] flags InputText flags to modify the behavior
/// @param[in] combo_popup_max_height_in_items Maximum height of the combo in number of items
/// @return Returns if the value or unit was changed
InputWithUnitChange InputFloat4WithUnit(const char* label, float itemWidth, float unitWidth,
                                        float v[4], int* combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                        const char* format = "%.3f", ImGuiInputTextFlags flags = 0,
                                        int combo_popup_max_height_in_items = -1);

/// @brief Shows an InputText GUI element to modify the provided value and also set its unit
/// @param[in] label Label to display beside the input. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in] itemWidth Width of the input element(s) + unit combo
/// @param[in] unitWidth Width of the unit combo
/// @param[in, out] v Pointer to the value to modify
/// @param[in, out] combo_current_item Pointer to the selected item in the unit combo
/// @param[in] combo_items_separated_by_zeros Items to display in the unit combo (separated by \0 and ends with \0\0)
/// @param[in] step Step size of the InputText
/// @param[in] step_fast Fast step size of the InputText
/// @param[in] format Printf format to display the value with
/// @param[in] flags InputText flags to modify the behavior
/// @param[in] combo_popup_max_height_in_items Maximum height of the combo in number of items
/// @return Returns if the value or unit was changed
InputWithUnitChange InputDoubleWithUnit(const char* label, float itemWidth, float unitWidth,
                                        double* v, int* combo_current_item, const char* combo_items_separated_by_zeros,
                                        double step = 0.0, double step_fast = 0.0, const char* format = "%.6f", ImGuiInputTextFlags flags = 0,
                                        int combo_popup_max_height_in_items = -1);

/// @brief Shows an InputText GUI element to modify the provided value and also set its unit
/// @param[in] label Label to display beside the input. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in] itemWidth Width of the input element(s) + unit combo
/// @param[in] unitWidth Width of the unit combo
/// @param[in, out] v Pointer to the value to modify
/// @param[in, out] combo_current_item Pointer to the selected item in the unit combo
/// @param[in] combo_items_separated_by_zeros Items to display in the unit combo (separated by \0 and ends with \0\0)
/// @param[in] format Printf format to display the value with
/// @param[in] flags InputText flags to modify the behavior
/// @param[in] combo_popup_max_height_in_items Maximum height of the combo in number of items
/// @return Returns if the value or unit was changed
InputWithUnitChange InputDouble2WithUnit(const char* label, float itemWidth, float unitWidth,
                                         double v[2], int* combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                         const char* format = "%.6f", ImGuiInputTextFlags flags = 0,
                                         int combo_popup_max_height_in_items = -1);

/// @brief Shows an InputText GUI element to modify the provided value and also set its unit
/// @param[in] label Label to display beside the input. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in] itemWidth Width of the input element(s) + unit combo
/// @param[in] unitWidth Width of the unit combo
/// @param[in, out] v Pointer to the value to modify
/// @param[in, out] combo_current_item Pointer to the selected item in the unit combo
/// @param[in] combo_items_separated_by_zeros Items to display in the unit combo (separated by \0 and ends with \0\0)
/// @param[in] format Printf format to display the value with
/// @param[in] flags InputText flags to modify the behavior
/// @param[in] combo_popup_max_height_in_items Maximum height of the combo in number of items
/// @return Returns if the value or unit was changed
InputWithUnitChange InputDouble3WithUnit(const char* label, float itemWidth, float unitWidth,
                                         double v[3], int* combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                         const char* format = "%.6f", ImGuiInputTextFlags flags = 0,
                                         int combo_popup_max_height_in_items = -1);

/// @brief Shows an InputText GUI element to modify the provided value and also set its unit
/// @param[in] label Label to display beside the input. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in] itemWidth Width of the input element(s) + unit combo
/// @param[in] unitWidth Width of the unit combo
/// @param[in, out] v Pointer to the value to modify
/// @param[in, out] combo_current_item Pointer to the selected item in the unit combo
/// @param[in] combo_items_separated_by_zeros Items to display in the unit combo (separated by \0 and ends with \0\0)
/// @param[in] format Printf format to display the value with
/// @param[in] flags InputText flags to modify the behavior
/// @param[in] combo_popup_max_height_in_items Maximum height of the combo in number of items
/// @return Returns if the value or unit was changed
InputWithUnitChange InputDouble4WithUnit(const char* label, float itemWidth, float unitWidth,
                                         double v[4], int* combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                         const char* format = "%.6f", ImGuiInputTextFlags flags = 0,
                                         int combo_popup_max_height_in_items = -1);

} // namespace NAV::gui::widgets