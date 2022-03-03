#include "InputWithUnit.hpp"

#include <fmt/format.h>

#include "imgui_ex.hpp"

/// @brief Shows an InputText GUI element to modify the provided value and also set its unit
/// @tparam _Scalar ImGui data type for the InputText
/// @tparam _Size Amount of elements in the input value pointer to modify
/// @tparam T Data type for the InputText
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
template<ImGuiDataType_ _Scalar, unsigned int _Size, typename T>
NAV::gui::widgets::InputWithUnitChange InputWithUnit(const char* label, float itemWidth, float unitWidth,
                                                     T v[_Size], int* combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                                     T step, T step_fast, const char* format, ImGuiInputTextFlags flags,
                                                     int combo_popup_max_height_in_items)
{
    NAV::gui::widgets::InputWithUnitChange retVal = NAV::gui::widgets::InputWithUnitChange_None;

    ImGui::SetNextItemWidth(itemWidth - unitWidth);
    if constexpr (_Size == 1)
    {
        flags |= ImGuiInputTextFlags_CharsScientific;
        if (ImGui::InputScalar(fmt::format("##{} - input", label).c_str(), _Scalar, static_cast<void*>(v), static_cast<void*>(step > 0.0 ? &step : nullptr), static_cast<void*>(step_fast > 0.0 ? &step_fast : nullptr), format, flags))
        {
            retVal = NAV::gui::widgets::InputWithUnitChange_Input;
        }
    }
    else
    {
        if (ImGui::InputScalarN(fmt::format("##{} - input", label).c_str(), _Scalar, v, _Size, nullptr, nullptr, format, flags))
        {
            retVal = NAV::gui::widgets::InputWithUnitChange_Input;
        }
    }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(unitWidth);
    if (ImGui::Combo(fmt::format("##{} - unit", label).c_str(), combo_current_item, combo_items_separated_by_zeros, combo_popup_max_height_in_items))
    {
        retVal = NAV::gui::widgets::InputWithUnitChange_Unit;
    }
    ImGui::SameLine();
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() - ImGui::GetStyle().ItemSpacing.x + ImGui::GetStyle().ItemInnerSpacing.x);
    std::string strLabel{ label };
    ImGui::TextUnformatted(strLabel.substr(0, strLabel.find('#')).c_str());

    return retVal;
}

NAV::gui::widgets::InputWithUnitChange NAV::gui::widgets::InputFloatWithUnit(const char* label, float itemWidth, float unitWidth,
                                                                             float* v, int* combo_current_item, const char* combo_items_separated_by_zeros,
                                                                             float step, float step_fast, const char* format, ImGuiInputTextFlags flags,
                                                                             int combo_popup_max_height_in_items)
{
    return InputWithUnit<ImGuiDataType_Float, 1, float>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, step, step_fast, format, flags, combo_popup_max_height_in_items);
}

NAV::gui::widgets::InputWithUnitChange NAV::gui::widgets::InputFloat2WithUnit(const char* label, float itemWidth, float unitWidth,
                                                                              float v[2], int* combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                                                              const char* format, ImGuiInputTextFlags flags,
                                                                              int combo_popup_max_height_in_items)
{
    return InputWithUnit<ImGuiDataType_Float, 2, float>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, 0.0, 0.0, format, flags, combo_popup_max_height_in_items);
}

NAV::gui::widgets::InputWithUnitChange NAV::gui::widgets::InputFloat3WithUnit(const char* label, float itemWidth, float unitWidth,
                                                                              float v[3], int* combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                                                              const char* format, ImGuiInputTextFlags flags,
                                                                              int combo_popup_max_height_in_items)
{
    return InputWithUnit<ImGuiDataType_Float, 3, float>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, 0.0, 0.0, format, flags, combo_popup_max_height_in_items);
}

NAV::gui::widgets::InputWithUnitChange NAV::gui::widgets::InputFloat4WithUnit(const char* label, float itemWidth, float unitWidth,
                                                                              float v[4], int* combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                                                              const char* format, ImGuiInputTextFlags flags,
                                                                              int combo_popup_max_height_in_items)
{
    return InputWithUnit<ImGuiDataType_Float, 4, float>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, 0.0, 0.0, format, flags, combo_popup_max_height_in_items);
}

NAV::gui::widgets::InputWithUnitChange NAV::gui::widgets::InputDoubleWithUnit(const char* label, float itemWidth, float unitWidth,
                                                                              double* v, int* combo_current_item, const char* combo_items_separated_by_zeros,
                                                                              double step, double step_fast, const char* format, ImGuiInputTextFlags flags,
                                                                              int combo_popup_max_height_in_items)
{
    return InputWithUnit<ImGuiDataType_Double, 1, double>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, step, step_fast, format, flags, combo_popup_max_height_in_items);
}

NAV::gui::widgets::InputWithUnitChange NAV::gui::widgets::InputDouble2WithUnit(const char* label, float itemWidth, float unitWidth,
                                                                               double v[2], int* combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                                                               const char* format, ImGuiInputTextFlags flags,
                                                                               int combo_popup_max_height_in_items)
{
    return InputWithUnit<ImGuiDataType_Double, 2, double>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, 0.0, 0.0, format, flags, combo_popup_max_height_in_items);
}

NAV::gui::widgets::InputWithUnitChange NAV::gui::widgets::InputDouble3WithUnit(const char* label, float itemWidth, float unitWidth,
                                                                               double v[3], int* combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                                                               const char* format, ImGuiInputTextFlags flags,
                                                                               int combo_popup_max_height_in_items)
{
    return InputWithUnit<ImGuiDataType_Double, 3, double>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, 0.0, 0.0, format, flags, combo_popup_max_height_in_items);
}

NAV::gui::widgets::InputWithUnitChange NAV::gui::widgets::InputDouble4WithUnit(const char* label, float itemWidth, float unitWidth,
                                                                               double v[4], int* combo_current_item, const char* combo_items_separated_by_zeros, // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                                                               const char* format, ImGuiInputTextFlags flags,
                                                                               int combo_popup_max_height_in_items)
{
    return InputWithUnit<ImGuiDataType_Double, 4, double>(label, itemWidth, unitWidth, v, combo_current_item, combo_items_separated_by_zeros, 0.0, 0.0, format, flags, combo_popup_max_height_in_items);
}