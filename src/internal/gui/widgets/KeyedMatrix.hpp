// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file KeyedMatrix.hpp
/// @brief Widgets related to KeyedMatrices
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-08-04

#pragma once

#include <imgui.h>
#include <application.h>
#include "util/Container/KeyedMatrix.hpp"

namespace NAV::gui::widgets
{

/// @brief Shows GUI elements to display the coefficients of a matrix
/// @tparam Scalar Numeric type, e.g. float, double, int or std::complex<float>.
/// @tparam RowKeyType Type of the key used for row lookup
/// @tparam ColKeyType Type of the key used for col lookup
/// @tparam Rows Number of rows, or \b Dynamic
/// @tparam Cols Number of columns, or \b Dynamic
/// @param[in] label Label to display beside. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in] matrix Pointer to the matrix to display
/// @param[in] tableHeight Height of the Table to show scrollbar afterwards (-1 means no scrollbar)
/// @param[in] tableFlags Flags to modify the Table behaviour
template<typename Scalar, typename RowKeyType, typename ColKeyType, int Rows, int Cols>
void KeyedMatrixView(const char* label, const KeyedMatrix<Scalar, RowKeyType, ColKeyType, Rows, Cols>* matrix,
                     float tableHeight = -1.0F,
                     ImGuiTableFlags tableFlags = ImGuiTableFlags_Borders
                                                  | ImGuiTableFlags_NoHostExtendX
                                                  | ImGuiTableFlags_SizingFixedFit
                                                  | ImGuiTableFlags_ScrollX)
{
    ImGui::PushFont(Application::MonoFont());
    ImVec2 outer_size = ImVec2(0.0F, tableHeight > 0.0F ? tableHeight
                                                        : ImGui::GetTextLineHeightWithSpacing() * static_cast<float>(matrix->rows() + 1));
    if (ImGui::BeginTable(label, static_cast<int>(matrix->cols()) + 1, tableFlags, outer_size))
    {
        ImGui::TableSetupScrollFreeze(1, 1);
        ImGui::TableSetupColumn(""); // Colum headers

        constexpr size_t colMinLength = 10UL;
        std::vector<size_t> colKeysLength;
        colKeysLength.reserve(static_cast<size_t>(matrix->cols()));

        for (int64_t col = 0; col < matrix->cols(); col++)
        {
            std::string colKeyStr = fmt::format("{}", matrix->colKeys().at(static_cast<size_t>(col)));
            colKeysLength.push_back(colKeyStr.length());
            ImGui::TableSetupColumn(colKeyStr.c_str());
        }
        ImGui::TableHeadersRow();

        for (int64_t row = 0; row < matrix->rows(); row++)
        {
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(fmt::format("{}", matrix->rowKeys().at(static_cast<size_t>(row))).c_str());
            ImU32 cell_bg_color = ImGui::GetColorU32(ImGui::GetStyle().Colors[ImGuiCol_TableHeaderBg]);
            ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, cell_bg_color);

            for (int64_t col = 0; col < matrix->cols(); col++)
            {
                ImGui::TableNextColumn();

                auto colLength = std::max(colKeysLength.at(static_cast<size_t>(col)), colMinLength);
                std::string text = fmt::format(" {:> {}.{}g}", (*matrix)(NAV::all, NAV::all)(row, col), colLength, colLength - 2);
                if (text.length() > colLength)
                {
                    text = fmt::format(" {:> {}.{}g}", (*matrix)(NAV::all, NAV::all)(row, col), colLength, colLength - 6);
                }
                ImGui::TextUnformatted(text.c_str());
                if (ImGui::IsItemHovered())
                {
                    ImGui::SetTooltip("%.8g", (*matrix)(NAV::all, NAV::all)(row, col));
                }
            }
        }
        ImGui::EndTable();
    }
    ImGui::PopFont();
}

/// @brief Shows GUI elements to display the coefficients of a matrix
/// @tparam Scalar Numeric type, e.g. float, double, int or std::complex<float>.
/// @tparam RowKeyType Type of the key used for row lookup
/// @tparam Rows Number of rows, or \b Dynamic
/// @param[in] label Label to display beside. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in] matrix Pointer to the matrix to display
/// @param[in] tableHeight Height of the Table to show scrollbar afterwards (-1 means no scrollbar)
/// @param[in] tableFlags Flags to modify the Table behaviour
template<typename Scalar, typename RowKeyType, int Rows>
void KeyedVectorView(const char* label, const KeyedVector<Scalar, RowKeyType, Rows>* matrix,
                     float tableHeight = -1.0F,
                     ImGuiTableFlags tableFlags = ImGuiTableFlags_Borders
                                                  | ImGuiTableFlags_NoHostExtendX
                                                  | ImGuiTableFlags_SizingFixedFit)
{
    ImGui::PushFont(Application::MonoFont());
    ImVec2 outer_size = ImVec2(0.0F, tableHeight > 0.0F ? tableHeight
                                                        : ImGui::GetTextLineHeightWithSpacing() * static_cast<float>(matrix->rows()));
    if (ImGui::BeginTable(label, 2, tableFlags, outer_size))
    {
        ImGui::TableSetupScrollFreeze(1, 0);

        constexpr size_t colLength = 10UL;

        for (int64_t row = 0; row < matrix->rows(); row++)
        {
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(fmt::format("{}", matrix->rowKeys().at(static_cast<size_t>(row))).c_str());
            ImU32 cell_bg_color = ImGui::GetColorU32(ImGui::GetStyle().Colors[ImGuiCol_TableHeaderBg]);
            ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, cell_bg_color);

            ImGui::TableNextColumn();

            std::string text = fmt::format(" {:> {}.{}g}", (*matrix)(NAV::all)(row), colLength, colLength - 2);
            if (text.length() > colLength)
            {
                text = fmt::format(" {:> {}.{}g}", (*matrix)(NAV::all)(row), colLength, colLength - 6);
            }
            ImGui::TextUnformatted(text.c_str());
            if (ImGui::IsItemHovered())
            {
                ImGui::SetTooltip("%.8g", (*matrix)(NAV::all)(row));
            }
        }
        ImGui::EndTable();
    }
    ImGui::PopFont();
}

/// @brief Shows GUI elements to display the coefficients of a matrix
/// @tparam Scalar Numeric type, e.g. float, double, int or std::complex<float>.
/// @tparam ColKeyType Type of the key used for col lookup
/// @tparam Cols Number of columns, or \b Dynamic
/// @param[in] label Label to display beside. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in] matrix Pointer to the matrix to display
/// @param[in] tableFlags Flags to modify the Table behaviour
template<typename Scalar, typename ColKeyType, int Cols>
void KeyedRowVectorView(const char* label, const KeyedRowVector<Scalar, ColKeyType, Cols>* matrix,
                        ImGuiTableFlags tableFlags = ImGuiTableFlags_Borders
                                                     | ImGuiTableFlags_NoHostExtendX
                                                     | ImGuiTableFlags_SizingFixedFit
                                                     | ImGuiTableFlags_ScrollX)
{
    ImGui::PushFont(Application::MonoFont());
    ImVec2 outer_size = ImVec2(0.0F, ImGui::GetTextLineHeightWithSpacing() * 2.0F);
    if (ImGui::BeginTable(label, static_cast<int>(matrix->cols()) + 1, tableFlags, outer_size))
    {
        ImGui::TableSetupScrollFreeze(0, 1);

        constexpr size_t colMinLength = 10UL;
        std::vector<size_t> colKeysLength;
        colKeysLength.reserve(static_cast<size_t>(matrix->cols()));

        for (int64_t col = 0; col < matrix->cols(); col++)
        {
            std::string colKeyStr = fmt::format("{}", matrix->colKeys().at(static_cast<size_t>(col)));
            colKeysLength.push_back(colKeyStr.length());
            ImGui::TableSetupColumn(colKeyStr.c_str());
        }
        ImGui::TableHeadersRow();

        for (int64_t col = 0; col < matrix->cols(); col++)
        {
            ImGui::TableNextColumn();

            auto colLength = std::max(colKeysLength.at(static_cast<size_t>(col)), colMinLength);
            std::string text = fmt::format(" {:> {}.{}g}", (*matrix)(NAV::all)(col), colLength, colLength - 2);
            if (text.length() > colLength)
            {
                text = fmt::format(" {:> {}.{}g}", (*matrix)(NAV::all)(col), colLength, colLength - 6);
            }
            ImGui::TextUnformatted(text.c_str());
            if (ImGui::IsItemHovered())
            {
                ImGui::SetTooltip("%.8g", (*matrix)(NAV::all)(col));
            }
        }

        ImGui::EndTable();
    }
    ImGui::PopFont();
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-nonliteral"

/// @brief Shows GUI elements to modify the coefficients of a matrix with
/// @tparam Scalar Numeric type, e.g. float, double, int or std::complex<float>.
/// @tparam RowKeyType Type of the key used for row lookup
/// @tparam ColKeyType Type of the key used for col lookup
/// @tparam Rows Number of rows, or \b Dynamic
/// @tparam Cols Number of columns, or \b Dynamic
/// @param[in] label Label to display beside. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in, out] matrix Pointer to the matrix to modify
/// @param[in] tableHeight Height of the Table to show scrollbar afterwards (-1 means no scrollbar)
/// @param[in] tableFlags Flags to modify the Table behaviour
/// @param[in] step Step size of the InputText
/// @param[in] step_fast Fast step size of the InputText
/// @param[in] format Printf format to display the value with
/// @param[in] inputTextFlags InputText flags to modify the behavior of the input fields
/// @return True if the value was changed
template<typename Scalar, typename RowKeyType, typename ColKeyType, int Rows, int Cols>
bool InputKeyedMatrix(const char* label, KeyedMatrix<Scalar, RowKeyType, ColKeyType, Rows, Cols>* matrix,
                      float tableHeight = -1.0F,
                      ImGuiTableFlags tableFlags = ImGuiTableFlags_Borders
                                                   | ImGuiTableFlags_NoHostExtendX
                                                   | ImGuiTableFlags_SizingFixedFit
                                                   | ImGuiTableFlags_ScrollX,
                      double step = 0.0, double step_fast = 0.0,
                      const char* format = "%.4g",
                      ImGuiInputTextFlags inputTextFlags = ImGuiInputTextFlags_None)
{
    bool changed = false;
    ImGui::PushFont(Application::MonoFont());
    const float TEXT_BASE_WIDTH = ImGui::CalcTextSize("A").x;
    ImVec2 outer_size = ImVec2(0.0F, tableHeight > 0.0F ? tableHeight
                                                        : ImGui::GetTextLineHeightWithSpacing()
                                                              + (ImGui::GetTextLineHeightWithSpacing() + 1.5F * ImGui::GetStyle().ItemSpacing.y)
                                                                    * static_cast<float>(matrix->rows()));
    if (ImGui::BeginTable(label, static_cast<int>(matrix->cols()) + 1, tableFlags, outer_size))
    {
        ImGui::TableSetupScrollFreeze(1, 1);
        ImGui::TableSetupColumn(""); // Colum headers

        constexpr size_t colMinLength = 11UL;
        std::vector<size_t> colKeysLength;
        colKeysLength.reserve(static_cast<size_t>(matrix->cols()));

        for (int64_t col = 0; col < matrix->cols(); col++)
        {
            std::string colKeyStr = fmt::format("{}", matrix->colKeys().at(static_cast<size_t>(col)));
            colKeysLength.push_back(colKeyStr.length());
            ImGui::TableSetupColumn(colKeyStr.c_str());
        }
        ImGui::TableHeadersRow();

        for (int64_t row = 0; row < matrix->rows(); row++)
        {
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(fmt::format("{}", matrix->rowKeys().at(static_cast<size_t>(row))).c_str());
            ImU32 cell_bg_color = ImGui::GetColorU32(ImGui::GetStyle().Colors[ImGuiCol_TableHeaderBg]);
            ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, cell_bg_color);

            for (int64_t col = 0; col < matrix->cols(); col++)
            {
                ImGui::TableNextColumn();

                auto colLength = std::max(colKeysLength.at(static_cast<size_t>(col)), colMinLength);
                ImGui::SetNextItemWidth(TEXT_BASE_WIDTH * static_cast<float>(colLength));
                if (ImGui::InputDouble(fmt::format("##{} ({}, {})", label, row, col).c_str(), &(*matrix)(NAV::all, NAV::all)(row, col), step, step_fast, format, inputTextFlags))
                {
                    changed = true;
                }
                if (ImGui::IsItemHovered())
                {
                    ImGui::SetTooltip("%.8g", (*matrix)(NAV::all, NAV::all)(row, col));
                }
            }
        }
        ImGui::EndTable();
    }
    ImGui::PopFont();

    return changed;
}

/// @brief Shows GUI elements to modify the coefficients of a matrix with
/// @tparam Scalar Numeric type, e.g. float, double, int or std::complex<float>.
/// @tparam RowKeyType Type of the key used for row lookup
/// @tparam Rows Number of rows, or \b Dynamic
/// @param[in] label Label to display beside. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in, out] matrix Pointer to the matrix to modify
/// @param[in] tableHeight Height of the Table to show scrollbar afterwards (-1 means no scrollbar)
/// @param[in] tableFlags Flags to modify the Table behaviour
/// @param[in] step Step size of the InputText
/// @param[in] step_fast Fast step size of the InputText
/// @param[in] format Printf format to display the value with
/// @param[in] inputTextFlags InputText flags to modify the behavior of the input fields
/// @return True if the value was changed
template<typename Scalar, typename RowKeyType, int Rows>
bool InputKeyedVector(const char* label, KeyedVector<Scalar, RowKeyType, Rows>* matrix,
                      float tableHeight = -1.0F,
                      ImGuiTableFlags tableFlags = ImGuiTableFlags_Borders
                                                   | ImGuiTableFlags_NoHostExtendX
                                                   | ImGuiTableFlags_SizingFixedFit
                                                   | ImGuiTableFlags_ScrollX,
                      double step = 0.0, double step_fast = 0.0,
                      const char* format = "%.4g",
                      ImGuiInputTextFlags inputTextFlags = ImGuiInputTextFlags_None)
{
    bool changed = false;
    ImGui::PushFont(Application::MonoFont());
    const float TEXT_BASE_WIDTH = ImGui::CalcTextSize("A").x;
    ImVec2 outer_size = ImVec2(0.0F, tableHeight > 0.0F ? tableHeight
                                                        : (ImGui::GetTextLineHeightWithSpacing() + 1.5F * ImGui::GetStyle().ItemSpacing.y)
                                                              * static_cast<float>(matrix->rows()));
    if (ImGui::BeginTable(label, 2, tableFlags, outer_size))
    {
        ImGui::TableSetupScrollFreeze(1, 0);

        constexpr size_t colLength = 11UL;

        for (int64_t row = 0; row < matrix->rows(); row++)
        {
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(fmt::format("{}", matrix->rowKeys().at(static_cast<size_t>(row))).c_str());
            ImU32 cell_bg_color = ImGui::GetColorU32(ImGui::GetStyle().Colors[ImGuiCol_TableHeaderBg]);
            ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, cell_bg_color);

            ImGui::TableNextColumn();

            ImGui::SetNextItemWidth(TEXT_BASE_WIDTH * static_cast<float>(colLength));
            if (ImGui::InputDouble(fmt::format("##{} ({})", label, row).c_str(), &(*matrix)(NAV::all)(row), step, step_fast, format, inputTextFlags))
            {
                changed = true;
            }
            if (ImGui::IsItemHovered())
            {
                ImGui::SetTooltip("%.8g", (*matrix)(NAV::all)(row));
            }
        }
        ImGui::EndTable();
    }
    ImGui::PopFont();

    return changed;
}

/// @brief Shows GUI elements to modify the coefficients of a matrix with
/// @tparam Scalar Numeric type, e.g. float, double, int or std::complex<float>.
/// @tparam ColKeyType Type of the key used for col lookup
/// @tparam Cols Number of columns, or \b Dynamic
/// @param[in] label Label to display beside. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in, out] matrix Pointer to the matrix to modify
/// @param[in] tableFlags Flags to modify the Table behaviour
/// @param[in] step Step size of the InputText
/// @param[in] step_fast Fast step size of the InputText
/// @param[in] format Printf format to display the value with
/// @param[in] inputTextFlags InputText flags to modify the behavior of the input fields
/// @return True if the value was changed
template<typename Scalar, typename ColKeyType, int Cols>
bool InputKeyedRowVector(const char* label, KeyedRowVector<Scalar, ColKeyType, Cols>* matrix,
                         ImGuiTableFlags tableFlags = ImGuiTableFlags_Borders
                                                      | ImGuiTableFlags_NoHostExtendX
                                                      | ImGuiTableFlags_SizingFixedFit
                                                      | ImGuiTableFlags_ScrollX,
                         double step = 0.0, double step_fast = 0.0,
                         const char* format = "%.4g",
                         ImGuiInputTextFlags inputTextFlags = ImGuiInputTextFlags_None)
{
    bool changed = false;
    ImGui::PushFont(Application::MonoFont());
    const float TEXT_BASE_WIDTH = ImGui::CalcTextSize("A").x;
    ImVec2 outer_size = ImVec2(0.0F, ImGui::GetTextLineHeightWithSpacing()
                                         + (ImGui::GetTextLineHeightWithSpacing() + 1.5F * ImGui::GetStyle().ItemSpacing.y));
    if (ImGui::BeginTable(label, static_cast<int>(matrix->cols()) + 1, tableFlags, outer_size))
    {
        ImGui::TableSetupScrollFreeze(0, 1);

        constexpr size_t colMinLength = 11UL;
        std::vector<size_t> colKeysLength;
        colKeysLength.reserve(static_cast<size_t>(matrix->cols()));

        for (int64_t col = 0; col < matrix->cols(); col++)
        {
            std::string colKeyStr = fmt::format("{}", matrix->colKeys().at(static_cast<size_t>(col)));
            colKeysLength.push_back(colKeyStr.length());
            ImGui::TableSetupColumn(colKeyStr.c_str());
        }
        ImGui::TableHeadersRow();

        for (int64_t col = 0; col < matrix->cols(); col++)
        {
            ImGui::TableNextColumn();

            auto colLength = std::max(colKeysLength.at(static_cast<size_t>(col)), colMinLength);
            ImGui::SetNextItemWidth(TEXT_BASE_WIDTH * static_cast<float>(colLength));
            if (ImGui::InputDouble(fmt::format("##{} ({})", label, col).c_str(), &(*matrix)(NAV::all)(col), step, step_fast, format, inputTextFlags))
            {
                changed = true;
            }
            if (ImGui::IsItemHovered())
            {
                ImGui::SetTooltip("%.8g", (*matrix)(NAV::all)(col));
            }
        }

        ImGui::EndTable();
    }
    ImGui::PopFont();

    return changed;
}

#pragma GCC diagnostic pop

} // namespace NAV::gui::widgets