// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Matrix.hpp
/// @brief Widgets related to Matrices
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-10-04

#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <imgui.h>

/// @brief Flags for MatrixView GUI elements @ref GuiMatrixViewFlags_
using GuiMatrixViewFlags = int;

/// @brief Flags to select the MatrixView behaviour
enum GuiMatrixViewFlags_ : uint8_t
{
    GuiMatrixViewFlags_None = 0,                                                                ///< None
    GuiMatrixViewFlags_RowHeader = 1 << 0,                                                      ///< Print the Row Header
    GuiMatrixViewFlags_ColumnHeader = 1 << 1,                                                   ///< Print the Col Header
    GuiMatrixViewFlags_Header = GuiMatrixViewFlags_RowHeader | GuiMatrixViewFlags_ColumnHeader, ///< Print all Header
};

namespace NAV::gui::widgets
{
#if defined(__GNUC__) || defined(__clang__)
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wformat-nonliteral"
#endif

/// @brief Shows GUI elements to display the coefficients of a matrix
/// @tparam _Scalar Data Type of the matrix
/// @tparam _Rows Amount of rows of the matrix
/// @tparam _Cols Amount of cols of the matrix
/// @param[in] label Label to display beside. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in] matrix Pointer to the matrix to display
/// @param[in] flags Flags to modify the behavior of the whole element
/// @param[in] tableFlags Flags to modify the Table behaviour
/// @param[in] format Printf format to display the value with
template<typename _Scalar, int _Rows, int _Cols>
void MatrixView(const char* label, const Eigen::Matrix<_Scalar, _Rows, _Cols>* matrix, GuiMatrixViewFlags flags = GuiMatrixViewFlags_None, ImGuiTableFlags tableFlags = ImGuiTableFlags_None,
                const char* format = "%.6f")
{
    if (ImGui::BeginTable(label, static_cast<int>(matrix->cols()) + ((flags & GuiMatrixViewFlags_RowHeader) == 1), tableFlags))
    {
        if (flags & GuiMatrixViewFlags_ColumnHeader)
        {
            if (flags & GuiMatrixViewFlags_RowHeader)
            {
                ImGui::TableSetupColumn("");
            }
            for (int64_t col = 0; col < matrix->cols(); col++)
            {
                ImGui::TableSetupColumn(std::to_string(col).c_str());
            }
            ImGui::TableHeadersRow();
        }

        for (int64_t row = 0; row < matrix->rows(); row++)
        {
            if (flags & GuiMatrixViewFlags_RowHeader)
            {
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(std::to_string(row).c_str());
                ImU32 cell_bg_color = ImGui::GetColorU32(ImGui::GetStyle().Colors[ImGuiCol_TableHeaderBg]);
                ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, cell_bg_color);
            }

            for (int64_t col = 0; col < matrix->cols(); col++)
            {
                ImGui::TableNextColumn();
                ImGui::Text(format, (*matrix)(row, col));
            }
        }
        ImGui::EndTable();
    }
}
#if defined(__GNUC__) || defined(__clang__)
    #pragma GCC diagnostic pop
#endif

/// @brief Shows GUI elements to modify the coefficients of a matrix with
/// @tparam _Scalar Data Type of the matrix
/// @tparam _Rows Amount of rows of the matrix
/// @tparam _Cols Amount of cols of the matrix
/// @param[in] label Label to display beside. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in, out] matrix Pointer to the matrix to modify
/// @param[in] flags Flags to modify the behavior of the whole element
/// @param[in] tableFlags Flags to modify the Table behaviour
/// @param[in] inputTextWidth With in px of each InputText
/// @param[in] step Step size of the InputText
/// @param[in] step_fast Fast step size of the InputText
/// @param[in] format Printf format to display the value with
/// @param[in] inputTextFlags InputText flags to modify the behavior of the input fields
/// @return True if the value was changed
template<typename _Scalar, int _Rows, int _Cols>
bool InputMatrix(const char* label, Eigen::Matrix<_Scalar, _Rows, _Cols>* matrix, GuiMatrixViewFlags flags = GuiMatrixViewFlags_None, ImGuiTableFlags tableFlags = ImGuiTableFlags_None,
                 float inputTextWidth = 50.0F, double step = 0.0, double step_fast = 0.0, const char* format = "%.6f", ImGuiInputTextFlags inputTextFlags = ImGuiInputTextFlags_None)
{
    bool changed = false;
    if (ImGui::BeginTable(label, static_cast<int>(matrix->cols()) + ((flags & GuiMatrixViewFlags_RowHeader) == 1), tableFlags))
    {
        if (flags & GuiMatrixViewFlags_ColumnHeader)
        {
            if (flags & GuiMatrixViewFlags_RowHeader)
            {
                ImGui::TableSetupColumn("");
            }
            for (int64_t col = 0; col < matrix->cols(); col++)
            {
                ImGui::TableSetupColumn(std::to_string(col).c_str());
            }
            ImGui::TableHeadersRow();
        }

        for (int64_t row = 0; row < matrix->rows(); row++)
        {
            if (flags & GuiMatrixViewFlags_RowHeader)
            {
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(std::to_string(row).c_str());
                ImU32 cell_bg_color = ImGui::GetColorU32(ImGui::GetStyle().Colors[ImGuiCol_TableHeaderBg]);
                ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, cell_bg_color);
            }

            for (int64_t col = 0; col < matrix->cols(); col++)
            {
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(inputTextWidth);
                if (ImGui::InputDouble(fmt::format("##{} ({}, {})", label, row, col).c_str(), &(*matrix)(row, col), step, step_fast, format, inputTextFlags))
                {
                    changed = true;
                }
            }
        }
        ImGui::EndTable();
    }

    return changed;
}

} // namespace NAV::gui::widgets