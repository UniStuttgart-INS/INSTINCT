/// @file Matrix.hpp
/// @brief Widgets related to Matrices
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-10-04

// Eigen::Matrix<double, 3, 3>

#pragma once

#include <Eigen/Core>
#include <imgui.h>

using GuiMatrixViewFlags = int; // -> enum GuiMatrixViewFlags_      // Flags: For MatrixView()

enum GuiMatrixViewFlags_
{
    // Features
    GuiMatrixViewFlags_None = 0,
    GuiMatrixViewFlags_RowHeader = 1 << 0,                                                      // Print the Row Header
    GuiMatrixViewFlags_ColumnHeader = 1 << 1,                                                   // Enable resizing columns.
    GuiMatrixViewFlags_Header = GuiMatrixViewFlags_RowHeader | GuiMatrixViewFlags_ColumnHeader, // Draw horizontal borders.
};

namespace NAV::gui::widgets
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-nonliteral"
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
#pragma GCC diagnostic pop

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