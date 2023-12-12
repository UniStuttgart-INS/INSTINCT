// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file PositionInput.hpp
/// @brief Position Input GUI widgets
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-08-14

#include "PositionInput.hpp"

#include <imgui.h>
#include <fmt/core.h>

#include "internal/gui/widgets/imgui_ex.hpp"
#include "internal/gui/widgets/EnumCombo.hpp"

namespace NAV
{

const char* to_string(gui::widgets::PositionWithFrame::ReferenceFrame refFrame)
{
    switch (refFrame)
    {
    case gui::widgets::PositionWithFrame::ReferenceFrame::ECEF:
        return "ECEF";
    case gui::widgets::PositionWithFrame::ReferenceFrame::LLA:
        return "LLA";
    case gui::widgets::PositionWithFrame::ReferenceFrame::COUNT:
        break;
    }
    return "";
}

namespace gui::widgets
{

/// @brief Shows a ComboBox to select the position input reference frame
/// @param[in] label Label to show beside the combo box. This has to be a unique id for ImGui.
/// @param[in] refFrame Reference to the frame to select
bool ComboPositionInputReferenceFrame(const char* label, PositionWithFrame::ReferenceFrame& refFrame)
{
    return gui::widgets::EnumCombo(label, refFrame);
}

} // namespace gui::widgets

} // namespace NAV

bool NAV::gui::widgets::PositionInput(const char* str, PositionWithFrame& position, PositionInputLayout layout, float itemWidth)
{
    bool changes = false;
    bool edited = false;

    ImGui::BeginGroup();

    if (layout == PositionInputLayout::SINGLE_ROW)
    {
        std::string txt = str;
        txt = txt.substr(0, txt.find_first_of('#'));
        ImGui::TextUnformatted(txt.c_str());
        ImGui::SameLine();
    }

    if (layout == PositionInputLayout::SINGLE_ROW) { ImGui::SetNextItemWidth(80.0F); }
    else { ImGui::SetNextItemWidth(itemWidth); }
    if (ComboPositionInputReferenceFrame(fmt::format("{}##ComboPositionInputReferenceFrame {}", layout != PositionInputLayout::SINGLE_ROW ? str : "", str).c_str(), position.frame))
    {
        changes = true;
    }
    if (layout == PositionInputLayout::SINGLE_ROW) { ImGui::SameLine(); }

    if (position.frame == PositionWithFrame::ReferenceFrame::ECEF)
    {
        ImGui::SetNextItemWidth(itemWidth);
        if (ImGui::InputDouble(fmt::format("{}##X {}", layout != PositionInputLayout::TWO_ROWS ? "X" : "", str).c_str(), &position.e_position(0), 0.0, 0.0, "%.4fm")) { changes = true; }

        if (layout == PositionInputLayout::SINGLE_ROW || layout == PositionInputLayout::TWO_ROWS) { ImGui::SameLine(); }
        ImGui::SetNextItemWidth(itemWidth);
        if (ImGui::InputDouble(fmt::format("{}##Y {}", layout != PositionInputLayout::TWO_ROWS ? "Y" : "", str).c_str(), &position.e_position(1), 0.0, 0.0, "%.4fm")) { changes = true; }

        if (layout == PositionInputLayout::SINGLE_ROW || layout == PositionInputLayout::TWO_ROWS) { ImGui::SameLine(); }
        ImGui::SetNextItemWidth(itemWidth);
        if (ImGui::InputDouble(fmt::format("{}##Z {}", layout != PositionInputLayout::TWO_ROWS ? "Z" : "", str).c_str(), &position.e_position(2), 0.0, 0.0, "%.4fm")) { changes = true; }

        if (layout == PositionInputLayout::TWO_ROWS)
        {
            ImGui::SameLine();
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() - ImGui::GetStyle().ItemSpacing.x + ImGui::GetStyle().ItemInnerSpacing.x);
            ImGui::TextUnformatted("XYZ");
        }
    }
    else if (position.frame == PositionWithFrame::ReferenceFrame::LLA)
    {
        auto lla_position = trafo::ecef2lla_WGS84(position.e_position);
        lla_position(0) = rad2deg(lla_position(0));
        lla_position(1) = rad2deg(lla_position(1));
        if (lla_position.hasNaN())
        {
            lla_position = Eigen::Vector3d(0, 0, 0);
            position.e_position = trafo::lla2ecef_WGS84(lla_position);
        }

        ImGui::SetNextItemWidth(itemWidth);
        if (ImGui::InputDoubleL(fmt::format("{}##Lat {}", layout != PositionInputLayout::TWO_ROWS ? "Latitude" : "", str).c_str(), &lla_position(0), -90.0, 90.0, 0.0, 0.0, "%.9f°")) { edited = true; }
        if (ImGui::IsItemDeactivatedAfterEdit()) { changes = true; }

        if (layout == PositionInputLayout::SINGLE_ROW || layout == PositionInputLayout::TWO_ROWS) { ImGui::SameLine(); }
        ImGui::SetNextItemWidth(itemWidth);
        if (ImGui::InputDoubleL(fmt::format("{}##Lon {}", layout != PositionInputLayout::TWO_ROWS ? "Longitude" : "", str).c_str(), &lla_position(1), -180.0, 180.0, 0.0, 0.0, "%.9f°")) { edited = true; }
        if (ImGui::IsItemDeactivatedAfterEdit()) { changes = true; }

        if (layout == PositionInputLayout::SINGLE_ROW || layout == PositionInputLayout::TWO_ROWS) { ImGui::SameLine(); }
        ImGui::SetNextItemWidth(itemWidth);
        if (ImGui::InputDouble(fmt::format("{}##Alt {}", layout != PositionInputLayout::TWO_ROWS ? "Altitude" : "", str).c_str(), &lla_position(2), 0.0, 0.0, "%.4fm")) { edited = true; }
        if (ImGui::IsItemDeactivatedAfterEdit()) { changes = true; }

        if (layout == PositionInputLayout::TWO_ROWS)
        {
            ImGui::SameLine();
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() - ImGui::GetStyle().ItemSpacing.x + ImGui::GetStyle().ItemInnerSpacing.x);
            ImGui::TextUnformatted("Lat, Lon, Alt");
        }

        if (changes || edited)
        {
            lla_position(0) = deg2rad(lla_position(0));
            lla_position(1) = deg2rad(lla_position(1));
            if (!lla_position.hasNaN())
            {
                position.e_position = trafo::lla2ecef_WGS84(lla_position);
            }
        }
    }

    ImGui::EndGroup();

    return changes;
}

void NAV::gui::widgets::to_json(json& j, const PositionWithFrame::ReferenceFrame& refFrame)
{
    j = to_string(refFrame);
}

void NAV::gui::widgets::from_json(const json& j, PositionWithFrame::ReferenceFrame& refFrame)
{
    auto text = j.get<std::string>();
    if (text == "ECEF") { refFrame = PositionWithFrame::ReferenceFrame::ECEF; }
    else if (text == "LLA") { refFrame = PositionWithFrame::ReferenceFrame::LLA; }
}
void NAV::gui::widgets::to_json(json& j, const PositionWithFrame& position)
{
    j["frame"] = position.frame;
    if (position.frame == PositionWithFrame::ReferenceFrame::ECEF)
    {
        j["position"] = position.e_position;
    }
    else if (position.frame == PositionWithFrame::ReferenceFrame::LLA)
    {
        auto lla = trafo::ecef2lla_WGS84(position.e_position);
        j["position"] = Eigen::Vector3d(rad2deg(lla(0)), rad2deg(lla(1)), lla(2));
    }
}

void NAV::gui::widgets::from_json(const json& j, PositionWithFrame& position)
{
    if (j.contains("frame"))
    {
        j.at("frame").get_to(position.frame);
    }
    if (j.contains("position"))
    {
        if (position.frame == PositionWithFrame::ReferenceFrame::ECEF)
        {
            j.at("position").get_to(position.e_position);
        }
        else if (position.frame == PositionWithFrame::ReferenceFrame::LLA)
        {
            auto lla = j.at("position").get<Eigen::Vector3d>();
            position.e_position = trafo::lla2ecef_WGS84({ deg2rad(lla(0)), deg2rad(lla(1)), lla(2) });
        }
    }
}