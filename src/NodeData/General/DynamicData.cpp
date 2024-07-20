// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file DynamicData.cpp
/// @brief Dynamic Data container
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-07-19

#include "DynamicData.hpp"

#include <algorithm>
#include <cstddef>
#include <ctime>
#include <imgui.h>
#include "Navigation/Time/InsTime.hpp"
#include <fmt/core.h>

void NAV::DynamicData::guiTooltip(bool detailView, bool firstOpen, const char* displayName, const char* id) const
{
    ImGui::SetNextItemOpen(detailView, firstOpen ? ImGuiCond_Always : ImGuiCond_Once);

    auto d = std::find_if(data.begin(), data.end(), [&](const auto& dat) {
        return dat.description == displayName;
    });
    if (d == data.end()) { return; }

    if (std::any_of(d->rawData.begin(), d->rawData.end(), [&](const auto& raw) {
            return raw.second->insTime != insTime;
        }))
    {
        bool sameLine = false;
        for (const auto& raw : d->rawData)
        {
            auto timeString = fmt::format("{}", raw.second->insTime.toYMDHMS(GPST));
            auto len = static_cast<int>(raw.first.length()) - static_cast<int>(timeString.length());

            if (raw.second->hasTooltip())
            {
                ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ImGui::GetStyle().ItemInnerSpacing.x);
                if (sameLine)
                {
                    ImGui::SameLine();
                    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ImGui::GetStyle().ItemInnerSpacing.x);
                }

                if (len > 0)
                {
                    ImGui::TextUnformatted(fmt::format("{}{:{w}}", timeString, " ", fmt::arg("w", len)).c_str());
                }
                else { ImGui::TextUnformatted(timeString.c_str()); }
                sameLine = true;
            }
        }
    }

    if (std::none_of(d->rawData.begin(), d->rawData.end(), [](const auto& raw) { return raw.second->hasTooltip(); })) { return; }

    if (ImGui::BeginTabBar(fmt::format("##dyndata {}", id).c_str(), ImGuiTabBarFlags_None))
    {
        for (const auto& raw : d->rawData)
        {
            if (!raw.second->hasTooltip()) { continue; }
            auto timeString = fmt::format("{}", raw.second->insTime.toYMDHMS(GPST));
            if (ImGui::BeginTabItem(fmt::format("{}##{} {}", raw.first, timeString, id).c_str()))
            {
                raw.second->guiTooltip(detailView, firstOpen, displayName, fmt::format("{} {} {}", raw.first, timeString, id).c_str());
                ImGui::EndTabItem();
            }
        }

        // if (ImGui::BeginTabItem(fmt::format("##{}", id).c_str()))
        // {
        //     ImGui::EndTabItem();
        // }

        ImGui::EndTabBar();
    }
}