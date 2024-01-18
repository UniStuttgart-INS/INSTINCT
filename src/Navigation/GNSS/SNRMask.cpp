// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file SNRMask.cpp
/// @brief Signal to Noise Ratio Mask
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-12-17

#include "SNRMask.hpp"

#include <imgui.h>
#include <fmt/format.h>

#include "internal/gui/widgets/imgui_ex.hpp"
#include "internal/gui/widgets/HelpMarker.hpp"

namespace NAV
{

SNRMask::SNRMask()
{
    const auto frequencies = Frequency::GetAll();
    for (size_t i = 0; i < frequencies.size(); i++)
    {
        mask.at(i).first = frequencies.at(i);
        mask.at(i).second.second = true;
    }
}

bool SNRMask::ShowGuiWidgets(const char* label)
{
    bool changed = false;
    if (!label) { label = "SNR Mask"; }

    if (isInactive()) { ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyle().Colors[ImGuiCol_TextDisabled]); }
    if (ImGui::Button(fmt::format("{}", label).c_str()))
    {
        ImGui::OpenPopup(fmt::format("SNR Mask##Popup - {}", label).c_str());
    }
    if (isInactive())
    {
        ImGui::PopStyleColor();
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Inactive due to all values being 0");
        }
    }

    constexpr float INPUT_WIDTH = 40.0F;

    if (ImGui::BeginPopup(fmt::format("SNR Mask##Popup - {}", label).c_str()))
    {
        if (ImGui::BeginTable(fmt::format("").c_str(), elevations.size() + 2, ImGuiTableFlags_Borders))
        {
            ImGui::TableSetupColumn("Elevation:");
            for (const auto& elevation : elevations)
            {
                ImGui::TableSetupColumn(fmt::format("< {:.0f}", rad2deg(elevation)).c_str());
            }
            ImGui::TableSetupColumn("");
            ImGui::TableHeadersRow();

            ImGui::TableNextColumn();
            ImGui::TextUnformatted("All");
            ImGui::SameLine();
            gui::widgets::HelpMarker("- The values for each frequency are used for the calculation,\n"
                                     "    this row is only used to change all frequencies at the same time.\n"
                                     "- Grayed out when not all frequencies below have the same value.");
            for (size_t i = 0; i < allOverride.first.size(); ++i)
            {
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(INPUT_WIDTH);
                if (allOverride.second && i != 0) { ImGui::BeginDisabled(); }
                bool anyValueDiffers = std::any_of(mask.begin(), mask.end(), [&](const auto& freqMask) {
                    return allOverride.first.at(i) != freqMask.second.first.at(i);
                });
                if (anyValueDiffers) { ImGui::PushStyleColor(ImGuiCol_FrameBg, ImGui::GetStyle().Colors[ImGuiCol_TextDisabled]); }
                if (ImGui::DragDouble(fmt::format("##all {} {}", label, i).c_str(), &allOverride.first.at(i), 1.0, 0.0, 100.0, "%.1f"))
                {
                    changed = true;
                    if (!allOverride.second)
                    {
                        for (auto& freqSNRs : mask)
                        {
                            freqSNRs.second.first.at(i) = allOverride.first.at(i);
                        }
                    }
                    else
                    {
                        for (size_t i = 0; i < allOverride.first.size(); ++i)
                        {
                            allOverride.first.at(i) = allOverride.first[0];
                            for (auto& freqSNRs : mask)
                            {
                                freqSNRs.second.first.at(i) = allOverride.first.at(i);
                            }
                        }
                    }
                }
                if (anyValueDiffers) { ImGui::PopStyleColor(); }
                if (allOverride.second && i != 0) { ImGui::EndDisabled(); }
            }
            ImGui::TableNextColumn();
            if (ImGui::Checkbox(fmt::format("##lock together - all {}", label).c_str(), &allOverride.second))
            {
                changed = true;
                if (allOverride.second)
                {
                    for (size_t i = 0; i < allOverride.first.size(); ++i)
                    {
                        allOverride.first.at(i) = allOverride.first[0];
                        for (auto& freqSNRs : mask)
                        {
                            freqSNRs.second.first.at(i) = allOverride.first.at(i);
                        }
                    }
                }
            }
            if (ImGui::IsItemHovered()) { ImGui::SetTooltip("Lock all values together"); }

            for (auto& [freq, SNRs] : mask)
            {
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(fmt::format("{}", freq).c_str());
                for (size_t i = 0; i < SNRs.first.size(); ++i)
                {
                    ImGui::TableNextColumn();
                    ImGui::SetNextItemWidth(INPUT_WIDTH);
                    if (SNRs.second && i != 0) { ImGui::BeginDisabled(); }
                    if (ImGui::DragDouble(fmt::format("##{} {} {}", freq, label, i).c_str(), &SNRs.first.at(i), 1.0, 0.0, 100.0, "%.1f"))
                    {
                        changed = true;
                        if (SNRs.second)
                        {
                            for (size_t j = 1; j < SNRs.first.size(); ++j)
                            {
                                SNRs.first.at(j) = SNRs.first.at(i);
                            }
                        }
                    }
                    if (SNRs.second && i != 0) { ImGui::EndDisabled(); }
                }
                ImGui::TableNextColumn();
                if (ImGui::Checkbox(fmt::format("##lock together - all {} {}", freq, label).c_str(), &SNRs.second))
                {
                    changed = true;
                    if (SNRs.second)
                    {
                        for (size_t i = 1; i < SNRs.first.size(); ++i)
                        {
                            SNRs.first.at(i) = SNRs.first[0];
                        }
                    }
                }
                if (ImGui::IsItemHovered()) { ImGui::SetTooltip("Lock all values together"); }
            }

            ImGui::EndTable();
        }

        ImGui::EndPopup();
    }

    return changed;
}

bool SNRMask::isInactive() const
{
    return std::all_of(mask.begin(), mask.end(), [](const auto& freqMask) {
        return std::all_of(freqMask.second.first.begin(), freqMask.second.first.end(), [](const auto& SNR) {
            return SNR < 1e-7;
        });
    });
}

bool SNRMask::checkSNRMask(Frequency freq, double elevation, double SNR) const
{
    for (const auto& [maskFreq, SNRs] : mask)
    {
        if (maskFreq == freq)
        {
            for (size_t i = 0; i < elevations.size(); i++)
            {
                if (elevation <= elevations.at(i))
                {
                    return SNR > SNRs.first.at(i);
                }
            }
        }
    }

    return false;
}

void to_json(json& j, const SNRMask& obj)
{
    j = json{
        { "allOverride", obj.allOverride },
        { "mask", obj.mask },
    };
}

void from_json(const json& j, SNRMask& obj)
{
    if (j.contains("allOverride")) { j.at("allOverride").get_to(obj.allOverride); }
    if (j.contains("mask")) { j.at("mask").get_to(obj.mask); }
}

} // namespace NAV
