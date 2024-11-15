// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "PlotTooltip.hpp"

#include <cstddef>
#include <implot_internal.h>

#include "Navigation/Time/InsTime.hpp"
#include "internal/gui/NodeEditorApplication.hpp"

bool NAV::ShowPlotTooltip(std::vector<PlotTooltip>& tooltips,
                          size_t plotItemIdx,
                          const std::string& plotName,
                          ImAxis axis,
                          const ScrollingBuffer<double>& xData,
                          const ScrollingBuffer<double>& yData,
                          bool otherHoverTooltipsShown,
                          const std::function<void(size_t)>& showTooltipCallback)
{
    bool tooltipShown = false;

    if (ImPlot::IsPlotHovered() && ImGui::IsKeyDown(ImGuiKey_ModCtrl))
    {
        if (const auto* item = ImPlot::GetCurrentPlot()->Items.GetItem(plotName.c_str());
            item && item->Show)
        {
            constexpr double HOVER_PIXEL_SIZE = 5.0;
            auto limits = ImPlot::GetPlotLimits(IMPLOT_AUTO, axis);

            ImVec2 scaling = ImVec2(static_cast<float>(HOVER_PIXEL_SIZE * (limits.X.Max - limits.X.Min) / ImPlot::GetCurrentPlot()->PlotRect.GetWidth()),
                                    static_cast<float>(HOVER_PIXEL_SIZE * (limits.Y.Max - limits.Y.Min) / ImPlot::GetCurrentPlot()->PlotRect.GetHeight()));
            ImPlotPoint mouse = ImPlot::GetPlotMousePos();

            for (size_t i = 0; i < yData.size(); i++)
            {
                if (std::abs(mouse.x - xData.at(i)) < scaling.x
                    && std::abs(mouse.y - yData.at(i)) < scaling.y)
                {
                    ImGui::PushFont(Application::MonoFont());
                    ImGui::BeginTooltip();
                    if (otherHoverTooltipsShown)
                    {
                        ImGui::PushStyleColor(ImGuiCol_Separator, ImGui::GetStyleColorVec4(ImGuiCol_SeparatorActive));
                        ImGui::Separator();
                        ImGui::Separator();
                        ImGui::PopStyleColor();
                    }

                    if (showTooltipCallback) { showTooltipCallback(i); }

                    ImGui::EndTooltip();
                    ImGui::PopFont();
                    tooltipShown = true;

                    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left))
                    {
                        if (std::ranges::find_if(tooltips, [&](const auto& tooltip) {
                                return tooltip.plotItemIdx == plotItemIdx && tooltip.dataIdx == i;
                            })
                            == tooltips.end())
                        {
                            auto mousePos = ImGui::GetMousePos();
                            mousePos.x += 25.0F;
                            tooltips.emplace_back(plotItemIdx, i, mousePos);
                        }
                    }
                    break;
                }
            }
        }
    }
    return tooltipShown;
}

void NAV::ShowPlotTooltipWindows(std::vector<PlotTooltip>& tooltips,
                                 size_t plotItemIdx,
                                 const std::string& plotName,
                                 const std::string& uid,
                                 const std::vector<int*>& parentWindows,
                                 const std::function<InsTime(size_t)>& getInsTime,
                                 const std::function<void(size_t, const char*)>& showTooltipCallback)
{
    std::vector<size_t> tooltipsToClose;
    for (size_t tipIdx = 0; tipIdx < tooltips.size(); tipIdx++)
    {
        auto& tooltip = tooltips.at(tipIdx);
        if (tooltip.plotItemIdx == plotItemIdx)
        {
            auto tooltipTime = getInsTime(tooltip.dataIdx).toYMDHMS(GPST);
            auto windowName = fmt::format("{} - {}", plotName, tooltipTime);
            auto tooltipUID = fmt::format("{} {}", uid, tooltip.dataIdx);

            auto windowUniqueTitle = fmt::format("{}##{}", windowName, tooltipUID);

            for (size_t i = 0; i < tooltips.size(); i++) // Prevent windows with same name/content
            {
                if (i == tipIdx) { continue; }
                const auto& tooltip = tooltips.at(i);
                auto uniqueTitle = fmt::format("{} - {}##{} {}",
                                               plotName, getInsTime(tooltip.dataIdx).toYMDHMS(GPST), uid, tooltip.dataIdx);
                if (uniqueTitle == windowUniqueTitle)
                {
                    tooltipsToClose.push_back(tipIdx);
                    break;
                }
            }
            if (!tooltipsToClose.empty() && tooltipsToClose.back() == tipIdx) { break; }

            bool open = true;
            if (tooltip.startPos)
            {
                ImGui::SetNextWindowPos(*tooltip.startPos);
                tooltip.startPos.reset();
            }
            ImGui::PushFont(Application::MonoFont());
            ImGui::Begin(windowUniqueTitle.c_str(), &open,
                         ImGuiWindowFlags_AlwaysAutoResize);

            for (int* parentWindow : parentWindows) // Do not make tooltips go behind the plot window
            {
                if (ImGuiWindow* tooltipWindow = ImGui::GetCurrentWindow();
                    ImGui::IsWindowAbove(reinterpret_cast<ImGuiWindow*>(parentWindow),
                                         tooltipWindow))
                {
                    ImGui::BringWindowToDisplayFront(tooltipWindow);
                }
            }

            if (ImGui::GetWindowSize().x < ImGui::CalcTextSize(windowName.c_str()).x + 50.0F)
            {
                ImGui::TextUnformatted(fmt::format("{}", tooltipTime).c_str());
                ImGui::Separator();
            }

            if (showTooltipCallback) { showTooltipCallback(tooltip.dataIdx, tooltipUID.c_str()); }

            ImGui::End();
            ImGui::PopFont();
            if (!open) { tooltipsToClose.push_back(tipIdx); }
        }
    }
    for (const size_t tipIdx : tooltipsToClose)
    {
        tooltips.erase(std::next(tooltips.begin(), static_cast<int64_t>(tipIdx)));
    }
}
