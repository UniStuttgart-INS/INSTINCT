// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "PlotItemStyle.hpp"

#include <cstddef>
#include <functional>
#include <imgui_stdlib.h>
#include <implot_internal.h>

#include "util/Logger.hpp"

namespace NAV
{

/// @brief Write info to a json object
/// @param[out] j Json output
/// @param[in] style Object to read info from
void to_json(json& j, const PlotItemStyle& style)
{
    j = json{
        { "legendName", style.legendName },
        { "stride", style.stride },
        { "lineType", style.lineType },
        { "color", style.color },
        { "colormapMask", style.colormapMask },
        { "colormapMaskDataCmpIdx", style.colormapMaskDataCmpIdx },
        { "thickness", style.thickness },
        { "markerColormapMask", style.markerColormapMask },
        { "markerColormapMaskDataCmpIdx", style.markerColormapMaskDataCmpIdx },
        { "markers", style.markers },
        { "markerStyle", style.markerStyle },
        { "markerSize", style.markerSize },
        { "markerWeight", style.markerWeight },
        { "markerFillColor", style.markerFillColor },
        { "markerOutlineColor", style.markerOutlineColor },
        { "eventsEnabled", style.eventsEnabled },
        { "eventMarkerStyle", style.eventMarkerStyle },
        { "eventMarkerSize", style.eventMarkerSize },
        { "eventMarkerWeight", style.eventMarkerWeight },
        { "eventMarkerFillColor", style.eventMarkerFillColor },
        { "eventMarkerOutlineColor", style.eventMarkerOutlineColor },
        { "eventTooltipFilterRegex", style.eventTooltipFilterRegex },
    };
    if (style.lineFlags) { j["lineFlags"] = style.lineFlags.value(); }
}
/// @brief Read info from a json object
/// @param[in] j Json variable to read info from
/// @param[out] style Output object
void from_json(const json& j, PlotItemStyle& style)
{
    if (j.contains("legendName")) { j.at("legendName").get_to(style.legendName); }
    if (j.contains("stride")) { j.at("stride").get_to(style.stride); }
    if (j.contains("lineType")) { j.at("lineType").get_to(style.lineType); }
    if (j.contains("color")) { j.at("color").get_to(style.color); }
    if (j.contains("colormapMask")) { j.at("colormapMask").get_to(style.colormapMask); }
    if (j.contains("colormapMaskDataCmpIdx")) { j.at("colormapMaskDataCmpIdx").get_to(style.colormapMaskDataCmpIdx); }
    if (j.contains("thickness")) { j.at("thickness").get_to(style.thickness); }
    if (j.contains("lineFlags")) { style.lineFlags.emplace(j.at("lineFlags").get<uint32_t>()); }
    if (j.contains("markerColormapMask")) { j.at("markerColormapMask").get_to(style.markerColormapMask); }
    if (j.contains("markerColormapMaskDataCmpIdx")) { j.at("markerColormapMaskDataCmpIdx").get_to(style.markerColormapMaskDataCmpIdx); }
    if (j.contains("markers")) { j.at("markers").get_to(style.markers); }
    if (j.contains("markerStyle")) { j.at("markerStyle").get_to(style.markerStyle); }
    if (j.contains("markerSize")) { j.at("markerSize").get_to(style.markerSize); }
    if (j.contains("markerWeight")) { j.at("markerWeight").get_to(style.markerWeight); }
    if (j.contains("markerFillColor")) { j.at("markerFillColor").get_to(style.markerFillColor); }
    if (j.contains("markerOutlineColor")) { j.at("markerOutlineColor").get_to(style.markerOutlineColor); }
    if (j.contains("eventsEnabled")) { j.at("eventsEnabled").get_to(style.eventsEnabled); }
    if (j.contains("eventMarkerStyle")) { j.at("eventMarkerStyle").get_to(style.eventMarkerStyle); }
    if (j.contains("eventMarkerSize")) { j.at("eventMarkerSize").get_to(style.eventMarkerSize); }
    if (j.contains("eventMarkerWeight")) { j.at("eventMarkerWeight").get_to(style.eventMarkerWeight); }
    if (j.contains("eventMarkerFillColor")) { j.at("eventMarkerFillColor").get_to(style.eventMarkerFillColor); }
    if (j.contains("eventMarkerOutlineColor")) { j.at("eventMarkerOutlineColor").get_to(style.eventMarkerOutlineColor); }
    if (j.contains("eventTooltipFilterRegex")) { j.at("eventTooltipFilterRegex").get_to(style.eventTooltipFilterRegex); }
}

bool PlotItemStyle::showLegendPopup(const char* id,
                                    const char* displayTitle,
                                    int plotDataBufferSize,
                                    int plotElementIdx,
                                    [[maybe_unused]] const char* nameId,
                                    ImPlotLineFlags plotLineFlags,
                                    ScrollingBuffer<ImU32>* colormapMaskColors,
                                    ScrollingBuffer<ImU32>* markerColormapMaskColors,
                                    const std::function<bool(size_t&, const char*)>& ShowColormapReferenceChooser,
                                    ScrollingBuffer<double>* eventMarker,
                                    std::vector<std::tuple<double, double, PlotEventTooltip>>* eventTooltips)
{
    bool changed = false;

    // Legend item context menu (right click on legend item)
    if (ImPlot::BeginLegendPopup(id))
    {
        if (displayTitle)
        {
            ImGui::TextUnformatted(displayTitle);
            ImGui::Separator();
        }

        if (legendNameGui.empty())
        {
            legendNameGui = legendName;
        }
        ImGui::InputText("Legend name", &legendNameGui);
        if (legendNameGui != legendName && !ImGui::IsItemActive())
        {
            legendName = legendNameGui;
            changed = true;
            LOG_DEBUG("{}: Legend changed to {}", nameId, legendName);
        }

        if (ImGui::InputInt("Stride", &stride))
        {
            if (stride < 0)
            {
                stride = 0;
            }
            if (stride > plotDataBufferSize - 1)
            {
                stride = plotDataBufferSize - 1;
            }
            changed = true;
            LOG_DEBUG("{}: Stride changed to {}", nameId, stride);
        }

        if (ImGui::Combo("Style", reinterpret_cast<int*>(&lineType),
                         "Scatter\0Line\0\0"))
        {
            changed = true;
        }
        ImPlotLineFlags lFlags = lineFlags.value_or(plotLineFlags);
        bool plotItemLineFlagsAuto = !lineFlags.has_value();
        if (plotItemLineFlagsAuto) { ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.8F); }
        if (ImGui::CheckboxFlags("NoClip", &lFlags, ImPlotLineFlags_NoClip))
        {
            lineFlags = lFlags;
            changed = true;
        }
        if (ImGui::IsItemHovered()) { ImGui::SetTooltip("Markers (if displayed) on the edge of a plot will not be clipped"); }
        ImGui::SameLine();
        if (ImGui::CheckboxFlags("SkipNaN", &lFlags, ImPlotLineFlags_SkipNaN))
        {
            lineFlags = lFlags;
            changed = true;
        }
        if (ImGui::IsItemHovered()) { ImGui::SetTooltip("NaNs values will be skipped instead of rendered as missing data"); }
        ImGui::SameLine();
        if (ImGui::CheckboxFlags("Loop", &lFlags, ImPlotLineFlags_Loop))
        {
            lineFlags = lFlags;
            changed = true;
        }
        if (ImGui::IsItemHovered()) { ImGui::SetTooltip("The last and first point will be connected to form a closed loop"); }
        if (lineFlags)
        {
            ImGui::SameLine();
            if (ImGui::Button("Auto##Line Flags"))
            {
                lineFlags.reset();
            }
        }
        if (plotItemLineFlagsAuto) { ImGui::PopStyleVar(); }
        if (lineType == PlotItemStyle::LineType::Line)
        {
            if (ImGui::DragFloat("Line Thickness", &thickness, 0.1F, 0.0F, 8.0F, "%.2f px"))
            {
                changed = true;
            }
            if (colormapMaskColors && ShowColormapSelector(colormapMask.first, colormapMask.second))
            {
                colormapMaskColors->clear();
                changed = true;
            }
            if (colormapMaskColors && colormapMask.first != ColormapMaskType::None
                && ShowColormapReferenceChooser(colormapMaskDataCmpIdx, ""))
            {
                colormapMaskColors->clear();
                changed = true;
            }
            if (colormapMask.first == ColormapMaskType::None)
            {
                bool isColorAuto = ImPlot::IsColorAuto(color);
                auto col = isColorAuto ? ImPlot::GetColormapColor(plotElementIdx) : color;
                if (ImGui::ColorEdit4("Line Color", &col.x))
                {
                    color = col;
                    changed = true;
                }
                if (!isColorAuto)
                {
                    ImGui::SameLine();
                    if (ImGui::Button("Auto##Line Color"))
                    {
                        color = IMPLOT_AUTO_COL;
                    }
                }
            }
            if (ImGui::Checkbox("Markers", &markers))
            {
                changed = true;
            }
        }
        if (lineType == PlotItemStyle::LineType::Scatter || markers)
        {
            if (ImGui::Combo("Marker Style", &markerStyle,
                             "Circle\0Square\0Diamond\0Up\0Down\0Left\0Right\0Cross\0Plus\0Asterisk\0\0"))
            {
                changed = true;
            }
            if (ImGui::DragFloat("Marker Size", &markerSize, 0.1F, 1.0F, 10.0F, "%.2f px"))
            {
                changed = true;
            }
            if (ImGui::DragFloat("Marker Weight", &markerWeight, 0.05F, 0.5F, 3.0F, "%.2f px"))
            {
                changed = true;
            }
            if (!markers)
            {
                if (colormapMaskColors && ShowColormapSelector(colormapMask.first, colormapMask.second))
                {
                    colormapMaskColors->clear();
                    changed = true;
                }
                if (colormapMaskColors && colormapMask.first != ColormapMaskType::None
                    && ShowColormapReferenceChooser(colormapMaskDataCmpIdx, ""))
                {
                    colormapMaskColors->clear();
                    changed = true;
                }
            }
            if (markers && lineType != PlotItemStyle::LineType::Scatter)
            {
                if (colormapMaskColors && ShowColormapSelector(markerColormapMask.first, markerColormapMask.second, "Marker "))
                {
                    markerColormapMaskColors->clear();
                    changed = true;
                }
                if (colormapMaskColors && markerColormapMask.first != ColormapMaskType::None
                    && ShowColormapReferenceChooser(markerColormapMaskDataCmpIdx, "Marker "))
                {
                    markerColormapMaskColors->clear();
                    changed = true;
                }
            }
            if (markerColormapMask.first == ColormapMaskType::None
                && (lineType != PlotItemStyle::LineType::Scatter
                    || colormapMask.first == ColormapMaskType::None))
            {
                bool isColorAuto = ImPlot::IsColorAuto(markerFillColor);
                auto col = isColorAuto ? ImPlot::GetColormapColor(plotElementIdx) : markerFillColor;
                if (ImGui::ColorEdit4("Marker Fill Color", &col.x))
                {
                    markerFillColor = col;
                    changed = true;
                }
                if (!isColorAuto)
                {
                    ImGui::SameLine();
                    if (ImGui::Button("Auto##Marker Fill Color"))
                    {
                        markerFillColor = IMPLOT_AUTO_COL;
                    }
                }

                isColorAuto = ImPlot::IsColorAuto(markerOutlineColor);
                col = isColorAuto ? ImPlot::GetColormapColor(plotElementIdx) : markerOutlineColor;
                if (ImGui::ColorEdit4("Marker Outline Color", &col.x))
                {
                    markerOutlineColor = col;
                    changed = true;
                }
                if (!isColorAuto)
                {
                    ImGui::SameLine();
                    if (ImGui::Button("Auto##Marker Outline Color"))
                    {
                        markerOutlineColor = IMPLOT_AUTO_COL;
                    }
                }
            }
        }
        if (eventMarker && eventTooltips)
        {
            ImGui::Separator();
            if (ImGui::Checkbox("Events", &eventsEnabled))
            {
                changed = true;
            }
            if (eventsEnabled)
            {
                if (ImGui::Combo("Event Marker Style", &eventMarkerStyle,
                                 "Circle\0Square\0Diamond\0Up\0Down\0Left\0Right\0Cross\0Plus\0Asterisk\0\0"))
                {
                    changed = true;
                }
                if (ImGui::DragFloat("Event Marker Size", &eventMarkerSize, 0.1F, 1.0F, 10.0F, "%.2f px"))
                {
                    changed = true;
                }
                if (ImGui::DragFloat("Event Marker Weight", &eventMarkerWeight, 0.05F, 0.5F, 3.0F, "%.2f px"))
                {
                    changed = true;
                }
                bool isColorAuto = ImPlot::IsColorAuto(eventMarkerFillColor);
                auto col = isColorAuto ? ImPlot::GetColormapColor(plotElementIdx) : eventMarkerFillColor;
                if (ImGui::ColorEdit4("Event Marker Fill Color", &col.x))
                {
                    eventMarkerFillColor = col;
                    changed = true;
                }
                if (!isColorAuto)
                {
                    ImGui::SameLine();
                    if (ImGui::Button("Auto##Event Marker Fill Color"))
                    {
                        eventMarkerFillColor = IMPLOT_AUTO_COL;
                    }
                }

                isColorAuto = ImPlot::IsColorAuto(eventMarkerOutlineColor);
                col = isColorAuto ? ImPlot::GetColormapColor(plotElementIdx) : eventMarkerOutlineColor;
                if (ImGui::ColorEdit4("Event Marker Outline Color", &col.x))
                {
                    eventMarkerOutlineColor = col;
                    changed = true;
                }
                if (!isColorAuto)
                {
                    ImGui::SameLine();
                    if (ImGui::Button("Auto##Event Marker Outline Color"))
                    {
                        eventMarkerOutlineColor = IMPLOT_AUTO_COL;
                    }
                }

                if (ImGui::InputText("Event Filter Regex", &eventTooltipFilterRegex))
                {
                    eventMarker->clear();
                    eventTooltips->clear();
                    changed = true;
                }
            }
        }

        ImPlot::EndLegendPopup();
    }
    return changed;
}

void PlotItemStyle::plotData(const char* plotName,
                             const ScrollingBuffer<double>& xData,
                             const ScrollingBuffer<double>& yData,
                             int plotElementIdx,
                             int defaultStride,
                             ImPlotLineFlags plotLineFlags,
                             ScrollingBuffer<ImU32>* colormapMaskColors,
                             ScrollingBuffer<ImU32>* markerColormapMaskColors) const
{
    if (lineType == PlotItemStyle::LineType::Line)
    {
        ImPlot::SetNextLineStyle(ImPlot::IsColorAuto(color) ? ImPlot::GetColormapColor(plotElementIdx) : color,
                                 thickness);
    }
    if (lineType == PlotItemStyle::LineType::Scatter || markers)
    {
        ImPlot::SetNextMarkerStyle(markerStyle,
                                   markerSize,
                                   ImPlot::IsColorAuto(markerFillColor) ? ImPlot::GetColormapColor(plotElementIdx) : markerFillColor,
                                   markerWeight,
                                   ImPlot::IsColorAuto(markerOutlineColor) ? ImPlot::GetColormapColor(plotElementIdx) : markerOutlineColor);
    }

    auto stride = this->stride ? this->stride : defaultStride;
    auto dataPointCount = static_cast<int>(std::ceil(static_cast<double>(yData.size())
                                                     / static_cast<double>(stride)));

    // Plot the data
    if (lineType == PlotItemStyle::LineType::Line)
    {
        if (colormapMaskColors && markerColormapMaskColors && colormapMask.first != ColormapMaskType::None)
        {
            ImPlot::SetNextColorsData(ImPlotCol_Line, colormapMaskColors->data(), stride * static_cast<int>(sizeof(ImU32)));
            if (markers)
            {
                ImPlot::SetNextColorsData(ImPlotCol_MarkerFill, markerColormapMask.first != ColormapMaskType::None ? markerColormapMaskColors->data() : colormapMaskColors->data(), stride * static_cast<int>(sizeof(ImU32)));
                ImPlot::SetNextColorsData(ImPlotCol_MarkerOutline, markerColormapMask.first != ColormapMaskType::None ? markerColormapMaskColors->data() : colormapMaskColors->data(), stride * static_cast<int>(sizeof(ImU32)));
            }
        }
        else if (markerColormapMaskColors && markers && markerColormapMask.first != ColormapMaskType::None)
        {
            ImPlot::SetNextColorsData(ImPlotCol_MarkerFill, markerColormapMaskColors->data(), stride * static_cast<int>(sizeof(ImU32)));
            ImPlot::SetNextColorsData(ImPlotCol_MarkerOutline, markerColormapMaskColors->data(), stride * static_cast<int>(sizeof(ImU32)));
        }
        ImPlot::PlotLine(plotName,
                         xData.data(),
                         yData.data(),
                         dataPointCount,
                         lineFlags.value_or(plotLineFlags),
                         static_cast<int>(std::ceil(static_cast<double>(yData.offset()) / static_cast<double>(stride))),
                         stride * static_cast<int>(sizeof(double)));
    }
    else if (lineType == PlotItemStyle::LineType::Scatter)
    {
        if (colormapMaskColors && colormapMask.first != ColormapMaskType::None && colormapMaskColors->isInfiniteBuffer())
        {
            ImPlot::SetNextColorsData(ImPlotCol_MarkerFill, colormapMaskColors->data(), stride * static_cast<int>(sizeof(ImU32)));
            ImPlot::SetNextColorsData(ImPlotCol_MarkerOutline, colormapMaskColors->data(), stride * static_cast<int>(sizeof(ImU32)));
        }
        ImPlot::PlotScatter(plotName,
                            xData.data(),
                            yData.data(),
                            dataPointCount,
                            lineFlags.value_or(plotLineFlags) & ImPlotLineFlags_NoClip ? ImPlotScatterFlags_NoClip : ImPlotScatterFlags_None,
                            static_cast<int>(std::ceil(static_cast<double>(yData.offset()) / static_cast<double>(stride))),
                            stride * static_cast<int>(sizeof(double)));
    }
}

} // namespace NAV