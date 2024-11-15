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
#include <imgui.h>
#include <imgui_stdlib.h>
#include <implot.h>
#include <implot_internal.h>
#include <muParser.h>
#include <string>

#include "internal/gui/widgets/HelpMarker.hpp"
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
        { "errorBoundsEnabled", style.errorBoundsEnabled },
        { "errorBoundsDataIdx", style.errorBoundsDataIdx },
        { "errorBoundsAlpha", style.errorBoundsAlpha },
        { "errorBoundsModifierExpression", style.errorBoundsModifierExpression },
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
    if (j.contains("errorBoundsEnabled")) { j.at("errorBoundsEnabled").get_to(style.errorBoundsEnabled); }
    if (j.contains("errorBoundsDataIdx")) { j.at("errorBoundsDataIdx").get_to(style.errorBoundsDataIdx); }
    if (j.contains("errorBoundsAlpha")) { j.at("errorBoundsAlpha").get_to(style.errorBoundsAlpha); }
    if (j.contains("errorBoundsModifierExpression")) { j.at("errorBoundsModifierExpression").get_to(style.errorBoundsModifierExpression); }
    if (j.contains("eventsEnabled")) { j.at("eventsEnabled").get_to(style.eventsEnabled); }
    if (j.contains("eventMarkerStyle")) { j.at("eventMarkerStyle").get_to(style.eventMarkerStyle); }
    if (j.contains("eventMarkerSize")) { j.at("eventMarkerSize").get_to(style.eventMarkerSize); }
    if (j.contains("eventMarkerWeight")) { j.at("eventMarkerWeight").get_to(style.eventMarkerWeight); }
    if (j.contains("eventMarkerFillColor")) { j.at("eventMarkerFillColor").get_to(style.eventMarkerFillColor); }
    if (j.contains("eventMarkerOutlineColor")) { j.at("eventMarkerOutlineColor").get_to(style.eventMarkerOutlineColor); }
    if (j.contains("eventTooltipFilterRegex")) { j.at("eventTooltipFilterRegex").get_to(style.eventTooltipFilterRegex); }
}

PlotItemStyle::LegendPopupReturn PlotItemStyle::showLegendPopup(const char* id,
                                                                const char* displayTitle,
                                                                int plotDataBufferSize,
                                                                int plotElementIdx,
                                                                [[maybe_unused]] const char* nameId,
                                                                ImPlotLineFlags plotLineFlags,
                                                                ScrollingBuffer<ImU32>* colormapMaskColors,
                                                                ScrollingBuffer<ImU32>* markerColormapMaskColors,
                                                                const std::function<bool(size_t&, const char*)>& ShowDataReferenceChooser,
                                                                ScrollingBuffer<double>* eventMarker,
                                                                std::vector<std::tuple<double, double, PlotEventTooltip>>* eventTooltips)
{
    LegendPopupReturn ret;

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
            ret.changed = true;
            LOG_DEBUG("{}: Legend changed to {}", nameId, legendName);
        }

        if (ImGui::InputInt("Stride", &stride))
        {
            stride = std::max(stride, 0);
            stride = std::min(stride, plotDataBufferSize - 1);
            ret.changed = true;
            LOG_DEBUG("{}: Stride changed to {}", nameId, stride);
        }

        if (ImGui::Combo("Style", reinterpret_cast<int*>(&lineType),
                         "Scatter\0Line\0\0"))
        {
            ret.changed = true;
        }
        ImPlotLineFlags lFlags = lineFlags.value_or(plotLineFlags);
        bool plotItemLineFlagsAuto = !lineFlags.has_value();
        if (plotItemLineFlagsAuto) { ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.8F); }
        if (ImGui::CheckboxFlags("NoClip", &lFlags, ImPlotLineFlags_NoClip))
        {
            lineFlags = lFlags;
            ret.changed = true;
        }
        if (ImGui::IsItemHovered()) { ImGui::SetTooltip("Markers (if displayed) on the edge of a plot will not be clipped"); }
        ImGui::SameLine();
        if (ImGui::CheckboxFlags("SkipNaN", &lFlags, ImPlotLineFlags_SkipNaN))
        {
            lineFlags = lFlags;
            ret.changed = true;
        }
        if (ImGui::IsItemHovered()) { ImGui::SetTooltip("NaNs values will be skipped instead of rendered as missing data"); }
        ImGui::SameLine();
        if (ImGui::CheckboxFlags("Loop", &lFlags, ImPlotLineFlags_Loop))
        {
            lineFlags = lFlags;
            ret.changed = true;
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
                ret.changed = true;
            }
            if (colormapMaskColors && ShowColormapSelector(colormapMask.first, colormapMask.second))
            {
                colormapMaskColors->clear();
                ret.changed = true;
            }
            if (colormapMaskColors && colormapMask.first != ColormapMaskType::None
                && ShowDataReferenceChooser(colormapMaskDataCmpIdx, "Colormap Ref"))
            {
                colormapMaskColors->clear();
                ret.changed = true;
            }
            if (colormapMask.first == ColormapMaskType::None)
            {
                bool isColorAuto = ImPlot::IsColorAuto(color);
                auto col = isColorAuto ? ImPlot::GetColormapColor(plotElementIdx) : color;
                if (ImGui::ColorEdit4("Line Color", &col.x))
                {
                    color = col;
                    ret.changed = true;
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
                ret.changed = true;
            }
        }
        if (lineType == PlotItemStyle::LineType::Scatter || markers)
        {
            if (ImGui::Combo("Marker Style", &markerStyle,
                             "Circle\0Square\0Diamond\0Up\0Down\0Left\0Right\0Cross\0Plus\0Asterisk\0\0"))
            {
                ret.changed = true;
            }
            if (ImGui::DragFloat("Marker Size", &markerSize, 0.1F, 1.0F, 10.0F, "%.2f px"))
            {
                ret.changed = true;
            }
            if (ImGui::DragFloat("Marker Weight", &markerWeight, 0.05F, 0.5F, 3.0F, "%.2f px"))
            {
                ret.changed = true;
            }
            if (!markers)
            {
                if (colormapMaskColors && ShowColormapSelector(colormapMask.first, colormapMask.second))
                {
                    colormapMaskColors->clear();
                    ret.changed = true;
                }
                if (colormapMaskColors && colormapMask.first != ColormapMaskType::None
                    && ShowDataReferenceChooser(colormapMaskDataCmpIdx, "Colormap Ref"))
                {
                    colormapMaskColors->clear();
                    ret.changed = true;
                }
            }
            if (markers && lineType != PlotItemStyle::LineType::Scatter)
            {
                if (colormapMaskColors && ShowColormapSelector(markerColormapMask.first, markerColormapMask.second, "Marker "))
                {
                    markerColormapMaskColors->clear();
                    ret.changed = true;
                }
                if (colormapMaskColors && markerColormapMask.first != ColormapMaskType::None
                    && ShowDataReferenceChooser(markerColormapMaskDataCmpIdx, "Marker Colormap Ref"))
                {
                    markerColormapMaskColors->clear();
                    ret.changed = true;
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
                    ret.changed = true;
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
                    ret.changed = true;
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
        if (lineType == PlotItemStyle::LineType::Line)
        {
            ImGui::Separator();
            ret.changed |= ImGui::Checkbox("Show Error Bounds", &errorBoundsEnabled);
            if (errorBoundsEnabled)
            {
                auto idx = errorBoundsDataIdx;
                if (ShowDataReferenceChooser(errorBoundsDataIdx, "Error Bounds Ref") && errorBoundsDataIdx != idx)
                {
                    ret.changed = true;
                    ret.errorBoundsReCalcNeeded = true;
                }
                ret.changed |= ImGui::SliderFloat("Error Bounds Alpha", &errorBoundsAlpha, 0.0F, 1.0F, "%.2f");
                if (errorBoundsModifierExpressionTemp.empty()) { errorBoundsModifierExpressionTemp = errorBoundsModifierExpression; }
                std::string expression = errorBoundsModifierExpressionTemp;
                if (errorBoundsModifierExpressionTemp != errorBoundsModifierExpression)
                {
                    ImGui::PushStyleColor(ImGuiCol_Text, ImColor(220, 20, 60).Value);
                }
                ImGui::InputTextWithHint("Error Bounds Modifier", "e.g. '3 * x' or 'sqrt(x)'", &expression);
                if (errorBoundsModifierExpressionTemp != errorBoundsModifierExpression)
                {
                    ImGui::PopStyleColor();
                    if (!errorBoundsModifierExpression.empty() && ImGui::IsItemHovered()) { ImGui::SetTooltip("Currently used expression:\n%s", errorBoundsModifierExpression.c_str()); }
                }
                if (expression != errorBoundsModifierExpressionTemp && !ImGui::IsItemActive())
                {
                    errorBoundsModifierExpressionTemp = expression;
                    try
                    {
                        if (!errorBoundsModifierExpressionTemp.empty())
                        {
                            double x = 1.0;
                            mu::Parser p;
                            p.DefineVar("x", &x);
                            p.SetExpr(errorBoundsModifierExpressionTemp);
                            x = p.Eval();
                        }

                        errorBoundsModifierExpression = errorBoundsModifierExpressionTemp;
                        errorBoundsModifierExpressionTemp.clear();
                        ret.changed = true;
                        ret.errorBoundsReCalcNeeded = true;
                    }
                    catch (mu::Parser::exception_type& e)
                    {
                        LOG_ERROR("{}: Error bound modifier parse error on '{}': {} in expression: {}", nameId, legendName, e.GetMsg(), errorBoundsModifierExpressionTemp);
                    }
                }
                ImGui::SameLine();
                if (gui::widgets::BeginHelpMarker("(?)", 0.0F))
                {
                    auto tableEntry = [](const char* first, const char* second, const char* third) {
                        ImGui::TableNextRow();
                        ImGui::TableNextColumn();
                        ImGui::TextUnformatted(first);
                        ImGui::TableNextColumn();
                        ImGui::TextUnformatted(second);
                        ImGui::TableNextColumn();
                        ImGui::TextUnformatted(third);
                    };

                    ImGui::BeginGroup();
                    {
                        ImGui::TextUnformatted("Functions");
                        if (ImGui::BeginTable(fmt::format("Functions##{}", id).c_str(), 3,
                                              ImGuiTableFlags_Borders | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_NoHostExtendX, ImVec2(0.0F, 0.0F)))
                        {
                            ImGui::TableSetupColumn("Name");
                            ImGui::TableSetupColumn("Argc.");
                            ImGui::TableSetupColumn("Explanation");
                            ImGui::TableHeadersRow();

                            tableEntry("sin", "1", "sine function");
                            tableEntry("cos", "1", "cosine function");
                            tableEntry("tan", "1", "tangens function");
                            tableEntry("asin", "1", "arcus sine function");
                            tableEntry("acos", "1", "arcus cosine function");
                            tableEntry("atan", "1", "arcus tangens function");
                            tableEntry("sinh", "1", "hyperbolic sine function");
                            tableEntry("cosh", "1", "hyperbolic cosine");
                            tableEntry("tanh", "1", "hyperbolic tangens function");
                            tableEntry("asinh", "1", "hyperbolic arcus sine function");
                            tableEntry("acosh", "1", "hyperbolic arcus tangens function");
                            tableEntry("atanh", "1", "hyperbolic arcur tangens function");
                            tableEntry("log2", "1", "logarithm to the base 2");
                            tableEntry("log10", "1", "logarithm to the base 10");
                            tableEntry("log", "1", "logarithm to base e (2.71828...)");
                            tableEntry("ln", "1", "logarithm to base e (2.71828...)");
                            tableEntry("exp", "1", "e raised to the power of x");
                            tableEntry("sqrt", "1", "square root of a value");
                            tableEntry("sign", "1", "sign function -1 if x<0; 1 if x>0");
                            tableEntry("rint", "1", "round to nearest integer");
                            tableEntry("abs", "1", "absolute value");
                            tableEntry("min", "var.", "min of all arguments");
                            tableEntry("max", "var.", "max of all arguments");
                            tableEntry("sum", "var.", "sum of all arguments");
                            tableEntry("avg", "var.", "mean value of all arguments");

                            ImGui::EndTable();
                        }
                    }
                    ImGui::EndGroup();

                    ImGui::SameLine();

                    ImGui::BeginGroup();
                    {
                        ImGui::TextUnformatted("Binary operators");
                        if (ImGui::BeginTable(fmt::format("Binary operators##{}", id).c_str(), 3,
                                              ImGuiTableFlags_Borders | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_NoHostExtendX, ImVec2(0.0F, 0.0F)))
                        {
                            ImGui::TableSetupColumn("Operator");
                            ImGui::TableSetupColumn("Description");
                            ImGui::TableSetupColumn("Priority");
                            ImGui::TableHeadersRow();

                            tableEntry("=", "assignement *", "0");
                            tableEntry("||", "logical or", "1");
                            tableEntry("&&", "logical and", "2");
                            tableEntry("|", "bitwise or", "3");
                            tableEntry("&", "bitwise and", "4");
                            tableEntry("<=", "less or equal", "5");
                            tableEntry(">=", "greater or equal", "5");
                            tableEntry("!=", "not equal", "5");
                            tableEntry("==", "equal", "5");
                            tableEntry(">", "greater than", "5");
                            tableEntry("<", "less than", "5");
                            tableEntry("+", "addition", "6");
                            tableEntry("-", "subtraction", "6");
                            tableEntry("*", "multiplication", "7");
                            tableEntry("/", "division", "7");
                            tableEntry("^", "raise x to the power of y", "8");

                            ImGui::EndTable();
                        }

                        ImGui::TextUnformatted("Ternary Operators");
                        if (ImGui::BeginTable(fmt::format("Ternary Operators##{}", id).c_str(), 3,
                                              ImGuiTableFlags_Borders | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_NoHostExtendX, ImVec2(0.0F, 0.0F)))
                        {
                            ImGui::TableSetupColumn("Operator");
                            ImGui::TableSetupColumn("Description");
                            ImGui::TableSetupColumn("Remarks");
                            ImGui::TableHeadersRow();

                            tableEntry("?:", "if then else operator", "C++ style syntax");

                            ImGui::EndTable();
                        }

                        ImGui::TextUnformatted("Constants");
                        if (ImGui::BeginTable(fmt::format("Constant##{}", id).c_str(), 3,
                                              ImGuiTableFlags_Borders | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_NoHostExtendX, ImVec2(0.0F, 0.0F)))
                        {
                            ImGui::TableSetupColumn("Constant");
                            ImGui::TableSetupColumn("Description");
                            ImGui::TableSetupColumn("Remarks");
                            ImGui::TableHeadersRow();

                            tableEntry("_pi", "The one and only pi", "3.14159265359");
                            tableEntry("_e", "Euler's number", "2.71828182846");

                            ImGui::EndTable();
                        }
                    }
                    ImGui::EndGroup();

                    gui::widgets::EndHelpMarker(false);
                }
            }
        }
        if (eventMarker && eventTooltips)
        {
            ImGui::Separator();
            if (ImGui::Checkbox("Events", &eventsEnabled))
            {
                ret.changed = true;
            }
            if (eventsEnabled)
            {
                if (ImGui::Combo("Event Marker Style", &eventMarkerStyle,
                                 "Circle\0Square\0Diamond\0Up\0Down\0Left\0Right\0Cross\0Plus\0Asterisk\0\0"))
                {
                    ret.changed = true;
                }
                if (ImGui::DragFloat("Event Marker Size", &eventMarkerSize, 0.1F, 1.0F, 10.0F, "%.2f px"))
                {
                    ret.changed = true;
                }
                if (ImGui::DragFloat("Event Marker Weight", &eventMarkerWeight, 0.05F, 0.5F, 3.0F, "%.2f px"))
                {
                    ret.changed = true;
                }
                bool isColorAuto = ImPlot::IsColorAuto(eventMarkerFillColor);
                auto col = isColorAuto ? ImPlot::GetColormapColor(plotElementIdx) : eventMarkerFillColor;
                if (ImGui::ColorEdit4("Event Marker Fill Color", &col.x))
                {
                    eventMarkerFillColor = col;
                    ret.changed = true;
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
                    ret.changed = true;
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
                    ret.changed = true;
                }
            }
        }

        ImPlot::EndLegendPopup();
    }
    return ret;
}

void PlotItemStyle::plotData(const char* plotName,
                             const ScrollingBuffer<double>& xData,
                             const ScrollingBuffer<double>& yData,
                             int plotElementIdx,
                             int defaultStride,
                             ImPlotLineFlags plotLineFlags,
                             ScrollingBuffer<ImU32>* colormapMaskColors,
                             ScrollingBuffer<ImU32>* markerColormapMaskColors,
                             const std::array<ScrollingBuffer<double>, 2>* yErrorData) const
{
    auto lineColor = ImPlot::IsColorAuto(color) ? ImPlot::GetColormapColor(plotElementIdx) : color;
    if (lineType == PlotItemStyle::LineType::Line)
    {
        ImPlot::SetNextLineStyle(lineColor, thickness);
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

        if (errorBoundsEnabled && yErrorData && ImPlot::GetCurrentPlot()->Items.GetItemByIndex(plotElementIdx)->Show)
        {
            ImPlot::SetNextFillStyle(lineColor, errorBoundsAlpha);
            ImPlot::PlotShaded("",
                               xData.data(),
                               (*yErrorData)[0].data(),
                               (*yErrorData)[1].data(),
                               dataPointCount,
                               ImPlotShadedFlags_None,
                               static_cast<int>(std::ceil(static_cast<double>(yData.offset()) / static_cast<double>(stride))),
                               stride * static_cast<int>(sizeof(double)));
        }
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