// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Plot.hpp"

#include "util/Logger.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"
#include "internal/ConfigManager.hpp"

#include "internal/gui/NodeEditorApplication.hpp"
#include "internal/gui/widgets/HelpMarker.hpp"
#include "internal/gui/widgets/Splitter.hpp"
#include "internal/gui/widgets/imgui_ex.hpp"
#include "util/Json.hpp"
#include "util/StringUtil.hpp"

#include "util/Container/Vector.hpp"

#include "util/Time/TimeBase.hpp"
#include "Navigation/Ellipsoid/Ellipsoid.hpp"
#include "Navigation/Transformations/CoordinateFrames.hpp"
#include "Navigation/Transformations/Units.hpp"

#include <implot_internal.h>

#include <algorithm>
#include <regex>

namespace NAV
{
/// @brief Write info to a json object
/// @param[out] j Json output
/// @param[in] data Object to read info from
void to_json(json& j, const Plot::PinData::PlotData& data)
{
    j = json{
        { "displayName", data.displayName },
    };
}
/// @brief Read info from a json object
/// @param[in] j Json variable to read info from
/// @param[out] data Output object
void from_json(const json& j, Plot::PinData::PlotData& data)
{
    if (j.contains("displayName"))
    {
        j.at("displayName").get_to(data.displayName);
    }
}

/// @brief Write info to a json object
/// @param[out] j Json output
/// @param[in] data Object to read info from
void to_json(json& j, const std::vector<Plot::PinData::PlotData>& data)
{
    for (const auto& i : data)
    {
        if (i.isDynamic) { continue; }
        j.push_back(i);
    }
}

/// @brief Write info to a json object
/// @param[out] j Json output
/// @param[in] data Object to read info from
void to_json(json& j, const Plot::PinData& data)
{
    j = json{
        { "dataIdentifier", data.dataIdentifier },
        { "size", data.size },
        { "plotData", data.plotData },
        { "pinType", data.pinType },
        { "stride", data.stride },
    };
}
/// @brief Read info from a json object
/// @param[in] j Json variable to read info from
/// @param[out] data Output object
void from_json(const json& j, Plot::PinData& data)
{
    if (j.contains("dataIdentifier"))
    {
        j.at("dataIdentifier").get_to(data.dataIdentifier);
    }
    if (j.contains("size"))
    {
        j.at("size").get_to(data.size);
    }
    if (j.contains("plotData") && j.at("plotData").is_array())
    {
        j.at("plotData").get_to(data.plotData);
        for (auto& plotData : data.plotData)
        {
            plotData.buffer = ScrollingBuffer<double>(static_cast<size_t>(data.size));
        }
    }
    if (j.contains("pinType"))
    {
        j.at("pinType").get_to(data.pinType);
    }
    if (j.contains("stride"))
    {
        j.at("stride").get_to(data.stride);
    }
}

/// @brief Write info to a json object
/// @param[out] j Json output
/// @param[in] style Object to read info from
void to_json(json& j, const Plot::PlotInfo::PlotItem::Style& style)
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
}
/// @brief Read info from a json object
/// @param[in] j Json variable to read info from
/// @param[out] style Output object
void from_json(const json& j, Plot::PlotInfo::PlotItem::Style& style)
{
    if (j.contains("legendName"))
    {
        j.at("legendName").get_to(style.legendName);
    }
    if (j.contains("stride"))
    {
        j.at("stride").get_to(style.stride);
    }
    if (j.contains("lineType"))
    {
        j.at("lineType").get_to(style.lineType);
    }
    if (j.contains("color"))
    {
        j.at("color").get_to(style.color);
    }
    if (j.contains("colormapMask"))
    {
        j.at("colormapMask").get_to(style.colormapMask);
    }
    if (j.contains("colormapMaskDataCmpIdx"))
    {
        j.at("colormapMaskDataCmpIdx").get_to(style.colormapMaskDataCmpIdx);
    }
    if (j.contains("thickness"))
    {
        j.at("thickness").get_to(style.thickness);
    }
    if (j.contains("markerColormapMask"))
    {
        j.at("markerColormapMask").get_to(style.markerColormapMask);
    }
    if (j.contains("markerColormapMaskDataCmpIdx"))
    {
        j.at("markerColormapMaskDataCmpIdx").get_to(style.markerColormapMaskDataCmpIdx);
    }
    if (j.contains("markers"))
    {
        j.at("markers").get_to(style.markers);
    }
    if (j.contains("markerStyle"))
    {
        j.at("markerStyle").get_to(style.markerStyle);
    }
    if (j.contains("markerSize"))
    {
        j.at("markerSize").get_to(style.markerSize);
    }
    if (j.contains("markerWeight"))
    {
        j.at("markerWeight").get_to(style.markerWeight);
    }
    if (j.contains("markerFillColor"))
    {
        j.at("markerFillColor").get_to(style.markerFillColor);
    }
    if (j.contains("markerOutlineColor"))
    {
        j.at("markerOutlineColor").get_to(style.markerOutlineColor);
    }
    if (j.contains("eventsEnabled"))
    {
        j.at("eventsEnabled").get_to(style.eventsEnabled);
    }
    if (j.contains("eventMarkerStyle"))
    {
        j.at("eventMarkerStyle").get_to(style.eventMarkerStyle);
    }
    if (j.contains("eventMarkerSize"))
    {
        j.at("eventMarkerSize").get_to(style.eventMarkerSize);
    }
    if (j.contains("eventMarkerWeight"))
    {
        j.at("eventMarkerWeight").get_to(style.eventMarkerWeight);
    }
    if (j.contains("eventMarkerFillColor"))
    {
        j.at("eventMarkerFillColor").get_to(style.eventMarkerFillColor);
    }
    if (j.contains("eventMarkerOutlineColor"))
    {
        j.at("eventMarkerOutlineColor").get_to(style.eventMarkerOutlineColor);
    }
    if (j.contains("eventTooltipFilterRegex"))
    {
        j.at("eventTooltipFilterRegex").get_to(style.eventTooltipFilterRegex);
    }
}

/// @brief Write info to a json object
/// @param[out] j Json output
/// @param[in] data Object to read info from
void to_json(json& j, const Plot::PlotInfo::PlotItem& data)
{
    j = json{
        { "pinIndex", data.pinIndex },
        { "dataIndex", data.dataIndex },
        { "displayName", data.displayName },
        { "axis", data.axis },
        { "style", data.style },
    };
}
/// @brief Read info from a json object
/// @param[in] j Json variable to read info from
/// @param[out] data Output object
void from_json(const json& j, Plot::PlotInfo::PlotItem& data)
{
    if (j.contains("pinIndex"))
    {
        j.at("pinIndex").get_to(data.pinIndex);
    }
    if (j.contains("dataIndex"))
    {
        j.at("dataIndex").get_to(data.dataIndex);
    }
    if (j.contains("displayName"))
    {
        j.at("displayName").get_to(data.displayName);
    }
    if (j.contains("axis"))
    {
        j.at("axis").get_to(data.axis);
    }
    if (j.contains("style"))
    {
        j.at("style").get_to(data.style);
    }
}

/// @brief Write info to a json object
/// @param[out] j Json output
/// @param[in] data Object to read info from
void to_json(json& j, const Plot::PlotInfo& data)
{
    j = json{
        { "size", data.size },
        { "xAxisFlags", data.xAxisFlags },
        { "yAxisFlags", data.yAxisFlags },
        { "xAxisScale", data.xAxisScale },
        { "yAxesScale", data.yAxesScale },
        { "headerText", data.headerText },
        { "leftPaneWidth", data.leftPaneWidth },
        { "plotFlags", data.plotFlags },
        { "rightPaneWidth", data.rightPaneWidth },
        { "selectedPin", data.selectedPin },
        { "selectedXdata", data.selectedXdata },
        { "plotItems", data.plotItems },
        { "title", data.title },
        { "overrideXAxisLabel", data.overrideXAxisLabel },
        { "xAxisLabel", data.xAxisLabel },
        { "y1AxisLabel", data.y1AxisLabel },
        { "y2AxisLabel", data.y2AxisLabel },
        { "y3AxisLabel", data.y3AxisLabel },
    };
}
/// @brief Read info from a json object
/// @param[in] j Json variable to read info from
/// @param[out] data Output object
void from_json(const json& j, Plot::PlotInfo& data)
{
    if (j.contains("size"))
    {
        j.at("size").get_to(data.size);
    }
    if (j.contains("xAxisFlags"))
    {
        j.at("xAxisFlags").get_to(data.xAxisFlags);
    }
    if (j.contains("yAxisFlags"))
    {
        j.at("yAxisFlags").get_to(data.yAxisFlags);
    }
    if (j.contains("xAxisScale"))
    {
        j.at("xAxisScale").get_to(data.xAxisScale);
    }
    if (j.contains("yAxesScale"))
    {
        j.at("yAxesScale").get_to(data.yAxesScale);
    }
    if (j.contains("headerText"))
    {
        j.at("headerText").get_to(data.headerText);
    }
    if (j.contains("leftPaneWidth"))
    {
        j.at("leftPaneWidth").get_to(data.leftPaneWidth);
    }
    if (j.contains("plotFlags"))
    {
        j.at("plotFlags").get_to(data.plotFlags);
    }
    if (j.contains("rightPaneWidth"))
    {
        j.at("rightPaneWidth").get_to(data.rightPaneWidth);
    }
    if (j.contains("selectedPin"))
    {
        j.at("selectedPin").get_to(data.selectedPin);
    }
    if (j.contains("selectedXdata"))
    {
        j.at("selectedXdata").get_to(data.selectedXdata);
    }
    if (j.contains("plotItems"))
    {
        j.at("plotItems").get_to(data.plotItems);
    }
    if (j.contains("title"))
    {
        j.at("title").get_to(data.title);
    }
    if (j.contains("overrideXAxisLabel"))
    {
        j.at("overrideXAxisLabel").get_to(data.overrideXAxisLabel);
    }
    if (j.contains("xAxisLabel"))
    {
        j.at("xAxisLabel").get_to(data.xAxisLabel);
    }
    if (j.contains("y1AxisLabel"))
    {
        j.at("y1AxisLabel").get_to(data.y1AxisLabel);
    }
    if (j.contains("y2AxisLabel"))
    {
        j.at("y2AxisLabel").get_to(data.y2AxisLabel);
    }
    if (j.contains("y3AxisLabel"))
    {
        j.at("y3AxisLabel").get_to(data.y3AxisLabel);
    }
}

} // namespace NAV

NAV::Plot::Plot()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _lockConfigDuringRun = false;
    _guiConfigDefaultWindowSize = { 750, 650 };

    // PinData::PinType::Flow:
    _pinData.at(0).pinType = PinData::PinType::Flow;
    inputPins.at(0).type = Pin::Type::Flow;
    inputPins.at(0).dataIdentifier = _dataIdentifier;
    inputPins.at(0).callback = static_cast<InputPin::FlowFirableCallbackFunc>(&Plot::plotData);
    // PinData::PinType::Bool:
    _pinData.at(1).pinType = PinData::PinType::Bool;
    inputPins.at(1).type = Pin::Type::Bool;
    inputPins.at(1).dataIdentifier.clear();
    inputPins.at(1).callback = static_cast<InputPin::DataChangedNotifyFunc>(&Plot::plotBoolean);
    // PinData::PinType::Int:
    _pinData.at(2).pinType = PinData::PinType::Int;
    inputPins.at(2).type = Pin::Type::Int;
    inputPins.at(2).dataIdentifier.clear();
    inputPins.at(2).callback = static_cast<InputPin::DataChangedNotifyFunc>(&Plot::plotInteger);
    // PinData::PinType::Float:
    _pinData.at(3).pinType = PinData::PinType::Float;
    inputPins.at(3).type = Pin::Type::Float;
    inputPins.at(3).dataIdentifier.clear();
    inputPins.at(3).callback = static_cast<InputPin::DataChangedNotifyFunc>(&Plot::plotFloat);
    // PinData::PinType::Matrix:
    _pinData.at(4).pinType = PinData::PinType::Matrix;
    inputPins.at(4).type = Pin::Type::Matrix;
    inputPins.at(4).dataIdentifier = { "Eigen::MatrixXd", "Eigen::VectorXd" };
    inputPins.at(4).callback = static_cast<InputPin::DataChangedNotifyFunc>(&Plot::plotMatrix);
}

NAV::Plot::~Plot()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::Plot::typeStatic()
{
    return "Plot";
}

std::string NAV::Plot::type() const
{
    return typeStatic();
}

std::string NAV::Plot::category()
{
    return "Plot";
}

void NAV::Plot::guiConfig()
{
    ImGui::SetNextItemOpen(false, ImGuiCond_FirstUseEver);
    if (ImGui::CollapsingHeader(fmt::format("Options##{}", size_t(id)).c_str()))
    {
        auto columnContentPinType = [&](size_t pinIndex) -> bool {
            auto& pinData = _pinData.at(pinIndex);
            ImGui::SetNextItemWidth(100.0F * gui::NodeEditorApplication::windowFontRatio());
            if (ImGui::Combo(fmt::format("##Pin Type for Pin {} - {}", pinIndex + 1, size_t(id)).c_str(),
                             reinterpret_cast<int*>(&pinData.pinType), "Flow\0Bool\0Int\0Float\0Matrix\0\0"))
            {
                if (inputPins.at(pinIndex).isPinLinked())
                {
                    inputPins.at(pinIndex).deleteLink();
                }

                switch (pinData.pinType)
                {
                case PinData::PinType::Flow:
                    inputPins.at(pinIndex).type = Pin::Type::Flow;
                    inputPins.at(pinIndex).dataIdentifier = _dataIdentifier;
                    inputPins.at(pinIndex).callback = static_cast<InputPin::FlowFirableCallbackFunc>(&Plot::plotData);
                    break;
                case PinData::PinType::Bool:
                    inputPins.at(pinIndex).type = Pin::Type::Bool;
                    inputPins.at(pinIndex).dataIdentifier.clear();
                    inputPins.at(pinIndex).callback = static_cast<InputPin::DataChangedNotifyFunc>(&Plot::plotBoolean);
                    break;
                case PinData::PinType::Int:
                    inputPins.at(pinIndex).type = Pin::Type::Int;
                    inputPins.at(pinIndex).dataIdentifier.clear();
                    inputPins.at(pinIndex).callback = static_cast<InputPin::DataChangedNotifyFunc>(&Plot::plotInteger);
                    break;
                case PinData::PinType::Float:
                    inputPins.at(pinIndex).type = Pin::Type::Float;
                    inputPins.at(pinIndex).dataIdentifier.clear();
                    inputPins.at(pinIndex).callback = static_cast<InputPin::DataChangedNotifyFunc>(&Plot::plotFloat);
                    break;
                case PinData::PinType::Matrix:
                    inputPins.at(pinIndex).type = Pin::Type::Matrix;
                    inputPins.at(pinIndex).dataIdentifier = { "Eigen::MatrixXd", "Eigen::VectorXd" };
                    inputPins.at(pinIndex).callback = static_cast<InputPin::DataChangedNotifyFunc>(&Plot::plotMatrix);
                    break;
                }

                return true;
            }
            return false;
        };
        auto columnContentDataPoints = [&](size_t pinIndex) -> bool {
            bool changed = false;
            auto& pinData = _pinData.at(pinIndex);
            ImGui::SetNextItemWidth(100.0F * gui::NodeEditorApplication::windowFontRatio());
            if (ImGui::DragInt(fmt::format("##Data Points {} - {}", size_t(id), pinIndex + 1).c_str(),
                               &pinData.size, 10.0F, 0, INT32_MAX / 2))
            {
                if (pinData.size < 0)
                {
                    pinData.size = 0;
                }
                std::scoped_lock<std::mutex> guard(pinData.mutex);
                for (auto& plotData : pinData.plotData)
                {
                    changed = true;
                    plotData.buffer.resize(static_cast<size_t>(pinData.size));
                }
            }
            if (ImGui::IsItemHovered())
            {
                ImGui::SetTooltip("The amount of data which should be stored before the buffer gets reused.\nEnter 0 to show all data.");
            }
            return changed;
        };
        auto columnContentStride = [&](size_t pinIndex) -> bool {
            bool changed = false;
            auto& pinData = _pinData.at(pinIndex);
            ImGui::SetNextItemWidth(100.0F * gui::NodeEditorApplication::windowFontRatio());
            if (ImGui::InputInt(fmt::format("##Stride {} - {}", size_t(id), pinIndex + 1).c_str(),
                                &pinData.stride))
            {
                if (pinData.stride < 1)
                {
                    pinData.stride = 1;
                }
                changed = true;
            }
            if (ImGui::IsItemHovered())
            {
                ImGui::SetTooltip("The amount of points to skip when plotting. This greatly reduces lag when plotting");
            }
            return changed;
        };

        if (_dynamicInputPins.ShowGuiWidgets(size_t(id), inputPins, this,
                                             { { "Pin Type", columnContentPinType },
                                               { "# Data Points", columnContentDataPoints },
                                               { "Stride", columnContentStride } }))
        {
            flow::ApplyChanges();
        }

        if (ImGui::Checkbox(fmt::format("Override local position origin (North/East)##{}", size_t(id)).c_str(), &_overridePositionStartValues))
        {
            flow::ApplyChanges();
            LOG_DEBUG("{}: overridePositionStartValues changed to {}", nameId(), _overridePositionStartValues);
            if (!_originPosition) { _originPosition = gui::widgets::PositionWithFrame(); }
        }
        if (_overridePositionStartValues)
        {
            ImGui::Indent();
            if (gui::widgets::PositionInput(fmt::format("Origin##{}", size_t(id)).c_str(), _originPosition.value(), gui::widgets::PositionInputLayout::SINGLE_ROW))
            {
                flow::ApplyChanges();
            }
            ImGui::Unindent();
        }
    }

    // Used to reset the member variabel _dragAndDropHeaderIndex in case no plot does a drag and drop action
    bool dragAndDropHeaderStillInProgress = false;

    auto showDragDropTargetHeader = [this](size_t plotIdxTarget) {
        ImGui::Dummy(ImVec2(-1.F, 2.F));

        bool selectableSelectedDummy = true;
        ImGui::PushStyleVar(ImGuiStyleVar_SelectableTextAlign, ImVec2(0.5F, 0.5F));
        ImGui::PushStyleColor(ImGuiCol_Header, IM_COL32(16, 173, 44, 79));
        ImGui::Selectable(fmt::format("[drop here]").c_str(), &selectableSelectedDummy, ImGuiSelectableFlags_None, ImVec2(ImGui::GetWindowContentRegionWidth(), 20.F));
        ImGui::PopStyleColor();
        ImGui::PopStyleVar();

        if (ImGui::BeginDragDropTarget())
        {
            if (const ImGuiPayload* payloadData = ImGui::AcceptDragDropPayload(fmt::format("DND ColHead {}", size_t(id)).c_str()))
            {
                auto plotIdxSource = *static_cast<size_t*>(payloadData->Data);

                if (plotIdxSource < plotIdxTarget)
                {
                    --plotIdxTarget;
                }

                move(_plots, plotIdxSource, plotIdxTarget);
                flow::ApplyChanges();
            }
            ImGui::EndDragDropTarget();
        }
        ImGui::Dummy(ImVec2(-1.F, 2.F));
    };

    if (_dragAndDropHeaderIndex > 0)
    {
        showDragDropTargetHeader(0);
    }

    for (size_t plotIdx = 0; plotIdx < _plots.size(); plotIdx++)
    {
        auto& plot = _plots.at(plotIdx);

        size_t plotElementIdx = 0;

        if (!plot.visible) // In the previous frame the x was pressed on the plot
        {
            LOG_DEBUG("{}: # Plot '{}' at index {} was deleted", nameId(), plot.headerText, plotIdx);
            _plots.erase(_plots.begin() + static_cast<int64_t>(plotIdx));
            _nPlots -= 1;
            flow::ApplyChanges();
            continue;
        }

        ImGui::SetNextItemOpen(true, ImGuiCond_Once);
        if (ImGui::CollapsingHeader(fmt::format("{}##Plot Header {} - {}", plot.headerText, size_t(id), plotIdx).c_str(), &plot.visible))
        {
            if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_None))
            {
                dragAndDropHeaderStillInProgress = true;
                _dragAndDropHeaderIndex = static_cast<int>(plotIdx);
                // Data is copied into heap inside the drag and drop
                ImGui::SetDragDropPayload(fmt::format("DND ColHead {}", size_t(id)).c_str(),
                                          &plotIdx, sizeof(plotIdx));
                ImGui::Dummy(ImVec2(ImGui::CalcTextSize(plot.headerText.c_str()).x + 60.F, -1.F));
                bool dnd_display_close = true;
                ImGui::CollapsingHeader(fmt::format("{}##Plot DND Header {} - {}", plot.headerText, size_t(id), plotIdx).c_str(), &dnd_display_close);
                ImGui::EndDragDropSource();
            }

            ImGui::SetNextItemOpen(false, ImGuiCond_FirstUseEver);
            if (ImGui::TreeNode(fmt::format("Options##{} - {}", size_t(id), plotIdx).c_str()))
            {
                std::string headerTitle = plot.headerText;
                ImGui::InputText(fmt::format("Header Title##{} - {}", size_t(id), plotIdx).c_str(), &headerTitle);
                if (plot.headerText != headerTitle && !ImGui::IsItemActive())
                {
                    plot.headerText = headerTitle;
                    flow::ApplyChanges();
                    LOG_DEBUG("{}: Header changed to {}", nameId(), plot.headerText);
                }
                if (ImGui::InputText(fmt::format("Plot Title##{} - {}", size_t(id), plotIdx).c_str(), &plot.title))
                {
                    flow::ApplyChanges();
                    LOG_DEBUG("{}: Plot Title changed to {}", nameId(), plot.title);
                }
                if (ImGui::SliderFloat(fmt::format("Plot Height##{} - {}", size_t(id), plotIdx).c_str(), &plot.size.y, 0.0F, 1000, "%.0f"))
                {
                    flow::ApplyChanges();
                }
                if (ImGui::Checkbox(fmt::format("Override X Axis Label##{} - {}", size_t(id), plotIdx).c_str(), &plot.overrideXAxisLabel))
                {
                    flow::ApplyChanges();
                }
                if (plot.overrideXAxisLabel)
                {
                    if (ImGui::InputText(fmt::format("X Axis Label##{} - {}", size_t(id), plotIdx).c_str(), &plot.xAxisLabel))
                    {
                        flow::ApplyChanges();
                    }
                }
                if (ImGui::InputText(fmt::format("Y1 Axis Label##{} - {}", size_t(id), plotIdx).c_str(), &plot.y1AxisLabel))
                {
                    flow::ApplyChanges();
                }
                if (plot.plotFlags & ImPlotFlags_YAxis2)
                {
                    if (ImGui::InputText(fmt::format("Y2 Axis Label##{} - {}", size_t(id), plotIdx).c_str(), &plot.y2AxisLabel))
                    {
                        flow::ApplyChanges();
                    }
                }
                if (plot.plotFlags & ImPlotFlags_YAxis3)
                {
                    if (ImGui::InputText(fmt::format("Y3 Axis Label##{} - {}", size_t(id), plotIdx).c_str(), &plot.y3AxisLabel))
                    {
                        flow::ApplyChanges();
                    }
                }
                if (ImGui::BeginTable(fmt::format("Pin Settings##{} - {}", size_t(id), plotIdx).c_str(), 2,
                                      ImGuiTableFlags_Borders | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_NoHostExtendX, ImVec2(0.0F, 0.0F)))
                {
                    ImGui::TableSetupColumn("Pin");
                    ImGui::TableSetupColumn("X Data");
                    ImGui::TableHeadersRow();

                    for (size_t pinIndex = 0; pinIndex < _pinData.size(); pinIndex++)
                    {
                        auto& pinData = _pinData.at(pinIndex);

                        ImGui::TableNextRow();
                        ImGui::TableNextColumn(); // Pin
                        ImGui::Text("%zu - %s", pinIndex + 1, pinData.dataIdentifier.c_str());

                        ImGui::TableNextColumn(); // X Data
                        if (!pinData.plotData.empty())
                        {
                            ImGui::SetNextItemWidth(200.0F * gui::NodeEditorApplication::windowFontRatio());
                            if (ImGui::BeginCombo(fmt::format("##X Data for Pin {} - {} - {}", pinIndex + 1, size_t(id), plotIdx).c_str(),
                                                  pinData.plotData.at(plot.selectedXdata.at(pinIndex)).displayName.c_str()))
                            {
                                for (size_t plotDataIndex = 0; plotDataIndex < pinData.plotData.size(); plotDataIndex++)
                                {
                                    auto& plotData = pinData.plotData.at(plotDataIndex);

                                    if (!plotData.hasData)
                                    {
                                        ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5F);
                                    }
                                    const bool is_selected = (plot.selectedXdata.at(pinIndex) == plotDataIndex);
                                    if (ImGui::Selectable(pinData.plotData.at(plotDataIndex).displayName.c_str(), is_selected))
                                    {
                                        flow::ApplyChanges();
                                        plot.selectedXdata.at(pinIndex) = plotDataIndex;
                                        for (auto& plotItem : plot.plotItems)
                                        {
                                            plotItem.eventMarker.clear();
                                            plotItem.eventTooltips.clear();
                                        }
                                    }
                                    if (!plotData.hasData)
                                    {
                                        ImGui::PopStyleVar();
                                    }

                                    // Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
                                    if (is_selected)
                                    {
                                        ImGui::SetItemDefaultFocus();
                                    }
                                }
                                ImGui::EndCombo();
                            }
                        }
                    }

                    ImGui::EndTable();
                }

                if (ImGui::CheckboxFlags(fmt::format("Y-Axis 2##{} - {}", size_t(id), plotIdx).c_str(),
                                         &plot.plotFlags, ImPlotFlags_YAxis2))
                {
                    flow::ApplyChanges();
                }
                ImGui::SameLine();
                if (ImGui::CheckboxFlags(fmt::format("Y-Axis 3##{} - {}", size_t(id), plotIdx).c_str(),
                                         &plot.plotFlags, ImPlotFlags_YAxis3))
                {
                    flow::ApplyChanges();
                }
                ImGui::SameLine();

                if (ImGui::CheckboxFlags(fmt::format("Auto Limit X-Axis##{} - {}", size_t(id), plotIdx).c_str(),
                                         &plot.xAxisFlags, ImPlotAxisFlags_AutoFit))
                {
                    flow::ApplyChanges();
                }
                ImGui::SameLine();
                if (ImGui::CheckboxFlags(fmt::format("Auto Limit Y-Axis##{} - {}", size_t(id), plotIdx).c_str(),
                                         &plot.yAxisFlags, ImPlotAxisFlags_AutoFit))
                {
                    flow::ApplyChanges();
                }

                auto axisScaleCombo = [&](const char* label, ImPlotScale& axisScale) {
                    auto getImPlotScaleString = [](ImPlotScale scale) {
                        switch (scale)
                        {
                        case ImPlotScale_Linear: // default linear scale
                            return "Linear";
                        case ImPlotScale_Time: // date/time scale
                            return "Time";
                        case ImPlotScale_Log10: // base 10 logartithmic scale
                            return "Log10";
                        case ImPlotScale_SymLog: // symmetric log scale
                            return "SymLog";
                        default:
                            return "-";
                        }
                        return "-";
                    };

                    ImGui::SetNextItemWidth(100.0F);
                    if (ImGui::BeginCombo(fmt::format("{}-Axis Scale##{} - {}", label, size_t(id), plotIdx).c_str(), getImPlotScaleString(axisScale)))
                    {
                        for (size_t n = 0; n < 4; ++n)
                        {
                            const bool is_selected = (static_cast<size_t>(axisScale) == n);
                            if (ImGui::Selectable(getImPlotScaleString(static_cast<ImPlotScale>(n)), is_selected))
                            {
                                axisScale = static_cast<ImPlotScale>(n);
                                flow::ApplyChanges();
                            }
                            if (is_selected) { ImGui::SetItemDefaultFocus(); } // Set the initial focus when opening the combo
                        }
                        ImGui::EndCombo();
                    }
                };
                axisScaleCombo("X", plot.xAxisScale);
                ImGui::SameLine();
                axisScaleCombo("Y1", plot.yAxesScale[0]);
                if (plot.plotFlags & ImPlotFlags_YAxis2)
                {
                    ImGui::SameLine();
                    axisScaleCombo("Y2", plot.yAxesScale[1]);
                }
                if (plot.plotFlags & ImPlotFlags_YAxis3)
                {
                    ImGui::SameLine();
                    axisScaleCombo("Y3", plot.yAxesScale[2]);
                }

                ImGui::TreePop();
            }

            gui::widgets::Splitter(fmt::format("Splitter {} - {}", size_t(id), plotIdx).c_str(),
                                   true, 4.0F, &plot.leftPaneWidth, &plot.rightPaneWidth, 3.0F, 80.0F, plot.size.y);

            ImGui::SetNextItemWidth(plot.leftPaneWidth - 2.0F);

            ImGui::BeginGroup();
            {
                if (ImGui::BeginCombo(fmt::format("##Data source pin selection{} - {}", size_t(id), plotIdx).c_str(),
                                      inputPins.at(plot.selectedPin).name.c_str()))
                {
                    for (size_t n = 0; n < inputPins.size(); ++n)
                    {
                        const bool is_selected = (plot.selectedPin == n);
                        if (ImGui::Selectable(inputPins.at(n).name.c_str(), is_selected, 0))
                        {
                            plot.selectedPin = n;
                        }

                        // Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
                        if (is_selected)
                        {
                            ImGui::SetItemDefaultFocus();
                        }
                    }
                    ImGui::EndCombo();
                }
                auto comboBoxSize = ImGui::GetItemRectSize();
                if (ImGui::Button(fmt::format("Clear##{} - {}", size_t(id), plotIdx).c_str(), ImVec2(plot.leftPaneWidth - 2.0F, 0)))
                {
                    plot.plotItems.clear();
                    flow::ApplyChanges();
                }
                if (ImGui::BeginDragDropTarget())
                {
                    if (const ImGuiPayload* payloadData = ImGui::AcceptDragDropPayload(fmt::format("DND PlotItem {} - {}", size_t(id), plotIdx).c_str()))
                    {
                        auto [pinIndex, dataIndex, displayName] = *static_cast<std::tuple<size_t, size_t, std::string*>*>(payloadData->Data);

                        auto iter = std::find(plot.plotItems.begin(), plot.plotItems.end(), PlotInfo::PlotItem{ pinIndex, dataIndex, *displayName });
                        if (iter != plot.plotItems.end())
                        {
                            plot.plotItems.erase(iter);
                            flow::ApplyChanges();
                        }
                    }
                    ImGui::EndDragDropTarget();
                }
                auto buttonSize = ImGui::GetItemRectSize();
                ImGui::BeginChild(fmt::format("Data Drag{} - {}", size_t(id), plotIdx).c_str(),
                                  ImVec2(plot.leftPaneWidth - 2.0F, plot.size.y - comboBoxSize.y - buttonSize.y - 2 * ImGui::GetStyle().ItemSpacing.y),
                                  true);
                {
                    // Left Data Selectables
                    for (size_t dataIndex = 0; dataIndex < _pinData.at(plot.selectedPin).plotData.size(); ++dataIndex)
                    {
                        auto& plotData = _pinData.at(plot.selectedPin).plotData.at(dataIndex);
                        auto plotDataHasData = plotData.hasData;
                        if (!plotDataHasData)
                        {
                            ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5F);
                        }
                        std::string label = plotData.displayName;

                        if (auto iter = std::find(plot.plotItems.begin(), plot.plotItems.end(), PlotInfo::PlotItem{ plot.selectedPin, dataIndex, plotData.displayName });
                            iter != plot.plotItems.end())
                        {
                            label += fmt::format(" (Y{})", iter->axis + 1 - 3);
                        }

                        ImGui::Selectable(label.c_str(), false, 0);
                        if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_None))
                        {
                            // Data is copied into heap inside the drag and drop
                            auto pinAndDataIndex = std::make_tuple(plot.selectedPin, dataIndex, &plotData.displayName);
                            ImGui::SetDragDropPayload(fmt::format("DND PlotItem {} - {}", size_t(id), plotIdx).c_str(),
                                                      &pinAndDataIndex, sizeof(pinAndDataIndex));
                            ImGui::TextUnformatted(label.c_str());
                            ImGui::EndDragDropSource();
                        }

                        if (!plotDataHasData)
                        {
                            ImGui::PopStyleVar();
                        }
                    }

                    ImGui::EndChild();
                }
                ImGui::EndGroup();
            }

            ImGui::SameLine();

            const char* xLabel = plot.overrideXAxisLabel ? (!plot.xAxisLabel.empty() ? plot.xAxisLabel.c_str() : nullptr)
                                                         : (!_pinData.at(0).plotData.empty() ? _pinData.at(0).plotData.at(plot.selectedXdata.at(0)).displayName.c_str() : nullptr);

            const char* y1Label = !plot.y1AxisLabel.empty() ? plot.y1AxisLabel.c_str() : nullptr;
            const char* y2Label = (plot.plotFlags & ImPlotFlags_YAxis2) && !plot.y2AxisLabel.empty() ? plot.y2AxisLabel.c_str() : nullptr;
            const char* y3Label = (plot.plotFlags & ImPlotFlags_YAxis3) && !plot.y3AxisLabel.empty() ? plot.y3AxisLabel.c_str() : nullptr;

            if (ImPlot::BeginPlot(fmt::format("{}##{} - {}", plot.title, size_t(id), plotIdx).c_str(), plot.size, plot.plotFlags))
            {
                ImPlot::SetupAxis(ImAxis_X1, xLabel, plot.xAxisFlags);
                ImPlot::SetupAxisScale(ImAxis_X1, plot.xAxisScale);
                ImPlot::SetupAxis(ImAxis_Y1, y1Label, plot.yAxisFlags);
                ImPlot::SetupAxisScale(ImAxis_Y1, plot.yAxesScale[0]);
                if (plot.plotFlags & ImPlotFlags_YAxis2)
                {
                    ImPlot::SetupAxis(ImAxis_Y2, y2Label, plot.yAxisFlags | ImPlotAxisFlags_NoGridLines | ImPlotAxisFlags_Opposite);
                    ImPlot::SetupAxisScale(ImAxis_Y2, plot.yAxesScale[1]);
                }
                if (plot.plotFlags & ImPlotFlags_YAxis3)
                {
                    ImPlot::SetupAxis(ImAxis_Y3, y3Label, plot.yAxisFlags | ImPlotAxisFlags_NoGridLines | ImPlotAxisFlags_Opposite);
                    ImPlot::SetupAxisScale(ImAxis_Y3, plot.yAxesScale[2]);
                }

                std::vector<PlotInfo::PlotItem> plotItemsToRemove;
                for (auto& plotItem : plot.plotItems)
                {
                    auto& pinData = _pinData.at(plotItem.pinIndex);

                    // Lock the buffer so no data can be inserted till plotting finishes
                    std::scoped_lock<std::mutex> guard(pinData.mutex);
                    // The next line needs already be locked, otherwise we have a data race

                    if (pinData.plotData.size() <= plotItem.dataIndex) { continue; } // Dynamic data can not be available yet
                    auto& plotData = pinData.plotData.at(plotItem.dataIndex);
                    const auto& plotDataX = pinData.plotData.at(plot.selectedXdata.at(plotItem.pinIndex));
                    if (plotData.displayName != plotItem.displayName)
                    {
                        if (plotItem.displayName.empty())
                        {
                            plotItem.displayName = plotData.displayName; // old flow file where it was not set yet
                        }
                        else
                        {
                            plotItemsToRemove.push_back(plotItem);
                            continue;
                        }
                    }

                    if (plotData.hasData
                        && (plotItem.axis == ImAxis_Y1
                            || (plotItem.axis == ImAxis_Y2 && (plot.plotFlags & ImPlotFlags_YAxis2))
                            || (plotItem.axis == ImAxis_Y3 && (plot.plotFlags & ImPlotFlags_YAxis3))))
                    {
                        ImPlot::SetAxis(plotItem.axis);

                        if (plotItem.style.colormapMask.first != ColormapMaskType::None)
                        {
                            if (const auto& cmap = ColormapSearch(plotItem.style.colormapMask.first, plotItem.style.colormapMask.second))
                            {
                                if (plotItem.colormapMaskVersion != cmap->get().version) { plotItem.colormapMaskColors.clear(); }
                                if (plotItem.colormapMaskColors.size() != plotData.buffer.size()
                                    && plotItem.style.colormapMaskDataCmpIdx < pinData.plotData.size())
                                {
                                    plotItem.colormapMaskVersion = cmap->get().version;
                                    const auto& cmpData = pinData.plotData.at(plotItem.style.colormapMaskDataCmpIdx);

                                    auto color = plotItem.style.lineType == PlotInfo::PlotItem::Style::LineType::Line
                                                     ? (ImPlot::IsColorAuto(plotItem.style.color) ? ImPlot::GetColormapColor(static_cast<int>(plotElementIdx)) : plotItem.style.color)
                                                     : (ImPlot::IsColorAuto(plotItem.style.markerFillColor) ? ImPlot::GetColormapColor(static_cast<int>(plotElementIdx)) : plotItem.style.markerFillColor);
                                    plotItem.colormapMaskColors.reserve(plotData.buffer.size());
                                    for (size_t i = plotItem.colormapMaskColors.size(); i < plotData.buffer.size(); i++)
                                    {
                                        plotItem.colormapMaskColors.push_back(i >= cmpData.buffer.size() ? ImColor(color) : cmap->get().getColor(cmpData.buffer.at(i), color));
                                    }
                                }
                            }
                            else
                            {
                                plotItem.style.colormapMask.first = ColormapMaskType::None;
                                plotItem.style.colormapMask.second = -1;
                                plotItem.colormapMaskColors.clear();
                            }
                        }
                        if (plotItem.style.markers && plotItem.style.markerColormapMask.first != ColormapMaskType::None)
                        {
                            if (const auto& cmap = ColormapSearch(plotItem.style.markerColormapMask.first, plotItem.style.markerColormapMask.second))
                            {
                                if (plotItem.markerColormapMaskVersion != cmap->get().version) { plotItem.markerColormapMaskColors.clear(); }
                                if (plotItem.markerColormapMaskColors.size() != plotData.buffer.size()
                                    && plotItem.style.markerColormapMaskDataCmpIdx < pinData.plotData.size())
                                {
                                    plotItem.markerColormapMaskVersion = cmap->get().version;
                                    const auto& cmpData = pinData.plotData.at(plotItem.style.markerColormapMaskDataCmpIdx);

                                    auto color = plotItem.style.lineType == PlotInfo::PlotItem::Style::LineType::Line
                                                     ? (ImPlot::IsColorAuto(plotItem.style.color) ? ImPlot::GetColormapColor(static_cast<int>(plotElementIdx)) : plotItem.style.color)
                                                     : (ImPlot::IsColorAuto(plotItem.style.markerFillColor) ? ImPlot::GetColormapColor(static_cast<int>(plotElementIdx)) : plotItem.style.markerFillColor);
                                    plotItem.markerColormapMaskColors.reserve(plotData.buffer.size());
                                    for (size_t i = plotItem.markerColormapMaskColors.size(); i < plotData.buffer.size(); i++)
                                    {
                                        plotItem.markerColormapMaskColors.push_back(i >= cmpData.buffer.size() ? ImColor(color) : cmap->get().getColor(cmpData.buffer.at(i), color));
                                    }
                                }
                            }
                            else
                            {
                                plotItem.style.markerColormapMask.first = ColormapMaskType::None;
                                plotItem.style.markerColormapMask.second = -1;
                                plotItem.markerColormapMaskColors.clear();
                            }
                        }
                        if (plotItem.style.eventsEnabled)
                        {
                            if (plotItem.eventMarker.size() != plotData.buffer.size())
                            {
                                auto& plotDataRelTime = pinData.plotData.at(0);
                                for (size_t i = plotItem.eventMarker.size(); i < plotData.buffer.size(); i++)
                                {
                                    double relTime = plotDataRelTime.buffer.at(i);
                                    PlotInfo::PlotItem::Tooltip tooltip;
                                    try
                                    {
                                        std::regex filter(plotItem.style.eventTooltipFilterRegex,
                                                          std::regex_constants::ECMAScript | std::regex_constants::icase);
                                        for (const auto& e : pinData.events)
                                        {
                                            if (std::abs(std::get<0>(e) - relTime) <= 1e-6)
                                            {
                                                tooltip.time = std::get<1>(e);
                                                if (plotItem.style.eventTooltipFilterRegex.empty() || std::regex_search(std::get<2>(e), filter))
                                                {
                                                    tooltip.texts.push_back(std::get<2>(e));
                                                }
                                            }
                                        }
                                    }
                                    catch (...)
                                    {}

                                    if (!tooltip.texts.empty())
                                    {
                                        plotItem.eventMarker.push_back(plotData.buffer.at(i));
                                        plotItem.eventTooltips.emplace_back(plotDataX.buffer.at(i), plotData.buffer.at(i), tooltip);
                                    }
                                    else
                                    {
                                        plotItem.eventMarker.push_back(std::nan(""));
                                    }
                                }
                            }
                        }

                        // Style options
                        if (plotItem.style.legendName.empty())
                        {
                            plotItem.style.legendName = fmt::format("{} ({})", plotData.displayName, inputPins.at(plotItem.pinIndex).name);
                        }
                        if (plotItem.style.lineType == PlotInfo::PlotItem::Style::LineType::Line)
                        {
                            ImPlot::SetNextLineStyle(ImPlot::IsColorAuto(plotItem.style.color) ? ImPlot::GetColormapColor(static_cast<int>(plotElementIdx)) : plotItem.style.color,
                                                     plotItem.style.thickness);
                        }
                        if (plotItem.style.lineType == PlotInfo::PlotItem::Style::LineType::Scatter || plotItem.style.markers)
                        {
                            ImPlot::SetNextMarkerStyle(plotItem.style.markerStyle,
                                                       plotItem.style.markerSize,
                                                       ImPlot::IsColorAuto(plotItem.style.markerFillColor) ? ImPlot::GetColormapColor(static_cast<int>(plotElementIdx)) : plotItem.style.markerFillColor,
                                                       plotItem.style.markerWeight,
                                                       ImPlot::IsColorAuto(plotItem.style.markerOutlineColor) ? ImPlot::GetColormapColor(static_cast<int>(plotElementIdx)) : plotItem.style.markerOutlineColor);
                        }

                        std::string plotName = fmt::format("{}##{} - {} - {}", plotItem.style.legendName, size_t(id), plotItem.pinIndex + 1, plotData.displayName);

                        auto stride = plotItem.style.stride ? plotItem.style.stride
                                                            : pinData.stride;
                        auto dataPointCount = static_cast<int>(std::ceil(static_cast<double>(plotData.buffer.size())
                                                                         / static_cast<double>(stride)));

                        // Plot the data
                        if (plotItem.style.lineType == PlotInfo::PlotItem::Style::LineType::Line)
                        {
                            if (plotItem.style.colormapMask.first != ColormapMaskType::None)
                            {
                                ImPlot::SetNextColorsData(ImPlotCol_Line, plotItem.colormapMaskColors.data(), stride * static_cast<int>(sizeof(ImU32)));
                                if (plotItem.style.markers)
                                {
                                    ImPlot::SetNextColorsData(ImPlotCol_MarkerFill, plotItem.style.markerColormapMask.first != ColormapMaskType::None ? plotItem.markerColormapMaskColors.data() : plotItem.colormapMaskColors.data(), stride * static_cast<int>(sizeof(ImU32)));
                                    ImPlot::SetNextColorsData(ImPlotCol_MarkerOutline, plotItem.style.markerColormapMask.first != ColormapMaskType::None ? plotItem.markerColormapMaskColors.data() : plotItem.colormapMaskColors.data(), stride * static_cast<int>(sizeof(ImU32)));
                                }
                            }
                            else if (plotItem.style.markers && plotItem.style.markerColormapMask.first != ColormapMaskType::None)
                            {
                                ImPlot::SetNextColorsData(ImPlotCol_MarkerFill, plotItem.markerColormapMaskColors.data(), stride * static_cast<int>(sizeof(ImU32)));
                                ImPlot::SetNextColorsData(ImPlotCol_MarkerOutline, plotItem.markerColormapMaskColors.data(), stride * static_cast<int>(sizeof(ImU32)));
                            }
                            ImPlot::PlotLine(plotName.c_str(),
                                             plotDataX.buffer.data(),
                                             plotData.buffer.data(),
                                             dataPointCount,
                                             ImPlotLineFlags_None,
                                             static_cast<int>(std::ceil(static_cast<double>(plotData.buffer.offset()) / static_cast<double>(stride))),
                                             stride * static_cast<int>(sizeof(double)));
                        }
                        else if (plotItem.style.lineType == PlotInfo::PlotItem::Style::LineType::Scatter)
                        {
                            if (plotItem.style.colormapMask.first != ColormapMaskType::None && plotItem.colormapMaskColors.isInfiniteBuffer())
                            {
                                ImPlot::SetNextColorsData(ImPlotCol_MarkerFill, plotItem.colormapMaskColors.data(), stride * static_cast<int>(sizeof(ImU32)));
                                ImPlot::SetNextColorsData(ImPlotCol_MarkerOutline, plotItem.colormapMaskColors.data(), stride * static_cast<int>(sizeof(ImU32)));
                            }
                            ImPlot::PlotScatter(plotName.c_str(),
                                                plotDataX.buffer.data(),
                                                plotData.buffer.data(),
                                                dataPointCount,
                                                ImPlotScatterFlags_None,
                                                static_cast<int>(std::ceil(static_cast<double>(plotData.buffer.offset()) / static_cast<double>(stride))),
                                                stride * static_cast<int>(sizeof(double)));
                        }

                        if (plotItem.style.eventsEnabled)
                        {
                            if (const auto* item = ImPlot::GetCurrentPlot()->Items.GetItem(plotName.c_str());
                                item && item->Show)
                            {
                                ImPlot::SetNextMarkerStyle(plotItem.style.eventMarkerStyle,
                                                           plotItem.style.eventMarkerSize,
                                                           ImPlot::IsColorAuto(plotItem.style.eventMarkerFillColor) ? ImPlot::GetColormapColor(static_cast<int>(plotElementIdx)) : plotItem.style.eventMarkerFillColor,
                                                           plotItem.style.eventMarkerWeight,
                                                           ImPlot::IsColorAuto(plotItem.style.eventMarkerOutlineColor) ? ImPlot::GetColormapColor(static_cast<int>(plotElementIdx)) : plotItem.style.eventMarkerOutlineColor);
                                ImPlot::PlotScatter(fmt::format("##{} - Events", plotName).c_str(),
                                                    plotDataX.buffer.data(),
                                                    plotItem.eventMarker.data(),
                                                    dataPointCount,
                                                    ImPlotScatterFlags_None,
                                                    static_cast<int>(std::ceil(static_cast<double>(plotItem.eventMarker.offset()) / static_cast<double>(stride))),
                                                    stride * static_cast<int>(sizeof(double)));

                                if (ImPlot::IsPlotHovered())
                                {
                                    constexpr double HOVER_PIXEL_SIZE = 5.0;
                                    auto limits = ImPlot::GetPlotLimits(IMPLOT_AUTO, plotItem.axis);

                                    ImVec2 scaling = ImVec2(static_cast<float>(HOVER_PIXEL_SIZE * (limits.X.Max - limits.X.Min) / ImPlot::GetCurrentPlot()->PlotRect.GetWidth()),
                                                            static_cast<float>(HOVER_PIXEL_SIZE * (limits.Y.Max - limits.Y.Min) / ImPlot::GetCurrentPlot()->PlotRect.GetHeight()));
                                    ImPlotPoint mouse = ImPlot::GetPlotMousePos();

                                    std::vector<PlotInfo::PlotItem::Tooltip> tooltips;
                                    for (const auto& e : plotItem.eventTooltips)
                                    {
                                        if (std::abs(mouse.x - std::get<0>(e)) < scaling.x
                                            && std::abs(mouse.y - std::get<1>(e)) < scaling.y)
                                        {
                                            tooltips.push_back(std::get<2>(e));
                                        }
                                    }
                                    if (!tooltips.empty())
                                    {
                                        ImGui::BeginTooltip();
                                        ImGui::PushFont(Application::MonoFont());
                                        for (size_t i = 0; i < tooltips.size(); i++)
                                        {
                                            ImGui::SetNextItemOpen(true, ImGuiCond_Always);
                                            if (ImGui::TreeNode(fmt::format("{} GPST", tooltips.at(i).time.toYMDHMS(GPST)).c_str()))
                                            {
                                                for (const auto& text : tooltips.at(i).texts)
                                                {
                                                    ImGui::BulletText("%s", text.c_str());
                                                }
                                                ImGui::TreePop();
                                            }
                                            if (i != tooltips.size() - 1) { ImGui::Separator(); }
                                        }
                                        ImGui::PopFont();
                                        ImGui::EndTooltip();
                                    }
                                }
                            }
                        }

                        // allow legend item labels to be DND sources
                        if (ImPlot::BeginDragDropSourceItem(plotName.c_str()))
                        {
                            // Data is copied into heap inside the drag and drop
                            auto pinAndDataIndex = std::make_tuple(plotItem.pinIndex, plotItem.dataIndex, &plotItem.displayName);
                            ImGui::SetDragDropPayload(fmt::format("DND PlotItem {} - {}", size_t(id), plotIdx).c_str(), &pinAndDataIndex, sizeof(pinAndDataIndex));
                            ImGui::TextUnformatted(plotData.displayName.c_str());
                            ImPlot::EndDragDropSource();
                        }

                        // Legend item context menu (right click on legend item)
                        if (ImPlot::BeginLegendPopup(plotName.c_str()))
                        {
                            ImGui::TextUnformatted(fmt::format("Pin {} - {}: {}", plotItem.pinIndex + 1, pinData.dataIdentifier, plotData.displayName).c_str());
                            ImGui::Separator();

                            auto ShowColormapReferenceChooser = [&](size_t& colormapMaskDataCmpIdx, const char* label = "") -> bool {
                                bool changed = false;
                                const char* preview = colormapMaskDataCmpIdx < pinData.plotData.size()
                                                          ? pinData.plotData.at(colormapMaskDataCmpIdx).displayName.c_str()
                                                          : "";
                                if (ImGui::BeginCombo(fmt::format("{}Colormap Ref", label).c_str(), preview))
                                {
                                    for (size_t plotDataIndex = 0; plotDataIndex < pinData.plotData.size(); plotDataIndex++)
                                    {
                                        auto& plotData = pinData.plotData.at(plotDataIndex);

                                        if (!plotData.hasData)
                                        {
                                            ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5F);
                                        }
                                        const bool is_selected = (colormapMaskDataCmpIdx == plotDataIndex);
                                        if (ImGui::Selectable(pinData.plotData.at(plotDataIndex).displayName.c_str(), is_selected))
                                        {
                                            changed = true;
                                            colormapMaskDataCmpIdx = plotDataIndex;
                                        }
                                        if (!plotData.hasData)
                                        {
                                            ImGui::PopStyleVar();
                                        }

                                        // Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
                                        if (is_selected)
                                        {
                                            ImGui::SetItemDefaultFocus();
                                        }
                                    }
                                    ImGui::EndCombo();
                                }
                                return changed;
                            };

                            if (plotItem.style.legendNameGui.empty())
                            {
                                plotItem.style.legendNameGui = plotItem.style.legendName;
                            }
                            ImGui::InputText("Legend name", &plotItem.style.legendNameGui);
                            if (plotItem.style.legendNameGui != plotItem.style.legendName && !ImGui::IsItemActive())
                            {
                                plotItem.style.legendName = plotItem.style.legendNameGui;
                                flow::ApplyChanges();
                                LOG_DEBUG("{}: Legend changed to {}", nameId(), plotItem.style.legendName);
                            }

                            if (ImGui::InputInt("Stride", &plotItem.style.stride))
                            {
                                if (plotItem.style.stride < 0)
                                {
                                    plotItem.style.stride = 0;
                                }
                                if (plotItem.style.stride > static_cast<int>(plotData.buffer.size()) - 1)
                                {
                                    plotItem.style.stride = static_cast<int>(plotData.buffer.size()) - 1;
                                }
                                flow::ApplyChanges();
                                LOG_DEBUG("{}: Stride changed to {}", nameId(), plotItem.style.stride);
                            }

                            if (ImGui::Combo("Style", reinterpret_cast<int*>(&plotItem.style.lineType),
                                             "Scatter\0Line\0\0"))
                            {
                                flow::ApplyChanges();
                            }
                            if (plotItem.style.lineType == PlotInfo::PlotItem::Style::LineType::Line)
                            {
                                if (ImGui::DragFloat("Line Thickness", &plotItem.style.thickness, 0.1F, 0.0F, 8.0F, "%.2f px"))
                                {
                                    flow::ApplyChanges();
                                }
                                if (ShowColormapSelector(plotItem.style.colormapMask.first, plotItem.style.colormapMask.second))
                                {
                                    plotItem.colormapMaskColors.clear();
                                    flow::ApplyChanges();
                                }
                                if (plotItem.style.colormapMask.first != ColormapMaskType::None && ShowColormapReferenceChooser(plotItem.style.colormapMaskDataCmpIdx))
                                {
                                    plotItem.colormapMaskColors.clear();
                                    flow::ApplyChanges();
                                }
                                if (plotItem.style.colormapMask.first == ColormapMaskType::None)
                                {
                                    bool isColorAuto = ImPlot::IsColorAuto(plotItem.style.color);
                                    auto col = isColorAuto ? ImPlot::GetColormapColor(static_cast<int>(plotElementIdx)) : plotItem.style.color;
                                    if (ImGui::ColorEdit4("Line Color", &col.x))
                                    {
                                        plotItem.style.color = col;
                                        flow::ApplyChanges();
                                    }
                                    if (!isColorAuto)
                                    {
                                        ImGui::SameLine();
                                        if (ImGui::Button("Auto##Line Color"))
                                        {
                                            plotItem.style.color = IMPLOT_AUTO_COL;
                                        }
                                    }
                                }
                                if (ImGui::Checkbox("Markers", &plotItem.style.markers))
                                {
                                    flow::ApplyChanges();
                                }
                            }
                            if (plotItem.style.lineType == PlotInfo::PlotItem::Style::LineType::Scatter || plotItem.style.markers)
                            {
                                if (ImGui::Combo("Marker Style", &plotItem.style.markerStyle,
                                                 "Circle\0Square\0Diamond\0Up\0Down\0Left\0Right\0Cross\0Plus\0Asterisk\0\0"))
                                {
                                    flow::ApplyChanges();
                                }
                                if (ImGui::DragFloat("Marker Size", &plotItem.style.markerSize, 0.1F, 1.0F, 10.0F, "%.2f px"))
                                {
                                    flow::ApplyChanges();
                                }
                                if (ImGui::DragFloat("Marker Weight", &plotItem.style.markerWeight, 0.05F, 0.5F, 3.0F, "%.2f px"))
                                {
                                    flow::ApplyChanges();
                                }
                                if (!plotItem.style.markers)
                                {
                                    if (ShowColormapSelector(plotItem.style.colormapMask.first, plotItem.style.colormapMask.second))
                                    {
                                        plotItem.colormapMaskColors.clear();
                                        flow::ApplyChanges();
                                    }
                                    if (plotItem.style.colormapMask.first != ColormapMaskType::None && ShowColormapReferenceChooser(plotItem.style.colormapMaskDataCmpIdx))
                                    {
                                        plotItem.colormapMaskColors.clear();
                                        flow::ApplyChanges();
                                    }
                                }
                                if (plotItem.style.markers && plotItem.style.lineType != PlotInfo::PlotItem::Style::LineType::Scatter)
                                {
                                    if (ShowColormapSelector(plotItem.style.markerColormapMask.first, plotItem.style.markerColormapMask.second, "Marker "))
                                    {
                                        plotItem.markerColormapMaskColors.clear();
                                        flow::ApplyChanges();
                                    }
                                    if (plotItem.style.markerColormapMask.first != ColormapMaskType::None && ShowColormapReferenceChooser(plotItem.style.markerColormapMaskDataCmpIdx, "Marker "))
                                    {
                                        plotItem.markerColormapMaskColors.clear();
                                        flow::ApplyChanges();
                                    }
                                }
                                if (plotItem.style.markerColormapMask.first == ColormapMaskType::None
                                    && (plotItem.style.lineType != PlotInfo::PlotItem::Style::LineType::Scatter
                                        || plotItem.style.colormapMask.first == ColormapMaskType::None))
                                {
                                    bool isColorAuto = ImPlot::IsColorAuto(plotItem.style.markerFillColor);
                                    auto col = isColorAuto ? ImPlot::GetColormapColor(static_cast<int>(plotElementIdx)) : plotItem.style.markerFillColor;
                                    if (ImGui::ColorEdit4("Marker Fill Color", &col.x))
                                    {
                                        plotItem.style.markerFillColor = col;
                                        flow::ApplyChanges();
                                    }
                                    if (!isColorAuto)
                                    {
                                        ImGui::SameLine();
                                        if (ImGui::Button("Auto##Marker Fill Color"))
                                        {
                                            plotItem.style.markerFillColor = IMPLOT_AUTO_COL;
                                        }
                                    }

                                    isColorAuto = ImPlot::IsColorAuto(plotItem.style.markerOutlineColor);
                                    col = isColorAuto ? ImPlot::GetColormapColor(static_cast<int>(plotElementIdx)) : plotItem.style.markerOutlineColor;
                                    if (ImGui::ColorEdit4("Marker Outline Color", &col.x))
                                    {
                                        plotItem.style.markerOutlineColor = col;
                                        flow::ApplyChanges();
                                    }
                                    if (!isColorAuto)
                                    {
                                        ImGui::SameLine();
                                        if (ImGui::Button("Auto##Marker Outline Color"))
                                        {
                                            plotItem.style.markerOutlineColor = IMPLOT_AUTO_COL;
                                        }
                                    }
                                }
                            }

                            ImGui::Separator();
                            if (ImGui::Checkbox("Events", &plotItem.style.eventsEnabled))
                            {
                                flow::ApplyChanges();
                            }
                            if (plotItem.style.eventsEnabled)
                            {
                                if (ImGui::Combo("Event Marker Style", &plotItem.style.eventMarkerStyle,
                                                 "Circle\0Square\0Diamond\0Up\0Down\0Left\0Right\0Cross\0Plus\0Asterisk\0\0"))
                                {
                                    flow::ApplyChanges();
                                }
                                if (ImGui::DragFloat("Event Marker Size", &plotItem.style.eventMarkerSize, 0.1F, 1.0F, 10.0F, "%.2f px"))
                                {
                                    flow::ApplyChanges();
                                }
                                if (ImGui::DragFloat("Event Marker Weight", &plotItem.style.eventMarkerWeight, 0.05F, 0.5F, 3.0F, "%.2f px"))
                                {
                                    flow::ApplyChanges();
                                }
                                bool isColorAuto = ImPlot::IsColorAuto(plotItem.style.eventMarkerFillColor);
                                auto col = isColorAuto ? ImPlot::GetColormapColor(static_cast<int>(plotElementIdx)) : plotItem.style.eventMarkerFillColor;
                                if (ImGui::ColorEdit4("Event Marker Fill Color", &col.x))
                                {
                                    plotItem.style.eventMarkerFillColor = col;
                                    flow::ApplyChanges();
                                }
                                if (!isColorAuto)
                                {
                                    ImGui::SameLine();
                                    if (ImGui::Button("Auto##Event Marker Fill Color"))
                                    {
                                        plotItem.style.eventMarkerFillColor = IMPLOT_AUTO_COL;
                                    }
                                }

                                isColorAuto = ImPlot::IsColorAuto(plotItem.style.eventMarkerOutlineColor);
                                col = isColorAuto ? ImPlot::GetColormapColor(static_cast<int>(plotElementIdx)) : plotItem.style.eventMarkerOutlineColor;
                                if (ImGui::ColorEdit4("Event Marker Outline Color", &col.x))
                                {
                                    plotItem.style.eventMarkerOutlineColor = col;
                                    flow::ApplyChanges();
                                }
                                if (!isColorAuto)
                                {
                                    ImGui::SameLine();
                                    if (ImGui::Button("Auto##Event Marker Outline Color"))
                                    {
                                        plotItem.style.eventMarkerOutlineColor = IMPLOT_AUTO_COL;
                                    }
                                }

                                if (ImGui::InputText("Event Filter Regex", &plotItem.style.eventTooltipFilterRegex))
                                {
                                    plotItem.eventMarker.clear();
                                    plotItem.eventTooltips.clear();
                                    flow::ApplyChanges();
                                }
                            }

                            ImPlot::EndLegendPopup();
                        }

                        plotElementIdx++;
                    }
                }

                for (const auto& plotItem : plotItemsToRemove)
                {
                    LOG_WARN("{}: Erasing plot item '{}' from plot '{}', because it does not match the order the data came in",
                             nameId(), plotItem.displayName, plot.headerText);
                    std::erase(plot.plotItems, plotItem);
                    flow::ApplyChanges();
                }

                auto addDragDropPlotToAxis = [this, plotIdx, &plot](ImAxis dragDropAxis) {
                    if (const ImGuiPayload* payloadData = ImGui::AcceptDragDropPayload(fmt::format("DND PlotItem {} - {}", size_t(id), plotIdx).c_str()))
                    {
                        auto [pinIndex, dataIndex, displayName] = *static_cast<std::tuple<size_t, size_t, std::string*>*>(payloadData->Data);

                        auto iter = std::find(plot.plotItems.begin(), plot.plotItems.end(), PlotInfo::PlotItem{ pinIndex, dataIndex, *displayName });
                        if (iter != plot.plotItems.end()) // Item gets dragged from one axis to another
                        {
                            iter->axis = dragDropAxis;
                        }
                        else
                        {
                            plot.plotItems.emplace_back(pinIndex, dataIndex, *displayName, dragDropAxis);
                        }
                        flow::ApplyChanges();
                    }
                };

                // allow the main plot area to be a DND target
                if (ImPlot::BeginDragDropTargetPlot())
                {
                    addDragDropPlotToAxis(ImAxis_Y1);
                    ImPlot::EndDragDropTarget();
                }
                // allow each y-axis to be a DND target
                for (ImAxis y = ImAxis_Y1; y <= ImAxis_Y3; ++y)
                {
                    if ((y == ImAxis_Y2 && !(plot.plotFlags & ImPlotFlags_YAxis2))
                        || (y == ImAxis_Y3 && !(plot.plotFlags & ImPlotFlags_YAxis3)))
                    {
                        continue;
                    }
                    if (ImPlot::BeginDragDropTargetAxis(y))
                    {
                        addDragDropPlotToAxis(y);
                        ImPlot::EndDragDropTarget();
                    }
                }

                ImPlot::EndPlot();
            }
        }

        if (_dragAndDropHeaderIndex >= 0
            && plotIdx != static_cast<size_t>(_dragAndDropHeaderIndex - 1)
            && plotIdx != static_cast<size_t>(_dragAndDropHeaderIndex))
        {
            showDragDropTargetHeader(plotIdx + 1);
        }
    }

    if (!dragAndDropHeaderStillInProgress)
    {
        _dragAndDropHeaderIndex = -1;
    }

    ImGui::Separator();
    if (ImGui::Button(fmt::format("Add Plot##{}", size_t(id)).c_str()))
    {
        ++_nPlots;
        LOG_DEBUG("{}: # Plots changed to {}", nameId(), _nPlots);
        flow::ApplyChanges();
        updateNumberOfPlots();
    }
}

[[nodiscard]] json NAV::Plot::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["dynamicInputPins"] = _dynamicInputPins;
    j["nPlots"] = _nPlots;
    j["pinData"] = _pinData;
    j["plots"] = _plots;
    j["overridePositionStartValues"] = _overridePositionStartValues;
    if (_overridePositionStartValues && _originPosition)
    {
        j["originPosition"] = _originPosition.value();
    }

    return j;
}

void NAV::Plot::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("dynamicInputPins"))
    {
        NAV::gui::widgets::from_json(j.at("dynamicInputPins"), _dynamicInputPins, this);
    }
    if (j.contains("nPlots"))
    {
        j.at("nPlots").get_to(_nPlots);
        updateNumberOfPlots();
    }
    if (j.contains("pinData"))
    {
        j.at("pinData").get_to(_pinData);

        for (size_t inputPinIndex = 0; inputPinIndex < inputPins.size(); inputPinIndex++)
        {
            if (inputPinIndex >= _pinData.size()) { break; }
            switch (_pinData.at(inputPinIndex).pinType)
            {
            case PinData::PinType::Flow:
                inputPins.at(inputPinIndex).type = Pin::Type::Flow;
                inputPins.at(inputPinIndex).dataIdentifier = _dataIdentifier;
                inputPins.at(inputPinIndex).callback = static_cast<InputPin::FlowFirableCallbackFunc>(&Plot::plotData);
                break;
            case Plot::PinData::PinType::Bool:
                inputPins.at(inputPinIndex).type = Pin::Type::Bool;
                inputPins.at(inputPinIndex).dataIdentifier.clear();
                inputPins.at(inputPinIndex).callback = static_cast<InputPin::DataChangedNotifyFunc>(&Plot::plotBoolean);
                break;
            case Plot::PinData::PinType::Int:
                inputPins.at(inputPinIndex).type = Pin::Type::Int;
                inputPins.at(inputPinIndex).dataIdentifier.clear();
                inputPins.at(inputPinIndex).callback = static_cast<InputPin::DataChangedNotifyFunc>(&Plot::plotInteger);
                break;
            case Plot::PinData::PinType::Float:
                inputPins.at(inputPinIndex).type = Pin::Type::Float;
                inputPins.at(inputPinIndex).dataIdentifier.clear();
                inputPins.at(inputPinIndex).callback = static_cast<InputPin::DataChangedNotifyFunc>(&Plot::plotFloat);
                break;
            case Plot::PinData::PinType::Matrix:
                inputPins.at(inputPinIndex).type = Pin::Type::Matrix;
                inputPins.at(inputPinIndex).dataIdentifier = { "Eigen::MatrixXd", "Eigen::VectorXd" };
                inputPins.at(inputPinIndex).callback = static_cast<InputPin::DataChangedNotifyFunc>(&Plot::plotMatrix);
                break;
            default:
                break;
            }
        }
    }
    if (j.contains("plots"))
    {
        j.at("plots").get_to(_plots);
    }
    if (j.contains("overridePositionStartValues"))
    {
        j.at("overridePositionStartValues").get_to(_overridePositionStartValues);
    }
    if (_overridePositionStartValues)
    {
        if (j.contains("originPosition"))
        {
            _originPosition = j.at("originPosition").get<gui::widgets::PositionWithFrame>();
        }
        else
        {
            _originPosition = gui::widgets::PositionWithFrame();
        }
    }
}

bool NAV::Plot::initialize()
{
    LOG_TRACE("{}: called", nameId());

    _startTime.reset();
    if (!_overridePositionStartValues) { _originPosition.reset(); }

    for (auto& pinData : _pinData)
    {
        std::scoped_lock<std::mutex> guard(pinData.mutex); // Lock the buffer for multithreaded access

        for (auto& plotData : pinData.plotData)
        {
            plotData.hasData = false;
            plotData.buffer.clear();
        }
        if (pinData.dynamicDataStartIndex != -1 && static_cast<int>(pinData.plotData.size()) >= pinData.dynamicDataStartIndex) // Erase all dynamic data
        {
            pinData.plotData.erase(pinData.plotData.begin() + pinData.dynamicDataStartIndex, pinData.plotData.end());
        }
        pinData.events.clear();
    }
    for (auto& plot : _plots)
    {
        for (auto& plotItem : plot.plotItems)
        {
            plotItem.colormapMaskColors.clear();
            plotItem.markerColormapMaskColors.clear();
            plotItem.eventMarker.clear();
            plotItem.eventTooltips.clear();
        }
    }

    return true;
}

void NAV::Plot::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

void NAV::Plot::afterCreateLink(OutputPin& startPin, InputPin& endPin)
{
    LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(startPin.id), size_t(endPin.id));

    size_t pinIndex = inputPinIndexFromId(endPin.id);

    for (auto& plotData : _pinData.at(pinIndex).plotData) // Mark all plot data for deletion
    {
        plotData.markedForDelete = true;
    }

    size_t i = 0;

    if (inputPins.at(pinIndex).type == Pin::Type::Flow)
    {
        if (startPin.dataIdentifier.size() > 1)
        {
            // Happens if connected Node supports multiple output values which can be chosen, but the node did not load yet.
            // But it will and then recreate the link
            return;
        }
        if (_pinData.at(pinIndex).dataIdentifier != startPin.dataIdentifier.front())
        {
            _pinData.at(pinIndex).plotData.clear();
            for (auto& plot : _plots)
            {
                while (true)
                {
                    auto plotItemIter = std::find_if(plot.plotItems.begin(), plot.plotItems.end(),
                                                     [pinIndex](const PlotInfo::PlotItem& plotItem) { return plotItem.pinIndex == pinIndex; });
                    if (plotItemIter != plot.plotItems.end())
                    {
                        plot.plotItems.erase(plotItemIter);
                    }
                    else
                    {
                        break;
                    }
                }
            }
        }

        _pinData.at(pinIndex).dataIdentifier = startPin.dataIdentifier.front();

        if (startPin.dataIdentifier.front() == Pos::type())
        {
            // NodeData
            _pinData.at(pinIndex).addPlotDataItem(i++, "Time [s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GPS time of week [s]");
            // Pos
            _pinData.at(pinIndex).addPlotDataItem(i++, "Latitude [deg]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Longitude [deg]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Altitude [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "North/South [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "East/West [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "X-ECEF [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Y-ECEF [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Z-ECEF [m]");
        }
        else if (startPin.dataIdentifier.front() == PosVel::type())
        {
            // NodeData
            _pinData.at(pinIndex).addPlotDataItem(i++, "Time [s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GPS time of week [s]");
            // PosVel
            _pinData.at(pinIndex).addPlotDataItem(i++, "Latitude [deg]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Longitude [deg]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Altitude [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "North/South [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "East/West [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "X-ECEF [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Y-ECEF [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Z-ECEF [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Velocity norm [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "X velocity ECEF [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Y velocity ECEF [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Z velocity ECEF [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "North velocity [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "East velocity [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Down velocity [m/s]");
        }
        else if (startPin.dataIdentifier.front() == PosVelAtt::type()
                 || startPin.dataIdentifier.front() == InertialNavSol::type())
        {
            // NodeData
            _pinData.at(pinIndex).addPlotDataItem(i++, "Time [s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GPS time of week [s]");
            // PosVelAtt
            _pinData.at(pinIndex).addPlotDataItem(i++, "Latitude [deg]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Longitude [deg]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Altitude [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "North/South [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "East/West [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "X-ECEF [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Y-ECEF [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Z-ECEF [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Velocity norm [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "X velocity ECEF [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Y velocity ECEF [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Z velocity ECEF [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "North velocity [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "East velocity [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Down velocity [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Roll [deg]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Pitch [deg]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Yaw [deg]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Quaternion::w");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Quaternion::x");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Quaternion::y");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Quaternion::z");
        }
        else if (startPin.dataIdentifier.front() == LcKfInsGnssErrors::type())
        {
            // NodeData
            _pinData.at(pinIndex).addPlotDataItem(i++, "Time [s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GPS time of week [s]");
            // PVAError
            _pinData.at(pinIndex).addPlotDataItem(i++, "Roll error [deg]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Pitch error [deg]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Yaw error [deg]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "North velocity error [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "East velocity error [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Down velocity error [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Latitude error [deg]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Longitude error [deg]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Altitude error [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Alpha_eb [deg]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Beta_eb [deg]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Gamma_eb [deg]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "ECEF X velocity error [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "ECEF Y velocity error [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "ECEF Z velocity error [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "ECEF X error [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "ECEF Y error [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "ECEF Z error [m]");
            // ImuBiases
            _pinData.at(pinIndex).addPlotDataItem(i++, "Accelerometer bias b_X [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Accelerometer bias b_Y [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Accelerometer bias b_Z [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Gyroscope bias b_X [rad/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Gyroscope bias b_Y [rad/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Gyroscope bias b_Z [rad/s]");
        }
        else if (startPin.dataIdentifier.front() == TcKfInsGnssErrors::type())
        {
            // NodeData
            _pinData.at(pinIndex).addPlotDataItem(i++, "Time [s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GPS time of week [s]");
            // PVAError
            _pinData.at(pinIndex).addPlotDataItem(i++, "Roll error [deg]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Pitch error [deg]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Yaw error [deg]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "North velocity error [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "East velocity error [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Down velocity error [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Latitude error [deg]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Longitude error [deg]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Altitude error [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Alpha_eb [deg]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Beta_eb [deg]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Gamma_eb [deg]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "ECEF X velocity error [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "ECEF Y velocity error [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "ECEF Z velocity error [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "ECEF X error [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "ECEF Y error [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "ECEF Z error [m]");
            // ImuBiases
            _pinData.at(pinIndex).addPlotDataItem(i++, "Accelerometer bias b_X [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Accelerometer bias b_Y [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Accelerometer bias b_Z [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Gyroscope bias b_X [rad/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Gyroscope bias b_Y [rad/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Gyroscope bias b_Z [rad/s]");
            // GnssErrors
            _pinData.at(pinIndex).addPlotDataItem(i++, "Receiver clock offset [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Receiver clock drift [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Receiver clock offset [s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Receiver clock drift [s/s]");
        }
        else if (startPin.dataIdentifier.front() == GnssCombination::type())
        {
            // NodeData
            _pinData.at(pinIndex).addPlotDataItem(i++, "Time [s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GPS time of week [s]");
            // GnssCombination
            _pinData.at(pinIndex).dynamicDataStartIndex = static_cast<int>(i);
        }
        else if (startPin.dataIdentifier.front() == GnssObs::type())
        {
            // NodeData
            _pinData.at(pinIndex).addPlotDataItem(i++, "Time [s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GPS time of week [s]");
            // GnssObs
            _pinData.at(pinIndex).dynamicDataStartIndex = static_cast<int>(i);
        }
        else if (startPin.dataIdentifier.front() == SppSolution::type())
        {
            // NodeData
            _pinData.at(pinIndex).addPlotDataItem(i++, "Time [s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GPS time of week [s]");
            // PosVel
            _pinData.at(pinIndex).addPlotDataItem(i++, "Latitude [deg]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Longitude [deg]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Altitude [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "North/South [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "East/West [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "X-ECEF [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Y-ECEF [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Z-ECEF [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Velocity norm [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "X velocity ECEF [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Y velocity ECEF [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Z velocity ECEF [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "North velocity [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "East velocity [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Down velocity [m/s]");
            // SppSolution
            _pinData.at(pinIndex).addPlotDataItem(i++, "Number satellites");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Receiver clock bias [s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Receiver clock drift [s/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "X-ECEF StDev [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Y-ECEF StDev [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Z-ECEF StDev [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "XY-ECEF StDev [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "XZ-ECEF StDev [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "YZ-ECEF StDev [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "North StDev [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "East StDev [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Down StDev [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "NE StDev [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "ND StDev [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "ED StDev [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "X velocity ECEF StDev [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Y velocity ECEF StDev [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Z velocity ECEF StDev [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "XY velocity StDev [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "XZ velocity StDev [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "YZ velocity StDev [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "North velocity StDev [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "East velocity StDev [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Down velocity StDev [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "NE velocity StDev [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "ND velocity StDev [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "ED velocity StDev [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Receiver clock bias StDev [s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Receiver clock drift StDev [s/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "System time reference system");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GPS system time difference [s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GAL system time difference [s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GLO system time difference [s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "BDS system time difference [s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "QZSS system time difference [s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "IRNSS system time difference [s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "SBAS system time difference [s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GPS system time drift difference [s/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GAL system time drift difference [s/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GLO system time drift difference [s/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "BDS system time drift difference [s/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "QZSS system time drift difference [s/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "IRNSS system time drift difference [s/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "SBAS system time drift difference [s/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GPS system time difference StDev [s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GAL system time difference StDev [s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GLO system time difference StDev [s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "BDS system time difference StDev [s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "QZSS system time difference StDev [s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "IRNSS system time difference StDev [s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "SBAS system time difference StDev [s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GPS system time drift difference StDev [s/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GAL system time drift difference StDev [s/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GLO system time drift difference StDev [s/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "BDS system time drift difference StDev [s/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "QZSS system time drift difference StDev [s/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "IRNSS system time drift difference StDev [s/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "SBAS system time drift difference StDev [s/s]");
            _pinData.at(pinIndex).dynamicDataStartIndex = static_cast<int>(i);
        }
        else if (startPin.dataIdentifier.front() == RtklibPosObs::type())
        {
            // NodeData
            _pinData.at(pinIndex).addPlotDataItem(i++, "Time [s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GPS time of week [s]");
            // RtklibPosObs
            _pinData.at(pinIndex).addPlotDataItem(i++, "X-ECEF [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Y-ECEF [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Z-ECEF [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Latitude [deg]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Longitude [deg]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Altitude [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "North/South [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "East/West [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Q [-]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "ns [-]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "sdx [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "sdy [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "sdz [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "sdn [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "sde [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "sdd [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "sdxy [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "sdyz [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "sdzx [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "sdne [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "sded [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "sddn [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "age [s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "ratio [-]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Velocity norm [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Velocity ECEF X [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Velocity ECEF Y [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Velocity ECEF Z [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Velocity North [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Velocity East [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Velocity Down [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "sdvn [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "sdve [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "sdvd [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "sdvne [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "sdved [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "sdvdn [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "sdvx [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "sdvy [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "sdvz [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "sdvxy [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "sdvyz [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "sdvzx [m/s]");
        }
        else if (startPin.dataIdentifier.front() == UbloxObs::type())
        {
            // NodeData
            _pinData.at(pinIndex).addPlotDataItem(i++, "Time [s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GPS time of week [s]");
            // UbloxObs
            _pinData.at(pinIndex).addPlotDataItem(i++, "X-ECEF [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Y-ECEF [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Z-ECEF [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Latitude [deg]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Longitude [deg]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Altitude [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "North/South [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "East/West [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Velocity norm [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Velocity N [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Velocity E [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Velocity D [m/s]");
        }
        else if (startPin.dataIdentifier.front() == ImuObs::type())
        {
            // NodeData
            _pinData.at(pinIndex).addPlotDataItem(i++, "Time [s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GPS time of week [s]");
            // ImuObs
            _pinData.at(pinIndex).addPlotDataItem(i++, "Time since startup [ns]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Mag uncomp X [Gauss]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Mag uncomp Y [Gauss]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Mag uncomp Z [Gauss]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Accel uncomp X [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Accel uncomp Y [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Accel uncomp Z [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Gyro uncomp X [rad/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Gyro uncomp Y [rad/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Gyro uncomp Z [rad/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Mag Comp X [Gauss]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Mag Comp Y [Gauss]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Mag Comp Z [Gauss]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Accel Comp X [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Accel Comp Y [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Accel Comp Z [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Gyro Comp X [rad/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Gyro Comp Y [rad/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Gyro Comp Z [rad/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Temperature [°C]");
        }
        else if (startPin.dataIdentifier.front() == ImuObsSimulated::type())
        {
            // NodeData
            _pinData.at(pinIndex).addPlotDataItem(i++, "Time [s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GPS time of week [s]");
            // ImuObs
            _pinData.at(pinIndex).addPlotDataItem(i++, "Time since startup [ns]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Mag uncomp X [Gauss]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Mag uncomp Y [Gauss]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Mag uncomp Z [Gauss]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Accel uncomp X [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Accel uncomp Y [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Accel uncomp Z [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Gyro uncomp X [rad/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Gyro uncomp Y [rad/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Gyro uncomp Z [rad/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Mag Comp X [Gauss]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Mag Comp Y [Gauss]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Mag Comp Z [Gauss]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Accel Comp X [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Accel Comp Y [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Accel Comp Z [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Gyro Comp X [rad/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Gyro Comp Y [rad/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Gyro Comp Z [rad/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Temperature [°C]");
            // ImuObsSimulated
            _pinData.at(pinIndex).addPlotDataItem(i++, "AccelDynamicsN [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "AccelDynamicsE [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "AccelDynamicsD [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "AngularRateN (ω_nb_n) [rad/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "AngularRateE (ω_nb_n) [rad/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "AngularRateD (ω_nb_n) [rad/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "AccelDynamicsX ECEF [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "AccelDynamicsY ECEF [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "AccelDynamicsZ ECEF [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "AngularRateX ECEF (ω_nb_e) [rad/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "AngularRateY ECEF (ω_nb_e) [rad/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "AngularRateZ ECEF (ω_nb_e) [rad/s]");
        }
        else if (startPin.dataIdentifier.front() == KvhObs::type())
        {
            // NodeData
            _pinData.at(pinIndex).addPlotDataItem(i++, "Time [s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GPS time of week [s]");
            // ImuObs
            _pinData.at(pinIndex).addPlotDataItem(i++, "Time since startup [ns]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Mag uncomp X [Gauss]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Mag uncomp Y [Gauss]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Mag uncomp Z [Gauss]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Accel uncomp X [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Accel uncomp Y [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Accel uncomp Z [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Gyro uncomp X [rad/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Gyro uncomp Y [rad/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Gyro uncomp Z [rad/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Mag Comp X [Gauss]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Mag Comp Y [Gauss]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Mag Comp Z [Gauss]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Accel Comp X [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Accel Comp Y [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Accel Comp Z [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Gyro Comp X [rad/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Gyro Comp Y [rad/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Gyro Comp Z [rad/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Temperature [°C]");
            // KvhObs
            _pinData.at(pinIndex).addPlotDataItem(i++, "Status [bits]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Sequence Number [.]");
        }
        else if (startPin.dataIdentifier.front() == ImuObsWDelta::type())
        {
            // NodeData
            _pinData.at(pinIndex).addPlotDataItem(i++, "Time [s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GPS time of week [s]");
            // ImuObs
            _pinData.at(pinIndex).addPlotDataItem(i++, "Time since startup [ns]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Mag uncomp X [Gauss]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Mag uncomp Y [Gauss]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Mag uncomp Z [Gauss]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Accel uncomp X [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Accel uncomp Y [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Accel uncomp Z [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Gyro uncomp X [rad/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Gyro uncomp Y [rad/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Gyro uncomp Z [rad/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Mag Comp X [Gauss]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Mag Comp Y [Gauss]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Mag Comp Z [Gauss]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Accel Comp X [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Accel Comp Y [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Accel Comp Z [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Gyro Comp X [rad/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Gyro Comp Y [rad/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Gyro Comp Z [rad/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Temperature [°C]");
            // ImuObsWDelta
            _pinData.at(pinIndex).addPlotDataItem(i++, "dTime [s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "dTheta X [deg]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "dTheta Y [deg]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "dTheta Z [deg]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "dVelocity X [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "dVelocity Y [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "dVelocity Z [m/s]");
        }
        else if (startPin.dataIdentifier.front() == VectorNavBinaryOutput::type())
        {
            // NodeData
            _pinData.at(pinIndex).addPlotDataItem(i++, "Time [s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GPS time of week [s]");
            // VectorNavBinaryOutput
            // Group 2 (Time)
            _pinData.at(pinIndex).addPlotDataItem(i++, "Time::TimeStartup [ns]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Time::TimeGps [ns]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Time::GpsTow [ns]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Time::GpsWeek");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Time::TimeSyncIn [ns]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Time::TimeGpsPps [ns]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Time::TimeUTC::year");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Time::TimeUTC::month");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Time::TimeUTC::day");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Time::TimeUTC::hour");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Time::TimeUTC::min");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Time::TimeUTC::sec");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Time::TimeUTC::ms");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Time::SyncInCnt");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Time::SyncOutCnt");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Time::TimeStatus::timeOk");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Time::TimeStatus::dateOk");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Time::TimeStatus::utcTimeValid");
            // Group 3 (IMU)
            _pinData.at(pinIndex).addPlotDataItem(i++, "IMU::ImuStatus");
            _pinData.at(pinIndex).addPlotDataItem(i++, "IMU::UncompMag::X [Gauss]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "IMU::UncompMag::Y [Gauss]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "IMU::UncompMag::Z [Gauss]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "IMU::UncompAccel::X [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "IMU::UncompAccel::Y [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "IMU::UncompAccel::Z [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "IMU::UncompGyro::X [rad/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "IMU::UncompGyro::Y [rad/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "IMU::UncompGyro::Z [rad/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "IMU::Temp [Celsius]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "IMU::Pres [kPa]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "IMU::DeltaTime [s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "IMU::DeltaTheta::X [deg]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "IMU::DeltaTheta::Y [deg]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "IMU::DeltaTheta::Z [deg]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "IMU::DeltaVel::X [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "IMU::DeltaVel::Y [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "IMU::DeltaVel::Z [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "IMU::Mag::X [Gauss]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "IMU::Mag::Y [Gauss]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "IMU::Mag::Z [Gauss]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "IMU::Accel::X [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "IMU::Accel::Y [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "IMU::Accel::Z [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "IMU::AngularRate::X [rad/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "IMU::AngularRate::Y [rad/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "IMU::AngularRate::Z [rad/s]");
            // Group 4 (GNSS1)
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS1::UTC::year");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS1::UTC::month");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS1::UTC::day");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS1::UTC::hour");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS1::UTC::min");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS1::UTC::sec");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS1::UTC::ms");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS1::Tow [ns]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS1::Week");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS1::NumSats");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS1::Fix");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS1::PosLla::latitude [deg]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS1::PosLla::longitude [deg]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS1::PosLla::altitude [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS1::PosEcef::X [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS1::PosEcef::Y [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS1::PosEcef::Z [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS1::VelNed::N [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS1::VelNed::E [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS1::VelNed::D [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS1::VelEcef::X [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS1::VelEcef::Y [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS1::VelEcef::Z [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS1::PosU::N [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS1::PosU::E [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS1::PosU::D [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS1::VelU [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS1::TimeU [s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS1::TimeInfo::Status::timeOk");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS1::TimeInfo::Status::dateOk");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS1::TimeInfo::Status::utcTimeValid");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS1::TimeInfo::LeapSeconds");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS1::DOP::g");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS1::DOP::p");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS1::DOP::t");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS1::DOP::v");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS1::DOP::h");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS1::DOP::n");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS1::DOP::e");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS1::SatInfo::NumSats");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS1::RawMeas::Tow [s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS1::RawMeas::Week");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS1::RawMeas::NumSats");
            // Group 5 (Attitude)
            _pinData.at(pinIndex).addPlotDataItem(i++, "Att::VpeStatus::AttitudeQuality");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Att::VpeStatus::GyroSaturation");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Att::VpeStatus::GyroSaturationRecovery");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Att::VpeStatus::MagDisturbance");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Att::VpeStatus::MagSaturation");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Att::VpeStatus::AccDisturbance");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Att::VpeStatus::AccSaturation");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Att::VpeStatus::KnownMagDisturbance");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Att::VpeStatus::KnownAccelDisturbance");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Att::YawPitchRoll::Y [deg]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Att::YawPitchRoll::P [deg]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Att::YawPitchRoll::R [deg]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Att::Quaternion::w");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Att::Quaternion::x");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Att::Quaternion::y");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Att::Quaternion::z");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Att::DCM::0-0");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Att::DCM::0-1");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Att::DCM::0-2");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Att::DCM::1-0");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Att::DCM::1-1");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Att::DCM::1-2");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Att::DCM::2-0");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Att::DCM::2-1");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Att::DCM::2-2");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Att::MagNed::N [Gauss]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Att::MagNed::E [Gauss]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Att::MagNed::D [Gauss]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Att::AccelNed::N [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Att::AccelNed::E [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Att::AccelNed::D [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Att::LinearAccelBody::X [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Att::LinearAccelBody::Y [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Att::LinearAccelBody::Z [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Att::LinearAccelNed::N [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Att::LinearAccelNed::E [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Att::LinearAccelNed::D [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Att::YprU::Y [deg]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Att::YprU::P [deg]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "Att::YprU::R [deg]");
            // Group 6 (INS)
            _pinData.at(pinIndex).addPlotDataItem(i++, "INS::InsStatus::Mode");
            _pinData.at(pinIndex).addPlotDataItem(i++, "INS::InsStatus::GpsFix");
            _pinData.at(pinIndex).addPlotDataItem(i++, "INS::InsStatus::Error::IMU");
            _pinData.at(pinIndex).addPlotDataItem(i++, "INS::InsStatus::Error::MagPres");
            _pinData.at(pinIndex).addPlotDataItem(i++, "INS::InsStatus::Error::GNSS");
            _pinData.at(pinIndex).addPlotDataItem(i++, "INS::InsStatus::GpsHeadingIns");
            _pinData.at(pinIndex).addPlotDataItem(i++, "INS::InsStatus::GpsCompass");
            _pinData.at(pinIndex).addPlotDataItem(i++, "INS::PosLla::latitude [deg]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "INS::PosLla::longitude [deg]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "INS::PosLla::altitude [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "INS::PosEcef::X [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "INS::PosEcef::Y [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "INS::PosEcef::Z [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "INS::VelBody::X [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "INS::VelBody::Y [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "INS::VelBody::Z [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "INS::VelNed::N [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "INS::VelNed::E [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "INS::VelNed::D [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "INS::VelEcef::X [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "INS::VelEcef::Y [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "INS::VelEcef::Z [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "INS::MagEcef::X [Gauss}");
            _pinData.at(pinIndex).addPlotDataItem(i++, "INS::MagEcef::Y [Gauss}");
            _pinData.at(pinIndex).addPlotDataItem(i++, "INS::MagEcef::Z [Gauss}");
            _pinData.at(pinIndex).addPlotDataItem(i++, "INS::AccelEcef::X [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "INS::AccelEcef::Y [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "INS::AccelEcef::Z [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "INS::LinearAccelEcef::X [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "INS::LinearAccelEcef::Y [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "INS::LinearAccelEcef::Z [m/s^2]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "INS::PosU [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "INS::VelU [m/s]");
            // Group 7 (GNSS2)
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS2::UTC::year");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS2::UTC::month");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS2::UTC::day");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS2::UTC::hour");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS2::UTC::min");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS2::UTC::sec");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS2::UTC::ms");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS2::Tow [ns]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS2::Week");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS2::NumSats");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS2::Fix");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS2::PosLla::latitude [deg]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS2::PosLla::longitude [deg]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS2::PosLla::altitude [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS2::PosEcef::X [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS2::PosEcef::Y [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS2::PosEcef::Z [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS2::VelNed::N [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS2::VelNed::E [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS2::VelNed::D [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS2::VelEcef::X [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS2::VelEcef::Y [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS2::VelEcef::Z [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS2::PosU::N [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS2::PosU::E [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS2::PosU::D [m]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS2::VelU [m/s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS2::TimeU [s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS2::TimeInfo::Status::timeOk");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS2::TimeInfo::Status::dateOk");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS2::TimeInfo::Status::utcTimeValid");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS2::TimeInfo::LeapSeconds");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS2::DOP::g");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS2::DOP::p");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS2::DOP::t");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS2::DOP::v");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS2::DOP::h");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS2::DOP::n");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS2::DOP::e");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS2::SatInfo::NumSats");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS2::RawMeas::Tow [s]");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS2::RawMeas::Week");
            _pinData.at(pinIndex).addPlotDataItem(i++, "GNSS2::RawMeas::NumSats");
        }
    }
    else if (inputPins.at(pinIndex).type == Pin::Type::Bool)
    {
        _pinData.at(pinIndex).dataIdentifier = startPin.name;

        // NodeData
        _pinData.at(pinIndex).addPlotDataItem(i++, "Time [s]");
        _pinData.at(pinIndex).addPlotDataItem(i++, "GPS time of week [s]");
        // Bool
        _pinData.at(pinIndex).addPlotDataItem(i++, "Boolean");
    }
    else if (inputPins.at(pinIndex).type == Pin::Type::Int)
    {
        _pinData.at(pinIndex).dataIdentifier = startPin.name;

        // NodeData
        _pinData.at(pinIndex).addPlotDataItem(i++, "Time [s]");
        _pinData.at(pinIndex).addPlotDataItem(i++, "GPS time of week [s]");
        // Int
        _pinData.at(pinIndex).addPlotDataItem(i++, "Integer");
    }
    else if (inputPins.at(pinIndex).type == Pin::Type::Float)
    {
        _pinData.at(pinIndex).dataIdentifier = startPin.name;

        // NodeData
        _pinData.at(pinIndex).addPlotDataItem(i++, "Time [s]");
        _pinData.at(pinIndex).addPlotDataItem(i++, "GPS time of week [s]");
        // Float
        _pinData.at(pinIndex).addPlotDataItem(i++, "Float");
    }
    else if (inputPins.at(pinIndex).type == Pin::Type::Matrix)
    {
        _pinData.at(pinIndex).dataIdentifier = startPin.name;

        // NodeData
        _pinData.at(pinIndex).addPlotDataItem(i++, "Time [s]");
        _pinData.at(pinIndex).addPlotDataItem(i++, "GPS time of week [s]");
        // Matrix
        if (startPin.dataIdentifier.front() == "Eigen::MatrixXd")
        {
            if (const auto* matrix = getInputValue<const Eigen::MatrixXd>(pinIndex))
            {
                auto* mutex = getInputValueMutex(pinIndex);
                if (mutex) { mutex->lock(); }
                for (int row = 0; row < matrix->rows(); row++)
                {
                    for (int col = 0; col < matrix->cols(); col++)
                    {
                        _pinData.at(pinIndex).addPlotDataItem(i++, fmt::format("{}, {}", row, col));
                    }
                }
                if (mutex) { mutex->unlock(); }
            }
        }
        else if (startPin.dataIdentifier.front() == "Eigen::VectorXd")
        {
            if (const auto* matrix = getInputValue<const Eigen::VectorXd>(pinIndex))
            {
                auto* mutex = getInputValueMutex(pinIndex);
                if (mutex) { mutex->lock(); }
                for (int row = 0; row < matrix->rows(); row++)
                {
                    _pinData.at(pinIndex).addPlotDataItem(i++, fmt::format("{}", row));
                }
                if (mutex) { mutex->unlock(); }
            }
        }

        for (size_t dataIndex = i; dataIndex < _pinData.at(pinIndex).plotData.size(); dataIndex++)
        {
            const auto& displayName = _pinData.at(pinIndex).plotData.at(dataIndex).displayName;
            for (auto& plot : _plots)
            {
                auto plotItemIter = std::find(plot.plotItems.begin(), plot.plotItems.end(), PlotInfo::PlotItem{ pinIndex, dataIndex, displayName });
                if (plotItemIter != plot.plotItems.end())
                {
                    plot.plotItems.erase(plotItemIter);
                }
            }
        }
    }

    for (size_t dataIndex = 0; dataIndex < _pinData.at(pinIndex).plotData.size(); ++dataIndex)
    {
        auto iter = _pinData.at(pinIndex).plotData.begin();
        std::advance(iter, dataIndex);
        if (iter->markedForDelete)
        {
            _pinData.at(pinIndex).plotData.erase(iter);
            --dataIndex;
        }
    }

    for (auto& plot : _plots)
    {
        if (plot.selectedXdata.at(pinIndex) > _pinData.at(pinIndex).plotData.size())
        {
            plot.selectedXdata.at(pinIndex) = 0;
        }
    }
}

void NAV::Plot::updateNumberOfPlots()
{
    while (_nPlots > _plots.size())
    {
        _plots.emplace_back(fmt::format("Plot {}", _plots.size() + 1), inputPins.size());
    }
    while (_nPlots < _plots.size())
    {
        _plots.pop_back();
    }
}

void NAV::Plot::pinAddCallback(Node* node)
{
    auto* plotNode = static_cast<Plot*>(node); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)

    nm::CreateInputPin(node, fmt::format("Pin {}", node->inputPins.size() + 1).c_str(), Pin::Type::Flow, plotNode->_dataIdentifier, &Plot::plotData);
    plotNode->_pinData.emplace_back();
    for (auto& plot : plotNode->_plots)
    {
        plot.selectedXdata.emplace_back(0);
    }
}

void NAV::Plot::pinDeleteCallback(Node* node, size_t pinIdx)
{
    auto* plotNode = static_cast<Plot*>(node); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)

    for (auto& plot : plotNode->_plots)
    {
        if (plot.selectedPin >= pinIdx)
        {
            plot.selectedPin--;
        }
        for (size_t plotItemIdx = 0; plotItemIdx < plot.plotItems.size(); ++plotItemIdx)
        {
            auto& plotItem = plot.plotItems.at(plotItemIdx);

            if (plotItem.pinIndex == pinIdx) // The index we want to delete
            {
                plot.plotItems.erase(plot.plotItems.begin() + static_cast<int64_t>(plotItemIdx));
                --plotItemIdx;
            }
            else if (plotItem.pinIndex > pinIdx) // Index higher -> Decrement
            {
                --(plotItem.pinIndex);
            }
        }

        plot.selectedXdata.erase(std::next(plot.selectedXdata.begin(), static_cast<int64_t>(pinIdx)));
    }

    nm::DeleteInputPin(node->inputPins.at(pinIdx));
    plotNode->_pinData.erase(std::next(plotNode->_pinData.begin(), static_cast<int64_t>(pinIdx)));
}

void NAV::Plot::addEvent(size_t pinIndex, InsTime insTime, const std::string& text)
{
    if (!insTime.empty() && !_startTime.empty())
    {
        double relTime = static_cast<double>((insTime - _startTime).count());
        _pinData.at(pinIndex).events.emplace_back(relTime, insTime, text);
    }
}

void NAV::Plot::addData(size_t pinIndex, size_t dataIndex, double value)
{
    auto& plotData = _pinData.at(pinIndex).plotData.at(dataIndex);

    plotData.buffer.push_back(value);
    if (!std::isnan(value))
    {
        plotData.hasData = true;
    }
}

void NAV::Plot::addData(size_t pinIndex, std::string displayName, double value)
{
    auto& pinData = _pinData.at(pinIndex);

    auto plotData = std::find_if(pinData.plotData.begin(), pinData.plotData.end(), [&](const auto& data) {
        return data.displayName == displayName;
    });
    if (plotData == pinData.plotData.end()) // Item is new
    {
        pinData.addPlotDataItem(pinData.plotData.size(), displayName);
        plotData = pinData.plotData.end() - 1;
        plotData->isDynamic = true;

        // We assume, there is a static item at the front (the time)
        for (size_t i = plotData->buffer.size(); i < pinData.plotData.front().buffer.size() - 1; i++) // Add empty NaN values to shift it to the correct start point
        {
            plotData->buffer.push_back(std::nan(""));
        }
    }
    else
    {
        // Item was there, but it could have been missing and came again
        for (size_t i = plotData->buffer.size(); i < pinData.plotData.front().buffer.size() - 1; i++) // Add empty NaN values to shift it to the correct start point
        {
            plotData->buffer.push_back(std::nan(""));
        }
    }

    addData(pinIndex, static_cast<size_t>(plotData - pinData.plotData.begin()), value);
}

void NAV::Plot::plotBoolean(const InsTime& insTime, size_t pinIdx)
{
    LOG_DATA("{}: Plotting boolean on pin '{}' with time {}", nameId(), inputPins[pinIdx].name, insTime.toYMDHMS());
    if (ConfigManager::Get<bool>("nogui"))
    {
        releaseInputValue(pinIdx);
        return;
    }

    const auto* value = getInputValue<const bool>(pinIdx);

    if (value != nullptr && !insTime.empty())
    {
        if (_startTime.empty()) { _startTime = insTime; }
        size_t i = 0;

        std::scoped_lock<std::mutex> guard(_pinData.at(pinIdx).mutex);

        // NodeData
        addData(pinIdx, i++, static_cast<double>((insTime - _startTime).count()));
        addData(pinIdx, i++, static_cast<double>(insTime.toGPSweekTow().tow));
        // Boolean
        addData(pinIdx, i++, static_cast<double>(*value));
    }
    releaseInputValue(pinIdx);
}

void NAV::Plot::plotInteger(const InsTime& insTime, size_t pinIdx)
{
    LOG_DATA("{}: Plotting integer on pin '{}' with time {}", nameId(), inputPins[pinIdx].name, insTime.toYMDHMS());
    if (ConfigManager::Get<bool>("nogui"))
    {
        releaseInputValue(pinIdx);
        return;
    }

    const auto* value = getInputValue<const int>(pinIdx);

    if (value != nullptr && !insTime.empty())
    {
        if (_startTime.empty()) { _startTime = insTime; }
        size_t i = 0;

        std::scoped_lock<std::mutex> guard(_pinData.at(pinIdx).mutex);

        // NodeData
        addData(pinIdx, i++, static_cast<double>((insTime - _startTime).count()));
        addData(pinIdx, i++, static_cast<double>(insTime.toGPSweekTow().tow));
        // Integer
        addData(pinIdx, i++, static_cast<double>(*value));
    }
    releaseInputValue(pinIdx);
}

void NAV::Plot::plotFloat(const InsTime& insTime, size_t pinIdx)
{
    LOG_DATA("{}: Plotting float on pin '{}' with time {}", nameId(), inputPins[pinIdx].name, insTime.toYMDHMS());
    if (ConfigManager::Get<bool>("nogui"))
    {
        releaseInputValue(pinIdx);
        return;
    }

    const auto* value = getInputValue<const double>(pinIdx);

    if (value != nullptr && !insTime.empty())
    {
        if (_startTime.empty()) { _startTime = insTime; }
        size_t i = 0;

        std::scoped_lock<std::mutex> guard(_pinData.at(pinIdx).mutex);

        // NodeData
        addData(pinIdx, i++, static_cast<double>((insTime - _startTime).count()));
        addData(pinIdx, i++, static_cast<double>(insTime.toGPSweekTow().tow));
        // Double
        addData(pinIdx, i++, *value);
    }
    releaseInputValue(pinIdx);
}

void NAV::Plot::plotMatrix(const InsTime& insTime, size_t pinIdx)
{
    LOG_DATA("{}: Plotting matrix on pin '{}' with time {}", nameId(), inputPins[pinIdx].name, insTime.toYMDHMS());
    if (ConfigManager::Get<bool>("nogui"))
    {
        releaseInputValue(pinIdx);
        return;
    }

    if (auto* sourcePin = inputPins[pinIdx].link.getConnectedPin())
    {
        if (sourcePin->dataIdentifier.front() == "Eigen::MatrixXd")
        {
            const auto* value = getInputValue<const Eigen::MatrixXd>(pinIdx);

            if (value != nullptr && !insTime.empty())
            {
                if (_startTime.empty()) { _startTime = insTime; }
                size_t i = 0;

                std::scoped_lock<std::mutex> guard(_pinData.at(pinIdx).mutex);

                // NodeData
                addData(pinIdx, i++, static_cast<double>((insTime - _startTime).count()));
                addData(pinIdx, i++, static_cast<double>(insTime.toGPSweekTow().tow));
                // Matrix
                for (int row = 0; row < value->rows(); row++)
                {
                    for (int col = 0; col < value->cols(); col++)
                    {
                        addData(pinIdx, i++, (*value)(row, col));
                    }
                }
            }
        }
        else if (sourcePin->dataIdentifier.front() == "Eigen::VectorXd")
        {
            const auto* value = getInputValue<const Eigen::VectorXd>(pinIdx);

            if (value != nullptr && !insTime.empty())
            {
                if (_startTime.empty()) { _startTime = insTime; }
                size_t i = 0;

                std::scoped_lock<std::mutex> guard(_pinData.at(pinIdx).mutex);

                // NodeData
                addData(pinIdx, i++, static_cast<double>((insTime - _startTime).count()));
                addData(pinIdx, i++, static_cast<double>(insTime.toGPSweekTow().tow));
                // Vector
                for (int row = 0; row < value->rows(); row++)
                {
                    addData(pinIdx, i++, (*value)(row));
                }
            }
        }
    }
    releaseInputValue(pinIdx);
}

void NAV::Plot::plotData(NAV::InputPin::NodeDataQueue& queue, size_t pinIdx)
{
    auto nodeData = queue.extract_front();

    if (ConfigManager::Get<bool>("nogui")) { return; }

    LOG_DATA("{}: Plotting data on pin '{}' with time {}", nameId(), inputPins[pinIdx].name, nodeData->insTime.toYMDHMS());

    if (auto* sourcePin = inputPins[pinIdx].link.getConnectedPin())
    {
        if (sourcePin->dataIdentifier.front() == Pos::type())
        {
            plotPos(std::static_pointer_cast<const Pos>(nodeData), pinIdx);
        }
        else if (sourcePin->dataIdentifier.front() == PosVel::type())
        {
            plotPosVel(std::static_pointer_cast<const PosVel>(nodeData), pinIdx);
        }
        else if (sourcePin->dataIdentifier.front() == PosVelAtt::type()
                 || sourcePin->dataIdentifier.front() == InertialNavSol::type())
        {
            plotPosVelAtt(std::static_pointer_cast<const PosVelAtt>(nodeData), pinIdx);
        }
        else if (sourcePin->dataIdentifier.front() == LcKfInsGnssErrors::type())
        {
            plotLcKfInsGnssErrors(std::static_pointer_cast<const LcKfInsGnssErrors>(nodeData), pinIdx);
        }
        else if (sourcePin->dataIdentifier.front() == TcKfInsGnssErrors::type())
        {
            plotTcKfInsGnssErrors(std::static_pointer_cast<const TcKfInsGnssErrors>(nodeData), pinIdx);
        }
        else if (sourcePin->dataIdentifier.front() == GnssCombination::type())
        {
            plotGnssCombination(std::static_pointer_cast<const GnssCombination>(nodeData), pinIdx);
        }
        else if (sourcePin->dataIdentifier.front() == GnssObs::type())
        {
            plotGnssObs(std::static_pointer_cast<const GnssObs>(nodeData), pinIdx);
        }
        else if (sourcePin->dataIdentifier.front() == SppSolution::type())
        {
            plotSppSolution(std::static_pointer_cast<const SppSolution>(nodeData), pinIdx);
        }
        else if (sourcePin->dataIdentifier.front() == RtklibPosObs::type())
        {
            plotRtklibPosObs(std::static_pointer_cast<const RtklibPosObs>(nodeData), pinIdx);
        }
        else if (sourcePin->dataIdentifier.front() == UbloxObs::type())
        {
            plotUbloxObs(std::static_pointer_cast<const UbloxObs>(nodeData), pinIdx);
        }
        else if (sourcePin->dataIdentifier.front() == ImuObs::type())
        {
            plotImuObs(std::static_pointer_cast<const ImuObs>(nodeData), pinIdx);
        }
        else if (sourcePin->dataIdentifier.front() == ImuObsSimulated::type())
        {
            plotImuObsSimulated(std::static_pointer_cast<const ImuObsSimulated>(nodeData), pinIdx);
        }
        else if (sourcePin->dataIdentifier.front() == KvhObs::type())
        {
            plotKvhObs(std::static_pointer_cast<const KvhObs>(nodeData), pinIdx);
        }
        else if (sourcePin->dataIdentifier.front() == ImuObsWDelta::type())
        {
            plotImuObsWDeltaObs(std::static_pointer_cast<const ImuObsWDelta>(nodeData), pinIdx);
        }
        else if (sourcePin->dataIdentifier.front() == VectorNavBinaryOutput::type())
        {
            plotVectorNavBinaryObs(std::static_pointer_cast<const VectorNavBinaryOutput>(nodeData), pinIdx);
        }
    }
}

void NAV::Plot::plotPos(const std::shared_ptr<const Pos>& obs, size_t pinIndex)
{
    if (!obs->insTime.empty() && _startTime.empty()) { _startTime = obs->insTime; }
    size_t i = 0;

    // [𝜙, λ, h] Latitude, Longitude and altitude in [rad, rad, m]
    Eigen::Vector3d lla_position = obs->lla_position();

    if (!_originPosition)
    {
        _originPosition = { .frame = gui::widgets::PositionWithFrame::ReferenceFrame::ECEF,
                            .e_position = obs->e_position() };
    }

    int sign = lla_position.x() > _originPosition->latitude() ? 1 : -1;
    // North/South deviation [m]
    double northSouth = calcGeographicalDistance(lla_position.x(), lla_position.y(),
                                                 _originPosition->latitude(), lla_position.y())
                        * sign;

    sign = lla_position.y() > _originPosition->longitude() ? 1 : -1;
    // East/West deviation [m]
    double eastWest = calcGeographicalDistance(lla_position.x(), lla_position.y(),
                                               lla_position.x(), _originPosition->longitude())
                      * sign;

    std::scoped_lock<std::mutex> guard(_pinData.at(pinIndex).mutex);

    // NodeData
    addData(pinIndex, i++, !obs->insTime.empty() ? static_cast<double>((obs->insTime - _startTime).count()) : std::nan(""));
    addData(pinIndex, i++, !obs->insTime.empty() ? static_cast<double>(obs->insTime.toGPSweekTow().tow) : std::nan(""));
    // Pos
    addData(pinIndex, i++, rad2deg(lla_position(0)));
    addData(pinIndex, i++, rad2deg(lla_position(1)));
    addData(pinIndex, i++, lla_position(2));
    addData(pinIndex, i++, northSouth);
    addData(pinIndex, i++, eastWest);
    addData(pinIndex, i++, obs->e_position().x());
    addData(pinIndex, i++, obs->e_position().y());
    addData(pinIndex, i++, obs->e_position().z());
}

void NAV::Plot::plotPosVel(const std::shared_ptr<const PosVel>& obs, size_t pinIndex)
{
    if (!obs->insTime.empty() && _startTime.empty()) { _startTime = obs->insTime; }
    size_t i = 0;

    // [𝜙, λ, h] Latitude, Longitude and altitude in [rad, rad, m]
    Eigen::Vector3d lla_position = obs->lla_position();

    if (!_originPosition)
    {
        _originPosition = { .frame = gui::widgets::PositionWithFrame::ReferenceFrame::ECEF,
                            .e_position = obs->e_position() };
    }

    int sign = lla_position.x() > _originPosition->latitude() ? 1 : -1;
    // North/South deviation [m]
    double northSouth = calcGeographicalDistance(lla_position.x(), lla_position.y(),
                                                 _originPosition->latitude(), lla_position.y())
                        * sign;

    sign = lla_position.y() > _originPosition->longitude() ? 1 : -1;
    // East/West deviation [m]
    double eastWest = calcGeographicalDistance(lla_position.x(), lla_position.y(),
                                               lla_position.x(), _originPosition->longitude())
                      * sign;

    std::scoped_lock<std::mutex> guard(_pinData.at(pinIndex).mutex);

    // NodeData
    addData(pinIndex, i++, !obs->insTime.empty() ? static_cast<double>((obs->insTime - _startTime).count()) : std::nan(""));
    addData(pinIndex, i++, !obs->insTime.empty() ? static_cast<double>(obs->insTime.toGPSweekTow().tow) : std::nan(""));
    // PosVel
    addData(pinIndex, i++, rad2deg(lla_position(0)));
    addData(pinIndex, i++, rad2deg(lla_position(1)));
    addData(pinIndex, i++, lla_position(2));
    addData(pinIndex, i++, northSouth);
    addData(pinIndex, i++, eastWest);
    addData(pinIndex, i++, obs->e_position().x());
    addData(pinIndex, i++, obs->e_position().y());
    addData(pinIndex, i++, obs->e_position().z());
    addData(pinIndex, i++, obs->e_velocity().norm());
    addData(pinIndex, i++, obs->e_velocity().x());
    addData(pinIndex, i++, obs->e_velocity().y());
    addData(pinIndex, i++, obs->e_velocity().z());
    addData(pinIndex, i++, obs->n_velocity().x());
    addData(pinIndex, i++, obs->n_velocity().y());
    addData(pinIndex, i++, obs->n_velocity().z());
}

void NAV::Plot::plotPosVelAtt(const std::shared_ptr<const PosVelAtt>& obs, size_t pinIndex)
{
    if (!obs->insTime.empty() && _startTime.empty()) { _startTime = obs->insTime; }
    size_t i = 0;

    // [𝜙, λ, h] Latitude, Longitude and altitude in [rad, rad, m]
    Eigen::Vector3d lla_position = obs->lla_position();

    if (!_originPosition)
    {
        _originPosition = { .frame = gui::widgets::PositionWithFrame::ReferenceFrame::ECEF,
                            .e_position = obs->e_position() };
    }

    int sign = lla_position.x() > _originPosition->latitude() ? 1 : -1;
    // North/South deviation [m]
    double northSouth = calcGeographicalDistance(lla_position.x(), lla_position.y(),
                                                 _originPosition->latitude(), lla_position.y())
                        * sign;

    sign = lla_position.y() > _originPosition->longitude() ? 1 : -1;
    // East/West deviation [m]
    double eastWest = calcGeographicalDistance(lla_position.x(), lla_position.y(),
                                               lla_position.x(), _originPosition->longitude())
                      * sign;

    std::scoped_lock<std::mutex> guard(_pinData.at(pinIndex).mutex);

    // NodeData
    addData(pinIndex, i++, !obs->insTime.empty() ? static_cast<double>((obs->insTime - _startTime).count()) : std::nan(""));
    addData(pinIndex, i++, !obs->insTime.empty() ? static_cast<double>(obs->insTime.toGPSweekTow().tow) : std::nan(""));
    // PosVelAtt
    addData(pinIndex, i++, rad2deg(lla_position(0)));
    addData(pinIndex, i++, rad2deg(lla_position(1)));
    addData(pinIndex, i++, lla_position(2));
    addData(pinIndex, i++, northSouth);
    addData(pinIndex, i++, eastWest);
    addData(pinIndex, i++, obs->e_position().x());
    addData(pinIndex, i++, obs->e_position().y());
    addData(pinIndex, i++, obs->e_position().z());
    addData(pinIndex, i++, obs->e_velocity().norm());
    addData(pinIndex, i++, obs->e_velocity().x());
    addData(pinIndex, i++, obs->e_velocity().y());
    addData(pinIndex, i++, obs->e_velocity().z());
    addData(pinIndex, i++, obs->n_velocity().x());
    addData(pinIndex, i++, obs->n_velocity().y());
    addData(pinIndex, i++, obs->n_velocity().z());
    addData(pinIndex, i++, rad2deg(obs->rollPitchYaw().x()));
    addData(pinIndex, i++, rad2deg(obs->rollPitchYaw().y()));
    addData(pinIndex, i++, rad2deg(obs->rollPitchYaw().z()));
    addData(pinIndex, i++, obs->n_Quat_b().w());
    addData(pinIndex, i++, obs->n_Quat_b().x());
    addData(pinIndex, i++, obs->n_Quat_b().y());
    addData(pinIndex, i++, obs->n_Quat_b().z());
}

void NAV::Plot::plotLcKfInsGnssErrors(const std::shared_ptr<const LcKfInsGnssErrors>& obs, size_t pinIndex)
{
    if (!obs->insTime.empty() && _startTime.empty()) { _startTime = obs->insTime; }
    size_t i = 0;

    std::scoped_lock<std::mutex> guard(_pinData.at(pinIndex).mutex);

    // NodeData
    addData(pinIndex, i++, !obs->insTime.empty() ? static_cast<double>((obs->insTime - _startTime).count()) : std::nan(""));
    addData(pinIndex, i++, !obs->insTime.empty() ? static_cast<double>(obs->insTime.toGPSweekTow().tow) : std::nan(""));
    // PVAError
    addData(pinIndex, i++, obs->frame == LcKfInsGnssErrors::Frame::NED ? rad2deg(obs->attitudeError(0)) : std::nan(""));
    addData(pinIndex, i++, obs->frame == LcKfInsGnssErrors::Frame::NED ? rad2deg(obs->attitudeError(1)) : std::nan(""));
    addData(pinIndex, i++, obs->frame == LcKfInsGnssErrors::Frame::NED ? rad2deg(obs->attitudeError(2)) : std::nan(""));
    addData(pinIndex, i++, obs->frame == LcKfInsGnssErrors::Frame::NED ? obs->velocityError(0) : std::nan(""));
    addData(pinIndex, i++, obs->frame == LcKfInsGnssErrors::Frame::NED ? obs->velocityError(1) : std::nan(""));
    addData(pinIndex, i++, obs->frame == LcKfInsGnssErrors::Frame::NED ? obs->velocityError(2) : std::nan(""));
    addData(pinIndex, i++, obs->frame == LcKfInsGnssErrors::Frame::NED ? rad2deg(obs->positionError(0)) : std::nan(""));
    addData(pinIndex, i++, obs->frame == LcKfInsGnssErrors::Frame::NED ? rad2deg(obs->positionError(1)) : std::nan(""));
    addData(pinIndex, i++, obs->frame == LcKfInsGnssErrors::Frame::NED ? obs->positionError(2) : std::nan(""));

    addData(pinIndex, i++, obs->frame == LcKfInsGnssErrors::Frame::ECEF ? rad2deg(obs->attitudeError(0)) : std::nan(""));
    addData(pinIndex, i++, obs->frame == LcKfInsGnssErrors::Frame::ECEF ? rad2deg(obs->attitudeError(1)) : std::nan(""));
    addData(pinIndex, i++, obs->frame == LcKfInsGnssErrors::Frame::ECEF ? rad2deg(obs->attitudeError(2)) : std::nan(""));
    addData(pinIndex, i++, obs->frame == LcKfInsGnssErrors::Frame::ECEF ? obs->velocityError(0) : std::nan(""));
    addData(pinIndex, i++, obs->frame == LcKfInsGnssErrors::Frame::ECEF ? obs->velocityError(1) : std::nan(""));
    addData(pinIndex, i++, obs->frame == LcKfInsGnssErrors::Frame::ECEF ? obs->velocityError(2) : std::nan(""));
    addData(pinIndex, i++, obs->frame == LcKfInsGnssErrors::Frame::ECEF ? obs->positionError(0) : std::nan(""));
    addData(pinIndex, i++, obs->frame == LcKfInsGnssErrors::Frame::ECEF ? obs->positionError(1) : std::nan(""));
    addData(pinIndex, i++, obs->frame == LcKfInsGnssErrors::Frame::ECEF ? obs->positionError(2) : std::nan(""));
    // ImuBiases
    addData(pinIndex, i++, obs->b_biasAccel(0));
    addData(pinIndex, i++, obs->b_biasAccel(1));
    addData(pinIndex, i++, obs->b_biasAccel(2));
    addData(pinIndex, i++, obs->b_biasGyro(0));
    addData(pinIndex, i++, obs->b_biasGyro(1));
    addData(pinIndex, i++, obs->b_biasGyro(2));
}

void NAV::Plot::plotTcKfInsGnssErrors(const std::shared_ptr<const TcKfInsGnssErrors>& obs, size_t pinIndex)
{
    if (!obs->insTime.empty() && _startTime.empty()) { _startTime = obs->insTime; }
    size_t i = 0;

    std::scoped_lock<std::mutex> guard(_pinData.at(pinIndex).mutex);

    // NodeData
    addData(pinIndex, i++, !obs->insTime.empty() ? static_cast<double>((obs->insTime - _startTime).count()) : std::nan(""));
    addData(pinIndex, i++, !obs->insTime.empty() ? static_cast<double>(obs->insTime.toGPSweekTow().tow) : std::nan(""));
    // PVAError
    addData(pinIndex, i++, obs->frame == TcKfInsGnssErrors::Frame::NED ? rad2deg(obs->attitudeError(0)) : std::nan(""));
    addData(pinIndex, i++, obs->frame == TcKfInsGnssErrors::Frame::NED ? rad2deg(obs->attitudeError(1)) : std::nan(""));
    addData(pinIndex, i++, obs->frame == TcKfInsGnssErrors::Frame::NED ? rad2deg(obs->attitudeError(2)) : std::nan(""));
    addData(pinIndex, i++, obs->frame == TcKfInsGnssErrors::Frame::NED ? obs->velocityError(0) : std::nan(""));
    addData(pinIndex, i++, obs->frame == TcKfInsGnssErrors::Frame::NED ? obs->velocityError(1) : std::nan(""));
    addData(pinIndex, i++, obs->frame == TcKfInsGnssErrors::Frame::NED ? obs->velocityError(2) : std::nan(""));
    addData(pinIndex, i++, obs->frame == TcKfInsGnssErrors::Frame::NED ? rad2deg(obs->positionError(0)) : std::nan(""));
    addData(pinIndex, i++, obs->frame == TcKfInsGnssErrors::Frame::NED ? rad2deg(obs->positionError(1)) : std::nan(""));
    addData(pinIndex, i++, obs->frame == TcKfInsGnssErrors::Frame::NED ? obs->positionError(2) : std::nan(""));

    addData(pinIndex, i++, obs->frame == TcKfInsGnssErrors::Frame::ECEF ? rad2deg(obs->attitudeError(0)) : std::nan(""));
    addData(pinIndex, i++, obs->frame == TcKfInsGnssErrors::Frame::ECEF ? rad2deg(obs->attitudeError(1)) : std::nan(""));
    addData(pinIndex, i++, obs->frame == TcKfInsGnssErrors::Frame::ECEF ? rad2deg(obs->attitudeError(2)) : std::nan(""));
    addData(pinIndex, i++, obs->frame == TcKfInsGnssErrors::Frame::ECEF ? obs->velocityError(0) : std::nan(""));
    addData(pinIndex, i++, obs->frame == TcKfInsGnssErrors::Frame::ECEF ? obs->velocityError(1) : std::nan(""));
    addData(pinIndex, i++, obs->frame == TcKfInsGnssErrors::Frame::ECEF ? obs->velocityError(2) : std::nan(""));
    addData(pinIndex, i++, obs->frame == TcKfInsGnssErrors::Frame::ECEF ? obs->positionError(0) : std::nan(""));
    addData(pinIndex, i++, obs->frame == TcKfInsGnssErrors::Frame::ECEF ? obs->positionError(1) : std::nan(""));
    addData(pinIndex, i++, obs->frame == TcKfInsGnssErrors::Frame::ECEF ? obs->positionError(2) : std::nan(""));
    // ImuBiases
    addData(pinIndex, i++, obs->b_biasAccel(0));
    addData(pinIndex, i++, obs->b_biasAccel(1));
    addData(pinIndex, i++, obs->b_biasAccel(2));
    addData(pinIndex, i++, obs->b_biasGyro(0));
    addData(pinIndex, i++, obs->b_biasGyro(1));
    addData(pinIndex, i++, obs->b_biasGyro(2));
    // GnssErrors
    addData(pinIndex, i++, obs->recvClkOffset);               // Receiver clock offset in [m]
    addData(pinIndex, i++, obs->recvClkDrift);                // Receiver clock drift in [m/s]
    addData(pinIndex, i++, obs->recvClkOffset / InsConst::C); // Receiver clock offset in [s]
    addData(pinIndex, i++, obs->recvClkDrift / InsConst::C);  // Receiver clock drift in [s/s]
}

void NAV::Plot::plotGnssCombination(const std::shared_ptr<const GnssCombination>& obs, size_t pinIndex)
{
    if (!obs->insTime.empty() && _startTime.empty()) { _startTime = obs->insTime; }
    size_t i = 0;

    std::scoped_lock<std::mutex> guard(_pinData.at(pinIndex).mutex);

    // NodeData
    addData(pinIndex, i++, !obs->insTime.empty() ? static_cast<double>((obs->insTime - _startTime).count()) : std::nan(""));
    addData(pinIndex, i++, !obs->insTime.empty() ? static_cast<double>(obs->insTime.toGPSweekTow().tow) : std::nan(""));

    // GnssCombination
    for (const auto& comb : obs->combinations)
    {
        addData(pinIndex, comb.description, comb.result.value_or(std::nan("")));
        addData(pinIndex, comb.description + " Cycle Slip", comb.cycleSlipResult ? static_cast<double>(*comb.cycleSlipResult) : std::nan(""));
        addData(pinIndex, comb.description + " Prediction", comb.cycleSlipPrediction.value_or(std::nan("")));
        addData(pinIndex, comb.description + " Meas - Pred", comb.cycleSlipMeasMinPred.value_or(std::nan("")));
    }
    for (const auto& comb : obs->combinations)
    {
        for (const auto& [insTime, poly, value] : comb.cycleSlipPolynomials)
        {
            auto t = static_cast<double>((insTime - _startTime).count());
            addData(pinIndex, fmt::format("{} [{:.1f}] ({})", comb.description, t, poly.toString()), value);
        }
    }
}

void NAV::Plot::plotGnssObs(const std::shared_ptr<const GnssObs>& obs, size_t pinIndex)
{
    if (!obs->insTime.empty() && _startTime.empty()) { _startTime = obs->insTime; }
    size_t i = 0;

    std::scoped_lock<std::mutex> guard(_pinData.at(pinIndex).mutex);

    // NodeData
    addData(pinIndex, i++, !obs->insTime.empty() ? static_cast<double>((obs->insTime - _startTime).count()) : std::nan(""));
    addData(pinIndex, i++, !obs->insTime.empty() ? static_cast<double>(obs->insTime.toGPSweekTow().tow) : std::nan(""));

    // GnssObs
    for (const auto& obsData : obs->data)
    {
        addData(pinIndex, fmt::format("{} Pseudorange [m]", obsData.satSigId), obsData.pseudorange ? obsData.pseudorange->value : std::nan(""));
        addData(pinIndex, fmt::format("{} Pseudorange SSI", obsData.satSigId), obsData.pseudorange ? obsData.pseudorange->SSI : std::nan(""));

        addData(pinIndex, fmt::format("{} Carrier-phase [cycles]", obsData.satSigId), obsData.carrierPhase ? obsData.carrierPhase->value : std::nan(""));
        addData(pinIndex, fmt::format("{} Carrier-phase SSI", obsData.satSigId), obsData.carrierPhase ? obsData.carrierPhase->SSI : std::nan(""));
        addData(pinIndex, fmt::format("{} Carrier-phase LLI", obsData.satSigId), obsData.carrierPhase ? obsData.carrierPhase->LLI : std::nan(""));

        addData(pinIndex, fmt::format("{} Doppler [Hz]", obsData.satSigId), obsData.doppler ? obsData.doppler.value() : std::nan(""));

        addData(pinIndex, fmt::format("{} Carrier-to-Noise density [dBHz]", obsData.satSigId), obsData.CN0 ? obsData.CN0.value() : std::nan(""));
    }
}

void NAV::Plot::plotSppSolution(const std::shared_ptr<const SppSolution>& obs, size_t pinIndex)
{
    namespace States = NAV::GNSS::Positioning::SPP::States;

    if (!obs->insTime.empty() && _startTime.empty()) { _startTime = obs->insTime; }
    size_t i = 0;

    if (!_originPosition)
    {
        _originPosition = { .frame = gui::widgets::PositionWithFrame::ReferenceFrame::ECEF,
                            .e_position = obs->e_position() };
    }

    int sign = obs->lla_position().x() > _originPosition->latitude() ? 1 : -1;
    // North/South deviation [m]
    double northSouth = calcGeographicalDistance(obs->lla_position().x(), obs->lla_position().y(),
                                                 _originPosition->latitude(), obs->lla_position().y())
                        * sign;

    sign = obs->lla_position().y() > _originPosition->longitude() ? 1 : -1;
    // East/West deviation [m]
    double eastWest = calcGeographicalDistance(obs->lla_position().x(), obs->lla_position().y(),
                                               obs->lla_position().x(), _originPosition->longitude())
                      * sign;

    std::scoped_lock<std::mutex> guard(_pinData.at(pinIndex).mutex);

    // NodeData
    addData(pinIndex, i++, !obs->insTime.empty() ? static_cast<double>((obs->insTime - _startTime).count()) : std::nan(""));
    addData(pinIndex, i++, !obs->insTime.empty() ? static_cast<double>(obs->insTime.toGPSweekTow().tow) : std::nan(""));
    // PosVel
    addData(pinIndex, i++, rad2deg(obs->lla_position()(0)));
    addData(pinIndex, i++, rad2deg(obs->lla_position()(1)));
    addData(pinIndex, i++, obs->lla_position()(2));
    addData(pinIndex, i++, northSouth);
    addData(pinIndex, i++, eastWest);
    addData(pinIndex, i++, obs->e_position().x());
    addData(pinIndex, i++, obs->e_position().y());
    addData(pinIndex, i++, obs->e_position().z());
    addData(pinIndex, i++, obs->e_velocity().norm());
    addData(pinIndex, i++, obs->e_velocity().x());
    addData(pinIndex, i++, obs->e_velocity().y());
    addData(pinIndex, i++, obs->e_velocity().z());
    addData(pinIndex, i++, obs->n_velocity().x());
    addData(pinIndex, i++, obs->n_velocity().y());
    addData(pinIndex, i++, obs->n_velocity().z());
    // SppSolution
    addData(pinIndex, i++, static_cast<double>(obs->nSatellites));                                                                                                                 // Number satellites
    addData(pinIndex, i++, obs->recvClk.bias.value);                                                                                                                               // Receiver clock bias [s]
    addData(pinIndex, i++, obs->recvClk.drift.value);                                                                                                                              // Receiver clock drift [s/s]
    addData(pinIndex, i++, obs->e_positionStdev()(0));                                                                                                                             // X-ECEF StDev [m]
    addData(pinIndex, i++, obs->e_positionStdev()(1));                                                                                                                             // Y-ECEF StDev [m]
    addData(pinIndex, i++, obs->e_positionStdev()(2));                                                                                                                             // Z-ECEF StDev [m]
    addData(pinIndex, i++, obs->e_CovarianceMatrix().rows() >= 1 && obs->e_CovarianceMatrix().cols() >= 2 ? obs->e_CovarianceMatrix()(States::PosX, States::PosY) : std::nan("")); // XY-ECEF StDev [m]
    addData(pinIndex, i++, obs->e_CovarianceMatrix().rows() >= 1 && obs->e_CovarianceMatrix().cols() >= 3 ? obs->e_CovarianceMatrix()(States::PosX, States::PosZ) : std::nan("")); // XZ-ECEF StDev [m]
    addData(pinIndex, i++, obs->e_CovarianceMatrix().rows() >= 2 && obs->e_CovarianceMatrix().cols() >= 3 ? obs->e_CovarianceMatrix()(States::PosY, States::PosZ) : std::nan("")); // YZ-ECEF StDev [m]
    addData(pinIndex, i++, obs->n_positionStdev()(0));                                                                                                                             // North StDev [m]
    addData(pinIndex, i++, obs->n_positionStdev()(1));                                                                                                                             // East StDev [m]
    addData(pinIndex, i++, obs->n_positionStdev()(2));                                                                                                                             // Down StDev [m]
    addData(pinIndex, i++, obs->n_CovarianceMatrix().rows() >= 1 && obs->n_CovarianceMatrix().cols() >= 2 ? obs->n_CovarianceMatrix()(States::PosX, States::PosY) : std::nan("")); // NE-ECEF StDev [m]
    addData(pinIndex, i++, obs->n_CovarianceMatrix().rows() >= 1 && obs->n_CovarianceMatrix().cols() >= 3 ? obs->n_CovarianceMatrix()(States::PosX, States::PosZ) : std::nan("")); // ND-ECEF StDev [m]
    addData(pinIndex, i++, obs->n_CovarianceMatrix().rows() >= 2 && obs->n_CovarianceMatrix().cols() >= 3 ? obs->n_CovarianceMatrix()(States::PosY, States::PosZ) : std::nan("")); // ED-ECEF StDev [m]
    addData(pinIndex, i++, obs->e_velocityStdev()(0));                                                                                                                             // X velocity ECEF StDev [m/s]
    addData(pinIndex, i++, obs->e_velocityStdev()(1));                                                                                                                             // Y velocity ECEF StDev [m/s]
    addData(pinIndex, i++, obs->e_velocityStdev()(2));                                                                                                                             // Z velocity ECEF StDev [m/s]
    addData(pinIndex, i++, obs->e_CovarianceMatrix().rows() >= 4 && obs->e_CovarianceMatrix().cols() >= 5 ? obs->e_CovarianceMatrix()(States::VelX, States::VelY) : std::nan("")); // XY velocity StDev [m]
    addData(pinIndex, i++, obs->e_CovarianceMatrix().rows() >= 4 && obs->e_CovarianceMatrix().cols() >= 6 ? obs->e_CovarianceMatrix()(States::VelX, States::VelZ) : std::nan("")); // XZ velocity StDev [m]
    addData(pinIndex, i++, obs->e_CovarianceMatrix().rows() >= 5 && obs->e_CovarianceMatrix().cols() >= 6 ? obs->e_CovarianceMatrix()(States::VelY, States::VelZ) : std::nan("")); // YZ velocity StDev [m]
    addData(pinIndex, i++, obs->n_velocityStdev()(0));                                                                                                                             // North velocity StDev [m/s]
    addData(pinIndex, i++, obs->n_velocityStdev()(1));                                                                                                                             // East velocity StDev [m/s]
    addData(pinIndex, i++, obs->n_velocityStdev()(2));                                                                                                                             // Down velocity StDev [m/s]
    addData(pinIndex, i++, obs->n_CovarianceMatrix().rows() >= 4 && obs->n_CovarianceMatrix().cols() >= 5 ? obs->n_CovarianceMatrix()(States::VelX, States::VelY) : std::nan("")); // NE velocity StDev [m]
    addData(pinIndex, i++, obs->n_CovarianceMatrix().rows() >= 4 && obs->n_CovarianceMatrix().cols() >= 6 ? obs->n_CovarianceMatrix()(States::VelX, States::VelZ) : std::nan("")); // ND velocity StDev [m]
    addData(pinIndex, i++, obs->n_CovarianceMatrix().rows() >= 5 && obs->n_CovarianceMatrix().cols() >= 6 ? obs->n_CovarianceMatrix()(States::VelY, States::VelZ) : std::nan("")); // ED velocity StDev [m]
    addData(pinIndex, i++, obs->recvClk.bias.stdDev);                                                                                                                              // Receiver clock bias StDev [s]
    addData(pinIndex, i++, obs->recvClk.drift.stdDev);                                                                                                                             // Receiver clock drift StDev [s/s]

    addData(pinIndex, i++, static_cast<double>(obs->recvClk.referenceTimeSatelliteSystem.toEnumeration())); // System time reference system

    addData(pinIndex, i++, obs->recvClk.sysTimeDiff.contains(GPS) ? obs->recvClk.sysTimeDiff.at(GPS).value : std::nan(""));     // GPS system time difference [s]
    addData(pinIndex, i++, obs->recvClk.sysTimeDiff.contains(GAL) ? obs->recvClk.sysTimeDiff.at(GAL).value : std::nan(""));     // GAL system time difference [s]
    addData(pinIndex, i++, obs->recvClk.sysTimeDiff.contains(GLO) ? obs->recvClk.sysTimeDiff.at(GLO).value : std::nan(""));     // GLO system time difference [s]
    addData(pinIndex, i++, obs->recvClk.sysTimeDiff.contains(BDS) ? obs->recvClk.sysTimeDiff.at(BDS).value : std::nan(""));     // BDS system time difference [s]
    addData(pinIndex, i++, obs->recvClk.sysTimeDiff.contains(QZSS) ? obs->recvClk.sysTimeDiff.at(QZSS).value : std::nan(""));   // QZSS system time difference [s]
    addData(pinIndex, i++, obs->recvClk.sysTimeDiff.contains(IRNSS) ? obs->recvClk.sysTimeDiff.at(IRNSS).value : std::nan("")); // IRNSS system time difference [s]
    addData(pinIndex, i++, obs->recvClk.sysTimeDiff.contains(SBAS) ? obs->recvClk.sysTimeDiff.at(SBAS).value : std::nan(""));   // SBAS system time difference [s]

    addData(pinIndex, i++, obs->recvClk.sysDriftDiff.contains(GPS) ? obs->recvClk.sysDriftDiff.at(GPS).value : std::nan(""));     // GPS system time drift difference [s/s]
    addData(pinIndex, i++, obs->recvClk.sysDriftDiff.contains(GAL) ? obs->recvClk.sysDriftDiff.at(GAL).value : std::nan(""));     // GAL system time drift difference [s/s]
    addData(pinIndex, i++, obs->recvClk.sysDriftDiff.contains(GLO) ? obs->recvClk.sysDriftDiff.at(GLO).value : std::nan(""));     // GLO system time drift difference [s/s]
    addData(pinIndex, i++, obs->recvClk.sysDriftDiff.contains(BDS) ? obs->recvClk.sysDriftDiff.at(BDS).value : std::nan(""));     // BDS system time drift difference [s/s]
    addData(pinIndex, i++, obs->recvClk.sysDriftDiff.contains(QZSS) ? obs->recvClk.sysDriftDiff.at(QZSS).value : std::nan(""));   // QZSS system time drift difference [s/s]
    addData(pinIndex, i++, obs->recvClk.sysDriftDiff.contains(IRNSS) ? obs->recvClk.sysDriftDiff.at(IRNSS).value : std::nan("")); // IRNSS system time drift difference [s/s]
    addData(pinIndex, i++, obs->recvClk.sysDriftDiff.contains(SBAS) ? obs->recvClk.sysDriftDiff.at(SBAS).value : std::nan(""));   // SBAS system time drift difference [s/s]

    addData(pinIndex, i++, obs->recvClk.sysTimeDiff.contains(GPS) ? obs->recvClk.sysTimeDiff.at(GPS).stdDev : std::nan(""));     // GPS system time difference StDev [s]
    addData(pinIndex, i++, obs->recvClk.sysTimeDiff.contains(GAL) ? obs->recvClk.sysTimeDiff.at(GAL).stdDev : std::nan(""));     // GAL system time difference StDev [s]
    addData(pinIndex, i++, obs->recvClk.sysTimeDiff.contains(GLO) ? obs->recvClk.sysTimeDiff.at(GLO).stdDev : std::nan(""));     // GLO system time difference StDev [s]
    addData(pinIndex, i++, obs->recvClk.sysTimeDiff.contains(BDS) ? obs->recvClk.sysTimeDiff.at(BDS).stdDev : std::nan(""));     // BDS system time difference StDev [s]
    addData(pinIndex, i++, obs->recvClk.sysTimeDiff.contains(QZSS) ? obs->recvClk.sysTimeDiff.at(QZSS).stdDev : std::nan(""));   // QZSS system time difference StDev [s]
    addData(pinIndex, i++, obs->recvClk.sysTimeDiff.contains(IRNSS) ? obs->recvClk.sysTimeDiff.at(IRNSS).stdDev : std::nan("")); // IRNSS system time difference StDev [s]
    addData(pinIndex, i++, obs->recvClk.sysTimeDiff.contains(SBAS) ? obs->recvClk.sysTimeDiff.at(SBAS).stdDev : std::nan(""));   // SBAS system time difference StDev [s]

    addData(pinIndex, i++, obs->recvClk.sysDriftDiff.contains(GPS) ? obs->recvClk.sysDriftDiff.at(GPS).stdDev : std::nan(""));     // GPS system time drift difference StDev [s/s]
    addData(pinIndex, i++, obs->recvClk.sysDriftDiff.contains(GAL) ? obs->recvClk.sysDriftDiff.at(GAL).stdDev : std::nan(""));     // GAL system time drift difference StDev [s/s]
    addData(pinIndex, i++, obs->recvClk.sysDriftDiff.contains(GLO) ? obs->recvClk.sysDriftDiff.at(GLO).stdDev : std::nan(""));     // GLO system time drift difference StDev [s/s]
    addData(pinIndex, i++, obs->recvClk.sysDriftDiff.contains(BDS) ? obs->recvClk.sysDriftDiff.at(BDS).stdDev : std::nan(""));     // BDS system time drift difference StDev [s/s]
    addData(pinIndex, i++, obs->recvClk.sysDriftDiff.contains(QZSS) ? obs->recvClk.sysDriftDiff.at(QZSS).stdDev : std::nan(""));   // QZSS system time drift difference StDev [s/s]
    addData(pinIndex, i++, obs->recvClk.sysDriftDiff.contains(IRNSS) ? obs->recvClk.sysDriftDiff.at(IRNSS).stdDev : std::nan("")); // IRNSS system time drift difference StDev [s/s]
    addData(pinIndex, i++, obs->recvClk.sysDriftDiff.contains(SBAS) ? obs->recvClk.sysDriftDiff.at(SBAS).stdDev : std::nan(""));   // SBAS system time drift difference StDev [s/s]

    // Dynamic data
    for (const auto& satData : obs->satData)
    {
        addData(pinIndex, fmt::format("{} SatPos ECEF X [m]", satData.satSigId), satData.e_satPos.x());
        addData(pinIndex, fmt::format("{} SatPos ECEF Y [m]", satData.satSigId), satData.e_satPos.y());
        addData(pinIndex, fmt::format("{} SatPos ECEF Z [m]", satData.satSigId), satData.e_satPos.z());
        addData(pinIndex, fmt::format("{} SatVel ECEF X [m/s]", satData.satSigId), satData.e_satVel.x());
        addData(pinIndex, fmt::format("{} SatVel ECEF Y [m/s]", satData.satSigId), satData.e_satVel.y());
        addData(pinIndex, fmt::format("{} SatVel ECEF Z [m/s]", satData.satSigId), satData.e_satVel.z());
        addData(pinIndex, fmt::format("{} Satellite clock bias [s]", satData.satSigId), satData.satClkBias);
        addData(pinIndex, fmt::format("{} Satellite clock drift [s/s]", satData.satSigId), satData.satClkDrift);
        addData(pinIndex, fmt::format("{} Elevation [deg]", satData.satSigId), rad2deg(satData.satElevation));
        addData(pinIndex, fmt::format("{} Azimuth [deg]", satData.satSigId), rad2deg(satData.satAzimuth));
        addData(pinIndex, fmt::format("{} Unhealthy (skipped)", satData.satSigId), static_cast<int>(satData.skipped));
        addData(pinIndex, fmt::format("{} Elevation mask triggered", satData.satSigId), static_cast<int>(satData.elevationMaskTriggered));
        addData(pinIndex, fmt::format("{} Estimated Pseudorange [m]", satData.satSigId), satData.psrEst);
        addData(pinIndex, fmt::format("{} Estimated Pseudorange rate [m/s]", satData.satSigId), satData.psrRateEst.has_value() ? satData.psrRateEst.value() : std::nan(""));
        addData(pinIndex, fmt::format("{} Geometric distance [m]", satData.satSigId), satData.geometricDist);
        addData(pinIndex, fmt::format("{} Estimated Inter-system clock bias [m]", satData.satSigId), satData.dpsr_clkISB);
        addData(pinIndex, fmt::format("{} Estimated ionosphere propagation error [m]", satData.satSigId), satData.dpsr_I);
        addData(pinIndex, fmt::format("{} Estimated troposphere propagation error [m]", satData.satSigId), satData.dpsr_T);
        addData(pinIndex, fmt::format("{} Sagnac correction [m]", satData.satSigId), satData.dpsr_ie);
    }
}

void NAV::Plot::plotRtklibPosObs(const std::shared_ptr<const RtklibPosObs>& obs, size_t pinIndex)
{
    if (!obs->insTime.empty() && _startTime.empty()) { _startTime = obs->insTime; }
    size_t i = 0;

    if (!_originPosition)
    {
        _originPosition = { .frame = gui::widgets::PositionWithFrame::ReferenceFrame::ECEF,
                            .e_position = obs->e_position() };
    }

    int sign = obs->lla_position().x() > _originPosition->latitude() ? 1 : -1;
    // North/South deviation [m]
    double northSouth = calcGeographicalDistance(obs->lla_position().x(), obs->lla_position().y(),
                                                 _originPosition->latitude(), obs->lla_position().y())
                        * sign;

    sign = obs->lla_position().y() > _originPosition->longitude() ? 1 : -1;
    // East/West deviation [m]
    double eastWest = calcGeographicalDistance(obs->lla_position().x(), obs->lla_position().y(),
                                               obs->lla_position().x(), _originPosition->longitude())
                      * sign;

    std::scoped_lock<std::mutex> guard(_pinData.at(pinIndex).mutex);

    // NodeData
    addData(pinIndex, i++, !obs->insTime.empty() ? static_cast<double>((obs->insTime - _startTime).count()) : std::nan(""));
    addData(pinIndex, i++, !obs->insTime.empty() ? static_cast<double>(obs->insTime.toGPSweekTow().tow) : std::nan(""));
    // RtklibPosObs
    addData(pinIndex, i++, obs->e_position().x());
    addData(pinIndex, i++, obs->e_position().y());
    addData(pinIndex, i++, obs->e_position().z());
    addData(pinIndex, i++, rad2deg(obs->lla_position()(0)));
    addData(pinIndex, i++, rad2deg(obs->lla_position()(1)));
    addData(pinIndex, i++, obs->lla_position()(2));
    addData(pinIndex, i++, northSouth);
    addData(pinIndex, i++, eastWest);
    addData(pinIndex, i++, obs->Q);
    addData(pinIndex, i++, obs->ns);
    addData(pinIndex, i++, obs->sdXYZ.x());
    addData(pinIndex, i++, obs->sdXYZ.y());
    addData(pinIndex, i++, obs->sdXYZ.z());
    addData(pinIndex, i++, obs->sdNED.x());
    addData(pinIndex, i++, obs->sdNED.y());
    addData(pinIndex, i++, obs->sdNED.z());
    addData(pinIndex, i++, obs->sdxy.has_value() ? obs->sdxy.value() : std::nan(""));
    addData(pinIndex, i++, obs->sdyz.has_value() ? obs->sdyz.value() : std::nan(""));
    addData(pinIndex, i++, obs->sdzx.has_value() ? obs->sdzx.value() : std::nan(""));
    addData(pinIndex, i++, obs->sdne.has_value() ? obs->sdne.value() : std::nan(""));
    addData(pinIndex, i++, obs->sded.has_value() ? obs->sded.value() : std::nan(""));
    addData(pinIndex, i++, obs->sddn.has_value() ? obs->sddn.value() : std::nan(""));
    addData(pinIndex, i++, obs->age);
    addData(pinIndex, i++, obs->ratio);
    addData(pinIndex, i++, obs->e_velocity().norm());
    addData(pinIndex, i++, obs->e_velocity().x());
    addData(pinIndex, i++, obs->e_velocity().y());
    addData(pinIndex, i++, obs->e_velocity().z());
    addData(pinIndex, i++, obs->n_velocity().x());
    addData(pinIndex, i++, obs->n_velocity().y());
    addData(pinIndex, i++, obs->n_velocity().z());
    addData(pinIndex, i++, obs->sdvNED.has_value() ? obs->sdvNED->x() : std::nan(""));
    addData(pinIndex, i++, obs->sdvNED.has_value() ? obs->sdvNED->y() : std::nan(""));
    addData(pinIndex, i++, obs->sdvNED.has_value() ? obs->sdvNED->z() : std::nan(""));
    addData(pinIndex, i++, obs->sdvne.has_value() ? obs->sdvne.value() : std::nan(""));
    addData(pinIndex, i++, obs->sdved.has_value() ? obs->sdved.value() : std::nan(""));
    addData(pinIndex, i++, obs->sdvdn.has_value() ? obs->sdvdn.value() : std::nan(""));
    addData(pinIndex, i++, obs->sdvXYZ.has_value() ? obs->sdvXYZ->x() : std::nan(""));
    addData(pinIndex, i++, obs->sdvXYZ.has_value() ? obs->sdvXYZ->y() : std::nan(""));
    addData(pinIndex, i++, obs->sdvXYZ.has_value() ? obs->sdvXYZ->z() : std::nan(""));
    addData(pinIndex, i++, obs->sdvxy.has_value() ? obs->sdvxy.value() : std::nan(""));
    addData(pinIndex, i++, obs->sdvyz.has_value() ? obs->sdvyz.value() : std::nan(""));
    addData(pinIndex, i++, obs->sdvzx.has_value() ? obs->sdvzx.value() : std::nan(""));
}

void NAV::Plot::plotUbloxObs(const std::shared_ptr<const UbloxObs>& obs, size_t pinIndex)
{
    if (!obs->insTime.empty() && _startTime.empty()) { _startTime = obs->insTime; }

    // Position in ECEF coordinates in [m]
    std::optional<Eigen::Vector3d> e_position;
    // [𝜙, λ, h] Latitude, Longitude and altitude in [rad, rad, m]
    std::optional<Eigen::Vector3d> lla_position;
    // Velocity in NED coordinates in [m/s]
    std::optional<Eigen::Vector3d> n_velocity;

    if (obs->msgClass == vendor::ublox::UbxClass::UBX_CLASS_NAV)
    {
        auto msgId = static_cast<vendor::ublox::UbxNavMessages>(obs->msgId);
        if (msgId == vendor::ublox::UbxNavMessages::UBX_NAV_POSECEF)
        {
            e_position.emplace(std::get<vendor::ublox::UbxNavPosecef>(obs->data).ecefX * 1e-2,
                               std::get<vendor::ublox::UbxNavPosecef>(obs->data).ecefY * 1e-2,
                               std::get<vendor::ublox::UbxNavPosecef>(obs->data).ecefZ * 1e-2);
            lla_position = trafo::ecef2lla_WGS84(e_position.value());
        }
        else if (msgId == vendor::ublox::UbxNavMessages::UBX_NAV_POSLLH)
        {
            lla_position.emplace(deg2rad(std::get<vendor::ublox::UbxNavPosllh>(obs->data).lat * 1e-7),
                                 deg2rad(std::get<vendor::ublox::UbxNavPosllh>(obs->data).lon * 1e-7),
                                 std::get<vendor::ublox::UbxNavPosllh>(obs->data).height * 1e-3);
        }
        else if (msgId == vendor::ublox::UbxNavMessages::UBX_NAV_VELNED)
        {
            n_velocity.emplace(std::get<vendor::ublox::UbxNavVelned>(obs->data).velN * 1e-2,
                               std::get<vendor::ublox::UbxNavVelned>(obs->data).velE * 1e-2,
                               std::get<vendor::ublox::UbxNavVelned>(obs->data).velD * 1e-2);
        }
        else
        {
            return;
        }
    }
    else
    {
        return;
    }
    // North/South deviation [m]
    std::optional<double> northSouth;
    // East/West deviation [m]
    std::optional<double> eastWest;

    if (lla_position.has_value())
    {
        if (!_originPosition)
        {
            _originPosition = { .frame = gui::widgets::PositionWithFrame::ReferenceFrame::ECEF,
                                .e_position = trafo::lla2ecef_WGS84(lla_position.value()) };
        }

        int sign = lla_position->x() > _originPosition->latitude() ? 1 : -1;
        northSouth = calcGeographicalDistance(lla_position->x(), lla_position->y(),
                                              _originPosition->latitude(), lla_position->y())
                     * sign;

        sign = lla_position->y() > _originPosition->longitude() ? 1 : -1;
        eastWest = calcGeographicalDistance(lla_position->x(), lla_position->y(),
                                            lla_position->x(), _originPosition->longitude())
                   * sign;

        lla_position->x() = rad2deg(lla_position->x());
        lla_position->y() = rad2deg(lla_position->y());
    }

    size_t i = 0;

    std::scoped_lock<std::mutex> guard(_pinData.at(pinIndex).mutex);

    // NodeData
    addData(pinIndex, i++, !obs->insTime.empty() ? static_cast<double>((obs->insTime - _startTime).count()) : std::nan(""));
    addData(pinIndex, i++, !obs->insTime.empty() ? static_cast<double>(obs->insTime.toGPSweekTow().tow) : std::nan(""));
    // UbloxObs
    addData(pinIndex, i++, e_position.has_value() ? e_position->x() : std::nan(""));
    addData(pinIndex, i++, e_position.has_value() ? e_position->y() : std::nan(""));
    addData(pinIndex, i++, e_position.has_value() ? e_position->z() : std::nan(""));
    addData(pinIndex, i++, lla_position.has_value() ? lla_position->x() : std::nan(""));
    addData(pinIndex, i++, lla_position.has_value() ? lla_position->y() : std::nan(""));
    addData(pinIndex, i++, lla_position.has_value() ? lla_position->z() : std::nan(""));
    addData(pinIndex, i++, northSouth.has_value() ? northSouth.value() : std::nan(""));
    addData(pinIndex, i++, eastWest.has_value() ? eastWest.value() : std::nan(""));
    addData(pinIndex, i++, n_velocity.has_value() ? n_velocity->norm() : std::nan(""));
    addData(pinIndex, i++, n_velocity.has_value() ? n_velocity->x() : std::nan(""));
    addData(pinIndex, i++, n_velocity.has_value() ? n_velocity->y() : std::nan(""));
    addData(pinIndex, i++, n_velocity.has_value() ? n_velocity->z() : std::nan(""));
}

void NAV::Plot::plotImuObs(const std::shared_ptr<const ImuObs>& obs, size_t pinIndex)
{
    if (!obs->insTime.empty() && _startTime.empty()) { _startTime = obs->insTime; }
    size_t i = 0;

    std::scoped_lock<std::mutex> guard(_pinData.at(pinIndex).mutex);

    // NodeData
    addData(pinIndex, i++, !obs->insTime.empty() ? static_cast<double>((obs->insTime - _startTime).count()) : std::nan(""));
    addData(pinIndex, i++, !obs->insTime.empty() ? static_cast<double>(obs->insTime.toGPSweekTow().tow) : std::nan(""));
    // ImuObs
    addData(pinIndex, i++, obs->timeSinceStartup.has_value() ? static_cast<double>(obs->timeSinceStartup.value()) : std::nan(""));
    addData(pinIndex, i++, obs->magUncompXYZ.has_value() ? obs->magUncompXYZ->x() : std::nan(""));
    addData(pinIndex, i++, obs->magUncompXYZ.has_value() ? obs->magUncompXYZ->y() : std::nan(""));
    addData(pinIndex, i++, obs->magUncompXYZ.has_value() ? obs->magUncompXYZ->z() : std::nan(""));
    addData(pinIndex, i++, obs->accelUncompXYZ.has_value() ? obs->accelUncompXYZ->x() : std::nan(""));
    addData(pinIndex, i++, obs->accelUncompXYZ.has_value() ? obs->accelUncompXYZ->y() : std::nan(""));
    addData(pinIndex, i++, obs->accelUncompXYZ.has_value() ? obs->accelUncompXYZ->z() : std::nan(""));
    addData(pinIndex, i++, obs->gyroUncompXYZ.has_value() ? obs->gyroUncompXYZ->x() : std::nan(""));
    addData(pinIndex, i++, obs->gyroUncompXYZ.has_value() ? obs->gyroUncompXYZ->y() : std::nan(""));
    addData(pinIndex, i++, obs->gyroUncompXYZ.has_value() ? obs->gyroUncompXYZ->z() : std::nan(""));
    addData(pinIndex, i++, obs->magCompXYZ.has_value() ? obs->magCompXYZ->x() : std::nan(""));
    addData(pinIndex, i++, obs->magCompXYZ.has_value() ? obs->magCompXYZ->y() : std::nan(""));
    addData(pinIndex, i++, obs->magCompXYZ.has_value() ? obs->magCompXYZ->z() : std::nan(""));
    addData(pinIndex, i++, obs->accelCompXYZ.has_value() ? obs->accelCompXYZ->x() : std::nan(""));
    addData(pinIndex, i++, obs->accelCompXYZ.has_value() ? obs->accelCompXYZ->y() : std::nan(""));
    addData(pinIndex, i++, obs->accelCompXYZ.has_value() ? obs->accelCompXYZ->z() : std::nan(""));
    addData(pinIndex, i++, obs->gyroCompXYZ.has_value() ? obs->gyroCompXYZ->x() : std::nan(""));
    addData(pinIndex, i++, obs->gyroCompXYZ.has_value() ? obs->gyroCompXYZ->y() : std::nan(""));
    addData(pinIndex, i++, obs->gyroCompXYZ.has_value() ? obs->gyroCompXYZ->z() : std::nan(""));
    addData(pinIndex, i++, obs->temperature.has_value() ? obs->temperature.value() : std::nan(""));
}

void NAV::Plot::plotImuObsSimulated(const std::shared_ptr<const ImuObsSimulated>& obs, size_t pinIndex)
{
    if (!obs->insTime.empty() && _startTime.empty()) { _startTime = obs->insTime; }
    size_t i = 0;

    std::scoped_lock<std::mutex> guard(_pinData.at(pinIndex).mutex);

    // NodeData
    addData(pinIndex, i++, !obs->insTime.empty() ? static_cast<double>((obs->insTime - _startTime).count()) : std::nan(""));
    addData(pinIndex, i++, !obs->insTime.empty() ? static_cast<double>(obs->insTime.toGPSweekTow().tow) : std::nan(""));
    // ImuObs
    addData(pinIndex, i++, obs->timeSinceStartup.has_value() ? static_cast<double>(obs->timeSinceStartup.value()) : std::nan(""));
    addData(pinIndex, i++, obs->magUncompXYZ.has_value() ? obs->magUncompXYZ->x() : std::nan(""));
    addData(pinIndex, i++, obs->magUncompXYZ.has_value() ? obs->magUncompXYZ->y() : std::nan(""));
    addData(pinIndex, i++, obs->magUncompXYZ.has_value() ? obs->magUncompXYZ->z() : std::nan(""));
    addData(pinIndex, i++, obs->accelUncompXYZ.has_value() ? obs->accelUncompXYZ->x() : std::nan(""));
    addData(pinIndex, i++, obs->accelUncompXYZ.has_value() ? obs->accelUncompXYZ->y() : std::nan(""));
    addData(pinIndex, i++, obs->accelUncompXYZ.has_value() ? obs->accelUncompXYZ->z() : std::nan(""));
    addData(pinIndex, i++, obs->gyroUncompXYZ.has_value() ? obs->gyroUncompXYZ->x() : std::nan(""));
    addData(pinIndex, i++, obs->gyroUncompXYZ.has_value() ? obs->gyroUncompXYZ->y() : std::nan(""));
    addData(pinIndex, i++, obs->gyroUncompXYZ.has_value() ? obs->gyroUncompXYZ->z() : std::nan(""));
    addData(pinIndex, i++, obs->magCompXYZ.has_value() ? obs->magCompXYZ->x() : std::nan(""));
    addData(pinIndex, i++, obs->magCompXYZ.has_value() ? obs->magCompXYZ->y() : std::nan(""));
    addData(pinIndex, i++, obs->magCompXYZ.has_value() ? obs->magCompXYZ->z() : std::nan(""));
    addData(pinIndex, i++, obs->accelCompXYZ.has_value() ? obs->accelCompXYZ->x() : std::nan(""));
    addData(pinIndex, i++, obs->accelCompXYZ.has_value() ? obs->accelCompXYZ->y() : std::nan(""));
    addData(pinIndex, i++, obs->accelCompXYZ.has_value() ? obs->accelCompXYZ->z() : std::nan(""));
    addData(pinIndex, i++, obs->gyroCompXYZ.has_value() ? obs->gyroCompXYZ->x() : std::nan(""));
    addData(pinIndex, i++, obs->gyroCompXYZ.has_value() ? obs->gyroCompXYZ->y() : std::nan(""));
    addData(pinIndex, i++, obs->gyroCompXYZ.has_value() ? obs->gyroCompXYZ->z() : std::nan(""));
    addData(pinIndex, i++, obs->temperature.has_value() ? obs->temperature.value() : std::nan(""));
    // ImuObsSimulated
    addData(pinIndex, i++, obs->n_accelDynamics.x());       // AccelDynamicsN [m/s^2]
    addData(pinIndex, i++, obs->n_accelDynamics.y());       // AccelDynamicsE [m/s^2]
    addData(pinIndex, i++, obs->n_accelDynamics.z());       // AccelDynamicsD [m/s^2]
    addData(pinIndex, i++, obs->n_angularRateDynamics.x()); // AngularRateN (ω_nb_n) [rad/s]
    addData(pinIndex, i++, obs->n_angularRateDynamics.y()); // AngularRateE (ω_nb_n) [rad/s]
    addData(pinIndex, i++, obs->n_angularRateDynamics.z()); // AngularRateD (ω_nb_n) [rad/s]
    addData(pinIndex, i++, obs->e_accelDynamics.x());       // AccelDynamicsX ECEF [m/s^2]
    addData(pinIndex, i++, obs->e_accelDynamics.y());       // AccelDynamicsY ECEF [m/s^2]
    addData(pinIndex, i++, obs->e_accelDynamics.z());       // AccelDynamicsZ ECEF [m/s^2]
    addData(pinIndex, i++, obs->e_angularRateDynamics.x()); // AngularRateX ECEF (ω_nb_e) [rad/s]
    addData(pinIndex, i++, obs->e_angularRateDynamics.y()); // AngularRateY ECEF (ω_nb_e) [rad/s]
    addData(pinIndex, i++, obs->e_angularRateDynamics.z()); // AngularRateZ ECEF (ω_nb_e) [rad/s]
}

void NAV::Plot::plotKvhObs(const std::shared_ptr<const KvhObs>& obs, size_t pinIndex)
{
    if (!obs->insTime.empty() && _startTime.empty()) { _startTime = obs->insTime; }
    size_t i = 0;

    std::scoped_lock<std::mutex> guard(_pinData.at(pinIndex).mutex);

    // NodeData
    addData(pinIndex, i++, !obs->insTime.empty() ? static_cast<double>((obs->insTime - _startTime).count()) : std::nan(""));
    addData(pinIndex, i++, !obs->insTime.empty() ? static_cast<double>(obs->insTime.toGPSweekTow().tow) : std::nan(""));
    // ImuObs
    addData(pinIndex, i++, obs->timeSinceStartup.has_value() ? static_cast<double>(obs->timeSinceStartup.value()) : std::nan(""));
    addData(pinIndex, i++, obs->magUncompXYZ.has_value() ? obs->magUncompXYZ->x() : std::nan(""));
    addData(pinIndex, i++, obs->magUncompXYZ.has_value() ? obs->magUncompXYZ->y() : std::nan(""));
    addData(pinIndex, i++, obs->magUncompXYZ.has_value() ? obs->magUncompXYZ->z() : std::nan(""));
    addData(pinIndex, i++, obs->accelUncompXYZ.has_value() ? obs->accelUncompXYZ->x() : std::nan(""));
    addData(pinIndex, i++, obs->accelUncompXYZ.has_value() ? obs->accelUncompXYZ->y() : std::nan(""));
    addData(pinIndex, i++, obs->accelUncompXYZ.has_value() ? obs->accelUncompXYZ->z() : std::nan(""));
    addData(pinIndex, i++, obs->gyroUncompXYZ.has_value() ? obs->gyroUncompXYZ->x() : std::nan(""));
    addData(pinIndex, i++, obs->gyroUncompXYZ.has_value() ? obs->gyroUncompXYZ->y() : std::nan(""));
    addData(pinIndex, i++, obs->gyroUncompXYZ.has_value() ? obs->gyroUncompXYZ->z() : std::nan(""));
    addData(pinIndex, i++, obs->magCompXYZ.has_value() ? obs->magCompXYZ->x() : std::nan(""));
    addData(pinIndex, i++, obs->magCompXYZ.has_value() ? obs->magCompXYZ->y() : std::nan(""));
    addData(pinIndex, i++, obs->magCompXYZ.has_value() ? obs->magCompXYZ->z() : std::nan(""));
    addData(pinIndex, i++, obs->accelCompXYZ.has_value() ? obs->accelCompXYZ->x() : std::nan(""));
    addData(pinIndex, i++, obs->accelCompXYZ.has_value() ? obs->accelCompXYZ->y() : std::nan(""));
    addData(pinIndex, i++, obs->accelCompXYZ.has_value() ? obs->accelCompXYZ->z() : std::nan(""));
    addData(pinIndex, i++, obs->gyroCompXYZ.has_value() ? obs->gyroCompXYZ->x() : std::nan(""));
    addData(pinIndex, i++, obs->gyroCompXYZ.has_value() ? obs->gyroCompXYZ->y() : std::nan(""));
    addData(pinIndex, i++, obs->gyroCompXYZ.has_value() ? obs->gyroCompXYZ->z() : std::nan(""));
    addData(pinIndex, i++, obs->temperature.has_value() ? obs->temperature.value() : std::nan(""));
    // KvhObs
    addData(pinIndex, i++, static_cast<double>(obs->status.to_ulong()));
    addData(pinIndex, i++, obs->sequenceNumber < 128 ? obs->sequenceNumber : std::nan(""));
}

void NAV::Plot::plotImuObsWDeltaObs(const std::shared_ptr<const ImuObsWDelta>& obs, size_t pinIndex)
{
    if (!obs->insTime.empty() && _startTime.empty()) { _startTime = obs->insTime; }
    size_t i = 0;

    std::scoped_lock<std::mutex> guard(_pinData.at(pinIndex).mutex);

    // NodeData
    addData(pinIndex, i++, !obs->insTime.empty() ? static_cast<double>((obs->insTime - _startTime).count()) : std::nan(""));
    addData(pinIndex, i++, !obs->insTime.empty() ? static_cast<double>(obs->insTime.toGPSweekTow().tow) : std::nan(""));
    // ImuObs
    addData(pinIndex, i++, obs->timeSinceStartup.has_value() ? static_cast<double>(obs->timeSinceStartup.value()) : std::nan(""));
    addData(pinIndex, i++, obs->magUncompXYZ.has_value() ? obs->magUncompXYZ->x() : std::nan(""));
    addData(pinIndex, i++, obs->magUncompXYZ.has_value() ? obs->magUncompXYZ->y() : std::nan(""));
    addData(pinIndex, i++, obs->magUncompXYZ.has_value() ? obs->magUncompXYZ->z() : std::nan(""));
    addData(pinIndex, i++, obs->accelUncompXYZ.has_value() ? obs->accelUncompXYZ->x() : std::nan(""));
    addData(pinIndex, i++, obs->accelUncompXYZ.has_value() ? obs->accelUncompXYZ->y() : std::nan(""));
    addData(pinIndex, i++, obs->accelUncompXYZ.has_value() ? obs->accelUncompXYZ->z() : std::nan(""));
    addData(pinIndex, i++, obs->gyroUncompXYZ.has_value() ? obs->gyroUncompXYZ->x() : std::nan(""));
    addData(pinIndex, i++, obs->gyroUncompXYZ.has_value() ? obs->gyroUncompXYZ->y() : std::nan(""));
    addData(pinIndex, i++, obs->gyroUncompXYZ.has_value() ? obs->gyroUncompXYZ->z() : std::nan(""));
    addData(pinIndex, i++, obs->magCompXYZ.has_value() ? obs->magCompXYZ->x() : std::nan(""));
    addData(pinIndex, i++, obs->magCompXYZ.has_value() ? obs->magCompXYZ->y() : std::nan(""));
    addData(pinIndex, i++, obs->magCompXYZ.has_value() ? obs->magCompXYZ->z() : std::nan(""));
    addData(pinIndex, i++, obs->accelCompXYZ.has_value() ? obs->accelCompXYZ->x() : std::nan(""));
    addData(pinIndex, i++, obs->accelCompXYZ.has_value() ? obs->accelCompXYZ->y() : std::nan(""));
    addData(pinIndex, i++, obs->accelCompXYZ.has_value() ? obs->accelCompXYZ->z() : std::nan(""));
    addData(pinIndex, i++, obs->gyroCompXYZ.has_value() ? obs->gyroCompXYZ->x() : std::nan(""));
    addData(pinIndex, i++, obs->gyroCompXYZ.has_value() ? obs->gyroCompXYZ->y() : std::nan(""));
    addData(pinIndex, i++, obs->gyroCompXYZ.has_value() ? obs->gyroCompXYZ->z() : std::nan(""));
    addData(pinIndex, i++, obs->temperature.has_value() ? obs->temperature.value() : std::nan(""));
    // ImuObsWDelta
    addData(pinIndex, i++, obs->dtime);
    addData(pinIndex, i++, obs->dtheta.has_value() ? obs->dtheta->x() : std::nan(""));
    addData(pinIndex, i++, obs->dtheta.has_value() ? obs->dtheta->y() : std::nan(""));
    addData(pinIndex, i++, obs->dtheta.has_value() ? obs->dtheta->z() : std::nan(""));
    addData(pinIndex, i++, obs->dvel.has_value() ? obs->dvel->x() : std::nan(""));
    addData(pinIndex, i++, obs->dvel.has_value() ? obs->dvel->y() : std::nan(""));
    addData(pinIndex, i++, obs->dvel.has_value() ? obs->dvel->z() : std::nan(""));
}

void NAV::Plot::plotVectorNavBinaryObs(const std::shared_ptr<const VectorNavBinaryOutput>& obs, size_t pinIndex)
{
    if (!obs->insTime.empty() && _startTime.empty()) { _startTime = obs->insTime; }
    size_t i = 0;

    std::scoped_lock<std::mutex> guard(_pinData.at(pinIndex).mutex);

    // NodeData
    addData(pinIndex, i++, !obs->insTime.empty() ? static_cast<double>((obs->insTime - _startTime).count()) : std::nan(""));
    addData(pinIndex, i++, !obs->insTime.empty() ? static_cast<double>(obs->insTime.toGPSweekTow().tow) : std::nan(""));
    // VectorNavBinaryOutput
    // Group 2 (Time)
    addData(pinIndex, i++, obs->timeOutputs && (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESTARTUP) ? static_cast<double>(obs->timeOutputs->timeStartup) : std::nan(""));
    addData(pinIndex, i++, obs->timeOutputs && (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEGPS) ? static_cast<double>(obs->timeOutputs->timeGps) : std::nan(""));
    addData(pinIndex, i++, obs->timeOutputs && (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_GPSTOW) ? static_cast<double>(obs->timeOutputs->gpsTow) : std::nan(""));
    addData(pinIndex, i++, obs->timeOutputs && (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_GPSWEEK) ? static_cast<double>(obs->timeOutputs->gpsWeek) : std::nan(""));
    addData(pinIndex, i++, obs->timeOutputs && (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESYNCIN) ? static_cast<double>(obs->timeOutputs->timeSyncIn) : std::nan(""));
    addData(pinIndex, i++, obs->timeOutputs && (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEGPSPPS) ? static_cast<double>(obs->timeOutputs->timePPS) : std::nan(""));
    addData(pinIndex, i++, obs->timeOutputs && (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEUTC) ? static_cast<double>(obs->timeOutputs->timeUtc.year) : std::nan(""));
    addData(pinIndex, i++, obs->timeOutputs && (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEUTC) ? static_cast<double>(obs->timeOutputs->timeUtc.month) : std::nan(""));
    addData(pinIndex, i++, obs->timeOutputs && (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEUTC) ? static_cast<double>(obs->timeOutputs->timeUtc.day) : std::nan(""));
    addData(pinIndex, i++, obs->timeOutputs && (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEUTC) ? static_cast<double>(obs->timeOutputs->timeUtc.hour) : std::nan(""));
    addData(pinIndex, i++, obs->timeOutputs && (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEUTC) ? static_cast<double>(obs->timeOutputs->timeUtc.min) : std::nan(""));
    addData(pinIndex, i++, obs->timeOutputs && (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEUTC) ? static_cast<double>(obs->timeOutputs->timeUtc.sec) : std::nan(""));
    addData(pinIndex, i++, obs->timeOutputs && (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEUTC) ? static_cast<double>(obs->timeOutputs->timeUtc.ms) : std::nan(""));
    addData(pinIndex, i++, obs->timeOutputs && (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_SYNCINCNT) ? static_cast<double>(obs->timeOutputs->syncInCnt) : std::nan(""));
    addData(pinIndex, i++, obs->timeOutputs && (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_SYNCOUTCNT) ? static_cast<double>(obs->timeOutputs->syncOutCnt) : std::nan(""));
    addData(pinIndex, i++, obs->timeOutputs && (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESTATUS) ? static_cast<double>(obs->timeOutputs->timeStatus.timeOk()) : std::nan(""));
    addData(pinIndex, i++, obs->timeOutputs && (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESTATUS) ? static_cast<double>(obs->timeOutputs->timeStatus.dateOk()) : std::nan(""));
    addData(pinIndex, i++, obs->timeOutputs && (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESTATUS) ? static_cast<double>(obs->timeOutputs->timeStatus.utcTimeValid()) : std::nan(""));
    // Group 3 (IMU)
    addData(pinIndex, i++, obs->imuOutputs && (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_IMUSTATUS) ? static_cast<double>(obs->imuOutputs->imuStatus) : std::nan(""));
    addData(pinIndex, i++, obs->imuOutputs && (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPMAG) ? static_cast<double>(obs->imuOutputs->uncompMag(0)) : std::nan(""));
    addData(pinIndex, i++, obs->imuOutputs && (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPMAG) ? static_cast<double>(obs->imuOutputs->uncompMag(1)) : std::nan(""));
    addData(pinIndex, i++, obs->imuOutputs && (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPMAG) ? static_cast<double>(obs->imuOutputs->uncompMag(2)) : std::nan(""));
    addData(pinIndex, i++, obs->imuOutputs && (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPACCEL) ? static_cast<double>(obs->imuOutputs->uncompAccel(0)) : std::nan(""));
    addData(pinIndex, i++, obs->imuOutputs && (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPACCEL) ? static_cast<double>(obs->imuOutputs->uncompAccel(1)) : std::nan(""));
    addData(pinIndex, i++, obs->imuOutputs && (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPACCEL) ? static_cast<double>(obs->imuOutputs->uncompAccel(2)) : std::nan(""));
    addData(pinIndex, i++, obs->imuOutputs && (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPGYRO) ? static_cast<double>(obs->imuOutputs->uncompGyro(0)) : std::nan(""));
    addData(pinIndex, i++, obs->imuOutputs && (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPGYRO) ? static_cast<double>(obs->imuOutputs->uncompGyro(1)) : std::nan(""));
    addData(pinIndex, i++, obs->imuOutputs && (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPGYRO) ? static_cast<double>(obs->imuOutputs->uncompGyro(2)) : std::nan(""));
    addData(pinIndex, i++, obs->imuOutputs && (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_TEMP) ? static_cast<double>(obs->imuOutputs->temp) : std::nan(""));
    addData(pinIndex, i++, obs->imuOutputs && (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_PRES) ? static_cast<double>(obs->imuOutputs->pres) : std::nan(""));
    addData(pinIndex, i++, obs->imuOutputs && (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_DELTATHETA) ? static_cast<double>(obs->imuOutputs->deltaTime) : std::nan(""));
    addData(pinIndex, i++, obs->imuOutputs && (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_DELTATHETA) ? static_cast<double>(obs->imuOutputs->deltaTheta(0)) : std::nan(""));
    addData(pinIndex, i++, obs->imuOutputs && (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_DELTATHETA) ? static_cast<double>(obs->imuOutputs->deltaTheta(1)) : std::nan(""));
    addData(pinIndex, i++, obs->imuOutputs && (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_DELTATHETA) ? static_cast<double>(obs->imuOutputs->deltaTheta(2)) : std::nan(""));
    addData(pinIndex, i++, obs->imuOutputs && (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_DELTAVEL) ? static_cast<double>(obs->imuOutputs->deltaV(0)) : std::nan(""));
    addData(pinIndex, i++, obs->imuOutputs && (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_DELTAVEL) ? static_cast<double>(obs->imuOutputs->deltaV(1)) : std::nan(""));
    addData(pinIndex, i++, obs->imuOutputs && (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_DELTAVEL) ? static_cast<double>(obs->imuOutputs->deltaV(2)) : std::nan(""));
    addData(pinIndex, i++, obs->imuOutputs && (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_MAG) ? static_cast<double>(obs->imuOutputs->mag(0)) : std::nan(""));
    addData(pinIndex, i++, obs->imuOutputs && (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_MAG) ? static_cast<double>(obs->imuOutputs->mag(1)) : std::nan(""));
    addData(pinIndex, i++, obs->imuOutputs && (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_MAG) ? static_cast<double>(obs->imuOutputs->mag(2)) : std::nan(""));
    addData(pinIndex, i++, obs->imuOutputs && (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_ACCEL) ? static_cast<double>(obs->imuOutputs->accel(0)) : std::nan(""));
    addData(pinIndex, i++, obs->imuOutputs && (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_ACCEL) ? static_cast<double>(obs->imuOutputs->accel(1)) : std::nan(""));
    addData(pinIndex, i++, obs->imuOutputs && (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_ACCEL) ? static_cast<double>(obs->imuOutputs->accel(2)) : std::nan(""));
    addData(pinIndex, i++, obs->imuOutputs && (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_ANGULARRATE) ? static_cast<double>(obs->imuOutputs->angularRate(0)) : std::nan(""));
    addData(pinIndex, i++, obs->imuOutputs && (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_ANGULARRATE) ? static_cast<double>(obs->imuOutputs->angularRate(1)) : std::nan(""));
    addData(pinIndex, i++, obs->imuOutputs && (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_ANGULARRATE) ? static_cast<double>(obs->imuOutputs->angularRate(2)) : std::nan(""));
    // Group 4 (GNSS1)
    addData(pinIndex, i++, obs->gnss1Outputs && (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC) ? static_cast<double>(obs->gnss1Outputs->timeUtc.year) : std::nan(""));
    addData(pinIndex, i++, obs->gnss1Outputs && (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC) ? static_cast<double>(obs->gnss1Outputs->timeUtc.month) : std::nan(""));
    addData(pinIndex, i++, obs->gnss1Outputs && (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC) ? static_cast<double>(obs->gnss1Outputs->timeUtc.day) : std::nan(""));
    addData(pinIndex, i++, obs->gnss1Outputs && (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC) ? static_cast<double>(obs->gnss1Outputs->timeUtc.hour) : std::nan(""));
    addData(pinIndex, i++, obs->gnss1Outputs && (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC) ? static_cast<double>(obs->gnss1Outputs->timeUtc.min) : std::nan(""));
    addData(pinIndex, i++, obs->gnss1Outputs && (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC) ? static_cast<double>(obs->gnss1Outputs->timeUtc.sec) : std::nan(""));
    addData(pinIndex, i++, obs->gnss1Outputs && (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC) ? static_cast<double>(obs->gnss1Outputs->timeUtc.ms) : std::nan(""));
    addData(pinIndex, i++, obs->gnss1Outputs && (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TOW) ? static_cast<double>(obs->gnss1Outputs->tow) : std::nan(""));
    addData(pinIndex, i++, obs->gnss1Outputs && (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_WEEK) ? static_cast<double>(obs->gnss1Outputs->week) : std::nan(""));
    addData(pinIndex, i++, obs->gnss1Outputs && (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_NUMSATS) ? static_cast<double>(obs->gnss1Outputs->numSats) : std::nan(""));
    addData(pinIndex, i++, obs->gnss1Outputs && (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_FIX) ? static_cast<double>(obs->gnss1Outputs->fix) : std::nan(""));
    addData(pinIndex, i++, obs->gnss1Outputs && (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSLLA) ? obs->gnss1Outputs->posLla(0) : std::nan(""));
    addData(pinIndex, i++, obs->gnss1Outputs && (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSLLA) ? obs->gnss1Outputs->posLla(1) : std::nan(""));
    addData(pinIndex, i++, obs->gnss1Outputs && (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSLLA) ? obs->gnss1Outputs->posLla(2) : std::nan(""));
    addData(pinIndex, i++, obs->gnss1Outputs && (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSECEF) ? obs->gnss1Outputs->posEcef(0) : std::nan(""));
    addData(pinIndex, i++, obs->gnss1Outputs && (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSECEF) ? obs->gnss1Outputs->posEcef(1) : std::nan(""));
    addData(pinIndex, i++, obs->gnss1Outputs && (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSECEF) ? obs->gnss1Outputs->posEcef(2) : std::nan(""));
    addData(pinIndex, i++, obs->gnss1Outputs && (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELNED) ? static_cast<double>(obs->gnss1Outputs->velNed(0)) : std::nan(""));
    addData(pinIndex, i++, obs->gnss1Outputs && (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELNED) ? static_cast<double>(obs->gnss1Outputs->velNed(1)) : std::nan(""));
    addData(pinIndex, i++, obs->gnss1Outputs && (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELNED) ? static_cast<double>(obs->gnss1Outputs->velNed(2)) : std::nan(""));
    addData(pinIndex, i++, obs->gnss1Outputs && (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELECEF) ? static_cast<double>(obs->gnss1Outputs->velEcef(0)) : std::nan(""));
    addData(pinIndex, i++, obs->gnss1Outputs && (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELECEF) ? static_cast<double>(obs->gnss1Outputs->velEcef(1)) : std::nan(""));
    addData(pinIndex, i++, obs->gnss1Outputs && (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELECEF) ? static_cast<double>(obs->gnss1Outputs->velEcef(2)) : std::nan(""));
    addData(pinIndex, i++, obs->gnss1Outputs && (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSU) ? static_cast<double>(obs->gnss1Outputs->posU(0)) : std::nan(""));
    addData(pinIndex, i++, obs->gnss1Outputs && (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSU) ? static_cast<double>(obs->gnss1Outputs->posU(1)) : std::nan(""));
    addData(pinIndex, i++, obs->gnss1Outputs && (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSU) ? static_cast<double>(obs->gnss1Outputs->posU(2)) : std::nan(""));
    addData(pinIndex, i++, obs->gnss1Outputs && (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELU) ? static_cast<double>(obs->gnss1Outputs->velU) : std::nan(""));
    addData(pinIndex, i++, obs->gnss1Outputs && (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEU) ? static_cast<double>(obs->gnss1Outputs->timeU) : std::nan(""));
    addData(pinIndex, i++, obs->gnss1Outputs && (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO) ? static_cast<double>(obs->gnss1Outputs->timeInfo.status.timeOk()) : std::nan(""));
    addData(pinIndex, i++, obs->gnss1Outputs && (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO) ? static_cast<double>(obs->gnss1Outputs->timeInfo.status.dateOk()) : std::nan(""));
    addData(pinIndex, i++, obs->gnss1Outputs && (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO) ? static_cast<double>(obs->gnss1Outputs->timeInfo.status.utcTimeValid()) : std::nan(""));
    addData(pinIndex, i++, obs->gnss1Outputs && (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO) ? static_cast<double>(obs->gnss1Outputs->timeInfo.leapSeconds) : std::nan(""));
    addData(pinIndex, i++, obs->gnss1Outputs && (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP) ? static_cast<double>(obs->gnss1Outputs->dop.gDop) : std::nan(""));
    addData(pinIndex, i++, obs->gnss1Outputs && (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP) ? static_cast<double>(obs->gnss1Outputs->dop.pDop) : std::nan(""));
    addData(pinIndex, i++, obs->gnss1Outputs && (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP) ? static_cast<double>(obs->gnss1Outputs->dop.tDop) : std::nan(""));
    addData(pinIndex, i++, obs->gnss1Outputs && (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP) ? static_cast<double>(obs->gnss1Outputs->dop.vDop) : std::nan(""));
    addData(pinIndex, i++, obs->gnss1Outputs && (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP) ? static_cast<double>(obs->gnss1Outputs->dop.hDop) : std::nan(""));
    addData(pinIndex, i++, obs->gnss1Outputs && (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP) ? static_cast<double>(obs->gnss1Outputs->dop.nDop) : std::nan(""));
    addData(pinIndex, i++, obs->gnss1Outputs && (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP) ? static_cast<double>(obs->gnss1Outputs->dop.eDop) : std::nan(""));
    addData(pinIndex, i++, obs->gnss1Outputs && (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_SATINFO) ? static_cast<double>(obs->gnss1Outputs->satInfo.numSats) : std::nan(""));
    addData(pinIndex, i++, obs->gnss1Outputs && (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS) ? obs->gnss1Outputs->raw.tow : std::nan(""));
    addData(pinIndex, i++, obs->gnss1Outputs && (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS) ? static_cast<double>(obs->gnss1Outputs->raw.week) : std::nan(""));
    addData(pinIndex, i++, obs->gnss1Outputs && (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS) ? static_cast<double>(obs->gnss1Outputs->raw.numSats) : std::nan(""));
    // Group 5 (Attitude)
    addData(pinIndex, i++, obs->attitudeOutputs && (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_VPESTATUS) ? static_cast<double>(obs->attitudeOutputs->vpeStatus.attitudeQuality()) : std::nan(""));
    addData(pinIndex, i++, obs->attitudeOutputs && (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_VPESTATUS) ? static_cast<double>(obs->attitudeOutputs->vpeStatus.gyroSaturation()) : std::nan(""));
    addData(pinIndex, i++, obs->attitudeOutputs && (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_VPESTATUS) ? static_cast<double>(obs->attitudeOutputs->vpeStatus.gyroSaturationRecovery()) : std::nan(""));
    addData(pinIndex, i++, obs->attitudeOutputs && (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_VPESTATUS) ? static_cast<double>(obs->attitudeOutputs->vpeStatus.magDisturbance()) : std::nan(""));
    addData(pinIndex, i++, obs->attitudeOutputs && (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_VPESTATUS) ? static_cast<double>(obs->attitudeOutputs->vpeStatus.magSaturation()) : std::nan(""));
    addData(pinIndex, i++, obs->attitudeOutputs && (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_VPESTATUS) ? static_cast<double>(obs->attitudeOutputs->vpeStatus.accDisturbance()) : std::nan(""));
    addData(pinIndex, i++, obs->attitudeOutputs && (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_VPESTATUS) ? static_cast<double>(obs->attitudeOutputs->vpeStatus.accSaturation()) : std::nan(""));
    addData(pinIndex, i++, obs->attitudeOutputs && (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_VPESTATUS) ? static_cast<double>(obs->attitudeOutputs->vpeStatus.knownMagDisturbance()) : std::nan(""));
    addData(pinIndex, i++, obs->attitudeOutputs && (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_VPESTATUS) ? static_cast<double>(obs->attitudeOutputs->vpeStatus.knownAccelDisturbance()) : std::nan(""));
    addData(pinIndex, i++, obs->attitudeOutputs && (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YAWPITCHROLL) ? static_cast<double>(obs->attitudeOutputs->ypr(0)) : std::nan(""));
    addData(pinIndex, i++, obs->attitudeOutputs && (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YAWPITCHROLL) ? static_cast<double>(obs->attitudeOutputs->ypr(1)) : std::nan(""));
    addData(pinIndex, i++, obs->attitudeOutputs && (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YAWPITCHROLL) ? static_cast<double>(obs->attitudeOutputs->ypr(2)) : std::nan(""));
    addData(pinIndex, i++, obs->attitudeOutputs && (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_QUATERNION) ? static_cast<double>(obs->attitudeOutputs->qtn.w()) : std::nan(""));
    addData(pinIndex, i++, obs->attitudeOutputs && (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_QUATERNION) ? static_cast<double>(obs->attitudeOutputs->qtn.x()) : std::nan(""));
    addData(pinIndex, i++, obs->attitudeOutputs && (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_QUATERNION) ? static_cast<double>(obs->attitudeOutputs->qtn.y()) : std::nan(""));
    addData(pinIndex, i++, obs->attitudeOutputs && (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_QUATERNION) ? static_cast<double>(obs->attitudeOutputs->qtn.z()) : std::nan(""));
    addData(pinIndex, i++, obs->attitudeOutputs && (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_DCM) ? static_cast<double>(obs->attitudeOutputs->dcm(0, 0)) : std::nan(""));
    addData(pinIndex, i++, obs->attitudeOutputs && (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_DCM) ? static_cast<double>(obs->attitudeOutputs->dcm(0, 1)) : std::nan(""));
    addData(pinIndex, i++, obs->attitudeOutputs && (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_DCM) ? static_cast<double>(obs->attitudeOutputs->dcm(0, 2)) : std::nan(""));
    addData(pinIndex, i++, obs->attitudeOutputs && (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_DCM) ? static_cast<double>(obs->attitudeOutputs->dcm(1, 0)) : std::nan(""));
    addData(pinIndex, i++, obs->attitudeOutputs && (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_DCM) ? static_cast<double>(obs->attitudeOutputs->dcm(1, 1)) : std::nan(""));
    addData(pinIndex, i++, obs->attitudeOutputs && (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_DCM) ? static_cast<double>(obs->attitudeOutputs->dcm(1, 2)) : std::nan(""));
    addData(pinIndex, i++, obs->attitudeOutputs && (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_DCM) ? static_cast<double>(obs->attitudeOutputs->dcm(2, 0)) : std::nan(""));
    addData(pinIndex, i++, obs->attitudeOutputs && (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_DCM) ? static_cast<double>(obs->attitudeOutputs->dcm(2, 1)) : std::nan(""));
    addData(pinIndex, i++, obs->attitudeOutputs && (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_DCM) ? static_cast<double>(obs->attitudeOutputs->dcm(2, 2)) : std::nan(""));
    addData(pinIndex, i++, obs->attitudeOutputs && (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_MAGNED) ? static_cast<double>(obs->attitudeOutputs->magNed(0)) : std::nan(""));
    addData(pinIndex, i++, obs->attitudeOutputs && (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_MAGNED) ? static_cast<double>(obs->attitudeOutputs->magNed(1)) : std::nan(""));
    addData(pinIndex, i++, obs->attitudeOutputs && (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_MAGNED) ? static_cast<double>(obs->attitudeOutputs->magNed(2)) : std::nan(""));
    addData(pinIndex, i++, obs->attitudeOutputs && (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_ACCELNED) ? static_cast<double>(obs->attitudeOutputs->accelNed(0)) : std::nan(""));
    addData(pinIndex, i++, obs->attitudeOutputs && (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_ACCELNED) ? static_cast<double>(obs->attitudeOutputs->accelNed(1)) : std::nan(""));
    addData(pinIndex, i++, obs->attitudeOutputs && (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_ACCELNED) ? static_cast<double>(obs->attitudeOutputs->accelNed(2)) : std::nan(""));
    addData(pinIndex, i++, obs->attitudeOutputs && (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_LINEARACCELBODY) ? static_cast<double>(obs->attitudeOutputs->linearAccelBody(0)) : std::nan(""));
    addData(pinIndex, i++, obs->attitudeOutputs && (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_LINEARACCELBODY) ? static_cast<double>(obs->attitudeOutputs->linearAccelBody(1)) : std::nan(""));
    addData(pinIndex, i++, obs->attitudeOutputs && (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_LINEARACCELBODY) ? static_cast<double>(obs->attitudeOutputs->linearAccelBody(2)) : std::nan(""));
    addData(pinIndex, i++, obs->attitudeOutputs && (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_LINEARACCELNED) ? static_cast<double>(obs->attitudeOutputs->linearAccelNed(0)) : std::nan(""));
    addData(pinIndex, i++, obs->attitudeOutputs && (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_LINEARACCELNED) ? static_cast<double>(obs->attitudeOutputs->linearAccelNed(1)) : std::nan(""));
    addData(pinIndex, i++, obs->attitudeOutputs && (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_LINEARACCELNED) ? static_cast<double>(obs->attitudeOutputs->linearAccelNed(2)) : std::nan(""));
    addData(pinIndex, i++, obs->attitudeOutputs && (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YPRU) ? static_cast<double>(obs->attitudeOutputs->yprU(0)) : std::nan(""));
    addData(pinIndex, i++, obs->attitudeOutputs && (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YPRU) ? static_cast<double>(obs->attitudeOutputs->yprU(1)) : std::nan(""));
    addData(pinIndex, i++, obs->attitudeOutputs && (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YPRU) ? static_cast<double>(obs->attitudeOutputs->yprU(2)) : std::nan(""));
    // Group 6 (INS)
    addData(pinIndex, i++, obs->insOutputs && (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_INSSTATUS) ? static_cast<double>(obs->insOutputs->insStatus.mode()) : std::nan(""));
    addData(pinIndex, i++, obs->insOutputs && (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_INSSTATUS) ? static_cast<double>(obs->insOutputs->insStatus.gpsFix()) : std::nan(""));
    addData(pinIndex, i++, obs->insOutputs && (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_INSSTATUS) ? static_cast<double>(obs->insOutputs->insStatus.errorIMU()) : std::nan(""));
    addData(pinIndex, i++, obs->insOutputs && (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_INSSTATUS) ? static_cast<double>(obs->insOutputs->insStatus.errorMagPres()) : std::nan(""));
    addData(pinIndex, i++, obs->insOutputs && (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_INSSTATUS) ? static_cast<double>(obs->insOutputs->insStatus.errorGnss()) : std::nan(""));
    addData(pinIndex, i++, obs->insOutputs && (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_INSSTATUS) ? static_cast<double>(obs->insOutputs->insStatus.gpsHeadingIns()) : std::nan(""));
    addData(pinIndex, i++, obs->insOutputs && (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_INSSTATUS) ? static_cast<double>(obs->insOutputs->insStatus.gpsCompass()) : std::nan(""));
    addData(pinIndex, i++, obs->insOutputs && (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSLLA) ? obs->insOutputs->posLla(0) : std::nan(""));
    addData(pinIndex, i++, obs->insOutputs && (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSLLA) ? obs->insOutputs->posLla(1) : std::nan(""));
    addData(pinIndex, i++, obs->insOutputs && (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSLLA) ? obs->insOutputs->posLla(2) : std::nan(""));
    addData(pinIndex, i++, obs->insOutputs && (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSECEF) ? obs->insOutputs->posEcef(0) : std::nan(""));
    addData(pinIndex, i++, obs->insOutputs && (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSECEF) ? obs->insOutputs->posEcef(1) : std::nan(""));
    addData(pinIndex, i++, obs->insOutputs && (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSECEF) ? obs->insOutputs->posEcef(2) : std::nan(""));
    addData(pinIndex, i++, obs->insOutputs && (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELBODY) ? static_cast<double>(obs->insOutputs->velBody(0)) : std::nan(""));
    addData(pinIndex, i++, obs->insOutputs && (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELBODY) ? static_cast<double>(obs->insOutputs->velBody(1)) : std::nan(""));
    addData(pinIndex, i++, obs->insOutputs && (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELBODY) ? static_cast<double>(obs->insOutputs->velBody(2)) : std::nan(""));
    addData(pinIndex, i++, obs->insOutputs && (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELNED) ? static_cast<double>(obs->insOutputs->velNed(0)) : std::nan(""));
    addData(pinIndex, i++, obs->insOutputs && (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELNED) ? static_cast<double>(obs->insOutputs->velNed(1)) : std::nan(""));
    addData(pinIndex, i++, obs->insOutputs && (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELNED) ? static_cast<double>(obs->insOutputs->velNed(2)) : std::nan(""));
    addData(pinIndex, i++, obs->insOutputs && (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELECEF) ? static_cast<double>(obs->insOutputs->velEcef(0)) : std::nan(""));
    addData(pinIndex, i++, obs->insOutputs && (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELECEF) ? static_cast<double>(obs->insOutputs->velEcef(1)) : std::nan(""));
    addData(pinIndex, i++, obs->insOutputs && (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELECEF) ? static_cast<double>(obs->insOutputs->velEcef(2)) : std::nan(""));
    addData(pinIndex, i++, obs->insOutputs && (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_MAGECEF) ? static_cast<double>(obs->insOutputs->magEcef(0)) : std::nan(""));
    addData(pinIndex, i++, obs->insOutputs && (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_MAGECEF) ? static_cast<double>(obs->insOutputs->magEcef(1)) : std::nan(""));
    addData(pinIndex, i++, obs->insOutputs && (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_MAGECEF) ? static_cast<double>(obs->insOutputs->magEcef(2)) : std::nan(""));
    addData(pinIndex, i++, obs->insOutputs && (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_ACCELECEF) ? static_cast<double>(obs->insOutputs->accelEcef(0)) : std::nan(""));
    addData(pinIndex, i++, obs->insOutputs && (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_ACCELECEF) ? static_cast<double>(obs->insOutputs->accelEcef(1)) : std::nan(""));
    addData(pinIndex, i++, obs->insOutputs && (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_ACCELECEF) ? static_cast<double>(obs->insOutputs->accelEcef(2)) : std::nan(""));
    addData(pinIndex, i++, obs->insOutputs && (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_LINEARACCELECEF) ? static_cast<double>(obs->insOutputs->linearAccelEcef(0)) : std::nan(""));
    addData(pinIndex, i++, obs->insOutputs && (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_LINEARACCELECEF) ? static_cast<double>(obs->insOutputs->linearAccelEcef(1)) : std::nan(""));
    addData(pinIndex, i++, obs->insOutputs && (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_LINEARACCELECEF) ? static_cast<double>(obs->insOutputs->linearAccelEcef(2)) : std::nan(""));
    addData(pinIndex, i++, obs->insOutputs && (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSU) ? static_cast<double>(obs->insOutputs->posU) : std::nan(""));
    addData(pinIndex, i++, obs->insOutputs && (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELU) ? static_cast<double>(obs->insOutputs->velU) : std::nan(""));
    // Group 7 (GNSS2)
    addData(pinIndex, i++, obs->gnss2Outputs && (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC) ? static_cast<double>(obs->gnss2Outputs->timeUtc.year) : std::nan(""));
    addData(pinIndex, i++, obs->gnss2Outputs && (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC) ? static_cast<double>(obs->gnss2Outputs->timeUtc.month) : std::nan(""));
    addData(pinIndex, i++, obs->gnss2Outputs && (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC) ? static_cast<double>(obs->gnss2Outputs->timeUtc.day) : std::nan(""));
    addData(pinIndex, i++, obs->gnss2Outputs && (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC) ? static_cast<double>(obs->gnss2Outputs->timeUtc.hour) : std::nan(""));
    addData(pinIndex, i++, obs->gnss2Outputs && (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC) ? static_cast<double>(obs->gnss2Outputs->timeUtc.min) : std::nan(""));
    addData(pinIndex, i++, obs->gnss2Outputs && (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC) ? static_cast<double>(obs->gnss2Outputs->timeUtc.sec) : std::nan(""));
    addData(pinIndex, i++, obs->gnss2Outputs && (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC) ? static_cast<double>(obs->gnss2Outputs->timeUtc.ms) : std::nan(""));
    addData(pinIndex, i++, obs->gnss2Outputs && (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TOW) ? static_cast<double>(obs->gnss2Outputs->tow) : std::nan(""));
    addData(pinIndex, i++, obs->gnss2Outputs && (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_WEEK) ? static_cast<double>(obs->gnss2Outputs->week) : std::nan(""));
    addData(pinIndex, i++, obs->gnss2Outputs && (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_NUMSATS) ? static_cast<double>(obs->gnss2Outputs->numSats) : std::nan(""));
    addData(pinIndex, i++, obs->gnss2Outputs && (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_FIX) ? static_cast<double>(obs->gnss2Outputs->fix) : std::nan(""));
    addData(pinIndex, i++, obs->gnss2Outputs && (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSLLA) ? obs->gnss2Outputs->posLla(0) : std::nan(""));
    addData(pinIndex, i++, obs->gnss2Outputs && (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSLLA) ? obs->gnss2Outputs->posLla(1) : std::nan(""));
    addData(pinIndex, i++, obs->gnss2Outputs && (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSLLA) ? obs->gnss2Outputs->posLla(2) : std::nan(""));
    addData(pinIndex, i++, obs->gnss2Outputs && (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSECEF) ? obs->gnss2Outputs->posEcef(0) : std::nan(""));
    addData(pinIndex, i++, obs->gnss2Outputs && (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSECEF) ? obs->gnss2Outputs->posEcef(1) : std::nan(""));
    addData(pinIndex, i++, obs->gnss2Outputs && (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSECEF) ? obs->gnss2Outputs->posEcef(2) : std::nan(""));
    addData(pinIndex, i++, obs->gnss2Outputs && (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELNED) ? static_cast<double>(obs->gnss2Outputs->velNed(0)) : std::nan(""));
    addData(pinIndex, i++, obs->gnss2Outputs && (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELNED) ? static_cast<double>(obs->gnss2Outputs->velNed(1)) : std::nan(""));
    addData(pinIndex, i++, obs->gnss2Outputs && (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELNED) ? static_cast<double>(obs->gnss2Outputs->velNed(2)) : std::nan(""));
    addData(pinIndex, i++, obs->gnss2Outputs && (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELECEF) ? static_cast<double>(obs->gnss2Outputs->velEcef(0)) : std::nan(""));
    addData(pinIndex, i++, obs->gnss2Outputs && (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELECEF) ? static_cast<double>(obs->gnss2Outputs->velEcef(1)) : std::nan(""));
    addData(pinIndex, i++, obs->gnss2Outputs && (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELECEF) ? static_cast<double>(obs->gnss2Outputs->velEcef(2)) : std::nan(""));
    addData(pinIndex, i++, obs->gnss2Outputs && (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSU) ? static_cast<double>(obs->gnss2Outputs->posU(0)) : std::nan(""));
    addData(pinIndex, i++, obs->gnss2Outputs && (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSU) ? static_cast<double>(obs->gnss2Outputs->posU(1)) : std::nan(""));
    addData(pinIndex, i++, obs->gnss2Outputs && (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSU) ? static_cast<double>(obs->gnss2Outputs->posU(2)) : std::nan(""));
    addData(pinIndex, i++, obs->gnss2Outputs && (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELU) ? static_cast<double>(obs->gnss2Outputs->velU) : std::nan(""));
    addData(pinIndex, i++, obs->gnss2Outputs && (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEU) ? static_cast<double>(obs->gnss2Outputs->timeU) : std::nan(""));
    addData(pinIndex, i++, obs->gnss2Outputs && (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO) ? static_cast<double>(obs->gnss2Outputs->timeInfo.status.timeOk()) : std::nan(""));
    addData(pinIndex, i++, obs->gnss2Outputs && (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO) ? static_cast<double>(obs->gnss2Outputs->timeInfo.status.dateOk()) : std::nan(""));
    addData(pinIndex, i++, obs->gnss2Outputs && (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO) ? static_cast<double>(obs->gnss2Outputs->timeInfo.status.utcTimeValid()) : std::nan(""));
    addData(pinIndex, i++, obs->gnss2Outputs && (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO) ? static_cast<double>(obs->gnss2Outputs->timeInfo.leapSeconds) : std::nan(""));
    addData(pinIndex, i++, obs->gnss2Outputs && (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP) ? static_cast<double>(obs->gnss2Outputs->dop.gDop) : std::nan(""));
    addData(pinIndex, i++, obs->gnss2Outputs && (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP) ? static_cast<double>(obs->gnss2Outputs->dop.pDop) : std::nan(""));
    addData(pinIndex, i++, obs->gnss2Outputs && (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP) ? static_cast<double>(obs->gnss2Outputs->dop.tDop) : std::nan(""));
    addData(pinIndex, i++, obs->gnss2Outputs && (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP) ? static_cast<double>(obs->gnss2Outputs->dop.vDop) : std::nan(""));
    addData(pinIndex, i++, obs->gnss2Outputs && (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP) ? static_cast<double>(obs->gnss2Outputs->dop.hDop) : std::nan(""));
    addData(pinIndex, i++, obs->gnss2Outputs && (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP) ? static_cast<double>(obs->gnss2Outputs->dop.nDop) : std::nan(""));
    addData(pinIndex, i++, obs->gnss2Outputs && (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP) ? static_cast<double>(obs->gnss2Outputs->dop.eDop) : std::nan(""));
    addData(pinIndex, i++, obs->gnss2Outputs && (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_SATINFO) ? static_cast<double>(obs->gnss2Outputs->satInfo.numSats) : std::nan(""));
    addData(pinIndex, i++, obs->gnss2Outputs && (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS) ? obs->gnss2Outputs->raw.tow : std::nan(""));
    addData(pinIndex, i++, obs->gnss2Outputs && (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS) ? static_cast<double>(obs->gnss2Outputs->raw.week) : std::nan(""));
    addData(pinIndex, i++, obs->gnss2Outputs && (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS) ? static_cast<double>(obs->gnss2Outputs->raw.numSats) : std::nan(""));
}