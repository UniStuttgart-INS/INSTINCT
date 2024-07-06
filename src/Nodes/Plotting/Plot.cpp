// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Plot.hpp"
#include <array>
#include <cmath>
#include <cstddef>
#include <imgui.h>
#include <implot.h>
#include <utility>

#include "util/Logger.hpp"

#include "internal/NodeManager.hpp"
#include <fmt/core.h>
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"
#include "internal/ConfigManager.hpp"
#include "NodeRegistry.hpp"

#include "internal/gui/NodeEditorApplication.hpp"
#include "internal/gui/widgets/HelpMarker.hpp"
#include "internal/gui/widgets/Splitter.hpp"
#include "internal/gui/widgets/imgui_ex.hpp"
#include "util/Json.hpp"
#include "util/StringUtil.hpp"
#include "util/Container/STL.hpp"

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
    if (j.contains("displayName")) { j.at("displayName").get_to(data.displayName); }
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
    if (j.contains("dataIdentifier")) { j.at("dataIdentifier").get_to(data.dataIdentifier); }
    if (j.contains("size")) { j.at("size").get_to(data.size); }
    if (j.contains("plotData") && j.at("plotData").is_array())
    {
        j.at("plotData").get_to(data.plotData);
        for (auto& plotData : data.plotData)
        {
            plotData.buffer = ScrollingBuffer<double>(static_cast<size_t>(data.size));
        }
    }
    if (j.contains("pinType")) { j.at("pinType").get_to(data.pinType); }
    if (j.contains("stride")) { j.at("stride").get_to(data.stride); }
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
    if (style.lineFlags) { j["lineFlags"] = style.lineFlags.value(); }
}
/// @brief Read info from a json object
/// @param[in] j Json variable to read info from
/// @param[out] style Output object
void from_json(const json& j, Plot::PlotInfo::PlotItem::Style& style)
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
    if (j.contains("pinIndex")) { j.at("pinIndex").get_to(data.pinIndex); }
    if (j.contains("dataIndex")) { j.at("dataIndex").get_to(data.dataIndex); }
    if (j.contains("displayName")) { j.at("displayName").get_to(data.displayName); }
    if (j.contains("axis")) { j.at("axis").get_to(data.axis); }
    if (j.contains("style")) { j.at("style").get_to(data.style); }
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
        { "lineFlags", data.lineFlags },
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
    if (j.contains("size")) { j.at("size").get_to(data.size); }
    if (j.contains("xAxisFlags")) { j.at("xAxisFlags").get_to(data.xAxisFlags); }
    if (j.contains("yAxisFlags")) { j.at("yAxisFlags").get_to(data.yAxisFlags); }
    if (j.contains("xAxisScale")) { j.at("xAxisScale").get_to(data.xAxisScale); }
    if (j.contains("yAxesScale")) { j.at("yAxesScale").get_to(data.yAxesScale); }
    if (j.contains("lineFlags")) { j.at("lineFlags").get_to(data.lineFlags); }
    if (j.contains("headerText")) { j.at("headerText").get_to(data.headerText); }
    if (j.contains("leftPaneWidth")) { j.at("leftPaneWidth").get_to(data.leftPaneWidth); }
    if (j.contains("plotFlags")) { j.at("plotFlags").get_to(data.plotFlags); }
    if (j.contains("rightPaneWidth")) { j.at("rightPaneWidth").get_to(data.rightPaneWidth); }
    if (j.contains("selectedPin")) { j.at("selectedPin").get_to(data.selectedPin); }
    if (j.contains("selectedXdata")) { j.at("selectedXdata").get_to(data.selectedXdata); }
    if (j.contains("plotItems")) { j.at("plotItems").get_to(data.plotItems); }
    if (j.contains("title")) { j.at("title").get_to(data.title); }
    if (j.contains("overrideXAxisLabel")) { j.at("overrideXAxisLabel").get_to(data.overrideXAxisLabel); }
    if (j.contains("xAxisLabel")) { j.at("xAxisLabel").get_to(data.xAxisLabel); }
    if (j.contains("y1AxisLabel")) { j.at("y1AxisLabel").get_to(data.y1AxisLabel); }
    if (j.contains("y2AxisLabel")) { j.at("y2AxisLabel").get_to(data.y2AxisLabel); }
    if (j.contains("y3AxisLabel")) { j.at("y3AxisLabel").get_to(data.y3AxisLabel); }
}

} // namespace NAV

NAV::Plot::PinData::PlotData::PlotData(std::string displayName, size_t size)
    : displayName(std::move(displayName)), buffer(size) {}

NAV::Plot::PinData::PinData(const PinData& other)
    : size(other.size),
      dataIdentifier(other.dataIdentifier),
      plotData(other.plotData),
      pinType(other.pinType),
      stride(other.stride) {}

NAV::Plot::PinData::PinData(PinData&& other) noexcept
    : size(other.size),
      dataIdentifier(std::move(other.dataIdentifier)),
      plotData(std::move(other.plotData)),
      pinType(other.pinType),
      stride(other.stride) {}

NAV::Plot::PinData& NAV::Plot::PinData::operator=(const PinData& rhs)
{
    if (&rhs != this)
    {
        size = rhs.size;
        dataIdentifier = rhs.dataIdentifier;
        plotData = rhs.plotData;
        pinType = rhs.pinType;
        stride = rhs.stride;
    }

    return *this;
}

NAV::Plot::PinData& NAV::Plot::PinData::operator=(PinData&& rhs) noexcept
{
    if (&rhs != this)
    {
        size = rhs.size;
        dataIdentifier = std::move(rhs.dataIdentifier);
        plotData = std::move(rhs.plotData);
        pinType = rhs.pinType;
        stride = rhs.stride;
    }

    return *this;
}

void NAV::Plot::PinData::addPlotDataItem(size_t dataIndex, const std::string& displayName)
{
    if (plotData.size() > dataIndex)
    {
        if (plotData.at(dataIndex).displayName == displayName) // Item was restored already at this position
        {
            plotData.at(dataIndex).markedForDelete = false;
            return;
        }

        // Some other item was restored at this position
        if (!plotData.at(dataIndex).markedForDelete)
        {
            LOG_WARN("Adding PlotData item '{}' at position {}, but at this position exists already the item '{}'. Reordering the items to match the data. Consider resaving the flow file.",
                     displayName, dataIndex, plotData.at(dataIndex).displayName);
        }
        auto searchIter = std::find_if(plotData.begin(),
                                       plotData.end(),
                                       [displayName](const PlotData& plotData) { return plotData.displayName == displayName; });
        auto iter = plotData.begin();
        std::advance(iter, dataIndex);
        iter->markedForDelete = false;
        if (searchIter == plotData.end()) // Item does not exist yet. Developer added a new item to the list
        {
            plotData.insert(iter, PlotData{ displayName, static_cast<size_t>(size) });
        }
        else // Item exists already. Developer reordered the items in the list
        {
            move(plotData, static_cast<size_t>(searchIter - plotData.begin()), dataIndex);
        }
    }
    else if (std::find_if(plotData.begin(),
                          plotData.end(),
                          [displayName](const PlotData& plotData) { return plotData.displayName == displayName; })
             != plotData.end())
    {
        LOG_ERROR("Adding the PlotData item {} at position {}, but this plot item was found at another position already",
                  displayName, dataIndex);
    }
    else // Item not there yet. Add to the end of the list
    {
        plotData.emplace_back(displayName, static_cast<size_t>(size));
    }
}

// ###########################################################################################################

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
    inputPins.at(0).callback = static_cast<InputPin::FlowFirableCallbackFunc>(&Plot::plotFlowData);
    // // PinData::PinType::Bool:
    // _pinData.at(1).pinType = PinData::PinType::Bool;
    // inputPins.at(1).type = Pin::Type::Bool;
    // inputPins.at(1).dataIdentifier.clear();
    // inputPins.at(1).callback = static_cast<InputPin::DataChangedNotifyFunc>(&Plot::plotBoolean);
    // // PinData::PinType::Int:
    // _pinData.at(2).pinType = PinData::PinType::Int;
    // inputPins.at(2).type = Pin::Type::Int;
    // inputPins.at(2).dataIdentifier.clear();
    // inputPins.at(2).callback = static_cast<InputPin::DataChangedNotifyFunc>(&Plot::plotInteger);
    // // PinData::PinType::Float:
    // _pinData.at(3).pinType = PinData::PinType::Float;
    // inputPins.at(3).type = Pin::Type::Float;
    // inputPins.at(3).dataIdentifier.clear();
    // inputPins.at(3).callback = static_cast<InputPin::DataChangedNotifyFunc>(&Plot::plotFloat);
    // // PinData::PinType::Matrix:
    // _pinData.at(4).pinType = PinData::PinType::Matrix;
    // inputPins.at(4).type = Pin::Type::Matrix;
    // inputPins.at(4).dataIdentifier = { "Eigen::MatrixXd", "Eigen::VectorXd" };
    // inputPins.at(4).callback = static_cast<InputPin::DataChangedNotifyFunc>(&Plot::plotMatrix);
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
                    inputPins.at(pinIndex).callback = static_cast<InputPin::FlowFirableCallbackFunc>(&Plot::plotFlowData);
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

            bool saveForceXaxisRange = false;
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
                                        if (plotDataIndex == GPST_PLOT_IDX) // GPST Time
                                        {
                                            // Set all data to plot over GPST Time
                                            for (auto& selectedX : plot.selectedXdata)
                                            {
                                                selectedX = plotDataIndex;
                                            }
                                        }
                                        else
                                        {
                                            // Remove all GPST Time on the x axis
                                            for (auto& selectedX : plot.selectedXdata)
                                            {
                                                if (selectedX == GPST_PLOT_IDX) { selectedX = 0; }
                                            }
                                        }

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
                ImGui::SameLine();
                if (ImGui::Button(fmt::format("Same X range all plots##{} - {}", size_t(id), plotIdx).c_str()))
                {
                    saveForceXaxisRange = true;
                    _forceXaxisRange.first.clear();
                    size_t pinIdx = plot.plotItems.empty() ? 0 : plot.plotItems.front().pinIndex;
                    const auto& xName = _pinData.at(pinIdx).plotData.at(plot.selectedXdata.at(pinIdx)).displayName;
                    for (size_t p = 0; p < _plots.size(); p++)
                    {
                        if (p == plotIdx) { continue; }
                        auto& plot = _plots.at(p);
                        size_t pinIdx = plot.plotItems.empty() ? 0 : plot.plotItems.front().pinIndex;
                        const auto& dispName = _pinData.at(pinIdx).plotData.at(plot.selectedXdata.at(pinIdx)).displayName;
                        if (xName == dispName) { _forceXaxisRange.first.insert(p); }
                    }
                }

                auto axisScaleCombo = [&](const char* label, ImPlotScale& axisScale) {
                    auto getImPlotScaleString = [](ImPlotScale scale) {
                        switch (scale)
                        {
                        case ImPlotScale_Linear: // default linear scale
                            return "Linear";
                        // case ImPlotScale_Time: // date/time scale
                        //     return "Time";
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
                            if (n == ImPlotScale_Time) { continue; }
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

                ImGui::SameLine();
                if (ImGui::CheckboxFlags(fmt::format("NoClip##LineFlags {} - {}", size_t(id), plotIdx).c_str(), &plot.lineFlags, ImPlotLineFlags_NoClip))
                {
                    flow::ApplyChanges();
                }
                if (ImGui::IsItemHovered()) { ImGui::SetTooltip("Markers (if displayed) on the edge of a plot will not be clipped"); }
                ImGui::SameLine();
                if (ImGui::CheckboxFlags(fmt::format("SkipNaN##LineFlags {} - {}", size_t(id), plotIdx).c_str(), &plot.lineFlags, ImPlotLineFlags_SkipNaN))
                {
                    flow::ApplyChanges();
                }
                if (ImGui::IsItemHovered()) { ImGui::SetTooltip("NaNs values will be skipped instead of rendered as missing data"); }
                ImGui::SameLine();
                if (ImGui::CheckboxFlags(fmt::format("Loop##LineFlags {} - {}", size_t(id), plotIdx).c_str(), &plot.lineFlags, ImPlotLineFlags_Loop))
                {
                    flow::ApplyChanges();
                }
                if (ImGui::IsItemHovered()) { ImGui::SetTooltip("The last and first point will be connected to form a closed loop"); }

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

                        if (!label.empty())
                        {
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

            bool timeScaleXaxis = false;
            std::array<double, 2> timeAxisMinMax = { std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity() };
            for (auto& plotItem : plot.plotItems)
            {
                if (plot.selectedXdata.at(plotItem.pinIndex) == GPST_PLOT_IDX)
                {
                    auto& pinData = _pinData.at(plotItem.pinIndex);
                    const auto& plotDataX = pinData.plotData.at(plot.selectedXdata.at(plotItem.pinIndex));

                    // Lock the buffer so no data can be inserted
                    std::scoped_lock<std::mutex> guard(pinData.mutex);
                    if (!plotDataX.buffer.empty())
                    {
                        timeScaleXaxis = true;
                        timeAxisMinMax[0] = std::min(timeAxisMinMax[0], plotDataX.buffer.front());
                        timeAxisMinMax[1] = std::max(timeAxisMinMax[1], plotDataX.buffer.back());
                    }
                }
            }

            if (ImPlot::BeginPlot(fmt::format("{}##{} - {}", plot.title, size_t(id), plotIdx).c_str(), plot.size, plot.plotFlags))
            {
                if (saveForceXaxisRange)
                {
                    std::get<ImPlotRange>(_forceXaxisRange) = ImPlot::GetCurrentPlot()->XAxis(ImAxis_X1).Range;
                }
                else if (_forceXaxisRange.first.contains(plotIdx))
                {
                    if (plot.xAxisFlags & ImPlotAxisFlags_AutoFit)
                    {
                        plot.xAxisFlags &= ~ImPlotAxisFlags_AutoFit;
                        flow::ApplyChanges();
                    }
                    ImPlot::GetCurrentPlot()->XAxis(ImAxis_X1).SetRange(std::get<ImPlotRange>(_forceXaxisRange));
                    _forceXaxisRange.first.erase(plotIdx);
                }

                ImPlot::SetupAxis(ImAxis_X1, xLabel, plot.xAxisFlags);
                ImPlot::SetupAxisScale(ImAxis_X1, timeScaleXaxis ? ImPlotScale_Time : plot.xAxisScale);
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
                                                if ((std::get<3>(e) == -1 || static_cast<size_t>(std::get<3>(e)) == plotItem.dataIndex)
                                                    && (plotItem.style.eventTooltipFilterRegex.empty() || std::regex_search(std::get<2>(e), filter)))
                                                {
                                                    tooltip.texts.push_back(std::get<2>(e));
                                                }
                                            }
                                        }
                                    }
                                    catch (...) // NOLINT(bugprone-empty-catch)
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
                                             plotItem.style.lineFlags.value_or(plot.lineFlags),
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
                                                plotItem.style.lineFlags.value_or(plot.lineFlags) & ImPlotLineFlags_NoClip ? ImPlotScatterFlags_NoClip : ImPlotScatterFlags_None,
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
                            ImPlotLineFlags lineFlags = plotItem.style.lineFlags.value_or(plot.lineFlags);
                            bool plotItemLineFlagsAuto = !plotItem.style.lineFlags.has_value();
                            if (plotItemLineFlagsAuto) { ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.8F); }
                            if (ImGui::CheckboxFlags("NoClip", &lineFlags, ImPlotLineFlags_NoClip))
                            {
                                plotItem.style.lineFlags = lineFlags;
                                flow::ApplyChanges();
                            }
                            if (ImGui::IsItemHovered()) { ImGui::SetTooltip("Markers (if displayed) on the edge of a plot will not be clipped"); }
                            ImGui::SameLine();
                            if (ImGui::CheckboxFlags("SkipNaN", &lineFlags, ImPlotLineFlags_SkipNaN))
                            {
                                plotItem.style.lineFlags = lineFlags;
                                flow::ApplyChanges();
                            }
                            if (ImGui::IsItemHovered()) { ImGui::SetTooltip("NaNs values will be skipped instead of rendered as missing data"); }
                            ImGui::SameLine();
                            if (ImGui::CheckboxFlags("Loop", &lineFlags, ImPlotLineFlags_Loop))
                            {
                                plotItem.style.lineFlags = lineFlags;
                                flow::ApplyChanges();
                            }
                            if (ImGui::IsItemHovered()) { ImGui::SetTooltip("The last and first point will be connected to form a closed loop"); }
                            if (plotItem.style.lineFlags)
                            {
                                ImGui::SameLine();
                                if (ImGui::Button("Auto##Line Flags"))
                                {
                                    plotItem.style.lineFlags.reset();
                                }
                            }
                            if (plotItemLineFlagsAuto) { ImGui::PopStyleVar(); }
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
                inputPins.at(inputPinIndex).callback = static_cast<InputPin::FlowFirableCallbackFunc>(&Plot::plotFlowData);
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

    CommonLog::initialize();

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

        // NodeData
        _pinData.at(pinIndex).addPlotDataItem(i++, "Time [s]");
        _pinData.at(pinIndex).addPlotDataItem(i++, "GPST Time");
        _pinData.at(pinIndex).addPlotDataItem(i++, "GPS time of week [s]");

        if (startPin.dataIdentifier.front() == Pos::type())
        {
            for (const auto& desc : Pos::GetStaticDataDescriptors()) { _pinData.at(pinIndex).addPlotDataItem(i++, desc); }
        }
        else if (startPin.dataIdentifier.front() == PosVel::type())
        {
            for (const auto& desc : PosVel::GetStaticDataDescriptors()) { _pinData.at(pinIndex).addPlotDataItem(i++, desc); }
        }
        else if (startPin.dataIdentifier.front() == PosVelAtt::type())
        {
            for (const auto& desc : PosVelAtt::GetStaticDataDescriptors()) { _pinData.at(pinIndex).addPlotDataItem(i++, desc); }
        }
        else if (startPin.dataIdentifier.front() == InsGnssLCKFSolution::type())
        {
            for (const auto& desc : InsGnssLCKFSolution::GetStaticDataDescriptors()) { _pinData.at(pinIndex).addPlotDataItem(i++, desc); }
        }
        else if (startPin.dataIdentifier.front() == InsGnssTCKFSolution::type())
        {
            for (const auto& desc : InsGnssTCKFSolution::GetStaticDataDescriptors()) { _pinData.at(pinIndex).addPlotDataItem(i++, desc); }
        }
        else if (startPin.dataIdentifier.front() == GnssCombination::type())
        {
            _pinData.at(pinIndex).dynamicDataStartIndex = static_cast<int>(i);
        }
        else if (startPin.dataIdentifier.front() == GnssObs::type())
        {
            _pinData.at(pinIndex).dynamicDataStartIndex = static_cast<int>(i);
        }
        else if (startPin.dataIdentifier.front() == SppSolution::type())
        {
            for (const auto& desc : SppSolution::GetStaticDataDescriptors()) { _pinData.at(pinIndex).addPlotDataItem(i++, desc); }
            _pinData.at(pinIndex).dynamicDataStartIndex = static_cast<int>(i);
        }
        else if (startPin.dataIdentifier.front() == RtklibPosObs::type())
        {
            for (const auto& desc : RtklibPosObs::GetStaticDataDescriptors()) { _pinData.at(pinIndex).addPlotDataItem(i++, desc); }
        }
        else if (startPin.dataIdentifier.front() == ImuObs::type())
        {
            for (const auto& desc : ImuObs::GetStaticDataDescriptors()) { _pinData.at(pinIndex).addPlotDataItem(i++, desc); }
        }
        else if (startPin.dataIdentifier.front() == ImuObsWDelta::type())
        {
            for (const auto& desc : ImuObsWDelta::GetStaticDataDescriptors()) { _pinData.at(pinIndex).addPlotDataItem(i++, desc); }
        }
        else if (startPin.dataIdentifier.front() == ImuObsSimulated::type())
        {
            for (const auto& desc : ImuObsSimulated::GetStaticDataDescriptors()) { _pinData.at(pinIndex).addPlotDataItem(i++, desc); }
        }
        else if (startPin.dataIdentifier.front() == KvhObs::type())
        {
            for (const auto& desc : KvhObs::GetStaticDataDescriptors()) { _pinData.at(pinIndex).addPlotDataItem(i++, desc); }
        }
        else if (startPin.dataIdentifier.front() == ImuObsWDelta::type())
        {
            for (const auto& desc : ImuObsWDelta::GetStaticDataDescriptors()) { _pinData.at(pinIndex).addPlotDataItem(i++, desc); }
        }
        else if (startPin.dataIdentifier.front() == VectorNavBinaryOutput::type())
        {
            for (const auto& desc : VectorNavBinaryOutput::GetStaticDataDescriptors()) { _pinData.at(pinIndex).addPlotDataItem(i++, desc); }
        }
    }
    else
    {
        _pinData.at(pinIndex).dataIdentifier = startPin.name;

        // NodeData
        _pinData.at(pinIndex).addPlotDataItem(i++, "Time [s]");
        _pinData.at(pinIndex).addPlotDataItem(i++, "GPST Time");
        _pinData.at(pinIndex).addPlotDataItem(i++, "GPS time of week [s]");

        if (inputPins.at(pinIndex).type == Pin::Type::Bool)
        {
            _pinData.at(pinIndex).addPlotDataItem(i++, "Boolean");
        }
        else if (inputPins.at(pinIndex).type == Pin::Type::Int)
        {
            _pinData.at(pinIndex).addPlotDataItem(i++, "Integer");
        }
        else if (inputPins.at(pinIndex).type == Pin::Type::Float)
        {
            _pinData.at(pinIndex).addPlotDataItem(i++, "Float");
        }
        else if (inputPins.at(pinIndex).type == Pin::Type::Matrix)
        {
            if (startPin.dataIdentifier.front() == "Eigen::MatrixXd")
            {
                if (auto matrix = getInputValue<Eigen::MatrixXd>(pinIndex))
                {
                    for (int row = 0; row < matrix->v->rows(); row++)
                    {
                        for (int col = 0; col < matrix->v->cols(); col++)
                        {
                            _pinData.at(pinIndex).addPlotDataItem(i++, fmt::format("{}, {}", row, col));
                        }
                    }
                }
            }
            else if (startPin.dataIdentifier.front() == "Eigen::VectorXd")
            {
                if (auto matrix = getInputValue<Eigen::VectorXd>(pinIndex))
                {
                    for (int row = 0; row < matrix->v->rows(); row++)
                    {
                        _pinData.at(pinIndex).addPlotDataItem(i++, fmt::format("{}", row));
                    }
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
            plot.selectedXdata.at(pinIndex) = 1;
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

    nm::CreateInputPin(node, fmt::format("Pin {}", node->inputPins.size() + 1).c_str(), Pin::Type::Flow, plotNode->_dataIdentifier, &Plot::plotFlowData);
    plotNode->_pinData.emplace_back();
    for (auto& plot : plotNode->_plots)
    {
        plot.selectedXdata.emplace_back(1);
    }
}

void NAV::Plot::pinDeleteCallback(Node* node, size_t pinIdx)
{
    auto* plotNode = static_cast<Plot*>(node); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)

    for (auto& plot : plotNode->_plots)
    {
        if (plot.selectedPin >= pinIdx)
        {
            plot.selectedPin -= 1;
        }
        for (int64_t plotItemIdx = 0; plotItemIdx < static_cast<int64_t>(plot.plotItems.size()); ++plotItemIdx)
        {
            auto& plotItem = plot.plotItems.at(static_cast<size_t>(plotItemIdx));

            if (plotItem.pinIndex == pinIdx) // The index we want to delete
            {
                plot.plotItems.erase(plot.plotItems.begin() + plotItemIdx);
                plotItemIdx -= 1;
            }
            else if (plotItem.pinIndex > pinIdx) // Index higher -> Decrement
            {
                plotItem.pinIndex -= 1;
            }
        }

        plot.selectedXdata.erase(std::next(plot.selectedXdata.begin(), static_cast<int64_t>(pinIdx)));
    }

    nm::DeleteInputPin(node->inputPins.at(pinIdx));
    plotNode->_pinData.erase(std::next(plotNode->_pinData.begin(), static_cast<int64_t>(pinIdx)));
}

void NAV::Plot::addEvent(size_t pinIndex, InsTime insTime, const std::string& text, int32_t dataIndex)
{
    if (!insTime.empty() && !_startTime.empty())
    {
        double relTime = static_cast<double>((insTime - _startTime).count());
        _pinData.at(pinIndex).events.emplace_back(relTime, insTime, text, dataIndex);
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

size_t NAV::Plot::addData(size_t pinIndex, std::string displayName, double value)
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
    auto dataIndex = static_cast<size_t>(plotData - pinData.plotData.begin());
    addData(pinIndex, dataIndex, value);
    return dataIndex;
}

NAV::CommonLog::LocalPosition NAV::Plot::calcLocalPosition(const Eigen::Vector3d& lla_position)
{
    if (!_originPosition)
    {
        _originPosition = { .frame = gui::widgets::PositionWithFrame::ReferenceFrame::ECEF,
                            .e_position = trafo::lla2ecef_WGS84(lla_position) };
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

    return { .northSouth = northSouth, .eastWest = eastWest };
}

void NAV::Plot::plotBoolean(const InsTime& insTime, size_t pinIdx)
{
    LOG_DATA("{}: Plotting boolean on pin '{}' with time {}", nameId(), inputPins[pinIdx].name, insTime.toYMDHMS());
    if (ConfigManager::Get<bool>("nogui"))
    {
        releaseInputValue(pinIdx);
        return;
    }

    if (auto value = getInputValue<bool>(pinIdx);
        value && !insTime.empty())
    {
        if (_startTime.empty()) { _startTime = insTime; }
        size_t i = 0;

        std::scoped_lock<std::mutex> guard(_pinData.at(pinIdx).mutex);

        // NodeData
        addData(pinIdx, i++, static_cast<double>((insTime - _startTime).count()));
        addData(pinIdx, i++, static_cast<double>(insTime.toUnixTime() + insTime.differenceToUTC(GPST)));
        addData(pinIdx, i++, static_cast<double>(insTime.toGPSweekTow().tow));
        // Boolean
        addData(pinIdx, i++, static_cast<double>(*value->v));
    }
}

void NAV::Plot::plotInteger(const InsTime& insTime, size_t pinIdx)
{
    LOG_DATA("{}: Plotting integer on pin '{}' with time {}", nameId(), inputPins[pinIdx].name, insTime.toYMDHMS());
    if (ConfigManager::Get<bool>("nogui"))
    {
        releaseInputValue(pinIdx);
        return;
    }

    if (auto value = getInputValue<int>(pinIdx);
        value && !insTime.empty())
    {
        if (_startTime.empty()) { _startTime = insTime; }
        size_t i = 0;

        std::scoped_lock<std::mutex> guard(_pinData.at(pinIdx).mutex);

        // NodeData
        addData(pinIdx, i++, static_cast<double>((insTime - _startTime).count()));
        addData(pinIdx, i++, static_cast<double>(insTime.toUnixTime() + insTime.differenceToUTC(GPST)));
        addData(pinIdx, i++, static_cast<double>(insTime.toGPSweekTow().tow));
        // Integer
        addData(pinIdx, i++, static_cast<double>(*value->v));
    }
}

void NAV::Plot::plotFloat(const InsTime& insTime, size_t pinIdx)
{
    LOG_DATA("{}: Plotting float on pin '{}' with time {}", nameId(), inputPins[pinIdx].name, insTime.toYMDHMS());
    if (ConfigManager::Get<bool>("nogui"))
    {
        releaseInputValue(pinIdx);
        return;
    }

    if (auto value = getInputValue<double>(pinIdx);
        value && !insTime.empty())
    {
        if (_startTime.empty()) { _startTime = insTime; }
        size_t i = 0;

        std::scoped_lock<std::mutex> guard(_pinData.at(pinIdx).mutex);

        // NodeData
        addData(pinIdx, i++, static_cast<double>((insTime - _startTime).count()));
        addData(pinIdx, i++, static_cast<double>(insTime.toUnixTime() + insTime.differenceToUTC(GPST)));
        addData(pinIdx, i++, static_cast<double>(insTime.toGPSweekTow().tow));
        // Double
        addData(pinIdx, i++, *value->v);
    }
}

void NAV::Plot::plotMatrix(const InsTime& insTime, size_t pinIdx)
{
    LOG_DATA("{}: Plotting matrix on pin '{}' with time {}", nameId(), inputPins[pinIdx].name, insTime.toYMDHMS());
    if (ConfigManager::Get<bool>("nogui"))
    {
        releaseInputValue(pinIdx);
        return;
    }

    if (auto* sourcePin = inputPins.at(pinIdx).link.getConnectedPin())
    {
        if (sourcePin->dataIdentifier.front() == "Eigen::MatrixXd")
        {
            if (auto value = getInputValue<Eigen::MatrixXd>(pinIdx);
                value && !insTime.empty())
            {
                if (_startTime.empty()) { _startTime = insTime; }
                size_t i = 0;

                std::scoped_lock<std::mutex> guard(_pinData.at(pinIdx).mutex);

                // NodeData
                addData(pinIdx, i++, static_cast<double>((insTime - _startTime).count()));
                addData(pinIdx, i++, static_cast<double>(insTime.toUnixTime() + insTime.differenceToUTC(GPST)));
                addData(pinIdx, i++, static_cast<double>(insTime.toGPSweekTow().tow));
                // Matrix
                for (int row = 0; row < value->v->rows(); row++)
                {
                    for (int col = 0; col < value->v->cols(); col++)
                    {
                        addData(pinIdx, i++, (*value->v)(row, col));
                    }
                }
            }
        }
        else if (sourcePin->dataIdentifier.front() == "Eigen::VectorXd")
        {
            if (auto value = getInputValue<Eigen::VectorXd>(pinIdx);
                value && !insTime.empty())
            {
                if (_startTime.empty()) { _startTime = insTime; }
                size_t i = 0;

                std::scoped_lock<std::mutex> guard(_pinData.at(pinIdx).mutex);

                // NodeData
                addData(pinIdx, i++, static_cast<double>((insTime - _startTime).count()));
                addData(pinIdx, i++, static_cast<double>(insTime.toUnixTime() + insTime.differenceToUTC(GPST)));
                addData(pinIdx, i++, static_cast<double>(insTime.toGPSweekTow().tow));
                // Vector
                for (int row = 0; row < value->v->rows(); row++)
                {
                    addData(pinIdx, i++, (*value->v)(row));
                }
            }
        }
        else
        {
            releaseInputValue(pinIdx);
        }
    }
}

void NAV::Plot::plotFlowData(NAV::InputPin::NodeDataQueue& queue, size_t pinIdx)
{
    auto nodeData = queue.extract_front();

    if (ConfigManager::Get<bool>("nogui")) { return; }

    LOG_DATA("{}: Plotting data on pin '{}' with time {} GPST", nameId(), inputPins[pinIdx].name, nodeData->insTime.toYMDHMS(GPST));

    std::scoped_lock<std::mutex> guard(_pinData.at(pinIdx).mutex);
    // NodeData
    size_t i = 0;
    addData(pinIdx, i++, CommonLog::calcTimeIntoRun(nodeData->insTime));
    addData(pinIdx, i++, static_cast<double>(nodeData->insTime.toUnixTime() + nodeData->insTime.differenceToUTC(GPST)));
    addData(pinIdx, i++, static_cast<double>(nodeData->insTime.toGPSweekTow(GPST).tow));

    if (auto* sourcePin = inputPins.at(pinIdx).link.getConnectedPin())
    {
        LOG_DATA("{}: Connected Pin data identifier: [{}]", nameId(), joinToString(sourcePin->dataIdentifier));
        // -------------------------------------------- General ----------------------------------------------
        if (sourcePin->dataIdentifier.front() == DynamicData::type())
        {
            plotDynamicData(std::static_pointer_cast<const DynamicData>(nodeData), pinIdx, i);
        }
        // --------------------------------------------- GNSS ------------------------------------------------
        else if (sourcePin->dataIdentifier.front() == GnssCombination::type())
        {
            plotGnssCombination(std::static_pointer_cast<const GnssCombination>(nodeData), pinIdx, i);
        }
        else if (sourcePin->dataIdentifier.front() == GnssObs::type())
        {
            plotGnssObs(std::static_pointer_cast<const GnssObs>(nodeData), pinIdx, i);
        }
        // ---------------------------------------------- IMU ------------------------------------------------
        else if (sourcePin->dataIdentifier.front() == ImuObs::type())
        {
            plotData(std::static_pointer_cast<const ImuObs>(nodeData), pinIdx, i);
        }
        else if (sourcePin->dataIdentifier.front() == ImuObsWDelta::type())
        {
            plotData(std::static_pointer_cast<const ImuObsWDelta>(nodeData), pinIdx, i);
        }
        else if (sourcePin->dataIdentifier.front() == ImuObsSimulated::type())
        {
            plotData(std::static_pointer_cast<const ImuObsSimulated>(nodeData), pinIdx, i);
        }
        else if (sourcePin->dataIdentifier.front() == KvhObs::type())
        {
            plotData(std::static_pointer_cast<const KvhObs>(nodeData), pinIdx, i);
        }
        else if (sourcePin->dataIdentifier.front() == ImuObsWDelta::type())
        {
            plotData(std::static_pointer_cast<const ImuObsWDelta>(nodeData), pinIdx, i);
        }
        else if (sourcePin->dataIdentifier.front() == VectorNavBinaryOutput::type())
        {
            plotData(std::static_pointer_cast<const VectorNavBinaryOutput>(nodeData), pinIdx, i);
        }
        // --------------------------------------------- State -----------------------------------------------
        else if (NAV::NodeRegistry::NodeDataTypeAnyIsChildOf(sourcePin->dataIdentifier, { Pos::type() }))
        {
            auto obs = std::static_pointer_cast<const Pos>(nodeData);
            auto localPosition = calcLocalPosition(obs->lla_position());

            for (size_t j = 0; j < Pos::GetStaticDescriptorCount(); ++j)
            {
                if (j == 3) { addData(pinIdx, i++, localPosition.northSouth); }
                else if (j == 4) { addData(pinIdx, i++, localPosition.eastWest); }
                else { addData(pinIdx, i++, obs->getValueAtOrNaN(j)); }
            }

            if (sourcePin->dataIdentifier.front() == PosVel::type())
            {
                plotData(std::static_pointer_cast<const PosVel>(nodeData), pinIdx, i, Pos::GetStaticDescriptorCount());
            }
            else if (sourcePin->dataIdentifier.front() == PosVelAtt::type())
            {
                plotData(std::static_pointer_cast<const PosVelAtt>(nodeData), pinIdx, i, Pos::GetStaticDescriptorCount());
            }
            else if (sourcePin->dataIdentifier.front() == InsGnssLCKFSolution::type())
            {
                plotData(std::static_pointer_cast<const InsGnssLCKFSolution>(nodeData), pinIdx, i, Pos::GetStaticDescriptorCount());
            }
            // ------------------------------------------- GNSS ----------------------------------------------
            else if (sourcePin->dataIdentifier.front() == SppSolution::type())
            {
                plotData(std::static_pointer_cast<const SppSolution>(nodeData), pinIdx, i, Pos::GetStaticDescriptorCount());
                plotSppSolutionDynamicData(std::static_pointer_cast<const SppSolution>(nodeData), pinIdx);
            }
            else if (sourcePin->dataIdentifier.front() == RtklibPosObs::type())
            {
                plotData(std::static_pointer_cast<const RtklibPosObs>(nodeData), pinIdx, i, Pos::GetStaticDescriptorCount());
            }
        }
        else if (sourcePin->dataIdentifier.front() == InsGnssTCKFSolution::type())
        {
            plotData(std::static_pointer_cast<const InsGnssTCKFSolution>(nodeData), pinIdx, i);
        }

        for (const auto& event : nodeData->events())
        {
            addEvent(pinIdx, nodeData->insTime, event, -1);
        }
    }
}

void NAV::Plot::plotDynamicData(const std::shared_ptr<const DynamicData>& obs, size_t pinIndex, size_t& plotIndex)
{
    plotData(obs, pinIndex, plotIndex);

    for (const auto& data : obs->data)
    {
        auto dataIndex = addData(pinIndex, data.description, data.value);

        for (const auto& event : data.events)
        {
            addEvent(pinIndex, obs->insTime, event, static_cast<int32_t>(dataIndex));
        }
    }
}

void NAV::Plot::plotGnssCombination(const std::shared_ptr<const GnssCombination>& obs, size_t pinIndex, size_t& plotIndex)
{
    plotData(obs, pinIndex, plotIndex);

    // Dynamic data
    for (const auto& comb : obs->combinations)
    {
        addData(pinIndex, comb.description, comb.result.value_or(std::nan("")));
        addData(pinIndex, comb.description + " Cycle Slip", comb.cycleSlipResult ? static_cast<double>(*comb.cycleSlipResult) : std::nan(""));
        addData(pinIndex, comb.description + " Prediction", comb.cycleSlipPrediction.value_or(std::nan("")));
        addData(pinIndex, comb.description + " Meas - Pred", comb.cycleSlipMeasMinPred.value_or(std::nan("")));
    }

    // TODO: KEEP THIS
    for (const auto& comb : obs->combinations)
    {
        for (const auto& [insTime, poly, value] : comb.cycleSlipPolynomials)
        {
            auto t = static_cast<double>((insTime - _startTime).count());
            addData(pinIndex, fmt::format("{} [{:.1f}] ({})", comb.description, t, poly.toString()), value);
        }
    }
}

void NAV::Plot::plotGnssObs(const std::shared_ptr<const GnssObs>& obs, size_t pinIndex, size_t& plotIndex)
{
    plotData(obs, pinIndex, plotIndex);

    // Dynamic data
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

void NAV::Plot::plotSppSolutionDynamicData(const std::shared_ptr<const SppSolution>& obs, size_t pinIndex)
{
    // Dynamic data
    for (const auto& bias : obs->interFrequencyBias)
    {
        addData(pinIndex, fmt::format("{} Inter-freq bias [s]", bias.first), bias.second.value);
        addData(pinIndex, fmt::format("{} Inter-freq bias StDev [s]", bias.first), bias.second.stdDev);
    }

    for (const auto& [satId, satData] : obs->satData)
    {
        addData(pinIndex, fmt::format("{} Elevation [deg]", satId), rad2deg(satData.satElevation));
        addData(pinIndex, fmt::format("{} Azimuth [deg]", satId), rad2deg(satData.satAzimuth));
        // addData(pinIndex, fmt::format("{} Satellite clock bias [s]", satData.first), satData.second.satClock.bias);
        // addData(pinIndex, fmt::format("{} Satellite clock drift [s/s]", satData.first), satData.second.satClock.drift);
        // addData(pinIndex, fmt::format("{} SatPos ECEF X [m]", satData.first), satData.second.e_satPos.x());
        // addData(pinIndex, fmt::format("{} SatPos ECEF Y [m]", satData.first), satData.second.e_satPos.y());
        // addData(pinIndex, fmt::format("{} SatPos ECEF Z [m]", satData.first), satData.second.e_satPos.z());
        // addData(pinIndex, fmt::format("{} SatPos Latitude [deg]", satData.first), rad2deg(satData.second.lla_satPos.x()));
        // addData(pinIndex, fmt::format("{} SatPos Longitude [deg]", satData.first), rad2deg(satData.second.lla_satPos.y()));
        // addData(pinIndex, fmt::format("{} SatPos Altitude [m]", satData.first), satData.second.lla_satPos.z());
        // addData(pinIndex, fmt::format("{} SatVel ECEF X [m/s]", satData.first), satData.second.e_satVel.x());
        // addData(pinIndex, fmt::format("{} SatVel ECEF Y [m/s]", satData.first), satData.second.e_satVel.y());
        // addData(pinIndex, fmt::format("{} SatVel ECEF Z [m/s]", satData.first), satData.second.e_satVel.z());
    }
}