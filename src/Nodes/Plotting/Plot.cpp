#include "Plot.hpp"

#include "util/Logger.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "internal/gui/widgets/Splitter.hpp"
#include "internal/Json.hpp"

#include "util/Time/TimeBase.hpp"
#include "util/InsTransformations.hpp"
#include "util/InsMath.hpp"
#include <algorithm>

namespace NAV
{
void to_json(json& j, const Plot::PlotStyle& style)
{
    j = json{
        { "legendName", style.legendName },
        { "stride", style.stride },
        { "lineType", style.lineType },
        { "color", style.color },
        { "thickness", style.thickness },
        { "markers", style.markers },
        { "markerStyle", style.markerStyle },
        { "markerSize", style.markerSize },
        { "markerWeight", style.markerWeight },
        { "markerFillColor", style.markerFillColor },
        { "markerOutlineColor", style.markerOutlineColor },
    };
}
void from_json(const json& j, Plot::PlotStyle& style)
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
    if (j.contains("thickness"))
    {
        j.at("thickness").get_to(style.thickness);
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
}

void to_json(json& j, const Plot::PinData::PlotData& data)
{
    j = json{
        { "plotOnAxisAndPlotStyle", data.plotOnAxis },
        { "displayName", data.displayName },
    };
}
void from_json(const json& j, Plot::PinData::PlotData& data)
{
    if (j.contains("plotOnAxisAndPlotStyle"))
    {
        j.at("plotOnAxisAndPlotStyle").get_to(data.plotOnAxis);
    }
    if (j.contains("displayName"))
    {
        j.at("displayName").get_to(data.displayName);
    }
}

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
    if (j.contains("plotData"))
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

void to_json(json& j, const Plot::PlotInfo& data)
{
    j = json{
        { "size", data.size },
        { "autoLimitXaxis", data.autoLimitXaxis },
        { "autoLimitYaxis", data.autoLimitYaxis },
        { "headerText", data.headerText },
        { "leftPaneWidth", data.leftPaneWidth },
        { "plotFlags", data.plotFlags },
        { "rightPaneWidth", data.rightPaneWidth },
        { "selectedPin", data.selectedPin },
        { "selectedXdata", data.selectedXdata },
        { "title", data.title },
        { "overrideXAxisLabel", data.overrideXAxisLabel },
        { "xAxisLabel", data.xAxisLabel },
        { "y1AxisLabel", data.y1AxisLabel },
        { "y2AxisLabel", data.y2AxisLabel },
        { "y3AxisLabel", data.y3AxisLabel },
    };
}
void from_json(const json& j, Plot::PlotInfo& data)
{
    if (j.contains("size"))
    {
        j.at("size").get_to(data.size);
    }
    if (j.contains("autoLimitXaxis"))
    {
        j.at("autoLimitXaxis").get_to(data.autoLimitXaxis);
    }
    if (j.contains("autoLimitYaxis"))
    {
        j.at("autoLimitYaxis").get_to(data.autoLimitYaxis);
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
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    hasConfig = true;
    guiConfigDefaultWindowSize = { 750, 650 };

    dataIdentifier = { PosVelAtt::type(), PVAError::type(), ImuBiases::type(),
                       RtklibPosObs::type(), UbloxObs::type(),
                       ImuObs::type(), KvhObs::type(), ImuObsWDelta::type(),
                       VectorNavBinaryOutput::type() };

    updateNumberOfInputPins();

    // PinData::PinType::Flow:
    data.at(0).pinType = PinData::PinType::Flow;
    inputPins.at(0).type = Pin::Type::Flow;
    inputPins.at(0).dataIdentifier = dataIdentifier;
    inputPins.at(0).data = Pin::PinData(static_cast<void (Node::*)(const std::shared_ptr<const NodeData>&, ax::NodeEditor::LinkId)>(&Plot::plotData));
    // PinData::PinType::Bool:
    data.at(1).pinType = PinData::PinType::Bool;
    inputPins.at(1).type = Pin::Type::Bool;
    inputPins.at(1).dataIdentifier.clear();
    inputPins.at(1).notifyFunc.emplace_back(this, static_cast<void (Node::*)(ax::NodeEditor::LinkId)>(&Plot::plotBoolean), 0);
    // PinData::PinType::Int:
    data.at(2).pinType = PinData::PinType::Int;
    inputPins.at(2).type = Pin::Type::Int;
    inputPins.at(2).dataIdentifier.clear();
    inputPins.at(2).notifyFunc.emplace_back(this, static_cast<void (Node::*)(ax::NodeEditor::LinkId)>(&Plot::plotInteger), 0);
    // PinData::PinType::Float:
    data.at(3).pinType = PinData::PinType::Float;
    inputPins.at(3).type = Pin::Type::Float;
    inputPins.at(3).dataIdentifier.clear();
    inputPins.at(3).notifyFunc.emplace_back(this, static_cast<void (Node::*)(ax::NodeEditor::LinkId)>(&Plot::plotFloat), 0);
    // PinData::PinType::Matrix:
    data.at(4).pinType = PinData::PinType::Matrix;
    inputPins.at(4).type = Pin::Type::Matrix;
    inputPins.at(4).dataIdentifier = { "Eigen::MatrixXd", "BlockMatrix" };
    inputPins.at(4).notifyFunc.emplace_back(this, static_cast<void (Node::*)(ax::NodeEditor::LinkId)>(&Plot::plotMatrix), 0);
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
    ImGui::SetNextItemOpen(true, ImGuiCond_Once);
    if (ImGui::CollapsingHeader(("Options##" + std::to_string(size_t(id))).c_str()))
    {
        if (ImGui::InputInt("# Input Pins", &nInputPins))
        {
            if (nInputPins < 1)
            {
                nInputPins = 1;
            }
            LOG_DEBUG("{}: # Input Pins changed to {}", nameId(), nInputPins);
            flow::ApplyChanges();
            updateNumberOfInputPins();
        }
        if (ImGui::InputInt("# Plots", &nPlots))
        {
            if (nPlots < 0)
            {
                nPlots = 0;
            }
            LOG_DEBUG("{}: # Plots changed to {}", nameId(), nPlots);
            flow::ApplyChanges();
            updateNumberOfPlots();
        }
        if (ImGui::BeginTable(("Pin Settings##" + std::to_string(size_t(id))).c_str(), 4,
                              ImGuiTableFlags_Borders | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_NoHostExtendX, ImVec2(0.0F, 0.0F)))
        {
            ImGui::TableSetupColumn("Pin");
            ImGui::TableSetupColumn("Pin Type");
            ImGui::TableSetupColumn("# Data Points");
            ImGui::TableSetupColumn("Stride");
            ImGui::TableHeadersRow();

            for (size_t pinIndex = 0; pinIndex < data.size(); pinIndex++)
            {
                auto& pinData = data.at(pinIndex);
                ImGui::TableNextRow();
                ImGui::TableNextColumn(); // Pin
                ImGui::Text("%zu - %s", pinIndex + 1, data.at(pinIndex).dataIdentifier.c_str());

                ImGui::TableNextColumn(); // Pin Type
                ImGui::SetNextItemWidth(100.0F);
                if (ImGui::Combo(("##Pin Type for Pin " + std::to_string(pinIndex + 1) + " - " + std::to_string(size_t(id))).c_str(),
                                 reinterpret_cast<int*>(&pinData.pinType), "Flow\0Bool\0Int\0Float\0Matrix\0\0"))
                {
                    if (Link* connectedLink = nm::FindConnectedLinkToInputPin(inputPins.at(pinIndex).id))
                    {
                        nm::DeleteLink(connectedLink->id);
                    }
                    inputPins.at(pinIndex).notifyFunc.clear();

                    switch (pinData.pinType)
                    {
                    case PinData::PinType::Flow:
                        inputPins.at(pinIndex).type = Pin::Type::Flow;
                        inputPins.at(pinIndex).dataIdentifier = dataIdentifier;
                        inputPins.at(pinIndex).data = Pin::PinData(static_cast<void (Node::*)(const std::shared_ptr<const NodeData>&, ax::NodeEditor::LinkId)>(&Plot::plotData));
                        break;
                    case PinData::PinType::Bool:
                        inputPins.at(pinIndex).type = Pin::Type::Bool;
                        inputPins.at(pinIndex).dataIdentifier.clear();
                        inputPins.at(pinIndex).notifyFunc.emplace_back(this, static_cast<void (Node::*)(ax::NodeEditor::LinkId)>(&Plot::plotBoolean), 0);
                        break;
                    case PinData::PinType::Int:
                        inputPins.at(pinIndex).type = Pin::Type::Int;
                        inputPins.at(pinIndex).dataIdentifier.clear();
                        inputPins.at(pinIndex).notifyFunc.emplace_back(this, static_cast<void (Node::*)(ax::NodeEditor::LinkId)>(&Plot::plotInteger), 0);
                        break;
                    case PinData::PinType::Float:
                        inputPins.at(pinIndex).type = Pin::Type::Float;
                        inputPins.at(pinIndex).dataIdentifier.clear();
                        inputPins.at(pinIndex).notifyFunc.emplace_back(this, static_cast<void (Node::*)(ax::NodeEditor::LinkId)>(&Plot::plotFloat), 0);
                        break;
                    case PinData::PinType::Matrix:
                        inputPins.at(pinIndex).type = Pin::Type::Matrix;
                        inputPins.at(pinIndex).dataIdentifier = { "Eigen::MatrixXd", "BlockMatrix" };
                        inputPins.at(pinIndex).notifyFunc.emplace_back(this, static_cast<void (Node::*)(ax::NodeEditor::LinkId)>(&Plot::plotMatrix), 0);
                        break;
                    }

                    flow::ApplyChanges();
                }

                ImGui::TableNextColumn(); // # Data Points
                ImGui::SetNextItemWidth(100.0F);
                if (ImGui::DragInt(("##Data Points" + std::to_string(size_t(id)) + " - " + std::to_string(pinIndex + 1)).c_str(),
                                   &pinData.size, 10.0F, 0, INT32_MAX / 2))
                {
                    if (pinData.size < 0)
                    {
                        pinData.size = 0;
                    }
                    for (auto& plotData : pinData.plotData)
                    {
                        flow::ApplyChanges();
                        plotData.buffer.resize(static_cast<size_t>(pinData.size));
                    }
                }
                if (ImGui::IsItemHovered())
                {
                    ImGui::SetTooltip("The amount of data which should be stored before the buffer gets reused.\nEnter 0 to show all data.");
                }

                ImGui::TableNextColumn(); // Stride
                ImGui::SetNextItemWidth(100.0F);
                if (ImGui::InputInt(("##Stride" + std::to_string(size_t(id)) + " - " + std::to_string(pinIndex + 1)).c_str(),
                                    &pinData.stride))
                {
                    if (pinData.stride < 1)
                    {
                        pinData.stride = 1;
                    }
                    flow::ApplyChanges();
                }
                if (ImGui::IsItemHovered())
                {
                    ImGui::SetTooltip("The amount of points to skip when plotting. This greatly reduces lag when plotting");
                }
            }

            ImGui::EndTable();
        }
    }

    for (size_t plotNum = 0; plotNum < static_cast<size_t>(nPlots); plotNum++)
    {
        auto& plotInfo = plotInfos.at(plotNum);
        ImGui::SetNextItemOpen(true, ImGuiCond_Once);
        if (ImGui::CollapsingHeader((plotInfo.headerText + "##" + std::to_string(size_t(id)) + " - " + std::to_string(plotNum)).c_str()))
        {
            ImGui::SetNextItemOpen(false, ImGuiCond_FirstUseEver);
            if (ImGui::TreeNode(("Options##" + std::to_string(size_t(id)) + " - " + std::to_string(plotNum)).c_str()))
            {
                ImGui::InputText(("Title##" + std::to_string(size_t(id)) + " - " + std::to_string(plotNum)).c_str(), &plotInfo.title);
                if (plotInfo.headerText != plotInfo.title && !ImGui::IsItemActive())
                {
                    plotInfo.headerText = plotInfo.title;
                    flow::ApplyChanges();
                    LOG_DEBUG("{}: # Header changed to {}", nameId(), plotInfo.headerText);
                }
                if (ImGui::SliderFloat(("Plot Height##" + std::to_string(size_t(id)) + " - " + std::to_string(plotNum)).c_str(), &plotInfo.size.y, 0.0F, 1000, "%.0f"))
                {
                    flow::ApplyChanges();
                }
                if (ImGui::Checkbox(("Override X Axis Label##" + std::to_string(size_t(id)) + " - " + std::to_string(plotNum)).c_str(), &plotInfo.overrideXAxisLabel))
                {
                    flow::ApplyChanges();
                }
                if (plotInfo.overrideXAxisLabel)
                {
                    if (ImGui::InputText(("X Axis Label##" + std::to_string(size_t(id)) + " - " + std::to_string(plotNum)).c_str(), &plotInfo.xAxisLabel))
                    {
                        flow::ApplyChanges();
                    }
                }
                if (ImGui::InputText(("Y1 Axis Label##" + std::to_string(size_t(id)) + " - " + std::to_string(plotNum)).c_str(), &plotInfo.y1AxisLabel))
                {
                    flow::ApplyChanges();
                }
                if (plotInfo.plotFlags & ImPlotFlags_YAxis2)
                {
                    if (ImGui::InputText(("Y2 Axis Label##" + std::to_string(size_t(id)) + " - " + std::to_string(plotNum)).c_str(), &plotInfo.y2AxisLabel))
                    {
                        flow::ApplyChanges();
                    }
                }
                if (plotInfo.plotFlags & ImPlotFlags_YAxis3)
                {
                    if (ImGui::InputText(("Y3 Axis Label##" + std::to_string(size_t(id)) + " - " + std::to_string(plotNum)).c_str(), &plotInfo.y3AxisLabel))
                    {
                        flow::ApplyChanges();
                    }
                }
                if (ImGui::BeginTable(("Pin Settings##" + std::to_string(size_t(id)) + " - " + std::to_string(plotNum)).c_str(), 2,
                                      ImGuiTableFlags_Borders | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_NoHostExtendX, ImVec2(0.0F, 0.0F)))
                {
                    ImGui::TableSetupColumn("Pin");
                    ImGui::TableSetupColumn("X Data");
                    ImGui::TableHeadersRow();

                    for (size_t pinIndex = 0; pinIndex < data.size(); pinIndex++)
                    {
                        ImGui::TableNextRow();
                        ImGui::TableNextColumn(); // Pin
                        ImGui::Text("%zu - %s", pinIndex + 1, data.at(pinIndex).dataIdentifier.c_str());

                        ImGui::TableNextColumn(); // X Data
                        auto& pinData = data.at(pinIndex);
                        if (!pinData.plotData.empty())
                        {
                            ImGui::SetNextItemWidth(200.0F);
                            if (ImGui::BeginCombo(("##X Data for Pin " + std::to_string(pinIndex + 1) + std::to_string(size_t(id)) + " - " + std::to_string(plotNum)).c_str(),
                                                  pinData.plotData.at(plotInfo.selectedXdata.at(pinIndex)).displayName.c_str()))
                            {
                                for (size_t plotDataIndex = 0; plotDataIndex < pinData.plotData.size(); plotDataIndex++)
                                {
                                    auto& plotData = pinData.plotData.at(plotDataIndex);

                                    if (!plotData.hasData)
                                    {
                                        ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5F);
                                    }
                                    const bool is_selected = (plotInfo.selectedXdata.at(pinIndex) == plotDataIndex);
                                    if (ImGui::Selectable(pinData.plotData.at(plotDataIndex).displayName.c_str(), is_selected))
                                    {
                                        flow::ApplyChanges();
                                        plotInfo.selectedXdata.at(pinIndex) = plotDataIndex;
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

                if (ImGui::CheckboxFlags(("Y-Axis 2##" + std::to_string(size_t(id)) + " - " + std::to_string(plotNum)).c_str(),
                                         &plotInfo.plotFlags, ImPlotFlags_YAxis2))
                {
                    flow::ApplyChanges();
                }
                ImGui::SameLine();
                if (ImGui::CheckboxFlags(("Y-Axis 3##" + std::to_string(size_t(id)) + " - " + std::to_string(plotNum)).c_str(),
                                         &plotInfo.plotFlags, ImPlotFlags_YAxis3))
                {
                    flow::ApplyChanges();
                }
                ImGui::SameLine();
                if (ImGui::Checkbox(("Auto Limit X-Axis##" + std::to_string(size_t(id)) + " - " + std::to_string(plotNum)).c_str(),
                                    &plotInfo.autoLimitXaxis))
                {
                    flow::ApplyChanges();
                }
                ImGui::SameLine();
                if (ImGui::Checkbox(("Auto Limit Y-Axis##" + std::to_string(size_t(id)) + " - " + std::to_string(plotNum)).c_str(),
                                    &plotInfo.autoLimitYaxis))
                {
                    flow::ApplyChanges();
                }

                ImGui::TreePop();
            }

            gui::widgets::Splitter((std::to_string(size_t(id)) + " - " + std::to_string(plotNum)).c_str(),
                                   true, 4.0F, &plotInfo.leftPaneWidth, &plotInfo.rightPaneWidth, 150.0F, 80.0F, plotInfo.size.y);

            ImGui::SetNextItemWidth(plotInfo.leftPaneWidth - 2.0F);
            ImGui::BeginGroup();
            if (ImGui::BeginCombo(("##" + std::to_string(size_t(id)) + " - " + std::to_string(plotNum)).c_str(),
                                  ("Pin " + std::to_string(plotInfo.selectedPin + 1)
                                   + " - " + data.at(static_cast<size_t>(plotInfo.selectedPin)).dataIdentifier)
                                      .c_str()))
            {
                for (int n = 0; n < nInputPins; n++)
                {
                    const bool is_selected = (plotInfo.selectedPin == n);
                    if (ImGui::Selectable(("Pin " + std::to_string(n + 1)
                                           + " - " + data.at(static_cast<size_t>(n)).dataIdentifier)
                                              .c_str(),
                                          is_selected, 0))
                    {
                        plotInfo.selectedPin = n;
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
            if (ImGui::Button(("Clear##" + std::to_string(size_t(id)) + " - " + std::to_string(plotNum)).c_str(), ImVec2(plotInfo.leftPaneWidth - 2.0F, 0)))
            {
                for (auto& pinData : data)
                {
                    for (auto& plotData : pinData.plotData)
                    {
                        if (plotData.plotOnAxis.contains(plotNum))
                        {
                            flow::ApplyChanges();
                        }
                        plotData.plotOnAxis.erase(plotNum);
                    }
                }
            }
            if (ImGui::BeginDragDropTarget())
            {
                if (const ImGuiPayload* payloadData = ImGui::AcceptDragDropPayload(("DND_DATA " + std::to_string(size_t(id)) + " - " + std::to_string(plotNum)).c_str()))
                {
                    auto* plotData = *static_cast<PinData::PlotData**>(payloadData->Data);
                    plotData->plotOnAxis.erase(plotNum);
                    flow::ApplyChanges();
                }
                ImGui::EndDragDropTarget();
            }
            auto buttonSize = ImGui::GetItemRectSize();
            ImGui::BeginChild(("Data Drag" + std::to_string(size_t(id)) + " - " + std::to_string(plotNum)).c_str(),
                              ImVec2(plotInfo.leftPaneWidth - 2.0F, plotInfo.size.y - comboBoxSize.y - buttonSize.y - 2 * ImGui::GetStyle().ItemSpacing.y),
                              true);

            // Left Data Selectables
            for (auto& plotData : data.at(static_cast<size_t>(plotInfo.selectedPin)).plotData)
            {
                auto plotDataHasData = plotData.hasData;
                if (!plotDataHasData)
                {
                    ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5F);
                }
                std::string label = plotData.displayName;
                if (plotData.plotOnAxis.contains(plotNum))
                {
                    label += fmt::format(" (Y{})", plotData.plotOnAxis.at(plotNum).first + 1);
                }
                ImGui::Selectable(label.c_str(), false, 0);
                if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_None))
                {
                    auto* ptrPlotData = &plotData;
                    ImGui::SetDragDropPayload(("DND_DATA " + std::to_string(size_t(id)) + " - " + std::to_string(plotNum)).c_str(),
                                              &ptrPlotData, sizeof(PinData::PlotData*));
                    ImGui::TextUnformatted(label.c_str());
                    ImGui::EndDragDropSource();
                }

                if (!plotDataHasData)
                {
                    ImGui::PopStyleVar();
                }
            }

            ImGui::EndChild();
            ImGui::EndGroup();

            ImGui::SameLine();

            const char* xLabel = plotInfo.overrideXAxisLabel ? (!plotInfo.xAxisLabel.empty() ? plotInfo.xAxisLabel.c_str() : nullptr)
                                                             : (!data.at(0).plotData.empty() ? data.at(0).plotData.at(plotInfo.selectedXdata.at(0)).displayName.c_str() : nullptr);

            const char* y1Label = !plotInfo.y1AxisLabel.empty() ? plotInfo.y1AxisLabel.c_str() : nullptr;
            const char* y2Label = (plotInfo.plotFlags & ImPlotFlags_YAxis2) && !plotInfo.y2AxisLabel.empty() ? plotInfo.y2AxisLabel.c_str() : nullptr;
            const char* y3Label = (plotInfo.plotFlags & ImPlotFlags_YAxis3) && !plotInfo.y3AxisLabel.empty() ? plotInfo.y3AxisLabel.c_str() : nullptr;

            ImPlot::FitNextPlotAxes(plotInfo.autoLimitXaxis, plotInfo.autoLimitYaxis, plotInfo.autoLimitYaxis, plotInfo.autoLimitYaxis);
            if (ImPlot::BeginPlot((plotInfo.title + "##" + std::to_string(size_t(id)) + " - " + std::to_string(plotNum)).c_str(),
                                  xLabel, y1Label, plotInfo.size, plotInfo.plotFlags,
                                  ImPlotAxisFlags_None, ImPlotAxisFlags_None, ImPlotAxisFlags_NoGridLines, ImPlotAxisFlags_NoGridLines, y2Label, y3Label))
            {
                for (size_t pinIndex = 0; pinIndex < data.size(); pinIndex++)
                {
                    for (auto& plotData : data.at(pinIndex).plotData)
                    {
                        if (plotData.plotOnAxis.contains(plotNum)
                            && plotData.hasData
                            && (plotData.plotOnAxis.at(plotNum).first == ImPlotYAxis_1
                                || (plotData.plotOnAxis.at(plotNum).first == ImPlotYAxis_2 && (plotInfo.plotFlags & ImPlotFlags_YAxis2))
                                || (plotData.plotOnAxis.at(plotNum).first == ImPlotYAxis_3 && (plotInfo.plotFlags & ImPlotFlags_YAxis3))))
                        {
                            ImPlot::SetPlotYAxis(plotData.plotOnAxis.at(plotNum).first);

                            // Style options
                            if (plotData.plotOnAxis.at(plotNum).second.legendName.empty())
                            {
                                plotData.plotOnAxis.at(plotNum).second.legendName = plotData.displayName + " (" + std::to_string(pinIndex + 1) + " - " + data.at(pinIndex).dataIdentifier + ")";
                            }
                            if (plotData.plotOnAxis.at(plotNum).second.color.w == -1)
                            {
                                plotData.plotOnAxis.at(plotNum).second.color = ImPlot::NextColormapColor();
                                plotData.plotOnAxis.at(plotNum).second.markerFillColor = plotData.plotOnAxis.at(plotNum).second.color;
                                plotData.plotOnAxis.at(plotNum).second.markerOutlineColor = plotData.plotOnAxis.at(plotNum).second.color;
                            }
                            if (plotData.plotOnAxis.at(plotNum).second.lineType == PlotStyle::LineType::Line)
                            {
                                ImPlot::SetNextLineStyle(plotData.plotOnAxis.at(plotNum).second.color, plotData.plotOnAxis.at(plotNum).second.thickness);
                            }
                            if (plotData.plotOnAxis.at(plotNum).second.lineType == PlotStyle::LineType::Scatter || plotData.plotOnAxis.at(plotNum).second.markers)
                            {
                                ImPlot::SetNextMarkerStyle(plotData.plotOnAxis.at(plotNum).second.markerStyle,
                                                           plotData.plotOnAxis.at(plotNum).second.markerSize,
                                                           plotData.plotOnAxis.at(plotNum).second.markerFillColor,
                                                           plotData.plotOnAxis.at(plotNum).second.markerWeight,
                                                           plotData.plotOnAxis.at(plotNum).second.markerOutlineColor);
                            }

                            std::string plotName = fmt::format("{}##{} - {} - {}", plotData.plotOnAxis.at(plotNum).second.legendName, size_t(id), pinIndex + 1, plotData.displayName);

                            auto stride = plotData.plotOnAxis.at(plotNum).second.stride ? plotData.plotOnAxis.at(plotNum).second.stride
                                                                                        : data.at(pinIndex).stride;
                            auto dataPointCount = static_cast<int>(std::ceil(static_cast<double>(plotData.buffer.size())
                                                                             / static_cast<double>(stride)));

                            // Plot the data
                            if (plotData.plotOnAxis.at(plotNum).second.lineType == PlotStyle::LineType::Line)
                            {
                                ImPlot::PlotLine(plotName.c_str(),
                                                 data.at(pinIndex).plotData.at(plotInfo.selectedXdata.at(pinIndex)).buffer.data(),
                                                 plotData.buffer.data(),
                                                 dataPointCount,
                                                 plotData.buffer.offset(),
                                                 stride * static_cast<int>(sizeof(double)));
                            }
                            else if (plotData.plotOnAxis.at(plotNum).second.lineType == PlotStyle::LineType::Scatter)
                            {
                                ImPlot::PlotScatter(plotName.c_str(),
                                                    data.at(pinIndex).plotData.at(plotInfo.selectedXdata.at(pinIndex)).buffer.data(),
                                                    plotData.buffer.data(),
                                                    dataPointCount,
                                                    plotData.buffer.offset(),
                                                    stride * static_cast<int>(sizeof(double)));
                            }

                            // allow legend labels to be dragged and dropped
                            if (ImPlot::BeginLegendDragDropSource(plotName.c_str()))
                            {
                                auto* ptrPlotData = &plotData;
                                ImGui::SetDragDropPayload(("DND_DATA " + std::to_string(size_t(id)) + " - " + std::to_string(plotNum)).c_str(),
                                                          &ptrPlotData, sizeof(PinData::PlotData*));
                                ImGui::TextUnformatted(plotData.displayName.c_str());
                                ImPlot::EndLegendDragDropSource();
                            }

                            // Legend item context menu (right click on legend item)
                            if (ImPlot::BeginLegendPopup(plotName.c_str()))
                            {
                                ImGui::TextUnformatted(fmt::format("Pin {} - {}: {}", pinIndex + 1, data.at(pinIndex).dataIdentifier, plotData.displayName).c_str());
                                ImGui::Separator();

                                if (plotData.plotOnAxis.at(plotNum).second.legendNameGui.empty())
                                {
                                    plotData.plotOnAxis.at(plotNum).second.legendNameGui = plotData.plotOnAxis.at(plotNum).second.legendName;
                                }
                                ImGui::InputText("Legend name", &plotData.plotOnAxis.at(plotNum).second.legendNameGui);
                                if (plotData.plotOnAxis.at(plotNum).second.legendNameGui != plotData.plotOnAxis.at(plotNum).second.legendName && !ImGui::IsItemActive())
                                {
                                    plotData.plotOnAxis.at(plotNum).second.legendName = plotData.plotOnAxis.at(plotNum).second.legendNameGui;
                                    flow::ApplyChanges();
                                    LOG_DEBUG("{}: Legend changed to {}", nameId(), plotData.plotOnAxis.at(plotNum).second.legendName);
                                }

                                if (ImGui::InputInt("Stride", &plotData.plotOnAxis.at(plotNum).second.stride))
                                {
                                    if (plotData.plotOnAxis.at(plotNum).second.stride < 0)
                                    {
                                        plotData.plotOnAxis.at(plotNum).second.stride = 0;
                                    }
                                    if (plotData.plotOnAxis.at(plotNum).second.stride > static_cast<int>(plotData.buffer.size()) - 1)
                                    {
                                        plotData.plotOnAxis.at(plotNum).second.stride = static_cast<int>(plotData.buffer.size()) - 1;
                                    }
                                    flow::ApplyChanges();
                                    LOG_DEBUG("{}: Stride changed to {}", nameId(), plotData.plotOnAxis.at(plotNum).second.stride);
                                }

                                if (ImGui::Combo("Style", reinterpret_cast<int*>(&plotData.plotOnAxis.at(plotNum).second.lineType),
                                                 "Scatter\0Line\0\0"))
                                {
                                    flow::ApplyChanges();
                                }
                                if (plotData.plotOnAxis.at(plotNum).second.lineType == PlotStyle::LineType::Line)
                                {
                                    if (ImGui::ColorEdit3("Line Color", &plotData.plotOnAxis.at(plotNum).second.color.x))
                                    {
                                        flow::ApplyChanges();
                                    }
                                    if (ImGui::DragFloat("Line Thickness", &plotData.plotOnAxis.at(plotNum).second.thickness, 0.1F, 0.0F, 8.0F, "%.2f px"))
                                    {
                                        flow::ApplyChanges();
                                    }
                                    if (ImGui::Checkbox("Markers", &plotData.plotOnAxis.at(plotNum).second.markers))
                                    {
                                        flow::ApplyChanges();
                                    }
                                }
                                if (plotData.plotOnAxis.at(plotNum).second.lineType == PlotStyle::LineType::Scatter || plotData.plotOnAxis.at(plotNum).second.markers)
                                {
                                    if (ImGui::Combo("Marker Style", &plotData.plotOnAxis.at(plotNum).second.markerStyle,
                                                     "Circle\0Square\0Diamond\0Up\0Down\0Left\0Right\0Cross\0Plus\0Asterisk\0\0"))
                                    {
                                        flow::ApplyChanges();
                                    }
                                    if (ImGui::DragFloat("Marker Size", &plotData.plotOnAxis.at(plotNum).second.markerSize, 0.1F, 1.0F, 10.0F, "%.2f px"))
                                    {
                                        flow::ApplyChanges();
                                    }
                                    if (ImGui::DragFloat("Marker Weight", &plotData.plotOnAxis.at(plotNum).second.markerWeight, 0.05F, 0.5F, 3.0F, "%.2f px"))
                                    {
                                        flow::ApplyChanges();
                                    }
                                    if (ImGui::ColorEdit4("Marker Fill Color", &plotData.plotOnAxis.at(plotNum).second.markerFillColor.x))
                                    {
                                        flow::ApplyChanges();
                                    }
                                    if (ImGui::ColorEdit4("Marker Outline Color", &plotData.plotOnAxis.at(plotNum).second.markerOutlineColor.x))
                                    {
                                        flow::ApplyChanges();
                                    }
                                }
                                ImPlot::EndLegendPopup();
                            }
                        }
                    }
                }

                // make our plot a drag and drop target
                if (ImGui::BeginDragDropTarget())
                {
                    if (const ImGuiPayload* payloadData = ImGui::AcceptDragDropPayload(("DND_DATA " + std::to_string(size_t(id)) + " - " + std::to_string(plotNum)).c_str()))
                    {
                        auto* plotData = *static_cast<PinData::PlotData**>(payloadData->Data);

                        PlotStyle style;
                        if (plotData->plotOnAxis.contains(plotNum))
                        {
                            style = plotData->plotOnAxis.at(plotNum).second;
                        }

                        plotData->plotOnAxis[plotNum] = { 0, style };
                        // set specific y-axis if hovered
                        for (int y = 0; y < 3; y++)
                        {
                            if (ImPlot::IsPlotYAxisHovered(y))
                            {
                                plotData->plotOnAxis[plotNum].first = y;
                            }
                        }
                        flow::ApplyChanges();
                    }
                    ImGui::EndDragDropTarget();
                }

                ImPlot::EndPlot();
            }
        }
    }
}

[[nodiscard]] json NAV::Plot::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["nInputPins"] = nInputPins;
    j["nPlots"] = nPlots;
    j["pinData"] = data;
    j["plotInfos"] = plotInfos;

    return j;
}

void NAV::Plot::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("nInputPins"))
    {
        j.at("nInputPins").get_to(nInputPins);
        updateNumberOfInputPins();
    }
    if (j.contains("nPlots"))
    {
        j.at("nPlots").get_to(nPlots);
        updateNumberOfPlots();
    }
    if (j.contains("pinData"))
    {
        j.at("pinData").get_to(data);

        for (size_t inputPinIndex = 0; inputPinIndex < inputPins.size(); inputPinIndex++)
        {
            inputPins.at(inputPinIndex).notifyFunc.clear();
            switch (data.at(inputPinIndex).pinType)
            {
            case PinData::PinType::Flow:
                inputPins.at(inputPinIndex).type = Pin::Type::Flow;
                inputPins.at(inputPinIndex).dataIdentifier = dataIdentifier;
                inputPins.at(inputPinIndex).data = Pin::PinData(static_cast<void (Node::*)(const std::shared_ptr<const NodeData>&, ax::NodeEditor::LinkId)>(&Plot::plotData));
                break;
            case Plot::PinData::PinType::Bool:
                inputPins.at(inputPinIndex).type = Pin::Type::Bool;
                inputPins.at(inputPinIndex).dataIdentifier.clear();
                inputPins.at(inputPinIndex).notifyFunc.emplace_back(this, static_cast<void (Node::*)(ax::NodeEditor::LinkId)>(&Plot::plotBoolean), 0);
                break;
            case Plot::PinData::PinType::Int:
                inputPins.at(inputPinIndex).type = Pin::Type::Int;
                inputPins.at(inputPinIndex).dataIdentifier.clear();
                inputPins.at(inputPinIndex).notifyFunc.emplace_back(this, static_cast<void (Node::*)(ax::NodeEditor::LinkId)>(&Plot::plotInteger), 0);
                break;
            case Plot::PinData::PinType::Float:
                inputPins.at(inputPinIndex).type = Pin::Type::Float;
                inputPins.at(inputPinIndex).dataIdentifier.clear();
                inputPins.at(inputPinIndex).notifyFunc.emplace_back(this, static_cast<void (Node::*)(ax::NodeEditor::LinkId)>(&Plot::plotFloat), 0);
                break;
            case Plot::PinData::PinType::Matrix:
                inputPins.at(inputPinIndex).type = Pin::Type::Matrix;
                inputPins.at(inputPinIndex).dataIdentifier = { "Eigen::MatrixXd", "BlockMatrix" };
                inputPins.at(inputPinIndex).notifyFunc.emplace_back(this, static_cast<void (Node::*)(ax::NodeEditor::LinkId)>(&Plot::plotMatrix), 0);
                break;
            default:
                break;
            }
        }
    }
    if (j.contains("plotInfos"))
    {
        j.at("plotInfos").get_to(plotInfos);
    }
}

bool NAV::Plot::initialize()
{
    LOG_TRACE("{}: called", nameId());

    startValue_Time = std::nan("");
    startValue_North = std::nan("");
    startValue_East = std::nan("");

    for (auto& pinData : data)
    {
        for (auto& plotData : pinData.plotData)
        {
            plotData.hasData = false;
            plotData.buffer.clear();
        }
    }

    return true;
}

void NAV::Plot::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

void NAV::Plot::afterCreateLink(Pin* startPin, Pin* endPin)
{
    LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(startPin->id), size_t(endPin->id));

    size_t pinIndex = pinIndexFromId(endPin->id);

    size_t i = 0;

    if (inputPins.at(pinIndex).type == Pin::Type::Flow)
    {
        data.at(pinIndex).dataIdentifier = startPin->dataIdentifier.front();

        if (startPin->dataIdentifier.front() == PosVelAtt::type()
            || startPin->dataIdentifier.front() == InertialNavSol::type())
        {
            // InsObs
            data.at(pinIndex).addPlotDataItem(i++, "Time [s]");
            data.at(pinIndex).addPlotDataItem(i++, "GPS time of week [s]");
            // PosVelAtt
            data.at(pinIndex).addPlotDataItem(i++, "Latitude [deg]");
            data.at(pinIndex).addPlotDataItem(i++, "Longitude [deg]");
            data.at(pinIndex).addPlotDataItem(i++, "Altitude [m]");
            data.at(pinIndex).addPlotDataItem(i++, "North/South [m]");
            data.at(pinIndex).addPlotDataItem(i++, "East/West [m]");
            data.at(pinIndex).addPlotDataItem(i++, "X-ECEF [m]");
            data.at(pinIndex).addPlotDataItem(i++, "Y-ECEF [m]");
            data.at(pinIndex).addPlotDataItem(i++, "Z-ECEF [m]");
            data.at(pinIndex).addPlotDataItem(i++, "North velocity [m/s]");
            data.at(pinIndex).addPlotDataItem(i++, "East velocity [m/s]");
            data.at(pinIndex).addPlotDataItem(i++, "Down velocity [m/s]");
            data.at(pinIndex).addPlotDataItem(i++, "Roll [deg]");
            data.at(pinIndex).addPlotDataItem(i++, "Pitch [deg]");
            data.at(pinIndex).addPlotDataItem(i++, "Yaw [deg]");
            data.at(pinIndex).addPlotDataItem(i++, "Quaternion::w");
            data.at(pinIndex).addPlotDataItem(i++, "Quaternion::x");
            data.at(pinIndex).addPlotDataItem(i++, "Quaternion::y");
            data.at(pinIndex).addPlotDataItem(i++, "Quaternion::z");
        }
        else if (startPin->dataIdentifier.front() == PVAError::type())
        {
            // InsObs
            data.at(pinIndex).addPlotDataItem(i++, "Time [s]");
            data.at(pinIndex).addPlotDataItem(i++, "GPS time of week [s]");
            // PVAError
            data.at(pinIndex).addPlotDataItem(i++, "Roll error [deg]");
            data.at(pinIndex).addPlotDataItem(i++, "Pitch error [deg]");
            data.at(pinIndex).addPlotDataItem(i++, "Yaw error [deg]");
            data.at(pinIndex).addPlotDataItem(i++, "North velocity error [m/s]");
            data.at(pinIndex).addPlotDataItem(i++, "East velocity error [m/s]");
            data.at(pinIndex).addPlotDataItem(i++, "Down velocity error [m/s]");
            data.at(pinIndex).addPlotDataItem(i++, "Latitude error [deg]");
            data.at(pinIndex).addPlotDataItem(i++, "Longitude error [deg]");
            data.at(pinIndex).addPlotDataItem(i++, "Altitude error [deg]");
        }
        else if (startPin->dataIdentifier.front() == ImuBiases::type())
        {
            // InsObs
            data.at(pinIndex).addPlotDataItem(i++, "Time [s]");
            data.at(pinIndex).addPlotDataItem(i++, "GPS time of week [s]");
            // ImuBiases
            data.at(pinIndex).addPlotDataItem(i++, "Accelerometer bias X_b accumulated [m/s^2]");
            data.at(pinIndex).addPlotDataItem(i++, "Accelerometer bias Y_b accumulated [m/s^2]");
            data.at(pinIndex).addPlotDataItem(i++, "Accelerometer bias Z_b accumulated [m/s^2]");
            data.at(pinIndex).addPlotDataItem(i++, "Gyroscope bias X_b accumulated [rad/s]");
            data.at(pinIndex).addPlotDataItem(i++, "Gyroscope bias Y_b accumulated [rad/s]");
            data.at(pinIndex).addPlotDataItem(i++, "Gyroscope bias Z_b accumulated [rad/s]");
        }
        else if (startPin->dataIdentifier.front() == RtklibPosObs::type())
        {
            // InsObs
            data.at(pinIndex).addPlotDataItem(i++, "Time [s]");
            data.at(pinIndex).addPlotDataItem(i++, "GPS time of week [s]");
            // RtklibPosObs
            data.at(pinIndex).addPlotDataItem(i++, "X-ECEF [m]");
            data.at(pinIndex).addPlotDataItem(i++, "Y-ECEF [m]");
            data.at(pinIndex).addPlotDataItem(i++, "Z-ECEF [m]");
            data.at(pinIndex).addPlotDataItem(i++, "Latitude [deg]");
            data.at(pinIndex).addPlotDataItem(i++, "Longitude [deg]");
            data.at(pinIndex).addPlotDataItem(i++, "Altitude [m]");
            data.at(pinIndex).addPlotDataItem(i++, "North/South [m]");
            data.at(pinIndex).addPlotDataItem(i++, "East/West [m]");
            data.at(pinIndex).addPlotDataItem(i++, "Q [-]");
            data.at(pinIndex).addPlotDataItem(i++, "ns [-]");
            data.at(pinIndex).addPlotDataItem(i++, "sdx [m]");
            data.at(pinIndex).addPlotDataItem(i++, "sdy [m]");
            data.at(pinIndex).addPlotDataItem(i++, "sdz [m]");
            data.at(pinIndex).addPlotDataItem(i++, "sdn [m]");
            data.at(pinIndex).addPlotDataItem(i++, "sde [m]");
            data.at(pinIndex).addPlotDataItem(i++, "sdu [m]");
            data.at(pinIndex).addPlotDataItem(i++, "sdxy [m]");
            data.at(pinIndex).addPlotDataItem(i++, "sdyz [m]");
            data.at(pinIndex).addPlotDataItem(i++, "sdzx [m]");
            data.at(pinIndex).addPlotDataItem(i++, "sdne [m]");
            data.at(pinIndex).addPlotDataItem(i++, "sdeu [m]");
            data.at(pinIndex).addPlotDataItem(i++, "sdun [m]");
            data.at(pinIndex).addPlotDataItem(i++, "age [s]");
            data.at(pinIndex).addPlotDataItem(i++, "ratio [-]");
        }
        else if (startPin->dataIdentifier.front() == UbloxObs::type())
        {
            // InsObs
            data.at(pinIndex).addPlotDataItem(i++, "Time [s]");
            data.at(pinIndex).addPlotDataItem(i++, "GPS time of week [s]");
            // UbloxObs
            data.at(pinIndex).addPlotDataItem(i++, "X-ECEF [m]");
            data.at(pinIndex).addPlotDataItem(i++, "Y-ECEF [m]");
            data.at(pinIndex).addPlotDataItem(i++, "Z-ECEF [m]");
            data.at(pinIndex).addPlotDataItem(i++, "Latitude [deg]");
            data.at(pinIndex).addPlotDataItem(i++, "Longitude [deg]");
            data.at(pinIndex).addPlotDataItem(i++, "Altitude [m]");
            data.at(pinIndex).addPlotDataItem(i++, "North/South [m]");
            data.at(pinIndex).addPlotDataItem(i++, "East/West [m]");
            data.at(pinIndex).addPlotDataItem(i++, "Velocity N [m/s]");
            data.at(pinIndex).addPlotDataItem(i++, "Velocity E [m/s]");
            data.at(pinIndex).addPlotDataItem(i++, "Velocity D [m/s]");
        }
        else if (startPin->dataIdentifier.front() == ImuObs::type())
        {
            // InsObs
            data.at(pinIndex).addPlotDataItem(i++, "Time [s]");
            data.at(pinIndex).addPlotDataItem(i++, "GPS time of week [s]");
            // ImuObs
            data.at(pinIndex).addPlotDataItem(i++, "Time since startup [ns]");
            data.at(pinIndex).addPlotDataItem(i++, "Mag uncomp X [Gauss]");
            data.at(pinIndex).addPlotDataItem(i++, "Mag uncomp Y [Gauss]");
            data.at(pinIndex).addPlotDataItem(i++, "Mag uncomp Z [Gauss]");
            data.at(pinIndex).addPlotDataItem(i++, "Accel uncomp X [m/s^2]");
            data.at(pinIndex).addPlotDataItem(i++, "Accel uncomp Y [m/s^2]");
            data.at(pinIndex).addPlotDataItem(i++, "Accel uncomp Z [m/s^2]");
            data.at(pinIndex).addPlotDataItem(i++, "Gyro uncomp X [rad/s]");
            data.at(pinIndex).addPlotDataItem(i++, "Gyro uncomp Y [rad/s]");
            data.at(pinIndex).addPlotDataItem(i++, "Gyro uncomp Z [rad/s]");
            data.at(pinIndex).addPlotDataItem(i++, "Mag Comp X [Gauss]");
            data.at(pinIndex).addPlotDataItem(i++, "Mag Comp Y [Gauss]");
            data.at(pinIndex).addPlotDataItem(i++, "Mag Comp Z [Gauss]");
            data.at(pinIndex).addPlotDataItem(i++, "Accel Comp X [m/s^2]");
            data.at(pinIndex).addPlotDataItem(i++, "Accel Comp Y [m/s^2]");
            data.at(pinIndex).addPlotDataItem(i++, "Accel Comp Z [m/s^2]");
            data.at(pinIndex).addPlotDataItem(i++, "Gyro Comp X [rad/s]");
            data.at(pinIndex).addPlotDataItem(i++, "Gyro Comp Y [rad/s]");
            data.at(pinIndex).addPlotDataItem(i++, "Gyro Comp Z [rad/s]");
            data.at(pinIndex).addPlotDataItem(i++, "Temperature [°C]");
        }
        else if (startPin->dataIdentifier.front() == KvhObs::type())
        {
            // InsObs
            data.at(pinIndex).addPlotDataItem(i++, "Time [s]");
            data.at(pinIndex).addPlotDataItem(i++, "GPS time of week [s]");
            // ImuObs
            data.at(pinIndex).addPlotDataItem(i++, "Time since startup [ns]");
            data.at(pinIndex).addPlotDataItem(i++, "Mag uncomp X [Gauss]");
            data.at(pinIndex).addPlotDataItem(i++, "Mag uncomp Y [Gauss]");
            data.at(pinIndex).addPlotDataItem(i++, "Mag uncomp Z [Gauss]");
            data.at(pinIndex).addPlotDataItem(i++, "Accel uncomp X [m/s^2]");
            data.at(pinIndex).addPlotDataItem(i++, "Accel uncomp Y [m/s^2]");
            data.at(pinIndex).addPlotDataItem(i++, "Accel uncomp Z [m/s^2]");
            data.at(pinIndex).addPlotDataItem(i++, "Gyro uncomp X [rad/s]");
            data.at(pinIndex).addPlotDataItem(i++, "Gyro uncomp Y [rad/s]");
            data.at(pinIndex).addPlotDataItem(i++, "Gyro uncomp Z [rad/s]");
            data.at(pinIndex).addPlotDataItem(i++, "Mag Comp X [Gauss]");
            data.at(pinIndex).addPlotDataItem(i++, "Mag Comp Y [Gauss]");
            data.at(pinIndex).addPlotDataItem(i++, "Mag Comp Z [Gauss]");
            data.at(pinIndex).addPlotDataItem(i++, "Accel Comp X [m/s^2]");
            data.at(pinIndex).addPlotDataItem(i++, "Accel Comp Y [m/s^2]");
            data.at(pinIndex).addPlotDataItem(i++, "Accel Comp Z [m/s^2]");
            data.at(pinIndex).addPlotDataItem(i++, "Gyro Comp X [rad/s]");
            data.at(pinIndex).addPlotDataItem(i++, "Gyro Comp Y [rad/s]");
            data.at(pinIndex).addPlotDataItem(i++, "Gyro Comp Z [rad/s]");
            data.at(pinIndex).addPlotDataItem(i++, "Temperature [°C]");
            // KvhObs
            data.at(pinIndex).addPlotDataItem(i++, "Status [bits]");
            data.at(pinIndex).addPlotDataItem(i++, "Sequence Number [.]");
        }
        else if (startPin->dataIdentifier.front() == ImuObsWDelta::type())
        {
            // InsObs
            data.at(pinIndex).addPlotDataItem(i++, "Time [s]");
            data.at(pinIndex).addPlotDataItem(i++, "GPS time of week [s]");
            // ImuObs
            data.at(pinIndex).addPlotDataItem(i++, "Time since startup [ns]");
            data.at(pinIndex).addPlotDataItem(i++, "Mag uncomp X [Gauss]");
            data.at(pinIndex).addPlotDataItem(i++, "Mag uncomp Y [Gauss]");
            data.at(pinIndex).addPlotDataItem(i++, "Mag uncomp Z [Gauss]");
            data.at(pinIndex).addPlotDataItem(i++, "Accel uncomp X [m/s^2]");
            data.at(pinIndex).addPlotDataItem(i++, "Accel uncomp Y [m/s^2]");
            data.at(pinIndex).addPlotDataItem(i++, "Accel uncomp Z [m/s^2]");
            data.at(pinIndex).addPlotDataItem(i++, "Gyro uncomp X [rad/s]");
            data.at(pinIndex).addPlotDataItem(i++, "Gyro uncomp Y [rad/s]");
            data.at(pinIndex).addPlotDataItem(i++, "Gyro uncomp Z [rad/s]");
            data.at(pinIndex).addPlotDataItem(i++, "Mag Comp X [Gauss]");
            data.at(pinIndex).addPlotDataItem(i++, "Mag Comp Y [Gauss]");
            data.at(pinIndex).addPlotDataItem(i++, "Mag Comp Z [Gauss]");
            data.at(pinIndex).addPlotDataItem(i++, "Accel Comp X [m/s^2]");
            data.at(pinIndex).addPlotDataItem(i++, "Accel Comp Y [m/s^2]");
            data.at(pinIndex).addPlotDataItem(i++, "Accel Comp Z [m/s^2]");
            data.at(pinIndex).addPlotDataItem(i++, "Gyro Comp X [rad/s]");
            data.at(pinIndex).addPlotDataItem(i++, "Gyro Comp Y [rad/s]");
            data.at(pinIndex).addPlotDataItem(i++, "Gyro Comp Z [rad/s]");
            data.at(pinIndex).addPlotDataItem(i++, "Temperature [°C]");
            // ImuObsWDelta
            data.at(pinIndex).addPlotDataItem(i++, "dTime [s]");
            data.at(pinIndex).addPlotDataItem(i++, "dTheta X [deg]");
            data.at(pinIndex).addPlotDataItem(i++, "dTheta Y [deg]");
            data.at(pinIndex).addPlotDataItem(i++, "dTheta Z [deg]");
            data.at(pinIndex).addPlotDataItem(i++, "dVelocity X [m/s]");
            data.at(pinIndex).addPlotDataItem(i++, "dVelocity Y [m/s]");
            data.at(pinIndex).addPlotDataItem(i++, "dVelocity Z [m/s]");
        }
        else if (startPin->dataIdentifier.front() == VectorNavBinaryOutput::type())
        {
            // InsObs
            data.at(pinIndex).addPlotDataItem(i++, "Time [s]");
            data.at(pinIndex).addPlotDataItem(i++, "GPS time of week [s]");
            // VectorNavBinaryOutput
            // Group 2 (Time)
            data.at(pinIndex).addPlotDataItem(i++, "Time::TimeStartup [ns]");
            data.at(pinIndex).addPlotDataItem(i++, "Time::TimeGps [ns]");
            data.at(pinIndex).addPlotDataItem(i++, "Time::GpsTow [ns]");
            data.at(pinIndex).addPlotDataItem(i++, "Time::GpsWeek");
            data.at(pinIndex).addPlotDataItem(i++, "Time::TimeSyncIn [ns]");
            data.at(pinIndex).addPlotDataItem(i++, "Time::TimeGpsPps [ns]");
            data.at(pinIndex).addPlotDataItem(i++, "Time::TimeUTC::year");
            data.at(pinIndex).addPlotDataItem(i++, "Time::TimeUTC::month");
            data.at(pinIndex).addPlotDataItem(i++, "Time::TimeUTC::day");
            data.at(pinIndex).addPlotDataItem(i++, "Time::TimeUTC::hour");
            data.at(pinIndex).addPlotDataItem(i++, "Time::TimeUTC::min");
            data.at(pinIndex).addPlotDataItem(i++, "Time::TimeUTC::sec");
            data.at(pinIndex).addPlotDataItem(i++, "Time::TimeUTC::ms");
            data.at(pinIndex).addPlotDataItem(i++, "Time::SyncInCnt");
            data.at(pinIndex).addPlotDataItem(i++, "Time::SyncOutCnt");
            data.at(pinIndex).addPlotDataItem(i++, "Time::TimeStatus::timeOk");
            data.at(pinIndex).addPlotDataItem(i++, "Time::TimeStatus::dateOk");
            data.at(pinIndex).addPlotDataItem(i++, "Time::TimeStatus::utcTimeValid");
            // Group 3 (IMU)
            data.at(pinIndex).addPlotDataItem(i++, "IMU::ImuStatus");
            data.at(pinIndex).addPlotDataItem(i++, "IMU::UncompMag::X [Gauss]");
            data.at(pinIndex).addPlotDataItem(i++, "IMU::UncompMag::Y [Gauss]");
            data.at(pinIndex).addPlotDataItem(i++, "IMU::UncompMag::Z [Gauss]");
            data.at(pinIndex).addPlotDataItem(i++, "IMU::UncompAccel::X [m/s^2]");
            data.at(pinIndex).addPlotDataItem(i++, "IMU::UncompAccel::Y [m/s^2]");
            data.at(pinIndex).addPlotDataItem(i++, "IMU::UncompAccel::Z [m/s^2]");
            data.at(pinIndex).addPlotDataItem(i++, "IMU::UncompGyro::X [rad/s]");
            data.at(pinIndex).addPlotDataItem(i++, "IMU::UncompGyro::Y [rad/s]");
            data.at(pinIndex).addPlotDataItem(i++, "IMU::UncompGyro::Z [rad/s]");
            data.at(pinIndex).addPlotDataItem(i++, "IMU::Temp [Celsius]");
            data.at(pinIndex).addPlotDataItem(i++, "IMU::Pres [kPa]");
            data.at(pinIndex).addPlotDataItem(i++, "IMU::DeltaTime [s]");
            data.at(pinIndex).addPlotDataItem(i++, "IMU::DeltaTheta::X [deg]");
            data.at(pinIndex).addPlotDataItem(i++, "IMU::DeltaTheta::Y [deg]");
            data.at(pinIndex).addPlotDataItem(i++, "IMU::DeltaTheta::Z [deg]");
            data.at(pinIndex).addPlotDataItem(i++, "IMU::DeltaVel::X [m/s]");
            data.at(pinIndex).addPlotDataItem(i++, "IMU::DeltaVel::Y [m/s]");
            data.at(pinIndex).addPlotDataItem(i++, "IMU::DeltaVel::Z [m/s]");
            data.at(pinIndex).addPlotDataItem(i++, "IMU::Mag::X [Gauss]");
            data.at(pinIndex).addPlotDataItem(i++, "IMU::Mag::Y [Gauss]");
            data.at(pinIndex).addPlotDataItem(i++, "IMU::Mag::Z [Gauss]");
            data.at(pinIndex).addPlotDataItem(i++, "IMU::Accel::X [m/s^2]");
            data.at(pinIndex).addPlotDataItem(i++, "IMU::Accel::Y [m/s^2]");
            data.at(pinIndex).addPlotDataItem(i++, "IMU::Accel::Z [m/s^2]");
            data.at(pinIndex).addPlotDataItem(i++, "IMU::AngularRate::X [rad/s]");
            data.at(pinIndex).addPlotDataItem(i++, "IMU::AngularRate::Y [rad/s]");
            data.at(pinIndex).addPlotDataItem(i++, "IMU::AngularRate::Z [rad/s]");
            // Group 4 (GNSS1)
            data.at(pinIndex).addPlotDataItem(i++, "GNSS1::UTC::year");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS1::UTC::month");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS1::UTC::day");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS1::UTC::hour");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS1::UTC::min");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS1::UTC::sec");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS1::UTC::ms");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS1::Tow [ns]");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS1::Week");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS1::NumSats");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS1::Fix");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS1::PosLla::latitude [deg]");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS1::PosLla::longitude [deg]");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS1::PosLla::altitude [m]");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS1::PosEcef::X [m]");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS1::PosEcef::Y [m]");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS1::PosEcef::Z [m]");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS1::VelNed::N [m/s]");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS1::VelNed::E [m/s]");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS1::VelNed::D [m/s]");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS1::VelEcef::X [m/s]");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS1::VelEcef::Y [m/s]");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS1::VelEcef::Z [m/s]");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS1::PosU::N [m]");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS1::PosU::E [m]");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS1::PosU::D [m]");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS1::VelU [m/s]");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS1::TimeU [s]");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS1::TimeInfo::Status::timeOk");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS1::TimeInfo::Status::dateOk");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS1::TimeInfo::Status::utcTimeValid");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS1::TimeInfo::LeapSeconds");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS1::DOP::g");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS1::DOP::p");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS1::DOP::t");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS1::DOP::v");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS1::DOP::h");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS1::DOP::n");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS1::DOP::e");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS1::SatInfo::NumSats");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS1::RawMeas::Tow [s]");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS1::RawMeas::Week");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS1::RawMeas::NumSats");
            // Group 5 (Attitude)
            data.at(pinIndex).addPlotDataItem(i++, "Att::VpeStatus::AttitudeQuality");
            data.at(pinIndex).addPlotDataItem(i++, "Att::VpeStatus::GyroSaturation");
            data.at(pinIndex).addPlotDataItem(i++, "Att::VpeStatus::GyroSaturationRecovery");
            data.at(pinIndex).addPlotDataItem(i++, "Att::VpeStatus::MagDisturbance");
            data.at(pinIndex).addPlotDataItem(i++, "Att::VpeStatus::MagSaturation");
            data.at(pinIndex).addPlotDataItem(i++, "Att::VpeStatus::AccDisturbance");
            data.at(pinIndex).addPlotDataItem(i++, "Att::VpeStatus::AccSaturation");
            data.at(pinIndex).addPlotDataItem(i++, "Att::VpeStatus::KnownMagDisturbance");
            data.at(pinIndex).addPlotDataItem(i++, "Att::VpeStatus::KnownAccelDisturbance");
            data.at(pinIndex).addPlotDataItem(i++, "Att::YawPitchRoll::Y [deg]");
            data.at(pinIndex).addPlotDataItem(i++, "Att::YawPitchRoll::P [deg]");
            data.at(pinIndex).addPlotDataItem(i++, "Att::YawPitchRoll::R [deg]");
            data.at(pinIndex).addPlotDataItem(i++, "Att::Quaternion::w");
            data.at(pinIndex).addPlotDataItem(i++, "Att::Quaternion::x");
            data.at(pinIndex).addPlotDataItem(i++, "Att::Quaternion::y");
            data.at(pinIndex).addPlotDataItem(i++, "Att::Quaternion::z");
            data.at(pinIndex).addPlotDataItem(i++, "Att::DCM::0-0");
            data.at(pinIndex).addPlotDataItem(i++, "Att::DCM::0-1");
            data.at(pinIndex).addPlotDataItem(i++, "Att::DCM::0-2");
            data.at(pinIndex).addPlotDataItem(i++, "Att::DCM::1-0");
            data.at(pinIndex).addPlotDataItem(i++, "Att::DCM::1-1");
            data.at(pinIndex).addPlotDataItem(i++, "Att::DCM::1-2");
            data.at(pinIndex).addPlotDataItem(i++, "Att::DCM::2-0");
            data.at(pinIndex).addPlotDataItem(i++, "Att::DCM::2-1");
            data.at(pinIndex).addPlotDataItem(i++, "Att::DCM::2-2");
            data.at(pinIndex).addPlotDataItem(i++, "Att::MagNed::N [Gauss]");
            data.at(pinIndex).addPlotDataItem(i++, "Att::MagNed::E [Gauss]");
            data.at(pinIndex).addPlotDataItem(i++, "Att::MagNed::D [Gauss]");
            data.at(pinIndex).addPlotDataItem(i++, "Att::AccelNed::N [m/s^2]");
            data.at(pinIndex).addPlotDataItem(i++, "Att::AccelNed::E [m/s^2]");
            data.at(pinIndex).addPlotDataItem(i++, "Att::AccelNed::D [m/s^2]");
            data.at(pinIndex).addPlotDataItem(i++, "Att::LinearAccelBody::X [m/s^2]");
            data.at(pinIndex).addPlotDataItem(i++, "Att::LinearAccelBody::Y [m/s^2]");
            data.at(pinIndex).addPlotDataItem(i++, "Att::LinearAccelBody::Z [m/s^2]");
            data.at(pinIndex).addPlotDataItem(i++, "Att::LinearAccelNed::N [m/s^2]");
            data.at(pinIndex).addPlotDataItem(i++, "Att::LinearAccelNed::E [m/s^2]");
            data.at(pinIndex).addPlotDataItem(i++, "Att::LinearAccelNed::D [m/s^2]");
            data.at(pinIndex).addPlotDataItem(i++, "Att::YprU::Y [deg]");
            data.at(pinIndex).addPlotDataItem(i++, "Att::YprU::P [deg]");
            data.at(pinIndex).addPlotDataItem(i++, "Att::YprU::R [deg]");
            // Group 6 (INS)
            data.at(pinIndex).addPlotDataItem(i++, "INS::InsStatus::Mode");
            data.at(pinIndex).addPlotDataItem(i++, "INS::InsStatus::GpsFix");
            data.at(pinIndex).addPlotDataItem(i++, "INS::InsStatus::Error::IMU");
            data.at(pinIndex).addPlotDataItem(i++, "INS::InsStatus::Error::MagPres");
            data.at(pinIndex).addPlotDataItem(i++, "INS::InsStatus::Error::GNSS");
            data.at(pinIndex).addPlotDataItem(i++, "INS::InsStatus::GpsHeadingIns");
            data.at(pinIndex).addPlotDataItem(i++, "INS::InsStatus::GpsCompass");
            data.at(pinIndex).addPlotDataItem(i++, "INS::PosLla::latitude [deg]");
            data.at(pinIndex).addPlotDataItem(i++, "INS::PosLla::longitude [deg]");
            data.at(pinIndex).addPlotDataItem(i++, "INS::PosLla::altitude [m]");
            data.at(pinIndex).addPlotDataItem(i++, "INS::PosEcef::X [m]");
            data.at(pinIndex).addPlotDataItem(i++, "INS::PosEcef::Y [m]");
            data.at(pinIndex).addPlotDataItem(i++, "INS::PosEcef::Z [m]");
            data.at(pinIndex).addPlotDataItem(i++, "INS::VelBody::X [m/s]");
            data.at(pinIndex).addPlotDataItem(i++, "INS::VelBody::Y [m/s]");
            data.at(pinIndex).addPlotDataItem(i++, "INS::VelBody::Z [m/s]");
            data.at(pinIndex).addPlotDataItem(i++, "INS::VelNed::N [m/s]");
            data.at(pinIndex).addPlotDataItem(i++, "INS::VelNed::E [m/s]");
            data.at(pinIndex).addPlotDataItem(i++, "INS::VelNed::D [m/s]");
            data.at(pinIndex).addPlotDataItem(i++, "INS::VelEcef::X [m/s]");
            data.at(pinIndex).addPlotDataItem(i++, "INS::VelEcef::Y [m/s]");
            data.at(pinIndex).addPlotDataItem(i++, "INS::VelEcef::Z [m/s]");
            data.at(pinIndex).addPlotDataItem(i++, "INS::MagEcef::X [Gauss}");
            data.at(pinIndex).addPlotDataItem(i++, "INS::MagEcef::Y [Gauss}");
            data.at(pinIndex).addPlotDataItem(i++, "INS::MagEcef::Z [Gauss}");
            data.at(pinIndex).addPlotDataItem(i++, "INS::AccelEcef::X [m/s^2]");
            data.at(pinIndex).addPlotDataItem(i++, "INS::AccelEcef::Y [m/s^2]");
            data.at(pinIndex).addPlotDataItem(i++, "INS::AccelEcef::Z [m/s^2]");
            data.at(pinIndex).addPlotDataItem(i++, "INS::LinearAccelEcef::X [m/s^2]");
            data.at(pinIndex).addPlotDataItem(i++, "INS::LinearAccelEcef::Y [m/s^2]");
            data.at(pinIndex).addPlotDataItem(i++, "INS::LinearAccelEcef::Z [m/s^2]");
            data.at(pinIndex).addPlotDataItem(i++, "INS::PosU [m]");
            data.at(pinIndex).addPlotDataItem(i++, "INS::VelU [m/s]");
            // Group 7 (GNSS2)
            data.at(pinIndex).addPlotDataItem(i++, "GNSS2::UTC::year");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS2::UTC::month");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS2::UTC::day");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS2::UTC::hour");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS2::UTC::min");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS2::UTC::sec");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS2::UTC::ms");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS2::Tow [ns]");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS2::Week");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS2::NumSats");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS2::Fix");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS2::PosLla::latitude [deg]");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS2::PosLla::longitude [deg]");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS2::PosLla::altitude [m]");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS2::PosEcef::X [m]");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS2::PosEcef::Y [m]");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS2::PosEcef::Z [m]");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS2::VelNed::N [m/s]");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS2::VelNed::E [m/s]");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS2::VelNed::D [m/s]");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS2::VelEcef::X [m/s]");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS2::VelEcef::Y [m/s]");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS2::VelEcef::Z [m/s]");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS2::PosU::N [m]");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS2::PosU::E [m]");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS2::PosU::D [m]");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS2::VelU [m/s]");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS2::TimeU [s]");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS2::TimeInfo::Status::timeOk");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS2::TimeInfo::Status::dateOk");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS2::TimeInfo::Status::utcTimeValid");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS2::TimeInfo::LeapSeconds");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS2::DOP::g");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS2::DOP::p");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS2::DOP::t");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS2::DOP::v");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS2::DOP::h");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS2::DOP::n");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS2::DOP::e");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS2::SatInfo::NumSats");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS2::RawMeas::Tow [s]");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS2::RawMeas::Week");
            data.at(pinIndex).addPlotDataItem(i++, "GNSS2::RawMeas::NumSats");
        }
    }
    else if (inputPins.at(pinIndex).type == Pin::Type::Bool)
    {
        data.at(pinIndex).dataIdentifier = startPin->name;

        // InsObs
        data.at(pinIndex).addPlotDataItem(i++, "Time [s]");
        data.at(pinIndex).addPlotDataItem(i++, "GPS time of week [s]");
        // Bool
        data.at(pinIndex).addPlotDataItem(i++, "Boolean");
    }
    else if (inputPins.at(pinIndex).type == Pin::Type::Int)
    {
        data.at(pinIndex).dataIdentifier = startPin->name;

        // InsObs
        data.at(pinIndex).addPlotDataItem(i++, "Time [s]");
        data.at(pinIndex).addPlotDataItem(i++, "GPS time of week [s]");
        // Int
        data.at(pinIndex).addPlotDataItem(i++, "Integer");
    }
    else if (inputPins.at(pinIndex).type == Pin::Type::Float)
    {
        data.at(pinIndex).dataIdentifier = startPin->name;

        // InsObs
        data.at(pinIndex).addPlotDataItem(i++, "Time [s]");
        data.at(pinIndex).addPlotDataItem(i++, "GPS time of week [s]");
        // Float
        data.at(pinIndex).addPlotDataItem(i++, "Float");
    }
    else if (inputPins.at(pinIndex).type == Pin::Type::Matrix)
    {
        data.at(pinIndex).dataIdentifier = startPin->name;

        // InsObs
        data.at(pinIndex).addPlotDataItem(i++, "Time [s]");
        data.at(pinIndex).addPlotDataItem(i++, "GPS time of week [s]");
        // Matrix
        if (startPin->dataIdentifier.front() == "Eigen::MatrixXd")
        {
            if (auto* matrix = getInputValue<Eigen::MatrixXd>(pinIndex))
            {
                for (int row = 0; row < matrix->rows(); row++)
                {
                    for (int col = 0; col < matrix->cols(); col++)
                    {
                        data.at(pinIndex).addPlotDataItem(i++, std::to_string(row) + ", " + std::to_string(col));
                    }
                }
            }
        }
        else if (startPin->dataIdentifier.front() == "BlockMatrix")
        {
            if (auto* mBlock = getInputValue<BlockMatrix>(pinIndex))
            {
                auto matrix = (*mBlock)();
                for (int row = 0; row < matrix.rows(); row++)
                {
                    for (int col = 0; col < matrix.cols(); col++)
                    {
                        data.at(pinIndex).addPlotDataItem(i++, std::to_string(row) + ", " + std::to_string(col));
                    }
                }
            }
        }
    }
}

void NAV::Plot::onDeleteLink([[maybe_unused]] Pin* startPin, Pin* endPin)
{
    LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(startPin->id), size_t(endPin->id));

    // Empty old pin data
    size_t pinIndex = pinIndexFromId(endPin->id);
    data.at(pinIndex).plotData.clear();
    data.at(pinIndex).dataIdentifier.clear();

    for (auto& plotInfo : plotInfos)
    {
        if (plotInfo.selectedXdata.size() > pinIndex)
        {
            plotInfo.selectedXdata.at(pinIndex) = 0;
        }
    }
}

void NAV::Plot::updateNumberOfInputPins()
{
    while (inputPins.size() < static_cast<size_t>(nInputPins))
    {
        nm::CreateInputPin(this, ("Pin " + std::to_string(inputPins.size() + 1)).c_str(), Pin::Type::Flow,
                           dataIdentifier, &Plot::plotData);
        data.emplace_back();
    }
    while (inputPins.size() > static_cast<size_t>(nInputPins))
    {
        for (auto& plotInfo : plotInfos)
        {
            if (plotInfo.selectedPin >= nInputPins)
            {
                plotInfo.selectedPin = nInputPins - 1;
            }
        }

        if (Link* connectedLink = nm::FindConnectedLinkToInputPin(inputPins.back().id))
        {
            nm::DeleteLink(connectedLink->id);
        }
        inputPins.pop_back();
        data.pop_back();
    }

    for (auto& plotInfo : plotInfos)
    {
        while (plotInfo.selectedXdata.size() < static_cast<size_t>(nInputPins))
        {
            plotInfo.selectedXdata.emplace_back(0);
        }
        while (plotInfo.selectedXdata.size() > static_cast<size_t>(nInputPins))
        {
            plotInfo.selectedXdata.pop_back();
        }
    }
}

void NAV::Plot::updateNumberOfPlots()
{
    while (static_cast<size_t>(nPlots) > plotInfos.size())
    {
        plotInfos.emplace_back("Plot " + std::to_string(plotInfos.size() + 1), nInputPins);
    }
    while (static_cast<size_t>(nPlots) < plotInfos.size())
    {
        plotInfos.pop_back();
    }
}

void NAV::Plot::addData(size_t pinIndex, size_t dataIndex, double value)
{
    auto& pinData = data.at(pinIndex);

    pinData.plotData.at(dataIndex).buffer.push_back(value);
    if (!std::isnan(value))
    {
        pinData.plotData.at(dataIndex).hasData = true;
    }
}

void NAV::Plot::plotBoolean(ax::NodeEditor::LinkId linkId)
{
    if (Link* link = nm::FindLink(linkId))
    {
        size_t pinIndex = pinIndexFromId(link->endPinId);

        auto currentTime = util::time::GetCurrentInsTime();
        auto* value = getInputValue<bool>(pinIndex);

        if (value != nullptr && !currentTime.empty())
        {
            if (std::isnan(startValue_Time))
            {
                startValue_Time = static_cast<double>(currentTime.toGPSweekTow().tow);
            }

            size_t i = 0;

            // InsObs
            addData(pinIndex, i++, static_cast<double>(currentTime.toGPSweekTow().tow) - startValue_Time);
            addData(pinIndex, i++, static_cast<double>(currentTime.toGPSweekTow().tow));
            // Boolean
            addData(pinIndex, i++, static_cast<double>(*value));
        }
    }
}

void NAV::Plot::plotInteger(ax::NodeEditor::LinkId linkId)
{
    if (Link* link = nm::FindLink(linkId))
    {
        size_t pinIndex = pinIndexFromId(link->endPinId);

        auto currentTime = util::time::GetCurrentInsTime();
        auto* value = getInputValue<int>(pinIndex);

        if (value != nullptr && !currentTime.empty())
        {
            if (std::isnan(startValue_Time))
            {
                startValue_Time = static_cast<double>(currentTime.toGPSweekTow().tow);
            }

            size_t i = 0;

            // InsObs
            addData(pinIndex, i++, static_cast<double>(currentTime.toGPSweekTow().tow) - startValue_Time);
            addData(pinIndex, i++, static_cast<double>(currentTime.toGPSweekTow().tow));
            // Integer
            addData(pinIndex, i++, static_cast<double>(*value));
        }
    }
}

void NAV::Plot::plotFloat(ax::NodeEditor::LinkId linkId)
{
    if (Link* link = nm::FindLink(linkId))
    {
        size_t pinIndex = pinIndexFromId(link->endPinId);

        auto currentTime = util::time::GetCurrentInsTime();
        auto* value = getInputValue<double>(pinIndex);

        if (value != nullptr && !currentTime.empty())
        {
            if (std::isnan(startValue_Time))
            {
                startValue_Time = static_cast<double>(currentTime.toGPSweekTow().tow);
            }

            size_t i = 0;

            // InsObs
            addData(pinIndex, i++, static_cast<double>(currentTime.toGPSweekTow().tow) - startValue_Time);
            addData(pinIndex, i++, static_cast<double>(currentTime.toGPSweekTow().tow));
            // Double
            addData(pinIndex, i++, *value);
        }
    }
}

void NAV::Plot::plotMatrix(ax::NodeEditor::LinkId linkId)
{
    if (Link* link = nm::FindLink(linkId))
    {
        if (Pin* sourcePin = nm::FindPin(link->startPinId))
        {
            size_t pinIndex = pinIndexFromId(link->endPinId);

            auto currentTime = util::time::GetCurrentInsTime();
            if (sourcePin->dataIdentifier.front() == "Eigen::MatrixXd")
            {
                auto* value = getInputValue<Eigen::MatrixXd>(pinIndex);

                if (value != nullptr && !currentTime.empty())
                {
                    if (std::isnan(startValue_Time))
                    {
                        startValue_Time = static_cast<double>(currentTime.toGPSweekTow().tow);
                    }

                    size_t i = 0;

                    // InsObs
                    addData(pinIndex, i++, static_cast<double>(currentTime.toGPSweekTow().tow) - startValue_Time);
                    addData(pinIndex, i++, static_cast<double>(currentTime.toGPSweekTow().tow));
                    // Matrix
                    for (int row = 0; row < value->rows(); row++)
                    {
                        for (int col = 0; col < value->cols(); col++)
                        {
                            addData(pinIndex, i++, (*value)(row, col));
                        }
                    }
                }
            }
            else if (sourcePin->dataIdentifier.front() == "BlockMatrix")
            {
                auto* value = getInputValue<BlockMatrix>(pinIndex);

                if (value != nullptr && !currentTime.empty())
                {
                    if (std::isnan(startValue_Time))
                    {
                        startValue_Time = static_cast<double>(currentTime.toGPSweekTow().tow);
                    }

                    size_t i = 0;

                    auto matrix = (*value)();

                    // InsObs
                    addData(pinIndex, i++, static_cast<double>(currentTime.toGPSweekTow().tow) - startValue_Time);
                    addData(pinIndex, i++, static_cast<double>(currentTime.toGPSweekTow().tow));
                    // Matrix
                    for (int row = 0; row < matrix.rows(); row++)
                    {
                        for (int col = 0; col < matrix.cols(); col++)
                        {
                            addData(pinIndex, i++, matrix(row, col));
                        }
                    }
                }
            }
        }
    }
}

void NAV::Plot::plotData(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::LinkId linkId)
{
    if (Link* link = nm::FindLink(linkId))
    {
        if (Pin* sourcePin = nm::FindPin(link->startPinId))
        {
            size_t pinIndex = pinIndexFromId(link->endPinId);

            if (sourcePin->dataIdentifier.front() == PosVelAtt::type()
                || sourcePin->dataIdentifier.front() == InertialNavSol::type())
            {
                plotPosVelAtt(std::static_pointer_cast<const PosVelAtt>(nodeData), pinIndex);
            }
            else if (sourcePin->dataIdentifier.front() == PVAError::type())
            {
                plotPVAError(std::static_pointer_cast<const PVAError>(nodeData), pinIndex);
            }
            else if (sourcePin->dataIdentifier.front() == ImuBiases::type())
            {
                plotImuBiases(std::static_pointer_cast<const ImuBiases>(nodeData), pinIndex);
            }
            else if (sourcePin->dataIdentifier.front() == RtklibPosObs::type())
            {
                plotRtklibPosObs(std::static_pointer_cast<const RtklibPosObs>(nodeData), pinIndex);
            }
            else if (sourcePin->dataIdentifier.front() == UbloxObs::type())
            {
                plotUbloxObs(std::static_pointer_cast<const UbloxObs>(nodeData), pinIndex);
            }
            else if (sourcePin->dataIdentifier.front() == ImuObs::type())
            {
                plotImuObs(std::static_pointer_cast<const ImuObs>(nodeData), pinIndex);
            }
            else if (sourcePin->dataIdentifier.front() == KvhObs::type())
            {
                plotKvhObs(std::static_pointer_cast<const KvhObs>(nodeData), pinIndex);
            }
            else if (sourcePin->dataIdentifier.front() == ImuObsWDelta::type())
            {
                plotImuObsWDeltaObs(std::static_pointer_cast<const ImuObsWDelta>(nodeData), pinIndex);
            }
            else if (sourcePin->dataIdentifier.front() == VectorNavBinaryOutput::type())
            {
                plotVectorNavBinaryObs(std::static_pointer_cast<const VectorNavBinaryOutput>(nodeData), pinIndex);
            }
        }
    }
}

void NAV::Plot::plotPosVelAtt(const std::shared_ptr<const PosVelAtt>& obs, size_t pinIndex)
{
    if (obs->insTime.has_value())
    {
        if (std::isnan(startValue_Time))
        {
            startValue_Time = static_cast<double>(obs->insTime.value().toGPSweekTow().tow);
        }
    }
    size_t i = 0;

    /// [𝜙, λ, h] Latitude, Longitude and altitude in [rad, rad, m]
    Eigen::Vector3d position_lla = obs->latLonAlt();

    if (std::isnan(startValue_North))
    {
        startValue_North = position_lla.x();
    }
    int sign = position_lla.x() > startValue_North ? 1 : -1;
    /// North/South deviation [m]
    double northSouth = measureDistance(position_lla.x(), position_lla.y(),
                                        startValue_North, position_lla.y())
                        * sign;

    if (std::isnan(startValue_East))
    {
        startValue_East = position_lla.y();
    }
    sign = position_lla.y() > startValue_East ? 1 : -1;
    /// East/West deviation [m]
    double eastWest = measureDistance(position_lla.x(), position_lla.y(),
                                      position_lla.x(), startValue_East)
                      * sign;

    // InsObs
    addData(pinIndex, i++, obs->insTime.has_value() ? static_cast<double>(obs->insTime->toGPSweekTow().tow) - startValue_Time : std::nan(""));
    addData(pinIndex, i++, obs->insTime.has_value() ? static_cast<double>(obs->insTime->toGPSweekTow().tow) : std::nan(""));
    // PosVelAtt
    addData(pinIndex, i++, trafo::rad2deg(position_lla(0)));
    addData(pinIndex, i++, trafo::rad2deg(position_lla(1)));
    addData(pinIndex, i++, position_lla(2));
    addData(pinIndex, i++, northSouth);
    addData(pinIndex, i++, eastWest);
    addData(pinIndex, i++, obs->position_ecef().x());
    addData(pinIndex, i++, obs->position_ecef().y());
    addData(pinIndex, i++, obs->position_ecef().z());
    addData(pinIndex, i++, obs->velocity_n().x());
    addData(pinIndex, i++, obs->velocity_n().y());
    addData(pinIndex, i++, obs->velocity_n().z());
    addData(pinIndex, i++, trafo::rad2deg(obs->rollPitchYaw().x()));
    addData(pinIndex, i++, trafo::rad2deg(obs->rollPitchYaw().y()));
    addData(pinIndex, i++, trafo::rad2deg(obs->rollPitchYaw().z()));
    addData(pinIndex, i++, obs->quaternion_nb().w());
    addData(pinIndex, i++, obs->quaternion_nb().x());
    addData(pinIndex, i++, obs->quaternion_nb().y());
    addData(pinIndex, i++, obs->quaternion_nb().z());
}

void NAV::Plot::plotPVAError(const std::shared_ptr<const PVAError>& obs, size_t pinIndex)
{
    if (obs->insTime.has_value())
    {
        if (std::isnan(startValue_Time))
        {
            startValue_Time = static_cast<double>(obs->insTime.value().toGPSweekTow().tow);
        }
    }
    size_t i = 0;

    // InsObs
    addData(pinIndex, i++, obs->insTime.has_value() ? static_cast<double>(obs->insTime->toGPSweekTow().tow) - startValue_Time : std::nan(""));
    addData(pinIndex, i++, obs->insTime.has_value() ? static_cast<double>(obs->insTime->toGPSweekTow().tow) : std::nan(""));
    // PVAError
    addData(pinIndex, i++, trafo::rad2deg(obs->attitudeError_n()(0)));
    addData(pinIndex, i++, trafo::rad2deg(obs->attitudeError_n()(1)));
    addData(pinIndex, i++, trafo::rad2deg(obs->attitudeError_n()(2)));
    addData(pinIndex, i++, obs->velocityError_n()(0));
    addData(pinIndex, i++, obs->velocityError_n()(1));
    addData(pinIndex, i++, obs->velocityError_n()(2));
    addData(pinIndex, i++, trafo::rad2deg(obs->positionError_lla()(0)) * 1e-3);
    addData(pinIndex, i++, trafo::rad2deg(obs->positionError_lla()(1)) * 1e-3);
    addData(pinIndex, i++, trafo::rad2deg(obs->positionError_lla()(2)));
}

void NAV::Plot::plotImuBiases(const std::shared_ptr<const ImuBiases>& obs, size_t pinIndex)
{
    if (obs->insTime.has_value())
    {
        if (std::isnan(startValue_Time))
        {
            startValue_Time = static_cast<double>(obs->insTime.value().toGPSweekTow().tow);
        }
    }
    size_t i = 0;

    // InsObs
    addData(pinIndex, i++, obs->insTime.has_value() ? static_cast<double>(obs->insTime->toGPSweekTow().tow) - startValue_Time : std::nan(""));
    addData(pinIndex, i++, obs->insTime.has_value() ? static_cast<double>(obs->insTime->toGPSweekTow().tow) : std::nan(""));
    // ImuBiases
    addData(pinIndex, i++, obs->biasAccel_b(0));
    addData(pinIndex, i++, obs->biasAccel_b(1));
    addData(pinIndex, i++, obs->biasAccel_b(2));
    addData(pinIndex, i++, obs->biasGyro_b(0));
    addData(pinIndex, i++, obs->biasGyro_b(1));
    addData(pinIndex, i++, obs->biasGyro_b(2));
}

void NAV::Plot::plotRtklibPosObs(const std::shared_ptr<const RtklibPosObs>& obs, size_t pinIndex)
{
    if (obs->insTime.has_value())
    {
        if (std::isnan(startValue_Time))
        {
            startValue_Time = static_cast<double>(obs->insTime.value().toGPSweekTow().tow);
        }
    }
    size_t i = 0;

    /// [𝜙, λ, h] Latitude, Longitude and altitude in [rad, rad, m]
    std::optional<Eigen::Vector3d> position_lla;
    /// North/South deviation [m]
    std::optional<double> northSouth;
    /// East/West deviation [m]
    std::optional<double> eastWest;
    if (obs->position_ecef.has_value())
    {
        position_lla = trafo::ecef2lla_WGS84(obs->position_ecef.value());

        if (std::isnan(startValue_North))
        {
            startValue_North = position_lla->x();
        }
        int sign = position_lla->x() > startValue_North ? 1 : -1;
        northSouth = measureDistance(position_lla->x(), position_lla->y(),
                                     startValue_North, position_lla->y())
                     * sign;

        if (std::isnan(startValue_East))
        {
            startValue_East = position_lla->y();
        }
        sign = position_lla->y() > startValue_East ? 1 : -1;
        eastWest = measureDistance(position_lla->x(), position_lla->y(),
                                   position_lla->x(), startValue_East)
                   * sign;

        position_lla->x() = trafo::rad2deg(position_lla->x());
        position_lla->y() = trafo::rad2deg(position_lla->y());
    }

    // InsObs
    addData(pinIndex, i++, obs->insTime.has_value() ? static_cast<double>(obs->insTime->toGPSweekTow().tow) - startValue_Time : std::nan(""));
    addData(pinIndex, i++, obs->insTime.has_value() ? static_cast<double>(obs->insTime->toGPSweekTow().tow) : std::nan(""));
    // RtklibPosObs
    addData(pinIndex, i++, obs->position_ecef.has_value() ? obs->position_ecef->x() : std::nan(""));
    addData(pinIndex, i++, obs->position_ecef.has_value() ? obs->position_ecef->y() : std::nan(""));
    addData(pinIndex, i++, obs->position_ecef.has_value() ? obs->position_ecef->z() : std::nan(""));
    addData(pinIndex, i++, position_lla.has_value() ? position_lla->x() : std::nan(""));
    addData(pinIndex, i++, position_lla.has_value() ? position_lla->y() : std::nan(""));
    addData(pinIndex, i++, position_lla.has_value() ? position_lla->z() : std::nan(""));
    addData(pinIndex, i++, northSouth.has_value() ? northSouth.value() : std::nan(""));
    addData(pinIndex, i++, eastWest.has_value() ? eastWest.value() : std::nan(""));
    addData(pinIndex, i++, obs->Q.has_value() ? obs->Q.value() : std::nan(""));
    addData(pinIndex, i++, obs->ns.has_value() ? obs->ns.value() : std::nan(""));
    addData(pinIndex, i++, obs->sdXYZ.has_value() ? obs->sdXYZ->x() : std::nan(""));
    addData(pinIndex, i++, obs->sdXYZ.has_value() ? obs->sdXYZ->y() : std::nan(""));
    addData(pinIndex, i++, obs->sdXYZ.has_value() ? obs->sdXYZ->z() : std::nan(""));
    addData(pinIndex, i++, obs->sdNEU.has_value() ? obs->sdNEU->x() : std::nan(""));
    addData(pinIndex, i++, obs->sdNEU.has_value() ? obs->sdNEU->y() : std::nan(""));
    addData(pinIndex, i++, obs->sdNEU.has_value() ? obs->sdNEU->z() : std::nan(""));
    addData(pinIndex, i++, obs->sdxy.has_value() ? obs->sdxy.value() : std::nan(""));
    addData(pinIndex, i++, obs->sdyz.has_value() ? obs->sdyz.value() : std::nan(""));
    addData(pinIndex, i++, obs->sdzx.has_value() ? obs->sdzx.value() : std::nan(""));
    addData(pinIndex, i++, obs->sdne.has_value() ? obs->sdne.value() : std::nan(""));
    addData(pinIndex, i++, obs->sdeu.has_value() ? obs->sdeu.value() : std::nan(""));
    addData(pinIndex, i++, obs->sdun.has_value() ? obs->sdun.value() : std::nan(""));
    addData(pinIndex, i++, obs->age.has_value() ? obs->age.value() : std::nan(""));
    addData(pinIndex, i++, obs->ratio.has_value() ? obs->ratio.value() : std::nan(""));
}

void NAV::Plot::plotUbloxObs(const std::shared_ptr<const UbloxObs>& obs, size_t pinIndex)
{
    if (obs->insTime.has_value())
    {
        if (std::isnan(startValue_Time))
        {
            startValue_Time = static_cast<double>(obs->insTime.value().toGPSweekTow().tow);
        }
    }

    /// Position in ECEF coordinates in [m]
    std::optional<Eigen::Vector3d> position_ecef;
    /// [𝜙, λ, h] Latitude, Longitude and altitude in [rad, rad, m]
    std::optional<Eigen::Vector3d> position_lla;
    /// Velocity in NED coordinates in [m/s]
    std::optional<Eigen::Vector3d> velocity_ned;

    if (obs->msgClass == sensors::ublox::UbxClass::UBX_CLASS_NAV)
    {
        auto msgId = static_cast<sensors::ublox::UbxNavMessages>(obs->msgId);
        if (msgId == sensors::ublox::UbxNavMessages::UBX_NAV_POSECEF)
        {
            position_ecef.emplace(std::get<sensors::ublox::UbxNavPosecef>(obs->data).ecefX * 1e-2,
                                  std::get<sensors::ublox::UbxNavPosecef>(obs->data).ecefY * 1e-2,
                                  std::get<sensors::ublox::UbxNavPosecef>(obs->data).ecefZ * 1e-2);
            position_lla = trafo::ecef2lla_WGS84(position_ecef.value());
        }
        else if (msgId == sensors::ublox::UbxNavMessages::UBX_NAV_POSLLH)
        {
            position_lla.emplace(trafo::deg2rad(std::get<sensors::ublox::UbxNavPosllh>(obs->data).lat * 1e-7),
                                 trafo::deg2rad(std::get<sensors::ublox::UbxNavPosllh>(obs->data).lon * 1e-7),
                                 std::get<sensors::ublox::UbxNavPosllh>(obs->data).height * 1e-3);
        }
        else if (msgId == sensors::ublox::UbxNavMessages::UBX_NAV_VELNED)
        {
            velocity_ned.emplace(std::get<sensors::ublox::UbxNavVelned>(obs->data).velN * 1e-2,
                                 std::get<sensors::ublox::UbxNavVelned>(obs->data).velE * 1e-2,
                                 std::get<sensors::ublox::UbxNavVelned>(obs->data).velD * 1e-2);
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
    /// North/South deviation [m]
    std::optional<double> northSouth;
    /// East/West deviation [m]
    std::optional<double> eastWest;

    if (position_lla.has_value())
    {
        if (std::isnan(startValue_North))
        {
            startValue_North = position_lla->x();
        }
        int sign = position_lla->x() > startValue_North ? 1 : -1;
        northSouth = measureDistance(position_lla->x(), position_lla->y(),
                                     startValue_North, position_lla->y())
                     * sign;

        if (std::isnan(startValue_East))
        {
            startValue_East = position_lla->y();
        }
        sign = position_lla->y() > startValue_East ? 1 : -1;
        eastWest = measureDistance(position_lla->x(), position_lla->y(),
                                   position_lla->x(), startValue_East)
                   * sign;

        position_lla->x() = trafo::rad2deg(position_lla->x());
        position_lla->y() = trafo::rad2deg(position_lla->y());
    }

    size_t i = 0;
    // InsObs
    addData(pinIndex, i++, obs->insTime.has_value() ? static_cast<double>(obs->insTime->toGPSweekTow().tow) - startValue_Time : std::nan(""));
    addData(pinIndex, i++, obs->insTime.has_value() ? static_cast<double>(obs->insTime->toGPSweekTow().tow) : std::nan(""));
    // UbloxObs
    addData(pinIndex, i++, position_ecef.has_value() ? position_ecef->x() : std::nan(""));
    addData(pinIndex, i++, position_ecef.has_value() ? position_ecef->y() : std::nan(""));
    addData(pinIndex, i++, position_ecef.has_value() ? position_ecef->z() : std::nan(""));
    addData(pinIndex, i++, position_lla.has_value() ? position_lla->x() : std::nan(""));
    addData(pinIndex, i++, position_lla.has_value() ? position_lla->y() : std::nan(""));
    addData(pinIndex, i++, position_lla.has_value() ? position_lla->z() : std::nan(""));
    addData(pinIndex, i++, northSouth.has_value() ? northSouth.value() : std::nan(""));
    addData(pinIndex, i++, eastWest.has_value() ? eastWest.value() : std::nan(""));
    addData(pinIndex, i++, velocity_ned.has_value() ? velocity_ned->x() : std::nan(""));
    addData(pinIndex, i++, velocity_ned.has_value() ? velocity_ned->y() : std::nan(""));
    addData(pinIndex, i++, velocity_ned.has_value() ? velocity_ned->z() : std::nan(""));
}

void NAV::Plot::plotImuObs(const std::shared_ptr<const ImuObs>& obs, size_t pinIndex)
{
    if (obs->insTime.has_value())
    {
        if (std::isnan(startValue_Time))
        {
            startValue_Time = static_cast<double>(obs->insTime.value().toGPSweekTow().tow);
        }
    }
    size_t i = 0;

    // InsObs
    addData(pinIndex, i++, obs->insTime.has_value() ? static_cast<double>(obs->insTime->toGPSweekTow().tow) - startValue_Time : std::nan(""));
    addData(pinIndex, i++, obs->insTime.has_value() ? static_cast<double>(obs->insTime->toGPSweekTow().tow) : std::nan(""));
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

void NAV::Plot::plotKvhObs(const std::shared_ptr<const KvhObs>& obs, size_t pinIndex)
{
    if (obs->insTime.has_value())
    {
        if (std::isnan(startValue_Time))
        {
            startValue_Time = static_cast<double>(obs->insTime.value().toGPSweekTow().tow);
        }
    }
    size_t i = 0;

    // InsObs
    addData(pinIndex, i++, obs->insTime.has_value() ? static_cast<double>(obs->insTime->toGPSweekTow().tow) - startValue_Time : std::nan(""));
    addData(pinIndex, i++, obs->insTime.has_value() ? static_cast<double>(obs->insTime->toGPSweekTow().tow) : std::nan(""));
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
    if (obs->insTime.has_value())
    {
        if (std::isnan(startValue_Time))
        {
            startValue_Time = static_cast<double>(obs->insTime.value().toGPSweekTow().tow);
        }
    }
    size_t i = 0;

    // InsObs
    addData(pinIndex, i++, obs->insTime.has_value() ? static_cast<double>(obs->insTime->toGPSweekTow().tow) - startValue_Time : std::nan(""));
    addData(pinIndex, i++, obs->insTime.has_value() ? static_cast<double>(obs->insTime->toGPSweekTow().tow) : std::nan(""));
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
    if (obs->insTime.has_value())
    {
        if (std::isnan(startValue_Time))
        {
            startValue_Time = static_cast<double>(obs->insTime.value().toGPSweekTow().tow);
        }
    }
    size_t i = 0;

    // InsObs
    addData(pinIndex, i++, obs->insTime.has_value() ? static_cast<double>(obs->insTime->toGPSweekTow().tow) - startValue_Time : std::nan(""));
    addData(pinIndex, i++, obs->insTime.has_value() ? static_cast<double>(obs->insTime->toGPSweekTow().tow) : std::nan(""));
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