#include "Plot.hpp"

#include "util/Logger.hpp"

#include "implot.h"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "gui/widgets/Splitter.hpp"

#include "util/Time/TimeBase.hpp"
#include "util/InsTransformations.hpp"
#include "util/InsMath.hpp"
#include <algorithm>

namespace NAV
{
void to_json(json& j, const Plot::PinData::PlotData& data)
{
    j = json{
        { "plotOnAxis", data.plotOnAxis },
        { "displayName", data.displayName },
    };
}
void from_json(const json& j, Plot::PinData::PlotData& data)
{
    if (j.contains("plotOnAxis"))
    {
        j.at("plotOnAxis").get_to(data.plotOnAxis);
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
        { "plotStyle", data.plotStyle },
        { "pinType", data.pinType },
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
    if (j.contains("plotStyle"))
    {
        j.at("plotStyle").get_to(data.plotStyle);
    }
    if (j.contains("pinType"))
    {
        j.at("pinType").get_to(data.pinType);
    }
}

void to_json(json& j, const Plot::PlotInfo& data)
{
    j = json{
        { "autoLimitXaxis", data.autoLimitXaxis },
        { "autoLimitYaxis", data.autoLimitYaxis },
        { "headerText", data.headerText },
        { "leftPaneWidth", data.leftPaneWidth },
        { "plotFlags", data.plotFlags },
        { "rightPaneWidth", data.rightPaneWidth },
        { "selectedPin", data.selectedPin },
        { "selectedXdata", data.selectedXdata },
        { "title", data.title },
    };
}
void from_json(const json& j, Plot::PlotInfo& data)
{
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
}

} // namespace NAV

NAV::Plot::Plot()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    color = ImColor(255, 128, 128);
    hasConfig = true;

    dataIdentifier = { RtklibPosObs::type(), UbloxObs::type(),
                       ImuObs::type(), KvhObs::type(), VectorNavObs::type() };

    updateNumberOfInputPins();

    // PinData::PinType::Flow:
    data.at(0).pinType = PinData::PinType::Flow;
    inputPins.at(0).type = Pin::Type::Flow;
    inputPins.at(0).dataIdentifier = dataIdentifier;
    inputPins.at(0).data = Pin::PinData(static_cast<void (Node::*)(const std::shared_ptr<NodeData>&, ax::NodeEditor::LinkId)>(&Plot::plotData));
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
                              ImGuiTableFlags_Borders | ImGuiTableFlags_ColumnsWidthFixed, ImVec2(0.0F, 0.0F)))
        {
            ImGui::TableSetupColumn("Pin");
            ImGui::TableSetupColumn("Pin Type");
            ImGui::TableSetupColumn("# Data Points");
            ImGui::TableSetupColumn("Plot Style");
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
                        inputPins.at(pinIndex).data = Pin::PinData(static_cast<void (Node::*)(const std::shared_ptr<NodeData>&, ax::NodeEditor::LinkId)>(&Plot::plotData));
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

                ImGui::TableNextColumn(); // Plot Style
                ImGui::SetNextItemWidth(100.0F);
                if (ImGui::Combo(("##Plot Style for Pin " + std::to_string(pinIndex + 1) + " - " + std::to_string(size_t(id))).c_str(),
                                 reinterpret_cast<int*>(&pinData.plotStyle), "Line\0Scatter\0\0"))
                {
                    flow::ApplyChanges();
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
            ImGui::SetNextItemOpen(true, ImGuiCond_Once);
            if (ImGui::TreeNode(("Options##" + std::to_string(size_t(id)) + " - " + std::to_string(plotNum)).c_str()))
            {
                ImGui::InputText(("Title##" + std::to_string(size_t(id)) + " - " + std::to_string(plotNum)).c_str(), &plotInfo.title);
                if (plotInfo.headerText != plotInfo.title && !ImGui::IsItemActive())
                {
                    plotInfo.headerText = plotInfo.title;
                    flow::ApplyChanges();
                    LOG_DEBUG("{}: # Header changed to {}", nameId(), plotInfo.headerText);
                }
                if (ImGui::BeginTable(("Pin Settings##" + std::to_string(size_t(id)) + " - " + std::to_string(plotNum)).c_str(), 2,
                                      ImGuiTableFlags_Borders | ImGuiTableFlags_ColumnsWidthFixed, ImVec2(0.0F, 0.0F)))
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
                                   true, 4.0F, &plotInfo.leftPaneWidth, &plotInfo.rightPaneWidth, 150.0F, 80.0F, ImPlot::GetStyle().PlotDefaultSize.y);

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
                              ImVec2(plotInfo.leftPaneWidth - 2.0F, ImPlot::GetStyle().PlotDefaultSize.y - comboBoxSize.y - buttonSize.y - 2 * ImGui::GetStyle().ItemSpacing.y),
                              true);

            // Left Data Selectables
            for (auto& plotData : data.at(static_cast<size_t>(plotInfo.selectedPin)).plotData)
            {
                if (!plotData.hasData)
                {
                    ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5F);
                }
                std::string label = plotData.displayName;
                if (plotData.plotOnAxis.contains(plotNum))
                {
                    label += fmt::format(" (Y{})", plotData.plotOnAxis.at(plotNum) + 1);
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

                if (!plotData.hasData)
                {
                    ImGui::PopStyleVar();
                }
            }

            ImGui::EndChild();
            ImGui::EndGroup();

            ImGui::SameLine();

            std::string xLabel = !data.at(0).plotData.empty() ? data.at(0).plotData.at(plotInfo.selectedXdata.at(0)).displayName : "";
            ImPlot::FitNextPlotAxes(plotInfo.autoLimitXaxis, plotInfo.autoLimitYaxis, plotInfo.autoLimitYaxis, plotInfo.autoLimitYaxis);
            if (ImPlot::BeginPlot((plotInfo.title + "##" + std::to_string(size_t(id)) + " - " + std::to_string(plotNum)).c_str(),
                                  xLabel.c_str(), nullptr, ImVec2(-1, 0), plotInfo.plotFlags))
            {
                for (size_t pinIndex = 0; pinIndex < data.size(); pinIndex++)
                {
                    for (auto& plotData : data.at(pinIndex).plotData)
                    {
                        if (plotData.plotOnAxis.contains(plotNum)
                            && plotData.hasData
                            && (plotData.plotOnAxis.at(plotNum) == ImPlotYAxis_1
                                || (plotData.plotOnAxis.at(plotNum) == ImPlotYAxis_2 && (plotInfo.plotFlags & ImPlotFlags_YAxis2))
                                || (plotData.plotOnAxis.at(plotNum) == ImPlotYAxis_3 && (plotInfo.plotFlags & ImPlotFlags_YAxis3))))
                        {
                            ImPlot::SetPlotYAxis(plotData.plotOnAxis.at(plotNum));
                            if (data.at(pinIndex).plotStyle == PinData::PlotStyle::Line)
                            {
                                ImPlot::PlotLine((plotData.displayName + " (" + std::to_string(pinIndex + 1) + " - " + data.at(pinIndex).dataIdentifier + ")").c_str(),
                                                 data.at(pinIndex).plotData.at(plotInfo.selectedXdata.at(pinIndex)).buffer.data(),
                                                 plotData.buffer.data(),
                                                 static_cast<int>(plotData.buffer.size()),
                                                 plotData.buffer.offset(), sizeof(double));
                            }
                            else if (data.at(pinIndex).plotStyle == PinData::PlotStyle::Scatter)
                            {
                                ImPlot::SetNextMarkerStyle(ImPlotMarker_Cross, 2, ImVec4(0, 0, 0, -1), IMPLOT_AUTO, ImVec4(0, 0, 0, -1));
                                ImPlot::PlotScatter((plotData.displayName + " (" + std::to_string(pinIndex + 1) + " - " + data.at(pinIndex).dataIdentifier + ")").c_str(),
                                                    data.at(pinIndex).plotData.at(plotInfo.selectedXdata.at(pinIndex)).buffer.data(),
                                                    plotData.buffer.data(),
                                                    static_cast<int>(plotData.buffer.size()),
                                                    plotData.buffer.offset(), sizeof(double));
                            }
                            // allow legend labels to be dragged and dropped
                            if (ImPlot::BeginLegendDragDropSource((plotData.displayName + " (" + std::to_string(pinIndex + 1) + " - " + data.at(pinIndex).dataIdentifier + ")").c_str()))
                            {
                                auto* ptrPlotData = &plotData;
                                ImGui::SetDragDropPayload(("DND_DATA " + std::to_string(size_t(id)) + " - " + std::to_string(plotNum)).c_str(),
                                                          &ptrPlotData, sizeof(PinData::PlotData*));
                                ImGui::TextUnformatted(plotData.displayName.c_str());
                                ImPlot::EndLegendDragDropSource();
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

                        plotData->plotOnAxis[plotNum] = 0;
                        // set specific y-axis if hovered
                        for (int y = 0; y < 3; y++)
                        {
                            if (ImPlot::IsPlotYAxisHovered(y))
                            {
                                plotData->plotOnAxis[plotNum] = y;
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
            case Plot::PinData::PinType::Bool:
                inputPins.at(inputPinIndex).notifyFunc.emplace_back(this, static_cast<void (Node::*)(ax::NodeEditor::LinkId)>(&Plot::plotBoolean), 0);
                break;
            case Plot::PinData::PinType::Int:
                inputPins.at(inputPinIndex).notifyFunc.emplace_back(this, static_cast<void (Node::*)(ax::NodeEditor::LinkId)>(&Plot::plotInteger), 0);
                break;
            case Plot::PinData::PinType::Float:
                inputPins.at(inputPinIndex).notifyFunc.emplace_back(this, static_cast<void (Node::*)(ax::NodeEditor::LinkId)>(&Plot::plotFloat), 0);
                break;
            case Plot::PinData::PinType::Matrix:
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

    if (inputPins.at(pinIndex).type == Pin::Type::Flow)
    {
        data.at(pinIndex).dataIdentifier = startPin->dataIdentifier.front();

        if (startPin->dataIdentifier.front() == RtklibPosObs::type())
        {
            // InsObs
            data.at(pinIndex).addPlotDataItem("Time [s]");
            data.at(pinIndex).addPlotDataItem("GPS time of week [s]");
            // RtklibPosObs
            data.at(pinIndex).addPlotDataItem("X-ECEF [m]");
            data.at(pinIndex).addPlotDataItem("Y-ECEF [m]");
            data.at(pinIndex).addPlotDataItem("Z-ECEF [m]");
            data.at(pinIndex).addPlotDataItem("Latitude [deg]");
            data.at(pinIndex).addPlotDataItem("Longitude [deg]");
            data.at(pinIndex).addPlotDataItem("Altitude [m]");
            data.at(pinIndex).addPlotDataItem("North/South [m]");
            data.at(pinIndex).addPlotDataItem("East/West [m]");
            data.at(pinIndex).addPlotDataItem("Q [-]");
            data.at(pinIndex).addPlotDataItem("ns [-]");
            data.at(pinIndex).addPlotDataItem("sdx [m]");
            data.at(pinIndex).addPlotDataItem("sdy [m]");
            data.at(pinIndex).addPlotDataItem("sdz [m]");
            data.at(pinIndex).addPlotDataItem("sdn [m]");
            data.at(pinIndex).addPlotDataItem("sde [m]");
            data.at(pinIndex).addPlotDataItem("sdu [m]");
            data.at(pinIndex).addPlotDataItem("sdxy [m]");
            data.at(pinIndex).addPlotDataItem("sdyz [m]");
            data.at(pinIndex).addPlotDataItem("sdzx [m]");
            data.at(pinIndex).addPlotDataItem("sdne [m]");
            data.at(pinIndex).addPlotDataItem("sdeu [m]");
            data.at(pinIndex).addPlotDataItem("sdun [m]");
            data.at(pinIndex).addPlotDataItem("age [s]");
            data.at(pinIndex).addPlotDataItem("ratio [-]");
        }
        else if (startPin->dataIdentifier.front() == UbloxObs::type())
        {
            // InsObs
            data.at(pinIndex).addPlotDataItem("Time [s]");
            data.at(pinIndex).addPlotDataItem("GPS time of week [s]");
            // UbloxObs
            data.at(pinIndex).addPlotDataItem("X-ECEF [m]");
            data.at(pinIndex).addPlotDataItem("Y-ECEF [m]");
            data.at(pinIndex).addPlotDataItem("Z-ECEF [m]");
            data.at(pinIndex).addPlotDataItem("Latitude [deg]");
            data.at(pinIndex).addPlotDataItem("Longitude [deg]");
            data.at(pinIndex).addPlotDataItem("Altitude [m]");
            data.at(pinIndex).addPlotDataItem("North/South [m]");
            data.at(pinIndex).addPlotDataItem("East/West [m]");
        }
        else if (startPin->dataIdentifier.front() == ImuObs::type())
        {
            // InsObs
            data.at(pinIndex).addPlotDataItem("Time [s]");
            data.at(pinIndex).addPlotDataItem("GPS time of week [s]");
            // ImuObs
            data.at(pinIndex).addPlotDataItem("Time since startup [ns]");
            data.at(pinIndex).addPlotDataItem("Mag uncomp X [Gauss]");
            data.at(pinIndex).addPlotDataItem("Mag uncomp Y [Gauss]");
            data.at(pinIndex).addPlotDataItem("Mag uncomp Z [Gauss]");
            data.at(pinIndex).addPlotDataItem("Accel uncomp X [m/s^2]");
            data.at(pinIndex).addPlotDataItem("Accel uncomp Y [m/s^2]");
            data.at(pinIndex).addPlotDataItem("Accel uncomp Z [m/s^2]");
            data.at(pinIndex).addPlotDataItem("Gyro uncomp X [rad/s]");
            data.at(pinIndex).addPlotDataItem("Gyro uncomp Y [rad/s]");
            data.at(pinIndex).addPlotDataItem("Gyro uncomp Z [rad/s]");
            data.at(pinIndex).addPlotDataItem("Temperature [°C]");
        }
        else if (startPin->dataIdentifier.front() == KvhObs::type())
        {
            // InsObs
            data.at(pinIndex).addPlotDataItem("Time [s]");
            data.at(pinIndex).addPlotDataItem("GPS time of week [s]");
            // ImuObs
            data.at(pinIndex).addPlotDataItem("Time since startup [ns]");
            data.at(pinIndex).addPlotDataItem("Mag uncomp X [Gauss]");
            data.at(pinIndex).addPlotDataItem("Mag uncomp Y [Gauss]");
            data.at(pinIndex).addPlotDataItem("Mag uncomp Z [Gauss]");
            data.at(pinIndex).addPlotDataItem("Accel uncomp X [m/s^2]");
            data.at(pinIndex).addPlotDataItem("Accel uncomp Y [m/s^2]");
            data.at(pinIndex).addPlotDataItem("Accel uncomp Z [m/s^2]");
            data.at(pinIndex).addPlotDataItem("Gyro uncomp X [rad/s]");
            data.at(pinIndex).addPlotDataItem("Gyro uncomp Y [rad/s]");
            data.at(pinIndex).addPlotDataItem("Gyro uncomp Z [rad/s]");
            data.at(pinIndex).addPlotDataItem("Temperature [°C]");
            // KvhObs
            data.at(pinIndex).addPlotDataItem("Status [bits]");
            data.at(pinIndex).addPlotDataItem("Sequence Number [.]");
        }
        else if (startPin->dataIdentifier.front() == VectorNavObs::type())
        {
            // InsObs
            data.at(pinIndex).addPlotDataItem("Time [s]");
            data.at(pinIndex).addPlotDataItem("GPS time of week [s]");
            // ImuObs
            data.at(pinIndex).addPlotDataItem("Time since startup [ns]");
            data.at(pinIndex).addPlotDataItem("Mag uncomp X [Gauss]");
            data.at(pinIndex).addPlotDataItem("Mag uncomp Y [Gauss]");
            data.at(pinIndex).addPlotDataItem("Mag uncomp Z [Gauss]");
            data.at(pinIndex).addPlotDataItem("Accel uncomp X [m/s^2]");
            data.at(pinIndex).addPlotDataItem("Accel uncomp Y [m/s^2]");
            data.at(pinIndex).addPlotDataItem("Accel uncomp Z [m/s^2]");
            data.at(pinIndex).addPlotDataItem("Gyro uncomp X [rad/s]");
            data.at(pinIndex).addPlotDataItem("Gyro uncomp Y [rad/s]");
            data.at(pinIndex).addPlotDataItem("Gyro uncomp Z [rad/s]");
            data.at(pinIndex).addPlotDataItem("Temperature [°C]");
            // VectorNavObs
            data.at(pinIndex).addPlotDataItem("Quaternion W []");
            data.at(pinIndex).addPlotDataItem("Quaternion X []");
            data.at(pinIndex).addPlotDataItem("Quaternion Y []");
            data.at(pinIndex).addPlotDataItem("Quaternion Z []");
            data.at(pinIndex).addPlotDataItem("Yaw [deg]");
            data.at(pinIndex).addPlotDataItem("Pitch [deg]");
            data.at(pinIndex).addPlotDataItem("Roll [deg]");
            data.at(pinIndex).addPlotDataItem("Time since syncIn [ns]");
            data.at(pinIndex).addPlotDataItem("SyncIn Count []");
            data.at(pinIndex).addPlotDataItem("Mag comp X [Gauss]");
            data.at(pinIndex).addPlotDataItem("Mag comp Y [Gauss]");
            data.at(pinIndex).addPlotDataItem("Mag comp Z [Gauss]");
            data.at(pinIndex).addPlotDataItem("Accel comp X [m/s^2]");
            data.at(pinIndex).addPlotDataItem("Accel comp Y [m/s^2]");
            data.at(pinIndex).addPlotDataItem("Accel comp Z [m/s^2]");
            data.at(pinIndex).addPlotDataItem("Gyro comp X [rad/s]");
            data.at(pinIndex).addPlotDataItem("Gyro comp Y [rad/s]");
            data.at(pinIndex).addPlotDataItem("Gyro comp Z [rad/s]");
            data.at(pinIndex).addPlotDataItem("dTime [s]");
            data.at(pinIndex).addPlotDataItem("dTheta X [deg]");
            data.at(pinIndex).addPlotDataItem("dTheta Y [deg]");
            data.at(pinIndex).addPlotDataItem("dTheta Z [deg]");
            data.at(pinIndex).addPlotDataItem("dVelocity X [m/s]");
            data.at(pinIndex).addPlotDataItem("dVelocity Y [m/s]");
            data.at(pinIndex).addPlotDataItem("dVelocity Z [m/s]");
            data.at(pinIndex).addPlotDataItem("VPE Status [bits]");
            data.at(pinIndex).addPlotDataItem("Pressure [kPa]");
            data.at(pinIndex).addPlotDataItem("Mag comp N [Gauss]");
            data.at(pinIndex).addPlotDataItem("Mag comp E [Gauss]");
            data.at(pinIndex).addPlotDataItem("Mag comp D [Gauss]");
            data.at(pinIndex).addPlotDataItem("Accel comp N [m/s^2]");
            data.at(pinIndex).addPlotDataItem("Accel comp E [m/s^2]");
            data.at(pinIndex).addPlotDataItem("Accel comp D [m/s^2]");
            data.at(pinIndex).addPlotDataItem("Gyro comp N [rad/s]");
            data.at(pinIndex).addPlotDataItem("Gyro comp E [rad/s]");
            data.at(pinIndex).addPlotDataItem("Gyro comp D [rad/s]");
            data.at(pinIndex).addPlotDataItem("Linear Accel X [m/s^2]");
            data.at(pinIndex).addPlotDataItem("Linear Accel Y [m/s^2]");
            data.at(pinIndex).addPlotDataItem("Linear Accel Z [m/s^2]");
            data.at(pinIndex).addPlotDataItem("Linear Accel N [m/s^2]");
            data.at(pinIndex).addPlotDataItem("Linear Accel E [m/s^2]");
            data.at(pinIndex).addPlotDataItem("Linear Accel D [m/s^2]");
        }
    }
    else if (inputPins.at(pinIndex).type == Pin::Type::Bool)
    {
        data.at(pinIndex).dataIdentifier = startPin->name;

        // InsObs
        data.at(pinIndex).addPlotDataItem("Time [s]");
        data.at(pinIndex).addPlotDataItem("GPS time of week [s]");
        // Bool
        data.at(pinIndex).addPlotDataItem("Boolean");
    }
    else if (inputPins.at(pinIndex).type == Pin::Type::Int)
    {
        data.at(pinIndex).dataIdentifier = startPin->name;

        // InsObs
        data.at(pinIndex).addPlotDataItem("Time [s]");
        data.at(pinIndex).addPlotDataItem("GPS time of week [s]");
        // Int
        data.at(pinIndex).addPlotDataItem("Integer");
    }
    else if (inputPins.at(pinIndex).type == Pin::Type::Float)
    {
        data.at(pinIndex).dataIdentifier = startPin->name;

        // InsObs
        data.at(pinIndex).addPlotDataItem("Time [s]");
        data.at(pinIndex).addPlotDataItem("GPS time of week [s]");
        // Float
        data.at(pinIndex).addPlotDataItem("Float");
    }
    else if (inputPins.at(pinIndex).type == Pin::Type::Matrix)
    {
        data.at(pinIndex).dataIdentifier = startPin->name;

        // InsObs
        data.at(pinIndex).addPlotDataItem("Time [s]");
        data.at(pinIndex).addPlotDataItem("GPS time of week [s]");
        // Matrix
        if (startPin->dataIdentifier.front() == "Eigen::MatrixXd")
        {
            if (auto* matrix = getInputValue<Eigen::MatrixXd>(pinIndex))
            {
                for (int row = 0; row < matrix->rows(); row++)
                {
                    for (int col = 0; col < matrix->cols(); col++)
                    {
                        data.at(pinIndex).addPlotDataItem(std::to_string(row) + ", " + std::to_string(col));
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
                        data.at(pinIndex).addPlotDataItem(std::to_string(row) + ", " + std::to_string(col));
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

    pinData.plotData.at(dataIndex).buffer.AddValue(value);
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

        LOG_DATA("{}: called on pin {}", nameId(), pinIndex);

        auto currentTime = util::time::GetCurrentTime();
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

        LOG_DATA("{}: called on pin {}", nameId(), pinIndex);

        auto currentTime = util::time::GetCurrentTime();
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

        LOG_DATA("{}: called on pin {}", nameId(), pinIndex);

        auto currentTime = util::time::GetCurrentTime();
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

            LOG_DATA("{}: called on pin {}", nameId(), pinIndex);

            auto currentTime = util::time::GetCurrentTime();
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
                    addData(pinIndex, i++, matrix(0, 0));
                }
            }
        }
    }
}

void NAV::Plot::plotData(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId linkId)
{
    if (Link* link = nm::FindLink(linkId))
    {
        if (Pin* sourcePin = nm::FindPin(link->startPinId))
        {
            size_t pinIndex = pinIndexFromId(link->endPinId);

            if (sourcePin->dataIdentifier.front() == RtklibPosObs::type())
            {
                plotRtklibPosObs(std::static_pointer_cast<RtklibPosObs>(nodeData), pinIndex);
            }
            else if (sourcePin->dataIdentifier.front() == UbloxObs::type())
            {
                plotUbloxObs(std::static_pointer_cast<UbloxObs>(nodeData), pinIndex);
            }
            else if (sourcePin->dataIdentifier.front() == ImuObs::type())
            {
                plotImuObs(std::static_pointer_cast<ImuObs>(nodeData), pinIndex);
            }
            else if (sourcePin->dataIdentifier.front() == KvhObs::type())
            {
                plotKvhObs(std::static_pointer_cast<KvhObs>(nodeData), pinIndex);
            }
            else if (sourcePin->dataIdentifier.front() == VectorNavObs::type())
            {
                plotVectorNavObs(std::static_pointer_cast<VectorNavObs>(nodeData), pinIndex);
            }
        }
    }
}

void NAV::Plot::plotRtklibPosObs(const std::shared_ptr<RtklibPosObs>& obs, size_t pinIndex)
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

void NAV::Plot::plotUbloxObs(const std::shared_ptr<UbloxObs>& obs, size_t pinIndex)
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
    }
}

void NAV::Plot::plotImuObs(const std::shared_ptr<ImuObs>& obs, size_t pinIndex)
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
    addData(pinIndex, i++, obs->temperature.has_value() ? obs->temperature.value() : std::nan(""));
}

void NAV::Plot::plotKvhObs(const std::shared_ptr<KvhObs>& obs, size_t pinIndex)
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
    addData(pinIndex, i++, obs->temperature.has_value() ? obs->temperature.value() : std::nan(""));
    // KvhObs
    addData(pinIndex, i++, static_cast<double>(obs->status.to_ulong()));
    addData(pinIndex, i++, obs->sequenceNumber < 128 ? obs->sequenceNumber : std::nan(""));
}

void NAV::Plot::plotVectorNavObs(const std::shared_ptr<VectorNavObs>& obs, size_t pinIndex)
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
    addData(pinIndex, i++, obs->temperature.has_value() ? obs->temperature.value() : std::nan(""));
    // VectorNavObs
    addData(pinIndex, i++, obs->quaternion.has_value() ? obs->quaternion->w() : std::nan(""));
    addData(pinIndex, i++, obs->quaternion.has_value() ? obs->quaternion->x() : std::nan(""));
    addData(pinIndex, i++, obs->quaternion.has_value() ? obs->quaternion->y() : std::nan(""));
    addData(pinIndex, i++, obs->quaternion.has_value() ? obs->quaternion->z() : std::nan(""));
    addData(pinIndex, i++, obs->yawPitchRoll.has_value() ? obs->yawPitchRoll->x() : std::nan(""));
    addData(pinIndex, i++, obs->yawPitchRoll.has_value() ? obs->yawPitchRoll->y() : std::nan(""));
    addData(pinIndex, i++, obs->yawPitchRoll.has_value() ? obs->yawPitchRoll->z() : std::nan(""));
    addData(pinIndex, i++, obs->timeSinceSyncIn.has_value() ? static_cast<double>(obs->timeSinceSyncIn.value()) : std::nan(""));
    addData(pinIndex, i++, obs->syncInCnt.has_value() ? obs->syncInCnt.value() : std::nan(""));
    addData(pinIndex, i++, obs->magCompXYZ.has_value() ? obs->magCompXYZ->x() : std::nan(""));
    addData(pinIndex, i++, obs->magCompXYZ.has_value() ? obs->magCompXYZ->y() : std::nan(""));
    addData(pinIndex, i++, obs->magCompXYZ.has_value() ? obs->magCompXYZ->z() : std::nan(""));
    addData(pinIndex, i++, obs->accelCompXYZ.has_value() ? obs->accelCompXYZ->x() : std::nan(""));
    addData(pinIndex, i++, obs->accelCompXYZ.has_value() ? obs->accelCompXYZ->y() : std::nan(""));
    addData(pinIndex, i++, obs->accelCompXYZ.has_value() ? obs->accelCompXYZ->z() : std::nan(""));
    addData(pinIndex, i++, obs->gyroCompXYZ.has_value() ? obs->gyroCompXYZ->x() : std::nan(""));
    addData(pinIndex, i++, obs->gyroCompXYZ.has_value() ? obs->gyroCompXYZ->y() : std::nan(""));
    addData(pinIndex, i++, obs->gyroCompXYZ.has_value() ? obs->gyroCompXYZ->z() : std::nan(""));
    addData(pinIndex, i++, obs->dtime.has_value() ? obs->dtime.value() : std::nan(""));
    addData(pinIndex, i++, obs->dtheta.has_value() ? obs->dtheta->x() : std::nan(""));
    addData(pinIndex, i++, obs->dtheta.has_value() ? obs->dtheta->y() : std::nan(""));
    addData(pinIndex, i++, obs->dtheta.has_value() ? obs->dtheta->z() : std::nan(""));
    addData(pinIndex, i++, obs->dvel.has_value() ? obs->dvel->x() : std::nan(""));
    addData(pinIndex, i++, obs->dvel.has_value() ? obs->dvel->y() : std::nan(""));
    addData(pinIndex, i++, obs->dvel.has_value() ? obs->dvel->z() : std::nan(""));
    addData(pinIndex, i++, obs->vpeStatus.has_value() ? obs->vpeStatus->status : std::nan(""));
    addData(pinIndex, i++, obs->pressure.has_value() ? obs->pressure.value() : std::nan(""));
    addData(pinIndex, i++, obs->magCompNED.has_value() ? obs->magCompNED->x() : std::nan(""));
    addData(pinIndex, i++, obs->magCompNED.has_value() ? obs->magCompNED->y() : std::nan(""));
    addData(pinIndex, i++, obs->magCompNED.has_value() ? obs->magCompNED->z() : std::nan(""));
    addData(pinIndex, i++, obs->accelCompNED.has_value() ? obs->accelCompNED->x() : std::nan(""));
    addData(pinIndex, i++, obs->accelCompNED.has_value() ? obs->accelCompNED->y() : std::nan(""));
    addData(pinIndex, i++, obs->accelCompNED.has_value() ? obs->accelCompNED->z() : std::nan(""));
    addData(pinIndex, i++, obs->gyroCompNED.has_value() ? obs->gyroCompNED->x() : std::nan(""));
    addData(pinIndex, i++, obs->gyroCompNED.has_value() ? obs->gyroCompNED->y() : std::nan(""));
    addData(pinIndex, i++, obs->gyroCompNED.has_value() ? obs->gyroCompNED->z() : std::nan(""));
    addData(pinIndex, i++, obs->linearAccelXYZ.has_value() ? obs->linearAccelXYZ->x() : std::nan(""));
    addData(pinIndex, i++, obs->linearAccelXYZ.has_value() ? obs->linearAccelXYZ->y() : std::nan(""));
    addData(pinIndex, i++, obs->linearAccelXYZ.has_value() ? obs->linearAccelXYZ->z() : std::nan(""));
    addData(pinIndex, i++, obs->linearAccelNED.has_value() ? obs->linearAccelNED->x() : std::nan(""));
    addData(pinIndex, i++, obs->linearAccelNED.has_value() ? obs->linearAccelNED->y() : std::nan(""));
    addData(pinIndex, i++, obs->linearAccelNED.has_value() ? obs->linearAccelNED->z() : std::nan(""));
}