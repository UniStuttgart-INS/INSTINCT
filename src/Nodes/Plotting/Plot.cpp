#include "Plot.hpp"

#include "util/Logger.hpp"

#include "implot.h"
#include "imgui_stdlib.h"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

NAV::Plot::Plot()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    color = ImColor(255, 128, 128);
    hasConfig = true;

    nm::CreateOutputPin(this, "", Pin::Type::Delegate, "Plot", this);

    updateNumberOfInputPins();
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
        while (static_cast<size_t>(nPlots) > plotInfos.size())
        {
            plotInfos.emplace_back("Plot " + std::to_string(plotInfos.size() + 1), nInputPins);
        }
        while (static_cast<size_t>(nPlots) < plotInfos.size())
        {
            plotInfos.pop_back();
        }

        LOG_DEBUG("{}: # Plots changed to {}", nameId(), nPlots);
        flow::ApplyChanges();
    }
    for (size_t plotNum = 0; plotNum < static_cast<size_t>(nPlots); plotNum++)
    {
        auto& plotInfo = plotInfos.at(plotNum);
        if (ImGui::CollapsingHeader((plotInfo.headerText + "##" + std::to_string(size_t(id)) + " - " + std::to_string(plotNum)).c_str()))
        {
            ImGui::InputText(("Title##" + std::to_string(size_t(id)) + " - " + std::to_string(plotNum)).c_str(), &plotInfo.title);
            if (plotInfo.headerText != plotInfo.title && !ImGui::IsItemActive())
            {
                plotInfo.headerText = plotInfo.title;
                LOG_DEBUG("{}: # Header changed to {}", nameId(), plotInfo.headerText);
            }
            for (size_t pinIndex = 0; pinIndex < data.size(); pinIndex++)
            {
                auto& pinData = data.at(pinIndex);

                if (!pinData.allDisplayNames.empty())
                {
                    if (ImGui::BeginCombo(("X Data for Pin " + std::to_string(pinIndex) + "##" + std::to_string(size_t(id)) + " - " + std::to_string(plotNum)).c_str(),
                                          pinData.allDisplayNames.at(plotInfo.selectedXdata.at(pinIndex)).c_str()))
                    {
                        for (size_t plotDataIndex = 0; plotDataIndex < pinData.plotData.size(); plotDataIndex++)
                        {
                            auto& plotData = pinData.plotData.at(plotDataIndex);

                            if (!plotData.show)
                            {
                                ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
                            }
                            const bool is_selected = (plotInfo.selectedXdata.at(pinIndex) == plotDataIndex);
                            if (ImGui::Selectable(pinData.allDisplayNames.at(plotDataIndex).c_str(), is_selected))
                            {
                                plotInfo.selectedXdata.at(pinIndex) = plotDataIndex;
                            }
                            if (!plotData.show)
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
            ImGui::CheckboxFlags(("Y-Axis 2##" + std::to_string(size_t(id)) + " - " + std::to_string(plotNum)).c_str(),
                                 &plotInfo.plotFlags, ImPlotFlags_YAxis2);
            ImGui::SameLine();
            ImGui::CheckboxFlags(("Y-Axis 3##" + std::to_string(size_t(id)) + " - " + std::to_string(plotNum)).c_str(),
                                 &plotInfo.plotFlags, ImPlotFlags_YAxis3);

            ImGui::SetNextItemWidth(100);
            ImGui::BeginGroup();
            if (ImGui::BeginCombo(("Pin##" + std::to_string(size_t(id)) + " - " + std::to_string(plotNum)).c_str(),
                                  ("Pin " + std::to_string(plotInfo.selectedPin + 1)).c_str()))
            {
                for (int n = 0; n < nInputPins; n++)
                {
                    const bool is_selected = (plotInfo.selectedPin == n);
                    if (ImGui::Selectable(("Pin " + std::to_string(n + 1)).c_str(), is_selected, 0))
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
            // data.at(plotInfo.selectedPin)

            // ImGui::Selectable(label, false, 0, ImVec2(100, 0));
            // if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_None))
            // {
            //     ImGui::SetDragDropPayload("DND_PLOT", &i, sizeof(int));
            //     ImGui::TextUnformatted(label);
            //     ImGui::EndDragDropSource();
            // }

            ImGui::EndGroup();
            ImGui::SameLine();

            if (ImPlot::BeginPlot((plotInfo.title + "##" + std::to_string(size_t(id)) + " - " + std::to_string(plotNum)).c_str(),
                                  "x", "f(x)", ImVec2(-1, 0), plotInfo.plotFlags))
            {
                static float xs1[1001], ys1[1001];
                for (int i = 0; i < 1001; ++i)
                {
                    xs1[i] = static_cast<float>(i) * 0.001F;
                    ys1[i] = 0.5F + 0.5F * sinf(50.0F * (xs1[i] + static_cast<float>(ImGui::GetTime()) / 10.0F));
                }
                static double xs2[11], ys2[11];
                for (int i = 0; i < 11; ++i)
                {
                    xs2[i] = static_cast<float>(i) * 0.1F;
                    ys2[i] = xs2[i] * xs2[i];
                }
                ImPlot::PlotLine("sin(x)", xs1, ys1, 1001);
                ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle);
                ImPlot::PlotLine("x^2", xs2, ys2, 11);
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
}

bool NAV::Plot::initialize()
{
    deinitialize();

    LOG_TRACE("{}: called", nameId());

    if (!Node::initialize())
    {
        return false;
    }

    return isInitialized = true;
}

void NAV::Plot::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    Node::deinitialize();
}

bool NAV::Plot::onCreateLink([[maybe_unused]] Pin* startPin, [[maybe_unused]] Pin* endPin)
{
    LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(startPin->id), size_t(endPin->id));

    if (!isDataTypeSupported(startPin->dataIdentifier))
    {
        return false;
    }

    size_t pinIndex = pinIndexFromId(endPin->id);

    if (startPin->dataIdentifier == VectorNavObs::type())
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

    return true;
}

void NAV::Plot::onDeleteLink([[maybe_unused]] Pin* startPin, [[maybe_unused]] Pin* endPin)
{
    LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(startPin->id), size_t(endPin->id));

    // Empty old pin data
    size_t pinIndex = pinIndexFromId(endPin->id);
    data.at(pinIndex).plotData.clear();
    data.at(pinIndex).allDisplayNames.clear();

    for (auto& plotInfo : plotInfos)
    {
        if (plotInfo.selectedXdata.size() > pinIndex)
        {
            plotInfo.selectedXdata.at(pinIndex) = 0;
            plotInfo.dataToPlot.at(pinIndex).clear();
        }
    }
}

bool NAV::Plot::isDataTypeSupported(std::string_view dataIdentifier)
{
    return dataIdentifier == VectorNavObs::type();
}

void NAV::Plot::updateNumberOfInputPins()
{
    while (inputPins.size() < static_cast<size_t>(nInputPins))
    {
        nm::CreateInputPin(this, ("Pin " + std::to_string(inputPins.size() + 1)).c_str(), Pin::Type::Flow, NAV::InsObs::type(), &Plot::plotData);
        data.emplace_back();
    }
    while (inputPins.size() > static_cast<size_t>(nInputPins))
    {
        auto connectedLinks = nm::FindConnectedLinksToPin(inputPins.back().id);
        for (Link* link : connectedLinks)
        {
            nm::DeleteLink(link->id);
        }
        inputPins.pop_back();
        data.pop_back();
    }

    for (auto& plotInfo : plotInfos)
    {
        while (plotInfo.selectedXdata.size() < static_cast<size_t>(nInputPins))
        {
            plotInfo.selectedXdata.emplace_back(0);
            plotInfo.dataToPlot[plotInfo.dataToPlot.size()] = {};
        }
        while (plotInfo.selectedXdata.size() > static_cast<size_t>(nInputPins))
        {
            plotInfo.selectedXdata.pop_back();
            plotInfo.dataToPlot.erase(plotInfo.dataToPlot.size() - 1);
        }
    }
}

void NAV::Plot::plotData(std::shared_ptr<NodeData> nodeData, ax::NodeEditor::LinkId linkId)
{
    LOG_TRACE("{}: called", nameId());

    if (Link* link = nm::FindLink(linkId))
    {
        if (Pin* sourcePin = nm::FindPin(link->startPinId))
        {
            size_t pinIndex = pinIndexFromId(link->endPinId);
            if (sourcePin->dataIdentifier == VectorNavObs::type())
            {
                plotVectorNavObs(std::static_pointer_cast<VectorNavObs>(nodeData), pinIndex);
            }
        }
    }
}

void NAV::Plot::plotVectorNavObs(std::shared_ptr<VectorNavObs> obs, size_t pinIndex)
{
    LOG_TRACE("{}: called", nameId());

    auto& pinData = data.at(pinIndex);

    auto addData = [&pinData](bool hasValue, size_t index, double value) {
        if (hasValue)
        {
            pinData.plotData.at(index).buffer.AddValue(value);
            pinData.plotData.at(index).show = true;
        }
        else
        {
            pinData.plotData.at(index).buffer.AddValue(std::nan(""));
        }
    };

    if (obs->insTime.has_value())
    {
        if (std::isnan(startValue_Time))
        {
            startValue_Time = static_cast<double>(obs->insTime.value().toGPSweekTow().tow);
        }
    }
    size_t i = 0;

    // InsObs
    addData(obs->insTime.has_value(), i++, static_cast<double>(obs->insTime->toGPSweekTow().tow) - startValue_Time);
    addData(obs->insTime.has_value(), i++, static_cast<double>(obs->insTime->toGPSweekTow().tow));
    // ImuObs
    addData(obs->timeSinceStartup.has_value(), i++, static_cast<double>(obs->timeSinceStartup.value()));
    addData(obs->magUncompXYZ.has_value(), i++, obs->magUncompXYZ->x());
    addData(obs->magUncompXYZ.has_value(), i++, obs->magUncompXYZ->y());
    addData(obs->magUncompXYZ.has_value(), i++, obs->magUncompXYZ->z());
    addData(obs->accelUncompXYZ.has_value(), i++, obs->accelUncompXYZ->x());
    addData(obs->accelUncompXYZ.has_value(), i++, obs->accelUncompXYZ->y());
    addData(obs->accelUncompXYZ.has_value(), i++, obs->accelUncompXYZ->z());
    addData(obs->gyroUncompXYZ.has_value(), i++, obs->gyroUncompXYZ->x());
    addData(obs->gyroUncompXYZ.has_value(), i++, obs->gyroUncompXYZ->y());
    addData(obs->gyroUncompXYZ.has_value(), i++, obs->gyroUncompXYZ->z());
    addData(obs->temperature.has_value(), i++, obs->temperature.value());
    // VectorNavObs
    addData(obs->quaternion.has_value(), i++, obs->quaternion->w());
    addData(obs->quaternion.has_value(), i++, obs->quaternion->x());
    addData(obs->quaternion.has_value(), i++, obs->quaternion->y());
    addData(obs->quaternion.has_value(), i++, obs->quaternion->z());
    addData(obs->yawPitchRoll.has_value(), i++, obs->yawPitchRoll->x());
    addData(obs->yawPitchRoll.has_value(), i++, obs->yawPitchRoll->y());
    addData(obs->yawPitchRoll.has_value(), i++, obs->yawPitchRoll->z());
    addData(obs->timeSinceSyncIn.has_value(), i++, static_cast<double>(obs->timeSinceSyncIn.value()));
    addData(obs->syncInCnt.has_value(), i++, obs->syncInCnt.value());
    addData(obs->magCompXYZ.has_value(), i++, obs->magCompXYZ->x());
    addData(obs->magCompXYZ.has_value(), i++, obs->magCompXYZ->y());
    addData(obs->magCompXYZ.has_value(), i++, obs->magCompXYZ->z());
    addData(obs->accelCompXYZ.has_value(), i++, obs->accelCompXYZ->x());
    addData(obs->accelCompXYZ.has_value(), i++, obs->accelCompXYZ->y());
    addData(obs->accelCompXYZ.has_value(), i++, obs->accelCompXYZ->z());
    addData(obs->gyroCompXYZ.has_value(), i++, obs->gyroCompXYZ->x());
    addData(obs->gyroCompXYZ.has_value(), i++, obs->gyroCompXYZ->y());
    addData(obs->gyroCompXYZ.has_value(), i++, obs->gyroCompXYZ->z());
    addData(obs->dtime.has_value(), i++, obs->dtime.value());
    addData(obs->dtheta.has_value(), i++, obs->dtheta->x());
    addData(obs->dtheta.has_value(), i++, obs->dtheta->y());
    addData(obs->dtheta.has_value(), i++, obs->dtheta->z());
    addData(obs->dvel.has_value(), i++, obs->dvel->x());
    addData(obs->dvel.has_value(), i++, obs->dvel->y());
    addData(obs->dvel.has_value(), i++, obs->dvel->z());
    addData(obs->vpeStatus.has_value(), i++, obs->vpeStatus->status);
    addData(obs->pressure.has_value(), i++, obs->pressure.value());
    addData(obs->magCompNED.has_value(), i++, obs->magCompNED->x());
    addData(obs->magCompNED.has_value(), i++, obs->magCompNED->y());
    addData(obs->magCompNED.has_value(), i++, obs->magCompNED->z());
    addData(obs->accelCompNED.has_value(), i++, obs->accelCompNED->x());
    addData(obs->accelCompNED.has_value(), i++, obs->accelCompNED->y());
    addData(obs->accelCompNED.has_value(), i++, obs->accelCompNED->z());
    addData(obs->gyroCompNED.has_value(), i++, obs->gyroCompNED->x());
    addData(obs->gyroCompNED.has_value(), i++, obs->gyroCompNED->y());
    addData(obs->gyroCompNED.has_value(), i++, obs->gyroCompNED->z());
    addData(obs->linearAccelXYZ.has_value(), i++, obs->linearAccelXYZ->x());
    addData(obs->linearAccelXYZ.has_value(), i++, obs->linearAccelXYZ->y());
    addData(obs->linearAccelXYZ.has_value(), i++, obs->linearAccelXYZ->z());
    addData(obs->linearAccelNED.has_value(), i++, obs->linearAccelNED->x());
    addData(obs->linearAccelNED.has_value(), i++, obs->linearAccelNED->y());
    addData(obs->linearAccelNED.has_value(), i++, obs->linearAccelNED->z());
}