// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "RealTimeKinematic.hpp"

#include <algorithm>
#include <unordered_set>
#include <ranges>

#include "util/Logger.hpp"
#include "util/Container/Vector.hpp"

#include "Navigation/Constants.hpp"

#include "internal/gui/NodeEditorApplication.hpp"
#include "internal/gui/widgets/HelpMarker.hpp"
#include "internal/gui/widgets/imgui_ex.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeData/State/Pos.hpp"
#include "NodeData/GNSS/GnssNavInfo.hpp"
#include "NodeData/GNSS/RtkSolution.hpp"

#include "Navigation/GNSS/Functions.hpp"
#include "Navigation/GNSS/Positioning/SinglePointPositioning.hpp"
#include "Navigation/GNSS/Satellite/Ephemeris/GLONASSEphemeris.hpp"
#include "Navigation/Math/LeastSquares.hpp"

NAV::RealTimeKinematic::RealTimeKinematic()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 407, 506 };

    nm::CreateInputPin(this, "Base Position", Pin::Type::Flow, { NAV::Pos::type() }, &RealTimeKinematic::recvBasePos, nullptr, 1);
    nm::CreateInputPin(this, "GnssObs (Base)", Pin::Type::Flow, { NAV::GnssObs::type() }, &RealTimeKinematic::recvBaseGnssObs);
    nm::CreateInputPin(this, "GnssObs (Rover)", Pin::Type::Flow, { NAV::GnssObs::type() }, &RealTimeKinematic::recvRoverGnssObs);
    updateNumberOfInputPins();

    nm::CreateOutputPin(this, NAV::RtkSolution::type().c_str(), Pin::Type::Flow, { NAV::RtkSolution::type() });
}

NAV::RealTimeKinematic::~RealTimeKinematic()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::RealTimeKinematic::typeStatic()
{
    return "RealTimeKinematic - RTK";
}

std::string NAV::RealTimeKinematic::type() const
{
    return typeStatic();
}

std::string NAV::RealTimeKinematic::category()
{
    return "Data Processor";
}

void NAV::RealTimeKinematic::guiConfig()
{
    if (ImGui::BeginTable(fmt::format("Pin Settings##{}", size_t(id)).c_str(), inputPins.size() > 1 ? 3 : 2,
                          ImGuiTableFlags_Borders | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_NoHostExtendX, ImVec2(0.0F, 0.0F)))
    {
        ImGui::TableSetupColumn("Pin");
        ImGui::TableSetupColumn("# Sat");
        if (inputPins.size() > 3)
        {
            ImGui::TableSetupColumn("");
        }
        ImGui::TableHeadersRow();

        // Used to reset the member variabel _dragAndDropPinIndex in case no plot does a drag and drop action
        bool dragAndDropPinStillInProgress = false;

        auto showDragDropTargetPin = [this](size_t pinIdxTarget) {
            ImGui::Dummy(ImVec2(-1.F, 2.F));

            bool selectableDummy = true;
            ImGui::PushStyleVar(ImGuiStyleVar_SelectableTextAlign, ImVec2(0.5F, 0.5F));
            ImGui::PushStyleColor(ImGuiCol_Header, IM_COL32(16, 173, 44, 79));
            ImGui::Selectable(fmt::format("[drop here]").c_str(), &selectableDummy, ImGuiSelectableFlags_None,
                              ImVec2(std::max(ImGui::GetColumnWidth(0), ImGui::CalcTextSize("[drop here]").x), 20.F));
            ImGui::PopStyleColor();
            ImGui::PopStyleVar();

            if (ImGui::BeginDragDropTarget())
            {
                if (const ImGuiPayload* payloadData = ImGui::AcceptDragDropPayload(fmt::format("DND Pin {}", size_t(id)).c_str()))
                {
                    auto pinIdxSource = *static_cast<size_t*>(payloadData->Data);

                    if (pinIdxSource < pinIdxTarget)
                    {
                        --pinIdxTarget;
                    }

                    move(inputPins, pinIdxSource, pinIdxTarget);
                    flow::ApplyChanges();
                }
                ImGui::EndDragDropTarget();
            }
            ImGui::Dummy(ImVec2(-1.F, 2.F));
        };

        for (size_t pinIndex = 0; pinIndex < inputPins.size(); pinIndex++)
        {
            ImGui::TableNextRow();
            ImGui::TableNextColumn(); // Pin

            if (pinIndex == INPUT_PORT_INDEX_GNSS_NAV_INFO && _dragAndDropPinIndex > static_cast<int>(INPUT_PORT_INDEX_GNSS_NAV_INFO))
            {
                showDragDropTargetPin(INPUT_PORT_INDEX_GNSS_NAV_INFO);
            }

            bool selectablePinDummy = false;
            ImGui::Selectable(fmt::format("{}##{}", inputPins.at(pinIndex).name, size_t(id)).c_str(), &selectablePinDummy);
            if (pinIndex >= INPUT_PORT_INDEX_GNSS_NAV_INFO && ImGui::BeginDragDropSource(ImGuiDragDropFlags_None))
            {
                dragAndDropPinStillInProgress = true;
                _dragAndDropPinIndex = static_cast<int>(pinIndex);
                // Data is copied into heap inside the drag and drop
                ImGui::SetDragDropPayload(fmt::format("DND Pin {}", size_t(id)).c_str(), &pinIndex, sizeof(pinIndex));
                ImGui::TextUnformatted(inputPins.at(pinIndex).name.c_str());
                ImGui::EndDragDropSource();
            }
            if (_dragAndDropPinIndex > 0 && pinIndex >= INPUT_PORT_INDEX_GNSS_NAV_INFO
                && pinIndex != static_cast<size_t>(_dragAndDropPinIndex - 1)
                && pinIndex != static_cast<size_t>(_dragAndDropPinIndex))
            {
                showDragDropTargetPin(pinIndex + 1);
            }
            if (pinIndex >= INPUT_PORT_INDEX_GNSS_NAV_INFO && ImGui::IsItemHovered())
            {
                ImGui::SetTooltip("This item can be dragged to reorder the pins");
            }

            ImGui::TableNextColumn(); // # Sat
            if (const auto* gnssNavInfo = getInputValue<const GnssNavInfo>(pinIndex))
            {
                size_t usedSatNum = 0;
                std::string usedSats;
                std::string allSats;

                std::string filler = ", ";
                for (const auto& satellite : gnssNavInfo->satellites())
                {
                    if ((satellite.first.satSys & _filterFreq)
                        && std::find(_excludedSatellites.begin(), _excludedSatellites.end(), satellite.first) == _excludedSatellites.end())
                    {
                        usedSatNum++;
                        usedSats += (allSats.empty() ? "" : filler) + fmt::format("{}", satellite.first);
                    }
                    allSats += (allSats.empty() ? "" : filler) + fmt::format("{}", satellite.first);
                }
                ImGui::TextUnformatted(fmt::format("{} / {}", usedSatNum, gnssNavInfo->nSatellites()).c_str());
                if (ImGui::IsItemHovered())
                {
                    ImGui::SetTooltip("Used satellites: %s\n"
                                      "All  satellites: %s",
                                      usedSats.c_str(), allSats.c_str());
                }
            }

            if (pinIndex >= INPUT_PORT_INDEX_GNSS_NAV_INFO && inputPins.size() > INPUT_PORT_INDEX_GNSS_NAV_INFO + 1)
            {
                ImGui::TableNextColumn(); // Delete
                if (ImGui::Button(fmt::format("x##{} - {}", size_t(id), pinIndex).c_str()))
                {
                    _nNavInfoPins--;
                    nm::DeleteInputPin(inputPins.at(pinIndex));
                    flow::ApplyChanges();
                }
                if (ImGui::IsItemHovered())
                {
                    ImGui::SetTooltip("Delete the pin");
                }
            }
        }

        if (!dragAndDropPinStillInProgress)
        {
            _dragAndDropPinIndex = -1;
        }

        ImGui::TableNextRow();
        ImGui::TableNextColumn(); // Pin
        if (ImGui::Button(fmt::format("Add Pin##{}", size_t(id)).c_str()))
        {
            _nNavInfoPins++;
            LOG_DEBUG("{}: # Input Pins changed to {}", nameId(), _nNavInfoPins);
            flow::ApplyChanges();
            updateNumberOfInputPins();
        }

        ImGui::EndTable();
    }

    const float itemWidth = 250.0F * gui::NodeEditorApplication::windowFontRatio();

    ImGui::SetNextItemWidth(itemWidth);
    if (ShowFrequencySelector(fmt::format("Satellite Frequencies##{}", size_t(id)).c_str(), _filterFreq))
    {
        flow::ApplyChanges();
    }

    ImGui::SetNextItemWidth(itemWidth);
    if (ShowCodeSelector(fmt::format("Signal Codes##{}", size_t(id)).c_str(), _filterCode, _filterFreq))
    {
        flow::ApplyChanges();
    }

    ImGui::SetNextItemWidth(itemWidth);
    if (ShowSatelliteSelector(fmt::format("Excluded satellites##{}", size_t(id)).c_str(), _excludedSatellites))
    {
        flow::ApplyChanges();
    }

    double elevationMaskDeg = rad2deg(_elevationMask);
    ImGui::SetNextItemWidth(itemWidth);
    if (ImGui::InputDoubleL(fmt::format("Elevation mask##{}", size_t(id)).c_str(), &elevationMaskDeg, 0.0, 90.0, 5.0, 5.0, "%.1f°", ImGuiInputTextFlags_AllowTabInput))
    {
        _elevationMask = deg2rad(elevationMaskDeg);
        LOG_DEBUG("{}: Elevation mask changed to {}°", nameId(), elevationMaskDeg);
        flow::ApplyChanges();
    }

    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    if (ImGui::TreeNode(fmt::format("Compensation models##{}", size_t(id)).c_str()))
    {
        ImGui::SetNextItemWidth(itemWidth - ImGui::GetStyle().IndentSpacing);
        if (ComboIonosphereModel(fmt::format("Ionosphere Model##{}", size_t(id)).c_str(), _ionosphereModel))
        {
            LOG_DEBUG("{}: Ionosphere Model changed to {}", nameId(), NAV::to_string(_ionosphereModel));
            flow::ApplyChanges();
        }
        if (ComboTroposphereModel(fmt::format("Troposphere Model##{}", size_t(id)).c_str(), _troposphereModels, itemWidth - ImGui::GetStyle().IndentSpacing))
        {
            flow::ApplyChanges();
        }
        ImGui::TreePop();
    }
}

[[nodiscard]] json NAV::RealTimeKinematic::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["nNavInfoPins"] = _nNavInfoPins;
    j["frequencies"] = Frequency_(_filterFreq);
    j["codes"] = _filterCode;
    j["excludedSatellites"] = _excludedSatellites;
    j["elevationMask"] = rad2deg(_elevationMask);
    j["ionosphereModel"] = _ionosphereModel;
    j["troposphereModels"] = _troposphereModels;

    return j;
}

void NAV::RealTimeKinematic::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("nNavInfoPins"))
    {
        j.at("nNavInfoPins").get_to(_nNavInfoPins);
        updateNumberOfInputPins();
    }
    if (j.contains("frequencies"))
    {
        uint64_t value = 0;
        j.at("frequencies").get_to(value);
        _filterFreq = Frequency_(value);
    }
    if (j.contains("codes"))
    {
        j.at("codes").get_to(_filterCode);
    }
    if (j.contains("excludedSatellites"))
    {
        j.at("excludedSatellites").get_to(_excludedSatellites);
    }
    if (j.contains("elevationMask"))
    {
        j.at("elevationMask").get_to(_elevationMask);
        _elevationMask = deg2rad(_elevationMask);
    }
    if (j.contains("ionosphereModel"))
    {
        j.at("ionosphereModel").get_to(_ionosphereModel);
    }
    if (j.contains("troposphereModels"))
    {
        j.at("troposphereModels").get_to(_troposphereModels);
    }
}

bool NAV::RealTimeKinematic::initialize()
{
    LOG_TRACE("{}: called", nameId());

    if (std::all_of(inputPins.begin() + INPUT_PORT_INDEX_GNSS_NAV_INFO, inputPins.end(), [](const InputPin& inputPin) { return !inputPin.isPinLinked(); }))
    {
        LOG_ERROR("{}: You need to connect a GNSS NavigationInfo provider", nameId());
        return false;
    }

    _e_basePosition = Eigen::Vector3d::Zero();
    _gnssObsBase = nullptr;
    _gnssObsRover = nullptr;

    _e_position = Eigen::Vector3d::Zero();
    _e_velocity = Eigen::Vector3d::Zero();
    _recvClk = {};

    LOG_DEBUG("RealTimeKinematic initialized");

    return true;
}

void NAV::RealTimeKinematic::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

void NAV::RealTimeKinematic::updateNumberOfInputPins()
{
    while (inputPins.size() - INPUT_PORT_INDEX_GNSS_NAV_INFO < _nNavInfoPins)
    {
        nm::CreateInputPin(this, NAV::GnssNavInfo::type().c_str(), Pin::Type::Object, { NAV::GnssNavInfo::type() });
    }
    while (inputPins.size() - INPUT_PORT_INDEX_GNSS_NAV_INFO > _nNavInfoPins)
    {
        nm::DeleteInputPin(inputPins.back());
    }
}

void NAV::RealTimeKinematic::recvBasePos(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    auto gnssObs = std::static_pointer_cast<const Pos>(queue.extract_front());
    LOG_WARN("{}: Received Base  Position for [{}]", nameId(), gnssObs->insTime);

    if (_e_basePosition.isZero())
    {
        _e_basePosition = gnssObs->e_position();
    }
    else
    {
        _e_basePosition = (_e_basePosition + gnssObs->e_position()) / 2;
    }
}

void NAV::RealTimeKinematic::recvBaseGnssObs(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    _gnssObsBase = std::static_pointer_cast<const GnssObs>(queue.extract_front());
    LOG_WARN("{}: Received Base  GNSS Obs for [{}]", nameId(), _gnssObsBase->insTime);

    if (!_e_basePosition.isZero() && _gnssObsBase && _gnssObsRover
        && _gnssObsBase->insTime == _gnssObsRover->insTime)
    {
        calcRealTimeKinematicSolution();
    }
    else if (_gnssObsRover && _gnssObsBase->insTime > _gnssObsRover->insTime)
    {
        calcFallbackSppSolution();
    }
}

void NAV::RealTimeKinematic::recvRoverGnssObs(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    if (!_gnssObsBase && _gnssObsRover) // Did not receive a base observation for the last rover observation
    {
        calcFallbackSppSolution();
    }

    _gnssObsRover = std::static_pointer_cast<const GnssObs>(queue.extract_front());
    LOG_WARN("{}: Received Rover GNSS Obs for [{}]", nameId(), _gnssObsRover->insTime);

    if (!_e_basePosition.isZero() && _gnssObsBase && _gnssObsRover
        && _gnssObsBase->insTime == _gnssObsRover->insTime)
    {
        calcRealTimeKinematicSolution();
    }
    else if (_gnssObsBase && _gnssObsBase->insTime < _gnssObsRover->insTime)
    {
        calcFallbackSppSolution();
    }
}

void NAV::RealTimeKinematic::calcRealTimeKinematicSolution()
{
    LOG_ERROR("{}: Calculate RTK Solution for  [{}]", nameId(), _gnssObsRover->insTime);

    // TODO: Do calculation

    _gnssObsBase = nullptr; // FIXME: Probable we should interpolate the base observations, in case we get them with lower frequency
    _gnssObsRover = nullptr;
}

void NAV::RealTimeKinematic::calcFallbackSppSolution()
{
    LOG_ERROR("{}: Calculate SPP Solution for  [{}] (fallback)", nameId(), _gnssObsRover->insTime);

    // Collection of all connected navigation data providers
    std::vector<const GnssNavInfo*> gnssNavInfos;
    for (size_t i = 0; i < _nNavInfoPins; i++)
    {
        if (const auto* gnssNavInfo = getInputValue<const GnssNavInfo>(INPUT_PORT_INDEX_GNSS_NAV_INFO + i))
        {
            gnssNavInfos.push_back(gnssNavInfo);
        }
    }
    if (!gnssNavInfos.empty())
    {
        SppState state = { .e_position = _e_position,
                           .e_velocity = _e_velocity,
                           .recvClk = _recvClk };
        if (auto sppSol = calcSppSolution(state, _gnssObsRover, gnssNavInfos,
                                          _ionosphereModel, _troposphereModels, SppEstimator::WEIGHTED_LEAST_SQUARES,
                                          _filterFreq, _filterCode, _excludedSatellites, _elevationMask))
        {
            _e_position = sppSol->e_position();
            _e_velocity = sppSol->e_velocity();
            _recvClk = sppSol->recvClk;

            auto rtkSol = std::make_shared<RtkSolution>();
            rtkSol->insTime = sppSol->insTime;
            rtkSol->setPositionAndStdDev_e(sppSol->e_position(), sppSol->e_positionStdev());
            rtkSol->setVelocityAndStdDev_e(sppSol->e_velocity(), sppSol->e_velocityStdev());
            rtkSol->nSatellitesPosition = sppSol->nSatellitesPosition;
            rtkSol->nSatellitesVelocity = sppSol->nSatellitesVelocity;
            rtkSol->recvClk = sppSol->recvClk;

            rtkSol->solType = RtkSolution::SolutionType::SPP;

            invokeCallbacks(OUTPUT_PORT_INDEX_RTKSOL, rtkSol);
        }
    }

    _gnssObsRover = nullptr;
}