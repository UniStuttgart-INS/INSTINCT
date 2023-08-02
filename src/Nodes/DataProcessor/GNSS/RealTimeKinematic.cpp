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
#include "internal/gui/widgets/InputWithUnit.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeData/State/Pos.hpp"

#include "Navigation/GNSS/Core/Frequency.hpp"
#include "Navigation/GNSS/Functions.hpp"
#include "Navigation/GNSS/Errors.hpp"
#include "Navigation/GNSS/Positioning/SppAlgorithm.hpp"
#include "Navigation/GNSS/Satellite/Ephemeris/GLONASSEphemeris.hpp"
#include "Navigation/Math/LeastSquares.hpp"

namespace States = NAV::RealTimeKinematicKF::States;
namespace Meas = NAV::RealTimeKinematicKF::Meas;

namespace NAV
{

RealTimeKinematic::SatData::ReceiverSpecificData::ReceiverSpecificData(std::shared_ptr<const GnssObs> gnssObs,
                                                                       const Eigen::Vector3d& e_satPos,
                                                                       const Eigen::Vector3d& lla_satPos,
                                                                       Eigen::Vector3d e_satVel,
                                                                       const Eigen::Vector3d& e_recPos,
                                                                       const Eigen::Vector3d& e_recVel)
    : gnssObs(std::move(gnssObs)), e_satPos(e_satPos), lla_satPos(lla_satPos), e_satVel(std::move(e_satVel))
{
    e_pLOS = e_calcLineOfSightUnitVector(e_recPos, e_satPos);
    e_vLOS = (e_recVel - e_satVel) / (e_recPos - e_satPos).norm();
    Eigen::Vector3d n_lineOfSightUnitVector = trafo::n_Quat_e(lla_satPos(0), lla_satPos(1)) * e_pLOS;
    satElevation = calcSatElevation(n_lineOfSightUnitVector);
    satAzimuth = calcSatAzimuth(n_lineOfSightUnitVector);

    LOG_DATA("    e_lineOfSightUnitVector {}", e_pLOS.transpose());
    LOG_DATA("    n_lineOfSightUnitVector {}", n_lineOfSightUnitVector.transpose());
    LOG_DATA("    satElevation {}°", rad2deg(satElevation));
    LOG_DATA("    satAzimuth   {}°", rad2deg(satAzimuth));
}

RealTimeKinematic::RealTimeKinematic()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 637, 617 };

    nm::CreateInputPin(this, "Base Position", Pin::Type::Flow, { Pos::type() }, &RealTimeKinematic::recvBasePos, nullptr, 1);
    nm::CreateInputPin(this, "GnssObs (Base)", Pin::Type::Flow, { GnssObs::type() }, &RealTimeKinematic::recvBaseGnssObs);
    nm::CreateInputPin(this, "GnssObs (Rover)", Pin::Type::Flow, { GnssObs::type() }, &RealTimeKinematic::recvRoverGnssObs);
    updateNumberOfInputPins();

    nm::CreateOutputPin(this, RtkSolution::type().c_str(), Pin::Type::Flow, { RtkSolution::type() });
}

RealTimeKinematic::~RealTimeKinematic()
{
    LOG_TRACE("{}: called", nameId());
}

std::string RealTimeKinematic::typeStatic()
{
    return "RealTimeKinematic - RTK";
}

std::string RealTimeKinematic::type() const
{
    return typeStatic();
}

std::string RealTimeKinematic::category()
{
    return "Data Processor";
}

void RealTimeKinematic::guiConfig()
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

    const float configWidth = 280.0F * gui::NodeEditorApplication::windowFontRatio();
    const float unitWidth = 120.0F * gui::NodeEditorApplication::windowFontRatio();
    const float itemWidth = 310.0F * gui::NodeEditorApplication::windowFontRatio();

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

    ImGui::BeginHorizontal(fmt::format("Observables##{}", size_t(id)).c_str(),
                           ImVec2(itemWidth - ImGui::GetStyle().ItemSpacing.x + ImGui::GetStyle().ItemInnerSpacing.x, 0.0F));
    if (ImGui::Checkbox(fmt::format("Pseudorange##{}", size_t(id)).c_str(), &_usedObservations[GnssObs::Pseudorange]))
    {
        LOG_DEBUG("{}: Using {}: {}", nameId(), GnssObs::Pseudorange, _usedObservations[GnssObs::Pseudorange]);
        flow::ApplyChanges();
    }
    if (ImGui::Checkbox(fmt::format("Carrier##{}", size_t(id)).c_str(), &_usedObservations[GnssObs::Carrier]))
    {
        LOG_DEBUG("{}: Using {}: {}", nameId(), GnssObs::Carrier, _usedObservations[GnssObs::Carrier]);
        flow::ApplyChanges();
    }
    if (ImGui::Checkbox(fmt::format("Doppler##{}", size_t(id)).c_str(), &_usedObservations[GnssObs::Doppler]))
    {
        LOG_DEBUG("{}: Using {}: {}", nameId(), GnssObs::Doppler, _usedObservations[GnssObs::Doppler]);
        flow::ApplyChanges();
    }
    ImGui::EndHorizontal();

    ImGui::SameLine();
    ImGui::TextUnformatted("Used observables");

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

    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    if (ImGui::TreeNode(fmt::format("System/Process noise (Standard Deviations)##{}", size_t(id)).c_str()))
    {
        if (gui::widgets::InputDouble2WithUnit(fmt::format("Acceleration due to user motion (Hor/Ver)##{}", size_t(id)).c_str(),
                                               configWidth, unitWidth, _gui_stdevAccel.data(), reinterpret_cast<int*>(&_gui_stdevAccelUnits), "m/√(s^3)\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: stdevAccel changed to horizontal {} and vertical {}", nameId(), _gui_stdevAccel.at(0), _gui_stdevAccel.at(1));
            LOG_DEBUG("{}: stdevAccelNoiseUnits changed to {}", nameId(), fmt::underlying(_gui_stdevAccelUnits));
            recalcVarAccel();
            flow::ApplyChanges();
        }

        ImGui::TreePop();
    }
    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    if (ImGui::TreeNode(fmt::format("GNSS Measurement Error##{}", size_t(id)).c_str()))
    {
        if (_gnssMeasurementErrorModel.ShowGuiWidgets(std::to_string(size_t(id)).c_str(), itemWidth - ImGui::GetStyle().IndentSpacing))
        {
            LOG_DEBUG("{}: GNSS Measurement Error Model changed.", nameId());
            flow::ApplyChanges();
        }
        ImGui::TreePop();
    }
}

void RealTimeKinematic::recalcVarAccel()
{
    // 𝜎_a Standard deviation of the acceleration due to user motion in horizontal and vertical component in [m / √(s^3)]
    switch (_gui_stdevAccelUnits)
    {
    case StdevAccelUnits::m_sqrts3: // [m / √(s^3)]
        _varAccel = { std::pow(_gui_stdevAccel.at(0), 2), std::pow(_gui_stdevAccel.at(1), 2) };
        break;
    }
    LOG_DATA("  sigma2_accel = h: {}, v: {} [m^2 / s^3]", nameId(), _varAccel.at(0), _varAccel.at(1));
}

[[nodiscard]] json RealTimeKinematic::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["nNavInfoPins"] = _nNavInfoPins;
    j["frequencies"] = Frequency_(_filterFreq);
    j["codes"] = _filterCode;
    j["excludedSatellites"] = _excludedSatellites;
    j["elevationMask"] = rad2deg(_elevationMask);
    j["usedObservations"] = _usedObservations;
    j["ionosphereModel"] = _ionosphereModel;
    j["troposphereModels"] = _troposphereModels;
    j["gnssMeasurementError"] = _gnssMeasurementErrorModel;

    j["stdevAccelUnits"] = _gui_stdevAccelUnits;
    j["stdevAccel"] = _gui_stdevAccel;

    return j;
}

void RealTimeKinematic::restore(json const& j)
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
    if (j.contains("usedObservations"))
    {
        j.at("usedObservations").get_to(_usedObservations);
    }
    if (j.contains("ionosphereModel"))
    {
        j.at("ionosphereModel").get_to(_ionosphereModel);
    }
    if (j.contains("troposphereModels"))
    {
        j.at("troposphereModels").get_to(_troposphereModels);
    }
    if (j.contains("gnssMeasurementError"))
    {
        j.at("gnssMeasurementError").get_to(_gnssMeasurementErrorModel);
    }

    if (j.contains("stdevAccelUnits"))
    {
        _gui_stdevAccelUnits = j.at("stdevAccelUnits");
        recalcVarAccel();
    }
    if (j.contains("stdevAccel"))
    {
        _gui_stdevAccel = j.at("stdevAccel");
        recalcVarAccel();
    }
}

bool RealTimeKinematic::initialize()
{
    LOG_TRACE("{}: called", nameId());

    if (std::all_of(inputPins.begin() + INPUT_PORT_INDEX_GNSS_NAV_INFO, inputPins.end(), [](const InputPin& inputPin) { return !inputPin.isPinLinked(); }))
    {
        LOG_ERROR("{}: You need to connect a GNSS NavigationInfo provider", nameId());
        return false;
    }

    _lastUpdate.reset();
    _receiver = { { Receiver(Base), Receiver(Rover) } };

    _kalmanFilter = KeyedKalmanFilterD<States::StateKeyTypes,
                                       Meas::MeasKeyTypes>{ States::PosVel, {} };

    _kalmanFilter.F.block<3>(States::Pos, States::Vel) = Eigen::Matrix3d::Identity();

    _kalmanFilter.P(all, all).diagonal() << 1e4, 1e4, 1e4, 1e2, 1e2, 1e2; // TODO: Initialize from SPP

    LOG_DEBUG("RealTimeKinematic initialized");

    return true;
}

void RealTimeKinematic::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

void RealTimeKinematic::updateNumberOfInputPins()
{
    while (inputPins.size() - INPUT_PORT_INDEX_GNSS_NAV_INFO < _nNavInfoPins)
    {
        nm::CreateInputPin(this, GnssNavInfo::type().c_str(), Pin::Type::Object, { GnssNavInfo::type() });
    }
    while (inputPins.size() - INPUT_PORT_INDEX_GNSS_NAV_INFO > _nNavInfoPins)
    {
        nm::DeleteInputPin(inputPins.back());
    }
}

void RealTimeKinematic::recvBasePos(InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    auto gnssObs = std::static_pointer_cast<const Pos>(queue.extract_front());
    LOG_DATA("{}: Received Base  Position for [{}]", nameId(), gnssObs->insTime);

    if (_receiver[Base].e_pos.isZero())
    {
        _receiver[Base].e_pos = gnssObs->e_position();
        _receiver[Base].lla_pos = gnssObs->lla_position();
    }
    else
    {
        _receiver[Base].e_pos = (_receiver[Base].e_pos + gnssObs->e_position()) / 2;
        _receiver[Base].lla_pos = trafo::ecef2lla_WGS84(_receiver[Base].e_pos);
    }
}

void RealTimeKinematic::recvBaseGnssObs(InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    _receiver[Base].gnssObs = std::static_pointer_cast<const GnssObs>(queue.extract_front());
    LOG_DATA("{}: Received Base  GNSS Obs for [{}]", nameId(), _receiver[Base].gnssObs->insTime);

    if (!_receiver[Base].e_pos.isZero() && _receiver[Base].gnssObs && _receiver[Rover].gnssObs
        && _receiver[Base].gnssObs->insTime == _receiver[Rover].gnssObs->insTime)
    {
        calcRealTimeKinematicSolution();
    }
    else if (_receiver[Rover].gnssObs && _receiver[Base].gnssObs->insTime > _receiver[Rover].gnssObs->insTime)
    {
        if (auto rtkSol = calcFallbackSppSolution())
        {
            invokeCallbacks(OUTPUT_PORT_INDEX_RTKSOL, rtkSol);
        }
        _receiver[Rover].gnssObs = nullptr;
    }
}

void RealTimeKinematic::recvRoverGnssObs(InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    if (!_receiver[Base].gnssObs && _receiver[Rover].gnssObs) // Did not receive a base observation for the last rover observation
    {
        if (auto rtkSol = calcFallbackSppSolution())
        {
            invokeCallbacks(OUTPUT_PORT_INDEX_RTKSOL, rtkSol);
        }
    }

    _receiver[Rover].gnssObs = std::static_pointer_cast<const GnssObs>(queue.extract_front());
    LOG_DATA("{}: Received Rover GNSS Obs for [{}]", nameId(), _receiver[Rover].gnssObs->insTime);

    if (!_receiver[Base].e_pos.isZero() && _receiver[Base].gnssObs && _receiver[Rover].gnssObs
        && _receiver[Base].gnssObs->insTime == _receiver[Rover].gnssObs->insTime)
    {
        calcRealTimeKinematicSolution();
    }
    else if (_receiver[Base].gnssObs && _receiver[Base].gnssObs->insTime < _receiver[Rover].gnssObs->insTime)
    {
        if (auto rtkSol = calcFallbackSppSolution())
        {
            invokeCallbacks(OUTPUT_PORT_INDEX_RTKSOL, rtkSol);
        }
        _receiver[Rover].gnssObs = nullptr;
    }
}

void RealTimeKinematic::calcRealTimeKinematicSolution()
{
    LOG_DATA("{}: Calculate RTK Solution for  [{}]", nameId(), _receiver[Rover].gnssObs->insTime);

    // Calculate a Single point solution if the Rover Position is not known yet
    if (_receiver[Rover].e_pos.isZero())
    {
        if (auto sol = calcFallbackSppSolution())
        {
            _lastUpdate = sol->insTime;
            _receiver[Rover].e_pos = sol->e_position();
            _receiver[Rover].lla_pos = trafo::ecef2lla_WGS84(_receiver[Rover].e_pos);
            _receiver[Rover].e_vel = sol->e_velocity();

            // #######################################################################################################
            //                                     TODO: Debugging, remove later
            // #######################################################################################################
            // _receiver[Base].lla_pos = Eigen::Vector3d(deg2rad(30), deg2rad(95), 0.0);
            // // _receiver[Base].e_pos = Eigen::Vector3d{ -481819.3135, 5507219.9538, 3170373.7354 };  // From the RINEX obs file
            // _receiver[Base].e_pos = trafo::lla2ecef_WGS84(_receiver[Base].lla_pos);
            // _receiver[Rover].lla_pos = Eigen::Vector3d(deg2rad(30), deg2rad(95.02), 0.0);
            // // _receiver[Rover].e_pos = Eigen::Vector3d{ -483741.6665, 5507051.4316, 3170373.7354 }; // From the RINEX obs file
            // _receiver[Rover].e_pos = trafo::lla2ecef_WGS84(_receiver[Rover].lla_pos);
            // _receiver[Rover].e_vel = Eigen::Vector3d::Zero();
            // #######################################################################################################

            _kalmanFilter.x.segment<3>(States::Pos) = _receiver[Rover].e_pos;
            _kalmanFilter.x.segment<3>(States::Vel) = _receiver[Rover].e_vel;

            LOG_TRACE("{}: Initial base  position: {}°, {}°, {}m (ECEF {} [m])", nameId(),
                      rad2deg(_receiver[Base].lla_pos(0)), rad2deg(_receiver[Base].lla_pos(1)), _receiver[Base].lla_pos(2),
                      _receiver[Base].e_pos.transpose());
            LOG_TRACE("{}: Initial rover position: {}°, {}°, {}m (ECEF {} [m])", nameId(),
                      rad2deg(_receiver[Rover].lla_pos(0)), rad2deg(_receiver[Rover].lla_pos(1)), _receiver[Rover].lla_pos(2),
                      _receiver[Rover].e_pos.transpose());
            LOG_TRACE("{}: Initial rover velocity: {} [m/s] (ECEF)", nameId(), _receiver[Rover].e_vel.transpose());
        }
        else
        {
            return;
        }
    }

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
        // Collection of all connected Ionospheric Corrections
        IonosphericCorrections ionosphericCorrections(gnssNavInfos);

        // ----------------------------------------- RTK Algorithm -------------------------------------------

        auto rtkSol = std::make_shared<RtkSolution>();
        rtkSol->insTime = _receiver[Rover].gnssObs->insTime;

        std::unordered_set<GnssObs::ObservationType> obsTypes;
        if (_usedObservations.at(GnssObs::Pseudorange)) { obsTypes.insert(GnssObs::Pseudorange); }
        if (_usedObservations.at(GnssObs::Carrier)) { obsTypes.insert(GnssObs::Carrier); }
        if (_usedObservations.at(GnssObs::Doppler)) { obsTypes.insert(GnssObs::Doppler); }

        // Data calculated for each satellite (only satellites filtered by GUI filter & NAV data available & ...)
        auto [satelliteData, observations, nObs] = selectSatObservationsForCalculation(obsTypes, gnssNavInfos);

        updatePivotSatellites(satelliteData, observations);

        calcObservationEstimates(satelliteData, observations, ionosphericCorrections);

        auto singleDifferences = calcSingleDifferences(observations);
        auto doubleDifferences = calcDoubleDifferences(singleDifferences);

        // ###################################################################################################
        //                                    Kalman Filter - Prediction
        // ###################################################################################################

        // Update the State transition matrix (𝚽) and the Process noise covariance matrix (𝐐)

        double dt = static_cast<double>((_receiver[Rover].gnssObs->insTime - _lastUpdate).count());
        LOG_TRACE("{}: dt = {}s", nameId(), dt);

        // TODO: Add states for new ambiguities and remove old ones which are gone for a certain amount of time
        // _kalmanFilter.addStates({ States::AmbiguitySD{ SatSigId{ G01, 1 } } });
        // _kalmanFilter.addNoiseStates({ States::AmbiguitySD{ SatSigId{ G01, 1 } } }); // Add same states as noise states

        LOG_DATA("{}: F =\n{}", nameId(), _kalmanFilter.F);

        _kalmanFilter.G.block<3>(States::Vel, States::Vel) = trafo::e_Quat_n(_receiver[Rover].lla_pos(0), _receiver[Rover].lla_pos(1)).toRotationMatrix();
        _kalmanFilter.W.block<3>(States::Vel, States::Vel).diagonal() << _varAccel.at(0), _varAccel.at(0), _varAccel.at(1);
        // _kalmanFilter.G(States::AmbiguitySD{ SatSigId{ G01, 1 } }, States::AmbiguitySD{ SatSigId{ G01, 1 } }) = 1;
        // Ambiguities are modeled as RW with very small noise to keep numerical stability
        // _kalmanFilter.W(States::AmbiguitySD{ SatSigId{ G01, 1 } }, States::AmbiguitySD{ SatSigId{ G01, 1 } }) = 1e-6;
        LOG_TRACE("{}: G =\n{}", nameId(), _kalmanFilter.G);
        LOG_TRACE("{}: W =\n{}", nameId(), _kalmanFilter.W);
        LOG_TRACE("{}: GWG^T =\n{}", nameId(),
                  KeyedMatrixXd<States::StateKeyTypes>(_kalmanFilter.G(all, all)
                                                           * _kalmanFilter.W(all, all)
                                                           * _kalmanFilter.G(all, all).transpose(),
                                                       _kalmanFilter.G.rowKeys()));

        _kalmanFilter.calcPhiAndQWithVanLoanMethod(dt);
        LOG_TRACE("{}: Phi =\n{}", nameId(), _kalmanFilter.Phi);
        LOG_TRACE("{}: Q =\n{}", nameId(), _kalmanFilter.Q);

        LOG_TRACE("{}: x (a posteriori, t-1 = {}) =\n{}", nameId(), _lastUpdate, _kalmanFilter.x.transposed());
        LOG_TRACE("{}: P (a posteriori, t-1 = {}) =\n{}", nameId(), _lastUpdate, _kalmanFilter.P);
        _kalmanFilter.predict();
        LOG_TRACE("{}: x (a priori    , t   = {}) =\n{}", nameId(), _receiver[Rover].gnssObs->insTime, _kalmanFilter.x.transposed());
        LOG_TRACE("{}: P (a priori    , t   = {}) =\n{}", nameId(), _receiver[Rover].gnssObs->insTime, _kalmanFilter.P);

        // ###################################################################################################
        //                                      Kalman Filter - Update
        // ###################################################################################################

        // Update the Measurement sensitivity Matrix (𝐇), the Measurement noise covariance matrix (𝐑) and the Measurement vector (𝐳)

        std::vector<Meas::MeasKeyTypes> measKeys;
        measKeys.reserve(doubleDifferences.size() * obsTypes.size());
        for (const auto& [satSigId, doubleDiff] : doubleDifferences)
        {
            for (const auto& [obsType, diff] : doubleDiff)
            {
                switch (obsType)
                {
                case GnssObs::Pseudorange:
                    measKeys.emplace_back(Meas::PsrDD{ satSigId });
                    break;
                case GnssObs::Carrier:
                    measKeys.emplace_back(Meas::CarrierDD{ satSigId });
                    break;
                case GnssObs::Doppler:
                    measKeys.emplace_back(Meas::DopplerDD{ satSigId });
                    break;
                }
            }
        }
        _kalmanFilter.setMeasurements(measKeys);

        for (const auto& [satSigId_s, doubleDiff] : doubleDifferences)
        {
            auto satData_s = std::find_if(satelliteData.begin(), satelliteData.end(),
                                          [&satSigId_s = satSigId_s](const SatData& satData) { return satData.satId == satSigId_s.toSatId(); });

            const auto& satSigId_1 = _pivotSatellites.at(satSigId_s.code).satSigId;
            auto satData_1 = std::find_if(satelliteData.begin(), satelliteData.end(),
                                          [&satSigId_1](const SatData& satData) { return satData.satId == satSigId_1.toSatId(); });

            const auto& e_pLOS_1 = satData_1->receiverData.at(Rover).e_pLOS;
            const auto& e_pLOS_s = satData_s->receiverData.at(Rover).e_pLOS;

            for (auto& [obsType, obs] : doubleDiff)
            {
                switch (obsType)
                {
                case GnssObs::Pseudorange:
                    _kalmanFilter.z(Meas::PsrDD{ satSigId_s }) = obs.measurement - obs.estimate;
                    _kalmanFilter.H.block<3>(Meas::PsrDD{ satSigId_s }, States::Pos) = (e_pLOS_1 - e_pLOS_s).transpose();
                    _kalmanFilter.R(Meas::PsrDD{ satSigId_s }, Meas::PsrDD{ satSigId_s }) = obs.measVar;
                    break;
                case GnssObs::Carrier:
                    _kalmanFilter.z(Meas::CarrierDD{ satSigId_s }) = obs.measurement - obs.estimate;
                    _kalmanFilter.H.block<3>(Meas::CarrierDD{ satSigId_s }, States::Pos) = (e_pLOS_1 - e_pLOS_s).transpose();
                    _kalmanFilter.R(Meas::CarrierDD{ satSigId_s }, Meas::CarrierDD{ satSigId_s }) = obs.measVar;
                    break;
                case GnssObs::Doppler:
                {
                    _kalmanFilter.z(Meas::DopplerDD{ satSigId_s }) = obs.measurement - obs.estimate;
                    const auto& e_vLOS_1 = satData_1->receiverData.at(Rover).e_vLOS;
                    const auto& e_vLOS_s = satData_s->receiverData.at(Rover).e_vLOS;

                    _kalmanFilter.H.block<3>(Meas::DopplerDD{ satSigId_s }, States::Pos) = Eigen::Vector3d(
                        -e_vLOS_1.x() * e_pLOS_1.x() * e_pLOS_1.x() + e_vLOS_1.x() + e_vLOS_s.x() * e_pLOS_s.x() * e_pLOS_s.x() - e_vLOS_s.x() - e_vLOS_1.y() * e_pLOS_1.x() * e_pLOS_1.y() + e_vLOS_s.y() * e_pLOS_s.x() * e_pLOS_s.y() - e_vLOS_1.z() * e_pLOS_1.x() * e_pLOS_1.z() + e_vLOS_s.z() * e_pLOS_s.x() * e_pLOS_s.z(),
                        -e_vLOS_1.x() * e_pLOS_1.x() * e_pLOS_1.y() + e_vLOS_s.x() * e_pLOS_s.x() * e_pLOS_s.y() - e_vLOS_1.y() * e_pLOS_1.y() * e_pLOS_1.y() + e_vLOS_1.y() + e_vLOS_s.y() * e_pLOS_s.y() * e_pLOS_s.y() - e_vLOS_s.y() - e_vLOS_1.z() * e_pLOS_1.y() * e_pLOS_1.z() + e_vLOS_s.z() * e_pLOS_s.y() * e_pLOS_s.z(),
                        -e_vLOS_1.x() * e_pLOS_1.x() * e_pLOS_1.z() + e_vLOS_s.x() * e_pLOS_s.x() * e_pLOS_s.z() - e_vLOS_1.y() * e_pLOS_1.y() * e_pLOS_1.z() + e_vLOS_s.y() * e_pLOS_s.y() * e_pLOS_s.z() - e_vLOS_1.z() * e_pLOS_1.z() * e_pLOS_1.z() + e_vLOS_1.z() + e_vLOS_s.z() * e_pLOS_s.z() * e_pLOS_s.z() - e_vLOS_s.z());
                    _kalmanFilter.R(Meas::DopplerDD{ satSigId_s }, Meas::DopplerDD{ satSigId_s }) = obs.measVar;
                    break;
                }
                }
            }
        }
        LOG_TRACE("{}: z =\n{}", nameId(), _kalmanFilter.z.transposed());
        LOG_TRACE("{}: H =\n{}", nameId(), _kalmanFilter.H);
        LOG_TRACE("{}: R =\n{}", nameId(), _kalmanFilter.R);

        LOG_TRACE("{}: HPH^T =\n{}", nameId(),
                  KeyedMatrixXd<Meas::MeasKeyTypes>(_kalmanFilter.H(all, all)
                                                        * _kalmanFilter.P(all, all)
                                                        * _kalmanFilter.H(all, all).transpose(),
                                                    _kalmanFilter.z.rowKeys()));
        LOG_TRACE("{}: S =\n{}", nameId(),
                  KeyedMatrixXd<Meas::MeasKeyTypes>((_kalmanFilter.H(all, all)
                                                         * _kalmanFilter.P(all, all)
                                                         * _kalmanFilter.H(all, all).transpose()
                                                     + _kalmanFilter.R(all, all))
                                                        .inverse(),
                                                    _kalmanFilter.z.rowKeys()));

        _kalmanFilter.correctWithMeasurementInnovation();
        _receiver[Rover].e_pos = _kalmanFilter.x.segment<3>(States::Pos);
        _receiver[Rover].e_vel = _kalmanFilter.x.segment<3>(States::Vel);
        _receiver[Rover].lla_pos = trafo::ecef2lla_WGS84(_receiver[Rover].e_pos);
        LOG_TRACE("{}: x (a posteriori, t   = {}) =\n{}", nameId(), _receiver[Rover].gnssObs->insTime, _kalmanFilter.x.transposed());
        LOG_TRACE("{}: P (a posteriori, t   = {}) =\n{}", nameId(), _receiver[Rover].gnssObs->insTime, _kalmanFilter.P);

        rtkSol->solType = RtkSolution::SolutionType::RTK_Float;
        rtkSol->nSatellites = satelliteData.size() - _pivotSatellites.size();
        rtkSol->setPositionAndStdDev_e(_receiver[Rover].e_pos, _kalmanFilter.P.block<3>(States::Pos, States::Pos));
        rtkSol->setVelocityAndStdDev_e(_receiver[Rover].e_vel, _kalmanFilter.P.block<3>(States::Vel, States::Vel));

        _lastUpdate = rtkSol->insTime;
        invokeCallbacks(OUTPUT_PORT_INDEX_RTKSOL, rtkSol);
    }

    _receiver[Base].gnssObs = nullptr; // FIXME: Probable we should interpolate the base observations, in case we get them with lower frequency
    _receiver[Rover].gnssObs = nullptr;
}

std::shared_ptr<RtkSolution> RealTimeKinematic::calcFallbackSppSolution()
{
    LOG_DATA("{}: Calculate SPP Solution for  [{}] (fallback)", nameId(), _receiver[Rover].gnssObs->insTime);

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
        GNSS::Positioning::SPP::State state = { .e_position = _receiver[Rover].e_pos,
                                                .e_velocity = _receiver[Rover].e_vel,
                                                .recvClk = {} };
        if (auto sppSol = GNSS::Positioning::SPP::calcSppSolutionLSE(state, _receiver[Rover].gnssObs, gnssNavInfos,
                                                                     _ionosphereModel, _troposphereModels, _gnssMeasurementErrorModel,
                                                                     GNSS::Positioning::SPP::EstimatorType::WEIGHTED_LEAST_SQUARES,
                                                                     _filterFreq, _filterCode, _excludedSatellites, _elevationMask))
        {
            auto rtkSol = std::make_shared<RtkSolution>();
            rtkSol->insTime = sppSol->insTime;
            rtkSol->setPosition_e(sppSol->e_position()); // TODO: Set covariance and stdDev
            rtkSol->setVelocity_e(sppSol->e_velocity());
            rtkSol->nSatellites = sppSol->nSatellitesPosition;

            rtkSol->solType = RtkSolution::SolutionType::SPP;

            return rtkSol;
        }
    }

    return nullptr;
}

std::tuple<std::vector<RealTimeKinematic::SatData>, RealTimeKinematic::Observations, size_t>
    RealTimeKinematic::selectSatObservationsForCalculation(const std::unordered_set<GnssObs::ObservationType>& obsTypes,
                                                           const std::vector<const GnssNavInfo*>& gnssNavInfos)
{
    const auto& gnssObs = _receiver[Rover].gnssObs;

    std::vector<SatData> satelliteData;
    satelliteData.reserve(gnssObs->data.size());

    Observations observations;
    observations.reserve(gnssObs->data.size());

    unordered_map<GnssObs::ObservationType, size_t> nMeasurements;
    nMeasurements.reserve(obsTypes.size());

    for (const auto& obsData : gnssObs->data)
    {
        auto satId = obsData.satSigId.toSatId();

        if (!(obsData.satSigId.freq() & _filterFreq)                                                                  // frequency is not selected in GUI
            || !(obsData.satSigId.code & _filterCode)                                                                 // code is not selected in GUI
            || !obsData.pseudorange                                                                                   // has an invalid pseudorange
            || std::find(_excludedSatellites.begin(), _excludedSatellites.end(), satId) != _excludedSatellites.end()) // is excluded
        {
            continue;
        }

        unordered_map<GnssObs::ObservationType, size_t> availableObservations;
        if (std::any_of(_receiver.begin(), _receiver.end(),
                        [&](const Receiver& recv) {
                            auto obsDataOther = std::find_if(recv.gnssObs->data.begin(), recv.gnssObs->data.end(),
                                                             [&obsData](const GnssObs::ObservationData& obsDataOther) {
                                                                 return obsDataOther.satSigId == obsData.satSigId;
                                                             });
                            for (const auto& obsType : obsTypes)
                            {
                                switch (obsType)
                                {
                                case GnssObs::Pseudorange:
                                    if (obsDataOther->pseudorange) { availableObservations[obsType]++; }
                                    break;
                                case GnssObs::Carrier:
                                    if (obsDataOther->carrierPhase) { availableObservations[obsType]++; }
                                    break;
                                case GnssObs::Doppler:
                                    if (obsDataOther->doppler) { availableObservations[obsType]++; }
                                    break;
                                }
                            }
                            return obsDataOther == recv.gnssObs->data.end() // All receivers must have this observation
                                   || !obsDataOther->pseudorange;           // Pseudorange must be available for all receivers
                        }))
        {
            continue;
        }

        std::shared_ptr<NAV::SatNavData> satNavData = nullptr;
        for (const auto& gnssNavInfo : gnssNavInfos)
        {
            auto satNav = gnssNavInfo->searchNavigationData(satId, gnssObs->insTime);
            if (satNav && satNav->isHealthy())
            {
                satNavData = satNav;
                break;
            }
        }
        if (satNavData == nullptr) { continue; } // can calculate satellite position

        if (std::find_if(satelliteData.begin(), satelliteData.end(),
                         [&satId](const SatData& satData) { return satData.satId == satId; })
            == satelliteData.end())
        {
            SatData satData = SatData(satId, satNavData);

            for (const auto& recv : _receiver)
            {
                auto recvObsData = std::find_if(recv.gnssObs->data.begin(), recv.gnssObs->data.end(),
                                                [&obsData](const GnssObs::ObservationData& recvObsData) {
                                                    return recvObsData.satSigId == obsData.satSigId;
                                                });

                auto satClk = satNavData->calcClockCorrections(gnssObs->insTime,
                                                               recvObsData->pseudorange->value,
                                                               recvObsData->satSigId.freq());

                auto satPosVel = satNavData->calcSatellitePosVel(satClk.transmitTime);
                auto lla_satPos = trafo::ecef2lla_WGS84(satPosVel.e_pos);

                LOG_DATA("{}: Adding satellite [{}] for receiver {}", nameId(), satData.satId, to_string(recv.type));
                satData.receiverData.emplace(recv.type, SatData::ReceiverSpecificData(recv.gnssObs, satPosVel.e_pos, lla_satPos, satPosVel.e_vel, recv.e_pos, recv.e_vel));

                if (satData.receiverData.at(recv.type).satElevation < _elevationMask)
                {
                    LOG_TRACE("{}: Satellite {} is skipped because of elevation mask. ({} < {})", nameId(), satId,
                              satData.receiverData.at(recv.type).satElevation, _elevationMask);
                    break;
                }
            }

            satelliteData.push_back(satData);
        }
        if (std::find_if(satelliteData.begin(), satelliteData.end(),
                         [&satId](const SatData& satData) { return satData.satId == satId; })
            == satelliteData.end())
        {
            continue; // Elevation mask triggered
        }

        for (const auto& recv : _receiver)
        {
            auto recvObsData = std::find_if(recv.gnssObs->data.begin(), recv.gnssObs->data.end(),
                                            [&obsData](const GnssObs::ObservationData& recvObsData) {
                                                return recvObsData.satSigId == obsData.satSigId;
                                            });

            auto obsIdx = static_cast<size_t>(recvObsData - recv.gnssObs->data.begin());

            for (const auto& obsType : obsTypes)
            {
                if (availableObservations.at(obsType) == _receiver.size())
                {
                    observations[recvObsData->satSigId][recv.type].emplace(obsType, Observation(recv.gnssObs, obsIdx));
                    nMeasurements[obsType]++;
                    switch (obsType)
                    {
                    case GnssObs::Pseudorange:
                        observations[recvObsData->satSigId][recv.type].at(obsType).measurement = recvObsData->pseudorange->value;
                        break;
                    case GnssObs::Carrier:
                        observations[recvObsData->satSigId][recv.type].at(obsType).measurement = recvObsData->carrierPhase->value;
                        break;
                    case GnssObs::Doppler:
                        observations[recvObsData->satSigId][recv.type].at(obsType).measurement = recvObsData->doppler.value();
                        break;
                    }
                }
            }
        }

        LOG_DATA("{}: Using observation [{}]", nameId(), obsData.satSigId);
    }

    size_t nMeasTotal = 0;
    std::string nMeasStr;
    for (auto& nMeas : nMeasurements)
    {
        nMeas.second /= _receiver.size();
        nMeasStr += std::to_string(nMeas.second) + ", ";
        nMeasTotal += nMeas.second;
    }
    if (nMeasStr.ends_with(", ")) { nMeasStr = nMeasStr.erase(nMeasStr.length() - 2); }

    LOG_DEBUG("{}: Using {} measurements ({}) from satellites", nameId(), nMeasTotal, nMeasStr);
    for (const auto& satData : satelliteData)
    {
        Frequency frequencies = Freq_None;
        Code codes;
        for (const auto& observation : observations)
        {
            if (satData.satId != observation.first.toSatId()) { continue; }
            frequencies |= observation.first.freq();
            codes |= observation.first.code;
        }
        LOG_DEBUG("{}:   [{}] on frequencies [{}] with codes [{}]", nameId(), satData.satId, frequencies, codes);
    }

    return std::make_tuple(satelliteData, observations, nMeasTotal);
}

void RealTimeKinematic::updatePivotSatellites(const std::vector<SatData>& satelliteData, const Observations& observations)
{
    // Update or erase pivot satellites from last epoch
    std::vector<Code> erasePivotCode;
    for (auto& pivotSatellite : _pivotSatellites)
    {
        auto satIter = std::find_if(satelliteData.begin(), satelliteData.end(), [&](const SatData& satData) {
            return pivotSatellite.second.satSigId.toSatId() == satData.satId;
        });

        if (satIter == satelliteData.end()
            || std::any_of(satIter->receiverData.begin(), satIter->receiverData.end(), // Do not use the pivot satellite anymore, if elevation < 10°
                           [](const auto& recvData) { return recvData.second.satElevation < deg2rad(10); }))
        {
            LOG_TRACE("{}: Dropping pivot satellite [{}] because: {}", nameId(), pivotSatellite.second.satSigId,
                      satIter == satelliteData.end() ? "Satellite not observed this epoch" : "Satellite elevation < 10°");
            erasePivotCode.push_back(pivotSatellite.first);
        }
    }
    for (const auto& code : erasePivotCode)
    {
        _pivotSatellites.erase(code);
    }

    // Determine pivot satellite
    for (const auto& satData : satelliteData)
    {
        for (const auto& observation : observations)
        {
            const auto satSigId = observation.first;
            const auto code = satSigId.code;
            if (satData.satId != satSigId.toSatId()) { continue; }

            if (!_pivotSatellites.contains(code))
            {
                _pivotSatellites.insert(std::make_pair(code, PivotSatellite{ .reevaluate = true, .satSigId = satSigId }));
                LOG_DATA("{}: Setting [{}] as new pivot satellite", nameId(), satSigId);
            }
            else if (auto& pivotSat = _pivotSatellites.at(code);
                     pivotSat.reevaluate) // Check if better pivot available
            {
                auto pivotIter = std::find_if(satelliteData.begin(), satelliteData.end(),
                                              [&pivotSat](const SatData& satData) { return satData.satId == pivotSat.satSigId.toSatId(); });
                if (pivotIter == satelliteData.end()) { LOG_CRITICAL("{}: None existing pivot satellite should have been removed earlier.", nameId()); }

                double elevation = 0.0;
                double pivotSatElevation = 0.0;
                for (const auto& recvObs : observation.second)
                {
                    elevation += satData.receiverData.at(recvObs.first).satElevation;
                    pivotSatElevation += pivotIter->receiverData.at(recvObs.first).satElevation;
                }
                elevation /= static_cast<double>(observation.second.size());
                pivotSatElevation /= static_cast<double>(observation.second.size());

                // Check if all frequencies available and select satellite with largest elevation
                LOG_DATA("{}: Pivot [{}] ele {}° <--> Sat [{}] ele {}°", nameId(), pivotIter->satId, rad2deg(pivotSatElevation),
                         satData.satId, rad2deg(elevation));
                if (elevation > pivotSatElevation)
                {
                    pivotSat = PivotSatellite{ .reevaluate = true, .satSigId = satSigId };
                    LOG_DATA("{}: Setting [{}] as new pivot satellite for satellite code [{}]", nameId(), satData.satId, satSigId.code);
                }
            }
        }
    }
    for (auto& [pivotCode, pivotSat] : _pivotSatellites)
    {
        if (pivotSat.reevaluate)
        {
            auto pivotIter = std::find_if(satelliteData.begin(), satelliteData.end(),
                                          [&pivotSat = pivotSat](const SatData& satData) { return satData.satId == pivotSat.satSigId.toSatId(); });

            LOG_TRACE("{}: Code [{}] uses [{}] as pivot satellite with elevation {:.4}°", nameId(),
                      pivotCode,
                      pivotSat.satSigId,
                      rad2deg(pivotIter->receiverData.begin()->second.satElevation));
            pivotSat.reevaluate = false;
        }
    }
}

void RealTimeKinematic::calcObservationEstimates(const std::vector<SatData>& satelliteData, Observations& observations, const IonosphericCorrections& ionosphericCorrections)
{
    for (const auto& satData : satelliteData)
    {
        for (auto& observation : observations)
        {
            const auto satSigId = observation.first;
            if (satData.satId != satSigId.toSatId()) { continue; }

            for (auto& [recv, recvObs] : observation.second)
            {
                const auto& receiver = _receiver.at(recv);
                const auto& recvSatData = satData.receiverData.at(recv);
                const auto freq = observation.first.freq();

                // Receiver-Satellite Range [m]
                double rho_r_s = (recvSatData.e_satPos - receiver.e_pos).norm();
                // Troposphere
                auto tropo_r_s = calcTroposphericDelayAndMapping(receiver.gnssObs->insTime, receiver.lla_pos,
                                                                 recvSatData.satElevation, recvSatData.satAzimuth, _troposphereModels);
                // Estimated troposphere propagation error [m]
                double dpsr_T_r_s = tropo_r_s.ZHD * tropo_r_s.zhdMappingFactor + tropo_r_s.ZWD * tropo_r_s.zwdMappingFactor;
                // Estimated ionosphere propagation error [m]
                double dpsr_I_r_s = calcIonosphericDelay(static_cast<double>(receiver.gnssObs->insTime.toGPSweekTow().tow),
                                                         freq, receiver.lla_pos, recvSatData.satElevation, recvSatData.satAzimuth,
                                                         _ionosphereModel, &ionosphericCorrections);
                // Sagnac correction [m]
                double dpsr_ie_r_s = calcSagnacCorrection(receiver.e_pos, recvSatData.e_satPos);

                for (auto& [obsType, obsData] : recvObs)
                {
                    switch (obsType)
                    {
                    case GnssObs::Pseudorange:
                        obsData.estimate = rho_r_s + dpsr_T_r_s + dpsr_I_r_s + dpsr_ie_r_s;
                        obsData.measVar = _gnssMeasurementErrorModel.psrMeasErrorVar(freq.getSatSys(), freq, recvSatData.satElevation);
                        break;
                    case GnssObs::Carrier:
                        obsData.estimate = rho_r_s + dpsr_T_r_s - dpsr_I_r_s + dpsr_ie_r_s;
                        obsData.measVar = _gnssMeasurementErrorModel.carrierMeasErrorVar(freq.getSatSys(), recvSatData.satElevation);
                        break;
                    case GnssObs::Doppler:
                        obsData.estimate = e_calcLineOfSightUnitVector(receiver.e_pos, recvSatData.e_satPos).transpose()
                                               * (recvSatData.e_satVel - receiver.e_vel)
                                           - calcSagnacRateCorrection(receiver.e_pos, recvSatData.e_satPos, receiver.e_vel, recvSatData.e_satVel);
                        obsData.measVar = _gnssMeasurementErrorModel.dopplerErrorVar();
                        break;
                    }
                }
            }
        }
    }
}

RealTimeKinematic::Differences RealTimeKinematic::calcSingleDifferences(const Observations& observations) const
{
    Differences singleDifferences;
    singleDifferences.reserve(observations.size());

    LOG_DATA("{}: Calculating single differences (rover - base):", nameId());
    for (const auto& observation : observations)
    {
        const auto satSigId = observation.first;

        const auto& baseObservations = observation.second.at(Base);
        const auto& roverObservations = observation.second.at(Rover);

        for (const auto& [obsType, baseObs] : baseObservations)
        {
            const auto& roverObs = roverObservations.at(obsType);

            singleDifferences[satSigId][obsType].estimate = roverObs.estimate - baseObs.estimate;
            singleDifferences[satSigId].at(obsType).measurement = roverObs.measurement - baseObs.measurement;
            singleDifferences[satSigId].at(obsType).measVar = roverObs.measVar + baseObs.measVar;

            if (obsType == GnssObs::Carrier)
            {
                // ------------------------------------------- Ambiguity ---------------------------------------------
                double lambda_j = InsConst::C / satSigId.freq().getFrequency(); // TODO: GLONASS frequency number
                double N_br_s = 0.0;                                            // TODO: Update with values from KF
                singleDifferences[satSigId][obsType].estimate += lambda_j * N_br_s;

                singleDifferences[satSigId].at(obsType).measurement *= lambda_j;
            }

            LOG_DATA("{}:   [{}][{:11}]e {} - {} = {}", nameId(), observation.first, NAV::to_string(obsType), roverObs.estimate, baseObs.estimate, singleDifferences[satSigId][obsType].estimate);
            LOG_DATA("{}:   [{}][{:11}]m {} - {} = {}", nameId(), observation.first, NAV::to_string(obsType), roverObs.measurement, baseObs.measurement, singleDifferences[satSigId][obsType].measurement);
        }
    }

    return singleDifferences;
}

RealTimeKinematic::Differences RealTimeKinematic::calcDoubleDifferences(const Differences& singleDifferences) const
{
    Differences doubleDifferences;
    doubleDifferences.reserve(singleDifferences.size() - 1);

    LOG_DATA("{}: Calculating double differences (sat - pivot):", nameId());
    for (const auto& [satSigId_s, singleDiff_s] : singleDifferences)
    {
        const auto& satSigId_1 = _pivotSatellites.at(satSigId_s.code).satSigId;
        if (satSigId_s == satSigId_1) { continue; } // No double difference with itself

        const auto& singleDiff_1 = singleDifferences.at(satSigId_1);

        for (const auto& [obsType, obs_s] : singleDiff_s)
        {
            const auto& obs_1 = singleDiff_1.at(obsType);

            doubleDifferences[satSigId_s][obsType].estimate = obs_s.estimate - obs_1.estimate;
            doubleDifferences[satSigId_s].at(obsType).measurement = obs_s.measurement - obs_1.measurement;
            doubleDifferences[satSigId_s].at(obsType).measVar = obs_s.measVar + obs_1.measVar;

            LOG_DATA("{}:   [{}][{:11}]e {} - {} = {}", nameId(), satSigId_s, NAV::to_string(obsType), obs_s.estimate, obs_1.estimate, doubleDifferences[satSigId_s][obsType].estimate);
            LOG_DATA("{}:   [{}][{:11}]m {} - {} = {}", nameId(), satSigId_s, NAV::to_string(obsType), obs_s.measurement, obs_1.measurement, doubleDifferences[satSigId_s][obsType].measurement);
        }
    }

    return doubleDifferences;
}

const char* RealTimeKinematic::to_string(RealTimeKinematic::ReceiverType receiver)
{
    switch (receiver)
    {
    case ReceiverType::Base:
        return "Base";
    case ReceiverType::Rover:
        return "Rover";
    }
    return "";
}

} // namespace NAV