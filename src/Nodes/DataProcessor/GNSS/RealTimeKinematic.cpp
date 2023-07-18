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
#include "Navigation/GNSS/Positioning/SppAlgorithm.hpp"
#include "Navigation/GNSS/Satellite/Ephemeris/GLONASSEphemeris.hpp"
#include "Navigation/Math/LeastSquares.hpp"

namespace NAV
{

RealTimeKinematic::SatData::ReceiverSpecificData::ReceiverSpecificData(const Eigen::Vector3d& e_satPos,
                                                                       const Eigen::Vector3d& lla_satPos,
                                                                       Eigen::Vector3d e_satVel,
                                                                       const Eigen::Vector3d& e_recPos)
    : e_satPos(e_satPos), lla_satPos(lla_satPos), e_satVel(std::move(e_satVel))
{
    e_lineOfSightUnitVector = e_calcLineOfSightUnitVector(e_recPos, e_satPos);
    Eigen::Vector3d n_lineOfSightUnitVector = trafo::n_Quat_e(lla_satPos(0), lla_satPos(1)) * e_lineOfSightUnitVector;
    satElevation = calcSatElevation(n_lineOfSightUnitVector);
    satAzimuth = calcSatAzimuth(n_lineOfSightUnitVector);

    LOG_DATA("    e_lineOfSightUnitVector {}", e_lineOfSightUnitVector.transpose());
    LOG_DATA("    n_lineOfSightUnitVector {}", n_lineOfSightUnitVector.transpose());
    LOG_DATA("    satElevation {}°", rad2deg(satElevation));
    LOG_DATA("    satAzimuth   {}°", rad2deg(satAzimuth));
}

RealTimeKinematic::RealTimeKinematic()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 407, 506 };

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
    j["ionosphereModel"] = _ionosphereModel;
    j["troposphereModels"] = _troposphereModels;

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
    if (j.contains("ionosphereModel"))
    {
        j.at("ionosphereModel").get_to(_ionosphereModel);
    }
    if (j.contains("troposphereModels"))
    {
        j.at("troposphereModels").get_to(_troposphereModels);
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

    _e_basePosition = Eigen::Vector3d::Zero();
    _lla_basePosition = Eigen::Vector3d::Zero();
    _gnssObsBase = nullptr;
    _gnssObsRover = nullptr;

    _e_roverPosition = Eigen::Vector3d::Zero();
    _lla_roverPosition = Eigen::Vector3d::Zero();
    _e_roverVelocity = Eigen::Vector3d::Zero();

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
    LOG_WARN("{}: Received Base  Position for [{}]", nameId(), gnssObs->insTime);

    if (_e_basePosition.isZero())
    {
        _e_basePosition = gnssObs->e_position();
        _lla_basePosition = gnssObs->lla_position();
    }
    else
    {
        _e_basePosition = (_e_basePosition + gnssObs->e_position()) / 2;
        _lla_basePosition = trafo::ecef2lla_WGS84(_e_basePosition);
    }
}

void RealTimeKinematic::recvBaseGnssObs(InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
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
        if (auto rtkSol = calcFallbackSppSolution())
        {
            invokeCallbacks(OUTPUT_PORT_INDEX_RTKSOL, rtkSol);
        }
        _gnssObsRover = nullptr;
    }
}

void RealTimeKinematic::recvRoverGnssObs(InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    if (!_gnssObsBase && _gnssObsRover) // Did not receive a base observation for the last rover observation
    {
        if (auto rtkSol = calcFallbackSppSolution())
        {
            invokeCallbacks(OUTPUT_PORT_INDEX_RTKSOL, rtkSol);
        }
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
        if (auto rtkSol = calcFallbackSppSolution())
        {
            invokeCallbacks(OUTPUT_PORT_INDEX_RTKSOL, rtkSol);
        }
        _gnssObsRover = nullptr;
    }
}

void RealTimeKinematic::calcRealTimeKinematicSolution()
{
    LOG_ERROR("{}: Calculate RTK Solution for  [{}]", nameId(), _gnssObsRover->insTime);

    // Calculate a Single point solution if the Rover Position is not known yet
    if (_e_roverPosition.isZero())
    {
        if (auto sol = calcFallbackSppSolution())
        {
            _e_roverPosition = sol->e_position();
            _lla_roverPosition = trafo::ecef2lla_WGS84(_e_roverPosition);
            _e_roverVelocity = sol->e_velocity();
        }
        else
        {
            return;
        }
    }

    // TODO: Debugging, remove later
    // _lla_roverPosition = Eigen::Vector3d(deg2rad(30), deg2rad(95.02), 0.0);
    // _e_roverPosition = trafo::lla2ecef_WGS84(_lla_roverPosition);

    _e_basePosition = Eigen::Vector3d{ -481819.3135, 5507219.9538, 3170373.7354 };
    _e_roverPosition = Eigen::Vector3d{ -483741.6665, 5507051.4316, 3170373.7354 };
    _lla_roverPosition = trafo::ecef2lla_WGS84(_e_roverPosition);
    _lla_basePosition = trafo::ecef2lla_WGS84(_e_basePosition);

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
        IonosphericCorrections ionosphericCorrections;
        for (const auto* gnssNavInfo : gnssNavInfos)
        {
            for (const auto& correction : gnssNavInfo->ionosphericCorrections.data())
            {
                if (!ionosphericCorrections.contains(correction.satSys, correction.alphaBeta))
                {
                    ionosphericCorrections.insert(correction.satSys, correction.alphaBeta, correction.data);
                }
            }
        }

        // ----------------------------------------- RTK Algorithm -------------------------------------------

        auto rtkSol = std::make_shared<RtkSolution>();
        rtkSol->insTime = _gnssObsRover->insTime;

        // Data calculated for each satellite (only satellites filtered by GUI filter & NAV data available & ...)
        std::vector<SatData> satelliteData = selectSatellitesForCalculation(gnssNavInfos);

        updatePivotSatellites(satelliteData);

        calculateMeasurementDoubleDifferences(satelliteData);
        calculateEstimatedDoubleDifferences(satelliteData, ionosphericCorrections);

        for (auto& satData : satelliteData)
        {
            for (auto& [freq, signal] : satData.signals)
            {
                const auto& pivotSatData = _pivotSatellites.at(freq).satData;
                if (satData.satId == pivotSatData.satId) { continue; } // No double-difference with itself

                if (pivotSatData.signals.contains(freq))
                {
                    // Measurement innovation (psr, phi)
                    // LOG_DEBUG("{}: [{}] d_psr_s  = {:+e} [m]", nameId(), SatSigId{ freq, satData.satId.satNum }, signal.obsRover.obs().pseudorange->value - signal.obsRover.psrEst);
                    // Measurement innovation (single-difference)
                    // LOG_DEBUG("{}: [{}] dz_psr_s  = {:+e} [m]", nameId(), SatSigId{ freq, satData.satId.satNum }, signal.singleDiffMeasPseudorange_br_s - signal.singleDiffEstPseudorange_br_s);
                    // LOG_DEBUG("{}: [{}] dz_phi_s  = {:+e} [m]", nameId(), SatSigId{ freq, satData.satId.satNum }, signal.singleDiffMeasCarrier_br_s - signal.singleDiffEstCarrier_br_s);
                    // Measurement innovation (double-difference)
                    LOG_DEBUG("{}: [{}] dz_psr_1s = {:+e} [m]", nameId(), SatSigId{ freq, satData.satId.satNum }, signal.doubleDiffMeasPseudorange_br_1s - signal.doubleDiffEstPseudorange_br_1s);
                    LOG_DEBUG("{}: [{}] dz_phi_1s = {:+e} [m]", nameId(), SatSigId{ freq, satData.satId.satNum }, signal.doubleDiffMeasCarrier_br_1s - signal.doubleDiffEstCarrier_br_1s);
                }
            }
        }

        invokeCallbacks(OUTPUT_PORT_INDEX_RTKSOL, rtkSol);
    }

    _gnssObsBase = nullptr; // FIXME: Probable we should interpolate the base observations, in case we get them with lower frequency
    _gnssObsRover = nullptr;
}

std::shared_ptr<RtkSolution> RealTimeKinematic::calcFallbackSppSolution()
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
        SppState state = { .e_position = _e_roverPosition,
                           .e_velocity = _e_roverVelocity,
                           .recvClk = {} };
        if (auto sppSol = calcSppSolution(state, _gnssObsRover, gnssNavInfos,
                                          _ionosphereModel, _troposphereModels, SppEstimator::WEIGHTED_LEAST_SQUARES,
                                          _filterFreq, _filterCode, _excludedSatellites, _elevationMask))
        {
            _e_roverPosition = sppSol->e_position();
            _lla_roverPosition = trafo::ecef2lla_WGS84(_e_roverPosition);
            _e_roverVelocity = sppSol->e_velocity();

            auto rtkSol = std::make_shared<RtkSolution>();
            rtkSol->insTime = sppSol->insTime;
            rtkSol->setPositionAndStdDev_e(sppSol->e_position(), sppSol->e_positionStdev());
            rtkSol->setVelocityAndStdDev_e(sppSol->e_velocity(), sppSol->e_velocityStdev());
            rtkSol->nSatellitesPosition = sppSol->nSatellitesPosition;
            rtkSol->nSatellitesVelocity = sppSol->nSatellitesVelocity;
            rtkSol->recvClk = sppSol->recvClk;

            rtkSol->solType = RtkSolution::SolutionType::SPP;

            return rtkSol;
        }
    }

    return nullptr;
}

std::vector<RealTimeKinematic::SatData> RealTimeKinematic::selectSatellitesForCalculation(const std::vector<const GnssNavInfo*>& gnssNavInfos)
{
    std::vector<SatData> satelliteData;
    satelliteData.reserve(_gnssObsRover->data.size());

    for (size_t obsIdxRover = 0; obsIdxRover < _gnssObsRover->data.size(); obsIdxRover++)
    {
        const auto& obsDataRover = _gnssObsRover->data.at(obsIdxRover);
        auto satId = obsDataRover.satSigId.toSatId();

        if ((obsDataRover.satSigId.freq & _filterFreq)                                                                // frequency is selected in GUI
            && (obsDataRover.code & _filterCode)                                                                      // code is selected in GUI
            && obsDataRover.pseudorange                                                                               // has a valid pseudorange
            && obsDataRover.carrierPhase                                                                              // has a valid carrier-phase
            && std::find(_excludedSatellites.begin(), _excludedSatellites.end(), satId) == _excludedSatellites.end()) // is not excluded
        {
            auto obsDataBase = std::find_if(_gnssObsBase->data.begin(), _gnssObsBase->data.end(),
                                            [&obsDataRover](const GnssObs::ObservationData& obsDataBase) {
                                                return obsDataBase.satSigId == obsDataRover.satSigId
                                                       && obsDataBase.code == obsDataRover.code;
                                            });

            if (obsDataBase != _gnssObsBase->data.end() // base also has observation for this satellite
                && obsDataBase->pseudorange             // has a valid pseudorange
                && obsDataBase->carrierPhase)           // has a valid carrier-phase
            {
                for (const auto& gnssNavInfo : gnssNavInfos)
                {
                    if (auto satNavData = gnssNavInfo->searchNavigationData(satId, _gnssObsRover->insTime); // can calculate satellite position
                        satNavData && satNavData->isHealthy())
                    {
                        LOG_DATA("{}: Using observation [{}] with code [{}]", nameId(), obsDataRover.satSigId, obsDataRover.code);

                        auto roverSatClk = satNavData->calcClockCorrections(_gnssObsRover->insTime,
                                                                            obsDataRover.pseudorange->value,
                                                                            obsDataRover.satSigId.freq);

                        auto roverSatPosVel = satNavData->calcSatellitePosVel(roverSatClk.transmitTime);
                        auto lla_roverSatPos = trafo::ecef2lla_WGS84(roverSatPosVel.e_pos);

                        auto baseSatClk = satNavData->calcClockCorrections(_gnssObsBase->insTime,
                                                                           obsDataBase->pseudorange->value,
                                                                           obsDataBase->satSigId.freq);

                        auto baseSatPosVel = satNavData->calcSatellitePosVel(baseSatClk.transmitTime);
                        auto lla_baseSatPos = trafo::ecef2lla_WGS84(baseSatPosVel.e_pos);

                        LOG_DATA("{}:     e_roverSatPos - e_baseSatPos: {}", nameId(), (roverSatPosVel.e_pos - baseSatPosVel.e_pos).transpose());

                        auto obsIdxBase = static_cast<size_t>(obsDataBase - _gnssObsBase->data.begin());
                        if (auto iter = std::find_if(satelliteData.begin(), satelliteData.end(), [&satId](const SatData& satData) {
                                return satData.satId == satId;
                            });
                            iter != satelliteData.end())
                        {
                            iter->signals.insert(std::make_pair(obsDataRover.satSigId.freq, SatData::Signal(_gnssObsRover, obsIdxRover, _gnssObsBase, obsIdxBase)));
                        }
                        else
                        {
                            SatData satData = SatData(satId, satNavData,
                                                      roverSatPosVel.e_pos, lla_roverSatPos, roverSatPosVel.e_vel,
                                                      baseSatPosVel.e_pos, lla_baseSatPos, baseSatPosVel.e_vel,
                                                      _e_roverPosition, _e_basePosition);

                            if (satData.rover.satElevation < _elevationMask
                                || satData.base.satElevation < _elevationMask)
                            {
                                LOG_DATA("   Satellite {} is skipped because of elevation mask. ({} < {})", satId,
                                         std::min(satData.rover.satElevation, satData.base.satElevation), _elevationMask);
                                break;
                            }

                            satData.signals.insert(std::make_pair(obsDataRover.satSigId.freq, SatData::Signal(_gnssObsRover, obsIdxRover, _gnssObsBase, obsIdxBase)));
                            satelliteData.push_back(satData);
                        }

                        break;
                    }
                }
            }
        }
    }

    LOG_TRACE("{}: Using satellites", nameId());
    for (const auto& satData : satelliteData)
    {
        Frequency frequencies = Freq_None;
        for (const auto& sig : satData.signals)
        {
            frequencies |= sig.first;
        }
        LOG_TRACE("{}:   [{}] on frequencies [{}]", nameId(), satData.satId, frequencies);
    }

    return satelliteData;
}

void RealTimeKinematic::updatePivotSatellites(const std::vector<SatData>& satelliteData)
{
    // Update or erase pivot satellites from last epoch
    std::vector<Frequency> erasePivotSatSys;
    for (auto& pivotSatellite : _pivotSatellites)
    {
        auto satIter = std::find_if(satelliteData.begin(), satelliteData.end(), [&](const SatData& satData) {
            return pivotSatellite.second.satData.satId == satData.satId;
        });
        auto& [pivotFreq, pivotSat] = pivotSatellite;
        if (satIter != satelliteData.end()
            && satIter->rover.satElevation > deg2rad(10) // Do not use the pivot satellite anymore, if elevation < 10°
            && satIter->base.satElevation > deg2rad(10))
        {
            LOG_DATA("{}: Updating pivot satellite [{}] from [{}] to [{}]", nameId(), satIter->satId,
                     pivotSat.satData.signals.begin()->second.obsRover.gnssObs->insTime,
                     satIter->signals.begin()->second.obsRover.gnssObs->insTime);
            pivotSat.satData = *satIter;
        }
        else
        {
            erasePivotSatSys.push_back(pivotFreq);
        }
    }
    for (const auto& satSys : erasePivotSatSys)
    {
        LOG_TRACE("{}: Dropping pivot satellite [{}] for system [{}]", nameId(), _pivotSatellites.at(satSys).satData.satId, satSys);
        _pivotSatellites.erase(satSys);
    }

    // Determine pivot satellite
    for (const auto& satData : satelliteData)
    {
        for (const auto& [freq, signal] : satData.signals)
        {
            if (!_pivotSatellites.contains(freq))
            {
                _pivotSatellites.insert(std::make_pair(freq, PivotSatellite{ .reevaluate = true, .satData = satData }));
                LOG_DATA("{}: Setting [{}] as new pivot satellite for satellite frequency [{}]", nameId(), satData.satId, freq);
            }
            else if (auto& pivotSat = _pivotSatellites.at(freq);
                     pivotSat.reevaluate) // Check if better pivot available
            {
                auto elevation = satData.rover.satElevation;
                auto pivotSatElevation = pivotSat.satData.rover.satElevation;

                // Check if all frequencies available and select satellite with largest elevation
                LOG_DATA("{}: Pivot [{}] ele {}° <--> Sat [{}] ele {}°", nameId(), pivotSat.satData.satId, rad2deg(pivotSatElevation),
                         satData.satId, rad2deg(elevation));
                if (elevation > pivotSatElevation)
                {
                    pivotSat = PivotSatellite{ .reevaluate = true, .satData = satData };
                    LOG_DATA("{}: Setting [{}] as new pivot satellite for satellite frequency [{}]", nameId(), satData.satId, freq);
                }
            }
        }
    }
    for (auto& [pivotSatSys, pivotSat] : _pivotSatellites)
    {
        LOG_TRACE("{}: Frequency [{}] uses [{}] as pivot satellite with frequencies '{}' and rover->sat elevation {:.4}°", nameId(),
                  pivotSatSys,
                  pivotSat.satData.satId,
                  _gnssObsRover->satData(pivotSat.satData.satId).value().get().frequencies,
                  rad2deg(pivotSat.satData.rover.satElevation));
        pivotSat.reevaluate = false;
    }
}

size_t RealTimeKinematic::calculateMeasurementDoubleDifferences(std::vector<SatData>& satelliteData)
{
    size_t doubleDifferencesCount = 0;
    for (auto& satData_s : satelliteData)
    {
        for (auto& [freq, signal_s] : satData_s.signals)
        {
            const auto& satData_1 = _pivotSatellites.at(freq).satData;
            if (satData_s.satId == satData_1.satId) { continue; } // No double-difference with itself

            if (satData_1.signals.contains(freq))
            {
                const auto& signal_1 = satData_1.signals.at(freq);

                signal_s.singleDiffMeasPseudorange_br_s = signal_s.obsRover.obs().pseudorange->value - signal_s.obsBase.obs().pseudorange->value;
                double singleDiffMeasPseudorange_br_1 = signal_1.obsRover.obs().pseudorange->value - signal_1.obsBase.obs().pseudorange->value;
                signal_s.doubleDiffMeasPseudorange_br_1s = signal_s.singleDiffMeasPseudorange_br_s - singleDiffMeasPseudorange_br_1;
                LOG_DATA("{}: [{}]           psrMeas_br_s     = {} [m]", nameId(), SatSigId{ freq, satData_s.satId.satNum }, signal_s.singleDiffMeasPseudorange_br_s);
                LOG_DATA("{}:           [{}]  - psrMeas_br_1  = {} [m]", nameId(), SatSigId{ freq, satData_1.satId.satNum }, singleDiffMeasPseudorange_br_1);
                LOG_DATA("{}: [{}] - [{}]  = psrMeas_br_1s = {} [m]", nameId(), SatSigId{ freq, satData_s.satId.satNum }, SatSigId{ freq, satData_1.satId.satNum }, signal_s.doubleDiffMeasPseudorange_br_1s);

                auto lambda_j = InsConst::C / freq.getFrequency();

                signal_s.singleDiffMeasCarrier_br_s = lambda_j * (signal_s.obsRover.obs().carrierPhase->value - signal_s.obsBase.obs().carrierPhase->value);
                double singleDiffMeasCarrier_br_1 = lambda_j * (signal_1.obsRover.obs().carrierPhase->value - signal_1.obsBase.obs().carrierPhase->value);
                signal_s.doubleDiffMeasCarrier_br_1s = signal_s.singleDiffMeasCarrier_br_s - singleDiffMeasCarrier_br_1;
                LOG_DATA("{}: [{}]           phiMeas_br_s     = {} [m]", nameId(), SatSigId{ freq, satData_s.satId.satNum }, signal_s.singleDiffMeasCarrier_br_s);
                LOG_DATA("{}:           [{}]  - phiMeas_br_1  = {} [m]", nameId(), SatSigId{ freq, satData_1.satId.satNum }, singleDiffMeasCarrier_br_1);
                LOG_DATA("{}: [{}] - [{}]  = phiMeas_br_1s = {} [m]", nameId(), SatSigId{ freq, satData_s.satId.satNum }, SatSigId{ freq, satData_1.satId.satNum }, signal_s.doubleDiffMeasCarrier_br_1s);

                doubleDifferencesCount++;
            }
        }
    }

    return doubleDifferencesCount;
}

void RealTimeKinematic::calculateEstimatedDoubleDifferences(std::vector<SatData>& satelliteData, const IonosphericCorrections& ionosphericCorrections)
{
    for (auto& satData_s : satelliteData)
    {
        for (auto& [freq, signal] : satData_s.signals)
        {
            const auto& satData_1 = _pivotSatellites.at(freq).satData;
            if (satData_s.satId == satData_1.satId) { continue; } // No double-difference with itself

            // ----------------------------------- Receiver-Satellite Range --------------------------------------
            double rho_r_s = (satData_s.rover.e_satPos - _e_roverPosition).norm();
            double rho_b_s = (satData_s.base.e_satPos - _e_basePosition).norm();
            double rho_r_1 = (satData_1.rover.e_satPos - _e_roverPosition).norm();
            double rho_b_1 = (satData_1.base.e_satPos - _e_basePosition).norm();
            double rho_br_s = rho_r_s - rho_b_s;
            double rho_br_1 = rho_r_1 - rho_b_1;

            // ------------------------------------------ Troposphere --------------------------------------------
            auto tropo_r_s = calcTroposphericDelayAndMapping(_gnssObsRover->insTime, _lla_roverPosition,
                                                             satData_s.rover.satElevation, satData_s.rover.satAzimuth, _troposphereModels);
            auto tropo_b_s = calcTroposphericDelayAndMapping(_gnssObsBase->insTime, _lla_roverPosition,
                                                             satData_s.base.satElevation, satData_s.base.satAzimuth, _troposphereModels);
            auto tropo_r_1 = calcTroposphericDelayAndMapping(_gnssObsRover->insTime, _lla_roverPosition,
                                                             satData_1.rover.satElevation, satData_1.rover.satAzimuth, _troposphereModels);
            auto tropo_b_1 = calcTroposphericDelayAndMapping(_gnssObsBase->insTime, _lla_roverPosition,
                                                             satData_1.base.satElevation, satData_1.base.satAzimuth, _troposphereModels);
            // Estimated troposphere propagation error [m]
            double dpsr_T_r_s = tropo_r_s.ZHD * tropo_r_s.zhdMappingFactor + tropo_r_s.ZWD * tropo_r_s.zwdMappingFactor;
            double dpsr_T_b_s = tropo_b_s.ZHD * tropo_b_s.zhdMappingFactor + tropo_b_s.ZWD * tropo_b_s.zwdMappingFactor;
            double dpsr_T_r_1 = tropo_r_1.ZHD * tropo_r_1.zhdMappingFactor + tropo_r_1.ZWD * tropo_r_1.zwdMappingFactor;
            double dpsr_T_b_1 = tropo_b_1.ZHD * tropo_b_1.zhdMappingFactor + tropo_b_1.ZWD * tropo_b_1.zwdMappingFactor;
            double dpsr_T_br_s = dpsr_T_r_s - dpsr_T_b_s;
            double dpsr_T_br_1 = dpsr_T_r_1 - dpsr_T_b_1;

            // Sagnac correction [m] - Springer Handbook ch. 19.1.1, eq. 19.7, p. 562
            double dpsr_ie_r_s = 1.0 / InsConst::C * (_e_roverPosition - satData_s.rover.e_satPos).dot(InsConst::e_omega_ie.cross(_e_roverPosition));
            double dpsr_ie_b_s = 1.0 / InsConst::C * (_e_basePosition - satData_s.base.e_satPos).dot(InsConst::e_omega_ie.cross(_e_basePosition));
            double dpsr_ie_r_1 = 1.0 / InsConst::C * (_e_roverPosition - satData_1.rover.e_satPos).dot(InsConst::e_omega_ie.cross(_e_roverPosition));
            double dpsr_ie_b_1 = 1.0 / InsConst::C * (_e_basePosition - satData_1.base.e_satPos).dot(InsConst::e_omega_ie.cross(_e_basePosition));
            double dpsr_ie_br_s = dpsr_ie_r_s - dpsr_ie_b_s;
            double dpsr_ie_br_1 = dpsr_ie_r_1 - dpsr_ie_b_1;

            // ------------------------------------------ Ionosphere ---------------------------------------------
            // Estimated ionosphere propagation error [s]
            double dpsr_I_r_s = calcIonosphericTimeDelay(static_cast<double>(_gnssObsRover->insTime.toGPSweekTow().tow), freq, _lla_roverPosition,
                                                         satData_s.rover.satElevation, satData_s.rover.satAzimuth, _ionosphereModel, &ionosphericCorrections);
            double dpsr_I_b_s = calcIonosphericTimeDelay(static_cast<double>(_gnssObsBase->insTime.toGPSweekTow().tow), freq, _lla_basePosition,
                                                         satData_s.base.satElevation, satData_s.base.satAzimuth, _ionosphereModel, &ionosphericCorrections);
            double dpsr_I_r_1 = calcIonosphericTimeDelay(static_cast<double>(_gnssObsRover->insTime.toGPSweekTow().tow), freq, _lla_roverPosition,
                                                         satData_1.rover.satElevation, satData_1.rover.satAzimuth, _ionosphereModel, &ionosphericCorrections);
            double dpsr_I_b_1 = calcIonosphericTimeDelay(static_cast<double>(_gnssObsBase->insTime.toGPSweekTow().tow), freq, _lla_basePosition,
                                                         satData_1.base.satElevation, satData_1.base.satAzimuth, _ionosphereModel, &ionosphericCorrections);
            double dpsr_I_br_s = (dpsr_I_r_s - dpsr_I_b_s) * InsConst::C;
            double dpsr_I_br_1 = (dpsr_I_r_1 - dpsr_I_b_1) * InsConst::C;

            // ------------------------------------------- Ambiguity ---------------------------------------------
            double lambda_j = InsConst::C / freq.getFrequency();
            double N_br_s = 0.0; // TODO: Update with values from KF
            double N_br_1 = 0.0;

            signal.obsRover.psrEst = rho_r_s + dpsr_T_r_s + dpsr_I_r_s + dpsr_ie_r_s;
            signal.obsBase.psrEst = rho_b_s + dpsr_T_b_s + dpsr_I_b_s + dpsr_ie_b_s;

            signal.singleDiffEstPseudorange_br_s = rho_br_s + dpsr_T_br_s + dpsr_I_br_s + dpsr_ie_br_s;
            double singleDiffEstPseudorange_br_1 = rho_br_1 + dpsr_T_br_1 + dpsr_I_br_1 + dpsr_ie_br_1;
            signal.doubleDiffEstPseudorange_br_1s = signal.singleDiffEstPseudorange_br_s - singleDiffEstPseudorange_br_1;
            LOG_DATA("{}: [{}]           psrEst_br_s     = {} [m]", nameId(), SatSigId{ freq, satData_s.satId.satNum }, signal.singleDiffEstPseudorange_br_s);
            LOG_DATA("{}:           [{}]  - psrEst_br_1  = {} [m]", nameId(), SatSigId{ freq, satData_1.satId.satNum }, singleDiffEstPseudorange_br_1);
            LOG_DATA("{}: [{}] - [{}]  = psrEst_br_1s = {} [m]", nameId(), SatSigId{ freq, satData_s.satId.satNum }, SatSigId{ freq, satData_1.satId.satNum }, signal.doubleDiffEstPseudorange_br_1s);

            signal.singleDiffEstCarrier_br_s = rho_br_s + dpsr_T_br_s - dpsr_I_br_s + dpsr_ie_br_s + lambda_j * N_br_s;
            double singleDiffEstCarrier_br_1 = rho_br_1 + dpsr_T_br_1 - dpsr_I_br_1 + dpsr_ie_br_1 + lambda_j * N_br_1;
            signal.doubleDiffEstCarrier_br_1s = signal.singleDiffEstCarrier_br_s - singleDiffEstCarrier_br_1;
            LOG_DATA("{}: [{}]           phiEst_br_s     = {} [m]", nameId(), SatSigId{ freq, satData_s.satId.satNum }, signal.singleDiffEstCarrier_br_s);
            LOG_DATA("{}:           [{}]  - phiEst_br_1  = {} [m]", nameId(), SatSigId{ freq, satData_1.satId.satNum }, singleDiffEstCarrier_br_1);
            LOG_DATA("{}: [{}] - [{}]  = phiEst_br_1s = {} [m]", nameId(), SatSigId{ freq, satData_s.satId.satNum }, SatSigId{ freq, satData_1.satId.satNum }, signal.doubleDiffEstCarrier_br_1s);
        }
    }
}

} // namespace NAV