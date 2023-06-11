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
                                                                       const Eigen::Vector3d& e_recPos)
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
                                               configWidth, unitWidth, _stdev_accel.data(), reinterpret_cast<int*>(&_stdevAccelUnits), "m/√(s^3)\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: stdev_accel changed to horizontal {} and vertical {}", nameId(), _stdev_accel.at(0), _stdev_accel.at(1));
            LOG_DEBUG("{}: stdevAccelNoiseUnits changed to {}", nameId(), fmt::underlying(_stdevAccelUnits));
            flow::ApplyChanges();
        }

        ImGui::TreePop();
    }
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

    j["stdevAccelUnits"] = _stdevAccelUnits;
    j["stdev_accel"] = _stdev_accel;

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
        _stdevAccelUnits = j.at("stdevAccelUnits");
    }
    if (j.contains("stdev_accel"))
    {
        _stdev_accel = j.at("stdev_accel");
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
    }
    else
    {
        _e_basePosition = (_e_basePosition + gnssObs->e_position()) / 2;
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
    _lla_roverPosition = Eigen::Vector3d(deg2rad(30), deg2rad(95.02), 0.0);
    _e_roverPosition = trafo::lla2ecef_WGS84(_lla_roverPosition);
    // _e_roverPosition = _e_basePosition;
    // _lla_roverPosition = trafo::ecef2lla_WGS84(_e_roverPosition);

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
        // ------------------------------------ GUI Settings conversion --------------------------------------

        // 𝜎_a Standard deviation of the acceleration due to user motion in horizontal and vertical component in [m / √(s^3)]
        std::array<double, 2> sigma2_accel{};
        switch (_stdevAccelUnits)
        {
        case StdevAccelUnits::m_sqrts3: // [m / √(s^3)]
            sigma2_accel = { std::pow(_stdev_accel.at(0), 2), std::pow(_stdev_accel.at(1), 2) };
            break;
        }
        LOG_DATA("  sigma2_accel = h: {}, v: {} [m^2 / s^3]", nameId(), sigma2_accel.at(0), sigma2_accel.at(1));

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
            const auto& pivotSatData = _pivotSatellites.at(satData.satId.satSys).satData;
            if (satData.satId == pivotSatData.satId) { continue; } // No double-difference with itself

            for (auto& [freq, signal] : satData.signals)
            {
                if (pivotSatData.signals.contains(freq))
                {
                    // Measurement innovation
                    LOG_DEBUG("{}: [{}] dz_psr = {:+} [m]", nameId(), SatSigId{ freq, satData.satId.satNum }, signal.doubleDifferenceMeasurementPseudorange - signal.doubleDifferenceEstimatePseudorange);
                    LOG_DEBUG("{}: [{}] dz_phi = {:+} [m]", nameId(), SatSigId{ freq, satData.satId.satNum }, signal.doubleDifferenceMeasurementCarrierPhase - signal.doubleDifferenceEstimateCarrierPhase);
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

                        auto satClk = satNavData->calcClockCorrections(_gnssObsRover->insTime,
                                                                       obsDataRover.pseudorange->value,
                                                                       obsDataRover.satSigId.freq);

                        auto satPosVel = satNavData->calcSatellitePosVel(satClk.transmitTime);
                        auto lla_satPos = trafo::ecef2lla_WGS84(satPosVel.e_pos);
                        LOG_DATA("{}:     e_satPos: {}", nameId(), satPosVel.e_pos.transpose());
                        LOG_DATA("{}:   lla_satPos: {}", nameId(), lla_satPos.transpose());
                        LOG_DATA("{}:     e_satVel: {}", nameId(), satPosVel.e_vel.transpose());

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
                            SatData satData = SatData(satId, satNavData, satPosVel.e_pos, lla_satPos, satPosVel.e_vel, _e_roverPosition, _e_basePosition);

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
    std::vector<SatelliteSystem> erasePivotSatSys;
    for (auto& [pivotSatSys, pivotSat] : _pivotSatellites)
    {
        auto satIter = std::find_if(satelliteData.begin(), satelliteData.end(), [&pivotSat](const SatData& satData) {
            return pivotSat.satData.satId == satData.satId;
        });
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
            erasePivotSatSys.push_back(pivotSatSys);
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
        auto satSys = satData.satId.satSys;

        if (!_pivotSatellites.contains(satSys))
        {
            _pivotSatellites.insert(std::make_pair(satSys, PivotSatellite{ .reevaluate = true, .satData = satData }));
            LOG_DATA("{}: Setting [{}] as new pivot satellite for satellite system [{}]", nameId(), satData.satId, satSys);
        }
        else if (auto& pivotSat = _pivotSatellites.at(satSys);
                 pivotSat.reevaluate) // Check if better pivot available
        {
            auto elevation = satData.rover.satElevation;
            auto pivotSatElevation = pivotSat.satData.rover.satElevation;

            size_t satFreqCount = 0;
            size_t pivotFreqCount = 0;
            if (auto gnssObsSatData = _gnssObsRover->satData(satData.satId))
            {
                satFreqCount = Frequency(gnssObsSatData.value().get().frequencies & _filterFreq).count();
                LOG_DATA("{}: Sat:   [{}] {} & {} = {}", nameId(), satData.satId, gnssObsSatData.value().get().frequencies, _filterFreq, satFreqCount);
            }
            if (auto gnssObsSatData = _gnssObsRover->satData(pivotSat.satData.satId))
            {
                pivotFreqCount = Frequency(gnssObsSatData.value().get().frequencies & _filterFreq).count();
                LOG_DATA("{}: Pivot: [{}] {} & {} = {}", nameId(), pivotSat.satData.satId, gnssObsSatData.value().get().frequencies, _filterFreq, pivotFreqCount);
            }
            // Check if all frequencies available and select satellite with largest elevation
            LOG_DATA("{}: Pivot [{}] freq {}, ele {}° <--> Sat [{}] freq {}, ele {}°", nameId(), pivotSat.satData.satId, pivotFreqCount, rad2deg(pivotSatElevation),
                     satData.satId, satFreqCount, rad2deg(elevation));
            if (satFreqCount > pivotFreqCount
                || (satFreqCount == pivotFreqCount && elevation > pivotSatElevation))
            {
                pivotSat = PivotSatellite{ .reevaluate = true, .satData = satData };
                LOG_DATA("{}: Setting [{}] as new pivot satellite for satellite system [{}]", nameId(), satData.satId, satSys);
            }
        }
    }
    for (auto& [pivotSatSys, pivotSat] : _pivotSatellites)
    {
        LOG_TRACE("{}: System [{}] uses [{}] as pivot satellite with frequencies '{}' and rover->sat elevation {:.4}°", nameId(),
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
    for (auto& satData : satelliteData)
    {
        const auto& pivotSatData = _pivotSatellites.at(satData.satId.satSys).satData;
        if (satData.satId == pivotSatData.satId) { continue; } // No double-difference with itself

        for (auto& [freq, signal] : satData.signals)
        {
            if (pivotSatData.signals.contains(freq))
            {
                const auto& pivotSignal = pivotSatData.signals.at(freq);

                double singleDiffMeasPseudorange_br_sat = signal.obsRover.obs().pseudorange->value - signal.obsBase.obs().pseudorange->value;
                double singleDiffMeasPseudorange_br_pivot = pivotSignal.obsRover.obs().pseudorange->value - pivotSignal.obsBase.obs().pseudorange->value;
                signal.doubleDifferenceMeasurementPseudorange = singleDiffMeasPseudorange_br_sat - singleDiffMeasPseudorange_br_pivot;
                LOG_DATA("{}: [{}]           psrMeas_br_s     = {} [m]", nameId(), SatSigId{ freq, satData.satId.satNum }, singleDiffMeasPseudorange_br_sat);
                LOG_DATA("{}:           [{}]  - psrMeas_br_1  = {} [m]", nameId(), SatSigId{ freq, pivotSatData.satId.satNum }, singleDiffMeasPseudorange_br_pivot);
                LOG_DATA("{}: [{}] - [{}]  = psrMeas_br_1s = {} [m]", nameId(), SatSigId{ freq, satData.satId.satNum }, SatSigId{ freq, pivotSatData.satId.satNum }, signal.doubleDifferenceMeasurementPseudorange);

                auto lambda_j = InsConst::C / freq.getFrequency();

                double singleDiffMeasCarrier_br_sat = lambda_j * (signal.obsRover.obs().carrierPhase->value - signal.obsBase.obs().carrierPhase->value);
                double singleDiffMeasCarrier_br_pivot = lambda_j * (pivotSignal.obsRover.obs().carrierPhase->value - pivotSignal.obsBase.obs().carrierPhase->value);
                signal.doubleDifferenceMeasurementCarrierPhase = singleDiffMeasCarrier_br_sat - singleDiffMeasCarrier_br_pivot;
                LOG_DATA("{}: [{}]           phiMeas_br_s     = {} [m]", nameId(), SatSigId{ freq, satData.satId.satNum }, singleDiffMeasCarrier_br_sat);
                LOG_DATA("{}:           [{}]  - phiMeas_br_1  = {} [m]", nameId(), SatSigId{ freq, pivotSatData.satId.satNum }, singleDiffMeasCarrier_br_pivot);
                LOG_DATA("{}: [{}] - [{}]  = phiMeas_br_1s = {} [m]", nameId(), SatSigId{ freq, satData.satId.satNum }, SatSigId{ freq, pivotSatData.satId.satNum }, signal.doubleDifferenceMeasurementCarrierPhase);

                doubleDifferencesCount++;
            }
        }
    }

    return doubleDifferencesCount;
}

void RealTimeKinematic::calculateEstimatedDoubleDifferences(std::vector<SatData>& satelliteData, const IonosphericCorrections& ionosphericCorrections)
{
    for (auto& satData : satelliteData)
    {
        const auto& pivotSatData = _pivotSatellites.at(satData.satId.satSys).satData;
        if (satData.satId == pivotSatData.satId) { continue; } // No double-difference with itself

        // ----------------------------------- Receiver-Satellite Range --------------------------------------
        double rho_r_sat = (satData.e_satPos - _e_roverPosition).norm();
        double rho_b_sat = (satData.e_satPos - _e_basePosition).norm();
        double rho_r_pivot = (pivotSatData.e_satPos - _e_roverPosition).norm();
        double rho_b_pivot = (pivotSatData.e_satPos - _e_basePosition).norm();
        // Double difference receiver-satellite range [m]
        double rho_br_1s = (rho_r_sat - rho_b_sat) - (rho_r_pivot - rho_b_pivot);
        LOG_DATA("{}: rho_br_1s {} [m]", nameId(), rho_br_1s);

        // ------------------------------------------ Troposphere --------------------------------------------
        auto tropo_r_sat = calcTroposphericDelayAndMapping(_gnssObsRover->insTime, _lla_roverPosition,
                                                           satData.rover.satElevation, satData.rover.satAzimuth, _troposphereModels);
        auto tropo_b_sat = calcTroposphericDelayAndMapping(_gnssObsBase->insTime, _lla_roverPosition,
                                                           satData.base.satElevation, satData.base.satAzimuth, _troposphereModels);
        auto tropo_r_pivot = calcTroposphericDelayAndMapping(_gnssObsRover->insTime, _lla_roverPosition,
                                                             pivotSatData.rover.satElevation, pivotSatData.rover.satAzimuth, _troposphereModels);
        auto tropo_b_pivot = calcTroposphericDelayAndMapping(_gnssObsBase->insTime, _lla_roverPosition,
                                                             pivotSatData.base.satElevation, pivotSatData.base.satAzimuth, _troposphereModels);
        // Estimated troposphere propagation error [m]
        double dpsr_T_r_sat = tropo_r_sat.ZHD * tropo_r_sat.zhdMappingFactor + tropo_r_sat.ZWD * tropo_r_sat.zwdMappingFactor;
        double dpsr_T_b_sat = tropo_b_sat.ZHD * tropo_b_sat.zhdMappingFactor + tropo_b_sat.ZWD * tropo_b_sat.zwdMappingFactor;
        double dpsr_T_r_pivot = tropo_r_pivot.ZHD * tropo_r_pivot.zhdMappingFactor + tropo_r_pivot.ZWD * tropo_r_pivot.zwdMappingFactor;
        double dpsr_T_b_pivot = tropo_b_pivot.ZHD * tropo_b_pivot.zhdMappingFactor + tropo_b_pivot.ZWD * tropo_b_pivot.zwdMappingFactor;
        // Double difference troposphere propagation error [m]
        double dpsr_T_br_1s = (dpsr_T_r_sat - dpsr_T_b_sat) - (dpsr_T_r_pivot - dpsr_T_b_pivot);
        LOG_DATA("{}: dpsr_T_br_1s {} [m]", nameId(), dpsr_T_br_1s);

        for (auto& [freq, signal] : satData.signals)
        {
            if (pivotSatData.signals.contains(freq))
            {
                // ------------------------------------------ Ionosphere ---------------------------------------------
                // Estimated ionosphere propagation error [s]
                double dpsr_I_r_sat = calcIonosphericTimeDelay(static_cast<double>(_gnssObsRover->insTime.toGPSweekTow().tow), freq, _lla_roverPosition,
                                                               satData.rover.satElevation, satData.rover.satAzimuth, _ionosphereModel, &ionosphericCorrections);
                double dpsr_I_b_sat = calcIonosphericTimeDelay(static_cast<double>(_gnssObsBase->insTime.toGPSweekTow().tow), freq, _lla_roverPosition,
                                                               satData.rover.satElevation, satData.rover.satAzimuth, _ionosphereModel, &ionosphericCorrections);
                double dpsr_I_r_pivot = calcIonosphericTimeDelay(static_cast<double>(_gnssObsRover->insTime.toGPSweekTow().tow), freq, _lla_roverPosition,
                                                                 pivotSatData.rover.satElevation, pivotSatData.rover.satAzimuth, _ionosphereModel, &ionosphericCorrections);
                double dpsr_I_b_pivot = calcIonosphericTimeDelay(static_cast<double>(_gnssObsBase->insTime.toGPSweekTow().tow), freq, _lla_roverPosition,
                                                                 pivotSatData.rover.satElevation, pivotSatData.rover.satAzimuth, _ionosphereModel, &ionosphericCorrections);
                // Double difference ionosphere propagation error [m]
                double dpsr_I_br_1s = ((dpsr_I_r_sat - dpsr_I_b_sat) - (dpsr_I_r_pivot - dpsr_I_b_pivot)) * InsConst::C;
                LOG_DATA("{}: [{}] dpsr_I_br_1s {} [m]", nameId(), freq, dpsr_I_br_1s);

                // ------------------------------------------- Ambiguity ---------------------------------------------
                double lambda_j = InsConst::C / freq.getFrequency();
                double N_br_1s = 0.0; // TODO: Update with values from KF
                LOG_DATA("{}: N_br_1s {} [-]", nameId(), N_br_1s);

                signal.doubleDifferenceEstimatePseudorange = rho_br_1s + dpsr_T_br_1s + dpsr_I_br_1s;
                LOG_DATA("{}: [{}] - [{}] psrEst_br_1s = {} [m]", nameId(), SatSigId{ freq, satData.satId.satNum }, SatSigId{ freq, pivotSatData.satId.satNum }, signal.doubleDifferenceEstimatePseudorange);
                signal.doubleDifferenceEstimateCarrierPhase = rho_br_1s + dpsr_T_br_1s - dpsr_I_br_1s + lambda_j * N_br_1s;
                LOG_DATA("{}: [{}] - [{}] phiEst_br_1s = {} [m]", nameId(), SatSigId{ freq, satData.satId.satNum }, SatSigId{ freq, pivotSatData.satId.satNum }, signal.doubleDifferenceEstimateCarrierPhase);
            }
        }
    }
}

} // namespace NAV