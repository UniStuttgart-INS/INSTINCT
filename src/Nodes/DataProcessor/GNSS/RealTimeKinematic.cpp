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
#include "NodeData/GNSS/GnssObs.hpp"
#include "NodeData/GNSS/GnssNavInfo.hpp"
#include "NodeData/GNSS/RtkSolution.hpp"

#include "Navigation/GNSS/Functions.hpp"
#include "Navigation/GNSS/Satellite/Ephemeris/GLONASSEphemeris.hpp"
#include "Navigation/Math/LeastSquares.hpp"

NAV::RealTimeKinematic::RealTimeKinematic()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 407, 506 };

    nm::CreateInputPin(this, "Base Position", Pin::Type::Flow, { NAV::Pos::type() }, &RealTimeKinematic::recvBasePos);
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

void NAV::RealTimeKinematic::recvBasePos(NAV::InputPin::NodeDataQueue& /* queue */, size_t /* pinIdx */)
{
}

void NAV::RealTimeKinematic::recvBaseGnssObs(NAV::InputPin::NodeDataQueue& /* queue */, size_t /* pinIdx */)
{
}

void NAV::RealTimeKinematic::recvRoverGnssObs(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    // Collection of all connected navigation data providers
    std::vector<const GnssNavInfo*> gnssNavInfos;
    for (size_t i = 0; i < _nNavInfoPins; i++)
    {
        if (const auto* gnssNavInfo = getInputValue<const GnssNavInfo>(INPUT_PORT_INDEX_GNSS_NAV_INFO + i))
        {
            gnssNavInfos.push_back(gnssNavInfo);
        }
    }
    if (gnssNavInfos.empty()) { return; }

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

    auto gnssObs = std::static_pointer_cast<const GnssObs>(queue.extract_front());
    LOG_DATA("{}: Calculating SPP for [{}]", nameId(), gnssObs->insTime);

    auto sppSol = std::make_shared<RtkSolution>();
    sppSol->insTime = gnssObs->insTime;

    // Data calculated for each observation
    struct CalcData
    {
        // Constructor
        explicit CalcData(const NAV::GnssObs::ObservationData& obsData, std::shared_ptr<NAV::SatNavData> satNavData)
            : obsData(obsData), satNavData(std::move(satNavData)) {}

        const NAV::GnssObs::ObservationData& obsData;          // GNSS Observation data
        std::shared_ptr<NAV::SatNavData> satNavData = nullptr; // Satellite Navigation data

        double satClkBias{};                        // Satellite clock bias [s]
        double satClkDrift{};                       // Satellite clock drift [s/s]
        Eigen::Vector3d e_satPos;                   // Satellite position in ECEF frame coordinates [m]
        Eigen::Vector3d e_satVel;                   // Satellite velocity in ECEF frame coordinates [m/s]
        double pseudorangeEst{ std::nan("") };      // Estimated Pseudorange [m]
        double pseudorangeRateMeas{ std::nan("") }; // Measured Pseudorange rate [m/s]

        // Data recalculated each iteration

        bool skipped = false;                                            // Whether to skip the measurement
        Eigen::Vector3d e_lineOfSightUnitVector;                         // Line-of-sight unit vector in ECEF frame coordinates
        Eigen::Vector3d n_lineOfSightUnitVector;                         // Line-of-sight unit vector in NED frame coordinates
        double satElevation = calcSatElevation(n_lineOfSightUnitVector); // Elevation [rad]
        double satAzimuth = calcSatAzimuth(n_lineOfSightUnitVector);     // Azimuth [rad]
    };

    // Data calculated for each satellite (only satellites filtered by GUI filter & NAV data available)
    std::vector<CalcData> calcData;
    std::vector<SatelliteSystem> availSatelliteSystems; // List of satellite systems
    for (const auto& obsData : gnssObs->data)
    {
        auto satId = obsData.satSigId.toSatId();

        if ((obsData.satSigId.freq & _filterFreq)                                                                     // frequency is selected in GUI
            && (obsData.code & _filterCode)                                                                           // code is selected in GUI
            && obsData.pseudorange                                                                                    // has a valid pseudorange
            && std::find(_excludedSatellites.begin(), _excludedSatellites.end(), satId) == _excludedSatellites.end()) // is not excluded
        {
            for (const auto& gnssNavInfo : gnssNavInfos)
            {
                if (auto satNavData = gnssNavInfo->searchNavigationData(satId, gnssObs->insTime)) // can calculate satellite position
                {
                    if (!satNavData->isHealthy())
                    {
                        LOG_DATA("{}: Satellite {} is skipped because the signal is not healthy.", nameId(), satId);
                        continue;
                    }
                    LOG_DATA("{}: Using observation from {} {}", nameId(), obsData.satSigId, obsData.code);
                    calcData.emplace_back(obsData, satNavData);
                    calcData.back().pseudorangeEst = obsData.pseudorange.value().value;
                    if (std::find(availSatelliteSystems.begin(), availSatelliteSystems.end(), satId.satSys) == availSatelliteSystems.end())
                    {
                        availSatelliteSystems.push_back(satId.satSys);
                    }
                    break;
                }
            }
        }
    }

    size_t nMeas = calcData.size();
    LOG_DATA("{}: nMeas {}", nameId(), nMeas);
    size_t nParam = 4 + availSatelliteSystems.size() - 1; // 3x pos, 1x clk, (N-1)x clkDiff

    // Frequency number (GLONASS only)
    int8_t freqNum = -128;

    // Find all observations providing a doppler measurement (for velocity calculation)
    size_t nDopplerMeas = 0;
    for (auto& calc : calcData)
    {
        const auto& obsData = calc.obsData;
        if (obsData.doppler)
        {
            nDopplerMeas++;
            // TODO: Find out what this is used for and find a way to use it, after the GLONASS orbit calculation is working
            if (obsData.satSigId.freq & (R01 | R02))
            {
                if (auto satNavData = std::dynamic_pointer_cast<GLONASSEphemeris>(calc.satNavData))
                {
                    freqNum = satNavData->frequencyNumber;
                }
            }

            calc.pseudorangeRateMeas = doppler2psrRate(obsData.doppler.value(), obsData.satSigId.freq, freqNum);
        }
    }

    // #####################################################################################################################################
    //                                                          Calculation
    // #####################################################################################################################################

    if (nMeas < nParam)
    {
        LOG_ERROR("{}: [{}] Cannot calculate position because only {} valid measurements ({} needed). Try changing filter settings or reposition your antenna.",
                  nameId(), (gnssObs->insTime + std::chrono::seconds(gnssObs->insTime.leapGps2UTC())), nMeas, nParam);
        sppSol->nSatellitesPosition = nMeas;
        sppSol->nSatellitesVelocity = nDopplerMeas;
        invokeCallbacks(OUTPUT_PORT_INDEX_RTKSOL, sppSol);
        return;
    }

    // Keeps track of skipped meausrements (because of elevation mask, ...)
    size_t cntSkippedMeas = 0;

    // Latitude, Longitude, Altitude of the receiver [rad, rad, m]
    Eigen::Vector3d lla_pos = trafo::ecef2lla_WGS84(_e_position);
    LOG_DATA("{}:     _e_position {}, {}, {}", nameId(), _e_position.x(), _e_position.y(), _e_position.z());
    LOG_DATA("{}:     lla_pos {}°, {}°, {}m", nameId(), rad2deg(lla_pos.x()), rad2deg(lla_pos.y()), lla_pos.z());
    LOG_DATA("{}:     _recvClk.bias {}", nameId(), _recvClk.bias.value);
    LOG_DATA("{}:     _recvClk.drift {}", nameId(), _recvClk.drift.value);

    std::vector<SatelliteSystem> satelliteSystems = availSatelliteSystems; // List of satellite systems

    SatelliteSystem_ usedSatelliteSystems = SatSys_None;
    for (size_t i = 0; i < nMeas; i++)
    {
        const auto& obsData = calcData[i].obsData;
        LOG_DATA("{}:     satellite {}", nameId(), obsData.satSigId);
        auto satId = obsData.satSigId.toSatId();

        // Calculate satellite clock, position and velocity
        LOG_DATA("{}:         pseudorangeEst {}", nameId(), calcData[i].pseudorangeEst);

        auto satClk = calcData[i].satNavData->calcClockCorrections(gnssObs->insTime, calcData[i].pseudorangeEst, obsData.satSigId.freq);
        calcData[i].satClkBias = satClk.bias;
        calcData[i].satClkDrift = satClk.drift;
        LOG_DATA("{}:         satClkBias {}, satClkDrift {}", nameId(), calcData[i].satClkBias, calcData[i].satClkDrift);

        auto satPosVel = calcData[i].satNavData->calcSatellitePosVel(satClk.transmitTime);
        calcData[i].e_satPos = satPosVel.e_pos;
        calcData[i].e_satVel = satPosVel.e_vel;
        LOG_DATA("{}:         e_satPos {}", nameId(), calcData[i].e_satPos.transpose());
        LOG_DATA("{}:         e_satVel {}", nameId(), calcData[i].e_satVel.transpose());

        // Line-of-sight unit vector in ECEF frame coordinates - Groves ch. 8.5.3, eq. 8.41, p. 341
        calcData[i].e_lineOfSightUnitVector = e_calcLineOfSightUnitVector(_e_position, calcData[i].e_satPos);
        LOG_DATA("{}:         e_lineOfSightUnitVector {}", nameId(), calcData[i].e_lineOfSightUnitVector.transpose());
        // Line-of-sight unit vector in NED frame coordinates - Groves ch. 8.5.3, eq. 8.41, p. 341
        calcData[i].n_lineOfSightUnitVector = trafo::n_Quat_e(lla_pos(0), lla_pos(1)) * calcData[i].e_lineOfSightUnitVector;
        LOG_DATA("{}:         n_lineOfSightUnitVector {}", nameId(), calcData[i].n_lineOfSightUnitVector.transpose());
        // Elevation [rad] - Groves ch. 8.5.4, eq. 8.57, p. 344
        calcData[i].satElevation = calcSatElevation(calcData[i].n_lineOfSightUnitVector);
        LOG_DATA("{}:         satElevation {}°", nameId(), rad2deg(calcData[i].satElevation));
        // Azimuth [rad] - Groves ch. 8.5.4, eq. 8.57, p. 344
        calcData[i].satAzimuth = calcSatAzimuth(calcData[i].n_lineOfSightUnitVector);
        LOG_DATA("{}:         satAzimuth {}°", nameId(), rad2deg(calcData[i].satAzimuth));

        if (!_e_position.isZero() && calcData[i].satElevation < _elevationMask) // Do not check elevation mask when not having a valid position
        {
            cntSkippedMeas++;
            calcData[i].skipped = true;
            LOG_DATA("{}:         [{}] Measurement is skipped because of elevation {:.1f}° and mask of {}° ({} valid measurements remaining)",
                     nameId(), obsData.satSigId, rad2deg(calcData[i].satElevation), rad2deg(_elevationMask), nMeas - cntSkippedMeas);

            if (!(usedSatelliteSystems & satId.satSys)
                && calcData.begin() + static_cast<int64_t>(i + 1) != calcData.end()                                         // This is the last satellite and the system did not appear before
                && std::none_of(calcData.begin() + static_cast<int64_t>(i + 1), calcData.end(), [&](const CalcData& data) { // The satellite system has no satellites available anymore
                       return data.obsData.satSigId.toSatId().satSys == satId.satSys;
                   }))
            {
                LOG_DEBUG("{}: The satellite system {} won't be used this iteration because no satellite complies with the elevation mask.",
                          nameId(), satId.satSys);
                nParam--;
                satelliteSystems.erase(std::find(satelliteSystems.begin(), satelliteSystems.end(), satId.satSys));
            }

            if (nMeas - cntSkippedMeas < nParam)
            {
                LOG_ERROR("{}: [{}] Cannot calculate position because only {} valid measurements ({} needed). Try changing filter settings or reposition your antenna.",
                          nameId(), gnssObs->insTime, nMeas - cntSkippedMeas, nParam);
                sppSol->nSatellitesPosition = nMeas - cntSkippedMeas;
                sppSol->nSatellitesVelocity = nDopplerMeas - cntSkippedMeas;
                invokeCallbacks(OUTPUT_PORT_INDEX_RTKSOL, sppSol);
                return;
            }
            continue;
        }

        usedSatelliteSystems |= satId.satSys;
    }

    _recvClk.referenceTimeSatelliteSystem = satelliteSystems.front();
    for (const auto& availSatSys : satelliteSystems)
    {
        if (SatelliteSystem_(availSatSys) < SatelliteSystem_(_recvClk.referenceTimeSatelliteSystem))
        {
            _recvClk.referenceTimeSatelliteSystem = availSatSys;
        }
    }
    satelliteSystems.erase(std::find(satelliteSystems.begin(), satelliteSystems.end(), _recvClk.referenceTimeSatelliteSystem));
    LOG_DATA("{}:     _recvClk.referenceTimeSatelliteSystem {} ({} other time systems)", nameId(), _recvClk.referenceTimeSatelliteSystem, satelliteSystems.size());

    invokeCallbacks(OUTPUT_PORT_INDEX_RTKSOL, sppSol);
}