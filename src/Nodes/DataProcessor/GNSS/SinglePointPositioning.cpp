// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "SinglePointPositioning.hpp"

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

#include "NodeData/GNSS/GnssObs.hpp"
#include "NodeData/GNSS/GnssNavInfo.hpp"
#include "NodeData/GNSS/SppSolution.hpp"

#include "Navigation/GNSS/Functions.hpp"
#include "Navigation/GNSS/Satellite/Ephemeris/GLONASSEphemeris.hpp"
#include "Navigation/Math/LeastSquares.hpp"

NAV::SinglePointPositioning::SinglePointPositioning()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 407, 506 };

    nm::CreateInputPin(this, NAV::GnssObs::type().c_str(), Pin::Type::Flow, { NAV::GnssObs::type() }, &SinglePointPositioning::recvGnssObs);
    updateNumberOfInputPins();

    nm::CreateOutputPin(this, NAV::SppSolution::type().c_str(), Pin::Type::Flow, { NAV::SppSolution::type() });
}

NAV::SinglePointPositioning::~SinglePointPositioning()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::SinglePointPositioning::typeStatic()
{
    return "SinglePointPositioning - SPP";
}

std::string NAV::SinglePointPositioning::type() const
{
    return typeStatic();
}

std::string NAV::SinglePointPositioning::category()
{
    return "Data Processor";
}

void NAV::SinglePointPositioning::guiConfig()
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

            if (inputPins.size() > 1)
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

    ImGui::SetNextItemWidth(itemWidth);
    if (ImGui::Checkbox(fmt::format("Weighted LSE (position)##{}", size_t(id)).c_str(), &_useWeightedLeastSquares))
    {
        LOG_DEBUG("{}: Use weighted least squares changed to {}°", nameId(), _useWeightedLeastSquares);
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

[[nodiscard]] json NAV::SinglePointPositioning::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["nNavInfoPins"] = _nNavInfoPins;
    j["frequencies"] = Frequency_(_filterFreq);
    j["codes"] = _filterCode;
    j["excludedSatellites"] = _excludedSatellites;
    j["elevationMask"] = rad2deg(_elevationMask);
    j["useWeightedLeastSquares"] = _useWeightedLeastSquares;
    j["ionosphereModel"] = _ionosphereModel;
    j["troposphereModels"] = _troposphereModels;

    return j;
}

void NAV::SinglePointPositioning::restore(json const& j)
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
    if (j.contains("useWeightedLeastSquares"))
    {
        j.at("useWeightedLeastSquares").get_to(_useWeightedLeastSquares);
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

bool NAV::SinglePointPositioning::initialize()
{
    LOG_TRACE("{}: called", nameId());

    if (std::all_of(inputPins.begin() + INPUT_PORT_INDEX_GNSS_NAV_INFO, inputPins.end(), [](const InputPin& inputPin) { return !inputPin.isPinLinked(); }))
    {
        LOG_ERROR("{}: You need to connect a GNSS NavigationInfo provider", nameId());
        return false;
    }

    _e_position = Eigen::Vector3d::Zero();
    _clkBias = 0.0;
    _e_velocity = Eigen::Vector3d::Zero();
    _clkDrift = 0.0;

    LOG_DEBUG("SinglePointPositioning initialized");

    return true;
}

void NAV::SinglePointPositioning::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

void NAV::SinglePointPositioning::updateNumberOfInputPins()
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

void NAV::SinglePointPositioning::recvGnssObs(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    std::vector<const GnssNavInfo*> gnssNavInfos(_nNavInfoPins);
    for (size_t i = 0; i < _nNavInfoPins; i++)
    {
        gnssNavInfos[i] = getInputValue<const GnssNavInfo>(INPUT_PORT_INDEX_GNSS_NAV_INFO + i);
    }

    if (std::all_of(gnssNavInfos.begin(), gnssNavInfos.end(), [](const GnssNavInfo* info) { return info == nullptr; }))
    {
        return;
    }

    // Collection of all connected Ionospheric Corrections
    IonosphericCorrections ionosphericCorrections;
    for (const auto* gnssNavInfo : gnssNavInfos)
    {
        if (gnssNavInfo)
        {
            for (const auto& correction : gnssNavInfo->ionosphericCorrections.data())
            {
                if (!ionosphericCorrections.contains(correction.satSys, correction.alphaBeta))
                {
                    ionosphericCorrections.insert(correction.satSys, correction.alphaBeta, correction.data);
                }
            }
        }
    }

    auto gnssObs = std::static_pointer_cast<const GnssObs>(queue.extract_front());
    LOG_DATA("{}: Calculating SPP for {}", nameId(), gnssObs->insTime.toYMDHMS());

    auto sppSol
#ifdef TESTING
        = std::make_shared<SppSolutionExtended>();
#else
        = std::make_shared<SppSolution>();
#endif
    sppSol->insTime = gnssObs->insTime;

    // Data calculated for each observation
    struct CalcData
    {
        // Constructor
        explicit CalcData(size_t obsIdx, size_t navIdx) : obsIdx(obsIdx), navIdx(navIdx) {}

        size_t obsIdx = 0;                      // Index in the provided GNSS Observation data
        size_t navIdx = 0;                      // Index in the provided GNSS Navigation data
        double satClkBias{};                    ///< Satellite clock bias [s]
        double satClkDrift{};                   ///< Satellite clock drift [s/s]
        Eigen::Vector3d e_satPos;               // Satellite position in ECEF frame coordinates [m]
        Eigen::Vector3d e_satVel;               // Satellite velocity in ECEF frame coordinates [m/s]
        double pseudorangeRate{ std::nan("") }; // Pseudorange rate [m/s]
    };

    // Data calculated for each satellite (only satellites filtered by GUI filter & NAV data available)
    std::vector<CalcData> calcData;
    for (size_t obsIdx = 0; obsIdx < gnssObs->data.size(); obsIdx++)
    {
        const auto& obsData = gnssObs->data[obsIdx];
        auto satId = obsData.satSigId.toSatId();

        if ((obsData.satSigId.freq & _filterFreq)                                                                     // frequency is selected in GUI
            && (obsData.code & _filterCode)                                                                           // code is selected in GUI
            && obsData.pseudorange                                                                                    // has a valid pseudorange
            && std::find(_excludedSatellites.begin(), _excludedSatellites.end(), satId) == _excludedSatellites.end()) // is not excluded
        {
            for (size_t navIdx = 0; navIdx < gnssNavInfos.size(); navIdx++)
            {
                if (gnssNavInfos[navIdx]->contains(satId)) // can calculate satellite position
                {
                    if (!gnssNavInfos[navIdx]->isHealthy(satId, gnssObs->insTime))
                    {
                        LOG_DATA("{}: Satellite {} is skipped because the signal is not healthy.", nameId(), satId);

#ifdef TESTING
                        auto& sppExtendedData = (*sppSol)(obsData.satSigId.freq, obsData.satSigId.satNum, obsData.code);
                        sppExtendedData.skipped = true;
#endif
                        continue;
                    }
                    LOG_DATA("{}: Using observation from {} {}", nameId(), obsData.satSigId, obsData.code);
                    calcData.emplace_back(obsIdx, navIdx);
                    break;
                }
            }
        }
    }

    for (size_t i = 0; i < calcData.size(); i++) // Remove less precise codes (e.g. if G1X (L1C combined) is present, don't use G1L (L1C pilot) and G1S (L1C data))
    {
        const auto& obsData = gnssObs->data[calcData.at(i).obsIdx];
        LOG_DATA("Code[{}] {}-{}, obsIdx {}", i, obsData.satSigId, obsData.code, calcData.at(i).obsIdx);

        auto eraseLessPreciseCodes = [&calcData, &i, &gnssObs, &obsData](const Code& third, const Code& second, const Code& prime) {
            auto eraseSatDataWithCode = [&calcData, &i, &gnssObs, &obsData](const Code& code) {
                LOG_DATA("    Searching for {}-{}", obsData.satSigId, code);
                auto iter = std::find_if(calcData.begin(), calcData.end(), [&gnssObs, &code, &obsData](const CalcData& sData) {
                    return obsData.satSigId == gnssObs->data[sData.obsIdx].satSigId && gnssObs->data[sData.obsIdx].code == code;
                });
                if (iter != calcData.end())
                {
                    if (iter < calcData.begin() + static_cast<int64_t>(i)) { i--; } // NOLINT(hicpp-use-nullptr,modernize-use-nullptr)
                    LOG_DATA("    Erasing {}-{}", obsData.satSigId, code);
                    calcData.erase(iter);
                }
            };
            if (obsData.code == prime)
            {
                eraseSatDataWithCode(second);
                eraseSatDataWithCode(third);
            }
            if (obsData.code == second)
            {
                eraseSatDataWithCode(third);
            }
        };

        eraseLessPreciseCodes(Code::G1S, Code::G1L, Code::G1X); ///< L1C (data, pilot, combined)
        eraseLessPreciseCodes(Code::G2S, Code::G2L, Code::G2X); ///< L2C-code (medium, long, combined)
        eraseLessPreciseCodes(Code::G5I, Code::G5Q, Code::G5X); ///< L5 (data, pilot, combined)

        eraseLessPreciseCodes(Code::E1B, Code::E1C, Code::E1X); ///< OS (data, pilot, combined)
        eraseLessPreciseCodes(Code::E5I, Code::E5Q, Code::E5X); ///< E5a (data, pilot, combined)
        eraseLessPreciseCodes(Code::E6B, Code::E6C, Code::E6X); ///< E6 (data, pilot, combined)
        eraseLessPreciseCodes(Code::E7I, Code::E7Q, Code::E7X); ///< E5b (data, pilot, combined)
        eraseLessPreciseCodes(Code::E8I, Code::E8Q, Code::E8X); ///< E5 AltBOC (data, pilot, combined)

        eraseLessPreciseCodes(Code::R3I, Code::R3Q, Code::R3X); ///< L3 (data, pilot, combined)
        eraseLessPreciseCodes(Code::R4A, Code::R4B, Code::R4X); ///< G1a (data, pilot, combined)
        eraseLessPreciseCodes(Code::R6A, Code::R6B, Code::R6X); ///< G2a (data, pilot, combined)

        eraseLessPreciseCodes(Code::B1D, Code::B1P, Code::B1X); ///< B1 (data, pilot, combined)
        eraseLessPreciseCodes(Code::B2I, Code::B2Q, Code::B2X); ///< B1I(OS), B1Q, combined
        eraseLessPreciseCodes(Code::B5D, Code::B5P, Code::B5X); ///< B2a (data, pilot, combined)
        eraseLessPreciseCodes(Code::B6I, Code::B6Q, Code::B6X); ///< B3I, B3Q, combined
        eraseLessPreciseCodes(Code::B7I, Code::B7Q, Code::B7X); ///< B2I(OS), B2Q, combined
        eraseLessPreciseCodes(Code::B7D, Code::B7P, Code::B7Z); ///< B2b (data, pilot, combined)
        eraseLessPreciseCodes(Code::B8D, Code::B8P, Code::B8X); ///< B2 (B2a+B2b) (data, pilot, combined)

        eraseLessPreciseCodes(Code::J1S, Code::J1L, Code::J1X); ///< L1C (data, pilot, combined)
        eraseLessPreciseCodes(Code::J2S, Code::J2L, Code::J2X); ///< L2C-code (medium, long, combined)
        eraseLessPreciseCodes(Code::J5I, Code::J5Q, Code::J5X); ///< L5 (data, pilot, combined)
        eraseLessPreciseCodes(Code::J5D, Code::J5P, Code::J5Z); ///< L5 (data, pilot, combined)
        eraseLessPreciseCodes(Code::J6S, Code::J6L, Code::J6X); ///< LEX signal (short, long, combined)

        eraseLessPreciseCodes(Code::I5B, Code::I5C, Code::I5X); ///< RS (data, pilot, combined)
        eraseLessPreciseCodes(Code::I9B, Code::I9C, Code::I9X); ///< RS (data, pilot, combined)

        eraseLessPreciseCodes(Code::S5I, Code::S5Q, Code::S5X); ///< L5 (data, pilot, combined)
    }

    size_t nMeas = calcData.size();

    // Find all observations providing a doppler measurement (for velocity calculation)
    size_t nDopplerMeas = 0;
    for (size_t i = 0; i < nMeas; i++)
    {
        const auto& obsData = gnssObs->data[calcData[i].obsIdx];
        if (obsData.doppler)
        {
            nDopplerMeas++;
            int8_t num = -128;
            // TODO: Find out what this is used for and find a way to use it, after the GLONASS orbit calculation is working
            if (obsData.satSigId.freq & (R01 | R02))
            {
                if (auto satNavData = std::dynamic_pointer_cast<GLONASSEphemeris>(
                        gnssNavInfos[calcData[i].navIdx]->satellites().at({ GLO, obsData.satSigId.satNum }).searchNavigationData(gnssObs->insTime)))
                {
                    num = satNavData->frequencyNumber;
                }
            }

            calcData[i].pseudorangeRate = doppler2psrRate(obsData.doppler.value(), obsData.satSigId.freq, num);
        }
    }

    // #####################################################################################################################################
    //                                                        Position calculation
    // #####################################################################################################################################

    // Measurement/Geometry matrix for the pseudorange
    Eigen::MatrixXd e_H_psr = Eigen::MatrixXd(static_cast<int>(nMeas), 4);
    // Corrected pseudorange estimates [m]
    Eigen::VectorXd psrEst_c = Eigen::VectorXd(static_cast<int>(nMeas));
    // Corrected pseudorange measurements [m]
    Eigen::VectorXd psrMeas_c = Eigen::VectorXd(static_cast<int>(nMeas));
    // Pseudorange measurement error weight matrix
    Eigen::MatrixXd W_psr = Eigen::MatrixXd::Zero(static_cast<int>(nMeas), static_cast<int>(nMeas));

    // Measurement/Geometry matrix for the pseudorange-rate
    Eigen::MatrixXd e_H_r = Eigen::MatrixXd(static_cast<int>(nDopplerMeas), 4);
    // Corrected pseudorange-rate estimates [m/s]
    Eigen::VectorXd psrRateEst_c = Eigen::VectorXd(static_cast<int>(nDopplerMeas));
    // Corrected pseudorange-rate measurements [m/s]
    Eigen::VectorXd psrRateMeas_c = Eigen::VectorXd(static_cast<int>(nDopplerMeas));
    // Pseudorange rate (doppler) measurement error weight matrix
    Eigen::MatrixXd W_psrRate = Eigen::MatrixXd::Zero(static_cast<int>(nDopplerMeas), static_cast<int>(nDopplerMeas));

    for (size_t i = 0; i < nMeas; i++) // Calculate satellite clock, position and velocity
    {
        const auto& obsData = gnssObs->data[calcData[i].obsIdx];

        auto satId = obsData.satSigId.toSatId();
        const auto* navInfo = gnssNavInfos[calcData[i].navIdx];

        LOG_DATA("{}: satellite {} {} [{}]", nameId(), obsData.satSigId.freq, obsData.satSigId.satNum, gnssObs->insTime);
        LOG_DATA("{}:     pseudorange  {}", nameId(), obsData.pseudorange.value().value);

        auto satClk = navInfo->calcSatelliteClockCorrections(obsData.satSigId.toSatId(), gnssObs->insTime, obsData.pseudorange.value().value, obsData.satSigId.freq);
        calcData[i].satClkBias = satClk.bias;
        calcData[i].satClkDrift = satClk.drift;
        LOG_DATA("{}:     satClkBias {}, satClkDrift {}", nameId(), calcData[i].satClkBias, calcData[i].satClkDrift);

        auto satPosVel = navInfo->calcSatellitePosVel(satId, satClk.transmitTime);
        calcData[i].e_satPos = satPosVel.e_pos;
        calcData[i].e_satVel = satPosVel.e_vel;
        LOG_DATA("{}:     e_satPos {}", nameId(), calcData[i].e_satPos.transpose());
        LOG_DATA("{}:     e_satVel {}", nameId(), calcData[i].e_satVel.transpose());

#ifdef TESTING
        auto& sppExtendedData = (*sppSol)(obsData.satSigId.freq, obsData.satSigId.satNum, obsData.code);
        sppExtendedData.transmitTime = satClk.transmitTime;
        sppExtendedData.satClkBias = satClk.bias;
        sppExtendedData.satClkDrift = satClk.drift;
        sppExtendedData.e_satPos = calcData[i].e_satPos;
        sppExtendedData.e_satVel = calcData[i].e_satVel;
#endif
    }

    LOG_DATA("{}: nMeas {}", nameId(), nMeas);

    for (size_t o = 0; o < 10; o++)
    {
        // Keeps track of skipped meausrements (because of elevation mask, ...)
        size_t cntSkippedMeas = 0;

        LOG_DATA("{}: Iteration {}", nameId(), o);
        // Latitude, Longitude, Altitude of the receiver [rad, rad, m]
        Eigen::Vector3d lla_pos = trafo::ecef2lla_WGS84(_e_position);
        LOG_TRACE("{}:     [{}] _e_position {}, {}, {}", nameId(), o, _e_position.x(), _e_position.y(), _e_position.z());
        LOG_TRACE("{}:     [{}] lla_pos {}°, {}°, {}m", nameId(), o, rad2deg(lla_pos.x()), rad2deg(lla_pos.y()), lla_pos.z());
        LOG_DATA("{}:     [{}] _clkBias {}", nameId(), o, _clkBias);
        LOG_DATA("{}:     [{}] _clkDrift {}", nameId(), o, _clkDrift);

        size_t ix = 0;
        size_t iv = 0;
        for (size_t i = 0; i < nMeas; i++)
        {
            const auto& obsData = gnssObs->data[calcData[i].obsIdx];
            LOG_DATA("{}:     [{}] satellite {} {} [{}]", nameId(), o, obsData.satSigId.freq, obsData.satSigId.satNum, gnssObs->insTime.toGPSweekTow());

            auto satId = obsData.satSigId.toSatId();

            // Line-of-sight unit vector in ECEF frame coordinates - Groves ch. 8.5.3, eq. 8.41, p. 341
            Eigen::Vector3d e_lineOfSightUnitVector = e_calcLineOfSightUnitVector(_e_position, calcData[i].e_satPos);
            LOG_DATA("{}:     [{}]     e_lineOfSightUnitVector {}", nameId(), o, e_lineOfSightUnitVector.transpose());
            // Line-of-sight unit vector in NED frame coordinates - Groves ch. 8.5.3, eq. 8.41, p. 341
            Eigen::Vector3d n_lineOfSightUnitVector = trafo::n_Quat_e(lla_pos(0), lla_pos(1)) * e_lineOfSightUnitVector;
            LOG_DATA("{}:     [{}]     n_lineOfSightUnitVector {}", nameId(), o, n_lineOfSightUnitVector.transpose());
            // Elevation [rad] - Groves ch. 8.5.4, eq. 8.57, p. 344
            double satElevation = calcSatElevation(n_lineOfSightUnitVector);
            LOG_DATA("{}:     [{}]     satElevation {}°", nameId(), o, rad2deg(satElevation));
            // Azimuth [rad] - Groves ch. 8.5.4, eq. 8.57, p. 344
            double satAzimuth = calcSatAzimuth(n_lineOfSightUnitVector);
            LOG_DATA("{}:     [{}]     satAzimuth {}°", nameId(), o, rad2deg(satAzimuth));

#ifdef TESTING
            auto& sppExtendedData = (*sppSol)(obsData.satSigId.freq, obsData.satSigId.satNum, obsData.code);
            sppExtendedData.satElevation = satElevation;
            sppExtendedData.satAzimuth = satAzimuth;
#endif

            if (satElevation < _elevationMask)
            {
                cntSkippedMeas++;
                LOG_DATA("{}:     [{}]     Measurement is skipped because of elevation mask of {}° ({} valid measurements remaining)",
                         nameId(), o, rad2deg(_elevationMask), nMeas - cntSkippedMeas);

#ifdef TESTING
                sppExtendedData.elevationMaskTriggered = true;
#endif

                if (nMeas - cntSkippedMeas < 4)
                {
                    LOG_ERROR("{}: [{}] Cannot calculate position because only {} valid measurements. Try changing filter settings or reposition your antenna.",
                              nameId(), (gnssObs->insTime + std::chrono::seconds(gnssObs->insTime.leapGps2UTC())), nMeas - cntSkippedMeas);
                    sppSol->nSatellitesPosition = nMeas - cntSkippedMeas;
                    sppSol->nSatellitesVelocity = nDopplerMeas - cntSkippedMeas;
                    invokeCallbacks(OUTPUT_PORT_INDEX_SPPSOL, sppSol);
                    return;
                }
                continue;
            }
#ifdef TESTING
            sppExtendedData.elevationMaskTriggered = false;
#endif

            // #############################################################################################################################
            //                                                    Position calculation
            // #############################################################################################################################

            // Pseudorange measurement [m] - Groves ch. 8.5.3, eq. 8.48, p. 342
            double psrMeas = obsData.pseudorange.value().value /* + (multipath and/or NLOS errors) + (tracking errors) */;
            // Estimated modulation ionosphere propagation error [m]
            double dpsr_I = calcIonosphericTimeDelay(static_cast<double>(gnssObs->insTime.toGPSweekTow().tow), obsData.satSigId.freq, lla_pos,
                                                     satElevation, satAzimuth, _ionosphereModel, &ionosphericCorrections)
                            * InsConst::C;
            LOG_DATA("{}:     [{}]     dpsr_I {} [m] (Estimated modulation ionosphere propagation error)", nameId(), o, dpsr_I);

            auto tropo = calcTroposphericDelayAndMapping(gnssObs->insTime, lla_pos, satElevation, satAzimuth, _troposphereModels);
            LOG_DATA("{}:     [{}]     ZHD {}", nameId(), o, tropo.ZHD);
            LOG_DATA("{}:     [{}]     ZWD {}", nameId(), o, tropo.ZWD);
            LOG_DATA("{}:     [{}]     zhdMappingFactor {}", nameId(), o, tropo.zhdMappingFactor);
            LOG_DATA("{}:     [{}]     zwdMappingFactor {}", nameId(), o, tropo.zwdMappingFactor);

            // Estimated modulation troposphere propagation error [m]
            double dpsr_T = tropo.ZHD * tropo.zhdMappingFactor + tropo.ZWD * tropo.zwdMappingFactor;
            LOG_DATA("{}:     [{}]     dpsr_T {} [m] (Estimated modulation troposphere propagation error)", nameId(), o, dpsr_T);

            // Corrected pseudorange measurements [m] - Groves ch. 8.5.3, eq. 8.49, p. 342
            psrMeas_c(static_cast<int>(ix)) = psrMeas - dpsr_I - dpsr_T + calcData[i].satClkBias * InsConst::C;
            LOG_DATA("{}:     [{}]     psrMeas_c({}) {}", nameId(), o, ix, psrMeas_c(static_cast<int>(ix)));

            // Measurement/Geometry matrix - Groves ch. 9.4.1, eq. 9.144, p. 412
            e_H_psr.block<1, 3>(static_cast<int>(ix), 0) = -e_lineOfSightUnitVector;
            e_H_psr(static_cast<int>(ix), 3) = 1;
            LOG_DATA("{}:     [{}]     e_H_psr.block<1, 4>({},0) {}", nameId(), o, ix, e_H_psr.block<1, 4>(static_cast<int>(ix), 0));

            // Sagnac correction - Springer Handbook ch. 19.1.1, eq. 19.7, p. 562
            double dpsr_ie = 1.0 / InsConst::C * (_e_position - calcData[i].e_satPos).dot(InsConst::e_omega_ie.cross(_e_position));
            LOG_DATA("{}:     [{}]     dpsr_ie {}", nameId(), o, dpsr_ie);
            // Geometric distance [m]
            double geometricDist = (calcData[i].e_satPos - _e_position).norm();
            LOG_DATA("{}:     [{}]     geometricDist {}", nameId(), o, geometricDist);
            // Corrected pseudorange estimate [m] - Groves ch. 9.4.1, eq. 9.142, p. 412
            psrEst_c(static_cast<int>(ix)) = geometricDist + _clkBias * InsConst::C + dpsr_ie;
            LOG_DATA("{}:     [{}]     psrEst_c({}) {}", nameId(), o, ix, psrEst_c(static_cast<int>(ix)));

            if (_useWeightedLeastSquares)
            {
                // Weight matrix - RTKLIB eq. E6.23, p. 158

                constexpr double ERR_BRDCI = 0.5;  // Broadcast iono model error factor (See GPS ICD ch. 20.3.3.5.2.5, p. 130: 50% reduction on RMS error)
                constexpr double ERR_SAAS = 0.3;   // Saastamoinen model error std [m] (maximum zenith wet delay - formulas with worst possible values)
                constexpr double ERR_CBIAS = 0.3;  // Code bias error Std (m)
                constexpr double EFACT_GPS = 1.0;  // Satellite system error factor GPS/GAL/QZS/BeiDou
                constexpr double EFACT_GLO = 1.5;  // Satellite system error factor GLONASS/IRNSS
                constexpr double EFACT_SBAS = 3.0; // Satellite system error factor SBAS

                double satSysErrFactor = satId.satSys == GLO || satId.satSys == IRNSS
                                             ? EFACT_GLO
                                             : (satId.satSys == SBAS
                                                    ? EFACT_SBAS
                                                    : EFACT_GPS);
                double ele = std::max(satElevation, deg2rad(5));

                // Code/Carrier-Phase Error Ratio - Measurement error standard deviation
                std::unordered_map<Frequency, double> codeCarrierPhaseErrorRatio = { { G01, 300.0 },
                                                                                     { G02, 300.0 },
                                                                                     { G05, 300.0 } };
                double carrierPhaseErrorA = 0.003; // Carrier-Phase Error Factor a [m] - Measurement error standard deviation
                double carrierPhaseErrorB = 0.003; // Carrier-Phase Error Factor b [m] - Measurement error standard deviation

                double varPsrMeas = std::pow(satSysErrFactor, 2) * std::pow(codeCarrierPhaseErrorRatio.at(G01), 2)
                                    * (std::pow(carrierPhaseErrorA, 2) + std::pow(carrierPhaseErrorB, 2) / std::sin(ele));
                LOG_DATA("{}:     [{}]     varPsrMeas {}", nameId(), o, varPsrMeas);

                double varEph = gnssNavInfos[calcData[i].navIdx]->calcSatellitePositionVariance(satId, gnssObs->insTime);
                LOG_DATA("{}:     [{}]     varEph {}", nameId(), o, varEph);
                double varIono = ratioFreqSquared(G01, obsData.satSigId.freq) * std::pow(dpsr_I * ERR_BRDCI, 2);
                LOG_DATA("{}:     [{}]     varIono {}", nameId(), o, varIono);
                double varTrop = dpsr_T == 0.0 ? 0.0 : std::pow(ERR_SAAS / (std::sin(satElevation) + 0.1), 2);
                LOG_DATA("{}:     [{}]     varTrop {}", nameId(), o, varTrop);
                double varBias = std::pow(ERR_CBIAS, 2);
                LOG_DATA("{}:     [{}]     varBias {}", nameId(), o, varBias);

                double varErrors = varPsrMeas + varEph + varIono + varTrop + varBias;
                LOG_DATA("{}:     [{}]     varErrors {}", nameId(), o, varErrors);

                W_psr(static_cast<int>(ix), static_cast<int>(ix)) = 1.0 / varErrors;
                LOG_DATA("{}:     [{}]     W_psr({},{}) {}", nameId(), o, ix, ix, W_psr(static_cast<int>(ix), static_cast<int>(ix)));
            }

            LOG_DATA("{}:     [{}]     dpsr({}) {}", nameId(), o, ix, psrMeas_c(static_cast<int>(ix)) - psrEst_c(static_cast<int>(ix)));

            // #############################################################################################################################
            //                                                    Velocity calculation
            // #############################################################################################################################

            if (nDopplerMeas >= 4 && !std::isnan(calcData[i].pseudorangeRate))
            {
                // Measurement/Geometry matrix - Groves ch. 9.4.1, eq. 9.144, p. 412
                e_H_r.row(static_cast<int>(iv)) = e_H_psr.row(static_cast<int>(ix));

                // Pseudorange-rate measurement [m/s] - Groves ch. 8.5.3, eq. 8.48, p. 342
                double psrRateMeas = calcData[i].pseudorangeRate /* + (multipath and/or NLOS errors) + (tracking errors) */;
                // Corrected pseudorange-rate measurements [m/s] - Groves ch. 8.5.3, eq. 8.49, p. 342
                psrRateMeas_c(static_cast<int>(iv)) = psrRateMeas + calcData[i].satClkDrift * InsConst::C;
                LOG_DATA("{}:     [{}]     psrRateMeas_c({}) {}", nameId(), o, iv, psrRateMeas_c(static_cast<int>(iv)));

                // Range-rate Sagnac correction - Groves ch. 8.5.3, eq. 8.46, p. 342
                double dpsr_dot_ie = InsConst::omega_ie / InsConst::C
                                     * (calcData[i].e_satVel.y() * _e_position.x() + calcData[i].e_satPos.y() * _e_velocity.x()
                                        - calcData[i].e_satVel.x() * _e_position.y() - calcData[i].e_satPos.x() * _e_velocity.y());
                LOG_DATA("{}:     [{}]     dpsr_dot_ie {}", nameId(), o, dpsr_dot_ie);
                // Corrected pseudorange-rate estimate [m/s] - Groves ch. 9.4.1, eq. 9.142, p. 412 (Sagnac correction different sign)
                psrRateEst_c(static_cast<int>(iv)) = e_lineOfSightUnitVector.transpose() * (calcData[i].e_satVel - _e_velocity) + _clkDrift * InsConst::C - dpsr_dot_ie;
                LOG_DATA("{}:     [{}]     psrRateEst_c({}) {}", nameId(), o, iv, psrRateEst_c(static_cast<int>(iv)));

                if (_useWeightedLeastSquares)
                {
                    // Weight matrix

                    double dopplerFrequency = 1; // Doppler Frequency error factor [Hz] - Measurement error standard deviation

                    double varDopMeas = std::pow(dopplerFrequency, 2);
                    LOG_DATA("{}:     [{}]     varDopMeas {}", nameId(), o, varDopMeas);

                    double varEph = gnssNavInfos[calcData[i].navIdx]->calcSatellitePositionVariance(satId, gnssObs->insTime);
                    LOG_DATA("{}:     [{}]     varEph {}", nameId(), o, varEph);

                    double varErrors = varDopMeas + varEph;
                    LOG_DATA("{}:     [{}]     varErrors {}", nameId(), o, varErrors);

                    W_psrRate(static_cast<int>(iv), static_cast<int>(iv)) = 1.0 / varErrors;
                    LOG_DATA("{}:     [{}]     W_psrRate({},{}) {}", nameId(), o, iv, iv, W_psrRate(static_cast<int>(iv), static_cast<int>(iv)));
                }

                iv++;
            }
#ifdef TESTING
            sppExtendedData.pseudorangeRate = calcData[i].pseudorangeRate;
            sppExtendedData.dpsr_I = dpsr_I;
            sppExtendedData.dpsr_T = dpsr_T;
            sppExtendedData.geometricDist = geometricDist;
#endif

            ix++;
        }
        LOG_DATA("{}:     [{}] e_H_psr \n{}", nameId(), o, e_H_psr.topRows(ix));
        if (_useWeightedLeastSquares)
        {
            LOG_DATA("{}:     [{}] W_psr \n{}", nameId(), o, W_psr.topLeftCorner(ix, ix));
        }
        LOG_DATA("{}:     [{}] psrMeas_c {}", nameId(), o, psrMeas_c.topRows(ix).transpose());
        LOG_DATA("{}:     [{}] psrEst_c {}", nameId(), o, psrEst_c.topRows(ix).transpose());

        if (nDopplerMeas >= 4)
        {
            LOG_DATA("{}:     [{}] e_H_r \n{}", nameId(), o, e_H_r.topRows(iv));
            if (_useWeightedLeastSquares)
            {
                LOG_DATA("{}:     [{}] W_psrRate \n{}", nameId(), o, W_psrRate.topLeftCorner(iv, iv));
            }
            LOG_DATA("{}:     [{}] psrRateMeas_c {}", nameId(), o, psrRateMeas_c.topRows(iv).transpose());
            LOG_DATA("{}:     [{}] psrRateEst_c {}", nameId(), o, psrRateEst_c.topRows(iv).transpose());
        }

        // #################################################################################################################################
        //                                                     Least squares solution
        // #################################################################################################################################

        // Difference between measured and estimated pseudorange
        Eigen::VectorXd dpsr = psrMeas_c.topRows(ix) - psrEst_c.topRows(ix);
        LOG_DATA("{}:     [{}] dpsr {}", nameId(), o, dpsr.transpose());

        LeastSquaresResult<Eigen::VectorXd, Eigen::MatrixXd> lsq;
        if (_useWeightedLeastSquares)
        {
            // [x, y, z, clkBias] - Groves ch. 9.4.1, eq. 9.141, p. 412
            lsq = solveWeightedLinearLeastSquaresUncertainties(e_H_psr.topRows(ix), W_psr.topLeftCorner(ix, ix), dpsr);
            LOG_DATA("{}:     [{}] dx (wlsq) {}, {}, {}, {}", nameId(), o, lsq.solution(0), lsq.solution(1), lsq.solution(2), lsq.solution(3));
            LOG_DATA("{}:     [{}] stdev_dx (wlsq)\n{}", nameId(), o, lsq.variance.cwiseSqrt());
        }
        else
        {
            lsq = solveLinearLeastSquaresUncertainties(e_H_psr.topRows(ix), dpsr);
            LOG_DATA("{}:     [{}] dx (lsq) {}, {}, {}, {}", nameId(), o, lsq.solution(0), lsq.solution(1), lsq.solution(2), lsq.solution(3));
            LOG_DATA("{}:     [{}] stdev_dx (lsq)\n{}", nameId(), o, lsq.variance.cwiseSqrt());
        }

        _e_position += lsq.solution.head<3>();
        _clkBias += lsq.solution(3) / InsConst::C;
        sppSol->nSatellitesPosition = ix;
        if (ix > 4)
        {
            sppSol->setPositionAndStdDev_e(_e_position, lsq.variance.topLeftCorner<3, 3>().cwiseSqrt());
            sppSol->clkBiasStdev = std::sqrt(lsq.variance(3, 3)) / InsConst::C;
        }
        else
        {
            sppSol->setPosition_e(_e_position);
        }
        sppSol->clkBias = _clkBias;

        bool solInaccurate = lsq.solution.norm() > 1e-4;

        // ---------------------------------------------------------- Velocity -------------------------------------------------------------

        if (iv < 4)
        {
            LOG_WARN("{}: [{} GPST] Cannot calculate velocity because only {} valid doppler measurements. Try changing filter settings or reposition your antenna.",
                     nameId(), (gnssObs->insTime + std::chrono::seconds(gnssObs->insTime.leapGps2UTC())).toYMDHMS(), iv);
            continue;
        }
        // Difference between measured and estimated pseudorange rates
        Eigen::VectorXd dpsr_dot = psrRateMeas_c.topRows(iv) - psrRateEst_c.topRows(iv);
        LOG_DATA("{}:     [{}] dpsr_dot {}", nameId(), o, dpsr_dot.transpose());

        // [vx, vy, vz, clkDrift] - Groves ch. 9.4.1, eq. 9.141, p. 412
        if (_useWeightedLeastSquares)
        {
            lsq = solveWeightedLinearLeastSquaresUncertainties(e_H_r.topRows(iv), W_psrRate.topLeftCorner(iv, iv), dpsr_dot);
            LOG_DATA("{}:     [{}] dv (wlsq) {}", nameId(), o, lsq.solution.transpose());
            LOG_DATA("{}:     [{}] stdev_dv (wlsq)\n{}", nameId(), o, lsq.variance.cwiseSqrt());
        }
        else
        {
            lsq = solveLinearLeastSquaresUncertainties(e_H_r.topRows(iv), dpsr_dot);
            LOG_DATA("{}:     [{}] dv (lsq) {}", nameId(), o, lsq.solution.transpose());
            LOG_DATA("{}:     [{}] stdev_dv (lsq)\n{}", nameId(), o, lsq.variance.cwiseSqrt());
        }

        _e_velocity += lsq.solution.head<3>();
        _clkDrift += lsq.solution(3) / InsConst::C;
        sppSol->nSatellitesVelocity = iv;
        if (iv > 4)
        {
            sppSol->setVelocityAndStdDev_e(_e_velocity, lsq.variance.topLeftCorner<3, 3>().cwiseSqrt());
            sppSol->clkDriftStdev = std::sqrt(lsq.variance(3, 3)) / InsConst::C;
        }
        else
        {
            sppSol->setVelocity_e(_e_velocity);
        }
        sppSol->clkDrift = _clkDrift;

        solInaccurate |= lsq.solution.norm() > 1e-4;

        if (!solInaccurate)
        {
            break;
        }
    }

    invokeCallbacks(OUTPUT_PORT_INDEX_SPPSOL, sppSol);
}