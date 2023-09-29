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
#include "internal/gui/widgets/InputWithUnit.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "Navigation/GNSS/Satellite/Ephemeris/GLONASSEphemeris.hpp"
#include "Navigation/Math/VanLoan.hpp"

namespace SPP = NAV::GNSS::Positioning::SPP;
using SppKalmanFilter = SPP::SppKalmanFilter;

NAV::SinglePointPositioning::SinglePointPositioning()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 538, 536 };

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

    ImGui::Separator();

    // ###########################################################################################################

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

    // ###########################################################################################################

    ImGui::BeginHorizontal(fmt::format("Observables##{}", size_t(id)).c_str(),
                           ImVec2(itemWidth - ImGui::GetStyle().ItemSpacing.x + ImGui::GetStyle().ItemInnerSpacing.x, 0.0F));
    if (ImGui::Checkbox(fmt::format("Use Doppler in addition to Pseudoranges##{}", size_t(id)).c_str(), &_useDoppler))
    {
        LOG_DEBUG("{}: Using Doppler: {}", nameId(), _useDoppler);
        flow::ApplyChanges();
    }
    ImGui::EndHorizontal();

    // ###########################################################################################################

    ImGui::SetNextItemWidth(itemWidth);
    if (SPP::ComboSppEstimatorType(fmt::format("Estimation algorithm##{}", size_t(id)).c_str(), _estimatorType))
    {
        LOG_DEBUG("{}: SPP estimator algorithm changed to {}", nameId(), NAV::to_string(_estimatorType));
        flow::ApplyChanges();
    }

    // ###########################################################################################################

    ImGui::Separator();

    // ###########################################################################################################

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
    if (_estimatorType != SPP::EstimatorType::LEAST_SQUARES)
    {
        ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
        if (_estimatorType == SPP::EstimatorType::WEIGHTED_LEAST_SQUARES ? ImGui::TreeNode(fmt::format("GNSS Measurement Error Model##{}", size_t(id)).c_str())
                                                                         : ImGui::TreeNode(fmt::format("Measurement noise##{}", size_t(id)).c_str()))
        {
            if (_gnssMeasurementErrorModel.ShowGuiWidgets(std::to_string(size_t(id)).c_str(), itemWidth - ImGui::GetStyle().IndentSpacing))
            {
                LOG_DEBUG("{}: GNSS Measurement Error Model changed.", nameId());
                flow::ApplyChanges();
            }
            ImGui::TreePop();
        }
    }

    // ###########################################################################################################

    if (_estimatorType == SPP::EstimatorType::KF)
    {
        _kalmanFilter.gui();
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
    j["useDoppler"] = _useDoppler;

    // ###########################################################################################################

    j["estimatorType"] = _estimatorType;
    j["kalmanFilter"] = _kalmanFilter;

    j["ionosphereModel"] = _ionosphereModel;
    j["troposphereModels"] = _troposphereModels;
    j["gnssMeasurementError"] = _gnssMeasurementErrorModel;

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
    if (j.contains("useDoppler"))
    {
        j.at("useDoppler").get_to(_useDoppler);
    }

    // ###########################################################################################################

    if (j.contains("estimatorType"))
    {
        j.at("estimatorType").get_to(_estimatorType);
    }
    if (j.contains("kalmanFilter"))
    {
        j.at("kalmanFilter").get_to(_kalmanFilter);
    }

    // ###########################################################################################################

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

    // ###########################################################################################################
}

bool NAV::SinglePointPositioning::initialize()
{
    LOG_TRACE("{}: called", nameId());

    if (std::all_of(inputPins.begin() + INPUT_PORT_INDEX_GNSS_NAV_INFO, inputPins.end(), [](const InputPin& inputPin) { return !inputPin.isPinLinked(); }))
    {
        LOG_ERROR("{}: You need to connect a GNSS NavigationInfo provider", nameId());
        return false;
    }

    _state = {};
    _kalmanFilter.initialize();

    LOG_DEBUG("{}: initialized", nameId());

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

    auto gnssObs = std::static_pointer_cast<const GnssObs>(queue.extract_front());
    LOG_DATA("{}: Calculating SPP for [{}]", nameId(), gnssObs->insTime);

    auto sppSol = std::make_shared<SppSolution>();

    if (_estimatorType == NAV::GNSS::Positioning::SPP::EstimatorType::LEAST_SQUARES || _estimatorType == NAV::GNSS::Positioning::SPP::EstimatorType::WEIGHTED_LEAST_SQUARES)
    {
        sppSol = calcSppSolutionLSE(_state, gnssObs, gnssNavInfos,
                                    _ionosphereModel, _troposphereModels, _gnssMeasurementErrorModel, _estimatorType,
                                    _filterFreq, _filterCode, _excludedSatellites, _elevationMask, _useDoppler);
    }
    else // if (_estimatorType == NAV::GNSS::Positioning::SPP::EstimatorType::KF)
    {
        sppSol = calcSppSolutionKF(_kalmanFilter, gnssObs, gnssNavInfos,
                                   _ionosphereModel, _troposphereModels, _gnssMeasurementErrorModel,
                                   _filterFreq, _filterCode, _excludedSatellites, _elevationMask, _useDoppler);
    }

    if (sppSol)
    {
        _state = SPP::State{ .e_position = sppSol->e_position(),
                             .e_velocity = sppSol->e_velocity(),
                             .recvClk = sppSol->recvClk };

        invokeCallbacks(OUTPUT_PORT_INDEX_SPPSOL, sppSol);
    }
}