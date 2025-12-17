// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "RealTimeKinematic.hpp"

#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <imgui.h>
#include <iterator>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <tuple>
#include <ranges>
#include <regex>
#include <utility>
#include <variant>
#include <vector>
#include <spdlog/common.h>
#include <spdlog/spdlog.h>

#include "Navigation/GNSS/Ambiguity/AmbiguityResolution.hpp"
#include "Navigation/GNSS/Ambiguity/CycleSlipDetector.hpp"
#include "Navigation/GNSS/Core/Code.hpp"
#include "Navigation/GNSS/Core/SatelliteIdentifier.hpp"
#include "Navigation/GNSS/Positioning/Observation.hpp"
#include "Navigation/GNSS/Positioning/RTK/Keys.hpp"
#include "Navigation/Math/KeyedLeastSquares.hpp"
#include "Navigation/Time/InsTime.hpp"
#include "Navigation/Time/TimeSystem.hpp"
#include "NodeData/GNSS/GnssObs.hpp"
#include "NodeData/GNSS/RtkSolution.hpp"
#include "internal/Node/Node.hpp"
#include "util/Json.hpp"
#include "util/Logger.hpp"
#include "util/Container/Vector.hpp"
#include "util/Container/UncertainValue.hpp"

#include "Navigation/Constants.hpp"

#include "internal/gui/NodeEditorApplication.hpp"
#include "internal/gui/widgets/HelpMarker.hpp"
#include "internal/gui/widgets/imgui_ex.hpp"
#include "internal/gui/widgets/InputWithUnit.hpp"
#include "internal/gui/widgets/EnumCombo.hpp"

#include <fmt/core.h>
#include <fmt/format.h>
#include "internal/FlowManager.hpp"

#include "NodeData/State/Pos.hpp"

#include "Navigation/GNSS/Ambiguity/internal/Search.hpp"
#include "Navigation/GNSS/Ambiguity/internal/Decorrelation.hpp"
#include "Navigation/GNSS/Core/Frequency.hpp"
#include "Navigation/GNSS/Functions.hpp"
#include "Navigation/Math/LeastSquares.hpp"

namespace States = NAV::RTK::States;
namespace Meas = NAV::RTK::Meas;

namespace NAV
{

RealTimeKinematic::RealTimeKinematic()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 633, 670 };

    CreateInputPin("Base Position", Pin::Type::Flow, { Pos::type() }, &RealTimeKinematic::recvBasePos, nullptr, 1);
    CreateInputPin("GnssObs (Base)", Pin::Type::Flow, { GnssObs::type() }, &RealTimeKinematic::recvBaseGnssObs);
    CreateInputPin("GnssObs (Rover)", Pin::Type::Flow, { GnssObs::type() }, &RealTimeKinematic::recvRoverGnssObs);
    _dynamicInputPins.addPin(this); // GnssNavInfo

    CreateOutputPin(RtkSolution::type().c_str(), Pin::Type::Flow, { RtkSolution::type() });
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
    auto nSatColumnContent = [&](size_t pinIndex) -> bool {
        if (auto gnssNavInfo = getInputValue<GnssNavInfo>(pinIndex))
        {
            size_t usedSatNum = 0;
            std::string usedSats;
            std::string allSats;

            std::string filler = ", ";
            for (const auto& satellite : gnssNavInfo->v->satellites())
            {
                if (_obsFilter.isSatelliteAllowed(satellite.first))
                {
                    usedSatNum++;
                    usedSats += (allSats.empty() ? "" : filler) + fmt::format("{}", satellite.first);
                }
                allSats += (allSats.empty() ? "" : filler) + fmt::format("{}", satellite.first);
            }
            ImGui::TextUnformatted(fmt::format("{} / {}", usedSatNum, gnssNavInfo->v->nSatellites()).c_str());
            if (ImGui::IsItemHovered())
            {
                ImGui::SetTooltip("Used  satellites: %s\n"
                                  "Avail satellites: %s",
                                  usedSats.c_str(), allSats.c_str());
            }
        }
        return false;
    };

    if (_dynamicInputPins.ShowGuiWidgets(size_t(id), inputPins, this, { { .header = "# Sat", .content = nSatColumnContent } }))
    {
        flow::ApplyChanges();
    }

    ImGui::SameLine();
    ImGui::BeginGroup();
    ImGui::PushFont(Application::MonoFont());
    auto totalSolutions = nFixSolutions + nFloatSolutions + nSingleSolutions;
    ImGui::SetNextItemOpen(true, ImGuiCond_Always);
    if (ImGui::TreeNode("Solution statistics:"))
    {
        ImGui::BulletText("Fix:    %6.2f%% (%zu)", totalSolutions > 0 ? static_cast<double>(nFixSolutions) / static_cast<double>(totalSolutions) * 100.0 : 0.0, nFixSolutions);
        ImGui::BulletText("Float:  %6.2f%% (%zu)", totalSolutions > 0 ? static_cast<double>(nFloatSolutions) / static_cast<double>(totalSolutions) * 100.0 : 0.0, nFloatSolutions);
        ImGui::BulletText("Single: %6.2f%% (%zu)", totalSolutions > 0 ? static_cast<double>(nSingleSolutions) / static_cast<double>(totalSolutions) * 100.0 : 0.0, nSingleSolutions);
        ImGui::TreePop();
    }
    if (ImGui::TreeNode(fmt::format("Cycle-slips: {}", _nCycleSlipsLLI + _nCycleSlipsSingle + _nCycleSlipsDual).c_str()))
    {
        ImGui::BulletText("LLI:         %zu", _nCycleSlipsLLI);
        ImGui::BulletText("Single Freq: %zu", _nCycleSlipsSingle);
        ImGui::SameLine();
        gui::widgets::HelpMarker("Single Frequency detection is confirmed by\nthe dual frequency detector before accepted.\nIt then only counts towards the single frequency count though.");
        ImGui::BulletText("Dual   Freq: %zu", _nCycleSlipsDual);
        ImGui::SameLine();
        gui::widgets::HelpMarker("Only cycle-slips which were not recognized by\nthe single frequency detector or had the LLI flag set.");
        ImGui::TreePop();
    }
    ImGui::BulletText("Pivot changes: %zu", _nPivotChange);
    if (_kalmanFilter.isNISenabled()) { ImGui::BulletText("NIS triggered: %zu", _nMeasExcludedNIS); }
    ImGui::PopFont();
    ImGui::EndGroup();

    const float unitWidth = 120.0F * gui::NodeEditorApplication::windowFontRatio();
    const float itemWidth = 310.0F * gui::NodeEditorApplication::windowFontRatio();
    const float cWidth = itemWidth - ImGui::GetStyle().IndentSpacing;

    if (_obsFilter.ShowGuiWidgets<ReceiverType>(nameId().c_str(), itemWidth))
    {
        flow::ApplyChanges();
    }

    if (_obsEstimator.ShowGuiWidgets<ReceiverType>(nameId().c_str(), itemWidth))
    {
        flow::ApplyChanges();
    }

    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    if (ImGui::TreeNode(fmt::format("Kalman Filter##{}", nameId()).c_str()))
    {
        if (_kalmanFilter.showKalmanFilterGUI(nameId().c_str(), itemWidth))
        {
            flow::ApplyChanges();
        }

        if (_kalmanFilter.isNISenabled())
        {
            auto tmp = static_cast<int>(_maxRemoveOutlier);
            ImGui::SetNextItemWidth(cWidth);
            if (ImGui::InputIntL(fmt::format("Max. remove per epoch##{}", nameId()).c_str(), &tmp, 0))
            {
                _maxRemoveOutlier = static_cast<size_t>(tmp);
                flow::ApplyChanges();
            }
            ImGui::SameLine();
            gui::widgets::HelpMarker("Maximum amount of observations to remove per epoch if the NIS triggers");

            tmp = static_cast<int>(_outlierRemoveEpochs);
            ImGui::SetNextItemWidth(cWidth);
            if (ImGui::InputIntL(fmt::format("Epochs to exclude outlier##{}", nameId()).c_str(), &tmp, 1))
            {
                _outlierRemoveEpochs = static_cast<size_t>(tmp);
                flow::ApplyChanges();
            }
            ImGui::SameLine();
            gui::widgets::HelpMarker("Amount of epochs to exclude an outlier for after being flagged");

            tmp = static_cast<int>(_outlierMinSat);
            ImGui::SetNextItemWidth(cWidth);
            if (ImGui::InputIntL(fmt::format("Min. satellites for check##{}", nameId()).c_str(), &tmp, 1))
            {
                _outlierMinSat = static_cast<size_t>(tmp);
                flow::ApplyChanges();
            }
            ImGui::SameLine();
            gui::widgets::HelpMarker("Minumum amount of satellites for doing a NIS check");

            tmp = static_cast<int>(_outlierMinPsrObsKeep);
            ImGui::SetNextItemWidth(cWidth);
            if (ImGui::InputIntL(fmt::format("# of pseudorange obs to keep##{}", nameId()).c_str(), &tmp, 1))
            {
                _outlierMinPsrObsKeep = static_cast<size_t>(tmp);
                flow::ApplyChanges();
            }
            ImGui::SameLine();
            gui::widgets::HelpMarker("Minumum amount of pseudorange observables to leave when doing a NIS check");

            ImGui::SetNextItemWidth(cWidth);
            if (ImGui::InputDoubleL(fmt::format("Max Position Var for NIS in startup##{}", size_t(id)).c_str(), &_outlierMaxPosVarStartup, 0, std::numeric_limits<double>::max(), 0.0, 0.0, "%.4f m"))
            {
                flow::ApplyChanges();
            }
            ImGui::SameLine();
            gui::widgets::HelpMarker("Maximum position variance\nin order to attempt outlier removal during the startup phase.");
        }
        ImGui::SetNextItemOpen(false, ImGuiCond_FirstUseEver);
        if (ImGui::TreeNode(fmt::format("Matrices##Kalman Filter {}", nameId()).c_str()))
        {
            _kalmanFilter.showKalmanFilterMatrixViews(nameId().c_str());
            ImGui::TreePop();
        }
        ImGui::TreePop();
    }

    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    if (ImGui::TreeNode(fmt::format("System/Process noise (Standard Deviations)##{}", size_t(id)).c_str()))
    {
        if (gui::widgets::InputDouble2WithUnit(fmt::format("Acceleration due to user motion (Hor/Ver)##{}", size_t(id)).c_str(), cWidth, unitWidth, _gui_stdevAccel.data(), _gui_stdevAccelUnits, "m/√(s^3)\0\0", "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: stdevAccel changed to horizontal {} and vertical {}", nameId(), _gui_stdevAccel.at(0), _gui_stdevAccel.at(1));
            LOG_DEBUG("{}: stdevAccelNoiseUnits changed to {}", nameId(), fmt::underlying(_gui_stdevAccelUnits));
            flow::ApplyChanges();
        }
        if (gui::widgets::InputDoubleWithUnit(fmt::format("Ambiguity noise (carrier-phase bias) (float)##{}", size_t(id)).c_str(),
                                              cWidth, unitWidth, &_gui_ambiguityFloatProcessNoiseStDev,
                                              _gui_stdevAmbiguityFloatUnits, "cycle\0\0",
                                              0.0, 0.0, "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: ambiguityFloatProcessNoiseStDev changed to {}", nameId(), _gui_ambiguityFloatProcessNoiseStDev);
            LOG_DEBUG("{}: stdevAmbiguityFloatUnits changed to {}", nameId(), fmt::underlying(_gui_stdevAmbiguityFloatUnits));

            switch (_gui_stdevAmbiguityFloatUnits)
            {
            case StdevAmbiguityUnits::Cycle:
                _ambiguityFloatProcessNoiseVariance = std::pow(_gui_ambiguityFloatProcessNoiseStDev, 2);
                break;
            }

            flow::ApplyChanges();
        }
        if (gui::widgets::InputDoubleWithUnit(fmt::format("Ambiguity noise (carrier-phase bias) (fix)##{}", size_t(id)).c_str(),
                                              cWidth, unitWidth, &_gui_ambiguityFixProcessNoiseStDev,
                                              _gui_stdevAmbiguityFixUnits, "cycle\0\0",
                                              0.0, 0.0, "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: ambiguityFixProcessNoiseStDev changed to {}", nameId(), _gui_ambiguityFixProcessNoiseStDev);
            LOG_DEBUG("{}: stdevAmbiguityFixUnits changed to {}", nameId(), fmt::underlying(_gui_stdevAmbiguityFixUnits));

            switch (_gui_stdevAmbiguityFixUnits)
            {
            case StdevAmbiguityUnits::Cycle:
                _ambiguityFixProcessNoiseVariance = std::pow(_gui_ambiguityFixProcessNoiseStDev, 2);
                break;
            }

            flow::ApplyChanges();
        }

        ImGui::TreePop();
    }

    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    if (ImGui::TreeNode(fmt::format("Ambiguity resolution##{}", size_t(id)).c_str()))
    {
        if (!_obsFilter.isObsTypeUsed(GnssObs::Carrier)) { ImGui::BeginDisabled(); }

        ImGui::SetNextItemWidth(cWidth);
        if (gui::widgets::EnumCombo(fmt::format("Strategy##{}", size_t(id)).c_str(), _ambiguityResolutionStrategy))
        {
            LOG_DEBUG("{}: Ambiguity resolution strategy changed to {}", nameId(), NAV::to_string(_ambiguityResolutionStrategy));
            flow::ApplyChanges();
        }

        if (GuiAmbiguityResolution(fmt::format("Ambiguity resolution##{}", size_t(id)).c_str(), _ambiguityResolutionParameters, itemWidth))
        {
            LOG_DEBUG("{}: Ambiguity resolution parameters changed to [{}][{}]", nameId(),
                      NAV::to_string(_ambiguityResolutionParameters.decorrelationAlgorithm),
                      NAV::to_string(_ambiguityResolutionParameters.searchAlgorithm));
            flow::ApplyChanges();
        }

        ImGui::SetNextItemWidth(cWidth);
        if (ImGui::InputDoubleL(fmt::format("Max Position Var for AR##{}", size_t(id)).c_str(), &_maxPosVar, 0, std::numeric_limits<double>::max(), 0.0, 0.0, "%.4f m"))
        {
            flow::ApplyChanges();
        }
        ImGui::SameLine();
        gui::widgets::HelpMarker("Maximum position variance\nin order to attempt ambiguity fixing.");

        auto tmp = static_cast<int>(_nMinSatForAmbFix);
        ImGui::SetNextItemWidth(cWidth);
        if (ImGui::InputIntL(fmt::format("Min Sat for Fix##{}", size_t(id)).c_str(), &tmp, 0))
        {
            _nMinSatForAmbFix = static_cast<size_t>(tmp);
            flow::ApplyChanges();
        }
        ImGui::SameLine();
        gui::widgets::HelpMarker("Minimum amount of satellites with carrier observations\nin order to attempt ambiguity fixing.");
        if (_ambiguityResolutionStrategy == AmbiguityResolutionStrategy::FixAndHold)
        {
            tmp = static_cast<int>(_nMinSatForAmbHold);
            ImGui::SetNextItemWidth(cWidth);
            if (ImGui::InputIntL(fmt::format("Min Sat for Hold##{}", size_t(id)).c_str(), &tmp, 0))
            {
                _nMinSatForAmbHold = static_cast<size_t>(tmp);
                flow::ApplyChanges();
            }
            ImGui::SameLine();
            gui::widgets::HelpMarker("Minimum amount of satellites with carrier observations\nin order to attempt ambiguity holding.");
        }
        if (ImGui::Checkbox(fmt::format("Apply fixed ambiguities with KF update##{}", size_t(id)).c_str(), &_applyFixedAmbiguitiesWithUpdate))
        {
            flow::ApplyChanges();
        }
        ImGui::SameLine();
        gui::widgets::HelpMarker("Make a Kalman Filter update with the fixed ambiguities when checked.\n"
                                 "Otherwise apply via\n"
                                 "- a = a_fix\n"
                                 "- b = b_float - Q_ba * Q_aa^-1 (a_fix - a_float)");

        if (_applyFixedAmbiguitiesWithUpdate
            && gui::widgets::InputDoubleWithUnit(fmt::format("Apply fixed ambiguities measurement noise##{}", size_t(id)).c_str(),
                                                 cWidth, unitWidth, &_gui_ambFixUpdateStdDev,
                                                 _gui_ambFixUpdateStdDevUnits, "cycle\0\0",
                                                 0.0, 0.0, "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: ambFixUpdateStdDev changed to {}", nameId(), _gui_ambFixUpdateStdDev);
            LOG_DEBUG("{}: ambFixUpdateStdDevUnits changed to {}", nameId(), fmt::underlying(_gui_ambFixUpdateStdDevUnits));

            switch (_gui_ambFixUpdateStdDevUnits)
            {
            case StdevAmbiguityUnits::Cycle:
                _ambFixUpdateVariance = std::pow(_gui_ambFixUpdateStdDev, 2);
                break;
            }

            flow::ApplyChanges();
        }

        ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
        if (ImGui::TreeNode(fmt::format("Cycle-slip detection##Tree{}", size_t(id)).c_str()))
        {
            ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
            if (ImGui::TreeNode(fmt::format("Base##Tree{}", size_t(id)).c_str()))
            {
                if (CycleSlipDetectorGui(fmt::format("Cycle-slip detector Base##{}", size_t(id)).c_str(), _cycleSlipDetector[Base], 145.0F))
                {
                    flow::ApplyChanges();
                }
                ImGui::TreePop();
            }
            ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
            if (ImGui::TreeNode(fmt::format("Rover##Tree{}", size_t(id)).c_str()))
            {
                if (CycleSlipDetectorGui(fmt::format("Cycle-slip detector Rover##{}", size_t(id)).c_str(), _cycleSlipDetector[Rover], 145.0F))
                {
                    flow::ApplyChanges();
                }
                ImGui::TreePop();
            }

            ImGui::TreePop();
        }

        if (!_obsFilter.isObsTypeUsed(GnssObs::Carrier)) { ImGui::EndDisabled(); }
        ImGui::TreePop();
    }
    if (ImGui::TreeNode(fmt::format("Misc##{}", size_t(id)).c_str()))
    {
        if (ImGui::Checkbox(fmt::format("Output a SPP solution if no base observation available##{}", size_t(id)).c_str(), &_calcSPPIfNoBase))
        {
            flow::ApplyChanges();
        }
        ImGui::SetNextItemWidth(cWidth);
        if (ImGui::InputDoubleL(fmt::format("Max Time between rover & base obs##{}", size_t(id)).c_str(), &_maxTimeBetweenBaseRoverForRTK, 1e-3))
        {
            flow::ApplyChanges();
        }

        ImGui::TreePop();
    }

    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    if (ImGui::TreeNode(fmt::format("Events##{}", size_t(id)).c_str()))
    {
        if (ImGui::Checkbox(fmt::format("Output State Change Events##{}", size_t(id)).c_str(), &_outputStateEvents))
        {
            flow::ApplyChanges();
        }
        ImGui::SetNextItemWidth(cWidth);
        if (ImGui::InputText(fmt::format("Events Filter Regex##{}", size_t(id)).c_str(), &_eventFilterRegex))
        {
            flow::ApplyChanges();
        }
        if (!_events.empty())
        {
            std::optional<std::regex> filter;
            try
            {
                filter = std::regex(_eventFilterRegex, std::regex_constants::ECMAScript | std::regex_constants::icase);
            }
            catch (...) // NOLINT(bugprone-empty-catch)
            {}

            if (ImGui::BeginChild(fmt::format("Events list {}", size_t(id)).c_str(), ImVec2(0.0F, 600.0F), true))
            {
                ImGui::PushFont(Application::MonoFont());
                for (size_t i = 0; i < _events.size(); i++)
                {
                    if (!std::any_of(_events.at(i).second.begin(), _events.at(i).second.end(), [&](const std::string& text) {
                            return _eventFilterRegex.empty() || (filter && std::regex_search(text, *filter));
                        }))
                    {
                        continue;
                    }

                    if (i != 0) { ImGui::Separator(); }
                    ImGui::SetNextItemOpen(true, ImGuiCond_Always);
                    if (ImGui::TreeNode(fmt::format("{} GPST", _events.at(i).first.toYMDHMS(GPST)).c_str()))
                    {
                        for (const auto& text : _events.at(i).second)
                        {
                            if (_eventFilterRegex.empty() || (filter && std::regex_search(text, *filter)))
                            {
                                ImGui::BulletText("%s", text.c_str());
                            }
                        }
                        ImGui::TreePop();
                    }
                }
                ImGui::PopFont();
            }
            ImGui::EndChild();
        }
        ImGui::TreePop();
    }
}

[[nodiscard]] json RealTimeKinematic::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["dynamicInputPins"] = _dynamicInputPins;

    j["obsFilter"] = _obsFilter;
    j["obsEstimator"] = _obsEstimator;
    j["kalmanFilter"] = _kalmanFilter;
    j["maxRemoveOutlier"] = _maxRemoveOutlier;
    j["outlierRemoveEpochs"] = _outlierRemoveEpochs;
    j["outlierMinSat"] = _outlierMinSat;
    j["outlierMinPsrObsKeep"] = _outlierMinPsrObsKeep;
    j["outlierMaxPosVarStartup"] = _outlierMaxPosVarStartup;
    j["calcSPPIfNoBase"] = _calcSPPIfNoBase;
    j["maxTimeBetweenBaseRoverForRTK"] = _maxTimeBetweenBaseRoverForRTK;

    j["ambiguityResolutionParameters"] = _ambiguityResolutionParameters;
    j["ambiguityResolutionStrategy"] = _ambiguityResolutionStrategy;
    j["nMinSatForAmbFix"] = _nMinSatForAmbFix;
    j["nMinSatForAmbHold"] = _nMinSatForAmbHold;
    j["maxPosVar"] = _maxPosVar;
    j["applyFixedAmbiguitiesWithUpdate"] = _applyFixedAmbiguitiesWithUpdate;
    j["ambFixUpdateStdDev"] = _gui_ambFixUpdateStdDev;
    j["ambFixUpdateStdDevUnits"] = _gui_ambFixUpdateStdDevUnits;

    j["cycleSlipDetector"] = _cycleSlipDetector;
    j["eventFilterRegex"] = _eventFilterRegex;
    j["outputStateEvents"] = _outputStateEvents;

    j["stdevAccelUnits"] = _gui_stdevAccelUnits;
    j["stdevAccel"] = _gui_stdevAccel;
    j["stdevAmbiguityFloatUnits"] = _gui_stdevAmbiguityFloatUnits;
    j["ambiguityFloatProcessNoiseStDev"] = _gui_ambiguityFloatProcessNoiseStDev;
    j["stdevAmbiguityFixUnits"] = _gui_stdevAmbiguityFixUnits;
    j["ambiguityFixProcessNoiseStDev"] = _gui_ambiguityFixProcessNoiseStDev;

    return j;
}

void RealTimeKinematic::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("dynamicInputPins")) { NAV::gui::widgets::from_json(j.at("dynamicInputPins"), _dynamicInputPins, this); }

    if (j.contains("obsFilter")) { j.at("obsFilter").get_to(_obsFilter); }
    if (j.contains("obsEstimator")) { j.at("obsEstimator").get_to(_obsEstimator); }
    if (j.contains("kalmanFilter")) { j.at("kalmanFilter").get_to(_kalmanFilter); }
    if (j.contains("maxRemoveOutlier")) { j.at("maxRemoveOutlier").get_to(_maxRemoveOutlier); }
    if (j.contains("outlierRemoveEpochs")) { j.at("outlierRemoveEpochs").get_to(_outlierRemoveEpochs); }
    if (j.contains("outlierMinSat")) { j.at("outlierMinSat").get_to(_outlierMinSat); }
    if (j.contains("outlierMinPsrObsKeep")) { j.at("outlierMinPsrObsKeep").get_to(_outlierMinPsrObsKeep); }
    if (j.contains("outlierMaxPosVarStartup")) { j.at("outlierMaxPosVarStartup").get_to(_outlierMaxPosVarStartup); }
    if (j.contains("calcSPPIfNoBase")) { j.at("calcSPPIfNoBase").get_to(_calcSPPIfNoBase); }
    if (j.contains("maxTimeBetweenBaseRoverForRTK")) { j.at("maxTimeBetweenBaseRoverForRTK").get_to(_maxTimeBetweenBaseRoverForRTK); }

    if (j.contains("ambiguityResolutionParameters")) { j.at("ambiguityResolutionParameters").get_to(_ambiguityResolutionParameters); }
    if (j.contains("ambiguityResolutionStrategy")) { j.at("ambiguityResolutionStrategy").get_to(_ambiguityResolutionStrategy); }
    if (j.contains("nMinSatForAmbFix")) { j.at("nMinSatForAmbFix").get_to(_nMinSatForAmbFix); }
    if (j.contains("nMinSatForAmbHold")) { j.at("nMinSatForAmbHold").get_to(_nMinSatForAmbHold); }
    if (j.contains("maxPosVar")) { j.at("maxPosVar").get_to(_maxPosVar); }
    if (j.contains("applyFixedAmbiguitiesWithUpdate")) { j.at("applyFixedAmbiguitiesWithUpdate").get_to(_applyFixedAmbiguitiesWithUpdate); }
    if (j.contains("ambFixUpdateStdDevUnits")) { j.at("ambFixUpdateStdDevUnits").get_to(_gui_ambFixUpdateStdDevUnits); }
    if (j.contains("ambFixUpdateStdDev"))
    {
        j.at("ambFixUpdateStdDev").get_to(_gui_ambFixUpdateStdDev);
        switch (_gui_ambFixUpdateStdDevUnits)
        {
        case StdevAmbiguityUnits::Cycle:
            _ambFixUpdateVariance = std::pow(_gui_ambFixUpdateStdDev, 2);
            break;
        }
    }

    if (j.contains("cycleSlipDetector")) { j.at("cycleSlipDetector").get_to(_cycleSlipDetector); }
    if (j.contains("eventFilterRegex")) { j.at("eventFilterRegex").get_to(_eventFilterRegex); }
    if (j.contains("outputStateEvents")) { j.at("outputStateEvents").get_to(_outputStateEvents); }

    if (j.contains("stdevAccelUnits")) { _gui_stdevAccelUnits = j.at("stdevAccelUnits"); }
    if (j.contains("stdevAccel")) { _gui_stdevAccel = j.at("stdevAccel"); }

    if (j.contains("stdevAmbiguityFloatUnits")) { j.at("stdevAmbiguityFloatUnits").get_to(_gui_stdevAmbiguityFloatUnits); }
    if (j.contains("ambiguityFloatProcessNoiseStDev"))
    {
        j.at("ambiguityFloatProcessNoiseStDev").get_to(_gui_ambiguityFloatProcessNoiseStDev);
        switch (_gui_stdevAmbiguityFloatUnits)
        {
        case StdevAmbiguityUnits::Cycle:
            _ambiguityFloatProcessNoiseVariance = std::pow(_gui_ambiguityFloatProcessNoiseStDev, 2);
            break;
        }
    }
    if (j.contains("stdevAmbiguityFixUnits")) { j.at("stdevAmbiguityFixUnits").get_to(_gui_stdevAmbiguityFixUnits); }

    if (j.contains("ambiguityFixProcessNoiseStDev"))
    {
        j.at("ambiguityFixProcessNoiseStDev").get_to(_gui_ambiguityFixProcessNoiseStDev);
        switch (_gui_stdevAmbiguityFixUnits)
        {
        case StdevAmbiguityUnits::Cycle:
            _ambiguityFixProcessNoiseVariance = std::pow(_gui_ambiguityFixProcessNoiseStDev, 2);
            break;
        }
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

    _obsFilter.reset();
    _events.clear();
    _pivotSatellites.clear();
    _lastUpdate.reset();
    _dataInterval = 1;
    _receiver = { { Receiver(static_cast<ReceiverType>(0), _obsFilter.getSystemFilter().toVector()),
                    Receiver(static_cast<ReceiverType>(1), _obsFilter.getSystemFilter().toVector()) } };
    _lastSolutionStatus = RtkSolution::SolutionType::RTK_Float;
    _ambiguitiesHold.clear();
    nFixSolutions = 0;
    nFloatSolutions = 0;
    nSingleSolutions = 0;
    _nPivotChange = 0;
    _nCycleSlipsLLI = 0;
    _nCycleSlipsSingle = 0;
    _nCycleSlipsDual = 0;
    _nMeasExcludedNIS = 0;

    _outlierMaxPosVarStartupTriggered = false;
    _maxPosVarTriggered = false;
    _nMinSatForAmbFixTriggered = false;
    _nMinSatForAmbHoldTriggered = false;

    _sppAlgorithm.reset();
    _sppAlgorithm._obsFilter = _obsFilter;
    _sppAlgorithm._obsFilter.useObsType(GnssObs::Pseudorange);
    _sppAlgorithm._obsFilter.useObsType(GnssObs::Doppler);
    _sppAlgorithm._obsEstimator = _obsEstimator;
    _sppAlgorithm._estimateInterFreqBiases = false;

    json j;
    to_json(j, _kalmanFilter);
    _kalmanFilter = KeyedKalmanFilterD<States::StateKeyType,
                                       Meas::MeasKeyTypes>{ States::PosVel, {} };
    from_json(j, _kalmanFilter);

    _kalmanFilter.F.block<3>(States::Pos, States::Vel) = Eigen::Matrix3d::Identity();

    for (auto& detector : _cycleSlipDetector) { detector.reset(); }

    // 𝜎²_a Variance of the acceleration due to user motion in horizontal and vertical component in [m^2 / s^3]
    std::array<double, 2> varAccel{};
    switch (_gui_stdevAccelUnits)
    {
    case StdevAccelUnits::m_sqrts3: // [m / √(s^3)]
        varAccel = { std::pow(_gui_stdevAccel.at(0), 2), std::pow(_gui_stdevAccel.at(1), 2) };
        break;
    }
    LOG_DATA("  sigma2_accel = h: {}, v: {} [m^2 / s^3]", nameId(), varAccel.at(0), varAccel.at(1));
    _kalmanFilter.W.block<3>(States::Vel, States::Vel).diagonal() << varAccel.at(0), varAccel.at(0), varAccel.at(1);

    LOG_DEBUG("RealTimeKinematic initialized");

    return true;
}

void RealTimeKinematic::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

void RealTimeKinematic::pinAddCallback(Node* node)
{
    node->CreateInputPin(NAV::GnssNavInfo::type().c_str(), Pin::Type::Object, { NAV::GnssNavInfo::type() });
}

void RealTimeKinematic::pinDeleteCallback(Node* node, size_t pinIdx)
{
    node->DeleteInputPin(pinIdx);
}

void RealTimeKinematic::addEventToGui(const std::shared_ptr<RtkSolution>& rtkSol, const std::string& text)
{
    auto iter = std::ranges::find_if(_events, [&](const auto& event) { return event.first == rtkSol->insTime; });
    if (iter == _events.end())
    {
        _events.emplace_back(rtkSol->insTime, std::vector{ text });
    }
    else
    {
        iter->second.emplace_back(text);
    }
    rtkSol->addEvent(text);
}

void RealTimeKinematic::printObservations([[maybe_unused]] const Observations& observations)
{
#if LOG_LEVEL <= LOG_LEVEL_DATA
    unordered_map<SatId, std::pair<Frequency, Code>> satData;
    unordered_map<SatSigId, std::set<GnssObs::ObservationType>> sigData;
    for (const auto& obs : observations.signals)
    {
        satData[obs.first.toSatId()].first |= obs.first.freq();
        satData[obs.first.toSatId()].second |= obs.first.code;
        for (size_t obsType = 0; obsType < GnssObs::ObservationType_COUNT; obsType++)
        {
            if (std::ranges::all_of(obs.second.recvObs, [&obsType](const auto& recvObs) {
                    return recvObs.second->obs.contains(static_cast<GnssObs::ObservationType>(obsType));
                }))
            {
                sigData[obs.first].insert(static_cast<GnssObs::ObservationType>(obsType));
            }
        }
    }
    for ([[maybe_unused]] const auto& [satId, freqCode] : satData)
    {
        LOG_DATA("{}:   [{}] on frequencies [{}] with codes [{}]", nameId(), satId, freqCode.first, freqCode.second);
        for (const auto& [satSigId, obs] : sigData)
        {
            if (satSigId.toSatId() != satId) { continue; }
            std::string str;
            for (const auto& o : obs)
            {
                if (!str.empty()) { str += ", "; }
                str += fmt::format("{}", o);
            }
            LOG_DATA("{}:       [{}] has obs: {}", nameId(), satSigId.code, str);
        }
    }
#endif
}

void RealTimeKinematic::recvBasePos(InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    auto gnssObs = std::static_pointer_cast<const Pos>(queue.extract_front());
    LOG_DATA("{}: Received Base  Position for [{}]", nameId(), gnssObs->insTime.toYMDHMS(GPST));

    if (_receiver[Base].e_posMarker.isZero())
    {
        _receiver[Base].e_posMarker = gnssObs->e_position();
        _receiver[Base].lla_posMarker = gnssObs->lla_position();
    }
    else
    {
        _receiver[Base].e_posMarker = (_receiver[Base].e_posMarker + gnssObs->e_position()) / 2;
        _receiver[Base].lla_posMarker = trafo::ecef2lla_WGS84(_receiver[Base].e_posMarker);
    }

    if (_receiver[Base].gnssObs && _receiver[Rover].gnssObs && std::chrono::abs(_receiver[Base].gnssObs->insTime - _receiver[Rover].gnssObs->insTime) < std::chrono::milliseconds(10))
    {
        calcRealTimeKinematicSolution();
    }
}

void RealTimeKinematic::recvBaseGnssObs(InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    _receiver[Base].gnssObs = std::static_pointer_cast<const GnssObs>(queue.extract_front());
    _baseObsReceivedThisEpoch = true;
    LOG_DATA("{}: Received Base  GNSS Obs for [{}]", nameId(), _receiver[Base].gnssObs->insTime.toYMDHMS(GPST));

    if (_receiver[Rover].gnssObs && (_receiver[Base].gnssObs->insTime - _receiver[Rover].gnssObs->insTime) > std::chrono::milliseconds(500))
    {
        LOG_TRACE("{}: Data gap between Base [{}] and Rover [{}] is {:.1f}s. Falling back to SPP for gaps larger than 0.5s", nameId(),
                  _receiver[Base].gnssObs->insTime.toYMDHMS(GPST), _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST),
                  static_cast<double>((_receiver[Base].gnssObs->insTime - _receiver[Rover].gnssObs->insTime).count()));
        if (auto rtkSol = calcFallbackSppSolution())
        {
            nSingleSolutions++;
            invokeCallbacks(OUTPUT_PORT_INDEX_RTKSOL, rtkSol);
        }
        _receiver[Rover].gnssObs = nullptr;
    }
    else if (!_receiver[Base].e_posMarker.isZero() && _receiver[Base].gnssObs && _receiver[Rover].gnssObs
             && std::chrono::abs(_receiver[Base].gnssObs->insTime - _receiver[Rover].gnssObs->insTime) < std::chrono::milliseconds(100))
    {
        calcRealTimeKinematicSolution();
    }
}

void RealTimeKinematic::recvRoverGnssObs(InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    _receiver[Rover].gnssObs = std::static_pointer_cast<const GnssObs>(queue.extract_front());
    LOG_DATA("{}: Received Rover GNSS Obs for [{}]", nameId(), _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST));

    if (_receiver[Base].gnssObs && (_receiver[Rover].gnssObs->insTime - _receiver[Base].gnssObs->insTime) > std::chrono::milliseconds(static_cast<int>(_maxTimeBetweenBaseRoverForRTK * 1e3)))
    {
        if (!_calcSPPIfNoBase) { return; }
        LOG_TRACE("{}: Calculating SPP Fallback, as Rover obs [{}] is {:.3f} seconds away from last base obs [{}]. Maximum time to consider the base observation is {} seconds.",
                  nameId(), _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST),
                  static_cast<double>((_receiver[Rover].gnssObs->insTime - _receiver[Base].gnssObs->insTime).count()),
                  _receiver[Base].gnssObs->insTime.toYMDHMS(GPST),
                  _maxTimeBetweenBaseRoverForRTK);
        if (auto rtkSol = calcFallbackSppSolution())
        {
            nSingleSolutions++;
            invokeCallbacks(OUTPUT_PORT_INDEX_RTKSOL, rtkSol);
        }
        _receiver[Rover].gnssObs = nullptr;
    }
    else if (!_receiver[Base].e_posMarker.isZero() && _receiver[Base].gnssObs && _receiver[Rover].gnssObs
             && std::chrono::abs(_receiver[Base].gnssObs->insTime - _receiver[Rover].gnssObs->insTime) < std::chrono::milliseconds(100))
    {
        calcRealTimeKinematicSolution();
    }
}

void RealTimeKinematic::assignSolutionToFilter(const std::shared_ptr<NAV::SppSolution>& sppSol)
{
    _lastUpdate = sppSol->insTime;
    _receiver[Rover].e_posMarker = sppSol->e_position();
    _receiver[Rover].lla_posMarker = trafo::ecef2lla_WGS84(_receiver[Rover].e_posMarker);
    if (!sppSol->e_velocity().hasNaN())
    {
        _receiver[Rover].e_vel = sppSol->e_velocity();
    }

    _kalmanFilter.x.segment<3>(States::Pos) = _receiver[Rover].e_posMarker;
    _kalmanFilter.x.segment<3>(States::Vel) = _receiver[Rover].e_vel;

    if (sppSol->e_CovarianceMatrix().has_value())
    {
        if (sppSol->e_CovarianceMatrix()->get().hasRows(Keys::PosVel<Keys::MotionModelKey>))
        {
            _kalmanFilter.P(States::PosVel, States::PosVel) = (*sppSol->e_CovarianceMatrix())(Keys::PosVel<Keys::MotionModelKey>, Keys::PosVel<Keys::MotionModelKey>);
        }
        else
        {
            _kalmanFilter.P(States::Pos, States::Pos) = (*sppSol->e_CovarianceMatrix())(Keys::Pos<Keys::MotionModelKey>, Keys::Pos<Keys::MotionModelKey>);
            _kalmanFilter.P(States::Vel, States::Vel) = Eigen::DiagonalMatrix<double, 3>(1, 1, 1);
        }
    }
}

void RealTimeKinematic::calcRealTimeKinematicSolution()
{
    auto dt = _lastUpdate.empty() ? 0.0 : static_cast<double>((_receiver[Rover].gnssObs->insTime - _lastUpdate).count());
    LOG_DATA("{}: Calculate RTK Solution for rover [{}], base [{}] (dt = {:.2f})", nameId(), _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), _receiver[Base].gnssObs->insTime.toYMDHMS(GPST), dt);

    // Collection of all connected navigation data providers
    std::vector<InputPin::IncomingLink::ValueWrapper<GnssNavInfo>> gnssNavInfoWrappers;
    std::vector<const GnssNavInfo*> gnssNavInfos;
    for (size_t i = 0; i < _dynamicInputPins.getNumberOfDynamicPins(); i++)
    {
        if (auto gnssNavInfo = getInputValue<GnssNavInfo>(INPUT_PORT_INDEX_GNSS_NAV_INFO + i))
        {
            gnssNavInfoWrappers.push_back(*gnssNavInfo);
            gnssNavInfos.push_back(gnssNavInfo->v);
        }
    }
    if (!gnssNavInfos.empty())
    {
        auto rtkSol = std::make_shared<RtkSolution>();
        rtkSol->insTime = _receiver[Rover].gnssObs->insTime;
        rtkSol->baseTime = _receiver[Base].gnssObs->insTime;

        auto nSolutions = static_cast<double>(nFixSolutions + nFloatSolutions + nSingleSolutions);
        bool startupPhase = dt == 0.0 ? true : (nSolutions * dt < 60.0 || nSolutions < 10.0);

        // 𝜎²_a Variance of the acceleration due to user motion in horizontal and vertical component in [m^2 / s^3]
        {
            std::array<double, 2> varAccel{};
            switch (_gui_stdevAccelUnits)
            {
            case StdevAccelUnits::m_sqrts3: // [m / √(s^3)]
                varAccel = { std::pow(_gui_stdevAccel.at(0), 2), std::pow(_gui_stdevAccel.at(1), 2) };
                break;
            }
            if (startupPhase)
            {
                _kalmanFilter.W.block<3>(States::Vel, States::Vel).diagonal() << std::max(1e-6, varAccel.at(0)), std::max(1e-6, varAccel.at(0)), std::max(1e-6, varAccel.at(1));
            }
            else
            {
                _kalmanFilter.W.block<3>(States::Vel, States::Vel).diagonal() << varAccel.at(0), varAccel.at(0), varAccel.at(1);
            }
        }

        if (!startupPhase && !_receiver[Rover].e_posMarker.isZero())
        {
            if (_kalmanFilter.x.rows() > States::KFStates_COUNT && dt > 3.0 * _dataInterval)
            {
                _ambiguitiesHold.clear();
                std::vector<States::StateKeyType> ambKeys;
                for (size_t i = States::KFStates_COUNT; i < _kalmanFilter.x.rowKeys().size(); i++) // 0-2 Pos, 3-5 Vel
                {
                    if (const auto* ambDD = std::get_if<States::AmbiguityDD>(&_kalmanFilter.x.rowKeys().at(i)))
                    {
                        ambKeys.emplace_back(*ambDD);
                    }
                }
                _kalmanFilter.removeStates(ambKeys);

                std::string text = fmt::format("Outage of {:.1f} [s] (interval {:.1f} [s]). Resetting ambiguities.", dt, _dataInterval);
                LOG_WARN("{}: [{}] {}", nameId(), _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), text);
                addEventToGui(rtkSol, text);
            }
        }

        // Calculate a Single point solution if the Rover Position is not known yet
        if (_receiver[Rover].e_posMarker.isZero())
        {
            if (auto sol = _sppAlgorithm.calcSppSolution(_receiver[Rover].gnssObs, gnssNavInfos, nameId()))
            {
                assignSolutionToFilter(sol);

                LOG_DEBUG("{}: Initial base  position: {}°, {}°, {}m (ECEF {} {} {} [m])", nameId(),
                          rad2deg(_receiver[Base].lla_posMarker(0)), rad2deg(_receiver[Base].lla_posMarker(1)), _receiver[Base].lla_posMarker(2),
                          _receiver[Base].e_posMarker.x(), _receiver[Base].e_posMarker.y(), _receiver[Base].e_posMarker.z());
                LOG_DEBUG("{}: Initial rover position: {}°, {}°, {}m (ECEF {} {} {} [m])", nameId(),
                          rad2deg(_receiver[Rover].lla_posMarker(0)), rad2deg(_receiver[Rover].lla_posMarker(1)), _receiver[Rover].lla_posMarker(2),
                          _receiver[Rover].e_posMarker.x(), _receiver[Rover].e_posMarker.y(), _receiver[Rover].e_posMarker.z());
                LOG_DEBUG("{}: Initial rover velocity: {} [m/s] (ECEF)", nameId(), _receiver[Rover].e_vel.transpose());
            }
            else
            {
                LOG_DATA("{}: Could not calculate initial position solution.", nameId());
                return;
            }
        }

        if (_outlierMaxPosVarStartupTriggered) { startupPhase = false; }
        else if (startupPhase)
        {
            if (double posVar = _kalmanFilter.P(States::Pos, States::Pos).diagonal().sum() / 3.0;
                posVar < _outlierMaxPosVarStartup)
            {
                _outlierMaxPosVarStartupTriggered = true;
                addEventToGui(rtkSol, fmt::format("Doing NIS check from now on because\nposition variance is below {:.4f}m^2", _outlierMaxPosVarStartup));
            }
        }

        // Collection of all connected Ionospheric Corrections
        auto ionosphericCorrections = std::make_shared<const IonosphericCorrections>(gnssNavInfos);

        kalmanFilterPrediction();

        {
            auto logLevel = spdlog::get_level();
            spdlog::set_level(spdlog::level::debug);
            // Calculate all possible observations here, to put into solution for analysis
            ObservationFilter obsFilter{ ReceiverType::ReceiverType_COUNT };
            obsFilter.disableFilter();

            Observations observations;
            obsFilter.selectObservationsForCalculation(Rover,
                                                       _receiver[Rover].e_posMarker,
                                                       _receiver[Rover].lla_posMarker,
                                                       _receiver[Rover].gnssObs,
                                                       gnssNavInfos,
                                                       observations, nullptr, nameId() + " no filter");
            obsFilter.selectObservationsForCalculation(Base,
                                                       _receiver[Base].e_posMarker,
                                                       _receiver[Base].lla_posMarker,
                                                       _receiver[Base].gnssObs,
                                                       gnssNavInfos,
                                                       observations, nullptr, nameId() + " no filter");
            removeSingleObservations(observations, nullptr, nullptr);

            for (const auto& [satSigId, sigObs] : observations.signals)
            {
                auto satId = satSigId.toSatId();
                if (std::ranges::find_if(rtkSol->satData, [&satId](const auto& sat) {
                        return satId == sat.first;
                    })
                    != rtkSol->satData.end())
                {
                    continue;
                }
                rtkSol->satData.emplace_back(satSigId.toSatId(),
                                             RtkSolution::SatData{ .satElevation = sigObs.recvObs.at(Rover)->satElevation(_receiver[Rover].e_posMarker, _receiver[Rover].lla_posMarker),
                                                                   .satAzimuth = sigObs.recvObs.at(Rover)->satElevation(_receiver[Rover].e_posMarker, _receiver[Rover].lla_posMarker) });
            }
            for (const auto& [satSigId, sigObs] : observations.signals)
            {
                for (const auto& obs : sigObs.recvObs.at(Rover)->obs)
                {
                    rtkSol->observableReceived.emplace(satSigId, obs.first);
                }
            }
            spdlog::set_level(logLevel);
        }

        Observations observations;
        ObservationFilter::Filtered filtered;
        _obsFilter.selectObservationsForCalculation(Rover,
                                                    _receiver[Rover].e_posMarker,
                                                    _receiver[Rover].lla_posMarker,
                                                    _receiver[Rover].gnssObs,
                                                    gnssNavInfos,
                                                    observations, &filtered, nameId());
        _obsFilter.selectObservationsForCalculation(Base,
                                                    _receiver[Base].e_posMarker,
                                                    _receiver[Base].lla_posMarker,
                                                    _receiver[Base].gnssObs,
                                                    gnssNavInfos,
                                                    observations, &filtered, nameId());
        removeSingleObservations(observations, &filtered, rtkSol);
        rtkSol->filtered = filtered;

        for (const auto& [satSigId, sigObs] : observations.signals)
        {
            for (const auto& obs : sigObs.recvObs.at(Rover)->obs)
            {
                rtkSol->observableFiltered.emplace(satSigId, obs.first);
            }
        }
        rtkSol->solType = RtkSolution::SolutionType::Predicted;

        if (!observations.signals.empty() && observations.satellites.size() > 1)
        {
            _obsEstimator.calcObservationEstimates(observations, _receiver[Rover], ionosphericCorrections, nameId(), ObservationEstimator::DoubleDifference);
            _obsEstimator.calcObservationEstimates(observations, _receiver[Base], ionosphericCorrections, nameId(), ObservationEstimator::DoubleDifference);

            checkForCycleSlip(observations, rtkSol);
            updatePivotSatellites(observations, rtkSol);
            addOrRemoveKalmanFilterAmbiguities(observations, rtkSol);

            printObservations(observations);

            bool nisEnabled = !startupPhase && _kalmanFilter.isNISenabled();
            size_t maxIter = nisEnabled ? (_maxRemoveOutlier == 0 ? 1000 : _maxRemoveOutlier) : 1;

            bool pivotChanged = true;
            Differences singleDifferences;
            Differences doubleDifferences;
            unordered_map<GnssObs::ObservationType, KeyedMatrixXd<RTK::Meas::SingleObs<RealTimeKinematic::ReceiverType>>> Rtilde;

            for (size_t i = 0; i < maxIter; i++)
            {
                LOG_DATA("{}: Iteration: {}/{}", nameId(), i, maxIter);

                if (pivotChanged)
                {
                    singleDifferences = calcSingleDifferences(observations);
                    doubleDifferences = calcDoubleDifferences(observations, singleDifferences);

                    Rtilde = calcSingleObsMeasurementNoiseMatrices(observations);
                    calcKalmanUpdateMatrices(observations, doubleDifferences, Rtilde);
                    pivotChanged = false;
                }

                if (nisEnabled && !startupPhase)
                {
                    LOG_DATA("{}: Doing NIS check", nameId());
                    auto nisResult = _kalmanFilter.checkForOutliersNIS(nameId());
                    if (!rtkSol->nisResultInitial) { rtkSol->nisResultInitial = nisResult; }
                    rtkSol->nisResultFinal = nisResult;
                    if (nisResult.triggered && observations.satellites.size() <= _outlierMinSat)
                    {
                        nisResult.triggered = false;
                        addEventToGui(rtkSol, "Stopped doing NIS check, as minimum amount of satellites reached.");
                    }
                    if (nisResult.triggered)
                    {
                        LOG_DATA("{}: NIS triggered", nameId());

                        if (auto outliers = removeOutlier(observations, rtkSol);
                            !outliers.empty())
                        {
                            removeSingleObservations(observations, &rtkSol->filtered, rtkSol);
                            for (const auto& outlier : outliers)
                            {
                                pivotChanged |= outlier.pivot.has_value();
                            }
                        }
                        else
                        {
                            LOG_DATA("{}: NIS check finished as nothing removed", nameId());
                            break;
                        }
                    }
                    else
                    {
                        LOG_DATA("{}: NIS check successful", nameId());
                        break;
                    }
                }
            }
            if (nisEnabled)
            {
                LOG_DATA("{}: Final NIS = {} < {} = r2. {} removed", nameId(), rtkSol->nisResultFinal->NIS, rtkSol->nisResultFinal->r2, rtkSol->nisRemovedCnt);
                observations.recalcObservableCounts(nameId());
            }

            if (kalmanFilterUpdate(rtkSol).valid)
            {
                if (resolveAmbiguities(observations.nObservablesUniqueSatellite.at(GnssObs::Carrier), rtkSol))
                {
                    rtkSol->solType = RtkSolution::SolutionType::RTK_Fixed;
                    nFixSolutions++;
                }
                else
                {
                    rtkSol->solType = RtkSolution::SolutionType::RTK_Float;
                    nFloatSolutions++;
                }
            }

            rtkSol->measInnovation = _kalmanFilter.z;
        }
        else
        {
            LOG_TRACE("{}: No observations found. Check that your base and rover file match and reconfigure the filter.", nameId());
        }

        _lastSolutionStatus = rtkSol->solType;

        // Write out results
        for (size_t obsType = 0; obsType < GnssObs::ObservationType_COUNT; obsType++)
        {
            rtkSol->nObservations.emplace(static_cast<GnssObs::ObservationType>(obsType), observations.nObservables.at(obsType));
            rtkSol->nObservationsUniqueSatellite.emplace(static_cast<GnssObs::ObservationType>(obsType), observations.nObservablesUniqueSatellite.at(obsType));
        }
        rtkSol->nSatellites = observations.satellites.size();
        rtkSol->distanceBaseRover = (_receiver[Base].e_posMarker - _receiver[Rover].e_posMarker).norm();

        rtkSol->setPosVelAndCov_e(_receiver[Rover].e_posMarker, _receiver[Rover].e_vel,
                                  _kalmanFilter.P.block<6>(States::PosVel, States::PosVel));
        if (rtkSol->solType == RtkSolution::SolutionType::RTK_Fixed || rtkSol->solType == RtkSolution::SolutionType::RTK_Float)
        {
            rtkSol->ambiguityDD_br.reserve(_kalmanFilter.x.rowKeys().size() - States::KFStates_COUNT);
            for (size_t i = States::KFStates_COUNT; i < _kalmanFilter.x.rowKeys().size(); i++) // 0-2 Pos, 3-5 Vel
            {
                if (const auto* ambDD = std::get_if<States::AmbiguityDD>(&_kalmanFilter.x.rowKeys().at(i)))
                {
                    const auto& key = States::AmbiguityDD(ambDD->satSigId);
                    LOG_DATA("{}: Key: {}", nameId(), key);
                    auto ambDDSol = RtkSolution::AmbiguityDD{
                        .pivotSatSigId = _pivotSatellites.at({ ambDD->satSigId.code, GnssObs::Carrier }).satSigId,
                        .satSigId = ambDD->satSigId,
                        .value = UncertainValue<double>{ .value = _kalmanFilter.x(key),
                                                         .stdDev = std::sqrt(_kalmanFilter.P(key, key)) }
                    };
                    rtkSol->ambiguityDD_br.push_back(ambDDSol);
                }
            }
            for (const auto& pivot : _pivotSatellites)
            {
                rtkSol->pivots.emplace(pivot.second.satSigId, pivot.first.second);
            }
            for (const auto& [satSigId, sigObs] : observations.signals)
            {
                for (const auto& obs : sigObs.recvObs.at(Rover)->obs)
                {
                    rtkSol->observableUsed.emplace(satSigId, obs.first);
                }
            }
        }

        if (dt > 1e-3) { _dataInterval = dt; }
        _lastUpdate = rtkSol->insTime;
        invokeCallbacks(OUTPUT_PORT_INDEX_RTKSOL, rtkSol);
    }

    _receiver[Rover].gnssObs = nullptr;
    _baseObsReceivedThisEpoch = false;
}

std::shared_ptr<RtkSolution> RealTimeKinematic::calcFallbackSppSolution()
{
    LOG_DATA("{}: Calculate SPP Solution for [{}] (fallback)", nameId(), _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST));

    // Collection of all connected navigation data providers
    std::vector<InputPin::IncomingLink::ValueWrapper<GnssNavInfo>> gnssNavInfoWrappers;
    std::vector<const GnssNavInfo*> gnssNavInfos;
    for (size_t i = 0; i < _dynamicInputPins.getNumberOfDynamicPins(); i++)
    {
        if (auto gnssNavInfo = getInputValue<GnssNavInfo>(INPUT_PORT_INDEX_GNSS_NAV_INFO + i))
        {
            gnssNavInfoWrappers.push_back(*gnssNavInfo);
            gnssNavInfos.push_back(gnssNavInfo->v);
        }
    }

    if (auto sppSol = _sppAlgorithm.calcSppSolution(_receiver[Rover].gnssObs, gnssNavInfos, nameId()))
    {
        auto rtkSol = std::make_shared<RtkSolution>();
        rtkSol->insTime = sppSol->insTime;
        rtkSol->solType = RtkSolution::SolutionType::SPP;
        rtkSol->nSatellites = sppSol->nSatellites;
        if (!_receiver[Base].e_posMarker.isZero())
        {
            rtkSol->distanceBaseRover = (_receiver[Base].e_posMarker - _receiver[Rover].e_posMarker).norm();
        }
        rtkSol->nObservations[GnssObs::Pseudorange] = sppSol->nMeasPsr;
        rtkSol->nObservations[GnssObs::Doppler] = sppSol->nMeasDopp;

        if (sppSol->e_CovarianceMatrix().has_value())
        {
            if (sppSol->e_CovarianceMatrix()->get().hasAnyRows(Keys::Vel<Keys::MotionModelKey>))
            {
                rtkSol->setPosVelAndCov_e(sppSol->e_position(), sppSol->e_velocity(),
                                          (*sppSol->e_CovarianceMatrix())(Keys::PosVel<Keys::MotionModelKey>, Keys::PosVel<Keys::MotionModelKey>));
            }
            else
            {
                rtkSol->setPositionAndCov_e(sppSol->e_position(),
                                            (*sppSol->e_CovarianceMatrix())(Keys::Pos<Keys::MotionModelKey>, Keys::Pos<Keys::MotionModelKey>));
            }
        }
        else
        {
            rtkSol->setPosition_e(sppSol->e_position());
            rtkSol->setVelocity_e(sppSol->e_velocity());
        }

        assignSolutionToFilter(sppSol);

        return rtkSol;
    }

    return nullptr;
}

void RealTimeKinematic::kalmanFilterPrediction()
{
    // Update the State transition matrix (𝚽) and the Process noise covariance matrix (𝐐)

    auto dt = static_cast<double>((_receiver[Rover].gnssObs->insTime - _lastUpdate).count());
    LOG_DATA("{}: dt = {}s", nameId(), dt);

    _kalmanFilter.G.block<3>(States::Vel, States::Vel) = trafo::e_Quat_n(_receiver[Rover].lla_posMarker(0), _receiver[Rover].lla_posMarker(1)).toRotationMatrix();

    LOG_DATA("{}: F =\n{}", nameId(), _kalmanFilter.F);
    LOG_DATA("{}: G =\n{}", nameId(), _kalmanFilter.G);
    LOG_DATA("{}: W =\n{}", nameId(), _kalmanFilter.W);
    LOG_DATA("{}: GWG^T =\n{}", nameId(),
             KeyedMatrixXd<States::StateKeyType>(_kalmanFilter.G(all, all)
                                                     * _kalmanFilter.W(all, all)
                                                     * _kalmanFilter.G(all, all).transpose(),
                                                 _kalmanFilter.G.rowKeys()));

    auto [Phi, Q] = calcPhiAndQWithVanLoanMethod(_kalmanFilter.F.block<6>(States::PosVel, States::PosVel),
                                                 _kalmanFilter.G.block<6>(States::PosVel, States::PosVel),
                                                 _kalmanFilter.W.block<6>(States::PosVel, States::PosVel),
                                                 dt);
    _kalmanFilter.Phi.block<6>(States::PosVel, States::PosVel) = Phi;
    _kalmanFilter.Q.block<6>(States::PosVel, States::PosVel) = Q;

    auto ambNoiseVar = _lastSolutionStatus == RtkSolution::SolutionType::RTK_Fixed ? _ambiguityFixProcessNoiseVariance : _ambiguityFloatProcessNoiseVariance;
    for (size_t i = States::KFStates_COUNT; i < _kalmanFilter.x.rowKeys().size(); i++) // 0-2 Pos, 3-5 Vel
    {
        if (const auto* ambDD = std::get_if<States::AmbiguityDD>(&_kalmanFilter.x.rowKeys().at(i)))
        {
            const auto& key = States::AmbiguityDD(ambDD->satSigId);

            _kalmanFilter.W(key, key) = ambNoiseVar;
            _kalmanFilter.Q(key, key) = ambNoiseVar * dt;
        }
    }

    LOG_DATA("{}: Phi =\n{}", nameId(), _kalmanFilter.Phi);
    LOG_DATA("{}: Q =\n{}", nameId(), _kalmanFilter.Q);

    LOG_DATA("{}: P (a posteriori, t-1 = {}) =\n{}", nameId(), _lastUpdate, _kalmanFilter.P);
    LOG_DATA("{}: x (a posteriori, t-1 = {}) =\n{}", nameId(), _lastUpdate, _kalmanFilter.x.transposed());
    _kalmanFilter.predict();
    LOG_DATA("{}: Doing KF prediction", nameId());
    LOG_DATA("{}: x (a priori    , t   = {}) =\n{}", nameId(), _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), _kalmanFilter.x.transposed());
    LOG_DATA("{}: P (a priori    , t   = {}) =\n{}", nameId(), _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), _kalmanFilter.P);
    _receiver[Rover].e_posMarker = _kalmanFilter.x.segment<3>(States::Pos);
    _receiver[Rover].e_vel = _kalmanFilter.x.segment<3>(States::Vel);
    _receiver[Rover].lla_posMarker = trafo::ecef2lla_WGS84(_receiver[Rover].e_posMarker);
}

void RealTimeKinematic::checkForCycleSlip(Observations& observations, const std::shared_ptr<RtkSolution>& rtkSol)
{
    if (std::ranges::none_of(_cycleSlipDetector, [](const CycleSlipDetector& detector) {
            return detector.isEnabled(CycleSlipDetector::Detector::LLI)
                   || detector.isEnabled(CycleSlipDetector::Detector::SingleFrequency)
                   || detector.isEnabled(CycleSlipDetector::Detector::DualFrequency);
        }))
    {
        return;
    }
    LOG_DATA("{}: Checking for cycle-slips", nameId());

    std::array<std::vector<CycleSlipDetector::SatelliteObservation>, ReceiverType_COUNT> cycleSlipObservations;

    for (const auto& satId : observations.satellites)
    {
        bool added = false;
        for (const auto& [satSigId, signalObs] : observations.signals) // Signals
        {
            if (satSigId.toSatId() != satId) { continue; }

            for (size_t r = 0; r < ReceiverType_COUNT; r++) // ReceiverType
            {
                auto recvType = static_cast<ReceiverType>(r);

                if (recvType == Base && !_baseObsReceivedThisEpoch) { continue; } // Process base observation only in epoch where received

                if (auto detector = _cycleSlipDetector.at(recvType);
                    !detector.isEnabled(CycleSlipDetector::Detector::LLI)
                    && !detector.isEnabled(CycleSlipDetector::Detector::SingleFrequency)
                    && !detector.isEnabled(CycleSlipDetector::Detector::DualFrequency))
                {
                    continue;
                }

                for (const auto& receiverObs : _receiver.at(recvType).gnssObs->data) // ObservationType
                {
                    if (receiverObs.satSigId.toSatId() == satId && receiverObs.carrierPhase)
                    {
                        [[maybe_unused]] auto lambda = InsConst::C / receiverObs.satSigId.freq().getFrequency(signalObs.freqNum());
                        LOG_DATA("[{}] [{}] Added to cycle-slip detection: {:.1f} [m]", receiverObs.satSigId, recvType, receiverObs.carrierPhase->value * lambda);

                        GnssObs::ObservationData::CarrierPhase carrier{ .value = receiverObs.carrierPhase->value, .LLI = receiverObs.carrierPhase->LLI };
                        auto signal = CycleSlipDetector::SatelliteObservation::Signal{ .code = receiverObs.satSigId.code, .measurement = carrier };

                        auto sat = std::ranges::find_if(cycleSlipObservations.at(recvType),
                                                        [&satId](const auto& satObs) { return satObs.satId == satId; });

                        if (sat == cycleSlipObservations.at(recvType).end())
                        {
                            CycleSlipDetector::SatelliteObservation satObs = { .satId = satId,
                                                                               .signals = { signal },
                                                                               .freqNum = signalObs.freqNum() };
                            cycleSlipObservations.at(recvType).push_back(satObs);
                        }
                        else
                        {
                            sat->signals.push_back(signal);
                        }
                        added = true;
                    }
                }
            }
            if (added) { break; }
        }
    }

    std::vector<NAV::Code> pivotsToChange;
    std::vector<States::AmbiguityDD> removedStates;
    auto removeSatSig = [&](const SatSigId& satSigId, [[maybe_unused]] const std::string& reason) -> bool {
        auto key = States::AmbiguityDD(satSigId);
        if (_kalmanFilter.x.hasRow(key))
        {
            LOG_TRACE("{}: [{}] Cycle-slip detected in [{}], due to {}. Removing state: {}", nameId(),
                      _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), satSigId, reason, key);
            _ambiguitiesHold.erase(satSigId);
            _kalmanFilter.removeState(key);
            if (_outputStateEvents) { removedStates.push_back(key); }
            for (auto& detector : _cycleSlipDetector) { detector.resetSignal(key.satSigId); }
        }
        else if (_pivotSatellites.contains({ satSigId.code, GnssObs::Carrier }) && _pivotSatellites.at({ satSigId.code, GnssObs::Carrier }).satSigId == satSigId)
        {
            LOG_TRACE("{}: [{}] Cycle-slip detected in pivot satellite [{}], due to {}", nameId(),
                      _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), satSigId, reason);
            pivotsToChange.push_back(satSigId.code);
        }
        else
        {
            LOG_DATA("{}: [{}] Cycle-slip detected in [{}], due to {}. Not yet estimated (no action taken).", nameId(),
                     _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), satSigId, reason);
            return false;
        }
        return true;
    };

    for (size_t i = 0; i < cycleSlipObservations.size(); ++i)
    {
        std::string receiverName = fmt::format("{}", static_cast<ReceiverType>(i));
        LOG_DATA("{}: [{}][{}] Checking for cycle-slips", nameId(), _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), receiverName);

        auto cycleSlips = _cycleSlipDetector.at(i).checkForCycleSlip(_receiver.at(i).gnssObs->insTime, cycleSlipObservations.at(i), nameId());
        for (const auto& cycleSlip : cycleSlips)
        {
            if (const auto* s = std::get_if<CycleSlipDetector::CycleSlipLossOfLockIndicator>(&cycleSlip))
            {
                if (s->signal.code & _obsFilter.getCodeFilter()
                    && removeSatSig(s->signal, "LLI set in " + receiverName))
                {
                    rtkSol->cycleSlipDetectorResult.emplace_back(cycleSlip, receiverName);
                }
            }
            else if (const auto* s = std::get_if<CycleSlipDetector::CycleSlipSingleFrequency>(&cycleSlip))
            {
                if (s->signal.code & _obsFilter.getCodeFilter()
                    && removeSatSig(s->signal, "single frequency check in " + receiverName))
                {
                    rtkSol->cycleSlipDetectorResult.emplace_back(cycleSlip, receiverName);
                }
            }
            else if (const auto* s = std::get_if<CycleSlipDetector::CycleSlipDualFrequency>(&cycleSlip))
            {
                bool first = true;
                for (const auto& signal : s->signals)
                {
                    if (signal.code & _obsFilter.getCodeFilter()
                        && removeSatSig(signal, "dual frequency check in " + receiverName)
                        && first)
                    {
                        rtkSol->cycleSlipDetectorResult.emplace_back(cycleSlip, receiverName);
                        first = false;
                    }
                }
            }
        }
    }

    for (const auto& code : pivotsToChange)
    {
        LOG_TRACE("{}: [{}] Changing pivot because of cycle-slip", nameId(), _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), code);
        updatePivotSatellite(code, GnssObs::Carrier, observations, rtkSol, RtkSolution::PivotChange::Reason::PivotCycleSlip);
    }

    for (const auto& cycleSlip : rtkSol->cycleSlipDetectorResult)
    {
        switch (cycleSlip.first.index())
        {
        case 0: // CycleSlipLossOfLockIndicator
            _nCycleSlipsLLI++;
            break;
        case 1: // CycleSlipDualFrequency
            _nCycleSlipsDual++;
            break;
        case 2: // CycleSlipSingleFrequency
            _nCycleSlipsSingle++;
            break;
        default:
            break;
        }
        auto text = fmt::format("{}: {}", cycleSlip.second, cycleSlip.first);
        LOG_DATA("{}: {}", nameId(), text);
        addEventToGui(rtkSol, text);
    }
    for (const auto& state : removedStates)
    {
        auto text = fmt::format("State [{}] removed, because cycle-slip detected.", state);
        LOG_DATA("{}: {}", nameId(), text);
        addEventToGui(rtkSol, text);
    }
}

void RealTimeKinematic::removeSingleObservations(Observations& observations, ObservationFilter::Filtered* filtered, const std::shared_ptr<RtkSolution>& rtkSol)
{
    LOG_DATA("{}: Checking for observations to remove when only one observable per code for double difference", nameId());

    std::unordered_map<std::pair<Code, GnssObs::ObservationType>, uint16_t> signalCount;
    for (const auto& [satSigId, sigObs] : observations.signals)
    {
        const auto& recvObs = sigObs.recvObs.at(Base);
        for (const auto& obs : recvObs->obs)
        {
            signalCount[std::make_pair(satSigId.code, obs.first)]++;
        }
    }

    std::vector<SatSigId> sigToRemove;
    for (auto& [satSigId, sigObs] : observations.signals)
    {
        for (auto& recvObs : sigObs.recvObs)
        {
            std::vector<GnssObs::ObservationType> obsToRemove;
            for (const auto& obs : recvObs.second->obs)
            {
                if (signalCount[std::make_pair(satSigId.code, obs.first)] < 2)
                {
                    obsToRemove.push_back(obs.first);
                }
            }
            for (const GnssObs::ObservationType& obs : obsToRemove)
            {
                LOG_DATA("{}:  Removing [{}] {}", nameId(), satSigId, obs);
                recvObs.second->obs.erase(obs);
                if (filtered) { filtered->singleObservation.push_back(satSigId); }
            }
            if (recvObs.second->obs.empty())
            {
                sigToRemove.push_back(satSigId);
                break;
            }
        }
    }

    for (const auto& satSigId : sigToRemove)
    {
        LOG_DATA("{}:  Removing [{}], because no more observations left for this signal", nameId(), satSigId);

        if (filtered) // Only update pivot if called with filtered (means we modifying the actual observation data)
        {
            for (const auto& obsType : _obsFilter.getUsedObservationTypes())
            {
                auto key = std::make_pair(satSigId.code, obsType);
                if (_pivotSatellites.contains(key) && signalCount[key] < 2)
                {
                    LOG_DATA("{}:  Pivot [{}][{}] does not have enough signals anymore", nameId(), satSigId.code, obsType);
                    updatePivotSatellite(satSigId.code, obsType, observations, rtkSol, RtkSolution::PivotChange::Reason::PivotNotObservedInEpoch);
                }
            }
        }

        observations.removeSignal(satSigId, nameId());
    }
}

std::optional<RtkSolution::PivotChange> RealTimeKinematic::updatePivotSatellite(Code code, GnssObs::ObservationType obsType,
                                                                                Observations& observations, const std::shared_ptr<RtkSolution>& rtkSol,
                                                                                const RtkSolution::PivotChange::Reason& reason)
{
    using Reason = RtkSolution::PivotChange::Reason;
    std::optional<RtkSolution::PivotChange> pivotChange;
    if (reason != Reason::None)
    {
        pivotChange = RtkSolution::PivotChange{};
        pivotChange->reason = reason;
        pivotChange->obsType = obsType;

        if (_pivotSatellites.contains({ code, obsType }))
        {
            const auto& pivot = _pivotSatellites.at({ code, obsType });
            if (observations.signals.contains(pivot.satSigId))
            {
                auto oldPivotObs = observations.signals.at(pivot.satSigId).recvObs.begin();
                pivotChange->oldPivotSat = pivot.satSigId;
                pivotChange->oldPivotElevation = oldPivotObs->second->satElevation(_receiver[Rover].e_posMarker, _receiver[Rover].lla_posMarker);
            }
            _pivotSatellites.erase({ code, obsType });
        }
    }

    auto sumRecvElevation = [&](double el, const auto& recvData) {
        return el + recvData.second->satElevation(_receiver[Rover].e_posMarker, _receiver[Rover].lla_posMarker);
    };

    for (const auto& [satSigId, observation] : observations.signals)
    {
        if (satSigId.code != code) { continue; }

        bool anySignalOnThisCodeWasEstimatedYet = std::ranges::any_of(observations.signals,
                                                                      [&](const auto& obs) { return obs.first.code == code
                                                                                                    && _kalmanFilter.x.hasRow(States::AmbiguityDD(obs.first)); });
        bool anySignalOnThisCodeWithFix = std::ranges::any_of(_ambiguitiesHold,
                                                              [&code](const auto& amb) { return amb.first.code == code; });

        LOG_DATA("{}: [{}] Checking   [{}] for new pivot and obsType [{}]",
                 nameId(), _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), satSigId, obsType);

        size_t nFound = 0;
        for (const auto& recvObs : observation.recvObs)
        {
            if (recvObs.second->obs.contains(obsType)) { nFound++; }
        }
        if (nFound != observation.recvObs.size()) { continue; }

#if LOG_LEVEL <= LOG_LEVEL_DATA
        if (obsType == GnssObs::Carrier)
        {
            std::string ambHold;
            ambHold.reserve(_ambiguitiesHold.size() * 17);
            for (const auto& amb : _ambiguitiesHold)
            {
                ambHold += fmt::format("[{} = {}]", amb.first, amb.second);
            }
            LOG_DATA("{}: [{}]   anySignalOnThisCodeWasEstimatedYet = {}, anySignalOnThisCodeWithFix = {}, ambHold = {}",
                     nameId(), _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), anySignalOnThisCodeWasEstimatedYet, anySignalOnThisCodeWithFix, ambHold);
        }
#endif

        // Don't allow same satellite to be pivot again
        if (pivotChange->oldPivotSat == satSigId) { continue; }

        if (_kalmanFilter.x.rows() != States::KFStates_COUNT                                                          // Ignore first epoch, as we need to select a pivot
            && obsType == GnssObs::Carrier                                                                            // Ambiguity do not matter for pseudorange or doppler
            && (anySignalOnThisCodeWasEstimatedYet || _pivotSatellites.contains({ code, GnssObs::Carrier }))          // We have at least one satellite with an estimated ambiguity on this code
            && !_kalmanFilter.x.hasRow(States::AmbiguityDD(satSigId))                                                 // Only consider signals which were estimated once (float solution)
            && _ambiguityResolutionParameters.searchAlgorithm != AmbiguityResolutionParameters::SearchAlgorithm::None // If we estimating ambiguities
            && (anySignalOnThisCodeWithFix || _pivotSatellites.contains({ code, GnssObs::Carrier }))                  // We have at least one satellite with a fix on this code
            && !_ambiguitiesHold.contains(satSigId))                                                                  // Only consider signals, which have a fix
        {
            continue;
        }

        double satElevation = std::accumulate(observation.recvObs.begin(), observation.recvObs.end(), 0.0, sumRecvElevation) // NOLINT(boost-use-ranges,modernize-use-ranges) // There is no ranges::accumulate
                              / static_cast<double>(observation.recvObs.size());

        if (!_pivotSatellites.contains({ code, obsType })) // No pivot for this code yet
        {
            _pivotSatellites.insert(std::make_pair(std::make_pair(code, obsType), satSigId));
            if (!pivotChange)
            {
                LOG_DATA("{}: [{}]   Setting [{}] as new pivot for obsType [{}] (elevation {:.4}°, no previous pivot on this code)", nameId(),
                         _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), satSigId, obsType, rad2deg(satElevation));
                pivotChange = RtkSolution::PivotChange{ .reason = Reason::NewCode,
                                                        .obsType = obsType,
                                                        .oldPivotSat = {},
                                                        .oldPivotElevation = 0.0,
                                                        .newPivotSat = satSigId,
                                                        .newPivotElevation = satElevation };
            }
            else
            {
                LOG_DATA("{}: [{}]   Setting [{}] as new pivot for obsType [{}] (elevation {:.4}°)", nameId(),
                         _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), satSigId, obsType, rad2deg(satElevation));
                pivotChange->newPivotSat = satSigId;
                pivotChange->newPivotElevation = satElevation;
            }
        }
        else // Check if better pivot available
        {
            const auto& pivotSat = _pivotSatellites.at({ code, obsType });

            if (!observations.signals.contains(pivotSat.satSigId)) // Old pivot satellite was not observed this epoch, so we have to choose another one
            {
                LOG_DATA("{}: [{}]   Setting [{}] as new pivot for obsType [{}] (elevation {:.4}°, old pivot [{}] not observed this epoch)", nameId(),
                         _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), satSigId, obsType, rad2deg(satElevation), _pivotSatellites.at({ code, obsType }).satSigId);
                pivotChange = RtkSolution::PivotChange{ .reason = Reason::PivotNotObservedInEpoch,
                                                        .obsType = obsType,
                                                        .oldPivotSat = _pivotSatellites.at({ code, obsType }).satSigId,
                                                        .oldPivotElevation = 0.0,
                                                        .newPivotSat = satSigId,
                                                        .newPivotElevation = satElevation };
                _pivotSatellites.at({ code, obsType }) = PivotSatellite(satSigId);
                continue;
            }

            const auto& pivotObs = observations.signals.at(pivotSat.satSigId);
            double pivElevation = std::accumulate(pivotObs.recvObs.begin(), pivotObs.recvObs.end(), 0.0, sumRecvElevation) // NOLINT(boost-use-ranges,modernize-use-ranges) // There is no ranges::accumulate
                                  / static_cast<double>(pivotObs.recvObs.size());

            if (satElevation > pivElevation)
            {
                LOG_DATA("{}: [{}]   Setting [{}] as new pivot for obsType [{}] (elevation {:.4}°, larger than previous pivot [{}] elevation {:.4}°)", nameId(),
                         _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), satSigId, obsType, rad2deg(satElevation), _pivotSatellites.at({ code, obsType }).satSigId, rad2deg(pivElevation));
                if (!pivotChange)
                {
                    pivotChange = RtkSolution::PivotChange{ .reason = Reason::HigherElevationFound,
                                                            .obsType = obsType,
                                                            .oldPivotSat = _pivotSatellites.at({ code, obsType }).satSigId,
                                                            .oldPivotElevation = pivElevation,
                                                            .newPivotSat = satSigId,
                                                            .newPivotElevation = satElevation };
                }
                else
                {
                    pivotChange->newPivotSat = satSigId;
                    pivotChange->newPivotElevation = satElevation;
                }
                _pivotSatellites.at({ code, obsType }) = PivotSatellite(satSigId);
            }
        }
    }

    // React on changed pivot satellite
    if (pivotChange)
    {
        INS_ASSERT_USER_ERROR(!_pivotSatellites.contains({ code, obsType })
                                  || _pivotSatellites.at({ code, obsType }).satSigId == pivotChange->newPivotSat,
                              "The new pivot satellite and current one must match.");
        INS_ASSERT_USER_ERROR(pivotChange->oldPivotSat != pivotChange->newPivotSat
                                  || pivotChange->newPivotSat == SatSigId{},
                              "Old and new pivot satellite have to be different.");

        LOG_TRACE("{}: [{}] {}", nameId(), _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), *pivotChange, obsType);

        rtkSol->changedPivotSatellites.emplace(std::make_pair(code, obsType), *pivotChange);
        _nPivotChange++;
        addEventToGui(rtkSol, fmt::format("{}", *pivotChange));

        if (pivotChange->newPivotSat == SatSigId{}) // No new pivot selected
        {
            observations.removeMeasurementsFor(pivotChange->oldPivotSat.code, obsType, nameId());
            if (obsType == GnssObs::Carrier)
            {
                for (size_t i = States::KFStates_COUNT; i < _kalmanFilter.x.rowKeys().size(); i++) // 0-2 Pos, 3-5 Vel
                {
                    const auto& key = _kalmanFilter.x.rowKeys().at(i);
                    if (const auto* ambDD = std::get_if<States::AmbiguityDD>(&key))
                    {
                        if (ambDD->satSigId.code != pivotChange->oldPivotSat.code) { continue; }

                        LOG_DATA("{}: [{}] Removing Amibguity as no pivot anymore {}", nameId(), ambDD->satSigId, key);
                        if (_outputStateEvents) { addEventToGui(rtkSol, fmt::format("State [{}] removed (not observed anymore)", key)); }
                        _ambiguitiesHold.erase(ambDD->satSigId);
                        _kalmanFilter.removeState(key); // After this 'key' is invalidated
                        i--;
                    }
                }
            }
        }
        else if (pivotChange->reason != Reason::NewCode && obsType == GnssObs::Carrier)
        {
            updateKalmanFilterAmbiguitiesForPivotChange(pivotChange->newPivotSat, pivotChange->oldPivotSat,
                                                        pivotChange->reason != Reason::PivotNotObservedInEpoch
                                                            && pivotChange->reason != Reason::PivotCycleSlip
                                                            && pivotChange->reason != Reason::PivotOutlier,
                                                        rtkSol);
        }
    }
    return pivotChange;
}

void RealTimeKinematic::printPivotSatellites()
{
    LOG_DATA("{}: Current pivot satellites:", nameId());
    for ([[maybe_unused]] const auto& [key, pivot] : _pivotSatellites)
    {
        LOG_DATA("{}:   [{} - {}]: {}", nameId(), key.first, key.second, pivot.satSigId);
        if (key.second != GnssObs::Carrier) { continue; }
        if (auto state = States::AmbiguityDD{ pivot.satSigId };
            _kalmanFilter.x.hasRow(state))
        {
            LOG_ERROR("{}: [{}] The Kalman Filter has the state [{}] but this is a pivot. Pivots should not be in the state.", nameId(), _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), state);
        }
    }
}

void RealTimeKinematic::updatePivotSatellites(Observations& observations, const std::shared_ptr<RtkSolution>& rtkSol)
{
    printPivotSatellites();

    std::set<Code> codes;
    for (const auto& [satSigId, observation] : observations.signals)
    {
        codes.insert(satSigId.code);
    }
    bool anyPivotChanged = false;
    for (const auto& code : codes)
    {
        for (const auto& obsType : _obsFilter.getUsedObservationTypes())
        {
            auto reason = RtkSolution::PivotChange::Reason::None;

            if (_pivotSatellites.contains({ code, obsType }))
            {
                const auto& pivotSat = _pivotSatellites.at({ code, obsType });
                const auto& pivotSatSigId = pivotSat.satSigId;
                auto obs = std::ranges::find_if(observations.signals, [&](const auto& obs) {
                    size_t nFound = 0;
                    for (const auto& recvObs : obs.second.recvObs)
                    {
                        if (recvObs.second->obs.contains(obsType)) { nFound++; }
                    }
                    return obs.first == pivotSatSigId && nFound == obs.second.recvObs.size();
                });
                if (obs == observations.signals.end())
                {
                    LOG_DATA("{}: [{}] Removing pivot satellite [{}] on observable [{}] because not observed this epoch.", nameId(),
                             _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), pivotSat.satSigId, obsType);
                    reason = RtkSolution::PivotChange::Reason::PivotNotObservedInEpoch;
                }
            }

            anyPivotChanged |= updatePivotSatellite(code, obsType, observations, rtkSol, reason).has_value();
        }
    }

    LOG_DATA("{}: {}", nameId(), anyPivotChanged ? "Pivot satellite changed" : "No pivot satellite change");
    if (anyPivotChanged)
    {
        removeSingleObservations(observations, &rtkSol->filtered, rtkSol);
        printPivotSatellites();
    }
}

void RealTimeKinematic::updateKalmanFilterAmbiguitiesForPivotChange(const SatSigId& newPivotSatSigId, const SatSigId& oldPivotSatSigId,
                                                                    bool oldPivotObservedInEpoch, const std::shared_ptr<RtkSolution>& rtkSol)
{
    auto newPivotKey = States::AmbiguityDD{ newPivotSatSigId };
    if (_kalmanFilter.x.hasRow(newPivotKey))
    {
        std::vector<States::StateKeyType> ambiguitiesToChange;
        for (size_t i = States::KFStates_COUNT; i < _kalmanFilter.x.rowKeys().size(); i++)
        {
            const auto* ambDD = std::get_if<States::AmbiguityDD>(&_kalmanFilter.x.rowKeys().at(i));
            if (ambDD && newPivotSatSigId.code == ambDD->satSigId.code
                && (oldPivotObservedInEpoch || newPivotSatSigId != ambDD->satSigId))
            {
                ambiguitiesToChange.emplace_back(*ambDD);
            }
        }
        LOG_TRACE("{}: [{}] New pivot [{}] adapts ambiguities [{}]", nameId(), _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST),
                  newPivotSatSigId, fmt::join(ambiguitiesToChange, ", "));

        auto nStates = static_cast<int>(_kalmanFilter.x.rowKeys().size());

        KeyedMatrixXd<States::StateKeyType> D(Eigen::MatrixXd::Identity(nStates, nStates),
                                              _kalmanFilter.x.rowKeys(), _kalmanFilter.x.rowKeys());
        D(ambiguitiesToChange, newPivotKey).setConstant(-1.0);
        if (!oldPivotObservedInEpoch) { D(newPivotKey, newPivotKey) = 0.0; }
        LOG_DATA("{}: D = \n{}", nameId(), D);

        LOG_DATA("{}: x_old = \n{}", nameId(), _kalmanFilter.x.transposed());
        _kalmanFilter.x(all) = D(all, all) * _kalmanFilter.x(all);
        LOG_DATA("{}: D * x_old = \n{}", nameId(), _kalmanFilter.x.transposed());

        LOG_DATA("{}: P_old = \n{}", nameId(), _kalmanFilter.P);
        // Error propagation
        _kalmanFilter.P(all, all) = D(all, all) * _kalmanFilter.P(all, all) * D(all, all).transpose();

#if LOG_LEVEL <= LOG_LEVEL_DATA
        {
            std::string ambHold;
            ambHold.reserve(_ambiguitiesHold.size() * 17);
            for (const auto& amb : ambiguitiesToChange)
            {
                if (const auto* ambDD = std::get_if<States::AmbiguityDD>(&amb))
                {
                    if (_ambiguitiesHold.contains(ambDD->satSigId))
                    {
                        ambHold += fmt::format("[{} = {}]", ambDD->satSigId, _ambiguitiesHold.at(ambDD->satSigId));
                    }
                }
            }
            LOG_DATA("{}: ambHold (pre ) = {}", nameId(), ambHold);
        }
#endif

        for (const auto& amb : ambiguitiesToChange)
        {
            if (const auto* ambDD = std::get_if<States::AmbiguityDD>(&amb))
            {
                if (_ambiguitiesHold.contains(ambDD->satSigId))
                {
                    _ambiguitiesHold.at(ambDD->satSigId) = _kalmanFilter.x(amb);
                }
            }
        }

        if (oldPivotObservedInEpoch)
        {
            auto oldPivotKey = States::AmbiguityDD{ oldPivotSatSigId };
            if (_ambiguityResolutionParameters.searchAlgorithm != AmbiguityResolutionParameters::SearchAlgorithm::None
                && _ambiguitiesHold.contains(newPivotSatSigId))
            {
                _ambiguitiesHold[oldPivotSatSigId] = _ambiguitiesHold.at(newPivotSatSigId);
                _ambiguitiesHold.erase(newPivotSatSigId);
            }
            _kalmanFilter.replaceState(newPivotKey, oldPivotKey);
            if (_outputStateEvents)
            {
                addEventToGui(rtkSol, fmt::format("State [{}] replaced with [{}] (because [{}] is new pivot satellite)", newPivotKey, oldPivotKey, newPivotSatSigId));
            }
            LOG_TRACE("{}: [{}] State [{}] replaced with [{}] (because [{}] is new pivot satellite)", nameId(),
                      _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), newPivotKey, oldPivotKey, newPivotSatSigId);
        }
        else
        {
            _ambiguitiesHold.erase(newPivotSatSigId);
            _kalmanFilter.removeState(newPivotKey);
            for (auto& detector : _cycleSlipDetector)
            {
                detector.resetSignal(newPivotSatSigId);
                detector.resetSignal(oldPivotSatSigId);
            }
            if (_outputStateEvents)
            {
                addEventToGui(rtkSol, fmt::format("State [{}] removed (because new pivot satellite)", newPivotKey));
            }
            LOG_TRACE("{}: [{}] State [{}] removed (because new pivot satellite)", nameId(), _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), newPivotKey);
        }
        LOG_DATA("{}: x_new = \n{}", nameId(), _kalmanFilter.x.transposed());
        LOG_DATA("{}: P_new = \n{}", nameId(), _kalmanFilter.P);

#if LOG_LEVEL <= LOG_LEVEL_DATA
        {
            std::erase_if(ambiguitiesToChange, [&newPivotSatSigId](const auto& amb) {
                if (const auto* ambDD = std::get_if<States::AmbiguityDD>(&amb))
                {
                    return ambDD->satSigId == newPivotSatSigId;
                }
                return false;
            });
            if (oldPivotObservedInEpoch)
            {
                ambiguitiesToChange.emplace_back(States::AmbiguityDD{ oldPivotSatSigId });
            }

            std::string ambHold;
            ambHold.reserve(_ambiguitiesHold.size() * 17);
            for (const auto& amb : ambiguitiesToChange)
            {
                if (const auto* ambDD = std::get_if<States::AmbiguityDD>(&amb))
                {
                    if (_ambiguitiesHold.contains(ambDD->satSigId))
                    {
                        ambHold += fmt::format("[{} = {}]", ambDD->satSigId, _ambiguitiesHold.at(ambDD->satSigId));
                    }
                }
            }
            LOG_DATA("{}: ambHold (post) = {}", nameId(), ambHold);
        }
#endif
    }
}

void RealTimeKinematic::addOrRemoveKalmanFilterAmbiguities(const Observations& observations, const std::shared_ptr<RtkSolution>& rtkSol)
{
    bool anyStateChanged = false;

    auto addEvent = [&](const std::string& text) {
        if (_outputStateEvents) { addEventToGui(rtkSol, text); }
    };

    std::vector<States::StateKeyType> newAddedAmbiguities;

    for (const auto& [satSigId, observation] : observations.signals)
    {
        if (!observation.recvObs.at(Rover)->obs.contains(GnssObs::Carrier)
            || !observation.recvObs.at(Base)->obs.contains(GnssObs::Carrier)) { continue; }

        auto key = States::AmbiguityDD{ satSigId };
        if (!_kalmanFilter.x.hasRow(key)                                                      // Ambiguity not in state yet
            && _pivotSatellites.contains({ satSigId.code, GnssObs::Carrier })                 // There should be a pivot satellite for this observation
            && _pivotSatellites.at({ satSigId.code, GnssObs::Carrier }).satSigId != satSigId) // Don't add a ambiguity for pivot satellite
        {
            anyStateChanged = true;
            LOG_TRACE("{}: [{}] Adding state: {}", nameId(), _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), key);
            _kalmanFilter.addState(key);
            newAddedAmbiguities.emplace_back(key);
            if (std::ranges::find(rtkSol->newEstimatedAmbiguity, satSigId)
                == rtkSol->newEstimatedAmbiguity.end())
            {
                rtkSol->newEstimatedAmbiguity.push_back(satSigId);
            }

            // F: Entries are all 0
            // Ambiguities are modeled as RW with very small noise to keep numerical stability
            _kalmanFilter.G(key, key) = 1;
            _kalmanFilter.Phi(key, key) = 1; // This is set, because we only calculate the matrix for PosVel

            _kalmanFilter.W(key, key) = _lastSolutionStatus == RtkSolution::SolutionType::RTK_Fixed ? _ambiguityFixProcessNoiseVariance : _ambiguityFloatProcessNoiseVariance;

            const auto& pivotSat = _pivotSatellites.at({ satSigId.code, GnssObs::Carrier });
            LOG_DATA("{}: {} - pivot {}", nameId(), satSigId, pivotSat.satSigId);
            if (!observations.signals.contains(pivotSat.satSigId))
            {
                LOG_CRITICAL("{}: [{}] The pivot satellite [{}] does not have a carrier measurement. It should have been switched before.",
                             nameId(), _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), pivotSat.satSigId);
            }
            const auto& pivotObs = observations.signals.at(pivotSat.satSigId);

            // Initialize with difference of 1/lambda (carrier-phase_DD - pseudorange_DD) measurement.
            double lambda_j = InsConst::C / satSigId.freq().getFrequency(observation.freqNum());

            // Carrier is always there, otherwise we would not initialize the ambiguity
            const auto& phi_r_1 = pivotObs.recvObs.at(Rover)->obs.at(GnssObs::Carrier);
            const auto& phi_b_1 = pivotObs.recvObs.at(Base)->obs.at(GnssObs::Carrier);
            const auto& phi_r_s = observation.recvObs.at(Rover)->obs.at(GnssObs::Carrier);
            const auto& phi_b_s = observation.recvObs.at(Base)->obs.at(GnssObs::Carrier);

            double phi_br_s_meas = phi_r_s.measurement - phi_b_s.measurement;
            double phi_br_1_meas = phi_r_1.measurement - phi_b_1.measurement;
            double phi_br_1s_meas = phi_br_s_meas - phi_br_1_meas;

            double varCarrierPseudorange = std::numeric_limits<double>::infinity();
            bool forceCarrierPseudorange = false;

            // Pseudorange could be missing
            if (observation.recvObs.at(Rover)->obs.contains(GnssObs::Pseudorange)
                && observation.recvObs.at(Base)->obs.contains(GnssObs::Pseudorange)
                && _pivotSatellites.contains({ satSigId.code, GnssObs::Pseudorange }))
            {
                const auto& pivotSatPsr = _pivotSatellites.at({ satSigId.code, GnssObs::Pseudorange });
                if (pivotSatPsr.satSigId == satSigId) // We cannot initialize the ambiguity with different pivot satellites
                {
                    const auto& pivotObsPsr = observations.signals.at(pivotSatPsr.satSigId);
                    const auto& p_r_1 = pivotObsPsr.recvObs.at(Rover)->obs.at(GnssObs::Pseudorange);
                    const auto& p_b_1 = pivotObsPsr.recvObs.at(Base)->obs.at(GnssObs::Pseudorange);
                    const auto& p_r_s = observation.recvObs.at(Rover)->obs.at(GnssObs::Pseudorange);
                    const auto& p_b_s = observation.recvObs.at(Base)->obs.at(GnssObs::Pseudorange);

                    varCarrierPseudorange = phi_r_s.measVar + p_r_s.measVar + phi_b_s.measVar + p_b_s.measVar
                                            + phi_r_1.measVar + p_r_1.measVar + phi_b_1.measVar + p_b_1.measVar;

                    double posUncertanty = _kalmanFilter.P.block<3>(States::Pos, States::Pos).diagonal().cwiseSqrt().norm();
                    forceCarrierPseudorange = posUncertanty > 0.06; // Always use the Carrier-Pseudorange difference when position is not good (stdDev 5cm)
                }
            }

            Eigen::Vector3d e_pLOS_1s = pivotObs.recvObs.at(Rover)->e_pLOS(_receiver[Rover].e_posMarker)
                                        - observation.recvObs.at(Rover)->e_pLOS(_receiver[Rover].e_posMarker);
            const double varCarrierGeometry = phi_r_s.measVar + phi_b_s.measVar + phi_r_1.measVar + phi_b_1.measVar
                                              + e_pLOS_1s.transpose() * _kalmanFilter.P.block<3>(States::Pos, States::Pos) * e_pLOS_1s;
            LOG_DATA("{}: varCarrierGeometry = {:.2g} [m], varCarrierPseudorange = {:.2g} [m]", nameId(), varCarrierGeometry, varCarrierPseudorange);

            if (varCarrierPseudorange < varCarrierGeometry || forceCarrierPseudorange)
            {
                const auto& p_r_1 = pivotObs.recvObs.at(Rover)->obs.at(GnssObs::Pseudorange);
                const auto& p_b_1 = pivotObs.recvObs.at(Base)->obs.at(GnssObs::Pseudorange);
                const auto& p_r_s = observation.recvObs.at(Rover)->obs.at(GnssObs::Pseudorange);
                const auto& p_b_s = observation.recvObs.at(Base)->obs.at(GnssObs::Pseudorange);

                double p_br_s_meas = p_r_s.measurement - p_b_s.measurement;
                double p_br_1_meas = p_r_1.measurement - p_b_1.measurement;
                double p_br_1s_meas = p_br_s_meas - p_br_1_meas;

                _kalmanFilter.x(key) = (phi_br_1s_meas - p_br_1s_meas) / lambda_j;

                _kalmanFilter.P(key, key) = varCarrierPseudorange * 3.0 / std::pow(lambda_j, 2); // Safety factor of 3 and division by lambda^2, because state in [cycles]
                auto msg = fmt::format("ddCP({:.1f}m) - ddPR({:.1f}m) = {:.1f} [cycles], std = {:.2f} [cycles]",
                                       phi_br_1s_meas, p_br_1s_meas, _kalmanFilter.x(key), std::sqrt(_kalmanFilter.P(key, key)));
                LOG_DATA("{}: [{}] Init Amibguity: {}", nameId(), satSigId, msg);

                addEvent(fmt::format("State [{}] added ({})", key, msg));
            }
            else // Carrier-Geometry Difference
            {
                double phi_br_s_est = phi_r_s.estimate - phi_b_s.estimate;
                double phi_br_1_est = phi_r_1.estimate - phi_b_1.estimate;
                double phi_br_1s_est = phi_br_s_est - phi_br_1_est;

                _kalmanFilter.x(key) = (phi_br_1s_meas - phi_br_1s_est) / lambda_j;

                _kalmanFilter.P(key, key) = varCarrierGeometry * 3.0 / std::pow(lambda_j, 2); // Safety factor of 3 and division by lambda^2, because state in [cycles]
                auto msg = fmt::format("ddCP({:.1f}m) - ddEst({:.1f}m) = {:.1f} [cycles], std = {:.2f} [cycles]",
                                       phi_br_1s_meas, phi_br_1s_est, _kalmanFilter.x(key), std::sqrt(_kalmanFilter.P(key, key)));
                LOG_DATA("{}: [{}] Init Amibguity: {}", nameId(), satSigId, msg);

                addEvent(fmt::format("State [{}] added ({})", key, msg));
            }
        }
    }

    for (size_t i = States::KFStates_COUNT; i < _kalmanFilter.x.rowKeys().size(); i++) // 0-2 Pos, 3-5 Vel
    {
        const auto& key = _kalmanFilter.x.rowKeys().at(i);
        if (const auto* ambDD = std::get_if<States::AmbiguityDD>(&key))
        {
            if (observations.countObservations(ambDD->satSigId, GnssObs::Carrier) == ReceiverType_COUNT) { continue; }

            LOG_DATA("{}: [{}] Removing Ambiguity as not observed anymore {}", nameId(), ambDD->satSigId, key);
            _ambiguitiesHold.erase(ambDD->satSigId);
            addEvent(fmt::format("State [{}] removed (not observed anymore)", key));
            _kalmanFilter.removeState(key); // After this 'key' is invalidated
            i--;
        }
    }

    if (anyStateChanged)
    {
        LOG_DATA("{}: x =\n{}", nameId(), _kalmanFilter.x.transposed());
        LOG_DATA("{}: P =\n{}", nameId(), _kalmanFilter.P);
        LOG_DATA("{}: G =\n{}", nameId(), _kalmanFilter.G);
        LOG_DATA("{}: W =\n{}", nameId(), _kalmanFilter.W);
    }
}

RealTimeKinematic::Differences RealTimeKinematic::calcSingleDifferences(const Observations& observations) const // NOLINT(readability-convert-member-functions-to-static)
{
    Differences singleDifferences;
    singleDifferences.reserve(observations.signals.size());

    LOG_DATA("{}: Calculating single differences (rover - base):", nameId());
    for (const auto& [satSigId, observation] : observations.signals)
    {
        const auto& baseObservations = observation.recvObs.at(Base)->obs;
        const auto& roverObservations = observation.recvObs.at(Rover)->obs;

        for (const auto& [obsType, baseObs] : baseObservations)
        {
            const auto& roverObs = roverObservations.at(obsType);

            singleDifferences[satSigId][obsType].estimate = roverObs.estimate - baseObs.estimate;
            singleDifferences[satSigId].at(obsType).measurement = roverObs.measurement - baseObs.measurement;

            LOG_DATA("{}:   [{}][{:11}][Meas] r {:.3f} - b {:.3f} = {:.3f}", nameId(), satSigId, obsType,
                     roverObs.measurement, baseObs.measurement, singleDifferences[satSigId][obsType].measurement);
            LOG_DATA("{}:   [{}][{:11}][Est ] r {:.3f} - b {:.3f} = {:.3f} (diff to meas {:.3e})", nameId(), satSigId, obsType,
                     roverObs.estimate, baseObs.estimate, singleDifferences[satSigId][obsType].estimate,
                     singleDifferences[satSigId][obsType].measurement - singleDifferences[satSigId][obsType].estimate);
        }
    }

    return singleDifferences;
}

RealTimeKinematic::Differences RealTimeKinematic::calcDoubleDifferences(const Observations& observations, const Differences& singleDifferences) const
{
    Differences doubleDifferences;
    if (singleDifferences.empty()) { return doubleDifferences; }
    doubleDifferences.reserve(singleDifferences.size() - 1);

    LOG_DATA("{}: [{}] Calculating double differences (sat - pivot):", nameId(), _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST));
    for (const auto& [satSigId_s, singleDiff_s] : singleDifferences)
    {
        for (const auto& [obsType, sDiff_s] : singleDiff_s)
        {
            if (!_pivotSatellites.contains({ satSigId_s.code, obsType }))
            {
                LOG_DATA("{}: [{}] Not calculating double difference for [{} {}] because no pivot satellite.", nameId(), _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), satSigId_s.code, obsType);
                continue;
            }
            const auto& satSigId_1 = _pivotSatellites.at({ satSigId_s.code, obsType }).satSigId;
            if (satSigId_s == satSigId_1) { continue; } // No double difference with itself

            if (!singleDifferences.contains(satSigId_1))
            {
                LOG_WARN("{}: [{}] Pivot [{}] has no single difference. This is a bug.", nameId(), _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), satSigId_1);
                continue;
            }
            if (!singleDifferences.at(satSigId_1).contains(obsType))
            {
                LOG_WARN("{}: [{}] Pivot [{}] has no observation type [{}]. This is a bug.", nameId(), _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), satSigId_1, obsType);
                continue;
            }

            const auto& singleDiff_1 = singleDifferences.at(satSigId_1);

            const auto& sDiff_1 = singleDiff_1.at(obsType);

            doubleDifferences[satSigId_s][obsType].estimate = sDiff_s.estimate - sDiff_1.estimate;
            doubleDifferences[satSigId_s].at(obsType).measurement = sDiff_s.measurement - sDiff_1.measurement;
            LOG_DATA("{}:   [{} - {}][{:11}][Meas] sat {:.3f} - piv {:.3f} = {:.3f}", nameId(), satSigId_s, satSigId_1, obsType,
                     sDiff_s.measurement, sDiff_1.measurement, doubleDifferences[satSigId_s][obsType].measurement);

            if (obsType == GnssObs::Carrier) // Pivot satellite ambiguity is 0 and not in state
            {
                double lambda_j_1 = InsConst::C / satSigId_1.freq().getFrequency(observations.signals.at(satSigId_1).freqNum());
                double N_br_1s = _kalmanFilter.x(States::AmbiguityDD{ satSigId_s });
                doubleDifferences[satSigId_s].at(obsType).estimate += lambda_j_1 * N_br_1s;
            }
            LOG_DATA("{}:   [{} - {}][{:11}][Est ] sat {:.3f} - piv {:.3f} = {:.3f} (diff to meas {:.3e})", nameId(), satSigId_s, satSigId_1, obsType,
                     sDiff_s.estimate, sDiff_1.estimate, doubleDifferences[satSigId_s][obsType].estimate,
                     doubleDifferences[satSigId_s][obsType].measurement - doubleDifferences[satSigId_s][obsType].estimate);
        }
    }

    return doubleDifferences;
}

unordered_map<GnssObs::ObservationType, KeyedMatrixXd<RTK::Meas::SingleObs<RealTimeKinematic::ReceiverType>>>
    RealTimeKinematic::calcSingleObsMeasurementNoiseMatrices(const Observations& observations) const // NOLINT(readability-convert-member-functions-to-static)
{
    unordered_map<GnssObs::ObservationType, std::vector<RTK::Meas::SingleObs<ReceiverType>>> measKeys;
    for (const auto& [satSigId, observation] : observations.signals)
    {
        for (const auto& [obsType, baseObs] : observation.recvObs.at(Base)->obs)
        {
            measKeys[obsType].emplace_back(satSigId, Rover, obsType);
            measKeys[obsType].emplace_back(satSigId, Base, obsType);
        }
    }

    unordered_map<GnssObs::ObservationType, KeyedMatrixXd<RTK::Meas::SingleObs<ReceiverType>>> Rtilde;
    for (const auto& [obsType, keys] : measKeys)
    {
        Rtilde[obsType] = KeyedMatrixXd<RTK::Meas::SingleObs<ReceiverType>>(Eigen::MatrixXd::Zero(static_cast<int>(keys.size()), static_cast<int>(keys.size())), keys);
    }

    for (const auto& [satSigId, observation] : observations.signals)
    {
        const auto& baseObservations = observation.recvObs.at(Base)->obs;
        const auto& roverObservations = observation.recvObs.at(Rover)->obs;

        for (const auto& [obsType, baseObs] : baseObservations)
        {
            const auto& roverObs = roverObservations.at(obsType);

            RTK::Meas::SingleObs<ReceiverType> roverKey(satSigId, Rover, obsType);
            RTK::Meas::SingleObs<ReceiverType> baseKey(satSigId, Base, obsType);

            Rtilde.at(obsType)(roverKey, roverKey) = roverObs.measVar;
            Rtilde.at(obsType)(baseKey, baseKey) = baseObs.measVar;
        }
    }

    // for ([[maybe_unused]] const auto& [obsType, R] : Rtilde)
    // {
    //     LOG_DATA("{}: R_tilde({}) =\n{}", nameId(), obsType, R);
    // }

    return Rtilde;
}

void RealTimeKinematic::calcKalmanUpdateMatrices(const Observations& observations, const Differences& doubleDifferences,
                                                 const unordered_map<GnssObs::ObservationType, KeyedMatrixXd<RTK::Meas::SingleObs<RealTimeKinematic::ReceiverType>>>& Rtilde)
{
    // Update the Measurement sensitivity Matrix (𝐇), the Measurement noise covariance matrix (𝐑) and the Measurement vector (𝐳)

    std::vector<Meas::MeasKeyTypes> measKeys;
    measKeys.reserve(doubleDifferences.size() * _obsFilter.getUsedObservationTypes().size());
    for (size_t i = 0; i < GnssObs::ObservationType_COUNT; i++)
    {
        GnssObs::ObservationType oType = i == 0 ? GnssObs::Carrier : (i == 1 ? GnssObs::Pseudorange : GnssObs::Doppler);

        for (const auto& [satSigId, doubleDiff] : doubleDifferences)
        {
            for (const auto& [obsType, diff] : doubleDiff)
            {
                if (obsType == oType)
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
                    case GnssObs::ObservationType_COUNT:
                        LOG_CRITICAL("{}: ObservationType_COUNT is not a valid value", nameId());
                        break;
                    }
                }
            }
        }
    }

    LOG_DATA("{}: Setting measurement keys: {}", nameId(), joinToString(measKeys));
    _kalmanFilter.setMeasurements(measKeys);

    // R matrix
    for (const auto& [obsType, R] : Rtilde)
    {
        std::vector<Meas::MeasKeyTypes> obsTypeMeasKeys;
        for (const auto& key : measKeys)
        {
            if (std::holds_alternative<Meas::PsrDD>(key) && obsType == GnssObs::Pseudorange) { obsTypeMeasKeys.push_back(key); }
            else if (std::holds_alternative<Meas::CarrierDD>(key) && obsType == GnssObs::Carrier) { obsTypeMeasKeys.push_back(key); }
            else if (std::holds_alternative<Meas::DopplerDD>(key) && obsType == GnssObs::Doppler) { obsTypeMeasKeys.push_back(key); }
        }
        KeyedMatrixXd<RTK::Meas::MeasKeyTypes, RTK::Meas::SingleObs<ReceiverType>> J(Eigen::MatrixXd::Zero(static_cast<int>(obsTypeMeasKeys.size()), R.rows()),
                                                                                     obsTypeMeasKeys, R.rowKeys());

        for (const auto& rowKey : obsTypeMeasKeys)
        {
            SatSigId satSigId_s;
            switch (obsType)
            {
            case GnssObs::Pseudorange:
                satSigId_s = std::get<Meas::PsrDD>(rowKey).satSigId;
                break;
            case GnssObs::Carrier:
                satSigId_s = std::get<Meas::CarrierDD>(rowKey).satSigId;
                break;
            case GnssObs::Doppler:
                satSigId_s = std::get<Meas::DopplerDD>(rowKey).satSigId;
                break;
            case GnssObs::ObservationType_COUNT:
                LOG_CRITICAL("{}: ObservationType_COUNT is not a valid value", nameId());
                break;
            }

            const auto& satSigId_1 = _pivotSatellites.at({ satSigId_s.code, obsType }).satSigId;

            J(rowKey, RTK::Meas::SingleObs<ReceiverType>(satSigId_s, Rover, obsType)) = 1;
            J(rowKey, RTK::Meas::SingleObs<ReceiverType>(satSigId_s, Base, obsType)) = -1;
            J(rowKey, RTK::Meas::SingleObs<ReceiverType>(satSigId_1, Rover, obsType)) = -1;
            J(rowKey, RTK::Meas::SingleObs<ReceiverType>(satSigId_1, Base, obsType)) = 1;
        }
        // LOG_DATA("{}: J({}) =\n{}", nameId(), obsType, J);

        _kalmanFilter.R(obsTypeMeasKeys, obsTypeMeasKeys) = J(all, all) * R(all, all) * J(all, all).transpose();
    }
    LOG_DATA("{}: R =\n{}", nameId(), _kalmanFilter.R);

    for (const auto& [satSigId_s, doubleDiff] : doubleDifferences)
    {
        const auto& obs_s = observations.signals.at(satSigId_s);

        double lambda_j = InsConst::C / satSigId_s.freq().getFrequency(obs_s.freqNum());

        const auto& e_pLOS_s = obs_s.recvObs.at(Rover)->e_pLOS(_receiver[Rover].e_posMarker);

        for (const auto& [obsType, obs] : doubleDiff)
        {
            const auto& satSigId_1 = _pivotSatellites.at({ satSigId_s.code, obsType }).satSigId;
            const auto& obs_1 = observations.signals.at(satSigId_1);
            const auto& e_pLOS_1 = obs_1.recvObs.at(Rover)->e_pLOS(_receiver[Rover].e_posMarker);

            switch (obsType)
            {
            case GnssObs::Pseudorange:
                _kalmanFilter.z(Meas::PsrDD{ satSigId_s }) = obs.measurement - obs.estimate;
                _kalmanFilter.H.block<3>(Meas::PsrDD{ satSigId_s }, States::Pos) = (e_pLOS_1 - e_pLOS_s).transpose();
                break;
            case GnssObs::Carrier:
                _kalmanFilter.z(Meas::CarrierDD{ satSigId_s }) = obs.measurement - obs.estimate;
                _kalmanFilter.H.block<3>(Meas::CarrierDD{ satSigId_s }, States::Pos) = (e_pLOS_1 - e_pLOS_s).transpose();
                _kalmanFilter.H(Meas::CarrierDD{ satSigId_s }, States::AmbiguityDD{ satSigId_s }) = lambda_j;
                break;
            case GnssObs::Doppler:
            {
                _kalmanFilter.z(Meas::DopplerDD{ satSigId_s }) = obs.measurement - obs.estimate;
                const auto& e_vLOS_1 = obs_1.recvObs.at(Rover)->e_vLOS(_receiver[Rover].e_posMarker, _receiver[Rover].e_vel);
                const auto& e_vLOS_s = obs_s.recvObs.at(Rover)->e_vLOS(_receiver[Rover].e_posMarker, _receiver[Rover].e_vel);

                _kalmanFilter.H.block<3>(Meas::DopplerDD{ satSigId_s }, States::Pos) = Eigen::RowVector3d(
                    -e_vLOS_1.x() * e_pLOS_1.x() * e_pLOS_1.x() + e_vLOS_1.x() + e_vLOS_s.x() * e_pLOS_s.x() * e_pLOS_s.x() - e_vLOS_s.x() - e_vLOS_1.y() * e_pLOS_1.x() * e_pLOS_1.y() + e_vLOS_s.y() * e_pLOS_s.x() * e_pLOS_s.y() - e_vLOS_1.z() * e_pLOS_1.x() * e_pLOS_1.z() + e_vLOS_s.z() * e_pLOS_s.x() * e_pLOS_s.z(),
                    -e_vLOS_1.x() * e_pLOS_1.x() * e_pLOS_1.y() + e_vLOS_s.x() * e_pLOS_s.x() * e_pLOS_s.y() - e_vLOS_1.y() * e_pLOS_1.y() * e_pLOS_1.y() + e_vLOS_1.y() + e_vLOS_s.y() * e_pLOS_s.y() * e_pLOS_s.y() - e_vLOS_s.y() - e_vLOS_1.z() * e_pLOS_1.y() * e_pLOS_1.z() + e_vLOS_s.z() * e_pLOS_s.y() * e_pLOS_s.z(),
                    -e_vLOS_1.x() * e_pLOS_1.x() * e_pLOS_1.z() + e_vLOS_s.x() * e_pLOS_s.x() * e_pLOS_s.z() - e_vLOS_1.y() * e_pLOS_1.y() * e_pLOS_1.z() + e_vLOS_s.y() * e_pLOS_s.y() * e_pLOS_s.z() - e_vLOS_1.z() * e_pLOS_1.z() * e_pLOS_1.z() + e_vLOS_1.z() + e_vLOS_s.z() * e_pLOS_s.z() * e_pLOS_s.z() - e_vLOS_s.z());
                _kalmanFilter.H.block<3>(Meas::DopplerDD{ satSigId_s }, States::Vel) = (e_pLOS_1 - e_pLOS_s).transpose();
                break;
            }
            case GnssObs::ObservationType_COUNT:
                LOG_CRITICAL("{}: ObservationType_COUNT is not a valid value", nameId());
                break;
            }
        }
    }

    LOG_DATA("{}: z =\n{}", nameId(), _kalmanFilter.z.transposed());
    LOG_DATA("{}: H =\n{}", nameId(), _kalmanFilter.H);
}

std::vector<RealTimeKinematic::OutlierInfo> RealTimeKinematic::removeOutlier(Observations& observations, const std::shared_ptr<RtkSolution>& rtkSol)
{
    KeyedVectorXd<RTK::Meas::MeasKeyTypes> normInno((_kalmanFilter.z(all).array().abs()
                                                     / _kalmanFilter.S(all, all).diagonal().array().abs().sqrt())
                                                        .matrix(),
                                                    _kalmanFilter.z.rowKeys());
    if (normInno(all).hasNaN())
    {
        LOG_ERROR("{}: NIS check has NaN values. Skipping check this epoch.", nameId());
        LOG_DATA("{}: normInno has NaN", nameId());
        LOG_DATA("{}: S =\n{}", nameId(), _kalmanFilter.S);
        return {};
    }

    LOG_DATA("{}: normInno = \n{}", nameId(), normInno.transposed());
    LOG_DATA("{}: dz (post-fit) = \n{}", nameId(), _kalmanFilter.z.transposed());
    LOG_DATA("{}: dz (innovation) = \n{}", nameId(), _kalmanFilter.savedPreUpdate().z.transposed());

    // Check pivot satellites for outlier by calculating mean / stdev
    OutlierInfo faultyMeas;

    for (auto& [pivotCodeObsType, pivot] : _pivotSatellites)
    {
        auto code = pivotCodeObsType.first;
        auto obsType = pivotCodeObsType.second;
        auto filterMeasurements = [&](const Meas::MeasKeyTypes& key) -> bool {
            SatSigId satSigId;
            GnssObs::ObservationType obsType2 = GnssObs::ObservationType_COUNT;
            if (const auto* meas = std::get_if<Meas::PsrDD>(&key))
            {
                obsType2 = GnssObs::Pseudorange;
                satSigId = meas->satSigId;
            }
            else if (const auto* meas = std::get_if<Meas::CarrierDD>(&key))
            {
                obsType2 = GnssObs::Carrier;
                satSigId = meas->satSigId;
            }
            else if (const auto* meas = std::get_if<Meas::DopplerDD>(&key))
            {
                obsType2 = GnssObs::Doppler;
                satSigId = meas->satSigId;
            }
            return satSigId.code != code || obsType2 != obsType;
        };
        double mean = 0.0;
        size_t amount = 0;
        for (const auto& key : _kalmanFilter.savedPreUpdate().z.rowKeys())
        {
            if (filterMeasurements(key)) { continue; }
            mean += _kalmanFilter.savedPreUpdate().z(key);
            amount++;
        }
        if (amount <= 1) { continue; }
        mean /= static_cast<double>(amount);

        double variance = 0.0;
        for (const auto& key : _kalmanFilter.savedPreUpdate().z.rowKeys())
        {
            if (filterMeasurements(key)) { continue; }
            variance += std::pow(std::abs(_kalmanFilter.savedPreUpdate().z(key) - mean), 2);
        }
        variance *= 1.0 / (static_cast<double>(amount) - 1.0);

        LOG_DATA("{}: pivot [{}][{}]: {:.3f} / {:.3f} = {:.3f}", nameId(), code, obsType, mean, std::sqrt(variance), mean / std::sqrt(variance));

        if (std::abs(mean / std::sqrt(variance)) > 5.0) // 3 or 5 sigma (but t-distributed https://en.wikipedia.org/wiki/Student%27s_t-distribution)
        {
            switch (obsType)
            {
            case GnssObs::Pseudorange:
                faultyMeas.key = Meas::PsrDD{ pivot.satSigId };
                break;
            case GnssObs::Carrier:
                faultyMeas.key = Meas::CarrierDD{ pivot.satSigId };
                break;
            case GnssObs::Doppler:
                faultyMeas.key = Meas::DopplerDD{ pivot.satSigId };
                break;
            case GnssObs::ObservationType_COUNT:
                break;
            }
            faultyMeas.obsType = obsType;
            faultyMeas.satSigId = pivot.satSigId;
            faultyMeas.pivot = RtkSolution::PivotChange::Reason::PivotOutlier;

            break; // Pivot is problematic throw out and then break. Only one pivot change at a time
        }
    }
    if (faultyMeas.obsType == GnssObs::ObservationType_COUNT)
    {
        int maxIdx = 0;
        normInno(all).maxCoeff(&maxIdx);
        faultyMeas.key = normInno.rowKeys().at(static_cast<size_t>(maxIdx));
        LOG_DATA("{}: Largest post-fit innovation: {} ({:.1f})", nameId(), faultyMeas.key, normInno(faultyMeas.key));

        if (const auto* meas = std::get_if<Meas::PsrDD>(&faultyMeas.key))
        {
            faultyMeas.obsType = GnssObs::Pseudorange;
            faultyMeas.satSigId = meas->satSigId;
        }
        else if (const auto* meas = std::get_if<Meas::CarrierDD>(&faultyMeas.key))
        {
            faultyMeas.obsType = GnssObs::Carrier;
            faultyMeas.satSigId = meas->satSigId;
        }
        else if (const auto* meas = std::get_if<Meas::DopplerDD>(&faultyMeas.key))
        {
            faultyMeas.obsType = GnssObs::Doppler;
            faultyMeas.satSigId = meas->satSigId;
        }

        auto& pivot = _pivotSatellites.at({ faultyMeas.satSigId.code, faultyMeas.obsType });
        LOG_DATA("{}: Pivot for [{}] is [{}]", nameId(), faultyMeas.satSigId, pivot.satSigId);
        if (pivot.satSigId == faultyMeas.satSigId)
        {
            faultyMeas.pivot = RtkSolution::PivotChange::Reason::PivotOutlier;
            LOG_DATA("{}: Pivot [{}][{}] with outlier.", nameId(), pivot.satSigId, faultyMeas.obsType);
        }
    }

    LOG_DATA("{}: faultyMeas = {}", nameId(), faultyMeas.key);

    if (faultyMeas.obsType == GnssObs::Pseudorange && _outlierMinPsrObsKeep != 0 && observations.nObservables[GnssObs::Pseudorange] <= _outlierMinPsrObsKeep)
    {
        addEventToGui(rtkSol, "Stopped doing NIS check, as minimum amount of pseudorange observables reached.");
        return {};
    }

    std::vector<RealTimeKinematic::OutlierInfo> faulty{ faultyMeas };
    // Exclude Pseudorange and Carrier toghether
    if (faultyMeas.obsType == GnssObs::Pseudorange && _kalmanFilter.z.hasRow(Meas::CarrierDD{ faultyMeas.satSigId }))
    {
        LOG_DATA("{}: Also flagging [{}][{}] as outlier.", nameId(), faultyMeas.satSigId, GnssObs::Carrier);
        faulty.push_back(faultyMeas);
        faulty.back().obsType = GnssObs::Carrier;
        faulty.back().key = Meas::CarrierDD{ faulty.back().satSigId };
    }
    else if (faultyMeas.obsType == GnssObs::Carrier && _kalmanFilter.z.hasRow(Meas::PsrDD{ faultyMeas.satSigId }))
    {
        LOG_DATA("{}: Also flagging [{}][{}] as outlier.", nameId(), faultyMeas.satSigId, GnssObs::Pseudorange);
        faulty.push_back(faultyMeas);
        faulty.back().obsType = GnssObs::Pseudorange;
        faulty.back().key = Meas::PsrDD{ faulty.back().satSigId };
    }
    if (faulty.size() == 2)
    {
        if (auto pivotKey = std::make_pair(faulty.back().satSigId.code, faulty.back().obsType);
            _pivotSatellites.contains(pivotKey) && _pivotSatellites.at(pivotKey).satSigId == faulty.back().satSigId)
        {
            faulty.back().pivot = RtkSolution::PivotChange::Reason::PivotOutlier;
        }
        else
        {
            faulty.back().pivot.reset();
        }
    }

    printPivotSatellites();
    for (const auto& faultyMeas : faulty)
    {
        LOG_DATA("{}: Erasing receiver observation [{}][{}], because outlier (NIS check).", nameId(), faultyMeas.satSigId, faultyMeas.obsType);

        if (faultyMeas.pivot)
        {
            LOG_DATA("{}: Removing pivot satellite [{}][{}] because outlier detected.", nameId(), faultyMeas.satSigId, faultyMeas.obsType);
            updatePivotSatellite(faultyMeas.satSigId.code, faultyMeas.obsType, observations, rtkSol, faultyMeas.pivot.value());
        }
        else
        {
            if (States::AmbiguityDD ambStateKey{ faultyMeas.satSigId };
                faultyMeas.obsType == GnssObs::Carrier && _kalmanFilter.hasState(ambStateKey))
            {
                LOG_TRACE("{}: Removing state [{}]", nameId(), ambStateKey);
                _kalmanFilter.removeState(ambStateKey);
                _ambiguitiesHold.erase(faultyMeas.satSigId);
                if (_outputStateEvents)
                {
                    addEventToGui(rtkSol, fmt::format("State [{}] removed (because outlier detected in carrier measurement [{}])", ambStateKey, faultyMeas.key));
                }
            }
            _kalmanFilter.removeMeasurement(faultyMeas.key);

            LOG_DATA("{}: x = \n{}", nameId(), _kalmanFilter.x.transposed());
            LOG_DATA("{}: z = \n{}", nameId(), _kalmanFilter.z.transposed());
        }

        observations.removeSignal(faultyMeas.satSigId, faultyMeas.obsType, nameId());
        _obsFilter.excludeSignalTemporarily(faultyMeas.satSigId, _outlierRemoveEpochs - 1);
        addEventToGui(rtkSol, fmt::format("Erasing receiver observation [{}][{}], because outlier (NIS check)", faultyMeas.satSigId, faultyMeas.obsType));

        _nMeasExcludedNIS++;
        rtkSol->nisRemovedCnt++;
        rtkSol->outliers.emplace_back(RtkSolution::Outlier::Type::NIS, faultyMeas.satSigId, faultyMeas.obsType);
    }
    return faulty;
}

RealTimeKinematic::UpdateStatus RealTimeKinematic::kalmanFilterUpdate(const std::shared_ptr<RtkSolution>& rtkSol)
{
    UpdateStatus solutionValid;
    if (_kalmanFilter.z.rows() == 0)
    {
        solutionValid.valid = false;
    }
    else
    {
        auto dt = static_cast<double>((_receiver[Rover].gnssObs->insTime - _lastUpdate).count());
        solutionValid.threshold = std::max({ 50.0,
                                             10.0 * _receiver[Rover].e_vel.norm() * dt,
                                             _kalmanFilter.P(States::Pos, States::Pos).diagonal().cwiseSqrt().maxCoeff() });

        _kalmanFilter.savePreUpdate();
        _kalmanFilter.correctWithMeasurementInnovation();

        LOG_DATA("{}: x (a posteriori, t   = {}) =\n{}", nameId(), _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), _kalmanFilter.x.transposed());
        LOG_DATA("{}: P (a posteriori, t   = {}) =\n{}", nameId(), _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), _kalmanFilter.P);

        LOG_DATA("{}: dx_ecef (a posteriori - a priori    ) = {}", nameId(), (_kalmanFilter.x.segment<3>(States::Pos) - _receiver[Rover].e_posMarker).transpose());
        LOG_DATA("{}: dv_ecef (a posteriori - a priori    ) = {}", nameId(), (_kalmanFilter.x.segment<3>(States::Vel) - _receiver[Rover].e_vel).transpose());

        solutionValid.dx = (_kalmanFilter.x.segment<3>(States::Pos) - _receiver[Rover].e_posMarker).norm();
        solutionValid.valid = solutionValid.dx < solutionValid.threshold;
    }

    if (solutionValid.valid)
    {
        _receiver[Rover].e_posMarker = _kalmanFilter.x.segment<3>(States::Pos);
        _receiver[Rover].e_vel = _kalmanFilter.x.segment<3>(States::Vel);
        _receiver[Rover].lla_posMarker = trafo::ecef2lla_WGS84(_receiver[Rover].e_posMarker);

        if (_kalmanFilter.isPreUpdateSaved()) { _kalmanFilter.discardPreUpdate(); }
    }
    else
    {
        std::string text = _kalmanFilter.z.rows() == 0
                               ? std::string("Update rejected - no observations left")
                               : fmt::format("Update rejected - position change due to update is {:.2f} [m] > {:.2f} [m]", solutionValid.dx, solutionValid.threshold);

        LOG_WARN("{}: [{}] {}", nameId(), _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), text);
        addEventToGui(rtkSol, text);
        if (_kalmanFilter.isPreUpdateSaved()) { _kalmanFilter.restorePreUpdate(); }
    }

    return solutionValid;
}

bool RealTimeKinematic::resolveAmbiguities(size_t nCarrierMeasUniqueSatellite, const std::shared_ptr<RtkSolution>& rtkSol)
{
    std::vector<States::StateKeyType> ambKeys;
    std::vector<Meas::MeasKeyTypes> ambMeasKeys;
    for (size_t i = States::KFStates_COUNT; i < _kalmanFilter.x.rowKeys().size(); i++) // 0-2 Pos, 3-5 Vel
    {
        if (const auto* ambDD = std::get_if<States::AmbiguityDD>(&_kalmanFilter.x.rowKeys().at(i)))
        {
            ambKeys.emplace_back(*ambDD);
            ambMeasKeys.emplace_back(*ambDD);
        }
    }
    size_t nAmbs = ambKeys.size();

    if (nCarrierMeasUniqueSatellite < _nMinSatForAmbFix)
    {
        LOG_TRACE("{}: [{}] Not fixing ambiguities because only {} satellites but minimum {} needed.",
                  nameId(), _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), nCarrierMeasUniqueSatellite, _nMinSatForAmbFix);
        if (!_nMinSatForAmbFixTriggered)
        {
            addEventToGui(rtkSol, fmt::format("Not fixing ambiguities anymore because\nonly {} satellites but minimum {} needed.", nCarrierMeasUniqueSatellite, _nMinSatForAmbFix));
        }
        _nMinSatForAmbFixTriggered = true;

        return false;
    }
    if (_nMinSatForAmbFixTriggered)
    {
        _nMinSatForAmbFixTriggered = false;
        addEventToGui(rtkSol, fmt::format("Resuming ambiguity fixing as\nminimum amount of satellites reached."));
    }

    if (double posVar = _kalmanFilter.P(States::Pos, States::Pos).diagonal().sum() / 3.0;
        posVar > _maxPosVar)
    {
        LOG_TRACE("{}: [{}] Not fixing ambiguities because position variance is {:.4f}m which is higher than {:.4f}m",
                  nameId(), _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), posVar, _maxPosVar);
        if (!_maxPosVarTriggered)
        {
            addEventToGui(rtkSol, fmt::format("Not fixing ambiguities anymore because\nposition variance is {:.4f}m^2 which is higher than {:.4f}m^2.", posVar, _maxPosVar));
        }
        _maxPosVarTriggered = true;
        return false;
    }
    else if (_maxPosVarTriggered) // NOLINT(readability-else-after-return,llvm-else-after-return)
    {
        _maxPosVarTriggered = false;
        addEventToGui(rtkSol, fmt::format("Resuming ambiguity fixing as\nposition variance smaller than limit."));
    }

    if (ambKeys.empty()) { return false; } // No ambiguities to estimate
    if (nCarrierMeasUniqueSatellite >= _nMinSatForAmbHold
        && _ambiguityResolutionStrategy == AmbiguityResolutionStrategy::FixAndHold && ambKeys.size() == _ambiguitiesHold.size()) // No new ambiguities to estimate
    {
#if LOG_LEVEL <= LOG_LEVEL_DATA
        {
            std::set<SatSigId> ambStates;
            for (const auto& key : ambKeys)
            {
                auto amb = std::get<States::AmbiguityDD>(key);
                if (ambStates.contains(amb.satSigId))
                {
                    LOG_CRITICAL("{}: The KF state has [{}] more than once.", nameId(), amb);
                }
                ambStates.insert(amb.satSigId);
            }
            LOG_DATA("{}: ambKeys: [{}]", nameId(), fmt::join(ambStates, ", "));

            std::set<SatSigId> ambHold;
            for (const auto& hold : _ambiguitiesHold)
            {
                if (ambHold.contains(hold.first))
                {
                    LOG_CRITICAL("{}: _ambiguitiesHold has [{}] more than once.", nameId(), hold.first);
                }
                ambHold.insert(hold.first);
            }
            LOG_DATA("{}: ambHold: [{}]", nameId(), fmt::join(ambHold, ", "));

            if (ambStates != ambHold)
            {
                std::vector<SatSigId> diff;
                std::set_difference(ambStates.begin(), ambStates.end(), ambHold.begin(), ambHold.end(), std::back_inserter(diff)); // NOLINT(boost-use-ranges,modernize-use-ranges) // There is no ranges::set_difference
                LOG_DATA("{}: Amb not in hold:   [{}]", nameId(), fmt::join(diff, ", "));
                diff.clear();
                std::set_difference(ambHold.begin(), ambHold.end(), ambStates.begin(), ambStates.end(), std::back_inserter(diff)); // NOLINT(boost-use-ranges,modernize-use-ranges) // There is no ranges::set_difference
                LOG_DATA("{}: Amb not in states: [{}]", nameId(), fmt::join(diff, ", "));
            }
        }
#endif
        Eigen::VectorXd fixedAmb = Eigen::VectorXd::Zero(static_cast<int>(ambKeys.size()));
        // Apply exact integers (after update it is still float in the late digits)
        for (size_t k = 0; k < ambKeys.size(); k++)
        {
            const auto& key = ambKeys.at(k);
            auto satSigId = std::get<States::AmbiguityDD>(key).satSigId;
            LOG_DATA("{}: FixAndHold: [{}] Holding {} to {} (after update)", nameId(), satSigId, _kalmanFilter.x.hasRow(key), _ambiguitiesHold.contains(satSigId));
            LOG_DATA("{}: FixAndHold: [{}] Holding {} to {} (after update)", nameId(), satSigId, _kalmanFilter.x(key), _ambiguitiesHold.at(satSigId));
            fixedAmb(static_cast<int>(k)) = _ambiguitiesHold.at(satSigId);
        }
        rtkSol->nAmbiguitiesFixed = ambKeys.size() + 1; // + 1 to also count the pivot
        applyFixedAmbiguities(fixedAmb, ambKeys, ambMeasKeys);

        if (_nMinSatForAmbHoldTriggered)
        {
            _nMinSatForAmbHoldTriggered = false;
            addEventToGui(rtkSol, fmt::format("Resuming ambiguity holding as\nminimum amount of satellites reached."));
        }

        return true;
    }
    if (nCarrierMeasUniqueSatellite < _nMinSatForAmbHold && _ambiguityResolutionStrategy == AmbiguityResolutionStrategy::FixAndHold)
    {
        LOG_TRACE("{}: [{}] Not holding ambiguities because only {} satellites but minimum {} needed.",
                  nameId(), _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), nCarrierMeasUniqueSatellite, _nMinSatForAmbHold);
        if (!_nMinSatForAmbHoldTriggered)
        {
            addEventToGui(rtkSol, fmt::format("Not holding ambiguities anymore because\nonly {} satellites but minimum {} needed.", nCarrierMeasUniqueSatellite, _nMinSatForAmbHold));
        }
        _nMinSatForAmbHoldTriggered = true;
    }

    LOG_DATA("{}: [{}] Estimating ambiguities", nameId(), _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST));
    auto floatKeys = States::PosVel;
    size_t partialFixTries = 0;
    do
    {
        LOG_DATA("{}: [{}] floatKeys = {}", nameId(), _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), fmt::join(floatKeys, ", "));
        LOG_DATA("{}: [{}] ambKeys   = {}", nameId(), _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), fmt::join(ambKeys, ", "));
        auto fixed = ResolveAmbiguities(_kalmanFilter.x(ambKeys), _kalmanFilter.P(ambKeys, ambKeys),
                                        _kalmanFilter.x(floatKeys), _kalmanFilter.P(floatKeys, floatKeys),
                                        _kalmanFilter.P(ambKeys, floatKeys), _kalmanFilter.P(floatKeys, ambKeys),
                                        _ambiguityResolutionParameters, nameId());
        rtkSol->ambiguityResolutionFailure = fixed.failure;
        rtkSol->ambiguityCriticalValueRatio = fixed.ambiguityCriticalValueRatio;

        if (fixed.failure == AmbiguityResolutionFailure::None)
        {
            _kalmanFilter.x(floatKeys) = fixed.b;
            LOG_DATA("{}: x(fixed.b) =\n{}", nameId(), _kalmanFilter.x.transposed());

            if (_applyFixedAmbiguitiesWithUpdate) { applyFixedAmbiguities(fixed.fixedAmb.front().a, ambKeys, ambMeasKeys); }
            else
            {
                _kalmanFilter.x(ambKeys) = fixed.fixedAmb.front().a;
                // _kalmanFilter.P(floatKeys, floatKeys) = fixed.Qb;
                // LOG_DATA("{}: P(fixed.Qb) =\n{}", nameId(), _kalmanFilter.P);
            }

            LOG_DATA("{}: dx_ecef (fix, update  - fix         ) = {}", nameId(), (_kalmanFilter.x.segment<3>(States::Pos) - _receiver[Rover].e_posMarker).transpose());
            LOG_DATA("{}: dv_ecef (fix, update  - fix         ) = {}", nameId(), (_kalmanFilter.x.segment<3>(States::Vel) - _receiver[Rover].e_vel).transpose());
            _receiver[Rover].e_posMarker = _kalmanFilter.x.segment<3>(States::Pos);
            _receiver[Rover].e_vel = _kalmanFilter.x.segment<3>(States::Vel);
            _receiver[Rover].lla_posMarker = trafo::ecef2lla_WGS84(_receiver[Rover].e_posMarker);

            rtkSol->ambiguityCriticalValueRatio = fixed.ambiguityCriticalValueRatio;
            rtkSol->nAmbiguitiesFixed = fixed.nFixed + 1; // + 1 to also count the pivot

            for (const auto& key_ : ambKeys)
            {
                const auto key = std::get<States::AmbiguityDD>(key_);

                if (_ambiguityResolutionStrategy == AmbiguityResolutionStrategy::FixAndHold
                    && _ambiguitiesHold.contains(key.satSigId)
                    && std::abs(_kalmanFilter.x(key) - _ambiguitiesHold.at(key.satSigId)) > 0.1
                    && fixed.nFixed == nAmbs && partialFixTries == 0)
                {
                    LOG_WARN("{}: Ambiguity [{}] changed from {} to {} (despite being FixAndHold)", nameId(), key.satSigId,
                             _kalmanFilter.x(key), _ambiguitiesHold.at(key.satSigId));
                }
                _ambiguitiesHold[key.satSigId] = _kalmanFilter.x(key);
            }

            return partialFixTries == 0;
        }
        LOG_DATA("{}: Fixing failed. partialFixTries = {}", nameId(), partialFixTries);

        if (_ambiguityResolutionParameters.partialFixing)
        {
            int maxAmb = 0;
            _kalmanFilter.P(ambKeys, ambKeys).diagonal().maxCoeff(&maxAmb);

            LOG_DATA("{}: [{}] Trying partial fixing by not fixing [{}] with highest variance of {}",
                     nameId(), _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST),
                     ambKeys.at(static_cast<size_t>(maxAmb)),
                     _kalmanFilter.P(ambKeys.at(static_cast<size_t>(maxAmb)), ambKeys.at(static_cast<size_t>(maxAmb))));
            floatKeys.push_back(ambKeys.at(static_cast<size_t>(maxAmb)));
            ambKeys.erase(std::next(ambKeys.begin(), maxAmb));
            ambMeasKeys.erase(std::next(ambMeasKeys.begin(), maxAmb));
            partialFixTries++;
        }
    } while (_ambiguityResolutionParameters.partialFixing && ambKeys.size() > 1 && partialFixTries < 6);

    if (_ambiguityResolutionParameters.searchAlgorithm != AmbiguityResolutionParameters::SearchAlgorithm::None)
    {
        LOG_TRACE("{}: [{}] Ambiguity resolution failed", nameId(), _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST));
        _ambiguitiesHold.clear();
    }

    return false;
}

void RealTimeKinematic::applyFixedAmbiguities(const Eigen::VectorXd& fixedAmb, const std::vector<States::StateKeyType>& ambKeys, const std::vector<Meas::MeasKeyTypes>& ambMeasKeys)
{
#if LOG_LEVEL <= LOG_LEVEL_DATA
    if (((_kalmanFilter.x(ambKeys).array() * 1e2).round() * 1e-2).matrix() != fixedAmb)
    {
        auto ambPrint = KeyedMatrixXd<States::StateKeyType>(
            (Eigen::MatrixXd(static_cast<int>(ambKeys.size()), 2) << _kalmanFilter.x(ambKeys), fixedAmb).finished(),
            ambKeys, std::vector<States::StateKeyType>{ States::AmbiguityDD(SatSigId(Code::G1C, 200)), States::AmbiguityDD(SatSigId(Code::G1C, 201)) });
        LOG_DATA("{}: [{}] Ambiguity estimate changed (relative to pivot) (200 = prev, 201 = new)\n{}", nameId(), _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), ambPrint.transposed());
        LOG_DATA("P(amb) =\n{}", _kalmanFilter.P(ambKeys, ambKeys));
    }
#endif

    // Make a KF update with the fixed ambiguities as observation in order to adapt the P matrix
    _kalmanFilter.setMeasurements(ambMeasKeys);

    // Update the Measurement sensitivity Matrix (𝐇), the Measurement noise covariance matrix (𝐑) and the Measurement vector (𝐳)
    _kalmanFilter.z.segment(ambMeasKeys) = fixedAmb - _kalmanFilter.x(ambKeys);
    _kalmanFilter.H(ambMeasKeys, ambKeys).setIdentity();

    // R matrix
    _kalmanFilter.R(all, all).diagonal().setConstant(_ambFixUpdateVariance);

    LOG_DATA("{}: z =\n{}", nameId(), _kalmanFilter.z.transposed());
    LOG_DATA("{}: H =\n{}", nameId(), _kalmanFilter.H);
    LOG_DATA("{}: R =\n{}", nameId(), _kalmanFilter.R);

    _kalmanFilter.correctWithMeasurementInnovation();

    LOG_DATA("{}: x (fix, update , t   = {}) = (Ambiguity update)\n{}", nameId(), _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), _kalmanFilter.x.transposed());
    // Apply the integer values, otherwise values not exact
    _kalmanFilter.x(ambKeys) = fixedAmb;
    LOG_DATA("{}: x (fix, update , t   = {}) = (Ambiguity integers)\n{}", nameId(), _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), _kalmanFilter.x.transposed());
}

const char* to_string(const RealTimeKinematic::ReceiverType& receiver)
{
    switch (receiver)
    {
    case RealTimeKinematic::ReceiverType::Base:
        return "Base";
    case RealTimeKinematic::ReceiverType::Rover:
        return "Rover";
    case RealTimeKinematic::ReceiverType::ReceiverType_COUNT:
        return "";
    }
    return "";
}

} // namespace NAV