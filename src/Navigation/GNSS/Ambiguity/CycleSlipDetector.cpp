// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file CycleSlipDetector.cpp
/// @brief Combination of different cycle-slip detection algorithms
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-11-10

#include "CycleSlipDetector.hpp"

#include "Navigation/Constants.hpp"
#include "util/Logger.hpp"
#include "internal/gui/widgets/HelpMarker.hpp"
#include <fmt/core.h>

namespace NAV
{

std::vector<CycleSlipDetector::Result> CycleSlipDetector::checkForCycleSlip(InsTime insTime, const std::vector<SatelliteObservation>& satObs, [[maybe_unused]] const std::string& nameId)
{
    std::vector<Result> cycleSlips;
    std::vector<SatSigId> resetThisEpoch;

    auto searchCycleSlipAlreadyFound = [&cycleSlips](const SatSigId& satSigId) {
        return std::find_if(cycleSlips.begin(), cycleSlips.end(), [&](const Result& cycleSlip) {
            if (const auto* s = std::get_if<CycleSlipLossOfLockIndicator>(&cycleSlip)) { return s->signal == satSigId; }
            if (const auto* s = std::get_if<CycleSlipSingleFrequency>(&cycleSlip)) { return s->signal == satSigId; }
            return false;
        });
    };

    for (const auto& obs : satObs)
    {
        std::string signals;
        for (const auto& signal : obs.signals)
        {
            if (!signals.empty()) { signals += ", "; }
            signals += fmt::format("{}", SatSigId(signal.code, obs.satId.satNum));
        }
        LOG_DATA("{}: [{}] {}", nameId, insTime.toYMDHMS(GPST), signals);
    }

    for (const auto& obs : satObs)
    {
        LOG_DATA("{}: [{}] Checking [{}]", nameId, insTime.toYMDHMS(GPST), obs.satId);
        for (const auto& signal : obs.signals)
        {
            auto satSigId = SatSigId(signal.code, obs.satId.satNum);
            if (_enableLLICheck)
            {
                if (signal.measurement.LLI)
                {
                    LOG_DATA("{}:   [{}] Cycle-slip detected, due to LLI set", nameId, satSigId);
                    cycleSlips.emplace_back(CycleSlipLossOfLockIndicator{ satSigId });
                    resetSignal(satSigId);
                }
                else
                {
                    LOG_DATA("{}:   [{}] LLI check passed", nameId, SatSigId(signal.code, obs.satId.satNum));
                }
            }
            auto lambda = InsConst<double>::C / satSigId.freq().getFrequency(obs.freqNum);

            auto result = _singleFrequencyDetector.checkForCycleSlip(satSigId, insTime,
                                                                     signal.measurement.value * lambda,
                                                                     lambda * _singleFrequencyThresholdPercentage);
            if (result == PolynomialCycleSlipDetectorResult::CycleSlip)
            {
                if (searchCycleSlipAlreadyFound(satSigId) == cycleSlips.end())
                {
                    LOG_DATA("{}:   [{}] Cycle-slip detected, due to single frequency check", nameId, satSigId);
                    cycleSlips.emplace_back(CycleSlipSingleFrequency{ satSigId });
                    resetThisEpoch.push_back(satSigId);
                }
                else
                {
                    LOG_DATA("{}:   [{}] Cycle-slip detected, due to single frequency check, but already found due to LLI.", nameId, satSigId);
                }
            }
            else if (result == PolynomialCycleSlipDetectorResult::NoCycleSlip)
            {
                LOG_DATA("{}:   [{}] Single Frequency check passed", nameId, satSigId);
            }
            else if (result == PolynomialCycleSlipDetectorResult::LessDataThanWindowSize)
            {
                LOG_DATA("{}:   [{}] Single Frequency check skipped, because not enough data", nameId, satSigId);
            }
        }

        if (obs.signals.size() >= 2)
        {
            // Signal with largest frequency, e.g. L1
            auto signal1 = std::max_element(obs.signals.begin(), obs.signals.end(), [&obs](const SatelliteObservation::Signal& s1, const SatelliteObservation::Signal& s2) {
                return s1.code.getFrequency().getFrequency(obs.freqNum) < s2.code.getFrequency().getFrequency(obs.freqNum);
            });
            // Signal with lower frequency, e.g. L2/L5
            for (const auto& signal2 : obs.signals)
            {
                if (signal1->code.getFrequency() == signal2.code.getFrequency()) { continue; }

                auto satSigId1 = SatSigId(signal1->code, obs.satId.satNum);
                auto satSigId2 = SatSigId(signal2.code, obs.satId.satNum);

                auto key = DualFrequencyCombination{ .satId = obs.satId, .sig1 = signal1->code, .sig2 = signal2.code };
                auto lambda1 = InsConst<double>::C / satSigId1.freq().getFrequency(obs.freqNum);
                auto lambda2 = InsConst<double>::C / satSigId2.freq().getFrequency(obs.freqNum);

                auto result = _dualFrequencyDetector.checkForCycleSlip(key, insTime,
                                                                       signal1->measurement.value * lambda1 - signal2.measurement.value * lambda2,
                                                                       lambda1 * _dualFrequencyThresholdPercentage);
                if (result == PolynomialCycleSlipDetectorResult::CycleSlip)
                {
                    auto sat1Slip = searchCycleSlipAlreadyFound(satSigId1);
                    auto sat2Slip = searchCycleSlipAlreadyFound(satSigId2);
                    if ((sat1Slip == cycleSlips.end() && sat2Slip != cycleSlips.end())
                        || (sat1Slip != cycleSlips.end() && sat2Slip == cycleSlips.end()))
                    {
                        auto satSigIdSlip = sat1Slip == cycleSlips.end() ? satSigId2 : satSigId1;
                        LOG_DATA("{}:   [{} / {}] Cycle-slip detected, due to dual frequency check. But slip was already found in [{}]. Assuming not both slipped.",
                                 nameId, satSigId1, satSigId2, satSigIdSlip);
                        resetSignal(satSigIdSlip);
                        resetThisEpoch.push_back(satSigIdSlip);
                        std::erase(resetThisEpoch, satSigIdSlip);
                        _singleFrequencyDetector.addMeasurement(satSigIdSlip, insTime,
                                                                sat1Slip == cycleSlips.end() ? signal2.measurement.value * lambda2 : signal1->measurement.value * lambda1);
                    }
                    else if (sat1Slip != cycleSlips.end() && sat2Slip != cycleSlips.end())
                    {
                        LOG_DATA("{}:   [{} / {}] Cycle-slip detected, due to dual frequency check. But both slips were already found.",
                                 nameId, satSigId1, satSigId2);
                        resetSignal(satSigId1);
                        resetSignal(satSigId2);
                        std::erase(resetThisEpoch, satSigId1);
                        std::erase(resetThisEpoch, satSigId2);
                        _singleFrequencyDetector.addMeasurement(satSigId1, insTime, signal1->measurement.value * lambda1);
                        _singleFrequencyDetector.addMeasurement(satSigId2, insTime, signal2.measurement.value * lambda2);
                    }
                    else
                    {
                        LOG_DATA("{}:   [{} / {}] Cycle-slip detected, due to dual frequency check", nameId, satSigId1, satSigId2);
                        cycleSlips.emplace_back(CycleSlipDualFrequency{ satSigId1, satSigId2 });
                        resetSignal(satSigId1);
                        resetSignal(satSigId2);
                        std::erase(resetThisEpoch, satSigId1);
                        std::erase(resetThisEpoch, satSigId2);
                        _singleFrequencyDetector.addMeasurement(satSigId1, insTime, signal1->measurement.value * lambda1);
                        _singleFrequencyDetector.addMeasurement(satSigId2, insTime, signal2.measurement.value * lambda2);
                    }
                    _dualFrequencyDetector.addMeasurement(key, insTime, signal1->measurement.value * lambda1 - signal2.measurement.value * lambda2);
                }
                else if (result == PolynomialCycleSlipDetectorResult::NoCycleSlip)
                {
                    if (auto slip = searchCycleSlipAlreadyFound(satSigId1);
                        slip != cycleSlips.end())
                    {
                        LOG_DATA("{}:   [{}] Cycle-slip detected, but dual frequency check could not detect it. Removing previous detection.", nameId, satSigId1);
                        cycleSlips.erase(slip);
                        std::erase(resetThisEpoch, satSigId1);
                    }
                    if (auto slip = searchCycleSlipAlreadyFound(satSigId2);
                        slip != cycleSlips.end())
                    {
                        LOG_DATA("{}:   [{}] Cycle-slip detected, but dual frequency check could not detect it. Removing previous detection.", nameId, satSigId2);
                        cycleSlips.erase(slip);
                        std::erase(resetThisEpoch, satSigId2);
                    }
                    LOG_DATA("{}:   [{} / {}] Dual Frequency check passed", nameId, satSigId1, satSigId2);
                }
                else if (result == PolynomialCycleSlipDetectorResult::LessDataThanWindowSize)
                {
                    LOG_DATA("{}:   [{} / {}] Dual Frequency check skipped, because not enough data", nameId, satSigId1, satSigId2);
                }
            }
        }
        for (const auto& signal : obs.signals)
        {
            auto satSigId = SatSigId(signal.code, obs.satId.satNum);
            auto iter = std::find(resetThisEpoch.begin(), resetThisEpoch.end(), satSigId);
            if (iter != resetThisEpoch.end())
            {
                resetSignal(satSigId);
                auto lambda = InsConst<double>::C / satSigId.freq().getFrequency(obs.freqNum);
                _singleFrequencyDetector.addMeasurement(satSigId, insTime, signal.measurement.value * lambda);
            }
        }
    }

    return cycleSlips;
}

void CycleSlipDetector::reset()
{
    _singleFrequencyDetector.clear();
    _dualFrequencyDetector.clear();
}

void CycleSlipDetector::resetSignal(const SatSigId& satSigId)
{
    _singleFrequencyDetector.reset(satSigId);
    std::erase_if(_dualFrequencyDetector._detectors, [&satSigId](const auto& detector) {
        return detector.first.satId == satSigId.toSatId()
               && (detector.first.sig1 == satSigId.code || detector.first.sig2 == satSigId.code);
    });
}

bool CycleSlipDetectorGui(const char* label, CycleSlipDetector& cycleSlipDetector, float width, bool dualFrequencyAvailable)
{
    bool changed = false;
    if (ImGui::Checkbox(fmt::format("LLI check##{}", label).c_str(), &cycleSlipDetector._enableLLICheck))
    {
        changed = true;
    }
    ImGui::SameLine();
    if (!cycleSlipDetector._singleFrequencyDetector.isEnabled()) { ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyle().Colors[ImGuiCol_TextDisabled]); }
    if (ImGui::Button(fmt::format("Single Frequency detector##{}", label).c_str()))
    {
        ImGui::OpenPopup(fmt::format("Single Frequency detector##Popup - {}", label).c_str());
    }
    if (!cycleSlipDetector._singleFrequencyDetector.isEnabled()) { ImGui::PopStyleColor(); }

    if (ImGui::BeginPopup(fmt::format("Single Frequency detector##Popup - {}", label).c_str()))
    {
        if (PolynomialCycleSlipDetectorGui(fmt::format("Single Frequency detector {}", label).c_str(),
                                           cycleSlipDetector._singleFrequencyDetector, width))
        {
            changed = true;
        }
        ImGui::SetNextItemWidth(width);
        if (double val = cycleSlipDetector._singleFrequencyThresholdPercentage * 100.0;
            ImGui::DragDouble(fmt::format("Threshold##single {}", label).c_str(), &val, 1.0F,
                              1.0, std::numeric_limits<double>::max(), "%.2f %%"))
        {
            cycleSlipDetector._singleFrequencyThresholdPercentage = val / 100.0;
            changed = true;
        }
        ImGui::SameLine();
        gui::widgets::HelpMarker("As percentage of the wavelength of the signals used.");

        ImGui::EndPopup();
    }
    ImGui::SameLine();
    if (!dualFrequencyAvailable || !cycleSlipDetector._dualFrequencyDetector.isEnabled()) { ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyle().Colors[ImGuiCol_TextDisabled]); }
    if (ImGui::Button(fmt::format("Dual Frequency detector##{}", label).c_str()))
    {
        ImGui::OpenPopup(fmt::format("Dual Frequency detector##Popup - {}", label).c_str());
    }
    if (!dualFrequencyAvailable && ImGui::IsItemHovered()) { ImGui::SetTooltip("Dual frequency not available due to filter settings."); }
    if (!dualFrequencyAvailable || !cycleSlipDetector._dualFrequencyDetector.isEnabled()) { ImGui::PopStyleColor(); }
    if (ImGui::BeginPopup(fmt::format("Dual Frequency detector##Popup - {}", label).c_str()))
    {
        if (PolynomialCycleSlipDetectorGui(fmt::format("Dual Frequency detector {}", label).c_str(),
                                           cycleSlipDetector._dualFrequencyDetector, width))
        {
            changed = true;
        }
        ImGui::SetNextItemWidth(width);
        if (double val = cycleSlipDetector._dualFrequencyThresholdPercentage * 100.0;
            ImGui::DragDouble(fmt::format("Threshold##dual {}", label).c_str(), &val, 1.0F,
                              1.0, std::numeric_limits<double>::max(), "%.2f %%"))
        {
            cycleSlipDetector._dualFrequencyThresholdPercentage = val / 100.0;
            changed = true;
        }
        ImGui::SameLine();
        gui::widgets::HelpMarker("As percentage of the smallest wavelength of the signals used.");

        ImGui::EndPopup();
    }

    return changed;
}

void to_json(json& j, const CycleSlipDetector& data)
{
    j = json{
        { "enableLLICheck", data._enableLLICheck },
        { "singleFrequencyThresholdPercentage", data._singleFrequencyThresholdPercentage },
        { "dualFrequencyThresholdPercentage", data._dualFrequencyThresholdPercentage },
        { "singleFrequencyDetector", data._singleFrequencyDetector },
        { "dualFrequencyDetector", data._dualFrequencyDetector },
    };
}
void from_json(const json& j, CycleSlipDetector& data)
{
    if (j.contains("enableLLICheck")) { j.at("enableLLICheck").get_to(data._enableLLICheck); }
    if (j.contains("singleFrequencyThresholdPercentage")) { j.at("singleFrequencyThresholdPercentage").get_to(data._singleFrequencyThresholdPercentage); }
    if (j.contains("dualFrequencyThresholdPercentage")) { j.at("dualFrequencyThresholdPercentage").get_to(data._dualFrequencyThresholdPercentage); }
    if (j.contains("singleFrequencyDetector")) { j.at("singleFrequencyDetector").get_to(data._singleFrequencyDetector); }
    if (j.contains("dualFrequencyDetector")) { j.at("dualFrequencyDetector").get_to(data._dualFrequencyDetector); }
}

std::string to_string(const CycleSlipDetector::Result& cycleSlip)
{
    if (const auto* s = std::get_if<CycleSlipDetector::CycleSlipLossOfLockIndicator>(&cycleSlip))
    {
        return fmt::format("Cycle-slip [{}] ({})", s->signal, "LLI set");
    }
    if (const auto* s = std::get_if<CycleSlipDetector::CycleSlipSingleFrequency>(&cycleSlip))
    {
        return fmt::format("Cycle-slip [{}] ({})", s->signal, "single frequency check");
    }
    if (const auto* s = std::get_if<CycleSlipDetector::CycleSlipDualFrequency>(&cycleSlip))
    {
        return fmt::format("Cycle-slip [{}] & [{}] ({})", s->signals[0], s->signals[1], "dual frequency check");
    }

    return "";
}

} // namespace NAV

std::ostream& operator<<(std::ostream& os, const NAV::CycleSlipDetector::Result& obj)
{
    return os << fmt::format("{}", obj);
}