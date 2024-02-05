// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "GnssAnalyzer.hpp"

#define IMGUI_DEFINE_MATH_OPERATORS
#include <imgui_internal.h>

#include "util/Logger.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "internal/gui/widgets/HelpMarker.hpp"

#include "Navigation/Constants.hpp"
#include "Navigation/GNSS/Core/Code.hpp"
#include "Navigation/GNSS/Core/SatelliteIdentifier.hpp"

#include "NodeData/GNSS/GnssObs.hpp"
#include "NodeData/GNSS/GnssCombination.hpp"

namespace NAV
{

/// @brief Write info to a json object
/// @param[out] j Json output
/// @param[in] data Object to read info from
void to_json(json& j, const GnssAnalyzer::Combination::Term& data)
{
    j = json{
        { "obsType", data.obsType },
        { "satSigId", data.satSigId },
        { "sign", data.sign },
    };
}
/// @brief Read info from a json object
/// @param[in] j Json variable to read info from
/// @param[out] data Output object
void from_json(const json& j, GnssAnalyzer::Combination::Term& data)
{
    if (j.contains("obsType")) { j.at("obsType").get_to(data.obsType); }
    if (j.contains("satSigId")) { j.at("satSigId").get_to(data.satSigId); }
    if (j.contains("sign")) { j.at("sign").get_to(data.sign); }
}

/// @brief Write info to a json object
/// @param[out] j Json output
/// @param[in] data Object to read info from
void to_json(json& j, const GnssAnalyzer::Combination& data)
{
    j = json{
        { "description", data.description() },
        { "terms", data.terms },
        { "unit", data.unit },
        { "polynomialCycleSlipDetector", data.polynomialCycleSlipDetector },
        { "polynomialCycleSlipDetector.thresholdPercentage", data.polynomialCycleSlipDetectorThresholdPercentage },
        { "polynomialCycleSlipDetector.outputWhenWindowSizeNotReached", data.polynomialCycleSlipDetectorOutputWhenWindowSizeNotReached },
        { "polynomialCycleSlipDetector.outputPolynomials", data.polynomialCycleSlipDetectorOutputPolynomials },
    };
}
/// @brief Read info from a json object
/// @param[in] j Json variable to read info from
/// @param[out] data Output object
void from_json(const json& j, GnssAnalyzer::Combination& data)
{
    if (j.contains("terms"))
    {
        j.at("terms").get_to(data.terms);
    }
    if (j.contains("unit"))
    {
        j.at("unit").get_to(data.unit);
    }
    if (j.contains("polynomialCycleSlipDetector"))
    {
        j.at("polynomialCycleSlipDetector").get_to(data.polynomialCycleSlipDetector);
    }
    if (j.contains("polynomialCycleSlipDetector.thresholdPercentage"))
    {
        j.at("polynomialCycleSlipDetector.thresholdPercentage").get_to(data.polynomialCycleSlipDetectorThresholdPercentage);
    }
    if (j.contains("polynomialCycleSlipDetector.outputWhenWindowSizeNotReached"))
    {
        j.at("polynomialCycleSlipDetector.outputWhenWindowSizeNotReached").get_to(data.polynomialCycleSlipDetectorOutputWhenWindowSizeNotReached);
    }
    if (j.contains("polynomialCycleSlipDetector.outputPolynomials"))
    {
        j.at("polynomialCycleSlipDetector.outputPolynomials").get_to(data.polynomialCycleSlipDetectorOutputPolynomials);
    }
}

} // namespace NAV

NAV::GnssAnalyzer::GnssAnalyzer()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 630, 410 };

    nm::CreateOutputPin(this, "GnssComb", Pin::Type::Flow, { NAV::GnssCombination::type() });

    nm::CreateInputPin(this, "GnssObs", Pin::Type::Flow, { NAV::GnssObs::type() }, &GnssAnalyzer::receiveGnssObs);
}

NAV::GnssAnalyzer::~GnssAnalyzer()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::GnssAnalyzer::typeStatic()
{
    return "Gnss Analyzer";
}

std::string NAV::GnssAnalyzer::type() const
{
    return typeStatic();
}

std::string NAV::GnssAnalyzer::category()
{
    return "Data Processor";
}

void NAV::GnssAnalyzer::guiConfig()
{
    ImGui::TextUnformatted("Create combinations of GNSS measurements by adding or subtracting signals.");

    std::vector<size_t> combToDelete;
    for (size_t c = 0; c < _combinations.size(); c++)
    {
        auto& comb = _combinations.at(c);

        bool keepCombination = true;
        if (ImGui::CollapsingHeader(fmt::format("Combination {}##id{}", c, size_t(id)).c_str(),
                                    _combinations.size() > 1 ? &keepCombination : nullptr, ImGuiTreeNodeFlags_DefaultOpen))
        {
            if (ImGui::Button(fmt::format("Add Term##id{} c{}", size_t(id), c).c_str()))
            {
                flow::ApplyChanges();
                comb.terms.emplace_back();
                if (comb.terms.size() != 2) { comb.polynomialCycleSlipDetector.setEnabled(false); }
            }

            ImGui::SameLine();
            int selected = comb.unit == Combination::Unit::Meters ? 0 : 1;
            ImGui::SetNextItemWidth(100.0F);
            if (ImGui::Combo(fmt::format("Output unit##id{} c{}", size_t(id), c).c_str(), &selected, "Meters\0Cycles\0\0"))
            {
                flow::ApplyChanges();
                comb.unit = selected == 0 ? Combination::Unit::Meters : Combination::Unit::Cycles;
            }

            ImGui::SameLine();
            if (ImGui::Button(fmt::format("Cycle-slip detector##id{} c{}", size_t(id), c).c_str()))
            {
                ImGui::OpenPopup(fmt::format("Cycle-slip detector##Popup - id{} c{}", size_t(id), c).c_str());
            }
            if (ImGui::BeginPopup(fmt::format("Cycle-slip detector##Popup - id{} c{}", size_t(id), c).c_str()))
            {
                constexpr float WIDTH = 145.0F;
                if (PolynomialCycleSlipDetectorGui(fmt::format("Cycle-slip detector id{} c{}", size_t(id), c).c_str(),
                                                   comb.polynomialCycleSlipDetector, WIDTH))
                {
                    flow::ApplyChanges();
                }

                ImGui::SetNextItemWidth(WIDTH);
                if (double val = comb.polynomialCycleSlipDetectorThresholdPercentage * 100.0;
                    ImGui::DragDouble(fmt::format("Threshold##id{} c{}", size_t(id), c).c_str(), &val, 1.0F,
                                      1.0, std::numeric_limits<double>::max(), "%.2f %%"))
                {
                    comb.polynomialCycleSlipDetectorThresholdPercentage = val / 100.0;
                    flow::ApplyChanges();
                }
                ImGui::SameLine();
                std::string description = "As percentage of the smallest wavelength of the combination terms.";
                if (auto maxF = std::max_element(comb.terms.begin(), comb.terms.end(), [](const Combination::Term& a, const Combination::Term& b) {
                        return a.satSigId.freq().getFrequency(a.freqNum) < b.satSigId.freq().getFrequency(b.freqNum);
                    });
                    maxF != comb.terms.end())
                {
                    double lambda = InsConst::C / maxF->satSigId.freq().getFrequency(maxF->freqNum);
                    double threshold = comb.polynomialCycleSlipDetectorThresholdPercentage * lambda;
                    description += fmt::format("\nFor [{} {}] the wavelength is λ = {:.3f} [m].\nThe threshold is then {:.3f} [m].",
                                               maxF->satSigId.toSatId().satSys, maxF->satSigId.freq(), lambda, threshold);
                }
                gui::widgets::HelpMarker(description.c_str());

                if (!comb.polynomialCycleSlipDetector.isEnabled()) { ImGui::BeginDisabled(); }
                if (ImGui::Checkbox(fmt::format("Output when insufficient points##id{} c{}", size_t(id), c).c_str(), &comb.polynomialCycleSlipDetectorOutputWhenWindowSizeNotReached))
                {
                    flow::ApplyChanges();
                }
                if (ImGui::Checkbox(fmt::format("Output polynomials##id{} c{}", size_t(id), c).c_str(), &comb.polynomialCycleSlipDetectorOutputPolynomials))
                {
                    flow::ApplyChanges();
                }
                if (!comb.polynomialCycleSlipDetector.isEnabled()) { ImGui::EndDisabled(); }

                ImGui::EndPopup();
            }

            std::vector<size_t> termToDelete;
            if (ImGui::BeginTable(fmt::format("##Table id{} c{}", size_t(id), c).c_str(), 3 * static_cast<int>(comb.terms.size()) + 1,
                                  ImGuiTableFlags_NoHostExtendX | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_ScrollX,
                                  ImVec2(0, 70.0F)))
            {
                ImGui::TableNextRow();

                for (size_t t = 0; t < comb.terms.size(); t++)
                {
                    auto& term = comb.terms.at(t);

                    ImGui::TableSetColumnIndex(static_cast<int>(t) * 3);
                    int selected = term.sign == 1 ? 0 : 1;
                    ImGui::SetNextItemWidth(50.0F);
                    if (ImGui::Combo(fmt::format("##Sign id{} c{} t{}", size_t(id), c, t).c_str(), &selected, "+1\0-1\0\0"))
                    {
                        flow::ApplyChanges();
                        term.sign = selected == 0 ? +1 : -1;
                    }

                    ImGui::TableSetColumnIndex(static_cast<int>(t) * 3 + 1);
                    selected = term.obsType == Combination::Term::ObservationType::Pseudorange ? 0 : 1;
                    ImGui::SetNextItemWidth(62.0F);
                    if (ImGui::Combo(fmt::format("##ObsType id{} c{} t{}", size_t(id), c, t).c_str(), &selected, comb.unit == Combination::Unit::Cycles ? "P\0Φ\0\0" : "p\0φ\0\0"))
                    {
                        flow::ApplyChanges();
                        term.obsType = selected == 0 ? Combination::Term::ObservationType::Pseudorange : Combination::Term::ObservationType::Carrier;
                    }

                    ImGui::TableSetColumnIndex(static_cast<int>(t) * 3 + 2);
                    ImGui::SetNextItemWidth(62.0F);
                    if (ShowCodeSelector(fmt::format("##Code id{} c{} t{}", size_t(id), c, t).c_str(), term.satSigId.code, Freq_All, true))
                    {
                        flow::ApplyChanges();
                    }
                    ImGui::SameLine();
                    ImGui::Dummy(ImVec2(10.0F, 0.0F));
                }
                ImGui::TableNextColumn();
                ImGui::TextUnformatted("= Combined Frequency");

                ImGui::TableNextRow();
                for (size_t t = 0; t < comb.terms.size(); t++)
                {
                    auto& term = comb.terms.at(t);

                    ImGui::TableSetColumnIndex(static_cast<int>(t) * 3);
                    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 8.0F);
                    float radius = 8.0F;
                    ImGui::GetWindowDrawList()->AddCircleFilled(ImGui::GetCursorScreenPos() + ImVec2(radius, ImGui::GetTextLineHeight() / 2.0F + 2.0F), radius,
                                                                term.receivedDuringRun ? IM_COL32(0, 255, 0, 255) : IM_COL32(255, 0, 0, 255));
                    ImGui::Dummy(ImVec2(radius * 2.0F, ImGui::GetTextLineHeight()));
                    if (ImGui::IsItemHovered()) { ImGui::SetTooltip(term.receivedDuringRun ? "Signal was received" : "Signal was not received"); }

                    if (comb.terms.size() > 1)
                    {
                        ImGui::SameLine();
                        if (ImGui::Button(fmt::format("X##id{} c{} t{}", size_t(id), c, t).c_str()))
                        {
                            flow::ApplyChanges();
                            termToDelete.push_back(t);
                        }
                        if (ImGui::IsItemHovered()) { ImGui::SetTooltip("Remove Term?"); }
                    }

                    ImGui::TableSetColumnIndex(static_cast<int>(t) * 3 + 1);
                    double f = term.satSigId.freq().getFrequency(term.freqNum);
                    ImGui::TextUnformatted(fmt::format("{:.2f}", f * 1e-6).c_str());
                    if (ImGui::IsItemHovered()) { ImGui::SetTooltip("%s", fmt::format("Frequency in [Mhz]\nλ = {:.3f}m", InsConst::C / f).c_str()); }

                    ImGui::TableSetColumnIndex(static_cast<int>(t) * 3 + 2);
                    ImGui::SetNextItemWidth(62.0F);
                    SatId satId = SatId(term.satSigId.toSatId().satSys, term.satSigId.satNum);
                    if (ShowSatelliteSelector(fmt::format("##SatNum id{} c{} t{}", size_t(id), c, t).c_str(), satId, satId.satSys, true))
                    {
                        term.satSigId.satNum = satId.satNum;
                        flow::ApplyChanges();
                    }
                }
                ImGui::TableNextColumn();
                double f = comb.calcCombinationFrequency();
                ImGui::TextUnformatted(fmt::format("          {:.2f} MHz", f * 1e-6).c_str());
                if (ImGui::IsItemHovered()) { ImGui::SetTooltip("%s", fmt::format("λ = {:.3f}m", InsConst::C / f).c_str()); }

                ImGui::EndTable();
            }

            for (const auto& t : termToDelete) { comb.terms.erase(std::next(comb.terms.begin(), static_cast<std::ptrdiff_t>(t))); }
        }

        if (!keepCombination) { combToDelete.push_back(c); }
    }
    for (const auto& c : combToDelete) { _combinations.erase(std::next(_combinations.begin(), static_cast<std::ptrdiff_t>(c))); }

    ImGui::Separator();
    if (ImGui::Button(fmt::format("Add Combination##id{}", size_t(id)).c_str()))
    {
        flow::ApplyChanges();
        _combinations.emplace_back();
    }
}

[[nodiscard]] json NAV::GnssAnalyzer::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["combinations"] = _combinations;

    return j;
}

void NAV::GnssAnalyzer::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("combinations"))
    {
        j.at("combinations").get_to(_combinations);
    }
}

bool NAV::GnssAnalyzer::initialize()
{
    LOG_TRACE("{}: called", nameId());

    for (auto& comb : _combinations)
    {
        comb.polynomialCycleSlipDetector.clear();
        comb.polynomials.clear();
        for (auto& term : comb.terms)
        {
            term.receivedDuringRun = false;
        }
    }

    return true;
}

void NAV::GnssAnalyzer::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    for (auto& comb : _combinations)
    {
        for (auto& term : comb.terms)
        {
            term.receivedDuringRun = false;
        }
    }
}

void NAV::GnssAnalyzer::receiveGnssObs(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    auto gnssObs = std::static_pointer_cast<const GnssObs>(queue.extract_front());
    LOG_DATA("{}: Received GnssObs for [{}]", nameId(), gnssObs->insTime);

    auto gnssComb = std::make_shared<GnssCombination>();
    gnssComb->insTime = gnssObs->insTime;

    for (size_t c = 0; c < _combinations.size(); c++)
    {
        auto& comb = _combinations.at(c);
        GnssCombination::Combination combination;
        combination.description = comb.description();
        for (size_t i = 0; i < _combinations.size(); i++)
        {
            if (i == c) { continue; }
            if (combination.description == _combinations.at(i).description())
            {
                combination.description += fmt::format(" - {}", c);
                break;
            }
        }

        double result = 0.0;
        double lambdaMin = 100.0;
        size_t termsFound = 0;
        for (auto& term : comb.terms)
        {
            GnssCombination::Combination::Term oTerm;
            oTerm.sign = term.sign;
            oTerm.satSigId = term.satSigId;
            oTerm.obsType = term.obsType == Combination::Term::ObservationType::Pseudorange
                                ? GnssObs::ObservationType::Pseudorange
                                : GnssObs::ObservationType::Carrier;

            double freq = term.satSigId.freq().getFrequency(term.freqNum);
            double lambda = InsConst::C / freq;
            lambdaMin = std::min(lambdaMin, lambda);

            if (auto obs = (*gnssObs)(term.satSigId))
            {
                if (term.obsType == Combination::Term::ObservationType::Pseudorange)
                {
                    if (auto psr = obs->get().pseudorange)
                    {
                        term.receivedDuringRun = true;

                        double value = psr->value;
                        result += static_cast<double>(term.sign) * value;

                        if (comb.unit == Combination::Unit::Cycles)
                        {
                            value /= lambda;
                        }
                        oTerm.value = value;

                        termsFound++;
                    }
                    else
                    {
                        comb.polynomialCycleSlipDetector.reset(comb.description());
                    }
                }
                else // if (term.obsType == Combination::Term::ObservationType::Carrier)
                {
                    if (auto carrier = obs->get().carrierPhase)
                    {
                        term.receivedDuringRun = true;

                        double value = carrier->value * lambda;
                        result += static_cast<double>(term.sign) * value;

                        if (comb.unit == Combination::Unit::Cycles)
                        {
                            value /= lambda;
                        }
                        oTerm.value = value;

                        termsFound++;
                    }
                    else
                    {
                        comb.polynomialCycleSlipDetector.reset(comb.description());
                    }
                }
            }

            combination.terms.push_back(oTerm);
        }
        if (termsFound == comb.terms.size())
        {
            auto lambda = InsConst::C / comb.calcCombinationFrequency();
            double resultCycles = result / lambda;
            combination.result = comb.unit == Combination::Unit::Cycles ? resultCycles : result;

            if (comb.polynomialCycleSlipDetector.isEnabled())
            {
                auto key = comb.description();
                combination.cycleSlipPrediction = comb.polynomialCycleSlipDetector.predictValue(key, gnssComb->insTime);
                if (combination.cycleSlipPrediction.has_value())
                {
                    combination.cycleSlipMeasMinPred = *combination.result - *combination.cycleSlipPrediction;
                }

                if (comb.polynomialCycleSlipDetectorOutputPolynomials)
                {
                    if (auto polynomial = comb.polynomialCycleSlipDetector.calcPolynomial(key))
                    {
                        comb.polynomials.emplace_back(gnssComb->insTime, *polynomial);
                    }
                    if (auto relTime = comb.polynomialCycleSlipDetector.calcRelativeTime(key, gnssComb->insTime))
                    {
                        for (const auto& poly : comb.polynomials)
                        {
                            double value = poly.second.f(*relTime);
                            LOG_DATA("f({:.2f}) = {:.2f} ({})", *relTime, value, poly.second.toString());
                            combination.cycleSlipPolynomials.emplace_back(poly.first, poly.second, value);
                        }
                    }
                }
                double threshold = comb.polynomialCycleSlipDetectorThresholdPercentage * lambdaMin;
                if (comb.unit == Combination::Unit::Cycles) { threshold /= lambda; }

                combination.cycleSlipResult = comb.polynomialCycleSlipDetector.checkForCycleSlip(key, gnssComb->insTime, *combination.result, threshold);
                if (!comb.polynomialCycleSlipDetectorOutputWhenWindowSizeNotReached
                    && *combination.cycleSlipResult == PolynomialCycleSlipDetectorResult::LessDataThanWindowSize)
                {
                    combination.cycleSlipPrediction.reset();
                    combination.cycleSlipMeasMinPred.reset();
                }
            }
        }

        gnssComb->combinations.push_back(combination);
    }

    invokeCallbacks(OUTPUT_PORT_INDEX_GNSS_COMBINATION, gnssComb);
}