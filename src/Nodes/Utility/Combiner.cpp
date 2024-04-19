// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Combiner.hpp"

#define IMGUI_DEFINE_MATH_OPERATORS
#include <imgui_internal.h>

#include "util/Assert.h"
#include "util/Logger.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "internal/gui/widgets/HelpMarker.hpp"

#include "NodeData/General/DynamicData.hpp"

namespace NAV
{

/// @brief Write info to a json object
/// @param[out] j Json output
/// @param[in] data Object to read info from
void to_json(json& j, const Combiner::Combination::Term& data)
{
    j = json{
        { "factor", data.factor },
        { "pinIndex", data.pinIndex },
        { "dataSelection", data.dataSelection },
    };
}
/// @brief Read info from a json object
/// @param[in] j Json variable to read info from
/// @param[out] data Output object
void from_json(const json& j, Combiner::Combination::Term& data)
{
    if (j.contains("factor")) { j.at("factor").get_to(data.factor); }
    if (j.contains("pinIndex")) { j.at("pinIndex").get_to(data.pinIndex); }
    if (j.contains("dataSelection")) { j.at("dataSelection").get_to(data.dataSelection); }
}

/// @brief Write info to a json object
/// @param[out] j Json output
/// @param[in] data Object to read info from
void to_json(json& j, const Combiner::Combination& data)
{
    j = json{
        { "terms", data.terms },
    };
}
/// @brief Read info from a json object
/// @param[in] j Json variable to read info from
/// @param[out] data Output object
void from_json(const json& j, Combiner::Combination& data)
{
    if (j.contains("terms")) { j.at("terms").get_to(data.terms); }
}

Combiner::Combiner()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 430, 410 };

    _dynamicInputPins.addPin(this);
    _dynamicInputPins.addPin(this);
    nm::CreateOutputPin(this, "Comb", Pin::Type::Flow, { DynamicData::type() });
}

Combiner::~Combiner()
{
    LOG_TRACE("{}: called", nameId());
}

std::string Combiner::typeStatic()
{
    return "Combiner";
}

std::string Combiner::type() const
{
    return typeStatic();
}

std::string Combiner::category()
{
    return "Utility";
}

void Combiner::guiConfig()
{
    ImGui::SetNextItemOpen(false, ImGuiCond_FirstUseEver);
    if (ImGui::CollapsingHeader(fmt::format("Pins##{}", size_t(id)).c_str()))
    {
        std::vector<size_t> pinIds;
        pinIds.reserve(inputPins.size());
        for (const auto& pin : inputPins) { pinIds.push_back(size_t(pin.id)); }
        if (_dynamicInputPins.ShowGuiWidgets(size_t(id), inputPins, this))
        {
            std::vector<size_t> inputPinIds;
            inputPinIds.reserve(inputPins.size());
            for (const auto& pin : inputPins) { inputPinIds.push_back(size_t(pin.id)); }
            LOG_DATA("{}: old Input pin ids {}", nameId(), fmt::join(pinIds, ", "));
            LOG_DATA("{}: new Input pin ids {}", nameId(), fmt::join(inputPinIds, ", "));
            if (inputPins.size() == pinIds.size()) // Reorder performed
            {
                for (size_t i = 0; i < pinIds.size(); i++)
                {
                    auto pinId = pinIds.at(i);
                    const auto& inputPinId = size_t(inputPins.at(i).id);
                    if (pinId != inputPinId)
                    {
                        size_t newPinIdx = inputPinIndexFromId(pinId);
                        LOG_DATA("{}: Pin {} moved index {} -> {}", nameId(), pinId, i, newPinIdx);
                        for (auto& comb : _combinations)
                        {
                            for (auto& term : comb.terms)
                            {
                                if (term.pinIndex == i) { term.pinIndex = newPinIdx + 10000; }
                            }
                        }
                    }
                }
                for (auto& comb : _combinations)
                {
                    for (auto& term : comb.terms)
                    {
                        if (term.pinIndex >= 10000) { term.pinIndex -= 10000; }
                    }
                }
            }
            flow::ApplyChanges();
        }
    }

    std::vector<size_t> combToDelete;
    for (size_t c = 0; c < _combinations.size(); c++)
    {
        auto& comb = _combinations.at(c);

        bool keepCombination = true;
        if (ImGui::CollapsingHeader(fmt::format("{}##id{} c{}", comb.description(this), size_t(id), c).c_str(),
                                    _combinations.size() > 1 ? &keepCombination : nullptr, ImGuiTreeNodeFlags_DefaultOpen))
        {
            constexpr int COL_PER_TERM = 3;
            constexpr float COL_SIGN_WIDTH = 50.0F;
            constexpr float COL_PIN_WIDTH = 100.0F;

            std::vector<size_t> termToDelete;
            bool addTerm = false;
            if (ImGui::BeginTable(fmt::format("##Table id{} c{}", size_t(id), c).c_str(), COL_PER_TERM * static_cast<int>(comb.terms.size()),
                                  ImGuiTableFlags_NoHostExtendX | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_ScrollX,
                                  ImVec2(0, 70.0F)))
            {
                ImGui::TableNextRow();

                for (size_t t = 0; t < comb.terms.size(); t++)
                {
                    auto& term = comb.terms.at(t);

                    ImGui::TableSetColumnIndex(static_cast<int>(t) * COL_PER_TERM);
                    ImGui::SetNextItemWidth(COL_SIGN_WIDTH);
                    if (ImGui::InputDouble(fmt::format("##factor id{} c{} t{}", size_t(id), c, t).c_str(), &term.factor, 0.0, 0.0, "%.2f"))
                    {
                        flow::ApplyChanges();
                    }
                    ImGui::SameLine();
                    ImGui::TextUnformatted("*");

                    ImGui::SameLine();
                    ImGui::SetNextItemWidth(COL_PIN_WIDTH);
                    if (ImGui::BeginCombo(fmt::format("##pinIndex id{} c{} t{}", size_t(id), c, t).c_str(), inputPins.at(term.pinIndex).name.c_str()))
                    {
                        for (size_t i = 0; i < inputPins.size(); i++)
                        {
                            const bool is_selected = term.pinIndex == i;
                            if (ImGui::Selectable(inputPins.at(i).name.c_str(), is_selected))
                            {
                                term.pinIndex = i;
                                term.dataSelection = size_t(0);
                                flow::ApplyChanges();
                            }
                            if (is_selected) { ImGui::SetItemDefaultFocus(); }
                        }
                        ImGui::EndCombo();
                    }

                    ImGui::TableSetColumnIndex(static_cast<int>(t) * COL_PER_TERM + 1);
                    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 2.0F);
                    float radius = 8.0F;
                    ImGui::GetWindowDrawList()->AddCircleFilled(ImGui::GetCursorScreenPos() + ImVec2(radius, ImGui::GetTextLineHeight() / 2.0F + 2.0F), radius,
                                                                term.polyReg.empty() ? IM_COL32(255, 0, 0, 255) : IM_COL32(0, 255, 0, 255));
                    ImGui::Dummy(ImVec2(radius * 2.0F, ImGui::GetTextLineHeight()));
                    if (ImGui::IsItemHovered()) { ImGui::SetTooltip(term.polyReg.empty() ? "Signal was not received" : "Signal was received"); }

                    ImGui::TableSetColumnIndex(static_cast<int>(t) * COL_PER_TERM + 2);
                    if (static_cast<int>(t) * COL_PER_TERM + 2 == COL_PER_TERM * static_cast<int>(comb.terms.size()) - 1)
                    {
                        addTerm |= ImGui::Button(fmt::format("+## Add Term id{} c{}", size_t(id), c).c_str());
                        if (ImGui::IsItemHovered()) { ImGui::SetTooltip("Add Term?"); }
                    }
                    else
                    {
                        ImGui::TextUnformatted("+");
                    }
                }

                ImGui::TableNextRow();
                for (size_t t = 0; t < comb.terms.size(); t++)
                {
                    auto& term = comb.terms.at(t);

                    ImGui::TableSetColumnIndex(static_cast<int>(t) * COL_PER_TERM);
                    auto dataDescriptors = getDataDescriptors(term.pinIndex);
                    if ((std::holds_alternative<size_t>(term.dataSelection) && std::get<size_t>(term.dataSelection) < dataDescriptors.size())
                        || std::holds_alternative<std::string>(term.dataSelection))
                    {
                        std::string previewText = std::holds_alternative<size_t>(term.dataSelection)
                                                      ? dataDescriptors.at(std::get<size_t>(term.dataSelection))
                                                      : std::get<std::string>(term.dataSelection);
                        float comboWidth = COL_SIGN_WIDTH + COL_PIN_WIDTH + ImGui::CalcTextSize("*").x + 2.0F * ImGui::GetStyle().ItemSpacing.x;
                        ImGui::SetNextItemWidth(comboWidth);
                        if (ImGui::BeginCombo(fmt::format("##dataIndex id{} c{} t{}", size_t(id), c, t).c_str(), previewText.c_str()))
                        {
                            for (size_t i = 0; i < dataDescriptors.size(); i++)
                            {
                                const bool is_selected = std::holds_alternative<size_t>(term.dataSelection)
                                                             ? std::get<size_t>(term.dataSelection) == i
                                                             : dataDescriptors.at(i) == previewText;
                                if (ImGui::Selectable(dataDescriptors.at(i).c_str(), is_selected))
                                {
                                    if (dataDescriptors.size() - i <= _pinData.at(term.pinIndex).dynDataDescriptors.size())
                                    {
                                        term.dataSelection = dataDescriptors.at(i);
                                    }
                                    else
                                    {
                                        term.dataSelection = i;
                                    }
                                    for (size_t t2 = 0; t2 < comb.terms.size(); t2++) // Set other terms to same data if possible
                                    {
                                        if (t2 == t) { continue; }
                                        auto& term2 = comb.terms.at(t2);
                                        auto dataDescriptors2 = getDataDescriptors(term2.pinIndex);
                                        auto iter = std::find(dataDescriptors2.begin(), dataDescriptors2.end(),
                                                              std::holds_alternative<size_t>(term.dataSelection)
                                                                  ? dataDescriptors.at(std::get<size_t>(term.dataSelection))
                                                                  : std::get<std::string>(term.dataSelection));
                                        if (iter != dataDescriptors2.end())
                                        {
                                            if (std::holds_alternative<size_t>(term.dataSelection))
                                            {
                                                term2.dataSelection = static_cast<size_t>(std::distance(dataDescriptors2.begin(), iter));
                                            }
                                            else
                                            {
                                                term2.dataSelection = term.dataSelection;
                                            }
                                        }
                                    }

                                    flow::ApplyChanges();
                                }
                                if (is_selected) { ImGui::SetItemDefaultFocus(); }
                            }
                            ImGui::EndCombo();
                        }
                        if (ImGui::IsItemHovered() && ImGui::CalcTextSize(previewText.c_str()).x > comboWidth - 15.0F)
                        {
                            ImGui::SetTooltip("%s", previewText.c_str());
                        }
                    }
                    else if (inputPins.at(term.pinIndex).isPinLinked())
                    {
                        ImGui::TextUnformatted("Please run the flow");
                        ImGui::SameLine();
                        gui::widgets::HelpMarker("Available data is collected when running the flow");
                    }

                    if (comb.terms.size() > 1)
                    {
                        ImGui::TableSetColumnIndex(static_cast<int>(t) * COL_PER_TERM + 1);
                        if (ImGui::Button(fmt::format("X##id{} c{} t{}", size_t(id), c, t).c_str()))
                        {
                            flow::ApplyChanges();
                            termToDelete.push_back(t);
                        }
                        if (ImGui::IsItemHovered()) { ImGui::SetTooltip("Remove Term?"); }
                    }
                }

                ImGui::EndTable();
            }

            for (const auto& t : termToDelete) { comb.terms.erase(std::next(comb.terms.begin(), static_cast<std::ptrdiff_t>(t))); }
            if (addTerm)
            {
                flow::ApplyChanges();
                comb.terms.emplace_back();
            }
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

[[nodiscard]] json Combiner::save() const
{
    LOG_TRACE("{}: called", nameId());

    return {
        { "dynamicInputPins", _dynamicInputPins },
        { "combinations", _combinations },
    };
}

void Combiner::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("dynamicInputPins")) { NAV::gui::widgets::from_json(j.at("dynamicInputPins"), _dynamicInputPins, this); }
    if (j.contains("combinations")) { j.at("combinations").get_to(_combinations); }
}

void Combiner::pinAddCallback(Node* node)
{
    auto* combiner = static_cast<Combiner*>(node); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)
    combiner->_pinData.emplace_back();
    nm::CreateInputPin(node, fmt::format("Pin {}", node->inputPins.size() + 1).c_str(), Pin::Type::Flow, _dataIdentifier, &Combiner::receiveData);
}

void Combiner::pinDeleteCallback(Node* node, size_t pinIdx)
{
    auto* combiner = static_cast<Combiner*>(node); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)
    combiner->_pinData.erase(std::next(combiner->_pinData.begin(), static_cast<int64_t>(pinIdx)));
    nm::DeleteInputPin(node->inputPins.at(pinIdx));

    for (auto& comb : combiner->_combinations)
    {
        for (int64_t t = 0; t < static_cast<int64_t>(comb.terms.size()); ++t)
        {
            auto& term = comb.terms.at(static_cast<size_t>(t));
            if (term.pinIndex == pinIdx) // The index we want to delete
            {
                comb.terms.erase(comb.terms.begin() + t);
                --t;
            }
            else if (term.pinIndex > pinIdx) // Index higher -> Decrement
            {
                --(term.pinIndex);
            }
        }
    }
}

bool Combiner::initialize()
{
    LOG_TRACE("{}: called", nameId());

    CommonLog::initialize();

    for (auto& comb : _combinations)
    {
        for (auto& term : comb.terms)
        {
            term.polyReg.reset();
        }
    }

    for (auto& pinData : _pinData)
    {
        pinData.dynDataDescriptors.clear();
    }

    _sendRequests.clear();

    return true;
}

void Combiner::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

std::vector<std::string> Combiner::getDataDescriptors(size_t pinIndex) const
{
    std::vector<std::string> dataDescriptors;
    if (auto* sourcePin = inputPins.at(pinIndex).link.getConnectedPin())
    {
        if (sourcePin->dataIdentifier.front() == Pos::type()) { dataDescriptors = Pos::GetStaticDataDescriptors(); }
        else if (sourcePin->dataIdentifier.front() == PosVel::type()) { dataDescriptors = PosVel::GetStaticDataDescriptors(); }
        else if (sourcePin->dataIdentifier.front() == PosVelAtt::type()) { dataDescriptors = PosVelAtt::GetStaticDataDescriptors(); }
        else if (sourcePin->dataIdentifier.front() == LcKfInsGnssErrors::type()) { dataDescriptors = LcKfInsGnssErrors::GetStaticDataDescriptors(); }
        else if (sourcePin->dataIdentifier.front() == TcKfInsGnssErrors::type()) { dataDescriptors = TcKfInsGnssErrors::GetStaticDataDescriptors(); }
        else if (sourcePin->dataIdentifier.front() == SppSolution::type()) { dataDescriptors = SppSolution::GetStaticDataDescriptors(); }
        else if (sourcePin->dataIdentifier.front() == RtklibPosObs::type()) { dataDescriptors = RtklibPosObs::GetStaticDataDescriptors(); }
        else if (sourcePin->dataIdentifier.front() == ImuObs::type()) { dataDescriptors = ImuObs::GetStaticDataDescriptors(); }
        else if (sourcePin->dataIdentifier.front() == ImuObsSimulated::type()) { dataDescriptors = ImuObsSimulated::GetStaticDataDescriptors(); }
        else if (sourcePin->dataIdentifier.front() == KvhObs::type()) { dataDescriptors = KvhObs::GetStaticDataDescriptors(); }
        else if (sourcePin->dataIdentifier.front() == ImuObsWDelta::type()) { dataDescriptors = ImuObsWDelta::GetStaticDataDescriptors(); }
        else if (sourcePin->dataIdentifier.front() == VectorNavBinaryOutput::type()) { dataDescriptors = VectorNavBinaryOutput::GetStaticDataDescriptors(); }
    }
    if (!_pinData.at(pinIndex).dynDataDescriptors.empty())
    {
        dataDescriptors.reserve(dataDescriptors.size() + _pinData.at(pinIndex).dynDataDescriptors.size());
        std::copy(_pinData.at(pinIndex).dynDataDescriptors.begin(), _pinData.at(pinIndex).dynDataDescriptors.end(), std::back_inserter(dataDescriptors));
    }
    return dataDescriptors;
}

[[nodiscard]] bool Combiner::isLastObsThisEpoch(const InsTime& insTime) const
{
    return std::none_of(inputPins.begin(), inputPins.end(), [&insTime](const InputPin& pin) {
        return !pin.queue.empty() && pin.queue.front()->insTime == insTime;
    });
}

void Combiner::receiveData(InputPin::NodeDataQueue& queue, size_t pinIdx)
{
    auto nodeData = queue.extract_front();
    auto nodeDataTimeIntoRun = math::round(calcTimeIntoRun(nodeData->insTime), 8);
    LOG_DATA("{}: [{:.3f}s][{} ({})] Received obs for time [{} GPST] ", nameId(),
             nodeDataTimeIntoRun, inputPins.at(pinIdx).name, pinIdx, nodeData->insTime.toYMDHMS(GPST));

    // Add dynamic data descriptors to display in GUI
    auto& dataDescriptors = _pinData.at(pinIdx).dynDataDescriptors;
    const std::vector<std::string> nodeDataDescriptors = nodeData->dynamicDataDescriptors();
    for (const auto& desc : nodeDataDescriptors)
    {
        if (std::find(dataDescriptors.begin(), dataDescriptors.end(), desc) == dataDescriptors.end())
        {
            dataDescriptors.push_back(desc);
        }
    }

    auto* sourcePin = inputPins.at(pinIdx).link.getConnectedPin();
    if (sourcePin == nullptr) { return; }

    for (size_t c = 0; c < _combinations.size(); ++c)
    {
        auto& comb = _combinations.at(c);
        LOG_DATA("{}:   Combination: {}", nameId(), comb.description(this));
        for (size_t t = 0; t < comb.terms.size(); ++t)
        {
            auto& term = comb.terms.at(t);
            if (term.pinIndex != pinIdx) { continue; }
            if (std::holds_alternative<size_t>(term.dataSelection) && nodeData->staticDescriptorCount() <= std::get<size_t>(term.dataSelection)
                && std::get<size_t>(term.dataSelection) < dataDescriptors.size())
            {
                term.dataSelection = dataDescriptors.at(std::get<size_t>(term.dataSelection));
                flow::ApplyChanges();
            }

            auto value = std::holds_alternative<size_t>(term.dataSelection) ? nodeData->getValueAt(std::get<size_t>(term.dataSelection))
                                                                            : nodeData->getDynamicDataAt(std::get<std::string>(term.dataSelection));
            if (!value) { continue; }

            LOG_DATA("{}:     Term '{}': {:.3g}", nameId(), term.description(this, getDataDescriptors(term.pinIndex)), *value);
            term.polyReg.push_back(std::make_pair(nodeDataTimeIntoRun, *value));
            term.events.push_back(nodeData->events());
            if (!nodeData->events().empty())
            {
                LOG_DATA("{}:       NodeData has {} events", nameId(), nodeData->events().size());
            }

            // Check for all combinations with new info:

            if (std::any_of(comb.terms.begin(), comb.terms.end(), [&](const auto& t) {
                    bool termHasNoData = t.polyReg.empty();
                    if (termHasNoData)
                    {
                        LOG_DATA("{}:       Skipping, because no data on other term '{}' yet", nameId(), t.description(this, getDataDescriptors(t.pinIndex)));
                    }
                    return termHasNoData;
                }))
            {
                continue;
            }

            bool termInAnySendRequestFound = false;
            if (!_sendRequests.empty()) { LOG_DATA("{}:       Checking if term is in a send request", nameId()); }
            for (auto& [sendRequestTime, sendRequests] : _sendRequests)
            {
                for (auto& sendRequest : sendRequests)
                {
                    if (sendRequest.combIndex != c) { continue; }
                    const auto& srComb = _combinations.at(sendRequest.combIndex);

                    auto poly = term.polyReg.calcPolynomial();

                    for (const auto& srTerm : srComb.terms)
                    {
                        if (srTerm.pinIndex != term.pinIndex || srTerm.dataSelection != term.dataSelection) { continue; }
                        LOG_DATA("{}:         [{:.3f}s] Term found in combination and term is {}", nameId(), math::round(calcTimeIntoRun(sendRequestTime), 8),
                                 sendRequest.termIndices.contains(t) ? "already calculated." : "still missing");
                        // We found this term within the send requests
                        termInAnySendRequestFound = true;

                        if (!sendRequest.termIndices.contains(t)) // The send request was waiting for this term
                        {
                            LOG_DATA("{}:           Updating send request: {} {:.2f} * {:.3g} (by interpolating to time [{:.3f}s])", nameId(),
                                     sendRequest.result, term.factor, poly.f(math::round(calcTimeIntoRun(sendRequestTime), 8)),
                                     math::round(calcTimeIntoRun(sendRequestTime), 8));
                            sendRequest.termIndices.insert(t);
                            sendRequest.result += term.factor * poly.f(math::round(calcTimeIntoRun(sendRequestTime), 8));
                            while (!term.events.empty())
                            {
                                if (!term.events.front().empty())
                                {
                                    LOG_DATA("{}:           Adding {} events", nameId(), term.events.front().size());
                                    std::copy(term.events.front().begin(), term.events.front().end(), std::back_inserter(sendRequest.events));
                                }
                                term.events.pop_front();
                            }
                        }
                    }
                }
            }
            if (termInAnySendRequestFound) { continue; }

            LOG_DATA("{}:       Checking if term should be sent. Other terms must be between or at right border of times.", nameId());
            if (std::all_of(comb.terms.begin(), comb.terms.end(), [&](const Combination::Term& t) {
                    LOG_DATA("{}:          [{:.3f}s {:.3f}s] and [{:.3f}s] on '{}'", nameId(),
                             term.polyReg.data().front().first, term.polyReg.data().back().first,
                             t.polyReg.data().back().first, t.description(this, getDataDescriptors(t.pinIndex)));

                    return (!term.polyReg.windowSizeReached() && t.polyReg.data().back().first == term.polyReg.data().back().first)
                           || (term.polyReg.data().front().first < t.polyReg.data().back().first
                               && t.polyReg.data().back().first <= term.polyReg.data().back().first);
                }))
            {
                LOG_DATA("{}:       Adding new send request", nameId());
                SendRequest sr{
                    .combIndex = c,
                    .termIndices = {},
                    .result = 0.0,
                    .events = {},
                };
                for (size_t t = 0; t < comb.terms.size(); t++)
                {
                    auto& term = comb.terms.at(t);
                    auto [timeIntoRun, val] = term.polyReg.data().back();
                    if (timeIntoRun == nodeDataTimeIntoRun)
                    {
                        auto poly = term.polyReg.calcPolynomial();
                        LOG_DATA("{}:         {}: {:.2f} * {:.3g}", nameId(), term.description(this, getDataDescriptors(term.pinIndex)),
                                 term.factor, poly.f(nodeDataTimeIntoRun));
                        sr.termIndices.insert(t);
                        sr.result += term.factor * poly.f(nodeDataTimeIntoRun);
                        while (!term.events.empty())
                        {
                            if (!term.events.front().empty())
                            {
                                LOG_DATA("{}:         {}: Adding {} events", nameId(), term.description(this, getDataDescriptors(term.pinIndex)), term.events.front().size());
                                std::copy(term.events.front().begin(), term.events.front().end(), std::back_inserter(sr.events));
                            }
                            term.events.pop_front();
                        }
                    }
                }
                _sendRequests[nodeData->insTime].push_back(sr);
            }
            else
            {
                LOG_DATA("{}:       Not adding send request", nameId());
            }
        }
    }

    if (!_sendRequests.empty()) { LOG_DATA("{}:   Send requests", nameId()); }
    for (const auto& [sendRequestTime, sendRequests] : _sendRequests)
    {
        LOG_DATA("{}:     [{:.3f}s]", nameId(), math::round(calcTimeIntoRun(sendRequestTime), 8));
        for (const auto& sendRequest : sendRequests)
        {
            const auto& comb = _combinations.at(sendRequest.combIndex);
            LOG_DATA("{}:       Combination: {}", nameId(), comb.description(this));
            for (size_t t = 0; t < comb.terms.size(); ++t)
            {
                LOG_DATA("{}:         Term '{}' is {}", nameId(), comb.terms.at(t).description(this, getDataDescriptors(comb.terms.at(t).pinIndex)),
                         sendRequest.termIndices.contains(t) ? "added" : "missing");
            }
        }
    }

    if (isLastObsThisEpoch(nodeData->insTime))
    {
        LOG_DATA("{}: [{:.3f}s] Checking wether a send request can be sent ({} requests)", nameId(), nodeDataTimeIntoRun, _sendRequests.size());
        std::vector<InsTime> requestsToRemove;
        for (const auto& [sendRequestTime, sendRequests] : _sendRequests)
        {
            LOG_DATA("{}:     [{:.3f}s]", nameId(), math::round(calcTimeIntoRun(sendRequestTime), 8));
            LOG_DATA("{}:       Combinations (all terms in all combinations must be calculated)", nameId());
            if (std::all_of(sendRequests.begin(), sendRequests.end(), [&](const auto& sendRequest) {
                    const auto& comb = _combinations.at(sendRequest.combIndex);
                    LOG_DATA("{}:         '{}' has {}/{} terms set", nameId(), comb.description(this), sendRequest.termIndices.size(), comb.terms.size());
                    return comb.terms.size() == sendRequest.termIndices.size();
                }))
            {
                // If all combinations for this send request epoch are calculated, send it out
                auto dynData = std::make_shared<DynamicData>();
                dynData->insTime = sendRequestTime;
                LOG_DATA("{}:       Sending out dynamic data and deleting send request", nameId());

                for (const auto& sendRequest : sendRequests)
                {
                    const auto& comb = _combinations.at(sendRequest.combIndex);
                    DynamicData::Data data{
                        .description = comb.description(this),
                        .value = sendRequest.result,
                        .events = sendRequest.events
                    };
                    dynData->data.push_back(data);
                }

                invokeCallbacks(OUTPUT_PORT_INDEX_DYN_DATA, dynData);

                requestsToRemove.push_back(sendRequestTime);
            }
            else
            {
                // If not, break, because we always have to send out the first request first
                break;
            }
        }
        for (const auto& insTime : requestsToRemove)
        {
            _sendRequests.erase(insTime);
        }
    }
}

} // namespace NAV