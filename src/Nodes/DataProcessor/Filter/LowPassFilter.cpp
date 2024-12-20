// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "LowPassFilter.hpp"

#include "NodeRegistry.hpp"
#include <algorithm>
#include <imgui.h>
#include "Navigation/INS/Units.hpp"
#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeData/IMU/ImuObsSimulated.hpp"
#include "NodeData/IMU/ImuObsWDelta.hpp"

#include "internal/gui/widgets/HelpMarker.hpp"
#include "internal/gui/widgets/imgui_ex.hpp"
#include "internal/gui/widgets/InputWithUnit.hpp"
#include "internal/gui/NodeEditorApplication.hpp"

#include "Navigation/Transformations/CoordinateFrames.hpp"
#include "Navigation/Transformations/Units.hpp"
#include "Navigation/GNSS/Functions.hpp"

#include "util/Eigen.hpp"
#include "util/StringUtil.hpp"
#include "util/Logger.hpp"

#include <imgui_internal.h>
#include <limits>
#include <set>
#include <type_traits>

NAV::LowPassFilter::LowPassFilter()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);
    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 500, 300 };

    nm::CreateInputPin(this, "Original", Pin::Type::Flow, { NAV::NodeData::type() }, &LowPassFilter::receiveObs);

    nm::CreateOutputPin(this, "Filtered", Pin::Type::Flow, { NAV::NodeData::type() });
}

NAV::LowPassFilter::~LowPassFilter()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::LowPassFilter::typeStatic()
{
    return "LowPassFilter";
}

std::string NAV::LowPassFilter::type() const
{
    return typeStatic();
}

std::string NAV::LowPassFilter::category()
{
    return "Data Processor";
}

void NAV::LowPassFilter::guiConfig()
{
    if (outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier.size() != 1)
    {
        ImGui::TextUnformatted("Please connect the input pin to show the options");
        return;
    }

    ImGui::SetNextItemWidth(400.0F * gui::NodeEditorApplication::defaultFontRatio());

    if (_gui_availableItemsSelection > _availableItems.size()) { _gui_availableItemsSelection = _availableItems.size() - (_availableItems.empty() ? 0 : 1); }
    bool noMoreItems = _filterItems.size() == _availableItems.size();
    if (noMoreItems) { ImGui::BeginDisabled(); }
    if (ImGui::BeginCombo(fmt::format("##Available data combo {}", size_t(id)).c_str(), !_availableItems.empty() ? _availableItems.at(_gui_availableItemsSelection).c_str() : ""))
    {
        for (size_t i = 0; i < _availableItems.size(); i++)
        {
            const auto& item = _availableItems.at(i);
            if (std::ranges::find_if(_filterItems, [&](const FilterItem& filterItem) {
                    return filterItem.dataDescription == item;
                })
                != _filterItems.end()) { continue; }

            const bool is_selected = (_gui_availableItemsSelection == i);
            if (ImGui::Selectable(item.c_str(), is_selected))
            {
                _gui_availableItemsSelection = i;
            }
            if (is_selected) // Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
            {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }
    ImGui::SameLine();
    if (ImGui::Button(fmt::format("Add##Filter item {}", size_t(id)).c_str()))
    {
        _filterItems.emplace_back(_availableItems.at(_gui_availableItemsSelection), _gui_availableItemsSelection);
        flow::ApplyChanges();
        bool selectionChanged = false;
        for (size_t i = _gui_availableItemsSelection + 1; i < _availableItems.size(); i++)
        {
            const auto& item = _availableItems.at(i);
            if (std::ranges::find_if(_filterItems, [&](const FilterItem& filterItem) {
                    return filterItem.dataDescription == item;
                })
                != _filterItems.end()) { continue; }

            _gui_availableItemsSelection = i;
            selectionChanged = true;
            break;
        }
        if (!selectionChanged && _gui_availableItemsSelection != 0)
        {
            for (int i = static_cast<int>(_gui_availableItemsSelection) - 1; i >= 0; i--)
            {
                const auto& item = _availableItems.at(static_cast<size_t>(i));
                if (std::ranges::find_if(_filterItems, [&](const FilterItem& filterItem) {
                        return filterItem.dataDescription == item;
                    })
                    != _filterItems.end()) { continue; }

                _gui_availableItemsSelection = static_cast<size_t>(i);
                break;
            }
        }
    }
    if (noMoreItems) { ImGui::EndDisabled(); }

    const float COMBO_WIDTH = 100.0F * gui::NodeEditorApplication::windowFontRatio();
    const float ITEM_WIDTH = 140.0F * gui::NodeEditorApplication::windowFontRatio();

    std::optional<size_t> itemToDelete;
    for (size_t i = 0; i < _filterItems.size(); i++)
    {
        auto& item = _filterItems.at(i);
        bool keep = true;
        ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
        if (ImGui::CollapsingHeader(fmt::format("{}##{}", item.dataDescription, size_t(id)).c_str(), &keep))
        {
            ImGui::SetNextItemWidth(COMBO_WIDTH);
            if (ImGui::BeginCombo(fmt::format("Filter Type##{}", size_t(id)).c_str(), to_string(item.filterType)))
            {
                for (size_t i = 0; i < static_cast<size_t>(FilterType::COUNT); i++)
                {
                    const bool is_selected = (static_cast<size_t>(item.filterType) == i);
                    if (ImGui::Selectable(to_string(static_cast<FilterType>(i)), is_selected))
                    {
                        item.filterType = static_cast<FilterType>(i);
                        LOG_DEBUG("{}: filterType changed to {}", nameId(), fmt::underlying(item.filterType));
                        flow::ApplyChanges();
                    }
                    if (is_selected) // Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
                    {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndCombo();
            }

            ImGui::SameLine();
            float size = 7.0F * gui::NodeEditorApplication::windowFontRatio();
            ImGui::GetWindowDrawList()->AddCircleFilled(ImVec2(ImGui::GetCursorScreenPos().x + size / 1.2F,
                                                               ImGui::GetCursorScreenPos().y + size * 1.8F),
                                                        size,
                                                        item.modified
                                                            ? ImColor(0.0F, 255.0F, 0.0F)
                                                            : ImColor(255.0F, 0.0F, 0.0F));
            ImGui::Dummy(ImVec2(2 * size, 3.0F * size));
            if (ImGui::IsItemHovered())
            {
                ImGui::SetTooltip(item.modified
                                      ? "Indicates wether the filter is working."
                                      : "Indicates wether the filter is working.\n"
                                        "Reasons why it is not working can be:\n"
                                        "- Data rate of the incoming values must be greater then 2 * dt\n"
                                        "- The data was never included in the observations (dynamic data)\n"
                                        "- The data cannot be modified because it is not implemented yet");
            }

            ImGui::Indent();
            if (item.filterType == FilterType::Linear)
            {
                ImGui::SetNextItemWidth(ITEM_WIDTH);
                if (ImGui::InputDoubleL(fmt::format("Cutoff Frequency##{} {}", size_t(id), item.dataDescription).c_str(), &item.linear_filter_cutoff_frequency, 1e-5, 1e5, 0.0, 0.0, "%.5f Hz"))
                {
                    LOG_DEBUG("{}: Cutoff Freq. {} changed to {}", nameId(), item.dataDescription, item.linear_filter_cutoff_frequency);
                    flow::ApplyChanges();
                }
            }
            ImGui::Unindent();
        }
        if (!keep) { itemToDelete = i; }
    }
    if (itemToDelete) { _filterItems.erase(std::next(_filterItems.begin(), static_cast<int64_t>(*itemToDelete))); }
}

json NAV::LowPassFilter::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["availableItems"] = _availableItems;
    j["filterItems"] = _filterItems;

    return j;
}

void NAV::LowPassFilter::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());
    if (j.contains("availableItems")) { j.at("availableItems").get_to(_availableItems); }
    if (j.contains("filterItems")) { j.at("filterItems").get_to(_filterItems); }
}

bool NAV::LowPassFilter::resetNode()
{
    LOG_TRACE("{}: called", nameId());

    for (auto& item : _filterItems)
    {
        item.dataToFilter.clear();
        item.modified = false;
    }

    return true;
}

void NAV::LowPassFilter::afterCreateLink(OutputPin& startPin, [[maybe_unused]] InputPin& endPin)
{
    LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(startPin.id), size_t(endPin.id));

    if (endPin.parentNode->id != id)
    {
        return; // Link on Output Port
    }

    // Store previous output pin identifier
    auto previousOutputPinDataIdentifier = outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier;
    // Overwrite output pin identifier with input pin identifier
    outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier = startPin.dataIdentifier;

    if (previousOutputPinDataIdentifier != outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier) // If the identifier changed
    {
        // Check if connected links on output port are still valid
        for (auto& link : outputPins.at(OUTPUT_PORT_INDEX_FLOW).links)
        {
            if (auto* endPin = link.getConnectedPin())
            {
                if (!outputPins.at(OUTPUT_PORT_INDEX_FLOW).canCreateLink(*endPin))
                {
                    // If the link is not valid anymore, delete it
                    outputPins.at(OUTPUT_PORT_INDEX_FLOW).deleteLink(*endPin);
                }
            }
        }

        // Refresh all links connected to the output pin if the type changed
        if (outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier != previousOutputPinDataIdentifier)
        {
            for (auto& link : outputPins.at(OUTPUT_PORT_INDEX_FLOW).links)
            {
                if (auto* connectedPin = link.getConnectedPin())
                {
                    outputPins.at(OUTPUT_PORT_INDEX_FLOW).recreateLink(*connectedPin);
                }
            }
        }
    }

    if (auto* pin = inputPins.at(INPUT_PORT_INDEX_FLOW).link.getConnectedPin();
        pin && _availableItems.empty())
    {
        _gui_availableItemsSelection = 0;
        _availableItems = NAV::NodeRegistry::GetStaticDataDescriptors(pin->dataIdentifier);
    }
}

void NAV::LowPassFilter::afterDeleteLink(OutputPin& startPin, InputPin& endPin)
{
    LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(startPin.id), size_t(endPin.id));

    _gui_availableItemsSelection = 0;
    _availableItems.clear();

    if ((endPin.parentNode->id != id                                  // Link on Output port is removed
         && !inputPins.at(INPUT_PORT_INDEX_FLOW).isPinLinked())       //     and the Input port is not linked
        || (startPin.parentNode->id != id                             // Link on Input port is removed
            && !outputPins.at(OUTPUT_PORT_INDEX_FLOW).isPinLinked())) //     and the Output port is not linked
    {
        outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier = { NodeData::type() };
    }
}

void NAV::LowPassFilter::receiveObs(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    auto obs = queue.extract_front();

    for (const auto& desc : obs->dynamicDataDescriptors())
    {
        if (std::ranges::none_of(_availableItems, [&](const auto& header) { return header == desc; }))
        {
            _availableItems.push_back(desc);
            flow::ApplyChanges();
        }
    }

    auto out = NAV::NodeRegistry::CopyNodeData(obs);

    for (auto& item : _filterItems)
    {
        LOG_DATA("{}: [{}] {}", nameId(), item.dataIndex, item.dataDescription);
        if (item.dataIndex < out->staticDescriptorCount())
        {
            if (auto value = out->getValueAt(item.dataIndex))
            {
                if (auto newValue = filterData(item, out->insTime, *value))
                {
                    item.modified |= out->setValueAt(item.dataIndex, *newValue);
                }
            }
        }
        else if (auto value = out->getDynamicDataAt(item.dataDescription))
        {
            if (auto newValue = filterData(item, out->insTime, *value))
            {
                item.modified |= out->setDynamicDataAt(item.dataDescription, *newValue);
            }
        }
    }

    invokeCallbacks(OUTPUT_PORT_INDEX_FLOW, out);
}

std::optional<double> NAV::LowPassFilter::filterData(FilterItem& item, const InsTime& insTime, double value)
{
    if (item.filterType == FilterType::Linear)
    {
        // first we filter accelerations
        item.dataToFilter[insTime] = value;
        // for testing at the moment
        double dt = 1.0 / item.linear_filter_cutoff_frequency;
        // remove all entries that are outside filter time window
        std::erase_if(item.dataToFilter, [&](const auto& pair) { return static_cast<double>((insTime - pair.first).count()) > dt; });

        if (item.dataToFilter.size() > 2)
        {
            // average accelerations first
            auto N11 = static_cast<double>(item.dataToFilter.size());
            double N12 = 0.0;
            double N22 = 0.0;
            double n1 = 0.0;
            double n2 = 0.0;
            for (const auto& key_val : item.dataToFilter)
            {
                auto delta_t = static_cast<double>((key_val.first - insTime).count());
                N12 += delta_t;
                N22 += delta_t * delta_t;
                n1 += key_val.second;
                n2 += delta_t * key_val.second;
            }
            double determinant_inverse = 1.0 / (N11 * N22 - N12 * N12);
            return determinant_inverse * (N22 * n1 - N12 * n2);
        }
    }
    return {};
}

const char* NAV::LowPassFilter::to_string(FilterType value)
{
    switch (value)
    {
    case FilterType::Linear:
        return "Linear fit";
    // case FilterType::Experimental:
    //     return "Experimental";
    case FilterType::COUNT:
        return "";
    }
    return "";
}

namespace NAV
{

void to_json(json& j, const LowPassFilter::FilterItem& data)
{
    j = json{
        { "dataDescription", data.dataDescription },
        { "filterType", data.filterType },
        { "linear_filter_cutoff_frequency", data.linear_filter_cutoff_frequency },
    };
}

void from_json(const json& j, LowPassFilter::FilterItem& data)
{
    if (j.contains("dataDescription")) { j.at("dataDescription").get_to(data.dataDescription); }
    if (j.contains("filterType")) { j.at("filterType").get_to(data.filterType); }
    if (j.contains("linear_filter_cutoff_frequency")) { j.at("linear_filter_cutoff_frequency").get_to(data.linear_filter_cutoff_frequency); }
}

} // namespace NAV