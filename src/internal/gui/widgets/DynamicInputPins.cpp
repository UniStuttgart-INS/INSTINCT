// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file DynamicInputPins.cpp
/// @brief Inputs pins which can be added dynamically
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-12-21

#include "DynamicInputPins.hpp"

#include <optional>
#include <imgui.h>

#include "util/Container/Vector.hpp"
#include "util/Logger.hpp"

namespace NAV::gui::widgets
{

DynamicInputPins::DynamicInputPins(size_t firstDynamicPin,
                                   Node* node,
                                   std::function<void(Node*)> pinAddCallback,
                                   std::function<void(Node*, size_t)> pinDeleteCallback,
                                   size_t defaultInputPins)
    : _firstDynamicPinIdx(firstDynamicPin), _pinAddCallback(std::move(pinAddCallback)), _pinDeleteCallback(std::move(pinDeleteCallback))
{
    while (_nDynamicInputPins < defaultInputPins)
    {
        _pinAddCallback(node);
        _nDynamicInputPins++;
    }
}

bool DynamicInputPins::ShowGuiWidgets(size_t id, std::vector<InputPin>& inputPins, Node* node, const std::vector<ExtraColumn>& extraColumns)
{
    bool changed = false;

    int nExtraColumns = static_cast<int>(extraColumns.size());
    if (ImGui::BeginTable(fmt::format("Pin Settings##{}", id).c_str(),
                          inputPins.size() > _firstDynamicPinIdx + 1 ? 2 + nExtraColumns : 1 + nExtraColumns,
                          ImGuiTableFlags_Borders | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_NoHostExtendX, ImVec2(0.0F, 0.0F)))
    {
        ImGui::TableSetupColumn("Pin");
        for (const auto& column : extraColumns)
        {
            ImGui::TableSetupColumn(column.header.c_str());
        }
        if (inputPins.size() > _firstDynamicPinIdx + 1)
        {
            ImGui::TableSetupColumn(""); // Delete Button column
        }
        ImGui::TableHeadersRow();

        // Used to reset the member variabel _dragAndDropPinIndex in case no plot does a drag and drop action
        bool dragAndDropPinStillInProgress = false;

        auto showDragDropTargetPin = [&](size_t pinIdxTarget) {
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
                if (const ImGuiPayload* payloadData = ImGui::AcceptDragDropPayload(fmt::format("DND Pin {}", id).c_str()))
                {
                    auto pinIdxSource = *static_cast<size_t*>(payloadData->Data);

                    if (pinIdxSource < pinIdxTarget)
                    {
                        --pinIdxTarget;
                    }

                    move(inputPins, pinIdxSource, pinIdxTarget);
                    changed = true;
                }
                ImGui::EndDragDropTarget();
            }
            ImGui::Dummy(ImVec2(-1.F, 2.F));
        };

        std::optional<size_t> deletePinIdx;
        for (size_t pinIndex = 0; pinIndex < inputPins.size(); pinIndex++)
        {
            ImGui::TableNextRow();
            ImGui::TableNextColumn(); // Pin

            if (pinIndex == _firstDynamicPinIdx && _dragAndDropPinIndex > static_cast<int>(_firstDynamicPinIdx))
            {
                showDragDropTargetPin(_firstDynamicPinIdx);
            }

            bool selectablePinDummy = false;
            ImGui::Selectable(fmt::format("{}##{}", inputPins.at(pinIndex).name, id).c_str(), &selectablePinDummy);
            if (pinIndex >= _firstDynamicPinIdx && ImGui::BeginDragDropSource(ImGuiDragDropFlags_None))
            {
                dragAndDropPinStillInProgress = true;
                _dragAndDropPinIndex = static_cast<int>(pinIndex);
                // Data is copied into heap inside the drag and drop
                ImGui::SetDragDropPayload(fmt::format("DND Pin {}", id).c_str(), &pinIndex, sizeof(pinIndex));
                ImGui::TextUnformatted(inputPins.at(pinIndex).name.c_str());
                ImGui::EndDragDropSource();
            }
            if (_dragAndDropPinIndex >= 0 && pinIndex >= _firstDynamicPinIdx
                && pinIndex != static_cast<size_t>(_dragAndDropPinIndex - 1)
                && pinIndex != static_cast<size_t>(_dragAndDropPinIndex))
            {
                showDragDropTargetPin(pinIndex + 1);
            }
            if (pinIndex >= _firstDynamicPinIdx && ImGui::IsItemHovered())
            {
                ImGui::SetTooltip("This item can be dragged to reorder the pins");
            }

            for (const auto& column : extraColumns)
            {
                ImGui::TableNextColumn();
                changed |= column.content(pinIndex);
            }

            if (pinIndex >= _firstDynamicPinIdx && inputPins.size() > _firstDynamicPinIdx + 1)
            {
                ImGui::TableNextColumn(); // Delete
                if (ImGui::Button(fmt::format("x##{} - {}", id, pinIndex).c_str()))
                {
                    deletePinIdx = pinIndex;
                }
                if (ImGui::IsItemHovered())
                {
                    ImGui::SetTooltip("Delete the pin");
                }
            }
        }
        if (deletePinIdx)
        {
            LOG_TRACE("{}: Deleting pin with index {}", id, *deletePinIdx);
            changed = true;
            _pinDeleteCallback(node, *deletePinIdx);
            _nDynamicInputPins--;
        }

        if (!dragAndDropPinStillInProgress)
        {
            _dragAndDropPinIndex = -1;
        }

        ImGui::TableNextRow();
        ImGui::TableNextColumn(); // Pin
        if (ImGui::Button(fmt::format("Add Pin##{}", id).c_str()))
        {
            LOG_TRACE("{}: Adding a new pin", id);
            changed = true;
            _pinAddCallback(node);
            _nDynamicInputPins++;
        }

        ImGui::EndTable();
    }

    return changed;
}

size_t DynamicInputPins::getNumberOfDynamicPins() const
{
    return _nDynamicInputPins;
}

void DynamicInputPins::addPin(Node* node)
{
    _pinAddCallback(node);
    _nDynamicInputPins++;
}

void to_json(json& j, const DynamicInputPins& obj)
{
    j = json{
        { "nDynamicInputPins", obj._nDynamicInputPins },
    };
}
void from_json(const json& j, DynamicInputPins& obj, Node* node)
{
    if (j.contains("nDynamicInputPins"))
    {
        size_t nPins = 0;
        j.at("nDynamicInputPins").get_to(nPins);
        while (obj._nDynamicInputPins < nPins)
        {
            obj._pinAddCallback(node);
            obj._nDynamicInputPins++;
        }
        while (obj._nDynamicInputPins > nPins)
        {
            obj._pinDeleteCallback(node, --obj._nDynamicInputPins);
        }
    }
}

} // namespace NAV::gui::widgets
