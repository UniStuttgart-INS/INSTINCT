// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "GlobalActions.hpp"

#include "internal/FlowManager.hpp"

#include "internal/gui/NodeEditorApplication.hpp"

#include <imgui_node_editor.h>
#include <imgui_node_editor_internal.h>
namespace ed = ax::NodeEditor;

#include <nlohmann/json.hpp>
using json = nlohmann::json; ///< json namespace
#include "util/Json.hpp"

#include "internal/ConfigManager.hpp"

#include <vector>
#include <deque>
#include <limits>
#include <iterator>

namespace NAV::gui
{
namespace
{
/// @brief Specifies if the elements in the clipboard are cutted or copied
bool elementsCutted = false;
/// @brief Clipboard storage
json clipboard;

} // namespace
} // namespace NAV::gui

bool NAV::gui::canCutOrCopyFlowElements()
{
    return static_cast<bool>(ed::GetSelectedNodes(nullptr, ed::GetSelectedObjectCount()));
}

bool NAV::gui::canPasteFlowElements()
{
    return !clipboard.empty();
}

void NAV::gui::cutFlowElements()
{
    std::vector<ax::NodeEditor::NodeId> selectedNodeIds;
    selectedNodeIds.resize(static_cast<size_t>(ed::GetSelectedObjectCount()));

    auto selectedNodesCount = ed::GetSelectedNodes(selectedNodeIds.data(), ed::GetSelectedObjectCount());
    selectedNodeIds.resize(static_cast<size_t>(selectedNodesCount));

    clipboard.clear();

    for (const auto& nodeId : selectedNodeIds)
    {
        const NAV::Node* node = flow::FindNode(nodeId);

        clipboard["nodes"]["node-" + std::to_string(size_t(node->id))] = *node;
        clipboard["nodes"]["node-" + std::to_string(size_t(node->id))]["data"] = node->save();

        for (const auto& outputPin : node->outputPins)
        {
            for (const auto& link : outputPin.links)
            {
                auto& j = clipboard["links"]["link-" + std::to_string(size_t(link.linkId))];
                j["id"] = size_t(link.linkId);
                j["startPinId"] = size_t(outputPin.id);
                j["endPinId"] = size_t(link.connectedPinId);
            }
        }

        flow::DeleteNode(nodeId);
    }

    elementsCutted = true;
}

void NAV::gui::copyFlowElements()
{
    std::vector<ax::NodeEditor::NodeId> selectedNodeIds;
    selectedNodeIds.resize(static_cast<size_t>(ed::GetSelectedObjectCount()));

    auto selectedNodesCount = ed::GetSelectedNodes(selectedNodeIds.data(), ed::GetSelectedObjectCount());
    selectedNodeIds.resize(static_cast<size_t>(selectedNodesCount));

    clipboard.clear();

    for (const auto& nodeId : selectedNodeIds)
    {
        const NAV::Node* node = flow::FindNode(nodeId);

        clipboard["nodes"]["node-" + std::to_string(size_t(node->id))] = *node;
        clipboard["nodes"]["node-" + std::to_string(size_t(node->id))]["data"] = node->save();

        for (const auto& outputPin : node->outputPins)
        {
            for (const auto& link : outputPin.links)
            {
                auto& j = clipboard["links"]["link-" + std::to_string(size_t(link.linkId))];
                j["id"] = size_t(link.linkId);
                j["startPinId"] = size_t(outputPin.id);
                j["endPinId"] = size_t(link.connectedPinId);
            }
        }
    }

    elementsCutted = false;
}

void NAV::gui::pasteFlowElements()
{
    // Store the node count to later iterate over the new nodes
    auto nodeCountBeforeLoad = flow::m_Nodes().size();

    LOG_DEBUG("Pasting clipboard {}", clipboard.dump(4));

    flow::LoadJson(clipboard, !elementsCutted);

    // Find Top Left Position of all new nodes to move them to the mouse cursor
    ImVec2 leftTopMostPos{ std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity() };
    if (clipboard.contains("nodes"))
    {
        for (const auto& nodeJson : clipboard.at("nodes"))
        {
            ImVec2 pos;
            if (nodeJson.contains("pos"))
            {
                nodeJson.at("pos").get_to(pos);

                leftTopMostPos.x = std::min(pos.x, leftTopMostPos.x);
                leftTopMostPos.y = std::min(pos.y, leftTopMostPos.y);
            }
        }
    }

    // Get Mouse Position in editor coordinates
    auto viewRect = reinterpret_cast<ax::NodeEditor::Detail::EditorContext*>(ed::GetCurrentEditor())->GetViewRect();
    ImVec2 mousePos = ImGui::GetMousePos();
    mousePos.x -= NodeEditorApplication::leftPaneWidth + NodeEditorApplication::SPLITTER_THICKNESS + 10.0F;
    mousePos.y -= NodeEditorApplication::menuBarHeight;
    mousePos *= ed::GetCurrentZoom();
    mousePos += viewRect.GetTL();

    // Move the Nodes relative to the current mouse position
    for (size_t i = nodeCountBeforeLoad; i < flow::m_Nodes().size(); i++)
    {
        auto* node = flow::m_Nodes().at(i);
        ed::SetNodePosition(node->id, mousePos + (ed::GetNodePosition(node->id) - leftTopMostPos));
    }

    // Collect the node ids which get new links to call the restoreAfterLinks function on them
    std::map<size_t, ed::NodeId> newlyLinkedNodes;

    // Recreate links
    if (clipboard.contains("links"))
    {
        for (const auto& linkJson : clipboard.at("links"))
        {
            auto startPinId = linkJson.at("startPinId").get<size_t>();
            auto endPinId = linkJson.at("endPinId").get<size_t>();

            size_t startPinOldParentNodeId = 0;
            size_t startPinParentNodeIndex = 0;
            size_t startPinIndex = 0;
            Pin::Kind startPinKind = Pin::Kind::None;

            size_t endPinOldParentNodeId = 0;
            size_t endPinParentNodeIndex = 0;
            size_t endPinIndex = 0;
            Pin::Kind endPinKind = Pin::Kind::None;

            // Search for the nodes and pins which where connected by the old link
            if (clipboard.contains("nodes"))
            {
                size_t nodeIndex = 0;
                for (const auto& nodeJson : clipboard.at("nodes"))
                {
                    if (nodeJson.contains("inputPins"))
                    {
                        size_t pinIndex = 0;
                        for (const auto& pinJson : nodeJson.at("inputPins"))
                        {
                            if (pinJson.at("id").get<size_t>() == startPinId)
                            {
                                startPinOldParentNodeId = nodeJson.at("id");
                                startPinParentNodeIndex = nodeCountBeforeLoad + nodeIndex;
                                startPinIndex = pinIndex;
                                startPinKind = Pin::Kind::Input;
                            }
                            if (pinJson.at("id").get<size_t>() == endPinId)
                            {
                                endPinOldParentNodeId = nodeJson.at("id");
                                endPinParentNodeIndex = nodeCountBeforeLoad + nodeIndex;
                                endPinIndex = pinIndex;
                                endPinKind = Pin::Kind::Input;
                            }
                            pinIndex++;
                        }
                    }
                    if (nodeJson.contains("outputPins"))
                    {
                        size_t pinIndex = 0;
                        for (const auto& pinJson : nodeJson.at("outputPins"))
                        {
                            if (pinJson.at("id").get<size_t>() == startPinId)
                            {
                                startPinOldParentNodeId = nodeJson.at("id");
                                startPinParentNodeIndex = nodeCountBeforeLoad + nodeIndex;
                                startPinIndex = pinIndex;
                                startPinKind = Pin::Kind::Output;
                            }
                            if (pinJson.at("id").get<size_t>() == endPinId)
                            {
                                endPinOldParentNodeId = nodeJson.at("id");
                                endPinParentNodeIndex = nodeCountBeforeLoad + nodeIndex;
                                endPinIndex = pinIndex;
                                endPinKind = Pin::Kind::Output;
                            }
                            pinIndex++;
                        }
                    }
                    nodeIndex++;
                }
            }

            if (startPinKind != Pin::Kind::None && endPinKind != Pin::Kind::None)
            {
                if (startPinKind == Pin::Kind::Output && endPinKind == Pin::Kind::Input)
                {
                    auto& startPin = flow::m_Nodes().at(startPinParentNodeIndex)->outputPins.at(startPinIndex);
                    auto& endPin = flow::m_Nodes().at(endPinParentNodeIndex)->inputPins.at(endPinIndex);

                    if (!endPin.isPinLinked())
                    {
                        startPin.createLink(endPin);
                    }
                }

                newlyLinkedNodes[startPinOldParentNodeId] = flow::m_Nodes().at(startPinParentNodeIndex)->id;
                newlyLinkedNodes[endPinOldParentNodeId] = flow::m_Nodes().at(endPinParentNodeIndex)->id;
            }
        }
    }
    if (clipboard.contains("nodes"))
    {
        for (auto [oldId, newId] : newlyLinkedNodes)
        {
            auto* node = flow::FindNode(newId);

            if (clipboard.at("nodes").contains("node-" + std::to_string(oldId)))
            {
                [[maybe_unused]] auto* oldNode = flow::FindNode(oldId);

                LOG_DEBUG("Calling restoreAtferLink() for new node '{}', which was copied from node '{}'", node->nameId(), oldNode->nameId());

                const auto& nodeJson = clipboard.at("nodes").at("node-" + std::to_string(oldId));
                if (nodeJson.contains("data"))
                {
                    node->restoreAtferLink(nodeJson.at("data"));
                }
            }
        }
    }

    elementsCutted = false;
}