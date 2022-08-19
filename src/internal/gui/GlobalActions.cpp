#include "GlobalActions.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;

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

/// @brief Specifies if the elements in the clipboard are cutted or copied
bool elementsCutted = false;
/// @brief Clipboard storage
json clipboard;

/// @brief Maximum size of the action list
constexpr size_t ACTION_LIST_MAX_SIZE = 20;
/// @brief Current action in the action list
size_t currentAction = 0;
/// @brief List of actions performed by the user
std::deque<json> actionList;

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
    NAV::flow::saveLastActions = false;

    for (const auto& link : nm::m_Links())
    {
        clipboard["links"]["link-" + std::to_string(size_t(link.id))] = link;
    }

    for (const auto& nodeId : selectedNodeIds)
    {
        const NAV::Node* node = nm::FindNode(nodeId);

        clipboard["nodes"]["node-" + std::to_string(size_t(node->id))] = *node;
        clipboard["nodes"]["node-" + std::to_string(size_t(node->id))]["data"] = node->save();

        nm::DeleteNode(nodeId);
    }

    elementsCutted = true;

    NAV::flow::saveLastActions = true;
    saveLastAction();
}

void NAV::gui::copyFlowElements()
{
    std::vector<ax::NodeEditor::NodeId> selectedNodeIds;
    selectedNodeIds.resize(static_cast<size_t>(ed::GetSelectedObjectCount()));

    auto selectedNodesCount = ed::GetSelectedNodes(selectedNodeIds.data(), ed::GetSelectedObjectCount());
    selectedNodeIds.resize(static_cast<size_t>(selectedNodesCount));

    clipboard.clear();

    for (const auto& link : nm::m_Links())
    {
        clipboard["links"]["link-" + std::to_string(size_t(link.id))] = link;
    }

    for (const auto& nodeId : selectedNodeIds)
    {
        const NAV::Node* node = nm::FindNode(nodeId);

        clipboard["nodes"]["node-" + std::to_string(size_t(node->id))] = *node;
        clipboard["nodes"]["node-" + std::to_string(size_t(node->id))]["data"] = node->save();
    }

    elementsCutted = false;
}

void NAV::gui::pasteFlowElements()
{
    // Store the node count to later iterate over the new nodes
    auto nodeCountBeforeLoad = nm::m_Nodes().size();

    NAV::flow::saveLastActions = false;

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

                if (pos.x < leftTopMostPos.x)
                {
                    leftTopMostPos.x = pos.x;
                }
                if (pos.y < leftTopMostPos.y)
                {
                    leftTopMostPos.y = pos.y;
                }
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
    for (size_t i = nodeCountBeforeLoad; i < nm::m_Nodes().size(); i++)
    {
        auto* node = nm::m_Nodes().at(i);
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
                    auto& startPin = nm::m_Nodes().at(startPinParentNodeIndex)->outputPins.at(startPinIndex);
                    auto& endPin = nm::m_Nodes().at(endPinParentNodeIndex)->inputPins.at(endPinIndex);

                    if (!nm::FindConnectedLinkToInputPin(endPin))
                    {
                        nm::CreateLink(startPin, endPin);
                    }
                }

                newlyLinkedNodes[startPinOldParentNodeId] = nm::m_Nodes().at(startPinParentNodeIndex)->id;
                newlyLinkedNodes[endPinOldParentNodeId] = nm::m_Nodes().at(endPinParentNodeIndex)->id;
            }
        }
    }
    if (clipboard.contains("nodes"))
    {
        for (auto [oldId, newId] : newlyLinkedNodes)
        {
            auto* node = nm::FindNode(newId);

            if (clipboard.at("nodes").contains("node-" + std::to_string(oldId)))
            {
                [[maybe_unused]] auto* oldNode = nm::FindNode(oldId);

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

    NAV::flow::loadingFrameCount = ImGui::GetFrameCount();
    NAV::flow::saveLastActions = true;
    saveLastAction();
}

bool NAV::gui::canUndoLastAction()
{
    return currentAction > 0;
}

bool NAV::gui::canRedoLastAction()
{
    return currentAction + 1 < actionList.size();
}

void NAV::gui::clearLastActionList()
{
    actionList.clear();
    currentAction = 0;
}

void restoreAction(const json& target)
{
    // TODO: Compare against current config and only load the nodes/links which were changed
    // json current;
    // for (const auto& node : nm::m_Nodes())
    // {
    //     current["nodes"]["node-" + std::to_string(size_t(node->id))] = *node;
    //     current["nodes"]["node-" + std::to_string(size_t(node->id))]["data"] = node->save();
    // }
    // for (const auto& link : nm::m_Links())
    // {
    //     current["links"]["link-" + std::to_string(size_t(link.id))] = link;
    // }

    NAV::flow::saveLastActions = false;
    nm::DeleteAllLinksAndNodes();

    NAV::flow::LoadJson(target);
    if (!target["unsavedChanges"].get<bool>())
    {
        NAV::flow::DiscardChanges();
    }
    else
    {
        NAV::flow::ApplyChanges();
    }

    NAV::flow::loadingFrameCount = ImGui::GetFrameCount();
    NAV::flow::saveLastActions = true;

    nm::InitializeAllNodesAsync();
}

void NAV::gui::undoLastAction()
{
    LOG_DEBUG("Undoing last action");

    restoreAction(actionList.at(--currentAction));
}

void NAV::gui::redoLastAction()
{
    LOG_DEBUG("Redoing last action");

    restoreAction(actionList.at(++currentAction));
}

void NAV::gui::saveLastAction()
{
    LOG_DEBUG("Saving last action to action list");

    // TODO: Check if event was triggered by a slider and discard the save, because it triggers it every step

    if (actionList.size() > ACTION_LIST_MAX_SIZE) // List is full
    {
        LOG_TRACE("Action list full, therefore discarding first element.");
        actionList.pop_front();
        if (currentAction)
        {
            currentAction--;
        }
    }
    while (currentAction + 1 < actionList.size())
    {
        LOG_TRACE("Discarding element which is past the current action");
        actionList.pop_back();
    }

    json j;
    for (const auto& node : nm::m_Nodes())
    {
        j["nodes"]["node-" + std::to_string(size_t(node->id))] = *node;
        j["nodes"]["node-" + std::to_string(size_t(node->id))]["data"] = node->save();
    }
    for (const auto& link : nm::m_Links())
    {
        j["links"]["link-" + std::to_string(size_t(link.id))] = link;
    }
    j["unsavedChanges"] = NAV::flow::HasUnsavedChanges();

    if (!actionList.empty())
    {
        currentAction++;
    }
    actionList.push_back(j);
}