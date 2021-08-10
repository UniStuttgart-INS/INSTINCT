#include "GlobalActions.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;

#include "internal/FlowManager.hpp"

#include <imgui_node_editor.h>
#include <imgui_node_editor_internal.h>
namespace ed = ax::NodeEditor;

#include <nlohmann/json.hpp>
using json = nlohmann::json;
#include "internal/Json.hpp"

#include "internal/gui/NodeEditorApplication.hpp"

#include <vector>
#include <limits>

bool elementsCutted = false;
json clipboard;

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

    for (const auto& nodeId : selectedNodeIds)
    {
        const NAV::Node* node = nm::FindNode(nodeId);

        clipboard["nodes"]["node-" + std::to_string(size_t(node->id))] = *node;
        clipboard["nodes"]["node-" + std::to_string(size_t(node->id))]["data"] = node->save();

        nm::DeleteNode(nodeId);
    }

    elementsCutted = true;
}

void NAV::gui::copyFlowElements()
{
    std::vector<ax::NodeEditor::NodeId> selectedNodeIds;
    selectedNodeIds.resize(static_cast<size_t>(ed::GetSelectedObjectCount()));

    auto selectedNodesCount = ed::GetSelectedNodes(selectedNodeIds.data(), ed::GetSelectedObjectCount());
    selectedNodeIds.resize(static_cast<size_t>(selectedNodesCount));

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

    for (size_t i = nodeCountBeforeLoad; i < nm::m_Nodes().size(); i++)
    {
        auto* node = nm::m_Nodes().at(i);
        ed::SetNodePosition(node->id, mousePos + (ed::GetNodePosition(node->id) - leftTopMostPos));
    }

    elementsCutted = false;
}

bool NAV::gui::canUndoLastAction()
{
    return true;
}

bool NAV::gui::canRedoLastAction()
{
    return true;
}

void NAV::gui::undoLastAction()
{
}

void NAV::gui::redoLastAction()
{
}