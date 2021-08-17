#include "internal/FlowManager.hpp"

#include "internal/Json.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;

#include <imgui_node_editor.h>
namespace ed = ax::NodeEditor;

#include "NodeRegistry.hpp"

#include "Nodes/Node.hpp"
#include "internal/Link.hpp"
#include "internal/Pin.hpp"
#include "internal/ConfigManager.hpp"

#include <fstream>
#include <iomanip>
#include <string>
#include <memory>

#include <iostream>

bool unsavedChanges = false;
bool NAV::flow::saveLastActions = true;

constexpr int loadingFramesToWait = 2;
int NAV::flow::loadingFrameCount = 0;

std::string currentFilename;
std::string programRootPath;

void NAV::flow::SaveFlow(GlobalActions& globalAction)
{
    if (currentFilename.empty())
    {
        globalAction = GlobalActions::SaveAs;
    }
    else
    {
        SaveFlowAs(currentFilename);
    }
}

void NAV::flow::SaveFlowAs(const std::string& filepath)
{
    std::ofstream filestream(filepath);

    if (!filestream.good())
    {
        std::cerr << "Save Flow error: Could not open file: " << filepath;
        return;
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

    filestream << std::setw(4) << j << std::endl;

    unsavedChanges = false;
}

bool NAV::flow::LoadFlow(const std::string& filepath)
{
    LOG_TRACE("called for path {}", filepath);
    bool loadSuccessful = true;
    std::ifstream filestream(filepath);

    if (!filestream.good())
    {
        LOG_ERROR("Load Flow error: Could not open file: {}", filepath);
        return false;
    }

    json j;
    filestream >> j;

    saveLastActions = false;

    nm::DeleteAllLinksAndNodes();

    LoadJson(j);

    if (ConfigManager::Get<bool>("nogui", false))
    {
        if (!nm::InitializeAllNodes())
        {
            loadSuccessful = false;
        }
    }
    else
    {
        nm::InitializeAllNodesAsync();
    }

    if (!ConfigManager::Get<bool>("nogui", false))
    {
        loadingFrameCount = ImGui::GetFrameCount();
    }
    unsavedChanges = false;
    saveLastActions = true;
    currentFilename = filepath;

    gui::clearLastActionList();
    gui::saveLastAction();

    return loadSuccessful;
}

bool NAV::flow::LoadJson(const json& j, bool requestNewIds)
{
    bool loadSuccessful = true;

    if (j.contains("nodes"))
    {
        for (const auto& nodeJson : j.at("nodes"))
        {
            if (!nodeJson.contains("type"))
            {
                LOG_ERROR("Node does not contain a type");
                continue;
            }
            Node* node = nullptr;
            for (const auto& registeredNode : NAV::NodeRegistry::RegisteredNodes())
            {
                for (const auto& nodeInfo : registeredNode.second)
                {
                    if (nodeInfo.type == nodeJson.at("type"))
                    {
                        node = nodeInfo.constructor();
                        break;
                    }
                }
                if (node != nullptr)
                {
                    break;
                }
            }
            if (node == nullptr)
            {
                LOG_ERROR("Node type ({}) is not a valid type.", nodeJson.at("type").get<std::string>());
                loadSuccessful = false;
                continue;
            }

            nm::AddNode(node);
            auto newNodeId = node->id;

            nodeJson.get_to<Node>(*node);
            if (nodeJson.contains("data"))
            {
                node->restore(nodeJson.at("data"));
            }
            // Load second time in case restore changed the amount of pins
            nodeJson.get_to<Node>(*node);

            if (requestNewIds)
            {
                node->id = newNodeId;
                for (auto& pin : node->inputPins)
                {
                    pin.id = nm::GetNextPinId();
                }
                for (auto& pin : node->outputPins)
                {
                    pin.id = nm::GetNextPinId();
                }
            }

            nm::UpdateNode(node);

            if (!ConfigManager::Get<bool>("nogui", false))
            {
                ed::SetNodePosition(node->id, nodeJson.at("pos").get<ImVec2>());

                if (node->size.x > 0 && node->size.y > 0)
                {
                    ed::SetGroupSize(node->id, node->size);
                }
            }
        }
    }

    if (j.contains("links"))
    {
        for (const auto& linkJson : j.at("links"))
        {
            Link link = linkJson.get<Link>();

            if (!nm::AddLink(link))
            {
                loadSuccessful = false;
            }
        }
    }
    if (j.contains("nodes"))
    {
        for (const auto& node : nm::m_Nodes())
        {
            if (j.at("nodes").contains("node-" + std::to_string(size_t(node->id))))
            {
                const auto& nodeJson = j.at("nodes").at("node-" + std::to_string(size_t(node->id)));
                if (nodeJson.contains("data"))
                {
                    node->restoreAtferLink(nodeJson.at("data"));
                }
            }
        }
    }

    return loadSuccessful;
}

bool NAV::flow::HasUnsavedChanges()
{
    return unsavedChanges;
}

void NAV::flow::ApplyChanges()
{
    // This prevents the newly loaded gui elements from triggering the unsaved changes
    if (ImGui::GetCurrentContext() && ImGui::GetFrameCount() - loadingFrameCount >= loadingFramesToWait)
    {
        unsavedChanges = true;
        if (saveLastActions)
        {
            gui::saveLastAction();
        }
    }
}

void NAV::flow::DiscardChanges()
{
    unsavedChanges = false;
}

std::string NAV::flow::GetCurrentFilename()
{
    return currentFilename;
}

void NAV::flow::SetCurrentFilename(const std::string& newFilename)
{
    currentFilename = newFilename;
}

std::string NAV::flow::GetProgramRootPath()
{
    return programRootPath;
}

void NAV::flow::SetProgramRootPath(const std::string& newRootPath)
{
    programRootPath = newRootPath;
}