#include "internal/FlowManager.hpp"

#include "util/Json.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;

#include <imgui_node_editor.h>
namespace ed = ax::NodeEditor;

#include "NodeRegistry.hpp"

#include "internal/Node/Node.hpp"
#include "internal/Node/Link.hpp"
#include "internal/Node/Pin.hpp"
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
std::filesystem::path programRootPath;

// The current number for the rotated parent folder
size_t currentRotatedParentFolderNumber;

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

    if (!ConfigManager::Get<bool>("nogui", false))
    {
        gui::clearLastActionList();
        gui::saveLastAction();
    }

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

                if (node->getSize().x > 0 && node->getSize().y > 0)
                {
                    ed::SetGroupSize(node->id, node->getSize());
                }
            }
        }
    }

    // Collect the node ids which get new links to call the restoreAfterLinks function on them
    std::set<size_t> newlyLinkedNodes;

    if (j.contains("links"))
    {
        for (const auto& linkJson : j.at("links"))
        {
            Link link = linkJson.get<Link>();

            if (!nm::FindLink(link.id))
            {
                if (!nm::AddLink(link))
                {
                    loadSuccessful = false;
                }

                if (auto* node = nm::FindConnectedNodeToInputPin(link.endPinId))
                {
                    newlyLinkedNodes.insert(size_t(node->id));
                }
                for (auto* node : nm::FindConnectedNodesToOutputPin(link.startPinId))
                {
                    newlyLinkedNodes.insert(size_t(node->id));
                }
            }
        }
    }
    if (j.contains("nodes"))
    {
        for (auto nodeId : newlyLinkedNodes)
        {
            auto* node = nm::FindNode(nodeId);
            if (j.at("nodes").contains("node-" + std::to_string(size_t(node->id))))
            {
                LOG_DEBUG("Calling restoreAtferLink() for new node '{}'", node->nameId());

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

std::filesystem::path NAV::flow::GetProgramRootPath()
{
    return programRootPath;
}

void NAV::flow::SetProgramRootPath(const std::filesystem::path& newRootPath)
{
    LOG_DEBUG("Program root path set to {}", newRootPath);
    programRootPath = newRootPath;
}

std::filesystem::path NAV::flow::GetOutputPath()
{
    std::filesystem::path filepath = flow::GetProgramRootPath();

    if (std::filesystem::path outputPath{ ConfigManager::Get<std::string>("output-path", "logs") };
        outputPath.is_relative())
    {
        filepath /= outputPath;
    }
    else
    {
        filepath = outputPath;
    }

    if (ConfigManager::Get<bool>("rotate-output", false))
    {
        filepath /= fmt::format("{:04d}", currentRotatedParentFolderNumber);
    }

    return filepath;
}

void NAV::flow::SetOutputPath()
{
    currentRotatedParentFolderNumber = 0;
    for (int i = 10000; i >= 0; --i)
    {
        std::filesystem::path outputDir{ programRootPath };

        if (std::filesystem::path outputPath{ ConfigManager::Get<std::string>("output-path", "logs") };
            outputPath.is_relative())
        {
            outputDir /= outputPath;
        }
        else
        {
            outputDir = outputPath;
        }
        outputDir /= fmt::format("{:04d}", i);
        if (std::filesystem::exists(outputDir))
        {
            currentRotatedParentFolderNumber = static_cast<size_t>(i + 1); // NOLINT(bugprone-misplaced-widening-cast)
            break;
        }
    }
    LOG_DEBUG("Output directory set to {}", GetOutputPath());
}

std::filesystem::path NAV::flow::GetInputPath()
{
    std::filesystem::path filepath = flow::GetProgramRootPath();

    if (std::filesystem::path inputPath{ ConfigManager::Get<std::string>("input-path", "data") };
        inputPath.is_relative())
    {
        filepath /= inputPath;
    }
    else
    {
        filepath = inputPath;
    }

    return filepath;
}

std::filesystem::path NAV::flow::GetFlowPath()
{
    std::filesystem::path filepath = flow::GetProgramRootPath();

    if (std::filesystem::path inputPath{ ConfigManager::Get<std::string>("flow-path", "flow") };
        inputPath.is_relative())
    {
        filepath /= inputPath;
    }
    else
    {
        filepath = inputPath;
    }

    return filepath;
}