// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "internal/FlowManager.hpp"

#include "util/Json.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;

#include <implot.h>
#include <imgui_node_editor.h>
namespace ed = ax::NodeEditor;

#include "NodeRegistry.hpp"

#include "internal/Node/Node.hpp"
#include "internal/Node/Pin.hpp"
#include "internal/ConfigManager.hpp"
#include "internal/FlowExecutor.hpp"

#include "internal/gui/windows/ImPlotStyleEditor.hpp"

#include <fstream>
#include <set>
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

        for (const auto& outputPin : node->outputPins)
        {
            for (const auto& link : outputPin.links)
            {
                auto& jLink = j["links"]["link-" + std::to_string(size_t(link.linkId))];
                jLink["id"] = size_t(link.linkId);
                jLink["startPinId"] = size_t(outputPin.id);
                jLink["endPinId"] = size_t(link.connectedPinId);
            }
        }
    }
    if (gui::windows::saveConfigInFlow)
    {
        j["implot"]["style"] = ImPlot::GetStyle();
        j["implot"]["prefereFlowOverGlobal"] = gui::windows::prefereFlowOverGlobal;
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

    if (FlowExecutor::isRunning()) { FlowExecutor::stop(); }

    json j;
    filestream >> j;

    saveLastActions = false;

    nm::DeleteAllNodes();

    LoadJson(j);

#ifdef TESTING
    nm::CallPreInitCallback();
#endif

    if (!ConfigManager::Get<bool>("noinit"))
    {
        if (ConfigManager::Get<bool>("nogui"))
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
    }

    if (!ConfigManager::Get<bool>("nogui"))
    {
        loadingFrameCount = ImGui::GetFrameCount();
    }
    unsavedChanges = false;
    saveLastActions = true;
    currentFilename = filepath;

    if (!ConfigManager::Get<bool>("nogui"))
    {
        gui::clearLastActionList();
        gui::saveLastAction();
    }

    std::string path = filepath;
    if (path.find(GetProgramRootPath().string()) != std::string::npos)
    {
        path = path.substr(GetProgramRootPath().string().size());
        if (path.starts_with('\\') || path.starts_with('/')) { path = path.substr(1); }
    }

    LOG_INFO("Loaded flow file: {}", path);

    return loadSuccessful;
}

bool NAV::flow::LoadJson(const json& j, bool requestNewIds)
{
    bool loadSuccessful = true;

    if (j.contains("implot"))
    {
        gui::windows::saveConfigInFlow = true;

        if (j.at("implot").contains("prefereFlowOverGlobal"))
        {
            j.at("implot").at("prefereFlowOverGlobal").get_to(gui::windows::prefereFlowOverGlobal);
        }

        std::filesystem::path filepath = flow::GetProgramRootPath();
        if (std::filesystem::path inputPath{ ConfigManager::Get<std::string>("implot-config") };
            inputPath.is_relative())
        {
            filepath /= inputPath;
        }
        else
        {
            filepath = inputPath;
        }

        if (gui::windows::prefereFlowOverGlobal || !std::filesystem::exists(filepath))
        {
            if (!ConfigManager::Get<bool>("nogui"))
            {
                if (j.at("implot").contains("style"))
                {
                    j.at("implot").at("style").get_to(ImPlot::GetStyle());
                }
            }
        }
    }

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
                    if (nodeInfo.type == nodeJson.at("type").get<std::string>())
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

            if (!ConfigManager::Get<bool>("nogui"))
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
    std::set<Node*> newlyLinkedNodes;

    if (j.contains("links"))
    {
        for (size_t i = 0; i < 2; i++) // Run twice because pins can change type depending on other links
        {
            for (const auto& linkJson : j.at("links"))
            {
                auto linkId = linkJson.at("id").get<size_t>();
                auto startPinId = linkJson.at("startPinId").get<size_t>();
                auto endPinId = linkJson.at("endPinId").get<size_t>();

                InputPin* endPin = nullptr;
                OutputPin* startPin = nullptr;
                for (auto* node : nm::m_Nodes())
                {
                    if (!endPin)
                    {
                        for (auto& inputPin : node->inputPins)
                        {
                            if (endPinId == size_t(inputPin.id)) { endPin = &inputPin; }
                        }
                    }
                    if (!startPin)
                    {
                        for (auto& outputPin : node->outputPins)
                        {
                            if (startPinId == size_t(outputPin.id)) { startPin = &outputPin; }
                        }
                    }
                    if (startPin && endPin) { break; }
                }
                if (startPin && endPin)
                {
                    if (!startPin->createLink(*endPin, linkId))
                    {
                        loadSuccessful = false;
                        continue;
                    }
                    newlyLinkedNodes.insert(startPin->parentNode);
                    newlyLinkedNodes.insert(endPin->parentNode);
                }
            }
        }
    }
    if (j.contains("nodes"))
    {
        for (auto* node : newlyLinkedNodes)
        {
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

    if (std::filesystem::path outputPath{ ConfigManager::Get<std::string>("output-path") };
        outputPath.is_relative())
    {
        filepath /= outputPath;
    }
    else
    {
        filepath = outputPath;
    }

    if (ConfigManager::Get<bool>("rotate-output"))
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

        if (std::filesystem::path outputPath{ ConfigManager::Get<std::string>("output-path") };
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

    if (std::filesystem::path inputPath{ ConfigManager::Get<std::string>("input-path") };
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

    if (std::filesystem::path inputPath{ ConfigManager::Get<std::string>("flow-path") };
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

std::filesystem::path NAV::flow::GetConfigPath()
{
    return flow::GetProgramRootPath() / "config";
}