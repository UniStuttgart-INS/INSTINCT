// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "internal/FlowManager.hpp"

#include "util/Json.hpp"

#include <implot.h>
#include <imgui_node_editor.h>
namespace ed = ax::NodeEditor;

#include "NodeRegistry.hpp"

#include "internal/Node/Node.hpp"
#include "internal/Node/Pin.hpp"
#include "internal/ConfigManager.hpp"
#include "internal/FlowExecutor.hpp"

#include "internal/gui/windows/ImPlotStyleEditor.hpp"
#include "internal/gui/NodeEditorApplication.hpp"
#include "internal/gui/windows/NodeEditorStyleEditor.hpp"
#include "util/Plot/Colormap.hpp"
#include "util/Logger/CommonLog.hpp"

#include <fstream>
#include <set>
#include <iomanip>
#include <string>
#include <memory>

#include <iostream>

namespace NAV::flow
{
namespace
{

std::vector<NAV::Node*> m_nodes;
size_t m_NextId = 1;

bool unsavedChanges = false;

constexpr int loadingFramesToWait = 2;

std::string currentFilename;
std::filesystem::path programRootPath;

// The current number for the rotated parent folder
size_t currentRotatedParentFolderNumber;

int loadingFrameCount = 0;

size_t GetNextId()
{
    return m_NextId++;
}

} // namespace

} // namespace NAV::flow

const std::vector<NAV::Node*>& NAV::flow::m_Nodes()
{
    return m_nodes;
}

ax::NodeEditor::NodeId NAV::flow::GetNextNodeId()
{
    return { GetNextId() };
}

ax::NodeEditor::LinkId NAV::flow::GetNextLinkId()
{
    return { GetNextId() };
}

ax::NodeEditor::PinId NAV::flow::GetNextPinId()
{
    return { GetNextId() };
}

void NAV::flow::AddNode(NAV::Node* node)
{
    if (!node->id)
    {
        node->id = GetNextNodeId();
    }
    m_nodes.push_back(node);
    LOG_DEBUG("Creating node: {}", node->nameId());

    for (auto& pin : node->inputPins)
    {
        pin.parentNode = node;
    }
    for (auto& pin : node->outputPins)
    {
        pin.parentNode = node;
    }

    m_NextId = std::max(m_NextId, size_t(node->id) + 1);
    for (const auto& pin : node->inputPins)
    {
        m_NextId = std::max(m_NextId, size_t(pin.id) + 1);
    }
    for (const auto& pin : node->outputPins)
    {
        m_NextId = std::max(m_NextId, size_t(pin.id) + 1);
    }

    flow::ApplyChanges();
}

void NAV::flow::UpdateNode(Node* node)
{
    LOG_TRACE("called for node: {}", node->nameId());
    for (auto& pin : node->inputPins)
    {
        pin.parentNode = node;
    }
    for (auto& pin : node->outputPins)
    {
        pin.parentNode = node;
    }

    for (const auto& pin : node->inputPins)
    {
        m_NextId = std::max(m_NextId, size_t(pin.id) + 1);
    }
    for (const auto& pin : node->outputPins)
    {
        m_NextId = std::max(m_NextId, size_t(pin.id) + 1);
    }
}

bool NAV::flow::DeleteNode(ax::NodeEditor::NodeId nodeId)
{
    LOG_TRACE("called for node with id {}", size_t(nodeId));

    auto it = std::ranges::find_if(m_nodes, [nodeId](const auto& node) { return node->id == nodeId; });
    if (it != m_nodes.end())
    {
        Node* node = *it;
        m_nodes.erase(it);
        LOG_DEBUG("Deleting node: {}", node->nameId());

        if (node->isInitialized())
        {
            node->doDeinitialize(true);
        }
        for (auto& inputPin : node->inputPins)
        {
            if (inputPin.isPinLinked())
            {
                inputPin.deleteLink();
            }
        }
        for (auto& outputPin : node->outputPins)
        {
            if (outputPin.isPinLinked())
            {
                outputPin.deleteLinks();
            }
        }

        delete node; // NOLINT(cppcoreguidelines-owning-memory)

        flow::ApplyChanges();

        return true;
    }

    return false;
}

void NAV::flow::DeleteAllNodes()
{
    LOG_TRACE("called");

    while (!m_nodes.empty())
    {
        flow::DeleteNode(m_nodes.back()->id);
    }

    m_NextId = 1;

    flow::ApplyChanges();
}

void NAV::flow::AddLink(ax::NodeEditor::LinkId linkId)
{
    m_NextId = std::max(m_NextId, size_t(linkId) + 1);
}

NAV::Node* NAV::flow::FindNode(ax::NodeEditor::NodeId id)
{
    for (auto& node : m_nodes)
    {
        if (node->id == id)
        {
            return node;
        }
    }

    return nullptr;
}

NAV::OutputPin* NAV::flow::FindOutputPin(ax::NodeEditor::PinId id)
{
    if (!id) { return nullptr; }

    for (auto& node : m_nodes)
    {
        if (!node || node->kind == Node::Kind::GroupBox) { continue; }
        for (auto& pin : node->outputPins)
        {
            if (pin.id == id) { return &pin; }
        }
    }

    return nullptr;
}

NAV::InputPin* NAV::flow::FindInputPin(ax::NodeEditor::PinId id)
{
    if (!id) { return nullptr; }

    for (auto& node : m_nodes)
    {
        if (!node || node->kind == Node::Kind::GroupBox) { continue; }
        for (auto& pin : node->inputPins)
        {
            if (pin.id == id) { return &pin; }
        }
    }

    return nullptr;
}

void NAV::flow::EnableAllCallbacks()
{
    LOG_TRACE("called");
    for (auto* node : m_nodes)
    {
        if (node && !node->isDisabled() && node->kind != Node::Kind::GroupBox)
        {
            node->callbacksEnabled = true;
        }
    }
}

void NAV::flow::DisableAllCallbacks()
{
    LOG_TRACE("called");
    for (auto* node : m_nodes)
    {
        node->callbacksEnabled = false;
    }
}

void NAV::flow::ClearAllNodeQueues()
{
    LOG_TRACE("called");
    for (auto* node : m_nodes)
    {
        for (auto& inputPin : node->inputPins)
        {
            inputPin.queue.clear();
        }
    }
}

bool NAV::flow::InitializeAllNodes()
{
    LOG_TRACE("called");
    bool nodeCouldNotInitialize = false;

    InitializeAllNodesAsync();

    for (auto* node : m_nodes)
    {
        if (node && node->kind != Node::Kind::GroupBox && !node->isDisabled() && !node->isInitialized())
        {
            if (!node->doInitialize(true))
            {
                LOG_ERROR("Node '{}' could not initialize.", node->nameId());
                nodeCouldNotInitialize = true;
            }
        }
    }

    return !nodeCouldNotInitialize;
}

void NAV::flow::InitializeAllNodesAsync()
{
    LOG_TRACE("called");

    for (auto* node : m_nodes)
    {
        if (node && node->kind != Node::Kind::GroupBox && !node->isDisabled() && !node->isInitialized())
        {
            node->doInitialize();
        }
    }
}

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
    for (const auto& node : m_Nodes())
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

    j["fonts"]["useBigDefaultFont"] = gui::NodeEditorApplication::isUsingBigDefaultFont();
    j["fonts"]["useBigWindowFont"] = gui::NodeEditorApplication::isUsingBigWindowFont();
    j["fonts"]["useBigPanelFont"] = gui::NodeEditorApplication::isUsingBigPanelFont();
    j["fonts"]["useBigMonoFont"] = gui::NodeEditorApplication::isUsingBigMonoFont();
    j["leftPane"]["hide"] = gui::NodeEditorApplication::hideLeftPane;
    j["leftPane"]["leftWidth"] = gui::NodeEditorApplication::leftPaneWidth;
    j["leftPane"]["rightWidth"] = gui::NodeEditorApplication::rightPaneWidth;
    j["bottomViewHeight"] = gui::NodeEditorApplication::bottomViewHeight;
    j["hideFPS"] = gui::NodeEditorApplication::hideFPS;
    j["lightMode"] = gui::windows::nodeEditorLightMode;
    j["gridLinesEnabled"] = ed::GetStyle().Colors[ed::StyleColor_Grid].w;
    j["transparentWindows"] = ImGui::GetStyle().Colors[ImGuiCol_WindowBg].w;
    j["header"]["colors"]["disabled"] = gui::NodeEditorApplication::headerColorDisabled;
    j["header"]["colors"]["initialized"] = gui::NodeEditorApplication::headerColorInitialized;
    j["header"]["colors"]["deinitialized"] = gui::NodeEditorApplication::headerColorDeinitialized;

    // if (ImGui::Checkbox("Light mode", &nodeEditorLightMode))
    // {
    //     ApplyDarkLightMode(NodeEditorApplication::m_colors);
    //     flow::ApplyChanges();
    // }

    j["colormaps"] = ColormapsFlow;
    j["commonLog"] = CommonLog::save();

    filestream << std::setw(4) << j << std::endl; // NOLINT(performance-avoid-endl)

    unsavedChanges = false;
}

bool NAV::flow::LoadFlow(const std::string& filepath)
{
    LOG_TRACE("called for path {}", filepath);
    bool loadSuccessful = true;

    try
    {
        std::ifstream filestream(filepath);

        if (!filestream.good())
        {
            LOG_ERROR("Load Flow error: Could not open file: {}", filepath);
            return false;
        }

        if (FlowExecutor::isRunning()) { FlowExecutor::stop(); }

        json j;
        filestream >> j;

        DeleteAllNodes();

        if (!j.contains("commonLog")) { CommonLog::restore(json{}); }
        LoadJson(j);

#ifdef TESTING
        CallPreInitCallback();
#endif

        if (!ConfigManager::Get<bool>("noinit"))
        {
            if (ConfigManager::Get<bool>("nogui"))
            {
                if (!InitializeAllNodes())
                {
                    loadSuccessful = false;
                }
            }
            else
            {
                InitializeAllNodesAsync();
            }
        }

        if (!ConfigManager::Get<bool>("nogui"))
        {
            loadingFrameCount = ImGui::GetFrameCount();
        }
        unsavedChanges = false;
        currentFilename = filepath;

        std::string path = filepath;
        if (path.find(GetProgramRootPath().string()) != std::string::npos)
        {
            path = path.substr(GetProgramRootPath().string().size());
            if (path.starts_with('\\') || path.starts_with('/')) { path = path.substr(1); }
        }

        LOG_INFO("Loaded flow file: {}", path);
    }
    catch (const std::exception& e)
    {
        LOG_ERROR("Loading flow file failed with error: {}", e.what());
        loadSuccessful = false;
    }

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
    else
    {
        gui::windows::saveConfigInFlow = false;
    }

    if (j.contains("colormaps"))
    {
        j.at("colormaps").get_to(ColormapsFlow);
    }
    else
    {
        ColormapsFlow.clear();
    }
    if (j.contains("commonLog")) { CommonLog::restore(j.at("commonLog")); }

    if (!ConfigManager::Get<bool>("nogui"))
    {
        if (j.contains("fonts"))
        {
            if (j.at("fonts").contains("useBigDefaultFont"))
            {
                gui::NodeEditorApplication::swapDefaultFont(j.at("fonts").at("useBigDefaultFont").get<bool>());
            }
            if (j.at("fonts").contains("useBigWindowFont"))
            {
                gui::NodeEditorApplication::swapWindowFont(j.at("fonts").at("useBigWindowFont").get<bool>());
            }
            if (j.at("fonts").contains("useBigPanelFont"))
            {
                gui::NodeEditorApplication::swapPanelFont(j.at("fonts").at("useBigPanelFont").get<bool>());
            }
            if (j.at("fonts").contains("useBigMonoFont"))
            {
                gui::NodeEditorApplication::swapMonoFont(j.at("fonts").at("useBigMonoFont").get<bool>());
            }
        }
        if (j.contains("leftPane"))
        {
            j.at("leftPane").at("hide").get_to(gui::NodeEditorApplication::hideLeftPane);
            j.at("leftPane").at("leftWidth").get_to(gui::NodeEditorApplication::leftPaneWidth);
            j.at("leftPane").at("rightWidth").get_to(gui::NodeEditorApplication::rightPaneWidth);
        }
        if (j.contains("bottomViewHeight")) { j.at("bottomViewHeight").get_to(gui::NodeEditorApplication::bottomViewHeight); }
        if (j.contains("hideFPS")) { j.at("hideFPS").get_to(gui::NodeEditorApplication::hideFPS); }
        if (j.contains("lightMode"))
        {
            j.at("lightMode").get_to(gui::windows::nodeEditorLightMode);
            gui::windows::ApplyDarkLightMode(gui::NodeEditorApplication::m_colors, ImGui::GetStyle().Colors[ImGuiCol_WindowBg].w != 1.0F);
        }
        if (j.contains("gridLinesEnabled")) { j.at("gridLinesEnabled").get_to(ed::GetStyle().Colors[ed::StyleColor_Grid].w); }
        if (j.contains("transparentWindows")) { j.at("transparentWindows").get_to(ImGui::GetStyle().Colors[ImGuiCol_WindowBg].w); }
        if (j.contains("header") && j.at("header").contains("colors"))
        {
            const auto& js = j.at("header").at("colors");
            if (js.contains("disabled")) { js.at("disabled").get_to(gui::NodeEditorApplication::headerColorDisabled); }
            if (js.contains("initialized")) { js.at("initialized").get_to(gui::NodeEditorApplication::headerColorInitialized); }
            if (js.contains("deinitialized")) { js.at("deinitialized").get_to(gui::NodeEditorApplication::headerColorDeinitialized); }
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

            AddNode(node);
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
                    pin.id = GetNextPinId();
                }
                for (auto& pin : node->outputPins)
                {
                    pin.id = GetNextPinId();
                }
            }

            UpdateNode(node);

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
                for (auto* node : m_Nodes())
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

#ifdef TESTING

namespace
{
std::vector<std::pair<ax::NodeEditor::PinId, NAV::InputPin::WatcherCallback>> watcherPinList;
std::vector<std::pair<ax::NodeEditor::LinkId, NAV::InputPin::WatcherCallback>> watcherLinkList;

std::function<void()> preInitCallback = nullptr;
std::function<void()> cleanupCallback = nullptr;

} // namespace

void NAV::flow::RegisterWatcherCallbackToInputPin(ax::NodeEditor::PinId id, const InputPin::WatcherCallback& callback)
{
    watcherPinList.emplace_back(id, callback);
}

void NAV::flow::RegisterWatcherCallbackToLink(ax::NodeEditor::LinkId id, const InputPin::WatcherCallback& callback)
{
    watcherLinkList.emplace_back(id, callback);
}

void NAV::flow::ApplyWatcherCallbacks()
{
    for (auto& [linkId, callback] : watcherLinkList)
    {
        for (auto& node : m_nodes)
        {
            for (size_t pinIdx = 0; pinIdx < node->inputPins.size(); pinIdx++)
            {
                auto& pin = node->inputPins[pinIdx];
                if (pin.isPinLinked() && pin.link.linkId == linkId)
                {
                    LOG_DEBUG("Adding watcher callback on node '{}' on pin with index {}", pin.parentNode->nameId(), pinIdx);
                    pin.watcherCallbacks.emplace_back(callback);
                }
            }
        }
    }

    for (auto& [id, callback] : watcherPinList)
    {
        for (auto& node : m_nodes)
        {
            for (size_t pinIdx = 0; pinIdx < node->inputPins.size(); pinIdx++)
            {
                auto& pin = node->inputPins[pinIdx];
                if (pin.id == id)
                {
                    LOG_DEBUG("Adding watcher callback on node '{}' on pin with index {}", pin.parentNode->nameId(), pinIdx);
                    pin.watcherCallbacks.emplace_back(callback);
                }
            }
        }
    }
}

void NAV::flow::RegisterPreInitCallback(std::function<void()> callback)
{
    preInitCallback = std::move(callback);
}

void NAV::flow::CallPreInitCallback()
{
    if (preInitCallback)
    {
        preInitCallback();
    }
}

void NAV::flow::RegisterCleanupCallback(std::function<void()> callback)
{
    cleanupCallback = std::move(callback);
}
void NAV::flow::CallCleanupCallback()
{
    if (cleanupCallback)
    {
        cleanupCallback();
    }
}

void NAV::flow::ClearRegisteredCallbacks()
{
    watcherPinList.clear();
    watcherLinkList.clear();
    preInitCallback = nullptr;
    cleanupCallback = nullptr;
}

#endif