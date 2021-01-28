#include "internal/FlowManager.hpp"

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;

#include <imgui_node_editor.h>
namespace ed = ax::NodeEditor;

#include "NodeRegistry.hpp"

#include "Nodes/Node.hpp"
#include "internal/Link.hpp"
#include "internal/Pin.hpp"
#include "util/ConfigManager.hpp"

#include <fstream>
#include <iomanip>
#include <string>
#include <memory>

#include <iostream>

bool unsavedChanges = false;

constexpr int loadingFramesToWait = 2;
int loadingFrameCount = 0;

std::string currentFilename;
std::string programRootPath;

void to_json(json& j, const ImColor& color)
{
    j = json{
        { "r", static_cast<int>(color.Value.x * 255.0F) },
        { "g", static_cast<int>(color.Value.y * 255.0F) },
        { "b", static_cast<int>(color.Value.z * 255.0F) },
        { "a", static_cast<int>(color.Value.w * 255.0F) },
    };
}
void from_json(const json& j, ImColor& color)
{
    int r = 0;
    int g = 0;
    int b = 0;
    int a = 0;
    j.at("r").get_to(r);
    j.at("g").get_to(g);
    j.at("b").get_to(b);
    j.at("a").get_to(a);

    color = ImColor(r, g, b, a);
}

void to_json(json& j, const ImVec2& vec2)
{
    j = json{
        { "x", vec2.x },
        { "y", vec2.y },
    };
}
void from_json(const json& j, ImVec2& vec2)
{
    j.at("x").get_to(vec2.x);
    j.at("y").get_to(vec2.y);
}

namespace NAV
{
void to_json(json& j, const Pin& pin)
{
    j = json{
        { "id", size_t(pin.id) },
        { "type", std::string(pin.type) },
        { "name", pin.name },
        { "dataIdentifier", pin.dataIdentifier },
    };
}
void from_json(const json& j, Pin& pin)
{
    size_t id = 0;
    j.at("id").get_to(id);
    pin.id = id;

    std::string typeString;
    j.at("type").get_to(typeString);
    pin.type = Pin::Type(typeString);

    j.at("name").get_to(pin.name);

    j.at("dataIdentifier").get_to(pin.dataIdentifier);
}

void to_json(json& j, const Node& node)
{
    ImVec2 realSize = ed::GetNodeSize(node.id);
    realSize.x -= 16;
    realSize.y -= 38;
    j = json{
        { "id", size_t(node.id) },
        { "type", node.type() },
        { "kind", std::string(node.kind) },
        { "name", node.name },
        { "color", node.color },
        { "size", node.size.x == 0 && node.size.y == 0 ? node.size : realSize },
        { "pos", ed::GetNodePosition(node.id) },
        { "inputPins", node.inputPins },
        { "outputPins", node.outputPins },
    };
}
void from_json(const json& j, Node& node)
{
    node.id = j.at("id").get<size_t>();
    node.kind = Node::Kind(j.at("kind").get<std::string>());

    j.at("name").get_to(node.name);
    j.at("color").get_to(node.color);
    j.at("size").get_to(node.size);

    auto inputPins = j.at("inputPins").get<std::vector<Pin>>();
    for (size_t i = 0; i < inputPins.size(); ++i)
    {
        if (node.inputPins.size() <= i)
        {
            break;
        }
        node.inputPins.at(i).id = inputPins.at(i).id;
        node.inputPins.at(i).type = inputPins.at(i).type;
        node.inputPins.at(i).name = inputPins.at(i).name;
        node.inputPins.at(i).dataIdentifier = inputPins.at(i).dataIdentifier;
    }

    auto outputPins = j.at("outputPins").get<std::vector<Pin>>();
    for (size_t i = 0; i < outputPins.size(); ++i)
    {
        if (node.outputPins.size() <= i)
        {
            break;
        }
        node.outputPins.at(i).id = outputPins.at(i).id;
        node.outputPins.at(i).type = outputPins.at(i).type;
        node.outputPins.at(i).name = outputPins.at(i).name;
        node.outputPins.at(i).dataIdentifier = outputPins.at(i).dataIdentifier;
    }
}

void to_json(json& j, const Link& link)
{
    j = json{
        { "id", size_t(link.id) },
        { "startPinId", size_t(link.startPinId) },
        { "endPinId", size_t(link.endPinId) },
        { "color", link.color },
    };
}
void from_json(const json& j, Link& link)
{
    size_t id = 0;
    j.at("id").get_to(id);
    link.id = id;

    j.at("startPinId").get_to(id);
    link.startPinId = id;

    j.at("endPinId").get_to(id);
    link.endPinId = id;

    j.at("color").get_to(link.color);
}

} // namespace NAV

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

    nm::DeleteAllLinks();
    nm::DeleteAllNodes();

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

            nodeJson.get_to<Node>(*node);
            if (nodeJson.contains("data"))
            {
                node->restore(nodeJson.at("data"));
            }
            // Load second time in case restore changed the amount of pins
            nodeJson.get_to<Node>(*node);

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
    currentFilename = filepath;

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

std::string NAV::flow::GetProgramRootPath()
{
    return programRootPath;
}

void NAV::flow::SetProgramRootPath(const std::string& newRootPath)
{
    programRootPath = newRootPath;
}