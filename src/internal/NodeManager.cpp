#include "internal/NodeManager.hpp"
namespace ed = ax::NodeEditor;

#include "Nodes/Node.hpp"
#include "internal/Link.hpp"
#include "internal/Pin.hpp"

#include "internal/FlowManager.hpp"

#include <algorithm>

/* -------------------------------------------------------------------------------------------------------- */
/*                                              Private Members                                             */
/* -------------------------------------------------------------------------------------------------------- */

std::vector<NAV::Node*> m_nodes;
std::vector<NAV::Link> m_links;
size_t m_NextId = 1;

/* -------------------------------------------------------------------------------------------------------- */
/*                                       Private Function Declarations                                      */
/* -------------------------------------------------------------------------------------------------------- */

namespace NAV::NodeManager
{
size_t GetNextId();

ed::NodeId GetNextNodeId();

ed::LinkId GetNextLinkId();

ed::PinId GetNextPinId();

} // namespace NAV::NodeManager

/* -------------------------------------------------------------------------------------------------------- */
/*                                               Public Members                                             */
/* -------------------------------------------------------------------------------------------------------- */

namespace NAV::NodeManager
{
bool showFlowWhenInvokingCallbacks = true;

} // namespace NAV::NodeManager

/* -------------------------------------------------------------------------------------------------------- */
/*                                           Function Definitions                                           */
/* -------------------------------------------------------------------------------------------------------- */

const std::vector<NAV::Node*>& NAV::NodeManager::m_Nodes()
{
    return m_nodes;
}

const std::vector<NAV::Link>& NAV::NodeManager::m_Links()
{
    return m_links;
}

void NAV::NodeManager::AddNode(NAV::Node* node)
{
    if (!node->id)
    {
        node->id = GetNextNodeId();
    }
    m_nodes.push_back(node);
    LOG_DEBUG("Creating node {}", size_t(node->id));

    if (node->outputPins.empty() || node->outputPins.front().type != Pin::Type::Delegate)
    {
        Pin pin = Pin(GetNextPinId(), "", Pin::Type::Delegate, Pin::Kind::Output, node);

        pin.data = node;
        pin.dataIdentifier = node->type();

        node->outputPins.insert(node->outputPins.begin(), pin);
    }

    for (auto& pin : node->inputPins)
    {
        pin.parentNode = node;
    }
    for (auto& pin : node->outputPins)
    {
        pin.parentNode = node;
    }

    m_NextId = std::max(m_NextId, reinterpret_cast<uintptr_t>(node->id.AsPointer()) + 1);
    for (const auto& pin : node->inputPins)
    {
        m_NextId = std::max(m_NextId, reinterpret_cast<uintptr_t>(pin.id.AsPointer()) + 1);
    }
    for (const auto& pin : node->outputPins)
    {
        m_NextId = std::max(m_NextId, reinterpret_cast<uintptr_t>(pin.id.AsPointer()) + 1);
    }

    flow::ApplyChanges();
}

bool NAV::NodeManager::DeleteNode(ed::NodeId nodeId)
{
    auto it = std::find_if(m_nodes.begin(),
                           m_nodes.end(),
                           [nodeId](const auto& node) { return node->id == nodeId; });
    if (it != m_nodes.end())
    {
        LOG_DEBUG("Deleting node {}", size_t(nodeId));
        for (Pin& inputPin : (*it)->inputPins)
        {
            for (size_t i = 0; i < m_links.size();)
            {
                if (m_links.at(i).endPinId == inputPin.id)
                {
                    NodeManager::DeleteLink(m_links.at(i).id);
                    continue;
                }
                i++;
            }
        }
        for (Pin& outputPin : (*it)->outputPins)
        {
            for (size_t i = 0; i < m_links.size();)
            {
                if (m_links.at(i).startPinId == outputPin.id)
                {
                    NodeManager::DeleteLink(m_links.at(i).id);
                    continue;
                }
                i++;
            }
        }

        (*it)->deinitialize();
        delete *it; // NOLINT(cppcoreguidelines-owning-memory)
        m_nodes.erase(it);

        flow::ApplyChanges();

        return true;
    }

    return false;
}

void NAV::NodeManager::DeleteAllNodes()
{
    for (auto& node : m_nodes)
    {
        node->deinitialize();
        delete node; // NOLINT(cppcoreguidelines-owning-memory)
    }

    m_nodes.clear();
    m_NextId = 1;

    for (const auto& link : m_links)
    {
        m_NextId = std::max(m_NextId, reinterpret_cast<uintptr_t>(link.id.AsPointer()) + 1);
    }

    flow::ApplyChanges();
}

NAV::Link* NAV::NodeManager::CreateLink(NAV::Pin* startPin, NAV::Pin* endPin)
{
    if (!startPin || !endPin)
    {
        return nullptr;
    }

    if (!startPin->parentNode || !startPin->parentNode->onCreateLink(startPin, endPin)
        || !endPin->parentNode || !endPin->parentNode->onCreateLink(startPin, endPin))
    {
        LOG_ERROR("The new Link was refused by one of the Nodes it should connect to.");
        return nullptr;
    }

    m_links.emplace_back(GetNextLinkId(), startPin->id, endPin->id, startPin->getIconColor());
    LOG_DEBUG("Creating link {} from pin {} to {}", size_t(m_links.back().id), size_t(startPin->id), size_t(endPin->id));

    if (endPin->type == Pin::Type::Flow)
    {
        startPin->callbacks.emplace_back(endPin->parentNode,
                                         std::get<void (NAV::Node::*)(const std::shared_ptr<NAV::NodeData>&, ax::NodeEditor::LinkId)>(endPin->data),
                                         m_links.back().id);
    }
    else
    {
        endPin->data = startPin->data;
        if (startPin->parentNode && endPin->parentNode && !startPin->parentNode->isInitialized)
        {
            endPin->parentNode->deinitialize();
        }
    }

    flow::ApplyChanges();

    return &m_links.back();
}

void NAV::NodeManager::AddLink(const NAV::Link& link)
{
    m_links.push_back(link);

    Pin* startPin = FindPin(link.startPinId);
    Pin* endPin = FindPin(link.endPinId);
    if (endPin && startPin)
    {
        if (!startPin->parentNode || !startPin->parentNode->onCreateLink(startPin, endPin)
            || !endPin->parentNode || !endPin->parentNode->onCreateLink(startPin, endPin))
        {
            LOG_ERROR("Link {} was refused by one of the Nodes it should connect to.", size_t(link.id));
            m_links.pop_back();
            return;
        }

        if (endPin->type == Pin::Type::Flow)
        {
            startPin->callbacks.emplace_back(endPin->parentNode,
                                             std::get<void (NAV::Node::*)(const std::shared_ptr<NAV::NodeData>&, ax::NodeEditor::LinkId)>(endPin->data),
                                             link.id);
        }
        else
        {
            endPin->data = startPin->data;
            if (startPin->parentNode && endPin->parentNode && !startPin->parentNode->isInitialized)
            {
                endPin->parentNode->deinitialize();
            }
        }
    }
    else
    {
        LOG_CRITICAL("Tried to add Link from pinId {} to {}, but one of them does not exist",
                     reinterpret_cast<uintptr_t>(link.startPinId.AsPointer()),
                     reinterpret_cast<uintptr_t>(link.endPinId.AsPointer()));
    }

    m_NextId = std::max(m_NextId, reinterpret_cast<uintptr_t>(link.id.AsPointer()) + 1);

    flow::ApplyChanges();
}

bool NAV::NodeManager::DeleteLink(ed::LinkId linkId)
{
    auto id = std::find_if(m_links.begin(),
                           m_links.end(),
                           [linkId](const auto& link) { return link.id == linkId; });
    if (id != m_links.end())
    {
        LOG_DEBUG("Deleting link {}", size_t(linkId));
        Pin* endPin = FindPin(id->endPinId);
        Pin* startPin = FindPin(id->startPinId);

        if (startPin && endPin)
        {
            if (startPin->parentNode)
            {
                startPin->parentNode->onDeleteLink(startPin, endPin);
            }
            if (endPin->parentNode)
            {
                endPin->parentNode->onDeleteLink(startPin, endPin);
            }

            if (endPin->type != Pin::Type::Flow)
            {
                endPin->data = static_cast<void*>(nullptr);
                if (endPin->parentNode)
                {
                    endPin->parentNode->deinitialize();
                }
            }
            else if (startPin->type == Pin::Type::Flow)
            {
                auto iter = std::find(startPin->callbacks.begin(), startPin->callbacks.end(),
                                      std::make_tuple(endPin->parentNode,
                                                      std::get<void (NAV::Node::*)(const std::shared_ptr<NAV::NodeData>&, ax::NodeEditor::LinkId)>(endPin->data),
                                                      linkId));
                if (iter != startPin->callbacks.end())
                {
                    startPin->callbacks.erase(iter);
                }
                else
                {
                    LOG_ERROR("Tried to delete link {}, with type Flow or Function, but could not find the callback.", linkId.AsPointer());
                }
            }
        }

        m_links.erase(id);

        flow::ApplyChanges();

        return true;
    }

    return false;
}

void NAV::NodeManager::DeleteAllLinks()
{
    while (!m_links.empty())
    {
        auto& link = m_links.front();
        NodeManager::DeleteLink(link.id);
    }

    m_NextId = 1;

    for (const auto& node : m_nodes)
    {
        m_NextId = std::max(m_NextId, reinterpret_cast<uintptr_t>(node->id.AsPointer()) + 1);
    }

    flow::ApplyChanges();
}

NAV::Pin* NAV::NodeManager::CreateInputPin(NAV::Node* node, const char* name, NAV::Pin::Type pinType, const std::string& dataIdentifier, NAV::Pin::PinData data)
{
    node->inputPins.emplace_back(GetNextPinId(), name, pinType, Pin::Kind::Input, node);

    node->inputPins.back().data = data;
    node->inputPins.back().dataIdentifier = dataIdentifier;

    flow::ApplyChanges();

    return &node->inputPins.back();
}

NAV::Pin* NAV::NodeManager::CreateOutputPin(NAV::Node* node, const char* name, NAV::Pin::Type pinType, const std::string& dataIdentifier, NAV::Pin::PinData data)
{
    node->outputPins.emplace_back(GetNextPinId(), name, pinType, Pin::Kind::Output, node);

    node->outputPins.back().data = data;
    node->outputPins.back().dataIdentifier = dataIdentifier;

    flow::ApplyChanges();

    return &node->outputPins.back();
}

size_t NAV::NodeManager::GetNextId()
{
    return m_NextId++;
}

ed::NodeId NAV::NodeManager::GetNextNodeId()
{
    return { GetNextId() };
}

ed::LinkId NAV::NodeManager::GetNextLinkId()
{
    return { GetNextId() };
}

ed::PinId NAV::NodeManager::GetNextPinId()
{
    return { GetNextId() };
}

NAV::Node* NAV::NodeManager::FindNode(ed::NodeId id)
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

NAV::Link* NAV::NodeManager::FindLink(ed::LinkId id)
{
    for (auto& link : m_links)
    {
        if (link.id == id)
        {
            return &link;
        }
    }

    return nullptr;
}

NAV::Pin* NAV::NodeManager::FindPin(ed::PinId id)
{
    if (!id)
    {
        return nullptr;
    }

    for (auto& node : m_nodes)
    {
        for (auto& pin : node->inputPins)
        {
            if (pin.id == id)
            {
                return &pin;
            }
        }

        for (auto& pin : node->outputPins)
        {
            if (pin.id == id)
            {
                return &pin;
            }
        }
    }

    return nullptr;
}

bool NAV::NodeManager::IsPinLinked(ed::PinId id)
{
    if (!id)
    {
        return false;
    }

    for (const auto& link : m_links)
    {
        if (link.startPinId == id || link.endPinId == id)
        {
            return true;
        }
    }

    return false;
}

std::vector<NAV::Node*> NAV::NodeManager::FindConnectedNodesToPin(ax::NodeEditor::PinId id)
{
    if (!id)
    {
        return {};
    }

    std::vector<NAV::Node*> connectedNodes;
    for (const auto& link : m_links)
    {
        if (link.startPinId == id)
        {
            connectedNodes.push_back(FindPin(link.endPinId)->parentNode);
        }
        else if (link.endPinId == id)
        {
            connectedNodes.push_back(FindPin(link.startPinId)->parentNode);
        }
    }

    return connectedNodes;
}

std::vector<NAV::Link*> NAV::NodeManager::FindConnectedLinksToPin(ax::NodeEditor::PinId id)
{
    if (!id)
    {
        return {};
    }

    std::vector<NAV::Link*> connectedLinks;
    for (auto& link : m_links)
    {
        if (link.startPinId == id || link.endPinId == id)
        {
            connectedLinks.push_back(&link);
        }
    }

    return connectedLinks;
}

void NAV::NodeManager::EnableAllCallbacks()
{
    for (auto* node : m_nodes)
    {
        node->callbacksEnabled = true;
    }
}

void NAV::NodeManager::DisableAllCallbacks()
{
    for (auto* node : m_nodes)
    {
        node->callbacksEnabled = false;
    }
}