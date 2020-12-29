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
    auto id = std::find_if(m_nodes.begin(),
                           m_nodes.end(),
                           [nodeId](const auto& node) { return node->id == nodeId; });
    if (id != m_nodes.end())
    {
        delete *id; // NOLINT(cppcoreguidelines-owning-memory)
        m_nodes.erase(id);

        flow::ApplyChanges();

        return true;
    }

    return false;
}

void NAV::NodeManager::DeleteAllNodes()
{
    for (auto& node : m_nodes)
    {
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

    m_links.emplace_back(GetNextLinkId(), startPin->id, endPin->id, startPin->getIconColor());

    endPin->data = startPin->data;

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
        endPin->data = startPin->data;
    }
    else
    {
        LOG_CRITICAL("Tried to add Link from pinId {} to {}, which one of them does not exist",
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
        FindPin(id->endPinId)->data = static_cast<void*>(nullptr);

        m_links.erase(id);

        flow::ApplyChanges();

        return true;
    }

    return false;
}

void NAV::NodeManager::DeleteAllLinks()
{
    for (auto& link : m_links)
    {
        FindPin(link.endPinId)->data = static_cast<void*>(nullptr);
    }

    m_links.clear();

    m_NextId = 1;

    for (const auto& node : m_nodes)
    {
        m_NextId = std::max(m_NextId, reinterpret_cast<uintptr_t>(node->id.AsPointer()) + 1);
    }

    flow::ApplyChanges();
}

NAV::Pin* NAV::NodeManager::CreateInputPin(NAV::Node* node, const char* name, NAV::Pin::Type pinType, const std::string_view& dataIdentifier)
{
    node->inputPins.emplace_back(GetNextPinId(), name, pinType, Pin::Kind::Input, node);

    node->inputPins.back().dataIdentifier = dataIdentifier;

    flow::ApplyChanges();

    return &node->inputPins.back();
}

NAV::Pin* NAV::NodeManager::CreateOutputPin(NAV::Node* node, const char* name, NAV::Pin::Type pinType, NAV::Pin::PinData data, const std::string_view& dataIdentifier)
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
