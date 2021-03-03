#include "internal/NodeManager.hpp"
namespace ed = ax::NodeEditor;

#include "Nodes/Node.hpp"
#include "internal/Link.hpp"
#include "internal/Pin.hpp"

#include "internal/FlowManager.hpp"

#include <algorithm>
#include <thread>

#include "NodeRegistry.hpp"

/* -------------------------------------------------------------------------------------------------------- */
/*                                              Private Members                                             */
/* -------------------------------------------------------------------------------------------------------- */

std::vector<NAV::Node*> m_nodes;
std::vector<NAV::Link> m_links;
size_t m_NextId = 1;
bool nodeInitThread_stopRequested = false;
std::thread nodeInitThread;

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
bool showFlowWhenNotifyingValueChange = true;

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

    // Create Delegate output pin
    if (node->kind == Node::Kind::Blueprint && (node->outputPins.empty() || node->outputPins.front().type != Pin::Type::Delegate))
    {
        Pin pin = Pin(GetNextPinId(), "", Pin::Type::Delegate, Pin::Kind::Output, node);

        for ([[maybe_unused]] const auto& [category, nodeInfoList] : NodeRegistry::RegisteredNodes())
        {
            for (const auto& nodeInfo : nodeInfoList)
            {
                if (nodeInfo.type == node->type())
                {
                    pin.data = reinterpret_cast<void*>(reinterpret_cast<char*>(node) - nodeInfo.addressOffsetNode);
                    break;
                }
            }
            if (std::get<void*>(pin.data) != nullptr)
            {
                break;
            }
        }

        pin.dataIdentifier.push_back(node->type());

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

void NAV::NodeManager::UpdateNode(Node* node)
{
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

bool NAV::NodeManager::DeleteNode(ed::NodeId nodeId)
{
    if (nodeInitThread.joinable())
    {
        // nodeInitThread.request_stop();
        nodeInitThread_stopRequested = true;
        nodeInitThread.join();
    }

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

        if ((*it)->isInitialized())
        {
            (*it)->deinitializeNode();
        }
        delete *it; // NOLINT(cppcoreguidelines-owning-memory)
        m_nodes.erase(it);

        flow::ApplyChanges();

        return true;
    }

    return false;
}

void NAV::NodeManager::DeleteAllNodes()
{
    if (nodeInitThread.joinable())
    {
        // nodeInitThread.request_stop();
        nodeInitThread_stopRequested = true;
        nodeInitThread.join();
    }

    while (!m_nodes.empty())
    {
        NodeManager::DeleteNode(m_nodes.back()->id);
    }

    m_NextId = 1;

    for (const auto& link : m_links)
    {
        m_NextId = std::max(m_NextId, size_t(link.id) + 1);
    }

    flow::ApplyChanges();
}

NAV::Link* NAV::NodeManager::CreateLink(NAV::Pin* startPin, NAV::Pin* endPin)
{
    if (!startPin || !endPin || !startPin->parentNode || !endPin->parentNode)
    {
        return nullptr;
    }

    if (!startPin->parentNode->onCreateLink(startPin, endPin) || !endPin->parentNode->onCreateLink(startPin, endPin))
    {
        LOG_ERROR("The new Link between node '{}' and '{}' was refused by one of the Nodes it should connect to.",
                  startPin->parentNode->nameId(), endPin->parentNode->nameId());
        return nullptr;
    }

    m_links.emplace_back(GetNextLinkId(), startPin->id, endPin->id, startPin->getIconColor());
    LOG_DEBUG("Creating link {} from pin {} to {}", size_t(m_links.back().id), size_t(startPin->id), size_t(endPin->id));

    if (endPin->type == Pin::Type::Flow)
    {
        if (endPin->data.index() == 0)
        {
            LOG_ERROR("Tried to register callback, but endPin has no data");
            m_links.pop_back();
            return nullptr;
        }

        startPin->callbacks.emplace_back(endPin->parentNode,
                                         std::get<void (NAV::Node::*)(const std::shared_ptr<NAV::NodeData>&, ax::NodeEditor::LinkId)>(endPin->data),
                                         m_links.back().id);
    }
    else
    {
        endPin->data = startPin->data;
        if (endPin->type != Pin::Type::Function && endPin->type != Pin::Type::Delegate)
        {
            if (!endPin->notifyFunc.empty())
            {
                startPin->notifyFunc.emplace_back(std::get<0>(endPin->notifyFunc.front()),
                                                  std::get<1>(endPin->notifyFunc.front()),
                                                  m_links.back().id);
            }
        }

        if (startPin->parentNode && endPin->parentNode && !startPin->parentNode->isInitialized())
        {
            if (endPin->parentNode->isInitialized())
            {
                endPin->parentNode->deinitializeNode();
            }
        }
    }

    if (startPin && endPin && startPin->parentNode && endPin->parentNode)
    {
        startPin->parentNode->afterCreateLink(startPin, endPin);
        endPin->parentNode->afterCreateLink(startPin, endPin);
    }

    flow::ApplyChanges();

    return &m_links.back();
}

bool NAV::NodeManager::AddLink(const NAV::Link& link)
{
    m_links.push_back(link);

    Pin* startPin = FindPin(link.startPinId);
    Pin* endPin = FindPin(link.endPinId);
    if (endPin && startPin)
    {
        if (!startPin->parentNode || !endPin->parentNode)
        {
            LOG_ERROR("Tried to add Link from pinId {} to {}, but the pins do not have parentNodes",
                      size_t(link.startPinId), size_t(link.endPinId));
            return false;
        }

        if (!startPin->canCreateLink(*endPin))
        {
            LOG_ERROR("Link {} between node '{}'-{} and '{}'-{} can not be added because the pins do not match", size_t(link.id),
                      startPin->parentNode->nameId(), size_t(startPin->id), endPin->parentNode->nameId(), size_t(endPin->id));
            m_links.pop_back();
            return false;
        }

        if (!startPin->parentNode->onCreateLink(startPin, endPin))
        {
            LOG_ERROR("Link {} between node '{}'-{} and '{}'-{} was refused by the start Node.", size_t(link.id),
                      startPin->parentNode->nameId(), size_t(startPin->id), endPin->parentNode->nameId(), size_t(endPin->id));
            m_links.pop_back();
            return false;
        }
        if (!endPin->parentNode->onCreateLink(startPin, endPin))
        {
            LOG_ERROR("Link {} between node '{}'-{} and '{}'-{} was refused by the end Node.", size_t(link.id),
                      startPin->parentNode->nameId(), size_t(startPin->id), endPin->parentNode->nameId(), size_t(endPin->id));
            // Undo the Link adding on the start node
            startPin->parentNode->onDeleteLink(startPin, endPin);
            m_links.pop_back();
            return false;
        }

        if (endPin->type == Pin::Type::Flow)
        {
            if (endPin->data.index() == 0)
            {
                LOG_ERROR("Tried to register callback, but endPin has no data");
                m_links.pop_back();
                return false;
            }

            startPin->callbacks.emplace_back(endPin->parentNode,
                                             std::get<void (NAV::Node::*)(const std::shared_ptr<NAV::NodeData>&, ax::NodeEditor::LinkId)>(endPin->data),
                                             link.id);
        }
        else
        {
            endPin->data = startPin->data;
            if (endPin->type != Pin::Type::Function && endPin->type != Pin::Type::Delegate)
            {
                if (!endPin->notifyFunc.empty())
                {
                    startPin->notifyFunc.emplace_back(std::get<0>(endPin->notifyFunc.front()),
                                                      std::get<1>(endPin->notifyFunc.front()),
                                                      m_links.back().id);
                }
            }

            if (startPin->parentNode && endPin->parentNode && !startPin->parentNode->isInitialized())
            {
                if (endPin->parentNode->isInitialized())
                {
                    endPin->parentNode->deinitializeNode();
                }
            }
        }

        if (startPin && endPin && startPin->parentNode && endPin->parentNode)
        {
            startPin->parentNode->afterCreateLink(startPin, endPin);
            endPin->parentNode->afterCreateLink(startPin, endPin);
        }
    }
    else
    {
        LOG_ERROR("Tried to add Link from pinId {} to {}, but one of them does not exist",
                  size_t(link.startPinId), size_t(link.endPinId));
        return false;
    }

    m_NextId = std::max(m_NextId, size_t(link.id) + 1);

    flow::ApplyChanges();

    return true;
}

void NAV::NodeManager::RefreshLink(ax::NodeEditor::LinkId linkId)
{
    Link* link = NodeManager::FindLink(linkId);

    if (link == nullptr)
    {
        LOG_ERROR("Tried to refresh Link {}, but the link does not exist", size_t(linkId));
        return;
    }

    Pin* startPin = FindPin(link->startPinId);
    Pin* endPin = FindPin(link->endPinId);
    if (endPin && startPin)
    {
        if (!startPin->parentNode || !endPin->parentNode)
        {
            LOG_ERROR("Tried to refresh Link from pinId {} to {}, but the pins do not have parentNodes",
                      size_t(link->startPinId), size_t(link->endPinId));
            return;
        }

        startPin->parentNode->onDeleteLink(startPin, endPin);
        if (!startPin->parentNode->onCreateLink(startPin, endPin))
        {
            LOG_ERROR("Link {} between node '{}'-{} and '{}'-{} was refused by the start Node.", size_t(link->id),
                      startPin->parentNode->nameId(), size_t(startPin->id), endPin->parentNode->nameId(), size_t(endPin->id));
        }

        endPin->parentNode->onDeleteLink(startPin, endPin);
        if (!endPin->parentNode->onCreateLink(startPin, endPin))
        {
            LOG_ERROR("Link {} between node '{}'-{} and '{}'-{} was refused by the end Node.", size_t(link->id),
                      startPin->parentNode->nameId(), size_t(startPin->id), endPin->parentNode->nameId(), size_t(endPin->id));
        }

        if (endPin->type == Pin::Type::Flow)
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
                LOG_ERROR("Tried to delete link {}, with type Flow, but could not find the callback.", linkId.AsPointer());
            }

            startPin->callbacks.emplace_back(endPin->parentNode,
                                             std::get<void (NAV::Node::*)(const std::shared_ptr<NAV::NodeData>&, ax::NodeEditor::LinkId)>(endPin->data),
                                             link->id);
        }
        else
        {
            endPin->data = startPin->data;
            if (endPin->type != Pin::Type::Function && endPin->type != Pin::Type::Delegate)
            {
                if (!endPin->notifyFunc.empty())
                {
                    auto iter = std::find(startPin->notifyFunc.begin(), startPin->notifyFunc.end(),
                                          std::make_tuple(std::get<0>(endPin->notifyFunc.front()),
                                                          std::get<1>(endPin->notifyFunc.front()),
                                                          linkId));
                    if (iter == startPin->notifyFunc.end())
                    {
                        startPin->notifyFunc.emplace_back(std::get<0>(endPin->notifyFunc.front()),
                                                          std::get<1>(endPin->notifyFunc.front()),
                                                          linkId);
                    }
                }
            }

            if (startPin->parentNode && endPin->parentNode && !startPin->parentNode->isInitialized())
            {
                if (endPin->parentNode->isInitialized())
                {
                    endPin->parentNode->deinitializeNode();
                }
            }
        }

        if (startPin && endPin && startPin->parentNode && endPin->parentNode)
        {
            startPin->parentNode->afterCreateLink(startPin, endPin);
            endPin->parentNode->afterCreateLink(startPin, endPin);
        }
    }
    else
    {
        LOG_ERROR("Tried to refresh Link from pinId {} to {}, but one of them does not exist",
                  size_t(link->startPinId), size_t(link->endPinId));
        return;
    }

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
                if (endPin->type != Pin::Type::Function && endPin->type != Pin::Type::Delegate)
                {
                    if (!endPin->notifyFunc.empty())
                    {
                        auto iter = std::find(startPin->notifyFunc.begin(), startPin->notifyFunc.end(),
                                              std::make_tuple(std::get<0>(endPin->notifyFunc.front()),
                                                              std::get<1>(endPin->notifyFunc.front()),
                                                              linkId));
                        if (iter != startPin->notifyFunc.end())
                        {
                            startPin->notifyFunc.erase(iter);
                        }
                    }
                }

                if (endPin->parentNode)
                {
                    endPin->parentNode->deinitializeNode();
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
                    LOG_ERROR("Tried to delete link {}, with type Flow, but could not find the callback.", linkId.AsPointer());
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
        NodeManager::DeleteLink(m_links.back().id);
    }

    m_NextId = 1;

    for (const auto& node : m_nodes)
    {
        m_NextId = std::max(m_NextId, size_t(node->id) + 1);
    }

    flow::ApplyChanges();
}

NAV::Pin* NAV::NodeManager::CreateInputPin(NAV::Node* node, const char* name, NAV::Pin::Type pinType, const std::vector<std::string>& dataIdentifier, NAV::Pin::PinData data)
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
    node->outputPins.back().dataIdentifier = { dataIdentifier };

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

std::vector<NAV::Node*> NAV::NodeManager::FindConnectedNodesToOutputPin(ax::NodeEditor::PinId id)
{
    std::vector<NAV::Node*> connectedNodes;
    for (const auto& link : m_links)
    {
        if (link.startPinId == id)
        {
            if (Pin* pin = FindPin(link.endPinId))
            {
                if (pin->parentNode)
                {
                    connectedNodes.push_back(pin->parentNode);
                }
            }
        }
    }

    return connectedNodes;
}

NAV::Node* NAV::NodeManager::FindConnectedNodeToInputPin(ax::NodeEditor::PinId id)
{
    for (const auto& link : m_links)
    {
        if (link.endPinId == id)
        {
            if (Pin* pin = FindPin(link.startPinId))
            {
                return pin->parentNode;
            }
        }
    }

    return nullptr;
}

std::vector<NAV::Link*> NAV::NodeManager::FindConnectedLinksToOutputPin(ax::NodeEditor::PinId id)
{
    std::vector<NAV::Link*> connectedLinks;
    for (auto& link : m_links)
    {
        if (link.startPinId == id)
        {
            connectedLinks.push_back(&link);
        }
    }

    return connectedLinks;
}

NAV::Link* NAV::NodeManager::FindConnectedLinkToInputPin(ax::NodeEditor::PinId id)
{
    for (auto& link : m_links)
    {
        if (link.endPinId == id)
        {
            return &link;
        }
    }

    return nullptr;
}

std::vector<NAV::Pin*> NAV::NodeManager::FindConnectedPinsToOutputPin(ax::NodeEditor::PinId id)
{
    std::vector<NAV::Pin*> connectedPins;
    for (auto& link : m_links)
    {
        if (link.startPinId == id)
        {
            connectedPins.push_back(FindPin(link.endPinId));
        }
    }

    return connectedPins;
}

NAV::Pin* NAV::NodeManager::FindConnectedPinToInputPin(ax::NodeEditor::PinId id)
{
    for (auto& link : m_links)
    {
        if (link.endPinId == id)
        {
            return FindPin(link.startPinId);
        }
    }

    return nullptr;
}

void NAV::NodeManager::EnableAllCallbacks()
{
    for (auto* node : m_nodes)
    {
        if (node->enabled)
        {
            node->callbacksEnabled = true;
        }
    }
}

void NAV::NodeManager::DisableAllCallbacks()
{
    for (auto* node : m_nodes)
    {
        node->callbacksEnabled = false;
    }
}

bool NAV::NodeManager::InitializeAllNodes()
{
    LOG_TRACE("called");
    bool nodeCouldNotInitialize = false;
    for (auto* node : m_nodes)
    {
        if (node->enabled && !node->isInitialized())
        {
            if (!node->initializeNode())
            {
                nodeCouldNotInitialize = true;
            }
        }
    }

    return !nodeCouldNotInitialize;
}

void NAV::NodeManager::InitializeAllNodesAsync()
{
    LOG_TRACE("called");
    if (nodeInitThread.joinable())
    {
        LOG_DEBUG("Joining old node Init Thread");
        // nodeInitThread.request_stop();
        nodeInitThread_stopRequested = true;
        nodeInitThread.join();
    }

    // nodeInitThread = std::jthread([](const std::stop_token& st) {
    nodeInitThread_stopRequested = false;
    nodeInitThread = std::thread([]() {
        // If this thread is running, and a node is added, the node vector will be moved and all pointers are invalid
        // That's why a for-range does not work here and we have to access the elements via at()
        size_t amountOfNodes = m_nodes.size();
        for (size_t i = 0; i < amountOfNodes && i < m_nodes.size(); i++)
        {
            // if (st.stop_requested())
            if (nodeInitThread_stopRequested)
            {
                break;
            }
            if (m_nodes.at(i)->enabled && !m_nodes.at(i)->isInitialized())
            {
                m_nodes.at(i)->initializeNode();
            }
        }
    });
}

void NAV::NodeManager::Stop()
{
    if (nodeInitThread.joinable())
    {
        LOG_DEBUG("Joining node Init Thread");
        // nodeInitThread.request_stop();
        nodeInitThread_stopRequested = true;
        nodeInitThread.join();
    }
}