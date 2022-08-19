#include "internal/NodeManager.hpp"

#include "internal/Node/Node.hpp"
#include "internal/Node/Link.hpp"
#include "internal/Node/Pin.hpp"

#include "internal/FlowManager.hpp"

#include "util/Assert.h"

#include <algorithm>
#include <thread>
#include <deque>

#include "NodeRegistry.hpp"

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

} // namespace NAV::NodeManager

/* -------------------------------------------------------------------------------------------------------- */
/*                                               Public Members                                             */
/* -------------------------------------------------------------------------------------------------------- */

namespace NAV::NodeManager
{
#if !defined(WIN32) && !defined(_WIN32) && !defined(__WIN32)
bool showFlowWhenInvokingCallbacks = true;
bool showFlowWhenNotifyingValueChange = true;
#else
bool showFlowWhenInvokingCallbacks = false;
bool showFlowWhenNotifyingValueChange = false;
#endif

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

void NAV::NodeManager::DeleteAllLinksAndNodes()
{
    LOG_TRACE("called");

    bool saveLastActionsValue = NAV::flow::saveLastActions;
    NAV::flow::saveLastActions = false;

    DeleteAllLinks();
    DeleteAllNodes();

    NAV::flow::saveLastActions = saveLastActionsValue;
    flow::ApplyChanges();
}

void NAV::NodeManager::AddNode(NAV::Node* node)
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

void NAV::NodeManager::UpdateNode(Node* node)
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

bool NAV::NodeManager::DeleteNode(ax::NodeEditor::NodeId nodeId)
{
    LOG_TRACE("called for node with id {}", size_t(nodeId));

    auto it = std::find_if(m_nodes.begin(),
                           m_nodes.end(),
                           [nodeId](const auto& node) { return node->id == nodeId; });
    if (it != m_nodes.end())
    {
        LOG_DEBUG("Deleting node: {}", (*it)->nameId());
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
            (*it)->doDeinitialize(true);
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
    LOG_TRACE("called");

    bool saveLastActionsValue = NAV::flow::saveLastActions;
    NAV::flow::saveLastActions = false;

    while (!m_nodes.empty())
    {
        NodeManager::DeleteNode(m_nodes.back()->id);
    }

    m_NextId = 1;

    for (const auto& link : m_links)
    {
        m_NextId = std::max(m_NextId, size_t(link.id) + 1);
    }

    NAV::flow::saveLastActions = saveLastActionsValue;
    flow::ApplyChanges();
}

NAV::Link* NAV::NodeManager::CreateLink(NAV::OutputPin& startPin, NAV::InputPin& endPin)
{
    if (!startPin.parentNode || !endPin.parentNode) { return nullptr; }
    LOG_TRACE("called: {} of [{}] ==> {} of [{}]", size_t(startPin.id), startPin.parentNode->nameId(), size_t(endPin.id), endPin.parentNode->nameId());

    if (!startPin.parentNode->onCreateLink(startPin, endPin) || !endPin.parentNode->onCreateLink(startPin, endPin))
    {
        LOG_ERROR("The new Link between node '{}' and '{}' was refused by one of the Nodes it should connect to.",
                  startPin.parentNode->nameId(), endPin.parentNode->nameId());
        return nullptr;
    }

    m_links.emplace_back(GetNextLinkId(), startPin.id, endPin.id);
    LOG_DEBUG("Creating link {} from pin {} of [{}] ==> {} of [{}]", size_t(m_links.back().id), size_t(startPin.id),
              startPin.parentNode->nameId(), size_t(endPin.id), endPin.parentNode->nameId());

    if (endPin.type == Pin::Type::Flow)
    {
        if (endPin.dataOld.index() == 0)
        {
            LOG_ERROR("Tried to register callback, but endPin has no data");
            m_links.pop_back();
            return nullptr;
        }

        startPin.callbacksOld.emplace_back(endPin.parentNode,
                                           std::get<void (NAV::Node::*)(const std::shared_ptr<const NAV::NodeData>&, ax::NodeEditor::LinkId)>(endPin.dataOld),
                                           m_links.back().id);
    }
    else
    {
        endPin.dataOld = startPin.dataOld;
        if (endPin.type != Pin::Type::Delegate)
        {
            if (!endPin.notifyFuncOld.empty())
            {
                startPin.notifyFuncOld.emplace_back(std::get<0>(endPin.notifyFuncOld.front()),
                                                    std::get<1>(endPin.notifyFuncOld.front()),
                                                    m_links.back().id);
            }
        }

        if (startPin.parentNode && endPin.parentNode && !startPin.parentNode->isInitialized())
        {
            if (endPin.parentNode->isInitialized())
            {
                endPin.parentNode->doDeinitialize(true);
            }
        }
    }

    if (startPin.parentNode && endPin.parentNode)
    {
        startPin.parentNode->afterCreateLink(startPin, endPin);
        endPin.parentNode->afterCreateLink(startPin, endPin);
    }

    flow::ApplyChanges();

    return &m_links.back();
}

bool NAV::NodeManager::AddLink(const NAV::Link& link)
{
    LOG_TRACE("called for link with id {}", size_t(link.id));
    m_links.push_back(link);

    auto* startPin = FindOutputPin(link.startPinId);
    auto* endPin = FindInputPin(link.endPinId);
    if (endPin && startPin)
    {
        if (!startPin->parentNode || !endPin->parentNode)
        {
            LOG_ERROR("Tried to add Link from pinId {} to {}, but the pins do not have parentNodes",
                      size_t(link.startPinId), size_t(link.endPinId));
            return false;
        }

        if (!startPin->parentNode->onCreateLink(*startPin, *endPin))
        {
            LOG_ERROR("Link {} between node '{}'-{} and '{}'-{} was refused by the start Node.", size_t(link.id),
                      startPin->parentNode->nameId(), size_t(startPin->id), endPin->parentNode->nameId(), size_t(endPin->id));
            m_links.pop_back();
            return false;
        }
        if (!endPin->parentNode->onCreateLink(*startPin, *endPin))
        {
            LOG_ERROR("Link {} between node '{}'-{} and '{}'-{} was refused by the end Node.", size_t(link.id),
                      startPin->parentNode->nameId(), size_t(startPin->id), endPin->parentNode->nameId(), size_t(endPin->id));
            // Undo the Link adding on the start node
            startPin->parentNode->onDeleteLink(*startPin, *endPin);
            m_links.pop_back();
            startPin->parentNode->afterDeleteLink(*startPin, *endPin);
            return false;
        }

        if (!startPin->canCreateLink(*endPin))
        {
            LOG_ERROR("Link {} between node '{}'-{} and '{}'-{} can not be added because the pins do not match", size_t(link.id),
                      startPin->parentNode->nameId(), size_t(startPin->id), endPin->parentNode->nameId(), size_t(endPin->id));
            // Undo the Link adding on the start and end node
            startPin->parentNode->onDeleteLink(*startPin, *endPin);
            endPin->parentNode->onDeleteLink(*startPin, *endPin);
            m_links.pop_back();
            startPin->parentNode->afterDeleteLink(*startPin, *endPin);
            endPin->parentNode->afterDeleteLink(*startPin, *endPin);
            return false;
        }

        if (endPin->type == Pin::Type::Flow)
        {
            if (endPin->dataOld.index() == 0)
            {
                LOG_ERROR("Tried to register callback, but endPin has no data");
                m_links.pop_back();
                return false;
            }

            startPin->callbacksOld.emplace_back(endPin->parentNode,
                                                std::get<void (NAV::Node::*)(const std::shared_ptr<const NAV::NodeData>&, ax::NodeEditor::LinkId)>(endPin->dataOld),
                                                link.id);
        }
        else
        {
            endPin->dataOld = startPin->dataOld;
            if (endPin->type != Pin::Type::Delegate)
            {
                if (!endPin->notifyFuncOld.empty())
                {
                    startPin->notifyFuncOld.emplace_back(std::get<0>(endPin->notifyFuncOld.front()),
                                                         std::get<1>(endPin->notifyFuncOld.front()),
                                                         m_links.back().id);
                }
            }

            if (startPin->parentNode && endPin->parentNode && !startPin->parentNode->isInitialized())
            {
                if (endPin->parentNode->isInitialized())
                {
                    endPin->parentNode->doDeinitialize(true);
                }
            }
        }

        if (startPin && endPin && startPin->parentNode && endPin->parentNode)
        {
            startPin->parentNode->afterCreateLink(*startPin, *endPin);
            endPin->parentNode->afterCreateLink(*startPin, *endPin);
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
    LOG_TRACE("called for link with id {}", size_t(linkId));
    Link* link = NodeManager::FindLink(linkId);

    if (link == nullptr)
    {
        LOG_ERROR("Tried to refresh Link {}, but the link does not exist", size_t(linkId));
        return;
    }

    auto* startPin = FindOutputPin(link->startPinId);
    auto* endPin = FindInputPin(link->endPinId);
    if (endPin && startPin)
    {
        if (!startPin->parentNode || !endPin->parentNode)
        {
            LOG_ERROR("Tried to refresh Link from pinId {} to {}, but the pins do not have parentNodes",
                      size_t(link->startPinId), size_t(link->endPinId));
            return;
        }

        startPin->parentNode->onDeleteLink(*startPin, *endPin);
        if (!startPin->parentNode->onCreateLink(*startPin, *endPin))
        {
            LOG_ERROR("Link {} between node '{}'-{} and '{}'-{} was refused by the start Node.", size_t(link->id),
                      startPin->parentNode->nameId(), size_t(startPin->id), endPin->parentNode->nameId(), size_t(endPin->id));
        }

        endPin->parentNode->onDeleteLink(*startPin, *endPin);
        if (!endPin->parentNode->onCreateLink(*startPin, *endPin))
        {
            LOG_ERROR("Link {} between node '{}'-{} and '{}'-{} was refused by the end Node.", size_t(link->id),
                      startPin->parentNode->nameId(), size_t(startPin->id), endPin->parentNode->nameId(), size_t(endPin->id));
        }

        if (endPin->type == Pin::Type::Flow)
        {
            auto iter = std::find(startPin->callbacksOld.begin(), startPin->callbacksOld.end(),
                                  std::make_tuple(endPin->parentNode,
                                                  std::get<void (NAV::Node::*)(const std::shared_ptr<const NAV::NodeData>&, ax::NodeEditor::LinkId)>(endPin->dataOld),
                                                  linkId));
            if (iter != startPin->callbacksOld.end())
            {
                startPin->callbacksOld.erase(iter);
            }
            else
            {
                LOG_ERROR("Tried to delete link {}, with type Flow, but could not find the callback.", linkId.AsPointer());
            }

            startPin->callbacksOld.emplace_back(endPin->parentNode,
                                                std::get<void (NAV::Node::*)(const std::shared_ptr<const NAV::NodeData>&, ax::NodeEditor::LinkId)>(endPin->dataOld),
                                                link->id);
        }
        else
        {
            endPin->dataOld = startPin->dataOld;
            if (endPin->type != Pin::Type::Delegate)
            {
                if (!endPin->notifyFuncOld.empty())
                {
                    auto iter = std::find(startPin->notifyFuncOld.begin(), startPin->notifyFuncOld.end(),
                                          std::make_tuple(std::get<0>(endPin->notifyFuncOld.front()),
                                                          std::get<1>(endPin->notifyFuncOld.front()),
                                                          linkId));
                    if (iter == startPin->notifyFuncOld.end())
                    {
                        startPin->notifyFuncOld.emplace_back(std::get<0>(endPin->notifyFuncOld.front()),
                                                             std::get<1>(endPin->notifyFuncOld.front()),
                                                             linkId);
                    }
                }
            }

            if (startPin->parentNode && endPin->parentNode && !startPin->parentNode->isInitialized())
            {
                if (endPin->parentNode->isInitialized())
                {
                    endPin->parentNode->doDeinitialize(true);
                }
            }
        }

        if (startPin && endPin && startPin->parentNode && endPin->parentNode)
        {
            startPin->parentNode->afterCreateLink(*startPin, *endPin);
            endPin->parentNode->afterCreateLink(*startPin, *endPin);
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

bool NAV::NodeManager::DeleteLink(ax::NodeEditor::LinkId linkId)
{
    LOG_TRACE("called for link with id {}", size_t(linkId));
    auto id = std::find_if(m_links.begin(),
                           m_links.end(),
                           [linkId](const auto& link) { return link.id == linkId; });
    if (id != m_links.end())
    {
        auto* startPin = FindOutputPin(id->startPinId);
        auto* endPin = FindInputPin(id->endPinId);

        if (startPin && endPin)
        {
            LOG_DEBUG("Deleting link {} from pin {} of [{}] ==> pin {} of [{}]", size_t(linkId),
                      size_t(startPin->id), startPin->parentNode->nameId(), size_t(endPin->id), endPin->parentNode->nameId());
            if (startPin->parentNode)
            {
                startPin->parentNode->onDeleteLink(*startPin, *endPin);
            }
            if (endPin->parentNode)
            {
                endPin->parentNode->onDeleteLink(*startPin, *endPin);
            }

            if (endPin->type != Pin::Type::Flow)
            {
                endPin->dataOld = static_cast<void*>(nullptr);
                if (endPin->type != Pin::Type::Delegate)
                {
                    if (!endPin->notifyFuncOld.empty())
                    {
                        auto iter = std::find(startPin->notifyFuncOld.begin(), startPin->notifyFuncOld.end(),
                                              std::make_tuple(std::get<0>(endPin->notifyFuncOld.front()),
                                                              std::get<1>(endPin->notifyFuncOld.front()),
                                                              linkId));
                        if (iter != startPin->notifyFuncOld.end())
                        {
                            startPin->notifyFuncOld.erase(iter);
                        }
                    }
                }

                if (endPin->parentNode)
                {
                    endPin->parentNode->doDeinitialize(true);
                }
            }
            else if (startPin->type == Pin::Type::Flow)
            {
                auto iter = std::find(startPin->callbacksOld.begin(), startPin->callbacksOld.end(),
                                      std::make_tuple(endPin->parentNode,
                                                      std::get<void (NAV::Node::*)(const std::shared_ptr<const NAV::NodeData>&, ax::NodeEditor::LinkId)>(endPin->dataOld),
                                                      linkId));
                if (iter != startPin->callbacksOld.end())
                {
                    startPin->callbacksOld.erase(iter);
                }
                else
                {
                    LOG_ERROR("Tried to delete link {}, with type Flow, but could not find the callback.", linkId.AsPointer());
                }
            }
        }

        // Iterator 'id' can be invalidated if the input link caused also the ouput link to be deleted
        // Therefore the iterator needs to be searched again
        id = std::find_if(m_links.begin(),
                          m_links.end(),
                          [linkId](const auto& link) { return link.id == linkId; });
        if (id != m_links.end())
        {
            m_links.erase(id);
        }

        if (startPin && endPin)
        {
            if (startPin->parentNode)
            {
                startPin->parentNode->afterDeleteLink(*startPin, *endPin);
            }
            if (endPin->parentNode)
            {
                endPin->parentNode->afterDeleteLink(*startPin, *endPin);
            }
        }

        flow::ApplyChanges();

        return true;
    }

    return false;
}

void NAV::NodeManager::DeleteAllLinks()
{
    LOG_TRACE("called");

    bool saveLastActionsValue = NAV::flow::saveLastActions;
    NAV::flow::saveLastActions = false;
    while (!m_links.empty())
    {
        NodeManager::DeleteLink(m_links.back().id);
    }

    m_NextId = 1;

    for (const auto& node : m_nodes)
    {
        m_NextId = std::max(m_NextId, size_t(node->id) + 1);
    }

    NAV::flow::saveLastActions = saveLastActionsValue;
    flow::ApplyChanges();
}

void NAV::NodeManager::DeleteLinksOnPin(const NAV::OutputPin& pin)
{
    LOG_TRACE("called for pin ({})", size_t(pin.id));

    auto connectedLinks = FindConnectedLinksToOutputPin(pin);
    for (auto* connectedLink : connectedLinks)
    {
        NAV::NodeManager::DeleteLink(connectedLink->id);
    }
}

void NAV::NodeManager::DeleteLinksOnPin(const NAV::InputPin& pin)
{
    LOG_TRACE("called for pin ({})", size_t(pin.id));

    if (auto* connectedLink = FindConnectedLinkToInputPin(pin))
    {
        NAV::NodeManager::DeleteLink(connectedLink->id);
    }
}

NAV::Pin* NAV::NodeManager::CreateInputPin(NAV::Node* node, const char* name, NAV::Pin::Type pinType, const std::vector<std::string>& dataIdentifier, NAV::Pin::PinDataOld data, int idx)
{
    LOG_TRACE("called for pin ({}) of type ({}) for node [{}]", name, std::string(pinType), node->nameId());
    if (idx < 0)
    {
        idx = static_cast<int>(node->inputPins.size());
    }
    idx = std::min(idx, static_cast<int>(node->inputPins.size()));
    auto iter = std::next(node->inputPins.begin(), idx);

    node->inputPins.emplace(iter, GetNextPinId(), name, pinType, node);

    node->inputPins.at(static_cast<size_t>(idx)).dataOld = data;
    node->inputPins.at(static_cast<size_t>(idx)).dataIdentifier = dataIdentifier;

    flow::ApplyChanges();

    return &node->inputPins.back();
}

NAV::Pin* NAV::NodeManager::CreateOutputPin(NAV::Node* node, const char* name, NAV::Pin::Type pinType, const std::vector<std::string>& dataIdentifier, NAV::Pin::PinDataOld data, int idx)
{
    LOG_TRACE("called for pin ({}) of type ({}) for node [{}]", name, std::string(pinType), node->nameId());
    if (idx < 0)
    {
        idx = static_cast<int>(node->outputPins.size());
    }
    idx = std::min(idx, static_cast<int>(node->outputPins.size()));
    auto iter = std::next(node->outputPins.begin(), idx);

    node->outputPins.emplace(iter, GetNextPinId(), name, pinType, node);

    node->outputPins.at(static_cast<size_t>(idx)).dataOld = data;
    node->outputPins.at(static_cast<size_t>(idx)).dataIdentifier = dataIdentifier;

    flow::ApplyChanges();

    return &node->outputPins.back();
}

bool NAV::NodeManager::DeleteOutputPin(const NAV::OutputPin& pin)
{
    LOG_TRACE("called for pin ({})", size_t(pin.id));

    DeleteLinksOnPin(pin);

    size_t pinIndex = pin.parentNode->pinIndexFromId(pin.id);
    pin.parentNode->outputPins.erase(pin.parentNode->outputPins.begin() + static_cast<int64_t>(pinIndex));

    return true;
}

bool NAV::NodeManager::DeleteInputPin(const NAV::InputPin& pin)
{
    LOG_TRACE("called for pin ({})", size_t(pin.id));

    DeleteLinksOnPin(pin);

    size_t pinIndex = pin.parentNode->pinIndexFromId(pin.id);
    pin.parentNode->inputPins.erase(pin.parentNode->inputPins.begin() + static_cast<int64_t>(pinIndex));

    return true;
}

size_t NAV::NodeManager::GetNextId()
{
    return m_NextId++;
}

ax::NodeEditor::NodeId NAV::NodeManager::GetNextNodeId()
{
    return { GetNextId() };
}

ax::NodeEditor::LinkId NAV::NodeManager::GetNextLinkId()
{
    return { GetNextId() };
}

ax::NodeEditor::PinId NAV::NodeManager::GetNextPinId()
{
    return { GetNextId() };
}

NAV::Node* NAV::NodeManager::FindNode(ax::NodeEditor::NodeId id)
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

NAV::Link* NAV::NodeManager::FindLink(ax::NodeEditor::LinkId id)
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

NAV::OutputPin* NAV::NodeManager::FindOutputPin(ax::NodeEditor::PinId id)
{
    if (!id) { return nullptr; }

    for (auto& node : m_nodes)
    {
        for (auto& pin : node->outputPins)
        {
            if (pin.id == id) { return &pin; }
        }
    }

    return nullptr;
}

NAV::InputPin* NAV::NodeManager::FindInputPin(ax::NodeEditor::PinId id)
{
    if (!id) { return nullptr; }

    for (auto& node : m_nodes)
    {
        for (auto& pin : node->inputPins)
        {
            if (pin.id == id) { return &pin; }
        }
    }

    return nullptr;
}

bool NAV::NodeManager::IsPinLinked(const OutputPin& pin)
{
    for (const auto& link : m_links)
    {
        if (link.startPinId == pin.id) { return true; }
    }

    return false;
}

bool NAV::NodeManager::IsPinLinked(const InputPin& pin)
{
    for (const auto& link : m_links)
    {
        if (link.endPinId == pin.id) { return true; }
    }

    return false;
}

std::vector<NAV::Node*> NAV::NodeManager::FindConnectedNodesToOutputPin(const OutputPin& pin)
{
    std::vector<NAV::Node*> connectedNodes;
    for (const auto& link : m_links)
    {
        if (link.startPinId == pin.id)
        {
            if (auto* connectedPin = FindInputPin(link.endPinId))
            {
                if (connectedPin->parentNode)
                {
                    connectedNodes.push_back(connectedPin->parentNode);
                }
            }
        }
    }

    return connectedNodes;
}

NAV::Node* NAV::NodeManager::FindConnectedNodeToInputPin(const InputPin& pin)
{
    for (const auto& link : m_links)
    {
        if (link.endPinId == pin.id)
        {
            if (auto* connectedPin = FindOutputPin(link.startPinId))
            {
                return connectedPin->parentNode;
            }
        }
    }

    return nullptr;
}

std::vector<NAV::Link*> NAV::NodeManager::FindConnectedLinksToOutputPin(const OutputPin& pin)
{
    std::vector<NAV::Link*> connectedLinks;
    for (auto& link : m_links)
    {
        if (link.startPinId == pin.id)
        {
            connectedLinks.push_back(&link);
        }
    }

    return connectedLinks;
}

NAV::Link* NAV::NodeManager::FindConnectedLinkToInputPin(const InputPin& pin)
{
    for (auto& link : m_links)
    {
        if (link.endPinId == pin.id)
        {
            return &link;
        }
    }

    return nullptr;
}

std::vector<NAV::InputPin*> NAV::NodeManager::FindConnectedPinsToOutputPin(const OutputPin& pin)
{
    std::vector<NAV::InputPin*> connectedPins;
    for (auto& link : m_links)
    {
        if (link.startPinId == pin.id)
        {
            connectedPins.push_back(FindInputPin(link.endPinId));
        }
    }

    return connectedPins;
}

NAV::OutputPin* NAV::NodeManager::FindConnectedPinToInputPin(const InputPin& pin)
{
    for (auto& link : m_links)
    {
        if (link.endPinId == pin.id)
        {
            return FindOutputPin(link.startPinId);
        }
    }

    return nullptr;
}

void NAV::NodeManager::EnableAllCallbacks()
{
    LOG_TRACE("called");
    for (auto* node : m_nodes)
    {
        if (!node->isDisabled())
        {
            node->callbacksEnabled = true;
        }
    }
}

void NAV::NodeManager::DisableAllCallbacks()
{
    LOG_TRACE("called");
    for (auto* node : m_nodes)
    {
        node->callbacksEnabled = false;
    }
}

bool NAV::NodeManager::InitializeAllNodes()
{
    LOG_TRACE("called");
    bool nodeCouldNotInitialize = false;

    InitializeAllNodesAsync();

    for (auto* node : m_nodes)
    {
        if (node && !node->isDisabled() && !node->isInitialized())
        {
            if (!node->doInitialize(true))
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

    for (auto* node : m_nodes)
    {
        if (node && !node->isDisabled() && !node->isInitialized())
        {
            node->doInitialize();
        }
    }
}

#ifdef TESTING

std::vector<std::pair<ax::NodeEditor::PinId, void (*)(const std::shared_ptr<const NAV::NodeData>&)>> watcherPinList;
std::vector<std::pair<ax::NodeEditor::LinkId, void (*)(const std::shared_ptr<const NAV::NodeData>&)>> watcherLinkList;

void (*cleanupCallback)() = nullptr;

void NAV::NodeManager::RegisterWatcherCallbackToOutputPin(ax::NodeEditor::PinId id, void (*callback)(const std::shared_ptr<const NodeData>&))
{
    watcherPinList.emplace_back(id, callback);
}

void NAV::NodeManager::RegisterWatcherCallbackToLink(ax::NodeEditor::LinkId id, void (*callback)(const std::shared_ptr<const NodeData>&))
{
    watcherLinkList.emplace_back(id, callback);
}

void NAV::NodeManager::ApplyWatcherCallbacks()
{
    for (auto& [linkId, callback] : watcherLinkList)
    {
        if (Link* link = FindLink(linkId))
        {
            if (Pin* pin = FindPin(link->startPinId))
            {
                if (pin->kind == Pin::Kind::Output)
                {
                    LOG_DEBUG("Adding watcher callback on node '{}' on pin {}", pin->parentNode->nameId(), pin->parentNode->pinIndexFromId(pin->id));
                    pin->watcherCallbacksOld.emplace_back(callback, linkId);
                }
            }
        }
    }

    for (auto& [id, callback] : watcherPinList)
    {
        if (Pin* pin = FindPin(id))
        {
            if (pin->kind == Pin::Kind::Output)
            {
                LOG_DEBUG("Adding watcher callback on node '{}' on pin {}", pin->parentNode->nameId(), pin->parentNode->pinIndexFromId(pin->id));
                pin->watcherCallbacksOld.emplace_back(callback, 0);
            }
        }
    }
}

void NAV::NodeManager::RegisterCleanupCallback(void (*callback)())
{
    cleanupCallback = callback;
}
void NAV::NodeManager::CallCleanupCallback()
{
    if (cleanupCallback)
    {
        cleanupCallback();
    }
}

void NAV::NodeManager::ClearRegisteredCallbacks()
{
    watcherPinList.clear();
    watcherLinkList.clear();
    cleanupCallback = nullptr;
}

#endif