// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "internal/NodeManager.hpp"

#include "internal/Node/Node.hpp"
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

    NAV::flow::saveLastActions = saveLastActionsValue;
    flow::ApplyChanges();
}

void NAV::NodeManager::AddLink(ax::NodeEditor::LinkId linkId)
{
    m_NextId = std::max(m_NextId, size_t(linkId) + 1);
}

NAV::InputPin* NAV::NodeManager::CreateInputPin(NAV::Node* node, const char* name, NAV::Pin::Type pinType, const std::vector<std::string>& dataIdentifier,
                                                InputPin::Callback callback, InputPin::FlowFirableCheckFunc firable, int priority, int idx)
{
    LOG_TRACE("called for pin ({}) of type ({}) for node [{}]", name, std::string(pinType), node->nameId());
    if (idx < 0)
    {
        idx = static_cast<int>(node->inputPins.size());
    }
    idx = std::min(idx, static_cast<int>(node->inputPins.size()));
    auto iter = std::next(node->inputPins.begin(), idx);

    node->inputPins.emplace(iter, GetNextPinId(), name, pinType, node);

    node->inputPins.at(static_cast<size_t>(idx)).callback = callback;
    if (firable != nullptr)
    {
        node->inputPins.at(static_cast<size_t>(idx)).firable = firable;
    }
    node->inputPins.at(static_cast<size_t>(idx)).dataIdentifier = dataIdentifier;
    node->inputPins.at(static_cast<size_t>(idx)).priority = priority;

    flow::ApplyChanges();

    return &node->inputPins.back();
}

NAV::OutputPin* NAV::NodeManager::CreateOutputPin(NAV::Node* node, const char* name, NAV::Pin::Type pinType, const std::vector<std::string>& dataIdentifier, OutputPin::PinData data, int idx)
{
    LOG_TRACE("called for pin ({}) of type ({}) for node [{}]", name, std::string(pinType), node->nameId());
    if (idx < 0)
    {
        idx = static_cast<int>(node->outputPins.size());
    }
    idx = std::min(idx, static_cast<int>(node->outputPins.size()));
    auto iter = std::next(node->outputPins.begin(), idx);

    node->outputPins.emplace(iter, GetNextPinId(), name, pinType, node);

    node->outputPins.at(static_cast<size_t>(idx)).data = data;
    node->outputPins.at(static_cast<size_t>(idx)).dataIdentifier = dataIdentifier;

    flow::ApplyChanges();

    return &node->outputPins.back();
}

bool NAV::NodeManager::DeleteOutputPin(NAV::OutputPin& pin)
{
    LOG_TRACE("called for pin ({})", size_t(pin.id));

    pin.deleteLinks();

    size_t pinIndex = pin.parentNode->outputPinIndexFromId(pin.id);
    pin.parentNode->outputPins.erase(pin.parentNode->outputPins.begin() + static_cast<int64_t>(pinIndex));

    return true;
}

bool NAV::NodeManager::DeleteInputPin(NAV::InputPin& pin)
{
    LOG_TRACE("called for pin ({})", size_t(pin.id));

    pin.deleteLink();

    size_t pinIndex = pin.parentNode->inputPinIndexFromId(pin.id);
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
                LOG_ERROR("Node '{}' could not initialize.", node->nameId());
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

std::vector<std::pair<ax::NodeEditor::PinId, NAV::InputPin::WatcherCallback>> watcherPinList;
std::vector<std::pair<ax::NodeEditor::LinkId, NAV::InputPin::WatcherCallback>> watcherLinkList;

std::function<void()> preInitCallback = nullptr;
std::function<void()> cleanupCallback = nullptr;

void NAV::NodeManager::RegisterWatcherCallbackToInputPin(ax::NodeEditor::PinId id, const InputPin::WatcherCallback& callback)
{
    watcherPinList.emplace_back(id, callback);
}

void NAV::NodeManager::RegisterWatcherCallbackToLink(ax::NodeEditor::LinkId id, const InputPin::WatcherCallback& callback)
{
    watcherLinkList.emplace_back(id, callback);
}

void NAV::NodeManager::ApplyWatcherCallbacks()
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

void NAV::NodeManager::RegisterPreInitCallback(std::function<void()> callback)
{
    preInitCallback = std::move(callback);
}

void NAV::NodeManager::CallPreInitCallback()
{
    if (preInitCallback)
    {
        preInitCallback();
    }
}

void NAV::NodeManager::RegisterCleanupCallback(std::function<void()> callback)
{
    cleanupCallback = std::move(callback);
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
    preInitCallback = nullptr;
    cleanupCallback = nullptr;
}

#endif