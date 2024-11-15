// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file NodeManager.hpp
/// @brief Manages all Nodes
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-12-14

#pragma once

#include <imgui_node_editor.h>

#include "internal/Node/Node.hpp"
#include "internal/Node/Pin.hpp"
#include "util/Assert.h"

#include "NodeData/NodeData.hpp"

#include <vector>
#include <functional>

namespace NAV::NodeManager
{
/// Flag if invokeCallbacks triggers a GUI Flow event
extern bool showFlowWhenInvokingCallbacks;

/// Flag if notifyOutputValueChanged & notifyInputValueChanged triggers a GUI Flow event
extern bool showFlowWhenNotifyingValueChange;

/// @brief List of all registered Nodes
const std::vector<Node*>& m_Nodes();

/// @brief Add the provided node object to the list of nodes
/// @param[in] node Node object to add to the list
void AddNode(Node* node);

/// @brief Update the provided node object
/// @param[in] node Node object to add to the list
void UpdateNode(Node* node);

/// @brief Delete the node provided by id
/// @param[in] nodeId Unique Id of the Node to delete
/// @return True if delete was successful, false if NodeId does not exist
bool DeleteNode(ax::NodeEditor::NodeId nodeId);

/// @brief Delete all nodes
void DeleteAllNodes();

/// @brief Adds the link
/// @param[in] linkId Unique Id of the link
void AddLink(ax::NodeEditor::LinkId linkId);

/// @brief Create an Input Pin object
/// @param[in] node Node to register the Pin for
/// @param[in] name Display name of the Pin
/// @param[in] pinType Type of the pin
/// @param[in] dataIdentifier Identifier of the data which is represented by the pin
/// @param[in] callback Callback to register with the pin
/// @param[in] firable Function to check whether the callback is firable
/// @param[in] priority Priority when checking firable condition related to other pins (higher priority gets triggered first)
/// @param[in] idx Index where to put the new pin (-1 means at the end)
/// @return Pointer to the created pin
InputPin* CreateInputPin(Node* node, const char* name, Pin::Type pinType, const std::vector<std::string>& dataIdentifier = {},
                         InputPin::Callback callback = static_cast<InputPin::FlowFirableCallbackFunc>(nullptr),
                         InputPin::FlowFirableCheckFunc firable = nullptr,
                         int priority = 0, int idx = -1);

/// @brief Create an Input Pin object
/// @tparam T Node Class where the function is member of
/// @param[in] node Node to register the Pin for
/// @param[in] name Display name of the Pin
/// @param[in] pinType Type of the pin
/// @param[in] dataIdentifier Identifier of the data which is represented by the pin
/// @param[in] callback Flow firable callback function to register with the pin
/// @param[in] firable Function to check whether the callback is firable
/// @param[in] priority Priority when checking firable condition related to other pins (higher priority gets triggered first)
/// @param[in] idx Index where to put the new pin (-1 means at the end)
/// @return Pointer to the created pin
template<std::derived_from<Node> T>
InputPin* CreateInputPin(Node* node, const char* name, Pin::Type pinType, const std::vector<std::string>& dataIdentifier = {},
                         void (T::*callback)(InputPin::NodeDataQueue&, size_t) = nullptr,
                         InputPin::FlowFirableCheckFunc firable = nullptr,
                         int priority = 0, int idx = -1)
{
    assert(pinType == Pin::Type::Flow);

    return CreateInputPin(node, name, pinType, dataIdentifier, InputPin::Callback(static_cast<InputPin::FlowFirableCallbackFunc>(callback)), firable, priority, idx);
}

/// @brief Create an Input Pin object
/// @tparam T Node Class where the function is member of
/// @param[in] node Node to register the Pin for
/// @param[in] name Display name of the Pin
/// @param[in] pinType Type of the pin
/// @param[in] dataIdentifier Identifier of the data which is represented by the pin
/// @param[in] notifyFunc Function to call when the data is updated
/// @param[in] idx Index where to put the new pin (-1 means at the end)
/// @return Pointer to the created pin
template<std::derived_from<Node> T>
InputPin* CreateInputPin(Node* node, const char* name, Pin::Type pinType, const std::vector<std::string>& dataIdentifier,
                         void (T::*notifyFunc)(const InsTime&, size_t), int idx = -1)
{
    assert(pinType != Pin::Type::Flow && pinType != Pin::Type::Delegate);

    return CreateInputPin(node, name, pinType, dataIdentifier, InputPin::Callback(static_cast<InputPin::DataChangedNotifyFunc>(notifyFunc)), nullptr, 0, idx);
}

/// @brief Create an Output Pin object
/// @param[in] node Node to register the Pin for
/// @param[in] name Display name of the Pin
/// @param[in] pinType Type of the pin
/// @param[in] dataIdentifier Identifier of the data which is represented by the pin
/// @param[in] data Pointer to data which is represented by the pin
/// @param[in] idx Index where to put the new pin (-1 means at the end)
/// @return Pointer to the created pin
OutputPin* CreateOutputPin(Node* node, const char* name, Pin::Type pinType, const std::vector<std::string>& dataIdentifier, OutputPin::PinData data = static_cast<void*>(nullptr), int idx = -1);

/// @brief Create an Output Pin object for Flow Pins
/// @tparam T Class where the function is member of
/// @param[in] node Node to register the Pin for
/// @param[in] name Display name of the Pin
/// @param[in] pinType Type of the pin
/// @param[in] dataIdentifier Identifier of the data which is represented by the pin
/// @param[in] peekPollDataFunc Function to poll for data on this pin
/// @param[in] idx Index where to put the new pin (-1 means at the end)
/// @return Pointer to the created pin
template<std::derived_from<Node> T>
OutputPin* CreateOutputPin(Node* node, const char* name, Pin::Type pinType, const std::vector<std::string>& dataIdentifier,
                           std::shared_ptr<const NAV::NodeData> (T::*peekPollDataFunc)(size_t, bool) = nullptr, int idx = -1)
{
    assert(pinType == Pin::Type::Flow);
    INS_ASSERT_USER_ERROR(std::none_of(node->outputPins.begin(), node->outputPins.end(),
                                       [](const OutputPin& outputPin) { return std::holds_alternative<OutputPin::PollDataFunc>(outputPin.data); }),
                          "You cannot mix PollDataFunc and PeekPollDataFunc output pins. Use only PeekPollDataFunc pins if multiple pins are needed.");

    return CreateOutputPin(node, name, pinType, dataIdentifier, OutputPin::PinData(static_cast<OutputPin::PeekPollDataFunc>(peekPollDataFunc)), idx);
}

/// @brief Create an Output Pin object for Flow Pins
/// @tparam T Class where the function is member of
/// @param[in] node Node to register the Pin for
/// @param[in] name Display name of the Pin
/// @param[in] pinType Type of the pin
/// @param[in] dataIdentifier Identifier of the data which is represented by the pin
/// @param[in] pollDataFunc Function to poll for data on this pin
/// @param[in] idx Index where to put the new pin (-1 means at the end)
/// @return Pointer to the created pin
template<std::derived_from<Node> T>
OutputPin* CreateOutputPin(Node* node, const char* name, Pin::Type pinType, const std::vector<std::string>& dataIdentifier,
                           std::shared_ptr<const NAV::NodeData> (T::*pollDataFunc)() = nullptr, int idx = -1)
{
    assert(pinType == Pin::Type::Flow);
    INS_ASSERT_USER_ERROR(std::none_of(node->outputPins.begin(), node->outputPins.end(),
                                       [](const OutputPin& outputPin) { return std::holds_alternative<OutputPin::PeekPollDataFunc>(outputPin.data)
                                                                               || std::holds_alternative<OutputPin::PollDataFunc>(outputPin.data); }),
                          "There can only be one poll pin if the poll only data function is chosen. If multiple are needed, create PeekPollDataFunc pins.");

    return CreateOutputPin(node, name, pinType, dataIdentifier, OutputPin::PinData(static_cast<OutputPin::PollDataFunc>(pollDataFunc)), idx);
}

/// @brief Deletes the output pin. Invalidates the pin reference given.
/// @param[in, out] pin Output Pin to delete
/// @return True if the pin was delete
bool DeleteOutputPin(OutputPin& pin);

/// @brief Deletes the input pin. Invalidates the pin reference given.
/// @param[in, out] pin Input Pin to delete
/// @return True if the pin was delete
bool DeleteInputPin(InputPin& pin);

/// @brief Finds the Node for the NodeId
/// @param[in] id Unique Id of the Node to search for
/// @return Pointer to the node or nullptr if the NodeId does not exist
Node* FindNode(ax::NodeEditor::NodeId id);

/// @brief Finds the Pin for the PinId
/// @param[in] id Unique Id of the Pin to search for
/// @return Pointer to the pin or nullptr if the PinId does not exist
OutputPin* FindOutputPin(ax::NodeEditor::PinId id);

/// @brief Finds the Pin for the PinId
/// @param[in] id Unique Id of the Pin to search for
/// @return Pointer to the pin or nullptr if the PinId does not exist
InputPin* FindInputPin(ax::NodeEditor::PinId id);

/// @brief Enables all Node callbacks
void EnableAllCallbacks();

/// @brief Disables all Node callbacks
void DisableAllCallbacks();

/// @brief Clears all nodes queues
void ClearAllNodeQueues();

/// @brief Initializes all nodes.
/// @return Returns false if one of the nodes could not initialize
bool InitializeAllNodes();

/// @brief Initializes all nodes in a separate thread
void InitializeAllNodesAsync();

/// @brief Generates a new node id
ax::NodeEditor::NodeId GetNextNodeId();

/// @brief Generates a new link id
ax::NodeEditor::LinkId GetNextLinkId();

/// @brief Generates a new pin id
ax::NodeEditor::PinId GetNextPinId();

#ifdef TESTING

/// @brief Registers the callback function to the watcher list
/// @param[in] id Output pin id to add the callback to
/// @param[in] callback Callback function
/// @attention ApplyWatcherCallbacks() needs to be called after loading the flow to apply the list to the pins.
void RegisterWatcherCallbackToInputPin(ax::NodeEditor::PinId id, const InputPin::WatcherCallback& callback);

/// @brief Registers the callback function to the watcher list
/// @param[in] id Link id to add the callback to
/// @param[in] callback Callback function
/// @attention ApplyWatcherCallbacks() needs to be called after loading the flow to apply the list to the pins.
void RegisterWatcherCallbackToLink(ax::NodeEditor::LinkId id, const InputPin::WatcherCallback& callback);

/// @brief Applies the watcher lists to the node pins
void ApplyWatcherCallbacks();

/// @brief Registers a callback function which gets called before the nodes are initialized. Used to change node settings.
/// @param[in] callback Callback function
void RegisterPreInitCallback(std::function<void()> callback);

/// @brief Calls the pre-init callback
void CallPreInitCallback();

/// @brief Registers a callback which gets called after flow execution before cleanup
/// @param[in] callback Callback function
void RegisterCleanupCallback(std::function<void()> callback);

/// @brief Calls the cleanup callback
void CallCleanupCallback();

/// @brief Clears the watcher list
void ClearRegisteredCallbacks();

#endif

} // namespace NAV::NodeManager
