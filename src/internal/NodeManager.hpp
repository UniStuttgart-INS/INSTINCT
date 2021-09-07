/// @file NodeManager.cpp
/// @brief Manages all Nodes
/// @author T. Topp (thomas@topp.cc)
/// @date 2020-12-14

#pragma once

#include <imgui_node_editor.h>

#include "internal/Node/Node.hpp"
#include "internal/Node/Link.hpp"
#include "internal/Node/Pin.hpp"

#include "NodeData/NodeData.hpp"

#include <vector>

namespace NAV::NodeManager
{
/// Flag if invokeCallbacks triggers a GUI Flow event
extern bool showFlowWhenInvokingCallbacks;

/// Flag if notifyOutputValueChanged & notifyInputValueChanged triggers a GUI Flow event
extern bool showFlowWhenNotifyingValueChange;

/// @brief List of all registered Nodes
const std::vector<Node*>& m_Nodes();

/// @brief List of all registered Links
const std::vector<Link>& m_Links();

/// @brief Delete all links and nodes
void DeleteAllLinksAndNodes();

/// @brief Add the provided node object to the list of nodes
/// @param[in] node Node object to add to the list
void AddNode(Node* node);

/// @brief Update the provided node object
/// @param[in] node Node object to add to the list
void UpdateNode(Node* node);

/// @brief Delete the node provided by id
/// @param[in] linkId Unique Id of the Node to delete
/// @return True if delete was successful, false if NodeId does not exist
bool DeleteNode(ax::NodeEditor::NodeId nodeId);

/// @brief Delete all nodes
void DeleteAllNodes();

/// @brief Create a Link object
/// @param[in] startPin Start Pin of the link
/// @param[in] endPin End Pin of the link
/// @return Pointer to the created link object
Link* CreateLink(Pin* startPin, Pin* endPin);

/// @brief Add the provided link object to the list of links
/// @param[in] link Link object to add to the list
bool AddLink(const Link& link);

/// @brief Refresh the link and the connected nodes
/// @param[in] linkId Unique Id of the Link to refresh
void RefreshLink(ax::NodeEditor::LinkId linkId);

/// @brief Delete the link provided by id
/// @param[in] linkId Unique Id of the Link to delete
/// @return True if delete was successful, false if LinkId does not exist
bool DeleteLink(ax::NodeEditor::LinkId linkId);

/// @brief Delete all links
void DeleteAllLinks();

/// @brief Create an Input Pin object
/// @param[in] node Node to register the Pin for
/// @param[in] name Display name of the Pin
/// @param[in] pinType Type of the pin
/// @param[in] dataIdentifier Identifier of the data which is represented by the pin
/// @param[in] data Pointer to data which is represented by the pin
/// @return Pointer to the created pin
Pin* CreateInputPin(Node* node, const char* name, Pin::Type pinType, const std::vector<std::string>& dataIdentifier = {}, Pin::PinData data = static_cast<void*>(nullptr));

/// @brief Create an Input Pin object
/// @tparam T Node Class where the function is member of
/// @tparam std::enable_if_t<std::is_base_of_v<Node, T>> Makes sure template only exists for classes with base class 'Node'
/// @param[in] node Node to register the Pin for
/// @param[in] name Display name of the Pin
/// @param[in] pinType Type of the pin
/// @param[in] dataIdentifier Identifier of the data which is represented by the pin
/// @param[in] callback Callback to register with the pin
/// @return Pointer to the created pin
template<typename T,
         typename = std::enable_if_t<std::is_base_of_v<Node, T>>>
Pin* CreateInputPin(Node* node, const char* name, Pin::Type pinType, const std::vector<std::string>& dataIdentifier = {}, void (T::*callback)(const std::shared_ptr<NodeData>&, ax::NodeEditor::LinkId) = nullptr)
{
    assert(pinType == Pin::Type::Flow);

    return CreateInputPin(node, name, pinType, dataIdentifier, Pin::PinData(static_cast<void (Node::*)(const std::shared_ptr<NodeData>&, ax::NodeEditor::LinkId)>(callback)));
}

/// @brief Create an Input Pin object
/// @tparam T Node Class where the function is member of
/// @tparam std::enable_if_t<std::is_base_of_v<Node, T>> Makes sure template only exists for classes with base class 'Node'
/// @param[in] node Node to register the Pin for
/// @param[in] name Display name of the Pin
/// @param[in] pinType Type of the pin
/// @param[in] dataIdentifier Identifier of the data which is represented by the pin
/// @param[in] notifyFunc Function to call when the data is updated
/// @return Pointer to the created pin
template<typename T,
         typename = std::enable_if_t<std::is_base_of_v<Node, T>>>
Pin* CreateInputPin(Node* node, const char* name, Pin::Type pinType, const std::vector<std::string>& dataIdentifier, void (T::*notifyFunc)(ax::NodeEditor::LinkId))
{
    assert(pinType != Pin::Type::Flow && pinType != Pin::Type::Delegate);

    Pin* pin = CreateInputPin(node, name, pinType, dataIdentifier);

    if (pin)
    {
        pin->notifyFunc.emplace_back(node, static_cast<void (Node::*)(ax::NodeEditor::LinkId)>(notifyFunc), 0);
    }

    return pin;
}

/// @brief Create an Output Pin object
/// @param[in] node Node to register the Pin for
/// @param[in] name Display name of the Pin
/// @param[in] pinType Type of the pin
/// @param[in] dataIdentifier Identifier of the data which is represented by the pin
/// @param[in] data Pointer to data which is represented by the pin
/// @return Pointer to the created pin
Pin* CreateOutputPin(Node* node, const char* name, Pin::Type pinType, const std::vector<std::string>& dataIdentifier, Pin::PinData data = static_cast<void*>(nullptr));

/// @brief Create an Output Pin object for Flow Pins
/// @tparam T Class where the function is member of
/// @tparam std::enable_if_t<std::is_base_of_v<Node, T>> Makes sure template only exists for classes with base class 'Node'
/// @param[in] node Node to register the Pin for
/// @param[in] name Display name of the Pin
/// @param[in] pinType Type of the pin
/// @param[in] dataIdentifier Identifier of the data which is represented by the pin
/// @param[in] callback Callback to register with the pin
/// @return Pointer to the created pin
template<typename T,
         typename = std::enable_if_t<std::is_base_of_v<Node, T>>>
Pin* CreateOutputPin(Node* node, const char* name, Pin::Type pinType, const std::vector<std::string>& dataIdentifier, std::shared_ptr<NAV::NodeData> (T::*callback)(bool) = nullptr)
{
    assert(pinType == Pin::Type::Flow);

    return CreateOutputPin(node, name, pinType, dataIdentifier, Pin::PinData(static_cast<std::shared_ptr<NAV::NodeData> (Node::*)(bool)>(callback)));
}

/// @brief Finds the Node for the NodeId
/// @param[in] id Unique Id of the Node to search for
/// @return Pointer to the node or nullptr if the NodeId does not exist
Node* FindNode(ax::NodeEditor::NodeId id);

/// @brief Finds the Link for the LinkId
/// @param[in] id Unique Id of the Link to search for
/// @return Pointer to the link or nullptr if the LinkId does not exist
Link* FindLink(ax::NodeEditor::LinkId id);

/// @brief Finds the Pin for the PinId
/// @param[in] id Unique Id of the Pin to search for
/// @return Pointer to the pin or nullptr if the PinId does not exist
Pin* FindPin(ax::NodeEditor::PinId id);

/// @brief Checks if a link exists, connecting to the provided Pin
/// @param[in] id Unique Id of the Pin to search for
/// @return true if the Pin is linked, otherwise false
bool IsPinLinked(ax::NodeEditor::PinId id);

/// @brief Searches all nodes which are connected to the provided output pin
/// @param[in] id Unique Id of the output Pin to search for
/// @return List of Nodes which are connected to the output pin
std::vector<Node*> FindConnectedNodesToOutputPin(ax::NodeEditor::PinId id);

/// @brief Searches the node which is connected to the provided input pin
/// @param[in] id Unique Id of the input Pin to search for
/// @return Pointer to the node which is connected to the input pin
Node* FindConnectedNodeToInputPin(ax::NodeEditor::PinId id);

/// @brief Searches all link which are connected to the provided output pin
/// @param[in] id Unique Id of the output Pin to search for
/// @return List of Links which are connected to the pin
std::vector<Link*> FindConnectedLinksToOutputPin(ax::NodeEditor::PinId id);

/// @brief Searches the link which is connected to the provided input pin
/// @param[in] id Unique Id of the input Pin to search for
/// @return Pointer to the link which is connected to the input pin
Link* FindConnectedLinkToInputPin(ax::NodeEditor::PinId id);

/// @brief Searches all pins which are connected to the provided output pin
/// @param[in] id Unique Id of the output Pin to search for
/// @return List of Pins which are connected to the pin
std::vector<Pin*> FindConnectedPinsToOutputPin(ax::NodeEditor::PinId id);

/// @brief Searches the pin which is connected to the provided input pin
/// @param[in] id Unique Id of the input Pin to search for
/// @return Pointer to the output pin which is connected to the input pin
Pin* FindConnectedPinToInputPin(ax::NodeEditor::PinId id);

/// @brief Enables all Node callbacks
void EnableAllCallbacks();

/// @brief Disables all Node callbacks
void DisableAllCallbacks();

/// @brief Initializes all nodes.
/// @return Returns false if one of the nodes could not initialize
bool InitializeAllNodes();

/// @brief Initializes all nodes in a separate thread
void InitializeAllNodesAsync();

/// @brief Stops all active threads
void Stop();

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
void RegisterWatcherCallbackToOutputPin(ax::NodeEditor::PinId id, void (*callback)(const std::shared_ptr<NodeData>&));

/// @brief Registers the callback function to the watcher list
/// @param[in] id Link id to add the callback to
/// @param[in] callback Callback function
/// @attention ApplyWatcherCallbacks() needs to be called after loading the flow to apply the list to the pins.
void RegisterWatcherCallbackToLink(ax::NodeEditor::LinkId id, void (*callback)(const std::shared_ptr<NodeData>&));

/// @brief Applies the watcher lists to the node pins
void ApplyWatcherCallbacks();

/// @brief Clears the watcher list
void ClearRegisteredCallbacks();

#endif

} // namespace NAV::NodeManager
