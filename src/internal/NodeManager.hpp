/// @file NodeManager.hpp
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
/// @param[in] nodeId Unique Id of the Node to delete
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

/// @brief Delete all links on the provided pin
/// @param[in] pinId Id of the pin
void DeleteLinksOnPin(ax::NodeEditor::PinId pinId);

/// @brief Create an Input Pin object
/// @param[in] node Node to register the Pin for
/// @param[in] name Display name of the Pin
/// @param[in] pinType Type of the pin
/// @param[in] dataIdentifier Identifier of the data which is represented by the pin
/// @param[in] data Pointer to data which is represented by the pin
/// @param[in] idx Index where to put the new pin (-1 means at the end)
/// @return Pointer to the created pin
Pin* CreateInputPin(Node* node, const char* name, Pin::Type pinType, const std::vector<std::string>& dataIdentifier = {}, Pin::PinDataOld data = static_cast<void*>(nullptr), int idx = -1);

/// @brief Create an Input Pin object
/// @tparam T Node Class where the function is member of
/// @param[in] node Node to register the Pin for
/// @param[in] name Display name of the Pin
/// @param[in] pinType Type of the pin
/// @param[in] dataIdentifier Identifier of the data which is represented by the pin
/// @param[in] callback Callback to register with the pin
/// @param[in] idx Index where to put the new pin (-1 means at the end)
/// @return Pointer to the created pin
template<typename T,
         typename = std::enable_if_t<std::is_base_of_v<Node, T>>>
Pin* CreateInputPin(Node* node, const char* name, Pin::Type pinType, const std::vector<std::string>& dataIdentifier = {}, void (T::*callback)(const std::shared_ptr<const NodeData>&, ax::NodeEditor::LinkId) = nullptr, int idx = -1)
{
    assert(pinType == Pin::Type::Flow);

    return CreateInputPin(node, name, pinType, dataIdentifier, Pin::PinDataOld(static_cast<void (Node::*)(const std::shared_ptr<const NodeData>&, ax::NodeEditor::LinkId)>(callback)), idx);
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
template<typename T,
         typename = std::enable_if_t<std::is_base_of_v<Node, T>>>
Pin* CreateInputPin(Node* node, const char* name, Pin::Type pinType, const std::vector<std::string>& dataIdentifier, void (T::*notifyFunc)(ax::NodeEditor::LinkId), int idx = -1)
{
    assert(pinType != Pin::Type::Flow && pinType != Pin::Type::Delegate);

    Pin* pin = CreateInputPin(node, name, pinType, dataIdentifier, static_cast<void*>(nullptr), idx);

    if (pin)
    {
        pin->notifyFuncOld.emplace_back(node, static_cast<void (Node::*)(ax::NodeEditor::LinkId)>(notifyFunc), 0);
    }

    return pin;
}

/// @brief Create an Output Pin object
/// @param[in] node Node to register the Pin for
/// @param[in] name Display name of the Pin
/// @param[in] pinType Type of the pin
/// @param[in] dataIdentifier Identifier of the data which is represented by the pin
/// @param[in] data Pointer to data which is represented by the pin
/// @param[in] idx Index where to put the new pin (-1 means at the end)
/// @return Pointer to the created pin
Pin* CreateOutputPin(Node* node, const char* name, Pin::Type pinType, const std::vector<std::string>& dataIdentifier, Pin::PinDataOld data = static_cast<void*>(nullptr), int idx = -1);

/// @brief Create an Output Pin object for Flow Pins
/// @tparam T Class where the function is member of
/// @param[in] node Node to register the Pin for
/// @param[in] name Display name of the Pin
/// @param[in] pinType Type of the pin
/// @param[in] dataIdentifier Identifier of the data which is represented by the pin
/// @param[in] callback Callback to register with the pin
/// @param[in] idx Index where to put the new pin (-1 means at the end)
/// @return Pointer to the created pin
template<typename T,
         typename = std::enable_if_t<std::is_base_of_v<Node, T>>>
Pin* CreateOutputPin(Node* node, const char* name, Pin::Type pinType, const std::vector<std::string>& dataIdentifier, std::shared_ptr<const NAV::NodeData> (T::*callback)(bool) = nullptr, int idx = -1)
{
    assert(pinType == Pin::Type::Flow);

    return CreateOutputPin(node, name, pinType, dataIdentifier, Pin::PinDataOld(static_cast<std::shared_ptr<const NAV::NodeData> (Node::*)(bool)>(callback)), idx);
}

/// @brief Deletes the output pin
/// @param[in] id Unique Id of the Pin to delete
/// @return True if the pin was delete
bool DeleteOutputPin(ax::NodeEditor::PinId id);

/// @brief Deletes the input pin
/// @param[in] id Unique Id of the Pin to delete
/// @return True if the pin was delete
bool DeleteInputPin(ax::NodeEditor::PinId id);

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
void RegisterWatcherCallbackToOutputPin(ax::NodeEditor::PinId id, void (*callback)(const std::shared_ptr<const NodeData>&));

/// @brief Registers the callback function to the watcher list
/// @param[in] id Link id to add the callback to
/// @param[in] callback Callback function
/// @attention ApplyWatcherCallbacks() needs to be called after loading the flow to apply the list to the pins.
void RegisterWatcherCallbackToLink(ax::NodeEditor::LinkId id, void (*callback)(const std::shared_ptr<const NodeData>&));

/// @brief Applies the watcher lists to the node pins
void ApplyWatcherCallbacks();

/// @brief Registers a callback which gets called after flow execution before cleanup
/// @param[in] callback Callback function
void RegisterCleanupCallback(void (*callback)());

/// @brief Calls the cleanup callback
void CallCleanupCallback();

/// @brief Clears the watcher list
void ClearRegisteredCallbacks();

#endif

} // namespace NAV::NodeManager
