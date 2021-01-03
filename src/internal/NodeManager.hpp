/// @file NodeManager.cpp
/// @brief Manages all Nodes
/// @author T. Topp (thomas@topp.cc)
/// @date 2020-12-14

#pragma once

#include <imgui_node_editor.h>

#include "Nodes/Node.hpp"
#include "internal/Link.hpp"
#include "internal/Pin.hpp"

#include "NodeData/NodeData.hpp"

#include <vector>

namespace NAV::NodeManager
{
/// Flag if invokeCallbacks triggers a GUI Flow event
extern bool showFlowWhenInvokingCallbacks;

/// @brief List of all registered Nodes
const std::vector<Node*>& m_Nodes();

/// @brief List of all registered Links
const std::vector<Link>& m_Links();

/// @brief Add the provided node object to the list of nodes
/// @param[in] node Node object to add to the list
void AddNode(Node* node);

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
void AddLink(const Link& link);

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
Pin* CreateInputPin(Node* node, const char* name, Pin::Type pinType, const std::string_view& dataIdentifier = std::string_view(""), Pin::PinData data = static_cast<void*>(nullptr));

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
Pin* CreateInputPin(Node* node, const char* name, Pin::Type pinType, const std::string_view& dataIdentifier = std::string_view(""), void (T::*callback)(std::shared_ptr<NodeData>) = nullptr)
{
    assert(pinType == Pin::Type::Flow);

    return CreateInputPin(node, name, pinType, dataIdentifier, Pin::PinData(static_cast<void (Node::*)(std::shared_ptr<NodeData>)>(callback)));
}

/// @brief Create an Output Pin object
/// @param[in] node Node to register the Pin for
/// @param[in] name Display name of the Pin
/// @param[in] pinType Type of the pin
/// @param[in] dataIdentifier Identifier of the data which is represented by the pin
/// @param[in] data Pointer to data which is represented by the pin
/// @return Pointer to the created pin
Pin* CreateOutputPin(Node* node, const char* name, Pin::Type pinType, const std::string_view& dataIdentifier = std::string_view(""), Pin::PinData data = static_cast<void*>(nullptr));

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
Pin* CreateOutputPin(Node* node, const char* name, Pin::Type pinType, const std::string_view& dataIdentifier = std::string_view(""), std::shared_ptr<NAV::NodeData> (T::*callback)(bool) = nullptr)
{
    assert(pinType == Pin::Type::Flow);

    return CreateOutputPin(node, name, pinType, dataIdentifier, Pin::PinData(static_cast<std::shared_ptr<NAV::NodeData> (Node::*)(bool)>(callback)));
}

/// @brief Create an Output Pin object for Function Pins
/// @tparam U
/// @tparam P
/// @tparam T
/// @tparam T,
/// typename
/// @tparam T>>
/// @param[in, out] node
/// @param[in, out] name
/// @param[in, out] pinType
/// @param[in, out] dataIdentifier
/// @param[in, out] callback
/// @return
template<typename U, typename... P, typename T,
         typename = std::enable_if_t<std::is_base_of_v<Node, T>>>
Pin* CreateOutputPin(Node* node, const char* name, Pin::Type pinType, const std::string_view& dataIdentifier = std::string_view(""), U (T::*callback)(P...) = nullptr)
{
    assert(pinType == Pin::Type::Function);

    return CreateOutputPin(node, name, pinType, dataIdentifier, Pin::PinData(std::make_pair(node, reinterpret_cast<void (Node::*)()>(callback))));
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

/// @brief Searches all nodes which are connected to the provided pin
/// @param[in] id Unique Id of the Pin to search for
/// @return List of Nodes which are connected to the pin
std::vector<Node*> FindConnectedNodesToPin(ax::NodeEditor::PinId id);

/// @brief Searches all link which are connected to the provided pin
/// @param[in] id Unique Id of the Pin to search for
/// @return List of Links which are connected to the pin
std::vector<Link*> FindConnectedLinksToPin(ax::NodeEditor::PinId id);

/// @brief Enables all Node callbacks
void EnableAllCallbacks();

/// @brief Disables all Node callbacks
void DisableAllCallbacks();

}; // namespace NAV::NodeManager
