/// @file NodeManager.cpp
/// @brief Manages all Nodes
/// @author T. Topp (thomas@topp.cc)
/// @date 2020-12-14

#pragma once

#include <imgui_node_editor.h>

#include "Node.hpp"
#include "Link.hpp"
#include "Pin.hpp"

#include <vector>

namespace NAV::NodeManager
{
const std::vector<Node*>& m_Nodes();

const std::vector<Link>& m_Links();

void AddNode(Node* node);

bool DeleteNode(ax::NodeEditor::NodeId nodeId);

void DeleteAllNodes();

Link* CreateLink(Pin* startPin, Pin* endPin);

void AddLink(const Link& link);

bool DeleteLink(ax::NodeEditor::LinkId linkId);

void DeleteAllLinks();

Pin* CreateInputPin(Node* node, const char* name, Pin::Type pinType, const std::string_view& dataIdentifier = std::string_view(""));

Pin* CreateOutputPin(Node* node, const char* name, Pin::Type pinType, Pin::PinData data, const std::string_view& dataIdentifier = std::string_view(""));

Node* FindNode(ax::NodeEditor::NodeId id);

Link* FindLink(ax::NodeEditor::LinkId id);

Pin* FindPin(ax::NodeEditor::PinId id);

bool IsPinLinked(ax::NodeEditor::PinId id);

}; // namespace NAV::NodeManager
