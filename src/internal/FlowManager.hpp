// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file FlowManager.hpp
/// @brief Save/Load the Nodes
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-12-16

#pragma once

#include <string>
#include <filesystem>
#include "internal/gui/GlobalActions.hpp"
#include "internal/Node/Node.hpp"
#include "NodeData/NodeData.hpp"

#include <nlohmann/json.hpp>
using json = nlohmann::json; ///< json namespace

namespace NAV::flow
{
/// @brief List of all registered Nodes
const std::vector<Node*>& m_Nodes();

/// @brief Generates a new node id
ax::NodeEditor::NodeId GetNextNodeId();

/// @brief Generates a new link id
ax::NodeEditor::LinkId GetNextLinkId();

/// @brief Generates a new pin id
ax::NodeEditor::PinId GetNextPinId();

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

/// @brief Saves the current flow into a file
/// @param[out] globalAction If currentfilename is empty this will be returned as GlobalActions::SaveAs
void SaveFlow(GlobalActions& globalAction);

/// @brief Saves the current flow into the specified file
/// @param[in] filepath Path where to save the flow
void SaveFlowAs(const std::string& filepath);

/// @brief Loads the flow from the specified file
/// @param[in] filepath Path where to load the flow
/// @return Whether the load was successfull
bool LoadFlow(const std::string& filepath);

/// @brief Loads the nodes and links from the specified json object
/// @param[in] j Json object containing nodes and links to load
/// @param[in] requestNewIds Set this true if the loaded nodes should receive new Ids (copy). False if the Ids should stay (cut/load)
/// @return Whether the load was successfull
bool LoadJson(const json& j, bool requestNewIds = false);

/// @brief Checks if the currently open flow has unsaved changes
/// @return True if there are changes
bool HasUnsavedChanges();

/// @brief Signals that there have been changes to the flow
void ApplyChanges();

/// @brief Discards the unsaved changes flag. Does not really discard the changes.
void DiscardChanges();

/// @brief Get the current filename of the open flow
/// @return Current filename of the open flow
std::string GetCurrentFilename();

/// @brief Set the current filename of the open flow
/// @param[in] newFilename New filename of the flow
void SetCurrentFilename(const std::string& newFilename);

/// @brief Get the program root path
/// @return The path to the program root
std::filesystem::path GetProgramRootPath();

/// @brief Set the program root path
/// @param[in] newRootPath The new program root path
void SetProgramRootPath(const std::filesystem::path& newRootPath);

/// @brief Get the path where logs and outputs are stored
std::filesystem::path GetOutputPath();

/// @brief Set the path where logs and outputs are stored
void SetOutputPath();

/// @brief Get the path where data files are searched
std::filesystem::path GetInputPath();

/// @brief Get the path where flow files are searched
std::filesystem::path GetFlowPath();

/// @brief Get the path where config files are searched
std::filesystem::path GetConfigPath();

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

} // namespace NAV::flow
