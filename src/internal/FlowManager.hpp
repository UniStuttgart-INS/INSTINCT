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

#include <nlohmann/json.hpp>
using json = nlohmann::json; ///< json namespace

namespace NAV::flow
{
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

/// @brief Whether actions should be saved to the last actions list
extern bool saveLastActions;

/// @brief Frame Count when changes were loaded to prevent nodes moving from triggering unsaved changes
extern int loadingFrameCount;

} // namespace NAV::flow
