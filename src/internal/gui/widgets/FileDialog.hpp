/// @file FileDialog.hpp
/// @brief File Chooser
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-01-18

#pragma once

#include <vector>
#include <string>
#include <filesystem>

namespace NAV::gui::widgets
{
/// @brief Shows an InputText and a button which opens a file dialog to select a path to save a file to
/// @param[in, out] path String to store the path in
/// @param[in] vName Window title
/// @param[in] vFilters Filter to apply for file names
/// @param[in] extensions Extensions to filter
/// @param[in] startPath Path to the directory to display initially
/// @param[in] id Unique id for creating the dialog uid
/// @param[in] nameId Name of the node triggering the window used for logging
/// @return True if the filepath was changed
bool FileDialogSave(std::string& path, const char* vName,
                    const char* vFilters, const std::vector<std::string>& extensions,
                    std::filesystem::path startPath,
                    size_t id, const std::string& nameId);

/// @brief Shows an InputText and a button which opens a file dialog to select a path to load a file to
/// @param[in, out] path String to store the path in
/// @param[in] vName Window title
/// @param[in] vFilters Filter to apply for file names
/// @param[in] extensions Extensions to filter
/// @param[in] startPath Path to the directory to display initially
/// @param[in] id Unique id for creating the dialog uid
/// @param[in] nameId Name of the node triggering the window used for logging
/// @return True if the filepath was changed
bool FileDialogLoad(std::string& path, const char* vName,
                    const char* vFilters, const std::vector<std::string>& extensions,
                    std::filesystem::path startPath,
                    size_t id, const std::string& nameId);

} // namespace NAV::gui::widgets
