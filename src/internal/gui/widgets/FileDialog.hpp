/// @file FileDialog.hpp
/// @brief File Chooser
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-01-18

#pragma once

#include <vector>
#include <string>

namespace NAV::gui::widgets
{
bool FileDialogSave(std::string& path, const char* vName,
                    const char* vFilters, const std::vector<std::string>& extensions,
                    size_t id, const std::string& nameId);

bool FileDialogLoad(std::string& path, const char* vName,
                    const char* vFilters, const std::vector<std::string>& extensions,
                    size_t id, const std::string& nameId);

} // namespace NAV::gui::widgets
