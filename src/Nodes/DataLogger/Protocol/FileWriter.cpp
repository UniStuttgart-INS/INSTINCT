// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "FileWriter.hpp"

#include "util/Logger.hpp"

#include "internal/FlowManager.hpp"
#include "internal/ConfigManager.hpp"

#include <imgui.h>
#include "internal/gui/widgets/FileDialog.hpp"
#include "internal/gui/widgets/HelpMarker.hpp"

bool NAV::FileWriter::guiConfig(const char* vFilters, const std::vector<std::string>& extensions, size_t id, const std::string& nameId)
{
    bool changesOccurred = false;

    if (gui::widgets::FileDialogSave(_path, "Save File", vFilters, extensions,
                                     flow::GetProgramRootPath() / ConfigManager::Get<std::string>("output-path"), id, nameId))
    {
        if (_path.starts_with(ConfigManager::Get<std::string>("output-path")))
        {
            _path = _path.substr(ConfigManager::Get<std::string>("output-path").size() + 1);
        }
        changesOccurred = true;
    }
    ImGui::SameLine();
    gui::widgets::HelpMarker(fmt::format("If a relative path is given, files will be stored inside {}.", flow::GetOutputPath()).c_str());

    return changesOccurred;
}

std::filesystem::path NAV::FileWriter::getFilepath()
{
    std::filesystem::path filepath{ _path };
    if (filepath.is_relative())
    {
        filepath = flow::GetOutputPath();
        filepath /= _path;
    }
    return filepath;
}

[[nodiscard]] json NAV::FileWriter::save() const
{
    LOG_TRACE("called");

    json j;

    j["path"] = _path;
    j["fileType"] = _fileType;

    return j;
}

void NAV::FileWriter::restore(json const& j)
{
    LOG_TRACE("called");

    if (j.contains("path"))
    {
        j.at("path").get_to(_path);
    }
    if (j.contains("fileType"))
    {
        j.at("fileType").get_to(_fileType);
    }
}

bool NAV::FileWriter::initialize()
{
    deinitialize();

    LOG_TRACE("called");

    if (_fileType == FileType::NONE)
    {
        LOG_ERROR("FileWriter needs the _fileType set in the child class.");
        return false;
    }

    std::filesystem::path filepath = getFilepath();

    if (!std::filesystem::exists(filepath.parent_path()) && !std::filesystem::create_directories(filepath.parent_path()))
    {
        LOG_ERROR("Could not create directory '{}' for file '{}'", filepath.parent_path(), filepath);
    }

    if (_fileType == FileType::ASCII || _fileType == FileType::BINARY)
    {
        // Does not enable binary read/write, but disables OS dependant treatment of \n, \r
        _filestream.open(filepath, std::ios_base::trunc | std::ios_base::binary);
    }

    if (!_filestream.good())
    {
        LOG_ERROR("Could not open file {}", filepath);
        return false;
    }

    return true;
}

void NAV::FileWriter::deinitialize()
{
    LOG_TRACE("called");

    try
    {
        if (_filestream.is_open())
        {
            _filestream.flush();
            _filestream.close();
        }
    }
    catch (...)
    {
    }

    _filestream.clear();
}

const char* NAV::FileWriter::to_string(NAV::FileWriter::FileType type)
{
    switch (type)
    {
    case FileType::NONE:
        return "None";
    case FileType::ASCII:
        return "CSV";
    case FileType::BINARY:
        return "Binary";
    default:
        return "Unkown";
    }
}