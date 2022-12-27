// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "FileReader.hpp"

#include "util/Logger.hpp"

#include "internal/ConfigManager.hpp"
#include "internal/FlowManager.hpp"

#include <sstream>
#include "util/StringUtil.hpp"

#include <imgui.h>
#include "internal/gui/widgets/FileDialog.hpp"
#include "internal/gui/widgets/HelpMarker.hpp"

NAV::FileReader::GuiResult NAV::FileReader::guiConfig(const char* vFilters, const std::vector<std::string>& extensions, size_t id, const std::string& nameId)
{
    GuiResult result = PATH_UNCHANGED;

    if (gui::widgets::FileDialogLoad(_path, "Select File", vFilters, extensions,
                                     flow::GetProgramRootPath() / ConfigManager::Get<std::string>("input-path"), id, nameId))
    {
        if (_path.starts_with(ConfigManager::Get<std::string>("input-path")))
        {
            _path = _path.substr(ConfigManager::Get<std::string>("input-path").size() + 1);
        }

        if (!std::filesystem::exists(getFilepath()))
        {
            result = PATH_CHANGED_INVALID;
        }
        else
        {
            result = PATH_CHANGED;
        }
    }
    ImGui::SameLine();
    gui::widgets::HelpMarker(fmt::format("If a relative path is given, files will be searched inside {}.", flow::GetInputPath()).c_str());

    return result;
}

std::filesystem::path NAV::FileReader::getFilepath()
{
    std::filesystem::path filepath{ _path };
    if (filepath.is_relative())
    {
        filepath = flow::GetInputPath();
        filepath /= _path;
    }
    return filepath;
}

[[nodiscard]] json NAV::FileReader::save() const
{
    LOG_TRACE("called");

    json j;

    j["path"] = _path;

    return j;
}

void NAV::FileReader::restore(json const& j)
{
    LOG_TRACE("called");

    if (j.contains("path"))
    {
        j.at("path").get_to(_path);
    }
}

bool NAV::FileReader::initialize()
{
    deinitialize();

    LOG_TRACE("called");

    if (_path.empty())
    {
        return false;
    }

    std::filesystem::path filepath = getFilepath();

    if (!std::filesystem::exists(filepath))
    {
        LOG_ERROR("File does not exist {}", filepath);
        return false;
    }
    if (std::filesystem::is_directory(filepath))
    {
        LOG_ERROR("Path is a directory {}", filepath);
        return false;
    }

    _fileType = determineFileType();

    if (_fileType == FileType::ASCII || _fileType == FileType::BINARY)
    {
        // Does not enable binary read/write, but disables OS dependant treatment of \n, \r
        _filestream = std::ifstream(filepath, std::ios_base::in | std::ios_base::binary);
    }
    else
    {
        return false;
    }

    if (!_filestream.good())
    {
        LOG_ERROR("Could not open file {}", filepath);
        return false;
    }
    _lineCnt = 0;

    readHeader();

    _lineCntDataStart = _lineCnt;
    _dataStart = _filestream.tellg();

    if (_fileType == FileType::ASCII)
    {
        LOG_DEBUG("ASCII-File successfully initialized");
    }
    else if (_fileType == FileType::BINARY)
    {
        LOG_DEBUG("Binary-File successfully initialized");
    }

    return true;
}

void NAV::FileReader::deinitialize()
{
    LOG_TRACE("called");

    _headerColumns.clear();

    if (_filestream.is_open())
    {
        _filestream.close();
    }

    _filestream.clear();
}

NAV::FileReader::FileType NAV::FileReader::determineFileType()
{
    LOG_TRACE("called");

    auto filepath = getFilepath();

    auto filestreamHeader = std::ifstream(filepath);
    if (_filestream.good())
    {
        std::string line;
        std::getline(filestreamHeader, line);
        filestreamHeader.close();

        auto n = std::count(line.begin(), line.end(), ',');

        if (n >= 3)
        {
            return FileType::ASCII;
        }

        return FileType::BINARY;
    }

    LOG_ERROR("Could not open file {}", filepath.string());
    return FileType::NONE;
}

void NAV::FileReader::readHeader()
{
    LOG_TRACE("called");

    if (_fileType == FileType::ASCII)
    {
        _headerColumns.clear();

        // Read header line
        std::string line;
        getline(line);
        // Remove any starting non text characters
        line.erase(line.begin(), std::find_if(line.begin(), line.end(), [](int ch) { return std::isalnum(ch); }));
        // Convert line into stream
        std::stringstream lineStream(line);
        std::string cell;
        // Split line at comma
        while (std::getline(lineStream, cell, ','))
        {
            // Remove any trailing non text characters
            cell.erase(std::find_if(cell.begin(), cell.end(), [](int ch) { return std::iscntrl(ch); }), cell.end());
            _headerColumns.push_back(cell);
        }
    }
}

void NAV::FileReader::resetReader()
{
    LOG_TRACE("called");

    // Return to position
    _filestream.clear();
    _filestream.seekg(_dataStart, std::ios_base::beg);
    _lineCnt = _lineCntDataStart;
}