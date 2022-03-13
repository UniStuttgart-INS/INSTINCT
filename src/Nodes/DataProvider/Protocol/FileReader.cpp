#include "FileReader.hpp"

#include "util/Logger.hpp"

#include "internal/ConfigManager.hpp"
#include "internal/FlowManager.hpp"

#include <sstream>
#include "util/StringUtil.hpp"

#include <imgui.h>
#include "internal/gui/widgets/FileDialog.hpp"
#include "internal/gui/widgets/HelpMarker.hpp"

bool NAV::FileReader::guiConfig(const char* vFilters, const std::vector<std::string>& extensions, size_t id, const std::string& nameId)
{
    bool changesOccurred = false;

    if (gui::widgets::FileDialogLoad(_path, "Select File", vFilters, extensions,
                                     flow::GetProgramRootPath() / ConfigManager::Get<std::string>("input-path", "data"), id, nameId))
    {
        if (_path.starts_with(ConfigManager::Get<std::string>("input-path", "data")))
        {
            _path = _path.substr(ConfigManager::Get<std::string>("input-path", "data").size() + 1);
        }
        changesOccurred = true;
    }
    ImGui::SameLine();
    gui::widgets::HelpMarker(fmt::format("If a relative path is given, files will be searched inside {}.", flow::GetInputPath()).c_str());

    return changesOccurred;
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

    if (_fileType == FileType::CSV || _fileType == FileType::BINARY)
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

    readHeader();

    _dataStart = _filestream.tellg();

    if (_fileType == FileType::CSV)
    {
        LOG_DEBUG("CSV-File successfully initialized");
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
            return FileType::CSV;
        }

        return FileType::BINARY;
    }

    LOG_ERROR("Could not open file {}", filepath.string());
    return FileType::NONE;
}

void NAV::FileReader::readHeader()
{
    LOG_TRACE("called");

    if (_fileType == FileType::CSV)
    {
        _headerColumns.clear();

        // Read header line
        std::string line;
        std::getline(_filestream, line);
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
}