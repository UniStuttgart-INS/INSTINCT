#include "FileReader.hpp"

#include "util/Logger.hpp"

#include "internal/FlowManager.hpp"

#include <sstream>
#include "util/StringUtil.hpp"

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

    _fileType = determineFileType();

    std::string filepath = _path;
    if (!_path.starts_with('/') && !_path.starts_with('~'))
    {
        filepath = flow::GetProgramRootPath() + '/' + _path;
    }

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

    std::string filepath = _path;
    if (!_path.starts_with('/') && !_path.starts_with('~'))
    {
        filepath = flow::GetProgramRootPath() + '/' + _path;
    }

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

    LOG_ERROR("Could not open file {}", filepath);
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