#include "FileReader.hpp"

#include "util/Logger.hpp"

#include <sstream>
#include "util/StringUtil.hpp"

[[nodiscard]] json NAV::FileReader::save() const
{
    LOG_TRACE("called");

    json j;

    j["path"] = path;

    return j;
}

void NAV::FileReader::restore(json const& j)
{
    LOG_TRACE("called");

    if (j.contains("path"))
    {
        j.at("path").get_to(path);
    }
}

bool NAV::FileReader::initialize()
{
    deinitialize();

    LOG_TRACE("called");

    fileType = determineFileType();

    if (fileType == FileType::ASCII)
    {
        filestream = std::ifstream(path);
    }
    else if (fileType == FileType::BINARY)
    {
        // Does not enable binary read/write, but disables OS dependant treatment of \n, \r
        filestream = std::ifstream(path, std::ios_base::in | std::ios_base::binary);
    }

    if (!filestream.good())
    {
        LOG_ERROR("Could not open file {}", path);
        return false;
    }

    readHeader();

    dataStart = filestream.tellg();

    if (fileType == FileType::ASCII)
    {
        LOG_DEBUG("ASCII-File successfully initialized");
    }
    else if (fileType == FileType::BINARY)
    {
        LOG_DEBUG("Binary-File successfully initialized");
    }

    return true;
}

void NAV::FileReader::deinitialize()
{
    LOG_TRACE("called");

    if (filestream.is_open())
    {
        filestream.close();
    }

    filestream.clear();
}

NAV::FileReader::FileType NAV::FileReader::determineFileType()
{
    LOG_TRACE("called");

    auto filestreamHeader = std::ifstream(path);
    if (filestream.good())
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

    LOG_ERROR("Could not open file {}", path);
    return FileType::NONE;
}

void NAV::FileReader::readHeader()
{
    LOG_TRACE("called");

    if (fileType == FileType::ASCII)
    {
        headerColumns.clear();

        // Read header line
        std::string line;
        std::getline(filestream, line);
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
            headerColumns.push_back(cell);
        }
    }
}

void NAV::FileReader::resetReader()
{
    LOG_TRACE("called");

    // Return to position
    filestream.clear();
    filestream.seekg(dataStart, std::ios_base::beg);
}