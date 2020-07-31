#include "FileReader.hpp"

#include "util/Logger.hpp"

#include <sstream>

NAV::FileReader::FileReader(std::string name, const std::map<std::string, std::string>& options)
    : parentNodeName(std::move(name))
{
    LOG_TRACE("{}: called", parentNodeName);

    if (options.count("Path"))
    {
        path = options.at("Path");
    }
    else
    {
        LOG_CRITICAL("{}: There was no path provided to the node", parentNodeName);
    }
}

void NAV::FileReader::initialize()
{
    LOG_TRACE("{}: called", parentNodeName);

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
        LOG_CRITICAL("{}: Could not open file {}", parentNodeName, path);
        return;
    }

    readHeader();

    dataStart = filestream.tellg();

    if (fileType == FileType::ASCII)
    {
        LOG_DEBUG("{}: ASCII-File successfully initialized", parentNodeName);
    }
    else if (fileType == FileType::BINARY)
    {
        LOG_DEBUG("{}: Binary-File successfully initialized", parentNodeName);
    }
}

NAV::FileReader::FileType NAV::FileReader::determineFileType()
{
    LOG_TRACE("{}: called", parentNodeName);

    auto filestream = std::ifstream(path);
    if (filestream.good())
    {
        std::string line;
        std::getline(filestream, line);
        filestream.close();

        auto n = std::count(line.begin(), line.end(), ',');

        if (n >= 3)
        {
            return FileType::ASCII;
        }

        return FileType::BINARY;
    }

    LOG_CRITICAL("{} could not open file {}", parentNodeName, path);
    return FileType::NONE;
}

void NAV::FileReader::readHeader()
{
    LOG_TRACE("{}: called", parentNodeName);

    if (fileType == FileType::ASCII)
    {
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
            columns.push_back(cell);
        }
    }
}

void NAV::FileReader::resetReader()
{
    // Return to position
    filestream.clear();
    filestream.seekg(dataStart, std::ios_base::beg);
}