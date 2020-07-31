#include "FileReader.hpp"

#include "util/Logger.hpp"

NAV::FileReader::FileReader(std::string name, const std::map<std::string, std::string>& options)
    : parentNodeName(std::move(name))
{
    LOG_TRACE("{}: called", parentNodeName);

    if (options.count("Path"))
    {
        path = options.at("Path");

        // Does not enable binary read write, but disables operating dependant treating of \n, \r
        filestream = std::ifstream(path, std::ios_base::in | std::ios_base::binary);

        if (!filestream.good())
        {
            LOG_CRITICAL("{}: Could not open file {}", parentNodeName, path);
        }
    }
    else
    {
        LOG_CRITICAL("{}: There was no path provided to the node", parentNodeName);
    }
}