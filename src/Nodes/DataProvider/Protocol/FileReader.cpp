#include "FileReader.hpp"

#include "util/Logger.hpp"

NAV::FileReader::FileReader(const std::map<std::string, std::string>& options)
{
    LOG_TRACE("called");

    if (options.contains("Path"))
    {
        path = options.at("Path");
    }
}