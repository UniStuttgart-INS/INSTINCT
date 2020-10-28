#include "DataLogger.hpp"

#include "util/Logger.hpp"

NAV::DataLogger::DataLogger(const std::string& name, const std::map<std::string, std::string>& options)
    : Node(name)
{
    LOG_TRACE("called for {}", name);

    if (options.count("Path"))
    {
        path = options.at("Path");
    }
    if (options.count("Type"))
    {
        if (options.at("Type") == "ascii")
        {
            fileType = FileType::ASCII;
        }
        else if (options.at("Type") == "binary")
        {
            fileType = FileType::BINARY;
        }
        else
        {
            LOG_CRITICAL("Node {} has unknown file type {}", name, options.at("Type"));
        }
    }

    if (fileType == FileType::BINARY)
    {
        filestream.open(path, std::ios_base::trunc | std::ios_base::binary);
    }
    else
    {
        filestream.open(path, std::ios_base::trunc);
    }

    if (!filestream.good())
    {
        LOG_CRITICAL("{} could not be opened", name);
    }
}

NAV::DataLogger::~DataLogger()
{
    LOG_TRACE("called for {}", name);

    try
    {
        if (filestream.is_open())
        {
            filestream.flush();
            filestream.close();
        }
    }
    catch (...)
    {
    }
}