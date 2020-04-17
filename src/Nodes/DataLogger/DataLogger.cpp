#include "DataLogger.hpp"

#include "util/Logger.hpp"

NAV::DataLogger::DataLogger(std::string name, std::deque<std::string>& options)
    : Node(name)
{
    LOG_TRACE("called for {}", name);

    if (options.size() >= 1)
    {
        path = options.at(0);
        options.pop_front();
    }
    if (options.size() >= 1)
    {
        if (options.at(0) == "ascii")
            isBinary = false;
        else if (options.at(0) == "binary")
            isBinary = true;
        else
            LOG_CRITICAL("Node {} has unknown file type {}", name, options.at(0));

        options.pop_front();
    }

    if (isBinary)
        filestream.open(path, std::ios_base::trunc | std::ios_base::binary);
    else
        filestream.open(path, std::ios_base::trunc);

    if (!filestream.good())
        LOG_CRITICAL("{} could not be opened", name);
}

NAV::DataLogger::~DataLogger()
{
    LOG_TRACE("called for {}", name);

    if (filestream.is_open())
    {
        filestream.flush();
        filestream.close();
    }

    LOG_DEBUG("{} successfully deinitialized", name);
}