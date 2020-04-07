#include "DataLogger.hpp"

#include "util/Logger.hpp"

NAV::DataLogger::DataLogger(std::string name, std::string path, bool isBinary)
    : Node(name), path(path), isBinary(isBinary) {}

NAV::DataLogger::DataLogger(std::string name)
    : Node(name) {}

NAV::DataLogger::~DataLogger()
{
    deinitialize();
}

NAV::NavStatus NAV::DataLogger::initialize()
{
    LOG_TRACE("called for {}", name);

    if (initialized)
    {
        LOG_WARN("{} already initialized!!!", name);
        return NavStatus::NAV_WARNING_ALREADY_INITIALIZED;
    }

    if (isBinary)
        filestream.open(path, std::ios_base::trunc | std::ios_base::binary);
    else
        filestream.open(path, std::ios_base::trunc);

    if (!filestream.good())
    {
        LOG_ERROR("{} could not be opened", name);
        return NavStatus::NAV_ERROR_COULD_NOT_OPEN_FILE;
    }
    else
        return NavStatus::NAV_OK;
}

NAV::NavStatus NAV::DataLogger::deinitialize()
{
    LOG_TRACE("called for {}", name);

    if (initialized)
    {
        if (filestream.is_open())
        {
            filestream.flush();
            filestream.close();
        }
        initialized = false;
        LOG_DEBUG("{} successfully deinitialized", name);
        return NavStatus::NAV_OK;
    }

    return NAV_WARNING_NOT_INITIALIZED;
}