#include "UbloxDataLogger.hpp"

#include "util/Logger.hpp"
#include "NodeData/GNSS/UbloxObs.hpp"

#include <iomanip> // std::setprecision

NAV::UbloxDataLogger::UbloxDataLogger(std::string name, std::vector<std::string> options)
    : DataLogger(name)
{
    LOG_TRACE("called for {}", name);

    isBinary = true;
    if (options.size() >= 1)
        path = options.at(0);
    if (options.size() >= 2)
    {
        if (options.at(1) == "ascii")
            isBinary = false;
        else if (options.at(1) == "binary")
            isBinary = true;
        else
            LOG_WARN("Node {} has unknown file type {}. Using binary instead", name, options.at(1));
    }
}

NAV::UbloxDataLogger::~UbloxDataLogger()
{
    LOG_TRACE("called for {}", name);

    deinitialize();
}

NAV::NavStatus NAV::UbloxDataLogger::initialize()
{
    LOG_TRACE("called for {}", name);

    // Initialize base class
    if (NavStatus result = DataLogger::initialize();
        result != NavStatus::NAV_OK)
        return result;

    LOG_DEBUG("{} successfully initialized", name);

    initialized = true;

    return NavStatus::NAV_OK;
}

NAV::NavStatus NAV::UbloxDataLogger::deinitialize()
{
    LOG_TRACE("called for {}", name);

    // Deinitialize base class after
    if (NavStatus result = DataLogger::deinitialize();
        result != NavStatus::NAV_OK)
        return result;

    return NavStatus::NAV_OK;
}

NAV::NavStatus NAV::UbloxDataLogger::writeObservation(std::shared_ptr<void> obs, std::shared_ptr<void> userData)
{
    auto ubObs = std::static_pointer_cast<UbloxObs>(obs);
    auto logger = std::static_pointer_cast<UbloxDataLogger>(userData);

    LOG_TRACE("called for {}", logger->name);

    if (logger->isBinary)
    {
        logger->filestream.write(reinterpret_cast<char*>(ubObs->p->getRawData()), static_cast<std::streamsize>(ubObs->p->getRawDataLength()));
    }
    else
    {
        // TODO: Implement this
        LOG_ERROR("ASCII Logging of UbloxObs is not implemented yet");
    }

    return NavStatus::NAV_OK;
}