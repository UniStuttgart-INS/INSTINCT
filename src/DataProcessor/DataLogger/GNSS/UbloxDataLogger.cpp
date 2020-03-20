#include "UbloxDataLogger.hpp"

#include "util/Logger.hpp"
#include "DataProvider/GNSS/Observations/UbloxObs.hpp"

#include <iomanip> // std::setprecision

NAV::UbloxDataLogger::UbloxDataLogger(std::string name, std::string path, bool isBinary)
    : DataLogger(name, path, isBinary) {}

NAV::UbloxDataLogger::~UbloxDataLogger()
{
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