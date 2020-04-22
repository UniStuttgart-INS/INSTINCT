#include "UbloxDataLogger.hpp"

#include "util/Logger.hpp"
#include "NodeData/GNSS/UbloxObs.hpp"

#include "NodeInterface.hpp"

#include <iomanip> // std::setprecision

NAV::UbloxDataLogger::UbloxDataLogger(std::string name, std::deque<std::string>& options)
    : DataLogger(name, options)
{
    LOG_TRACE("called for {}", name);
}

NAV::UbloxDataLogger::~UbloxDataLogger()
{
    LOG_TRACE("called for {}", name);
}

NAV::NavStatus NAV::UbloxDataLogger::writeObservation(std::shared_ptr<NAV::NodeData> obs, std::shared_ptr<NAV::Node> userData)
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
        LOG_CRITICAL("ASCII Logging of UbloxObs is not implemented yet");
    }

    return logger->invokeCallbacks(NodeInterface::getCallbackPort("UbloxDataLogger", "UbloxObs"), obs);
}