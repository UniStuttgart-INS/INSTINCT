#include "UbloxDataLogger.hpp"

#include "util/Logger.hpp"

#include <iomanip> // std::setprecision

NAV::UbloxDataLogger::UbloxDataLogger(const std::string& name, std::deque<std::string>& options)
    : DataLogger(name, options)
{
    LOG_TRACE("called for {}", name);
}

NAV::UbloxDataLogger::~UbloxDataLogger()
{
    LOG_TRACE("called for {}", name);
}

void NAV::UbloxDataLogger::writeObservation(std::shared_ptr<NAV::UbloxObs>& obs)
{
    if (fileType == FileType::BINARY)
    {
        filestream.write(reinterpret_cast<const char*>(obs->raw.getRawData()), static_cast<std::streamsize>(obs->raw.getRawDataLength()));
    }
    else
    {
        // TODO: Implement this
        LOG_CRITICAL("ASCII Logging of UbloxObs is not implemented yet");
    }

    invokeCallbacks(obs);
}