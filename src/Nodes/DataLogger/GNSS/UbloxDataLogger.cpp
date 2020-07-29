#include "UbloxDataLogger.hpp"

#include "util/Logger.hpp"

#include <iomanip> // std::setprecision

NAV::UbloxDataLogger::UbloxDataLogger(const std::string& name, const std::map<std::string, std::string>& options)
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
        if (obs->raw.getRawDataLength() > 0)
        {
            filestream.write(reinterpret_cast<const char*>(obs->raw.getRawData().data()), static_cast<std::streamsize>(obs->raw.getRawDataLength()));
        }
        else
        {
            LOG_ERROR("{}: Tried to write binary, but observation had no binary data.", name);
        }
    }
    else
    {
        // TODO: Implement this
        LOG_CRITICAL("ASCII Logging of UbloxObs is not implemented yet");
    }

    invokeCallbacks(obs);
}