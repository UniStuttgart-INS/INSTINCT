#include "EmlidDataLogger.hpp"

#include "util/Logger.hpp"

#include <iomanip> // std::setprecision

NAV::EmlidDataLogger::EmlidDataLogger(const std::string& name, const std::map<std::string, std::string>& options)
    : DataLogger(name, options)
{
    LOG_TRACE("called for {}", name);
}

NAV::EmlidDataLogger::~EmlidDataLogger()
{
    LOG_TRACE("called for {}", name);
}

void NAV::EmlidDataLogger::writeObservation(std::shared_ptr<NAV::EmlidObs>& obs)
{
    if (fileType == FileType::BINARY)
    {
        filestream.write(reinterpret_cast<const char*>(obs->raw.getRawData()), static_cast<std::streamsize>(obs->raw.getRawDataLength()));
    }
    else
    {
        // TODO: Implement this
        LOG_CRITICAL("ASCII Logging of EmlidObs is not implemented yet");
    }

    invokeCallbacks(obs);
}