#include "UsbSyncSignal.hpp"

#include "util/Logger.hpp"

#include <fcntl.h>  // open
#include <unistd.h> // close

NAV::UsbSyncSignal::UsbSyncSignal(std::string name, std::string port)
    : DataProcessor(name), port(port) {}

NAV::UsbSyncSignal::~UsbSyncSignal()
{
    deinitialize();
}

NAV::NavStatus NAV::UsbSyncSignal::initialize()
{
    LOG_TRACE("called for {}", name);

    if (initialized)
    {
        LOG_WARN("{} already initialized!!!", name);
        return NavStatus::NAV_WARNING_ALREADY_INITIALIZED;
    }

    fd = open(port.c_str(), O_RDWR | O_NOCTTY); //Open Serial Port

    if (fd)
    {
        if (ioctl(fd, TIOCMBIC, &pinFlag) == -1) //Clear RTS pin
        {
            LOG_WARN("{} could not clear pin on port {}", name, port);
            return NavStatus::NAV_ERROR;
        }
        else
            return NavStatus::NAV_OK;
    }
    else
    {
        LOG_CRITICAL("{} could not open port {}", name, port);
        return NavStatus::NAV_ERROR;
    }
}

NAV::NavStatus NAV::UsbSyncSignal::deinitialize()
{
    LOG_TRACE("called for {}", name);

    if (initialized)
    {
        close(fd);

        initialized = false;

        LOG_DEBUG("{} successfully deinitialized", name);
        return NavStatus::NAV_OK;
    }

    return NAV_WARNING_NOT_INITIALIZED;
}