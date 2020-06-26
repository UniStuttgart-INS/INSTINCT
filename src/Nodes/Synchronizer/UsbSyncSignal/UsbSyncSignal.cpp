#include "UsbSyncSignal.hpp"

#include "util/Logger.hpp"

#include <fcntl.h>  // open
#include <unistd.h> // close

NAV::UsbSyncSignal::UsbSyncSignal(const std::string& name, const std::map<std::string, std::string>& options)
    : Node(name)
{
    LOG_TRACE("called for {}", name);

    if (options.contains("Port"))
    {
        port = options.at("Port");
    }
    else
    {
        LOG_CRITICAL("Node {} has not enough options", name);
    }

    // NOLINTNEXTLINE
    fd = open(port.c_str(), O_RDWR | O_NOCTTY); // Open Serial Port

    if (fd)
    {
        // NOLINTNEXTLINE
        if (ioctl(fd, TIOCMBIC, &pinFlag) == -1) // Clear RTS pin
        {
            LOG_WARN("{} could not clear pin on port {}", name, port);
        }
    }
    else
    {
        LOG_CRITICAL("{} could not open port {}", name, port);
    }
}

NAV::UsbSyncSignal::~UsbSyncSignal()
{
    LOG_TRACE("called for {}", name);

    if (fd)
    {
        close(fd);
    }
}