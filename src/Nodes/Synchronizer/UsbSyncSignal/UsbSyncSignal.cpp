#include "UsbSyncSignal.hpp"

#include "util/Logger.hpp"

#include <fcntl.h>  // open
#include <unistd.h> // close

NAV::UsbSyncSignal::UsbSyncSignal(std::string name, std::deque<std::string>& options)
    : Node(name)
{
    LOG_TRACE("called for {}", name);

    if (options.size() >= 1)
    {
        port = options.at(0);
        options.pop_front();
    }
    else
        LOG_CRITICAL("Node {} has not enough options", name);

    fd = open(port.c_str(), O_RDWR | O_NOCTTY); //Open Serial Port

    if (fd)
    {
        if (ioctl(fd, TIOCMBIC, &pinFlag) == -1) //Clear RTS pin
            LOG_WARN("{} could not clear pin on port {}", name, port);
    }
    else
        LOG_CRITICAL("{} could not open port {}", name, port);
}

NAV::UsbSyncSignal::~UsbSyncSignal()
{
    LOG_TRACE("called for {}", name);

    if (fd)
        close(fd);
}