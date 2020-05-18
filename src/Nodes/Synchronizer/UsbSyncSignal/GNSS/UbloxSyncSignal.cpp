#include "UbloxSyncSignal.hpp"

#include "util/Logger.hpp"
#include "NodeData/GNSS/UbloxObs.hpp"

NAV::UbloxSyncSignal::UbloxSyncSignal(const std::string& name, std::deque<std::string>& options)
    : UsbSyncSignal(name, options)
{
    LOG_TRACE("called for {}", name);

    // type, msgClass, msgId
    if (options.size() >= 3)
    {
        if (options.at(0) == "UBX")
        {
            triggerClass = ub::protocol::uart::getMsgClassFromString(options.at(1));
            triggerId = ub::protocol::uart::getMsgIdFromString(triggerClass, options.at(2));
            options.pop_front();
            options.pop_front();
            options.pop_front();
        }
        else
        {
            LOG_CRITICAL("Node {} has unknown type {}", name, options.at(0));
        }
    }
    else
    {
        LOG_CRITICAL("Node {} has not enough options", name);
    }
}

void NAV::UbloxSyncSignal::triggerSync(std::shared_ptr<NAV::UbloxObs>& obs)
{
    if (obs->msgClass == triggerClass && obs->msgId == triggerId)
    {
        // NOLINTNEXTLINE
        if (ioctl(fd, TIOCMBIS, &pinFlag) == -1) //Set RTS pin
        {
            LOG_WARN("{} could not set pin on port {}", name, port);
            return;
        }
        // NOLINTNEXTLINE
        if (ioctl(fd, TIOCMBIC, &pinFlag) == -1) //Clear RTS pin
        {
            LOG_WARN("{} could not clear pin on port {}", name, port);
            return;
        }
    }

    return invokeCallbacks(obs);
}