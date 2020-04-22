#include "UbloxSyncSignal.hpp"

#include "NodeInterface.hpp"

#include "util/Logger.hpp"
#include "NodeData/GNSS/UbloxObs.hpp"

NAV::UbloxSyncSignal::UbloxSyncSignal(std::string name, std::deque<std::string>& options)
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
            LOG_CRITICAL("Node {} has unknown type {}", name, options.at(0));
    }
    else
        LOG_CRITICAL("Node {} has not enough options", name);
}

NAV::UbloxSyncSignal::~UbloxSyncSignal()
{
    LOG_TRACE("called for {}", name);
}

NAV::NavStatus NAV::UbloxSyncSignal::triggerSync(std::shared_ptr<NAV::NodeData> observation, std::shared_ptr<NAV::Node> userData)
{
    auto obs = std::static_pointer_cast<UbloxObs>(observation);
    auto sync = std::static_pointer_cast<UbloxSyncSignal>(userData);

    LOG_TRACE("called for {}", sync->name);

    if (obs->msgClass == sync->triggerClass && obs->msgId == sync->triggerId)
    {
        if (ioctl(sync->fd, TIOCMBIS, &sync->pinFlag) == -1) //Set RTS pin
        {
            LOG_WARN("{} could not set pin on port {}", sync->name, sync->port);
            return NavStatus::NAV_ERROR;
        }
        if (ioctl(sync->fd, TIOCMBIC, &sync->pinFlag) == -1) //Clear RTS pin
        {
            LOG_WARN("{} could not clear pin on port {}", sync->name, sync->port);
            return NavStatus::NAV_ERROR;
        }
    }

    return sync->invokeCallbacks(NodeInterface::getCallbackPort("UbloxSyncSignal", "UbloxObs"), observation);
}