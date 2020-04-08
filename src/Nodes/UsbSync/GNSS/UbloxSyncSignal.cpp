#include "UbloxSyncSignal.hpp"

#include "NodeInterface.hpp"

#include "util/Logger.hpp"
#include "NodeData/GNSS/UbloxObs.hpp"

NAV::UbloxSyncSignal::UbloxSyncSignal(std::string name, std::vector<std::string> options)
    : UsbSyncSignal(name)
{
    LOG_TRACE("called for {}", name);

    //SensorPort, type, msgClass, msgId
    if (options.size() >= 1)
        port = options.at(0);
    if (options.size() >= 4)
    {
        if (options.at(1) == "UBX")
        {
            triggerClass = ub::protocol::uart::getMsgClassFromString(options.at(2));
            triggerId = ub::protocol::uart::getMsgIdFromString(triggerClass, options.at(3));
        }
        else
            LOG_CRITICAL("Node {} has unknown type {}", name, options.at(1));
    }
    else
        LOG_CRITICAL("Node {} has not enough options", name);
}

NAV::UbloxSyncSignal::~UbloxSyncSignal()
{
    LOG_TRACE("called for {}", name);

    deinitialize();
}

NAV::NavStatus NAV::UbloxSyncSignal::initialize()
{
    LOG_TRACE("called for {}", name);

    // Initialize base class
    if (NavStatus result = UsbSyncSignal::initialize();
        result != NavStatus::NAV_OK)
        return result;

    LOG_DEBUG("{} successfully initialized", name);

    initialized = true;

    return NavStatus::NAV_OK;
}

NAV::NavStatus NAV::UbloxSyncSignal::deinitialize()
{
    LOG_TRACE("called for {}", name);

    // Deinitialize base class after
    if (NavStatus result = UsbSyncSignal::deinitialize();
        result != NavStatus::NAV_OK)
        return result;

    return NavStatus::NAV_OK;
}

NAV::NavStatus NAV::UbloxSyncSignal::triggerSync(std::shared_ptr<void> observation, std::shared_ptr<void> userData)
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

        sync->invokeCallbacks(NodeInterface::getCallbackPort("UbloxSyncSignal", "UbloxObs"), observation);
    }
    return NavStatus::NAV_OK;
}