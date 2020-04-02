#include "UbloxSyncSignal.hpp"

#include "util/Logger.hpp"
#include "DataProvider/GNSS/Observations/UbloxObs.hpp"

NAV::UbloxSyncSignal::UbloxSyncSignal(std::string name, std::string port, ub::protocol::uart::UbxClass triggerClass, uint8_t triggerId)
    : UsbSyncSignal(name, port), triggerClass(triggerClass), triggerId(triggerId) {}

NAV::UbloxSyncSignal::~UbloxSyncSignal()
{
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

        sync->invokeCallbacks(observation);
    }
    return NavStatus::NAV_OK;
}