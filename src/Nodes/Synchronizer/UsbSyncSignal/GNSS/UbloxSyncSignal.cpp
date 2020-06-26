#ifndef DISABLE_UB_SENSORS

    #include "UbloxSyncSignal.hpp"

    #include "util/Logger.hpp"
    #include "NodeData/GNSS/UbloxObs.hpp"

NAV::UbloxSyncSignal::UbloxSyncSignal(const std::string& name, const std::map<std::string, std::string>& options)
    : UsbSyncSignal(name, options)
{
    LOG_TRACE("called for {}", name);

    // type, msgClass, msgId
    if (options.contains("Protocol") && options.contains("Class") && options.contains("Msg Id"))
    {
        if (options.at("Protocol") == "UBX")
        {
            triggerClass = ublox::getMsgClassFromString(options.at("Class"));
            triggerId = ublox::getMsgIdFromString(triggerClass, options.at("Msg Id"));
        }
        else
        {
            LOG_CRITICAL("Node {} has unknown type {}", name, options.at("Protocol"));
        }
    }
    else
    {
        LOG_CRITICAL("Node {} has not enough options", name);
    }
}

void NAV::UbloxSyncSignal::triggerSync(const std::shared_ptr<NAV::UbloxObs>& obs)
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

#endif