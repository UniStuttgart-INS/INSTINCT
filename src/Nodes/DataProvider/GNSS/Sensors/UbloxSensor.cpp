#include "UbloxSensor.hpp"

#include "NodeInterface.hpp"

#include "NodeData/GNSS/UbloxObs.hpp"
#include "util/Logger.hpp"

NAV::UbloxSensor::UbloxSensor(std::string name, std::vector<std::string> options)
    : Gnss(name)
{
    LOG_TRACE("called for {}", name);

    if (options.size() >= 1)
        config.outputFrequency = static_cast<uint16_t>(std::stoul(options.at(0)));
    if (options.size() >= 2)
        sensorPort = options.at(1);
    if (options.size() >= 3)
    {
        if (options.at(2) == "Fastest")
            sensorBaudrate = UartSensor::Baudrate::BAUDRATE_FASTEST;
        else
            sensorBaudrate = static_cast<UartSensor::Baudrate>(std::stoul(options.at(2)));
    }
}

NAV::UbloxSensor::~UbloxSensor()
{
    LOG_TRACE("called for {}", name);
    deinitialize();
}

NAV::NavStatus NAV::UbloxSensor::initialize()
{
    LOG_TRACE("called for {}", name);
    if (initialized)
    {
        LOG_WARN("{} already initialized!!!", name);
        return NavStatus::NAV_WARNING_ALREADY_INITIALIZED;
    }

    // connect to the sensor
    try
    {
        // TODO: Update the library to handle different baudrates
        sensorBaudrate = Baudrate::BAUDRATE_9600;

        ub.connect(sensorPort, sensorBaudrate);

        LOG_DEBUG("{} connected on port {} with baudrate {}", name, sensorPort, sensorBaudrate);
    }
    catch (...)
    {
        LOG_ERROR("{} could not connect", name);
        return NavStatus::NAV_ERROR_COULD_NOT_CONNECT;
    }

    ub.registerAsyncPacketReceivedHandler(this, asciiOrBinaryAsyncMessageReceived);

    LOG_DEBUG("{} successfully initialized", name);

    initialized = true;

    return NavStatus::NAV_OK;
}

NAV::NavStatus NAV::UbloxSensor::deinitialize()
{
    LOG_TRACE("called for {}", name);
    if (initialized)
    {
        removeAllCallbacks();
        callbacksEnabled = false;
        ub.unregisterAsyncPacketReceivedHandler();
        if (ub.isConnected())
        {
            ub.disconnect();
        }
        initialized = false;
        LOG_DEBUG("{} successfully deinitialized", name);
        return NAV_OK;
    }

    return NAV_WARNING_NOT_INITIALIZED;
}

std::shared_ptr<NAV::InsObs> NAV::UbloxSensor::pollObservation()
{
    LOG_TRACE("called for {}", name);

    // TODO: Implement this
    throw std::logic_error("Not implemented");

    auto obs = std::make_shared<UbloxObs>();

    LOG_DATA("DATA({}): {}, {}, {}",
             name, obs->msgClass, obs->msgId, obs->payloadLength);

    // Calls all the callbacks
    invokeCallbacks(NodeInterface::getCallbackPort("UbloxSensor", "UbloxObs"), obs);

    return obs;
}

void NAV::UbloxSensor::asciiOrBinaryAsyncMessageReceived(void* userData, ub::protocol::uart::Packet& p, size_t /*index*/)
{
    UbloxSensor* ubSensor = static_cast<UbloxSensor*>(userData);
    LOG_TRACE("called for {}", ubSensor->name);

    if (!ubSensor->initialized)
        return;

    if (p.type() == ub::protocol::uart::Packet::TYPE_BINARY)
    {
        auto obs = std::make_shared<UbloxObs>();
        obs->p = &p;

        obs->msgClass = static_cast<ub::protocol::uart::UbxClass>(p.extractUint8());
        obs->msgId = p.extractUint8();
        obs->payloadLength = p.extractUint16();

        if (obs->msgClass == ub::protocol::uart::UbxClass::UBX_CLASS_NAV)
        {
            ub::protocol::uart::UbxNavMessages msgId = static_cast<ub::protocol::uart::UbxNavMessages>(obs->msgId);
            if (msgId == ub::protocol::uart::UbxNavMessages::UBX_NAV_ATT)
            {
                LOG_DATA("DATA({}): UBX:  NAV-ATT, Size {}", ubSensor->name, (obs->payloadLength + 8));
            }
            else if (msgId == ub::protocol::uart::UbxNavMessages::UBX_NAV_POSLLH)
            {
                LOG_DATA("DATA({}): UBX:  NAV-POSLLH, Size {}", ubSensor->name, (obs->payloadLength + 8));
            }
            else if (msgId == ub::protocol::uart::UbxNavMessages::UBX_NAV_VELNED)
            {
                LOG_DATA("DATA({}): UBX:  NAV-VELNED, Size {}", ubSensor->name, (obs->payloadLength + 8));
            }
            else
            {
                LOG_DATA("DATA({}): UBX:  NAV-{:x}, Size {}", ubSensor->name, msgId, (obs->payloadLength + 8));
            }
        }
        else if (obs->msgClass == ub::protocol::uart::UbxClass::UBX_CLASS_RXM)
        {
            ub::protocol::uart::UbxRxmMessages msgId = static_cast<ub::protocol::uart::UbxRxmMessages>(obs->msgId);
            if (msgId == ub::protocol::uart::UbxRxmMessages::UBX_RXM_SFRBX)
            {
                LOG_DATA("DATA({}): UBX:  RXM-SFRBX, Size {}", ubSensor->name, (obs->payloadLength + 8));
            }
            else if (msgId == ub::protocol::uart::UbxRxmMessages::UBX_RXM_RAWX)
            {
                obs->gpsTimeOfWeek = p.extractDouble();
                obs->gpsWeek = p.extractUint16();

                LOG_DATA("DATA({}): UBX:  RXM-RAWX, Size {}, rcvTow {}, week {}", ubSensor->name,
                         (obs->payloadLength + 8), obs->gpsTimeOfWeek.value(), obs->gpsWeek.value());
            }
            else
            {
                LOG_DATA("DATA({}): UBX:  RXM-{:x}, Size {}", ubSensor->name, msgId, (obs->payloadLength + 8));
            }
        }
        else if (obs->msgClass == ub::protocol::uart::UbxClass::UBX_CLASS_ESF)
        {
            ub::protocol::uart::UbxEsfMessages msgId = static_cast<ub::protocol::uart::UbxEsfMessages>(obs->msgId);
            if (msgId == ub::protocol::uart::UbxEsfMessages::UBX_ESF_RAW)
            {
                LOG_DATA("DATA({}): UBX:  ESF-RAW, Size {}", ubSensor->name, (obs->payloadLength + 8));
            }
            else
            {
                LOG_DATA("DATA({}): UBX:  ESF-{:x}, Size {}", ubSensor->name, msgId, (obs->payloadLength + 8));
            }
        }
        else
        {
            LOG_DATA("DATA({}): UBX:  {:x}-{:x}, Size {}", ubSensor->name, obs->msgClass, obs->msgId, (obs->payloadLength + 8));
        }

        ubSensor->invokeCallbacks(NodeInterface::getCallbackPort("UbloxSensor", "UbloxObs"), obs);
    }
    else if (p.type() == ub::protocol::uart::Packet::TYPE_ASCII)
    {
        LOG_DATA("DATA({}): NMEA: {}", ubSensor->name, p.datastr());

        auto obs = std::make_shared<UbloxObs>();
        obs->p = &p;

        ubSensor->invokeCallbacks(NodeInterface::getCallbackPort("UbloxSensor", "UbloxObs"), obs);
    }
}