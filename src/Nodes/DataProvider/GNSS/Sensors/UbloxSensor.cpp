#ifndef DISABLE_UB_SENSORS

    #include "UbloxSensor.hpp"

    #include "util/Logger.hpp"

NAV::UbloxSensor::UbloxSensor(const std::string& name, std::deque<std::string>& options)
    : UartSensor(options), Gnss(name, options)
{
    LOG_TRACE("called for {}", name);

    if (!options.empty())
    {
        config.outputFrequency = static_cast<uint16_t>(std::stoul(options.at(0)));
        options.pop_front();
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
        LOG_CRITICAL("{} could not connect", name);
    }

    ub.registerAsyncPacketReceivedHandler(this, asciiOrBinaryAsyncMessageReceived);

    LOG_DEBUG("{} successfully initialized", name);
}

NAV::UbloxSensor::~UbloxSensor()
{
    LOG_TRACE("called for {}", name);

    removeAllCallbacksOfType<UbloxObs>();
    callbacksEnabled = false;
    if (ub.isConnected())
    {
        ub.unregisterAsyncPacketReceivedHandler();
        ub.disconnect();
    }
}

void NAV::UbloxSensor::asciiOrBinaryAsyncMessageReceived(void* userData, ub::protocol::uart::Packet& p, size_t /*index*/)
{
    auto* ubSensor = static_cast<UbloxSensor*>(userData);

    auto obs = std::make_shared<UbloxObs>();
    obs->raw.setData(p.getRawData(), p.getRawDataLength());

    if (p.type() == ub::protocol::uart::Packet::TYPE_BINARY)
    {
        obs->msgClass = static_cast<ublox::UbxClass>(obs->raw.extractUint8());
        obs->msgId = obs->raw.extractUint8();
        obs->payloadLength = obs->raw.extractUint16();

        if (obs->msgClass == ublox::UbxClass::UBX_CLASS_NAV)
        {
            auto msgId = static_cast<ublox::UbxNavMessages>(obs->msgId);
            if (msgId == ublox::UbxNavMessages::UBX_NAV_ATT)
            {
                LOG_DATA("DATA({}): UBX:  NAV-ATT, Size {}", ubSensor->name, (obs->payloadLength + 8));
            }
            else if (msgId == ublox::UbxNavMessages::UBX_NAV_POSLLH)
            {
                LOG_DATA("DATA({}): UBX:  NAV-POSLLH, Size {}", ubSensor->name, (obs->payloadLength + 8));
            }
            else if (msgId == ublox::UbxNavMessages::UBX_NAV_VELNED)
            {
                LOG_DATA("DATA({}): UBX:  NAV-VELNED, Size {}", ubSensor->name, (obs->payloadLength + 8));
            }
            else
            {
                LOG_DATA("DATA({}): UBX:  NAV-{:x}, Size {}", ubSensor->name, msgId, (obs->payloadLength + 8));
            }
        }
        else if (obs->msgClass == ublox::UbxClass::UBX_CLASS_RXM)
        {
            auto msgId = static_cast<ublox::UbxRxmMessages>(obs->msgId);
            if (msgId == ublox::UbxRxmMessages::UBX_RXM_SFRBX)
            {
                LOG_DATA("DATA({}): UBX:  RXM-SFRBX, Size {}", ubSensor->name, (obs->payloadLength + 8));
            }
            else if (msgId == ublox::UbxRxmMessages::UBX_RXM_RAWX)
            {
                double gpsTimeOfWeek = obs->raw.extractDouble();
                uint16_t gpsWeek = obs->raw.extractUint16();

                obs->insTime.emplace(gpsWeek, static_cast<long double>(gpsTimeOfWeek), 0);

                LOG_DATA("DATA({}): UBX:  RXM-RAWX, Size {}, rcvTow {}, week {}", ubSensor->name,
                         (obs->payloadLength + 8), obs->gpsTimeOfWeek.value(), obs->gpsWeek.value());
            }
            else
            {
                LOG_DATA("DATA({}): UBX:  RXM-{:x}, Size {}", ubSensor->name, msgId, (obs->payloadLength + 8));
            }
        }
        else if (obs->msgClass == ublox::UbxClass::UBX_CLASS_ESF)
        {
            auto msgId = static_cast<ublox::UbxEsfMessages>(obs->msgId);
            if (msgId == ublox::UbxEsfMessages::UBX_ESF_RAW)
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
    }
    else if (p.type() == ub::protocol::uart::Packet::TYPE_ASCII)
    {
        LOG_DATA("DATA({}): NMEA: {}", ubSensor->name, p.datastr());
    }

    ubSensor->invokeCallbacks(obs);
}

#endif