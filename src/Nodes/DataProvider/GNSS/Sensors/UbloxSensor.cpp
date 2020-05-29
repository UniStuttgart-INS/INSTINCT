#ifndef DISABLE_UB_SENSORS

    #include "UbloxSensor.hpp"

    #include "util/Logger.hpp"

    #include "util/Ublox/UbloxDecryptor.hpp"

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

    ublox::decryptUbloxObs(obs, ubSensor->currentInsTime);

    ubSensor->invokeCallbacks(obs);
}

#endif