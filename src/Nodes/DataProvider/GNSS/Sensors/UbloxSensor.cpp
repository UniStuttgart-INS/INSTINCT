#ifndef DISABLE_SENSORS

    #include "UbloxSensor.hpp"

    #include "util/Logger.hpp"

    #include "util/Ublox/UbloxDecryptor.hpp"

NAV::UbloxSensor::UbloxSensor(const std::string& name, const std::map<std::string, std::string>& options)
    : UartSensor(options), Gnss(name, options)
{
    LOG_TRACE("called for {}", name);

    // connect to the sensor
    try
    {
        // TODO: Update the library to handle different baudrates
        sensorBaudrate = Baudrate::BAUDRATE_9600;

        sensor->connect(sensorPort, sensorBaudrate);

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

void NAV::UbloxSensor::asciiOrBinaryAsyncMessageReceived(void* userData, uart::protocol::Packet& p, size_t /*index*/)
{
    auto* ubSensor = static_cast<UbloxSensor*>(userData);

    auto obs = std::make_shared<UbloxObs>();
    obs->raw.setData(p.getRawData(), p.getRawDataLength());

    ublox::decryptUbloxObs(obs, ubSensor->currentInsTime);

    ubSensor->invokeCallbacks(obs);
}

#endif