#ifndef DISABLE_SENSORS

    #include "EmlidSensor.hpp"

    #include "util/Logger.hpp"

    #include "util/Emlid/EmlidDecryptor.hpp"

NAV::EmlidSensor::EmlidSensor(const std::string& name, const std::map<std::string, std::string>& options)
    : UartSensor(options), Gnss(name, options)
{
    LOG_TRACE("called for {}", name);

    if (options.contains("Frequency"))
    {
        config.outputFrequency = static_cast<uint16_t>(std::stoul(options.at("Frequency")));
    }

    // connect to the sensor
    try
    {
        // TODO: Update the library to handle different baudrates
        sensorBaudrate = Baudrate::BAUDRATE_9600;

        er.connect(sensorPort, sensorBaudrate);

        LOG_DEBUG("{} connected on port {} with baudrate {}", name, sensorPort, sensorBaudrate);
    }
    catch (...)
    {
        LOG_CRITICAL("{} could not connect", name);
    }

    er.registerAsyncPacketReceivedHandler(this, asciiOrBinaryAsyncMessageReceived);

    LOG_DEBUG("{} successfully initialized", name);
}

NAV::EmlidSensor::~EmlidSensor()
{
    LOG_TRACE("called for {}", name);

    removeAllCallbacksOfType<EmlidObs>();
    callbacksEnabled = false;
    if (er.isConnected())
    {
        er.unregisterAsyncPacketReceivedHandler();
        er.disconnect();
    }
}

void NAV::EmlidSensor::asciiOrBinaryAsyncMessageReceived(void* userData, er::protocol::uart::Packet& p, size_t /*index*/)
{
    auto* erSensor = static_cast<EmlidSensor*>(userData);

    auto obs = std::make_shared<EmlidObs>();
    obs->raw.setData(p.getRawData(), p.getRawDataLength());

    Emlid::decryptEmlidObs(obs, erSensor->currentInsTime);

    erSensor->invokeCallbacks(obs);
}

#endif