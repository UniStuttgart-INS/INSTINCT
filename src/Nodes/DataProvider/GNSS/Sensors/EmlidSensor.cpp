#ifndef DISABLE_SENSORS

    #include "EmlidSensor.hpp"

    #include "util/Logger.hpp"

    #include "util/UartSensors/Emlid/EmlidUtilities.hpp"

NAV::EmlidSensor::EmlidSensor(const std::string& name, const std::map<std::string, std::string>& options)
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

    sensor->registerAsyncPacketReceivedHandler(this, asciiOrBinaryAsyncMessageReceived);

    LOG_DEBUG("{} successfully initialized", name);
}

NAV::EmlidSensor::~EmlidSensor()
{
    LOG_TRACE("called for {}", name);

    removeAllCallbacksOfType<EmlidObs>();
    callbacksEnabled = false;
    if (sensor->isConnected())
    {
        sensor->unregisterAsyncPacketReceivedHandler();
        sensor->disconnect();
    }
}

void NAV::EmlidSensor::asciiOrBinaryAsyncMessageReceived(void* userData, uart::protocol::Packet& p, size_t /*index*/)
{
    auto* erSensor = static_cast<EmlidSensor*>(userData);

    auto obs = std::make_shared<EmlidObs>(p);

    sensors::emlid::decryptEmlidObs(obs, erSensor->currentInsTime);

    erSensor->invokeCallbacks(obs);
}

#endif