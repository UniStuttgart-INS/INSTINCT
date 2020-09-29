#ifndef DISABLE_SENSORS

    #include "KvhSensor.hpp"

    #include "util/Logger.hpp"

    #include "util/UartSensors/KVH/KvhUtilities.hpp"

NAV::KvhSensor::KvhSensor(const std::string& name, const std::map<std::string, std::string>& options)
    : UartSensor(options), Imu(name, options), sensor(name)
{
    LOG_TRACE("called for {}", name);

    // connect to the sensor
    try
    {
        // TODO: Update the library to handle different baudrates
        sensorBaudrate = Baudrate::BAUDRATE_921600;

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

NAV::KvhSensor::~KvhSensor()
{
    LOG_TRACE("called for {}", name);

    removeAllCallbacksOfType<KvhObs>();
    callbacksEnabled = false;
    if (sensor->isConnected())
    {
        sensor->unregisterAsyncPacketReceivedHandler();
        sensor->disconnect();
    }
}

void NAV::KvhSensor::asciiOrBinaryAsyncMessageReceived(void* userData, uart::protocol::Packet& p, [[maybe_unused]] size_t index)
{
    auto* kvhSensor = static_cast<KvhSensor*>(userData);

    if (p.type() == uart::protocol::Packet::Type::TYPE_BINARY)
    {
        auto obs = std::make_shared<KvhObs>(p);

        sensors::kvh::decryptKvhObs(obs);

        LOG_DATA("DATA({}): {}, {}, {}",
                 kvhSensor->name, obs->sequenceNumber, obs->temperature.value(), obs->status);

        // Check if a packet was skipped
        if (kvhSensor->prevSequenceNumber == UINT8_MAX)
        {
            kvhSensor->prevSequenceNumber = obs->sequenceNumber;
        }
        if (obs->sequenceNumber != 0 && (obs->sequenceNumber < kvhSensor->prevSequenceNumber || obs->sequenceNumber > kvhSensor->prevSequenceNumber + 2))
        {
            LOG_WARN("{}: Sequence Number changed from {} to {}", kvhSensor->name, kvhSensor->prevSequenceNumber, obs->sequenceNumber);
        }
        kvhSensor->prevSequenceNumber = obs->sequenceNumber;

        // Calls all the callbacks
        kvhSensor->invokeCallbacks(obs);
    }
    else if (p.type() == uart::protocol::Packet::Type::TYPE_ASCII)
    {
        LOG_WARN("{}: Received an ASCII Async message: {}", kvhSensor->name, p.datastr());
    }
}

#endif