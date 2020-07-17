#ifndef DISABLE_SENSORS

    #include "KvhSensor.hpp"

    #include "util/Logger.hpp"
    #include "util/Constants.hpp"

NAV::KvhSensor::KvhSensor(const std::string& name, const std::map<std::string, std::string>& options)
    : UartSensor(options), Imu(name, options)
{
    LOG_TRACE("called for {}", name);

    // connect to the sensor
    try
    {
        // TODO: Update the library to handle different baudrates
        sensorBaudrate = Baudrate::BAUDRATE_921600;

        sensor.connect(sensorPort, sensorBaudrate);

        LOG_DEBUG("{} connected on port {} with baudrate {}", name, sensorPort, sensorBaudrate);
    }
    catch (...)
    {
        LOG_CRITICAL("{} could not connect", name);
    }

    sensor.registerAsyncPacketReceivedHandler(this, asciiOrBinaryAsyncMessageReceived);

    LOG_DEBUG("{} successfully initialized", name);
}

NAV::KvhSensor::~KvhSensor()
{
    LOG_TRACE("called for {}", name);

    removeAllCallbacksOfType<KvhObs>();
    callbacksEnabled = false;
    if (sensor.isConnected())
    {
        sensor.unregisterAsyncPacketReceivedHandler();
        // sensor.reset(true);
        sensor.disconnect();
    }
}

void NAV::KvhSensor::asciiOrBinaryAsyncMessageReceived(void* userData, kvh::protocol::uart::Packet& p, size_t /* index */)
{
    auto* kvhSensor = static_cast<KvhSensor*>(userData);

    if (p.type() == kvh::protocol::uart::Packet::Type::TYPE_BINARY_FMT_A)
    {
        auto obs = std::make_shared<KvhObs>();
        obs->raw.setData(p.getRawData(), p.getRawDataLength());

        obs->gyroUncompXYZ.emplace(p.extractFloat(), p.extractFloat(), p.extractFloat());

        obs->accelUncompXYZ.emplace(p.extractFloat(), p.extractFloat(), p.extractFloat());
        obs->accelUncompXYZ.value() *= InsConst::G_NORM;

        obs->status = p.extractUint8();
        obs->sequenceNumber = p.extractUint8();
        obs->temperature = p.extractUint16();

        LOG_DATA("DATA({}): A {}, {}, {}",
                 kvhSensor->name, obs->sequenceNumber, obs->temperature, obs->status);

        // Calls all the callbacks
        kvhSensor->invokeCallbacks(obs);
    }
    else if (p.type() == kvh::protocol::uart::Packet::Type::TYPE_BINARY_FMT_B)
    {
        auto obs = std::make_shared<KvhObs>();
        obs->raw.setData(p.getRawData(), p.getRawDataLength());

        obs->gyroUncompXYZ.emplace(p.extractFloat(), p.extractFloat(), p.extractFloat());

        obs->accelUncompXYZ.emplace(p.extractFloat(), p.extractFloat(), p.extractFloat());
        obs->accelUncompXYZ.value() *= InsConst::G_NORM;

        obs->timeSinceStartup.emplace(p.extractUint32() * 1000);
        obs->status = p.extractUint8();
        obs->sequenceNumber = p.extractUint8();
        obs->temperature = p.extractUint16();

        LOG_DATA("DATA({}): B {}, {}, {}, {}",
                 kvhSensor->name, obs->timeSinceStartup.value(), obs->sequenceNumber, obs->temperature, obs->status);

        // Calls all the callbacks
        kvhSensor->invokeCallbacks(obs);
    }
    else if (p.type() == kvh::protocol::uart::Packet::Type::TYPE_BINARY_FMT_C)
    {
        auto obs = std::make_shared<KvhObs>();
        obs->raw.setData(p.getRawData(), p.getRawDataLength());

        obs->gyroUncompXYZ.emplace(p.extractFloat(), p.extractFloat(), p.extractFloat());

        obs->accelUncompXYZ.emplace(p.extractFloat(), p.extractFloat(), p.extractFloat());
        obs->accelUncompXYZ.value() *= InsConst::G_NORM;

        auto OneOfTempMagXYZ = p.extractFloat();

        obs->status = p.extractUint8();
        obs->sequenceNumber = p.extractUint8();

        static std::array<double, 4> tempMagXYZ{ std::nan(""), std::nan(""), std::nan(""), std::nan("") };
        tempMagXYZ.at(obs->sequenceNumber % 4) = static_cast<double>(OneOfTempMagXYZ);

        obs->temperature = tempMagXYZ[0];
        obs->magUncompXYZ = Eigen::Vector3d(tempMagXYZ[1], tempMagXYZ[2], tempMagXYZ[3]);

        LOG_DATA("DATA({}): C {}, {}, {}",
                 kvhSensor->name, obs->sequenceNumber, obs->temperature, obs->status);

        // Calls all the callbacks
        kvhSensor->invokeCallbacks(obs);
    }
    else if (p.type() == kvh::protocol::uart::Packet::Type::TYPE_ASCII)
    {
        LOG_WARN("{} received an ASCII Async message: {}", kvhSensor->name, p.datastr());
    }
}

#endif