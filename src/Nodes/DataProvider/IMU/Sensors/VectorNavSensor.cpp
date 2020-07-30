#ifndef DISABLE_SENSORS

    #include "VectorNavSensor.hpp"

    #include "util/Debug.hpp"
    #include "util/Logger.hpp"
    #include "vn/searcher.h"

NAV::VectorNavSensor::VectorNavSensor(const std::string& name, const std::map<std::string, std::string>& options)
    : UartSensor(options), Imu(name, options)
{
    LOG_TRACE("called for {}", name);

    if (options.count("Frequency"))
    {
        config.outputFrequency = static_cast<uint16_t>(std::stoul(options.at("Frequency")));
    }

    ASSERT(config.outputFrequency <= IMU_DEFAULT_FREQUENCY, "Configured Output Frequency has to be less than IMU_DEFAULT_FREQUENCY");

    // connect to the sensor
    if (sensorBaudrate == BAUDRATE_FASTEST)
    {
        sensorBaudrate = static_cast<Baudrate>(vn::sensors::VnSensor::supportedBaudrates()[vn::sensors::VnSensor::supportedBaudrates().size() - 1]);
    }

    Baudrate connectedBaudrate{};

    // Search for the VectorNav Sensor
    if (int32_t foundBaudrate = 0;
        vn::sensors::Searcher::search(sensorPort, &foundBaudrate))
    {
        // Sensor was found at specified port with the baudrate 'foundBaudrate'
        connectedBaudrate = static_cast<Baudrate>(foundBaudrate);
    }
    else if (std::vector<std::pair<std::string, uint32_t>> foundSensors = vn::sensors::Searcher::search();
             !foundSensors.empty())
    {
        sensorPort = "";
        // Some VectorNav sensors where found, try to identify the wanted one by it's name
        for (auto [port, baudrate] : foundSensors)
        {
            vs.connect(port, baudrate);
            std::string modelNumber = vs.readModelNumber();
            vs.disconnect();

            LOG_DEBUG("{} found VectorNav Sensor {} on port {} with baudrate {}", name, modelNumber, port, baudrate);

            // Regex search may be better, but simple find is used here
            if (modelNumber.find(name) != std::string::npos)
            {
                sensorPort = port;
                connectedBaudrate = static_cast<Baudrate>(baudrate);
                break;
            }
        }
        // Sensor could not be identified
        if (sensorPort.empty())
        {
            // This point is also reached if a sensor is connected with USB but external power is off
            LOG_CRITICAL("{} could not connect", name);
            throw std::runtime_error(name + " could not connect");
        }
    }
    else
    {
        LOG_CRITICAL("{} could not connect", name);
        throw std::runtime_error(name + " could not connect");
    }

    // Connect to the sensor (vs.verifySensorConnectivity does not have to be called as sensor is already tested)
    vs.connect(sensorPort, connectedBaudrate);

    // Query the sensor's model number
    LOG_DEBUG("{} connected on port {} with baudrate {}", vs.readModelNumber(), sensorPort, connectedBaudrate);

    // Change Connection Baudrate
    if (sensorBaudrate != connectedBaudrate)
    {
        auto suppBaud = vn::sensors::VnSensor::supportedBaudrates();
        if (std::find(suppBaud.begin(), suppBaud.end(), sensorBaudrate) != suppBaud.end())
        {
            vs.changeBaudRate(sensorBaudrate);
            LOG_DEBUG("{} baudrate changed to {}", name, sensorBaudrate);
        }
        else
        {
            LOG_CRITICAL("{} does not support baudrate {}", name, sensorBaudrate);
            throw std::runtime_error(fmt::format("{} does not support baudrate {}", name, sensorBaudrate));
        }
    }
    ASSERT(vs.readSerialBaudRate() == sensorBaudrate, "Baudrate was not changed");

    // Change Heading Mode (and enable Filtering Mode, Tuning Mode)
    vn::sensors::VpeBasicControlRegister vpeReg = vs.readVpeBasicControl();
    vpeReg.headingMode = config.headingMode;
    vs.writeVpeBasicControl(vpeReg);
    ASSERT(vs.readVpeBasicControl().headingMode == config.headingMode, "Heading Mode was not changed");

    vn::sensors::DeltaThetaAndDeltaVelocityConfigurationRegister dtdvConfReg(config.delThetaDelVeloIntegrationFrame,
                                                                             config.delThetaDelVeloGyroCompensation,
                                                                             config.delThetaDelVeloAccelCompensation);
    vs.writeDeltaThetaAndDeltaVelocityConfiguration(dtdvConfReg);
    ASSERT(vs.readDeltaThetaAndDeltaVelocityConfiguration().integrationFrame == config.delThetaDelVeloIntegrationFrame, "Integration Frame was not changed");
    ASSERT(vs.readDeltaThetaAndDeltaVelocityConfiguration().gyroCompensation == config.delThetaDelVeloGyroCompensation, "Gyro Compensation was not changed");
    ASSERT(vs.readDeltaThetaAndDeltaVelocityConfiguration().accelCompensation == config.delThetaDelVeloAccelCompensation, "Acceleration Compensation was not changed");

    // Stop the AsciiAsync messages
    vs.writeAsyncDataOutputType(vn::protocol::uart::AsciiAsync::VNOFF);

    // Configure Binary Output 1
    vn::sensors::BinaryOutputRegister bor(config.asyncMode,
                                          static_cast<uint16_t>(IMU_DEFAULT_FREQUENCY / config.outputFrequency),
                                          config.commonField,
                                          config.timeField,
                                          config.imuField,
                                          vn::protocol::uart::GpsGroup::GPSGROUP_NONE,
                                          config.attitudeField,
                                          vn::protocol::uart::InsGroup::INSGROUP_NONE,
                                          vn::protocol::uart::GpsGroup::GPSGROUP_NONE);
    try
    {
        vs.writeBinaryOutput1(bor);
    }
    catch (const std::exception& e)
    {
        LOG_CRITICAL("{} could not configure binary output register ({})", name, e.what());
    }

    vs.registerAsyncPacketReceivedHandler(this, asciiOrBinaryAsyncMessageReceived);

    LOG_DEBUG("{} successfully initialized", name);
}

NAV::VectorNavSensor::~VectorNavSensor()
{
    LOG_TRACE("called for {}", name);

    removeAllCallbacksOfType<VectorNavObs>();
    callbacksEnabled = false;
    if (vs.isConnected())
    {
        vs.unregisterAsyncPacketReceivedHandler();
        vs.reset(true);
        vs.disconnect();
    }
}

void NAV::VectorNavSensor::asciiOrBinaryAsyncMessageReceived(void* userData, vn::protocol::uart::Packet& p, [[maybe_unused]] size_t index)
{
    auto* vnSensor = static_cast<VectorNavSensor*>(userData);

    if (p.type() == vn::protocol::uart::Packet::TYPE_BINARY)
    {
        // Make sure that the binary packet is from the type we expect
        if (p.isCompatible(vnSensor->config.commonField,
                           vnSensor->config.timeField,
                           vnSensor->config.imuField,
                           vn::protocol::uart::GpsGroup::GPSGROUP_NONE,
                           vnSensor->config.attitudeField,
                           vn::protocol::uart::InsGroup::INSGROUP_NONE,
                           vn::protocol::uart::GpsGroup::GPSGROUP_NONE))
        {
            auto obs = std::make_shared<VectorNavObs>();

            // Group 1 (Common)
            obs->timeSinceStartup.emplace(p.extractUint64());
            obs->timeSinceSyncIn.emplace(p.extractUint64());
            obs->dtime.emplace(p.extractFloat());
            auto dtheta = p.extractVec3f();
            obs->dtheta.emplace(dtheta.x, dtheta.y, dtheta.z);
            auto dvel = p.extractVec3f();
            obs->dvel.emplace(dvel.x, dvel.y, dvel.z);
            obs->syncInCnt.emplace(p.extractUint32());
            // Group 2 (Time)
            // Group 3 (IMU)
            auto magUncompXYZ = p.extractVec3f();
            obs->magUncompXYZ.emplace(magUncompXYZ.x, magUncompXYZ.y, magUncompXYZ.z);
            auto accelUncompXYZ = p.extractVec3f();
            obs->accelUncompXYZ.emplace(accelUncompXYZ.x, accelUncompXYZ.y, accelUncompXYZ.z);
            auto gyroUncompXYZ = p.extractVec3f();
            obs->gyroUncompXYZ.emplace(gyroUncompXYZ.x, gyroUncompXYZ.y, gyroUncompXYZ.z);
            obs->temperature.emplace(p.extractFloat());
            obs->pressure.emplace(p.extractFloat());
            auto magCompXYZ = p.extractVec3f();
            obs->magCompXYZ.emplace(magCompXYZ.x, magCompXYZ.y, magCompXYZ.z);
            auto accelCompXYZ = p.extractVec3f();
            obs->accelCompXYZ.emplace(accelCompXYZ.x, accelCompXYZ.y, accelCompXYZ.z);
            auto gyroCompXYZ = p.extractVec3f();
            obs->gyroCompXYZ.emplace(gyroCompXYZ.x, gyroCompXYZ.y, gyroCompXYZ.z);
            // Group 4 (GPS)
            // Group 5 (Attitude)
            obs->vpeStatus.emplace(p.extractUint16());
            auto yawPitchRoll = p.extractVec3f();
            obs->yawPitchRoll.emplace(yawPitchRoll.x, yawPitchRoll.y, yawPitchRoll.z);
            auto quaternion = p.extractVec4f();
            obs->quaternion.emplace(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
            auto magCompNED = p.extractVec3f();
            obs->magCompNED.emplace(magCompNED.x, magCompNED.y, magCompNED.z);
            auto accelCompNED = p.extractVec3f();
            obs->accelCompNED.emplace(accelCompNED.x, accelCompNED.y, accelCompNED.z);
            auto linearAccelXYZ = p.extractVec3f();
            obs->linearAccelXYZ.emplace(linearAccelXYZ.x, linearAccelXYZ.y, linearAccelXYZ.z);
            auto linearAccelNED = p.extractVec3f();
            obs->linearAccelNED.emplace(linearAccelNED.x, linearAccelNED.y, linearAccelNED.z);
            auto yawPitchRollUncertainty = p.extractVec3f();
            obs->yawPitchRollUncertainty.emplace(yawPitchRollUncertainty.x, yawPitchRollUncertainty.y, yawPitchRollUncertainty.z);

            LOG_DATA("DATA({}): {}, {}, {}, {}, {}",
                     vnSensor->name, obs->timeSinceStartup.value(), obs->syncInCnt.value(), obs->timeSinceSyncIn.value(),
                     obs->vpeStatus.value().status, obs->temperature.value());

            // Calls all the callbacks
            vnSensor->invokeCallbacks(obs);
        }
        else if (p.type() == vn::protocol::uart::Packet::TYPE_ASCII)
        {
            LOG_WARN("{} received an ASCII Async message: {}", vnSensor->name, p.datastr());
        }
    }
}

#endif