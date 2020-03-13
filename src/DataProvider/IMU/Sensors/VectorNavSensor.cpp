#include "VectorNavSensor.hpp"

#include "../Observations/VectorNavObs.hpp"
#include "util/Logger.hpp"
#include "vn/searcher.h"

#include <string>
#include <vector>

NAV::VectorNavSensor::VectorNavSensor(std::string name, const VNConfig sensorConfig)
    : Imu(name), UartSensor(sensorConfig.sensorPort, sensorConfig.sensorBaudrate), config(sensorConfig)
{
    LOG_TRACE("called for {}", name);
}

NAV::VectorNavSensor::~VectorNavSensor()
{
    LOG_TRACE("called for {}", name);
    deinitialize();
}

NAV::NavStatus NAV::VectorNavSensor::initialize()
{
    LOG_TRACE("called for {})", name);
    if (initialized)
    {
        LOG_WARN("{} already initialized!!!", name);
        return NavStatus::NAV_WARNING_ALREADY_INITIALIZED;
    }

    ASSERT(config.outputFrequency <= IMU_DEFAULT_FREQUENCY, "Configured Output Frequency has to be less than IMU_DEFAULT_FREQUENCY");

    // connect to the sensor
    if (config.sensorBaudrate == BAUDRATE_FASTEST)
        sensorBaudrate = static_cast<Baudrate>(vs.supportedBaudrates()[vs.supportedBaudrates().size() - 1]);

    Baudrate connectedBaudrate;

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
            LOG_ERROR("{} could not connect", name);
            return NavStatus::NAV_ERROR_COULD_NOT_CONNECT;
        }
    }
    else
    {
        LOG_ERROR("{} could not connect", name);
        return NavStatus::NAV_ERROR_COULD_NOT_CONNECT;
    }

    // Connect to the sensor (vs.verifySensorConnectivity does not have to be called as sensor is already tested)
    vs.connect(sensorPort, connectedBaudrate);

    // Query the sensor's model number
    name = vs.readModelNumber();
    LOG_DEBUG("{} connected on port {} with baudrate {}", name, sensorPort, connectedBaudrate);

    // Change Connection Baudrate
    if (sensorBaudrate != connectedBaudrate)
    {
        auto suppBaud = vs.supportedBaudrates();
        if (std::find(suppBaud.begin(), suppBaud.end(), sensorBaudrate) != suppBaud.end())
        {
            vs.changeBaudRate(sensorBaudrate);
            LOG_DEBUG("{} baudrate changed to {}", name, sensorBaudrate);
        }
        else
        {
            LOG_ERROR("{} does not support baudrate {}", name, sensorBaudrate);
            return NavStatus::NAV_ERROR_CONFIGURATION_FAULT;
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
        LOG_ERROR("{} could not configure binary output register ({})", name, e.what());
        return NavStatus::NAV_ERROR_CONFIGURATION_FAULT;
    }

    vs.registerAsyncPacketReceivedHandler(this, asciiOrBinaryAsyncMessageReceived);

    LOG_INFO("{} successfully initialized", name);

    // TODO: USB Pin reset here???
    // USBHelper::resetRTSPin(sensorPort);

    initialized = true;

    return NavStatus::NAV_OK;
}

NAV::NavStatus NAV::VectorNavSensor::deinitialize()
{
    LOG_TRACE("called for {}", name);
    if (initialized)
    {
        removeAllObservationReceivedCallbacks();
        vs.unregisterAsyncPacketReceivedHandler();
        if (vs.isConnected())
        {
            vs.reset(true);
            vs.disconnect();
        }
        initialized = false;
        LOG_DEBUG("{} successfully deinitialized", name);
        return NAV_OK;
    }
    LOG_DEBUG("{} should be deinitialized but was not initialized before", name);

    return NAV_WARNING_NOT_INITIALIZED;
}

std::shared_ptr<NAV::InsObs> NAV::VectorNavSensor::pollObservation()
{
    LOG_TRACE("called for {}", name);
    auto obs = std::make_shared<VectorNavObs>();

    // Group 1 (Common)
    auto deltaReg = vs.readDeltaThetaAndDeltaVelocity();
    obs->dtime = deltaReg.deltaTime;
    obs->dtheta = deltaReg.deltaTheta;
    obs->dvel = deltaReg.deltaVelocity;
    // Group 2 (Time)
    // Group 3 (IMU)
    auto imuReg = vs.readImuMeasurements();
    obs->magUncompXYZ = imuReg.mag;
    obs->accelUncompXYZ = imuReg.accel;
    obs->gyroUncompXYZ = imuReg.gyro;
    obs->temperature = imuReg.temp;
    obs->pressure = imuReg.pressure;
    auto qmag = vs.readQuaternionMagneticAccelerationAndAngularRates();
    obs->magCompXYZ = qmag.mag;
    obs->accelCompXYZ = qmag.accel;
    obs->gyroCompXYZ = qmag.gyro;
    // Group 4 (GPS)
    // Group 5 (Attitude)
    obs->quaternion = qmag.quat;

    LOG_TRACE("DATA({}): {}, {}, {}, {}, {}, {}, {}",
              name, obs->timeSinceStartup.value(), obs->syncInCnt.value(), obs->timeSinceSyncIn.value(),
              obs->vpeStatus.value(), obs->temperature.value(), obs->pressure.value(), obs->quaternion.value());

    // Calls all the callbacks
    observationReceived(obs);

    return obs;
}

void NAV::VectorNavSensor::asciiOrBinaryAsyncMessageReceived(void* userData, vn::protocol::uart::Packet& p, size_t /*index*/)
{
    VectorNavSensor* vnSensor = static_cast<VectorNavSensor*>(userData);
    LOG_TRACE("called for {}", vnSensor->name);

    if (!vnSensor->initialized)
        return;

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
            obs->timeSinceStartup = p.extractUint64();
            obs->timeSinceSyncIn = p.extractUint64();
            obs->dtime = p.extractFloat();
            obs->dtheta = p.extractVec3f();
            obs->dvel = p.extractVec3f();
            obs->syncInCnt = p.extractUint32();
            // Group 2 (Time)
            // Group 3 (IMU)
            obs->magUncompXYZ = p.extractVec3f();
            obs->accelUncompXYZ = p.extractVec3f();
            obs->gyroUncompXYZ = p.extractVec3f();
            obs->temperature = p.extractFloat();
            obs->pressure = p.extractFloat();
            obs->magCompXYZ = p.extractVec3f();
            obs->accelCompXYZ = p.extractVec3f();
            obs->gyroCompXYZ = p.extractVec3f();
            // Group 4 (GPS)
            // Group 5 (Attitude)
            obs->vpeStatus = p.extractUint16();
            obs->quaternion = p.extractVec4f();
            obs->magCompNED = p.extractVec3f();
            obs->accelCompNED = p.extractVec3f();
            obs->linearAccelXYZ = p.extractVec3f();
            obs->linearAccelNED = p.extractVec3f();
            obs->yawPitchRollUncertainty = p.extractVec3f();

            LOG_TRACE("DATA({}): {}, {}, {}, {}, {}, {}, {}",
                      vnSensor->name, obs->timeSinceStartup.value(), obs->syncInCnt.value(), obs->timeSinceSyncIn.value(),
                      obs->vpeStatus.value(), obs->temperature.value(), obs->pressure.value(), obs->quaternion.value());

            // Calls all the callbacks
            vnSensor->observationReceived(obs);
        }
        else if (p.type() == vn::protocol::uart::Packet::TYPE_ASCII)
        {
            LOG_WARN("{} received an ASCII Async message: {}", vnSensor->name, p.datastr());
        }
    }
}