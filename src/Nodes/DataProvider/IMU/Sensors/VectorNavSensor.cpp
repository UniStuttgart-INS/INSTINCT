#include "VectorNavSensor.hpp"

#include "util/Debug.hpp"
#include "util/Logger.hpp"
#include "vn/searcher.h"

#include "gui/widgets/HelpMarker.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeData/IMU/VectorNavObs.hpp"

#include "util/Time/TimeBase.hpp"

#include <map>

NAV::VectorNavSensor::VectorNavSensor()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    color = ImColor(255, 128, 128);
    hasConfig = true;

    nm::CreateOutputPin(this, "VectorNavObs", Pin::Type::Flow, NAV::VectorNavObs::type());

    dividerFrequency = []() {
        std::map<int, int, std::greater<>> divFreq;
        for (int freq = 1; freq <= IMU_DEFAULT_FREQUENCY; freq++)
        {
            int divider = static_cast<int>(std::round(IMU_DEFAULT_FREQUENCY / freq));

            if (!divFreq.contains(divider)
                || std::abs(divider - IMU_DEFAULT_FREQUENCY / freq) < std::abs(divider - IMU_DEFAULT_FREQUENCY / divFreq.at(divider)))
            {
                divFreq[divider] = freq;
            }
        }
        std::vector<uint16_t> divs;
        std::vector<std::string> freqs;
        for (auto& [divider, freq] : divFreq)
        {
            divs.push_back(static_cast<uint16_t>(divider));
            freqs.push_back(std::to_string(freq) + " Hz");
        }
        return std::make_pair(divs, freqs);
    }();
}

NAV::VectorNavSensor::~VectorNavSensor()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::VectorNavSensor::typeStatic()
{
    return "VectorNavSensor";
}

std::string NAV::VectorNavSensor::type() const
{
    return typeStatic();
}

std::string NAV::VectorNavSensor::category()
{
    return "Data Provider";
}

void NAV::VectorNavSensor::guiConfig()
{
    if (ImGui::InputTextWithHint("SensorPort", "/dev/ttyUSB0", &sensorPort))
    {
        LOG_DEBUG("{}: SensorPort changed to {}", nameId(), sensorPort);
        flow::ApplyChanges();
        deinitializeNode();
    }
    ImGui::SameLine();
    gui::widgets::HelpMarker("COM port where the sensor is attached to\n"
                             "- \"COM1\" (Windows format for physical and virtual (USB) serial port)\n"
                             "- \"/dev/ttyS1\" (Linux format for physical serial port)\n"
                             "- \"/dev/ttyUSB0\" (Linux format for virtual (USB) serial port)\n"
                             "- \"/dev/tty.usbserial-FTXXXXXX\" (Mac OS X format for virtual (USB) serial port)\n"
                             "- \"/dev/ttyS0\" (CYGWIN format. Usually the Windows COM port number minus 1. This would connect to COM1)");

    std::array<const char*, 10> items = { "Fastest", "9600", "19200", "38400", "57600", "115200", "128000", "230400", "460800", "921600" };
    if (ImGui::Combo("Baudrate", &selectedBaudrate, items.data(), items.size()))
    {
        LOG_DEBUG("{}: Baudrate changed to {}", nameId(), sensorBaudrate());
        flow::ApplyChanges();
        deinitializeNode();
    }

    const char* currentFrequency = (selectedFrequency >= 0 && static_cast<size_t>(selectedFrequency) < dividerFrequency.second.size())
                                       ? dividerFrequency.second.at(static_cast<size_t>(selectedFrequency)).c_str()
                                       : "Unknown";
    if (ImGui::SliderInt("Frequency", &selectedFrequency, 0, static_cast<int>(dividerFrequency.second.size()) - 1, currentFrequency))
    {
        LOG_DEBUG("{}: Frequency changed to {}", nameId(), dividerFrequency.second.at(static_cast<size_t>(selectedFrequency)));
        flow::ApplyChanges();
        deinitializeNode();
    }
}

[[nodiscard]] json NAV::VectorNavSensor::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["UartSensor"] = UartSensor::save();
    j["Frequency"] = dividerFrequency.second.at(static_cast<size_t>(selectedFrequency));

    return j;
}

void NAV::VectorNavSensor::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("UartSensor"))
    {
        UartSensor::restore(j.at("UartSensor"));
    }
    if (j.contains("Frequency"))
    {
        std::string frequency;
        j.at("Frequency").get_to(frequency);
        for (size_t i = 0; i < dividerFrequency.second.size(); i++)
        {
            if (dividerFrequency.second.at(i) == frequency)
            {
                selectedFrequency = static_cast<int>(i);
                break;
            }
        }
    }
}

bool NAV::VectorNavSensor::resetNode()
{
    return true;
}

bool NAV::VectorNavSensor::initialize()
{
    LOG_TRACE("{}: called", nameId());

    // Choose baudrate
    Baudrate targetBaudrate = sensorBaudrate() == BAUDRATE_FASTEST
                                  ? static_cast<Baudrate>(vn::sensors::VnSensor::supportedBaudrates()[vn::sensors::VnSensor::supportedBaudrates().size() - 1])
                                  : sensorBaudrate();

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
        if (foundSensors.size() == 1)
        {
            sensorPort = foundSensors.at(0).first;
            connectedBaudrate = static_cast<Baudrate>(foundSensors.at(0).second);
        }
        else
        {
            sensorPort = "";
            // Some VectorNav sensors where found, try to identify the wanted one by it's name
            for (auto [port, baudrate] : foundSensors)
            {
                vs.connect(port, baudrate);
                std::string modelNumber = vs.readModelNumber();
                vs.disconnect();

                LOG_DEBUG("{} found VectorNav Sensor {} on port {} with baudrate {}", nameId(), modelNumber, port, baudrate);

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
                LOG_ERROR("{} could not connect", nameId());
                return false;
            }
        }
    }
    else
    {
        LOG_ERROR("{} could not connect. Is the sensor connected and do you have read permissions?", nameId());
        return false;
    }

    try
    {
        // Connect to the sensor (vs.verifySensorConnectivity does not have to be called as sensor is already tested)
        vs.connect(sensorPort, connectedBaudrate);
    }
    catch (const std::exception& e)
    {
        LOG_ERROR("{}: Failed to connect to sensor on port {} with baudrate {} with error: {}", nameId(),
                  sensorPort, connectedBaudrate, e.what());
        return false;
    }

    if (!vs.verifySensorConnectivity())
    {
        LOG_ERROR("{}: Connected to sensor on port {} with baudrate {} but sensor does not answer", nameId(),
                  sensorPort, connectedBaudrate);
        return false;
    }
    // Query the sensor's model number
    LOG_DEBUG("{} connected on port {} with baudrate {}", vs.readModelNumber(), sensorPort, connectedBaudrate);

    // Change Connection Baudrate
    if (targetBaudrate != connectedBaudrate)
    {
        auto suppBaud = vn::sensors::VnSensor::supportedBaudrates();
        if (std::find(suppBaud.begin(), suppBaud.end(), targetBaudrate) != suppBaud.end())
        {
            vs.changeBaudRate(targetBaudrate);
            LOG_DEBUG("{} baudrate changed to {}", nameId(), static_cast<size_t>(targetBaudrate));
        }
        else
        {
            LOG_ERROR("{} does not support baudrate {}", nameId(), static_cast<size_t>(targetBaudrate));
            return false;
        }
    }
    ASSERT(vs.readSerialBaudRate() == targetBaudrate, "Baudrate was not changed");

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
                                          dividerFrequency.first.at(static_cast<size_t>(selectedFrequency)),
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
        LOG_ERROR("{} could not configure binary output register ({})", nameId(), e.what());
        deinitializeNode();
        return false;
    }

    vs.registerAsyncPacketReceivedHandler(this, asciiOrBinaryAsyncMessageReceived);

    LOG_DEBUG("{} successfully initialized", nameId());

    return true;
}

void NAV::VectorNavSensor::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    if (!isInitialized())
    {
        return;
    }

    if (vs.isConnected())
    {
        try
        {
            vs.unregisterAsyncPacketReceivedHandler();
        }
        catch (...)
        {}
        try
        {
            vs.reset(true);
        }
        catch (...)
        {}
        try
        {
            vs.disconnect();
        }
        catch (...)
        {}
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
            auto obs = std::make_shared<VectorNavObs>(vnSensor->imuPos);

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
                     vnSensor->nameId(), obs->timeSinceStartup.value(), obs->syncInCnt.value(), obs->timeSinceSyncIn.value(),
                     obs->vpeStatus.value().status, obs->temperature.value());

            // Calls all the callbacks
            if (InsTime currentTime = util::time::GetCurrentTime();
                !currentTime.empty())
            {
                obs->insTime = currentTime;
            }
            vnSensor->invokeCallbacks(VectorNavSensor::OutputPortIndex_VectorNavObs, obs);
        }
        else if (p.type() == vn::protocol::uart::Packet::TYPE_ASCII)
        {
            LOG_WARN("{} received an ASCII Async message: {}", vnSensor->nameId(), p.datastr());
        }
    }
}