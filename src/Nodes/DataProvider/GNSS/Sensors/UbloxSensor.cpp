#include "UbloxSensor.hpp"

#include "util/Logger.hpp"

#include "util/Time/TimeBase.hpp"
#include "util/UartSensors/Ublox/UbloxUtilities.hpp"

#include "internal/gui/widgets/HelpMarker.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeData/GNSS/UbloxObs.hpp"

NAV::UbloxSensor::UbloxSensor()
    : sensor(typeStatic())
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    hasConfig = true;
    guiConfigDefaultWindowSize = { 360, 70 };

    // TODO: Update the library to handle different baudrates
    selectedBaudrate = baudrate2Selection(Baudrate::BAUDRATE_9600);
    sensorPort = "/dev/ttyACM0";

    nm::CreateOutputPin(this, "UbloxObs", Pin::Type::Flow, NAV::UbloxObs::type());
}

NAV::UbloxSensor::~UbloxSensor()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::UbloxSensor::typeStatic()
{
    return "UbloxSensor";
}

std::string NAV::UbloxSensor::type() const
{
    return typeStatic();
}

std::string NAV::UbloxSensor::category()
{
    return "Data Provider";
}

void NAV::UbloxSensor::guiConfig()
{
    if (ImGui::InputTextWithHint("SensorPort", "/dev/ttyACM0", &sensorPort))
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
}

[[nodiscard]] json NAV::UbloxSensor::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["UartSensor"] = UartSensor::save();

    return j;
}

void NAV::UbloxSensor::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("UartSensor"))
    {
        UartSensor::restore(j.at("UartSensor"));
    }
}

bool NAV::UbloxSensor::resetNode()
{
    return true;
}

bool NAV::UbloxSensor::initialize()
{
    LOG_TRACE("{}: called", nameId());

    // connect to the sensor
    try
    {
        sensor->connect(sensorPort, sensorBaudrate());

        LOG_DEBUG("{} connected on port {} with baudrate {}", nameId(), sensorPort, sensorBaudrate());
    }
    catch (...)
    {
        LOG_ERROR("{} could not connect", nameId());
        return false;
    }

    sensor->registerAsyncPacketReceivedHandler(this, asciiOrBinaryAsyncMessageReceived);

    return true;
}

void NAV::UbloxSensor::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    if (!isInitialized())
    {
        return;
    }

    if (sensor->isConnected())
    {
        try
        {
            sensor->unregisterAsyncPacketReceivedHandler();
        }
        catch (...)
        {}

        sensor->disconnect();
    }
}

void NAV::UbloxSensor::asciiOrBinaryAsyncMessageReceived(void* userData, uart::protocol::Packet& p, [[maybe_unused]] size_t index)
{
    auto* ubSensor = static_cast<UbloxSensor*>(userData);

    auto obs = std::make_shared<UbloxObs>(p);

    sensors::ublox::decryptUbloxObs(obs);

    if (obs->insTime.has_value())
    {
        if (util::time::GetMode() == util::time::Mode::REAL_TIME)
        {
            util::time::SetCurrentTime(obs->insTime.value());
        }
    }
    else if (auto currentTime = util::time::GetCurrentInsTime();
             !currentTime.empty())
    {
        obs->insTime = currentTime;
    }

    ubSensor->invokeCallbacks(OutputPortIndex_UbloxObs, obs);
}