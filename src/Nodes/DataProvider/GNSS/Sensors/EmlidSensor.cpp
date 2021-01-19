#include "EmlidSensor.hpp"

#include "util/Logger.hpp"

#include "util/UartSensors/Emlid/EmlidUtilities.hpp"

#include "gui/widgets/HelpMarker.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeData/GNSS/EmlidObs.hpp"

NAV::EmlidSensor::EmlidSensor()
    : sensor(typeStatic())
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    color = ImColor(255, 128, 128);
    hasConfig = true;

    // TODO: Update the library to handle different baudrates
    selectedBaudrate = baudrate2Selection(Baudrate::BAUDRATE_9600);

    nm::CreateOutputPin(this, "EmlidObs", Pin::Type::Flow, NAV::EmlidObs::type());
}

NAV::EmlidSensor::~EmlidSensor()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::EmlidSensor::typeStatic()
{
    return "EmlidSensor";
}

std::string NAV::EmlidSensor::type() const
{
    return typeStatic();
}

std::string NAV::EmlidSensor::category()
{
    return "Data Provider";
}

void NAV::EmlidSensor::guiConfig()
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

[[nodiscard]] json NAV::EmlidSensor::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["UartSensor"] = UartSensor::save();

    return j;
}

void NAV::EmlidSensor::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("UartSensor"))
    {
        UartSensor::restore(j.at("UartSensor"));
    }
}

bool NAV::EmlidSensor::initialize()
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

void NAV::EmlidSensor::deinitialize()
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

void NAV::EmlidSensor::asciiOrBinaryAsyncMessageReceived(void* userData, uart::protocol::Packet& p, [[maybe_unused]] size_t index)
{
    auto* erSensor = static_cast<EmlidSensor*>(userData);

    auto obs = std::make_shared<EmlidObs>(p);

    sensors::emlid::decryptEmlidObs(obs, erSensor->currentInsTime);

    erSensor->invokeCallbacks(OutputPortIndex_EmlidObs, obs);
}