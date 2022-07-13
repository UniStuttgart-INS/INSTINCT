#include "UbloxSensor.hpp"

#include "util/Logger.hpp"

#include "util/Time/TimeBase.hpp"

#include "internal/gui/widgets/HelpMarker.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeData/General/UartPacket.hpp"

NAV::UbloxSensor::UbloxSensor()
    : _sensor(typeStatic())
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 360, 70 };

    // TODO: Update the library to handle different baudrates
    _selectedBaudrate = baudrate2Selection(Baudrate::BAUDRATE_9600);
    _sensorPort = "/dev/ttyACM0";

    nm::CreateOutputPin(this, "UartPacket", Pin::Type::Flow, { NAV::UartPacket::type() });
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
    if (ImGui::InputTextWithHint("SensorPort", "/dev/ttyACM0", &_sensorPort))
    {
        LOG_DEBUG("{}: SensorPort changed to {}", nameId(), _sensorPort);
        flow::ApplyChanges();
        nm::DeinitializeNode(*this);
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
        _sensor->connect(_sensorPort, sensorBaudrate());

        LOG_DEBUG("{} connected on port {} with baudrate {}", nameId(), _sensorPort, sensorBaudrate());
    }
    catch (...)
    {
        LOG_ERROR("{} could not connect", nameId());
        return false;
    }

    _sensor->registerAsyncPacketReceivedHandler(this, asciiOrBinaryAsyncMessageReceived);

    return true;
}

void NAV::UbloxSensor::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    if (getState() != State::Initialized)
    {
        return;
    }

    if (_sensor->isConnected())
    {
        try
        {
            _sensor->unregisterAsyncPacketReceivedHandler();
        }
        catch (...)
        {}

        _sensor->disconnect();
    }
}

void NAV::UbloxSensor::asciiOrBinaryAsyncMessageReceived(void* userData, uart::protocol::Packet& p, [[maybe_unused]] size_t index)
{
    auto* ubSensor = static_cast<UbloxSensor*>(userData);

    ubSensor->invokeCallbacks(OUTPUT_PORT_INDEX_UBLOX_OBS, std::make_shared<UartPacket>(p));
}