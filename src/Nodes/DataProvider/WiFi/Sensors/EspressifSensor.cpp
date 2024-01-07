// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "EspressifSensor.hpp"

#include "util/Logger.hpp"

#include "util/Time/TimeBase.hpp"

#include "internal/gui/widgets/HelpMarker.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeData/General/UartPacket.hpp"

NAV::EspressifSensor::EspressifSensor()
    : Node(typeStatic()), _sensor(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _onlyRealTime = true;
    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 360, 70 };

    // TODO: Update the library to handle different baudrates
    _selectedBaudrate = baudrate2Selection(Baudrate::BAUDRATE_9600);
    _sensorPort = "/dev/ttyACM0";

    nm::CreateOutputPin(this, "UartPacket", Pin::Type::Flow, { NAV::UartPacket::type() });
}

NAV::EspressifSensor::~EspressifSensor()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::EspressifSensor::typeStatic()
{
    return "EspressifSensor";
}

std::string NAV::EspressifSensor::type() const
{
    return typeStatic();
}

std::string NAV::EspressifSensor::category()
{
    return "Data Provider";
}

void NAV::EspressifSensor::guiConfig()
{
    if (ImGui::InputTextWithHint("SensorPort", "/dev/ttyACM0", &_sensorPort))
    {
        LOG_DEBUG("{}: SensorPort changed to {}", nameId(), _sensorPort);
        flow::ApplyChanges();
        doDeinitialize();
    }
    ImGui::SameLine();
    gui::widgets::HelpMarker("COM port where the sensor is attached to\n"
                             "- \"COM1\" (Windows format for physical and virtual (USB) serial port)\n"
                             "- \"/dev/ttyS1\" (Linux format for physical serial port)\n"
                             "- \"/dev/ttyUSB0\" (Linux format for virtual (USB) serial port)\n"
                             "- \"/dev/tty.usbserial-FTXXXXXX\" (Mac OS X format for virtual (USB) serial port)\n"
                             "- \"/dev/ttyS0\" (CYGWIN format. Usually the Windows COM port number minus 1. This would connect to COM1)");
}

[[nodiscard]] json NAV::EspressifSensor::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["UartSensor"] = UartSensor::save();

    return j;
}

void NAV::EspressifSensor::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("UartSensor"))
    {
        UartSensor::restore(j.at("UartSensor"));
    }
}

bool NAV::EspressifSensor::resetNode()
{
    return true;
}

bool NAV::EspressifSensor::initialize()
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

    _sensor->registerAsyncPacketReceivedHandler(this, binaryAsyncMessageReceived);

    return true;
}

void NAV::EspressifSensor::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    if (!isInitialized())
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

void NAV::EspressifSensor::binaryAsyncMessageReceived(void* userData, uart::protocol::Packet& p, [[maybe_unused]] size_t index)
{
    LOG_INFO("binaryAsyncMessageReceived");
    auto* ubSensor = static_cast<EspressifSensor*>(userData);

    auto obs = std::make_shared<UartPacket>(p);
    obs->insTime = util::time::GetCurrentInsTime();

    ubSensor->invokeCallbacks(OUTPUT_PORT_INDEX_Espressif_OBS, obs);
}
