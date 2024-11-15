// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include <memory>
#if __linux__ || __APPLE__

    #include "mavlinkSend.hpp"
    #include <array>

    #include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
    #include "internal/FlowManager.hpp"

    #include "internal/gui/widgets/imgui_ex.hpp"
    #include "internal/gui/widgets/HelpMarker.hpp"
    #include "internal/gui/NodeEditorApplication.hpp"

    #include "mavlink/common/mavlink.h"
    #include "util/Vendor/MavLink/autopilot_interface.hpp"

    #include "util/Logger.hpp"

NAV::MavlinkSend::MavlinkSend()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);
    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 479.0, 197.0 };

    nm::CreateInputPin(this, "PosVelAtt", Pin::Type::Flow, { NAV::PosVelAtt::type() }, &MavlinkSend::receivePosVelAtt);
}

NAV::MavlinkSend::~MavlinkSend()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::MavlinkSend::typeStatic()
{
    return "MavlinkSend";
}

std::string NAV::MavlinkSend::type() const
{
    return typeStatic();
}

std::string NAV::MavlinkSend::category()
{
    return "Data Link";
}

void NAV::MavlinkSend::guiConfig()
{
    float columnWidth = 140.0F * gui::NodeEditorApplication::windowFontRatio();
    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    if (ImGui::TreeNode("Transmission protocol"))
    {
        if (ImGui::RadioButton(fmt::format("UDP-Port##{}", size_t(id)).c_str(), reinterpret_cast<int*>(&_portType), static_cast<int>(PortType::UDP_Port)))
        {
            LOG_DEBUG("{}: portType changed to {}", nameId(), fmt::underlying(_portType));
            flow::ApplyChanges();
        }
        if (_portType == PortType::UDP_Port)
        {
            ImGui::Indent();

            ImGui::SetNextItemWidth(150 * gui::NodeEditorApplication::windowFontRatio());
            if (ImGui::InputInt4L(fmt::format("IPv4##{}", size_t(id)).c_str(), _ip.data(), IP_LIMITS[0], IP_LIMITS[1]))
            {
                flow::ApplyChanges();
            }
            ImGui::SetNextItemWidth(150 * gui::NodeEditorApplication::windowFontRatio());
            if (ImGui::InputIntL(fmt::format("Port##{}", size_t(id)).c_str(), &_portNumber, PORT_LIMITS[0], PORT_LIMITS[1]))
            {
                flow::ApplyChanges();
            }

            ImGui::Unindent();
        }

        if (ImGui::RadioButton(fmt::format("Serial-Port##{}", size_t(id)).c_str(), reinterpret_cast<int*>(&_portType), static_cast<int>(PortType::Serial_Port)))
        {
            LOG_DEBUG("{}: portType changed to {}", nameId(), fmt::underlying(_portType));
            flow::ApplyChanges();
        }
        if (_portType == PortType::Serial_Port)
        {
            ImGui::Indent();

            ImGui::SetNextItemWidth(300 * gui::NodeEditorApplication::windowFontRatio());
            if (ImGui::InputTextWithHint("MavLinkPort", "/dev/ttyACM0", &_serialPort))
            {
                LOG_DEBUG("{}: MavLinkPort changed to {}", nameId(), _serialPort);
                flow::ApplyChanges();
                doDeinitialize();
            }
            ImGui::SameLine();
            gui::widgets::HelpMarker("COM port where the MavLink device is attached to\n"
                                     "- \"COM1\" (Windows format for physical and virtual (USB) serial port)\n"
                                     "- \"/dev/ttyS1\" (Linux format for physical serial port)\n"
                                     "- \"/dev/ttyUSB0\" (Linux format for virtual (USB) serial port)\n"
                                     "- \"/dev/tty.usbserial-FTXXXXXX\" (Mac OS X format for virtual (USB) serial port)\n"
                                     "- \"/dev/ttyS0\" (CYGWIN format. Usually the Windows COM port number minus 1. This would connect to COM1)");

            ImGui::SetNextItemWidth(150 * gui::NodeEditorApplication::windowFontRatio());
            if (ImGui::BeginCombo(fmt::format("Baudrate##{}", size_t(id)).c_str(), to_string(_baudrate)))
            {
                for (size_t i = 0; i < static_cast<size_t>(Baudrate::COUNT); i++)
                {
                    const bool is_selected = (static_cast<size_t>(_baudrate) == i);
                    if (ImGui::Selectable(to_string(static_cast<Baudrate>(i)), is_selected))
                    {
                        _baudrate = static_cast<Baudrate>(i);
                        LOG_DEBUG("{}: Baudrate changed to {}", nameId(), fmt::underlying(_baudrate));
                        flow::ApplyChanges();
                        doDeinitialize();
                    }

                    if (is_selected) // Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
                    {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndCombo();
            }
            ImGui::Unindent();
        }
        if (NAV::MavlinkSend::isInitialized())
        {
            doDeinitialize();
        }
        ImGui::TreePop();
    }
    if (ImGui::TreeNode("MavLink Messages"))
    {
        if (ImGui::Checkbox(fmt::format("Send GPS_INPUT (#232) ##{}", size_t(id)).c_str(), &_GPS_INPUT_Active))
        {
            flow::ApplyChanges();
            LOG_DEBUG("{}: GPS_INPUT changed to {}", nameId(), _GPS_INPUT_Active);
            autopilot_interface.setGPS_Active(_GPS_INPUT_Active);
        }
        if (_GPS_INPUT_Active)
        {
            ImGui::Indent();
            ImGui::SetNextItemWidth(columnWidth);
            if (ImGui::InputDoubleL(fmt::format("GPS output rate##{}", size_t(id)).c_str(), &_GPS_INPUT_Frequency, 0, 100, 0, 0, "%.3f Hz"))
            {
                LOG_DEBUG("{}: GPS_INPUT_Frequency changed to {}", nameId(), _GPS_INPUT_Frequency);
                autopilot_interface.setGPS_Frequency(_GPS_INPUT_Frequency);
                flow::ApplyChanges();
            }
            ImGui::Unindent();
        }

        if (ImGui::Checkbox(fmt::format("Send ATT_POS_MOCAP (#138) ##{}", size_t(id)).c_str(), &_ATT_POS_MOCAP_Active))
        {
            flow::ApplyChanges();
            LOG_DEBUG("{}: ATT_POS_MOCAP changed to {}", nameId(), _ATT_POS_MOCAP_Active);
            autopilot_interface.setMOCAP_Active(_ATT_POS_MOCAP_Active);
        }
        if (_ATT_POS_MOCAP_Active)
        {
            ImGui::Indent();
            ImGui::SetNextItemWidth(columnWidth);
            if (ImGui::InputDoubleL(fmt::format("MOCAP output rate##{}", size_t(id)).c_str(), &_ATT_POS_MOCAP_Frequency, 0, 10000, 0, 0, "%.3f Hz"))
            {
                LOG_DEBUG("{}: ATT_POS_MOCAP_Frequency changed to {}", nameId(), _ATT_POS_MOCAP_Frequency);
                autopilot_interface.setMOCAP_Frequency(_ATT_POS_MOCAP_Frequency);
                flow::ApplyChanges();
            }
            ImGui::Unindent();
        }
        ImGui::TreePop();
    }
}

bool NAV::MavlinkSend::resetNode()
{
    return true;
}

json NAV::MavlinkSend::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["portType"] = _portType;

    j["ip"] = _ip;
    j["portNumber"] = _portNumber;

    j["serialPort"] = _serialPort;
    j["baudrate"] = _baudrate;

    j["GPS_INPUT_Active"] = _GPS_INPUT_Active;
    j["GPS_INPUT_Frequency"] = _GPS_INPUT_Frequency;

    j["ATT_POS_MOCAP_Active"] = _ATT_POS_MOCAP_Active;
    j["ATT_POS_MOCAP_Frequency"] = _ATT_POS_MOCAP_Frequency;
    return j;
}

void NAV::MavlinkSend::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("portType"))
    {
        j.at("portType").get_to(_portType);
    }

    if (j.contains("ip"))
    {
        j.at("ip").get_to(_ip);
    }
    if (j.contains("portNumber"))
    {
        j.at("portNumber").get_to(_portNumber);
    }

    if (j.contains("serialPort"))
    {
        j.at("serialPort").get_to(_serialPort);
    }
    if (j.contains("baudrate"))
    {
        j.at("baudrate").get_to(_baudrate);
    }

    if (j.contains("GPS_INPUT_Active"))
    {
        j.at("GPS_INPUT_Active").get_to(_GPS_INPUT_Active);
    }
    if (j.contains("GPS_INPUT_Frequency"))
    {
        j.at("GPS_INPUT_Frequency").get_to(_GPS_INPUT_Frequency);
    }

    if (j.contains("ATT_POS_MOCAP_Active"))
    {
        j.at("ATT_POS_MOCAP_Active").get_to(_ATT_POS_MOCAP_Active);
    }
    if (j.contains("ATT_POS_MOCAP_Frequency"))
    {
        j.at("ATT_POS_MOCAP_Frequency").get_to(_ATT_POS_MOCAP_Frequency);
    }
}

bool NAV::MavlinkSend::initialize()
{
    if (_portType == PortType::UDP_Port)
    {
        port = std::make_shared<UDP_Port>(convertArrayToIPAddress(_ip).c_str(), _portNumber);
    }
    if (_portType == PortType::Serial_Port)
    {
        port = std::make_shared<Serial_Port>(_serialPort.c_str(), getBaudrateValue(_baudrate));
    }
    autopilot_interface.setPort(port);
    autopilot_interface.setGPS_Active(_GPS_INPUT_Active);
    autopilot_interface.setMOCAP_Active(_ATT_POS_MOCAP_Active);
    autopilot_interface.setGPS_Frequency(_GPS_INPUT_Frequency);
    autopilot_interface.setMOCAP_Frequency(_ATT_POS_MOCAP_Frequency);
    try
    {
        port->start(); // Open Port
    }
    catch (...)
    {
        LOG_ERROR("{} could not connect", nameId());
        return false;
    }
    try
    {
        autopilot_interface.start();
    }
    catch (...)
    {
        port->stop(); // Close Port
        LOG_ERROR("{} could not start autopilot_interface", nameId());
        return false;
    }

    return true;
}

void NAV::MavlinkSend::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
    autopilot_interface.stop();
    port->stop(); // Close Port
}

void NAV::MavlinkSend::receivePosVelAtt(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */) // NOLINT(readability-convert-member-functions-to-static)
{
    // de-initialize node if cable is pulled
    if (port->cabelCheck == 1)
    {
        doDeinitialize();
    }
    auto posVelAtt = std::make_shared<PosVelAtt>(*std::static_pointer_cast<const PosVelAtt>(queue.extract_front()));

    // Mocap Message
    auto quat = posVelAtt->n_Quat_b();
    auto w = static_cast<float>(quat.w());
    auto x = static_cast<float>(quat.x());
    auto y = static_cast<float>(quat.y());
    auto z = static_cast<float>(quat.z());
    autopilot_interface.setMOCAP(w, x, y, z);

    // GPS Message
    auto lat_d = static_cast<int32_t>(((posVelAtt->latitude()) * 180.0 / std::numbers::pi_v<double>)*10000000);  // Latitude (WGS84)  (degE7)
    auto lon_d = static_cast<int32_t>(((posVelAtt->longitude()) * 180.0 / std::numbers::pi_v<double>)*10000000); // Longitude (WGS84) (degE7)
    auto alt = static_cast<float>(posVelAtt->altitude());                                                        // Altitude (MSL)  Positive for up (m)
    auto time_week = static_cast<uint16_t>(posVelAtt->insTime.toGPSweekTow().gpsWeek);                           // GPS week number
    auto time_week_ms = static_cast<uint32_t>((posVelAtt->insTime.toGPSweekTow().tow) / 1000);                   // GPS time (from start of GPS week)

    auto vn = static_cast<float>(posVelAtt->n_velocity().x()); // GPS velocity in north direction in earth-fixed NED frame
    auto ve = static_cast<float>(posVelAtt->n_velocity().y()); // GPS velocity in east direction in earth-fixed NED frame
    auto vd = static_cast<float>(posVelAtt->n_velocity().z()); // GPS velocity in down direction in earth-fixed NED frame
    autopilot_interface.setGPS(lat_d, lon_d, alt, vn, ve, vd, time_week_ms, time_week);
}

std::string NAV::MavlinkSend::convertArrayToIPAddress(const std::array<int, 4>& ipArray)
{
    return fmt::format("{}", fmt::join(ipArray, "."));
}

const char* NAV::MavlinkSend::to_string(Baudrate value)
{
    static const std::unordered_map<Baudrate, const char*> baudrateMap = {
        { Baudrate::BAUDRATE_1200, "1200" },
        { Baudrate::BAUDRATE_2400, "2400" },
        { Baudrate::BAUDRATE_4800, "4800" },
        { Baudrate::BAUDRATE_9600, "9600" },
        { Baudrate::BAUDRATE_19200, "19200" },
        { Baudrate::BAUDRATE_38400, "38400" },
        { Baudrate::BAUDRATE_57600, "57600" },
        { Baudrate::BAUDRATE_111100, "111100" },
        { Baudrate::BAUDRATE_115200, "115200" },
        { Baudrate::BAUDRATE_230400, "230400" },
        { Baudrate::BAUDRATE_256000, "256000" },
        { Baudrate::BAUDRATE_460800, "460800" },
        { Baudrate::BAUDRATE_500000, "500000" },
        { Baudrate::BAUDRATE_921600, "921600" },
        { Baudrate::BAUDRATE_1500000, "1500000" },
        { Baudrate::COUNT, "" }
    };
    return baudrateMap.at(value);
}

int NAV::MavlinkSend::getBaudrateValue(Baudrate value)
{
    static const std::unordered_map<Baudrate, int> baudrateMap = {
        { Baudrate::BAUDRATE_1200, 1200 },
        { Baudrate::BAUDRATE_2400, 2400 },
        { Baudrate::BAUDRATE_4800, 4800 },
        { Baudrate::BAUDRATE_9600, 9600 },
        { Baudrate::BAUDRATE_19200, 19200 },
        { Baudrate::BAUDRATE_38400, 38400 },
        { Baudrate::BAUDRATE_57600, 57600 },
        { Baudrate::BAUDRATE_111100, 111100 },
        { Baudrate::BAUDRATE_115200, 115200 },
        { Baudrate::BAUDRATE_230400, 230400 },
        { Baudrate::BAUDRATE_256000, 256000 },
        { Baudrate::BAUDRATE_460800, 460800 },
        { Baudrate::BAUDRATE_500000, 500000 },
        { Baudrate::BAUDRATE_921600, 921600 },
        { Baudrate::BAUDRATE_1500000, 1500000 }
    };
    return baudrateMap.at(value);
}
#endif