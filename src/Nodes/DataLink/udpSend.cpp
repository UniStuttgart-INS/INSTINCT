// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "udpSend.hpp"

#include "NodeData/GNSS/GnssObs.hpp"
#include "NodeData/State/PosVelAtt.hpp"
#include "internal/Node/Pin.hpp"
#include "internal/FlowManager.hpp"
#include "NodeRegistry.hpp"

#include "util/Logger.hpp"
#include "internal/gui/widgets/imgui_ex.hpp"
#include "internal/gui/NodeEditorApplication.hpp"

#include <cstring>
#include <string>
#include <vector>

// ---------------------------------------------------------- Private variabels ------------------------------------------------------------

namespace NAV
{
/// List of supported data identifiers
const std::vector<std::string> supportedDataIdentifier{
    PosVelAtt::type(),
    PosVel::type(),
    Pos::type(),
    GnssObs::type()
};

} // namespace NAV

// ---------------------------------------------------------- Member functions -------------------------------------------------------------

NAV::UdpSend::UdpSend()
    : Node(typeStatic()), _socket(_io_context, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 0)), _resolver(_io_context)
{
    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 202, 96 };

    CreateInputPin("Data", Pin::Type::Flow, supportedDataIdentifier, &UdpSend::receiveData);
}

NAV::UdpSend::~UdpSend()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::UdpSend::typeStatic()
{
    return "UdpSend";
}

std::string NAV::UdpSend::type() const
{
    return typeStatic();
}

std::string NAV::UdpSend::category()
{
    return "Data Link";
}

void NAV::UdpSend::guiConfig()
{
    ImGui::SetNextItemWidth(150 * gui::NodeEditorApplication::windowFontRatio());
    if (ImGui::InputInt4L(fmt::format("IPv4##{}", size_t(id)).c_str(), _ip.data(), IP_LIMITS[0], IP_LIMITS[1]))
    {
        flow::ApplyChanges();
    }
    ImGui::SetNextItemWidth(150 * gui::NodeEditorApplication::windowFontRatio());
    if (ImGui::InputIntL(fmt::format("Port##{}", size_t(id)).c_str(), &_port, UdpUtil::PORT_LIMITS[0], UdpUtil::PORT_LIMITS[1]))
    {
        flow::ApplyChanges();
    }
}

bool NAV::UdpSend::resetNode()
{
    return true;
}

json NAV::UdpSend::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["ip"] = _ip;
    j["port"] = _port;

    return j;
}

void NAV::UdpSend::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());
    if (j.contains("ip"))
    {
        j.at("ip").get_to(_ip);
    }
    if (j.contains("port"))
    {
        j.at("port").get_to(_port);
    }
}

bool NAV::UdpSend::initialize()
{
    LOG_TRACE("{}: called", nameId());

    std::string ipString{};
    for (size_t i = 0; i < 4; i++)
    {
        ipString.append(std::to_string(_ip.at(i)));
        i < 3 ? ipString.append(".") : ipString.append("");
    }

    _endpoints = _resolver.resolve(boost::asio::ip::udp::v4(), ipString, std::to_string(_port));

    return true;
}

void NAV::UdpSend::deinitialize()
{
    _io_context.stop();

    LOG_TRACE("{}: called", nameId());
}

void NAV::UdpSend::receiveData(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    auto data = queue.extract_front();

    std::vector<char> data2send{};

    // Identify message type
    if (NAV::NodeRegistry::NodeDataTypeAnyIsChildOf({ data->getType() }, { PosVelAtt::type() }))
    {
        _msgType = UdpUtil::MessageType::PosVelAtt;
        data2send.resize(UdpUtil::Size::TOTAL_POSVELATT);
        setMsgTypeAndTime(data2send, data->insTime);
    }
    else if (NAV::NodeRegistry::NodeDataTypeAnyIsChildOf({ data->getType() }, { PosVel::type() }))
    {
        _msgType = UdpUtil::MessageType::PosVel;
        data2send.resize(UdpUtil::Size::TOTAL_POSVEL);
        setMsgTypeAndTime(data2send, data->insTime);
    }
    else if (NAV::NodeRegistry::NodeDataTypeAnyIsChildOf({ data->getType() }, { Pos::type() }))
    {
        _msgType = UdpUtil::MessageType::Pos;
        data2send.resize(UdpUtil::Size::TOTAL_POS);
        setMsgTypeAndTime(data2send, data->insTime);
    }
    else if (NAV::NodeRegistry::NodeDataTypeAnyIsChildOf({ data->getType() }, { GnssObs::type() }))
    {
        _msgType = UdpUtil::MessageType::GnssObs;
    }

    // Copy data
    if (_msgType == UdpUtil::MessageType::Pos)
    {
        auto pos = std::static_pointer_cast<const Pos>(data);
        std::memcpy(data2send.data() + UdpUtil::Offset::POS, pos->lla_position().data(), UdpUtil::Size::POS);
    }
    else if (_msgType == UdpUtil::MessageType::PosVel)
    {
        auto posVel = std::static_pointer_cast<const PosVel>(data);
        std::memcpy(data2send.data() + UdpUtil::Offset::POS, posVel->lla_position().data(), UdpUtil::Size::POS);
        std::memcpy(data2send.data() + UdpUtil::Offset::VEL, posVel->n_velocity().data(), UdpUtil::Size::VEL);
    }
    else if (_msgType == UdpUtil::MessageType::PosVelAtt)
    {
        auto posVelAtt = std::static_pointer_cast<const PosVelAtt>(data);
        std::memcpy(data2send.data() + UdpUtil::Offset::POS, posVelAtt->lla_position().data(), UdpUtil::Size::POS);
        std::memcpy(data2send.data() + UdpUtil::Offset::VEL, posVelAtt->n_velocity().data(), UdpUtil::Size::VEL);
        std::memcpy(data2send.data() + UdpUtil::Offset::QUAT, &posVelAtt->n_Quat_b().x(), UdpUtil::Size::QUAT);
        std::memcpy(data2send.data() + UdpUtil::Offset::QUAT + UdpUtil::Size::QUAT, &posVelAtt->n_Quat_b().y(), UdpUtil::Size::QUAT);
        std::memcpy(data2send.data() + UdpUtil::Offset::QUAT + 2 * UdpUtil::Size::QUAT, &posVelAtt->n_Quat_b().z(), UdpUtil::Size::QUAT);
        std::memcpy(data2send.data() + UdpUtil::Offset::QUAT + 3 * UdpUtil::Size::QUAT, &posVelAtt->n_Quat_b().w(), UdpUtil::Size::QUAT);
    }
    else if (_msgType == UdpUtil::MessageType::GnssObs)
    {
        auto gnssObs = std::static_pointer_cast<const GnssObs>(data);
        const size_t sizeGnssData = UdpUtil::Size::SINGLE_OBSERVATION_DATA * gnssObs->data.size();

        auto sizeTotal = sizeGnssData + UdpUtil::Size::MSGTYPE + UdpUtil::Size::GPSCYCLE + UdpUtil::Size::GPSWEEK + UdpUtil::Size::GPSTOW + UdpUtil::Size::SIZE;
        if (sizeTotal > UdpUtil::MAXIMUM_BYTES)
        {
            LOG_ERROR("{}: gnssObs msg is bigger than the maximum size of a single UDP package: {} bytes.", nameId(), sizeTotal);
        }

        data2send.resize(sizeTotal);

        setMsgTypeAndTime(data2send, data->insTime);

        std::memcpy(data2send.data() + UdpUtil::Offset::SIZE, &sizeGnssData, UdpUtil::Size::SIZE);
        std::memcpy(data2send.data() + UdpUtil::Offset::GNSSDATA, gnssObs->data.data(), sizeGnssData);
    }
    _socket.send_to(boost::asio::buffer(data2send), *_endpoints.begin());
}

void NAV::UdpSend::setMsgTypeAndTime(std::vector<char>& data2send, const InsTime& insTime)
{
    auto gpsCycle = insTime.toGPSweekTow().gpsCycle;
    auto gpsWeek = insTime.toGPSweekTow().gpsWeek;
    auto gpsTow = static_cast<double>(insTime.toGPSweekTow().tow);

    std::memcpy(data2send.data(), &_msgType, UdpUtil::Size::MSGTYPE);

    std::memcpy(data2send.data() + UdpUtil::Offset::GPSCYCLE, &gpsCycle, UdpUtil::Size::GPSCYCLE);
    std::memcpy(data2send.data() + UdpUtil::Offset::GPSWEEK, &gpsWeek, UdpUtil::Size::GPSWEEK);
    std::memcpy(data2send.data() + UdpUtil::Offset::GPSTOW, &gpsTow, UdpUtil::Size::GPSTOW);
}