// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "udpSend.hpp"

#include "NodeRegistry.hpp"
#include <cstring>
#include <string>
#include <vector>
#include "NodeData/GNSS/GnssObs.hpp"
#include "internal/Node/Pin.hpp"
#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "internal/gui/widgets/imgui_ex.hpp"
#include "internal/gui/NodeEditorApplication.hpp"

#include "NodeData/State/PosVelAtt.hpp"

#include "util/Logger.hpp"

// ---------------------------------------------------------- Private variabels ------------------------------------------------------------

namespace NAV
{
/// List of supported data identifiers
const std::vector<std::string> supportedDataIdentifier{
    PosVelAtt::type(),
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

    nm::CreateInputPin(this, "Data", Pin::Type::Flow, supportedDataIdentifier, &UdpSend::receiveData);
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
    if (ImGui::InputIntL(fmt::format("Port##{}", size_t(id)).c_str(), &_port, PORT_LIMITS[0], PORT_LIMITS[1]))
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

    // std::vector<char> data2send{};

    if (NAV::NodeRegistry::NodeDataTypeAnyIsChildOf({ data->getType() }, { PosVelAtt::type() }))
    {
        _msgType = 0;

        auto posVelAtt = std::make_shared<PosVelAtt>(*std::static_pointer_cast<const PosVelAtt>(data));

        auto sizePosLla = 3 * sizeof(posVelAtt->lla_position().data());
        auto sizeVelNed = 3 * sizeof(posVelAtt->n_velocity().data());
        auto sizeQuat = 4 * sizeof(posVelAtt->n_Quat_b().x());

        auto offsetTimestamp = SIZE_MSGTYPE;
        auto offsetPosLla = offsetTimestamp + SIZE_TIMESTAMP;
        auto offsetVelNed = offsetPosLla + sizePosLla;
        auto offsetQuat = offsetVelNed + sizeVelNed;

        auto sizeTotal = offsetQuat + sizeQuat;

        std::vector<char> data2send(sizeTotal);
        LOG_WARN("dataSize: {}", sizeTotal);

        std::memcpy(data2send.data(), &_msgType, SIZE_MSGTYPE);
        std::memcpy(data2send.data() + offsetTimestamp, &posVelAtt->insTime, SIZE_TIMESTAMP);

        std::memcpy(data2send.data() + offsetPosLla, posVelAtt->lla_position().data(), sizePosLla);
        std::memcpy(data2send.data() + offsetVelNed, posVelAtt->n_velocity().data(), sizeVelNed);
        std::memcpy(data2send.data() + offsetQuat, &posVelAtt->n_Quat_b().x(), sizeQuat);
        std::memcpy(data2send.data() + offsetQuat + sizeQuat, &posVelAtt->n_Quat_b().y(), sizeQuat);
        std::memcpy(data2send.data() + offsetQuat + 2 * sizeQuat, &posVelAtt->n_Quat_b().z(), sizeQuat);
        std::memcpy(data2send.data() + offsetQuat + 3 * sizeQuat, &posVelAtt->n_Quat_b().w(), sizeQuat);

        _socket.send_to(boost::asio::buffer(data2send), *_endpoints.begin());
    }
    else if (NAV::NodeRegistry::NodeDataTypeAnyIsChildOf({ data->getType() }, { GnssObs::type() }))
    {
        _msgType = 1;

        auto gnssObs = std::make_shared<GnssObs>(*std::static_pointer_cast<const GnssObs>(data));
        const size_t gnssDataSize = SIZE_SINGLE_OBSERVATION_DATA * gnssObs->data.size();

        std::vector<char> data2send(gnssDataSize + SIZE_TIMESTAMP);
        LOG_WARN("dataSize: {}, SIZE_TIMESTAMP: {}", gnssDataSize, SIZE_TIMESTAMP);
        std::memcpy(data2send.data(), &_msgType, SIZE_MSGTYPE);
        std::memcpy(data2send.data() + SIZE_MSGTYPE, &gnssObs->insTime, SIZE_TIMESTAMP);
        std::memcpy(data2send.data() + SIZE_MSGTYPE + SIZE_TIMESTAMP, gnssObs->data.data(), gnssDataSize);

        _socket.send_to(boost::asio::buffer(data2send), *_endpoints.begin());
    }
    else
    {
        LOG_ERROR("{}: Data type {} not sendable, yet.", nameId(), data->getType());
    }
}