// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "udpSend.hpp"
#include "UdpUtil.hpp"

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

    if (!NAV::NodeRegistry::NodeDataTypeAnyIsChildOf({ data->getType() }, { GnssObs::type(), PosVelAtt::type() }))
    {
        LOG_ERROR("{}: Data type {} not sendable, yet.", nameId(), data->getType());
        return;
    }

    auto gpsCycle = data->insTime.toGPSweekTow().gpsCycle;
    auto gpsWeek = data->insTime.toGPSweekTow().gpsWeek;
    auto gpsTow = static_cast<double>(data->insTime.toGPSweekTow().tow);

    std::vector<char> data2send{};

    if (NAV::NodeRegistry::NodeDataTypeAnyIsChildOf({ data->getType() }, { PosVelAtt::type() }))
    {
        _msgType = 0;

        auto posVelAtt = std::static_pointer_cast<const PosVelAtt>(data);

        data2send.resize(UdpUtil::SIZE_TOTAL);

        std::memcpy(data2send.data(), &_msgType, UdpUtil::SIZE_MSGTYPE);

        std::memcpy(data2send.data() + UdpUtil::OFFSET_GPSCYCLE, &gpsCycle, UdpUtil::SIZE_GPSCYCLE);
        std::memcpy(data2send.data() + UdpUtil::OFFSET_GPSWEEK, &gpsWeek, UdpUtil::SIZE_GPSWEEK);
        std::memcpy(data2send.data() + UdpUtil::OFFSET_GPSTOW, &gpsTow, UdpUtil::SIZE_GPSTOW);

        std::memcpy(data2send.data() + UdpUtil::OFFSET_POS, posVelAtt->lla_position().data(), UdpUtil::SIZE_POS);
        std::memcpy(data2send.data() + UdpUtil::OFFSET_VEL, posVelAtt->n_velocity().data(), UdpUtil::SIZE_VEL);
        std::memcpy(data2send.data() + UdpUtil::OFFSET_QUAT, &posVelAtt->n_Quat_b().x(), UdpUtil::SIZE_QUAT);
        std::memcpy(data2send.data() + UdpUtil::OFFSET_QUAT + UdpUtil::SIZE_QUAT, &posVelAtt->n_Quat_b().y(), UdpUtil::SIZE_QUAT);
        std::memcpy(data2send.data() + UdpUtil::OFFSET_QUAT + 2 * UdpUtil::SIZE_QUAT, &posVelAtt->n_Quat_b().z(), UdpUtil::SIZE_QUAT);
        std::memcpy(data2send.data() + UdpUtil::OFFSET_QUAT + 3 * UdpUtil::SIZE_QUAT, &posVelAtt->n_Quat_b().w(), UdpUtil::SIZE_QUAT);
    }
    else if (NAV::NodeRegistry::NodeDataTypeAnyIsChildOf({ data->getType() }, { GnssObs::type() }))
    {
        _msgType = 1;

        auto gnssObs = std::static_pointer_cast<const GnssObs>(data);
        const size_t sizeGnssData = UdpUtil::SIZE_SINGLE_OBSERVATION_DATA * gnssObs->data.size();

        auto sizeTotal = sizeGnssData + UdpUtil::SIZE_MSGTYPE + UdpUtil::SIZE_GPSCYCLE + UdpUtil::SIZE_GPSWEEK + UdpUtil::SIZE_GPSTOW + UdpUtil::SIZE_SIZE;
        if (sizeTotal > UdpUtil::MAXIMUM_BYTES)
        {
            LOG_ERROR("{}: gnssObs msg is bigger than the maximum size of a single UDP package: {} bytes.", nameId(), sizeTotal);
        }

        data2send.resize(sizeTotal);
        std::memcpy(data2send.data(), &_msgType, UdpUtil::SIZE_MSGTYPE);

        std::memcpy(data2send.data() + UdpUtil::OFFSET_GPSCYCLE, &gpsCycle, UdpUtil::SIZE_GPSCYCLE);
        std::memcpy(data2send.data() + UdpUtil::OFFSET_GPSWEEK, &gpsWeek, UdpUtil::SIZE_GPSWEEK);
        std::memcpy(data2send.data() + UdpUtil::OFFSET_GPSTOW, &gpsTow, UdpUtil::SIZE_GPSTOW);

        std::memcpy(data2send.data() + UdpUtil::OFFSET_SIZE, &sizeGnssData, UdpUtil::SIZE_SIZE);
        std::memcpy(data2send.data() + UdpUtil::OFFSET_GNSSDATA, gnssObs->data.data(), sizeGnssData);
    }
    _socket.send_to(boost::asio::buffer(data2send), *_endpoints.begin());
}