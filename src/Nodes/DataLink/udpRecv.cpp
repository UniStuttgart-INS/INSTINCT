// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "udpRecv.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "internal/gui/widgets/HelpMarker.hpp"
#include "internal/gui/NodeEditorApplication.hpp"

#include "util/Logger.hpp"

NAV::UdpRecv::UdpRecv()
    : Node(typeStatic()), _socket(_io_context, udp::endpoint(udp::v4(), static_cast<unsigned short>(4567)))
{
    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 202, 96 };

    nm::CreateOutputPin(this, "PosVelAtt", Pin::Type::Flow, { NAV::PosVelAtt::type() });
}

NAV::UdpRecv::~UdpRecv()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::UdpRecv::typeStatic()
{
    return "UdpRecv";
}

std::string NAV::UdpRecv::type() const
{
    return typeStatic();
}

std::string NAV::UdpRecv::category()
{
    return "Data Link";
}

void NAV::UdpRecv::guiConfig()
{
    ImGui::SetNextItemWidth(150 * gui::NodeEditorApplication::windowFontRatio());
    if (ImGui::InputInt(fmt::format("Port##{}", size_t(id)).c_str(), &_port))
    {
        flow::ApplyChanges();
    }
}

bool NAV::UdpRecv::resetNode()
{
    return true;
}

json NAV::UdpRecv::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    return j;
}

void NAV::UdpRecv::restore(json const& /* j */)
{
    LOG_TRACE("{}: called", nameId());
}

bool NAV::UdpRecv::initialize()
{
    LOG_TRACE("{}: called", nameId());
    _running = true;
    _flagsenderstopped = 0.0;
    _recvThread = std::thread(&NAV::UdpRecv::pollPosVelAtt, this);

    return true;
}

void NAV::UdpRecv::deinitialize()
{
    _running = false;
    _recvThread.join();

    LOG_TRACE("{}: called", nameId());
}

void NAV::UdpRecv::pollPosVelAtt()
{
    std::vector<double> v(10);

    while (_running && (_flagsenderstopped < 1.0))
    {
        _socket.receive_from(boost::asio::buffer(v), _sender_endpoint);

        auto obs = std::make_shared<PosVelAtt>();

        Eigen::Vector3d test;
        test << v.at(0), 0, 0;
        _flagsenderstopped = v.at(9);

        obs->setPosition_lla(test);
        // obs->insTime()

        this->invokeCallbacks(OUTPUT_PORT_INDEX_NODE_DATA, obs);
    }
}