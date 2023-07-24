// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "udpSend.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "internal/gui/widgets/HelpMarker.hpp"
#include "internal/gui/NodeEditorApplication.hpp"

// #include "util/Eigen.hpp"
// #include "Navigation/Time/InsTime.hpp"

// #include "NodeData/General/CsvData.hpp"

// #include <cstdlib>
// #include <boost/asio.hpp>

#include "util/Logger.hpp"

NAV::UdpSend::UdpSend()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 677, 580 };

    nm::CreateInputPin(this, "PosVelAtt", Pin::Type::Flow, { NAV::PosVelAtt::type() }, &UdpSend::receivePosVelAtt);
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
    if (ImGui::InputInt4("IPv4", _ip))
    {
        flow::ApplyChanges();
    }
    ImGui::SetNextItemWidth(150 * gui::NodeEditorApplication::windowFontRatio());
    if (ImGui::InputInt(fmt::format("Port##{}", size_t(id)).c_str(), &_port))
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

    return j;
}

void NAV::UdpSend::restore(json const& /* j */)
{
    LOG_TRACE("{}: called", nameId());
}

bool NAV::UdpSend::initialize()
{
    LOG_TRACE("{}: called", nameId());

    return true;
}

void NAV::UdpSend::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

void NAV::UdpSend::receivePosVelAtt(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    auto posVelAtt = std::make_shared<PosVelAtt>(*std::static_pointer_cast<const PosVelAtt>(queue.extract_front()));

    // TODO: buffer, etc. here
}