// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "udpRecv.hpp"

#include "Nodes/DataLink/UdpUtil.hpp"
#include "NodeData/GNSS/GnssObs.hpp"
#include "NodeData/State/PosVelAtt.hpp"
#include "Navigation/GNSS/Core/SatelliteIdentifier.hpp"
#include "Navigation/Time/InsTime.hpp"

#include "util/Assert.h"
#include "internal/FlowManager.hpp"

#include "util/Logger.hpp"
#include "internal/gui/widgets/imgui_ex.hpp"
#include "internal/gui/widgets/EnumCombo.hpp"
#include "internal/gui/NodeEditorApplication.hpp"

#include <cstdint>
#include <cstring>
#include <memory>
#include <boost/system/detail/error_code.hpp>

NAV::UdpRecv::UdpRecv()
    : Node(typeStatic()), _socket(_io_context)
{
    LOG_TRACE("{}: called", name);

    _onlyRealTime = true;
    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 261, 95 };

    CreateOutputPin("Data", Pin::Type::Flow, { NAV::PosVelAtt::type() });
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
    if (ImGui::InputIntL(fmt::format("Port##{}", size_t(id)).c_str(), &_port, UdpUtil::PORT_LIMITS[0], UdpUtil::PORT_LIMITS[1]))
    {
        flow::ApplyChanges();
    }
    ImGui::SetNextItemWidth(150 * gui::NodeEditorApplication::windowFontRatio());
    if (gui::widgets::EnumCombo(fmt::format("Output Type##{}", size_t(id)).c_str(), _outputType))
    {
        LOG_DEBUG("{}: Output Type changed to {}", nameId(), to_string(_outputType));
        if (_outputType == UdpUtil::MessageType::PosVelAtt)
        {
            outputPins.at(OUTPUT_PORT_INDEX_NODE_DATA).dataIdentifier = { NAV::PosVelAtt::type() };
            outputPins.at(OUTPUT_PORT_INDEX_NODE_DATA).name = NAV::PosVelAtt::type();
        }
        else if (_outputType == UdpUtil::MessageType::PosVel)
        {
            outputPins.at(OUTPUT_PORT_INDEX_NODE_DATA).dataIdentifier = { NAV::PosVel::type() };
            outputPins.at(OUTPUT_PORT_INDEX_NODE_DATA).name = NAV::PosVel::type();
        }
        else if (_outputType == UdpUtil::MessageType::Pos)
        {
            outputPins.at(OUTPUT_PORT_INDEX_NODE_DATA).dataIdentifier = { NAV::Pos::type() };
            outputPins.at(OUTPUT_PORT_INDEX_NODE_DATA).name = NAV::Pos::type();
        }
        else if (_outputType == UdpUtil::MessageType::GnssObs)
        {
            outputPins.at(OUTPUT_PORT_INDEX_NODE_DATA).dataIdentifier = { NAV::GnssObs::type() };
            outputPins.at(OUTPUT_PORT_INDEX_NODE_DATA).name = NAV::GnssObs::type();
        }

        for (auto& link : outputPins.front().links)
        {
            if (auto* connectedPin = link.getConnectedPin())
            {
                outputPins.front().recreateLink(*connectedPin);
            }
        }

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
    j["port"] = _port;
    j["outputType"] = _outputType;

    return j;
}

void NAV::UdpRecv::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());
    if (j.contains("port"))
    {
        j.at("port").get_to(_port);
    }
    if (j.contains("outputType"))
    {
        j.at("outputType").get_to(_outputType);

        if (!outputPins.empty())
        {
            if (_outputType == UdpUtil::MessageType::PosVelAtt)
            {
                outputPins.at(OUTPUT_PORT_INDEX_NODE_DATA).dataIdentifier = { NAV::PosVelAtt::type() };
                outputPins.at(OUTPUT_PORT_INDEX_NODE_DATA).name = NAV::PosVelAtt::type();
            }
            else if (_outputType == UdpUtil::MessageType::PosVel)
            {
                outputPins.at(OUTPUT_PORT_INDEX_NODE_DATA).dataIdentifier = { NAV::PosVel::type() };
                outputPins.at(OUTPUT_PORT_INDEX_NODE_DATA).name = NAV::PosVel::type();
            }
            else if (_outputType == UdpUtil::MessageType::Pos)
            {
                outputPins.at(OUTPUT_PORT_INDEX_NODE_DATA).dataIdentifier = { NAV::Pos::type() };
                outputPins.at(OUTPUT_PORT_INDEX_NODE_DATA).name = NAV::Pos::type();
            }
            else if (_outputType == UdpUtil::MessageType::GnssObs)
            {
                outputPins.at(OUTPUT_PORT_INDEX_NODE_DATA).dataIdentifier = { NAV::GnssObs::type() };
                outputPins.at(OUTPUT_PORT_INDEX_NODE_DATA).name = NAV::GnssObs::type();
            }
        }
    }
}

bool NAV::UdpRecv::initialize()
{
    LOG_TRACE("{}: called", nameId());

    try
    {
        _socket = boost::asio::ip::udp::socket(_io_context, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), static_cast<uint16_t>(_port)));
    }
    catch (const std::exception& /* e */)
    {
        LOG_ERROR("{}: Port {} is already in use. Choose a different port for this instance.", nameId(), _port);
        return false;
    }

    _running = true;

    asyncReceive();

    if (_isStartup)
    {
        _recvThread = std::thread([this]() {
            _io_context.run();
        });
    }
    else
    {
        _recvThread = std::thread([this]() {
            _io_context.restart();
            _io_context.run();
        });
    }

    _isStartup = false;

    return true;
}

void NAV::UdpRecv::deinitialize()
{
    _running = false;
    _io_context.stop();
    _recvThread.join();
    _socket.close();

    LOG_TRACE("{}: called", nameId());
}

void NAV::UdpRecv::asyncReceive()
{
    _socket.async_receive_from(
        boost::asio::buffer(_charArray, UdpUtil::MAXIMUM_BYTES), _sender_endpoint,
        [this](boost::system::error_code errorRcvd, std::size_t bytesRcvd) {
            if ((!errorRcvd) && (bytesRcvd > 0))
            {
                UdpUtil::MessageType msgType{};
                std::memcpy(&msgType, _charArray.data(), UdpUtil::Size::MSGTYPE);
                LOG_DATA("{}: Received {} bytes (message type {})", nameId(), bytesRcvd, fmt::underlying(msgType));

                int32_t gpsCycle{};
                int32_t gpsWeek{};
                double gpsTow{};

                std::memcpy(&gpsCycle, _charArray.data() + UdpUtil::Offset::GPSCYCLE, UdpUtil::Size::GPSCYCLE);
                std::memcpy(&gpsWeek, _charArray.data() + UdpUtil::Offset::GPSWEEK, UdpUtil::Size::GPSWEEK);
                std::memcpy(&gpsTow, _charArray.data() + UdpUtil::Offset::GPSTOW, UdpUtil::Size::GPSTOW);

                if (msgType == UdpUtil::MessageType::PosVelAtt)
                {
                    if (outputPins.at(OUTPUT_PORT_INDEX_NODE_DATA).name != NAV::PosVelAtt::type())
                    {
                        LOG_ERROR("{}: Change output type to 'PosVelAtt'!", nameId());
                        return;
                    }
                    auto obs = std::make_shared<PosVelAtt>();

                    obs->insTime = InsTime(gpsCycle, gpsWeek, gpsTow);

                    // Position in LLA coordinates
                    Eigen::Vector3d posLLA{};
                    std::memcpy(posLLA.data(), _charArray.data() + UdpUtil::Offset::POS, UdpUtil::Size::POS);

                    // Velocity in local frame
                    Eigen::Vector3d vel_n{};
                    std::memcpy(vel_n.data(), _charArray.data() + UdpUtil::Offset::VEL, UdpUtil::Size::VEL);

                    // Attitude
                    Eigen::Quaterniond n_Quat_b{};
                    std::memcpy(&n_Quat_b.x(), _charArray.data() + UdpUtil::Offset::QUAT, UdpUtil::Size::QUAT);
                    std::memcpy(&n_Quat_b.y(), _charArray.data() + UdpUtil::Offset::QUAT + UdpUtil::Size::QUAT, UdpUtil::Size::QUAT);
                    std::memcpy(&n_Quat_b.z(), _charArray.data() + UdpUtil::Offset::QUAT + 2 * UdpUtil::Size::QUAT, UdpUtil::Size::QUAT);
                    std::memcpy(&n_Quat_b.w(), _charArray.data() + UdpUtil::Offset::QUAT + 3 * UdpUtil::Size::QUAT, UdpUtil::Size::QUAT);

                    obs->setPosVelAtt_n(posLLA, vel_n, n_Quat_b);

                    this->invokeCallbacks(OUTPUT_PORT_INDEX_NODE_DATA, obs);
                }
                else if (msgType == UdpUtil::MessageType::PosVel)
                {
                    if (outputPins.at(OUTPUT_PORT_INDEX_NODE_DATA).name != NAV::PosVel::type())
                    {
                        LOG_ERROR("{}: Change output type to 'PosVel'!", nameId());
                        return;
                    }
                    auto obs = std::make_shared<PosVel>();

                    obs->insTime = InsTime(gpsCycle, gpsWeek, gpsTow);

                    // Position in LLA coordinates
                    Eigen::Vector3d posLLA{};
                    std::memcpy(posLLA.data(), _charArray.data() + UdpUtil::Offset::POS, UdpUtil::Size::POS);

                    // Velocity in local frame
                    Eigen::Vector3d vel_n{};
                    std::memcpy(vel_n.data(), _charArray.data() + UdpUtil::Offset::VEL, UdpUtil::Size::VEL);

                    obs->setPosVel_n(posLLA, vel_n);

                    this->invokeCallbacks(OUTPUT_PORT_INDEX_NODE_DATA, obs);
                }
                else if (msgType == UdpUtil::MessageType::Pos)
                {
                    if (outputPins.at(OUTPUT_PORT_INDEX_NODE_DATA).name != NAV::Pos::type())
                    {
                        LOG_ERROR("{}: Change output type to 'Pos'!", nameId());
                        return;
                    }
                    auto obs = std::make_shared<Pos>();

                    obs->insTime = InsTime(gpsCycle, gpsWeek, gpsTow);

                    // Position in LLA coordinates
                    Eigen::Vector3d posLLA{};
                    std::memcpy(posLLA.data(), _charArray.data() + UdpUtil::Offset::POS, UdpUtil::Size::POS);

                    obs->setPosition_lla(posLLA);

                    this->invokeCallbacks(OUTPUT_PORT_INDEX_NODE_DATA, obs);
                }
                else if (msgType == UdpUtil::MessageType::GnssObs)
                {
                    if (outputPins.at(OUTPUT_PORT_INDEX_NODE_DATA).name != NAV::GnssObs::type())
                    {
                        LOG_ERROR("{}: Change output type to 'GnssObs'!", nameId());
                        return;
                    }
                    auto gnssObs = std::make_shared<GnssObs>();
                    gnssObs->insTime = InsTime(gpsCycle, gpsWeek, gpsTow);

                    size_t byteSizeGnssData{};
                    std::memcpy(&byteSizeGnssData, _charArray.data() + UdpUtil::Offset::SIZE, UdpUtil::Size::SIZE);
                    size_t sizeGnssData = byteSizeGnssData / UdpUtil::Size::SINGLE_OBSERVATION_DATA;
                    INS_ASSERT_USER_ERROR(byteSizeGnssData % UdpUtil::Size::SINGLE_OBSERVATION_DATA == 0,
                                          "The UdpRecv node received a not dividable amount of bytes for the GnssObs data.");
                    LOG_DATA("{}:   {} GNSS signals ({} bytes)", nameId(), sizeGnssData, byteSizeGnssData);
                    gnssObs->data.resize(sizeGnssData, GnssObs::ObservationData(SatSigId()));
                    std::memcpy(gnssObs->data.data(), _charArray.data() + UdpUtil::Offset::GNSSDATA, byteSizeGnssData);

                    this->invokeCallbacks(OUTPUT_PORT_INDEX_NODE_DATA, gnssObs);
                }
                else
                {
                    LOG_ERROR("{}: Data type not receivable, yet.", nameId());
                }
            }
            else
            {
                LOG_ERROR("{}: Error receiving the UDP network stream: {}, Received bytes: {}", nameId(), errorRcvd.what(), bytesRcvd);
            }

            if (_running)
            {
                asyncReceive();
            }
        });
}

const char* NAV::to_string(NAV::UdpUtil::MessageType value)
{
    switch (value)
    {
    case NAV::UdpUtil::MessageType::PosVelAtt:
        return "PosVelAtt";
    case NAV::UdpUtil::MessageType::PosVel:
        return "PosVel";
    case NAV::UdpUtil::MessageType::Pos:
        return "Pos";
    case NAV::UdpUtil::MessageType::GnssObs:
        return "GnssObs";
    case NAV::UdpUtil::MessageType::COUNT:
        return "";
    }
    return "";
}