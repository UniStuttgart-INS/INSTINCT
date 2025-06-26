// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "udpRecv.hpp"
#include <cstdint>
#include <cstring>
#include <memory>
#include <boost/system/detail/error_code.hpp>

#include "Navigation/GNSS/Core/SatelliteIdentifier.hpp"
#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "internal/gui/widgets/imgui_ex.hpp"
#include "internal/gui/widgets/EnumCombo.hpp"
#include "internal/gui/NodeEditorApplication.hpp"

#include "NodeData/State/PosVelAtt.hpp"
#include "NodeData/GNSS/GnssObs.hpp"

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

NAV::UdpRecv::UdpRecv()
    : Node(typeStatic()), _socket(_io_context)
{
    LOG_TRACE("{}: called", name);

    _onlyRealTime = true;
    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 202, 66 };

    nm::CreateOutputPin(this, "Data", Pin::Type::Flow, supportedDataIdentifier);
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
    if (ImGui::InputIntL(fmt::format("Port##{}", size_t(id)).c_str(), &_port, PORT_LIMITS[0], PORT_LIMITS[1]))
    {
        flow::ApplyChanges();
    }
    if (gui::widgets::EnumCombo(fmt::format("Output Type##{}", size_t(id)).c_str(), _outputType))
    {
        LOG_DEBUG("{}: Output Type changed to {}", nameId(), to_string(_outputType));
        if (_outputType == OutputType::PosVelAtt)
        {
            outputPins.at(OUTPUT_PORT_INDEX_NODE_DATA).dataIdentifier = { NAV::PosVelAtt::type() };
            outputPins.at(OUTPUT_PORT_INDEX_NODE_DATA).name = NAV::PosVelAtt::type();
        }
        else if (_outputType == OutputType::GnssObs)
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
            if (_outputType == OutputType::PosVelAtt)
            {
                outputPins.at(OUTPUT_PORT_INDEX_NODE_DATA).dataIdentifier = { NAV::PosVelAtt::type() };
                outputPins.at(OUTPUT_PORT_INDEX_NODE_DATA).name = NAV::PosVelAtt::type();
            }
            else if (_outputType == OutputType::GnssObs)
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
        boost::asio::buffer(_charArray, MAXIMUM_BYTES), _sender_endpoint,
        [this](boost::system::error_code errorRcvd, std::size_t bytesRcvd) {
            if ((!errorRcvd) && (bytesRcvd > 0))
            {
                std::memcpy(&_msgType, _charArray.data(), SIZE_MSGTYPE);
                if (_msgType == 0)
                {
                    if (outputPins.at(OUTPUT_PORT_INDEX_NODE_DATA).name != NAV::PosVelAtt::type())
                    {
                        LOG_ERROR("{}: Change output type to 'PosVelAtt'!", nameId());
                        return;
                    }
                    auto obs = std::make_shared<PosVelAtt>();

                    std::memcpy(&obs->insTime, _charArray.data() + OFFSET_TIMESTAMP, SIZE_TIMESTAMP);

                    // Position in LLA coordinates
                    Eigen::Vector3d posLLA{};
                    std::memcpy(posLLA.data(), _charArray.data() + OFFSET_POS, SIZE_POS);

                    // Velocity in local frame
                    Eigen::Vector3d vel_n{};
                    std::memcpy(vel_n.data(), _charArray.data() + OFFSET_VEL, SIZE_VEL);

                    // Attitude
                    Eigen::Quaterniond n_Quat_b{};
                    std::memcpy(&n_Quat_b.x(), _charArray.data() + OFFSET_QUAT, SIZE_QUAT);
                    std::memcpy(&n_Quat_b.y(), _charArray.data() + OFFSET_QUAT + SIZE_QUAT, SIZE_QUAT);
                    std::memcpy(&n_Quat_b.z(), _charArray.data() + OFFSET_QUAT + 2 * SIZE_QUAT, SIZE_QUAT);
                    std::memcpy(&n_Quat_b.w(), _charArray.data() + OFFSET_QUAT + 3 * SIZE_QUAT, SIZE_QUAT);

                    obs->setPosVelAtt_n(posLLA, vel_n, n_Quat_b);

                    this->invokeCallbacks(OUTPUT_PORT_INDEX_NODE_DATA, obs);
                }
                else if (_msgType == 1)
                {
                    if (outputPins.at(OUTPUT_PORT_INDEX_NODE_DATA).name != NAV::GnssObs::type())
                    {
                        LOG_ERROR("{}: Change output type to 'GnssObs'!", nameId());
                        return;
                    }
                    auto gnssObs = std::make_shared<GnssObs>();

                    size_t sizeGnssData{};

                    std::memcpy(&gnssObs->insTime, _charArray.data() + OFFSET_TIMESTAMP, SIZE_TIMESTAMP);
                    std::memcpy(&sizeGnssData, _charArray.data() + OFFSET_SIZE, SIZE_SIZE);
                    gnssObs->data.resize(sizeGnssData, GnssObs::ObservationData(SatSigId()));
                    std::memcpy(gnssObs->data.data(), _charArray.data() + OFFSET_GNSSDATA, sizeGnssData);

                    this->invokeCallbacks(OUTPUT_PORT_INDEX_NODE_DATA, gnssObs);
                    LOG_DATA("{}: Received bytes: {}", nameId(), bytesRcvd);
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

const char* NAV::to_string(NAV::UdpRecv::OutputType value)
{
    switch (value)
    {
    case NAV::UdpRecv::OutputType::PosVelAtt:
        return "PosVelAtt";
    case NAV::UdpRecv::OutputType::GnssObs:
        return "GnssObs";
    case NAV::UdpRecv::OutputType::COUNT:
        return "";
    }
    return "";
}