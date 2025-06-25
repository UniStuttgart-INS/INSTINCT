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
#include <type_traits>
#include <boost/system/detail/error_code.hpp>

#include "Navigation/GNSS/Core/SatelliteIdentifier.hpp"
#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "internal/gui/widgets/imgui_ex.hpp"
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

    return j;
}

void NAV::UdpRecv::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());
    if (j.contains("port"))
    {
        j.at("port").get_to(_port);
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
        boost::asio::buffer(charArray, MAXIMUM_BYTES), _sender_endpoint,
        [this](boost::system::error_code errorRcvd, std::size_t bytesRcvd) {
            if ((!errorRcvd) && (bytesRcvd > 0))
            {
                std::memcpy(&_msgType, charArray.data(), SIZE_MSGTYPE);
                if (_msgType == 0)
                {
                    auto obs = std::make_shared<PosVelAtt>();

                    auto sizePosLla = 3 * sizeof(obs->lla_position().data());
                    auto sizeVelNed = 3 * sizeof(obs->n_velocity().data());
                    auto sizeQuat = 4 * sizeof(obs->n_Quat_b().x());

                    auto offsetTimestamp = SIZE_MSGTYPE;
                    auto offsetPosLla = offsetTimestamp + SIZE_TIMESTAMP;
                    auto offsetVelNed = offsetPosLla + sizePosLla;
                    auto offsetQuat = offsetVelNed + sizeVelNed;

                    // auto sizeTotal = offsetQuat + sizeQuat;

                    std::memcpy(&obs->insTime, charArray.data() + offsetTimestamp, SIZE_TIMESTAMP);

                    // Position in LLA coordinates
                    Eigen::Vector3d posLLA{};
                    std::memcpy(posLLA.data(), charArray.data() + offsetPosLla, sizePosLla);

                    // Velocity in local frame
                    Eigen::Vector3d vel_n{};
                    std::memcpy(vel_n.data(), charArray.data() + offsetVelNed, sizeVelNed);

                    // Attitude
                    Eigen::Quaterniond n_Quat_b{};
                    std::memcpy(&n_Quat_b.x(), charArray.data() + offsetQuat, sizeQuat);
                    std::memcpy(&n_Quat_b.y(), charArray.data() + offsetQuat + sizeQuat, sizeQuat);
                    std::memcpy(&n_Quat_b.z(), charArray.data() + offsetQuat + 2 * sizeQuat, sizeQuat);
                    std::memcpy(&n_Quat_b.w(), charArray.data() + offsetQuat + 3 * sizeQuat, sizeQuat);

                    obs->setPosVelAtt_n(posLLA, vel_n, n_Quat_b);
                    // obs->insTime // TODO

                    this->invokeCallbacks(OUTPUT_PORT_INDEX_NODE_DATA, obs);
                }
                else if (_msgType == 1)
                {
                    auto gnssObs = std::make_shared<GnssObs>();

                    uint32_t timeSize = 32;
                    auto dataSize = (sizeof(charArray) - timeSize) / sizeof(GnssObs::ObservationData);

                    gnssObs->data.resize(1, GnssObs::ObservationData(SatSigId()));

                    std::memcpy(&gnssObs->insTime, charArray.data(), timeSize);
                    std::memcpy(gnssObs->data.data(), charArray.data() + timeSize, dataSize);

                    this->invokeCallbacks(OUTPUT_PORT_INDEX_NODE_DATA, gnssObs);
                    LOG_INFO("Received bytes: {}", bytesRcvd);
                }
                else
                {
                    LOG_ERROR("{}: Data type not receivable, yet.", nameId());
                }
            }
            else
            {
                LOG_ERROR("Error receiving the UDP network stream: {}, Received bytes: {}", errorRcvd.what(), bytesRcvd);
            }

            if (_running)
            {
                asyncReceive();
            }
            //         }
            //     }
            // )
            // {_socket.async_receive_from(
            //     boost::asio::buffer(_data, _maxBytes), _sender_endpoint,
            //     [this](boost::system::error_code errorRcvd, std::size_t bytesRcvd) {
            //         if ((!errorRcvd) && (bytesRcvd > 0))
            //         {
            //             auto obs = std::make_shared<PosVelAtt>();

            //             // Position in LLA coordinates
            //             Eigen::Vector3d posLLA{ _data.at(0), _data.at(1), _data.at(2) };

            //             // Velocity in local frame
            //             Eigen::Vector3d vel_n{ _data.at(3), _data.at(4), _data.at(5) };

            //             // Attitude
            //             Eigen::Quaterniond n_Quat_b{};
            //             n_Quat_b.x() = _data.at(6);
            //             n_Quat_b.y() = _data.at(7);
            //             n_Quat_b.z() = _data.at(8);
            //             n_Quat_b.w() = _data.at(9);

            //             // Time
            //             auto gpsC = static_cast<int32_t>(_data.at(10));
            //             auto gpsW = static_cast<int32_t>(_data.at(11));
            //             auto gpsT = static_cast<long double>(_data.at(12));

            //             obs->setPosVelAtt_n(posLLA, vel_n, n_Quat_b);
            //             obs->insTime = InsTime(gpsC, gpsW, gpsT);

            //             this->invokeCallbacks(OUTPUT_PORT_INDEX_NODE_DATA, obs);
            //         }
            //         else
            //         {
            //             LOG_ERROR("Error receiving the UDP network stream.");
            //         }

            //         if (_running)
            //         {
            //             asyncReceive();
            //         }
        });
}