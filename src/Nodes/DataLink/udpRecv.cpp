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

#include "util/Time/TimeBase.hpp"

#include "util/Logger.hpp"

NAV::UdpRecv::UdpRecv()
    : Node(typeStatic()), _socket(_io_context, udp::endpoint(udp::v4(), static_cast<unsigned short>(_port)))
{
    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 202, 66 };

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

    pollData();

    if (_isStartup)
    {
        _recvThread = std::thread([=, this]() {
            _io_context.run();
        });
    }
    else
    {
        _recvThread = std::thread([=, this]() {
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

    LOG_TRACE("{}: called", nameId());
}

void NAV::UdpRecv::pollData()
{
    _socket.async_receive_from(
        boost::asio::buffer(_data, _maxBytes), _sender_endpoint,
        [this](boost::system::error_code errorRcvd, std::size_t bytesRcvd) {
            if ((!errorRcvd) && (bytesRcvd > 0))
            {
                auto obsG = std::make_shared<PosVelAtt>();

                double posX = _data.at(0);
                double posY = _data.at(1);
                double posZ = _data.at(2);
                // double attRoll = 0.0;
                // double attPitch = 0.0;
                // double attYaw = 0.0;

                // Set GNSS values
                Eigen::Vector3d e_position{ posX, posY, posZ };
                // Eigen::Vector3d lla_position = trafo::ecef2lla_WGS84(e_position);
                // Eigen::Quaterniond e_Quat_b;
                // e_Quat_b = trafo::e_Quat_n(lla_position(0), lla_position(1)) * trafo::n_Quat_b(attRoll, attPitch, attYaw);

                obsG->setPosition_e(e_position);
                Eigen::Vector3d velDummy{ 0, 0, 0 }; // TODO: Add velocity output in Skydel API and NetStream
                obsG->setVelocity_e(velDummy);
                // obsG->setAttitude_e_Quat_b(e_Quat_b); // Attitude MUST BE set after Position, because the n- to e-sys trafo depends on lla_position

                // Set IMU values
                // obs->accelCompXYZ.emplace(accelX, accelY, accelZ);
                // obs->accelUncompXYZ = obs->accelCompXYZ;
                // obs->gyroCompXYZ.emplace(gyroX, gyroY, gyroZ);
                // obs->gyroUncompXYZ = obs->gyroCompXYZ;
                // obs->magCompXYZ.emplace(0.0, 0.0, 0.0); // TODO: Add magnetometer model to Skydel API 'InstinctDataStream'
                // obs->magUncompXYZ.emplace(0.0, 0.0, 0.0);

                InsTime currentTime = util::time::GetCurrentInsTime();
                if (!currentTime.empty())
                {
                    // obs->insTime = currentTime;
                    obsG->insTime = currentTime;
                }

                // if (obs->timeSinceStartup.has_value())
                // {
                //     if (_lastMessageTime)
                //     {
                //         // FIXME: This seems like a bug in clang-tidy. Check if it is working in future versions of clang-tidy
                //         // NOLINTNEXTLINE(hicpp-use-nullptr, modernize-use-nullptr)
                //         if (obs->timeSinceStartup.value() - _lastMessageTime >= static_cast<uint64_t>(1.5 / _dataRate * 1e9))
                //         {
                //             LOG_WARN("{}: Potentially lost a message. Previous message was at {} and current message at {} which is a time difference of {} seconds.", nameId(),
                //                      _lastMessageTime, obs->timeSinceStartup.value(), static_cast<double>(obs->timeSinceStartup.value() - _lastMessageTime) * 1e-9);
                //         }
                //     }
                //     _lastMessageTime = obs->timeSinceStartup.value();
                // }

                this->invokeCallbacks(OUTPUT_PORT_INDEX_NODE_DATA, obsG);
                // this->invokeCallbacks(OUTPUT_PORT_INDEX_IMU_OBS, obs);

                // Data rate (for visualization in GUI)
                // _packageCount++;

                // if (_startCounter < _startNow)
                // {
                //     _packageCount = 0;
                //     _startCounter++;
                // }

                // if (_packageCount == 1)
                // {
                //     _startPoint = std::chrono::steady_clock::now();
                // }
                // else if (_packageCount == _packagesNumber)
                // {
                //     std::chrono::duration<double> elapsed_seconds = std::chrono::steady_clock::now() - _startPoint;
                //     _dataRate = static_cast<double>(_packagesNumber - 1) / elapsed_seconds.count();

                //     // Dynamic adaptation of data rate to a human-readable display update rate in GUI (~ 1 Hz)
                //     if ((_dataRate > 2) && (_dataRate < 1001)) // restriction on 'reasonable' sensor data rates (Skydel max. is 1000 Hz)
                //     {
                //         _packagesNumber = static_cast<int>(_dataRate);
                //     }
                //     else if (_dataRate >= 1001)
                //     {
                //         _packagesNumber = 1000;
                //     }

                //     _packageCount = 0;

                //     LOG_DATA("Elapsed Seconds = {}", elapsed_seconds.count());
                //     LOG_DATA("SkydelNetworkStream: dataRate = {}", _dataRate);
                // }
            }
            else
            {
                LOG_ERROR("Error receiving the UDP network stream.");
            }

            if (_running)
            {
                pollData();
            }
        });
}