// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "SkydelNetworkStream.hpp"

#include <boost/asio.hpp>
#include <thread>
#include <string>
#include <vector>
#include <chrono>
#include <cmath>
#include <sstream>

#include "util/Logger.hpp"
#include "internal/NodeManager.hpp"
#include "internal/FlowManager.hpp"
#include "NodeData/IMU/ImuObs.hpp"
#include "util/Time/TimeBase.hpp"
#include "Navigation/Transformations/CoordinateFrames.hpp"
#include "NodeData/State/PosVelAtt.hpp"
#include "internal/gui/widgets/HelpMarker.hpp"

namespace nm = NAV::NodeManager;
using boost::asio::ip::udp;

namespace NAV::experimental
{

SkydelNetworkStream::SkydelNetworkStream()
    : Imu(typeStatic()), _senderEndpoint(udp::v4(), 4444), _socket(_ioservice, _senderEndpoint)
{
    _onlyRealTime = true;
    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 305, 70 };

    nm::CreateOutputPin(this, "ImuObs", Pin::Type::Flow, { NAV::ImuObs::type() });
    nm::CreateOutputPin(this, "PosVelAtt", Pin::Type::Flow, { NAV::PosVelAtt::type() });
}

SkydelNetworkStream::~SkydelNetworkStream()
{
    LOG_TRACE("{}: called", nameId());
}

std::string SkydelNetworkStream::typeStatic()
{
    return "SkydelNetworkStream";
}

std::string SkydelNetworkStream::type() const
{
    return typeStatic();
}

std::string SkydelNetworkStream::category()
{
    return "Data Provider";
}

void SkydelNetworkStream::guiConfig()
{
    std::string str;

    if (_startCounter < _startNow)
    {
        str = "(loading)";
    }
    else
    {
        std::ostringstream strs;
        strs << _dataRate;
        str = strs.str();
    }

    ImGui::LabelText(str.c_str(), "data rate [Hz]");
    ImGui::SameLine();
    gui::widgets::HelpMarker("The data rate can be adjusted in Skydel: Settings/Plug-ins/<Plug-in-name>/Plug-in UI. Make sure to enable either WiFi or a LAN connection. Enabling both can lead to loss of data, because Skydel only knows one ip address.");
}

bool SkydelNetworkStream::resetNode()
{
    return true;
}

void SkydelNetworkStream::do_receive()
{
    _socket.async_receive_from(
        boost::asio::buffer(_data, _maxLength), _senderEndpoint,
        [this](boost::system::error_code errorRcvd, std::size_t bytesRcvd) {
            if ((!errorRcvd) && (bytesRcvd > 0))
            {
                // Splitting the incoming string analogous to 'ImuFile.cpp'
                std::stringstream lineStream(std::string(_data.begin(), _data.end()));
                std::string cell;
                auto obsG = std::make_shared<PosVelAtt>();
                auto obs = std::make_shared<ImuObs>(this->_imuPos);

                //  Inits for simulated measurement variables
                double posX = 0.0;
                double posY = 0.0;
                double posZ = 0.0;
                double attRoll = 0.0;
                double attPitch = 0.0;
                double attYaw = 0.0;

                double accelX = 0.0;
                double accelY = 0.0;
                double accelZ = 0.0;
                double gyroX = 0.0;
                double gyroY = 0.0;
                double gyroZ = 0.0;

                for (size_t i = 0; i < 13; i++)
                {
                    // Reading string from csv
                    if (std::getline(lineStream, cell, ','))
                    {
                        switch (i)
                        {
                        case 0:
                            obs->timeSinceStartup = std::stod(cell) * 1e6; // [ns] = [ms] * 1e6
                            break;
                        case 1:
                            posX = std::stod(cell);
                            break;
                        case 2:
                            posY = std::stod(cell);
                            break;
                        case 3:
                            posZ = std::stod(cell);
                            break;
                        case 4:
                            attRoll = std::stod(cell);
                            break;
                        case 5:
                            attPitch = std::stod(cell);
                            break;
                        case 6:
                            attYaw = std::stod(cell);
                            break;
                        case 7:
                            accelX = std::stod(cell);
                            break;
                        case 8:
                            accelY = std::stod(cell);
                            break;
                        case 9:
                            accelZ = std::stod(cell);
                            break;
                        case 10:
                            gyroX = std::stod(cell);
                            break;
                        case 11:
                            gyroY = std::stod(cell);
                            break;
                        case 12:
                            gyroZ = std::stod(cell);
                            break;

                        default:
                            LOG_ERROR("Error in network stream: Cell index is out of bounds");
                            break;
                        }
                    }
                    else
                    {
                        LOG_ERROR("Error in IMU stream: Reading a string from csv failed");
                        return;
                    }
                }

                // Set GNSS values
                Eigen::Vector3d e_position{ posX, posY, posZ };
                Eigen::Vector3d lla_position = trafo::ecef2lla_WGS84(e_position);
                Eigen::Quaterniond e_Quat_b;
                e_Quat_b = trafo::e_Quat_n(lla_position(0), lla_position(1)) * trafo::n_Quat_b(attRoll, attPitch, attYaw);

                obsG->setPosition_e(e_position);
                Eigen::Vector3d velDummy{ 0, 0, 0 }; // TODO: Add velocity output in Skydel API and NetStream
                obsG->setVelocity_e(velDummy);
                obsG->setAttitude_e_Quat_b(e_Quat_b); // Attitude MUST BE set after Position, because the n- to e-sys trafo depends on lla_position

                // Set IMU values
                obs->accelCompXYZ.emplace(accelX, accelY, accelZ);
                obs->accelUncompXYZ = obs->accelCompXYZ;
                obs->gyroCompXYZ.emplace(gyroX, gyroY, gyroZ);
                obs->gyroUncompXYZ = obs->gyroCompXYZ;
                obs->magCompXYZ.emplace(0.0, 0.0, 0.0); // TODO: Add magnetometer model to Skydel API 'InstinctDataStream'
                obs->magUncompXYZ.emplace(0.0, 0.0, 0.0);

                InsTime currentTime = util::time::GetCurrentInsTime();
                if (!currentTime.empty())
                {
                    obs->insTime = currentTime;
                    obsG->insTime = currentTime;
                }

                if (obs->timeSinceStartup.has_value())
                {
                    if (_lastMessageTime)
                    {
                        // FIXME: This seems like a bug in clang-tidy. Check if it is working in future versions of clang-tidy
                        // NOLINTNEXTLINE(hicpp-use-nullptr, modernize-use-nullptr)
                        if (obs->timeSinceStartup.value() - _lastMessageTime >= static_cast<uint64_t>(1.5 / _dataRate * 1e9))
                        {
                            LOG_WARN("{}: Potentially lost a message. Previous message was at {} and current message at {} which is a time difference of {} seconds.", nameId(),
                                     _lastMessageTime, obs->timeSinceStartup.value(), static_cast<double>(obs->timeSinceStartup.value() - _lastMessageTime) * 1e-9);
                        }
                    }
                    _lastMessageTime = obs->timeSinceStartup.value();
                }

                this->invokeCallbacks(OUTPUT_PORT_INDEX_GNSS_OBS, obsG);
                this->invokeCallbacks(OUTPUT_PORT_INDEX_IMU_OBS, obs);

                // Data rate (for visualization in GUI)
                _packageCount++;

                if (_startCounter < _startNow)
                {
                    _packageCount = 0;
                    _startCounter++;
                }

                if (_packageCount == 1)
                {
                    _startPoint = std::chrono::steady_clock::now();
                }
                else if (_packageCount == _packagesNumber)
                {
                    std::chrono::duration<double> elapsed_seconds = std::chrono::steady_clock::now() - _startPoint;
                    _dataRate = static_cast<double>(_packagesNumber - 1) / elapsed_seconds.count();

                    // Dynamic adaptation of data rate to a human-readable display update rate in GUI (~ 1 Hz)
                    if ((_dataRate > 2) && (_dataRate < 1001)) // restriction on 'reasonable' sensor data rates (Skydel max. is 1000 Hz)
                    {
                        _packagesNumber = static_cast<int>(_dataRate);
                    }
                    else if (_dataRate >= 1001)
                    {
                        _packagesNumber = 1000;
                    }

                    _packageCount = 0;

                    LOG_DATA("Elapsed Seconds = {}", elapsed_seconds.count());
                    LOG_DATA("SkydelNetworkStream: dataRate = {}", _dataRate);
                }
            }
            else
            {
                LOG_ERROR("Error receiving the network stream from Skydel");
            }

            if (!_stop)
            {
                do_receive();
            }
        });
}

bool SkydelNetworkStream::initialize()
{
    LOG_TRACE("{}: called", nameId());

    _stop = false;
    _packageCount = 0;
    _startCounter = 0;
    _packagesNumber = 2;

    _lastMessageTime = 0;

    do_receive();

    if (_isStartup)
    {
        _testThread = std::thread([=, this]() {
            _ioservice.run();
        });
    }
    else
    {
        _testThread = std::thread([=, this]() {
            _ioservice.restart();
            _ioservice.run();
        });
    }

    _isStartup = false;

    return true;
}

void SkydelNetworkStream::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    _stop = true;
    _ioservice.stop();
    _testThread.join();
}

} // namespace NAV::experimental