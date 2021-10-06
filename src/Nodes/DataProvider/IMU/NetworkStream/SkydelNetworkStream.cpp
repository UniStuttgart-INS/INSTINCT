#include "SkydelNetworkStream.hpp"

#include <boost/asio.hpp>
#include <thread>
#include <string>
#include <vector>
#include <chrono>
#include <cmath>
#include <sstream>

#include "util/Debug.hpp"
#include "util/Logger.hpp"
#include "internal/NodeManager.hpp"
#include "internal/FlowManager.hpp"
#include "NodeData/IMU/ImuObs.hpp"
#include "util/Time/TimeBase.hpp"
#include "util/InsTransformations.hpp"
#include "NodeData/State/PosVelAtt.hpp"
#include "internal/gui/widgets/HelpMarker.hpp"

namespace nm = NAV::NodeManager;
using boost::asio::ip::udp;

NAV::SkydelNetworkStream::SkydelNetworkStream()
    : m_senderEndpoint(udp::v4(), 4444), m_socket(ioservice, m_senderEndpoint)
{
    name = typeStatic();

    hasConfig = true;
    guiConfigDefaultWindowSize = { 305, 70 };

    nm::CreateOutputPin(this, "ImuObs", Pin::Type::Flow, { NAV::ImuObs::type() });
    nm::CreateOutputPin(this, "PosVelAtt", Pin::Type::Flow, { NAV::PosVelAtt::type() });
}

NAV::SkydelNetworkStream::~SkydelNetworkStream()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::SkydelNetworkStream::typeStatic()
{
    return "SkydelNetworkStream";
}

std::string NAV::SkydelNetworkStream::type() const
{
    return typeStatic();
}

std::string NAV::SkydelNetworkStream::category()
{
    return "Data Provider";
}

void NAV::SkydelNetworkStream::guiConfig()
{
    std::string str;

    if (startCounter < startNow)
    {
        str = "(loading)";
    }
    else
    {
        std::ostringstream strs;
        strs << dataRate;
        str = strs.str();
    }

    ImGui::LabelText(str.c_str(), "data rate [Hz]");
    ImGui::SameLine();
    gui::widgets::HelpMarker("The data rate can be adjusted in Skydel: Settings/Plug-ins/<Plug-in-name>/Plug-in UI. Make sure to enable either WiFi or a LAN connection. Enabling both can lead to loss of data, because Skydel only knows one ip address.");
}

bool NAV::SkydelNetworkStream::resetNode()
{
    return true;
}

void NAV::SkydelNetworkStream::do_receive()
{
    m_socket.async_receive_from(
        boost::asio::buffer(m_data, max_length), m_senderEndpoint,
        [this](boost::system::error_code errorRcvd, std::size_t bytesRcvd) {
            if ((!errorRcvd) && (bytesRcvd > 0))
            {
                // Splitting the incoming string analogous to 'ImuFile.cpp'
                std::stringstream lineStream(std::string(m_data.begin(), m_data.end()));
                std::string cell;
                auto obsG = std::make_shared<PosVelAtt>();
                auto obs = std::make_shared<ImuObs>(this->imuPos);

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
                Eigen::Vector3d pos_ecef{ posX, posY, posZ };
                Eigen::Vector3d posLLA = trafo::ecef2lla_WGS84(pos_ecef);
                Eigen::Quaterniond quat_eb;
                quat_eb = trafo::quat_en(posLLA(0), posLLA(1)) * trafo::quat_nb(attRoll, attPitch, attYaw);

                obsG->setPosition_e(pos_ecef, posLLA);
                Eigen::Vector3d velDummy{ 0, 0, 0 }; //TODO: Add velocity output in Skydel API and NetStream
                obsG->setVelocity_e(velDummy);
                obsG->setAttitude_eb(quat_eb); // Attitude MUST BE set after Position, because the n- to e-sys trafo depends on posLLA

                // Set IMU values
                obs->accelCompXYZ.emplace(accelX, accelY, accelZ);
                obs->gyroCompXYZ.emplace(gyroX, gyroY, gyroZ);
                obs->magCompXYZ.emplace(0.0, 0.0, 0.0); //TODO: Add magnetometer model to Skydel API 'InstinctDataStream'
                obs->magUncompXYZ.emplace(0.0, 0.0, 0.0);

                InsTime currentTime = util::time::GetCurrentInsTime();
                if (!currentTime.empty())
                {
                    obs->insTime = currentTime;
                    obsG->insTime = currentTime;
                }

                this->invokeCallbacks(OutputPortIndex_GnssObs, obsG);
                this->invokeCallbacks(OutputPortIndex_ImuObs, obs);

                // Data rate (for visualization in GUI)
                packageCount++;

                if (startCounter < startNow)
                {
                    packageCount = 0;
                    startCounter++;
                }

                if (packageCount == 1)
                {
                    startPoint = std::chrono::steady_clock::now();
                }
                else if (packageCount == packagesNumber)
                {
                    std::chrono::duration<double> elapsed_seconds = std::chrono::steady_clock::now() - startPoint;
                    dataRate = static_cast<double>(packagesNumber - 1) / elapsed_seconds.count();

                    // Dynamic adaptation of data rate to a human-readable display update rate in GUI (~ 1 Hz)
                    if ((dataRate > 2) && (dataRate < 1001)) // restriction on 'reasonable' sensor data rates (Skydel max. is 1000 Hz)
                    {
                        packagesNumber = static_cast<int>(dataRate);
                    }
                    else if (dataRate >= 1001)
                    {
                        packagesNumber = 1000;
                    }

                    packageCount = 0;

                    LOG_DATA("Elapsed Seconds = {}", elapsed_seconds.count());
                    LOG_DATA("SkydelNetworkStream: dataRate = {}", dataRate);
                }
            }
            else
            {
                LOG_ERROR("Error receiving the network stream from Skydel");
            }

            if (!stop)
            {
                do_receive();
            }
        });
}

bool NAV::SkydelNetworkStream::initialize()
{
    LOG_TRACE("{}: called", nameId());

    stop = false;
    packageCount = 0;
    startCounter = 0;
    packagesNumber = 2;

    do_receive();

    if (isStartup)
    {
        TestThread = std::thread([=, this]() {
            ioservice.run();
        });
    }
    else
    {
        TestThread = std::thread([=, this]() {
            ioservice.restart();
            ioservice.run();
        });
    }

    isStartup = false;

    return true;
}

void NAV::SkydelNetworkStream::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    stop = true;
    ioservice.stop();
    TestThread.join();
}
