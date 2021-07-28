#include "SkydelNetworkStream.hpp"

#include <boost/asio.hpp>
#include <thread>
#include <string>
#include <vector>

#include "util/Debug.hpp"
#include "util/Logger.hpp"
#include "internal/NodeManager.hpp"
#include "internal/FlowManager.hpp"
#include "NodeData/IMU/ImuObs.hpp"
#include "util/Time/TimeBase.hpp"
#include "NodeData/GNSS/SkydelObs.hpp"

namespace nm = NAV::NodeManager;
using boost::asio::ip::udp;

NAV::SkydelNetworkStream::SkydelNetworkStream()
    : m_senderEndpoint(udp::v4(), 4444), m_socket(ioservice, m_senderEndpoint)
{
    name = typeStatic();

    hasConfig = false;

    nm::CreateOutputPin(this, "ImuObs", Pin::Type::Flow, NAV::ImuObs::type());
    nm::CreateOutputPin(this, "SkydelObs", Pin::Type::Flow, NAV::SkydelObs::type());
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
    //TODO: Configure slider to enable custom data rate setting
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
                auto obsG = std::make_shared<SkydelObs>();
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

                // Arranging the network stream data into output format
                obsG->posXYZ.emplace(posX, posY, posZ);
                obsG->attRPY.emplace(attRoll, attPitch, attYaw);

                obs->accelCompXYZ.emplace(accelX, accelY, accelZ);
                obs->gyroCompXYZ.emplace(gyroX, gyroY, gyroZ);

                InsTime currentTime = util::time::GetCurrentInsTime();
                if (!currentTime.empty())
                {
                    obs->insTime = currentTime;
                    obsG->insTime = currentTime;
                }

                this->invokeCallbacks(OutputPortIndex_GnssObs, obsG);
                this->invokeCallbacks(OutputPortIndex_ImuObs, obs);
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
