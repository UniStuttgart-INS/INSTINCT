#include "SkydelImuStream.hpp"

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

namespace nm = NAV::NodeManager;
using boost::asio::ip::udp;

NAV::SkydelImuStream::SkydelImuStream()
    : m_senderEndpoint(udp::v4(), 4444), m_socket(ioservice, m_senderEndpoint)
{
    name = typeStatic();

    hasConfig = false;

    nm::CreateOutputPin(this, "ImuObs", Pin::Type::Flow, NAV::ImuObs::type());
}

NAV::SkydelImuStream::~SkydelImuStream()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::SkydelImuStream::typeStatic()
{
    return "SkydelImuStream";
}

std::string NAV::SkydelImuStream::type() const
{
    return typeStatic();
}

std::string NAV::SkydelImuStream::category()
{
    return "Data Provider";
}

void NAV::SkydelImuStream::guiConfig()
{
    //TODO: Configure slider to enable custom data rate setting
}

bool NAV::SkydelImuStream::resetNode()
{
    return true;
}

void NAV::SkydelImuStream::do_receive()
{
    m_socket.async_receive_from(
        boost::asio::buffer(m_data, max_length), m_senderEndpoint,
        [this](boost::system::error_code errorRcvd, std::size_t bytesRcvd) {
            if ((!errorRcvd) && (bytesRcvd > 0))
            {
                // Splitting the incoming string analogous to 'ImuFile.cpp'
                std::stringstream lineStream(std::string(m_data.begin(), m_data.end()));
                std::string cell;
                auto obs = std::make_shared<ImuObs>(this->imuPos);

                //  Inits for simulated measurement variables
                double accelX = 0.0;
                double accelY = 0.0;
                double accelZ = 0.0;
                double gyroX = 0.0;
                double gyroY = 0.0;
                double gyroZ = 0.0;

                for (size_t i = 0; i < 7; i++)
                {
                    // Reading string from csv
                    if (std::getline(lineStream, cell, ','))
                    {
                        switch (i)
                        {
                        case 0:
                            obs->timeSinceStartup = std::stod(cell) * 1e6;
                            break;
                        case 1:
                            accelX = std::stod(cell);
                            break;
                        case 2:
                            accelY = std::stod(cell);
                            break;
                        case 3:
                            accelZ = std::stod(cell);
                            break;
                        case 4:
                            gyroX = std::stod(cell);
                            break;
                        case 5:
                            gyroY = std::stod(cell);
                            break;
                        case 6:
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
                obs->accelCompXYZ.emplace(accelX, accelY, accelZ);
                obs->gyroCompXYZ.emplace(gyroX, gyroY, gyroZ);

                InsTime currentTime = util::time::GetCurrentInsTime();
                if (!currentTime.empty())
                {
                    obs->insTime = currentTime;
                }

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

bool NAV::SkydelImuStream::initialize()
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

void NAV::SkydelImuStream::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    stop = true;
    ioservice.stop();
    TestThread.join();
}
