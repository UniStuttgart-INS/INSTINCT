#include "SkydelImuStream.hpp"

#include "util/Debug.hpp"
#include "util/Logger.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeData/IMU/ImuObs.hpp"

#include "util/Time/TimeBase.hpp"

// Includes for IMU Stream
// #include <cstdlib>
// #include <iostream>
#include <boost/asio.hpp>
#include <thread>
// #include <fstream>
#include <string>
#include <vector>
#include <boost/algorithm/string.hpp>

using boost::asio::ip::udp;

NAV::SkydelImuStream::SkydelImuStream()
    : m_senderEndpoint(udp::v4(), m_port), m_socket(ioservice, m_senderEndpoint)
{
    name = typeStatic();

    color = ImColor(255, 128, 128);
    hasConfig = true;

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
    /*  // Checkbox Beispiel von TT
    static bool chk1 = false;
    if (ImGui::Checkbox("Test##1", &chk1))
    {
        LOG_DEBUG("Test Button 1");
    }
    static bool chk2 = false;
    if (ImGui::Checkbox("Test##2", &chk2))
    {
        LOG_DEBUG("Test Button 2");
    }
    */
}

[[nodiscard]] json NAV::SkydelImuStream::save() const
{
    //LOG_TRACE("{}: called", nameId());

    json j;

    j["Frequency"] = 1;

    return j;
}

void NAV::SkydelImuStream::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("Frequency"))
    {
        // j.at("Frequency").get_to(outputFrequency);
    }
}

bool NAV::SkydelImuStream::resetNode()
{
    return true;
}

void NAV::SkydelImuStream::do_receive()
{
    LOG_DEBUG("stop = {} (asyncReceive)", stop);

    m_socket.async_receive_from(
        boost::asio::buffer(m_data, max_length), m_senderEndpoint,
        [this](boost::system::error_code errorRcvd, std::size_t bytesRcvd) {
            // LOG_DEBUG("errorRcvd = {}", errorRcvd);
            if (((!errorRcvd) & (bytesRcvd > 0)) | ((!errorRcvd) & (bytesRcvd > 0)))
            {
                // LOG_DEBUG("bytesRcvd {}", bytesRcvd);
                boost::algorithm::split(splittedStringRcvd, m_data, boost::is_any_of(","));

                auto obs = std::make_shared<ImuObs>(this->imuPos);
                obs->timeSinceStartup = std::stod(splittedStringRcvd.at(0)) * 1e6;

                obs->accelCompXYZ.emplace(std::stod(splittedStringRcvd.at(1)), std::stod(splittedStringRcvd.at(2)), std::stod(splittedStringRcvd.at(3)));
                obs->gyroCompXYZ.emplace(std::stod(splittedStringRcvd.at(4)), std::stod(splittedStringRcvd.at(5)), std::stod(splittedStringRcvd.at(6)));
                obs->magCompXYZ.emplace(0.0, 0.0, 0.0);

                // LOG_DEBUG("std::stod(splittedStringRcvd.at(3)) = {}", std::stod(splittedStringRcvd.at(3)));

                InsTime currentTime = util::time::GetCurrentTime();
                if (!currentTime.empty())
                {
                    obs->insTime = currentTime;
                }

                this->invokeCallbacks(OutputPortIndex_ImuObs, obs);
            }

            // LOG_DEBUG("stop = {} (asyncReceive)", stop);

            if (!stop)
            {
                do_receive();
            }
        });
}

bool NAV::SkydelImuStream::initialize()
{
    // LOG_DEBUG("stop = {} (initialize)", stop);
    stop = 0;

    do_receive();
    TestThread = std::thread([=, this]() {
        // LOG_ERROR("Initialize: TestThread before");
        ioservice.run();
        // LOG_ERROR("Initialize: TestThread after");
    });

    LOG_TRACE("{}: initialized", name);
    return true;
}

void NAV::SkydelImuStream::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    // breakStream = true;

    // LOG_TRACE("{}: de-initialized", nameId());

    // if (!isInitialized())
    // {
    //     return;
    // }
    stop = 1;
    TestThread.join();
    ioservice.restart();

    LOG_DEBUG("Deinitialize TestThread joined");

    // startTime = InsTime{};
}

void NAV::SkydelImuStream::readImuThread(void* userData)
{
    //LOG_TRACE("mm: Test");
    //return true;
    auto* skydel = static_cast<SkydelImuStream*>(userData);
    auto obs = std::make_shared<ImuObs>(skydel->imuPos);
    LOG_TRACE("mm: Test");
    //if (InsTime currentTime = util::time::GetCurrentTime();
    //   !currentTime.empty())
    //{
    //    obs->insTime = currentTime;
    //}
    skydel->invokeCallbacks(OutputPortIndex_ImuObs, obs);
}