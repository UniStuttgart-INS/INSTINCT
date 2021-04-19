#include "SkydelImuStream.hpp"

#include "util/Debug.hpp"
#include "util/Logger.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeData/IMU/ImuObs.hpp"

#include "util/Time/TimeBase.hpp"

NAV::SkydelImuStream::SkydelImuStream()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

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
}

[[nodiscard]] json NAV::SkydelImuStream::save() const
{
    LOG_TRACE("{}: called", nameId());

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

bool NAV::SkydelImuStream::initialize()
{
    LOG_TRACE("{}: called", nameId());

    return true;
}

void NAV::SkydelImuStream::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

// void NAV::SkydelImuStream::readImuThread()
// void NAV::SkydelImuStream::readImuThread(void* userData)
// {
//     auto* navio = static_cast<SkydelImuStream*>(userData);
// auto obs = std::make_shared<ImuObs>(navio->imuPos);

//     LOG_DATA("DATA({}): {}, {}°C, a=({}, {}, {})", navio->name, obs->timeSinceStartup.value(), obs->temperature.value(),
//              navio->ax, navio->ay, navio->az);

//     if (InsTime currentTime = util::time::GetCurrentTime();
//         !currentTime.empty())
//     {
//         obs->insTime = currentTime;
//     }
// navio->invokeCallbacks(OutputPortIndex_ImuObs, obs);
// }