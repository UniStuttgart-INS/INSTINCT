#include "TimeSynchronizer.hpp"

#include "util/Logger.hpp"

#include "NodeData/IMU/VectorNavObs.hpp"
#include "NodeData/GNSS/UbloxObs.hpp"
#include "ub/protocol/types.hpp"

NAV::TimeSynchronizer::TimeSynchronizer(const std::string& name, std::deque<std::string>& options)
    : Node(name)
{
    LOG_TRACE("called for {}", name);

    if (!options.empty())
    {
        useFixedStartTime = static_cast<bool>(std::stoi(options.at(0)));
        options.pop_front();
    }
    if (useFixedStartTime && options.size() >= 3)
    {
        auto cycle = static_cast<uint16_t>(std::stoul(options.at(0)));
        auto week = static_cast<uint16_t>(std::stoul(options.at(1)));
        long double tow = std::stold(options.at(2));

        startupGpsTime.emplace(week, tow, cycle);

        options.pop_front();
        options.pop_front();
        options.pop_front();
    }
}

void NAV::TimeSynchronizer::syncTime(std::shared_ptr<NAV::InsObs>& obs)
{
    if (obs->insTime.has_value() && !startupGpsTime.has_value())
    {
        startupGpsTime = obs->insTime;
    }
}

void NAV::TimeSynchronizer::syncVectorNavSensor(std::shared_ptr<NAV::VectorNavObs>& obs)
{
    LOG_TRACE("called for {}", name);

    if (startupGpsTime.has_value())
    {
        if (!startupImuTime.has_value())
        {
            startupImuTime = obs->timeSinceStartup;
        }
        else
        {
            obs->insTime = startupGpsTime;
            obs->insTime.value().addDiffSec(static_cast<long double>(obs->timeSinceStartup.value() - startupImuTime.value()) / 1000000000.0L);
        }
    }

    return invokeCallbacks(obs);
}