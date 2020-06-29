#include "TimeSynchronizer.hpp"

#include "util/Logger.hpp"

NAV::TimeSynchronizer::TimeSynchronizer(const std::string& name, const std::map<std::string, std::string>& options)
    : Node(name)
{
    LOG_TRACE("called for {}", name);

    if (options.count("Use Fixed Start Time"))
    {
        useFixedStartTime = static_cast<bool>(std::stoi(options.at("Use Fixed Start Time")));
    }
    if (useFixedStartTime && options.count("Gps Cycle") && options.count("Gps Week") && options.count("Gps Time of Week"))
    {
        auto cycle = static_cast<uint16_t>(std::stoul(options.at("Gps Cycle")));
        auto week = static_cast<uint16_t>(std::stoul(options.at("Gps Week")));
        long double tow = std::stold(options.at("Gps Time of Week"));

        startupGpsTime.emplace(week, tow, cycle);
    }
}

bool NAV::TimeSynchronizer::updateInsTime(std::shared_ptr<NAV::VectorNavObs>& obs)
{
    if (obs == nullptr)
    {
        return false;
    }

    if (startupGpsTime.has_value())
    {
        obs->insTime = startupGpsTime;

        if (!startupImuTime.has_value())
        {
            startupImuTime = obs->timeSinceStartup;
        }
        else
        {
            obs->insTime.value().addDiffSec(static_cast<long double>(obs->timeSinceStartup.value() - startupImuTime.value()) / 1000000000.0L);
        }
        return true;
    }

    return obs->insTime.has_value();
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
    if (updateInsTime(obs))
    {
        invokeCallbacks(obs);
    }
}