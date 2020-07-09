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

    if (options.count("1-Port Type"))
    {
        portDataType = options.at("1-Port Type");
    }
}

void NAV::TimeSynchronizer::syncTime(std::shared_ptr<NAV::InsObs>& obs)
{
    if (obs->insTime.has_value() && !startupGpsTime.has_value())
    {
        startupGpsTime = obs->insTime;
    }
}

bool NAV::TimeSynchronizer::syncVectorNavObs(std::shared_ptr<NAV::VectorNavObs>& obs)
{
    if (obs == nullptr)
    {
        return false;
    }

    if (startupGpsTime.has_value())
    {
        static uint64_t startupImuTime = obs->timeSinceStartup.value();

        obs->insTime = startupGpsTime;
        obs->insTime.value().addDiffSec(static_cast<long double>(obs->timeSinceStartup.value() - startupImuTime) / 1000000000.0L);

        return true;
    }

    return obs->insTime.has_value();
}

bool NAV::TimeSynchronizer::syncKvhObs(std::shared_ptr<NAV::KvhObs>& obs)
{
    if (obs == nullptr)
    {
        return false;
    }

    if (startupGpsTime.has_value())
    {
        constexpr long double dataRate = 1000.0L;
        static uint8_t prevSequenceNumber = obs->sequenceNumber;

        int sequenceNumberDiff = obs->sequenceNumber - prevSequenceNumber;
        if (sequenceNumberDiff < -100)
        {
            sequenceNumberDiff = obs->sequenceNumber + 128 - prevSequenceNumber;
        }
        prevSequenceNumber = obs->sequenceNumber;

        obs->insTime = startupGpsTime;
        obs->insTime.value().addDiffSec(static_cast<long double>(sequenceNumberDiff) / dataRate);

        startupGpsTime = obs->insTime;
    }

    return obs->insTime.has_value();
}
