#include "TimeLimiter.hpp"

#include "util/Logger.hpp"

NAV::TimeLimiter::TimeLimiter(const std::string& name, const std::map<std::string, std::string>& options)
    : Node(name)
{
    LOG_TRACE("called for {}", name);

    if (options.count("Lower Limit Gps Cycle")
        && options.count("Lower Limit Gps Week")
        && options.count("Lower Limit Gps Time of Week"))
    {
        auto cycle = static_cast<uint16_t>(std::stoul(options.at("Lower Limit Gps Cycle")));
        auto week = static_cast<uint16_t>(std::stoul(options.at("Lower Limit Gps Week")));
        long double tow = std::stold(options.at("Lower Limit Gps Time of Week"));

        lowerLimit = InsTime(cycle, week, tow);
    }

    if (options.count("Upper Limit Gps Cycle")
        && options.count("Upper Limit Gps Week")
        && options.count("Upper Limit Gps Time of Week"))
    {
        auto cycle = static_cast<uint16_t>(std::stoul(options.at("Upper Limit Gps Cycle")));
        auto week = static_cast<uint16_t>(std::stoul(options.at("Upper Limit Gps Week")));
        long double tow = std::stold(options.at("Upper Limit Gps Time of Week"));

        upperLimit = InsTime(cycle, week, tow);
    }

    if (options.count("1-Port Type"))
    {
        portDataType = options.at("1-Port Type");
    }
}