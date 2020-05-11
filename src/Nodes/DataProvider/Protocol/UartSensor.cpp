#include "UartSensor.hpp"

#include "util/Logger.hpp"

NAV::UartSensor::UartSensor(std::deque<std::string>& options)
{
    LOG_TRACE("called");

    if (!options.empty())
    {
        sensorPort = options.at(0);
        options.pop_front();

        if (!options.empty())
        {
            if (options.at(0) == "Fastest")
            {
                sensorBaudrate = UartSensor::Baudrate::BAUDRATE_FASTEST;
            }
            else
            {
                sensorBaudrate = static_cast<UartSensor::Baudrate>(std::stoul(options.at(0)));
            }
            options.pop_front();
        }
    }
}

NAV::UartSensor::~UartSensor()
{
    LOG_TRACE("called");
}