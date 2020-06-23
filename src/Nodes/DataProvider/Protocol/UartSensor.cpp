#include "UartSensor.hpp"

#include "util/Logger.hpp"

NAV::UartSensor::UartSensor(const std::map<std::string, std::string>& options)
{
    LOG_TRACE("called");

    if (options.count("Port"))
    {
        sensorPort = options.at("Port");

        if (options.count("Baudrate"))
        {
            if (options.at("Baudrate") == "Fastest")
            {
                sensorBaudrate = UartSensor::Baudrate::BAUDRATE_FASTEST;
            }
            else
            {
                sensorBaudrate = static_cast<UartSensor::Baudrate>(std::stoul(options.at("Baudrate")));
            }
        }
    }
}