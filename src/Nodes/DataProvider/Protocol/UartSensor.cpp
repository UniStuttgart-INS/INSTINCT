#include "UartSensor.hpp"

NAV::UartSensor::UartSensor(std::deque<std::string>& options)
{
    if (options.size() >= 1)
    {
        sensorPort = options.at(0);
        options.pop_front();
    }
    if (options.size() >= 1)
    {
        if (options.at(0) == "Fastest")
            sensorBaudrate = UartSensor::Baudrate::BAUDRATE_FASTEST;
        else
            sensorBaudrate = static_cast<UartSensor::Baudrate>(std::stoul(options.at(0)));
        options.pop_front();
    }
}

NAV::UartSensor::~UartSensor() {}