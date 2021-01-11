#include "UartSensor.hpp"

#include "util/Logger.hpp"

[[nodiscard]] json NAV::UartSensor::save() const
{
    LOG_TRACE("called");

    json j;

    j["sensorPort"] = sensorPort;
    j["sensorBaudrate"] = static_cast<size_t>(sensorBaudrate());

    return j;
}

void NAV::UartSensor::restore(json const& j)
{
    LOG_TRACE("called");

    if (j.contains("sensorPort"))
    {
        j.at("sensorPort").get_to(sensorPort);
    }
    if (j.contains("sensorBaudrate"))
    {
        size_t baudrate = 0;
        j.at("sensorBaudrate").get_to(baudrate);
        selectedBaudrate = baudrate2Selection(static_cast<Baudrate>(baudrate));
    }
}

NAV::UartSensor::Baudrate NAV::UartSensor::sensorBaudrate() const
{
    switch (selectedBaudrate)
    {
    case 0:
        return BAUDRATE_FASTEST;
        break;
    case 1:
        return BAUDRATE_9600;
        break;
    case 2:
        return BAUDRATE_19200;
        break;
    case 3:
        return BAUDRATE_38400;
        break;
    case 4:
        return BAUDRATE_57600;
        break;
    case 5:
        return BAUDRATE_115200;
        break;
    case 6:
        return BAUDRATE_128000;
        break;
    case 7:
        return BAUDRATE_230400;
        break;
    case 8:
        return BAUDRATE_460800;
        break;
    case 9:
        return BAUDRATE_921600;
        break;

    default:
        return BAUDRATE_FASTEST;
    }
}

int NAV::UartSensor::baudrate2Selection(Baudrate baud)
{
    switch (baud)
    {
    case BAUDRATE_FASTEST:
        return 0;
    case BAUDRATE_9600:
        return 1;
    case BAUDRATE_19200:
        return 2;
    case BAUDRATE_38400:
        return 3;
    case BAUDRATE_57600:
        return 4;
    case BAUDRATE_115200:
        return 5;
    case BAUDRATE_128000:
        return 6;
    case BAUDRATE_230400:
        return 7;
    case BAUDRATE_460800:
        return 8;
    case BAUDRATE_921600:
        return 9;
    }
    return 0;
}
