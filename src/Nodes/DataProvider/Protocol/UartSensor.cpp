#include "UartSensor.hpp"

NAV::UartSensor::UartSensor(const std::string sensorPort, const Baudrate sensorBaudrate)
    : sensorPort(sensorPort), sensorBaudrate(sensorBaudrate) {}

NAV::UartSensor::UartSensor() {}

NAV::UartSensor::~UartSensor() {}