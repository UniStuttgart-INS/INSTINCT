// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef HAS_UARTSENSOR_LIBRARY

    #include "uart/sensors/sensors.hpp"
    #include "uart/xplat/timestamp.hpp"

    #include <string>
    #include <queue>
    #include <string>
    #include <cstdio>
    #include <stdexcept>
    #include <array>
    #include <vector>

namespace uart::sensors
{

std::vector<uint32_t> UartSensor::supportedBaudrates()
{
    return {
        9600,
        19200,
        38400,
        57600,
        115200,
        128000,
        230400,
        460800,
        921600
    };
}

UartSensor::UartSensor(Endianness endianness,
                       PacketFinderFunction packetFinderFunction,
                       void* packetFinderUserData,
                       PacketTypeFunction packetTypeFunction,
                       PacketCheckFunction isValidFunction,
                       PacketCheckFunction isErrorFunction,
                       PacketCheckFunction isResponseFunction,
                       size_t packetHeaderLength)
    : _endianness(endianness),
      _packetFinderFunction(packetFinderFunction),
      _packetFinderUserData(packetFinderUserData),
      _packetTypeFunction(packetTypeFunction),
      _isValidFunction(isValidFunction),
      _isErrorFunction(isErrorFunction),
      _isResponseFunction(isResponseFunction),
      _packetHeaderLength(packetHeaderLength) {}

uint32_t UartSensor::baudrate()
{
    return 115200;
}

std::string UartSensor::port()
{
    return "N/A";
}

bool UartSensor::isConnected()
{
    return false;
}

bool UartSensor::verifySensorConnectivity()
{
    return false;
}

void UartSensor::connect(const std::string& /* portName */, uint32_t /* baudrate */)
{
}

void UartSensor::disconnect()
{
}

void UartSensor::registerRawDataReceivedHandler(void* /* userData */, RawDataReceivedHandler /* handler */)
{
}

void UartSensor::unregisterRawDataReceivedHandler()
{
}

void UartSensor::registerPossiblePacketFoundHandler(void* /* userData */, PossiblePacketFoundHandler /* handler */)
{
}

void UartSensor::unregisterPossiblePacketFoundHandler()
{
}

void UartSensor::registerAsyncPacketReceivedHandler(void* /* userData */, AsyncPacketReceivedHandler /* handler */)
{
}

void UartSensor::unregisterAsyncPacketReceivedHandler()
{
}

void UartSensor::registerErrorPacketReceivedHandler(void* /* userData */, ErrorPacketReceivedHandler /* handler */)
{
}

void UartSensor::unregisterErrorPacketReceivedHandler()
{
}

void UartSensor::changeBaudRate(uint32_t /* baudrate */)
{
}

} // namespace uart::sensors

#endif