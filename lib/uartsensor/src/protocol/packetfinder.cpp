#include "uart/protocol/packetfinder.hpp"
#include "uart/util/utilities.hpp"

#include <arpa/inet.h>

#include <queue>
#include <list>
#include <cstring>
#include <stdexcept>
#include <iostream>

namespace uart::protocol
{
void PacketFinder::processReceivedData(const std::vector<uint8_t>& data, const xplat::TimeStamp& timestamp)
{
    if (_processReceivedDataHandler != nullptr && _possiblePacketFoundHandler != nullptr)
    {
        _processReceivedDataHandler(data, timestamp, _possiblePacketFoundHandler, _possiblePacketFoundUserData);
    }
}

void PacketFinder::registerProcessReceivedDataHandler(void* userData, ProcessReceivedDataHandler handler)
{
    if (_processReceivedDataHandler != nullptr)
    {
        throw std::logic_error("Unregister packet handler first");
    }

    _processReceivedDataHandler = handler;
    _processReceivedDataUserData = userData;
}

void PacketFinder::unregisterProcessReceivedDataHandler()
{
    if (_processReceivedDataHandler == nullptr)
    {
        throw std::logic_error("Register received data handler first");
    }

    _processReceivedDataHandler = nullptr;
    _processReceivedDataUserData = nullptr;
}

void PacketFinder::registerPossiblePacketFoundHandler(void* userData, ValidPacketFoundHandler handler)
{
    if (_possiblePacketFoundHandler != nullptr)
    {
        throw std::logic_error("Unregister packet handler first");
    }

    _possiblePacketFoundHandler = handler;
    _possiblePacketFoundUserData = userData;
}

void PacketFinder::unregisterPossiblePacketFoundHandler()
{
    if (_possiblePacketFoundHandler == nullptr)
    {
        throw std::logic_error("Register packet handler first");
    }

    _possiblePacketFoundHandler = nullptr;
    _possiblePacketFoundUserData = nullptr;
}

} // namespace uart::protocol