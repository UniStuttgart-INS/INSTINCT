// #include "uart/protocol/packetfinder.hpp"
// #include "uart/util/utilities.hpp"

// #include <arpa/inet.h>

// #include <queue>
// #include <list>
// #include <cstring>
// #include <stdexcept>
// #include <iostream>

// namespace uart::protocol
// {
// void PacketFinder::processReceivedData(const std::vector<uint8_t>& data, const xplat::TimeStamp& timestamp)
// {
//     if (_packageFinderFunction != nullptr && _possiblePacketFoundHandler != nullptr)
//     {
//         _packageFinderFunction(data, timestamp, _possiblePacketFoundHandler, _possiblePacketFoundUserData);
//     }
// }

// void PacketFinder::registerPossiblePacketFoundHandler(void* userData, ValidPacketFoundHandler handler)
// {
//     if (_possiblePacketFoundHandler != nullptr)
//     {
//         throw std::logic_error("Unregister packet handler first");
//     }

//     _possiblePacketFoundHandler = handler;
//     _possiblePacketFoundUserData = userData;
// }

// void PacketFinder::unregisterPossiblePacketFoundHandler()
// {
//     if (_possiblePacketFoundHandler == nullptr)
//     {
//         throw std::logic_error("Register packet handler first");
//     }

//     _possiblePacketFoundHandler = nullptr;
//     _possiblePacketFoundUserData = nullptr;
// }

// } // namespace uart::protocol