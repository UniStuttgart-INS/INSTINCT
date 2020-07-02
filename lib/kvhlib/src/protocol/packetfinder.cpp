#include "kvh/protocol/packetfinder.hpp"
#include "kvh/util/utilities.hpp"
#include "kvh/util/checksum.hpp"

#include <arpa/inet.h>

#include <queue>
#include <list>
#include <cstring>
#include <stdexcept>
#include <iostream>
#include <array>

using namespace std;
using namespace kvh::xplat;
using namespace kvh::util;

namespace kvh::protocol::uart
{
struct PacketFinder::Impl
{
    static const size_t DefaultReceiveBufferSize = 1024;
    static const uint8_t AsciiEndChar1 = '\r';
    static const uint8_t AsciiEndChar2 = '\n';
    static const uint8_t AsciiEscapeChar = '\0';
    static const size_t MaximumSizeForAsciiPacket = 256;

    xplat::TimeStamp timeFound;
    bool currentlyBuildingAsciiPacket;
    bool currentlyBuildingBinaryPacket;
    bool asciiEndChar1Found;

    Packet::Type packetType;

    unsigned char* _buffer;
    const size_t _bufferSize;
    size_t _bufferAppendLocation;
    size_t _runningDataIndex; // Used for correlating raw data with where the packet was found for the end user.
    void* _possiblePacketFoundUserData;
    ValidPacketFoundHandler _possiblePacketFoundHandler;

    explicit Impl(PacketFinder* backReference)
        : _buffer(new uint8_t[DefaultReceiveBufferSize]),
          _bufferSize(DefaultReceiveBufferSize),
          _bufferAppendLocation(0),
          _runningDataIndex(0),
          _possiblePacketFoundUserData(NULL),
          _possiblePacketFoundHandler(NULL),
          currentlyBuildingAsciiPacket(false),
          currentlyBuildingBinaryPacket(false),
          asciiEndChar1Found(false)

    {
    }

    Impl(PacketFinder* backReference, size_t internalReceiveBufferSize)
        : _buffer(new uint8_t[internalReceiveBufferSize]),
          _bufferSize(internalReceiveBufferSize),
          _bufferAppendLocation(0),
          _runningDataIndex(0),
          _possiblePacketFoundUserData(NULL),
          _possiblePacketFoundHandler(NULL),
          currentlyBuildingAsciiPacket(false),
          currentlyBuildingBinaryPacket(false),
          asciiEndChar1Found(false)
    {
    }

    ~Impl()
    {
        delete[] _buffer;
    }

    void resetTracking()
    {
        currentlyBuildingAsciiPacket = false;
        currentlyBuildingBinaryPacket = false;
        asciiEndChar1Found = false;
        _bufferAppendLocation = 0;
    }

    static Packet::Type bFindImuHeader(uint8_t ui8Data)
    {
        typedef enum tagState
        {
            SM_H1,
            SM_H2,
            SM_H3,
            SM_IDLE
        } E_STATE;

        static E_STATE eState = SM_IDLE;

        bool xbit = false;

        if (eState == SM_IDLE)
        {
            if (ui8Data == ((Packet::HEADER_FMT_A >> 24) & 0xFF)
                || ui8Data == ((Packet::HEADER_FMT_B >> 24) & 0xFF)
                || ui8Data == ((Packet::HEADER_FMT_C >> 24) & 0xFF)
                || ui8Data == ((Packet::HEADER_FMT_XBIT >> 24) & 0xFF)
                || ui8Data == ((Packet::HEADER_FMT_XBIT2 >> 24) & 0xFF))
            {
                eState = SM_H1;
            }
        }
        else if (eState == SM_H1)
        {
            if (ui8Data == ((Packet::HEADER_FMT_A >> 16) & 0xFF))
            {
                eState = SM_H2;
            }
            else
            {
                eState = SM_IDLE;
            }
        }
        else if (eState == SM_H2)
        {
            if (ui8Data == ((Packet::HEADER_FMT_A >> 8) & 0xFF))
            {
                eState = SM_H3;
                xbit = false;
            }
            else if (ui8Data == ((Packet::HEADER_FMT_XBIT >> 8) & 0xFF))
            {
                eState = SM_H3;
                xbit = true;
            }
            else
            {
                eState = SM_IDLE;
            }
        }
        else if (eState == SM_H3)
        {
            if (!xbit)
            {
                if (ui8Data == (Packet::HEADER_FMT_A & 0xFF))
                {
                    eState = SM_IDLE;
                    return Packet::Type::TYPE_BINARY_FMT_A;
                }
                else if (ui8Data == (Packet::HEADER_FMT_B & 0xFF))
                {
                    eState = SM_IDLE;
                    return Packet::Type::TYPE_BINARY_FMT_B;
                }
                else if (ui8Data == (Packet::HEADER_FMT_C & 0xFF))
                {
                    eState = SM_IDLE;
                    return Packet::Type::TYPE_BINARY_FMT_C;
                }
                else
                {
                    eState = SM_IDLE;
                }
            }
            else
            {
                if (ui8Data == (Packet::HEADER_FMT_XBIT & 0xFF))
                {
                    eState = SM_IDLE;
                    return Packet::Type::TYPE_BINARY_TEST1;
                }
                else if (ui8Data == (Packet::HEADER_FMT_XBIT2 & 0xFF))
                {
                    eState = SM_IDLE;
                    return Packet::Type::TYPE_BINARY_TEST2;
                }
                else
                {
                    eState = SM_IDLE;
                }
            }
        }
        else
        {
            eState = SM_IDLE;
        }

        return Packet::Type::TYPE_UNKNOWN;
    }

    void dataReceived(uint8_t data[], size_t length, TimeStamp timestamp)
    {
        static unsigned int invalPkgCount = 0;

        // Assume that since the _runningDataIndex is unsigned, any overflows
        // will naturally go to zero, which is the behavior that we want.
        for (size_t i = 0; i < length; i++, _runningDataIndex++)
        {
            auto binaryPacketType = bFindImuHeader(data[i]);
            if (binaryPacketType != Packet::Type::TYPE_UNKNOWN)
            {
                packetType = binaryPacketType;

                resetTracking();
                currentlyBuildingBinaryPacket = true;
                timeFound = timestamp;
                uint32_t header;
                switch (binaryPacketType)
                {
                case Packet::Type::TYPE_BINARY_FMT_A:
                    header = stoh(Packet::HEADER_FMT_A);
                    break;
                case Packet::Type::TYPE_BINARY_FMT_B:
                    header = stoh(Packet::HEADER_FMT_B);
                    break;
                case Packet::Type::TYPE_BINARY_FMT_C:
                    header = stoh(Packet::HEADER_FMT_C);
                    break;
                case Packet::Type::TYPE_BINARY_TEST1:
                    header = stoh(Packet::HEADER_FMT_XBIT);
                    break;
                case Packet::Type::TYPE_BINARY_TEST2:
                    header = stoh(Packet::HEADER_FMT_XBIT2);
                    break;
                default:
                    break;
                }
                memcpy(_buffer, &header, 4);
                _bufferAppendLocation += 4;

                continue;
            }

            if (!currentlyBuildingAsciiPacket && !currentlyBuildingBinaryPacket)
            {
                resetTracking();
                currentlyBuildingAsciiPacket = true;
                timeFound = timestamp;
            }
            else if (currentlyBuildingBinaryPacket)
            {
                _buffer[_bufferAppendLocation++] = data[i];

                if ((packetType == Packet::Type::TYPE_BINARY_FMT_A && _bufferAppendLocation == 36)
                    || (packetType == Packet::Type::TYPE_BINARY_FMT_B && _bufferAppendLocation == 40)
                    || (packetType == Packet::Type::TYPE_BINARY_FMT_C && _bufferAppendLocation == 38)
                    || (packetType == Packet::Type::TYPE_BINARY_TEST1 && _bufferAppendLocation == 11)
                    || (packetType == Packet::Type::TYPE_BINARY_TEST2 && _bufferAppendLocation == 13))
                {
                    Packet p(reinterpret_cast<unsigned char*>(_buffer), _bufferAppendLocation);
                    if (p.isValid())
                    {
                        // We have a valid binary packet!!!.
                        dispatchPacket(p, _runningDataIndex, timeFound);
                    }
                    else
                    {
                        // Invalid packet!
                        std::cout << "Invalid Package found " << ++invalPkgCount << '\n';
                    }
                    resetTracking();
                }
                if (_bufferAppendLocation >= 40)
                    resetTracking();
            }
            else if (currentlyBuildingAsciiPacket)
            {
                _buffer[_bufferAppendLocation++] = data[i];

                if (data[i] == AsciiEscapeChar)
                    resetTracking();
                else if (data[i] == AsciiEndChar1)
                    asciiEndChar1Found = true;
                else if (asciiEndChar1Found)
                {
                    if (data[i] == AsciiEndChar2)
                    {
                        Packet p(reinterpret_cast<unsigned char*>(_buffer), _bufferAppendLocation);

                        if (p.isValid())
                        {
                            dispatchPacket(p, _runningDataIndex, timeFound);
                        }
                        else
                        {
                            // Invalid packet!
                            std::cout << "Invalid Package found " << ++invalPkgCount << '\n';
                        }
                    }

                    resetTracking();
                }
                else if (i + 1 > MaximumSizeForAsciiPacket)
                    resetTracking();
            }
        }
    }

    void dispatchPacket(Packet& packet, size_t runningDataIndexAtPacketStart, TimeStamp timestamp)
    {
        if (_possiblePacketFoundHandler != NULL)
        {
            _possiblePacketFoundHandler(_possiblePacketFoundUserData, packet, runningDataIndexAtPacketStart, timestamp);
        }
    }
}; // namespace kvh::protocol::uart

PacketFinder::PacketFinder()
    : _pi(new Impl(this))
{
}

PacketFinder::PacketFinder(size_t internalReceiveBufferSize)
    : _pi(new Impl(this, internalReceiveBufferSize))
{
}

PacketFinder::~PacketFinder()
{
    delete _pi;
}

void PacketFinder::processReceivedData(char data[], size_t length)
{
    TimeStamp placeholder;

    processReceivedData(data, length, placeholder);
}

void PacketFinder::processReceivedData(char data[], size_t length, TimeStamp timestamp)
{
    _pi->dataReceived(reinterpret_cast<uint8_t*>(data), length, timestamp);
}

void PacketFinder::registerPossiblePacketFoundHandler(void* userData, ValidPacketFoundHandler handler)
{
    if (_pi->_possiblePacketFoundHandler != NULL)
        throw std::logic_error("Unregister packet handler first");

    _pi->_possiblePacketFoundHandler = handler;
    _pi->_possiblePacketFoundUserData = userData;
}

void PacketFinder::unregisterPossiblePacketFoundHandler()
{
    if (_pi->_possiblePacketFoundHandler == NULL)
        throw std::logic_error("Register packet handler first");

    _pi->_possiblePacketFoundHandler = NULL;
    _pi->_possiblePacketFoundUserData = NULL;
}

} // namespace kvh::protocol::uart