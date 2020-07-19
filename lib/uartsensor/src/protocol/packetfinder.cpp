#include "uart/protocol/packetfinder.hpp"
#include "uart/util/utilities.hpp"
#include "uart/util/checksum.hpp"

#include <arpa/inet.h>

#include <queue>
#include <list>
#include <cstring>
#include <stdexcept>
#include <iostream>

using namespace std;
using namespace uart::xplat;
using namespace uart::util;

namespace uart::protocol
{
struct PacketFinder::Impl
{
    static const size_t DefaultReceiveBufferSize = 2048;
    static const uint8_t AsciiStartChar = '$';
    static const uint8_t BinaryStartChar1 = 0xB5;
    static const uint8_t BinaryStartChar2 = 0x62;
    static const uint8_t AsciiEndChar1 = '\r';
    static const uint8_t AsciiEndChar2 = '\n';
    static const uint8_t AsciiEscapeChar = '\0';
    static const size_t MaximumSizeExpectedForBinaryPacket = 1024;
    static const size_t MaximumSizeForAsciiPacket = 256;

    xplat::TimeStamp timeFound;
    size_t possibleStartIndex;
    bool startFoundInProvidedDataBuffer;
    bool currentlyBuildingAsciiPacket;
    bool currentlyBuildingBinaryPacket;
    size_t numOfBytesRemainingForCompletePacket;

    bool asciiEndChar1Found;
    bool binaryStartChar1Found;
    bool binaryMsgClassFound;
    bool binaryMsgIdFound;
    bool binaryPayloadLengthFound;

    unsigned char binaryMsgClass;
    unsigned char binaryMsgId;
    unsigned short binaryPayloadLength;

    PacketFinder* _backReference;
    unsigned char* _buffer;
    const size_t _bufferSize;
    size_t _bufferAppendLocation;
    size_t _runningDataIndex; // Used for correlating raw data with where the packet was found for the end user.
    void* _possiblePacketFoundUserData;
    ValidPacketFoundHandler _possiblePacketFoundHandler;

    explicit Impl(PacketFinder* backReference)
        : _backReference(backReference),
          _buffer(new uint8_t[DefaultReceiveBufferSize]),
          _bufferSize(DefaultReceiveBufferSize),
          _bufferAppendLocation(0),
          _runningDataIndex(0),
          _possiblePacketFoundUserData(NULL),
          _possiblePacketFoundHandler(NULL),
          possibleStartIndex(0),
          startFoundInProvidedDataBuffer(false),
          currentlyBuildingAsciiPacket(false),
          currentlyBuildingBinaryPacket(false),
          asciiEndChar1Found(false),
          binaryStartChar1Found(false),
          binaryMsgIdFound(false),
          binaryPayloadLengthFound(false)

    {
    }

    Impl(PacketFinder* backReference, size_t internalReceiveBufferSize)
        : _backReference(backReference),
          _buffer(new uint8_t[internalReceiveBufferSize]),
          _bufferSize(internalReceiveBufferSize),
          _bufferAppendLocation(0),
          _runningDataIndex(0),
          _possiblePacketFoundUserData(NULL),
          _possiblePacketFoundHandler(NULL),
          possibleStartIndex(0),
          startFoundInProvidedDataBuffer(false),
          currentlyBuildingAsciiPacket(false),
          currentlyBuildingBinaryPacket(false),
          asciiEndChar1Found(false),
          binaryStartChar1Found(false),
          binaryMsgIdFound(false),
          binaryPayloadLengthFound(false)
    {
    }

    ~Impl()
    {
        delete[] _buffer;
    }

    void resetTracking()
    {
        possibleStartIndex = 0;
        startFoundInProvidedDataBuffer = false;
        currentlyBuildingAsciiPacket = false;
        currentlyBuildingBinaryPacket = false;
        asciiEndChar1Found = false;
        binaryStartChar1Found = false;
        binaryMsgClassFound = false;
        binaryMsgIdFound = false;
        binaryPayloadLengthFound = false;
        _bufferAppendLocation = 0;
    }

    void dataReceived(uint8_t data[], size_t length, TimeStamp timestamp)
    {
        static unsigned int invalPkgCount = 0;

        // Assume that since the _runningDataIndex is unsigned, any overflows
        // will naturally go to zero, which is the behavior that we want.
        for (size_t i = 0; i < length; i++, _runningDataIndex++)
        {
            if (!currentlyBuildingAsciiPacket && !currentlyBuildingBinaryPacket)
            {
                // This byte must be the start char
                if (data[i] == AsciiStartChar)
                {
                    resetTracking();
                    currentlyBuildingAsciiPacket = true;
                    possibleStartIndex = i;
                    timeFound = timestamp;
                    startFoundInProvidedDataBuffer = true;
                }
                else if (data[i] == BinaryStartChar1)
                {
                    resetTracking();
                    binaryStartChar1Found = true;
                    currentlyBuildingBinaryPacket = true;
                    possibleStartIndex = i;
                    timeFound = timestamp;
                    startFoundInProvidedDataBuffer = true;
                }
            }
            else if (currentlyBuildingBinaryPacket)
            {
                if (binaryStartChar1Found)
                {
                    // This byte must be the second sync char
                    if (data[i] == BinaryStartChar2)
                        binaryStartChar1Found = false;
                    else
                        resetTracking();
                }
                else if (!binaryMsgClassFound)
                {
                    // This byte must be the message class
                    binaryMsgClassFound = true;
                    binaryMsgClass = data[i];
                }
                else if (!binaryMsgIdFound)
                {
                    // This byte must be the message id
                    binaryMsgIdFound = true;
                    binaryMsgId = data[i];
                }
                else if (!binaryPayloadLengthFound)
                {
                    // This byte must be the payload length
                    if (i + 1 <= length - 1)
                    {
                        binaryPayloadLengthFound = true;
                        binaryPayloadLength = 100;
                        // memcpy(&(binaryPayloadLength), (data + i), 2);
                        numOfBytesRemainingForCompletePacket = binaryPayloadLength + 3;
                        if (binaryPayloadLength > 8192)
                        {
                            std::cout << "Invalid Package found (" << ++invalPkgCount << "): " << std::hex << (int)binaryMsgClass << ", " << (int)binaryMsgId << ", " << std::dec << binaryPayloadLength << std::endl;
                            resetTracking();
                        }
                    }
                    else
                        resetTracking(); // Bad splitting of length // TODO: Handle the case that the 2-byte length is split
                }
                else
                {
                    // We are currently collecting data for our packet.
                    numOfBytesRemainingForCompletePacket--;

                    if (numOfBytesRemainingForCompletePacket == 0)
                    {
                        // We have a possible binary packet!
                        uint8_t* packetStart = nullptr;
                        size_t packetLength = 0;

                        if (startFoundInProvidedDataBuffer)
                        {
                            // The binary packet exists completely in the user's provided buffer.
                            packetStart = data + possibleStartIndex;
                            packetLength = binaryPayloadLength + 6 + 2;
                        }
                        else
                        {
                            // The packet is split between our receive buffer and the user's buffer.
                            if (_bufferAppendLocation + i < _bufferSize)
                            {
                                std::memcpy(_buffer + _bufferAppendLocation, data, i + 1);

                                packetStart = _buffer + possibleStartIndex;
                                packetLength = binaryPayloadLength + 6 + 2;
                            }
                            else
                            {
                                // About to overrun our receive buffer!
                                resetTracking();
                                continue;
                            }
                        }

                        Packet p(reinterpret_cast<unsigned char*>(packetStart), packetLength);

                        if (p.isValid())
                        {
                            // We have a valid binary packet!!!.
                            dispatchPacket(p, _runningDataIndex, timeFound);
                        }
                        else
                        {
                            // Invalid packet!
                            std::cout << "Invalid Package found (" << ++invalPkgCount << "): " << std::hex << (int)binaryMsgClass << ", " << (int)binaryMsgId << ", " << std::dec << binaryPayloadLength << std::endl;
                        }

                        resetTracking();
                    }
                }
            }
            else if (currentlyBuildingAsciiPacket)
            {
                if (data[i] == AsciiEscapeChar)
                    resetTracking();
                else if (data[i] == AsciiEndChar1)
                    asciiEndChar1Found = true;
                else if (asciiEndChar1Found)
                {
                    if (data[i] == AsciiEndChar2)
                    {
                        // We have a possible data packet
                        uint8_t* startOfAsciiPacket = nullptr;
                        size_t packetLength = 0;

                        if (startFoundInProvidedDataBuffer)
                        {
                            // All the packet was in this data buffer so we don't need to do any copying.
                            startOfAsciiPacket = data + possibleStartIndex;
                            packetLength = i - possibleStartIndex + 1;
                        }
                        else
                        {
                            // The packet was split between the running data buffer and
                            // the current data buffer. We need to copy the data
                            // over before further processing.

                            if (_bufferAppendLocation + i < _bufferSize)
                            {
                                memcpy(_buffer + _bufferAppendLocation, data, i + 1);

                                startOfAsciiPacket = _buffer + possibleStartIndex;
                                packetLength = _bufferAppendLocation + i + 1 - possibleStartIndex;
                            }
                            else
                            {
                                // We are about to overflow our buffer.
                                resetTracking();
                                continue;
                            }
                        }

                        Packet p(reinterpret_cast<unsigned char*>(startOfAsciiPacket), packetLength);

                        if (p.isValid())
                            dispatchPacket(p, _runningDataIndex, timeFound);
                    }

                    resetTracking();
                }
                else if (i + 1 > MaximumSizeForAsciiPacket)
                    resetTracking();
            }
        }

        if (!currentlyBuildingAsciiPacket && !currentlyBuildingBinaryPacket)
            // No data to copy over.
            return;

        // Perform any data copying to our receive buffer.

        size_t dataIndexToStartCopyingFrom = 0;

        if (startFoundInProvidedDataBuffer)
        {
            dataIndexToStartCopyingFrom = possibleStartIndex;
            startFoundInProvidedDataBuffer = false;

            possibleStartIndex = _bufferAppendLocation;
        }

        if (_bufferAppendLocation + length - dataIndexToStartCopyingFrom < _bufferSize)
        {
            // Safe to copy over the data.
            size_t numOfBytesToCopyOver = length - dataIndexToStartCopyingFrom;
            uint8_t* copyFromStart = data + dataIndexToStartCopyingFrom;

            std::memcpy(_buffer + _bufferAppendLocation, copyFromStart, numOfBytesToCopyOver);
            _bufferAppendLocation += numOfBytesToCopyOver;
        }
        else
        {
            // We are about to overflow our buffer.
            resetTracking();
        }
    }

    void
        dispatchPacket(Packet& packet, size_t runningDataIndexAtPacketStart, TimeStamp timestamp)
    {
        if (_possiblePacketFoundHandler != NULL)
        {
            _possiblePacketFoundHandler(_possiblePacketFoundUserData, packet, runningDataIndexAtPacketStart, timestamp);
        }
    }
}; // namespace uart::protocol::uart

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

} // namespace uart::protocol