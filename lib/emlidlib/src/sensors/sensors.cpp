#include "er/sensors/sensors.hpp"
#include "er/xplat/serialport.hpp"
#include "er/xplat/criticalsection.hpp"
#include "er/xplat/ertime.hpp"
#include "er/xplat/event.hpp"
#include "er/util/checksum.hpp"

#include <string>
#include <queue>
#include <string>
#include <cstdio>
#include <stdexcept>

using namespace std;
using namespace er::xplat;
using namespace er::protocol::uart;
using namespace er::util;

#define COMMAND_MAX_LENGTH 0x100

namespace er::sensors
{
struct ErSensor::Impl
{
    static const size_t DefaultReadBufferSize = 1024;
    static const uint16_t DefaultResponseTimeoutMs = 500;
    static const uint16_t DefaultRetransmitDelayMs = 200;

    SerialPort* pSerialPort;
    IPort* port;
    bool SimplePortIsOurs;
    bool DidWeOpenSimplePort;
    RawDataReceivedHandler _rawDataReceivedHandler;
    void* _rawDataReceivedUserData;
    PossiblePacketFoundHandler _possiblePacketFoundHandler;
    void* _possiblePacketFoundUserData;
    PacketFinder _packetFinder;
    size_t _dataRunningIndex;
    AsyncPacketReceivedHandler _asyncPacketReceivedHandler;
    void* _asyncPacketReceivedUserData;
    ErSensor* BackReference;
    queue<Packet> _receivedResponses;
    CriticalSection _transactionCS;
    bool _waitingForResponse;
    ErrorPacketReceivedHandler _errorPacketReceivedHandler;
    void* _errorPacketReceivedUserData;
    uint16_t _responseTimeoutMs;
    uint16_t _retransmitDelayMs;
    xplat::Event _newResponsesEvent;

    explicit Impl(ErSensor* backReference)
        : pSerialPort(NULL),
          port(NULL),
          SimplePortIsOurs(false),
          DidWeOpenSimplePort(false),
          _rawDataReceivedHandler(NULL),
          _rawDataReceivedUserData(NULL),
          _possiblePacketFoundHandler(NULL),
          _possiblePacketFoundUserData(NULL),
          _dataRunningIndex(0),
          _asyncPacketReceivedHandler(NULL),
          _asyncPacketReceivedUserData(NULL),
          BackReference(backReference),
          _waitingForResponse(false),
          _errorPacketReceivedHandler(NULL),
          _errorPacketReceivedUserData(NULL),
          _responseTimeoutMs(DefaultResponseTimeoutMs),
          _retransmitDelayMs(DefaultRetransmitDelayMs)
    {
        _packetFinder.registerPossiblePacketFoundHandler(this, possiblePacketFoundHandler);
    }

    ~Impl()
    {
        _packetFinder.unregisterPossiblePacketFoundHandler();
    }

    void onPossiblePacketFound(Packet& possiblePacket, size_t packetStartRunningIndex)
    {
        if (_possiblePacketFoundHandler != NULL)
            _possiblePacketFoundHandler(_possiblePacketFoundUserData, possiblePacket, packetStartRunningIndex);
    }

    void onAsyncPacketReceived(Packet& asciiPacket, size_t runningIndex, TimeStamp /*timestamp*/)
    {
        if (_asyncPacketReceivedHandler != NULL)
            _asyncPacketReceivedHandler(_asyncPacketReceivedUserData, asciiPacket, runningIndex);
    }

    void onErrorPacketReceived(Packet& errorPacket, size_t runningIndex)
    {
        if (_errorPacketReceivedHandler != NULL)
            _errorPacketReceivedHandler(_errorPacketReceivedUserData, errorPacket, runningIndex);
    }

    static void possiblePacketFoundHandler(void* userData, Packet& possiblePacket, size_t packetStartRunningIndex, TimeStamp timestamp)
    {
        Impl* pThis = static_cast<Impl*>(userData);

        pThis->onPossiblePacketFound(possiblePacket, packetStartRunningIndex);

        if (!possiblePacket.isValid())
            return;

        if (possiblePacket.isError())
        {
            if (pThis->_waitingForResponse)
            {
                pThis->_transactionCS.enter();
                pThis->_receivedResponses.push(possiblePacket);
                pThis->_newResponsesEvent.signal();
                pThis->_transactionCS.leave();
            }

            pThis->onErrorPacketReceived(possiblePacket, packetStartRunningIndex);

            return;
        }

        if (possiblePacket.isResponse() && pThis->_waitingForResponse)
        {
            pThis->_transactionCS.enter();
            pThis->_receivedResponses.push(possiblePacket);
            pThis->_newResponsesEvent.signal();
            pThis->_transactionCS.leave();

            return;
        }

        // This wasn't anything else. We assume it is an async packet.
        pThis->onAsyncPacketReceived(possiblePacket, packetStartRunningIndex, timestamp);
    }

    static void dataReceivedHandler(void* userData)
    {
        static unsigned char readBuffer[DefaultReadBufferSize];
        Impl* pi = static_cast<Impl*>(userData);

        size_t numOfBytesRead = 0;

        pi->port->read(
            readBuffer,
            DefaultReadBufferSize,
            numOfBytesRead);

        if (numOfBytesRead == 0)
            return;

        TimeStamp t = TimeStamp::get();

        if (pi->_rawDataReceivedHandler != NULL)
            pi->_rawDataReceivedHandler(pi->_rawDataReceivedUserData, reinterpret_cast<char*>(readBuffer), numOfBytesRead, pi->_dataRunningIndex);

        pi->_packetFinder.processReceivedData(reinterpret_cast<char*>(readBuffer), numOfBytesRead, t);

        pi->_dataRunningIndex += numOfBytesRead;
    }

    bool isConnected()
    {
        return port != NULL && port->isOpen();
    }
};

vector<uint32_t> ErSensor::supportedBaudrates()
{
    uint32_t br[] = {
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

    return vector<uint32_t>(br, br + sizeof(br) / sizeof(uint32_t));
}

ErSensor::ErSensor()
    : _pi(new Impl(this))
{
}

ErSensor::~ErSensor()
{
    if (_pi != NULL)
    {
        if (_pi->SimplePortIsOurs && _pi->DidWeOpenSimplePort && isConnected())
            disconnect();

        delete _pi;
        _pi = NULL;
    }
}

uint32_t ErSensor::baudrate()
{
    if (_pi->pSerialPort == NULL)
        throw std::logic_error("We are not connected to a known serial port.");

    return _pi->pSerialPort->baudrate();
}

std::string ErSensor::port()
{
    if (_pi->pSerialPort == NULL)
        throw std::logic_error("We are not connected to a known serial port.");

    return _pi->pSerialPort->port();
}

bool ErSensor::isConnected()
{
    return _pi->isConnected();
}

uint16_t ErSensor::responseTimeoutMs()
{
    return _pi->_responseTimeoutMs;
}

void ErSensor::setResponseTimeoutMs(uint16_t timeout)
{
    _pi->_responseTimeoutMs = timeout;
}

uint16_t ErSensor::retransmitDelayMs()
{
    return _pi->_retransmitDelayMs;
}

void ErSensor::setRetransmitDelayMs(uint16_t delay)
{
    _pi->_retransmitDelayMs = delay;
}

bool ErSensor::verifySensorConnectivity()
{
    try
    {
        // TODO: Check here if connected
        // readModelNumber();
        throw std::logic_error("Not implemented");

        return true;
    }
    catch (...)
    {
    }

    return false;
}

void ErSensor::connect(const string& portName, uint32_t baudrate)
{
    _pi->pSerialPort = new SerialPort(portName, baudrate);

    connect(dynamic_cast<IPort*>(_pi->pSerialPort));
}

void ErSensor::connect(IPort* simplePort)
{
    _pi->port = simplePort;
    _pi->SimplePortIsOurs = false;

    _pi->port->registerDataReceivedHandler(_pi, Impl::dataReceivedHandler);

    if (!_pi->port->isOpen())
    {
        _pi->port->open();
        _pi->DidWeOpenSimplePort = true;
    }
}

void ErSensor::disconnect()
{
    if (_pi->port == NULL || !_pi->port->isOpen())
        throw std::logic_error("We are not connected.");

    _pi->port->unregisterDataReceivedHandler();

    if (_pi->DidWeOpenSimplePort)
    {
        _pi->port->close();
    }

    _pi->DidWeOpenSimplePort = false;

    if (_pi->SimplePortIsOurs)
    {
        delete _pi->port;

        _pi->port = NULL;
    }

    if (_pi->pSerialPort != NULL)
    {
        // Assuming we created this serial port.
        delete _pi->pSerialPort;
        _pi->pSerialPort = NULL;
    }
}

void ErSensor::registerRawDataReceivedHandler(void* userData, RawDataReceivedHandler handler)
{
    if (_pi->_rawDataReceivedHandler != NULL)
        throw std::logic_error("Already registered handler");

    _pi->_rawDataReceivedHandler = handler;
    _pi->_rawDataReceivedUserData = userData;
}

void ErSensor::unregisterRawDataReceivedHandler()
{
    if (_pi->_rawDataReceivedHandler == NULL)
        throw std::logic_error("Register handler first");

    _pi->_rawDataReceivedHandler = NULL;
    _pi->_rawDataReceivedUserData = NULL;
}

void ErSensor::registerPossiblePacketFoundHandler(void* userData, PossiblePacketFoundHandler handler)
{
    if (_pi->_possiblePacketFoundHandler != NULL)
        throw std::logic_error("Already registered handler");

    _pi->_possiblePacketFoundHandler = handler;
    _pi->_possiblePacketFoundUserData = userData;
}

void ErSensor::unregisterPossiblePacketFoundHandler()
{
    if (_pi->_possiblePacketFoundHandler == NULL)
        throw std::logic_error("Register handler first");

    _pi->_possiblePacketFoundHandler = NULL;
    _pi->_possiblePacketFoundUserData = NULL;
}

void ErSensor::registerAsyncPacketReceivedHandler(void* userData, AsyncPacketReceivedHandler handler)
{
    if (_pi->_asyncPacketReceivedHandler != NULL)
        throw std::logic_error("Already registered handler");

    _pi->_asyncPacketReceivedHandler = handler;
    _pi->_asyncPacketReceivedUserData = userData;
}

void ErSensor::unregisterAsyncPacketReceivedHandler()
{
    if (_pi->_asyncPacketReceivedHandler == NULL)
        throw std::logic_error("Register handler first");

    _pi->_asyncPacketReceivedHandler = NULL;
    _pi->_asyncPacketReceivedUserData = NULL;
}

void ErSensor::registerErrorPacketReceivedHandler(void* userData, ErrorPacketReceivedHandler handler)
{
    if (_pi->_errorPacketReceivedHandler != NULL)
        throw std::logic_error("Already registered handler");

    _pi->_errorPacketReceivedHandler = handler;
    _pi->_errorPacketReceivedUserData = userData;
}

void ErSensor::unregisterErrorPacketReceivedHandler()
{
    if (_pi->_errorPacketReceivedHandler == NULL)
        throw std::logic_error("Register handler first");

    _pi->_errorPacketReceivedHandler = NULL;
    _pi->_errorPacketReceivedUserData = NULL;
}

void ErSensor::changeBaudRate(uint32_t baudrate)
{
    // TODO: Change baudrate here
    //writeSerialBaudRate(baudrate, true);
    throw std::logic_error("Not implemented");

    _pi->pSerialPort->changeBaudrate(baudrate);
}

} // namespace er::sensors
