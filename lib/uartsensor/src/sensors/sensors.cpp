#include "uart/sensors/sensors.hpp"
#include "uart/xplat/serialport.hpp"
#include "uart/xplat/criticalsection.hpp"
#include "uart/xplat/timestamp.hpp"
#include "uart/xplat/event.hpp"
#include "uart/util/checksum.hpp"

#include <string>
#include <queue>
#include <string>
#include <cstdio>
#include <stdexcept>

using namespace std;
using namespace uart::xplat;
using namespace uart::protocol;
using namespace uart::util;

#define COMMAND_MAX_LENGTH 0x100

namespace uart::sensors
{
struct UartSensor::Impl
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
    UartSensor* BackReference;
    queue<Packet> _receivedResponses;
    CriticalSection _transactionCS;
    bool _waitingForResponse;
    ErrorPacketReceivedHandler _errorPacketReceivedHandler;
    void* _errorPacketReceivedUserData;
    uint16_t _responseTimeoutMs;
    uint16_t _retransmitDelayMs;
    xplat::Event _newResponsesEvent;

    explicit Impl(UartSensor* backReference)
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

    size_t finalizeCommandToSend(char* /*toSend*/, size_t length)
    {
        // length += sprintf(toSend + length, "%02X\r\n", Checksum::checksumASCII(toSend, length - 2));

        return length;
    }

    Packet transactionWithWait(char* toSend, size_t length, uint16_t responseTimeoutMs, uint16_t retransmitDelayMs)
    {
        // Make sure we don't have any existing responses.
        _transactionCS.enter();

        while (!_receivedResponses.empty())
        {
            _receivedResponses.pop();
        }

        _waitingForResponse = true;
        _transactionCS.leave();

        // Send the command and continue sending if retransmits are enabled
        // until we receive the response or timeout.
        Stopwatch timeoutSw;

        port->write(toSend, length);
        float curElapsedTime = timeoutSw.elapsedMs();

        while (true)
        {
            bool shouldRetransmit = false;

            // Compute how long we should wait for a response before taking
            // more action.
            float responseWaitTime = responseTimeoutMs - curElapsedTime;
            if (responseWaitTime > retransmitDelayMs)
            {
                responseWaitTime = retransmitDelayMs;
                shouldRetransmit = true;
            }

            // See if we have time left.
            if (responseWaitTime < 0)
            {
                _waitingForResponse = false;
                throw std::runtime_error("Timeout");
            }

            // Wait for any new responses that come in or until it is time to
            // send a new retransmit.
            xplat::Event::WaitResult waitResult = _newResponsesEvent.waitUs(static_cast<uint32_t>(responseWaitTime * 1000));

            queue<Packet> responsesToProcess;

            if (waitResult == xplat::Event::WAIT_TIMEDOUT)
            {
                if (!shouldRetransmit)
                {
                    _waitingForResponse = false;
                    throw std::runtime_error("Timeout");
                }
            }

            if (waitResult == xplat::Event::WAIT_SIGNALED)
            {
                // Get the current collection of responses.
                _transactionCS.enter();

                while (!_receivedResponses.empty())
                {
                    responsesToProcess.push(_receivedResponses.front());
                    _receivedResponses.pop();
                }

                _transactionCS.leave();
            }

            // Process the collection of responses we have.
            while (!responsesToProcess.empty())
            {
                Packet p = responsesToProcess.front();
                responsesToProcess.pop();

                if (p.isError())
                {
                    _waitingForResponse = false;
                    throw std::runtime_error("Sensor Error");
                }

                // We must have a response packet.
                _waitingForResponse = false;
                return p;
            }

            // Retransmit.
            port->write(toSend, length);
            curElapsedTime = timeoutSw.elapsedMs();
        }
    }

    void transactionNoFinalize(char* toSend, size_t length, bool waitForReply, Packet* response, uint16_t responseTimeoutMs, uint16_t retransmitDelayMs)
    {
        if (!isConnected())
            throw std::logic_error("Connect first");

        if (waitForReply)
        {
            *response = transactionWithWait(toSend, length, responseTimeoutMs, retransmitDelayMs);

            if (response->isError())
                throw std::runtime_error("Sensor Error");
        }
        else
        {
            port->write(toSend, length);
        }
    }

    void transactionNoFinalize(char* toSend, size_t length, bool waitForReply, Packet* response)
    {
        transactionNoFinalize(toSend, length, waitForReply, response, _responseTimeoutMs, _retransmitDelayMs);
    }

    void transaction(char* toSend, size_t length, bool waitForReply, Packet* response, uint16_t responseTimeoutMs, uint16_t retransmitDelayMs)
    {
        length = finalizeCommandToSend(toSend, length);

        transactionNoFinalize(toSend, length, waitForReply, response, responseTimeoutMs, retransmitDelayMs);
    }

    // Performs a communication transaction with the sensor. If waitForReply is
    // set to true, we will retransmit the message until we receive a response
    // or until we time out, depending on current settings.
    void transaction(char* toSend, size_t length, bool waitForReply, Packet* response)
    {
        transaction(toSend, length, waitForReply, response, _responseTimeoutMs, _retransmitDelayMs);
    }
};

vector<uint32_t> UartSensor::supportedBaudrates()
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

UartSensor::UartSensor()
    : _pi(new Impl(this))
{
}

UartSensor::~UartSensor()
{
    if (_pi != NULL)
    {
        if (_pi->SimplePortIsOurs && _pi->DidWeOpenSimplePort && isConnected())
            disconnect();

        delete _pi;
        _pi = NULL;
    }
}

uint32_t UartSensor::baudrate()
{
    if (_pi->pSerialPort == NULL)
        throw std::logic_error("We are not connected to a known serial port.");

    return _pi->pSerialPort->baudrate();
}

std::string UartSensor::port()
{
    if (_pi->pSerialPort == NULL)
        throw std::logic_error("We are not connected to a known serial port.");

    return _pi->pSerialPort->port();
}

bool UartSensor::isConnected()
{
    return _pi->isConnected();
}

uint16_t UartSensor::responseTimeoutMs()
{
    return _pi->_responseTimeoutMs;
}

void UartSensor::setResponseTimeoutMs(uint16_t timeout)
{
    _pi->_responseTimeoutMs = timeout;
}

uint16_t UartSensor::retransmitDelayMs()
{
    return _pi->_retransmitDelayMs;
}

void UartSensor::setRetransmitDelayMs(uint16_t delay)
{
    _pi->_retransmitDelayMs = delay;
}

bool UartSensor::verifySensorConnectivity()
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

void UartSensor::connect(const string& portName, uint32_t baudrate)
{
    _pi->pSerialPort = new SerialPort(portName, baudrate);

    connect(dynamic_cast<IPort*>(_pi->pSerialPort));
}

void UartSensor::connect(IPort* simplePort)
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

void UartSensor::disconnect()
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

string UartSensor::transaction(string toSend)
{
    char buffer[COMMAND_MAX_LENGTH];
    size_t finalLength = toSend.length();
    Packet response;

    // Copy over what was provided.
    copy(toSend.begin(), toSend.end(), buffer);

    if (toSend[toSend.length() - 2] != '\r' && toSend[toSend.length() - 1] != '\n')
    {
        buffer[toSend.length()] = '\r';
        buffer[toSend.length() + 1] = '\n';
        finalLength += 2;
    }

    _pi->transactionNoFinalize(buffer, finalLength, true, &response);

    return response.datastr();
}

string UartSensor::send(string toSend, bool waitForReply)
{
    Packet p;
    char* buffer = new char[toSend.size() + 8]; // Extra room for possible additions.
    size_t curToSendLength = toSend.size();

    // Do we need to add "\r\n"?
    if (buffer[curToSendLength - 1] != '\n')
    {
        buffer[curToSendLength++] = '\r';
        buffer[curToSendLength++] = '\n';
    }

    _pi->transactionNoFinalize(buffer, curToSendLength, waitForReply, &p, _pi->_responseTimeoutMs, _pi->_retransmitDelayMs);

    delete[] buffer;

    return p.datastr();
}

void UartSensor::registerRawDataReceivedHandler(void* userData, RawDataReceivedHandler handler)
{
    if (_pi->_rawDataReceivedHandler != NULL)
        throw std::logic_error("Already registered handler");

    _pi->_rawDataReceivedHandler = handler;
    _pi->_rawDataReceivedUserData = userData;
}

void UartSensor::unregisterRawDataReceivedHandler()
{
    if (_pi->_rawDataReceivedHandler == NULL)
        throw std::logic_error("Register handler first");

    _pi->_rawDataReceivedHandler = NULL;
    _pi->_rawDataReceivedUserData = NULL;
}

void UartSensor::registerPossiblePacketFoundHandler(void* userData, PossiblePacketFoundHandler handler)
{
    if (_pi->_possiblePacketFoundHandler != NULL)
        throw std::logic_error("Already registered handler");

    _pi->_possiblePacketFoundHandler = handler;
    _pi->_possiblePacketFoundUserData = userData;
}

void UartSensor::unregisterPossiblePacketFoundHandler()
{
    if (_pi->_possiblePacketFoundHandler == NULL)
        throw std::logic_error("Register handler first");

    _pi->_possiblePacketFoundHandler = NULL;
    _pi->_possiblePacketFoundUserData = NULL;
}

void UartSensor::registerAsyncPacketReceivedHandler(void* userData, AsyncPacketReceivedHandler handler)
{
    if (_pi->_asyncPacketReceivedHandler != NULL)
        throw std::logic_error("Already registered handler");

    _pi->_asyncPacketReceivedHandler = handler;
    _pi->_asyncPacketReceivedUserData = userData;
}

void UartSensor::unregisterAsyncPacketReceivedHandler()
{
    if (_pi->_asyncPacketReceivedHandler == NULL)
        throw std::logic_error("Register handler first");

    _pi->_asyncPacketReceivedHandler = NULL;
    _pi->_asyncPacketReceivedUserData = NULL;
}

void UartSensor::registerErrorPacketReceivedHandler(void* userData, ErrorPacketReceivedHandler handler)
{
    if (_pi->_errorPacketReceivedHandler != NULL)
        throw std::logic_error("Already registered handler");

    _pi->_errorPacketReceivedHandler = handler;
    _pi->_errorPacketReceivedUserData = userData;
}

void UartSensor::unregisterErrorPacketReceivedHandler()
{
    if (_pi->_errorPacketReceivedHandler == NULL)
        throw std::logic_error("Register handler first");

    _pi->_errorPacketReceivedHandler = NULL;
    _pi->_errorPacketReceivedUserData = NULL;
}

void UartSensor::changeBaudRate(uint32_t baudrate)
{
    // TODO: Change baudrate here
    //writeSerialBaudRate(baudrate, true);
    throw std::logic_error("Not implemented");

    _pi->pSerialPort->changeBaudrate(baudrate);
}

} // namespace uart::sensors
