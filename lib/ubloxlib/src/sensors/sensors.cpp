#include "ub/sensors/sensors.hpp"
#include "ub/xplat/serialport.hpp"
#include "ub/xplat/criticalsection.hpp"
#include "ub/xplat/ubtime.hpp"
#include "ub/xplat/event.hpp"
#include "ub/util/checksum.hpp"

#include <string>
#include <queue>
#include <string>
#include <cstdio>
#include <stdexcept>

using namespace std;
using namespace ub::xplat;
using namespace ub::protocol::uart;
using namespace ub::util;

#define COMMAND_MAX_LENGTH 0x100

namespace ub::sensors
{
struct UbSensor::Impl
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
    UbSensor* BackReference;
    queue<Packet> _receivedResponses;
    CriticalSection _transactionCS;
    bool _waitingForResponse;
    ErrorPacketReceivedHandler _errorPacketReceivedHandler;
    void* _errorPacketReceivedUserData;
    uint16_t _responseTimeoutMs;
    uint16_t _retransmitDelayMs;
    xplat::Event _newResponsesEvent;

    explicit Impl(UbSensor* backReference)
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

    size_t finalizeCommandToSend(char* toSend, size_t length)
    {
        length += sprintf(toSend + length, "%02X\r\n", ubloxChecksum::checksumNMEA(toSend, length - 2));

        return length;
    }

    Packet transactionWithWait(char* toSend, size_t length, uint16_t responseTimeoutMs, uint16_t retransmitDelayMs)
    {
        // Make sure we don't have any existing responses.
        _transactionCS.enter();

        while (!_receivedResponses.empty())
            _receivedResponses.pop();

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

vector<uint32_t> UbSensor::supportedBaudrates()
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

UbSensor::UbSensor()
    : _pi(new Impl(this))
{
}

UbSensor::~UbSensor()
{
    if (_pi != NULL)
    {
        if (_pi->SimplePortIsOurs && _pi->DidWeOpenSimplePort && isConnected())
            disconnect();

        delete _pi;
        _pi = NULL;
    }
}

uint32_t UbSensor::baudrate()
{
    if (_pi->pSerialPort == NULL)
        throw std::logic_error("We are not connected to a known serial port.");

    return _pi->pSerialPort->baudrate();
}

std::string UbSensor::port()
{
    if (_pi->pSerialPort == NULL)
        throw std::logic_error("We are not connected to a known serial port.");

    return _pi->pSerialPort->port();
}

bool UbSensor::isConnected()
{
    return _pi->isConnected();
}

uint16_t UbSensor::responseTimeoutMs()
{
    return _pi->_responseTimeoutMs;
}

void UbSensor::setResponseTimeoutMs(uint16_t timeout)
{
    _pi->_responseTimeoutMs = timeout;
}

uint16_t UbSensor::retransmitDelayMs()
{
    return _pi->_retransmitDelayMs;
}

void UbSensor::setRetransmitDelayMs(uint16_t delay)
{
    _pi->_retransmitDelayMs = delay;
}

bool UbSensor::verifySensorConnectivity()
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

void UbSensor::connect(const string& portName, uint32_t baudrate)
{
    _pi->pSerialPort = new SerialPort(portName, baudrate);

    connect(dynamic_cast<IPort*>(_pi->pSerialPort));
}

void UbSensor::connect(IPort* simplePort)
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

void UbSensor::disconnect()
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

string UbSensor::transaction(string toSend)
{
    char buffer[COMMAND_MAX_LENGTH];
    size_t finalLength = toSend.length();
    Packet response;

    // Copy over what was provided.
    copy(toSend.begin(), toSend.end(), buffer);

    // First see if an '*' is present.
    if (toSend.find('*') == string::npos)
    {
        buffer[toSend.length()] = '*';
        finalLength = _pi->finalizeCommandToSend(buffer, toSend.length() + 1);
    }
    else if (toSend[toSend.length() - 2] != '\r' && toSend[toSend.length() - 1] != '\n')
    {
        buffer[toSend.length()] = '\r';
        buffer[toSend.length() + 1] = '\n';
        finalLength += 2;
    }

    _pi->transactionNoFinalize(buffer, finalLength, true, &response);

    return response.datastr();
}

string UbSensor::send(string toSend, bool waitForReply)
{
    Packet p;
    char* buffer = new char[toSend.size() + 8]; // Extra room for possible additions.
    size_t curToSendLength = toSend.size();

    // See if a '$' needs to be prepended.
    if (toSend[0] == '$')
    {
        toSend.copy(buffer, toSend.size());
    }
    else
    {
        buffer[0] = '$';
        toSend.copy(buffer + 1, toSend.size());
        curToSendLength++;
    }

    // Locate any '*' in the command.
    size_t astrickLocation = string::npos;
    for (size_t i = 0; i < curToSendLength; i++)
    {
        if (buffer[i] == '*')
        {
            astrickLocation = i;
            break;
        }
    }

    // Do we need to add a '*'?
    if (astrickLocation == string::npos)
    {
        buffer[curToSendLength] = '*';
        astrickLocation = curToSendLength++;
    }

    // Do we need to add a checksum/CRC?
    if (astrickLocation == curToSendLength - 1)
    {
        curToSendLength += sprintf(buffer + curToSendLength, "%02X\r\n", ubloxChecksum::checksumNMEA(buffer + 1, curToSendLength));
    }
    // Do we need to add "\r\n"?
    else if (buffer[curToSendLength - 1] != '\n')
    {
        buffer[curToSendLength++] = '\r';
        buffer[curToSendLength++] = '\n';
    }

    _pi->transactionNoFinalize(buffer, curToSendLength, waitForReply, &p, _pi->_responseTimeoutMs, _pi->_retransmitDelayMs);

    delete[] buffer;

    return p.datastr();
}

void UbSensor::registerRawDataReceivedHandler(void* userData, RawDataReceivedHandler handler)
{
    if (_pi->_rawDataReceivedHandler != NULL)
        throw std::logic_error("Already registered handler");

    _pi->_rawDataReceivedHandler = handler;
    _pi->_rawDataReceivedUserData = userData;
}

void UbSensor::unregisterRawDataReceivedHandler()
{
    if (_pi->_rawDataReceivedHandler == NULL)
        throw std::logic_error("Register handler first");

    _pi->_rawDataReceivedHandler = NULL;
    _pi->_rawDataReceivedUserData = NULL;
}

void UbSensor::registerPossiblePacketFoundHandler(void* userData, PossiblePacketFoundHandler handler)
{
    if (_pi->_possiblePacketFoundHandler != NULL)
        throw std::logic_error("Already registered handler");

    _pi->_possiblePacketFoundHandler = handler;
    _pi->_possiblePacketFoundUserData = userData;
}

void UbSensor::unregisterPossiblePacketFoundHandler()
{
    if (_pi->_possiblePacketFoundHandler == NULL)
        throw std::logic_error("Register handler first");

    _pi->_possiblePacketFoundHandler = NULL;
    _pi->_possiblePacketFoundUserData = NULL;
}

void UbSensor::registerAsyncPacketReceivedHandler(void* userData, AsyncPacketReceivedHandler handler)
{
    if (_pi->_asyncPacketReceivedHandler != NULL)
        throw std::logic_error("Already registered handler");

    _pi->_asyncPacketReceivedHandler = handler;
    _pi->_asyncPacketReceivedUserData = userData;
}

void UbSensor::unregisterAsyncPacketReceivedHandler()
{
    if (_pi->_asyncPacketReceivedHandler == NULL)
        throw std::logic_error("Register handler first");

    _pi->_asyncPacketReceivedHandler = NULL;
    _pi->_asyncPacketReceivedUserData = NULL;
}

void UbSensor::registerErrorPacketReceivedHandler(void* userData, ErrorPacketReceivedHandler handler)
{
    if (_pi->_errorPacketReceivedHandler != NULL)
        throw std::logic_error("Already registered handler");

    _pi->_errorPacketReceivedHandler = handler;
    _pi->_errorPacketReceivedUserData = userData;
}

void UbSensor::unregisterErrorPacketReceivedHandler()
{
    if (_pi->_errorPacketReceivedHandler == NULL)
        throw std::logic_error("Register handler first");

    _pi->_errorPacketReceivedHandler = NULL;
    _pi->_errorPacketReceivedUserData = NULL;
}

void UbSensor::changeBaudRate(uint32_t baudrate)
{
    // TODO: Change baudrate here
    //writeSerialBaudRate(baudrate, true);
    throw std::logic_error("Not implemented");

    _pi->pSerialPort->changeBaudrate(baudrate);
}

} // namespace ub::sensors
