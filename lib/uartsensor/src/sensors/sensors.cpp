#include "uart/sensors/sensors.hpp"
#include "uart/xplat/serialport.hpp"
#include "uart/xplat/criticalsection.hpp"
#include "uart/xplat/timestamp.hpp"
#include "uart/xplat/event.hpp"

#include <string>
#include <queue>
#include <string>
#include <cstdio>
#include <stdexcept>
#include <array>
#include <vector>

namespace uart::sensors
{
static constexpr unsigned int COMMAND_MAX_LENGTH = 0x100;

struct UartSensor::Impl
{
    static constexpr uint16_t DefaultResponseTimeoutMs = 500;
    static constexpr uint16_t DefaultRetransmitDelayMs = 200;

    xplat::SerialPort* pSerialPort;
    xplat::IPort* port;
    bool SimplePortIsOurs;
    bool DidWeOpenSimplePort;
    RawDataReceivedHandler _rawDataReceivedHandler;
    void* _rawDataReceivedUserData;
    PossiblePacketFoundHandler _possiblePacketFoundHandler;
    void* _possiblePacketFoundUserData;
    size_t _dataRunningIndex;
    AsyncPacketReceivedHandler _asyncPacketReceivedHandler;
    void* _asyncPacketReceivedUserData;
    UartSensor* BackReference;
    std::queue<protocol::Packet> _receivedResponses;
    xplat::CriticalSection _transactionCS;
    bool _waitingForResponse;
    ErrorPacketReceivedHandler _errorPacketReceivedHandler;
    void* _errorPacketReceivedUserData;
    uint16_t _responseTimeoutMs;
    uint16_t _retransmitDelayMs;
    xplat::Event _newResponsesEvent;

    std::vector<uint8_t> readBuffer;

    explicit Impl(UartSensor* backReference)
        : pSerialPort(nullptr),
          port(nullptr),
          SimplePortIsOurs(false),
          DidWeOpenSimplePort(false),
          _rawDataReceivedHandler(nullptr),
          _rawDataReceivedUserData(nullptr),
          _possiblePacketFoundHandler(nullptr),
          _possiblePacketFoundUserData(nullptr),
          _dataRunningIndex(0),
          _asyncPacketReceivedHandler(nullptr),
          _asyncPacketReceivedUserData(nullptr),
          BackReference(backReference),
          _waitingForResponse(false),
          _errorPacketReceivedHandler(nullptr),
          _errorPacketReceivedUserData(nullptr),
          _responseTimeoutMs(DefaultResponseTimeoutMs),
          _retransmitDelayMs(DefaultRetransmitDelayMs),
          readBuffer(DefaultReadBufferSize) {}

    /// Destructor
    ~Impl() = default;

    Impl(const Impl&) = delete;            ///< Copy constructor
    Impl(Impl&&) = delete;                 ///< Move constructor
    Impl& operator=(const Impl&) = delete; ///< Copy assignment operator
    Impl& operator=(Impl&&) = delete;      ///< Move assignment operator

    void onPossiblePacketFound(protocol::Packet& possiblePacket, size_t packetStartRunningIndex) const
    {
        if (_possiblePacketFoundHandler != nullptr)
        {
            _possiblePacketFoundHandler(_possiblePacketFoundUserData, possiblePacket, packetStartRunningIndex);
        }
    }

    void onAsyncPacketReceived(protocol::Packet& asciiPacket, size_t runningIndex, const xplat::TimeStamp& /*timestamp*/) const
    {
        if (_asyncPacketReceivedHandler != nullptr)
        {
            _asyncPacketReceivedHandler(_asyncPacketReceivedUserData, asciiPacket, runningIndex);
        }
    }

    void onErrorPacketReceived(protocol::Packet& errorPacket, size_t runningIndex) const
    {
        if (_errorPacketReceivedHandler != nullptr)
        {
            _errorPacketReceivedHandler(_errorPacketReceivedUserData, errorPacket, runningIndex);
        }
    }

    static void possiblePacketFoundHandler(void* userData, protocol::Packet& possiblePacket, size_t packetStartRunningIndex, const xplat::TimeStamp& timestamp)
    {
        Impl* pThis = static_cast<Impl*>(userData);

        pThis->onPossiblePacketFound(possiblePacket, packetStartRunningIndex);

        if (!possiblePacket.isValid())
        {
            return;
        }

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
        Impl* pi = static_cast<Impl*>(userData);

        pi->readBuffer.resize(0);

        pi->port->read(pi->readBuffer);

        if (pi->readBuffer.empty())
        {
            return;
        }

        xplat::TimeStamp t = xplat::TimeStamp::get();

        if (pi->_rawDataReceivedHandler != nullptr)
        {
            pi->_rawDataReceivedHandler(pi->_rawDataReceivedUserData, pi->readBuffer, pi->_dataRunningIndex);
        }

        // pi->_packetFinder.processReceivedData(pi->readBuffer, t);
        pi->BackReference->_packetFinderFunction(pi->readBuffer, t, possiblePacketFoundHandler, userData, pi->BackReference->_packetFinderUserData);

        pi->_dataRunningIndex += pi->readBuffer.size();
    }

    [[nodiscard]] bool isConnected() const
    {
        return port != nullptr && port->isOpen();
    }

    static size_t finalizeCommandToSend(char* /*toSend*/, size_t length)
    {
        // length += sprintf(toSend + length, "%02X\r\n", Checksum::checksumASCII(toSend, length - 2));

        return length;
    }

    protocol::Packet transactionWithWait(char* toSend, size_t length, uint16_t responseTimeoutMs, uint16_t retransmitDelayMs)
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
        xplat::Stopwatch timeoutSw;

        port->write(toSend, length);
        float curElapsedTime = timeoutSw.elapsedMs();

        while (true)
        {
            bool shouldRetransmit = false;

            // Compute how long we should wait for a response before taking
            // more action.
            float responseWaitTime = static_cast<float>(responseTimeoutMs) - curElapsedTime;
            if (responseWaitTime > static_cast<float>(retransmitDelayMs))
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

            std::queue<protocol::Packet> responsesToProcess;

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
                protocol::Packet p = responsesToProcess.front();
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

    void transactionNoFinalize(char* toSend, size_t length, bool waitForReply, protocol::Packet* response, uint16_t responseTimeoutMs, uint16_t retransmitDelayMs)
    {
        if (!isConnected())
        {
            throw std::logic_error("Connect first");
        }

        if (waitForReply)
        {
            *response = transactionWithWait(toSend, length, responseTimeoutMs, retransmitDelayMs);

            if (response->isError())
            {
                throw std::runtime_error("Sensor Error");
            }
        }
        else
        {
            port->write(toSend, length);
        }
    }

    void transactionNoFinalize(char* toSend, size_t length, bool waitForReply, protocol::Packet* response)
    {
        transactionNoFinalize(toSend, length, waitForReply, response, _responseTimeoutMs, _retransmitDelayMs);
    }

    void transaction(char* toSend, size_t length, bool waitForReply, protocol::Packet* response, uint16_t responseTimeoutMs, uint16_t retransmitDelayMs)
    {
        length = finalizeCommandToSend(toSend, length);

        transactionNoFinalize(toSend, length, waitForReply, response, responseTimeoutMs, retransmitDelayMs);
    }

    // Performs a communication transaction with the sensor. If waitForReply is
    // set to true, we will retransmit the message until we receive a response
    // or until we time out, depending on current settings.
    void transaction(char* toSend, size_t length, bool waitForReply, protocol::Packet* response)
    {
        transaction(toSend, length, waitForReply, response, _responseTimeoutMs, _retransmitDelayMs);
    }
};

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
    : _pi(new Impl(this)),
      _endianness(endianness),
      _packetFinderFunction(packetFinderFunction),
      _packetFinderUserData(packetFinderUserData),
      _packetTypeFunction(packetTypeFunction),
      _isValidFunction(isValidFunction),
      _isErrorFunction(isErrorFunction),
      _isResponseFunction(isResponseFunction),
      _packetHeaderLength(packetHeaderLength) {}

UartSensor::~UartSensor()
{
    if (_pi != nullptr)
    {
        if (_pi->SimplePortIsOurs && _pi->DidWeOpenSimplePort && isConnected())
        {
            disconnect();
        }

        delete _pi;
        _pi = nullptr;
    }
}

uint32_t UartSensor::baudrate()
{
    if (_pi->pSerialPort == nullptr)
    {
        throw std::logic_error("We are not connected to a known serial port.");
    }

    return _pi->pSerialPort->baudrate();
}

std::string UartSensor::port()
{
    if (_pi->pSerialPort == nullptr)
    {
        throw std::logic_error("We are not connected to a known serial port.");
    }

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

void UartSensor::connect(const std::string& portName, uint32_t baudrate)
{
    _pi->pSerialPort = new xplat::SerialPort(portName, baudrate); // NOLINT

    connect(dynamic_cast<xplat::IPort*>(_pi->pSerialPort));
}

void UartSensor::connect(xplat::IPort* simplePort)
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
    if (_pi->port == nullptr || !_pi->port->isOpen())
    {
        return;
    }

    _pi->port->unregisterDataReceivedHandler();

    if (_pi->DidWeOpenSimplePort)
    {
        _pi->port->close();
    }

    _pi->DidWeOpenSimplePort = false;

    if (_pi->SimplePortIsOurs)
    {
        delete _pi->port;

        _pi->port = nullptr;
    }

    if (_pi->pSerialPort != nullptr)
    {
        // Assuming we created this serial port.
        delete _pi->pSerialPort;
        _pi->pSerialPort = nullptr;
    }
}

std::string UartSensor::transaction(const std::string& toSend)
{
    std::array<char, COMMAND_MAX_LENGTH> buffer{};
    size_t finalLength = toSend.length();
    protocol::Packet response(this);

    // Copy over what was provided.
    copy(toSend.begin(), toSend.end(), buffer.data());

    if (toSend[toSend.length() - 2] != '\r' && toSend[toSend.length() - 1] != '\n')
    {
        buffer.at(toSend.length()) = '\r';
        buffer.at(toSend.length() + 1) = '\n';
        finalLength += 2;
    }

    _pi->transactionNoFinalize(buffer.data(), finalLength, true, &response);

    return response.datastr();
}

std::string UartSensor::send(const std::string& toSend, bool waitForReply)
{
    protocol::Packet p(this);
    auto buffer = std::vector<char>(toSend.size() + 8); // Extra room for possible additions.
    size_t curToSendLength = toSend.size();

    // Do we need to add "\r\n"?
    if (buffer.at(curToSendLength - 1) != '\n')
    {
        buffer.at(curToSendLength++) = '\r';
        buffer.at(curToSendLength++) = '\n';
    }

    _pi->transactionNoFinalize(buffer.data(), curToSendLength, waitForReply, &p, _pi->_responseTimeoutMs, _pi->_retransmitDelayMs);

    return p.datastr();
}

void UartSensor::registerRawDataReceivedHandler(void* userData, RawDataReceivedHandler handler)
{
    if (_pi->_rawDataReceivedHandler != nullptr)
    {
        throw std::logic_error("Already registered handler");
    }

    _pi->_rawDataReceivedHandler = handler;
    _pi->_rawDataReceivedUserData = userData;
}

void UartSensor::unregisterRawDataReceivedHandler()
{
    if (_pi->_rawDataReceivedHandler == nullptr)
    {
        throw std::logic_error("Register handler first");
    }

    _pi->_rawDataReceivedHandler = nullptr;
    _pi->_rawDataReceivedUserData = nullptr;
}

void UartSensor::registerPossiblePacketFoundHandler(void* userData, PossiblePacketFoundHandler handler)
{
    if (_pi->_possiblePacketFoundHandler != nullptr)
    {
        throw std::logic_error("Already registered handler");
    }

    _pi->_possiblePacketFoundHandler = handler;
    _pi->_possiblePacketFoundUserData = userData;
}

void UartSensor::unregisterPossiblePacketFoundHandler()
{
    if (_pi->_possiblePacketFoundHandler == nullptr)
    {
        throw std::logic_error("Register handler first");
    }

    _pi->_possiblePacketFoundHandler = nullptr;
    _pi->_possiblePacketFoundUserData = nullptr;
}

void UartSensor::registerAsyncPacketReceivedHandler(void* userData, AsyncPacketReceivedHandler handler)
{
    if (_pi->_asyncPacketReceivedHandler != nullptr)
    {
        throw std::logic_error("Already registered handler");
    }

    _pi->_asyncPacketReceivedHandler = handler;
    _pi->_asyncPacketReceivedUserData = userData;
}

void UartSensor::unregisterAsyncPacketReceivedHandler()
{
    if (_pi->_asyncPacketReceivedHandler == nullptr)
    {
        throw std::logic_error("Register handler first");
    }

    _pi->_asyncPacketReceivedHandler = nullptr;
    _pi->_asyncPacketReceivedUserData = nullptr;
}

void UartSensor::registerErrorPacketReceivedHandler(void* userData, ErrorPacketReceivedHandler handler)
{
    if (_pi->_errorPacketReceivedHandler != nullptr)
    {
        throw std::logic_error("Already registered handler");
    }

    _pi->_errorPacketReceivedHandler = handler;
    _pi->_errorPacketReceivedUserData = userData;
}

void UartSensor::unregisterErrorPacketReceivedHandler()
{
    if (_pi->_errorPacketReceivedHandler == nullptr)
    {
        throw std::logic_error("Register handler first");
    }

    _pi->_errorPacketReceivedHandler = nullptr;
    _pi->_errorPacketReceivedUserData = nullptr;
}

void UartSensor::changeBaudRate(uint32_t baudrate)
{
    // TODO: Change baudrate here
    //writeSerialBaudRate(baudrate, true);
    throw std::logic_error("Not implemented");

    _pi->pSerialPort->changeBaudrate(baudrate);
}

} // namespace uart::sensors
