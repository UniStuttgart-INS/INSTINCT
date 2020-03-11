#include "ub/xplat/serialport.hpp"

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <cstring>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <sys/select.h>
#include <sstream>

#include <linux/serial.h>

#include <list>
#include <iostream>

#include "ub/xplat/thread.hpp"
#include "ub/xplat/criticalsection.hpp"
#include "ub/xplat/event.hpp"

using namespace std;

namespace ub::xplat
{
struct SerialPort::Impl
{
    // Constants //////////////////////////////////////////////////////////////

    static const size_t NumberOfBytesToPurgeOnOpeningSerialPort = 100;

    static const uint8_t WaitTimeForSerialPortReadsInMs = 100;

    // Members ////////////////////////////////////////////////////////////////

    int SerialPortHandle;

    Thread* pThreadForHandlingReceivedDataInternally;

    // The name of the serial port.
    string PortName;

    // The serial port's baudrate.
    uint32_t Baudrate;

    // Indicates if the serial port is open.
    bool IsOpen;

    // Critical section for registering, unregistering, and notifying observers
    // of events.
    CriticalSection ObserversCriticalSection;

    DataReceivedHandler _dataReceivedHandler;
    void* _dataReceivedUserData;

    Thread* pSerialPortEventsThread;

    bool ContinueHandlingSerialPortEvents;
    bool ChangingBaudrate;

    bool PurgeFirstDataBytesWhenSerialPortIsFirstOpened;

    bool ThreadIsRunning;

    SerialPort* BackReference;

    StopBits stopBits;

    Event WaitForBaudrateChange;
    Event NotificationsThreadStopped;

    explicit Impl(SerialPort* backReference)
        : SerialPortHandle(0),
          pThreadForHandlingReceivedDataInternally(NULL),
          Baudrate(0),
          IsOpen(false),
          _dataReceivedHandler(NULL),
          _dataReceivedUserData(NULL),
          pSerialPortEventsThread(NULL),
          ContinueHandlingSerialPortEvents(false),
          ChangingBaudrate(false),
          PurgeFirstDataBytesWhenSerialPortIsFirstOpened(true),
          ThreadIsRunning(false),
          BackReference(backReference),
          stopBits(ONE_STOP_BIT)
    {
    }

    ~Impl()
    {
    }

    static void HandleSerialPortNotifications(void* data)
    {
        static_cast<Impl*>(data)->HandleSerialPortNotifications();
    }

    void close(bool checkAndToggleIsOpenFlag = true)
    {
        if (checkAndToggleIsOpenFlag)
            ensureOpened();

        StopSerialPortNotificationsThread();

        if (::close(SerialPortHandle) == -1)
            throw std::runtime_error("Serial Port could not be closed.");

        if (checkAndToggleIsOpenFlag)
            IsOpen = false;
    }

    void closeAfterUsbCableUnplugged()
    {
        if (::close(SerialPortHandle) == -1)
            throw std::runtime_error("Serial Port could not be closed.");

        IsOpen = false;
    }

    void HandleSerialPortNotifications()
    {
        bool userUnpluggedUsbCable = false;

        fd_set readfs;
        int error;
        timeval readWaitTime;

    IgnoreError:

        try
        {
            ThreadIsRunning = true;

            while (ContinueHandlingSerialPortEvents)
            {
                FD_ZERO(&readfs);
                FD_SET(SerialPortHandle, &readfs);

                // Select sets the values in readWaitTime.
                readWaitTime.tv_sec = 0;
                readWaitTime.tv_usec = WaitTimeForSerialPortReadsInMs * 1000;

                error = select(
                    SerialPortHandle + 1,
                    &readfs,
                    NULL,
                    NULL,
                    &readWaitTime);

                if (error == -1)
                {
                    // Something unexpected happened.
                    break;
                }

                if (!FD_ISSET(SerialPortHandle, &readfs))
                    continue;

                OnDataReceived();
            }
        }
        catch (...)
        {
            // Don't want user-code exceptions stopping the thread.
            goto IgnoreError;
        }

        ThreadIsRunning = false;

        if (ContinueHandlingSerialPortEvents)
            // An error must have occurred.
            throw std::runtime_error("ContinueHandlingSerialPortEvents");

        if (userUnpluggedUsbCable)
            closeAfterUsbCableUnplugged();
    }

    void StartSerialPortNotificationsThread()
    {
        ContinueHandlingSerialPortEvents = true;

        pSerialPortEventsThread = Thread::startNew(
            HandleSerialPortNotifications,
            this);
    }

    void PurgeFirstDataBytesFromSerialPort()

    {
        unsigned char buffer[NumberOfBytesToPurgeOnOpeningSerialPort];
        size_t numOfBytesRead;

        BackReference->read(
            buffer,
            NumberOfBytesToPurgeOnOpeningSerialPort,
            numOfBytesRead);
    }

    void StopSerialPortNotificationsThread()
    {
        ContinueHandlingSerialPortEvents = false;

        pSerialPortEventsThread->join();

        delete pSerialPortEventsThread;
    }

    void OnDataReceived()
    {
        bool exception_happened = false;
        exception rethrow;

        ObserversCriticalSection.enter();

        // This is a critical section block
        // The exception must be handled here or the critical section will never
        // unlock
        try
        {
            // Moved this NULL check down here due to the nature of critical sections.
            // The handler could easily be changed to NULL while waiting for the lock
            if (_dataReceivedHandler != NULL)
            {
                _dataReceivedHandler(_dataReceivedUserData);
            }
        }
        catch (exception& e)
        {
            // We still want to throw an exception if it happens
            // Set a flag to indicate we need to throw an exception
            exception_happened = true;
            rethrow = e;
        }

        ObserversCriticalSection.leave();

        // Rethrow the exception
        if (exception_happened)
        {
            throw rethrow;
        }
    }

    void ensureOpened()
    {
        if (!IsOpen)
            throw std::logic_error("Port is not opened.");
    }

    void ensureClosed()
    {
        if (IsOpen)
            throw std::logic_error("Port is not closed.");
    }

    void open(bool checkAndToggleIsOpenFlag = true)
    {
        if (checkAndToggleIsOpenFlag)
            ensureClosed();

        int portFd = -1;

        portFd = ::open(
            PortName.c_str(),
            O_RDWR | O_NOCTTY);

        if (portFd == -1)
        {
            switch (errno)
            {
            case EACCES:
                throw std::runtime_error("Permission denied: " + PortName);
            case ENXIO:
            case ENOTDIR:
            case ENOENT:
                throw std::runtime_error("Not found: " + PortName);
            default:
                throw std::runtime_error("Unknown error: " + PortName);
            }
        }

        termios portSettings;

        memset(
            &portSettings,
            0,
            sizeof(termios));

        tcflag_t baudrateFlag;

        switch (Baudrate)
        {
        case 9600:
            baudrateFlag = B9600;
            break;
        case 19200:
            baudrateFlag = B19200;
            break;
        case 38400:
            baudrateFlag = B38400;
            break;
        case 57600:
            baudrateFlag = B57600;
            break;
        case 115200:
            baudrateFlag = B115200;
            break;
        case 230400:
            baudrateFlag = B230400;
            break;
        case 460800:
            baudrateFlag = B460800;
            break;
        case 921600:
            baudrateFlag = B921600;
            break;
        default:
            throw std::runtime_error("Unknown Baudrate");
        }

        // Set baudrate, 8n1, no modem control, and enable receiving characters.
        portSettings.c_cflag = baudrateFlag;
        portSettings.c_cflag |= CS8 | CLOCAL | CREAD;

        portSettings.c_iflag = IGNPAR; // Ignore bytes with parity errors.
        portSettings.c_oflag = 0;      // Enable raw data output.
        portSettings.c_cc[VTIME] = 0;  // Do not use inter-character timer.
        portSettings.c_cc[VMIN] = 0;   // Block on reads until 0 characters are received.

        // Clear the serial port buffers.
        if (tcflush(portFd, TCIFLUSH) != 0)
            throw std::runtime_error("Serial Port buffer could not be cleared!");

        if (tcsetattr(portFd, TCSANOW, &portSettings) != 0)
            throw std::runtime_error("Serial Port buffer attributes could not be set!");

        SerialPortHandle = portFd;

        if (checkAndToggleIsOpenFlag)
            IsOpen = true;

        if (PurgeFirstDataBytesWhenSerialPortIsFirstOpened)
            PurgeFirstDataBytesFromSerialPort();

        StartSerialPortNotificationsThread();
    }
};

SerialPort::SerialPort(
    const string& portName,
    uint32_t baudrate)
    : _pi(new Impl(this))
{
    _pi->PortName = portName;
    _pi->Baudrate = baudrate;
}

SerialPort::~SerialPort()
{
    if (_pi->IsOpen)
    {
        try
        {
            close();
        }
        catch (...)
        {
            // Something happened but don't want to throw out of the
            // destructor.
        }
    }

    delete _pi;
}

std::vector<std::string> SerialPort::getPortNames()
{
    std::vector<std::string> comPorts;

    // comPorts;

    int portFd = -1;
    const size_t MAX_PORTS = 255;
    std::string portName;
    std::stringstream stream;

    for (size_t index = 0; index < MAX_PORTS; ++index)
    {
        stream << "/dev/ttyUSB" << index;
        portName = stream.str();
        portFd = ::open(portName.c_str(),
                        O_RDWR | O_NOCTTY);

        if (portFd != -1)
        {
            comPorts.push_back(portName);
            ::close(portFd);
        }

        portName.clear();
        stream.str(std::string());
    }

    return comPorts;
}

void SerialPort::open()
{
    _pi->open();
}

void SerialPort::close()
{
    _pi->close();
}

bool SerialPort::isOpen()
{
    return _pi->IsOpen;
}

SerialPort::StopBits SerialPort::stopBits()
{
    return _pi->stopBits;
}

void SerialPort::setStopBits(SerialPort::StopBits stopBits)
{
    _pi->ensureClosed();

    _pi->stopBits = stopBits;
}

void SerialPort::write(const char data[], size_t length)
{
    _pi->ensureOpened();

    ssize_t numOfBytesWritten = ::write(
        _pi->SerialPortHandle,
        data,
        length);

    if (numOfBytesWritten == -1)
        throw std::runtime_error("Could not write to serial port");
}

void SerialPort::read(unsigned char dataBuffer[], size_t numOfBytesToRead, size_t& numOfBytesActuallyRead)
{
    _pi->ensureOpened();

    int result = ::read(
        _pi->SerialPortHandle,
        dataBuffer,
        numOfBytesToRead);

    if (result == -1)
        throw std::runtime_error("Could not read from serial port");

    numOfBytesActuallyRead = static_cast<size_t>(result);
}

void SerialPort::registerDataReceivedHandler(void* userData, DataReceivedHandler handler)
{
    if (_pi->_dataReceivedHandler != NULL)
        throw std::logic_error("Need to unregister old handler first!");

    _pi->ObserversCriticalSection.enter();

    _pi->_dataReceivedHandler = handler;
    _pi->_dataReceivedUserData = userData;

    _pi->ObserversCriticalSection.leave();
}

void SerialPort::unregisterDataReceivedHandler()
{
    if (_pi->_dataReceivedHandler == NULL)
        throw std::logic_error("Need to register a handler first!");

    _pi->ObserversCriticalSection.enter();

    _pi->_dataReceivedHandler = NULL;
    _pi->_dataReceivedUserData = NULL;

    _pi->ObserversCriticalSection.leave();
}

uint32_t SerialPort::baudrate()
{
    return _pi->Baudrate;
}

std::string SerialPort::port()
{
    return _pi->PortName;
}

void SerialPort::changeBaudrate(uint32_t baudrate)
{
    _pi->ensureOpened();

    _pi->close(false);

    _pi->Baudrate = baudrate;

    _pi->open(false);
}

size_t SerialPort::NumberOfReceiveDataDroppedSections()
{
    serial_icounter_struct serialStatus;

    ioctl(
        _pi->SerialPortHandle,
        TIOCGICOUNT,
        &serialStatus);

    return serialStatus.overrun + serialStatus.buf_overrun;
}

} // namespace ub::xplat
