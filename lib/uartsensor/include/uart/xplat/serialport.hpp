#pragma once

#include <string>
#include <vector>
#include <list>

#include "uart/xplat/export.hpp"
#include "uart/xplat/port.hpp"
#include "uart/util/nocopy.hpp"

namespace uart::xplat
{
/// \brief Represents a cross-platform serial port.
///
/// When the SerialPort if first created and the connection opened, the user
/// will normally have to poll the method \ref read to see if any new data is
/// available on the serial port. However, if the user code registers a
/// handler with the method \ref registerDataReceivedHandler, the SerialPort
/// object will start an internal thread that monitors the serial port for new
/// data, and when new data is available, it will alert the user code through
/// the callback handler. Then the user can call \ref read to retrieve the
/// data.
class proglib_DLLEXPORT SerialPort : public IPort, util::NoCopy
{
    // Types //////////////////////////////////////////////////////////////////

  public:
    enum StopBits
    {
        ONE_STOP_BIT,
        TWO_STOP_BITS
    };

    // Constructors ///////////////////////////////////////////////////////////

    /// \brief Creates a new \ref SerialPort with the provided connection parameters.
    ///
    /// \param[in] portName The name of the serial port.
    /// \param[in] baudrate The baudrate to open the serial port at.
    SerialPort(const std::string& portName, uint32_t baudrate);

    /// Destructor
    ~SerialPort() override;

    /// Copy constructor
    SerialPort(const SerialPort&) = delete;
    /// Move constructor
    SerialPort(SerialPort&&) = delete;
    /// Copy assignment operator
    SerialPort& operator=(const SerialPort&) = delete;
    /// Move assignment operator
    SerialPort& operator=(SerialPort&&) = delete;

    // Public Methods /////////////////////////////////////////////////////////

    /// \brief Returns a list of the names of all the available serial ports on the system.
    ///
    /// \return The list of available serial port names.
    static std::vector<std::string> getPortNames();

    /// \brief Opens the port.
    void open() override;

    /// \brief Closes the port.
    void close() override;

    /// \brief Indicates if the port is open.
    ///
    /// \return <c>true</c> if the serial port is open; otherwise <c>false</c>.
    bool isOpen() override;

    /// \brief Writes out data to the port.
    ///
    /// \param[in] data The data array to write out.
    /// \param[in] length The length of the data array to write out.
    void write(const char* data, size_t length) override;

    /// \brief Allows reading data from the port.
    ///
    /// \param[out] dataBuffer The data buffer to write the read data bytes to.
    ///     from the port.
    void read(std::vector<unsigned char>& dataBuffer) override;

    /// \brief Registers a callback method for notification when new data is
    ///     received on the port.
    ///
    /// \param[in] userData Pointer to user data, which will be provided to the
    ///     callback method.
    /// \param[in] handler The callback method.
    void registerDataReceivedHandler(void* userData, DataReceivedHandler handler) override;

    /// \brief Unregisters the registered callback method.
    void unregisterDataReceivedHandler() override;

    /// \brief Returns the baudrate connected at.
    ///
    /// \return The connected baudrate.
    uint32_t baudrate();

    /// \brief Returns the port connected to.
    ///
    /// \return The port name.
    std::string port();

    /// \brief Changes the connected baudrate of the port.
    ///
    /// \param[in] baudrate The baudrate to change the port to.
    void changeBaudrate(uint32_t baudrate);

    /// \brief Returns the stop bit configuration.
    ///
    /// \return The current stop bit configuration.
    StopBits stopBits();

    /// \brief Sets the stop bit configuration.
    ///
    /// \param[in] stopBits The stop bit configuration.
    void setStopBits(StopBits stopBits);

    /// \brief Returns the number of dropped sections of received data.
    ///
    /// \return The number of sections of dropped data sections. Note this is
    ///     not indicative of the total number of dropped bytes.
    size_t NumberOfReceiveDataDroppedSections();

    /// \brief With regard to optimizing COM ports provided by FTDI drivers, this
    /// method will check if the COM port has been optimized.
    ///
    /// \param[in] portName The COM port name to check.
    /// \return <c>true</c> if the COM port is optimized; otherwise <c>false</c>.
    static bool determineIfPortIsOptimized(const std::string& portName);

    /// \brief This will perform optimization of FTDI USB serial ports.
    ///
    /// If calling this method on Windows, the process must have administrator
    /// privileges to write settings to the registry. Otherwise an
    ///
    /// \param[in] portName The FTDI USB Serial Port to optimize.
    static void optimizePort(const std::string& portName);

    // Private Members ////////////////////////////////////////////////////////

  private:
    // Contains internal data, mainly stuff that is required for cross-platform support.
    struct Impl;
    Impl* _pi;
};

} // namespace uart::xplat