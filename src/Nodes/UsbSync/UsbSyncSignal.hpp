/**
 * @file UsbSyncSignal.hpp
 * @brief Class to send Sync Signals over the USB RTS Pin
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-03-20
 */

#pragma once

#include <string>

#include "Nodes/Node.hpp"

#include <sys/ioctl.h> //ioctl

namespace NAV
{
/// Class to send Sync Signals over the USB RTS Pin
class UsbSyncSignal : public Node
{
  protected:
    /**
     * @brief Construct a new Usb Sync Signal object
     * 
     * @param[in] name Name of the Object
     * @param[in] port COM port where to send the sync to
     */
    UsbSyncSignal(std::string name, std::string port);

    /**
     * @brief Construct a new Usb Sync Signal object
     * 
     * @param[in] name Name of the Object
     */
    UsbSyncSignal(std::string name);

    /// Default destructor
    ~UsbSyncSignal();

    /**
     * @brief Initialize the Usb Sync Signal
     * 
     * @retval NavStatus Indicates whether initialization was successfull
     */
    NavStatus initialize() override;

    /**
     * @brief Deinitialize the Usb Sync Signal
     * 
     * @retval NavStatus Indicates whether deinitialization was successfull
     */
    NavStatus deinitialize() override;

  protected:
    /// File descriptor of the sensor port
    int fd;

    /// COM port where to send the sync to
    std::string port;

    /// Flag for the USB Pin to trigger
    int pinFlag = TIOCM_RTS;
};

} // namespace NAV
