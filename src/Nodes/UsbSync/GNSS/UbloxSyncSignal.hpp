/**
 * @file UbloxSyncSignal.hpp
 * @brief 
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-03-20
 */

#pragma once

#include "../UsbSyncSignal.hpp"
#include "ub/protocol/types.hpp"

namespace NAV
{
class UbloxSyncSignal : public UsbSyncSignal
{
  public:
    /**
     * @brief Construct a new Usb Sync Signal object
     * 
     * @param[in] name Name of the Object
     * @param[in] options Program options string list
     */
    UbloxSyncSignal(std::string name, std::vector<std::string> options);

    /// Default destructor
    virtual ~UbloxSyncSignal();

    /**
     * @brief Initialize the Ublox Usb Sync Signal
     * 
     * @retval NavStatus Indicates whether initialization was successfull
     */
    NavStatus initialize();

    /**
     * @brief Deinitialize the Ublox Usb Sync Signal
     * 
     * @retval NavStatus Indicates whether deinitialization was successfull
     */
    NavStatus deinitialize();

    /**
     * @brief Triggers a Sync Signal to the USB port
     * 
     * @param[in] observation UbloxObservation to process
     * @param[in] userData UbloxSyncSignal pointer
     * @retval NavStatus Indicates whether the sync was successfully sent
     */
    static NavStatus triggerSync(std::shared_ptr<void> observation, std::shared_ptr<void> userData);

  private:
    /// Message Class to send the sync on
    ub::protocol::uart::UbxClass triggerClass;

    /// Message Id to send the sync on
    uint8_t triggerId;
};

} // namespace NAV
