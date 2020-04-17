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
     * @param[in, out] options Program options string list
     */
    UbloxSyncSignal(std::string name, std::deque<std::string>& options);

    /// Default destructor
    virtual ~UbloxSyncSignal();

    /**
     * @brief Triggers a Sync Signal to the USB port
     * 
     * @param[in] observation UbloxObservation to process
     * @param[in] userData UbloxSyncSignal pointer
     * @retval NavStatus Indicates whether the sync was successfully sent
     */
    static NavStatus triggerSync(std::shared_ptr<NodeData> observation, std::shared_ptr<Node> userData);

  private:
    /// Message Class to send the sync on
    ub::protocol::uart::UbxClass triggerClass;

    /// Message Id to send the sync on
    uint8_t triggerId;
};

} // namespace NAV
