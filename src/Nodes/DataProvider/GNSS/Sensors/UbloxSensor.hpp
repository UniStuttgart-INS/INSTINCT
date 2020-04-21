/**
 * @file UbloxSensor.hpp
 * @brief Ublox Sensor Class
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-03-19
 */

#pragma once

#include "../Gnss.hpp"
#include "../../Protocol/UartSensor.hpp"
#include "ub/sensors/sensors.hpp"

namespace NAV
{
/// Ublox Sensor Class
class UbloxSensor : public UartSensor, public Gnss
{
  public:
    /// Config Structure for the sensor
    typedef struct Config
    {
        /// OutputFrequency of async packages
        uint16_t outputFrequency = 1;
    } Config;

    /**
     * @brief Construct a new ublox Sensor object
     * 
     * @param[in] name Name of the Sensor
     * @param[in, out] options Program options string list
     */
    UbloxSensor(std::string name, std::deque<std::string>& options);

    /// Default Destructor
    virtual ~UbloxSensor();

  private:
    /**
     * @brief Callback handler for notifications of new asynchronous data packets received
     * 
     * @param[in, out] userData Pointer to the data we supplied when we called registerAsyncPacketReceivedHandler
     * @param[in] p Encapsulation of the data packet. At this state, it has already been validated and identified as an asynchronous data message
     * @param[in] index Advanced usage item and can be safely ignored for now
     */
    static void asciiOrBinaryAsyncMessageReceived(void* userData, ub::protocol::uart::Packet& p, size_t index);

    /// UbSensor Object
    ub::sensors::UbSensor ub;

    /// Config Object
    UbloxSensor::Config config;
};

} // namespace NAV
