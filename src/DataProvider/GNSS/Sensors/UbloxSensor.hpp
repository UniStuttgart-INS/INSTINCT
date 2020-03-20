/**
 * @file UbloxSensor.hpp
 * @brief Ublox Sensor Class
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-03-19
 */

#pragma once

#include "../Gnss.hpp"
#include "DataProvider/Protocol/UartSensor.hpp"
#include "ub/sensors/sensors.hpp"

namespace NAV
{
/// Ublox Sensor Class
class UbloxSensor : public Gnss, public UartSensor
{
  public:
    /// Config Structure for the sensor
    typedef struct Config
    {
        /// OutputFrequency of async packages
        uint16_t outputFrequency = 1;

        /// COM port where the sensor is attached to
        std::string sensorPort;

        /// Baudrate for the sensor (factory default is 9600)
        Baudrate sensorBaudrate = BAUDRATE_9600;
    } Config;

    /**
     * @brief Construct a new ublox Sensor object
     * 
     * @param[in] name Name of the Sensor
     * @param[in] sensorConfig Config Structure for the sensor
     */
    UbloxSensor(std::string name, const UbloxSensor::Config sensorConfig);

    /// Default Destructor
    virtual ~UbloxSensor();

    /**
     * @brief Initialize the Sensor
     * 
     * @retval NavStatus Indicates whether initialization was successfull
     */
    NavStatus initialize() final;

    /**
     * @brief Deinitialize the Sensor
     * 
     * @retval NavStatus Indicates whether deinitialization was successfull
     */
    NavStatus deinitialize() final;

    /**
     * @brief Polls the current Gnss Data
     * 
     * @retval std::shared_ptr<InsObs> Pointer to an UbloxObs object with the current data
     */
    std::shared_ptr<InsObs> pollObservation() final;

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
    const UbloxSensor::Config config;
};

} // namespace NAV
