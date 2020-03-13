/**
 * @file VectorNavSensor.hpp
 * @brief Vector Nav Sensors
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-03-12
 */

#pragma once

#include "../Imu.hpp"
#include "DataProvider/Protocol/UartSensor.hpp"
#include "vn/sensors.h"

namespace NAV
{
/// Vector Nav Sensor Class
class VectorNavSensor : public Imu, public UartSensor
{
  public:
    /// Config Structure for the sensor
    typedef struct vnConfig
    {
        /// OutputFrequency to calculate rateDivisor field.
        uint16_t outputFrequency = 1;

        /// COM port where the sensor is attached to. If Sensor is not found under this port, others will be searched
        std::string sensorPort;

        /// Baudrate for the sensor (factory default is 115200)
        Baudrate sensorBaudrate = BAUDRATE_FASTEST;

        /// The asyncMode field
        vn::protocol::uart::AsyncMode asyncMode = vn::protocol::uart::AsyncMode::ASYNCMODE_PORT1;

        /// Controls how the VPE interprets the magnetic measurements to estimate the heading angle
        vn::protocol::uart::HeadingMode headingMode = vn::protocol::uart::HeadingMode::HEADINGMODE_RELATIVE;

        /// Delta Theta and Delta Velocity Configuration - The integrationFrame field
        vn::protocol::uart::IntegrationFrame delThetaDelVeloIntegrationFrame = vn::protocol::uart::IntegrationFrame::INTEGRATIONFRAME_BODY;
        /// Delta Theta and Delta Velocity Configuration - The gyroCompensation field
        vn::protocol::uart::CompensationMode delThetaDelVeloGyroCompensation = vn::protocol::uart::CompensationMode::COMPENSATIONMODE_NONE;
        /// Delta Theta and Delta Velocity Configuration - The accelCompensation field
        vn::protocol::uart::CompensationMode delThetaDelVeloAccelCompensation = vn::protocol::uart::CompensationMode::COMPENSATIONMODE_NONE;

        /// Group 1 (Common)
        vn::protocol::uart::CommonGroup commonField = vn::protocol::uart::CommonGroup::COMMONGROUP_TIMESTARTUP | vn::protocol::uart::CommonGroup::COMMONGROUP_TIMESYNCIN | vn::protocol::uart::CommonGroup::COMMONGROUP_DELTATHETA | vn::protocol::uart::CommonGroup::COMMONGROUP_SYNCINCNT;
        /// Group 2 (Time)
        vn::protocol::uart::TimeGroup timeField = vn::protocol::uart::TimeGroup::TIMEGROUP_NONE;
        /// Group 3 (IMU)
        vn::protocol::uart::ImuGroup imuField = vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPMAG | vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPACCEL | vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPGYRO | vn::protocol::uart::ImuGroup::IMUGROUP_TEMP | vn::protocol::uart::ImuGroup::IMUGROUP_PRES | vn::protocol::uart::ImuGroup::IMUGROUP_MAG | vn::protocol::uart::ImuGroup::IMUGROUP_ACCEL | vn::protocol::uart::ImuGroup::IMUGROUP_ANGULARRATE;
        /// Group 5 (Attitude)
        vn::protocol::uart::AttitudeGroup attitudeField = vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_VPESTATUS | vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_QUATERNION | vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_MAGNED | vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_ACCELNED | vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_LINEARACCELBODY | vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_LINEARACCELNED | vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YPRU;
        // Group 4 (GPS)
        // vn::protocol::uart::GpsGroup gpsField = vn::protocol::uart::GpsGroup::GPSGROUP_NONE;
        // Group 6 (INS)
        // vn::protocol::uart::InsGroup insField = vn::protocol::uart::InsGroup::INSGROUP_NONE;
        // Group 7 (GPS2)
        // vn::protocol::uart::GpsGroup gps2Field = vn::protocol::uart::GpsGroup::GPSGROUP_NONE;
    } VNConfig;

    /**
     * @brief Construct a new Vector Nav Sensor object
     * 
     * @param[in] name Name of the Sensor
     * @param[in] sensorConfig Config Structure for the VectorNavSensor
     */
    VectorNavSensor(std::string name, const VNConfig sensorConfig);

    /// Default Destructor
    virtual ~VectorNavSensor();

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
     * @brief Polls the current Imu Data
     * 
     * @retval std::shared_ptr<InsObs> Pointer to an ImuObs object with the current data
     */
    std::shared_ptr<InsObs> pollObservation() final;

  private:
    /**
     * @brief Callback handler for notifications of new asynchronous data packets received
     * 
     * @param[in, out] userData Pointer to the data we supplied when we called registerAsyncPacketReceivedHandler
     * @param[in, out] p Encapsulation of the data packet. At this state, it has already been validated and identified as an asynchronous data message
     * @param[in, out] index Advanced usage item and can be safely ignored for now
     */
    static void asciiOrBinaryAsyncMessageReceived(void* userData, vn::protocol::uart::Packet& p, size_t index);

    /// VnSensor Object
    vn::sensors::VnSensor vs;

    /// Config Object
    const VNConfig config;

    /// Internal Frequency of the Sensor
    const double IMU_DEFAULT_FREQUENCY = 800;
};

} // namespace NAV
