/// @file VectorNavSensor.hpp
/// @brief Vector Nav Sensors
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-03-12

#pragma once

#include "Nodes/DataProvider/IMU/Imu.hpp"
#include "Nodes/DataProvider/Protocol/UartSensor.hpp"
#include "vn/sensors.h"

#include <vector>

namespace NAV
{
/// Vector Nav Sensor Class
class VectorNavSensor : public Imu, public UartSensor
{
  public:
    /// Config Structure for the sensor
    struct Config
    {
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
        vn::protocol::uart::CommonGroup commonField = vn::protocol::uart::CommonGroup::COMMONGROUP_TIMESTARTUP
                                                      | vn::protocol::uart::CommonGroup::COMMONGROUP_TIMESYNCIN
                                                      | vn::protocol::uart::CommonGroup::COMMONGROUP_DELTATHETA
                                                      | vn::protocol::uart::CommonGroup::COMMONGROUP_SYNCINCNT;
        /// Group 2 (Time)
        vn::protocol::uart::TimeGroup timeField = vn::protocol::uart::TimeGroup::TIMEGROUP_NONE;
        /// Group 3 (IMU)
        vn::protocol::uart::ImuGroup imuField = vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPMAG
                                                | vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPACCEL
                                                | vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPGYRO
                                                | vn::protocol::uart::ImuGroup::IMUGROUP_TEMP
                                                | vn::protocol::uart::ImuGroup::IMUGROUP_PRES
                                                | vn::protocol::uart::ImuGroup::IMUGROUP_MAG
                                                | vn::protocol::uart::ImuGroup::IMUGROUP_ACCEL
                                                | vn::protocol::uart::ImuGroup::IMUGROUP_ANGULARRATE;
        /// Group 5 (Attitude)
        vn::protocol::uart::AttitudeGroup attitudeField = vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_VPESTATUS
                                                          | vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YAWPITCHROLL
                                                          | vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_QUATERNION
                                                          | vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_MAGNED
                                                          | vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_ACCELNED
                                                          | vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_LINEARACCELBODY
                                                          | vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_LINEARACCELNED
                                                          | vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YPRU;
        // Group 4 (GPS)
        // vn::protocol::uart::GpsGroup gpsField = vn::protocol::uart::GpsGroup::GPSGROUP_NONE;
        // Group 6 (INS)
        // vn::protocol::uart::InsGroup insField = vn::protocol::uart::InsGroup::INSGROUP_NONE;
        // Group 7 (GPS2)
        // vn::protocol::uart::GpsGroup gps2Field = vn::protocol::uart::GpsGroup::GPSGROUP_NONE;
    };

    /// @brief Default constructor
    VectorNavSensor();
    /// @brief Destructor
    ~VectorNavSensor() override;
    /// @brief Copy constructor
    VectorNavSensor(const VectorNavSensor&) = delete;
    /// @brief Move constructor
    VectorNavSensor(VectorNavSensor&&) = delete;
    /// @brief Copy assignment operator
    VectorNavSensor& operator=(const VectorNavSensor&) = delete;
    /// @brief Move assignment operator
    VectorNavSensor& operator=(VectorNavSensor&&) = delete;

    /// @brief String representation of the Class Type
    [[nodiscard]] static std::string typeStatic();

    /// @brief String representation of the Class Type
    [[nodiscard]] std::string type() const override;

    /// @brief String representation of the Class Category
    [[nodiscard]] static std::string category();

    /// @brief ImGui config window which is shown on double click
    /// @attention Don't forget to set hasConfig to true in the constructor of the node
    void guiConfig() override;

    /// @brief Saves the node into a json object
    [[nodiscard]] json save() const override;

    /// @brief Restores the node from a json object
    /// @param[in] j Json object with the node state
    void restore(const json& j) override;

  private:
    constexpr static size_t OutputPortIndex_VectorNavObs = 1; ///< @brief Flow (VectorNavObs)

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Callback handler for notifications of new asynchronous data packets received
    /// @param[in, out] userData Pointer to the data we supplied when we called registerAsyncPacketReceivedHandler
    /// @param[in] p Encapsulation of the data packet. At this state, it has already been validated and identified as an asynchronous data message
    /// @param[in] index Advanced usage item and can be safely ignored for now
    static void asciiOrBinaryAsyncMessageReceived(void* userData, vn::protocol::uart::Packet& p, size_t index);

    /// VnSensor Object
    vn::sensors::VnSensor vs;

    /// Config Object
    VectorNavSensor::Config config;

    /// Internal Frequency of the Sensor
    constexpr static double IMU_DEFAULT_FREQUENCY = 800;

    /// The in the GUI selected Frequency
    int selectedFrequency = 0;
    /// First: List of dividers, Second: List of Matching Frequencies
    std::pair<std::vector<uint16_t>, std::vector<std::string>> dividerFrequency;
};

} // namespace NAV