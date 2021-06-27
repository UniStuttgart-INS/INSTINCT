/// @file VectorNavSensor.hpp
/// @brief Vector Nav Sensors
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-03-12

#pragma once

#include "Nodes/DataProvider/IMU/Imu.hpp"
#include "Nodes/DataProvider/Protocol/UartSensor.hpp"
#include "vn/sensors.h"

#include <vector>
#include <array>

namespace NAV
{
/// Vector Nav Sensor Class
class VectorNavSensor : public Imu, public UartSensor
{
  public:
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

    /// @brief Resets the node. It is guaranteed that the node is initialized when this is called.
    bool resetNode() override;

  private:
    constexpr static size_t OutputPortIndex_VectorNavObs = 1; ///< @brief Flow (VectorNavObs)

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
        // Group 4 (GNSS1)
        vn::protocol::uart::GpsGroup gnss1Field = vn::protocol::uart::GpsGroup::GPSGROUP_NONE;

        /// Group 5 (Attitude)
        vn::protocol::uart::AttitudeGroup attitudeField = vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_VPESTATUS
                                                          | vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YAWPITCHROLL
                                                          | vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_QUATERNION
                                                          | vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_MAGNED
                                                          | vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_ACCELNED
                                                          | vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_LINEARACCELBODY
                                                          | vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_LINEARACCELNED
                                                          | vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YPRU;
        // Group 6 (INS)
        vn::protocol::uart::InsGroup insField = vn::protocol::uart::InsGroup::INSGROUP_NONE;
        // Group 7 (GNSS2)
        vn::protocol::uart::GpsGroup gnss2Field = vn::protocol::uart::GpsGroup::GPSGROUP_NONE;
    };

    /// @brief VectorNav Model enumeration
    enum class VectorNavModel : int
    {
        /// VN-100/SMD (Miniature, lightweight and high-performance IMU & AHRS)
        /// VN-110/E (Rugged and Miniature Tactical-Grade IMU and AHRS)
        VN100_VN110,
        /// VN-310/E (Tactical-Grade GNSS/INS with Integrated GNSS-Compass)
        VN310,
    };

    /// @brief The sensor model which is selected in the GUI
    VectorNavModel sensorModel = VectorNavModel::VN100_VN110;

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
    static constexpr double IMU_DEFAULT_FREQUENCY = 800;

    /// The selected Frequency in the GUI
    int selectedFrequency = 0;
    /// First: List of dividers, Second: List of Matching Frequencies
    std::pair<std::vector<uint16_t>, std::vector<std::string>> dividerFrequency;

    /// @brief Needed data to display a binary group in the GUI
    struct BinaryGroupData
    {
        const char* name;                              ///< Name of the output
        int flagsValue;                                ///< Enum value of the output
        bool (*isEnabled)(VectorNavModel sensorModel); ///< Function which checks if the ouput is enabled (e.g. for a sensorModel)
        void (*tooltip)();                             ///< Function providing a tooltip
    };

    /// @brief Binary group 1 contains a wide assortment of commonly used data required for most applications.
    ///
    /// All of the outputs found in group 1 are also present in the other groups. In this sense, group 1 is a subset of
    /// commonly used outputs from the other groups. This simplifies the configuration of binary output messages for
    /// applications that only require access to the commonly used data found in group 1. For these applications
    /// you can hard code the group field to 1, and not worry about implemented support for the other binary groups.
    /// Using group 1 for commonly used outputs also has the advantage of reducing the overall packet size, since
    /// the packet length is dependent upon the number of binary groups active.
    static const std::array<BinaryGroupData, 15> binaryGroupCommon;

    /// @brief Binary group 2 provides all timing and event counter related outputs.
    ///
    /// Some of these outputs (such as the TimeGps, TimePps, and TimeUtc), require either that the internal GNSS to be
    /// enabled, or an external GNSS must be present.
    static const std::array<BinaryGroupData, 10> binaryGroupTime;

    /// @brief Binary group 3 provides all outputs which are dependent upon the measurements collected from the
    ///        onboard IMU, or an external IMU (if enabled).
    static const std::array<BinaryGroupData, 11> binaryGroupIMU;

    /// @brief Binary group 4 provides all outputs which are dependent upon the measurements collected from the primary
    ///        onboard, Binary group 7 from the secondary onboard GNSS, or external GNSS (if enabled).
    ///
    /// All data in this group is updated at the rate of the GNSS receiver (nominally 5Hz for the internal GNSS).
    ///
    /// @note If data is asynchronously sent from group 4/7 at a rate equal to the GNSS update rate, then packets
    ///       will be sent out when updated by the GNSS receiver. For all other rates, the output will be based
    ///       on the divisor selected and the internal IMU sampling rate.
    static const std::array<BinaryGroupData, 16> binaryGroupGNSS;

    /// @brief Binary group 5 provides all estimated outputs which are dependent upon the estimated attitude solution.
    ///
    /// The attitude will be derived from either the AHRS or the INS, depending upon which filter is currently active and
    /// tracking. All of the fields in this group will only be valid if the AHRS/INS filter is currently enabled and tracking.
    static const std::array<BinaryGroupData, 9> binaryGroupAttitude;

    /// @brief Binary group 6 provides all estimated outputs which are dependent upon the onboard INS state solution.
    ///
    /// All of the fields in this group will only be valid if the INS filter is currently enabled and tracking.
    static const std::array<BinaryGroupData, 11> binaryGroupINS;
};

} // namespace NAV