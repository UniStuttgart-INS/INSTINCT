/**
 * @file VectorNavSensor.hpp
 * @brief Vector Nav Sensors
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-03-12
 */

#pragma once

#ifndef DISABLE_VN_SENSORS

    #include "NodeData/IMU/VectorNavObs.hpp"
    #include "../Imu.hpp"
    #include "../../Protocol/UartSensor.hpp"
    #include "vn/sensors.h"

namespace NAV
{
/// Vector Nav Sensor Class
class VectorNavSensor final : public UartSensor, public Imu
{
  public:
    /// Config Structure for the sensor
    using Config = struct
    {
        /// OutputFrequency to calculate rateDivisor field.
        uint16_t outputFrequency = 1;

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
    };

    /**
     * @brief Construct a new Vector Nav Sensor object
     * 
     * @param[in] name Name of the Sensor
     * @param[in] options Program options string map
     */
    VectorNavSensor(const std::string& name, const std::map<std::string, std::string>& options);

    VectorNavSensor() = default;                                 ///< Default Constructor
    ~VectorNavSensor() final;                                    ///< Destructor
    VectorNavSensor(const VectorNavSensor&) = delete;            ///< Copy constructor
    VectorNavSensor(VectorNavSensor&&) = delete;                 ///< Move constructor
    VectorNavSensor& operator=(const VectorNavSensor&) = delete; ///< Copy assignment operator
    VectorNavSensor& operator=(VectorNavSensor&&) = delete;      ///< Move assignment operator

    /**
     * @brief Returns the String representation of the Class Type
     * 
     * @retval constexpr std::string_view The class type
     */
    [[nodiscard]] constexpr std::string_view type() const final
    {
        return std::string_view("VectorNavSensor");
    }

    /**
     * @brief Returns the String representation of the Class Category
     * 
     * @retval constexpr std::string_view The class category
     */
    [[nodiscard]] constexpr std::string_view category() const final
    {
        return std::string_view("DataProvider");
    }

    /**
     * @brief Returns Gui Configuration options for the class
     * 
     * @retval std::vector<ConfigOptions> The gui configuration
     */
    [[nodiscard]] std::vector<ConfigOptions> guiConfig() const final
    {
        return { { CONFIG_STRING, "Port", "COM port where the sensor is attached to\n"
                                          "- \"COM1\" (Windows format for physical and virtual (USB) serial port)\n"
                                          "- \"/dev/ttyS1\" (Linux format for physical serial port)\n"
                                          "- \"/dev/ttyUSB0\" (Linux format for virtual (USB) serial port)\n"
                                          "- \"/dev/tty.usbserial-FTXXXXXX\" (Mac OS X format for virtual (USB) serial port)\n"
                                          "- \"/dev/ttyS0\" (CYGWIN format. Usually the Windows COM port number minus 1. This would connect to COM1)",
                   { "/dev/ttyUSB0" } },
                 { CONFIG_LIST, "Baudrate", "Target Baudrate for the sensor", { "[Fastest]", "9600", "19200", "38400", "57600", "115200", "128000", "230400", "460800", "921600" } },
                 { CONFIG_INT, "Frequency", "Data Output Frequency", { "0", "100", "200" } } };
    }

    /**
     * @brief Returns the context of the class
     * 
     * @retval constexpr std::string_view The class context
     */
    [[nodiscard]] constexpr NodeContext context() const final
    {
        return NodeContext::REAL_TIME;
    }

    /**
     * @brief Returns the number of Ports
     * 
     * @param[in] portType Specifies the port type
     * @retval constexpr uint8_t The number of ports
     */
    [[nodiscard]] constexpr uint8_t nPorts(PortType portType) const final
    {
        switch (portType)
        {
        case PortType::In:
            break;
        case PortType::Out:
            return 1U;
        }

        return 0U;
    }

    /**
     * @brief Returns the data types provided by this class
     * 
     * @param[in] portType Specifies the port type
     * @param[in] portIndex Port index on which the data is sent
     * @retval constexpr std::string_view The data type
     */
    [[nodiscard]] constexpr std::string_view dataType(PortType portType, uint8_t portIndex) const final
    {
        switch (portType)
        {
        case PortType::In:
            break;
        case PortType::Out:
            if (portIndex == 0)
            {
                return VectorNavObs().type();
            }
        }

        return std::string_view("");
    }

    /**
     * @brief Handles the data sent on the input port
     * 
     * @param[in] portIndex The input port index
     * @param[in, out] data The data send on the input port
     */
    void handleInputData(uint8_t /* portIndex */, std::shared_ptr<NodeData> /* data */) final {}

    /**
     * @brief Requests the node to send out its data
     * 
     * @param[in] portIndex The output port index
     * @retval std::shared_ptr<NodeData> The requested data or nullptr if no data available
     */
    [[nodiscard]] std::shared_ptr<NodeData> requestOutputData(uint8_t /* portIndex */) final { return nullptr; }

    /**
     * @brief Requests the node to peek its output data
     * 
     * @param[in] portIndex The output port index
     * @retval std::shared_ptr<NodeData> The requested data or nullptr if no data available
     */
    [[nodiscard]] std::shared_ptr<NodeData> requestOutputDataPeek(uint8_t /* portIndex */) final { return nullptr; }

  private:
    /**
     * @brief Callback handler for notifications of new asynchronous data packets received
     * 
     * @param[in, out] userData Pointer to the data we supplied when we called registerAsyncPacketReceivedHandler
     * @param[in] p Encapsulation of the data packet. At this state, it has already been validated and identified as an asynchronous data message
     * @param[in] index Advanced usage item and can be safely ignored for now
     */
    static void asciiOrBinaryAsyncMessageReceived(void* userData, vn::protocol::uart::Packet& p, size_t index);

    /// VnSensor Object
    vn::sensors::VnSensor vs;

    /// Config Object
    VectorNavSensor::Config config;

    /// Internal Frequency of the Sensor
    constexpr static double IMU_DEFAULT_FREQUENCY = 800;
};

} // namespace NAV

#endif