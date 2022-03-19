/// @file VectorNavSensor.hpp
/// @brief Vector Nav Sensors
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-03-12

#pragma once

#include "Nodes/DataProvider/IMU/Imu.hpp"
#include "Nodes/DataProvider/Protocol/UartSensor.hpp"
#include "vn/sensors.h"

#include "NodeData/IMU/VectorNavBinaryOutput.hpp"

#include "Navigation/Time/InsTime.hpp"
#include "util/Container/ScrollingBuffer.hpp"

#include <vector>
#include <array>
#include <cstdint>

namespace NAV
{
class VectorNavFile;

/// Vector Nav Sensor Class
class VectorNavSensor : public Imu, public UartSensor
{
  public:
    /// Information needed to sync Master/Slave sensors
    struct TimeSync
    {
        InsTime ppsTime{};     ///< Time of the last message with GNSS Time available (or empty otherwise)
        uint32_t syncOutCnt{}; ///< The number of SyncOut trigger events that have occurred.
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
    /// @attention Don't forget to set _hasConfig to true in the constructor of the node
    void guiConfig() override;

    /// @brief Saves the node into a json object
    [[nodiscard]] json save() const override;

    /// @brief Restores the node from a json object
    /// @param[in] j Json object with the node state
    void restore(const json& j) override;

    /// @brief Resets the node. It is guaranteed that the node is initialized when this is called.
    bool resetNode() override;

  private:
    constexpr static size_t OUTPUT_PORT_INDEX_ASCII_OUTPUT = 0; ///< @brief Flow (StringObs)

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Merges the content of the two observations into one
    /// @param[in, out] target The observation used to store the merged information
    /// @param[in] source The observation where information is taken from
    static void mergeVectorNavBinaryObservations(std::shared_ptr<VectorNavBinaryOutput> target, std::shared_ptr<VectorNavBinaryOutput> source);

    /// @brief Callback handler for notifications of new asynchronous data packets received
    /// @param[in, out] userData Pointer to the data we supplied when we called registerAsyncPacketReceivedHandler
    /// @param[in] p Encapsulation of the data packet. At this state, it has already been validated and identified as an asynchronous data message
    /// @param[in] index Advanced usage item and can be safely ignored for now
    static void asciiOrBinaryAsyncMessageReceived(void* userData, vn::protocol::uart::Packet& p, size_t index);

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
    VectorNavModel _sensorModel = VectorNavModel::VN100_VN110;

    /// VnSensor Object
    vn::sensors::VnSensor _vs;

    /// Connected sensor port
    std::string _connectedSensorPort;

    /// Internal Frequency of the Sensor
    static constexpr double IMU_DEFAULT_FREQUENCY = 800;

    /// First: List of RateDividers, Second: List of Matching Frequencies
    std::pair<std::vector<uint16_t>, std::vector<std::string>> _dividerFrequency;

    /// @brief Stores the time of the last received message
    std::array<InsTime, 3> _lastMessageTime{};

    // ###########################################################################################################
    //                                               SYSTEM MODULE
    // ###########################################################################################################

    /// @brief Async Data Output Type Register
    /// @note See User manual VN-310 - 8.2.7 (p 92f) / VN-100 - 5.2.7 (p 65)
    vn::protocol::uart::AsciiAsync _asyncDataOutputType = vn::protocol::uart::AsciiAsync::VNOFF;

    /// @brief Possible values for the Async Data Output Frequency Register
    /// @note See User manual VN-310 - 8.2.8 (p 94) / VN-100 - 5.2.8 (p 66)
    static constexpr std::array _possibleAsyncDataOutputFrequency = { 1, 2, 4, 5, 10, 20, 25, 40, 50, 100, 200 };

    /// @brief Async Data Output Frequency Register
    /// @note See User manual VN-310 - 8.2.8 (p 94) / VN-100 - 5.2.8 (p 66)
    uint32_t _asyncDataOutputFrequency = 40;
    /// @brief Selected Frequency of the Async Ascii Output in the GUI
    int _asyncDataOutputFrequencySelected = 7;

    /// @brief Max size of the Ascii Output
    int _asciiOutputBufferSize = 10;

    /// @brief Buffer to store Ascii Output Messages
    ScrollingBuffer<std::string> _asciiOutputBuffer{ static_cast<size_t>(_asciiOutputBufferSize) };

    /// @brief Synchronization Control.
    ///
    /// Contains parameters which allow the timing of the VN-310E to be synchronized with external devices.
    /// @note See User manual VN-310 - 8.2.9 (p 95f) / VN-100 - 5.2.9 (p 67f)
    vn::sensors::SynchronizationControlRegister _synchronizationControlRegister{
        vn::protocol::uart::SyncInMode::SYNCINMODE_COUNT,              // SyncInMode
        vn::protocol::uart::SyncInEdge::SYNCINEDGE_RISING,             // SyncInEdge
        0,                                                             // SyncInSkipFactor
        vn::protocol::uart::SyncOutMode::SYNCOUTMODE_NONE,             // SyncOutMode
        vn::protocol::uart::SyncOutPolarity::SYNCOUTPOLARITY_POSITIVE, // SyncOutPolarity
        0,                                                             // SyncOutSkipFactor
        100000000                                                      // SyncOutPulseWidth
    };

    /// @brief Time synchronization for master sensors
    TimeSync _timeSyncOut;

    /// Show the SyncIn Pin
    bool _syncInPin = false;

    /// @brief Communication Protocol Control.
    ///
    /// Contains parameters that controls the communication protocol used by the sensor.
    /// @note See User manual VN-310 - 8.2.10 (p 97ff) / VN-100 - 5.2.10 (p 69ff)
    vn::sensors::CommunicationProtocolControlRegister _communicationProtocolControlRegister{
        vn::protocol::uart::CountMode::COUNTMODE_NONE,           // SerialCount
        vn::protocol::uart::StatusMode::STATUSMODE_OFF,          // SerialStatus
        vn::protocol::uart::CountMode::COUNTMODE_NONE,           // SPICount
        vn::protocol::uart::StatusMode::STATUSMODE_OFF,          // SPIStatus
        vn::protocol::uart::ChecksumMode::CHECKSUMMODE_CHECKSUM, // SerialChecksum
        vn::protocol::uart::ChecksumMode::CHECKSUMMODE_OFF,      // SPIChecksum
        vn::protocol::uart::ErrorMode::ERRORMODE_SEND            // ErrorMode
    };

    /// Possible Merge combinations between the binary output registers
    enum class BinaryRegisterMerge
    {
        None,
        Output1_Output2,
        Output1_Output3,
        Output2_Output3,
    };

    /// Merge binary output registers together. This has to be done because VectorNav sensors have a buffer overflow when packages get too big.
    BinaryRegisterMerge _binaryOutputRegisterMerge = BinaryRegisterMerge::None;

    /// First observation received, which should be merged together
    std::shared_ptr<VectorNavBinaryOutput> _binaryOutputRegisterMergeObservation = nullptr;
    /// Index of the binary output for the merge observation stored
    size_t _binaryOutputRegisterMergeIndex{};

    /// @brief Binary Output Register 1 - 3.
    ///
    /// This register allows the user to construct a custom binary output message that
    /// contains a collection of desired estimated states and sensor measurements.
    /// @note See User manual VN-310 - 8.2.11-13 (p 100ff) / VN-100 - 5.2.11-13 (p 73ff)
    std::array<vn::sensors::BinaryOutputRegister, 3> _binaryOutputRegister = { vn::sensors::BinaryOutputRegister{
                                                                                   vn::protocol::uart::AsyncMode::ASYNCMODE_NONE,         // AsyncMode
                                                                                   800,                                                   // RateDivisor
                                                                                   vn::protocol::uart::CommonGroup::COMMONGROUP_NONE,     // CommonGroup
                                                                                   vn::protocol::uart::TimeGroup::TIMEGROUP_NONE,         // TimeGroup
                                                                                   vn::protocol::uart::ImuGroup::IMUGROUP_NONE,           // IMUGroup
                                                                                   vn::protocol::uart::GpsGroup::GPSGROUP_NONE,           // GNSS1Group
                                                                                   vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_NONE, // AttitudeGroup
                                                                                   vn::protocol::uart::InsGroup::INSGROUP_NONE,           // INSGroup
                                                                                   vn::protocol::uart::GpsGroup::GPSGROUP_NONE            // GNSS2Group
                                                                               },
                                                                               vn::sensors::BinaryOutputRegister{
                                                                                   vn::protocol::uart::AsyncMode::ASYNCMODE_NONE,         // AsyncMode
                                                                                   800,                                                   // RateDivisor
                                                                                   vn::protocol::uart::CommonGroup::COMMONGROUP_NONE,     // CommonGroup
                                                                                   vn::protocol::uart::TimeGroup::TIMEGROUP_NONE,         // TimeGroup
                                                                                   vn::protocol::uart::ImuGroup::IMUGROUP_NONE,           // IMUGroup
                                                                                   vn::protocol::uart::GpsGroup::GPSGROUP_NONE,           // GNSS1Group
                                                                                   vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_NONE, // AttitudeGroup
                                                                                   vn::protocol::uart::InsGroup::INSGROUP_NONE,           // INSGroup
                                                                                   vn::protocol::uart::GpsGroup::GPSGROUP_NONE            // GNSS2Group
                                                                               },
                                                                               vn::sensors::BinaryOutputRegister{
                                                                                   vn::protocol::uart::AsyncMode::ASYNCMODE_NONE,         // AsyncMode
                                                                                   800,                                                   // RateDivisor
                                                                                   vn::protocol::uart::CommonGroup::COMMONGROUP_NONE,     // CommonGroup
                                                                                   vn::protocol::uart::TimeGroup::TIMEGROUP_NONE,         // TimeGroup
                                                                                   vn::protocol::uart::ImuGroup::IMUGROUP_NONE,           // IMUGroup
                                                                                   vn::protocol::uart::GpsGroup::GPSGROUP_NONE,           // GNSS1Group
                                                                                   vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_NONE, // AttitudeGroup
                                                                                   vn::protocol::uart::InsGroup::INSGROUP_NONE,           // INSGroup
                                                                                   vn::protocol::uart::GpsGroup::GPSGROUP_NONE            // GNSS2Group
                                                                               } };
    /// @brief Selected Frequency of the Binary Outputs in the GUI
    std::array<size_t, 3> _binaryOutputSelectedFrequency{};

    // ###########################################################################################################
    //                                               IMU SUBSYSTEM
    // ###########################################################################################################

    /// @brief Reference Frame Rotation.
    ///
    /// Allows the measurements of the VN-310E to be rotated into a different reference frame.
    /// @note See User manual VN-310 - 9.2.4 (p 114) / VN-100 - 6.2.4 (p 85)
    vn::math::mat3f _referenceFrameRotationMatrix{ { 1, 0, 0 },
                                                   { 0, 1, 0 },
                                                   { 0, 0, 1 } };

    /// @brief IMU Filtering Configuration.
    ///
    /// Controls the level of filtering performed on the raw IMU measurements.
    /// @note See User manual VN-310 - 9.2.5 (p 115) / VN-100 - 6.2.5 (p 86)
    vn::sensors::ImuFilteringConfigurationRegister _imuFilteringConfigurationRegister{
        0,                                                      // MagWindowSize
        4,                                                      // AccelWindowSize
        4,                                                      // GyroWindowSize
        4,                                                      // TempWindowSize
        0,                                                      // PresWindowSize
        vn::protocol::uart::FilterMode::FILTERMODE_NOFILTERING, // MagFilterMode
        vn::protocol::uart::FilterMode::FILTERMODE_BOTH,        // AccelFilterMode
        vn::protocol::uart::FilterMode::FILTERMODE_BOTH,        // GyroFilterMode
        vn::protocol::uart::FilterMode::FILTERMODE_BOTH,        // TempFilterMode
        vn::protocol::uart::FilterMode::FILTERMODE_NOFILTERING  // PresFilterMode
    };

    /// @brief Delta Theta and Delta Velocity Configuration.
    ///
    /// This register contains configuration options for the internal coning/sculling calculations.
    /// @note See User manual VN-310 - 9.2.6 (p 116) / VN-100 - 6.2.6 (p 87)
    vn::sensors::DeltaThetaAndDeltaVelocityConfigurationRegister _deltaThetaAndDeltaVelocityConfigurationRegister{
        vn::protocol::uart::IntegrationFrame::INTEGRATIONFRAME_BODY,       // IntegrationFrame
        vn::protocol::uart::CompensationMode::COMPENSATIONMODE_NONE,       // GyroCompensation
        vn::protocol::uart::AccCompensationMode::ACCCOMPENSATIONMODE_NONE, // AccelCompensation
        vn::protocol::uart::EarthRateCorrection::EARTHRATECORR_NONE        // EarthRateCorrection
    };

    // ###########################################################################################################
    //                                              GNSS SUBSYSTEM
    // ###########################################################################################################

    /// @brief  GNSS Configuration.
    /// @note See User manual VN-310 - 10.2.1 (p 124)
    vn::sensors::GpsConfigurationRegister _gpsConfigurationRegister{
        vn::protocol::uart::GpsMode::GPSMODE_ONBOARDGPS,       // Mode
        vn::protocol::uart::PpsSource::PPSSOURCE_GPSPPSRISING, // PpsSource
        vn::protocol::uart::GpsRate::GPSRATE_5HZ,              // Rate
        vn::protocol::uart::AntPower::ANTPOWER_INTERNAL        // AntPower
    };

    /// @brief GNSS Antenna A Offset.
    ///
    /// Configures the position offset of GNSS antenna A from the VN-310E in the vehicle reference frame.
    /// @note See User manual VN-310 - 10.2.2 (p 125)
    vn::math::vec3f _gpsAntennaOffset{
        0, 0, 0 // [m]
    };

    /// @brief GNSS Compass Baseline.
    ///
    /// Configures the position offset and measurement uncertainty of the second GNSS
    /// antenna relative to the first GNSS antenna in the vehicle reference frame.
    /// @note See User manual VN-310 - 10.2.3 (p 126f)
    vn::sensors::GpsCompassBaselineRegister _gpsCompassBaselineRegister{
        vn::math::vec3f{ 1.0F, 0.0F, 0.0F },      // Position [m]
        vn::math::vec3f{ 0.254F, 0.254F, 0.254F } // Uncertainty [m]
    };

    // ###########################################################################################################
    //                                            ATTITUDE SUBSYSTEM
    // ###########################################################################################################

    /// @brief VPE Basic Control.
    ///
    /// Provides control over various features relating to the onboard attitude filtering algorithm.
    /// @note See User manual VN-310 - 11.3.1 (p 158) / VN-100 - 7.3.1 (p 104)
    vn::sensors::VpeBasicControlRegister _vpeBasicControlRegister{
        vn::protocol::uart::VpeEnable::VPEENABLE_ENABLE,       // Enable
        vn::protocol::uart::HeadingMode::HEADINGMODE_RELATIVE, // HeadingMode
        vn::protocol::uart::VpeMode::VPEMODE_MODE1,            // FilteringMode
        vn::protocol::uart::VpeMode::VPEMODE_MODE1             // TuningMode
    };

    /// @brief VPE Magnetometer Basic Tuning.
    ///
    /// Provides basic control of the adaptive filtering and tuning for the magnetometer..
    /// @note See User manual VN-100 - 7.3.2 (p 105)
    vn::sensors::VpeMagnetometerBasicTuningRegister _vpeMagnetometerBasicTuningRegister{
        vn::math::vec3f{ 4.0F, 4.0F, 4.0F }, // BaseTuning [0 - 10]
        vn::math::vec3f{ 5.0F, 5.0F, 5.0F }, // AdaptiveTuning [0 - 10]
        vn::math::vec3f{ 5.5F, 5.5F, 5.5F }  // AdaptiveFiltering [0 - 10]
    };

    /// @brief VPE Accelerometer Basic Tuning.
    ///
    /// Provides basic control of the adaptive filtering and tuning for the accelerometer.
    /// @note See User manual VN-100 - 7.3.3 (p 106)
    vn::sensors::VpeAccelerometerBasicTuningRegister _vpeAccelerometerBasicTuningRegister{
        vn::math::vec3f{ 6.0F, 6.0F, 6.0F }, // BaseTuning [0 - 10]
        vn::math::vec3f{ 3.0F, 3.0F, 3.0F }, // AdaptiveTuning [0 - 10]
        vn::math::vec3f{ 5.0F, 5.0F, 5.0F }  // AdaptiveFiltering [0 - 10]
    };

    /// @brief VPE Gyro Basic Tuning.
    ///
    /// Provides basic control of the adaptive filtering and tuning for the gyro.
    /// @note See User manual VN-100 - 7.3.5 (p 108)
    vn::sensors::VpeGyroBasicTuningRegister _vpeGyroBasicTuningRegister{
        vn::math::vec3f{ 8.0F, 8.0F, 8.0F }, // VarianceAngularWalk [0 - 10]
        vn::math::vec3f{ 4.0F, 4.0F, 4.0F }, // BaseTuning [0 - 10]
        vn::math::vec3f{ 0.0F, 0.0F, 0.0F }  // AdaptiveTuning [0 - 10]
    };

    /// @brief Filter Startup Gyro Bias.
    ///
    /// The filter gyro bias estimate used at startup.
    /// @note See User manual VN-100 - 7.3.4 (p 107)
    vn::math::vec3f _filterStartupGyroBias{
        0, 0, 0 // [rad/s]
    };

    // ###########################################################################################################
    //                                               INS SUBSYSTEM
    // ###########################################################################################################

    /// @brief  INS Basic Configuration.
    /// @note See User manual VN-310 - 12.3.1 (p 166)
    vn::sensors::InsBasicConfigurationRegisterVn300 _insBasicConfigurationRegisterVn300{
        vn::protocol::uart::Scenario::SCENARIO_GPSMOVINGBASELINEDYNAMIC, // Scenario
        true,                                                            // AhrsAiding
        true                                                             // EstBaseline
    };

    /// @brief Startup Filter Bias Estimate.
    ///
    /// Sets the initial estimate for the filter bias states.
    /// @note See User manual VN-310 - 12.3.2 (p 167)
    vn::sensors::StartupFilterBiasEstimateRegister _startupFilterBiasEstimateRegister{
        vn::math::vec3f{ 0, 0, 0 }, // GyroBias [rad/s]
        vn::math::vec3f{ 0, 0, 0 }, // AccelBias [m/s^2]
        0.0F                        // PressureBiasIn [m]
    };

    // ###########################################################################################################
    //                                    HARD/SOFT IRON ESTIMATOR SUBSYSTEM
    // ###########################################################################################################

    /// @brief Magnetometer Calibration Control.
    ///
    /// Controls the magnetometer real-time calibration algorithm.
    /// @note See User manual VN-310 - 13.1.1 (p 169) / VN-100 - 8.1.1 (p 110)
    vn::sensors::MagnetometerCalibrationControlRegister _magnetometerCalibrationControlRegister{
        vn::protocol::uart::HsiMode::HSIMODE_RUN,            // HSIMode
        vn::protocol::uart::HsiOutput::HSIOUTPUT_USEONBOARD, // HSIOutput
        5                                                    // ConvergeRate
    };

    // ###########################################################################################################
    //                                      WORLD MAGNETIC & GRAVITY MODULE
    // ###########################################################################################################

    /// @brief Magnetic and Gravity Reference Vectors.
    ///
    /// Magnetic and gravity reference vectors.
    /// @note See User manual VN-310 - 14.1.1 (p 175) / VN-100 - 9.1.1 (p 115)
    vn::sensors::MagneticAndGravityReferenceVectorsRegister _magneticAndGravityReferenceVectorsRegister{
        vn::math::vec3f{ 1.0F, 0.0F, 1.8F },      // MagRef [Gauss]
        vn::math::vec3f{ 0.0F, 0.0F, -9.793746F } // AccRef [m/s^2]
    };

    /// @brief Reference Vector Configuration.
    ///
    /// Control register for both the onboard world magnetic and gravity model corrections.
    /// @note See User manual VN-310 - 14.1.2 (p 176) / VN-100 - 9.1.2 (p 116)
    vn::sensors::ReferenceVectorConfigurationRegister _referenceVectorConfigurationRegister{
        true,                      // UseMagModel
        true,                      // UseGravityModel
        1000,                      // RecalcThreshold [m]
        0.0F,                      // Year [years]
        vn::math::vec3d{ 0, 0, 0 } // Position (Lat Lon Alt [deg deg m])
    };

    // ###########################################################################################################
    //                                              VELOCITY AIDING
    // ###########################################################################################################

    /// @brief Velocity Compensation Control.
    ///
    /// Provides control over the velocity compensation feature for the attitude filter.
    /// @note See User manual VN-100 - 10.2.1 (p 123)
    vn::sensors::VelocityCompensationControlRegister _velocityCompensationControlRegister{
        vn::protocol::uart::VelocityCompensationMode::VELOCITYCOMPENSATIONMODE_BODYMEASUREMENT, // Mode
        0.1F,                                                                                   // VelocityTuning
        0.01F                                                                                   // RateTuning
    };

    // ###########################################################################################################
    //                                       Binary Group GUI Definitions
    // ###########################################################################################################

    /// @brief Needed data to display a binary group in the GUI
    struct BinaryGroupData
    {
        /// Name of the output
        const char* name = nullptr;
        /// Enum value of the output
        int flagsValue = 0;
        /// Function providing a tooltip
        void (*tooltip)() = nullptr;
        /// Function which checks if the ouput is enabled (e.g. for a sensorModel)
        bool (*isEnabled)(VectorNavModel sensorModel, const vn::sensors::BinaryOutputRegister& bor, uint32_t binaryField) =
            [](VectorNavModel /* sensorModel */, const vn::sensors::BinaryOutputRegister& /* bor */, uint32_t /* binaryField */) { return true; };
        /// Function to toggle other bits depending on the status
        void (*toggleFields)(vn::sensors::BinaryOutputRegister& bor, uint32_t& /* binaryField */) = nullptr;
    };

    /// @brief Binary group 1 contains a wide assortment of commonly used data required for most applications.
    ///
    /// All of the outputs found in group 1 are also present in the other groups. In this sense, group 1 is a subset of
    /// commonly used outputs from the other groups. This simplifies the configuration of binary output messages for
    /// applications that only require access to the commonly used data found in group 1. For these applications
    /// you can hard code the group field to 1, and not worry about implemented support for the other binary groups.
    /// Using group 1 for commonly used outputs also has the advantage of reducing the overall packet size, since
    /// the packet length is dependent upon the number of binary groups active.
    static const std::array<BinaryGroupData, 15> _binaryGroupCommon;

    /// @brief Binary group 2 provides all timing and event counter related outputs.
    ///
    /// Some of these outputs (such as the TimeGps, TimePps, and TimeUtc), require either that the internal GNSS to be
    /// enabled, or an external GNSS must be present.
    static const std::array<BinaryGroupData, 10> _binaryGroupTime;

    /// @brief Binary group 3 provides all outputs which are dependent upon the measurements collected from the
    ///        onboard IMU, or an external IMU (if enabled).
    static const std::array<BinaryGroupData, 11> _binaryGroupIMU;

    /// @brief Binary group 4 provides all outputs which are dependent upon the measurements collected from the primary
    ///        onboard, Binary group 7 from the secondary onboard GNSS, or external GNSS (if enabled).
    ///
    /// All data in this group is updated at the rate of the GNSS receiver (nominally 5Hz for the internal GNSS).
    ///
    /// @note If data is asynchronously sent from group 4/7 at a rate equal to the GNSS update rate, then packets
    ///       will be sent out when updated by the GNSS receiver. For all other rates, the output will be based
    ///       on the divisor selected and the internal IMU sampling rate.
    static const std::array<BinaryGroupData, 16> _binaryGroupGNSS;

    /// @brief Binary group 5 provides all estimated outputs which are dependent upon the estimated attitude solution.
    ///
    /// The attitude will be derived from either the AHRS or the INS, depending upon which filter is currently active and
    /// tracking. All of the fields in this group will only be valid if the AHRS/INS filter is currently enabled and tracking.
    static const std::array<BinaryGroupData, 9> _binaryGroupAttitude;

    /// @brief Binary group 6 provides all estimated outputs which are dependent upon the onboard INS state solution.
    ///
    /// All of the fields in this group will only be valid if the INS filter is currently enabled and tracking.
    static const std::array<BinaryGroupData, 11> _binaryGroupINS;

    friend class NAV::VectorNavFile;
};

} // namespace NAV