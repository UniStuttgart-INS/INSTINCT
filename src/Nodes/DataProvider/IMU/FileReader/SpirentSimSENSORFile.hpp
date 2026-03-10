// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file SpirentSimSENSORFile.hpp
/// @brief File Reader for Spirent SimSENSOR binary files
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2026-03-10

#pragma once

#include "Navigation/Time/InsTime.hpp"
#include "Nodes/DataProvider/IMU/Imu.hpp"
#include "Nodes/DataProvider/Protocol/FileReader.hpp"

#include "internal/gui/widgets/TimeEdit.hpp"

namespace NAV
{
/// File Reader for Spirent SimSENSOR binary files
class SpirentSimSENSORFile : public Imu, public FileReader
{
  public:
    /// @brief Default constructor
    SpirentSimSENSORFile();
    /// @brief Destructor
    ~SpirentSimSENSORFile() override;
    /// @brief Copy constructor
    SpirentSimSENSORFile(const SpirentSimSENSORFile&) = delete;
    /// @brief Move constructor
    SpirentSimSENSORFile(SpirentSimSENSORFile&&) = delete;
    /// @brief Copy assignment operator
    SpirentSimSENSORFile& operator=(const SpirentSimSENSORFile&) = delete;
    /// @brief Move assignment operator
    SpirentSimSENSORFile& operator=(SpirentSimSENSORFile&&) = delete;

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

    /// @brief Resets the node. Moves the read cursor to the start
    bool resetNode() override;

  private:
    constexpr static size_t OUTPUT_PORT_INDEX_IMU_OBS = 0; ///< @brief Flow (ImuObs)

    InsTime _firstEpoch;                                     ///< Epoch of the first message
    InsTime _lastEpoch;                                      ///< Epoch of the last message
    size_t _msgCount = 0;                                    ///< Message count
    double _dtMin = std::numeric_limits<double>::infinity(); ///< Min time between messages

    InsTime _startTime{ 2026, 1, 1, 10, 0, 0.0, GPST }; ///< Time the run was simulated
    gui::widgets::TimeEditFormat _startTimeFormat;      ///< Format for the start time

    static constexpr uint32_t MESSAGE_SIZE = 1472; ///< Maximum message size [bytes]

    /// Vehicle Identifier
    /// @note Implementation from EthernetShare.h v31.1
    struct Vehicle_id
    {
        /// Status types // NOLINTNEXTLINE
        enum Type : uint32_t
        {
            real_veh, ///< the normal case
            spoof_veh ///< for a spoof vehicle
        };

        uint32_t id; ///< Id
        Type type;   ///< Type
    };

    /// Status structure
    /// @note SimREMOTE Software User Manual for version 10.02.00, ch. 7.2.4, p. 7-33 (319)
    struct Status
    {
        /// Status types // NOLINTNEXTLINE
        enum Type : uint32_t
        {
            no_scenario_loaded = 0, ///< no_scenario_loaded
            loading = 1,            ///< loading
            ready = 2,              ///< ready
            arming = 3,             ///< arming
            armed = 4,              ///< armed
            running = 5,            ///< running
            unused = 6,             ///< unused
            ended = 7               ///< ended
        };
        /// Serial poll byte bits //NOLINTNEXTLINE
        enum Serial_poll_byte_bits
        {
            scenario_is_incomplete = 16, ///< scenario_is_incomplete
            incompatible_hardware = 32,  ///< incompatible_hardware
            srq = 64,                    ///< Not used, SimGEN software will never raise an SRQ
            spare = 128                  ///< spare
        };
        Type status;                    ///< Status
        int32_t simulation_update_rate; ///< Set to the SimGEN simulation iteration rate (range 4 to 100 ms). In milliseconds
        int32_t UDP_output_rate_ms;     ///< UDP output rate in milliseconds
        uint32_t serial_poll_byte;      ///< Upper 4 bits of least significant byte defined in Serial_poll_byte_bits, lower 4 bits will match status above
    };

    /// SimSENSOR stucture
    /// @note SimSENSOR Software User Manual 1-02, ch. 2.13.1, table 2-16, p. 2-27 (35) (deprecated)
    /// @note Implementation from EthernetShare.h v31.1
    struct SimSENSOR
    {
        /// Gyroscope measurements (noise included)
        struct Gyro
        {
            std::array<double, 3> delta_theta; ///< Delta theta [rad]
            std::array<double, 3> avg_rate;    ///< Averaged angular rate [rad/s]
        };

        /// Accelerometer measurements (noise included)
        struct Accelerometer
        {
            std::array<double, 3> delta_velocity;   ///< Delta velocity [m/s]
            std::array<double, 3> avg_acceleration; ///< Averaged acceleration [m/s^2]
        };
        /// Magnetometer measurements (Body frame axis)
        struct Magnetometer
        {
            std::array<double, 3> magnetic_flux_density_uT; ///<< Magnetic flux density [µT]
        };
        /// Compass measurements
        struct Compass
        {
            double magnetic_heading_rad; ///< Magnetic heading [rad]
            double true_heading_rad;     ///< True heading (yaw) [rad]
            double bank_rad;             ///< Bank angle (roll) [rad] (only used for three axis compass)
            double elevation_rad;        ///< Elevation angle (pitch) [rad] (only used for three axis compass)
        };
        /// Barometer measurements
        struct Barometer
        {
            double height_m;        ///< Barometric height [m]
            double height_rate_mps; ///< Barometric height rate [m/s]
        };
        /// True airspeed indicator
        struct True_airspeed_indicator
        {
            double true_airspeed_mps; ///< True airspeed [m/s]
        };

        Vehicle_id vehicle_id;     ///< Vehicle id
        uint32_t model_number;     ///< Model number
        uint32_t spare;            ///< Pad to 8-byte boundary
        double time_of_validity_s; ///< The actual time of validity of the data [s]

        Gyro gyro;                                       ///< Gyroscope measurements (noise included)
        Accelerometer accelerometer;                     ///< Accelerometer measurements (noise included)
        Magnetometer magnetometer;                       ///< Magnetometer measurements (Body frame axis)
        Compass compass;                                 ///< Compass measurements
        Barometer barometer;                             ///< Barometric measurements
        True_airspeed_indicator true_airspeed_indicator; ///< True airspeed indicator
    };

    /// Ethernet Message Structure
    /// @note Implementation from EthernetShare.h v31.1
    struct Message
    {
        /// Message Types // NOLINTNEXTLINE
        enum Type : uint32_t
        {
            status = 8,          ///< Status
            simsensor_data = 15, ///< SimSENSOR
        };
        Type type;              ///< Message type
        uint16_t version_major; ///< EthernetShare major version (31)
        uint16_t version_minor; ///< EthernetShare minor version ( 1)

        /// Indicates the time into the scenario the datagram was generated.
        /// This time will precede the time_of_validity as datagrams are generated two scenario time steps in advance.
        /// Units are in microseconds.
        uint64_t time_into_run;

        /// Indicates the time into the scenario at which the data in the datagram is valid.
        /// Units are in microseconds.
        uint64_t time_of_validity;

        union
        {
            Status status_info;
            SimSENSOR simsensor;
        } data;
    };

    static constexpr uint8_t ETHERNET_SHARE_VERSION_MAJOR = 31; ///< EthernetShare major version
    static constexpr uint8_t ETHERNET_SHARE_VERSION_MINOR = 1;  ///< EthernetShare minor version

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Polls data from the file
    /// @return The read observation
    [[nodiscard]] std::shared_ptr<const NodeData> pollData();

    /// @brief Determines the type of the file
    /// @return The File Type
    [[nodiscard]] FileType determineFileType() override;
};

} // namespace NAV
