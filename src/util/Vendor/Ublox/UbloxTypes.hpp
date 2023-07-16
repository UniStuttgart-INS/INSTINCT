// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file UbloxTypes.hpp
/// @brief Type Definitions for Ublox messages
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-05-19

#pragma once

#include <string>
#include <array>
#include <optional>
#include <bitset>
#include <vector>
#include <cstdint>
#include <fmt/ostream.h>

namespace NAV::vendor::ublox
{
/// @brief Error detection modes available
enum ErrorDetectionMode
{
    ERRORDETECTIONMODE_NONE,     ///< No error detection is used
    ERRORDETECTIONMODE_CHECKSUM, ///< 16-bit checksum is used
};

/// @brief Enumeration of the available asynchronous ASCII talker IDs.
enum NmeaTalkerID
{
    NMEA_TALKER_ID_OFF = 0, ///< Asynchronous output is turned off.
    NMEA_TALKER_ID_GP = 1,  ///< GPS, SBAS, QZSS
    NMEA_TALKER_ID_GL = 2,  ///< GLONASS
    NMEA_TALKER_ID_GA = 3,  ///< Galileo
    NMEA_TALKER_ID_GB = 4,  ///< BeiDou
    NMEA_TALKER_ID_GN = 5   ///< Any combination of GNSS
};

/// @brief NMEA Message Type.
enum NmeaMessageClass
{
    NMEA_MSG_CLASS_STANDARD = 0xF0, ///< Standard Messages
    NMEA_MSG_CLASS_PUBX = 0xF1      ///< PUBX Messages
};

/// @brief NMEA Standard Messages.
/// Class ID = 0xF0
enum NmeaStandardMessages
{
    NMEA_STANDARD_MSG_DTM = 0x0A, ///< Datum Reference
    NMEA_STANDARD_MSG_GBQ = 0x44, ///< Poll a standard message (if the current Talker ID is GB)
    NMEA_STANDARD_MSG_GBS = 0x09, ///< GNSS Satellite Fault Detection
    NMEA_STANDARD_MSG_GGA = 0x00, ///< Global positioning system fix data
    NMEA_STANDARD_MSG_GLL = 0x01, ///< Latitude and longitude, with time of position fix and status
    NMEA_STANDARD_MSG_GLQ = 0x43, ///< Poll a standard message (if the current Talker ID is GL)
    NMEA_STANDARD_MSG_GNQ = 0x42, ///< Poll a standard message (if the current Talker ID is GN)
    NMEA_STANDARD_MSG_GNS = 0x0D, ///< GNSS fix data
    NMEA_STANDARD_MSG_GPQ = 0x40, ///< Poll a standard message (if the current Talker ID is GP)
    NMEA_STANDARD_MSG_GRS = 0x06, ///< GNSS Range Residuals
    NMEA_STANDARD_MSG_GSA = 0x02, ///< GNSS DOP and Active Satellites
    NMEA_STANDARD_MSG_GST = 0x07, ///< GNSS Pseudo Range Error Statistics
    NMEA_STANDARD_MSG_GSV = 0x03, ///< GNSS Satellites in View
    NMEA_STANDARD_MSG_RMC = 0x04, ///< Recommended Minimum data
    NMEA_STANDARD_MSG_TXT = 0x41, ///< Text Transmission
    NMEA_STANDARD_MSG_VLW = 0x0F, ///< Dual ground/water distance
    NMEA_STANDARD_MSG_VTG = 0x05, ///< Course over ground and Ground speed
    NMEA_STANDARD_MSG_ZDA = 0x08  ///< Time and Date
};

/// @brief NMEA PUBX Messages.
/// Class ID = 0xF1
enum NmeaPubxMessages
{
    NMEA_PUBX_MSG_CONFIG = 0x41,   ///< Set Protocols and Baudrate
    NMEA_PUBX_MSG_POSITION = 0x00, ///< Lat/Long Position Data
    NMEA_PUBX_MSG_RATE = 0x40,     ///< Set NMEA message output rate
    NMEA_PUBX_MSG_SVSTATUS = 0x03, ///< Satellite Status
    NMEA_PUBX_MSG_TIME = 0x04      ///< Time of Day and Clock Information
};

/// @brief The available UBX Class IDs
enum UbxClass
{
    UBX_CLASS_NONE = 0x00,               ///< No Message Class specified
    UBX_CLASS_NAV = 0x01,                ///< Navigation Results Messages: Position, Speed, Time, Acceleration, Heading, DOP, SVs used
    UBX_CLASS_RXM = 0x02,                ///< Receiver Manager Messages: Satellite Status, RTC Status
    UBX_CLASS_INF = 0x04,                ///< Information Messages: Printf-Style Messages, with IDs such as Error, Warning, Notice
    UBX_CLASS_ACK = 0x05,                ///< Ack/Nak Messages: Acknowledge or Reject messages to UBX-CFG input messages
    UBX_CLASS_CFG = 0x06,                ///< Configuration Input Messages: Configure the receiver
    UBX_CLASS_UPD = 0x09,                ///< Firmware Update Messages: Memory/Flash erase/write, Reboot, Flash identification, etc.
    UBX_CLASS_MON = 0x0A,                ///< Monitoring Messages: Communication Status, CPU Load, Stack Usage, Task Status
    UBX_CLASS_AID [[deprecated]] = 0x0B, ///< AssistNow Aiding Messages: Ephemeris, Almanac, other A-GPS data input
    UBX_CLASS_TIM = 0x0D,                ///< Timing Messages: Time Pulse Output, Time Mark Results
    UBX_CLASS_ESF = 0x10,                ///< External Sensor Fusion Messages: External Sensor Measurements and Status Information
    UBX_CLASS_MGA = 0x13,                ///< Multiple GNSS Assistance Messages: Assistance data for various GNSS
    UBX_CLASS_LOG = 0x21,                ///< Logging Messages: Log creation, deletion, info and retrieval
    UBX_CLASS_SEC = 0x27,                ///< Security Feature Messages
    UBX_CLASS_HNR = 0x28                 ///< High Rate Navigation Results Messages: High rate time, position, speed, heading
};

/// @brief The available ACK Messages
enum UbxAckMessages
{
    /// Message Acknowledged (Length = 2; Type = Output)
    UBX_ACK_ACK = 0x01,
    /// Message Not-Acknowledged (Length = 2; Type = Output)
    UBX_ACK_NAK = 0x00
};

/// @brief Message Acknowledged
///
/// Output upon processing of an input message. ACK Message is sent as soon as possible but at least within one second.
struct UbxAckAck
{
    uint8_t clsID = 0; ///< Class ID of the Acknowledged Message
    uint8_t msgID = 0; ///< Message ID of the Acknowledged Message
};

/// @brief Message Not-Acknowledged
///
/// Output upon processing of an input message. NAK Message is sent as soon as possible but at least within one second.
struct UbxAckNak
{
    uint8_t clsID = 0; ///< Class ID of the Not-Acknowledged Message
    uint8_t msgID = 0; ///< Message ID of the Not-Acknowledged Message
};

/// @brief The available CFG Messages
enum UbxCfgMessages
{
    /// Antenna Control Settings (Length = 4; Type = Get/Set)
    UBX_CFG_ANT = 0x13,
    /// Get/Set data batching configuration (Length = 8; Type = Get/Set)
    UBX_CFG_BATCH = 0x93,
    /// Clear, Save and Load configurations (Length = (12) or (13); Type = Command)
    UBX_CFG_CFG = 0x09,
    /// - Set User-defined Datum. (Length = 44; Type = Set)
    /// - The currently defined Datum (Length = 52; Type = Get)
    UBX_CFG_DAT = 0x06,
    /// DGNSS configuration (Length = 4; Type = Get/Set)
    UBX_CFG_DGNSS = 0x70,
    /// Disciplined oscillator configuration (Length = 4 + 32*numOsc; Type = Get/Set)
    UBX_CFG_DOSC = 0x61,
    /// External synchronization source configuration (Length = 4 + 36*numSources; Type = Get/Set)
    UBX_CFG_ESRC = 0x60,
    /// Geofencing configuration (Length = 8 + 12*numFences; Type = Get/Set)
    UBX_CFG_GEOFENCE = 0x69,
    /// GNSS system configuration (Length = 4 + 8*numConfigBlocks; Type = Get/Set)
    UBX_CFG_GNSS = 0x3E,
    /// High Navigation Rate Settings (Length = 4; Type = Get/Set)
    UBX_CFG_HNR = 0x5C,
    /// - Poll configuration for one protocol (Length = 1; Type = Poll Request)
    /// - Information message configuration (Length = 0 + 10*N; Type = Get/Set)
    UBX_CFG_INF = 0x02,
    /// Jamming/Interference Monitor configuration (Length = 8; Type = Get/Set)
    UBX_CFG_ITFM = 0x39,
    /// Data Logger Configuration (Length = 12; Type = Get/Set)
    UBX_CFG_LOGFILTER = 0x47,
    /// - Poll a message configuration (Length = 2; Type = Poll Request)
    /// - Set Message Rate(s) (Length = 8; Type = Get/Set)
    /// - Set Message Rate (Length = 3; Type = Get/Set)
    UBX_CFG_MSG = 0x01,
    /// Navigation Engine Settings (Length = 36; Type = Get/Set)
    UBX_CFG_NAV5 = 0x24,
    /// - Navigation Engine Expert Settings (Length = 40; Type = Get/Set)
    /// - Navigation Engine Expert Settings (Length = 40; Type = Get/Set)
    /// - Navigation Engine Expert Settings (Length = 44; Type = Get/Set)
    UBX_CFG_NAVX5 = 0x23,
    /// - NMEA protocol configuration (deprecated) (Length = 4; Type = Get/Set)
    /// - NMEA protocol configuration V0 (deprecated) (Length = 12; Type = Get/Set)
    /// - Extended NMEA protocol configuration V1 (Length = 20; Type = Get/Set)
    UBX_CFG_NMEA = 0x17,
    /// Odometer, Low-speed COG Engine Settings (Length = 20; Type = Get/Set)
    UBX_CFG_ODO = 0x1E,
    /// - Extended Power Management configuration (Length = 44; Type = Get/Set)
    /// - Extended Power Management configuration (Length = 48; Type = Get/Set)
    /// - Extended Power Management configuration (Length = 48; Type = Get/Set)
    UBX_CFG_PM2 = 0x3B,
    /// Power Mode Setup (Length = 8; Type = Get/Set)
    UBX_CFG_PMS = 0x86,
    /// - Polls the configuration for one I/O Port (Length = 1; Type = Poll Request)
    /// - Port Configuration for UART (Length = 20; Type = Get/Set)
    /// - Port Configuration for USB Port (Length = 20; Type = Get/Set)
    /// - Port Configuration for SPI Port (Length = 20; Type = Get/Set)
    /// - Port Configuration for DDC Port (Length = 20; Type = Get/Set)
    UBX_CFG_PRT = 0x00,
    /// Put receiver in a defined power state (Length = 8; Type = Set)
    UBX_CFG_PWR = 0x57,
    /// Navigation/Measurement Rate Settings (Length = 6; Type = Get/Set)
    UBX_CFG_RATE = 0x08,
    /// Contents of Remote Inventory (Length = 1 + 1*N; Type = Get/Set)
    UBX_CFG_RINV = 0x34,
    /// Reset Receiver / Clear Backup Data Structures (Length = 4; Type = Command)
    UBX_CFG_RST = 0x04,
    /// - RXM configuration (Length = 2; Type = Get/Set)
    /// - RXM configuration (Length = 2; Type = Get/Set)
    UBX_CFG_RXM = 0x11,
    /// SBAS Configuration (Length = 8; Type = Get/Set)
    UBX_CFG_SBAS = 0x16,
    /// SLAS Configuration (Length = 4; Type = Get/Set)
    UBX_CFG_SLAS = 0x8D,
    /// Synchronization manager configuration (Length = 20; Type = Get/Set)
    UBX_CFG_SMGR = 0x62,
    /// Time Mode Settings 2 (Length = 28; Type = Get/Set)
    UBX_CFG_TMODE2 = 0x3D,
    /// Time Mode Settings 3 (Length = 40; Type = Get/Set)
    UBX_CFG_TMODE3 = 0x71,
    /// - Poll Time Pulse Parameters for Time Pulse 0 (Length = 0; Type = Poll Request)
    /// - Poll Time Pulse Parameters (Length = 1; Type = Poll Request)
    /// - Time Pulse Parameters (Length = 32; Type = Get/Set)
    /// - Time Pulse Parameters (Length = 32; Type = Get/Se
    UBX_CFG_TP5 = 0x31,
    /// TX buffer time slots configuration (Length = 16; Type = Set)
    UBX_CFG_TXSLOT = 0x53,
    /// USB Configuration (Length = 108; Type = Get/Set)
    UBX_CFG_USB = 0x1B,
};

/// @brief The available ESF Messages
enum UbxEsfMessages
{
    /// Vehicle dynamics information (Length = 36; Type = Periodic/Polled)
    UBX_ESF_INS = 0x15,
    /// External Sensor Fusion Measurements (Length = (8 + 4*numMeas) or (12 + 4*numMeas); Type = Input/Output)
    UBX_ESF_MEAS = 0x02,
    /// Raw sensor measurements (Length = 4 + 8*N; Type = Output)
    UBX_ESF_RAW = 0x03,
    /// External Sensor Fusion (ESF) status information (Length = 16 + 4*numSens; Type = Periodic/Polled)
    UBX_ESF_STATUS = 0x10,
};

/// @brief Vehicle dynamics information
///
/// This message outputs information about the vehicle dynamics.
/// For ADR products (in protocol versions less than 19.2), the output dynamics
/// information (angular rates and accelerations) is expressed with respect to the
/// vehicle-frame. More information can be found in the ADR Navigation Output
/// section.
/// For ADR products, the output dynamics information (angular rates and
/// accelerations) is expressed with respect to the vehicle-frame. More information
/// can be found in the ADR Navigation Output section.
/// For UDR products, the output dynamics information (angular rates and
/// accelerations) are expressed with respect to the body-frame. More information
/// can be found in the UDR Navigation Output section.
struct UbxEsfIns
{
    std::bitset<4UL * 8UL> bitfield0;   ///< Bitfield (zAccelValid, yAccelValid, xAccelValid, zAngRateValid, yAngRateValid, xAngRateValid, version)
    std::array<uint8_t, 4> reserved1{}; ///< Reserved
    uint32_t iTOW = 0;                  ///< GPS time of week of the navigation epoch. See the description of iTOW for details. [ms]
    int32_t xAngRate = 0;               ///< Compensated x-axis angular rate. [deg/s * 1e-3]
    int32_t yAngRate = 0;               ///< Compensated y-axis angular rate. [deg/s * 1e-3]
    int32_t zAngRate = 0;               ///< Compensated z-axis angular rate. [deg/s * 1e-3]
    int32_t xAccel = 0;                 ///< Compensated x-axis acceleration (gravity-free). [m/s^2 * 1e-2]
    int32_t yAccel = 0;                 ///< Compensated y-axis acceleration (gravity-free). [m/s^2 * 1e-2]
    int32_t zAccel = 0;                 ///< Compensated z-axis acceleration (gravity-free). [m/s^2 * 1e-2]
};

/// @brief External Sensor Fusion Measurements
///
/// Possible data types for the data field are described in the ESF Measurement Data section.
struct UbxEsfMeas
{
    uint32_t timeTag = 0;                           ///< Time tag of measurement generated by external sensor
    std::bitset<2UL * 8UL> flags;                   ///< Flags. Set all unused bits to zero (timeMarkSent, timeMarkEdge, calibTtagValid, numMeas)
    uint16_t id = 0;                                ///< Identification number of data provider
    std::vector<uint32_t> data;                     ///< data (see graphic below)
    std::optional<std::vector<uint32_t>> calibTtag; ///< Receiver local time calibrated. This field must not be supplied when calibTtagValid is set to 0. [ms]
};

/// @brief Raw sensor measurements
///
/// The message contains measurements from the active inertial sensors
/// connected to the GNSS chip. Possible data types for the data field are
/// accelerometer, gyroscope and temperature readings as described in the ESF
/// Measurement Data section.
/// Note that the rate selected in UBX-CFG-MSG is not respected. If a positive rate is
/// selected then all raw measurements will be output.
/// See also Raw Sensor Measurement Data.
struct UbxEsfRaw
{
    /// @brief Repeated data in this message
    struct UbxEsfRawData
    {
        uint32_t data = 0;  ///< data. Same as in UBX-ESF-MEAS (see graphic below)
        uint32_t sTtag = 0; ///< sensor time tag
    };

    std::array<uint8_t, 4> reserved1{}; ///< Reserved
    std::vector<UbxEsfRawData> data;    ///< Repeated block
};

/// @brief  External Sensor Fusion (ESF) status information
struct UbxEsfStatus
{
    /// @brief Repeated data in this message
    struct UbxEsfStatusSensor
    {
        std::bitset<1UL * 8UL> sensStatus1; ///< Sensor status, part 1 (seegraphic below)
        std::bitset<1UL * 8UL> sensStatus2; ///< Sensor status, part 2 (seegraphic below)
        uint8_t freq = 0;                   ///< Observation frequency [Hz]
        std::bitset<1UL * 8UL> faults;      ///< Sensor faults (see graphic below)
    };

    uint32_t iTOW = 0;                  ///< GPS time of week of the navigation epoch. See the description of iTOW for details. [ms]
    uint8_t version = 0;                ///< Message version (2 for this version)
    std::array<uint8_t, 7> reserved1{}; ///< Reserved
    /// Fusion mode:0: Initialization mode: receiver is initializing some unknown values required for doing sensor fusion
    /// 1: Fusion mode: GNSS and sensor data are used for navigation solution computation
    /// 2: Suspended fusion mode: sensor fusion is temporarily disabled due to e.g. invalid sensor data or detected ferry
    /// 3: Disabled fusion mode: sensor fusion is permanently disabled until receiver reset due e.g. to sensor error More details can be found in the Fusion Modes section.
    uint8_t fusionMode = 0;
    std::array<uint8_t, 2> reserved2{};      ///< Reserved
    uint8_t numSens = 0;                     ///< Number of sensors
    std::vector<UbxEsfStatusSensor> sensors; ///< Repeated block
};

/// @brief The available HNR Messages
enum UbxHnrMessages
{
    /// Vehicle dynamics information (Length = 36; Type = Periodic/Polled)
    UBX_HNR_INS = 0x02,
    /// High Rate Output of PVT Solution (Length = 72; Type = Periodic/Polled)
    UBX_HNR_PVT = 0x00,
};

/// @brief The available INF Messages
enum UbxInfMessages
{
    /// ASCII output with debug contents (Length = 0 + 1*N; Type = Output)
    UBX_INF_DEBUG = 0x04,
    /// ASCII output with error contents (Length = 0 + 1*N; Type = Output)
    UBX_INF_ERROR = 0x00,
    /// ASCII output with informational contents (Length = 0 + 1*N; Type = Output)
    UBX_INF_NOTICE = 0x02,
    /// ASCII output with test contents (Length = 0 + 1*N; Type = Output)
    UBX_INF_TEST = 0x03,
    /// ASCII output with warning contents (Length = 0 + 1*N; Type = Output)
    UBX_INF_WARNING = 0x01,
};

/// @brief The available LOG Messages
enum UbxLogMessages
{
    /// Batched data (Length = 100; Type = Polled)
    UBX_LOG_BATCH = 0x11,
    /// Create Log File (Length = 8; Type = Command)
    UBX_LOG_CREATE = 0x07,
    /// Erase Logged Data (Length = 0; Type = Command)
    UBX_LOG_ERASE = 0x03,
    /// - Find index of a log entry based on a given time (Length = 12; Type = Input)
    /// - Response to FINDTIME request (Length = 8; Type = Output)
    UBX_LOG_FINDTIME = 0x0E,
    /// - Poll for log information (Length = 0; Type = Poll Request)
    /// - Log information (Length = 48; Type = Output)
    UBX_LOG_INFO = 0x08,
    /// Request batch data (Length = 4; Type = Command)
    UBX_LOG_RETRIEVEBATCH = 0x10,
    /// Odometer log entry (Length = 32; Type = Output)
    UBX_LOG_RETRIEVEPOSEXTRA = 0x0F,
    /// Position fix log entry (Length = 40; Type = Output)
    UBX_LOG_RETRIEVEPOS = 0x0B,
    /// Byte string log entry (Length = 16 + 1*byteCount; Type = )
    UBX_LOG_RETRIEVESTRING = 0x0D,
    /// Request log data (Length = 12; Type = Command)
    UBX_LOG_RETRIEVE = 0x09,
    /// Store arbitrary string in on-board flash (Length = 0 + 1*N; Type = Command)
    UBX_LOG_STRING = 0x04,
};

/// @brief The available MGA Messages
enum UbxMgaMessages
{
    /// Multiple GNSS Acknowledge message (Length = 8; Type = Output)
    UBX_MGA_ACK_DATA0 = 0x60,
    /// Multiple GNSS AssistNow Offline Assistance (Length = 76; Type = Input)
    UBX_MGA_ANO = 0x20,
    /// BDS Ephemeris Assistance (Length = 88; Type = Input)
    UBX_MGA_BDS_EPH = 0x03,
    /// BDS Almanac Assistance (Length = 40; Type = Input)
    UBX_MGA_BDS_ALM = 0x03,
    /// BDS Health Assistance (Length = 68; Type = Input)
    UBX_MGA_BDS_HEALTH = 0x03,
    /// BDS UTC Assistance (Length = 20; Type = Input)
    UBX_MGA_BDS_UTC = 0x03,
    /// BDS Ionospheric Assistance (Length = 16; Type = Input)
    UBX_MGA_BDS_IONO = 0x03,
    /// - Poll the Navigation Database (Length = 0; Type = Poll Request)
    /// - Navigation Database Dump Entry (Length = 12 + 1*N; Type = Input/Output)
    UBX_MGA_DBD = 0x80,
    /// Transfer MGA-ANO data block to flash (Length = 6 + 1*size; Type = Input)
    UBX_MGA_FLASH_DATA = 0x21,
    /// Finish flashing MGA-ANO data (Length = 2; Type = Input)
    UBX_MGA_FLASH_STOP = 0x21,
    /// Acknowledge last FLASH-DATA or -STOP (Length = 6; Type = Output)
    UBX_MGA_FLASH_ACK = 0x21,
    /// Galileo Ephemeris Assistance (Length = 76; Type = Input)
    UBX_MGA_GAL_EPH = 0x02,
    /// Galileo Almanac Assistance (Length = 32; Type = Input)
    UBX_MGA_GAL_ALM = 0x02,
    /// Galileo GPS time offset assistance (Length = 12; Type = Input)
    UBX_MGA_GAL_TIMEOFFSET = 0x02,
    /// Galileo UTC Assistance (Length = 20; Type = Input)
    UBX_MGA_GAL_UTC = 0x02,
    /// GLONASS Ephemeris Assistance (Length = 48; Type = Input)
    UBX_MGA_GLO_EPH = 0x06,
    /// GLONASS Almanac Assistance (Length = 36; Type = Input)
    UBX_MGA_GLO_ALM = 0x06,
    /// GLONASS Auxiliary Time Offset Assistance (Length = 20; Type = Input)
    UBX_MGA_GLO_TIMEOFFSET = 0x06,
    /// GPS Ephemeris Assistance (Length = 68; Type = Input)
    UBX_MGA_GPS_EPH = 0x00,
    /// GPS Almanac Assistance (Length = 36; Type = Input)
    UBX_MGA_GPS_ALM = 0x00,
    /// GPS Health Assistance (Length = 40; Type = Input)
    UBX_MGA_GPS_HEALTH = 0x00,
    /// GPS UTC Assistance (Length = 20; Type = Input)
    UBX_MGA_GPS_UTC = 0x00,
    /// GPS Ionosphere Assistance (Length = 16; Type = Input)
    UBX_MGA_GPS_IONO = 0x00,
    /// Initial Position Assistance (Length = 20; Type = Input)
    UBX_MGA_INI_POS_XYZ = 0x40,
    /// Initial Position Assistance (Length = 20; Type = Input)
    UBX_MGA_INI_POS_LLH = 0x40,
    /// Initial Time Assistance (Length = 24; Type = Input)
    UBX_MGA_INI_TIME_UTC = 0x40,
    /// Initial Time Assistance (Length = 24; Type = Input)
    UBX_MGA_INI_TIME_GNSS = 0x40,
    /// Initial Clock Drift Assistance (Length = 12; Type = Input)
    UBX_MGA_INI_CLKD = 0x40,
    /// Initial Frequency Assistance (Length = 12; Type = Input)
    UBX_MGA_INI_FREQ = 0x40,
    /// Earth Orientation Parameters Assistance (Length = 72; Type = Input)
    UBX_MGA_INI_EOP = 0x40,
    /// QZSS Ephemeris Assistance (Length = 68; Type = Input)
    UBX_MGA_QZSS_EPH = 0x05,
    /// QZSS Almanac Assistance (Length = 36; Type = Input)
    UBX_MGA_QZSS_ALM = 0x05,
    /// QZSS Health Assistance (Length = 12; Type = Input)
    UBX_MGA_QZSS_HEALTH = 0x05,
};

/// @brief The available MON Messages
enum UbxMonMessages
{
    /// Data batching buffer status (Length = 12; Type = Polled)
    UBX_MON_BATCH = 0x32,
    /// Information message major GNSS selection (Length = 8; Type = Polled)
    UBX_MON_GNSS = 0x28,
    /// Extended Hardware Status (Length = 28; Type = Periodic/Polled)
    UBX_MON_HW2 = 0x0B,
    /// Hardware Status (Length = 60; Type = Periodic/Polled)
    UBX_MON_HW = 0x09,
    /// I/O Subsystem Status (Length = 0 + 20*N; Type = Periodic/Polled)
    UBX_MON_IO = 0x02,
    /// Message Parse and Process Status (Length = 120; Type = Periodic/Polled)
    UBX_MON_MSGPP = 0x06,
    /// - Poll Request for installed patches (Length = 0; Type = Poll Request)
    /// - Output information about installed patches (Length = 4 + 16*nEntries; Type = Polled)
    UBX_MON_PATCH = 0x27,
    /// Receiver Buffer Status (Length = 24; Type = Periodic/Polled)
    UBX_MON_RXBUFF = 0x07,
    /// Receiver Status Information (Length = 1; Type = Output)
    UBX_MON_RXR = 0x21,
    /// Synchronization Manager Status (Length = 16; Type = Periodic/Polled)
    UBX_MON_SMGR = 0x2E,
    /// Transmitter Buffer Status (Length = 28; Type = Periodic/Polled)
    UBX_MON_TXBUFF = 0x08,
    /// - Poll Receiver/Software Version (Length = 0; Type = Poll Request)
    /// - Receiver/Software Version (Length = 40 + 30*N; Type = Polled)
    UBX_MON_VER = 0x04,
};

/// @brief The available NAV Messages
enum UbxNavMessages
{
    /// AssistNow Autonomous Status (Length = 16; Type = Periodic/Polled)
    UBX_NAV_AOPSTATUS = 0x60,
    /// Attitude Solution (Length = 32; Type = Periodic/Polled)
    UBX_NAV_ATT = 0x05,
    /// Clock Solution (Length = 20; Type = Periodic/Polled)
    UBX_NAV_CLOCK = 0x22,
    /// DGPS Data Used for NAV (Length = 16 + 12*numCh; Type = Periodic/Polled)
    UBX_NAV_DGPS = 0x31,
    /// Dilution of precision (Length = 18; Type = Periodic/Polled)
    UBX_NAV_DOP = 0x04,
    /// End Of Epoch (Length = 4; Type = Periodic)
    UBX_NAV_EOE = 0x61,
    /// Geofencing status (Length = 8 + 2*numFences; Type = Periodic/Polled)
    UBX_NAV_GEOFENCE = 0x39,
    /// High Precision Position Solution in ECEF (Length = 28; Type = Periodic/Polled)
    UBX_NAV_HPPOSECEF = 0x13,
    /// High Precision Geodetic Position Solution (Length = 36; Type = Periodic/Polled)
    UBX_NAV_HPPOSLLH = 0x14,
    /// Odometer Solution (Length = 20; Type = Periodic/Polled)
    UBX_NAV_ODO = 0x09,
    /// GNSS Orbit Database Info (Length = 8 + 6*numSv; Type = Periodic/Polled)
    UBX_NAV_ORB = 0x34,
    /// Position Solution in ECEF (Length = 20; Type = Periodic/Polled)
    UBX_NAV_POSECEF = 0x01,
    /// Geodetic Position Solution (Length = 28; Type = Periodic/Polled)
    UBX_NAV_POSLLH = 0x02,
    /// Navigation Position Velocity Time Solution (Length = 92; Type = Periodic/Polled)
    UBX_NAV_PVT = 0x07,
    /// Relative Positioning Information in NED frame (Length = 40; Type = Periodic/Polled)
    UBX_NAV_RELPOSNED = 0x3C,
    /// Reset odometer (Length = 0; Type = Command)
    UBX_NAV_RESETODO = 0x10,
    /// Satellite Information (Length = 8 + 12*numSvs; Type = Periodic/Polled)
    UBX_NAV_SAT = 0x35,
    /// SBAS Status Data (Length = 12 + 12*cnt; Type = Periodic/Polled)
    UBX_NAV_SBAS = 0x32,
    /// QZSS L1S SLAS Status Data (Length = 20 + 8*cnt; Type = Periodic/Polled)
    UBX_NAV_SLAS = 0x42,
    /// Navigation Solution Information (Length = 52; Type = Periodic/Polled)
    UBX_NAV_SOL = 0x06,
    /// Receiver Navigation Status (Length = 16; Type = Periodic/Polled)
    UBX_NAV_STATUS = 0x03,
    /// Space Vehicle Information (Length = 8 + 12*numCh; Type = Periodic/Polled)
    UBX_NAV_SVINFO = 0x30,
    /// Survey-in data (Length = 40; Type = Periodic/Polled)
    UBX_NAV_SVIN = 0x3B,
    /// BDS Time Solution (Length = 20; Type = Periodic/Polled)
    UBX_NAV_TIMEBDS = 0x24,
    /// Galileo Time Solution (Length = 20; Type = Periodic/Polled)
    UBX_NAV_TIMEGAL = 0x25,
    /// GLO Time Solution (Length = 20; Type = Periodic/Polled)
    UBX_NAV_TIMEGLO = 0x23,
    /// GPS Time Solution (Length = 16; Type = Periodic/Polled)
    UBX_NAV_TIMEGPS = 0x20,
    /// Leap second event information (Length = 24; Type = Periodic/Polled)
    UBX_NAV_TIMELS = 0x26,
    /// UTC Time Solution (Length = 20; Type = Periodic/Polled)
    UBX_NAV_TIMEUTC = 0x21,
    /// Velocity Solution in ECEF (Length = 20; Type = Periodic/Polled)
    UBX_NAV_VELECEF = 0x11,
    /// Velocity Solution in NED (Length = 36; Type = Periodic/Polled)
    UBX_NAV_VELNED = 0x12,
};

/// @brief Attitude Solution
///
/// This message outputs the attitude solution as roll, pitch and heading angles.
/// More details about vehicle attitude can be found in the Vehicle Attitude Output
/// (ADR) section for ADR products.
/// More details about vehicle attitude can be found in the Vehicle Attitude Output
/// (UDR) section for UDR products.
struct UbxNavAtt
{
    uint32_t iTOW = 0;                  ///< GPS time of week of the navigation epoch [ms]. See the description of iTOW for details.
    uint8_t version = 0;                ///< Message version (0 for this version)
    std::array<uint8_t, 3> reserved1{}; ///< Reserved
    int32_t roll = 0;                   ///< Vehicle roll [deg * 1e-5]
    int32_t pitch = 0;                  ///< Vehicle pitch [deg * 1e-5]
    int32_t heading = 0;                ///< Vehicle heading [deg * 1e-5]
    uint32_t accRoll = 0;               ///< Vehicle roll accuracy [deg * 1e-5] (if null, roll angle is not available).
    uint32_t accPitch = 0;              ///< Vehicle pitch accuracy [deg * 1e-5] (if null, pitch angle is not available).
    uint32_t accHeading = 0;            ///< Vehicle heading accuracy [deg * 1e-5] (if null, heading angle is not available).
};

/// @brief Position Solution in ECEF
///
/// See important comments concerning validity of position given in section
/// Navigation Output Filters.
struct UbxNavPosecef
{
    uint32_t iTOW = 0; ///< GPS time of week of the navigation epoch [ms]. See the description of iTOW for details.
    int32_t ecefX = 0; ///< ECEF X coordinate [cm]
    int32_t ecefY = 0; ///< ECEF Y coordinate [cm]
    int32_t ecefZ = 0; ///< ECEF Z coordinate [cm]
    uint32_t pAcc = 0; ///< Position Accuracy Estimate [cm]
};

/// @brief Geodetic Position Solution
///
/// See important comments concerning validity of position given in section
/// Navigation Output Filters.
/// This message outputs the Geodetic position in the currently selected ellipsoid.
/// The default is the WGS84 Ellipsoid, but can be changed with the message UBX-CFG-DAT.
struct UbxNavPosllh
{
    uint32_t iTOW = 0;  ///< GPS time of week of the navigation epoch [ms]. See the description of iTOW for details.
    int32_t lon = 0;    ///< Longitude [deg * 1e-7]
    int32_t lat = 0;    ///< Latitude [deg * 1e-7]
    int32_t height = 0; ///< Height above ellipsoid [mm]
    int32_t hMSL = 0;   ///< Height above mean sea level [mm]
    uint32_t hAcc = 0;  ///< Horizontal accuracy estimate [mm]
    uint32_t vAcc = 0;  ///< Vertical accuracy estimate [mm]
};

/// @brief Velocity Solution in NED
///
/// See important comments concerning validity of position given in section Navigation Output Filters.
struct UbxNavVelned
{
    uint32_t iTOW = 0;   ///< GPS time of week of the navigation epoch [ms]. See the description of iTOW for details.
    int32_t velN = 0;    ///< North velocity component [cm/s]
    int32_t velE = 0;    ///< East velocity component [cm/s]
    int32_t velD = 0;    ///< Down velocity component [cm/s]
    uint32_t speed = 0;  ///< Speed (3-D)
    uint32_t gSpeed = 0; ///< Ground speed (2-D)
    int32_t heading = 0; ///< Heading of motion 2-D [deg * 1e-5]
    uint32_t sAcc = 0;   ///< Speed accuracy Estimation [cm/s]
    uint32_t cAcc = 0;   ///< Course / Heading accuracy estimation [deg * 1e-5]
};

/// @brief The available RXM Messages
enum UbxRxmMessages
{
    /// Indoor Messaging System Information (Length = 4 + 44*numTx; Type = Periodic/Polled)
    UBX_RXM_IMES = 0x61,
    /// - Satellite Measurements for RRLP (Length = 44 + 24*numSV; Type = Periodic/Polled)
    UBX_RXM_MEASX = 0x14,
    /// - Requests a Power Management task (Length = 8; Type = Command)
    /// - Requests a Power Management task (Length = 16; Type = Command)
    UBX_RXM_PMREQ = 0x41,
    /// - Multi-GNSS Raw Measurement Data (Length = 16 + 32*numMeas; Type = Periodic/Polled)
    /// - Multi-GNSS Raw Measurement Data (Length = 16 + 32*numMeas; Type = Periodic/Polled)
    UBX_RXM_RAWX = 0x15,
    /// - Galileo SAR Short-RLM report (Length = 16; Type = Output)
    /// - Galileo SAR Long-RLM report (Length = 28; Type = Output)
    UBX_RXM_RLM = 0x59,
    /// RTCM input status (Length = 8; Type = Output)
    UBX_RXM_RTCM = 0x32,
    /// - Broadcast Navigation Data Subframe (Length = 8 + 4*numWords; Type = Output)
    /// - Broadcast Navigation Data Subframe (Length = 8 + 4*numWords; Type = Output)
    UBX_RXM_SFRBX = 0x13,
    /// SV Status Info (Length = 8 + 6*numSV; Type = Periodic/Polled)
    UBX_RXM_SVSI = 0x20,
};

/// @brief Multi-GNSS Raw Measurement Data
///
/// This message contains the information needed to be able to generate a RINEX 3 multi-GNSS observation file.
/// This message contains pseudorange, Doppler, carrier phase, phase lock and
/// signal quality information for GNSS satellites once signals have been
/// synchronized. This message supports all active GNSS.
struct UbxRxmRawx
{
    /// @brief Repeated data in this message
    struct UbxRxmRawxData
    {
        /// Pseudorange measurement [m].
        /// GLONASS inter frequency channel delays are compensated with an internal calibration table.
        double prMes = 0.0;
        /// Carrier phase measurement [cycles].
        /// The carrier phase initial ambiguity is initialized using an approximate
        /// value to make the magnitude of the phase close to the pseudorange measurement.
        /// Clock resets are applied to both phase and code measurements in accordance with the RINEX specification.
        double cpMes = 0.0;
        float doMes = 0.0F;             ///< Doppler measurement (positive sign for approaching satellites) [Hz]
        uint8_t gnssId = 0;             ///< GNSS identifier (see Satellite Numbering for a list of identifiers)
        uint8_t svId = 0;               ///< Satellite identifier (see Satellite Numbering)
        uint8_t sigId = 0;              ///< New style signal identifier (see Signal Identifiers).(not supported in protocol versions less than 27)
        uint8_t freqId = 0;             ///< Only used for GLONASS: This is the frequency slot + 7 (range from 0 to 13)
        uint16_t locktime = 0;          ///< Carrier phase locktime counter [ms] (maximum 64500ms)
        uint8_t cno = 0;                ///< Carrier-to-noise density ratio (signal strength) [dB-Hz]
        std::bitset<1UL * 8UL> prStdev; ///< Estimated pseudorange measurement standard deviation [m * 0.01*2^n] (see graphic below)
        std::bitset<1UL * 8UL> cpStdev; ///< Estimated carrier phase measurement standard deviation [cycles * 0.004] (note a raw value of 0x0F indicates the value is invalid) (see graphic below)
        std::bitset<1UL * 8UL> doStdev; ///< Estimated Doppler measurement standard deviation. [Hz * 0.002*2^n] (see graphic below)
        std::bitset<1UL * 8UL> trkStat; ///< Tracking status bitfield (see graphic below)
        uint8_t reserved2 = 0;          ///< Reserved
    };

    /// Measurement time of week in receiverblocal time approximately aligned to the GPS time system.
    /// The receiver local time of week, week number and leap second information can be used to translate the
    /// time to other time systems. More information about the difference in time systems can be found in RINEX 3
    /// documentation. For a receiver operating in GLONASS only mode, UTC time can be determined by subtracting the leapS field
    /// from GPS time regardless of whether the GPS leap seconds are valid. [s]
    double rcvTow = 0.0;
    uint16_t week = 0; ///< GPS week number in receiver local time [weeks]
    /// GPS leap seconds (GPS-UTC). This field represents the receiver's best knowledge of the leap seconds offset.
    /// A flag is given in the recStat bitfield to indicate if the leap seconds are known.
    int8_t leapS = 0;
    uint8_t numMeas = 0;                ///< Number of measurements to follow
    std::bitset<1UL * 8UL> recStat;     ///< Receiver tracking status bitfield (see graphic below)
    uint8_t version = 0;                ///< Message version (0x01 for this version).
    std::array<uint8_t, 2> reserved1{}; ///< Reserved
    std::vector<UbxRxmRawxData> data;   ///< Repeated block
};

/// @brief Broadcast Navigation Data Subframe
///
/// This message reports a complete subframe of broadcast navigation data
/// decoded from a single signal. The number of data words reported in each
/// message depends on the nature of the signal.
/// See the section on Broadcast Navigation Data for further details.
struct UbxRxmSfrbx
{
    uint8_t gnssId = 0;         ///< GNSS identifier (see Satellite Numbering)
    uint8_t svId = 0;           ///< Satellite identifier (see Satellite Numbering)
    uint8_t reserved1 = 0;      ///< Reserved
    uint8_t freqId = 0;         ///< Only used for GLONASS: This is the frequency slot + 7 (range from 0 to 13)
    uint8_t numWords = 0;       ///< The number of data words contained in this message (0..16)
    uint8_t chn = 0;            ///< The tracking channel number the message was received on
    uint8_t version = 0;        ///< Message version (0x01 for this version)
    uint8_t reserved2 = 0;      ///< Reserved
    std::vector<uint32_t> dwrd; ///< The data words

    // TODO: Make this into functions
    // uint8_t subFrameId = 0; ///< bit 20-22 of word 2/dwrd[1]. 3 bits subframe id (HOW = Handover Word) of GPS
    // uint8_t wrdType = 0;    ///< bit 3-8 of dwrd[0]. 6 bits word types of Galileo I/NAV
};

/// @brief The available SEC Messages
enum UbxSecMessages
{
    /// Unique Chip ID (Length = 9; Type = Output)
    UBX_SEC_UNIQID = 0x03,
};

/// @brief The available TIM Messages
enum UbxTimMessages
{
    /// Oscillator frequency changed notification (Length = 32; Type = Periodic/Polled)
    UBX_TIM_FCHG = 0x16,
    /// Host oscillator control (Length = 8; Type = Input)
    UBX_TIM_HOC = 0x17,
    /// Source measurement (Length = 12 + 24*numMeas; Type = Input/Output)
    UBX_TIM_SMEAS = 0x13,
    /// Survey-in data (Length = 28; Type = Periodic/Polled)
    UBX_TIM_SVIN = 0x04,
    /// Time mark data (Length = 28; Type = Periodic/Polled)
    UBX_TIM_TM2 = 0x03,
    /// Time Pulse Time and Frequency Data (Length = 56; Type = Periodic)
    UBX_TIM_TOS = 0x12,
    /// Time Pulse Timedata (Length = 16; Type = Periodic/Polled)
    UBX_TIM_TP = 0x01,
    /// - Stop calibration (Length = 1; Type = Command)
    /// - VCO calibration extended command (Length = 12; Type = Command)
    /// - Results of the calibration (Length = 12; Type = Periodic/Polled)
    UBX_TIM_VCOCAL = 0x15,
    /// Sourced Time Verification (Length = 20; Type = Periodic/Polled)
    UBX_TIM_VRFY = 0x06,
};

/// @brief The available UPD Messages
enum UbxUpdMessages
{
    /// - Poll Backup File Restore Status (Length = 0; Type = Poll Request)
    /// - Create Backup File in Flash (Length = 4; Type = Command)
    /// - Clear Backup in Flash (Length = 4; Type = command)
    /// - Backup File Creation Acknowledge (Length = 8; Type = Output)
    /// - System Restored from Backup (Length = 8; Type = Output)
    UBX_UPD_SOS = 0x14,
};

/// @brief Get the UBX Msg Class From String object
///
/// @param[in] className String of the UBX Class
/// @return The UBX class
[[nodiscard]] UbxClass getMsgClassFromString(const std::string& className);

/// @brief Get the UBX Msg Id From String object
///
/// @param[in] msgClass The Ubx Msg Class to search in
/// @param[in] idName String of the Msg Id
/// @return The Msg Id integer
[[nodiscard]] uint8_t getMsgIdFromString(UbxClass msgClass, const std::string& idName);

/// @brief Get the UBX Msg Id From String objects
///
/// @param[in] className String of the UBX class
/// @param[in] idName String of the Msg Id
/// @return The Msg Id integer
[[nodiscard]] uint8_t getMsgIdFromString(const std::string& className, const std::string& idName);

} // namespace NAV::vendor::ublox

#ifndef DOXYGEN_IGNORE

template<>
struct fmt::formatter<NAV::vendor::ublox::ErrorDetectionMode> : ostream_formatter
{};
template<>
struct fmt::formatter<NAV::vendor::ublox::NmeaTalkerID> : ostream_formatter
{};
template<>
struct fmt::formatter<NAV::vendor::ublox::NmeaMessageClass> : ostream_formatter
{};
template<>
struct fmt::formatter<NAV::vendor::ublox::NmeaStandardMessages> : ostream_formatter
{};
template<>
struct fmt::formatter<NAV::vendor::ublox::NmeaPubxMessages> : ostream_formatter
{};
template<>
struct fmt::formatter<NAV::vendor::ublox::UbxClass> : ostream_formatter
{};
template<>
struct fmt::formatter<NAV::vendor::ublox::UbxAckMessages> : ostream_formatter
{};
template<>
struct fmt::formatter<NAV::vendor::ublox::UbxCfgMessages> : ostream_formatter
{};
template<>
struct fmt::formatter<NAV::vendor::ublox::UbxEsfMessages> : ostream_formatter
{};
template<>
struct fmt::formatter<NAV::vendor::ublox::UbxHnrMessages> : ostream_formatter
{};
template<>
struct fmt::formatter<NAV::vendor::ublox::UbxInfMessages> : ostream_formatter
{};
template<>
struct fmt::formatter<NAV::vendor::ublox::UbxLogMessages> : ostream_formatter
{};
template<>
struct fmt::formatter<NAV::vendor::ublox::UbxMgaMessages> : ostream_formatter
{};
template<>
struct fmt::formatter<NAV::vendor::ublox::UbxMonMessages> : ostream_formatter
{};
template<>
struct fmt::formatter<NAV::vendor::ublox::UbxNavMessages> : ostream_formatter
{};
template<>
struct fmt::formatter<NAV::vendor::ublox::UbxRxmMessages> : ostream_formatter
{};
template<>
struct fmt::formatter<NAV::vendor::ublox::UbxSecMessages> : ostream_formatter
{};
template<>
struct fmt::formatter<NAV::vendor::ublox::UbxTimMessages> : ostream_formatter
{};
template<>
struct fmt::formatter<NAV::vendor::ublox::UbxUpdMessages> : ostream_formatter
{};

#endif