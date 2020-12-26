/// @file EmlidTypes.hpp
/// @brief Type Definitions for Emlid messages for Emlid Reach M2 ER Protocol
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-06-23

#pragma once

#include <string_view>
#include <array>
#include <optional>
#include <bitset>
#include <vector>

namespace NAV::sensors::emlid
{
enum ErrorDetectionMode
{
    ERRORDETECTIONMODE_NONE,     ///< No error detection is used
    ERRORDETECTIONMODE_CHECKSUM, ///< 16-bit checksum is used
};

/// @brief The available ERB Message IDs
enum ErbMessageID
{
    ERB_MessageId_NONE = 0x00, ///< No Message Class specified
    ERB_MessageId_VER = 0x01,  ///< Version of protocol
    ERB_MessageId_POS = 0x02,  ///< Geodetic position solution
    ERB_MessageId_STAT = 0x03, ///< Receiver navigation status
    ERB_MessageId_DPOS = 0x04, ///< Dilution of precision
    ERB_MessageId_VEL = 0x05,  ///< Velocity solution in NED
    ERB_MessageId_SVI = 0x06,  ///< Space vehicle information
    ERB_MessageId_RTK = 0x07,  ///< RTK information
};

/// @brief Version of Protocol
///
/// This message contains version of the ERB protocol.
/// It comprises 3 numbers: high level of version,
/// medium level of version and low level of version. Full version of protocol looks as follows:
/// [High level].[Medium level].[Low level]
/// For example 1.2.3, where:
/// 1 – high level of version (verH)
/// 2 – medium level of version (verM)
/// 3 – low level of version (verL)
struct ErbVer
{
    uint32_t iTOW = 0; ///< GPS time of week of the navigation epoch [ms]. See the description of iTOW for details.
    uint8_t verH = 0;  ///< High level of version
    uint8_t verM = 0;  ///< Medium level of version
    uint8_t verL = 0;  ///< Low level of version
};

/// @brief Geodetic Position Solution
///
/// This message outputs the geodetic coordinates: longitude, latitude, altitude and information about
/// accuracy estimate.
struct ErbPos
{
    uint32_t iTOW = 0;   ///< GPS time of week of the navigation epoch [ms]. See the description of iTOW for details.
    double lon = 0.0;    ///< Longitude [deg]
    double lat = 0.0;    ///< Latitude [deg]
    double height = 0.0; ///< Height above ellipsoid [m]
    double hMSL = 0.0;   ///< Height above mean sea level [m]
    uint32_t hAcc = 0;   ///< Horizontal accuracy estimate [mm]
    uint32_t vAcc = 0;   ///< Vertical accuracy estimate [mm]
};

/// @brief Receiver Navigation Status
/// This message contains status of Fix, its type and also the number of used satellites
struct ErbStat
{
    uint32_t iTOW = 0;     ///< GPS time of week of the navigation epoch [ms]. See the description of iTOW for details.
    uint16_t weekGPS = 0;  ///< GPS week number of the navigation epoch [weeks]
    uint8_t fixType = 0;   ///< GPSfix type: 0x00 – no Fix, 0x01 – Single, 0x02 – Float, 0x03 – RTK Fix
    uint8_t fixStatus = 0; ///< Navigation Fix Status. If position and velocity are valid 0x01, else 0x00
    uint8_t numSV = 0;     ///< Number of used SVs
};

/// @brief Dilution of Precision
/// This message outputs dimensionless values of DOP. These values are scaled by factor 100.
struct ErbDops
{
    uint32_t iTOW = 0;   ///< GPS time of week of the navigation epoch [ms]. See the description of iTOW for details.
    uint16_t dopGeo = 0; ///< Geometric DOP [*0.01]
    uint16_t dopPos = 0; ///< Position DOP [*0.01]
    uint16_t dopVer = 0; ///< Vertical DOP [*0.01]
    uint16_t dopHor = 0; ///< Horizontal DOP [*0.01]
};

/// @brief Velocity Solution in NED
/// See important comments concerning validity of position given in section Navigation Output Filters.
struct ErbVel
{
    uint32_t iTOW = 0;   ///< GPS time of week of the navigation epoch [ms]. See the description of iTOW for details.
    int32_t velN = 0;    ///< North velocity component [cm/s]
    int32_t velE = 0;    ///< East velocity component [cm/s]
    int32_t velD = 0;    ///< Down velocity component [cm/s]
    uint32_t speed = 0;  ///< Speed (3-D)
    uint32_t gSpeed = 0; ///< Ground speed (2-D)
    int32_t heading = 0; ///< Heading of motion 2-D [deg/// 1e-5]
    uint32_t sAcc = 0;   ///< Speed accuracy Estimation [cm/s]
};

/// @brief Space Vehicle Information
/// This message output information about observation satellites.
struct ErbSvi
{
    uint32_t iTOW = 0;  ///< GPS time of week of the navigation epoch [ms]. See the description of iTOW for details.
    uint8_t nSV = 0;    ///< Number of Satellites
    uint8_t idSV = 0;   ///< Satellite identifier (see Satellite Numbering)
    uint8_t typeSV = 0; ///< GNSS identifier 0-GPS, 1-GLONASS, 2-Galileo, 3-QZSS, 4-Beidou, 5-LEO, 6-SBAS
    int32_t carPh = 0;  ///< Carrier phase [cycle/// 1e-2]
    int32_t psRan = 0;  ///<  Pseudo range residual [m]
    int32_t freqD = 0;  ///<  Doppler frequency [Hz/// 1e-3]
    uint16_t snr = 0;   ///<  Signal strength [dBhz/// 0.25]
    uint16_t azim = 0;  ///<  Azimuth in degrees [deg/// 1e-1]
    uint16_t elev = 0;  ///<  Elevation in degrees [deg/// 1e-1]
};

/// @brief RTK Information
/// This message output information about RTK
struct ErbRtk
{
    uint8_t numSV = 0;     ///< Number of satellites used for RTK calculation
    uint16_t age = 0;      ///< Age of differential [s/// 1e-2] (0 when no corrections, 0xFFFF indicates overflow)
    int32_t baselineN = 0; ///< Distance between base and rover along the north axis [mm]
    int32_t baselineE = 0; ///<Distance between base and rover along the east axis [mm]
    int32_t baselineD = 0; ///<Distance between base and rover along the down axis [mm]
    uint16_t arRatio = 0;  ///< AR Ratio [*1e-1]
    uint16_t weekGPS = 0;  ///< GPS Week Number of last baseline [weeks]
    uint32_t timeGPS = 0;  ///< GPS Time of Week of last baseline [ms]
};

/// @brief Get the ERB Msg ID From String object
///
/// @param[in] idName String of the ERB Class
/// @return The ERB ID
[[nodiscard]] ErbMessageID getMsgIdFromString(const std::string_view& idName);

} // namespace NAV::sensors::emlid
