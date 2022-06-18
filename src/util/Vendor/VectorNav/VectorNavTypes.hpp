/// @file VectorNavTypes.hpp
/// @brief Type Definitions for VectorNav messages
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-07-01

#pragma once

#include <cstdint>
#include <vector>

namespace NAV::vendor::vectornav
{
/// @brief The VPE status bitfield
///
/// Bit | Name           | Description
///  0  | timeOk         | GpsTow is valid.
///  1  | dateOk         | TimeGps and GpsWeek are valid.
///  2  | utcTimeValid   | UTC time is valid.
class TimeStatus
{
  public:
    /// Constructor
    /// @param[in] status Status to set
    explicit TimeStatus(uint8_t status) : _status(status) {}

    /// @brief Assignment operator
    /// @param[in] status Status to set
    TimeStatus& operator=(const uint8_t& status)
    {
        _status = status;
        return *this;
    }

    /// @brief Default constructor
    TimeStatus() = default;

    /// @brief Returns a reference to the status
    [[nodiscard]] constexpr uint8_t& status()
    {
        return _status;
    }

    /// GpsTow is valid
    [[nodiscard]] constexpr uint8_t timeOk() const
    {
        return ((_status & (1U << 0U)) >> 0U);
    }
    /// TimeGps and GpsWeek are valid.
    [[nodiscard]] constexpr uint8_t dateOk() const
    {
        return ((_status & (1U << 1U)) >> 1U); // NOLINT
    }
    /// UTC time is valid.
    [[nodiscard]] constexpr uint8_t utcTimeValid() const
    {
        return ((_status & (1U << 2U)) >> 2U); // NOLINT
    }

  private:
    /// The storage field
    uint8_t _status;
};

/// @brief Storage class for UTC Time
struct UTC
{
    int8_t year{};   ///< The year is given as a signed byte year offset from the year 2000. For example the year 2013 would be given as year 13.
    uint8_t month{}; ///< Months
    uint8_t day{};   ///< Days
    uint8_t hour{};  ///< Hours
    uint8_t min{};   ///< Minutes
    uint8_t sec{};   ///< Seconds
    uint16_t ms{};   ///< Milliseconds
};

/// @brief GNSS fix.
enum GnssFix : uint8_t
{
    GnssFix_NoFix,     ///< No fix
    GnssFix_TimeOnly,  ///< Time only
    GnssFix_2D,        ///< 2D
    GnssFix_3D,        ///< 3D
    GnssFix_SBAS,      ///< SBAS
    GnssFix_RTK_Float, ///< RTK Float (only GNSS1)
    GnssFix_RTK_Fixed  ///< RTK Fixed (only GNSS1)
};

/// @brief Flags for valid GPS TOW, week number and UTC and current leap seconds.
struct TimeInfo
{
    ///     Fields: timeOk | dateOk | utcTimeValid | resv | resv | resv | resv | resv
    /// Bit Offset:   0    |   1    |       2      |   3  |   4  |   5  |   6  |   7
    ///
    /// Name         | Description
    /// ------------ | ----------------------------------
    /// timeOk       | 1 – GpsTow is valid.
    /// dateOk       | 1 – TimeGps and GpsWeek are valid.
    /// utcTimeValid | 1 – UTC time is valid.
    /// resv         | Reserved for future use.
    TimeStatus status{};
    /// @brief Amount of leap seconds
    int8_t leapSeconds{};
};

/// @brief Dilution of precision
struct DOP
{
    float gDop{}; ///< Geometric DOP
    float pDop{}; ///< Positional DOP (Overall 3D position precision)
    float tDop{}; ///< Time DOP (time precision)
    float vDop{}; ///< Vertical DOP (vertical position precision)
    float hDop{}; ///< Horizontal DOP (2D position precision)
    float nDop{}; ///< North DOP
    float eDop{}; ///< East DOP
};

/// @brief Satellite Constellation
enum class SatSys : uint8_t
{
    GPS = 0,
    SBAS = 1,
    Galileo = 2,
    BeiDou = 3,
    IMES = 4,
    QZSS = 5,
    GLONASS = 6,
};

/// @brief Information and measurements pertaining to each GNSS satellite in view.
///
/// The size of this packet will vary depending upon the number of satellites in view. To parse this packet you
/// will first need to read the number of satellites (numSats) in the beginning of the packet to determine the
/// packets overall length. The total length of the packet payload will be 2 + N*8 bytes where N is the number of
/// satellites (numSats).
struct SatInfo
{
    /// @brief Information for a certain satellite
    struct SatInfoElement
    {
        /// @brief Tracking info flags
        enum class Flags : uint8_t
        {
            None = 0,                        ///< No flag set
            Healthy = 1 << 0,                ///< Healthy
            Almanac = 1 << 1,                ///< Almanac
            Ephemeris = 1 << 2,              ///< Ephemeris
            DifferentialCorrection = 1 << 3, ///< Differential Correction
            UsedForNavigation = 1 << 4,      ///< Used for Navigation
            AzimuthElevationValid = 1 << 5,  ///< Azimuth / Elevation Valid
            UsedForRTK = 1 << 6,             ///< Used for RTK
        };

        /// @brief Binary or-operator
        /// @param[in] lhs Left-hand side
        /// @param[in] rhs Right-hand side
        /// @return Binary or-ed result
        friend Flags operator|(Flags lhs, Flags rhs)
        {
            return Flags(int(lhs) | int(rhs));
        }

        /// @brief Quality Indicator
        enum class QualityIndicator : uint8_t
        {
            NoSignal = 0,                                 ///< No signal
            SearchingSignal = 1,                          ///< Searching signal
            SignalAcquired = 2,                           ///< Signal acquired
            SignalDetectedButUnstable = 3,                ///< Signal detected but unstable
            CodeLockedAndTimeSynchronized = 4,            ///< Code locked and time synchronized
            CodeAndCarrierLockedAndTimeSynchronized1 = 5, ///< Code and carrier locked and time synchronized
            CodeAndCarrierLockedAndTimeSynchronized2 = 6, ///< Code and carrier locked and time synchronized
            CodeAndCarrierLockedAndTimeSynchronized3 = 7, ///< Code and carrier locked and time synchronized
        };

        /// @brief Default Constructor
        SatInfoElement() = default;

        /// @brief Constructor
        /// @param[in] sys GNSS constellation indicator
        /// @param[in] svId Space vehicle Id
        /// @param[in] flags Tracking info flags
        /// @param[in] cno Carrier-to-noise density ratio (signal strength) [dB-Hz]
        /// @param[in] qi Quality Indicator
        /// @param[in] el Elevation in degrees
        /// @param[in] az Azimuth angle in degrees
        SatInfoElement(uint8_t sys, uint8_t svId, uint8_t flags, uint8_t cno, uint8_t qi, int8_t el, int16_t az)
            : sys(static_cast<SatSys>(sys)), svId(svId), flags(static_cast<Flags>(flags)), cno(cno), qi(static_cast<QualityIndicator>(qi)), el(el), az(az) {}

        /// @brief Constructor
        /// @param[in] sys GNSS constellation indicator
        /// @param[in] svId Space vehicle Id
        /// @param[in] healthy Healthy
        /// @param[in] almanac Almanac
        /// @param[in] ephemeris Ephemeris
        /// @param[in] differentialCorrection Differential Correction
        /// @param[in] usedForNavigation Used for Navigation
        /// @param[in] azimuthElevationValid Azimuth / Elevation Valid
        /// @param[in] usedForRTK Used for RTK
        /// @param[in] cno Carrier-to-noise density ratio (signal strength) [dB-Hz]
        /// @param[in] qi Quality Indicator
        /// @param[in] el Elevation in degrees
        /// @param[in] az Azimuth angle in degrees
        SatInfoElement(uint8_t sys, uint8_t svId,
                       uint8_t healthy, uint8_t almanac, uint8_t ephemeris, uint8_t differentialCorrection, uint8_t usedForNavigation, uint8_t azimuthElevationValid, uint8_t usedForRTK,
                       uint8_t cno, uint8_t qi, int8_t el, int16_t az)
            : sys(static_cast<SatSys>(sys)), svId(svId), flags((healthy ? Flags::Healthy : Flags::None) | (almanac ? Flags::Almanac : Flags::None) | (ephemeris ? Flags::Ephemeris : Flags::None) | (differentialCorrection ? Flags::DifferentialCorrection : Flags::None) | (usedForNavigation ? Flags::UsedForNavigation : Flags::None) | (azimuthElevationValid ? Flags::AzimuthElevationValid : Flags::None) | (usedForRTK ? Flags::UsedForRTK : Flags::None)), cno(cno), qi(static_cast<QualityIndicator>(qi)), el(el), az(az) {}

        SatSys sys{};          ///< GNSS constellation indicator
        uint8_t svId{};        ///< Space vehicle Id
        Flags flags{};         ///< Tracking info flags
        uint8_t cno{};         ///< Carrier-to-noise density ratio (signal strength) [dB-Hz]
        QualityIndicator qi{}; ///< Quality Indicator
        int8_t el{};           ///< Elevation in degrees
        int16_t az{};          ///< Azimuth angle in degrees
    };

    /// @brief Number of measurements to follow.
    uint8_t numSats{};
    /// @brief SatInfo container
    std::vector<SatInfoElement> satellites;
};

/// @brief Allows combining flags of the SatInfo::SatInfoElement::Flags enum.
///
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ANDed value.
constexpr SatInfo::SatInfoElement::Flags operator&(SatInfo::SatInfoElement::Flags lhs, SatInfo::SatInfoElement::Flags rhs)
{
    return SatInfo::SatInfoElement::Flags(int(lhs) & int(rhs));
}

/// @brief Raw measurements pertaining to each GNSS satellite in view.
struct RawMeas
{
    /// @brief Raw measurements for a certain satellite
    struct SatRawElement
    {
        /// @brief Tracking info flags
        enum class Flags : uint16_t
        {
            None = 0,                     ///< No flag set
            Searching = 1 << 0,           ///< Searching
            Tracking = 1 << 1,            ///< Tracking
            TimeValid = 1 << 2,           ///< Time Valid
            CodeLock = 1 << 3,            ///< Code Lock
            PhaseLock = 1 << 4,           ///< Phase Lock
            PhaseHalfAmbiguity = 1 << 5,  ///< Phase Half Ambiguity
            PhaseHalfSub = 1 << 6,        ///< Phase Half Sub
            PhaseSlip = 1 << 7,           ///< Phase Slip
            PseudorangeSmoothed = 1 << 8, ///< Pseudorange Smoothed
        };

        /// @brief Binary or-operator
        /// @param[in] lhs Left-hand side
        /// @param[in] rhs Right-hand side
        /// @return Binary or-ed result
        friend Flags operator|(Flags lhs, Flags rhs)
        {
            return Flags(int(lhs) | int(rhs));
        }

        /// @brief Channel Indicator
        enum class Chan : uint8_t
        {
            P_Code = 0,       ///< P-code (GPS,GLO)
            CA_Code = 1,      ///< C/A-code (GPS,GLO,SBAS,QZSS), C chan (GAL)
            SemiCodeless = 2, ///< semi-codeless (GPS)
            Y_Code = 3,       ///< Y-code (GPS)
            M_Code = 4,       ///< M-code (GPS)
            Codeless = 5,     ///< codeless (GPS)
            A_Chan = 6,       ///< A chan (GAL)
            B_Chan = 7,       ///< B chan (GAL)
            I_Chan = 8,       ///< I chan (GPS,GAL,QZSS,BDS)
            Q_Chan = 9,       ///< Q chan (GPS,GAL,QZSS,BDS)
            M_Chan = 10,      ///< M chan (L2CGPS, L2CQZSS), D chan (GPS,QZSS)
            L_Chan = 11,      ///< L chan (L2CGPS, L2CQZSS), P chan (GPS,QZSS)
            BC_Chan = 12,     ///< B+C chan (GAL), I+Q chan (GPS,GAL,QZSS,BDS), M+L chan (GPS,QZSS), D+P chan (GPS,QZSS)
            Z_Tracking = 13,  ///< based on Z-tracking (GPS)
            ABC = 14,         ///< A+B+C (GAL)
        };

        /// @brief Frequency indicator
        enum class Freq : uint8_t
        {
            RxChannel = 0, ///< Rx Channel
            L1 = 1,        ///< L1(GPS,QZSS,SBAS), G1(GLO), E2-L1-E1(GAL), B1(BDS)
            L2 = 2,        ///< L2(GPS,QZSS), G2(GLO)
            L5 = 3,        ///< L5(GPS,QZSS,SBAS), E5a(GAL)
            E6 = 4,        ///< E6(GAL), LEX(QZSS), B3(BDS)
            E5b = 5,       ///< E5b(GAL), B2(BDS)
            E5a = 6,       ///< E5a+b(GAL)
        };

        /// @brief Default Constructor
        SatRawElement() = default;

        /// @brief Constructor
        /// @param[in] sys GNSS constellation indicator
        /// @param[in] svId Space vehicle Id
        /// @param[in] freq Frequency indicator
        /// @param[in] chan Channel Indicator
        /// @param[in] slot Slot Id
        /// @param[in] cno Carrier-to-noise density ratio (signal strength) [dB-Hz]
        /// @param[in] flags Tracking info flags
        /// @param[in] pr Pseudorange measurement in meters
        /// @param[in] cp Carrier phase measurement in cycles
        /// @param[in] dp Doppler measurement in Hz. Positive sign for approaching satellites
        SatRawElement(uint8_t sys, uint8_t svId, uint8_t freq, uint8_t chan, int8_t slot, uint8_t cno, uint16_t flags, double pr, double cp, float dp)
            : sys(static_cast<SatSys>(sys)), svId(svId), freq(static_cast<Freq>(freq)), chan(static_cast<Chan>(chan)), slot(slot), cno(cno), flags(static_cast<Flags>(flags)), pr(pr), cp(cp), dp(dp) {}

        /// @brief Constructor
        /// @param[in] sys GNSS constellation indicator
        /// @param[in] svId Space vehicle Id
        /// @param[in] freq Frequency indicator
        /// @param[in] chan Channel Indicator
        /// @param[in] slot Slot Id
        /// @param[in] cno Carrier-to-noise density ratio (signal strength) [dB-Hz]
        /// @param[in] searching Searching
        /// @param[in] tracking Tracking
        /// @param[in] timeValid Time Valid
        /// @param[in] codeLock Code Lock
        /// @param[in] phaseLock Phase Lock
        /// @param[in] phaseHalfAmbiguity Phase Half Ambiguity
        /// @param[in] phaseHalfSub Phase Half Sub
        /// @param[in] phaseSlip Phase Slip
        /// @param[in] pseudorangeSmoothed Pseudorange Smoothed
        /// @param[in] pr Pseudorange measurement in meters
        /// @param[in] cp Carrier phase measurement in cycles
        /// @param[in] dp Doppler measurement in Hz. Positive sign for approaching satellites
        SatRawElement(uint8_t sys, uint8_t svId, uint8_t freq, uint8_t chan, int8_t slot, uint8_t cno,
                      uint8_t searching, uint8_t tracking, uint8_t timeValid, uint8_t codeLock, uint8_t phaseLock, uint8_t phaseHalfAmbiguity, uint8_t phaseHalfSub, uint8_t phaseSlip, uint8_t pseudorangeSmoothed,
                      double pr, double cp, double dp)
            : sys(static_cast<SatSys>(sys)), svId(svId), freq(static_cast<Freq>(freq)), chan(static_cast<Chan>(chan)), slot(slot), cno(cno), flags((searching ? Flags::Searching : Flags::None) | (tracking ? Flags::Tracking : Flags::None) | (timeValid ? Flags::TimeValid : Flags::None) | (codeLock ? Flags::CodeLock : Flags::None) | (phaseLock ? Flags::PhaseLock : Flags::None) | (phaseHalfAmbiguity ? Flags::PhaseHalfAmbiguity : Flags::None) | (phaseHalfSub ? Flags::PhaseHalfSub : Flags::None) | (phaseSlip ? Flags::PhaseSlip : Flags::None) | (pseudorangeSmoothed ? Flags::PseudorangeSmoothed : Flags::None)), pr(pr), cp(cp), dp(static_cast<float>(dp)) {}

        SatSys sys{};   ///< GNSS constellation indicator
        uint8_t svId{}; ///< Space vehicle Id
        Freq freq{};    ///< Frequency indicator
        Chan chan{};    ///< Channel Indicator
        int8_t slot{};  ///< Slot Id
        uint8_t cno{};  ///< Carrier-to-noise density ratio (signal strength) [dB-Hz]
        Flags flags{};  ///< Tracking info flags
        double pr{};    ///< Pseudorange measurement in meters
        double cp{};    ///< Carrier phase measurement in cycles
        float dp{};     ///< Doppler measurement in Hz. Positive sign for approaching satellites
    };

    /// @brief Time of week in seconds
    double tow{};
    /// @brief GPS week number
    uint16_t week{};
    /// @brief Number of measurements to follow
    uint8_t numSats{};
    /// @brief SatRaw container
    std::vector<SatRawElement> satellites;
};

/// @brief Allows combining flags of the RawMeas::SatRawElement::Flags enum.
///
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ANDed value.
constexpr RawMeas::SatRawElement::Flags operator&(RawMeas::SatRawElement::Flags lhs, RawMeas::SatRawElement::Flags rhs)
{
    return RawMeas::SatRawElement::Flags(int(lhs) & int(rhs));
}

/// @brief The VPE status bitfield
///
/// Bit | Name                    | Description
/// 0+1 | AttitudeQuality         | Provides an indication of the quality of the attitude solution. 0 - Excellent, 1 - Good, 2 - Bad, 3 - Not tracking
///  2  | GyroSaturation          | At least one gyro axis is currently saturated.
///  3  | GyroSaturationRecovery  | Filter is in the process of recovering from a gyro saturation event.
/// 4+5 | MagDisturbance          | A magnetic DC disturbance has been detected. 0 – No magnetic disturbance. 1 to 3 – Magnetic disturbance is present.
///  6  | MagSaturation           | At least one magnetometer axis is currently saturated.
/// 7+8 | AccDisturbance          | A strong acceleration disturbance has been detected. 0 – No acceleration disturbance. 1 to 3 – Acceleration disturbance has been detected.
///  9  | AccSaturation           | At least one accelerometer axis is currently saturated.
/// 11  | KnownMagDisturbance     | A known magnetic disturbance has been reported by the user and the magnetometer is currently tuned out.
/// 12  | KnownAccelDisturbance   | A known acceleration disturbance has been reported by the user and the accelerometer is currently tuned out.
class VpeStatus
{
  public:
    /// Constructor
    /// @param[in] status Status to set
    explicit VpeStatus(uint16_t status) : _status(status) {}

    /// @brief Assignment operator
    /// @param[in] status Status to set
    VpeStatus& operator=(const uint16_t& status)
    {
        _status = status;
        return *this;
    }

    /// @brief Default constructor
    VpeStatus() = default;

    /// @brief Returns a reference to the status
    [[nodiscard]] constexpr uint16_t& status()
    {
        return _status;
    }

    /// Extract the attitude quality from the vpe status
    [[nodiscard]] constexpr uint8_t attitudeQuality() const
    {
        return ((_status & (1U << 0U | 1U << 1U)) >> 0U);
    }
    /// Extract the gyro saturation from the vpe status
    [[nodiscard]] constexpr uint8_t gyroSaturation() const
    {
        return ((_status & (1U << 2U)) >> 2U); // NOLINT
    }
    /// Extract the gyro saturation recovery from the vpe status
    [[nodiscard]] constexpr uint8_t gyroSaturationRecovery() const
    {
        return ((_status & (1U << 3U)) >> 3U); // NOLINT
    }
    /// Extract the magnetic disturbance from the vpe status
    [[nodiscard]] constexpr uint8_t magDisturbance() const
    {
        return ((_status & (1U << 4U | 1U << 5U)) >> 4U); // NOLINT
    }
    /// Extract the magnetic saturation from the vpe status
    [[nodiscard]] constexpr uint8_t magSaturation() const
    {
        return ((_status & (1U << 6U)) >> 6U); // NOLINT
    }
    /// Extract the acceleration disturbance from the vpe status
    [[nodiscard]] constexpr uint8_t accDisturbance() const
    {
        return ((_status & (1U << 7U | 1U << 8U)) >> 7U); // NOLINT
    }
    /// Extract the acceleration saturation from the vpe status
    [[nodiscard]] constexpr uint8_t accSaturation() const
    {
        return ((_status & (1U << 9U)) >> 9U); // NOLINT
    }
    /// Extract the known magnetic disturbance from the vpe status
    [[nodiscard]] constexpr uint8_t knownMagDisturbance() const
    {
        return ((_status & (1U << 11U)) >> 11U); // NOLINT
    }
    /// Extract the known acceleration disturbance from the vpe status
    [[nodiscard]] constexpr uint8_t knownAccelDisturbance() const
    {
        return ((_status & (1U << 12U)) >> 12U); // NOLINT
    }

  private:
    /// The storage field
    uint16_t _status;
};

/// @brief The INS status bitfield
///
/// Bit | Name                    | Description
/// 0+1 | Mode                    | Indicates the current mode of the INS filter.
///                               | 0 = Not tracking. GNSS Compass is initializing. Output heading is based on magnetometer measurements.
///                               | 1 = Aligning.
///                               |     INS Filter is dynamically aligning.
///                               |     For a stationary startup: GNSS Compass has initialized and INS Filter is
///                               |     aligning from the magnetic heading to the GNSS Compass heading.
///                               |     For a dynamic startup: INS Filter has initialized and is dynamically aligning to
///                               |     True North heading.
///                               |     In operation, if the INS Filter drops from INS Mode 2 back down to 1, the
///                               |     attitude uncertainty has increased above 2 degrees.
///                               | 2 = Tracking. The INS Filter is tracking and operating within specification.
///                               | 3 = Loss of GNSS. A GNSS outage has lasted more than 45 seconds. The INS Filter will
///                               |     no longer update the position and velocity outputs, but the attitude remains valid.
///  2  | GpsFix                  | Indicates whether the GNSS has a proper fix.
///  4  | IMU Error               | High if IMU communication error is detected.
///  5  | Mag/Pres Error          | High if Magnetometer or Pressure sensor error is detected.
///  6  | GNSS Error              | High if GNSS communication error is detected,
///  8  | GpsHeadingIns           | In stationary operation, if set the INS Filter has fully aligned to the GNSS Compass solution.
///                               | In dynamic operation, the GNSS Compass solution is currently aiding the INS Filter heading solution.
///  9  | GpsCompass              | Indicates if the GNSS compass is operational and reporting a heading solution.
class InsStatus
{
  public:
    /// Constructor
    /// @param[in] status Status to set
    explicit InsStatus(uint16_t status) : _status(status) {}

    /// @brief Assignment operator
    /// @param[in] status Status to set
    InsStatus& operator=(const uint16_t& status)
    {
        _status = status;
        return *this;
    }

    /// @brief Default constructor
    InsStatus() = default;

    /// @brief Returns a reference to the status
    [[nodiscard]] constexpr uint16_t& status()
    {
        return _status;
    }

    /// Extract the current mode of the INS filter from the ins status
    [[nodiscard]] constexpr uint8_t mode() const
    {
        return ((_status & (1U << 0U | 1U << 1U)) >> 0U);
    }
    /// Extract the GPS Fix from the ins status
    [[nodiscard]] constexpr bool gpsFix() const
    {
        return ((_status & (1U << 2U)) >> 2U); // NOLINT
    }
    /// Extract the IMU Error from the ins status
    [[nodiscard]] constexpr bool errorIMU() const
    {
        return ((_status & (1U << 4U)) >> 4U); // NOLINT
    }
    /// Extract the Mag/Pres Error from the ins status
    [[nodiscard]] constexpr bool errorMagPres() const
    {
        return ((_status & (1U << 5U)) >> 5U); // NOLINT
    }
    /// Extract the GNSS Error from the ins status
    [[nodiscard]] constexpr bool errorGnss() const
    {
        return ((_status & (1U << 6U)) >> 6U); // NOLINT
    }
    /// Extract the GPS Heading INS from the ins status
    [[nodiscard]] constexpr bool gpsHeadingIns() const
    {
        return ((_status & (1U << 8U)) >> 8U); // NOLINT
    }
    /// Extract the GPS Compass from the ins status
    [[nodiscard]] constexpr bool gpsCompass() const
    {
        return ((_status & (1U << 9U)) >> 9U); // NOLINT
    }

  private:
    /// The storage field
    uint16_t _status;
};

} // namespace NAV::vendor::vectornav