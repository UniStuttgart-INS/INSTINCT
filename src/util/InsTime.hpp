/// @file InsTime.hpp
/// @brief The class is responsible for all time-related tasks
/// @author T. Lambertus (tomke-jantje.lambertus@nav.uni-stuttgart.de)
/// @date 2020-12-02
///
/// @details This class contains all time-transformations functions. One instance is created for each InsTime object (defined in the structs).
///          Intern, only the MJD-time (Modified Julien Date) is stored (here MJD is a struct which has mjd_day and mjd_frac).

#pragma once

#include <string>
#include <array>
#include <compare>
#include <limits>
#include <iostream>

#include <cmath>

namespace NAV
{
/// The class is responsible for all time-related tasks
class InsTime
{
  public:
    // --------------------------------------- Structs -------------------------------------------

    /// @brief List of all time systems
    enum TIME_SYSTEM
    {
        UTC,
        GPST,
        GLNT,
        GST,
        BDT,
        QZSST,
        IRNSST
    };

    /// Modified Julien Date [UTC]
    struct InsTime_MJD
    {
        unsigned int mjd_day = endOfTheCentury; //!< Full days of the Modified Julien Date [UTC]
        long double mjd_frac = 0.0L;            //!< Decimal fractions of a day of the Modified Julien Date [UTC]

        /// @brief Default constructor
        constexpr InsTime_MJD() = default;

        /// @brief Constructor
        /// @param[in] mjd_day Full days of the Modified Julien Date [UTC]
        /// @param[in] mjd_frac Decimal fractions of a day of the Modified Julien Date [UTC]
        constexpr InsTime_MJD(unsigned int mjd_day, long double mjd_frac)
            : mjd_day(mjd_day), mjd_frac(mjd_frac) {}

        /// @brief Constructor
        /// @param[in] insTime InsTime object
        constexpr explicit InsTime_MJD(const InsTime& insTime)
            : mjd_day(insTime.mjd.mjd_day), mjd_frac(insTime.mjd.mjd_frac) {}

        // FIXME: Replace with Spaceship operator after it becomes available in the C++20 standard
        // constexpr std::strong_ordering operator<=>(const InsTime_MJD& rhs) const
        // {
        //     // perform a three-way comparison between the mjd_day's.
        //     // If that result != 0 (that is, the mjd_day's differ),
        //     // then that's the result of the overall comparison
        //     if (auto cmp = mjd_day <=> rhs.mjd_day; cmp != nullptr)
        //     {
        //         return cmp;
        //     }
        //     if (std::abs(rhs.mjd_frac - mjd_frac) <= epsilon)
        //     {
        //         return std::strong_ordering::equivalent;
        //     }
        //     return mjd_frac > rhs.mjd_frac ? std::strong_ordering::greater : std::strong_ordering::less;
        // }

        constexpr bool operator==(const InsTime_MJD& rhs) const
        {
            return (rhs.mjd_day == mjd_day
                    && std::abs(rhs.mjd_frac - mjd_frac) <= epsilon);
        }
        bool operator!=(const InsTime_MJD& rhs) const
        {
            return (rhs.mjd_day != mjd_day
                    || std::abs(rhs.mjd_frac - mjd_frac) >= epsilon);
        }
        constexpr bool operator<=(const InsTime_MJD& rhs) const
        {
            return (mjd_day < rhs.mjd_day
                    || (mjd_day == rhs.mjd_day && mjd_frac <= rhs.mjd_frac));
        }
        constexpr bool operator>=(const InsTime_MJD& rhs) const
        {
            return (mjd_day > rhs.mjd_day
                    || (mjd_day == rhs.mjd_day && mjd_frac >= rhs.mjd_frac));
        }
        constexpr bool operator<(const InsTime_MJD& rhs) const
        {
            return (mjd_day < rhs.mjd_day
                    || (mjd_day == rhs.mjd_day && mjd_frac < rhs.mjd_frac));
        }
        constexpr bool operator>(const InsTime_MJD& rhs) const
        {
            return (mjd_day > rhs.mjd_day
                    || (mjd_day == rhs.mjd_day && mjd_frac > rhs.mjd_frac));
        }
    };

    /// Julien Date [UTC]
    struct InsTime_JD
    {
        unsigned int jd_day{}; //!< Full days of the Julien Date [UTC]
        long double jd_frac{}; //!< Decimal fractions of a day of the Julien Date [UTC]

        /// @brief Default constructor
        constexpr InsTime_JD() = default;

        /// @brief Constructor
        /// @param[in] jd_day Full days of the Julien Date [UTC]
        /// @param[in] jd_frac Decimal fractions of a day of the Julien Date [UTC]
        constexpr InsTime_JD(unsigned int jd_day, long double jd_frac)
            : jd_day(jd_day), jd_frac(jd_frac) {}

        /// @brief Constructor
        /// @param[in] insTime InsTime object
        constexpr explicit InsTime_JD(const InsTime& insTime)
        {
            jd_day = insTime.mjd.mjd_day + 2400000;
            jd_frac = insTime.mjd.mjd_frac + 0.5L;

            if (jd_frac >= 1.0L)
            {
                jd_day++;
                jd_frac--;
            }
        }
    };

    /// GPS week and time of week in GPS standard time [GPST]
    struct InsTime_GPSweekTow
    {
        friend InsTime;

        uint16_t gpsWeek{};  //!< Contains GPS week in GPS standard time [GPST]
        long double tow{};   //!< Contains GPS time of week in GPS standard time [GPST]
        uint16_t gpsCycle{}; //!< Contains GPS cycle in GPS standard time [GPST]

        /// @brief Default constructor
        constexpr InsTime_GPSweekTow() = default;

        /// @brief Constructor
        /// @param[in] gpsWeek GPS week in GPS standard time [GPST]
        /// @param[in] tow GPS time of week in GPS standard time [GPST]
        /// @param[in] gpsCycle GPS cycle in GPS standard time [GPST]
        constexpr InsTime_GPSweekTow(uint16_t gpsWeek, long double tow, uint16_t gpsCycle)
            : gpsWeek(gpsWeek), tow(tow), gpsCycle(gpsCycle) {}

        /// @brief Constructor
        /// @param[in] insTime InsTime object
        explicit InsTime_GPSweekTow(const InsTime& insTime) // TODO: constexpr
        {
            InsTime_MJD mjd = insTime.mjd;
            mjd.mjd_frac += static_cast<long double>(leapGps2UTC(mjd)) / SEC_IN_DAY; // from UTC to GPST
            if (mjd.mjd_frac < 0.0L)
            {
                mjd.mjd_day -= 1;
                mjd.mjd_frac += 1.0L;
            }
            gpsCycle = static_cast<uint16_t>(static_cast<double>(mjd.mjd_day - DIFF_TO_6_1_1980) / (1024.0 * 7.0));
            gpsWeek = static_cast<uint16_t>((mjd.mjd_day - DIFF_TO_6_1_1980 - gpsCycle * DAYS_PER_GPS_CYCLE * DAYS_PER_WEEK)
                                            / static_cast<double>(DAYS_PER_WEEK));
            tow = static_cast<long double>((mjd.mjd_day - DIFF_TO_6_1_1980) % DAYS_PER_WEEK) * SEC_IN_DAY + mjd.mjd_frac * SEC_IN_DAY;
        }

      private:
        static constexpr unsigned int DIFF_TO_6_1_1980 = 44244;
        static constexpr unsigned int DAYS_PER_GPS_CYCLE = 1024;
        static constexpr unsigned int DAYS_PER_WEEK = 7;
    };

    /// GPS year and day of year in GPS standard time [GPST]
    struct InsTime_YDoySod
    {
        uint16_t year{};   //!< Contains year in GPS standard time [GPST]
        uint16_t doy{};    //!< Contains day of year in GPS standard time [GPST]
        long double sod{}; //!< Contains second of day in GPS standard time [GPST]

        /// @brief Default constructor
        constexpr InsTime_YDoySod() = default;

        /// @brief Constructor
        /// @param[in] year Year in GPS standard time [GPST]
        /// @param[in] doy Day of year in GPS standard time [GPST]
        /// @param[in] sod Second of day in GPS standard time [GPST]
        constexpr InsTime_YDoySod(uint16_t year, uint16_t doy, long double sod)
            : year(year), doy(doy), sod(sod) {}

        /// @brief Constructor
        /// @param[in] insTime InsTime object
        explicit InsTime_YDoySod(const InsTime& insTime) // TODO: constexpr
        {
            auto yearMonthDayHMS = InsTime_YMDHMS(insTime);

            constexpr std::array<uint16_t, 11> daysOfMonth{ 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334 };

            year = yearMonthDayHMS.year;
            sod = static_cast<long double>(yearMonthDayHMS.hour * 3600.0 + yearMonthDayHMS.min * 60.0)
                  + yearMonthDayHMS.sec
                  + static_cast<long double>(leapGps2UTC(yearMonthDayHMS));

            if (yearMonthDayHMS.month == 1)
            {
                doy = yearMonthDayHMS.day;
            }
            else if (yearMonthDayHMS.month == 2)
            {
                doy = daysOfMonth.at(0) + yearMonthDayHMS.day;
            }
            else // for march to december
            {
                if (isLeapYear(yearMonthDayHMS.year))
                {
                    doy = 1U + daysOfMonth.at(yearMonthDayHMS.month - 2) + yearMonthDayHMS.day;
                }
                else
                {
                    doy = daysOfMonth.at(yearMonthDayHMS.month - 2) + yearMonthDayHMS.day;
                }
            }
            if (sod >= SEC_IN_DAY)
            {
                doy++;
                sod -= SEC_IN_DAY;
                if (doy == 366 && !isLeapYear(yearMonthDayHMS.year))
                {
                    year--;
                    doy = 365;
                }
            }
        }
    };

    /// Universal Time Coordinated [UTC]
    struct InsTime_YMDHMS
    {
        uint16_t year{};   //!< Contains year in Universal Time Coordinated [UTC]
        uint16_t month{};  //!< Contains month in Universal Time Coordinated [UTC]
        uint16_t day{};    //!< Contains day in Universal Time Coordinated [UTC]
        uint16_t hour{};   //!< Contains hour in Universal Time Coordinated [UTC]
        uint16_t min{};    //!< Contains minute in Universal Time Coordinated [UTC]
        long double sec{}; //!< Contains second in Universal Time Coordinated [UTC]

        /// @brief Default constructor
        constexpr InsTime_YMDHMS() = default;

        /// @brief Constructor
        /// @param[in] year Year in Universal Time Coordinated [UTC]
        /// @param[in] month Month in Universal Time Coordinated [UTC]
        /// @param[in] day Day in Universal Time Coordinated [UTC]
        /// @param[in] hour Hour in Universal Time Coordinated [UTC]
        /// @param[in] min Minute in Universal Time Coordinated [UTC]
        /// @param[in] sec Second in Universal Time Coordinated [UTC]
        InsTime_YMDHMS(uint16_t year, uint16_t month, uint16_t day, uint16_t hour, uint16_t min, long double sec)
            : year(year), month(month), day(day), hour(hour), min(min), sec(sec) {}

        /// @brief Constructor
        /// @param[in] insTime InsTime object
        explicit InsTime_YMDHMS(const InsTime& insTime) // TODO: constexpr
        {
            // transform MJD to JD
            auto jd = InsTime_JD(InsTime(insTime.mjd));
            jd.jd_frac = jd.jd_frac + 0.5L;
            if (jd.jd_frac >= 1.0L)
            {
                jd.jd_day += 1;
                jd.jd_frac -= 1.0L;
            }
            // transform JD to YMDHMS
            double a = std::floor(static_cast<double>(jd.jd_day) + 0.0) + 32044.0;
            double b = std::floor((4.0 * a + 3.0) / 146097.0);
            double c = a - std::floor((b * 146097.0) / 4.0);

            double d = std::floor((4.0 * c + 3.0) / 1461.0);
            double e = c - std::floor((1461.0 * d) / 4.0);
            double m = std::floor((5.0 * e + 2.0) / 153.0);

            day = static_cast<uint16_t>(e - std::floor((153.0 * m + 2.0) / 5.0) + 1);
            month = static_cast<uint16_t>(m + 3 - 12 * std::floor(m / 10.0));
            year = static_cast<uint16_t>(b * 100 + d - 4800.0 + std::floor(m / 10.0));

            hour = static_cast<uint16_t>(std::floor(jd.jd_frac * 24.0L));
            min = static_cast<uint16_t>(std::floor(jd.jd_frac * 24.0L * 60.0L) - hour * 60.0L);
            sec = jd.jd_frac * 24.0L * 3600.0L - static_cast<long double>(hour * 3600.0 + min * 60.0);
        }
    };

    // --------------------------- Constructors  ----------------------------------

    /// @brief Default Constructor
    constexpr InsTime() = default;

    /// @brief Constructor
    /// @param[in] mjd Time in Modified Julien Date [UTC]
    explicit InsTime(const InsTime_MJD& mjd);

    /// @brief Constructor
    /// @param[in] jd Time in Julien Date [UTC]
    explicit InsTime(const InsTime_JD& jd);

    /// @brief Constructor
    /// @param[in] gpsWeekTow Time in GPS standard time [GPST]
    explicit InsTime(const InsTime_GPSweekTow& gpsWeekTow);

    /// @brief Constructor
    /// @param[in] yearDoySod Time in GPS standard time [GPST]
    explicit InsTime(const InsTime_YDoySod& yearDoySod);

    /// @brief Constructor
    /// @param[in] yearMonthDayHMS Time in Universal Time Coordinated [UTC]
    explicit InsTime(const InsTime_YMDHMS& yearMonthDayHMS);

    /// @brief Constructor
    /// @param[in] gpsWeek GPS week in GPS standard time [GPST]
    /// @param[in] tow GPS time of week in GPS standard time [GPST]
    /// @param[in] gpsCycle GPS cycle in GPS standard time [GPST]
    InsTime(uint16_t gpsWeek, long double tow, uint16_t gpsCycle);

    /// @brief Constructor
    /// @param[in] year Year
    /// @param[in] month Month
    /// @param[in] day Day
    /// @param[in] hour Hour
    /// @param[in] min Minute
    /// @param[in] sec Second
    /// @param[in] timesys Time System in which the previous values are given in
    InsTime(uint16_t year, uint16_t month, uint16_t day, uint16_t hour, uint16_t min, long double sec, TIME_SYSTEM timesys = TIME_SYSTEM::UTC);

    /// @brief Destructor
    ~InsTime() = default;
    /// @brief Copy constructor
    InsTime(const InsTime&) = default;
    /// @brief Move constructor
    InsTime(InsTime&&) = default;
    /// @brief Copy assignment operator
    InsTime& operator=(const InsTime&) = default;
    /// @brief Move assignment operator
    InsTime& operator=(InsTime&&) = default;

    // ---------------------------- Transformation function --------------------------------------

    [[nodiscard]] InsTime_GPSweekTow toGPSweekTow() const; /*!< Returns the current InsTime_MJD struct in InsTime_GPSweekTow format */
    [[nodiscard]] InsTime_MJD toMJD() const;               /*!< Returns the current InsTime_MJD struct */
    [[nodiscard]] InsTime_JD toJD() const;                 /*!< Returns the current InsTime_MJD struct in InsTime_JD format */
    [[nodiscard]] InsTime_YDoySod toYDoySod() const;       /*!< Returns the current InsTime_MJD struct in InsTime_YDoySod format */
    [[nodiscard]] InsTime_YMDHMS toYMDHMS() const;         /*!< Returns the current InsTime_MJD struct in InsTime_YMDHMS format */

    static constexpr long double hms2sec(int hour, int min, long double sec); /*!< Transforms given hour, minute, second to seconds */

    // -------------------------------- Leap functions -------------------------------------------

    [[nodiscard]] uint16_t leapGps2UTC() const;                  /*!< Returns the current number of leap seconds (offset GPST to UTC) */
    static uint16_t leapGps2UTC(InsTime insTime);                /*!< Returns the number of leap seconds (offset GPST to UTC) for the provided InsTime object */
    static uint16_t leapGps2UTC(InsTime_YDoySod yearDoySod);     /*!< Returns the number of leap seconds (offset GPST to UTC) for the provided InsTime_YDoySod time */
    static uint16_t leapGps2UTC(InsTime_GPSweekTow gpsWeekTow);  /*!< Returns the number of leap seconds (offset GPST to UTC) for the provided InsTime_GPSweekTow time */
    static uint16_t leapGps2UTC(InsTime_YMDHMS yearMonthDayHMS); /*!< Returns the number of leap seconds (offset GPST to UTC) for the provided InsTime_YMDHMS time */
    static uint16_t leapGps2UTC(InsTime_MJD mjd_in);             /*!< Returns the number of leap seconds (offset GPST to UTC) for the provided InsTime_MJD time */
    static bool isLeapYear(uint16_t year);                       /*!< Returns true if the provided year is a leap year, false otherwise */
    [[nodiscard]] bool isLeapYear() const;                       /*!< Returns true if the current time (InsTime_MJD mjd) is a leap year, false otherwise */

    // ------------------------ Comparison bigger/smaller/equal ----------------------------------

    // FIXME: Replace with Spaceship operator after it becomes available in the C++20 standard
    // constexpr auto operator<=>(const InsTime& rhs) const = default;
    // constexpr auto operator<=>(const InsTime& rhs) const
    // {
    //     return mjd <=> rhs.mjd;
    // }

    constexpr bool operator==(const InsTime& rhs) const { return mjd == rhs.mjd; }
    constexpr bool operator!=(const InsTime& rhs) const { return mjd != rhs.mjd; }
    constexpr bool operator<=(const InsTime& rhs) const { return mjd <= rhs.mjd; }
    constexpr bool operator>=(const InsTime& rhs) const { return mjd >= rhs.mjd; }
    constexpr bool operator<(const InsTime& rhs) const { return mjd < rhs.mjd; }
    constexpr bool operator>(const InsTime& rhs) const { return mjd > rhs.mjd; }

    // ------------------------------------------------------------------------------

    void addDiffSec(long double diffSec);                         /*!< Adds or subtracts the provided time difference [seconds] to/from the current time */
    [[nodiscard]] long double getTimeDiff(InsTime insTime) const; /*!< Returns the time difference [seconds] between the given InsTime object and the current time (current - given) */
    void RoundToFullDay();                                        /*!< Rounds the current time to a full day (changes time to 0:0:0h UTC of current day) */
    void MakeTimeFromGloOrbit(double UTC_sec);                    /*!< Adds the difference [seconds] between toe (OBRIT-0 last element) and toc (ORBIT-0 first element) to the current time (changes time, so that it corresponds to the time of GLONASS ORBIT last element) */
    [[nodiscard]] std::string GetStringOfDate() const;            /*!< Returns the current time as a string with format YYYYMMDDHHMMSS */

    /// @brief Converts string into a time system (InsTime::TIME_SYSTEM)
    /// @param[in] sys String which defines the name of the time system
    /// @return The Time System
    static TIME_SYSTEM MakeTimeSysFromString(const std::string& sys);

    /// @brief Converts a time system (InsTime::TIME_SYSTEM) into a string
    /// @param[in] sys Time System we want the get the string for
    /// @return String representation of the time system
    static std::string MakeStringFromTimeSys(InsTime::TIME_SYSTEM sys);

  private:
    InsTime_MJD mjd{};

    static constexpr long double epsilon = 2.0L * std::numeric_limits<long double>::epsilon();
    static constexpr long double SEC_IN_WEEK = 604800.0L;
    static constexpr long double SEC_IN_DAY = 86400.0L;
    static constexpr unsigned int endOfTheCentury = 400000U;

    /// Maps GPS leap seconds to a time: array<leap seconds>, index + 1 is amount of leap seconds
    static constexpr std::array<uint32_t, 20> GpsLeapSec = {
        0,     // 1 Jan 1980
        44786, // 1 Jul 1981  //diff UTC-TAI: 20
        45151, // 1 Jul 1982  //diff UTC-TAI: 21
        45516, // 1 Jul 1983  //diff UTC-TAI: 22
        46247, // 1 Jul 1985  //diff UTC-TAI: 23
        47161, // 1 Jan 1988  //diff UTC-TAI: 24
        47892, // 1 Jan 1990  //diff UTC-TAI: 25
        48257, // 1 Jan 1991  //diff UTC-TAI: 26
        48804, // 1 Jul 1992  //diff UTC-TAI: 27
        49169, // 1 Jul 1993  //diff UTC-TAI: 28
        49534, // 1 Jul 1994  //diff UTC-TAI: 29
        50083, // 1 Jan 1996  //diff UTC-TAI: 30
        50630, // 1 Jul 1997  //diff UTC-TAI: 31
        51179, // 1 Jan 1999  //diff UTC-TAI: 32
        53736, // 1 Jan 2006  //diff UTC-TAI: 33
        54832, // 1 Jan 2009  //diff UTC-TAI: 34
        56109, // 1 Jul 2012  //diff UTC-TAI: 35
        57204, // 1 Jul 2015  //diff UTC-TAI: 36
        57754, // 1 Jan 2017  //diff UTC-TAI: 37
        99999, // future
    };
};

std::ostream& operator<<(std::ostream& os, const InsTime& insTime);
std::ostream& operator<<(std::ostream& os, const InsTime::InsTime_YMDHMS& ymdhms);
std::ostream& operator<<(std::ostream& os, const InsTime::InsTime_MJD& mjd);

} // namespace NAV