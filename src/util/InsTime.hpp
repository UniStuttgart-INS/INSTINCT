/// @file InsTime.hpp
/// @brief The class is responsible for all time-related tasks
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @author T. Lambertus (tomke-jantje.lambertus@nav.uni-stuttgart.de)
/// @date 2020-12-02
///
/// @details This class contains all time-transformations functions. One instance is created for each InsTime object (defined in the structs).
///          Internally, only the MJD-time (Modified Julien Date) is stored (here MJD is a struct which has mjd_day and mjd_frac).

#pragma once

#include <string>
#include <array>
#include <limits>
#include <iostream>
#include <chrono>

#include "gcem.hpp"

namespace NAV
{
/// @brief Utility Namespace for Time related tasks
namespace InsTimeUtil
{
constexpr int32_t END_OF_THE_CENTURY_MJD = 400000; ///< Modified Julian Date of the end of the century (15.01.2954)
constexpr int32_t WEEKS_PER_GPS_CYCLE = 1024;      ///< Weeks per GPS cycle
constexpr int32_t DIFF_TO_6_1_1980_MJD = 44244;    ///< 06.01.1980 in Modified Julian Date

constexpr int32_t DIFF_MJD_TO_JD_DAYS = 2400000;  ///< Difference of the days between MJD and JD
constexpr long double DIFF_MJD_TO_JD_FRAC = 0.5L; ///< Difference of the fraction between MJD and JD

constexpr int32_t MONTHS_PER_YEAR = 12;                                                    ///< Months / Year
constexpr int32_t DAYS_PER_YEAR = 365;                                                     ///< Days / Year
constexpr int32_t DAYS_PER_WEEK = 7;                                                       ///< Days / Week
constexpr int32_t HOURS_PER_DAY = 24;                                                      ///< Hours / Day
constexpr int32_t HOURS_PER_WEEK = HOURS_PER_DAY * DAYS_PER_WEEK;                          ///< Hours / Week
constexpr int32_t MINUTES_PER_HOUR = 60;                                                   ///< Minutes / Hour
constexpr int32_t MINUTES_PER_DAY = MINUTES_PER_HOUR * HOURS_PER_DAY;                      ///< Minutes / Day
constexpr int32_t MINUTES_PER_WEEK = MINUTES_PER_DAY * DAYS_PER_WEEK;                      ///< Minutes / Week
constexpr int32_t SECONDS_PER_MINUTE = 60;                                                 ///< Seconds / Minute
constexpr int32_t SECONDS_PER_HOUR = SECONDS_PER_MINUTE * MINUTES_PER_HOUR;                ///< Seconds / Hour
constexpr int32_t SECONDS_PER_DAY = SECONDS_PER_MINUTE * MINUTES_PER_HOUR * HOURS_PER_DAY; ///< Seconds / Day
constexpr int32_t SECONDS_PER_WEEK = SECONDS_PER_DAY * DAYS_PER_WEEK;                      ///< Seconds / Week

/// Numerical precision for 'long double' variables
constexpr long double EPSILON = 2.0L * std::numeric_limits<long double>::epsilon();

/// Maps GPS leap seconds to a time: array<mjd_day>, index + 1 is amount of leap seconds
constexpr std::array<int32_t, 20> GPS_LEAP_SEC_MJD = {
    0,     // 1 Jan 1980 and before
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

/// @brief Returns true if the provided year is a leap year, false otherwise
/// @param[in] year The year to check
/// @return True if the year is a leap year, otherwise false
constexpr bool isLeapYear(int32_t year)
{
    return (((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0));
}

/// @brief Returns the number of days in the specified month of the year
/// @param[in] month Month [1-12]
/// @param[in] year Year to determine the leap year
/// @return Number of days in the specified month
constexpr int32_t daysInMonth(int32_t month, int32_t year)
{
    while (month > InsTimeUtil::MONTHS_PER_YEAR)
    {
        month -= InsTimeUtil::MONTHS_PER_YEAR;
        year++;
    }
    while (month < 1)
    {
        month += MONTHS_PER_YEAR;
        year--;
    }

    switch (month)
    {
    case 1:
        return 31;
    case 2:
        if (isLeapYear(year))
        {
            return 29;
        }
        return 28;
    case 3:
        return 31;
    case 4:
        return 30;
    case 5:
        return 31;
    case 6:
        return 30;
    case 7:
        return 31;
    case 8:
        return 31;
    case 9:
        return 30;
    case 10:
        return 31;
    case 11:
        return 30;
    case 12:
        return 31;
    default:
        return 366;
    }
}
} // namespace InsTimeUtil

/// Modified Julien Date [UTC]
struct InsTime_MJD
{
    int32_t mjd_day = 0;         ///< Full days since 17. November 1858 [UTC]
    long double mjd_frac = 0.0L; ///< Decimal fractions of a day of the Modified Julien Date [UTC]

    /// @brief Default constructor
    constexpr InsTime_MJD() = default;

    /// @brief Constructor
    /// @param[in] mjd_day Full days of the Modified Julien Date [UTC]
    /// @param[in] mjd_frac Decimal fractions of a day of the Modified Julien Date [UTC]
    constexpr InsTime_MJD(int32_t mjd_day, long double mjd_frac)
        : mjd_day(mjd_day), mjd_frac(mjd_frac)
    {
        while (this->mjd_frac >= 1.0L)
        {
            this->mjd_frac -= 1.0L;
            this->mjd_day++;
        }
        while (this->mjd_frac < 0.0L)
        {
            this->mjd_frac += 1.0L;
            this->mjd_day--;
        }
    }

    /// @brief Equal comparison operator (takes double precision into account)
    /// @param[in] rhs Right-hand side to compare with
    /// @return Comparison result
    constexpr bool operator==(const InsTime_MJD& rhs) const
    {
        return (mjd_day == rhs.mjd_day
                && gcem::abs(mjd_frac - rhs.mjd_frac) <= InsTimeUtil::EPSILON);
    }
    /// @brief Inequal comparison operator (takes double precision into account)
    /// @param[in] rhs Right-hand side to compare with
    /// @return Comparison result
    constexpr bool operator!=(const InsTime_MJD& rhs) const
    {
        return !(*this == rhs);
    }
    /// @brief Smaller or equal comparison operator (takes double precision into account)
    /// @param[in] rhs Right-hand side to compare with
    /// @return Comparison result
    constexpr bool operator<=(const InsTime_MJD& rhs) const
    {
        return *this < rhs || *this == rhs;
    }
    /// @brief Greater or equal comparison operator (takes double precision into account)
    /// @param[in] rhs Right-hand side to compare with
    /// @return Comparison result
    constexpr bool operator>=(const InsTime_MJD& rhs) const
    {
        return *this > rhs || *this == rhs;
    }
    /// @brief Smaller comparison operator (takes double precision into account)
    /// @param[in] rhs Right-hand side to compare with
    /// @return Comparison result
    constexpr bool operator<(const InsTime_MJD& rhs) const
    {
        return (mjd_day < rhs.mjd_day
                || (mjd_day == rhs.mjd_day && (mjd_frac < rhs.mjd_frac && *this != rhs)));
    }
    /// @brief Greater comparison operator (takes double precision into account)
    /// @param[in] rhs Right-hand side to compare with
    /// @return Comparison result
    constexpr bool operator>(const InsTime_MJD& rhs) const
    {
        return !(*this <= rhs);
    }
};

/// Julien Date [UTC]
struct InsTime_JD
{
    int32_t jd_day{};      ///< Full days since 1. January −4712 [UTC]
    long double jd_frac{}; ///< Decimal fractions of a day of the Julien Date [UTC]

    /// @brief Constructor
    /// @param[in] jd_day Full days of the Julien Date [UTC]
    /// @param[in] jd_frac Decimal fractions of a day of the Julien Date [UTC]
    constexpr InsTime_JD(int32_t jd_day, long double jd_frac)
        : jd_day(jd_day), jd_frac(jd_frac)
    {
        while (this->jd_frac >= 1.0L)
        {
            this->jd_frac -= 1.0L;
            this->jd_day++;
        }
        while (this->jd_frac < 0.0L)
        {
            this->jd_frac += 1.0L;
            this->jd_day--;
        }
    }

    /// @brief Equal comparison operator (takes double precision into account)
    /// @param[in] rhs Right-hand side to compare with
    /// @return Comparison result
    constexpr bool operator==(const InsTime_JD& rhs) const
    {
        return (jd_day == rhs.jd_day
                && gcem::abs(jd_frac - rhs.jd_frac) <= InsTimeUtil::EPSILON);
    }
    /// @brief Inequal comparison operator (takes double precision into account)
    /// @param[in] rhs Right-hand side to compare with
    /// @return Comparison result
    constexpr bool operator!=(const InsTime_JD& rhs) const
    {
        return !(*this == rhs);
    }
    /// @brief Smaller or equal comparison operator (takes double precision into account)
    /// @param[in] rhs Right-hand side to compare with
    /// @return Comparison result
    constexpr bool operator<=(const InsTime_JD& rhs) const
    {
        return *this < rhs || *this == rhs;
    }
    /// @brief Greater or equal comparison operator (takes double precision into account)
    /// @param[in] rhs Right-hand side to compare with
    /// @return Comparison result
    constexpr bool operator>=(const InsTime_JD& rhs) const
    {
        return *this > rhs || *this == rhs;
    }
    /// @brief Smaller comparison operator (takes double precision into account)
    /// @param[in] rhs Right-hand side to compare with
    /// @return Comparison result
    constexpr bool operator<(const InsTime_JD& rhs) const
    {
        return (jd_day < rhs.jd_day
                || (jd_day == rhs.jd_day && jd_frac < rhs.jd_frac && *this != rhs));
    }
    /// @brief Greater comparison operator (takes double precision into account)
    /// @param[in] rhs Right-hand side to compare with
    /// @return Comparison result
    constexpr bool operator>(const InsTime_JD& rhs) const
    {
        return !(*this <= rhs);
    }
};

/// GPS week and time of week in GPS standard time [GPST]
struct InsTime_GPSweekTow
{
    int32_t gpsCycle{}; ///< Contains GPS cycle in GPS standard time [GPST]
    int32_t gpsWeek{};  ///< Contains GPS week in GPS standard time [GPST]
    long double tow{};  ///< Contains GPS time of week in seconds in GPS standard time [GPST]

    /// @brief Constructor
    /// @param[in] gpsCycle GPS cycle in GPS standard time [GPST]
    /// @param[in] gpsWeek GPS week in GPS standard time [GPST]
    /// @param[in] tow GPS time of week in seconds in GPS standard time [GPST]
    constexpr InsTime_GPSweekTow(int32_t gpsCycle, int32_t gpsWeek, long double tow)
        : gpsCycle(gpsCycle), gpsWeek(gpsWeek), tow(tow)
    {
        while (this->tow >= InsTimeUtil::SECONDS_PER_WEEK)
        {
            this->tow -= InsTimeUtil::SECONDS_PER_WEEK;
            this->gpsWeek++;
        }
        while (this->tow < 0.0L)
        {
            this->tow += InsTimeUtil::SECONDS_PER_WEEK;
            this->gpsWeek--;
        }
        while (this->gpsWeek >= InsTimeUtil::WEEKS_PER_GPS_CYCLE)
        {
            this->gpsWeek -= InsTimeUtil::WEEKS_PER_GPS_CYCLE;
            this->gpsCycle++;
        }
        while (this->gpsWeek < 0)
        {
            this->gpsWeek += InsTimeUtil::WEEKS_PER_GPS_CYCLE;
            this->gpsCycle--;
        }
    };

    /// @brief Equal comparison operator (takes double precision into account)
    /// @param[in] rhs Right-hand side to compare with
    /// @return Comparison result
    constexpr bool operator==(const InsTime_GPSweekTow& rhs) const
    {
        return (gpsCycle == rhs.gpsCycle
                && gpsWeek == rhs.gpsWeek
                && gcem::abs(tow - rhs.tow) <= InsTimeUtil::EPSILON);
    }
    /// @brief Inequal comparison operator (takes double precision into account)
    /// @param[in] rhs Right-hand side to compare with
    /// @return Comparison result
    constexpr bool operator!=(const InsTime_GPSweekTow& rhs) const
    {
        return !(*this == rhs);
    }
    /// @brief Smaller or equal comparison operator (takes double precision into account)
    /// @param[in] rhs Right-hand side to compare with
    /// @return Comparison result
    constexpr bool operator<=(const InsTime_GPSweekTow& rhs) const
    {
        return *this < rhs || *this == rhs;
    }
    /// @brief Greater or equal comparison operator (takes double precision into account)
    /// @param[in] rhs Right-hand side to compare with
    /// @return Comparison result
    constexpr bool operator>=(const InsTime_GPSweekTow& rhs) const
    {
        return *this > rhs || *this == rhs;
    }
    /// @brief Smaller comparison operator (takes double precision into account)
    /// @param[in] rhs Right-hand side to compare with
    /// @return Comparison result
    constexpr bool operator<(const InsTime_GPSweekTow& rhs) const
    {
        return (gpsCycle < rhs.gpsCycle
                || gpsWeek < rhs.gpsWeek
                || (gpsCycle == rhs.gpsCycle
                    && gpsWeek == rhs.gpsWeek
                    && tow < rhs.tow
                    && *this != rhs));
    }
    /// @brief Greater comparison operator (takes double precision into account)
    /// @param[in] rhs Right-hand side to compare with
    /// @return Comparison result
    constexpr bool operator>(const InsTime_GPSweekTow& rhs) const
    {
        return !(*this <= rhs);
    }
};

/// Universal Time Coordinated [UTC]
struct InsTime_YMDHMS
{
    int32_t year{};    ///< Contains year in Universal Time Coordinated [UTC]
    int32_t month{};   ///< Contains month in Universal Time Coordinated [UTC]
    int32_t day{};     ///< Contains day in Universal Time Coordinated [UTC]
    int32_t hour{};    ///< Contains hour in Universal Time Coordinated [UTC]
    int32_t min{};     ///< Contains minute in Universal Time Coordinated [UTC]
    long double sec{}; ///< Contains second in Universal Time Coordinated [UTC]

    /// @brief Constructor
    /// @param[in] year Year in Universal Time Coordinated [UTC]
    /// @param[in] month Month in Universal Time Coordinated [UTC]
    /// @param[in] day Day in Universal Time Coordinated [UTC]
    /// @param[in] hour Hour in Universal Time Coordinated [UTC]
    /// @param[in] min Minute in Universal Time Coordinated [UTC]
    /// @param[in] sec Second in Universal Time Coordinated [UTC]
    constexpr InsTime_YMDHMS(int32_t year, int32_t month, int32_t day, int32_t hour, int32_t min, long double sec)
        : year(year), month(month), day(day), hour(hour), min(min), sec(sec)
    {
        while (this->sec >= InsTimeUtil::SECONDS_PER_MINUTE)
        {
            this->sec -= InsTimeUtil::SECONDS_PER_MINUTE;
            this->min++;
        }
        while (this->sec < 0.0L)
        {
            this->sec += InsTimeUtil::SECONDS_PER_MINUTE;
            this->min--;
        }

        while (this->min >= InsTimeUtil::MINUTES_PER_HOUR)
        {
            this->min -= InsTimeUtil::MINUTES_PER_HOUR;
            this->hour++;
        }
        while (this->min < 0)
        {
            this->min += InsTimeUtil::MINUTES_PER_HOUR;
            this->hour--;
        }

        while (this->hour >= InsTimeUtil::HOURS_PER_DAY)
        {
            this->hour -= InsTimeUtil::HOURS_PER_DAY;
            this->day++;
        }
        while (this->hour < 0)
        {
            this->hour += InsTimeUtil::HOURS_PER_DAY;
            this->day--;
        }

        while (this->day > InsTimeUtil::daysInMonth(this->month, this->year))
        {
            this->day -= InsTimeUtil::daysInMonth(this->month, this->year);
            this->month++;
        }
        while (this->day < 1)
        {
            this->day += InsTimeUtil::daysInMonth(this->month, this->year);
            this->month--;
        }

        while (this->month > InsTimeUtil::MONTHS_PER_YEAR)
        {
            this->month -= InsTimeUtil::MONTHS_PER_YEAR;
            this->year++;
        }
        while (this->month < 1)
        {
            this->month += InsTimeUtil::MONTHS_PER_YEAR;
            this->year--;
        }
    }

    /// @brief Equal comparison operator (takes double precision into account)
    /// @param[in] rhs Right-hand side to compare with
    /// @return Comparison result
    constexpr bool operator==(const InsTime_YMDHMS& rhs) const
    {
        return (year == rhs.year
                && month == rhs.month
                && day == rhs.day
                && hour == rhs.hour
                && min == rhs.min
                && gcem::abs(sec - rhs.sec) <= InsTimeUtil::EPSILON);
    }
    /// @brief Inequal comparison operator (takes double precision into account)
    /// @param[in] rhs Right-hand side to compare with
    /// @return Comparison result
    constexpr bool operator!=(const InsTime_YMDHMS& rhs) const
    {
        return !(*this == rhs);
    }
    /// @brief Smaller or equal comparison operator (takes double precision into account)
    /// @param[in] rhs Right-hand side to compare with
    /// @return Comparison result
    constexpr bool operator<=(const InsTime_YMDHMS& rhs) const
    {
        return *this < rhs || *this == rhs;
    }
    /// @brief Greater or equal comparison operator (takes double precision into account)
    /// @param[in] rhs Right-hand side to compare with
    /// @return Comparison result
    constexpr bool operator>=(const InsTime_YMDHMS& rhs) const
    {
        return *this > rhs || *this == rhs;
    }
    /// @brief Smaller comparison operator (takes double precision into account)
    /// @param[in] rhs Right-hand side to compare with
    /// @return Comparison result
    constexpr bool operator<(const InsTime_YMDHMS& rhs) const
    {
        return (year < rhs.year
                || month < rhs.month
                || day < rhs.day
                || hour < rhs.hour
                || min < rhs.min
                || (year == rhs.year
                    && month == rhs.month
                    && day == rhs.day
                    && hour == rhs.hour
                    && min == rhs.min
                    && sec < rhs.sec
                    && *this != rhs));
    }
    /// @brief Greater comparison operator (takes double precision into account)
    /// @param[in] rhs Right-hand side to compare with
    /// @return Comparison result
    constexpr bool operator>(const InsTime_YMDHMS& rhs) const
    {
        return !(*this <= rhs);
    }
};

/// GPS year and day of year in GPS standard time [GPST]
struct InsTime_YDoySod
{
    int32_t year{};    ///< Contains year in GPS standard time [GPST]
    int32_t doy{};     ///< Contains day of year in GPS standard time [GPST]
    long double sod{}; ///< Contains second of day in GPS standard time [GPST]

    /// @brief Constructor
    /// @param[in] year Year in GPS standard time [GPST]
    /// @param[in] doy Day of year in GPS standard time [1-365(/366)]
    /// @param[in] sod Second of day in GPS standard time [GPST]
    constexpr InsTime_YDoySod(int32_t year, int32_t doy, long double sod)
        : year(year), doy(doy), sod(sod)
    {
        while (this->sod >= InsTimeUtil::SECONDS_PER_DAY)
        {
            this->sod -= InsTimeUtil::SECONDS_PER_DAY;
            this->doy++;
        }
        while (this->sod < 0)
        {
            this->sod += InsTimeUtil::SECONDS_PER_DAY;
            this->doy--;
        }

        while (this->doy > InsTimeUtil::DAYS_PER_YEAR + static_cast<int32_t>(InsTimeUtil::isLeapYear(this->year)))
        {
            this->doy -= InsTimeUtil::DAYS_PER_YEAR + static_cast<int32_t>(InsTimeUtil::isLeapYear(this->year));
            this->year++;
        }
        while (this->doy < 1)
        {
            this->doy += InsTimeUtil::DAYS_PER_YEAR + static_cast<int32_t>(InsTimeUtil::isLeapYear(this->year - 1));
            this->year--;
        }
    }

    /// @brief Equal comparison operator (takes double precision into account)
    /// @param[in] rhs Right-hand side to compare with
    /// @return Comparison result
    constexpr bool operator==(const InsTime_YDoySod& rhs) const
    {
        return (year == rhs.year
                && doy == rhs.doy
                && gcem::abs(sod - rhs.sod) <= InsTimeUtil::EPSILON);
    }
    /// @brief Inequal comparison operator (takes double precision into account)
    /// @param[in] rhs Right-hand side to compare with
    /// @return Comparison result
    constexpr bool operator!=(const InsTime_YDoySod& rhs) const
    {
        return !(*this == rhs);
    }
    /// @brief Smaller or equal comparison operator (takes double precision into account)
    /// @param[in] rhs Right-hand side to compare with
    /// @return Comparison result
    constexpr bool operator<=(const InsTime_YDoySod& rhs) const
    {
        return *this < rhs || *this == rhs;
    }
    /// @brief Greater or equal comparison operator (takes double precision into account)
    /// @param[in] rhs Right-hand side to compare with
    /// @return Comparison result
    constexpr bool operator>=(const InsTime_YDoySod& rhs) const
    {
        return *this > rhs || *this == rhs;
    }
    /// @brief Smaller comparison operator (takes double precision into account)
    /// @param[in] rhs Right-hand side to compare with
    /// @return Comparison result
    constexpr bool operator<(const InsTime_YDoySod& rhs) const
    {
        return (year < rhs.year
                || doy < rhs.doy
                || (year == rhs.year
                    && doy == rhs.doy
                    && sod < rhs.sod
                    && *this != rhs));
    }
    /// @brief Greater comparison operator (takes double precision into account)
    /// @param[in] rhs Right-hand side to compare with
    /// @return Comparison result
    constexpr bool operator>(const InsTime_YDoySod& rhs) const
    {
        return !(*this <= rhs);
    }
};

/// The class is responsible for all time-related tasks
class InsTime
{
  public:
    /* ----------------------------- Public Members ----------------------------- */

    /// @brief List of all time systems
    enum TIME_SYSTEM
    {
        UTC,    ///< Coordinated Universal Time
        GPST,   ///< GPS Time
        GLNT,   ///< GLONASS Time (GLONASST)
        GST,    ///< Galileo System Time
        BDT,    ///< BeiDou Time
        QZSST,  ///< Quasi-Zenith Satellite System Time
        IRNSST, ///< Indian Regional Navigation Satellite System Time
    };

    /* ------------------------------ Constructors ------------------------------ */

    /// @brief Default Constructor
    constexpr InsTime() = default;

    /// @brief Constructor
    /// @param[in] mjd Time in Modified Julien Date [UTC]
    constexpr explicit InsTime(const InsTime_MJD& mjd)
        : mjd(mjd) {}

    /// @brief Constructor
    /// @param[in] jd Time in Julien Date [UTC]
    constexpr explicit InsTime(const InsTime_JD& jd)
        : mjd(jd.jd_day - InsTimeUtil::DIFF_MJD_TO_JD_DAYS, jd.jd_frac - InsTimeUtil::DIFF_MJD_TO_JD_FRAC) {}

    /// @brief Constructor
    /// @param[in] gpsWeekTow Time in GPS standard time [GPST]
    constexpr explicit InsTime(const InsTime_GPSweekTow& gpsWeekTow)
    {
        auto mjd_day = static_cast<int32_t>((gpsWeekTow.gpsCycle * InsTimeUtil::WEEKS_PER_GPS_CYCLE + gpsWeekTow.gpsWeek) * InsTimeUtil::DAYS_PER_WEEK
                                            + gcem::floor(gpsWeekTow.tow / InsTimeUtil::SECONDS_PER_DAY)
                                            + InsTimeUtil::DIFF_TO_6_1_1980_MJD);
        long double mjd_frac = gcem::fmod(gpsWeekTow.tow, InsTimeUtil::SECONDS_PER_DAY) / InsTimeUtil::SECONDS_PER_DAY;

        mjd_frac -= static_cast<long double>(leapGps2UTC(InsTime_MJD(mjd_day, mjd_frac))) / InsTimeUtil::SECONDS_PER_DAY; // from GPST to UTC

        mjd = InsTime_MJD(mjd_day, mjd_frac);
    }

    /// @brief Constructor
    /// @param[in] yearMonthDayHMS Time in Universal Time Coordinated [UTC]
    constexpr explicit InsTime(const InsTime_YMDHMS& yearMonthDayHMS)
    {
        auto a = static_cast<int32_t>((14 - yearMonthDayHMS.month) / static_cast<double>(InsTimeUtil::MONTHS_PER_YEAR));
        int32_t y = yearMonthDayHMS.year + 4800 - a;
        int32_t m = yearMonthDayHMS.month + InsTimeUtil::MONTHS_PER_YEAR * a - 3;

        auto jd_day = static_cast<int32_t>(yearMonthDayHMS.day
                                           + gcem::floor((153.0 * static_cast<double>(m) + 2.0) / 5.0)
                                           + y * InsTimeUtil::DAYS_PER_YEAR
                                           + gcem::floor(static_cast<double>(y) / 4.0)
                                           - gcem::floor(static_cast<double>(y) / 100.0)
                                           + gcem::floor(static_cast<double>(y) / 400.0)
                                           - 32045);
        auto jd_frac = (yearMonthDayHMS.sec
                        + static_cast<long double>(yearMonthDayHMS.min) * InsTimeUtil::SECONDS_PER_MINUTE
                        + static_cast<long double>(yearMonthDayHMS.hour - 12.0) * InsTimeUtil::SECONDS_PER_HOUR)
                       / InsTimeUtil::SECONDS_PER_DAY;

        mjd = InsTime(InsTime_JD(jd_day, jd_frac)).toMJD();
    }

    /// @brief Constructor
    /// @param[in] yearDoySod Time in GPS standard time [GPST]
    constexpr explicit InsTime(const InsTime_YDoySod& yearDoySod)
    {
        auto year = yearDoySod.year;
        auto doy = yearDoySod.doy;
        auto sod = yearDoySod.sod;

        int32_t month = 1;
        while (doy > InsTimeUtil::daysInMonth(month, year))
        {
            doy -= InsTimeUtil::daysInMonth(month, year);
            month++;
        }
        int32_t day = doy;

        sod -= static_cast<long double>(leapGps2UTC(InsTime_YMDHMS(year, month, day, 0, 0, sod)));

        mjd = InsTime(InsTime_YMDHMS(year, month, day, 0, 0, sod)).toMJD();
    }

    /// @brief Constructor
    /// @param[in] gpsCycle GPS cycle in GPS standard time [GPST]
    /// @param[in] gpsWeek GPS week in GPS standard time [GPST]
    /// @param[in] tow GPS time of week in GPS standard time [GPST]
    constexpr InsTime(int32_t gpsCycle, int32_t gpsWeek, long double tow)
        : InsTime(InsTime_GPSweekTow(gpsCycle, gpsWeek, tow)) {}

    /// @brief Constructor
    /// @param[in] year Year
    /// @param[in] month Month
    /// @param[in] day Day
    /// @param[in] hour Hour
    /// @param[in] min Minute
    /// @param[in] sec Second
    /// @param[in] timesys Time System in which the previous values are given in
    constexpr InsTime(uint16_t year, uint16_t month, uint16_t day, uint16_t hour, uint16_t min, long double sec, TIME_SYSTEM timesys = TIME_SYSTEM::UTC)
        : InsTime(InsTime_YMDHMS(year, month, day, hour, min, sec))
    {
        if (timesys == TIME_SYSTEM::GPST) // = GPS Time (UTC - leap_seconds)
        {
            auto leapSec = this->leapGps2UTC();
            *this -= std::chrono::duration<long double>(leapSec);
        }
        else if (timesys == TIME_SYSTEM::GLNT) // = GLONASS Time (UTC+ 3h)
        {
            constexpr auto utcDiff = 3 * InsTimeUtil::SECONDS_PER_HOUR;
            *this -= std::chrono::duration<long double>(utcDiff);
        }
        else if (timesys == TIME_SYSTEM::GST) // = GALILEO Time (~ GPS TIME_SYSTEM )
        {
            //  is synchronised with TAI with a nominal offset below 50 ns
            auto leapSec = this->leapGps2UTC(); // UTC = GST - 18
            *this -= std::chrono::duration<long double>(leapSec);
        }
        else if (timesys == TIME_SYSTEM::BDT) // = BeiDou Time (UTC)
        {
            // is synchronised with UTC within 100 ns<
        }
    }

    /// @brief Destructor
    ~InsTime() = default;
    /// @brief Copy constructor
    constexpr InsTime(const InsTime&) = default;
    /// @brief Move constructor
    constexpr InsTime(InsTime&&) = default;
    /// @brief Copy assignment operator
    constexpr InsTime& operator=(const InsTime&) = default;
    /// @brief Move assignment operator
    constexpr InsTime& operator=(InsTime&&) = default;

    /* ------------------------ Transformation functions ------------------------ */

    /// @brief Converts this time object into a different format
    /// @return InsTime_MJD structure of the this object
    [[nodiscard]] constexpr InsTime_MJD toMJD() const
    {
        return mjd;
    }

    /// @brief Converts this time object into a different format
    /// @return InsTime_JD structure of the this object
    [[nodiscard]] constexpr InsTime_JD toJD() const
    {
        auto jd_day = mjd.mjd_day + InsTimeUtil::DIFF_MJD_TO_JD_DAYS;
        auto jd_frac = mjd.mjd_frac + InsTimeUtil::DIFF_MJD_TO_JD_FRAC;

        return InsTime_JD(jd_day, jd_frac);
    }

    /// @brief Converts this time object into a different format
    /// @return InsTime_GPSweekTow structure of the this object
    [[nodiscard]] constexpr InsTime_GPSweekTow toGPSweekTow() const
    {
        InsTime_MJD mjd_leap = mjd;
        // Convert from UTC to GPST
        mjd_leap.mjd_frac += static_cast<long double>(leapGps2UTC(mjd_leap)) / static_cast<long double>(InsTimeUtil::SECONDS_PER_DAY);

        // Put everything in the time of week, as it gets splitted in InsTime_GPSweekTow constructor
        auto tow = static_cast<long double>((mjd_leap.mjd_day - InsTimeUtil::DIFF_TO_6_1_1980_MJD)) * InsTimeUtil::SECONDS_PER_DAY
                   + mjd_leap.mjd_frac * InsTimeUtil::SECONDS_PER_DAY;

        return InsTime_GPSweekTow(0, 0, tow);
    }

    /// @brief Converts this time object into a different format
    /// @return InsTime_YMDHMS structure of the this object
    [[nodiscard]] constexpr InsTime_YMDHMS toYMDHMS() const
    {
        // transform MJD to JD
        InsTime_JD jd = toJD();
        jd.jd_frac = jd.jd_frac + 0.5L;
        if (jd.jd_frac >= 1.0L)
        {
            jd.jd_day += 1;
            jd.jd_frac -= 1.0L;
        }
        // transform JD to YMDHMS
        double a = 32044.0 + jd.jd_day;
        double b = gcem::floor((4.0 * a + 3.0) / 146097.0);
        double c = a - gcem::floor((b * 146097.0) / 4.0);

        double d = gcem::floor((4.0 * c + 3.0) / 1461.0);
        double e = c - gcem::floor((1461.0 * d) / 4.0);
        double m = gcem::floor((5.0 * e + 2.0) / 153.0);

        auto day = static_cast<uint16_t>(e - gcem::floor((153.0 * m + 2.0) / 5.0) + 1);
        auto month = static_cast<uint16_t>(m + 3 - 12 * gcem::floor(m / 10.0));
        auto year = static_cast<uint16_t>(b * 100 + d - 4800.0 + gcem::floor(m / 10.0));

        long double sec = jd.jd_frac * InsTimeUtil::SECONDS_PER_DAY;

        return InsTime_YMDHMS(year, month, day, 0, 0, sec);
    }

    /// @brief Converts this time object into a different format
    /// @return InsTime_YDoySod structure of the this object
    [[nodiscard]] constexpr InsTime_YDoySod toYDoySod() const
    {
        InsTime_YMDHMS yearMonthDayHMS = toYMDHMS();

        auto year = yearMonthDayHMS.year;
        long double sod = static_cast<long double>(yearMonthDayHMS.hour * InsTimeUtil::SECONDS_PER_HOUR
                                                   + yearMonthDayHMS.min * InsTimeUtil::SECONDS_PER_MINUTE)
                          + yearMonthDayHMS.sec
                          + static_cast<long double>(leapGps2UTC());

        int32_t doy = 0;
        for (int32_t i = 1; i < yearMonthDayHMS.month; i++)
        {
            doy += InsTimeUtil::daysInMonth(i, year);
        }
        doy += yearMonthDayHMS.day;

        return InsTime_YDoySod(year, doy, sod);
    }

    /// @brief Returns the current time rounded to a full day (changes time to 0:0:0h UTC of current day)
    /// @return The rounded time object
    [[nodiscard]] constexpr InsTime toFullDay() const
    {
        return InsTime(InsTime_MJD(mjd.mjd_day, 0.0L));
    }

    /* ----------------------------- Leap functions ----------------------------- */

    /// @brief Returns the current number of leap seconds (offset GPST to UTC)
    /// @return Number of leap seconds
    [[nodiscard]] constexpr uint16_t leapGps2UTC() const
    {
        return leapGps2UTC(mjd);
    }

    /// @brief Returns the number of leap seconds (offset GPST to UTC) for the provided InsTime object
    /// @param[in] insTime Time point
    /// @return Number of leap seconds
    static constexpr uint16_t leapGps2UTC(const InsTime& insTime)
    {
        return leapGps2UTC(insTime.mjd);
    }

    /// @brief Returns the number of leap seconds (offset GPST to UTC) for the provided InsTime_GPSweekTow object
    /// @param[in] gpsWeekTow Time point
    /// @return Number of leap seconds
    static constexpr uint16_t leapGps2UTC(const InsTime_GPSweekTow& gpsWeekTow)
    {
        return leapGps2UTC(InsTime(gpsWeekTow).toMJD());
    }

    /// @brief Returns the number of leap seconds (offset GPST to UTC) for the provided InsTime_YDoySod object
    /// @param[in] yearDoySod Time point
    /// @return Number of leap seconds
    static constexpr uint16_t leapGps2UTC(const InsTime_YDoySod& yearDoySod)
    {
        return leapGps2UTC(InsTime(yearDoySod).toMJD());
    }

    /// @brief Returns the number of leap seconds (offset GPST to UTC) for the provided InsTime_YMDHMS object
    /// @param[in] yearMonthDayHMS Time point
    /// @return Number of leap seconds
    static constexpr uint16_t leapGps2UTC(const InsTime_YMDHMS& yearMonthDayHMS)
    {
        return leapGps2UTC(InsTime(yearMonthDayHMS).toMJD());
    }

    /// @brief Returns the number of leap seconds (offset GPST to UTC) for the provided InsTime_JD object
    /// @param[in] jd Time point
    /// @return Number of leap seconds
    static constexpr uint16_t leapGps2UTC(const InsTime_JD& jd)
    {
        return leapGps2UTC(InsTime(jd).toMJD());
    }

    // TODO: Remove with std::upper_bound, as soon as C++20 gets releases

    /// @brief Returns an iterator pointing to the first element in the range [first, last) that is greater than value, or last if no such element is found.
    /// @tparam ForwardIt Iterator of the type of the elements
    /// @tparam T Type of the elements
    /// @param[in] first Iterator to the first element to search
    /// @param[in] last Iterator to the last element to search
    /// @param[in] value Value to search for
    /// @return Iterator pointing to the first element in the range [first, last) that is greater than value, or last if no such element is found.
    template<class ForwardIt, class T>
    static constexpr ForwardIt upper_bound(ForwardIt first, ForwardIt last, const T& value)
    {
        ForwardIt it;
        typename std::iterator_traits<ForwardIt>::difference_type count;
        typename std::iterator_traits<ForwardIt>::difference_type step;
        count = std::distance(first, last);

        while (count > 0)
        {
            it = first;
            step = count / 2;
            std::advance(it, step);
            if (!(value < *it))
            {
                first = ++it;
                count -= step + 1;
            }
            else
            {
                count = step;
            }
        }
        return first;
    }

    /// @brief Returns the number of leap seconds (offset GPST to UTC) for the provided InsTime_MJD object
    /// @param[in] mjd_in Time point
    /// @return Number of leap seconds
    static constexpr uint16_t leapGps2UTC(const InsTime_MJD& mjd_in)
    {
        // TODO: Remove with std::upper_bound, as soon as C++20 gets releases
        return static_cast<uint16_t>(upper_bound(InsTimeUtil::GPS_LEAP_SEC_MJD.begin(), InsTimeUtil::GPS_LEAP_SEC_MJD.end(), mjd_in.mjd_day) - InsTimeUtil::GPS_LEAP_SEC_MJD.begin() - 1);
    }

    /// @brief Checks if the current time is a leap year
    /// @return True if the current time is a leap year, false otherwise
    [[nodiscard]] constexpr bool isLeapYear() const
    {
        return InsTimeUtil::isLeapYear(toYMDHMS().year);
    }

    /* --------------------- Comparison operator overloading -------------------- */

    /// @brief Equal comparison operator (takes double precision into account)
    /// @param[in] rhs Right-hand side to compare with
    /// @return Comparison result
    constexpr bool operator==(const InsTime& rhs) const { return mjd == rhs.mjd; }
    /// @brief Inequal comparison operator (takes double precision into account)
    /// @param[in] rhs Right-hand side to compare with
    /// @return Comparison result
    constexpr bool operator!=(const InsTime& rhs) const { return !(*this == rhs); }
    /// @brief Smaller or equal comparison operator (takes double precision into account)
    /// @param[in] rhs Right-hand side to compare with
    /// @return Comparison result
    constexpr bool operator<=(const InsTime& rhs) const { return *this < rhs || *this == rhs; }
    /// @brief Greater or equal comparison operator (takes double precision into account)
    /// @param[in] rhs Right-hand side to compare with
    /// @return Comparison result
    constexpr bool operator>=(const InsTime& rhs) const { return *this > rhs || *this == rhs; }
    /// @brief Smaller comparison operator (takes double precision into account)
    /// @param[in] rhs Right-hand side to compare with
    /// @return Comparison result
    constexpr bool operator<(const InsTime& rhs) const { return mjd < rhs.mjd; }
    /// @brief Greater comparison operator (takes double precision into account)
    /// @param[in] rhs Right-hand side to compare with
    /// @return Comparison result
    constexpr bool operator>(const InsTime& rhs) const { return !(*this <= rhs); }

    /* --------------------- Arithmetic operator overloading -------------------- */

    /// @brief Substracts 2 points in time
    /// @param[in] lhs The left hand side time point
    /// @param[in] rhs The right hand side time point
    /// @return Time difference in [seconds]
    constexpr friend std::chrono::duration<long double> operator-(const InsTime& lhs, const InsTime& rhs)
    {
        auto diffDays = lhs.mjd.mjd_day - rhs.mjd.mjd_day;
        auto diffFrac = lhs.mjd.mjd_frac - rhs.mjd.mjd_frac;
        long double diffSec = (diffFrac + static_cast<long double>(diffDays)) * InsTimeUtil::SECONDS_PER_DAY;
        return std::chrono::duration<long double>(diffSec);
    }

    /// @brief Adds a duration to this time point
    /// @param[in] duration The duration to add
    /// @return Reference to this object
    constexpr InsTime& operator+=(const std::chrono::duration<long double>& duration)
    {
        auto duration_mjd_frac = std::chrono::duration<long double, std::ratio<InsTimeUtil::SECONDS_PER_DAY>>(duration).count();
        this->mjd = InsTime_MJD(this->mjd.mjd_day,
                                this->mjd.mjd_frac + duration_mjd_frac);
        return *this;
    }

    /// @brief Substracts a duration to this time point
    /// @param[in] duration The duration to substract
    /// @return Reference to this object
    constexpr InsTime& operator-=(const std::chrono::duration<long double>& duration)
    {
        auto duration_mjd_frac = std::chrono::duration<long double, std::ratio<InsTimeUtil::SECONDS_PER_DAY>>(duration).count();
        this->mjd = InsTime_MJD(this->mjd.mjd_day,
                                this->mjd.mjd_frac - duration_mjd_frac);
        return *this;
    }

    /// @brief Adds a duration to a time point
    /// @param[in] time The left hand side time point
    /// @param[in] duration The right hand side duration
    /// @return Time point with the added duration
    constexpr friend InsTime operator+(const InsTime& time, const std::chrono::duration<long double>& duration)
    {
        return InsTime(time) += duration;
    }

    /// @brief Substracts a duration from a time point
    /// @param[in] time The left hand side time point
    /// @param[in] duration The right hand side duration
    /// @return Time point with the substracted duration
    constexpr friend InsTime operator-(const InsTime& time, const std::chrono::duration<long double>& duration)
    {
        return InsTime(time) -= duration;
    }

    /* ---------------------------- Utility Functions --------------------------- */

    /// @brief Checks if the Time object has a value
    [[nodiscard]] constexpr bool empty() const
    {
        return mjd.mjd_day == 0 && mjd.mjd_frac == 0.0L;
    }

    /// @brief Adds the difference [seconds] between toe (OBRIT-0 last element) and toc (ORBIT-0 first element) to the current time
    /// (Changes time, so that it corresponds to the time of GLONASS ORBIT last element)
    /// @param[in] UTC_sec Seconds in UTC time
    void MakeTimeFromGloOrbit(double UTC_sec)
    {
        auto ymdhms = toYMDHMS();
        // difference between toe (OBRIT-0 last element) and toc (ORBIT-0 first element) in seconds
        long double diff = gcem::fmod(static_cast<long double>(UTC_sec), InsTimeUtil::SECONDS_PER_DAY)
                           - (ymdhms.hour * InsTimeUtil::SECONDS_PER_HOUR
                              + ymdhms.min * InsTimeUtil::SECONDS_PER_MINUTE
                              + ymdhms.sec);
        // std::cout << "orbit diff " << diff << "\n";
        *this += std::chrono::duration<long double>(diff);
    }

    /// @brief Returns the current time as a string
    /// @return Time String with format YYYYMMDDHHMMSS
    [[nodiscard]] std::string GetStringOfDate() const;

    /// @brief Converts string into a time system (InsTime::TIME_SYSTEM)
    /// @param[in] sys String which defines the name of the time system
    /// @return The Time System
    static TIME_SYSTEM MakeTimeSysFromString(const std::string& sys);

    /// @brief Converts a time system (InsTime::TIME_SYSTEM) into a string
    /// @param[in] sys Time System we want the get the string for
    /// @return String representation of the time system
    static std::string MakeStringFromTimeSys(InsTime::TIME_SYSTEM sys);

  private:
    /// @brief Modified Julien Date of this InsTime object
    InsTime_MJD mjd{};
};

/// @brief Stream insertion operator overload
/// @param[in, out] os Output stream object to stream the time into
/// @param[in] mjd Object to print
/// @return Returns the output stream object in order to chain stream insertions
std::ostream& operator<<(std::ostream& os, const InsTime_MJD& mjd);
/// @brief Stream insertion operator overload
/// @param[in, out] os Output stream object to stream the time into
/// @param[in] jd Object to print
/// @return Returns the output stream object in order to chain stream insertions
std::ostream& operator<<(std::ostream& os, const InsTime_JD& jd);
/// @brief Stream insertion operator overload
/// @param[in, out] os Output stream object to stream the time into
/// @param[in] gpsWeekTow Object to print
/// @return Returns the output stream object in order to chain stream insertions
std::ostream& operator<<(std::ostream& os, const InsTime_GPSweekTow& gpsWeekTow);
/// @brief Stream insertion operator overload
/// @param[in, out] os Output stream object to stream the time into
/// @param[in] ymdhms Object to print
/// @return Returns the output stream object in order to chain stream insertions
std::ostream& operator<<(std::ostream& os, const InsTime_YMDHMS& ymdhms);
/// @brief Stream insertion operator overload
/// @param[in, out] os Output stream object to stream the time into
/// @param[in] yDoySod Object to print
/// @return Returns the output stream object in order to chain stream insertions
std::ostream& operator<<(std::ostream& os, const InsTime_YDoySod& yDoySod);
/// @brief Stream insertion operator overload
/// @param[in, out] os Output stream object to stream the time into
/// @param[in] insTime Object to print
/// @return Returns the output stream object in order to chain stream insertions
std::ostream& operator<<(std::ostream& os, const InsTime& insTime);

} // namespace NAV