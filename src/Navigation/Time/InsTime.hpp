// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

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

#include <fmt/ostream.h>
#include "gcem.hpp"

#include <nlohmann/json.hpp>
using json = nlohmann::json; ///< json namespace

#include "TimeSystem.hpp"

namespace NAV
{
/// @brief Utility Namespace for Time related tasks
namespace InsTimeUtil
{
constexpr int32_t END_OF_THE_CENTURY_MJD = 400000;   ///< Modified Julian Date of the end of the century (15.01.2954)
constexpr int32_t WEEKS_PER_GPS_CYCLE = 1024;        ///< Weeks per GPS cycle
constexpr int32_t DIFF_TO_6_1_1980_MJD = 44244;      ///< 06.01.1980 in Modified Julian Date
constexpr int32_t DIFF_BDT_WEEK_TO_GPST_WEEK = 1356; ///< BeiDou starts zero at 1-Jan-2006 and GPS starts 6-Jan-1980

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

/// Numerical precision/epsilon for 'long double' variables
/// - Linux/x64:   1e-19
/// - Windows/x64  1e-16
/// - Linux/armv8: 1e-34 (Epsilon reports too small precision. Real precision is only approx. 1e-17)
constexpr long double EPSILON = std::max(1e-17L, 10 * std::numeric_limits<long double>::epsilon());

/// Maps GPS leap seconds to a time: array<mjd_day>, index + 1 is amount of leap seconds
constexpr std::array<int32_t, 20> GPS_LEAP_SEC_MJD = {
    0,     // + 0 at 1 Jan 1980 and before
    44786, // + 1 at 1 Jul 1981 = diff UTC-TAI: 20
    45151, // + 2 at 1 Jul 1982 = diff UTC-TAI: 21
    45516, // + 3 at 1 Jul 1983 = diff UTC-TAI: 22
    46247, // + 4 at 1 Jul 1985 = diff UTC-TAI: 23
    47161, // + 5 at 1 Jan 1988 = diff UTC-TAI: 24
    47892, // + 6 at 1 Jan 1990 = diff UTC-TAI: 25
    48257, // + 7 at 1 Jan 1991 = diff UTC-TAI: 26
    48804, // + 8 at 1 Jul 1992 = diff UTC-TAI: 27
    49169, // + 9 at 1 Jul 1993 = diff UTC-TAI: 28
    49534, // +10 at 1 Jul 1994 = diff UTC-TAI: 29
    50083, // +11 at 1 Jan 1996 = diff UTC-TAI: 30
    50630, // +12 at 1 Jul 1997 = diff UTC-TAI: 31
    51179, // +13 at 1 Jan 1999 = diff UTC-TAI: 32
    53736, // +14 at 1 Jan 2006 = diff UTC-TAI: 33
    54832, // +15 at 1 Jan 2009 = diff UTC-TAI: 34
    56109, // +16 at 1 Jul 2012 = diff UTC-TAI: 35
    57204, // +17 at 1 Jul 2015 = diff UTC-TAI: 36
    57754, // +18 at 1 Jan 2017 = diff UTC-TAI: 37
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
    --month; // Make month 0 based
    if (month >= InsTimeUtil::MONTHS_PER_YEAR)
    {
        year += month / InsTimeUtil::MONTHS_PER_YEAR;
        month %= InsTimeUtil::MONTHS_PER_YEAR;
    }
    while (month < 0)
    {
        month += MONTHS_PER_YEAR;
        year--;
    }
    ++month; // Make month 1 based

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
        if (this->mjd_frac >= 1.0L)
        {
            this->mjd_day += static_cast<int32_t>(this->mjd_frac);
            this->mjd_frac -= static_cast<int32_t>(this->mjd_frac);
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
        if (mjd_day == rhs.mjd_day)
        {
            auto difference = gcem::abs(mjd_frac - rhs.mjd_frac);
            return difference <= InsTimeUtil::EPSILON;
        }
        if (auto diffDays = mjd_day - rhs.mjd_day;
            gcem::abs(diffDays) == 1)
        {
            auto difference = 1 + diffDays * (mjd_frac - rhs.mjd_frac);
            return difference <= InsTimeUtil::EPSILON;
        }
        return false;
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
        return (mjd_day < rhs.mjd_day || (mjd_day == rhs.mjd_day && mjd_frac < rhs.mjd_frac))
               && *this != rhs;
    }
    /// @brief Greater comparison operator (takes double precision into account)
    /// @param[in] rhs Right-hand side to compare with
    /// @return Comparison result
    constexpr bool operator>(const InsTime_MJD& rhs) const
    {
        return !(*this <= rhs);
    }

    /// @brief Converts the object into a readable string
    explicit operator std::string() const;
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
        if (this->jd_frac >= 1.0L)
        {
            this->jd_day += static_cast<int32_t>(this->jd_frac);
            this->jd_frac -= static_cast<int32_t>(this->jd_frac);
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
        if (jd_day == rhs.jd_day)
        {
            auto difference = gcem::abs(jd_frac - rhs.jd_frac);
            return difference <= InsTimeUtil::EPSILON;
        }
        if (auto diffDays = jd_day - rhs.jd_day;
            gcem::abs(diffDays) == 1)
        {
            auto difference = 1 + diffDays * (jd_frac - rhs.jd_frac);
            return difference <= InsTimeUtil::EPSILON;
        }
        return false;
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
        return (jd_day < rhs.jd_day || (jd_day == rhs.jd_day && jd_frac < rhs.jd_frac))
               && *this != rhs;
    }
    /// @brief Greater comparison operator (takes double precision into account)
    /// @param[in] rhs Right-hand side to compare with
    /// @return Comparison result
    constexpr bool operator>(const InsTime_JD& rhs) const
    {
        return !(*this <= rhs);
    }

    /// @brief Converts the object into a readable string
    explicit operator std::string() const;
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
        if (this->tow >= InsTimeUtil::SECONDS_PER_WEEK)
        {
            this->gpsWeek += static_cast<int32_t>(this->tow / InsTimeUtil::SECONDS_PER_WEEK);
            this->tow = static_cast<int64_t>(this->tow) % InsTimeUtil::SECONDS_PER_WEEK + (this->tow - gcem::floor(this->tow));
        }
        while (this->tow < 0.0L)
        {
            this->tow += InsTimeUtil::SECONDS_PER_WEEK;
            this->gpsWeek--;
        }

        if (this->gpsWeek >= InsTimeUtil::WEEKS_PER_GPS_CYCLE)
        {
            this->gpsCycle += this->gpsWeek / InsTimeUtil::WEEKS_PER_GPS_CYCLE;
            this->gpsWeek %= InsTimeUtil::WEEKS_PER_GPS_CYCLE;
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
        if (gpsCycle == rhs.gpsCycle && gpsWeek == rhs.gpsWeek)
        {
            return gcem::abs(tow - rhs.tow) <= InsTimeUtil::EPSILON;
        }
        if (auto diffWeeks = gpsCycle * InsTimeUtil::WEEKS_PER_GPS_CYCLE + gpsWeek - (rhs.gpsCycle * InsTimeUtil::WEEKS_PER_GPS_CYCLE + rhs.gpsWeek);
            gcem::abs(diffWeeks) == 1)
        {
            return gcem::abs(tow + diffWeeks * InsTimeUtil::SECONDS_PER_WEEK - rhs.tow) <= InsTimeUtil::EPSILON;
        }
        return false;
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
                || (gpsCycle == rhs.gpsCycle && gpsWeek < rhs.gpsWeek)
                || (gpsCycle == rhs.gpsCycle && gpsWeek == rhs.gpsWeek && tow < rhs.tow))
               && *this != rhs;
    }
    /// @brief Greater comparison operator (takes double precision into account)
    /// @param[in] rhs Right-hand side to compare with
    /// @return Comparison result
    constexpr bool operator>(const InsTime_GPSweekTow& rhs) const
    {
        return !(*this <= rhs);
    }

    /// @brief Converts the object into a readable string
    explicit operator std::string() const;
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
    /// @param[in] month Month in Universal Time Coordinated (1 = January) [UTC]
    /// @param[in] day Day in Universal Time Coordinated (1 = first day) [UTC]
    /// @param[in] hour Hour in Universal Time Coordinated [UTC]
    /// @param[in] min Minute in Universal Time Coordinated [UTC]
    /// @param[in] sec Second in Universal Time Coordinated [UTC]
    constexpr InsTime_YMDHMS(int32_t year, int32_t month, int32_t day, int32_t hour, int32_t min, long double sec)
        : year(year), month(month), day(day), hour(hour), min(min), sec(sec)
    {
        if (this->sec >= InsTimeUtil::SECONDS_PER_MINUTE)
        {
            this->min += static_cast<int32_t>(this->sec / InsTimeUtil::SECONDS_PER_MINUTE);
            this->sec = static_cast<int64_t>(this->sec) % InsTimeUtil::SECONDS_PER_MINUTE + (this->sec - gcem::floor(this->sec));
        }
        while (this->sec < 0.0L)
        {
            this->sec += InsTimeUtil::SECONDS_PER_MINUTE;
            this->min--;
        }

        if (this->min >= InsTimeUtil::MINUTES_PER_HOUR)
        {
            this->hour += this->min / InsTimeUtil::MINUTES_PER_HOUR;
            this->min %= InsTimeUtil::MINUTES_PER_HOUR;
        }
        while (this->min < 0)
        {
            this->min += InsTimeUtil::MINUTES_PER_HOUR;
            this->hour--;
        }

        if (this->hour >= InsTimeUtil::HOURS_PER_DAY)
        {
            this->day += this->hour / InsTimeUtil::HOURS_PER_DAY;
            this->hour %= InsTimeUtil::HOURS_PER_DAY;
        }
        while (this->hour < 0)
        {
            this->hour += InsTimeUtil::HOURS_PER_DAY;
            this->day--;
        }

        while (this->day >= InsTimeUtil::DAYS_PER_YEAR + static_cast<int32_t>(InsTimeUtil::isLeapYear(this->year)))
        {
            this->day -= InsTimeUtil::DAYS_PER_YEAR + static_cast<int32_t>(InsTimeUtil::isLeapYear(this->year));
            this->year++;
        }

        while (this->day > InsTimeUtil::daysInMonth(this->month, this->year))
        {
            this->day -= InsTimeUtil::daysInMonth(this->month, this->year);
            this->month++;
        }
        while (this->day < 1)
        {
            this->month--;
            this->day += InsTimeUtil::daysInMonth(this->month, this->year);
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
        if (year == rhs.year && month == rhs.month && day == rhs.day && hour == rhs.hour && min == rhs.min)
        {
            return gcem::abs(sec - rhs.sec) <= InsTimeUtil::EPSILON;
        }
        InsTime_YMDHMS this_plus = InsTime_YMDHMS(this->year, this->month, this->day, this->hour, this->min, this->sec + 10);
        InsTime_YMDHMS rhs_plus = InsTime_YMDHMS(rhs.year, rhs.month, rhs.day, rhs.hour, rhs.min, rhs.sec + 10);
        if (this_plus.year == rhs_plus.year && this_plus.month == rhs_plus.month && this_plus.day == rhs_plus.day && this_plus.hour == rhs_plus.hour && this_plus.min == rhs_plus.min)
        {
            return gcem::abs(this_plus.sec - rhs_plus.sec) <= InsTimeUtil::EPSILON;
        }
        return false;
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
                || (year == rhs.year && month < rhs.month)
                || (year == rhs.year && month == rhs.month && day < rhs.day)
                || (year == rhs.year && month == rhs.month && day == rhs.day && hour < rhs.hour)
                || (year == rhs.year && month == rhs.month && day == rhs.day && hour == rhs.hour && min < rhs.min)
                || (year == rhs.year && month == rhs.month && day == rhs.day && hour == rhs.hour && min == rhs.min && sec < rhs.sec))
               && *this != rhs;
    }
    /// @brief Greater comparison operator (takes double precision into account)
    /// @param[in] rhs Right-hand side to compare with
    /// @return Comparison result
    constexpr bool operator>(const InsTime_YMDHMS& rhs) const
    {
        return !(*this <= rhs);
    }

    /// @brief Converts the object into a readable string
    explicit operator std::string() const;
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
        if (this->sod >= InsTimeUtil::SECONDS_PER_DAY)
        {
            this->doy += static_cast<int32_t>(this->sod / InsTimeUtil::SECONDS_PER_DAY);
            this->sod = static_cast<int64_t>(this->sod) % InsTimeUtil::SECONDS_PER_DAY + (this->sod - gcem::floor(this->sod));
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
        if (year == rhs.year && doy == rhs.doy)
        {
            return gcem::abs(sod - rhs.sod) <= InsTimeUtil::EPSILON;
        }
        if (auto diffDays = year * InsTimeUtil::DAYS_PER_YEAR + doy - (rhs.year * InsTimeUtil::DAYS_PER_YEAR + rhs.doy);
            gcem::abs(diffDays) == 1)
        {
            auto difference = gcem::abs(sod + diffDays * InsTimeUtil::SECONDS_PER_DAY - rhs.sod);
            return difference <= InsTimeUtil::EPSILON;
        }
        return false;
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
                || (year == rhs.year && doy < rhs.doy)
                || (year == rhs.year && doy == rhs.doy && sod < rhs.sod))
               && *this != rhs;
    }
    /// @brief Greater comparison operator (takes double precision into account)
    /// @param[in] rhs Right-hand side to compare with
    /// @return Comparison result
    constexpr bool operator>(const InsTime_YDoySod& rhs) const
    {
        return !(*this <= rhs);
    }

    /// @brief Converts the object into a readable string
    explicit operator std::string() const;
};

/// The class is responsible for all time-related tasks
class InsTime
{
  public:
    /* ------------------------------ Constructors ------------------------------ */

    /// @brief Default Constructor
    constexpr InsTime() = default;

    /// @brief Constructor
    /// @param[in] mjd Time in Modified Julien Date
    /// @param[in] timesys Time System in which the previous values are given in
    constexpr explicit InsTime(const InsTime_MJD& mjd, TimeSystem timesys = UTC)
        : _mjd(mjd)
    {
        *this -= std::chrono::duration<long double>(differenceToUTC(timesys));
    }

    /// @brief Constructor
    /// @param[in] jd Time in Julien Date
    /// @param[in] timesys Time System in which the previous values are given in
    constexpr explicit InsTime(const InsTime_JD& jd, TimeSystem timesys = UTC)
        : _mjd(jd.jd_day - InsTimeUtil::DIFF_MJD_TO_JD_DAYS, jd.jd_frac - InsTimeUtil::DIFF_MJD_TO_JD_FRAC)
    {
        *this -= std::chrono::duration<long double>(differenceToUTC(timesys));
    }

    /// @brief Constructor
    /// @param[in] gpsWeekTow Time as week and time of week
    /// @param[in] timesys Time System in which the previous values are given in
    constexpr explicit InsTime(const InsTime_GPSweekTow& gpsWeekTow, TimeSystem timesys = GPST)
    {
        auto mjd_day = static_cast<int32_t>((gpsWeekTow.gpsCycle * InsTimeUtil::WEEKS_PER_GPS_CYCLE + gpsWeekTow.gpsWeek) * InsTimeUtil::DAYS_PER_WEEK
                                            + gcem::floor(gpsWeekTow.tow / InsTimeUtil::SECONDS_PER_DAY)
                                            + InsTimeUtil::DIFF_TO_6_1_1980_MJD);
        long double mjd_frac = gcem::fmod(gpsWeekTow.tow, InsTimeUtil::SECONDS_PER_DAY) / InsTimeUtil::SECONDS_PER_DAY;

        _mjd = InsTime_MJD(mjd_day, mjd_frac);

        *this -= std::chrono::duration<long double>(differenceToUTC(timesys));
    }

    /// @brief Constructor
    /// @param[in] yearMonthDayHMS Time in Universal Time Coordinated
    /// @param[in] timesys Time System in which the previous values are given in
    constexpr explicit InsTime(const InsTime_YMDHMS& yearMonthDayHMS, TimeSystem timesys = UTC)
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

        _mjd = InsTime(InsTime_JD(jd_day, jd_frac)).toMJD();

        *this -= std::chrono::duration<long double>(differenceToUTC(timesys));
    }

    /// @brief Constructor
    /// @param[in] yearDoySod Time as Year, day of year and seconds of day
    /// @param[in] timesys Time System in which the previous values are given in
    constexpr explicit InsTime(const InsTime_YDoySod& yearDoySod, TimeSystem timesys = UTC)
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

        _mjd = InsTime(InsTime_YMDHMS(year, month, doy, 0, 0, sod)).toMJD();

        *this -= std::chrono::duration<long double>(differenceToUTC(timesys));
    }

    /// @brief Constructor
    /// @param[in] gpsCycle GPS cycle in GPS standard time
    /// @param[in] gpsWeek GPS week in GPS standard time
    /// @param[in] tow GPS time of week in GPS standard time
    /// @param[in] timesys Time System in which the previous values are given in
    constexpr InsTime(int32_t gpsCycle, int32_t gpsWeek, long double tow, TimeSystem timesys = GPST)
        : InsTime(InsTime_GPSweekTow(gpsCycle, gpsWeek, tow), timesys) {}

    /// @brief Constructor
    /// @param[in] year Year
    /// @param[in] month Month (1 = January)
    /// @param[in] day Day (1 = first day)
    /// @param[in] hour Hour
    /// @param[in] min Minute
    /// @param[in] sec Second
    /// @param[in] timesys Time System in which the previous values are given in
    constexpr InsTime(int32_t year, int32_t month, int32_t day, int32_t hour, int32_t min, long double sec, TimeSystem timesys = UTC)
        : InsTime(InsTime_YMDHMS(year, month, day, hour, min, sec), timesys) {}

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
    /// @param timesys Time System in which the time should be given
    /// @return InsTime_MJD structure of the this object
    [[nodiscard]] constexpr InsTime_MJD toMJD(TimeSystem timesys = UTC) const
    {
        long double mjdFrac = _mjd.mjd_frac + static_cast<long double>(differenceToUTC(timesys)) / static_cast<long double>(InsTimeUtil::SECONDS_PER_DAY);
        return { _mjd.mjd_day, mjdFrac };
    }

    /// @brief Converts this time object into a different format
    /// @param timesys Time System in which the time should be given
    /// @return InsTime_JD structure of the this object
    [[nodiscard]] constexpr InsTime_JD toJD(TimeSystem timesys = UTC) const
    {
        auto jd_day = _mjd.mjd_day + InsTimeUtil::DIFF_MJD_TO_JD_DAYS;
        auto jd_frac = _mjd.mjd_frac + InsTimeUtil::DIFF_MJD_TO_JD_FRAC;

        jd_frac += static_cast<long double>(differenceToUTC(timesys)) / static_cast<long double>(InsTimeUtil::SECONDS_PER_DAY);

        return { jd_day, jd_frac };
    }

    /// @brief Converts this time object into a different format
    /// @param timesys Time System in which the time should be given
    /// @return InsTime_GPSweekTow structure of the this object
    [[nodiscard]] constexpr InsTime_GPSweekTow toGPSweekTow(TimeSystem timesys = GPST) const
    {
        InsTime_MJD mjd_leap = _mjd;
        // Convert from UTC to intended time system
        mjd_leap.mjd_frac += static_cast<long double>(differenceToUTC(timesys)) / static_cast<long double>(InsTimeUtil::SECONDS_PER_DAY);

        // Put everything in the time of week, as it gets splitted in InsTime_GPSweekTow constructor
        auto tow = static_cast<long double>((mjd_leap.mjd_day - InsTimeUtil::DIFF_TO_6_1_1980_MJD)) * InsTimeUtil::SECONDS_PER_DAY
                   + mjd_leap.mjd_frac * InsTimeUtil::SECONDS_PER_DAY;

        return { 0, 0, tow };
    }

    /// @brief Converts this time object into a different format
    /// @param timesys Time System in which the time should be given
    /// @return InsTime_YMDHMS structure of the this object
    [[nodiscard]] constexpr InsTime_YMDHMS toYMDHMS(TimeSystem timesys = UTC) const
    {
        // transform MJD to JD
        InsTime_JD jd = toJD();
        jd.jd_frac = jd.jd_frac + 0.5L;
        jd.jd_frac += static_cast<long double>(differenceToUTC(timesys)) / static_cast<long double>(InsTimeUtil::SECONDS_PER_DAY);
        while (jd.jd_frac >= 1.0L)
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

        return { year, month, day, 0, 0, sec };
    }

    /// @brief Converts this time object into a different format
    /// @param timesys Time System in which the time should be given
    /// @return InsTime_YDoySod structure of the this object
    [[nodiscard]] constexpr InsTime_YDoySod toYDoySod(TimeSystem timesys = UTC) const
    {
        InsTime_YMDHMS yearMonthDayHMS = toYMDHMS();

        auto year = yearMonthDayHMS.year;
        long double sod = static_cast<long double>(yearMonthDayHMS.hour * InsTimeUtil::SECONDS_PER_HOUR
                                                   + yearMonthDayHMS.min * InsTimeUtil::SECONDS_PER_MINUTE)
                          + yearMonthDayHMS.sec
                          + static_cast<long double>(differenceToUTC(timesys));

        int32_t doy = 0;
        for (int32_t i = 1; i < yearMonthDayHMS.month; i++)
        {
            doy += InsTimeUtil::daysInMonth(i, year);
        }
        doy += yearMonthDayHMS.day;

        return { year, doy, sod };
    }

    /// @brief Returns the current time rounded/cutted to a full day (changes time to 0:0:0h UTC of current day)
    /// @return The rounded/cutted time object
    [[nodiscard]] constexpr InsTime toFullDay() const
    {
        return InsTime(InsTime_MJD(_mjd.mjd_day, 0.0L));
    }

    /* ----------------------------- Leap functions ----------------------------- */

    /// @brief Returns the current number of leap seconds (offset GPST to UTC)
    /// @return Number of leap seconds
    [[nodiscard]] constexpr uint16_t leapGps2UTC() const
    {
        return leapGps2UTC(_mjd);
    }

    /// @brief Returns the number of leap seconds (offset GPST to UTC) for the provided InsTime object
    /// @param[in] insTime Time point
    /// @return Number of leap seconds
    static constexpr uint16_t leapGps2UTC(const InsTime& insTime)
    {
        return leapGps2UTC(insTime._mjd);
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

    /// @brief Returns the number of leap seconds (offset GPST to UTC) for the provided InsTime_MJD object
    /// @param[in] mjd_in Time point
    /// @return Number of leap seconds
    static constexpr uint16_t leapGps2UTC(const InsTime_MJD& mjd_in)
    {
        return static_cast<uint16_t>(std::upper_bound(InsTimeUtil::GPS_LEAP_SEC_MJD.begin(), InsTimeUtil::GPS_LEAP_SEC_MJD.end(), mjd_in.mjd_day) - InsTimeUtil::GPS_LEAP_SEC_MJD.begin() - 1);
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
    constexpr bool operator==(const InsTime& rhs) const { return _mjd == rhs._mjd; }
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
    constexpr bool operator<(const InsTime& rhs) const { return _mjd < rhs._mjd; }
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
        auto diffDays = lhs._mjd.mjd_day - rhs._mjd.mjd_day;
        auto diffFrac = lhs._mjd.mjd_frac - rhs._mjd.mjd_frac;
        long double diffSec = (diffFrac + static_cast<long double>(diffDays)) * InsTimeUtil::SECONDS_PER_DAY;
        return std::chrono::duration<long double>(diffSec);
    }

    /// @brief Adds a duration to this time point
    /// @param[in] duration The duration to add
    /// @return Reference to this object
    constexpr InsTime& operator+=(const std::chrono::duration<long double>& duration)
    {
        auto duration_mjd_frac = std::chrono::duration<long double, std::ratio<InsTimeUtil::SECONDS_PER_DAY>>(duration).count();
        this->_mjd = InsTime_MJD(this->_mjd.mjd_day,
                                 this->_mjd.mjd_frac + duration_mjd_frac);
        return *this;
    }

    /// @brief Substracts a duration to this time point
    /// @param[in] duration The duration to substract
    /// @return Reference to this object
    constexpr InsTime& operator-=(const std::chrono::duration<long double>& duration)
    {
        auto duration_mjd_frac = std::chrono::duration<long double, std::ratio<InsTimeUtil::SECONDS_PER_DAY>>(duration).count();
        this->_mjd = InsTime_MJD(this->_mjd.mjd_day,
                                 this->_mjd.mjd_frac - duration_mjd_frac);
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

    /// @brief Converts the object into a readable string
    explicit operator std::string() const;

    /// @brief Checks if the Time object has a value
    [[nodiscard]] constexpr bool empty() const
    {
        return _mjd.mjd_day == 0 && _mjd.mjd_frac == 0.0L;
    }

    /// @brief Resets the InsTime object
    void reset()
    {
        _mjd.mjd_day = 0;
        _mjd.mjd_frac = 0.0L;
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

    /// @brief Returns the time difference in [s] of a time system and UTC
    /// @param[in] timesys Time system to get the difference from UTC
    [[nodiscard]] constexpr int differenceToUTC(TimeSystem timesys) const
    {
        switch (TimeSystem_(timesys))
        {
        case GPST: // = GPS Time (UTC + leap_seconds)
            return this->leapGps2UTC();
        case GLNT: // = GLONASS Time (UTC+ 3h)
            return 3 * InsTimeUtil::SECONDS_PER_HOUR;
        case GST: // = GALILEO Time (~ GPS) (UTC = GST - 18) is synchronized with TAI with a nominal offset below 50 ns
            return this->leapGps2UTC();
        case QZSST:
            return 0; // TODO: Implement QZSST<->UTC time difference
        case IRNSST:
            return 0; // TODO: Implement IRNSST<->UTC time difference
        case BDT:     // = BeiDou Time (UTC) is synchronized with UTC within 100 ns<
            return this->leapGps2UTC() - 14;
        case UTC:
        case TimeSys_None:
            return 0;
        }
        return 0;
    }

  private:
    /// @brief Modified Julien Date of this InsTime object
    InsTime_MJD _mjd{};
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

/// @brief Converts the provided InsTime into a json object
/// @param[out] j Return Json object
/// @param[in] insTime Time to convert
void to_json(json& j, const InsTime& insTime);
/// @brief Converts the provided json object into an InsTime
/// @param[in] j Json object with the time values
/// @param[out] insTime Time to return
void from_json(const json& j, InsTime& insTime);

} // namespace NAV

#ifndef DOXYGEN_IGNORE

template<>
struct fmt::formatter<NAV::InsTime_MJD> : ostream_formatter
{};
template<>
struct fmt::formatter<NAV::InsTime_JD> : ostream_formatter
{};
template<>
struct fmt::formatter<NAV::InsTime_GPSweekTow> : ostream_formatter
{};
template<>
struct fmt::formatter<NAV::InsTime_YMDHMS> : ostream_formatter
{};
template<>
struct fmt::formatter<NAV::InsTime_YDoySod> : ostream_formatter
{};
template<>
struct fmt::formatter<NAV::InsTime> : ostream_formatter
{};

#endif