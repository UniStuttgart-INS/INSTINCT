#include "InsTime.hpp"

#include <limits>
#include <cmath>

#include <iostream>
#include <vector>
#include <sstream>
#include <string>
#include <iomanip>

#include "util/Logger.hpp"

namespace NAV
{
// --------------------------- Constructors  ----------------------------------

InsTime::InsTime(const InsTime::InsTime_YDoySod& yearDoySod)
{
    constexpr std::array<int, 11> daysOfMonth = { 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334 };

    auto year = yearDoySod.year;
    auto doy = yearDoySod.doy;
    auto sod = yearDoySod.sod;

    int leapDay = 0;
    if (isLeapYear(year) && doy > daysOfMonth.at(1))
    {
        leapDay = 1;
    }

    const auto* low = std::lower_bound(daysOfMonth.begin(), daysOfMonth.end(), (doy - leapDay));
    auto month_number = static_cast<uint16_t>(low - daysOfMonth.begin() + 1); // Month number by substracting iterators to get position
    sod -= static_cast<long double>(leapGps2UTC(yearDoySod));                 // FIXME: Dauerschleife, da leapGps2UTC den Constructor selbst aufruft
    if (sod < 0.0L)
    {
        doy -= 1;
        sod += SEC_IN_DAY;
        if (doy < 1)
        {
            year -= 1;
            doy += 365;
            if (isLeapYear(year) && month_number > 2)
            {
                doy += 1;
            }
        }
    }

    InsTime_YMDHMS yearMonthDayHMS{};
    yearMonthDayHMS.year = year;
    yearMonthDayHMS.month = month_number;
    yearMonthDayHMS.day = static_cast<uint16_t>(static_cast<int>(doy) - leapDay - daysOfMonth.at(month_number - 2));
    yearMonthDayHMS.hour = static_cast<uint16_t>(std::floor(sod / 3600.0L));
    yearMonthDayHMS.min = static_cast<uint16_t>(std::floor(sod / 60.0L) - yearMonthDayHMS.hour * 60.0L);
    yearMonthDayHMS.sec = sod - (yearMonthDayHMS.hour * 3600.0L + yearMonthDayHMS.min * 60.0L);

    mjd = InsTime(yearMonthDayHMS).toMJD();
}

InsTime::InsTime(const InsTime::InsTime_YMDHMS& yearMonthDayHMS)
{
    InsTime_JD jd{};
    int a = static_cast<int>(std::floor((14 - yearMonthDayHMS.month) / 12.0));
    int y = yearMonthDayHMS.year + 4800 - a;
    int m = yearMonthDayHMS.month + 12 * a - 3;

    jd.jd_day = static_cast<unsigned int>(yearMonthDayHMS.day
                                          + std::floor((153.0 * static_cast<double>(m) + 2.0) / 5.0)
                                          + y * 365
                                          + std::floor(static_cast<double>(y) / 4.0)
                                          - std::floor(static_cast<double>(y) / 100.0)
                                          + std::floor(static_cast<double>(y) / 400.0)
                                          - 32045);
    jd.jd_frac = (yearMonthDayHMS.sec
                  + static_cast<long double>(yearMonthDayHMS.min) * 60.0L
                  + static_cast<long double>(static_cast<int>(yearMonthDayHMS.hour) - 12.0) * 3600.0L)
                 / SEC_IN_DAY;

    mjd = InsTime(jd).toMJD();
}

InsTime::InsTime(uint16_t gpsWeek, long double tow, uint16_t gpsCycle)
    : InsTime(InsTime_GPSweekTow(gpsWeek, tow, gpsCycle)) {}

InsTime::InsTime(uint16_t year, uint16_t month, uint16_t day, uint16_t hour, uint16_t min, long double sec, InsTime::TIME_SYSTEM timesys)
    : InsTime(InsTime_YMDHMS(year, month, day, hour, min, sec))
{
    if (timesys == TIME_SYSTEM::GPST) // = GPS Time (UTC - leap_seconds)
    {
        int leapSec = this->leapGps2UTC();
        this->addDiffSec(-leapSec);
    }
    else if (timesys == TIME_SYSTEM::GLNT) // = GLONASS Time (UTC+ 3h)
    {
        constexpr int leapSec = 10800;
        this->addDiffSec(-leapSec);
    }
    else if (timesys == TIME_SYSTEM::GST) // = GALILEO Time (~ GPS TIME_SYSTEM )
    {
        //  is synchronised with TAI with a nominal offset below 50 ns
        int leapSec = this->leapGps2UTC(); // UTC = GST - 18
        this->addDiffSec(-leapSec);
    }
    else if (timesys == TIME_SYSTEM::BDT) // = BeiDou Time (UTC)
    {
        // is synchronised with UTC within 100 ns<
    }
}

// ------------------------------------------------------------------------------

void InsTime::addDiffSec(long double diffSec)
{
    mjd.mjd_frac += diffSec / SEC_IN_DAY;

    if (mjd.mjd_frac >= 1.0L || mjd.mjd_frac < 0.0L)
    {
        mjd.mjd_day += static_cast<unsigned int>(std::floor(mjd.mjd_frac));
        mjd.mjd_frac -= std::floor(mjd.mjd_frac);
    }
}

long double InsTime::getTimeDiff(InsTime insTime) const
{
    int64_t diffDays = static_cast<int64_t>(mjd.mjd_day) - static_cast<int64_t>(insTime.toMJD().mjd_day);
    long double diffSec = (mjd.mjd_frac - insTime.toMJD().mjd_frac) * SEC_IN_DAY + static_cast<long double>(diffDays) * SEC_IN_DAY;
    return diffSec;
}

void InsTime::RoundToFullDay()
{
    // change time to 0:0:0h UTC of respective day
    // eg from 01.02.2019 3:14:55 UTC to 01.02.2019 00:00:00 UTC
    this->addDiffSec(-this->toMJD().mjd_frac * SEC_IN_DAY);
}

void InsTime::MakeTimeFromGloOrbit(double UTC_sec)
{
    auto ymdhms = toYMDHMS();
    // difference between toe (OBRIT-0 last element) and toc (ORBIT-0 first element) in seconds
    long double diff = std::fmod(static_cast<long double>(UTC_sec), SEC_IN_DAY)
                       - (ymdhms.hour * 3600.0L
                          + ymdhms.min * 60.0L
                          + ymdhms.sec);
    // std::cout << "orbit diff " << diff << "\n";
    this->addDiffSec(diff);
}

std::string InsTime::GetStringOfDate() const
{
    auto ymdhms = toYMDHMS();
    std::string s = std::to_string(ymdhms.year)
                    + std::to_string(ymdhms.month)
                    + std::to_string(ymdhms.day)
                    + std::to_string(ymdhms.hour)
                    + std::to_string(ymdhms.min)
                    + std::to_string(static_cast<int>(ymdhms.sec));
    return s;
}

InsTime::TIME_SYSTEM InsTime::MakeTimeSysFromString(const std::string& sys)
{
    if (sys == "UTC")
    {
        return InsTime::UTC;
    }
    if (sys == "GPST")
    {
        return InsTime::GPST;
    }
    if (sys == "GLNT")
    {
        return InsTime::GLNT;
    }
    if (sys == "BDT")
    {
        return InsTime::BDT;
    }
    if (sys == "QZSST")
    {
        return InsTime::QZSST;
    }
    if (sys == "IRNSST")
    {
        return InsTime::IRNSST;
    }

    LOG_ERROR("Unknown Time system {}. Time sys is set to UTC", sys);
    return InsTime::UTC;
}

std::string InsTime::MakeStringFromTimeSys(InsTime::TIME_SYSTEM sys)
{
    switch (sys)
    {
    case InsTime::TIME_SYSTEM::UTC:
        return "UTC";
    case InsTime::TIME_SYSTEM::GPST:
        return "GPST";
    case InsTime::TIME_SYSTEM::GLNT:
        return "GLNT";
    case InsTime::TIME_SYSTEM::BDT:
        return "BDT";
    case InsTime::TIME_SYSTEM::QZSST:
        return "QZSST";
    case InsTime::TIME_SYSTEM::IRNSST:
        return "IRNSST";

    default:
        LOG_ERROR("Unknown Time System with enum number: {}", sys);
        break;
    }

    return "unknown";
}

// ----------------------- Out of Class Definitions --------------------------

std::ostream& operator<<(std::ostream& os, const InsTime& insTime)
{
    return os << insTime.toYMDHMS() << "(" << insTime.toMJD() << ")";
}

std::ostream& operator<<(std::ostream& os, const InsTime::InsTime_YMDHMS& ymdhms)
{
    return os << ymdhms.year << "-"
              << ymdhms.month << "-"
              << ymdhms.day << " "
              << ymdhms.hour << ":"
              << ymdhms.min << ":"
              << std::setprecision(5) << ymdhms.sec;
}

std::ostream& operator<<(std::ostream& os, const InsTime::InsTime_MJD& mjd)
{
    return os << std::setprecision(19) << static_cast<long double>(mjd.mjd_day) + mjd.mjd_frac;
}

} // namespace NAV