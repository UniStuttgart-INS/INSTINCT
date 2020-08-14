#include "InsTime.hpp"

#include <limits>
#include <cmath>

#include <iostream>
#include <vector>
#include <map>
#include <sstream>
#include <string>
#include <iomanip>

#include "util/Logger.hpp"

namespace NAV
{
constexpr InsTime::InsTime()
    : mjd(endOfTheCentury, 0.0L) {}

InsTime::InsTime(const InsTime::InsTime_GPSweekTow& gpsWeekTow)
    : mjd(GPSweekTow2MJD(gpsWeekTow)) {}
InsTime& InsTime::operator=(const InsTime::InsTime_GPSweekTow& gpsWeekTow)
{
    mjd = GPSweekTow2MJD(gpsWeekTow);
    return *this;
}

InsTime::InsTime(const InsTime::InsTime_MJD& mjd_in)
    : mjd(mjd_in) {}

InsTime::InsTime(const InsTime::InsTime_JD& jd_in)
    : mjd(JD2MJD(jd_in)) {}

InsTime::InsTime(const InsTime::InsTime_YDoySod& yearDoySod)
    : mjd(YDoySod2MJD(yearDoySod)) {}

InsTime::InsTime(const InsTime::InsTime_YMDHMS& yearMonthDayHMS)
    : mjd(YMDHMS2MJD(yearMonthDayHMS)) {}
InsTime& InsTime::operator=(const InsTime::InsTime_YMDHMS& yearMonthDayHMS)
{
    mjd = YMDHMS2MJD(yearMonthDayHMS);
    return *this;
}

InsTime::InsTime(uint16_t gpsWeek, long double tow, uint16_t gpsCycle)
    : InsTime(InsTime_GPSweekTow(gpsWeek, tow, gpsCycle)) {}

InsTime::InsTime(uint16_t year, uint16_t month, uint16_t day, uint16_t hour, uint16_t min, long double sec, InsTime::TIME timesys)
    : mjd(YMDHMS2MJD(InsTime_YMDHMS(year, month, day, hour, min, sec)))
{
    if (timesys == TIME::GPST) // = GPS Time (UTC - leap_seconds)
    {
        int leapSec = this->leapGps2UTC();
        this->addDiffSec(-leapSec);
    }
    else if (timesys == TIME::GLNT) // = GLONASS Time (UTC+ 3h)
    {
        constexpr int leapSec = 10800;
        this->addDiffSec(-leapSec);
    }
    else if (timesys == TIME::GST) // = GALILEO Time (~ GPS TIME )
    {
        //  is synchronised with TAI with a nominal offset below 50 ns
        int leapSec = this->leapGps2UTC(); // UTC = GST - 18
        this->addDiffSec(-leapSec);
    }
    else if (timesys == TIME::BDT) // = BeiDou Time (UTC)
    {
        // is synchronised with UTC within 100 ns<
    }
}

// update, change time
void InsTime::SetInsTime(InsTime t_in)
{
    mjd = t_in.GetMJD();
}
void InsTime::SetInsTime(InsTime::InsTime_GPSweekTow gpsWeekTow)
{
    mjd = GPSweekTow2MJD(gpsWeekTow);
}
void InsTime::SetInsTime(InsTime::InsTime_MJD mjd_in)
{
    mjd = mjd_in;
}
void InsTime::SetInsTime(InsTime::InsTime_JD jd)
{
    mjd = JD2MJD(jd);
}
void InsTime::SetInsTime(InsTime::InsTime_YDoySod yearDoySod)
{
    mjd = YDoySod2MJD(yearDoySod);
}

void InsTime::SetInsTime(InsTime::InsTime_YMDHMS yearMonthDayHMS)
{
    mjd = YMDHMS2MJD(yearMonthDayHMS);
}
void InsTime::SetInsTime(uint16_t gpsWeek, long double tow, uint16_t gpsCycle)
{
    InsTime_GPSweekTow gpsWeekTow = { gpsWeek, tow, gpsCycle };
    mjd = GPSweekTow2MJD(gpsWeekTow);
}
void InsTime::SetInsTime(uint16_t year, uint16_t month, uint16_t day, uint16_t hour, uint16_t min, long double sec, InsTime::TIME timesys)
{
    InsTime::InsTime_YMDHMS yearMonthDayHMS(year, month, day, hour, min, sec);
    mjd = YMDHMS2MJD(yearMonthDayHMS);

    if (timesys == TIME::GPST) // = GPS Time (UTC - leap_seconds)
    {
        int leapSec = this->leapGps2UTC();
        this->addDiffSec(-leapSec);
    }
    else if (timesys == TIME::GLNT) // = GLONASS Time GLNT (UTC+ 3h)
    {
        constexpr int leapSec = 10800;
        this->addDiffSec(-leapSec);
    }
    else if (timesys == TIME::GST) // = GALILEO Time (TAI)
    {
        //  is synchronised with TAI with a nominal offset below 50 ns
        int leapSec = this->leapGps2UTC();
        this->addDiffSec(-leapSec);
    }
    // TODO: Implement time systems
    // else if (timesys == TIME::BDT) // = BeiDou Time (UTC)
    // {
    //     // is synchronised with UTC within 100 ns
    // }
    // else if (timesys == TIME::QZSST) // = QZSS Time (???)
    // {
    //     // implementation missing
    // }
    // else if (timesys == TIME::IRNSST) // = IRNSS Time (???)
    // {
    //     // implementation missing
    // }
}

// accessors, get time
InsTime::InsTime_GPSweekTow InsTime::GetGPSTime() const
{
    return MJD2GPSweekTow(mjd);
}
InsTime::InsTime_MJD InsTime::GetMJD() const
{
    return mjd;
}
InsTime::InsTime_JD InsTime::GetJD() const
{
    return MJD2JD(mjd);
}
InsTime::InsTime_YDoySod InsTime::GetYDoySod() const
{
    return MJD2YDoySod(mjd);
}
InsTime::InsTime_YMDHMS InsTime::GetYMDHMS() const
{
    return MJD2YMDHMS(mjd);
}

// Transformation Functions
InsTime::InsTime_MJD InsTime::GPSweekTow2MJD(InsTime::InsTime_GPSweekTow gpsWeeekTow) // Transformation GPSweekTow to MJD (GPST to UTC)
{
    constexpr int DAYS_PER_GPS_CYCLE = 1024;
    constexpr int DAYS_PER_WEEK = 7;

    constexpr int DIFF_TO_6_1_1980 = 44244;

    InsTime_MJD mjd{};
    mjd.mjd_day = static_cast<unsigned int>((gpsWeeekTow.gpsCycle * DAYS_PER_GPS_CYCLE + gpsWeeekTow.gpsWeek) * DAYS_PER_WEEK
                                            + std::floor(gpsWeeekTow.tow / SEC_IN_DAY) + DIFF_TO_6_1_1980);
    mjd.mjd_frac = fmodl(gpsWeeekTow.tow, SEC_IN_DAY) / SEC_IN_DAY;
    mjd.mjd_frac -= static_cast<long double>(leapGps2UTC(mjd)) / SEC_IN_DAY; // from GPST to UTC
    if (mjd.mjd_frac < 0.0L)
    {
        mjd.mjd_day -= 1;
        mjd.mjd_frac += 1.0L;
    }
    return mjd;
}

InsTime::InsTime_MJD InsTime::YDoySod2MJD(InsTime::InsTime_YDoySod yearDoySod) // Transformation YDoySod to MJD (GPST to UTC)
{
    InsTime_YMDHMS yearMonthDayHMS = YDoySod2YMDHMS(yearDoySod);
    InsTime_MJD mjd = YMDHMS2MJD(yearMonthDayHMS);
    return mjd;
}

InsTime::InsTime_MJD InsTime::YMDHMS2MJD(InsTime::InsTime_YMDHMS yearMonthDayHMS) // Transformation YMDHMS to MJD (UTC to UTC)
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
    return JD2MJD(jd);
}

InsTime::InsTime_MJD InsTime::JD2MJD(InsTime::InsTime_JD jd) // Transformation JD to MJD (UTC to UTC)
{
    InsTime_MJD mjd{};
    mjd.mjd_day = jd.jd_day - 2400000;
    mjd.mjd_frac = jd.jd_frac - 0.5L;
    if (mjd.mjd_frac < 0.0L)
    {
        mjd.mjd_day -= 1;
        mjd.mjd_frac += 1.0L;
    }
    return mjd;
}

InsTime::InsTime_GPSweekTow InsTime::MJD2GPSweekTow(InsTime::InsTime_MJD mjd) // Transformation MJD to GPSweekTow (UTC to GPST)
{
    mjd.mjd_frac += static_cast<long double>(leapGps2UTC(mjd)) / SEC_IN_DAY; // from UTC to GPST
    if (mjd.mjd_frac < 0.0L)
    {
        mjd.mjd_day -= 1;
        mjd.mjd_frac += 1.0L;
    }
    InsTime_GPSweekTow gpsWeekTow{};
    gpsWeekTow.gpsCycle = static_cast<uint16_t>(std::floor(static_cast<double>(mjd.mjd_day - 44244) / (1024.0 * 7.0)));
    gpsWeekTow.gpsWeek = static_cast<uint16_t>(std::floor((mjd.mjd_day - 44244 - gpsWeekTow.gpsCycle * 1024 * 7) / 7.0)); // number of full weeks
    gpsWeekTow.tow = static_cast<long double>((mjd.mjd_day - 44244) % 7) * SEC_IN_DAY + mjd.mjd_frac * SEC_IN_DAY;        // remaining seconds
    return gpsWeekTow;
}

InsTime::InsTime_YDoySod InsTime::MJD2YDoySod(InsTime::InsTime_MJD mjd) // Transformation MJD to YDoySod (UTC to GPST)
{
    InsTime_YDoySod yearDoySod{};
    InsTime_YMDHMS yearMonthDayHMS = MJD2YMDHMS(mjd);
    yearDoySod = YMDHMS2YDoySod(yearMonthDayHMS);
    return yearDoySod;
}

InsTime::InsTime_YMDHMS InsTime::MJD2YMDHMS(InsTime::InsTime_MJD mjd) // Transformation MJD to YMDHMS (UTC to UTC)
{
    // transform MJD to JD
    InsTime_JD jd = MJD2JD(mjd);
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

    auto day = static_cast<uint16_t>(e - std::floor((153.0 * m + 2.0) / 5.0) + 1);
    auto month = static_cast<uint16_t>(m + 3 - 12 * std::floor(m / 10.0));
    auto year = static_cast<uint16_t>(b * 100 + d - 4800.0 + std::floor(m / 10.0));

    auto hour = static_cast<uint16_t>(std::floor(jd.jd_frac * 24.0L));
    auto min = static_cast<uint16_t>(std::floor(jd.jd_frac * 24.0L * 60.0L) - hour * 60.0L);
    long double sec = jd.jd_frac * 24.0L * 3600.0L - static_cast<long double>(hour * 3600.0 + min * 60.0);

    return InsTime_YMDHMS(year, month, day, hour, min, sec);
}

InsTime::InsTime_JD InsTime::MJD2JD(InsTime::InsTime_MJD mjd) // Transformation MJD to JD (UTC to UTC)
{
    InsTime_JD jd{};
    jd.jd_day = mjd.mjd_day + 2400000;
    jd.jd_frac = mjd.mjd_frac + 0.5L;
    if (jd.jd_frac >= 1.0L)
    {
        jd.jd_day++;
        jd.jd_frac--;
    }
    return jd;
}

InsTime::InsTime_YMDHMS InsTime::YDoySod2YMDHMS(InsTime::InsTime_YDoySod yearDoySod) // Transformation YDoySod to YMDHMS (GPST to UTC)
{
    int leapDay = 0;
    if (isLeapYear(yearDoySod.year) && yearDoySod.doy > 59)
    {
        leapDay = 1;
    }

    constexpr std::array<int, 11> daysOfMonth = { 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334 };
    const auto* low = std::lower_bound(daysOfMonth.begin(), daysOfMonth.end(), (yearDoySod.doy - leapDay));
    auto month_number = static_cast<uint16_t>(low - daysOfMonth.begin() + 1); // Month number by substracting iterators to get position
    yearDoySod.sod -= static_cast<long double>(leapGps2UTC(yearDoySod));
    if (yearDoySod.sod < 0.0L)
    {
        yearDoySod.doy -= 1;
        yearDoySod.sod += SEC_IN_DAY;
        if (yearDoySod.doy < 1)
        {
            yearDoySod.year -= 1;
            yearDoySod.doy += 365;
            if (isLeapYear(yearDoySod.year) && month_number > 2)
            {
                yearDoySod.doy += 1;
            }
        }
    }

    InsTime_YMDHMS yearMonthDayHMS{};
    yearMonthDayHMS.year = yearDoySod.year;
    yearMonthDayHMS.month = month_number;
    yearMonthDayHMS.day = static_cast<uint16_t>(static_cast<int>(yearDoySod.doy) - leapDay - daysOfMonth.at(month_number - 2));
    yearMonthDayHMS.hour = static_cast<uint16_t>(std::floor(yearDoySod.sod / 3600.0L));
    yearMonthDayHMS.min = static_cast<uint16_t>(std::floor(yearDoySod.sod / 60.0L) - yearMonthDayHMS.hour * 60.0L);
    yearMonthDayHMS.sec = yearDoySod.sod - (yearMonthDayHMS.hour * 3600.0L + yearMonthDayHMS.min * 60.0L);

    return yearMonthDayHMS;
}

InsTime::InsTime_YDoySod InsTime::YMDHMS2YDoySod(InsTime::InsTime_YMDHMS yearMonthDayHMS) // Transformation YMDHMS to YDoySod (UTC to GPST)
{
    constexpr std::array<uint16_t, 11> daysOfMonth{ 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334 };

    InsTime_YDoySod yearDoySod{};
    yearDoySod.year = yearMonthDayHMS.year;
    yearDoySod.sod = static_cast<long double>(yearMonthDayHMS.hour * 3600.0 + yearMonthDayHMS.min * 60.0)
                     + yearMonthDayHMS.sec
                     + static_cast<long double>(leapGps2UTC(yearMonthDayHMS));

    if (yearMonthDayHMS.month == 1)
    {
        yearDoySod.doy = yearMonthDayHMS.day;
    }
    else if (yearMonthDayHMS.month == 2)
    {
        yearDoySod.doy = daysOfMonth.at(0) + yearMonthDayHMS.day;
    }
    else // for march to december
    {
        if (isLeapYear(yearMonthDayHMS.year))
        {
            yearDoySod.doy = 1U + daysOfMonth.at(yearMonthDayHMS.month - 2) + yearMonthDayHMS.day;
        }
        else
        {
            yearDoySod.doy = daysOfMonth.at(yearMonthDayHMS.month - 2) + yearMonthDayHMS.day;
        }
    }
    if (yearDoySod.sod >= SEC_IN_DAY)
    {
        yearDoySod.doy++;
        yearDoySod.sod -= SEC_IN_DAY;
        if (yearDoySod.doy == 366 && !isLeapYear(yearMonthDayHMS.year))
        {
            yearDoySod.year--;
            yearDoySod.doy = 365;
        }
    }
    return yearDoySod;
}

void InsTime::RoundToFullDay()
{
    // change time to 0:0:0h UTC of respective day
    // eg from 01.02.2019 3:14:55 UTC to 01.02.2019 00:00:00 UTC
    this->addDiffSec(-this->GetMJD().mjd_frac * SEC_IN_DAY);
}
void InsTime::addDiffSec(long double diffSec)
{
    mjd.mjd_frac += diffSec / SEC_IN_DAY;

    if (mjd.mjd_frac >= 1.0L || mjd.mjd_frac < 0.0L)
    {
        mjd.mjd_day += static_cast<unsigned int>(std::floor(mjd.mjd_frac));
        mjd.mjd_frac -= std::floor(mjd.mjd_frac);
    }
}
void InsTime::MakeTimeFromGloOrbit(double UTC_sec)
{
    auto ymdhms = GetYMDHMS();
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
    auto ymdhms = GetYMDHMS();
    std::string s = std::to_string(ymdhms.year)
                    + std::to_string(ymdhms.month)
                    + std::to_string(ymdhms.day)
                    + std::to_string(ymdhms.hour)
                    + std::to_string(ymdhms.min)
                    + std::to_string(static_cast<int>(ymdhms.sec));
    return s;
}

void InsTime::MakeTimeSysFromString(const std::string& sys, InsTime::TIME* timesys)
{
    if (sys == "UTC")
    {
        *timesys = InsTime::UTC;
    }
    else if (sys == "GPST")
    {
        *timesys = InsTime::GPST;
    }
    else if (sys == "GLNT")
    {
        *timesys = InsTime::GLNT;
    }
    else if (sys == "BDT")
    {
        *timesys = InsTime::BDT;
    }
    else if (sys == "QZSST")
    {
        *timesys = InsTime::QZSST;
    }
    else if (sys == "IRNSST")
    {
        *timesys = InsTime::IRNSST;
    }
    else
    {
        *timesys = InsTime::UTC;
        std::cout << "Unknown Time system " << sys << ". Time sys is set to UTC\n";
    }
}

std::string InsTime::MakeStringFromTimeSys(InsTime::TIME sys)
{
    switch (sys)
    {
    case InsTime::TIME::UTC:
        return "UTC";
    case InsTime::TIME::GPST:
        return "GPST";
    case InsTime::TIME::GLNT:
        return "GLNT";
    case InsTime::TIME::BDT:
        return "BDT";
    case InsTime::TIME::QZSST:
        return "QZSST";
    case InsTime::TIME::IRNSST:
        return "IRNSST";

    default:
        LOG_ERROR("Unknown Time System with enum number: {}", sys);
        break;
    }

    return "unknown";
}

// leap seconds

// NOLINTNEXTLINE(cert-err58-cpp)
const std::map<unsigned int, uint16_t> InsTime::GpsLeapSec = {
    { 44786, 1 },  // 1 Jul 1981  //diff UTC-TAI: 20
    { 45151, 2 },  // 1 Jul 1982  //diff UTC-TAI: 21
    { 45516, 3 },  // 1 Jul 1983  //diff UTC-TAI: 22
    { 46247, 4 },  // 1 Jul 1985  //diff UTC-TAI: 23
    { 47161, 5 },  // 1 Jan 1988  //diff UTC-TAI: 24
    { 47892, 6 },  // 1 Jan 1990  //diff UTC-TAI: 25
    { 48257, 7 },  // 1 Jan 1991  //diff UTC-TAI: 26
    { 48804, 8 },  // 1 Jul 1992  //diff UTC-TAI: 27
    { 49169, 9 },  // 1 Jul 1993  //diff UTC-TAI: 28
    { 49534, 10 }, // 1 Jul 1994  //diff UTC-TAI: 29
    { 50083, 11 }, // 1 Jan 1996  //diff UTC-TAI: 30
    { 50630, 12 }, // 1 Jul 1997  //diff UTC-TAI: 31
    { 51179, 13 }, // 1 Jan 1999  //diff UTC-TAI: 32
    { 53736, 14 }, // 1 Jan 2006  //diff UTC-TAI: 33
    { 54832, 15 }, // 1 Jan 2009  //diff UTC-TAI: 34
    { 56109, 16 }, // 1 Jul 2012  //diff UTC-TAI: 35
    { 57204, 17 }, // 1 Jul 2015  //diff UTC-TAI: 36
    { 57754, 18 }, // 1 Jan 2017  //diff UTC-TAI: 37
    { 99999, 19 }  // future
};

uint16_t InsTime::leapGps2UTC() const
{
    return leapGps2UTC(mjd);
}
uint16_t InsTime::leapGps2UTC(InsTime insTime)
{
    return leapGps2UTC(insTime.mjd);
}
uint16_t InsTime::leapGps2UTC(InsTime::InsTime_YDoySod yearDoySod)
{
    return leapGps2UTC(YDoySod2MJD(yearDoySod));
}
uint16_t InsTime::leapGps2UTC(InsTime::InsTime_GPSweekTow gpsWeekTow)
{
    return leapGps2UTC(GPSweekTow2MJD(gpsWeekTow));
}
uint16_t InsTime::leapGps2UTC(InsTime::InsTime_YMDHMS yearMonthDayHMS)
{
    return leapGps2UTC(YMDHMS2MJD(yearMonthDayHMS));
}
uint16_t InsTime::leapGps2UTC(InsTime::InsTime_MJD mjd_in)
{
    //std::map<unsigned int,uint16_t>::iterator pLeapSec;
    auto pLeapSec = InsTime::GpsLeapSec.upper_bound(mjd_in.mjd_day);
    //std::cout << "leap sec " <<pLeapSec->first << " is " <<pLeapSec->second -1 << "sec for mjd: "<< mjd.mjd_day << "\n";
    return pLeapSec->second - 1;
}

// Function for time comparision: bigger/equal
bool InsTime::isBigger(InsTime insTime) const
{
    return isBigger(insTime.mjd);
}

bool InsTime::isBigger(InsTime::InsTime_YDoySod yearDoySod) const
{
    return isBigger(YDoySod2MJD(yearDoySod));
}
bool InsTime::isBigger(InsTime::InsTime_GPSweekTow gpsWeekTow) const
{
    return isBigger(GPSweekTow2MJD(gpsWeekTow));
}
bool InsTime::isBigger(InsTime::InsTime_YMDHMS yearMonthDayHMS) const
{
    return isBigger(YMDHMS2MJD(yearMonthDayHMS));
}
bool InsTime::isBigger(InsTime::InsTime_MJD mjd_compare) const // is true if mjd_compare is bigger (later)
{
    bool is_bigger = false;
    if (mjd_compare.mjd_day > mjd.mjd_day)
    {
        is_bigger = true;
    }
    else if (mjd_compare.mjd_day == mjd.mjd_day && mjd_compare.mjd_frac > mjd.mjd_frac)
    {
        is_bigger = true;
    }
    return is_bigger;
}
bool InsTime::isEqual(InsTime insTime) const
{
    return isEqual(insTime.mjd);
}
bool InsTime::isEqual(InsTime::InsTime_YDoySod yearDoySod) const
{
    return isEqual(YDoySod2MJD(yearDoySod));
}
bool InsTime::isEqual(InsTime::InsTime_GPSweekTow gpsWeekTow) const
{
    return isEqual(GPSweekTow2MJD(gpsWeekTow));
}
bool InsTime::isEqual(InsTime::InsTime_YMDHMS yearMonthDayHMS) const
{
    return isEqual(YMDHMS2MJD(yearMonthDayHMS));
}
bool InsTime::isEqual(InsTime::InsTime_MJD mjd_compare) const
{
    bool is_equal = false;
    if (mjd_compare.mjd_day == mjd.mjd_day && mjd_compare.mjd_frac == mjd.mjd_frac)
    {
        is_equal = true; // frage auf wie viele nachkommastellen genau? ??
    }
    return is_equal;
}

long double InsTime::getTimeDiff(InsTime insTime) const
{
    int64_t diffDays = static_cast<int64_t>(mjd.mjd_day) - static_cast<int64_t>(insTime.GetMJD().mjd_day);
    long double diffSec = (mjd.mjd_frac - insTime.GetMJD().mjd_frac) * SEC_IN_DAY + static_cast<long double>(diffDays) * SEC_IN_DAY;
    return diffSec;
}

long double InsTime::hms2sec(int hour, int min, long double sec)
{
    return static_cast<long double>(hour) * 3600.0L
           + static_cast<long double>(min) * 60.0L
           + sec;
}

bool InsTime::isLeapYear(uint16_t year)
{
    bool leap = (((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0));
    return leap;
}
bool InsTime::isLeapYear() const
{
    uint16_t year = GetYDoySod().year;
    bool leap = (((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0));
    return leap;
}

std::string InsTime::outputTime() const
{
    InsTime_YMDHMS time = MJD2YMDHMS(mjd);
    std::stringstream ss;
    ss << "YMDHMS "
       << time.year << " "
       << time.month << " "
       << time.day << " "
       << time.hour << " "
       << time.min << " "
       << std::setprecision(5) << std::round(time.sec * 1000.0L) / 1000.0L;
    return ss.str();
}
std::string InsTime::outputTowTime() const
{
    //InsTime_GPSweekTow time = MJD2GPSweekTow(mjd);
    std::stringstream ss;
    ss << "tow " << std::round(GetGPSTime().tow * 10.0L) / 10.0L;
    return ss.str();
}
std::string InsTime::outputTime(const InsTime& time)
{
    std::stringstream ss;
    ss << "YMDHMS time "
       << time.GetYMDHMS().year << " "
       << time.GetYMDHMS().month << " "
       << time.GetYMDHMS().day << " "
       << time.GetYMDHMS().hour << " "
       << time.GetYMDHMS().min << " "
       << std::setprecision(5) << time.GetYMDHMS().sec;
    return ss.str();
}

// overload operator
bool InsTime::operator<(const InsTime& mjd_compare) const
{
    return (mjd_compare.mjd.mjd_day > mjd.mjd_day
            || (mjd_compare.mjd.mjd_day == mjd.mjd_day && mjd_compare.mjd.mjd_frac > mjd.mjd_frac));
}
bool InsTime::operator>(const InsTime& mjd_compare) const
{
    return (mjd_compare.mjd.mjd_day < mjd.mjd_day
            || (mjd_compare.mjd.mjd_day == mjd.mjd_day && mjd_compare.mjd.mjd_frac < mjd.mjd_frac));
}
bool InsTime::operator==(const InsTime& mjd_compare) const
{
    return (mjd_compare.mjd.mjd_day == mjd.mjd_day
            && std::abs(mjd_compare.mjd.mjd_frac - mjd.mjd_frac) < std::numeric_limits<long double>::epsilon());
    //	std::cout << " vergleich " << mjd_compare.mjd.mjd_day << " " << mjd_compare.mjd.mjd_frac << "\n";
    //	std::cout << " local " << mjd.mjd_day << " " << mjd.mjd_frac << "\n";
}
bool InsTime::operator!=(const InsTime& mjd_compare) const
{
    return (mjd_compare.mjd.mjd_day != mjd.mjd_day
            || std::abs(mjd_compare.mjd.mjd_frac - mjd.mjd_frac) > std::numeric_limits<long double>::epsilon());
}
bool InsTime::operator<=(const InsTime& mjd_compare) const
{
    return (mjd_compare.mjd.mjd_day > mjd.mjd_day
            || (mjd_compare.mjd.mjd_day == mjd.mjd_day && mjd_compare.mjd.mjd_frac >= mjd.mjd_frac));
}
bool InsTime::operator>=(const InsTime& mjd_compare) const
{
    return (mjd_compare.mjd.mjd_day < mjd.mjd_day
            || (mjd_compare.mjd.mjd_day == mjd.mjd_day && mjd_compare.mjd.mjd_frac <= mjd.mjd_frac));
}

} // namespace NAV