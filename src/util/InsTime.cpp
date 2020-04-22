#include "InsTime.hpp"

#include <iostream>
#include <cmath>
#include <vector>
#include <map>
#include <sstream>
#include <string>
#include <iomanip>

#include "util/Logger.hpp"

namespace NAV
{
// FIXME: Enable all the warnings again and fix them
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-conversion"
#pragma GCC diagnostic ignored "-Wdouble-promotion"
#pragma GCC diagnostic ignored "-Wold-style-cast"
#pragma GCC diagnostic ignored "-Wuseless-cast"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wconversion"

//// Prototypes
InsTime::InsTime_MJD GPSweekTow2MJD(InsTime::InsTime_GPSweekTow); // Transformation GPSweekTow to MJD
InsTime::InsTime_MJD YDoySod2MJD(InsTime::InsTime_YDoySod);       // Transformation YDoySod to MJD
InsTime::InsTime_MJD YMDHMS2MJD(InsTime::InsTime_YMDHMS);         // Transformation YMDHMS to MJD
InsTime::InsTime_MJD JD2MJD(InsTime::InsTime_MJD);                // Transformation JD to MJD

InsTime::InsTime_GPSweekTow MJD2GPSweekTow(InsTime::InsTime_MJD); // Transformation MJD to GPSweekTow
InsTime::InsTime_YDoySod MJD2YDoySod(InsTime::InsTime_MJD);       // Transformation MJD to YDoySod
InsTime::InsTime_YMDHMS MJD2YMDHMS(InsTime::InsTime_MJD);         // Transformation MJD to YMDHMS
InsTime::InsTime_JD MJD2JD(InsTime::InsTime_MJD);                 // Transformation MJD to JD

InsTime::InsTime_YMDHMS YDoySod2YMDHMS(InsTime::InsTime_YDoySod); // Transformation YDoySod to YMDHMS (Doy to Date)
InsTime::InsTime_YDoySod YMDHMS2YDoySod(InsTime::InsTime_YMDHMS); // Transformation YMDHMS to YDoySod (Date to Doy)

unsigned short int leapGps2UTC(InsTime);                     // get number of leap seconds, offset GPS to UTC
unsigned short int leapGps2UTC(InsTime::InsTime_YDoySod);    // get number of leap seconds, offset GPS to UTC
unsigned short int leapGps2UTC(InsTime::InsTime_GPSweekTow); // get number of leap seconds, offset GPS to UTC
unsigned short int leapGps2UTC(InsTime::InsTime_YMDHMS);     // get number of leap seconds, offset GPS to UTC
unsigned short int leapGps2UTC(InsTime::InsTime_MJD);        // get number of leap seconds, offset GPS to UTC

// is 1 when InsTime_YDoySod is later than the own time
bool isBigger(InsTime);
bool isBigger(InsTime::InsTime_YDoySod);
bool isBigger(InsTime::InsTime_GPSweekTow);
bool isBigger(InsTime::InsTime_YMDHMS);
bool isBigger(InsTime::InsTime_MJD);
// is 1 when InsTime_YDoySod and own time are euqal
bool isEqual(InsTime);
bool isEqual(InsTime::InsTime_YDoySod);
bool isEqual(InsTime::InsTime_GPSweekTow);
bool isEqual(InsTime::InsTime_YMDHMS);
bool isEqual(InsTime::InsTime_MJD);

long double hms2sec(int hour, int min, long double sec); // Transformation Hour-Min-Sec to Sec
bool isLeapYear(unsigned short int year);                // check if year is a leap year
bool isLeapYear();                                       // check if own class time is a leap year

void addDiffSec(long double);     // add seconds to time
long double getTimeDiff(InsTime); // returns the time difference in sec of two epochs

// Constructors

InsTime::InsTime()
{
    mjd.mjd_day = 400000; // end of this century
    mjd.mjd_frac = 0.0;
}
InsTime::InsTime(InsTime::InsTime_GPSweekTow gpsWeekTow)
{
    //std::cout << "in\n";
    mjd = GPSweekTow2MJD(gpsWeekTow);
}

InsTime::InsTime(InsTime::InsTime_MJD mjd_in)
{
    mjd = mjd_in;
}
InsTime::InsTime(InsTime::InsTime_JD jd_in)
{
    mjd = JD2MJD(jd_in);
}
InsTime::InsTime(InsTime::InsTime_YDoySod yearDoySod)
{
    mjd = YDoySod2MJD(yearDoySod);
}

InsTime::InsTime(InsTime::InsTime_YMDHMS yearMonthDayHMS)
{
    mjd = YMDHMS2MJD(yearMonthDayHMS);
}
InsTime::InsTime(unsigned short int gpsWeek, long double tow, unsigned short int gpsCycle)
{
    InsTime::InsTime_GPSweekTow gpsWeekTow = { gpsWeek, tow, gpsCycle };
    mjd = GPSweekTow2MJD(gpsWeekTow);
}
InsTime::InsTime(unsigned short int year, unsigned short int month, unsigned short int day, unsigned short int hour, unsigned short int min, long double sec, InsTime::TIME timesys)
{
    InsTime::InsTime_YMDHMS yearMonthDayHMS = { year, month, day, hour, min, sec };
    mjd = YMDHMS2MJD(yearMonthDayHMS);

    if (timesys == 1) // = GPS Time (UTC - leap_seconds)
    {
        int leapSec = this->leapGps2UTC();
        this->addDiffSec(-leapSec);
    }
    else if (timesys == 2) // = GLONASS Time (UTC+ 3h)
    {
        int leapSec = 10800;
        this->addDiffSec(-leapSec);
    }
    else if (timesys == 3) // = GALILEO Time (~ GPS TIME )
    {
        //  is synchronised with TAI with a nominal offset below 50 ns
        int leapSec = this->leapGps2UTC(); // UTC = GST - 18
        this->addDiffSec(-leapSec);
    }
    else if (timesys == 4) // = BeiDou Time (UTC)
    {
        // is synchronised with UTC within 100 ns<
    }
}

// Destructor
InsTime::~InsTime() {}

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
void InsTime::SetInsTime(unsigned short int gpsWeek, long double tow, unsigned short int gpsCycle)
{
    InsTime_GPSweekTow gpsWeekTow = { gpsWeek, tow, gpsCycle };
    mjd = GPSweekTow2MJD(gpsWeekTow);
}
void InsTime::SetInsTime(unsigned short int year, unsigned short int month, unsigned short int day, unsigned short int hour, unsigned short int min, long double sec, InsTime::TIME timesys)
{
    InsTime::InsTime_YMDHMS yearMonthDayHMS = { year, month, day, hour, min, sec };
    mjd = YMDHMS2MJD(yearMonthDayHMS);

    if (timesys == 1) // = GPS Time (UTC - leap_seconds)
    {
        int leapSec = this->leapGps2UTC();
        this->addDiffSec(-leapSec);
    }
    else if (timesys == 2) // = GLONASS Time GLNT (UTC+ 3h)
    {
        int leapSec = 10800;
        this->addDiffSec(-leapSec);
    }
    else if (timesys == 3) // = GALILEO Time (TAI)
    {
        //  is synchronised with TAI with a nominal offset below 50 ns
        int leapSec = this->leapGps2UTC();
        this->addDiffSec(-leapSec);
    }
    else if (timesys == 4) // = BeiDou Time (UTC)
    {
        // is synchronised with UTC within 100 ns
    }
    else if (timesys == 5) // = QZSS Time (???)
    {
        // implementation missing
    }
    else if (timesys == 6) // = IRNSS Time (???)
    {
        // implementation missing
    }
}

// accessors, get time
InsTime::InsTime_GPSweekTow InsTime::GetGPSTime()
{
    return MJD2GPSweekTow(mjd);
}
InsTime::InsTime_MJD InsTime::GetMJD()
{
    return mjd;
}
InsTime::InsTime_JD InsTime::GetJD()
{
    return MJD2JD(mjd);
}
InsTime::InsTime_YDoySod InsTime::GetYDoySod()
{
    return MJD2YDoySod(mjd);
}
InsTime::InsTime_YMDHMS InsTime::GetYMDHMS()
{
    return MJD2YMDHMS(mjd);
}

// Transformation Functions
InsTime::InsTime_MJD InsTime::GPSweekTow2MJD(InsTime::InsTime_GPSweekTow gpsWeeekTow) // Transformation GPSweekTow to MJD (GPST to UTC)
{
    mjd.mjd_day = (gpsWeeekTow.gpsCycle * 1024 + gpsWeeekTow.gpsWeek) * 7 + floor(gpsWeeekTow.tow / SEC_IN_DAY) + 44244; // 442244 =  diff to 6.1.1980
    mjd.mjd_frac = (long double)fmod(gpsWeeekTow.tow, SEC_IN_DAY) / SEC_IN_DAY;
    mjd.mjd_frac -= (long double)leapGps2UTC(mjd) / SEC_IN_DAY; // from GPST to UTC
    if (mjd.mjd_frac < 0.0)
    {
        mjd.mjd_day -= 1;
        mjd.mjd_frac += 1.0;
    }
    return mjd;
}

InsTime::InsTime_MJD InsTime::YDoySod2MJD(InsTime::InsTime_YDoySod yearDoySod) // Transformation YDoySod to MJD (GPST to UTC)
{
    InsTime_YMDHMS yearMonthDayHMS = YDoySod2YMDHMS(yearDoySod);
    mjd = YMDHMS2MJD(yearMonthDayHMS);
    return mjd;
}

InsTime::InsTime_MJD InsTime::YMDHMS2MJD(InsTime::InsTime_YMDHMS yearMonthDayHMS) // Transformation YMDHMS to MJD (UTC to UTC)
{
    InsTime_JD jd;
    int a = floor((14.0 - (double)yearMonthDayHMS.month) / 12.0);
    int y = yearMonthDayHMS.year + 4800 - a;
    int m = yearMonthDayHMS.month + 12 * a - 3;

    jd.jd_day = yearMonthDayHMS.day + floor((153.0 * (double)m + 2.0) / 5.0) + y * 365 + floor((double)y / 4.0) - floor((double)y / 100.0) + floor((double)y / 400.0) - 32045;
    jd.jd_frac = (yearMonthDayHMS.sec + (double)yearMonthDayHMS.min * 60.0 + (double)(yearMonthDayHMS.hour - 12.0) * 3600.0) / SEC_IN_DAY;
    mjd = JD2MJD(jd);
    return mjd;
}

InsTime::InsTime_MJD InsTime::JD2MJD(InsTime::InsTime_JD jd) // Transformation JD to MJD (UTC to UTC)
{
    mjd.mjd_day = jd.jd_day - 2400000;
    mjd.mjd_frac = jd.jd_frac - 0.5;
    if (mjd.mjd_frac < 0.0)
    {
        mjd.mjd_day -= 1;
        mjd.mjd_frac += 1.0;
    }
    return mjd;
}

InsTime::InsTime_GPSweekTow InsTime::MJD2GPSweekTow(InsTime::InsTime_MJD mjd) // Transformation MJD to GPSweekTow (UTC to GPST)
{
    mjd.mjd_frac += (long double)leapGps2UTC(mjd) / SEC_IN_DAY; // from UTC to GPST
    if (mjd.mjd_frac < 0.0)
    {
        mjd.mjd_day -= 1;
        mjd.mjd_frac += 1;
    }
    InsTime_GPSweekTow gpsWeekTow;
    gpsWeekTow.gpsCycle = floor((double)(mjd.mjd_day - 44244) / (1024.0 * 7.0));
    gpsWeekTow.gpsWeek = floor((mjd.mjd_day - 44244 - gpsWeekTow.gpsCycle * 1024 * 7) / 7);             // number of full weeks
    gpsWeekTow.tow = (long double)((mjd.mjd_day - 44244) % 7) * SEC_IN_DAY + mjd.mjd_frac * SEC_IN_DAY; // remaining seconds
    return gpsWeekTow;
}

InsTime::InsTime_YDoySod InsTime::MJD2YDoySod(InsTime::InsTime_MJD mjd) // Transformation MJD to YDoySod (UTC to GPST)
{
    InsTime_YDoySod yearDoySod;
    InsTime_YMDHMS yearMonthDayHMS = MJD2YMDHMS(mjd);
    yearDoySod = YMDHMS2YDoySod(yearMonthDayHMS);
    return yearDoySod;
}

InsTime::InsTime_YMDHMS InsTime::MJD2YMDHMS(InsTime::InsTime_MJD mjd) // Transformation MJD to YMDHMS (UTC to UTC)
{
    // transform MJD to JD
    InsTime_JD jd = MJD2JD(mjd);
    jd.jd_frac = jd.jd_frac + 0.5;
    if (jd.jd_frac >= 1.0)
    {
        jd.jd_day += 1;
        jd.jd_frac -= 1.0;
    }
    // transform JD to YMDHMS
    double a = floor((double)jd.jd_day + 0) + 32044.0;
    double b = floor((4.0 * a + 3.0) / 146097.0);
    double c = a - floor((b * 146097.0) / 4.0);

    double d = floor((4.0 * c + 3.0) / 1461.0);
    double e = c - floor((1461.0 * d) / 4.0);
    double m = floor((5.0 * e + 2.0) / 153.0);

    unsigned short int day = e - floor((153.0 * m + 2.0) / 5.0) + 1;
    unsigned short int month = m + 3 - 12 * floor(m / 10.0);
    unsigned short int year = b * 100 + d - 4800.0 + floor(m / 10.0);

    unsigned short int hour = floor(jd.jd_frac * 24.0);
    unsigned short int min = floor(jd.jd_frac * 24.0 * 60.0) - hour * 60;
    long double sec = (long double)jd.jd_frac * 24.0 * 3600.0 - ((double)hour * 3600.0 + (double)min * 60.0);

    InsTime_YMDHMS yearMonthDayHMS;
    yearMonthDayHMS.year = year;
    yearMonthDayHMS.month = month;
    yearMonthDayHMS.day = day;
    yearMonthDayHMS.hour = hour;
    yearMonthDayHMS.min = min;
    yearMonthDayHMS.sec = sec;
    return yearMonthDayHMS;
}

InsTime::InsTime_JD InsTime::MJD2JD(InsTime::InsTime_MJD mjd) // Transformation MJD to JD (UTC to UTC)
{
    InsTime_JD jd;
    jd.jd_day = mjd.mjd_day + 2400000;
    jd.jd_frac = mjd.mjd_frac + 0.5;
    if (jd.jd_frac >= 1.0)
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
        leapDay = 1;

    std::vector<int> daysOfMonth = { 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334 };
    std::vector<int>::iterator low;
    low = std::lower_bound(daysOfMonth.begin(), daysOfMonth.end(), (yearDoySod.doy - leapDay));
    int month_number = low - daysOfMonth.begin() + 1;
    yearDoySod.sod -= (long double)leapGps2UTC(yearDoySod);
    if (yearDoySod.sod < 0.0)
    {
        yearDoySod.doy -= 1;
        yearDoySod.sod += SEC_IN_DAY;
        if (yearDoySod.doy < 1)
        {
            yearDoySod.year -= 1;
            yearDoySod.doy += 365;
            if (isLeapYear(yearDoySod.year) && month_number > 2)
                yearDoySod.doy += 1;
        }
    }

    InsTime_YMDHMS yearMonthDayHMS;
    yearMonthDayHMS.year = yearDoySod.year;
    yearMonthDayHMS.month = month_number;
    yearMonthDayHMS.day = yearDoySod.doy - leapDay - daysOfMonth[month_number - 2];
    yearMonthDayHMS.hour = floor(yearDoySod.sod / 3600.0);
    yearMonthDayHMS.min = floor(yearDoySod.sod / 60.0) - yearMonthDayHMS.hour * 60;
    yearMonthDayHMS.sec = yearDoySod.sod - (long double)(yearMonthDayHMS.hour * 3600.0 + yearMonthDayHMS.min * 60.0);

    return yearMonthDayHMS;
}

InsTime::InsTime_YDoySod InsTime::YMDHMS2YDoySod(InsTime::InsTime_YMDHMS yearMonthDayHMS) // Transformation YMDHMS to YDoySod (UTC to GPST)
{
    int daysOfMonth[11] = { 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334 };

    InsTime_YDoySod yearDoySod;
    yearDoySod.year = yearMonthDayHMS.year;
    yearDoySod.sod = (double)(yearMonthDayHMS.hour * 3600.0 + yearMonthDayHMS.min * 60.0) + (long double)yearMonthDayHMS.sec + (double)leapGps2UTC(yearMonthDayHMS);

    if (yearMonthDayHMS.month == 1)
        yearDoySod.doy = yearMonthDayHMS.day;
    else if (yearMonthDayHMS.month == 2)
        yearDoySod.doy = daysOfMonth[0] + yearMonthDayHMS.day;
    else // for march to december
    {
        if (isLeapYear(yearMonthDayHMS.year))
        {
            yearDoySod.doy = 1 + daysOfMonth[yearMonthDayHMS.month - 2] + yearMonthDayHMS.day;
        }
        else
        {
            yearDoySod.doy = daysOfMonth[yearMonthDayHMS.month - 2] + yearMonthDayHMS.day;
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
    mjd.mjd_frac += (long double)diffSec / SEC_IN_DAY;

    if (mjd.mjd_frac >= 1.0 || mjd.mjd_frac < 0.0)
    {
        mjd.mjd_day += floor(mjd.mjd_frac);
        mjd.mjd_frac -= floor(mjd.mjd_frac);
    }
}
void InsTime::MakeTimeFromGloOrbit(double UTC_sec)
{
    // difference between toe (OBRIT-0 last element) and toc (ORBIT-0 first element) in seconds
    double diff = fmod(UTC_sec, SEC_IN_DAY) - (this->GetYMDHMS().hour * 3600.0 + this->GetYMDHMS().min * 60 + this->GetYMDHMS().sec); // std::cout << "orbit diff " << diff << "\n";
    this->addDiffSec(diff);
}
std::string InsTime::GetStringOfDate()
{
    std::string s = std::to_string(this->GetYMDHMS().year) + std::to_string(this->GetYMDHMS().month) + std::to_string(this->GetYMDHMS().day) + std::to_string(this->GetYMDHMS().hour) + std::to_string(this->GetYMDHMS().min) + std::to_string((int)this->GetYMDHMS().sec);
    return s;
}

void InsTime::MakeTimeSysFromString(std::string sys, InsTime::TIME* timesys)
{
    *timesys = InsTime::UTC;
    if (sys == "UTC")
        *timesys = InsTime::UTC;
    else if (sys == "GPST")
        *timesys = InsTime::GPST;
    else
        std::cout << "Unknown Time system " << sys << ". Time sys is set to UTC\n";
}
std::string InsTime::MakeStringFromTimeSys(InsTime::TIME sys)
{
    if (sys == 0)
        return "UTC";
    else if (sys == 1)
        return "GPST";
    else if (sys == 2)
        return "GLNT";
    else if (sys == 3)
        return "BDT";
    else if (sys == 4)
        return "QZSST";
    else if (sys == 5)
        return "IRNSST";
    else
        std::cout << "Unknown Time system\n";

    return "unknown";
}

// leap seconds

const std::map<unsigned int, unsigned short int> InsTime::GpsLeapSec = {
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

unsigned short int InsTime::leapGps2UTC()
{
    return leapGps2UTC(mjd);
}
unsigned short int InsTime::leapGps2UTC(InsTime insTime)
{
    return leapGps2UTC(insTime.mjd);
}
unsigned short int InsTime::leapGps2UTC(InsTime::InsTime_YDoySod yearDoySod)
{
    return leapGps2UTC(YDoySod2MJD(yearDoySod));
}
unsigned short int InsTime::leapGps2UTC(InsTime::InsTime_GPSweekTow gpsWeekTow)
{
    return leapGps2UTC(GPSweekTow2MJD(gpsWeekTow));
}
unsigned short int InsTime::leapGps2UTC(InsTime::InsTime_YMDHMS yearMonthDayHMS)
{
    return leapGps2UTC(YMDHMS2MJD(yearMonthDayHMS));
}
unsigned short int InsTime::leapGps2UTC(InsTime::InsTime_MJD mjd_in)
{
    //std::map<unsigned int,unsigned short int>::iterator pLeapSec;
    auto pLeapSec = InsTime::GpsLeapSec.upper_bound(mjd_in.mjd_day);
    //std::cout << "leap sec " <<pLeapSec->first << " is " <<pLeapSec->second -1 << "sec for mjd: "<< mjd.mjd_day << "\n";
    return pLeapSec->second - 1;
}

// Function for time comparision: bigger/equal
bool InsTime::isBigger(InsTime insTime)
{
    return isBigger(insTime.mjd);
}

bool InsTime::isBigger(InsTime::InsTime_YDoySod yearDoySod)
{
    return isBigger(YDoySod2MJD(yearDoySod));
}
bool InsTime::isBigger(InsTime::InsTime_GPSweekTow gpsWeekTow)
{
    return isBigger(GPSweekTow2MJD(gpsWeekTow));
}
bool InsTime::isBigger(InsTime::InsTime_YMDHMS yearMonthDayHMS)
{
    return isBigger(YMDHMS2MJD(yearMonthDayHMS));
}
bool InsTime::isBigger(InsTime::InsTime_MJD mjd_compare) // is true if mjd_compare is bigger (later)
{
    bool is_bigger = false;
    if (mjd_compare.mjd_day > mjd.mjd_day)
        is_bigger = true;
    else if (mjd_compare.mjd_day == mjd.mjd_day && mjd_compare.mjd_frac > mjd.mjd_frac)
        is_bigger = true;
    return is_bigger;
}
bool InsTime::isEqual(InsTime insTime)
{
    return isEqual(insTime.mjd);
}
bool InsTime::isEqual(InsTime::InsTime_YDoySod yearDoySod)
{
    return isEqual(YDoySod2MJD(yearDoySod));
}
bool InsTime::isEqual(InsTime::InsTime_GPSweekTow gpsWeekTow)
{
    return isEqual(GPSweekTow2MJD(gpsWeekTow));
}
bool InsTime::isEqual(InsTime::InsTime_YMDHMS yearMonthDayHMS)
{
    return isEqual(YMDHMS2MJD(yearMonthDayHMS));
}
bool InsTime::isEqual(InsTime::InsTime_MJD mjd_compare)
{
    bool is_equal = false;
    if (mjd_compare.mjd_day == mjd.mjd_day && mjd_compare.mjd_frac == mjd.mjd_frac)
        is_equal = true; // frage auf wie viele nachkommastellen genau? ??
    return is_equal;
}

long double InsTime::getTimeDiff(InsTime insTime) // returns the time difference in sec of two epochs
{
    int diffDays = mjd.mjd_day - insTime.GetMJD().mjd_day;
    long double diffSec = (mjd.mjd_frac - insTime.GetMJD().mjd_frac) * SEC_IN_DAY + (double)diffDays * SEC_IN_DAY;
    return diffSec;
}

long double InsTime::hms2sec(int hour, int min, long double sec) // Transformation Hour-Min-Sec to Sec
{
    return (double)hour * 3600.0 + (double)min * 60.0 + (long double)sec;
}

bool InsTime::isLeapYear(unsigned short int year)
{
    bool leap = (((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0));
    return leap;
}
bool InsTime::isLeapYear()
{
    unsigned short int year = this->GetYDoySod().year;
    bool leap = (((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0));
    return leap;
}

std::string InsTime::outputTime()
{
    InsTime_YMDHMS time = MJD2YMDHMS(mjd);
    std::stringstream ss;
    ss << "YMDHMS " << time.year << " " << time.month << " " << time.day << " " << time.hour << " " << time.min << " " << roundf((float)time.sec * 1000) / 1000;
    std::string output = ss.str();
    return output;
}
std::string InsTime::outputTowTime()
{
    //InsTime_GPSweekTow time = MJD2GPSweekTow(mjd);
    std::stringstream ss;
    ss << "tow " << roundf(this->GetGPSTime().tow * 10.0) / 10.0;
    std::string output = ss.str();
    return output;
}
std::string InsTime::outputTime(InsTime time)
{
    std::stringstream ss;
    ss << "YMDHMS time " << time.GetYMDHMS().year << " " << time.GetYMDHMS().month << " " << time.GetYMDHMS().day << " " << time.GetYMDHMS().hour << " " << time.GetYMDHMS().min << " " << std::setprecision(5) << time.GetYMDHMS().sec;
    std::string output = ss.str();
    return output;
}

// overload operator
bool InsTime::operator<(const InsTime& mjd_compare) const
{
    if (mjd_compare.mjd.mjd_day > mjd.mjd_day)
        return true;
    if (mjd_compare.mjd.mjd_day == mjd.mjd_day && mjd_compare.mjd.mjd_frac > mjd.mjd_frac)
        return true;
    return false;
}
bool InsTime::operator>(const InsTime& mjd_compare) const
{
    if (mjd_compare.mjd.mjd_day < mjd.mjd_day)
        return true;
    if (mjd_compare.mjd.mjd_day == mjd.mjd_day && mjd_compare.mjd.mjd_frac < mjd.mjd_frac)
        return true;
    return false;
}
bool InsTime::operator==(const InsTime& mjd_compare) const
{
    if (mjd_compare.mjd.mjd_day == mjd.mjd_day && fabs(mjd_compare.mjd.mjd_frac - mjd.mjd_frac) < 0.000001)
        return true;
    return false;
    //	std::cout << " vergleich " << mjd_compare.mjd.mjd_day << " " << mjd_compare.mjd.mjd_frac << "\n";
    //	std::cout << " local " << mjd.mjd_day << " " << mjd.mjd_frac << "\n";
}
bool InsTime::operator!=(const InsTime& mjd_compare) const
{
    if (mjd_compare.mjd.mjd_day != mjd.mjd_day || fabs(mjd_compare.mjd.mjd_frac - mjd.mjd_frac) > 0.000001)
        return true;
    return false;
}
bool InsTime::operator<=(const InsTime& mjd_compare) const
{
    if (mjd_compare.mjd.mjd_day > mjd.mjd_day)
        return true;
    if (mjd_compare.mjd.mjd_day == mjd.mjd_day && mjd_compare.mjd.mjd_frac >= mjd.mjd_frac)
        return true;
    return false;
}
bool InsTime::operator>=(const InsTime& mjd_compare) const
{
    if (mjd_compare.mjd.mjd_day < mjd.mjd_day)
        return true;
    if (mjd_compare.mjd.mjd_day == mjd.mjd_day && mjd_compare.mjd.mjd_frac <= mjd.mjd_frac)
        return true;
    return false;
}

#pragma GCC diagnostic pop

} // namespace NAV