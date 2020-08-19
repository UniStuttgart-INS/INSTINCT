#include "InsTime.hpp"

#include <iostream>
#include <string>
#include <iomanip>

#include "util/Logger.hpp"

namespace NAV
{
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

std::ostream& operator<<(std::ostream& os, const InsTime_MJD& mjd)
{
    return os << "day=" << mjd.mjd_day << ", frac=" << std::setprecision(20) << mjd.mjd_frac;
}

std::ostream& operator<<(std::ostream& os, const InsTime_JD& jd)
{
    return os << "day=" << jd.jd_day << ", frac=" << std::setprecision(20) << jd.jd_frac;
}

std::ostream& operator<<(std::ostream& os, const InsTime_GPSweekTow& gpsWeekTow)
{
    return os << "cylce=" << gpsWeekTow.gpsCycle
              << ", week=" << gpsWeekTow.gpsWeek
              << ", tow=" << std::setprecision(20) << gpsWeekTow.tow;
}

std::ostream& operator<<(std::ostream& os, const InsTime_YMDHMS& ymdhms)
{
    return os << ymdhms.year << "-"
              << ymdhms.month << "-"
              << ymdhms.day << " "
              << ymdhms.hour << ":"
              << ymdhms.min << ":"
              << std::setprecision(5) << ymdhms.sec;
}

std::ostream& operator<<(std::ostream& os, const InsTime_YDoySod& yDoySod)
{
    return os << "year=" << yDoySod.year
              << ", doy=" << yDoySod.doy
              << ", sod=" << std::setprecision(20) << yDoySod.sod;
}

} // namespace NAV