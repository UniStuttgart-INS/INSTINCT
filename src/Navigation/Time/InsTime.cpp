#include "InsTime.hpp"

#include <iostream>
#include <string>
#include <iomanip>

#include "util/Logger.hpp"

namespace NAV
{

// ----------------------- Out of Class Definitions --------------------------

std::ostream& operator<<(std::ostream& os, const InsTime& insTime)
{
    return os << insTime.toYMDHMS() << " (" << insTime.toMJD() << ")";
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
    return os << "cycle=" << gpsWeekTow.gpsCycle
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

void to_json(json& j, const InsTime& insTime)
{
    auto mjd = insTime.toMJD();

    j = json{
        { "mjd_day", mjd.mjd_day },
        { "mjd_frac", mjd.mjd_frac },
    };
}

void from_json(const json& j, InsTime& insTime)
{
    InsTime_MJD mjd;

    if (j.contains("mjd_day"))
    {
        j.at("mjd_day").get_to(mjd.mjd_day);
    }
    if (j.contains("mjd_frac"))
    {
        j.at("mjd_frac").get_to(mjd.mjd_frac);
    }
    insTime = InsTime{ mjd };
}

} // namespace NAV