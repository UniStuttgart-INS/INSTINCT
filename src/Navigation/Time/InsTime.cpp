// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "InsTime.hpp"

#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>

#include "util/Logger.hpp"

namespace NAV
{
InsTime::operator std::string() const
{
    std::stringstream stream;
    stream << *this;
    return stream.str();
}

InsTime_MJD::operator std::string() const
{
    std::stringstream stream;
    stream << *this;
    return stream.str();
}

InsTime_JD::operator std::string() const
{
    std::stringstream stream;
    stream << *this;
    return stream.str();
}

InsTime_GPSweekTow::operator std::string() const
{
    std::stringstream stream;
    stream << *this;
    return stream.str();
}

InsTime_YMDHMS::operator std::string() const
{
    std::stringstream stream;
    stream << *this;
    return stream.str();
}

InsTime_YDoySod::operator std::string() const
{
    std::stringstream stream;
    stream << *this;
    return stream.str();
}

// ----------------------- Out of Class Definitions --------------------------

std::ostream& operator<<(std::ostream& os, const InsTime& insTime)
{
    return os << fmt::format("{}", insTime);
}

std::ostream& operator<<(std::ostream& os, const InsTime_MJD& mjd)
{
    return os << fmt::format("{}", mjd);
}

std::ostream& operator<<(std::ostream& os, const InsTime_JD& jd)
{
    return os << fmt::format("{}", jd);
}

std::ostream& operator<<(std::ostream& os, const InsTime_GPSweekTow& gpsWeekTow)
{
    return os << fmt::format("{}", gpsWeekTow);
}

std::ostream& operator<<(std::ostream& os, const InsTime_YMDHMS& ymdhms)
{
    return os << fmt::format("{}", ymdhms);
}

std::ostream& operator<<(std::ostream& os, const InsTime_YDoySod& yDoySod)
{
    return os << fmt::format("{}", yDoySod);
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