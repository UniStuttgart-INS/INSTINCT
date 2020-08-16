#include <catch2/catch.hpp>

#include <iostream>
#include <limits>

#include "util/InsTime.hpp"

namespace NAV
{
TEST_CASE("[InsTime] Constructors", "[InsTime]")
{
    auto insTime = InsTime(2004, 2, 10, 13, 25, 58);

    auto insTime_MJD = InsTime::InsTime_MJD(insTime);
    REQUIRE(insTime == InsTime(insTime_MJD));

    auto insTime_JD = InsTime::InsTime_JD(insTime);
    REQUIRE(insTime == InsTime(insTime_JD));

    auto insTime_GPSweekTow = InsTime::InsTime_GPSweekTow(insTime);
    REQUIRE(insTime == InsTime(insTime_GPSweekTow));

    auto insTime_YDoySod = InsTime::InsTime_YDoySod(insTime);
    REQUIRE(insTime == InsTime(insTime_YDoySod));

    auto insTime_YMDHMS = InsTime::InsTime_YMDHMS(insTime);
    REQUIRE(insTime == InsTime(insTime_YMDHMS));
}

TEST_CASE("[InsTime] Leap Seconds", "[InsTime]")
{
    REQUIRE(InsTime::leapGps2UTC(InsTime::InsTime_YMDHMS(1980, 1, 1, 0, 0, 0)) == 0);

    REQUIRE(InsTime::leapGps2UTC(InsTime::InsTime_YMDHMS(1981, 6, 30, 23, 59, 59)) == 0);
    REQUIRE(InsTime::leapGps2UTC(InsTime::InsTime_YMDHMS(1981, 7, 1, 0, 0, 0)) == 1);

    REQUIRE(InsTime::leapGps2UTC(InsTime::InsTime_YMDHMS(1982, 6, 30, 23, 59, 59)) == 1);
    REQUIRE(InsTime::leapGps2UTC(InsTime::InsTime_YMDHMS(1982, 7, 1, 0, 0, 0)) == 2);

    REQUIRE(InsTime::leapGps2UTC(InsTime::InsTime_YMDHMS(1983, 6, 30, 23, 59, 59)) == 2);
    REQUIRE(InsTime::leapGps2UTC(InsTime::InsTime_YMDHMS(1983, 7, 1, 0, 0, 0)) == 3);

    REQUIRE(InsTime::leapGps2UTC(InsTime::InsTime_YMDHMS(1985, 6, 30, 23, 59, 59)) == 3);
    REQUIRE(InsTime::leapGps2UTC(InsTime::InsTime_YMDHMS(1985, 7, 1, 0, 0, 0)) == 4);

    REQUIRE(InsTime::leapGps2UTC(InsTime::InsTime_YMDHMS(1987, 12, 31, 23, 59, 59)) == 4);
    REQUIRE(InsTime::leapGps2UTC(InsTime::InsTime_YMDHMS(1988, 1, 1, 0, 0, 0)) == 5);

    REQUIRE(InsTime::leapGps2UTC(InsTime::InsTime_YMDHMS(1989, 12, 31, 23, 59, 59)) == 5);
    REQUIRE(InsTime::leapGps2UTC(InsTime::InsTime_YMDHMS(1990, 1, 1, 0, 0, 0)) == 6);

    REQUIRE(InsTime::leapGps2UTC(InsTime::InsTime_YMDHMS(1990, 12, 31, 23, 59, 59)) == 6);
    REQUIRE(InsTime::leapGps2UTC(InsTime::InsTime_YMDHMS(1991, 1, 1, 0, 0, 0)) == 7);

    REQUIRE(InsTime::leapGps2UTC(InsTime::InsTime_YMDHMS(1992, 6, 30, 23, 59, 59)) == 7);
    REQUIRE(InsTime::leapGps2UTC(InsTime::InsTime_YMDHMS(1992, 7, 1, 0, 0, 0)) == 8);

    REQUIRE(InsTime::leapGps2UTC(InsTime::InsTime_YMDHMS(1993, 6, 30, 23, 59, 59)) == 8);
    REQUIRE(InsTime::leapGps2UTC(InsTime::InsTime_YMDHMS(1993, 7, 1, 0, 0, 0)) == 9);

    REQUIRE(InsTime::leapGps2UTC(InsTime::InsTime_YMDHMS(1994, 6, 30, 23, 59, 59)) == 9);
    REQUIRE(InsTime::leapGps2UTC(InsTime::InsTime_YMDHMS(1994, 7, 1, 0, 0, 0)) == 10);

    REQUIRE(InsTime::leapGps2UTC(InsTime::InsTime_YMDHMS(1995, 12, 31, 23, 59, 59)) == 10);
    REQUIRE(InsTime::leapGps2UTC(InsTime::InsTime_YMDHMS(1996, 1, 1, 0, 0, 0)) == 11);

    REQUIRE(InsTime::leapGps2UTC(InsTime::InsTime_YMDHMS(1997, 6, 30, 23, 59, 59)) == 11);
    REQUIRE(InsTime::leapGps2UTC(InsTime::InsTime_YMDHMS(1997, 7, 1, 0, 0, 0)) == 12);

    REQUIRE(InsTime::leapGps2UTC(InsTime::InsTime_YMDHMS(1998, 12, 31, 23, 59, 59)) == 12);
    REQUIRE(InsTime::leapGps2UTC(InsTime::InsTime_YMDHMS(1999, 1, 1, 0, 0, 0)) == 13);

    REQUIRE(InsTime::leapGps2UTC(InsTime::InsTime_YMDHMS(2005, 12, 31, 23, 59, 59)) == 13);
    REQUIRE(InsTime::leapGps2UTC(InsTime::InsTime_YMDHMS(2006, 1, 1, 0, 0, 0)) == 14);

    REQUIRE(InsTime::leapGps2UTC(InsTime::InsTime_YMDHMS(2008, 12, 31, 23, 59, 59)) == 14);
    REQUIRE(InsTime::leapGps2UTC(InsTime::InsTime_YMDHMS(2009, 1, 1, 0, 0, 0)) == 15);

    REQUIRE(InsTime::leapGps2UTC(InsTime::InsTime_YMDHMS(2012, 6, 30, 23, 59, 59)) == 15);
    REQUIRE(InsTime::leapGps2UTC(InsTime::InsTime_YMDHMS(2012, 7, 1, 0, 0, 0)) == 16);

    REQUIRE(InsTime::leapGps2UTC(InsTime::InsTime_YMDHMS(2015, 6, 30, 23, 59, 59)) == 16);
    REQUIRE(InsTime::leapGps2UTC(InsTime::InsTime_YMDHMS(2015, 7, 1, 0, 0, 0)) == 17);

    REQUIRE(InsTime::leapGps2UTC(InsTime::InsTime_YMDHMS(2016, 12, 31, 23, 59, 59)) == 17);
    REQUIRE(InsTime::leapGps2UTC(InsTime::InsTime_YMDHMS(2017, 1, 1, 0, 0, 0)) == 18);
}

TEST_CASE("[InsTime] Equality Comparision", "[InsTime]")
{
    auto insTime = InsTime(2004, 2, 10, 13, 25, 58);

    // Conversion: https://heasarc.gsfc.nasa.gov/cgi-bin/Tools/xTime/xTime.pl
    auto insTime_MJD = InsTime::InsTime_MJD(53045, 0.5596990740740740740L);
    REQUIRE(insTime == InsTime(insTime_MJD));

    // Conversion: https://heasarc.gsfc.nasa.gov/cgi-bin/Tools/xTime/xTime.pl
    auto insTime_JD = InsTime::InsTime_JD(2453046, 0.0596990740740740740L);
    REQUIRE(insTime == InsTime(insTime_JD));

    // Conversion: https://www.labsat.co.uk/index.php/en/gps-time-calculator
    auto insTime_GPSweekTow = InsTime::InsTime_GPSweekTow(233, 221158.0L + InsTime::leapGps2UTC(insTime), 1);
    REQUIRE(insTime == InsTime(insTime_GPSweekTow));

    // Conversion: https://www.labsat.co.uk/index.php/en/gps-time-calculator
    auto insTime_YDoySod = InsTime::InsTime_YDoySod(2004, 41, 48358.0L);
    REQUIRE(insTime == InsTime(insTime_YDoySod));

    auto insTime_YMDHMS = InsTime::InsTime_YMDHMS(2004, 2, 10, 13, 25, 58.0L);
    REQUIRE(insTime == InsTime(insTime_YMDHMS));
}

// TEST_CASE(" initialization should throw exception on write protected path", "[InsTime]")
// {
//     // CHECK_THROWS(Logger("/etc/log/LoggerTest.log"));
// }

} // namespace NAV