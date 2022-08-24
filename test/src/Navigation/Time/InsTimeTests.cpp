#include <catch2/catch.hpp>

#include <iostream>
#include <limits>
#include <chrono>

#include "util/Logger.hpp"
#include "Navigation/Time/InsTime.hpp"

namespace NAV::TEST::InsTimeTests
{
#define TEST_EQUAL_OBJECT(lhs, rhs) \
    REQUIRE(lhs == rhs);            \
    REQUIRE(rhs == lhs);            \
    REQUIRE_FALSE(lhs != rhs);      \
    REQUIRE(lhs <= rhs);            \
    REQUIRE(lhs >= rhs);

#define STATIC_TEST_EQUAL_OBJECT(lhs, rhs) \
    STATIC_REQUIRE(lhs == rhs);            \
    STATIC_REQUIRE(rhs == lhs);            \
    STATIC_REQUIRE_FALSE(lhs != rhs);      \
    STATIC_REQUIRE(lhs <= rhs);            \
    STATIC_REQUIRE(lhs >= rhs);

#define TEST_LESSER_OBJECT(lhs, rhs) \
    REQUIRE_FALSE(lhs == rhs);       \
    REQUIRE(lhs != rhs);             \
    REQUIRE(lhs < rhs);              \
    REQUIRE_FALSE(lhs > rhs);        \
    REQUIRE(lhs <= rhs);             \
    REQUIRE_FALSE(lhs >= rhs);

#define STATIC_TEST_LESSER_OBJECT(lhs, rhs) \
    STATIC_REQUIRE_FALSE(lhs == rhs);       \
    STATIC_REQUIRE(lhs != rhs);             \
    STATIC_REQUIRE(lhs < rhs);              \
    STATIC_REQUIRE_FALSE(lhs > rhs);        \
    STATIC_REQUIRE(lhs <= rhs);             \
    STATIC_REQUIRE_FALSE(lhs >= rhs);

#define TEST_GREATER_OBJECT(lhs, rhs) \
    REQUIRE_FALSE(lhs == rhs);        \
    REQUIRE(lhs != rhs);              \
    REQUIRE_FALSE(lhs > rhs);         \
    REQUIRE(lhs < rhs);               \
    REQUIRE_FALSE(lhs >= rhs);        \
    REQUIRE(lhs <= rhs);

#define STATIC_TEST_GREATER_OBJECT(lhs, rhs) \
    STATIC_REQUIRE_FALSE(lhs == rhs);        \
    STATIC_REQUIRE(lhs != rhs);              \
    STATIC_REQUIRE_FALSE(lhs > rhs);         \
    STATIC_REQUIRE(lhs < rhs);               \
    STATIC_REQUIRE_FALSE(lhs >= rhs);        \
    STATIC_REQUIRE(lhs <= rhs);

TEST_CASE("[InsTime_MJD] Comparisons", "[InsTime]")
{
    Logger consoleSink;

    auto time_l_1 = InsTime_MJD(53044, 0.5596990740740740740L);
    auto time_l_2 = InsTime_MJD(53045, 0.5596990640740740740L);
    auto time = InsTime_MJD(53045, 0.5596990740740740740L);
    auto time_e_1 = InsTime_MJD(53045, 0.5596990740740740740L);
    auto time_e_2 = InsTime_MJD(53045 - 2, 0.5596990740740740740L + 2.0L);
    auto time_e_3 = InsTime_MJD(53045 + 2, 0.5596990740740740740L - 2.0L);
    auto time_g_1 = InsTime_MJD(53045, 0.5597990740740740740L);
    auto time_g_2 = InsTime_MJD(53046, 0.5596990740740740740L);

    TEST_EQUAL_OBJECT(time, time_e_1);
    TEST_EQUAL_OBJECT(time, time_e_2);
    TEST_EQUAL_OBJECT(time, time_e_3);

    TEST_LESSER_OBJECT(time_l_1, time);
    TEST_LESSER_OBJECT(time_l_2, time);

    TEST_GREATER_OBJECT(time, time_g_1);
    TEST_GREATER_OBJECT(time, time_g_2);

    auto time_eps_0 = InsTime_MJD(53045, 0.0L + InsTimeUtil::EPSILON / 3.L);
    auto time_eps_1 = InsTime_MJD(53045, 0.0L - InsTimeUtil::EPSILON / 3.L);
    TEST_EQUAL_OBJECT(time_eps_0, time_eps_1);
}

TEST_CASE("[InsTime_MJD] Comparisons constexpr", "[InsTime]")
{
    Logger consoleSink;

    constexpr auto time_l_1 = InsTime_MJD(53044, 0.5596990740740740740L);
    constexpr auto time_l_2 = InsTime_MJD(53045, 0.5596990640740740740L);
    constexpr auto time = InsTime_MJD(53045, 0.5596990740740740740L);
    constexpr auto time_e_1 = InsTime_MJD(53045, 0.5596990740740740740L);
    constexpr auto time_e_2 = InsTime_MJD(53045 - 2, 0.5596990740740740740L + 2.0L);
    constexpr auto time_e_3 = InsTime_MJD(53045 + 2, 0.5596990740740740740L - 2.0L);
    constexpr auto time_g_1 = InsTime_MJD(53045, 0.5597990740740740740L);
    constexpr auto time_g_2 = InsTime_MJD(53046, 0.5596990740740740740L);

    STATIC_TEST_EQUAL_OBJECT(time, time_e_1);
    STATIC_TEST_EQUAL_OBJECT(time, time_e_2);
    STATIC_TEST_EQUAL_OBJECT(time, time_e_3);

    STATIC_TEST_LESSER_OBJECT(time_l_1, time);
    STATIC_TEST_LESSER_OBJECT(time_l_2, time);

    STATIC_TEST_GREATER_OBJECT(time, time_g_1);
    STATIC_TEST_GREATER_OBJECT(time, time_g_2);

    constexpr auto time_eps_0 = InsTime_MJD(53045, 0.0L + InsTimeUtil::EPSILON / 3.L);
    constexpr auto time_eps_1 = InsTime_MJD(53045, 0.0L - InsTimeUtil::EPSILON / 3.L);
    STATIC_TEST_EQUAL_OBJECT(time_eps_0, time_eps_1);
}

TEST_CASE("[InsTime_JD] Comparisons", "[InsTime]")
{
    Logger consoleSink;

    auto time_l_1 = InsTime_JD(2453045, 0.5596990740740740740L);
    auto time_l_2 = InsTime_JD(2453046, 0.5596990640740740740L);
    auto time = InsTime_JD(2453046, 0.5596990740740740740L);
    auto time_e_1 = InsTime_JD(2453046, 0.5596990740740740740L);
    auto time_e_2 = InsTime_JD(2453046 - 5, 0.5596990740740740740L + 5.0L);
    auto time_e_3 = InsTime_JD(2453046 + 5, 0.5596990740740740740L - 5.0L);
    auto time_g_1 = InsTime_JD(2453046, 0.5597990740740740740L);
    auto time_g_2 = InsTime_JD(2453047, 0.5596990740740740740L);

    TEST_EQUAL_OBJECT(time, time_e_1);
    TEST_EQUAL_OBJECT(time, time_e_2);
    TEST_EQUAL_OBJECT(time, time_e_3);

    TEST_LESSER_OBJECT(time_l_1, time);
    TEST_LESSER_OBJECT(time_l_2, time);

    TEST_GREATER_OBJECT(time, time_g_1);
    TEST_GREATER_OBJECT(time, time_g_2);

    auto time_eps_0 = InsTime_JD(53045, 0.0L + InsTimeUtil::EPSILON / 3.L);
    auto time_eps_1 = InsTime_JD(53045, 0.0L - InsTimeUtil::EPSILON / 3.L);
    TEST_EQUAL_OBJECT(time_eps_0, time_eps_1);
}

TEST_CASE("[InsTime_JD] Comparisons constexpr", "[InsTime]")
{
    Logger consoleSink;

    constexpr auto time_l_1 = InsTime_JD(2453045, 0.5596990740740740740L);
    constexpr auto time_l_2 = InsTime_JD(2453046, 0.5596990640740740740L);
    constexpr auto time = InsTime_JD(2453046, 0.5596990740740740740L);
    constexpr auto time_e_1 = InsTime_JD(2453046, 0.5596990740740740740L);
    constexpr auto time_e_2 = InsTime_JD(2453046 - 5, 0.5596990740740740740L + 5.0L);
    constexpr auto time_e_3 = InsTime_JD(2453046 + 5, 0.5596990740740740740L - 5.0L);
    constexpr auto time_g_1 = InsTime_JD(2453046, 0.5597990740740740740L);
    constexpr auto time_g_2 = InsTime_JD(2453047, 0.5596990740740740740L);

    STATIC_TEST_EQUAL_OBJECT(time, time_e_1);
    STATIC_TEST_EQUAL_OBJECT(time, time_e_2);
    STATIC_TEST_EQUAL_OBJECT(time, time_e_3);

    STATIC_TEST_LESSER_OBJECT(time_l_1, time);
    STATIC_TEST_LESSER_OBJECT(time_l_2, time);

    STATIC_TEST_GREATER_OBJECT(time, time_g_1);
    STATIC_TEST_GREATER_OBJECT(time, time_g_2);

    constexpr auto time_eps_0 = InsTime_JD(53045, 0.0L + InsTimeUtil::EPSILON / 3.L);
    constexpr auto time_eps_1 = InsTime_JD(53045, 0.0L - InsTimeUtil::EPSILON / 3.L);
    STATIC_TEST_EQUAL_OBJECT(time_eps_0, time_eps_1);
}

TEST_CASE("[InsTime_GPSweekTow] Comparisons", "[InsTime]")
{
    Logger consoleSink;

    auto time_l_1 = InsTime_GPSweekTow(1, 232, 221158.0L);
    auto time_l_2 = InsTime_GPSweekTow(1, 233, 221058.0L);
    auto time_l_3 = InsTime_GPSweekTow(0, 233, 221158.0L);
    auto time = InsTime_GPSweekTow(1, 233, 221158.0L);
    auto time_e_1 = InsTime_GPSweekTow(1 - 1, 233 + 1 * 1024, 221158.0L);
    auto time_e_2 = InsTime_GPSweekTow(1, 233 - 3, 221158.0L + 604800.0L * 3.0L);
    auto time_e_3 = InsTime_GPSweekTow(1, 233 + 2, 221158.0L - 2.0L * InsTimeUtil::SECONDS_PER_WEEK);
    auto time_e_4 = InsTime_GPSweekTow(1 + 1, 233 - InsTimeUtil::WEEKS_PER_GPS_CYCLE, 221158.0L);
    auto time_g_1 = InsTime_GPSweekTow(1, 234, 221158.0L);
    auto time_g_2 = InsTime_GPSweekTow(1, 233, 231158.0L);
    auto time_g_3 = InsTime_GPSweekTow(2, 233, 221158.0L);

    TEST_EQUAL_OBJECT(time, time_e_1);
    TEST_EQUAL_OBJECT(time, time_e_2);
    TEST_EQUAL_OBJECT(time, time_e_3);
    TEST_EQUAL_OBJECT(time, time_e_4);

    TEST_LESSER_OBJECT(time_l_1, time);
    TEST_LESSER_OBJECT(time_l_2, time);
    TEST_LESSER_OBJECT(time_l_3, time);

    TEST_GREATER_OBJECT(time, time_g_1);
    TEST_GREATER_OBJECT(time, time_g_2);
    TEST_GREATER_OBJECT(time, time_g_3);

    auto time_eps_0 = InsTime_GPSweekTow(1, 234, 0.0L + InsTimeUtil::EPSILON / 3.L);
    auto time_eps_1 = InsTime_GPSweekTow(1, 234, 0.0L - InsTimeUtil::EPSILON / 3.L);
    TEST_EQUAL_OBJECT(time_eps_0, time_eps_1);

    auto time_eps_2 = InsTime_GPSweekTow(1, 0, 0.0L + InsTimeUtil::EPSILON / 3.L);
    auto time_eps_3 = InsTime_GPSweekTow(1, 0, 0.0L - InsTimeUtil::EPSILON / 3.L);
    TEST_EQUAL_OBJECT(time_eps_2, time_eps_3);
}

TEST_CASE("[InsTime_GPSweekTow] Comparisons constexpr", "[InsTime]")
{
    Logger consoleSink;

    constexpr auto time_l_1 = InsTime_GPSweekTow(1, 232, 221158.0L);
    constexpr auto time_l_2 = InsTime_GPSweekTow(1, 233, 221058.0L);
    constexpr auto time_l_3 = InsTime_GPSweekTow(0, 233, 221158.0L);
    constexpr auto time = InsTime_GPSweekTow(1, 233, 221158.0L);
    constexpr auto time_e_1 = InsTime_GPSweekTow(1 - 1, 233 + 1 * 1024, 221158.0L);
    constexpr auto time_e_2 = InsTime_GPSweekTow(1, 233 - 3, 221158.0L + 604800.0L * 3.0L);
    constexpr auto time_e_3 = InsTime_GPSweekTow(1, 233 + 2, 221158.0L - 2.0L * InsTimeUtil::SECONDS_PER_WEEK);
    constexpr auto time_e_4 = InsTime_GPSweekTow(1 + 1, 233 - InsTimeUtil::WEEKS_PER_GPS_CYCLE, 221158.0L);
    constexpr auto time_g_1 = InsTime_GPSweekTow(1, 234, 221158.0L);
    constexpr auto time_g_2 = InsTime_GPSweekTow(1, 233, 231158.0L);
    constexpr auto time_g_3 = InsTime_GPSweekTow(2, 233, 221158.0L);

    STATIC_TEST_EQUAL_OBJECT(time, time_e_1);
    STATIC_TEST_EQUAL_OBJECT(time, time_e_2);
    STATIC_TEST_EQUAL_OBJECT(time, time_e_3);
    STATIC_TEST_EQUAL_OBJECT(time, time_e_4);

    STATIC_TEST_LESSER_OBJECT(time_l_1, time);
    STATIC_TEST_LESSER_OBJECT(time_l_2, time);
    STATIC_TEST_LESSER_OBJECT(time_l_3, time);

    STATIC_TEST_GREATER_OBJECT(time, time_g_1);
    STATIC_TEST_GREATER_OBJECT(time, time_g_2);
    STATIC_TEST_GREATER_OBJECT(time, time_g_3);

    constexpr auto time_eps_0 = InsTime_GPSweekTow(1, 234, 0.0L + InsTimeUtil::EPSILON / 3.L);
    constexpr auto time_eps_1 = InsTime_GPSweekTow(1, 234, 0.0L - InsTimeUtil::EPSILON / 3.L);
    STATIC_TEST_EQUAL_OBJECT(time_eps_0, time_eps_1);

    constexpr auto time_eps_2 = InsTime_GPSweekTow(1, 0, 0.0L + InsTimeUtil::EPSILON / 3.L);
    constexpr auto time_eps_3 = InsTime_GPSweekTow(1, 0, 0.0L - InsTimeUtil::EPSILON / 3.L);
    STATIC_TEST_EQUAL_OBJECT(time_eps_2, time_eps_3);
}

TEST_CASE("[InsTime_YMDHMS] Comparisons lesser", "[InsTime]")
{
    Logger consoleSink;

    auto time_l_1 = InsTime_YMDHMS(2003, 2, 10, 13, 25, 58.0L);
    auto time_l_2 = InsTime_YMDHMS(2004, 1, 10, 13, 25, 58.0L);
    auto time_l_3 = InsTime_YMDHMS(2004, 2, 9, 13, 25, 58.0L);
    auto time_l_4 = InsTime_YMDHMS(2004, 2, 10, 12, 25, 58.0L);
    auto time_l_5 = InsTime_YMDHMS(2004, 2, 10, 12, 24, 58.0L);
    auto time_l_6 = InsTime_YMDHMS(2004, 2, 10, 13, 25, 57.0L);
    auto time = InsTime_YMDHMS(2004, 2, 10, 13, 25, 58.0L);

    TEST_LESSER_OBJECT(time_l_1, time);
    TEST_LESSER_OBJECT(time_l_2, time);
    TEST_LESSER_OBJECT(time_l_3, time);
    TEST_LESSER_OBJECT(time_l_4, time);
    TEST_LESSER_OBJECT(time_l_5, time);
    TEST_LESSER_OBJECT(time_l_6, time);
}

TEST_CASE("[InsTime_YMDHMS] Comparisons lesser constexpr", "[InsTime]")
{
    Logger consoleSink;

    constexpr auto time_l_1 = InsTime_YMDHMS(2003, 2, 10, 13, 25, 58.0L);
    constexpr auto time_l_2 = InsTime_YMDHMS(2004, 1, 10, 13, 25, 58.0L);
    constexpr auto time_l_3 = InsTime_YMDHMS(2004, 2, 9, 13, 25, 58.0L);
    constexpr auto time_l_4 = InsTime_YMDHMS(2004, 2, 10, 12, 25, 58.0L);
    constexpr auto time_l_5 = InsTime_YMDHMS(2004, 2, 10, 12, 24, 58.0L);
    constexpr auto time_l_6 = InsTime_YMDHMS(2004, 2, 10, 13, 25, 57.0L);
    constexpr auto time = InsTime_YMDHMS(2004, 2, 10, 13, 25, 58.0L);

    STATIC_TEST_LESSER_OBJECT(time_l_1, time);
    STATIC_TEST_LESSER_OBJECT(time_l_2, time);
    STATIC_TEST_LESSER_OBJECT(time_l_3, time);
    STATIC_TEST_LESSER_OBJECT(time_l_4, time);
    STATIC_TEST_LESSER_OBJECT(time_l_5, time);
    STATIC_TEST_LESSER_OBJECT(time_l_6, time);
}

TEST_CASE("[InsTime_YMDHMS] Comparisons equal", "[InsTime]")
{
    Logger consoleSink;

    auto time = InsTime_YMDHMS(2004, 2, 10, 13, 25, 58.0L);
    auto time_e_1 = InsTime_YMDHMS(2004, 2, 10, 13, 25 - 3, 58.0L + 60.0L * 3.0L);
    auto time_e_2 = InsTime_YMDHMS(2004, 2, 10, 13 - 7, 25 + 60 * 7, 58.0L);
    auto time_e_3 = InsTime_YMDHMS(2004, 2, 10 - 9, 13 + 24 * 9, 25, 58.0L);
    auto time_e_4 = InsTime_YMDHMS(2004, 1, 10 + 31, 13, 25, 58.0L);
    auto time_e_5 = InsTime_YMDHMS(2004 - 8, 2 + 12 * 8, 10, 13, 25, 58.0L);
    auto time_e_6 = InsTime_YMDHMS(2003, 12, 10 + 31 + 31, 13, 25, 58.0L);
    auto time_e_7 = InsTime_YMDHMS(2004, 3, -19, 13, 25, 58.0L);

    TEST_EQUAL_OBJECT(time, time_e_1);
    TEST_EQUAL_OBJECT(time, time_e_2);
    TEST_EQUAL_OBJECT(time, time_e_3);
    TEST_EQUAL_OBJECT(time, time_e_4);
    TEST_EQUAL_OBJECT(time, time_e_5);
    TEST_EQUAL_OBJECT(time, time_e_6);
    TEST_EQUAL_OBJECT(time, time_e_7);

    auto time2 = InsTime_YMDHMS(2004, 10, 10, 13, 25, 58.0L);
    auto time2_e_1 = InsTime_YMDHMS(2005, -2, 10, 13, 25, 58.0L);
    TEST_EQUAL_OBJECT(time2, time2_e_1);

    auto time3 = InsTime_YMDHMS(2003, 10, 10, 13, 25, 58.0L);
    auto time3_e_1 = InsTime_YMDHMS(2004, 10, 10 - 366, 13, 25, 58.0L);
    TEST_EQUAL_OBJECT(time3, time3_e_1);

    auto time4 = InsTime_YMDHMS(2003, 10, 10, 13, 25, 58.0L);
    auto time4_e_1 = InsTime_YMDHMS(2004, 10, 10, 13 - 366 * InsTimeUtil::HOURS_PER_DAY, 25, 58.0L);
    TEST_EQUAL_OBJECT(time4, time4_e_1);

    auto time5 = InsTime_YMDHMS(2003, 10, 10, 13, 25, 58.0L);
    auto time5_e_1 = InsTime_YMDHMS(2004, 10, 10, 13, 25 - 366 * InsTimeUtil::MINUTES_PER_DAY, 58.0L);
    TEST_EQUAL_OBJECT(time5, time5_e_1);

    auto time6 = InsTime_YMDHMS(2003, 12, 31, 13, 25, 58.0L);
    auto time6_e_1 = InsTime_YMDHMS(2004, 3, 10, 13, 25, 58.0L - (10 + 31 + 29) * InsTimeUtil::SECONDS_PER_DAY);
    TEST_EQUAL_OBJECT(time6, time6_e_1);

    auto time_eps_0 = InsTime_YMDHMS(2000, 1, 1, 0, 0, 0.0L + InsTimeUtil::EPSILON / 3.L);
    auto time_eps_1 = InsTime_YMDHMS(2000, 1, 1, 0, 0, 0.0L - InsTimeUtil::EPSILON / 3.L);
    TEST_EQUAL_OBJECT(time_eps_0, time_eps_1);
}

TEST_CASE("[InsTime_YMDHMS] Comparisons equal constexpr", "[InsTime]")
{
    Logger consoleSink;

    constexpr auto time = InsTime_YMDHMS(2004, 2, 10, 13, 25, 58.0L);
    constexpr auto time_e_1 = InsTime_YMDHMS(2004, 2, 10, 13, 25 - 3, 58.0L + 60.0L * 3.0L);
    constexpr auto time_e_2 = InsTime_YMDHMS(2004, 2, 10, 13 - 7, 25 + 60 * 7, 58.0L);
    constexpr auto time_e_3 = InsTime_YMDHMS(2004, 2, 10 - 9, 13 + 24 * 9, 25, 58.0L);
    constexpr auto time_e_4 = InsTime_YMDHMS(2004, 1, 10 + 31, 13, 25, 58.0L);
    constexpr auto time_e_5 = InsTime_YMDHMS(2004 - 8, 2 + 12 * 8, 10, 13, 25, 58.0L);
    constexpr auto time_e_6 = InsTime_YMDHMS(2003, 12, 10 + 31 + 31, 13, 25, 58.0L);

    STATIC_TEST_EQUAL_OBJECT(time, time_e_1);
    STATIC_TEST_EQUAL_OBJECT(time, time_e_2);
    STATIC_TEST_EQUAL_OBJECT(time, time_e_3);
    STATIC_TEST_EQUAL_OBJECT(time, time_e_4);
    STATIC_TEST_EQUAL_OBJECT(time, time_e_5);
    STATIC_TEST_EQUAL_OBJECT(time, time_e_6);

    constexpr auto time2 = InsTime_YMDHMS(2004, 10, 10, 13, 25, 58.0L);
    constexpr auto time2_e_1 = InsTime_YMDHMS(2005, -2, 10, 13, 25, 58.0L);
    STATIC_TEST_EQUAL_OBJECT(time2, time2_e_1);

    constexpr auto time3 = InsTime_YMDHMS(2003, 10, 10, 13, 25, 58.0L);
    constexpr auto time3_e_1 = InsTime_YMDHMS(2004, 10, 10 - 366, 13, 25, 58.0L);
    STATIC_TEST_EQUAL_OBJECT(time3, time3_e_1);

    constexpr auto time4 = InsTime_YMDHMS(2003, 10, 10, 13, 25, 58.0L);
    constexpr auto time4_e_1 = InsTime_YMDHMS(2004, 10, 10, 13 - 366 * InsTimeUtil::HOURS_PER_DAY, 25, 58.0L);
    STATIC_TEST_EQUAL_OBJECT(time4, time4_e_1);

    constexpr auto time5 = InsTime_YMDHMS(2003, 10, 10, 13, 25, 58.0L);
    constexpr auto time5_e_1 = InsTime_YMDHMS(2004, 10, 10, 13, 25 - 366 * InsTimeUtil::MINUTES_PER_DAY, 58.0L);
    STATIC_TEST_EQUAL_OBJECT(time5, time5_e_1);

    constexpr auto time6 = InsTime_YMDHMS(2003, 12, 31, 13, 25, 58.0L);
    constexpr auto time6_e_1 = InsTime_YMDHMS(2004, 3, 10, 13, 25, 58.0L - (10 + 31 + 29) * InsTimeUtil::SECONDS_PER_DAY);
    STATIC_TEST_EQUAL_OBJECT(time6, time6_e_1);

    constexpr auto time_eps_0 = InsTime_YMDHMS(2000, 1, 1, 0, 0, 0.0L + InsTimeUtil::EPSILON / 3.L);
    constexpr auto time_eps_1 = InsTime_YMDHMS(2000, 1, 1, 0, 0, 0.0L - InsTimeUtil::EPSILON / 3.L);
    STATIC_TEST_EQUAL_OBJECT(time_eps_0, time_eps_1);
}

TEST_CASE("[InsTime_YMDHMS] Comparisons greater", "[InsTime]")
{
    Logger consoleSink;

    auto time = InsTime_YMDHMS(2004, 2, 10, 13, 25, 58.0L);
    auto time_g_1 = InsTime_YMDHMS(2005, 2, 10, 13, 25, 58.0L);
    auto time_g_2 = InsTime_YMDHMS(2004, 3, 10, 13, 25, 58.0L);
    auto time_g_3 = InsTime_YMDHMS(2004, 2, 11, 13, 25, 58.0L);
    auto time_g_4 = InsTime_YMDHMS(2004, 2, 10, 14, 25, 58.0L);
    auto time_g_5 = InsTime_YMDHMS(2004, 2, 10, 13, 26, 58.0L);
    auto time_g_6 = InsTime_YMDHMS(2004, 2, 10, 13, 25, 59.0L);

    TEST_GREATER_OBJECT(time, time_g_1);
    TEST_GREATER_OBJECT(time, time_g_2);
    TEST_GREATER_OBJECT(time, time_g_3);
    TEST_GREATER_OBJECT(time, time_g_4);
    TEST_GREATER_OBJECT(time, time_g_5);
    TEST_GREATER_OBJECT(time, time_g_6);
}

TEST_CASE("[InsTime_YMDHMS] Comparisons greater constexpr", "[InsTime]")
{
    Logger consoleSink;

    constexpr auto time = InsTime_YMDHMS(2004, 2, 10, 13, 25, 58.0L);
    constexpr auto time_g_1 = InsTime_YMDHMS(2005, 2, 10, 13, 25, 58.0L);
    constexpr auto time_g_2 = InsTime_YMDHMS(2004, 3, 10, 13, 25, 58.0L);
    constexpr auto time_g_3 = InsTime_YMDHMS(2004, 2, 11, 13, 25, 58.0L);
    constexpr auto time_g_4 = InsTime_YMDHMS(2004, 2, 10, 14, 25, 58.0L);
    constexpr auto time_g_5 = InsTime_YMDHMS(2004, 2, 10, 13, 26, 58.0L);
    constexpr auto time_g_6 = InsTime_YMDHMS(2004, 2, 10, 13, 25, 59.0L);

    STATIC_TEST_GREATER_OBJECT(time, time_g_1);
    STATIC_TEST_GREATER_OBJECT(time, time_g_2);
    STATIC_TEST_GREATER_OBJECT(time, time_g_3);
    STATIC_TEST_GREATER_OBJECT(time, time_g_4);
    STATIC_TEST_GREATER_OBJECT(time, time_g_5);
    STATIC_TEST_GREATER_OBJECT(time, time_g_6);
}

TEST_CASE("[InsTime_YDoySod] Comparisons", "[InsTime]")
{
    Logger consoleSink;

    auto time_l_1 = InsTime_YDoySod(2004, 41, 48358.0L);
    auto time_l_2 = InsTime_YDoySod(2005, 40, 48358.0L);
    auto time_l_3 = InsTime_YDoySod(2005, 41, 48348.0L);
    auto time = InsTime_YDoySod(2005, 41, 48358.0L);
    auto time_e_1 = InsTime_YDoySod(2005, 41 - 7, 48358.0L + (60 * 60 * 24) * 7);
    auto time_e_2 = InsTime_YDoySod(2005 - 3, 41 + 365 * 2 + 366, 48358.0L);
    auto time_g_1 = InsTime_YDoySod(2006, 41, 48358.0L);
    auto time_g_2 = InsTime_YDoySod(2005, 42, 48358.0L);
    auto time_g_3 = InsTime_YDoySod(2005, 41, 48358.1L);

    TEST_EQUAL_OBJECT(time, time_e_1);
    TEST_EQUAL_OBJECT(time, time_e_2);

    TEST_LESSER_OBJECT(time_l_1, time);
    TEST_LESSER_OBJECT(time_l_2, time);
    TEST_LESSER_OBJECT(time_l_3, time);

    TEST_GREATER_OBJECT(time, time_g_1);
    TEST_GREATER_OBJECT(time, time_g_2);
    TEST_GREATER_OBJECT(time, time_g_3);

    auto time2 = InsTime_YDoySod(2004, 365, 48358.0L);
    auto time2_e_1 = InsTime_YDoySod(2005, -1, 48358.0L);
    TEST_EQUAL_OBJECT(time2, time2_e_1);

    auto time3 = InsTime_YDoySod(2003, 364, 48358.0L);
    auto time3_e_1 = InsTime_YDoySod(2004, -1, 48358.0L);
    TEST_EQUAL_OBJECT(time3, time3_e_1);

    auto time4 = InsTime_YDoySod(2003, 364, 48358.0L);
    auto time4_e_1 = InsTime_YDoySod(2004, 6 + 31 + 29, 48358.0L - (7 + 31 + 29) * InsTimeUtil::SECONDS_PER_DAY);
    TEST_EQUAL_OBJECT(time4, time4_e_1);

    auto time_eps_0 = InsTime_YDoySod(2004, 1, 0.0L + InsTimeUtil::EPSILON / 3.L);
    auto time_eps_1 = InsTime_YDoySod(2004, 1, 0.0L - InsTimeUtil::EPSILON / 3.L);
    TEST_EQUAL_OBJECT(time_eps_0, time_eps_1);
}

TEST_CASE("[InsTime_YDoySod] Comparisons constexpr", "[InsTime]")
{
    Logger consoleSink;

    constexpr auto time_l_1 = InsTime_YDoySod(2004, 41, 48358.0L);
    constexpr auto time_l_2 = InsTime_YDoySod(2005, 40, 48358.0L);
    constexpr auto time_l_3 = InsTime_YDoySod(2005, 41, 48348.0L);
    constexpr auto time = InsTime_YDoySod(2005, 41, 48358.0L);
    constexpr auto time_e_1 = InsTime_YDoySod(2005, 41 - 7, 48358.0L + (60 * 60 * 24) * 7);
    constexpr auto time_e_2 = InsTime_YDoySod(2005 - 3, 41 + 365 * 2 + 366, 48358.0L);
    constexpr auto time_g_1 = InsTime_YDoySod(2006, 41, 48358.0L);
    constexpr auto time_g_2 = InsTime_YDoySod(2005, 42, 48358.0L);
    constexpr auto time_g_3 = InsTime_YDoySod(2005, 41, 48358.1L);

    STATIC_TEST_EQUAL_OBJECT(time, time_e_1);
    STATIC_TEST_EQUAL_OBJECT(time, time_e_2);

    STATIC_TEST_LESSER_OBJECT(time_l_1, time);
    STATIC_TEST_LESSER_OBJECT(time_l_2, time);
    STATIC_TEST_LESSER_OBJECT(time_l_3, time);

    STATIC_TEST_GREATER_OBJECT(time, time_g_1);
    STATIC_TEST_GREATER_OBJECT(time, time_g_2);
    STATIC_TEST_GREATER_OBJECT(time, time_g_3);

    constexpr auto time2 = InsTime_YDoySod(2004, 365, 48358.0L);
    constexpr auto time2_e_1 = InsTime_YDoySod(2005, -1, 48358.0L);
    STATIC_TEST_EQUAL_OBJECT(time2, time2_e_1);

    constexpr auto time3 = InsTime_YDoySod(2003, 364, 48358.0L);
    constexpr auto time3_e_1 = InsTime_YDoySod(2004, -1, 48358.0L);
    STATIC_TEST_EQUAL_OBJECT(time3, time3_e_1);

    constexpr auto time4 = InsTime_YDoySod(2003, 364, 48358.0L);
    constexpr auto time4_e_1 = InsTime_YDoySod(2004, 6 + 31 + 29, 48358.0L - (7 + 31 + 29) * InsTimeUtil::SECONDS_PER_DAY);
    STATIC_TEST_EQUAL_OBJECT(time4, time4_e_1);

    constexpr auto time_eps_0 = InsTime_YDoySod(2004, 1, 0.0L + InsTimeUtil::EPSILON / 3.L);
    constexpr auto time_eps_1 = InsTime_YDoySod(2004, 1, 0.0L - InsTimeUtil::EPSILON / 3.L);
    STATIC_TEST_EQUAL_OBJECT(time_eps_0, time_eps_1);
}

TEST_CASE("[InsTime] Constructors & Conversion", "[InsTime]")
{
    Logger consoleSink;

    auto insTime = InsTime(2004, 3, 10, 13, 25, 58);

    auto insTime_MJD = insTime.toMJD();
    TEST_EQUAL_OBJECT(insTime, InsTime(insTime_MJD));

    auto insTime_JD = insTime.toJD();
    TEST_EQUAL_OBJECT(insTime, InsTime(insTime_JD));

    auto insTime_GPSweekTow = insTime.toGPSweekTow();
    TEST_EQUAL_OBJECT(insTime, InsTime(insTime_GPSweekTow));

    auto insTime_YDoySod = insTime.toYDoySod();
    TEST_EQUAL_OBJECT(insTime, InsTime(insTime_YDoySod));

    auto insTime_YMDHMS = insTime.toYMDHMS();
    TEST_EQUAL_OBJECT(insTime, InsTime(insTime_YMDHMS));

    auto insTime_YMDHMS_YDoySod_GPSweekTow_JD_MJD = InsTime(InsTime(InsTime(InsTime(insTime.toMJD())
                                                                                .toJD())
                                                                        .toGPSweekTow())
                                                                .toYDoySod())
                                                        .toYMDHMS();
    TEST_EQUAL_OBJECT(insTime, InsTime(insTime_YMDHMS_YDoySod_GPSweekTow_JD_MJD));
}

TEST_CASE("[InsTime] Constructors & Conversion constexpr", "[InsTime]")
{
    Logger consoleSink;

    constexpr auto insTime = InsTime(2004, 3, 10, 13, 25, 58);

    constexpr auto insTime_MJD = insTime.toMJD();
    STATIC_TEST_EQUAL_OBJECT(insTime, InsTime(insTime_MJD));

    constexpr auto insTime_JD = insTime.toJD();
    STATIC_TEST_EQUAL_OBJECT(insTime, InsTime(insTime_JD));

    constexpr auto insTime_GPSweekTow = insTime.toGPSweekTow();
    STATIC_TEST_EQUAL_OBJECT(insTime, InsTime(insTime_GPSweekTow));

    constexpr auto insTime_YDoySod = insTime.toYDoySod();
    STATIC_TEST_EQUAL_OBJECT(insTime, InsTime(insTime_YDoySod));

    constexpr auto insTime_YMDHMS = insTime.toYMDHMS();
    STATIC_TEST_EQUAL_OBJECT(insTime, InsTime(insTime_YMDHMS));

    constexpr auto insTime_YMDHMS_YDoySod_GPSweekTow_JD_MJD = InsTime(InsTime(InsTime(InsTime(insTime.toMJD())
                                                                                          .toJD())
                                                                                  .toGPSweekTow())
                                                                          .toYDoySod())
                                                                  .toYMDHMS();
    STATIC_TEST_EQUAL_OBJECT(insTime, InsTime(insTime_YMDHMS_YDoySod_GPSweekTow_JD_MJD));
}

TEST_CASE("[InsTime] Leap Seconds constexpr", "[InsTime]")
{
    Logger consoleSink;

    STATIC_REQUIRE(InsTime::leapGps2UTC(InsTime_YMDHMS(1980, 1, 1, 0, 0, 0)) == 0);

    STATIC_REQUIRE(InsTime::leapGps2UTC(InsTime_YMDHMS(1981, 6, 30, 23, 59, 59)) == 0);
    STATIC_REQUIRE(InsTime::leapGps2UTC(InsTime_YMDHMS(1981, 7, 1, 0, 0, 0)) == 1);

    STATIC_REQUIRE(InsTime::leapGps2UTC(InsTime_YMDHMS(1982, 6, 30, 23, 59, 59)) == 1);
    STATIC_REQUIRE(InsTime::leapGps2UTC(InsTime_YMDHMS(1982, 7, 1, 0, 0, 0)) == 2);

    STATIC_REQUIRE(InsTime::leapGps2UTC(InsTime_YMDHMS(1983, 6, 30, 23, 59, 59)) == 2);
    STATIC_REQUIRE(InsTime::leapGps2UTC(InsTime_YMDHMS(1983, 7, 1, 0, 0, 0)) == 3);

    STATIC_REQUIRE(InsTime::leapGps2UTC(InsTime_YMDHMS(1985, 6, 30, 23, 59, 59)) == 3);
    STATIC_REQUIRE(InsTime::leapGps2UTC(InsTime_YMDHMS(1985, 7, 1, 0, 0, 0)) == 4);

    STATIC_REQUIRE(InsTime::leapGps2UTC(InsTime_YMDHMS(1987, 12, 31, 23, 59, 59)) == 4);
    STATIC_REQUIRE(InsTime::leapGps2UTC(InsTime_YMDHMS(1988, 1, 1, 0, 0, 0)) == 5);

    STATIC_REQUIRE(InsTime::leapGps2UTC(InsTime_YMDHMS(1989, 12, 31, 23, 59, 59)) == 5);
    STATIC_REQUIRE(InsTime::leapGps2UTC(InsTime_YMDHMS(1990, 1, 1, 0, 0, 0)) == 6);

    STATIC_REQUIRE(InsTime::leapGps2UTC(InsTime_YMDHMS(1990, 12, 31, 23, 59, 59)) == 6);
    STATIC_REQUIRE(InsTime::leapGps2UTC(InsTime_YMDHMS(1991, 1, 1, 0, 0, 0)) == 7);

    STATIC_REQUIRE(InsTime::leapGps2UTC(InsTime_YMDHMS(1992, 6, 30, 23, 59, 59)) == 7);
    STATIC_REQUIRE(InsTime::leapGps2UTC(InsTime_YMDHMS(1992, 7, 1, 0, 0, 0)) == 8);

    STATIC_REQUIRE(InsTime::leapGps2UTC(InsTime_YMDHMS(1993, 6, 30, 23, 59, 59)) == 8);
    STATIC_REQUIRE(InsTime::leapGps2UTC(InsTime_YMDHMS(1993, 7, 1, 0, 0, 0)) == 9);

    STATIC_REQUIRE(InsTime::leapGps2UTC(InsTime_YMDHMS(1994, 6, 30, 23, 59, 59)) == 9);
    STATIC_REQUIRE(InsTime::leapGps2UTC(InsTime_YMDHMS(1994, 7, 1, 0, 0, 0)) == 10);

    STATIC_REQUIRE(InsTime::leapGps2UTC(InsTime_YMDHMS(1995, 12, 31, 23, 59, 59)) == 10);
    STATIC_REQUIRE(InsTime::leapGps2UTC(InsTime_YMDHMS(1996, 1, 1, 0, 0, 0)) == 11);

    STATIC_REQUIRE(InsTime::leapGps2UTC(InsTime_YMDHMS(1997, 6, 30, 23, 59, 59)) == 11);
    STATIC_REQUIRE(InsTime::leapGps2UTC(InsTime_YMDHMS(1997, 7, 1, 0, 0, 0)) == 12);

    STATIC_REQUIRE(InsTime::leapGps2UTC(InsTime_YMDHMS(1998, 12, 31, 23, 59, 59)) == 12);
    STATIC_REQUIRE(InsTime::leapGps2UTC(InsTime_YMDHMS(1999, 1, 1, 0, 0, 0)) == 13);

    STATIC_REQUIRE(InsTime::leapGps2UTC(InsTime_YMDHMS(2005, 12, 31, 23, 59, 59)) == 13);
    STATIC_REQUIRE(InsTime::leapGps2UTC(InsTime_YMDHMS(2006, 1, 1, 0, 0, 0)) == 14);

    STATIC_REQUIRE(InsTime::leapGps2UTC(InsTime_YMDHMS(2008, 12, 31, 23, 59, 59)) == 14);
    STATIC_REQUIRE(InsTime::leapGps2UTC(InsTime_YMDHMS(2009, 1, 1, 0, 0, 0)) == 15);

    STATIC_REQUIRE(InsTime::leapGps2UTC(InsTime_YMDHMS(2012, 6, 30, 23, 59, 59)) == 15);
    STATIC_REQUIRE(InsTime::leapGps2UTC(InsTime_YMDHMS(2012, 7, 1, 0, 0, 0)) == 16);

    STATIC_REQUIRE(InsTime::leapGps2UTC(InsTime_YMDHMS(2015, 6, 30, 23, 59, 59)) == 16);
    STATIC_REQUIRE(InsTime::leapGps2UTC(InsTime_YMDHMS(2015, 7, 1, 0, 0, 0)) == 17);

    STATIC_REQUIRE(InsTime::leapGps2UTC(InsTime_YMDHMS(2016, 12, 31, 23, 59, 59)) == 17);
    STATIC_REQUIRE(InsTime::leapGps2UTC(InsTime_YMDHMS(2017, 1, 1, 0, 0, 0)) == 18);
}

TEST_CASE("[InsTime] Leap Functions", "[InsTime]")
{
    Logger consoleSink;

    auto insTime = InsTime(2004, 2, 10, 13, 25, 58);

    auto insTime_MJD = insTime.toMJD();
    REQUIRE(InsTime::leapGps2UTC(insTime_MJD) == 13);

    auto insTime_JD = insTime.toJD();
    REQUIRE(InsTime::leapGps2UTC(insTime_JD) == 13);

    auto insTime_GPSweekTow = insTime.toGPSweekTow();
    REQUIRE(InsTime::leapGps2UTC(insTime_GPSweekTow) == 13);

    auto insTime_YDoySod = insTime.toYDoySod();
    REQUIRE(InsTime::leapGps2UTC(insTime_YDoySod) == 13);

    auto insTime_YMDHMS = insTime.toYMDHMS();
    REQUIRE(InsTime::leapGps2UTC(insTime_YMDHMS) == 13);

    REQUIRE(InsTimeUtil::isLeapYear(2003) == false);
    REQUIRE(InsTimeUtil::isLeapYear(2004) == true);
    REQUIRE(insTime.isLeapYear() == true);
}

TEST_CASE("[InsTime] Leap Functions constexpr", "[InsTime]")
{
    Logger consoleSink;

    constexpr auto insTime = InsTime(2004, 2, 10, 13, 25, 58);

    constexpr auto insTime_MJD = insTime.toMJD();
    STATIC_REQUIRE(InsTime::leapGps2UTC(insTime_MJD) == 13);

    constexpr auto insTime_JD = insTime.toJD();
    STATIC_REQUIRE(InsTime::leapGps2UTC(insTime_JD) == 13);

    constexpr auto insTime_GPSweekTow = insTime.toGPSweekTow();
    STATIC_REQUIRE(InsTime::leapGps2UTC(insTime_GPSweekTow) == 13);

    constexpr auto insTime_YDoySod = insTime.toYDoySod();
    STATIC_REQUIRE(InsTime::leapGps2UTC(insTime_YDoySod) == 13);

    constexpr auto insTime_YMDHMS = insTime.toYMDHMS();
    STATIC_REQUIRE(InsTime::leapGps2UTC(insTime_YMDHMS) == 13);

    STATIC_REQUIRE(InsTimeUtil::isLeapYear(2003) == false);
    STATIC_REQUIRE(InsTimeUtil::isLeapYear(2004) == true);
    STATIC_REQUIRE(insTime.isLeapYear() == true);
}

TEST_CASE("[InsTime] Adding values over leap second", "[InsTime]")
{
    InsTime timeBeforeLeap(2016, 12, 31, 23, 30, 0.0, UTC);
    CHECK(timeBeforeLeap.leapGps2UTC() == 17);

    constexpr int64_t dt = 3600;
    InsTime timeAfterLeap = timeBeforeLeap + std::chrono::seconds(dt);
    CHECK(timeAfterLeap.leapGps2UTC() == 18);

    auto calDate_UTC = timeAfterLeap.toYMDHMS(GPST);
    CHECK(calDate_UTC.year == 2017);
    CHECK(calDate_UTC.month == 1);
    CHECK(calDate_UTC.day == 1);
    CHECK(calDate_UTC.hour == 0);
    CHECK(calDate_UTC.min == 30);
    CHECK(calDate_UTC.sec == Approx(18).margin(1.0e-14));

    InsTime recoveredTime = timeAfterLeap - std::chrono::seconds(dt);

    CHECK(recoveredTime == timeBeforeLeap);
}

TEST_CASE("[InsTime] Comparisons lesser", "[InsTime]")
{
    Logger consoleSink;

    auto insTime = InsTime(2004, 2, 10, 13, 25, 58.0L);

    // Conversion: https://heasarc.gsfc.nasa.gov/cgi-bin/Tools/xTime/xTime.pl
    auto insTime_MJD_l_1 = InsTime_MJD(53044, 0.5596990740740740740L);
    auto insTime_MJD_l_2 = InsTime_MJD(53045, 0.5596980740740740740L);
    TEST_LESSER_OBJECT(InsTime(insTime_MJD_l_1), insTime);
    TEST_LESSER_OBJECT(InsTime(insTime_MJD_l_2), insTime);

    // Conversion: https://heasarc.gsfc.nasa.gov/cgi-bin/Tools/xTime/xTime.pl
    auto insTime_JD_l_1 = InsTime_JD(2453045, 0.0596990740740740740L);
    auto insTime_JD_l_2 = InsTime_JD(2453046, 0.0596990730740740740L);
    TEST_LESSER_OBJECT(InsTime(insTime_JD_l_1), insTime);
    TEST_LESSER_OBJECT(InsTime(insTime_JD_l_2), insTime);

    // Conversion: https://www.labsat.co.uk/index.php/en/gps-time-calculator
    auto insTime_GPSweekTow_l_1 = InsTime_GPSweekTow(0, 233, 221158.0L);
    auto insTime_GPSweekTow_l_2 = InsTime_GPSweekTow(1, 232, 221158.0L);
    auto insTime_GPSweekTow_l_3 = InsTime_GPSweekTow(1, 233, 221157.9L);
    TEST_LESSER_OBJECT(InsTime(insTime_GPSweekTow_l_1), insTime);
    TEST_LESSER_OBJECT(InsTime(insTime_GPSweekTow_l_2), insTime);
    TEST_LESSER_OBJECT(InsTime(insTime_GPSweekTow_l_3), insTime);

    // Conversion: https://www.labsat.co.uk/index.php/en/gps-time-calculator
    auto insTime_YDoySod_l_1 = InsTime_YDoySod(2003, 41, 48358.0L);
    auto insTime_YDoySod_l_2 = InsTime_YDoySod(2004, 40, 48358.0L);
    auto insTime_YDoySod_l_3 = InsTime_YDoySod(2004, 41, 48357.8L);
    TEST_LESSER_OBJECT(InsTime(insTime_YDoySod_l_1), insTime);
    TEST_LESSER_OBJECT(InsTime(insTime_YDoySod_l_2), insTime);
    TEST_LESSER_OBJECT(InsTime(insTime_YDoySod_l_3), insTime);

    auto insTime_YMDHMS_l_1 = InsTime_YMDHMS(2004, 1, 10, 13, 25, 58.0L);
    auto insTime_YMDHMS_l_2 = InsTime_YMDHMS(2004, 2, 9, 13, 25, 58.0L);
    auto insTime_YMDHMS_l_3 = InsTime_YMDHMS(2004, 2, 10, 12, 25, 58.0L);
    auto insTime_YMDHMS_l_4 = InsTime_YMDHMS(2004, 2, 10, 13, 24, 58.0L);
    auto insTime_YMDHMS_l_5 = InsTime_YMDHMS(2004, 2, 10, 13, 25, 57.5L);
    TEST_LESSER_OBJECT(InsTime(insTime_YMDHMS_l_1), insTime);
    TEST_LESSER_OBJECT(InsTime(insTime_YMDHMS_l_2), insTime);
    TEST_LESSER_OBJECT(InsTime(insTime_YMDHMS_l_3), insTime);
    TEST_LESSER_OBJECT(InsTime(insTime_YMDHMS_l_4), insTime);
    TEST_LESSER_OBJECT(InsTime(insTime_YMDHMS_l_5), insTime);
}

TEST_CASE("[InsTime] Comparisons lesser constexpr", "[InsTime]")
{
    Logger consoleSink;

    constexpr auto insTime = InsTime(2004, 2, 10, 13, 25, 58.0L);

    // Conversion: https://heasarc.gsfc.nasa.gov/cgi-bin/Tools/xTime/xTime.pl
    constexpr auto insTime_MJD_l_1 = InsTime_MJD(53044, 0.5596990740740740740L);
    constexpr auto insTime_MJD_l_2 = InsTime_MJD(53045, 0.5596980740740740740L);
    STATIC_TEST_LESSER_OBJECT(InsTime(insTime_MJD_l_1), insTime);
    STATIC_TEST_LESSER_OBJECT(InsTime(insTime_MJD_l_2), insTime);

    // Conversion: https://heasarc.gsfc.nasa.gov/cgi-bin/Tools/xTime/xTime.pl
    constexpr auto insTime_JD_l_1 = InsTime_JD(2453045, 0.0596990740740740740L);
    constexpr auto insTime_JD_l_2 = InsTime_JD(2453046, 0.0596990730740740740L);
    STATIC_TEST_LESSER_OBJECT(InsTime(insTime_JD_l_1), insTime);
    STATIC_TEST_LESSER_OBJECT(InsTime(insTime_JD_l_2), insTime);

    // Conversion: https://www.labsat.co.uk/index.php/en/gps-time-calculator
    constexpr auto insTime_GPSweekTow_l_1 = InsTime_GPSweekTow(0, 233, 221158.0L);
    constexpr auto insTime_GPSweekTow_l_2 = InsTime_GPSweekTow(1, 232, 221158.0L);
    constexpr auto insTime_GPSweekTow_l_3 = InsTime_GPSweekTow(1, 233, 221157.9L);
    STATIC_TEST_LESSER_OBJECT(InsTime(insTime_GPSweekTow_l_1), insTime);
    STATIC_TEST_LESSER_OBJECT(InsTime(insTime_GPSweekTow_l_2), insTime);
    STATIC_TEST_LESSER_OBJECT(InsTime(insTime_GPSweekTow_l_3), insTime);

    // Conversion: https://www.labsat.co.uk/index.php/en/gps-time-calculator
    constexpr auto insTime_YDoySod_l_1 = InsTime_YDoySod(2003, 41, 48358.0L);
    constexpr auto insTime_YDoySod_l_2 = InsTime_YDoySod(2004, 40, 48358.0L);
    constexpr auto insTime_YDoySod_l_3 = InsTime_YDoySod(2004, 41, 48357.8L);
    STATIC_TEST_LESSER_OBJECT(InsTime(insTime_YDoySod_l_1), insTime);
    STATIC_TEST_LESSER_OBJECT(InsTime(insTime_YDoySod_l_2), insTime);
    STATIC_TEST_LESSER_OBJECT(InsTime(insTime_YDoySod_l_3), insTime);

    constexpr auto insTime_YMDHMS_l_1 = InsTime_YMDHMS(2004, 1, 10, 13, 25, 58.0L);
    constexpr auto insTime_YMDHMS_l_2 = InsTime_YMDHMS(2004, 2, 9, 13, 25, 58.0L);
    constexpr auto insTime_YMDHMS_l_3 = InsTime_YMDHMS(2004, 2, 10, 12, 25, 58.0L);
    constexpr auto insTime_YMDHMS_l_4 = InsTime_YMDHMS(2004, 2, 10, 13, 24, 58.0L);
    constexpr auto insTime_YMDHMS_l_5 = InsTime_YMDHMS(2004, 2, 10, 13, 25, 57.5L);
    STATIC_TEST_LESSER_OBJECT(InsTime(insTime_YMDHMS_l_1), insTime);
    STATIC_TEST_LESSER_OBJECT(InsTime(insTime_YMDHMS_l_2), insTime);
    STATIC_TEST_LESSER_OBJECT(InsTime(insTime_YMDHMS_l_3), insTime);
    STATIC_TEST_LESSER_OBJECT(InsTime(insTime_YMDHMS_l_4), insTime);
    STATIC_TEST_LESSER_OBJECT(InsTime(insTime_YMDHMS_l_5), insTime);
}

TEST_CASE("[InsTime] Comparisons equal", "[InsTime]")
{
    Logger consoleSink;

    auto insTime = InsTime(2004, 2, 10, 13, 25, 58.0L);

    // Conversion: https://heasarc.gsfc.nasa.gov/cgi-bin/Tools/xTime/xTime.pl
    auto insTime_MJD = InsTime(InsTime_MJD(53045, 0.5596990740740740740L));
    LOG_DEBUG("insTime_MJD: {} <= {}", (insTime - insTime_MJD).count(), InsTimeUtil::EPSILON);
    TEST_EQUAL_OBJECT(insTime, insTime_MJD);

    // Conversion: https://heasarc.gsfc.nasa.gov/cgi-bin/Tools/xTime/xTime.pl
    auto insTime_JD = InsTime(InsTime_JD(2453046, 0.0596990740740740740L));
    LOG_DEBUG("insTime_JD: {} <= {}", (insTime - insTime_JD).count(), InsTimeUtil::EPSILON);
    TEST_EQUAL_OBJECT(insTime, insTime_JD);

    // Conversion: https://www.labsat.co.uk/index.php/en/gps-time-calculator
    auto insTime_GPSweekTow = InsTime(InsTime_GPSweekTow(1, 233, 221158.0L), UTC);
    LOG_DEBUG("insTime_GPSweekTow: {} <= {}", (insTime - insTime_GPSweekTow).count(), InsTimeUtil::EPSILON);
    TEST_EQUAL_OBJECT(insTime, insTime_GPSweekTow);
    auto insTime_GPSweekTow2 = InsTime(InsTime_GPSweekTow(1, 233, 221158.0L + InsTime::leapGps2UTC(insTime)));
    LOG_DEBUG("insTime_GPSweekTow2: {} <= {}", (insTime - insTime_GPSweekTow2).count(), InsTimeUtil::EPSILON);
    TEST_EQUAL_OBJECT(insTime, insTime_GPSweekTow2);

    // Conversion: https://www.labsat.co.uk/index.php/en/gps-time-calculator
    auto insTime_YDoySod = InsTime(InsTime_YDoySod(2004, 41, 48358.0L));
    LOG_DEBUG("insTime_YDoySod: {} <= {}", (insTime - insTime_YDoySod).count(), InsTimeUtil::EPSILON);
    TEST_EQUAL_OBJECT(insTime, insTime_YDoySod);

    auto insTime_YMDHMS = InsTime(InsTime_YMDHMS(2004, 2, 10, 13, 25, 58.0L));
    LOG_DEBUG("insTime_YMDHMS: {} <= {}", (insTime - insTime_YMDHMS).count(), InsTimeUtil::EPSILON);
    TEST_EQUAL_OBJECT(insTime, insTime_YMDHMS);
}

TEST_CASE("[InsTime] Comparisons equal constexpr", "[InsTime]")
{
    Logger consoleSink;

    constexpr auto insTime = InsTime(2004, 2, 10, 13, 25, 58.0L);

    // Conversion: https://heasarc.gsfc.nasa.gov/cgi-bin/Tools/xTime/xTime.pl
    constexpr auto insTime_MJD = InsTime(InsTime_MJD(53045, 0.5596990740740740740L));
    STATIC_TEST_EQUAL_OBJECT(insTime, insTime_MJD);

    // Conversion: https://heasarc.gsfc.nasa.gov/cgi-bin/Tools/xTime/xTime.pl
    constexpr auto insTime_JD = InsTime(InsTime_JD(2453046, 0.0596990740740740740L));
    STATIC_TEST_EQUAL_OBJECT(insTime, insTime_JD);

    // Conversion: https://www.labsat.co.uk/index.php/en/gps-time-calculator
    constexpr auto insTime_GPSweekTow = InsTime(InsTime_GPSweekTow(1, 233, 221158.0L), UTC);
    STATIC_TEST_EQUAL_OBJECT(insTime, insTime_GPSweekTow);
    constexpr auto insTime_GPSweekTow2 = InsTime(InsTime_GPSweekTow(1, 233, 221158.0L + InsTime::leapGps2UTC(insTime)));
    STATIC_TEST_EQUAL_OBJECT(insTime, insTime_GPSweekTow2);

    // Conversion: https://www.labsat.co.uk/index.php/en/gps-time-calculator
    constexpr auto insTime_YDoySod = InsTime(InsTime_YDoySod(2004, 41, 48358.0L));
    STATIC_TEST_EQUAL_OBJECT(insTime, insTime_YDoySod);

    constexpr auto insTime_YMDHMS = InsTime(InsTime_YMDHMS(2004, 2, 10, 13, 25, 58.0L));
    STATIC_TEST_EQUAL_OBJECT(insTime, insTime_YMDHMS);
}

TEST_CASE("[InsTime] Comparisons greater", "[InsTime]")
{
    Logger consoleSink;

    auto insTime = InsTime(2004, 2, 10, 13, 25, 58.0L);

    // Conversion: https://heasarc.gsfc.nasa.gov/cgi-bin/Tools/xTime/xTime.pl
    auto insTime_MJD_g_1 = InsTime(InsTime_MJD(53046, 0.5596990740740740740L));
    auto insTime_MJD_g_2 = InsTime(InsTime_MJD(53045, 0.5596991740740740740L));
    LOG_DEBUG("insTime_MJD_g_1: {}", (insTime - insTime_MJD_g_1).count());
    TEST_GREATER_OBJECT(insTime, insTime_MJD_g_1);
    LOG_DEBUG("insTime_MJD_g_2: {}", (insTime - insTime_MJD_g_2).count());
    TEST_GREATER_OBJECT(insTime, insTime_MJD_g_2);

    // Conversion: https://heasarc.gsfc.nasa.gov/cgi-bin/Tools/xTime/xTime.pl
    auto insTime_JD_g_1 = InsTime(InsTime_JD(2453047, 0.0596990740740740740L));
    auto insTime_JD_g_2 = InsTime(InsTime_JD(2453046, 0.0596990740740740740L + 2 * InsTimeUtil::EPSILON));
    LOG_DEBUG("insTime_JD_g_1: {}", (insTime - insTime_JD_g_1).count());
    TEST_GREATER_OBJECT(insTime, insTime_JD_g_1);
    LOG_DEBUG("insTime_JD_g_2: {}", (insTime - insTime_JD_g_2).count());
    TEST_GREATER_OBJECT(insTime, insTime_JD_g_2);

    // Conversion: https://www.labsat.co.uk/index.php/en/gps-time-calculator
    auto insTime_GPSweekTow_g_1 = InsTime(InsTime_GPSweekTow(2, 233, 221158.0L + InsTime::leapGps2UTC(insTime)));
    auto insTime_GPSweekTow_g_2 = InsTime(InsTime_GPSweekTow(1, 234, 221158.0L + InsTime::leapGps2UTC(insTime)));
    auto insTime_GPSweekTow_g_3 = InsTime(InsTime_GPSweekTow(1, 233, 221158.000000001L + InsTime::leapGps2UTC(insTime)));
    LOG_DEBUG("insTime_GPSweekTow_g_1: {}", (insTime - insTime_GPSweekTow_g_1).count());
    TEST_GREATER_OBJECT(insTime, insTime_GPSweekTow_g_1);
    LOG_DEBUG("insTime_GPSweekTow_g_2: {}", (insTime - insTime_GPSweekTow_g_2).count());
    TEST_GREATER_OBJECT(insTime, insTime_GPSweekTow_g_2);
    LOG_DEBUG("insTime_GPSweekTow_g_3: {}", (insTime - insTime_GPSweekTow_g_3).count());
    TEST_GREATER_OBJECT(insTime, insTime_GPSweekTow_g_3);

    // Conversion: https://www.labsat.co.uk/index.php/en/gps-time-calculator
    auto insTime_YDoySod_g_1 = InsTime(InsTime_YDoySod(2005, 41, 48358.0L + InsTime::leapGps2UTC(insTime)));
    auto insTime_YDoySod_g_2 = InsTime(InsTime_YDoySod(2004, 42, 48358.0L + InsTime::leapGps2UTC(insTime)));
    auto insTime_YDoySod_g_3 = InsTime(InsTime_YDoySod(2004, 41, 48358.01L + InsTime::leapGps2UTC(insTime)));
    LOG_DEBUG("insTime_YDoySod_g_1: {}", (insTime - insTime_YDoySod_g_1).count());
    TEST_GREATER_OBJECT(insTime, insTime_YDoySod_g_1);
    LOG_DEBUG("insTime_YDoySod_g_2: {}", (insTime - insTime_YDoySod_g_2).count());
    TEST_GREATER_OBJECT(insTime, insTime_YDoySod_g_2);
    LOG_DEBUG("insTime_YDoySod_g_3: {}", (insTime - insTime_YDoySod_g_3).count());
    TEST_GREATER_OBJECT(insTime, insTime_YDoySod_g_3);

    auto insTime_YMDHMS_g_1 = InsTime(InsTime_YMDHMS(2004, 3, 10, 13, 25, 58.0L));
    auto insTime_YMDHMS_g_2 = InsTime(InsTime_YMDHMS(2004, 2, 11, 13, 25, 58.0L));
    auto insTime_YMDHMS_g_3 = InsTime(InsTime_YMDHMS(2004, 2, 10, 14, 25, 58.0L));
    auto insTime_YMDHMS_g_4 = InsTime(InsTime_YMDHMS(2004, 2, 10, 13, 26, 58.0L));
    auto insTime_YMDHMS_g_5 = InsTime(InsTime_YMDHMS(2004, 2, 10, 13, 25, 58.000000001L));
    LOG_DEBUG("insTime_YMDHMS_g_1: {}", (insTime - insTime_YMDHMS_g_1).count());
    TEST_GREATER_OBJECT(insTime, insTime_YMDHMS_g_1);
    LOG_DEBUG("insTime_YMDHMS_g_2: {}", (insTime - insTime_YMDHMS_g_2).count());
    TEST_GREATER_OBJECT(insTime, insTime_YMDHMS_g_2);
    LOG_DEBUG("insTime_YMDHMS_g_3: {}", (insTime - insTime_YMDHMS_g_3).count());
    TEST_GREATER_OBJECT(insTime, insTime_YMDHMS_g_3);
    LOG_DEBUG("insTime_YMDHMS_g_4: {}", (insTime - insTime_YMDHMS_g_4).count());
    TEST_GREATER_OBJECT(insTime, insTime_YMDHMS_g_4);
    LOG_DEBUG("insTime_YMDHMS_g_5: {}", (insTime - insTime_YMDHMS_g_5).count());
    TEST_GREATER_OBJECT(insTime, insTime_YMDHMS_g_5);
}

TEST_CASE("[InsTime] Comparisons greater constexpr", "[InsTime]")
{
    Logger consoleSink;

    constexpr auto insTime = InsTime(2004, 2, 10, 13, 25, 58.0L);

    // Conversion: https://heasarc.gsfc.nasa.gov/cgi-bin/Tools/xTime/xTime.pl
    constexpr auto insTime_MJD_g_1 = InsTime_MJD(53046, 0.5596990740740740740L);
    constexpr auto insTime_MJD_g_2 = InsTime_MJD(53045, 0.5596991740740740740L);
    STATIC_TEST_GREATER_OBJECT(insTime, InsTime(insTime_MJD_g_1));
    STATIC_TEST_GREATER_OBJECT(insTime, InsTime(insTime_MJD_g_2));

    // Conversion: https://heasarc.gsfc.nasa.gov/cgi-bin/Tools/xTime/xTime.pl
    constexpr auto insTime_JD_g_1 = InsTime(InsTime_JD(2453047, 0.0596990740740740740L));
    constexpr auto insTime_JD_g_2 = InsTime(InsTime_JD(2453046, 0.0596990740740740740L + 2 * InsTimeUtil::EPSILON));
    STATIC_TEST_GREATER_OBJECT(insTime, insTime_JD_g_1);
    STATIC_TEST_GREATER_OBJECT(insTime, insTime_JD_g_2);

    // Conversion: https://www.labsat.co.uk/index.php/en/gps-time-calculator
    constexpr auto insTime_GPSweekTow_g_1 = InsTime_GPSweekTow(2, 233, 221158.0L + InsTime::leapGps2UTC(insTime));
    constexpr auto insTime_GPSweekTow_g_2 = InsTime_GPSweekTow(1, 234, 221158.0L + InsTime::leapGps2UTC(insTime));
    constexpr auto insTime_GPSweekTow_g_3 = InsTime_GPSweekTow(1, 233, 221158.000000001L + InsTime::leapGps2UTC(insTime));
    STATIC_TEST_GREATER_OBJECT(insTime, InsTime(insTime_GPSweekTow_g_1));
    STATIC_TEST_GREATER_OBJECT(insTime, InsTime(insTime_GPSweekTow_g_2));
    STATIC_TEST_GREATER_OBJECT(insTime, InsTime(insTime_GPSweekTow_g_3));

    // Conversion: https://www.labsat.co.uk/index.php/en/gps-time-calculator
    constexpr auto insTime_YDoySod_g_1 = InsTime_YDoySod(2005, 41, 48358.0L + InsTime::leapGps2UTC(insTime));
    constexpr auto insTime_YDoySod_g_2 = InsTime_YDoySod(2004, 42, 48358.0L + InsTime::leapGps2UTC(insTime));
    constexpr auto insTime_YDoySod_g_3 = InsTime_YDoySod(2004, 41, 48358.01L + InsTime::leapGps2UTC(insTime));
    STATIC_TEST_GREATER_OBJECT(insTime, InsTime(insTime_YDoySod_g_1));
    STATIC_TEST_GREATER_OBJECT(insTime, InsTime(insTime_YDoySod_g_2));
    STATIC_TEST_GREATER_OBJECT(insTime, InsTime(insTime_YDoySod_g_3));

    constexpr auto insTime_YMDHMS_g_1 = InsTime_YMDHMS(2004, 3, 10, 13, 25, 58.0L);
    constexpr auto insTime_YMDHMS_g_2 = InsTime_YMDHMS(2004, 2, 11, 13, 25, 58.0L);
    constexpr auto insTime_YMDHMS_g_3 = InsTime_YMDHMS(2004, 2, 10, 14, 25, 58.0L);
    constexpr auto insTime_YMDHMS_g_4 = InsTime_YMDHMS(2004, 2, 10, 13, 26, 58.0L);
    constexpr auto insTime_YMDHMS_g_5 = InsTime_YMDHMS(2004, 2, 10, 13, 25, 58.000000001L);
    STATIC_TEST_GREATER_OBJECT(insTime, InsTime(insTime_YMDHMS_g_1));
    STATIC_TEST_GREATER_OBJECT(insTime, InsTime(insTime_YMDHMS_g_2));
    STATIC_TEST_GREATER_OBJECT(insTime, InsTime(insTime_YMDHMS_g_3));
    STATIC_TEST_GREATER_OBJECT(insTime, InsTime(insTime_YMDHMS_g_4));
    STATIC_TEST_GREATER_OBJECT(insTime, InsTime(insTime_YMDHMS_g_5));
}

TEST_CASE("[InsTime] Arithmetic operators", "[InsTime]")
{
    Logger consoleSink;

    using namespace std::chrono_literals;

    auto insTime = InsTime(2004, 2, 28, 13, 25, 58.0L);
    auto insTime_target = InsTime(2004, 2,
                                  28 + 7 - 2,
                                  13 + 1,
                                  25 - 2,
                                  58.0L + 3.004005006L);

    auto insTime_eq_1 = insTime
                        + 1 * 7 * 24h
                        - 2 * 24h
                        + 1h
                        - 2min
                        + 3s + 4ms + 5us + 6ns;
    TEST_EQUAL_OBJECT(insTime_target, insTime_eq_1);

    auto diff = insTime_target - insTime;
    TEST_LESSER_OBJECT(diff
                           - (1 * 7 * 24h
                              - 2 * 24h
                              + 1h
                              - 2min
                              + 3s + 4ms + 5us + 6ns),
                       std::chrono::duration<long double>(InsTimeUtil::EPSILON));
    TEST_LESSER_OBJECT(diff.count()
                           - (1.0L * InsTimeUtil::SECONDS_PER_WEEK
                              - 2.0L * InsTimeUtil::SECONDS_PER_DAY
                              + 1.0L * InsTimeUtil::SECONDS_PER_HOUR
                              - 2.0L * InsTimeUtil::SECONDS_PER_MINUTE
                              + 3.004005006L),
                       InsTimeUtil::EPSILON);

    insTime += 1 * 7 * 24h;
    insTime -= 2 * 24h;
    insTime += 1h;
    insTime -= 2min;
    insTime += 3s + 4ms + 5us + 6ns;
    TEST_EQUAL_OBJECT(insTime_target, insTime);
}

TEST_CASE("[InsTime] Arithmetic operators constexpr", "[InsTime]")
{
    Logger consoleSink;

    using namespace std::chrono_literals;

    constexpr auto insTime = InsTime(2004, 2, 28, 13, 25, 58.0L);
    constexpr auto insTime_target = InsTime(2004, 2,
                                            28 + 7 - 2,
                                            13 + 1,
                                            25 - 2,
                                            58.0L + 3.004005006L);

    constexpr auto insTime_eq_1 = insTime
                                  + 1 * 7 * 24h
                                  - 2 * 24h
                                  + 1h
                                  - 2min
                                  + 3s + 4ms + 5us + 6ns;
    STATIC_TEST_EQUAL_OBJECT(insTime_target, insTime_eq_1);

    constexpr auto diff = insTime_target - insTime;
    STATIC_TEST_LESSER_OBJECT(diff
                                  - (1 * 7 * 24h
                                     - 2 * 24h
                                     + 1h
                                     - 2min
                                     + 3s + 4ms + 5us + 6ns),
                              std::chrono::duration<long double>(InsTimeUtil::EPSILON));
    STATIC_TEST_LESSER_OBJECT(diff.count()
                                  - (1.0L * InsTimeUtil::SECONDS_PER_WEEK
                                     - 2.0L * InsTimeUtil::SECONDS_PER_DAY
                                     + 1.0L * InsTimeUtil::SECONDS_PER_HOUR
                                     - 2.0L * InsTimeUtil::SECONDS_PER_MINUTE
                                     + 3.004005006L),
                              InsTimeUtil::EPSILON);
}

} // namespace NAV::TEST::InsTimeTests