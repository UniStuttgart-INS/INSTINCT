#include <catch2/catch.hpp>

#include "util/Logger.hpp"

TEST_CASE("Logger initialization", "[Logger]")
{
    Logger logger("/tmp/LoggerTest.log");
}

TEST_CASE("Logger initialization should throw exception on write protected path", "[Logger]")
{
    CHECK_THROWS(Logger("/etc/log/LoggerTest.log"));
}
