#include "util/Common.hpp"
#include "util/Logger.hpp"
#include "util/Version.hpp"
#include "util/Sleep.hpp"
#include "docopt/docopt.h"

static const char USAGE[] =
    R"(NavSoS - Navigation Software Stuttgart

    Usage:
      navsos (-h | --help)
      navsos --version
      navsos [--time=<s> | --sigterm] [--vn100] [--vn110] [--ublox]

    Options:
      -h --help                  Show this screen
      --version                  Show version
      --time=<s>                 Programm time in seconds [default: 5]
      --sigterm                  Programm ends by sending -SIGUSR1 / -SIGINT / -SIGTERM
      --vn100                    Enables VN100 Logging
      --vn110                    Enables VN110 Logging
      --ublox                    Enables ublox Logging
)";

int main(int argc, const char** argv)
{
    if (NAV::Logger::initialize() != NAV::NavStatus::NAV_OK)
        return EXIT_FAILURE;

    // Decode the command line options
    std::map<std::string, docopt::value> args = docopt::docopt(USAGE,
                                                               { argv + 1, argv + argc },
                                                               true,                    // show help if requested
                                                               PROJECT_VERSION_STRING); // version string

    // Write the Log Header
    NAV::Logger::writeHeader();

    // Print the Command Line Options
    for (auto const& arg : args)
        SPDLOG_DEBUG("Command line arguments: \"{}\" = {}", arg.first, arg.second);

    if (args["--ublox"].asBool())
    {
        SPDLOG_TRACE("ublox logging specified");
    }

    if (args["--vn100"].asBool())
    {
        SPDLOG_TRACE("VN100 logging specified");
    }

    if (args["--vn110"].asBool())
    {
        SPDLOG_TRACE("VN110 logging specified");
    }

    if (args["--sigterm"].asBool())
        NAV::Sleep::waitForSignal();
    else
        NAV::Sleep::countDownSeconds(static_cast<size_t>(args["--time"].asLong()));

    NAV::Logger::writeFooter();
}