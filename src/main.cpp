#include "util/Common.hpp"
#include "util/Logger.hpp"
#include "util/Version.hpp"
#include "util/Sleep.hpp"
#include "util/Config.hpp"

int main(int argc, const char** argv)
{
    if (NAV::Logger::initialize("logs/navsos.log") != NAV::NavStatus::NAV_OK)
        return EXIT_FAILURE;

    // Read Command Line Options
    NAV::Config* pConfig = NAV::Config::Get();
    if (NAV::NavStatus result = pConfig->AddOptions(argc, argv);
        result == NAV::NavStatus::NAV_REQUEST_EXIT)
        return EXIT_SUCCESS;
    else if (result != NAV::NavStatus::NAV_OK)
        return EXIT_FAILURE;

    // Write the Log Header
    NAV::Logger::writeHeader();

    if (pConfig->DecodeOptions() != NAV::NavStatus::NAV_OK)
        return EXIT_FAILURE;

    LOG_TRACE("Trace");
    LOG_DEBUG("Debug");
    LOG_INFO("Info");
    LOG_WARN("Warn");
    LOG_ERROR("Error");
    LOG_CRITICAL("Critical");

    if (pConfig->GetSigterm())
        NAV::Sleep::waitForSignal();
    else
        NAV::Sleep::countDownSeconds(pConfig->GetProgExecTime());

    NAV::Logger::writeFooter();
}