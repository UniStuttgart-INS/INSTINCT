#include <iostream>

#include "util/Logger.hpp"
#include "internal/Version.hpp"

#include "internal/AppLogic.hpp"
#include "internal/ConfigManager.hpp"

int Main(int argc, const char* argv[]) // NOLINT(cppcoreguidelines-avoid-c-arrays,hicpp-avoid-c-arrays,modernize-avoid-c-arrays)
{
    // Config Manager object
    NAV::ConfigManager::initialize();

    if (argc == 2)
    {
        // User requested the version of the program
        if (!strcmp(argv[1], "--version") || !strcmp(argv[1], "-v"))
        {
            std::cout << PROJECT_VERSION_STRING << '\n';
            return EXIT_SUCCESS;
        }

        // User requested the help text of the program
        if (!strcmp(argv[1], "--help") || !strcmp(argv[1], "-h"))
        {
            std::cout << "INSTINCT " << PROJECT_VERSION_STRING << " - INS Toolkit for Integrated Navigation Concepts and Training\n\n"
                      << NAV::ConfigManager::GetProgramOptions() << '\n';
            return EXIT_SUCCESS;
        }
    }

    try
    {
        Logger logger("logs/instinct.log");

        return NAV::AppLogic::processCommandLineArguments(argc, argv);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Critical Event occurred: " << e.what() << '\n';
        return EXIT_FAILURE;
    }
}