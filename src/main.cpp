// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include <iostream>

#include "internal/Version.hpp"

#include "internal/AppLogic.hpp"
#include "internal/ConfigManager.hpp"

namespace
{

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
        return NAV::AppLogic::processCommandLineArguments(argc, argv);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Critical Event occurred: " << e.what() << '\n';
        return EXIT_FAILURE;
    }
}
} // namespace

#if defined(_WIN32) && !defined(_CONSOLE)
    #include <windows.h>
    #include <stdlib.h> // __argc, argv

int WINAPI WinMain(_In_ HINSTANCE hInstance, _In_opt_ HINSTANCE hPrevInstance, _In_ LPSTR lpCmdLine, _In_ int nShowCmd)
{
    return Main(__argc, const_cast<const char**>(__argv));
}

#else

int main(int argc, const char* argv[])
{
    return Main(argc, argv);
}

#endif
