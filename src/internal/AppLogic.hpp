/// @file Application.hpp
/// @brief Application logic
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-08-17

#pragma once

namespace NAV::AppLogic
{
/// @brief Processes the command line arguments
/// @param[in] argc Argument Count
/// @param[in] argv Argument Values
int processCommandLineArguments(int argc, const char* argv[]); // NOLINT(cppcoreguidelines-avoid-c-arrays,hicpp-avoid-c-arrays,modernize-avoid-c-arrays)

} // namespace NAV::AppLogic