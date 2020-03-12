/**
 * @file Config.hpp
 * @brief Config management for the Project
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-03-11
 */

#pragma once

#include "Common.hpp"
#include "Version.hpp"

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>

namespace NAV
{
/// Config management Class for the Project
class Config
{
  public:
    /// Construct a new Config object
    Config();
    /// Destroy the Config object
    ~Config();

    /**
     * @brief Returns a pointer to a static Config object
     * 
     * @retval Config* The static config object
     */
    static Config* Get();

    /**
     * @brief Adds the Command Line Parameters to the Variables Map
     * 
     * @param[in] argc Command Line Arguments Count
     * @param[in] argv Command Line Arguments Vector
     * @retval NavStatus Indicates whether there was a problem with the config
     */
    NavStatus AddOptions(const int argc, const char* argv[]);

    /**
     * @brief Reads the variable map and saves the values
     * 
     * @retval NavStatus Indicates whether all Options could be read
     */
    NavStatus DecodeOptions();

    bool GetSigterm();
    size_t GetProgExecTime();

  private:
    /// Program option description
    boost::program_options::options_description program_options{ "Allowed options" };

    /// Map which stores all options
    boost::program_options::variables_map vm;

    /// Specifies if the program waits for a signal
    bool sigterm = false;
    /// Program execution duration (disabled if Config::sigterm is true)
    size_t progExecTime = 0;
};

} // namespace NAV
