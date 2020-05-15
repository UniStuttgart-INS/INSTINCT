/**
 * @file ConfigManager.hpp
 * @brief Config management for the Project
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-03-11
 */

#pragma once

#include <vector>
#include <deque>
#include <memory>

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>

namespace NAV
{
/// Config management Class for the Project
class ConfigManager
{
  public:
    ConfigManager();                                         /// Constructor
    ~ConfigManager() = default;                              /// Destructor
    ConfigManager(const ConfigManager&) = delete;            ///< Copy constructor
    ConfigManager(ConfigManager&&) = delete;                 ///< Move constructor
    ConfigManager& operator=(const ConfigManager&) = delete; ///< Copy assignment operator
    ConfigManager& operator=(ConfigManager&&) = delete;      ///< Move assignment operator

    [[nodiscard]] static const boost::program_options::options_description& GetProgramOptions();

    static void FetchConfigs(const int argc, const char* argv[]); // NOLINT

    /// Retrieves the value of a corresponding key from the configuration, if one exists.
    template<typename T>
    static const T& Get(const std::string& key, const T& defaultValue)
    {
        if (vm.contains(key))
        {
            return vm[key].as<T>();
        }

        return defaultValue;
    }

    /// Checks if a corresponding key exists in the configuration.
    static bool HasKey(const std::string& key);

    /// Returns all keys in the configuration, as a vector.
    static std::vector<std::string> GetKeys();

  private:
    /// Program option description
    static boost::program_options::options_description program_options;

    /// Map which stores all options
    static boost::program_options::variables_map vm;
};

} // namespace NAV
