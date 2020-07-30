/// @file ConfigManager.hpp
/// @brief Config management for the Project
/// @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
/// @date 2020-03-11

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
    /// @brief Constructor
    ConfigManager();
    /// @brief Destructor
    ~ConfigManager() = default;
    /// @brief Copy constructor
    ConfigManager(const ConfigManager&) = delete;
    /// @brief Move constructor
    ConfigManager(ConfigManager&&) = delete;
    /// @brief Copy assignment operator
    ConfigManager& operator=(const ConfigManager&) = delete;
    /// @brief Move assignment operator
    ConfigManager& operator=(ConfigManager&&) = delete;

    /// @brief Get the Program Options object
    /// @return The object
    [[nodiscard]] static const boost::program_options::options_description& GetProgramOptions();

    /// @brief Fetches the configs from the command line parameters
    /// @param[in, out] argc Number of command line parameters
    /// @param[in, out] argv Array of the command line parameters
    static void FetchConfigs(const int argc, const char* argv[]); // NOLINT

    /// @brief Retrieves the value of a corresponding key from the configuration, if one exists.
    /// @tparam T Return value type
    /// @param[in] key Key to search for
    /// @param[in] defaultValue If key is not found, the default value is returned
    /// @return The value found with the key or the default value
    template<typename T>
    static const T& Get(const std::string& key, const T& defaultValue)
    {
        if (vm.count(key))
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
