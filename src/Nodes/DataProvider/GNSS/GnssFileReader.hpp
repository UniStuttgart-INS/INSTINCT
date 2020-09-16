/// @file GnssFileReader.hpp
/// @brief Abstract IMU FileReader Class
/// @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
/// @date 2020-03-12

#pragma once

#include "Gnss.hpp"
#include "Nodes/DataProvider/Protocol/FileReader.hpp"

namespace NAV
{
/// Abstract IMU FileReader Class
class GnssFileReader : public FileReader, public Gnss
{
  public:
    /// @brief Copy constructor
    GnssFileReader(const GnssFileReader&) = delete;
    /// @brief Move constructor
    GnssFileReader(GnssFileReader&&) = delete;
    /// @brief Copy assignment operator
    GnssFileReader& operator=(const GnssFileReader&) = delete;
    /// @brief Move assignment operator
    GnssFileReader& operator=(GnssFileReader&&) = delete;

    /// @brief Resets the node. Moves the read cursor to the start
    void resetNode() final;

  protected:
    /// @brief Constructor
    /// @param[in] name Name of the Node
    /// @param[in] options Program options string map
    GnssFileReader(const std::string& name, const std::map<std::string, std::string>& options);

    /// @brief Default constructor
    GnssFileReader() = default;

    /// @brief Destructor
    ~GnssFileReader() override = default;

    /// @brief Initialize the node
    void initialize() override;

    /// @brief Returns Gui Configuration options for the class
    /// @return The gui configuration
    [[nodiscard]] std::vector<ConfigOptions> guiConfig() const override
    {
        std::vector<ConfigOptions> configs = { { CONFIG_STRING, "Path", "Path to the File to read", { "" } },
                                               { CONFIG_STRING, "Time Start", "Lower time limit (GPST)\nFormat: 2020/01/01 - 00:00:00", { "0000/01/01 - 00:00:00" } },
                                               { CONFIG_STRING, "Time End", "Upper time limit (GPST)\nFormat: 2020/01/01 - 00:00:00", { "3000/01/01 - 00:00:00" } } };
        auto gnssConfigs = Gnss::guiConfig();
        configs.insert(configs.end(), gnssConfigs.begin(), gnssConfigs.end());
        return configs;
    }
};

} // namespace NAV
