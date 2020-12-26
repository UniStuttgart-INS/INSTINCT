/// @file ImuFileReader.hpp
/// @brief Abstract IMU FileReader Class
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-03-12

#pragma once

#include "Imu.hpp"
#include "Nodes/DataProvider/Protocol/FileReader.hpp"

namespace NAV
{
/// Abstract IMU FileReader Class
class ImuFileReader : public FileReader, public Imu
{
  public:
    /// @brief Copy constructor
    ImuFileReader(const ImuFileReader&) = delete;
    /// @brief Move constructor
    ImuFileReader(ImuFileReader&&) = delete;
    /// @brief Copy assignment operator
    ImuFileReader& operator=(const ImuFileReader&) = delete;
    /// @brief Move assignment operator
    ImuFileReader& operator=(ImuFileReader&&) = delete;

    /// @brief Resets the node. Moves the read cursor to the start
    void resetNode() final;

  protected:
    /// @brief Constructor
    /// @param[in] name Name of the Node
    /// @param[in] options Program options string map
    ImuFileReader(const std::string& name, const std::map<std::string, std::string>& options);

    /// @brief Default constructor
    ImuFileReader() = default;

    /// @brief Destructor
    ~ImuFileReader() override = default;

    /// @brief Initialize the node
    void initialize() override;

    /// @brief Returns Gui Configuration options for the class
    /// @return The gui configuration
    [[nodiscard]] std::vector<ConfigOptions> guiConfig() const override
    {
        std::vector<ConfigOptions> configs = { { CONFIG_STRING, "Path", "Path to the File to read", { "" } },
                                               { CONFIG_STRING, "Time Start", "Lower time limit (GPST)\nFormat: 2020/01/01 - 00:00:00", { "0000/01/01 - 00:00:00" } },
                                               { CONFIG_STRING, "Time End", "Upper time limit (GPST)\nFormat: 2020/01/01 - 00:00:00", { "3000/01/01 - 00:00:00" } } };
        auto imuConfigs = Imu::guiConfig();
        configs.insert(configs.end(), imuConfigs.begin(), imuConfigs.end());
        return configs;
    }
};

} // namespace NAV
