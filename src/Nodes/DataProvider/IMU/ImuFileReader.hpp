/// @file ImuFileReader.hpp
/// @brief Abstract IMU FileReader Class
/// @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
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
};

} // namespace NAV
