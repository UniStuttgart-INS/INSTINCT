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
};

} // namespace NAV
