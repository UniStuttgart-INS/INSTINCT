/// @file StringObs.hpp
/// @brief Wrapper for String Messages
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-07-01

#pragma once

#include <string>

#include "NodeData/InsObs.hpp"

namespace NAV
{
/// IMU Observation storage class
class StringObs : public InsObs
{
  public:
    /// @brief Constructor
    /// @param[in] str String to store in the observation
    explicit StringObs(std::string str)
        : data(std::move(str)) {}

    /// @brief Default constructor
    StringObs() = default;
    /// @brief Destructor
    ~StringObs() override = default;
    /// @brief Copy constructor
    StringObs(const StringObs&) = delete;
    /// @brief Move constructor
    StringObs(StringObs&&) = delete;
    /// @brief Copy assignment operator
    StringObs& operator=(const StringObs&) = delete;
    /// @brief Move assignment operator
    StringObs& operator=(StringObs&&) = delete;

    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type()
    {
        return std::string("StringObs");
    }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string> parentTypes()
    {
        return { InsObs::type() };
    }

    /// @brief The string to transport
    std::string data;
};

} // namespace NAV
