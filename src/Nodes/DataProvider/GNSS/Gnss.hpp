/// @file Gnss.hpp
/// @brief Abstract GNSS Data Provider Class
/// @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
/// @date 2020-03-19

#pragma once

#include "Nodes/Node.hpp"

#include "util/notidy/InsTime.hpp"

namespace NAV
{
/// Abstract GNSS Data Provider Class
class Gnss : public Node
{
  public:
    /// @brief Copy constructor
    Gnss(const Gnss&) = delete;
    /// @brief Move constructor
    Gnss(Gnss&&) = delete;
    /// @brief Copy assignment operator
    Gnss& operator=(const Gnss&) = delete;
    /// @brief Move assignment operator
    Gnss& operator=(Gnss&&) = delete;

  protected:
    /// @brief Constructor
    /// @param[in] name Name of the Gnss object
    /// @param[in] options Program options string map
    Gnss(const std::string& name, const std::map<std::string, std::string>& options);

    /// Default constructor
    Gnss() = default;

    /// Destructor
    ~Gnss() override = default;

    /// Current Ins Time
    std::optional<InsTime> currentInsTime;
};

} // namespace NAV
