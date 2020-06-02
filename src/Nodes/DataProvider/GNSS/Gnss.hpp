/**
 * @file Gnss.hpp
 * @brief Abstract GNSS Data Provider Class
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-03-19
 */

#pragma once

#include "Nodes/Node.hpp"

#include "util/notidy/InsTime.hpp"

namespace NAV
{
/// Abstract GNSS Data Provider Class
class Gnss : public Node
{
  public:
    Gnss(const Gnss&) = delete;            ///< Copy constructor
    Gnss(Gnss&&) = delete;                 ///< Move constructor
    Gnss& operator=(const Gnss&) = delete; ///< Copy assignment operator
    Gnss& operator=(Gnss&&) = delete;      ///< Move assignment operator

  protected:
    /**
     * @brief Construct a new Gnss object
     * 
     * @param[in] name Name of the Gnss object
     * @param[in, out] options Program options string list
     */
    Gnss(const std::string& name, std::deque<std::string>& options);

    /// Default constructor
    Gnss() = default;

    /// Destructor
    ~Gnss() override = default;

    /// Current Ins Time
    std::optional<InsTime> currentInsTime;
};

} // namespace NAV
