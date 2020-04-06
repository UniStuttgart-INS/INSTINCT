/**
 * @file Gnss.hpp
 * @brief Abstract GNSS Data Provider Class
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-03-19
 */

#pragma once

#include "Nodes/Node.hpp"
#include "../DataProvider.hpp"

namespace NAV
{
/// Abstract GNSS Data Provider Class
class Gnss : public Node, public DataProvider
{
  protected:
    /**
     * @brief Construct a new Gnss object
     * 
     * @param[in] name Name of the Gnss object
     */
    Gnss(std::string name);

    /// Destroy the Gnss object
    ~Gnss();
};

} // namespace NAV
