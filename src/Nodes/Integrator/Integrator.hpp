/**
 * @file Integrator.hpp
 * @brief Abstract Integrator class
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-03-24
 */

#pragma once

#include "Nodes/Node.hpp"

namespace NAV
{
/// Abstract Integrator class
class Integrator : public Node
{
  protected:
    /**
     * @brief Construct a new Integrator object
     * 
     * @param[in] name Name of the Integrator
     */
    Integrator(std::string name)
        : Node(name) {}

    /// Default destructor
    ~Integrator() {}
};

} // namespace NAV
