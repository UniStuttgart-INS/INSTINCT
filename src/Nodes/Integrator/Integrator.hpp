/**
 * @file Integrator.hpp
 * @brief Abstract Integrator Class
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-04-23
 */

#pragma once

#include "Nodes/Node.hpp"

namespace NAV
{
/// Abstract Integrator Class
class Integrator : public Node
{
  public:
    Integrator(const Integrator&) = delete;            ///< Copy constructor
    Integrator(Integrator&&) = delete;                 ///< Move constructor
    Integrator& operator=(const Integrator&) = delete; ///< Copy assignment operator
    Integrator& operator=(Integrator&&) = delete;      ///< Move assignment operator

  protected:
    /**
     * @brief Construct a new Integrator object
     * 
     * @param[in] name Name of the Node
     * @param[in, out] options Program options string list
     */
    Integrator(const std::string& name, std::deque<std::string>& options);

    /// Default Destructor
    ~Integrator() override;
};

} // namespace NAV
