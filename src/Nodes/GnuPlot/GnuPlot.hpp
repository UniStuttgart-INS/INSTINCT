/**
 * @file GnuPlot.hpp
 * @brief Abstract class for Gnuplotting
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-04-14
 */

#pragma once

#include "gnuplot-iostream.h"

#include "../Node.hpp"

namespace NAV
{
/// Abstract class for Gnuplotting
class GnuPlot : public Node
{
  protected:
    /// Gnuplot object
    Gnuplot gp;

    /**
     * @brief Construct a new Gnu Plot object
     * 
     * @param[in] name Name of the Node
     */
    GnuPlot(std::string name);

    /// Default Destructor
    virtual ~GnuPlot();

    double timeFrame = 10.0f;
};

} // namespace NAV
