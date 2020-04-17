/**
 * @file GnuPlot.hpp
 * @brief Abstract class for Gnuplotting
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-04-14
 */

#pragma once

#include "../Node.hpp"

#include "gnuplot-iostream.h"
#include <memory>

namespace NAV
{
/// Abstract class for Gnuplotting
class GnuPlot : public Node
{
  public:
    class GnuPlotWindow
    {
      public:
        class GnuPlotData
        {
          public:
            /// Legend entry for the data
            std::string legend;
            /// x and y data which can be passed to the plot stream
            std::deque<std::pair<double, double>> xy;
        };

        gnuplotio::Gnuplot* gp;

        std::vector<GnuPlotData> data;

        /// Default Constructor
        GnuPlotWindow();

        /// Default Destructor
        ~GnuPlotWindow();

        size_t addNewDataSet(std::string legend);
    };

    /**
     * @brief Construct a new Gnu Plot object
     * 
     * @param[in] name Name of the Node
     * @param[in, out] options Program options string list
     */
    GnuPlot(std::string name, std::deque<std::string>& options);

    /// Default Destructor
    virtual ~GnuPlot();

    double timeFrame = 10.0;
    double updateFrequency = 10.0;

    std::vector<std::pair<std::string, size_t>> dataToPlot;

    static std::vector<std::shared_ptr<GnuPlotWindow>> plotWindows;

    static NavStatus updateWindows(std::shared_ptr<NAV::GnuPlot> obj);
};

} // namespace NAV
