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
#include <tuple>

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

        /// Destructor
        ~GnuPlotWindow();

        GnuPlotWindow(const GnuPlotWindow&) = delete;            ///< Copy constructor
        GnuPlotWindow(GnuPlotWindow&&) = delete;                 ///< Move constructor
        GnuPlotWindow& operator=(const GnuPlotWindow&) = delete; ///< Copy assignment operator
        GnuPlotWindow& operator=(GnuPlotWindow&&) = delete;      ///< Move assignment operator

        size_t addNewDataSet(const std::string& legend);

        std::string xLabel;
    };

    /**
     * @brief Construct a new Gnu Plot object
     * 
     * @param[in] name Name of the Node
     * @param[in, out] options Program options string list
     */
    GnuPlot(const std::string& name, std::deque<std::string>& options);

    GnuPlot() = default;                         ///< Default Constructor
    ~GnuPlot() override = default;               ///< Destructor
    GnuPlot(const GnuPlot&) = delete;            ///< Copy constructor
    GnuPlot(GnuPlot&&) = delete;                 ///< Move constructor
    GnuPlot& operator=(const GnuPlot&) = delete; ///< Copy assignment operator
    GnuPlot& operator=(GnuPlot&&) = delete;      ///< Move assignment operator

    static bool update();

    void requestUpdate() const;

  protected:
    /// Vector of Instructions what to plot
    /// xData, yData, windowIndex, dataSetIndices
    std::vector<std::tuple<std::string, std::string, size_t, std::vector<size_t>>> dataToPlot;

    static std::vector<std::shared_ptr<GnuPlotWindow>> plotWindows;

    double timeFrame = 10.0;
    double updateFrequency = 10.0;
};

} // namespace NAV
