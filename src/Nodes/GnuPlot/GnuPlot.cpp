#include "GnuPlot.hpp"

#include "util/Logger.hpp"

std::vector<std::shared_ptr<NAV::GnuPlot::GnuPlotWindow>> NAV::GnuPlot::plotWindows;

NAV::GnuPlot::GnuPlotWindow::GnuPlotWindow()
{
    LOG_TRACE("called");
    gp = new gnuplotio::Gnuplot();
}

NAV::GnuPlot::GnuPlotWindow::~GnuPlotWindow()
{
    LOG_TRACE("called");

    gp->clearTmpfiles();
    delete gp;
    gp = nullptr;
}

size_t NAV::GnuPlot::GnuPlotWindow::addNewDataSet(std::string legend)
{
    data.push_back({ legend, {} });
    return data.size() - 1;
}

NAV::GnuPlot::GnuPlot(std::string name, std::deque<std::string>& options)
    : Node(name)
{
    LOG_TRACE("called for {}", name);

    if (options.size() >= 1)
    {
        timeFrame = std::stod(options.at(0));
        options.pop_front();
    }

    if (options.size() >= 1)
    {
        updateFrequency = std::stod(options.at(0));
        options.pop_front();
    }

    for (auto it = options.begin(); it != options.end() && it + 1 != options.end() && it + 2 != options.end(); it = options.begin())
    {
        int wIndex = std::stoi(*(it + 2));
        if (wIndex >= 0)
        {
            dataToPlot.push_back(std::make_tuple(*it, *(it + 1), wIndex));

            while (static_cast<size_t>(wIndex) + 1 > plotWindows.size())
                plotWindows.push_back(std::make_shared<GnuPlotWindow>());
        }

        options.pop_front();
        options.pop_front();
        options.pop_front();
    }

    for (auto& plotWindow : plotWindows)
    {
        *plotWindow->gp << "set autoscale xy\n";

        *plotWindow->gp << "set grid ytics lc rgb \"#bbbbbb\" lw 1 lt 0\n";
        *plotWindow->gp << "set grid xtics lc rgb \"#bbbbbb\" lw 1 lt 0\n";

        *plotWindow->gp << "set pointsize 0\n";
    }
}

NAV::GnuPlot::~GnuPlot() {}

NAV::NavStatus NAV::GnuPlot::updateWindows(std::shared_ptr<NAV::GnuPlot> obj)
{
    // Check if update Interval is reached
    bool plot = false;
    static double lastPlotTime = 0;
    for (auto& plotWindow : obj->plotWindows)
    {
        for (auto& plotData : plotWindow->data)
        {
            if (plotData.xy.back().first - lastPlotTime >= 1.0 / obj->updateFrequency)
            {
                plot = true;
                break;
            }
        }
        if (plot)
            break;
    }

    if (plot)
    {
        for (auto& plotWindow : obj->plotWindows)
        {
            if (!plotWindow->data.empty())
            {
                *plotWindow->gp << "plot";

                // for (auto &plotData : plotWindow->data)
                for (size_t i = 0; i < plotWindow->data.size(); i++)
                {
                    // Remove data which is too old
                    while (obj->timeFrame && plotWindow->data.at(i).xy.back().first - plotWindow->data.at(i).xy.front().first > obj->timeFrame)
                        plotWindow->data.at(i).xy.pop_front();

                    std::string lineStyle = "lines";
                    if (i >= 8)
                        lineStyle = "points";

                    *plotWindow->gp << plotWindow->gp->binFile1d(plotWindow->data.at(i).xy, "record") << "with " << lineStyle << " title '" << plotWindow->data.at(i).legend << "',";

                    if (plotWindow->data.at(i).xy.back().first > lastPlotTime)
                        lastPlotTime = plotWindow->data.at(i).xy.back().first;
                }

                *plotWindow->gp << "\n";
                plotWindow->gp->flush();
            }
        }
    }

    return NavStatus::NAV_OK;
}