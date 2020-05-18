#include "GnuPlot.hpp"

#include "util/Logger.hpp"
#include "Nodes/NodeManager.hpp"

std::vector<std::shared_ptr<NAV::GnuPlot::GnuPlotWindow>> NAV::GnuPlot::plotWindows;

NAV::GnuPlot::GnuPlotWindow::GnuPlotWindow()
{
    LOG_TRACE("called");
    // NOLINTNEXTLINE
    gp = new gnuplotio::Gnuplot("gnuplot -persist > /dev/null 2>&1");
}

NAV::GnuPlot::GnuPlotWindow::~GnuPlotWindow()
{
    gp->clearTmpfiles();
    delete gp;
    gp = nullptr;
}

size_t NAV::GnuPlot::GnuPlotWindow::addNewDataSet(const std::string& legend)
{
    data.push_back({ legend, {} });
    return data.size() - 1;
}

NAV::GnuPlot::GnuPlot(const std::string& name, std::deque<std::string>& options)
    : Node(name)
{
    LOG_TRACE("called for {}", name);

    if (!options.empty())
    {
        timeFrame = std::stod(options.at(0));
        options.pop_front();
    }

    if (!options.empty())
    {
        updateFrequency = std::stod(options.at(0));
        options.pop_front();
    }

    for (auto it = options.begin(); it != options.end() && it + 1 != options.end() && it + 2 != options.end(); it = options.begin())
    {
        int wIndex = std::stoi(*(it + 2));
        if (wIndex >= 0)
        {
            dataToPlot.emplace_back(std::make_tuple(*it, *(it + 1), wIndex, std::vector<size_t>()));

            while (static_cast<size_t>(wIndex) + 1 > plotWindows.size())
            {
                plotWindows.push_back(std::make_shared<GnuPlotWindow>());
            }
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

void NAV::GnuPlot::requestUpdate() const
{
    if (NodeManager::appContext == NAV::Node::NodeContext::REAL_TIME)
    {
        // Check if update Interval is reached
        bool plot = false;
        static double lastPlotTime = 0;
        for (auto& plotWindow : plotWindows)
        {
            for (auto& plotData : plotWindow->data)
            {
                if (!plotData.xy.empty() && plotData.xy.back().first - lastPlotTime >= 1.0 / updateFrequency)
                {
                    plot = true;
                    lastPlotTime = plotData.xy.back().first;
                    break;
                }
            }
            if (plot)
            {
                break;
            }
        }

        // Update the GnuPlot Windows
        if (plot)
        {
            update();
        }
    }
}

bool NAV::GnuPlot::update()
{
    bool somethingWasPlotted = false;
    for (auto& plotWindow : plotWindows)
    {
        if (!plotWindow->data.empty())
        {
            *plotWindow->gp << "plot";

            // for (auto &plotData : plotWindow->data)
            for (size_t i = 0; i < plotWindow->data.size(); i++)
            {
                if (plotWindow->data.at(i).xy.size() >= 2)
                {
                    std::string lineStyle = "lines";
                    if (i >= 8)
                    {
                        lineStyle = "points";
                    }

                    *plotWindow->gp << plotWindow->gp->binFile1d(plotWindow->data.at(i).xy, "record") << "with " << lineStyle << " title '" << plotWindow->data.at(i).legend << "',";
                }
            }

            *plotWindow->gp << "\n";
            plotWindow->gp->flush();
            somethingWasPlotted = true;
        }
    }
    return somethingWasPlotted;
}