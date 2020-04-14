#include "VectorNavGnuPlot.hpp"

#include "util/Logger.hpp"
#include "util/Config.hpp"
#include "NodeData/IMU/VectorNavObs.hpp"

#include <tuple>
#include <cmath>
#include <deque>

NAV::VectorNavGnuPlot::VectorNavGnuPlot(std::string name, std::vector<std::string> options)
    : GnuPlot(name)
{
    LOG_TRACE("called for {}", name);

    if (options.size() >= 1)
        timeFrame = std::stod(options.at(0));
}

NAV::VectorNavGnuPlot::~VectorNavGnuPlot()
{
    deinitialize();
}

NAV::NavStatus NAV::VectorNavGnuPlot::initialize()
{
    LOG_TRACE("called for {}", name);

    NAV::Config* pConfig = NAV::Config::Get();

    if (!pConfig->GetSigterm())
    {
        gp << "set autoscale y\n";
        gp << "set xrange [0:" << pConfig->GetProgExecTime() << "]\n";
    }
    else
        gp << "set autoscale xy\n";

    gp << "set grid ytics lc rgb \"#bbbbbb\" lw 1 lt 0\n";
    gp << "set grid xtics lc rgb \"#bbbbbb\" lw 1 lt 0\n";

    return NavStatus::NAV_OK;
}

NAV::NavStatus NAV::VectorNavGnuPlot::deinitialize()
{
    LOG_TRACE("called for {}", name);

    return NavStatus::NAV_OK;
}

NAV::NavStatus NAV::VectorNavGnuPlot::plotVectorNavObs(std::shared_ptr<void> observation, std::shared_ptr<void> userData)
{
    VectorNavGnuPlot* plot = static_cast<VectorNavGnuPlot*>(userData.get());
    VectorNavObs* obs = static_cast<VectorNavObs*>(observation.get());

    if (!plot->timeStart)
        plot->timeStart = obs->timeSinceStartup.value();

    static std::deque<std::pair<double, double>> gyroCompX;
    static std::deque<std::pair<double, double>> gyroCompY;
    static std::deque<std::pair<double, double>> gyroCompZ;

    static std::deque<std::pair<double, double>> accelCompX;
    static std::deque<std::pair<double, double>> accelCompY;
    static std::deque<std::pair<double, double>> accelCompZ;

    static std::deque<std::pair<double, double>> magCompX;
    static std::deque<std::pair<double, double>> magCompY;
    static std::deque<std::pair<double, double>> magCompZ;

    double time = static_cast<double>(obs->timeSinceStartup.value() - plot->timeStart) / std::pow(10, 9);

    gyroCompX.push_back(std::make_pair(time, obs->gyroCompXYZ.value().x()));
    gyroCompY.push_back(std::make_pair(time, obs->gyroCompXYZ.value().y()));
    gyroCompZ.push_back(std::make_pair(time, obs->gyroCompXYZ.value().z()));

    accelCompX.push_back(std::make_pair(time, obs->accelCompXYZ.value().x()));
    accelCompY.push_back(std::make_pair(time, obs->accelCompXYZ.value().y()));
    accelCompZ.push_back(std::make_pair(time, obs->accelCompXYZ.value().z()));

    magCompX.push_back(std::make_pair(time, obs->magCompXYZ.value().x()));
    magCompY.push_back(std::make_pair(time, obs->magCompXYZ.value().y()));
    magCompZ.push_back(std::make_pair(time, obs->magCompXYZ.value().z()));

    if (plot->timeFrame && time - gyroCompX.front().first > plot->timeFrame)
    {
        gyroCompX.pop_front();
        gyroCompY.pop_front();
        gyroCompZ.pop_front();

        accelCompX.pop_front();
        accelCompY.pop_front();
        accelCompZ.pop_front();

        magCompX.pop_front();
        magCompY.pop_front();
        magCompZ.pop_front();
    }

    static double lastDraw = gyroCompX.back().first;
    if (time - lastDraw > 1.0 / 10)
    {
        plot->gp << "set multiplot layout 2, 2 title \"VectorNav Data\" font \",14\"\n";
        plot->gp << "set tmargin 2\n";

        plot->gp << "set title \"Compensated Angular Rates [rad/s]\"\n";
        plot->gp << "plot "
                 << plot->gp.binFile1d(gyroCompX, "record") << "with lines title 'X',"
                 << plot->gp.binFile1d(gyroCompY, "record") << "with lines title 'Y',"
                 << plot->gp.binFile1d(gyroCompZ, "record") << "with lines title 'Z'\n";

        plot->gp << "set title \"Compensated Acceleration [m/s^2]\"\n";
        plot->gp << "plot "
                 << plot->gp.binFile1d(accelCompX, "record") << "with lines title 'X',"
                 << plot->gp.binFile1d(accelCompY, "record") << "with lines title 'Y',"
                 << plot->gp.binFile1d(accelCompZ, "record") << "with lines title 'Z'\n";

        plot->gp << "set title \"Compensated Magnetic Field [Gauss]\"\n";
        plot->gp << "plot "
                 << plot->gp.binFile1d(magCompX, "record") << "with lines title 'X',"
                 << plot->gp.binFile1d(magCompY, "record") << "with lines title 'Y',"
                 << plot->gp.binFile1d(magCompZ, "record") << "with lines title 'Z'\n";

        // plot->gp << "set title \"Compensated Acceleration [m/s^2]\"\n";
        // plot->gp << "plot "
        //          << plot->gp.binFile1d(accelCompX, "record") << "with lines title 'X',"
        //          << plot->gp.binFile1d(accelCompY, "record") << "with lines title 'Y',"
        //          << plot->gp.binFile1d(accelCompZ, "record") << "with lines title 'Z'\n";

        plot->gp << "unset multiplot\n";
        plot->gp.flush();
        lastDraw = gyroCompX.back().first;
    }

    return NavStatus::NAV_OK;
}