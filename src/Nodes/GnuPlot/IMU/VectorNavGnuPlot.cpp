#include "VectorNavGnuPlot.hpp"

#include "util/Logger.hpp"
#include "NodeData/IMU/VectorNavObs.hpp"

#include <tuple>
#include <cmath>

NAV::VectorNavGnuPlot::VectorNavGnuPlot(std::string name, std::deque<std::string>& options)
    : GnuPlot(name, options)
{
    LOG_TRACE("called for {}", name);
}

NAV::VectorNavGnuPlot::~VectorNavGnuPlot()
{
    LOG_TRACE("called for {}", name);
}

NAV::NavStatus NAV::VectorNavGnuPlot::plotVectorNavObs(std::shared_ptr<NAV::NodeData> observation, std::shared_ptr<NAV::Node> userData)
{
    auto obj = std::static_pointer_cast<VectorNavGnuPlot>(userData);
    auto obs = std::static_pointer_cast<VectorNavObs>(observation);

    if (!obj->timeStart)
        obj->timeStart = obs->timeSinceStartup.value();

    double plotX = static_cast<double>(obs->timeSinceStartup.value() - obj->timeStart) / std::pow(10, 9);

    for (auto& [xData, yData, wIndex] : obj->dataToPlot)
    {
        auto plotWindow = GnuPlot::plotWindows.at(wIndex);

        if (yData == "timeSinceStartup" && obs->timeSinceStartup.has_value())
        {
            static size_t dataIndex0 = plotWindow->addNewDataSet("Time Since Startup [ns]");

            plotWindow->data.at(dataIndex0).xy.push_back(std::make_pair(plotX, obs->timeSinceStartup.value()));
        }
        else if (yData == "quaternion" && obs->quaternion.has_value())
        {
            static size_t dataIndex0 = plotWindow->addNewDataSet("Attitude Quaternion X");
            static size_t dataIndex1 = plotWindow->addNewDataSet("Attitude Quaternion Y");
            static size_t dataIndex2 = plotWindow->addNewDataSet("Attitude Quaternion Z");
            static size_t dataIndex3 = plotWindow->addNewDataSet("Attitude Quaternion W");

            plotWindow->data.at(dataIndex0).xy.push_back(std::make_pair(plotX, obs->quaternion.value().x()));
            plotWindow->data.at(dataIndex1).xy.push_back(std::make_pair(plotX, obs->quaternion.value().y()));
            plotWindow->data.at(dataIndex2).xy.push_back(std::make_pair(plotX, obs->quaternion.value().z()));
            plotWindow->data.at(dataIndex3).xy.push_back(std::make_pair(plotX, obs->quaternion.value().w()));
        }
        else if (yData == "magUncompXYZ" && obs->magUncompXYZ.has_value())
        {
            static size_t dataIndex0 = plotWindow->addNewDataSet("Uncompensated Magnetic Field X [Gauss]");
            static size_t dataIndex1 = plotWindow->addNewDataSet("Uncompensated Magnetic Field Y [Gauss]");
            static size_t dataIndex2 = plotWindow->addNewDataSet("Uncompensated Magnetic Field Z [Gauss]");

            plotWindow->data.at(dataIndex0).xy.push_back(std::make_pair(plotX, obs->magUncompXYZ.value().x()));
            plotWindow->data.at(dataIndex1).xy.push_back(std::make_pair(plotX, obs->magUncompXYZ.value().y()));
            plotWindow->data.at(dataIndex2).xy.push_back(std::make_pair(plotX, obs->magUncompXYZ.value().z()));
        }
        else if (yData == "accelUncompXYZ" && obs->accelUncompXYZ.has_value())
        {
            static size_t dataIndex0 = plotWindow->addNewDataSet("Uncompensated Acceleration X [m/s^2]");
            static size_t dataIndex1 = plotWindow->addNewDataSet("Uncompensated Acceleration Y [m/s^2]");
            static size_t dataIndex2 = plotWindow->addNewDataSet("Uncompensated Acceleration Z [m/s^2]");

            plotWindow->data.at(dataIndex0).xy.push_back(std::make_pair(plotX, obs->accelUncompXYZ.value().x()));
            plotWindow->data.at(dataIndex1).xy.push_back(std::make_pair(plotX, obs->accelUncompXYZ.value().y()));
            plotWindow->data.at(dataIndex2).xy.push_back(std::make_pair(plotX, obs->accelUncompXYZ.value().z()));
        }
        else if (yData == "gyroUncompXYZ" && obs->gyroUncompXYZ.has_value())
        {
            static size_t dataIndex0 = plotWindow->addNewDataSet("Uncompensated Angular Rates X [rad/s]");
            static size_t dataIndex1 = plotWindow->addNewDataSet("Uncompensated Angular Rates Y [rad/s]");
            static size_t dataIndex2 = plotWindow->addNewDataSet("Uncompensated Angular Rates Z [rad/s]");

            plotWindow->data.at(dataIndex0).xy.push_back(std::make_pair(plotX, obs->gyroUncompXYZ.value().x()));
            plotWindow->data.at(dataIndex1).xy.push_back(std::make_pair(plotX, obs->gyroUncompXYZ.value().y()));
            plotWindow->data.at(dataIndex2).xy.push_back(std::make_pair(plotX, obs->gyroUncompXYZ.value().z()));
        }
        else if (yData == "magCompXYZ" && obs->magCompXYZ.has_value())
        {
            static size_t dataIndex0 = plotWindow->addNewDataSet("Compensated Magnetic Field X [Gauss]");
            static size_t dataIndex1 = plotWindow->addNewDataSet("Compensated Magnetic Field Y [Gauss]");
            static size_t dataIndex2 = plotWindow->addNewDataSet("Compensated Magnetic Field Z [Gauss]");

            plotWindow->data.at(dataIndex0).xy.push_back(std::make_pair(plotX, obs->magCompXYZ.value().x()));
            plotWindow->data.at(dataIndex1).xy.push_back(std::make_pair(plotX, obs->magCompXYZ.value().y()));
            plotWindow->data.at(dataIndex2).xy.push_back(std::make_pair(plotX, obs->magCompXYZ.value().z()));
        }
        else if (yData == "accelCompXYZ" && obs->accelCompXYZ.has_value())
        {
            static size_t dataIndex0 = plotWindow->addNewDataSet("Compensated Acceleration X [m/s^2]");
            static size_t dataIndex1 = plotWindow->addNewDataSet("Compensated Acceleration Y [m/s^2]");
            static size_t dataIndex2 = plotWindow->addNewDataSet("Compensated Acceleration Z [m/s^2]");

            plotWindow->data.at(dataIndex0).xy.push_back(std::make_pair(plotX, obs->accelCompXYZ.value().x()));
            plotWindow->data.at(dataIndex1).xy.push_back(std::make_pair(plotX, obs->accelCompXYZ.value().y()));
            plotWindow->data.at(dataIndex2).xy.push_back(std::make_pair(plotX, obs->accelCompXYZ.value().z()));
        }
        else if (yData == "gyroCompXYZ" && obs->gyroCompXYZ.has_value())
        {
            static size_t dataIndex0 = plotWindow->addNewDataSet("Compensated Angular Rates X [rad/s]");
            static size_t dataIndex1 = plotWindow->addNewDataSet("Compensated Angular Rates Y [rad/s]");
            static size_t dataIndex2 = plotWindow->addNewDataSet("Compensated Angular Rates Z [rad/s]");

            plotWindow->data.at(dataIndex0).xy.push_back(std::make_pair(plotX, obs->gyroCompXYZ.value().x()));
            plotWindow->data.at(dataIndex1).xy.push_back(std::make_pair(plotX, obs->gyroCompXYZ.value().y()));
            plotWindow->data.at(dataIndex2).xy.push_back(std::make_pair(plotX, obs->gyroCompXYZ.value().z()));
        }
        else if (yData == "syncInCnt" && obs->syncInCnt.has_value())
        {
            static size_t dataIndex0 = plotWindow->addNewDataSet("Sync In Count [-]");

            plotWindow->data.at(dataIndex0).xy.push_back(std::make_pair(plotX, obs->syncInCnt.value()));
        }
        else if (yData == "dtime" && obs->dtime.has_value())
        {
            static size_t dataIndex0 = plotWindow->addNewDataSet("Delta Time Integration Interval [s]");

            plotWindow->data.at(dataIndex0).xy.push_back(std::make_pair(plotX, obs->dtime.value()));
        }
        else if (yData == "dtheta" && obs->dtheta.has_value())
        {
            static size_t dataIndex0 = plotWindow->addNewDataSet("Delta rotation angles X [Degree]");
            static size_t dataIndex1 = plotWindow->addNewDataSet("Delta rotation angles Y [Degree]");
            static size_t dataIndex2 = plotWindow->addNewDataSet("Delta rotation angles Z [Degree]");

            plotWindow->data.at(dataIndex0).xy.push_back(std::make_pair(plotX, obs->dtheta.value().x()));
            plotWindow->data.at(dataIndex1).xy.push_back(std::make_pair(plotX, obs->dtheta.value().y()));
            plotWindow->data.at(dataIndex2).xy.push_back(std::make_pair(plotX, obs->dtheta.value().z()));
        }
        else if (yData == "dvel" && obs->dvel.has_value())
        {
            static size_t dataIndex0 = plotWindow->addNewDataSet("Delta velocity X [m/s]");
            static size_t dataIndex1 = plotWindow->addNewDataSet("Delta velocity Y [m/s]");
            static size_t dataIndex2 = plotWindow->addNewDataSet("Delta velocity Z [m/s]");

            plotWindow->data.at(dataIndex0).xy.push_back(std::make_pair(plotX, obs->dvel.value().x()));
            plotWindow->data.at(dataIndex1).xy.push_back(std::make_pair(plotX, obs->dvel.value().y()));
            plotWindow->data.at(dataIndex2).xy.push_back(std::make_pair(plotX, obs->dvel.value().z()));
        }
        else if (yData == "vpeStatus" && obs->vpeStatus.has_value())
        {
            static size_t dataIndex0 = plotWindow->addNewDataSet("VPE Status Bitfield");

            plotWindow->data.at(dataIndex0).xy.push_back(std::make_pair(plotX, obs->vpeStatus.value()));
        }
        else if (yData == "temperature" && obs->temperature.has_value())
        {
            static size_t dataIndex0 = plotWindow->addNewDataSet("Temperature [Celsius]");

            plotWindow->data.at(dataIndex0).xy.push_back(std::make_pair(plotX, obs->temperature.value()));
        }
        else if (yData == "pressure" && obs->pressure.has_value())
        {
            static size_t dataIndex0 = plotWindow->addNewDataSet("Absolute IMU pressure [kPa]");

            plotWindow->data.at(dataIndex0).xy.push_back(std::make_pair(plotX, obs->pressure.value()));
        }
        else if (yData == "magCompNED" && obs->magCompNED.has_value())
        {
            static size_t dataIndex0 = plotWindow->addNewDataSet("Compensated Magnetic Field N [Gauss]");
            static size_t dataIndex1 = plotWindow->addNewDataSet("Compensated Magnetic Field E [Gauss]");
            static size_t dataIndex2 = plotWindow->addNewDataSet("Compensated Magnetic Field D [Gauss]");

            plotWindow->data.at(dataIndex0).xy.push_back(std::make_pair(plotX, obs->magCompNED.value().x()));
            plotWindow->data.at(dataIndex1).xy.push_back(std::make_pair(plotX, obs->magCompNED.value().y()));
            plotWindow->data.at(dataIndex2).xy.push_back(std::make_pair(plotX, obs->magCompNED.value().z()));
        }
        else if (yData == "accelCompNED" && obs->accelCompNED.has_value())
        {
            static size_t dataIndex0 = plotWindow->addNewDataSet("Compensated Acceleration N [m/s^2]");
            static size_t dataIndex1 = plotWindow->addNewDataSet("Compensated Acceleration E [m/s^2]");
            static size_t dataIndex2 = plotWindow->addNewDataSet("Compensated Acceleration D [m/s^2]");

            plotWindow->data.at(dataIndex0).xy.push_back(std::make_pair(plotX, obs->accelCompNED.value().x()));
            plotWindow->data.at(dataIndex1).xy.push_back(std::make_pair(plotX, obs->accelCompNED.value().y()));
            plotWindow->data.at(dataIndex2).xy.push_back(std::make_pair(plotX, obs->accelCompNED.value().z()));
        }
        else if (yData == "gyroCompNED" && obs->gyroCompNED.has_value())
        {
            static size_t dataIndex0 = plotWindow->addNewDataSet("Compensated Angular Rates N [rad/s]");
            static size_t dataIndex1 = plotWindow->addNewDataSet("Compensated Angular Rates E [rad/s]");
            static size_t dataIndex2 = plotWindow->addNewDataSet("Compensated Angular Rates D [rad/s]");

            plotWindow->data.at(dataIndex0).xy.push_back(std::make_pair(plotX, obs->gyroCompNED.value().x()));
            plotWindow->data.at(dataIndex1).xy.push_back(std::make_pair(plotX, obs->gyroCompNED.value().y()));
            plotWindow->data.at(dataIndex2).xy.push_back(std::make_pair(plotX, obs->gyroCompNED.value().z()));
        }
        else if (yData == "linearAccelXYZ" && obs->linearAccelXYZ.has_value())
        {
            static size_t dataIndex0 = plotWindow->addNewDataSet("Linear Acceleration X (w/o g) [m/s^2]");
            static size_t dataIndex1 = plotWindow->addNewDataSet("Linear Acceleration Y (w/o g) [m/s^2]");
            static size_t dataIndex2 = plotWindow->addNewDataSet("Linear Acceleration Z (w/o g) [m/s^2]");

            plotWindow->data.at(dataIndex0).xy.push_back(std::make_pair(plotX, obs->linearAccelXYZ.value().x()));
            plotWindow->data.at(dataIndex1).xy.push_back(std::make_pair(plotX, obs->linearAccelXYZ.value().y()));
            plotWindow->data.at(dataIndex2).xy.push_back(std::make_pair(plotX, obs->linearAccelXYZ.value().z()));
        }
        else if (yData == "linearAccelNED" && obs->linearAccelNED.has_value())
        {
            static size_t dataIndex0 = plotWindow->addNewDataSet("Linear Acceleration N (w/o g) [m/s^2]");
            static size_t dataIndex1 = plotWindow->addNewDataSet("Linear Acceleration E (w/o g) [m/s^2]");
            static size_t dataIndex2 = plotWindow->addNewDataSet("Linear Acceleration D (w/o g) [m/s^2]");

            plotWindow->data.at(dataIndex0).xy.push_back(std::make_pair(plotX, obs->linearAccelNED.value().x()));
            plotWindow->data.at(dataIndex1).xy.push_back(std::make_pair(plotX, obs->linearAccelNED.value().y()));
            plotWindow->data.at(dataIndex2).xy.push_back(std::make_pair(plotX, obs->linearAccelNED.value().z()));
        }
        else if (yData == "yawPitchRollUncertainty" && obs->yawPitchRollUncertainty.has_value())
        {
            static size_t dataIndex0 = plotWindow->addNewDataSet("Yaw Uncertainty (1 Sigma) [Degree]");
            static size_t dataIndex1 = plotWindow->addNewDataSet("Pitch Uncertainty (1 Sigma) [Degree]");
            static size_t dataIndex2 = plotWindow->addNewDataSet("Roll Uncertainty (1 Sigma) [Degree]");

            plotWindow->data.at(dataIndex0).xy.push_back(std::make_pair(plotX, obs->yawPitchRollUncertainty.value().x()));
            plotWindow->data.at(dataIndex1).xy.push_back(std::make_pair(plotX, obs->yawPitchRollUncertainty.value().y()));
            plotWindow->data.at(dataIndex2).xy.push_back(std::make_pair(plotX, obs->yawPitchRollUncertainty.value().z()));
        }
    }

    return updateWindows(obj);
}