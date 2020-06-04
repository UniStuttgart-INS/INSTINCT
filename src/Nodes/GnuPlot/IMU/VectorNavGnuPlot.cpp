#include "VectorNavGnuPlot.hpp"

#include "util/Logger.hpp"
#include "NodeData/IMU/VectorNavObs.hpp"
#include "Nodes/NodeManager.hpp"

#include <tuple>
#include <cmath>

NAV::VectorNavGnuPlot::VectorNavGnuPlot(const std::string& name, const std::map<std::string, std::string>& options)
    : GnuPlot(name, options)
{
    LOG_TRACE("called for {}", name);

    for (auto& [xData, yData, wIndex, dataIndices] : dataToPlot)
    {
        auto plotWindow = plotWindows.at(wIndex);

        if (plotWindow->xLabel.empty())
        {
            if (xData == "gpsToW")
            {
                plotWindow->xLabel = "GPS Time of Week [s]";
            }
        }

        if (yData == "timeSinceStartup")
        {
            dataIndices.push_back(plotWindow->addNewDataSet("Time Since Startup [ns]"));
        }
        else if (yData == "quaternion")
        {
            dataIndices.push_back(plotWindow->addNewDataSet("Attitude Quaternion X"));
            dataIndices.push_back(plotWindow->addNewDataSet("Attitude Quaternion Y"));
            dataIndices.push_back(plotWindow->addNewDataSet("Attitude Quaternion Z"));
            dataIndices.push_back(plotWindow->addNewDataSet("Attitude Quaternion W"));
        }
        else if (yData == "magUncompXYZ")
        {
            dataIndices.push_back(plotWindow->addNewDataSet("Uncompensated Magnetic Field X [Gauss]"));
            dataIndices.push_back(plotWindow->addNewDataSet("Uncompensated Magnetic Field Y [Gauss]"));
            dataIndices.push_back(plotWindow->addNewDataSet("Uncompensated Magnetic Field Z [Gauss]"));
        }
        else if (yData == "accelUncompXYZ")
        {
            dataIndices.push_back(plotWindow->addNewDataSet("Uncompensated Acceleration X [m/s^2]"));
            dataIndices.push_back(plotWindow->addNewDataSet("Uncompensated Acceleration Y [m/s^2]"));
            dataIndices.push_back(plotWindow->addNewDataSet("Uncompensated Acceleration Z [m/s^2]"));
        }
        else if (yData == "gyroUncompXYZ")
        {
            dataIndices.push_back(plotWindow->addNewDataSet("Uncompensated Angular Rates X [rad/s]"));
            dataIndices.push_back(plotWindow->addNewDataSet("Uncompensated Angular Rates Y [rad/s]"));
            dataIndices.push_back(plotWindow->addNewDataSet("Uncompensated Angular Rates Z [rad/s]"));
        }
        else if (yData == "magCompXYZ")
        {
            dataIndices.push_back(plotWindow->addNewDataSet("Compensated Magnetic Field X [Gauss]"));
            dataIndices.push_back(plotWindow->addNewDataSet("Compensated Magnetic Field Y [Gauss]"));
            dataIndices.push_back(plotWindow->addNewDataSet("Compensated Magnetic Field Z [Gauss]"));
        }
        else if (yData == "accelCompXYZ")
        {
            dataIndices.push_back(plotWindow->addNewDataSet("Compensated Acceleration X [m/s^2]"));
            dataIndices.push_back(plotWindow->addNewDataSet("Compensated Acceleration Y [m/s^2]"));
            dataIndices.push_back(plotWindow->addNewDataSet("Compensated Acceleration Z [m/s^2]"));
        }
        else if (yData == "gyroCompXYZ")
        {
            dataIndices.push_back(plotWindow->addNewDataSet("Compensated Angular Rates X [rad/s]"));
            dataIndices.push_back(plotWindow->addNewDataSet("Compensated Angular Rates Y [rad/s]"));
            dataIndices.push_back(plotWindow->addNewDataSet("Compensated Angular Rates Z [rad/s]"));
        }
        else if (yData == "syncInCnt")
        {
            dataIndices.push_back(plotWindow->addNewDataSet("Sync In Count [-]"));
        }
        else if (yData == "dtime")
        {
            dataIndices.push_back(plotWindow->addNewDataSet("Delta Time Integration Interval [s]"));
        }
        else if (yData == "dtheta")
        {
            dataIndices.push_back(plotWindow->addNewDataSet("Delta rotation angles X [Degree]"));
            dataIndices.push_back(plotWindow->addNewDataSet("Delta rotation angles Y [Degree]"));
            dataIndices.push_back(plotWindow->addNewDataSet("Delta rotation angles Z [Degree]"));
        }
        else if (yData == "dvel")
        {
            dataIndices.push_back(plotWindow->addNewDataSet("Delta velocity X [m/s]"));
            dataIndices.push_back(plotWindow->addNewDataSet("Delta velocity Y [m/s]"));
            dataIndices.push_back(plotWindow->addNewDataSet("Delta velocity Z [m/s]"));
        }
        else if (yData == "vpeStatus")
        {
            dataIndices.push_back(plotWindow->addNewDataSet("VPE Status Bitfield"));
        }
        else if (yData == "temperature")
        {
            dataIndices.push_back(plotWindow->addNewDataSet("Temperature [Celsius]"));
        }
        else if (yData == "pressure")
        {
            dataIndices.push_back(plotWindow->addNewDataSet("Absolute IMU pressure [kPa]"));
        }
        else if (yData == "magCompNED")
        {
            dataIndices.push_back(plotWindow->addNewDataSet("Compensated Magnetic Field N [Gauss]"));
            dataIndices.push_back(plotWindow->addNewDataSet("Compensated Magnetic Field E [Gauss]"));
            dataIndices.push_back(plotWindow->addNewDataSet("Compensated Magnetic Field D [Gauss]"));
        }
        else if (yData == "accelCompNED")
        {
            dataIndices.push_back(plotWindow->addNewDataSet("Compensated Acceleration N [m/s^2]"));
            dataIndices.push_back(plotWindow->addNewDataSet("Compensated Acceleration E [m/s^2]"));
            dataIndices.push_back(plotWindow->addNewDataSet("Compensated Acceleration D [m/s^2]"));
        }
        else if (yData == "gyroCompNED")
        {
            dataIndices.push_back(plotWindow->addNewDataSet("Compensated Angular Rates N [rad/s]"));
            dataIndices.push_back(plotWindow->addNewDataSet("Compensated Angular Rates E [rad/s]"));
            dataIndices.push_back(plotWindow->addNewDataSet("Compensated Angular Rates D [rad/s]"));
        }
        else if (yData == "linearAccelXYZ")
        {
            dataIndices.push_back(plotWindow->addNewDataSet("Linear Acceleration X (w/o g) [m/s^2]"));
            dataIndices.push_back(plotWindow->addNewDataSet("Linear Acceleration Y (w/o g) [m/s^2]"));
            dataIndices.push_back(plotWindow->addNewDataSet("Linear Acceleration Z (w/o g) [m/s^2]"));
        }
        else if (yData == "linearAccelNED")
        {
            dataIndices.push_back(plotWindow->addNewDataSet("Linear Acceleration N (w/o g) [m/s^2]"));
            dataIndices.push_back(plotWindow->addNewDataSet("Linear Acceleration E (w/o g) [m/s^2]"));
            dataIndices.push_back(plotWindow->addNewDataSet("Linear Acceleration D (w/o g) [m/s^2]"));
        }
        else if (yData == "yawPitchRollUncertainty")
        {
            dataIndices.push_back(plotWindow->addNewDataSet("Yaw Uncertainty (1 Sigma) [Degree]"));
            dataIndices.push_back(plotWindow->addNewDataSet("Pitch Uncertainty (1 Sigma) [Degree]"));
            dataIndices.push_back(plotWindow->addNewDataSet("Roll Uncertainty (1 Sigma) [Degree]"));
        }
    }
}

NAV::VectorNavGnuPlot::~VectorNavGnuPlot()
{
    LOG_TRACE("called for {}", name);
}

void NAV::VectorNavGnuPlot::plotVectorNavObs(std::shared_ptr<NAV::VectorNavObs>& obs)
{
    for (auto& [xData, yData, wIndex, dataIndices] : dataToPlot)
    {
        auto plotWindow = plotWindows.at(wIndex);

        double plotX = 0.0;

        if (xData == "gpsToW" && obs->insTime.has_value())
        {
            plotX = static_cast<double>(obs->insTime.value().GetGPSTime().tow);
        }
        else
        {
            continue;
        }

        if (yData == "timeSinceStartup" && obs->timeSinceStartup.has_value())
        {
            plotWindow->data.at(dataIndices.at(0)).xy.emplace_back(std::make_pair(plotX, obs->timeSinceStartup.value()));
        }
        else if (yData == "quaternion" && obs->quaternion.has_value())
        {
            plotWindow->data.at(dataIndices.at(0)).xy.emplace_back(std::make_pair(plotX, obs->quaternion.value().x()));
            plotWindow->data.at(dataIndices.at(1)).xy.emplace_back(std::make_pair(plotX, obs->quaternion.value().y()));
            plotWindow->data.at(dataIndices.at(2)).xy.emplace_back(std::make_pair(plotX, obs->quaternion.value().z()));
            plotWindow->data.at(dataIndices.at(3)).xy.emplace_back(std::make_pair(plotX, obs->quaternion.value().w()));
        }
        else if (yData == "magUncompXYZ" && obs->magUncompXYZ.has_value())
        {
            plotWindow->data.at(dataIndices.at(0)).xy.emplace_back(std::make_pair(plotX, obs->magUncompXYZ.value().x()));
            plotWindow->data.at(dataIndices.at(1)).xy.emplace_back(std::make_pair(plotX, obs->magUncompXYZ.value().y()));
            plotWindow->data.at(dataIndices.at(2)).xy.emplace_back(std::make_pair(plotX, obs->magUncompXYZ.value().z()));
        }
        else if (yData == "accelUncompXYZ" && obs->accelUncompXYZ.has_value())
        {
            plotWindow->data.at(dataIndices.at(0)).xy.emplace_back(std::make_pair(plotX, obs->accelUncompXYZ.value().x()));
            plotWindow->data.at(dataIndices.at(1)).xy.emplace_back(std::make_pair(plotX, obs->accelUncompXYZ.value().y()));
            plotWindow->data.at(dataIndices.at(2)).xy.emplace_back(std::make_pair(plotX, obs->accelUncompXYZ.value().z()));
        }
        else if (yData == "gyroUncompXYZ" && obs->gyroUncompXYZ.has_value())
        {
            plotWindow->data.at(dataIndices.at(0)).xy.emplace_back(std::make_pair(plotX, obs->gyroUncompXYZ.value().x()));
            plotWindow->data.at(dataIndices.at(1)).xy.emplace_back(std::make_pair(plotX, obs->gyroUncompXYZ.value().y()));
            plotWindow->data.at(dataIndices.at(2)).xy.emplace_back(std::make_pair(plotX, obs->gyroUncompXYZ.value().z()));
        }
        else if (yData == "magCompXYZ" && obs->magCompXYZ.has_value())
        {
            plotWindow->data.at(dataIndices.at(0)).xy.emplace_back(std::make_pair(plotX, obs->magCompXYZ.value().x()));
            plotWindow->data.at(dataIndices.at(1)).xy.emplace_back(std::make_pair(plotX, obs->magCompXYZ.value().y()));
            plotWindow->data.at(dataIndices.at(2)).xy.emplace_back(std::make_pair(plotX, obs->magCompXYZ.value().z()));
        }
        else if (yData == "accelCompXYZ" && obs->accelCompXYZ.has_value())
        {
            plotWindow->data.at(dataIndices.at(0)).xy.emplace_back(std::make_pair(plotX, obs->accelCompXYZ.value().x()));
            plotWindow->data.at(dataIndices.at(1)).xy.emplace_back(std::make_pair(plotX, obs->accelCompXYZ.value().y()));
            plotWindow->data.at(dataIndices.at(2)).xy.emplace_back(std::make_pair(plotX, obs->accelCompXYZ.value().z()));
        }
        else if (yData == "gyroCompXYZ" && obs->gyroCompXYZ.has_value())
        {
            plotWindow->data.at(dataIndices.at(0)).xy.emplace_back(std::make_pair(plotX, obs->gyroCompXYZ.value().x()));
            plotWindow->data.at(dataIndices.at(1)).xy.emplace_back(std::make_pair(plotX, obs->gyroCompXYZ.value().y()));
            plotWindow->data.at(dataIndices.at(2)).xy.emplace_back(std::make_pair(plotX, obs->gyroCompXYZ.value().z()));
        }
        else if (yData == "syncInCnt" && obs->syncInCnt.has_value())
        {
            plotWindow->data.at(dataIndices.at(0)).xy.emplace_back(std::make_pair(plotX, obs->syncInCnt.value()));
        }
        else if (yData == "dtime" && obs->dtime.has_value())
        {
            plotWindow->data.at(dataIndices.at(0)).xy.emplace_back(std::make_pair(plotX, obs->dtime.value()));
        }
        else if (yData == "dtheta" && obs->dtheta.has_value())
        {
            plotWindow->data.at(dataIndices.at(0)).xy.emplace_back(std::make_pair(plotX, obs->dtheta.value().x()));
            plotWindow->data.at(dataIndices.at(1)).xy.emplace_back(std::make_pair(plotX, obs->dtheta.value().y()));
            plotWindow->data.at(dataIndices.at(2)).xy.emplace_back(std::make_pair(plotX, obs->dtheta.value().z()));
        }
        else if (yData == "dvel" && obs->dvel.has_value())
        {
            plotWindow->data.at(dataIndices.at(0)).xy.emplace_back(std::make_pair(plotX, obs->dvel.value().x()));
            plotWindow->data.at(dataIndices.at(1)).xy.emplace_back(std::make_pair(plotX, obs->dvel.value().y()));
            plotWindow->data.at(dataIndices.at(2)).xy.emplace_back(std::make_pair(plotX, obs->dvel.value().z()));
        }
        else if (yData == "vpeStatus" && obs->vpeStatus.has_value())
        {
            plotWindow->data.at(dataIndices.at(0)).xy.emplace_back(std::make_pair(plotX, obs->vpeStatus.value().status));
        }
        else if (yData == "temperature" && obs->temperature.has_value())
        {
            plotWindow->data.at(dataIndices.at(0)).xy.emplace_back(std::make_pair(plotX, obs->temperature.value()));
        }
        else if (yData == "pressure" && obs->pressure.has_value())
        {
            plotWindow->data.at(dataIndices.at(0)).xy.emplace_back(std::make_pair(plotX, obs->pressure.value()));
        }
        else if (yData == "magCompNED" && obs->magCompNED.has_value())
        {
            plotWindow->data.at(dataIndices.at(0)).xy.emplace_back(std::make_pair(plotX, obs->magCompNED.value().x()));
            plotWindow->data.at(dataIndices.at(1)).xy.emplace_back(std::make_pair(plotX, obs->magCompNED.value().y()));
            plotWindow->data.at(dataIndices.at(2)).xy.emplace_back(std::make_pair(plotX, obs->magCompNED.value().z()));
        }
        else if (yData == "accelCompNED" && obs->accelCompNED.has_value())
        {
            plotWindow->data.at(dataIndices.at(0)).xy.emplace_back(std::make_pair(plotX, obs->accelCompNED.value().x()));
            plotWindow->data.at(dataIndices.at(1)).xy.emplace_back(std::make_pair(plotX, obs->accelCompNED.value().y()));
            plotWindow->data.at(dataIndices.at(2)).xy.emplace_back(std::make_pair(plotX, obs->accelCompNED.value().z()));
        }
        else if (yData == "gyroCompNED" && obs->gyroCompNED.has_value())
        {
            plotWindow->data.at(dataIndices.at(0)).xy.emplace_back(std::make_pair(plotX, obs->gyroCompNED.value().x()));
            plotWindow->data.at(dataIndices.at(1)).xy.emplace_back(std::make_pair(plotX, obs->gyroCompNED.value().y()));
            plotWindow->data.at(dataIndices.at(2)).xy.emplace_back(std::make_pair(plotX, obs->gyroCompNED.value().z()));
        }
        else if (yData == "linearAccelXYZ" && obs->linearAccelXYZ.has_value())
        {
            plotWindow->data.at(dataIndices.at(0)).xy.emplace_back(std::make_pair(plotX, obs->linearAccelXYZ.value().x()));
            plotWindow->data.at(dataIndices.at(1)).xy.emplace_back(std::make_pair(plotX, obs->linearAccelXYZ.value().y()));
            plotWindow->data.at(dataIndices.at(2)).xy.emplace_back(std::make_pair(plotX, obs->linearAccelXYZ.value().z()));
        }
        else if (yData == "linearAccelNED" && obs->linearAccelNED.has_value())
        {
            plotWindow->data.at(dataIndices.at(0)).xy.emplace_back(std::make_pair(plotX, obs->linearAccelNED.value().x()));
            plotWindow->data.at(dataIndices.at(1)).xy.emplace_back(std::make_pair(plotX, obs->linearAccelNED.value().y()));
            plotWindow->data.at(dataIndices.at(2)).xy.emplace_back(std::make_pair(plotX, obs->linearAccelNED.value().z()));
        }
        else if (yData == "yawPitchRollUncertainty" && obs->yawPitchRollUncertainty.has_value())
        {
            plotWindow->data.at(dataIndices.at(0)).xy.emplace_back(std::make_pair(plotX, obs->yawPitchRollUncertainty.value().x()));
            plotWindow->data.at(dataIndices.at(1)).xy.emplace_back(std::make_pair(plotX, obs->yawPitchRollUncertainty.value().y()));
            plotWindow->data.at(dataIndices.at(2)).xy.emplace_back(std::make_pair(plotX, obs->yawPitchRollUncertainty.value().z()));
        }

        // Delete old data
        if (NodeManager::appContext != NodeContext::POST_PROCESSING)
        {
            for (auto& dataIndex : dataIndices)
            {
                while (timeFrame != 0.0 && plotWindow->data.at(dataIndex).xy.back().first - plotWindow->data.at(dataIndex).xy.front().first > timeFrame)
                {
                    plotWindow->data.at(dataIndex).xy.pop_front();
                }
            }
        }
    }

    requestUpdate();
}