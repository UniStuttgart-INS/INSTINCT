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

    double plotTime = static_cast<double>(obs->timeSinceStartup.value() - obj->timeStart) / std::pow(10, 9);

    for (auto& [name, wIndex] : obj->dataToPlot)
    {
        auto plotWindow = GnuPlot::plotWindows.at(wIndex);

        if (name == "quaternion" && obs->quaternion.has_value())
        {
            static size_t dataIndex0 = plotWindow->addNewDataSet("Attitude Quaternion X");
            static size_t dataIndex1 = plotWindow->addNewDataSet("Attitude Quaternion Y");
            static size_t dataIndex2 = plotWindow->addNewDataSet("Attitude Quaternion Z");
            static size_t dataIndex3 = plotWindow->addNewDataSet("Attitude Quaternion W");

            plotWindow->data.at(dataIndex0).xy.push_back(std::make_pair(plotTime, obs->quaternion.value().x()));
            plotWindow->data.at(dataIndex1).xy.push_back(std::make_pair(plotTime, obs->quaternion.value().y()));
            plotWindow->data.at(dataIndex2).xy.push_back(std::make_pair(plotTime, obs->quaternion.value().z()));
            plotWindow->data.at(dataIndex3).xy.push_back(std::make_pair(plotTime, obs->quaternion.value().w()));
        }
        else if (name == "magUncompXYZ" && obs->magUncompXYZ.has_value())
        {
            static size_t dataIndex0 = plotWindow->addNewDataSet("Uncompensated Magnetic Field X [Gauss]");
            static size_t dataIndex1 = plotWindow->addNewDataSet("Uncompensated Magnetic Field Y [Gauss]");
            static size_t dataIndex2 = plotWindow->addNewDataSet("Uncompensated Magnetic Field Z [Gauss]");

            plotWindow->data.at(dataIndex0).xy.push_back(std::make_pair(plotTime, obs->magUncompXYZ.value().x()));
            plotWindow->data.at(dataIndex1).xy.push_back(std::make_pair(plotTime, obs->magUncompXYZ.value().y()));
            plotWindow->data.at(dataIndex2).xy.push_back(std::make_pair(plotTime, obs->magUncompXYZ.value().z()));
        }
        else if (name == "accelUncompXYZ" && obs->accelUncompXYZ.has_value())
        {
            static size_t dataIndex0 = plotWindow->addNewDataSet("Uncompensated Acceleration X [m/s^2]");
            static size_t dataIndex1 = plotWindow->addNewDataSet("Uncompensated Acceleration Y [m/s^2]");
            static size_t dataIndex2 = plotWindow->addNewDataSet("Uncompensated Acceleration Z [m/s^2]");

            plotWindow->data.at(dataIndex0).xy.push_back(std::make_pair(plotTime, obs->accelUncompXYZ.value().x()));
            plotWindow->data.at(dataIndex1).xy.push_back(std::make_pair(plotTime, obs->accelUncompXYZ.value().y()));
            plotWindow->data.at(dataIndex2).xy.push_back(std::make_pair(plotTime, obs->accelUncompXYZ.value().z()));
        }
        else if (name == "gyroUncompXYZ" && obs->gyroUncompXYZ.has_value())
        {
            static size_t dataIndex0 = plotWindow->addNewDataSet("Uncompensated Angular Rates X [rad/s]");
            static size_t dataIndex1 = plotWindow->addNewDataSet("Uncompensated Angular Rates Y [rad/s]");
            static size_t dataIndex2 = plotWindow->addNewDataSet("Uncompensated Angular Rates Z [rad/s]");

            plotWindow->data.at(dataIndex0).xy.push_back(std::make_pair(plotTime, obs->gyroUncompXYZ.value().x()));
            plotWindow->data.at(dataIndex1).xy.push_back(std::make_pair(plotTime, obs->gyroUncompXYZ.value().y()));
            plotWindow->data.at(dataIndex2).xy.push_back(std::make_pair(plotTime, obs->gyroUncompXYZ.value().z()));
        }
        else if (name == "magCompXYZ" && obs->magCompXYZ.has_value())
        {
            static size_t dataIndex0 = plotWindow->addNewDataSet("Compensated Magnetic Field X [Gauss]");
            static size_t dataIndex1 = plotWindow->addNewDataSet("Compensated Magnetic Field Y [Gauss]");
            static size_t dataIndex2 = plotWindow->addNewDataSet("Compensated Magnetic Field Z [Gauss]");

            plotWindow->data.at(dataIndex0).xy.push_back(std::make_pair(plotTime, obs->magCompXYZ.value().x()));
            plotWindow->data.at(dataIndex1).xy.push_back(std::make_pair(plotTime, obs->magCompXYZ.value().y()));
            plotWindow->data.at(dataIndex2).xy.push_back(std::make_pair(plotTime, obs->magCompXYZ.value().z()));
        }
        else if (name == "accelCompXYZ" && obs->accelCompXYZ.has_value())
        {
            static size_t dataIndex0 = plotWindow->addNewDataSet("Compensated Acceleration X [m/s^2]");
            static size_t dataIndex1 = plotWindow->addNewDataSet("Compensated Acceleration Y [m/s^2]");
            static size_t dataIndex2 = plotWindow->addNewDataSet("Compensated Acceleration Z [m/s^2]");

            plotWindow->data.at(dataIndex0).xy.push_back(std::make_pair(plotTime, obs->accelCompXYZ.value().x()));
            plotWindow->data.at(dataIndex1).xy.push_back(std::make_pair(plotTime, obs->accelCompXYZ.value().y()));
            plotWindow->data.at(dataIndex2).xy.push_back(std::make_pair(plotTime, obs->accelCompXYZ.value().z()));
        }
        else if (name == "gyroCompXYZ" && obs->gyroCompXYZ.has_value())
        {
            static size_t dataIndex0 = plotWindow->addNewDataSet("Compensated Angular Rates X [rad/s]");
            static size_t dataIndex1 = plotWindow->addNewDataSet("Compensated Angular Rates Y [rad/s]");
            static size_t dataIndex2 = plotWindow->addNewDataSet("Compensated Angular Rates Z [rad/s]");

            plotWindow->data.at(dataIndex0).xy.push_back(std::make_pair(plotTime, obs->gyroCompXYZ.value().x()));
            plotWindow->data.at(dataIndex1).xy.push_back(std::make_pair(plotTime, obs->gyroCompXYZ.value().y()));
            plotWindow->data.at(dataIndex2).xy.push_back(std::make_pair(plotTime, obs->gyroCompXYZ.value().z()));
        }
        else if (name == "syncInCnt" && obs->syncInCnt.has_value())
        {
            static size_t dataIndex0 = plotWindow->addNewDataSet("Sync In Count [-]");

            plotWindow->data.at(dataIndex0).xy.push_back(std::make_pair(plotTime, obs->syncInCnt.value()));
        }
        else if (name == "dtime" && obs->dtime.has_value())
        {
            static size_t dataIndex0 = plotWindow->addNewDataSet("Delta Time Integration Interval [s]");

            plotWindow->data.at(dataIndex0).xy.push_back(std::make_pair(plotTime, obs->dtime.value()));
        }
        else if (name == "dtheta" && obs->dtheta.has_value())
        {
            static size_t dataIndex0 = plotWindow->addNewDataSet("Delta rotation angles X [Degree]");
            static size_t dataIndex1 = plotWindow->addNewDataSet("Delta rotation angles Y [Degree]");
            static size_t dataIndex2 = plotWindow->addNewDataSet("Delta rotation angles Z [Degree]");

            plotWindow->data.at(dataIndex0).xy.push_back(std::make_pair(plotTime, obs->dtheta.value().x()));
            plotWindow->data.at(dataIndex1).xy.push_back(std::make_pair(plotTime, obs->dtheta.value().y()));
            plotWindow->data.at(dataIndex2).xy.push_back(std::make_pair(plotTime, obs->dtheta.value().z()));
        }
        else if (name == "dvel" && obs->dvel.has_value())
        {
            static size_t dataIndex0 = plotWindow->addNewDataSet("Delta velocity X [m/s]");
            static size_t dataIndex1 = plotWindow->addNewDataSet("Delta velocity Y [m/s]");
            static size_t dataIndex2 = plotWindow->addNewDataSet("Delta velocity Z [m/s]");

            plotWindow->data.at(dataIndex0).xy.push_back(std::make_pair(plotTime, obs->dvel.value().x()));
            plotWindow->data.at(dataIndex1).xy.push_back(std::make_pair(plotTime, obs->dvel.value().y()));
            plotWindow->data.at(dataIndex2).xy.push_back(std::make_pair(plotTime, obs->dvel.value().z()));
        }
        else if (name == "vpeStatus" && obs->vpeStatus.has_value())
        {
            static size_t dataIndex0 = plotWindow->addNewDataSet("VPE Status Bitfield");

            plotWindow->data.at(dataIndex0).xy.push_back(std::make_pair(plotTime, obs->vpeStatus.value()));
        }
        else if (name == "temperature" && obs->temperature.has_value())
        {
            static size_t dataIndex0 = plotWindow->addNewDataSet("Temperature [Celsius]");

            plotWindow->data.at(dataIndex0).xy.push_back(std::make_pair(plotTime, obs->temperature.value()));
        }
        else if (name == "pressure" && obs->pressure.has_value())
        {
            static size_t dataIndex0 = plotWindow->addNewDataSet("Absolute IMU pressure [kPa]");

            plotWindow->data.at(dataIndex0).xy.push_back(std::make_pair(plotTime, obs->pressure.value()));
        }
        else if (name == "magCompNED" && obs->magCompNED.has_value())
        {
            static size_t dataIndex0 = plotWindow->addNewDataSet("Compensated Magnetic Field N [Gauss]");
            static size_t dataIndex1 = plotWindow->addNewDataSet("Compensated Magnetic Field E [Gauss]");
            static size_t dataIndex2 = plotWindow->addNewDataSet("Compensated Magnetic Field D [Gauss]");

            plotWindow->data.at(dataIndex0).xy.push_back(std::make_pair(plotTime, obs->magCompNED.value().x()));
            plotWindow->data.at(dataIndex1).xy.push_back(std::make_pair(plotTime, obs->magCompNED.value().y()));
            plotWindow->data.at(dataIndex2).xy.push_back(std::make_pair(plotTime, obs->magCompNED.value().z()));
        }
        else if (name == "accelCompNED" && obs->accelCompNED.has_value())
        {
            static size_t dataIndex0 = plotWindow->addNewDataSet("Compensated Acceleration N [m/s^2]");
            static size_t dataIndex1 = plotWindow->addNewDataSet("Compensated Acceleration E [m/s^2]");
            static size_t dataIndex2 = plotWindow->addNewDataSet("Compensated Acceleration D [m/s^2]");

            plotWindow->data.at(dataIndex0).xy.push_back(std::make_pair(plotTime, obs->accelCompNED.value().x()));
            plotWindow->data.at(dataIndex1).xy.push_back(std::make_pair(plotTime, obs->accelCompNED.value().y()));
            plotWindow->data.at(dataIndex2).xy.push_back(std::make_pair(plotTime, obs->accelCompNED.value().z()));
        }
        else if (name == "gyroCompNED" && obs->gyroCompNED.has_value())
        {
            static size_t dataIndex0 = plotWindow->addNewDataSet("Compensated Angular Rates N [rad/s]");
            static size_t dataIndex1 = plotWindow->addNewDataSet("Compensated Angular Rates E [rad/s]");
            static size_t dataIndex2 = plotWindow->addNewDataSet("Compensated Angular Rates D [rad/s]");

            plotWindow->data.at(dataIndex0).xy.push_back(std::make_pair(plotTime, obs->gyroCompNED.value().x()));
            plotWindow->data.at(dataIndex1).xy.push_back(std::make_pair(plotTime, obs->gyroCompNED.value().y()));
            plotWindow->data.at(dataIndex2).xy.push_back(std::make_pair(plotTime, obs->gyroCompNED.value().z()));
        }
        else if (name == "linearAccelXYZ" && obs->linearAccelXYZ.has_value())
        {
            static size_t dataIndex0 = plotWindow->addNewDataSet("Linear Acceleration X (w/o g) [m/s^2]");
            static size_t dataIndex1 = plotWindow->addNewDataSet("Linear Acceleration Y (w/o g) [m/s^2]");
            static size_t dataIndex2 = plotWindow->addNewDataSet("Linear Acceleration Z (w/o g) [m/s^2]");

            plotWindow->data.at(dataIndex0).xy.push_back(std::make_pair(plotTime, obs->linearAccelXYZ.value().x()));
            plotWindow->data.at(dataIndex1).xy.push_back(std::make_pair(plotTime, obs->linearAccelXYZ.value().y()));
            plotWindow->data.at(dataIndex2).xy.push_back(std::make_pair(plotTime, obs->linearAccelXYZ.value().z()));
        }
        else if (name == "linearAccelNED" && obs->linearAccelNED.has_value())
        {
            static size_t dataIndex0 = plotWindow->addNewDataSet("Linear Acceleration N (w/o g) [m/s^2]");
            static size_t dataIndex1 = plotWindow->addNewDataSet("Linear Acceleration E (w/o g) [m/s^2]");
            static size_t dataIndex2 = plotWindow->addNewDataSet("Linear Acceleration D (w/o g) [m/s^2]");

            plotWindow->data.at(dataIndex0).xy.push_back(std::make_pair(plotTime, obs->linearAccelNED.value().x()));
            plotWindow->data.at(dataIndex1).xy.push_back(std::make_pair(plotTime, obs->linearAccelNED.value().y()));
            plotWindow->data.at(dataIndex2).xy.push_back(std::make_pair(plotTime, obs->linearAccelNED.value().z()));
        }
        else if (name == "yawPitchRollUncertainty" && obs->yawPitchRollUncertainty.has_value())
        {
            static size_t dataIndex0 = plotWindow->addNewDataSet("Yaw Uncertainty (1 Sigma) [Degree]");
            static size_t dataIndex1 = plotWindow->addNewDataSet("Pitch Uncertainty (1 Sigma) [Degree]");
            static size_t dataIndex2 = plotWindow->addNewDataSet("Roll Uncertainty (1 Sigma) [Degree]");

            plotWindow->data.at(dataIndex0).xy.push_back(std::make_pair(plotTime, obs->yawPitchRollUncertainty.value().x()));
            plotWindow->data.at(dataIndex1).xy.push_back(std::make_pair(plotTime, obs->yawPitchRollUncertainty.value().y()));
            plotWindow->data.at(dataIndex2).xy.push_back(std::make_pair(plotTime, obs->yawPitchRollUncertainty.value().z()));
        }
    }

    return updateWindows(obj);
}