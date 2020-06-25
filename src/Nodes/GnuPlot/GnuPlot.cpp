#include "GnuPlot.hpp"

#include "util/Logger.hpp"
#include "Nodes/NodeManager.hpp"

NAV::GnuPlot::GnuPlot(const std::string& name, const std::map<std::string, std::string>& options)
    : Node(name)
{
    LOG_TRACE("called for {}", name);

    if (options.contains("Input Ports"))
    {
        nInputPorts = static_cast<uint8_t>(std::stoul(options.at("Input Ports")));
    }

    for (size_t i = 1; options.contains(std::to_string(i) + "-Port Type"); i++)
    {
        inputPortDataTypes[i - 1] = options.at(std::to_string(i) + "-Port Type");
    }

    if (options.contains("X Display Scope"))
    {
        xDisplayScope = std::stod(options.at("X Display Scope"));
    }

    if (options.contains("Update Frequency"))
    {
        updateFrequency = std::stod(options.at("Update Frequency"));
    }

    for (size_t i = 1; options.contains(std::to_string(i) + "-Data to plot"); i++)
    {
        if (options.at(std::to_string(i) + "-Data to plot").empty())
        {
            continue;
        }

        LOG_DEBUG("Plot Instructions for port {}: {}", i, options.at(std::to_string(i) + "-Data to plot"));

        std::string dataXY;
        std::stringstream lineStream(options.at(std::to_string(i) + "-Data to plot"));
        while (std::getline(lineStream, dataXY, ';'))
        {
            std::stringstream cellStream(dataXY);
            std::string dataX;
            std::string dataY;
            std::getline(cellStream, dataX, '|');
            std::getline(cellStream, dataY, '|');
            LOG_DEBUG("Inserting Plot {} -> {}", dataX, dataY);
            plotData[i - 1].emplace_back(dataX, dataY);
        }
    }

    if (options.contains("Start"))
    {
        gp << options.at("Start");
    }
}

NAV::GnuPlot::~GnuPlot()
{
    LOG_TRACE("called for {}", name);
}

void NAV::GnuPlot::requestUpdate()
{
    if (NodeManager::appContext == NAV::Node::NodeContext::REAL_TIME)
    {
        // Check if update Interval is reached
        bool plot = false;
        static double lastPlotTime = 0;
        for (auto iter = plotData.begin(); iter != plotData.end(); iter++)
        {
            for (const auto& data : iter->second)
            {
                if (!data.xy.empty() && data.xy.back().first - lastPlotTime >= 1.0 / updateFrequency)
                {
                    plot = true;
                    lastPlotTime = data.xy.back().first;
                    break;
                }
            }
        }

        // Update the GnuPlot Windows
        if (plot)
        {
            // Discard the nodiscard value
            static_cast<void>(update());
        }
    }
}

bool NAV::GnuPlot::update()
{
    bool somethingWasPlotted = false;
    std::string plotInstructions = "plot";
    std::string xLabel;

    for (auto iter = plotData.begin(); iter != plotData.end(); iter++)
    {
        for (size_t i = 0; i < iter->second.size(); i++)
        {
            if (!iter->second.at(i).xy.empty())
            {
                if (iter->second.at(i).xy.size() >= 2)
                {
                    std::string lineStyle = "lines";
                    if (i >= 8)
                    {
                        lineStyle = "points";
                    }

                    plotInstructions += gp.binFile1d(iter->second.at(i).xy, "record") + "with " + lineStyle + " title '" + iter->second.at(i).yData + "',";
                }

                xLabel = "set xlabel \"" + iter->second.at(i).xData + "\"\n";
                somethingWasPlotted = true;
            }
        }
    }
    if (somethingWasPlotted)
    {
        plotInstructions = plotInstructions.substr(0, plotInstructions.size() - 1);
        plotInstructions += '\n';
        LOG_DEBUG("Plot Instructions: {}", plotInstructions + xLabel);
        gp << plotInstructions;
        gp << xLabel;
        gp.flush();
    }
    return somethingWasPlotted;
}

void NAV::GnuPlot::handleVectorNavObs(std::shared_ptr<NAV::VectorNavObs>& obs, size_t portIndex)
{
    for (auto& data : plotData[portIndex])
    {
        double plotX = 0.0;

        if (data.xData == "GPS time of week" && obs->insTime.has_value())
        {
            plotX = static_cast<double>(obs->insTime.value().GetGPSTime().tow);
        }
        else
        {
            continue;
        }

        if (data.yData == "Time since startup" && obs->timeSinceStartup.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->timeSinceStartup.value()));
        }
        else if (data.yData == "Quaternion W" && obs->quaternion.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->quaternion.value().w()));
        }
        else if (data.yData == "Quaternion X" && obs->quaternion.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->quaternion.value().x()));
        }
        else if (data.yData == "Quaternion Y" && obs->quaternion.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->quaternion.value().y()));
        }
        else if (data.yData == "Quaternion Z" && obs->quaternion.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->quaternion.value().z()));
        }
        else if (data.yData == "Yaw" && obs->yawPitchRoll.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->yawPitchRoll.value().x()));
        }
        else if (data.yData == "Pitch" && obs->yawPitchRoll.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->yawPitchRoll.value().y()));
        }
        else if (data.yData == "Roll" && obs->yawPitchRoll.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->yawPitchRoll.value().z()));
        }
        else if (data.yData == "Mag uncomp X" && obs->magUncompXYZ.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->magUncompXYZ.value().x()));
        }
        else if (data.yData == "Mag uncomp Y" && obs->magUncompXYZ.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->magUncompXYZ.value().y()));
        }
        else if (data.yData == "Mag uncomp Z" && obs->magUncompXYZ.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->magUncompXYZ.value().z()));
        }
        else if (data.yData == "Accel uncomp X" && obs->accelUncompXYZ.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->accelUncompXYZ.value().x()));
        }
        else if (data.yData == "Accel uncomp Y" && obs->accelUncompXYZ.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->accelUncompXYZ.value().y()));
        }
        else if (data.yData == "Accel uncomp Z" && obs->accelUncompXYZ.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->accelUncompXYZ.value().z()));
        }
        else if (data.yData == "Gyro uncomp X" && obs->gyroUncompXYZ.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->gyroUncompXYZ.value().x()));
        }
        else if (data.yData == "Gyro uncomp Y" && obs->gyroUncompXYZ.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->gyroUncompXYZ.value().y()));
        }
        else if (data.yData == "Gyro uncomp Z" && obs->gyroUncompXYZ.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->gyroUncompXYZ.value().z()));
        }
        else if (data.yData == "Mag comp X" && obs->magCompXYZ.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->magCompXYZ.value().x()));
        }
        else if (data.yData == "Mag comp Y" && obs->magCompXYZ.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->magCompXYZ.value().y()));
        }
        else if (data.yData == "Mag comp Z" && obs->magCompXYZ.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->magCompXYZ.value().z()));
        }
        else if (data.yData == "Accel comp X" && obs->accelCompXYZ.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->accelCompXYZ.value().x()));
        }
        else if (data.yData == "Accel comp Y" && obs->accelCompXYZ.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->accelCompXYZ.value().y()));
        }
        else if (data.yData == "Accel comp Z" && obs->accelCompXYZ.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->accelCompXYZ.value().z()));
        }
        else if (data.yData == "Gyro comp X" && obs->gyroCompXYZ.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->gyroCompXYZ.value().x()));
        }
        else if (data.yData == "Gyro comp Y" && obs->gyroCompXYZ.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->gyroCompXYZ.value().y()));
        }
        else if (data.yData == "Gyro comp Z" && obs->gyroCompXYZ.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->gyroCompXYZ.value().z()));
        }
        else if (data.yData == "Sync In Count" && obs->syncInCnt.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->syncInCnt.value()));
        }
        else if (data.yData == "dtime" && obs->dtime.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->dtime.value()));
        }
        else if (data.yData == "dtheta X" && obs->dtheta.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->dtheta.value().x()));
        }
        else if (data.yData == "dtheta Y" && obs->dtheta.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->dtheta.value().y()));
        }
        else if (data.yData == "dtheta Z" && obs->dtheta.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->dtheta.value().z()));
        }
        else if (data.yData == "dvel X" && obs->dvel.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->dvel.value().x()));
        }
        else if (data.yData == "dvel Y" && obs->dvel.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->dvel.value().y()));
        }
        else if (data.yData == "dvel Z" && obs->dvel.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->dvel.value().z()));
        }
        else if (data.yData == "Vpe Status" && obs->vpeStatus.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->vpeStatus.value().status));
        }
        else if (data.yData == "Temperature" && obs->temperature.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->temperature.value()));
        }
        else if (data.yData == "Pressure" && obs->pressure.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->pressure.value()));
        }
        else if (data.yData == "Mag comp N" && obs->magCompNED.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->magCompNED.value().x()));
        }
        else if (data.yData == "Mag comp E" && obs->magCompNED.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->magCompNED.value().y()));
        }
        else if (data.yData == "Mag comp D" && obs->magCompNED.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->magCompNED.value().z()));
        }
        else if (data.yData == "Accel comp N" && obs->accelCompNED.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->accelCompNED.value().x()));
        }
        else if (data.yData == "Accel comp E" && obs->accelCompNED.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->accelCompNED.value().y()));
        }
        else if (data.yData == "Accel comp D" && obs->accelCompNED.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->accelCompNED.value().z()));
        }
        else if (data.yData == "Gyro comp N" && obs->gyroCompNED.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->gyroCompNED.value().x()));
        }
        else if (data.yData == "Gyro comp E" && obs->gyroCompNED.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->gyroCompNED.value().y()));
        }
        else if (data.yData == "Gyro comp D" && obs->gyroCompNED.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->gyroCompNED.value().z()));
        }
        else if (data.yData == "Linear Accel X" && obs->linearAccelXYZ.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->linearAccelXYZ.value().x()));
        }
        else if (data.yData == "Linear Accel Y" && obs->linearAccelXYZ.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->linearAccelXYZ.value().y()));
        }
        else if (data.yData == "Linear Accel Z" && obs->linearAccelXYZ.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->linearAccelXYZ.value().z()));
        }
        else if (data.yData == "Linear Accel N" && obs->linearAccelNED.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->linearAccelNED.value().x()));
        }
        else if (data.yData == "Linear Accel E" && obs->linearAccelNED.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->linearAccelNED.value().y()));
        }
        else if (data.yData == "Linear Accel D" && obs->linearAccelNED.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->linearAccelNED.value().z()));
        }
        else if (data.yData == "Yaw Uncertainty" && obs->yawPitchRollUncertainty.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->yawPitchRollUncertainty.value().x()));
        }
        else if (data.yData == "Pitch Uncertainty" && obs->yawPitchRollUncertainty.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->yawPitchRollUncertainty.value().y()));
        }
        else if (data.yData == "Roll Uncertainty" && obs->yawPitchRollUncertainty.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->yawPitchRollUncertainty.value().z()));
        }

        // Delete old data
        if (NodeManager::appContext != NodeContext::POST_PROCESSING)
        {
            while (xDisplayScope != 0.0 && data.xy.back().first - data.xy.front().first > xDisplayScope)
            {
                data.xy.pop_front();
            }
        }
    }

    requestUpdate();

    invokeCallbacks(obs);
}

void NAV::GnuPlot::handleRtklibPosObs(std::shared_ptr<NAV::RtklibPosObs>& obs, size_t portIndex)
{
    for (auto& data : plotData[portIndex])
    {
        double plotX = 0.0;

        if (data.xData == "GPS time of week" && obs->insTime.has_value())
        {
            plotX = static_cast<double>(obs->insTime.value().GetGPSTime().tow);
        }
        else if (data.xData == "Latitude" && obs->positionLLH.has_value())
        {
            plotX = obs->positionLLH.value()(1);
        }
        else if (data.xData == "Longitude" && obs->positionLLH.has_value())
        {
            plotX = obs->positionLLH.value()(0);
        }
        else if (data.xData == "Height" && obs->positionLLH.has_value())
        {
            plotX = obs->positionLLH.value()(2);
        }
        else if (data.xData == "X-ECEF" && obs->positionXYZ.has_value())
        {
            plotX = obs->positionXYZ.value().x();
        }
        else if (data.xData == "Y-ECEF" && obs->positionXYZ.has_value())
        {
            plotX = obs->positionXYZ.value().y();
        }
        else if (data.xData == "Z-ECEF" && obs->positionXYZ.has_value())
        {
            plotX = obs->positionXYZ.value().z();
        }
        else
        {
            continue;
        }

        if (data.yData == "GPS time of week" && obs->insTime.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, static_cast<double>(obs->insTime.value().GetGPSTime().tow)));
        }
        else if (data.yData == "Latitude" && obs->positionLLH.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->positionLLH.value()(1)));
        }
        else if (data.yData == "Longitude" && obs->positionLLH.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->positionLLH.value()(0)));
        }
        else if (data.yData == "Height" && obs->positionLLH.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->positionLLH.value()(2)));
        }
        else if (data.yData == "X-ECEF" && obs->positionXYZ.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->positionXYZ.value().x()));
        }
        else if (data.yData == "Y-ECEF" && obs->positionXYZ.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->positionXYZ.value().y()));
        }
        else if (data.yData == "Z-ECEF" && obs->positionXYZ.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->positionXYZ.value().z()));
        }
        else if (data.yData == "Q" && obs->Q.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->Q.value()));
        }
        else if (data.yData == "ns" && obs->ns.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->ns.value()));
        }
        else if (data.yData == "sdn" && obs->sdNEU.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->sdNEU.value()(0)));
        }
        else if (data.yData == "sde" && obs->sdNEU.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->sdNEU.value()(1)));
        }
        else if (data.yData == "sdu" && obs->sdNEU.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->sdNEU.value()(2)));
        }
        else if (data.yData == "sdx" && obs->sdXYZ.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->sdXYZ.value().x()));
        }
        else if (data.yData == "sdy" && obs->sdXYZ.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->sdXYZ.value().y()));
        }
        else if (data.yData == "sdz" && obs->sdXYZ.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->sdXYZ.value().z()));
        }
        else if (data.yData == "sdne" && obs->sdne.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->sdne.value()));
        }
        else if (data.yData == "sdeu" && obs->sdeu.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->sdeu.value()));
        }
        else if (data.yData == "sdun" && obs->sdun.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->sdun.value()));
        }
        else if (data.yData == "sdxy" && obs->sdxy.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->sdxy.value()));
        }
        else if (data.yData == "sdyz" && obs->sdyz.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->sdyz.value()));
        }
        else if (data.yData == "sdzx" && obs->sdzx.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->sdzx.value()));
        }
        else if (data.yData == "Age" && obs->age.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->age.value()));
        }
        else if (data.yData == "Ratio" && obs->ratio.has_value())
        {
            data.xy.emplace_back(std::make_pair(plotX, obs->ratio.value()));
        }

        // Delete old data
        if (NodeManager::appContext != NodeContext::POST_PROCESSING)
        {
            while (xDisplayScope != 0.0 && data.xy.back().first - data.xy.front().first > xDisplayScope)
            {
                data.xy.pop_front();
            }
        }
    }

    requestUpdate();

    invokeCallbacks(obs);
}