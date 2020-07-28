#include "GnuPlot.hpp"

#include <chrono>
#include <regex>

#include "util/Logger.hpp"
#include "Nodes/NodeManager.hpp"

NAV::GnuPlot::GnuPlot(const std::string& name, const std::map<std::string, std::string>& options)
    : Node(name)
{
    LOG_TRACE("called for {}", name);

    gp.flush();

    if (options.count("Input Ports"))
    {
        nInputPorts = static_cast<uint8_t>(std::stoul(options.at("Input Ports")));
    }

    for (size_t i = 1; options.count(std::to_string(i) + "-Port Type"); i++)
    {
        inputPortDataTypes[i - 1] = options.at(std::to_string(i) + "-Port Type");
    }

    if (options.count("Clear after x data"))
    {
        xDisplayScope = static_cast<uint32_t>(std::stoul(options.at("Clear after x data")));
    }

    if (options.count("Update Frequency"))
    {
        updateFrequency = static_cast<uint32_t>(std::stoul(options.at("Update Frequency")));
    }

    for (size_t i = 1; options.count(std::to_string(i) + "-Data to plot"); i++)
    {
        if (options.at(std::to_string(i) + "-Data to plot").empty())
        {
            continue;
        }

        LOG_DEBUG("Plot Data for port {}: {}", i, options.at(std::to_string(i) + "-Data to plot"));

        std::string dataIdentifier;
        std::stringstream lineStream(options.at(std::to_string(i) + "-Data to plot"));
        while (std::getline(lineStream, dataIdentifier, ';'))
        {
            plotData[i - 1].emplace_back(dataIdentifier);
        }
    }

    if (options.count("Start"))
    {
        LOG_DEBUG("Plot Start Instructions:\n{}", options.at("Start"));
        gp << options.at("Start") << '\n';
    }

    for (size_t i = 1; options.count(std::to_string(i) + "-Update"); i++)
    {
        portUpdateStrings.push_back(options.at(std::to_string(i) + "-Update"));

        if (options.at(std::to_string(i) + "-Update").empty()
            || plotData[i - 1].empty())
        {
            continue;
        }

        LOG_DEBUG("Plot Update Instructions for port {}:\n{}", i, options.at(std::to_string(i) + "-Update"));
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
        static auto start = std::chrono::high_resolution_clock::now();

        auto finish = std::chrono::high_resolution_clock::now();

        std::chrono::duration<double> elapsed = finish - start;
        // Update the GnuPlot Windows
        if (1.0 / elapsed.count() <= updateFrequency)
        {
            // Discard the nodiscard value
            static_cast<void>(update());

            start = finish;
        }
    }
}

bool NAV::GnuPlot::update()
{
    bool somethingWasPlotted = false;
    std::string plotInstructions;
    // std::string xLabel;

    for (auto iter = plotData.begin(); iter != plotData.end(); iter++)
    {
        if (iter->second.empty())
        {
            continue;
        }

        std::string portUpdateString = portUpdateStrings.at(iter->first);
        while (true)
        {
            size_t startPos = portUpdateString.find("[~");
            if (startPos == std::string::npos)
            {
                break;
            }
            size_t endPos = portUpdateString.find("~]");

            std::string dataSelection = portUpdateString.substr(startPos + 2, endPos - startPos - 2);
            std::stringstream lineStream(dataSelection);
            std::string cell;

            std::vector<std::vector<double>> joinedData;
            bool errorOccured = false;
            int64_t dataLength = -1;
            // Split line at comma
            while (std::getline(lineStream, cell, ','))
            {
                for (size_t i = 0; i < iter->second.size(); i++)
                {
                    if (std::stoul(cell) == i + 1)
                    {
                        if (dataLength < 0)
                        {
                            dataLength = static_cast<int64_t>(iter->second.at(i).data.size());
                        }
                        if (iter->second.at(i).data.empty())
                        {
                            LOG_ERROR("{} wants to plot {}, but has no data.", name, iter->second.at(i).dataIdentifier);
                            errorOccured = true;
                            break;
                        }
                        if (static_cast<int64_t>(iter->second.at(i).data.size()) != dataLength)
                        {
                            LOG_ERROR("{} wants to plot {} with length {}, but previous data had length {}", name, iter->second.at(i).dataIdentifier, iter->second.at(i).data.size(), dataLength);
                            errorOccured = true;
                            break;
                        }

                        joinedData.emplace_back(iter->second.at(i).data.begin(), iter->second.at(i).data.end());
                        break;
                    }
                }
                if (errorOccured)
                {
                    break;
                }
            }
            if (errorOccured)
            {
                portUpdateString.clear();
                break;
            }

            somethingWasPlotted = true;

            std::string dataGnuplot;
            if (!joinedData.empty())
            {
                dataGnuplot = gp.binFile1d_colmajor(joinedData, "record");
            }

            while (true)
            {
                std::string replaceString = "[~" + dataSelection + "~]";
                size_t replaceStart = portUpdateString.find(replaceString);
                if (replaceStart == std::string::npos)
                {
                    break;
                }
                portUpdateString.replace(replaceStart, replaceString.length(), dataGnuplot);
            }
        }
        plotInstructions += portUpdateString + '\n';
    }

    // gnuplot only supports one 'plot' command. So merge them if more than one is found
    size_t firstPlotCmdPos = plotInstructions.find("plot");
    if (firstPlotCmdPos != std::string::npos)
    {
        std::regex plot_regex(R"((plot )(.*\\\n)*.*)");
        auto words_begin = std::sregex_iterator(plotInstructions.begin(), plotInstructions.end(), plot_regex);
        auto words_end = std::sregex_iterator();
        std::vector<std::string> plotCommands;

        for (std::sregex_iterator i = words_begin; i != words_end; ++i)
        {
            std::smatch match = *i;
            plotCommands.push_back(match.str());
        }
        if (plotCommands.size() > 1)
        {
            plotCommands.at(0) += ", \\\n";
            for (size_t i = 1; i < plotCommands.size() - 1; i++)
            {
                plotCommands.at(i) = plotCommands.at(i).substr(5) + ", \\\n";
            }
            plotCommands.at(plotCommands.size() - 1) = plotCommands.at(plotCommands.size() - 1).substr(5);

            plotInstructions = std::regex_replace(plotInstructions, plot_regex, "");
            for (int i = static_cast<int>(plotCommands.size()) - 1; i >= 0; i--)
            {
                plotInstructions.insert(firstPlotCmdPos, plotCommands.at(static_cast<size_t>(i)));
            }
        }
    }

    if (somethingWasPlotted)
    {
        LOG_DEBUG("{} Plot Instructions: \n{}", name, plotInstructions);
        gp << plotInstructions;
        gp.flush();
    }
    return somethingWasPlotted;
}

void NAV::GnuPlot::handleVectorNavObs(std::shared_ptr<NAV::VectorNavObs>& obs, size_t portIndex)
{
    for (auto& gnuplotData : plotData[portIndex])
    {
        if (gnuplotData.dataIdentifier == "GPS time of week" && obs->insTime.has_value())
        {
            gnuplotData.data.emplace_back(static_cast<double>(obs->insTime.value().GetGPSTime().tow));
        }
        else if (gnuplotData.dataIdentifier == "Time since startup" && obs->timeSinceStartup.has_value())
        {
            gnuplotData.data.emplace_back(obs->timeSinceStartup.value());
        }
        else if (gnuplotData.dataIdentifier == "Quaternion W" && obs->quaternion.has_value())
        {
            gnuplotData.data.emplace_back(obs->quaternion.value().w());
        }
        else if (gnuplotData.dataIdentifier == "Quaternion X" && obs->quaternion.has_value())
        {
            gnuplotData.data.emplace_back(obs->quaternion.value().x());
        }
        else if (gnuplotData.dataIdentifier == "Quaternion Y" && obs->quaternion.has_value())
        {
            gnuplotData.data.emplace_back(obs->quaternion.value().y());
        }
        else if (gnuplotData.dataIdentifier == "Quaternion Z" && obs->quaternion.has_value())
        {
            gnuplotData.data.emplace_back(obs->quaternion.value().z());
        }
        else if (gnuplotData.dataIdentifier == "Yaw" && obs->yawPitchRoll.has_value())
        {
            gnuplotData.data.emplace_back(obs->yawPitchRoll.value().x());
        }
        else if (gnuplotData.dataIdentifier == "Pitch" && obs->yawPitchRoll.has_value())
        {
            gnuplotData.data.emplace_back(obs->yawPitchRoll.value().y());
        }
        else if (gnuplotData.dataIdentifier == "Roll" && obs->yawPitchRoll.has_value())
        {
            gnuplotData.data.emplace_back(obs->yawPitchRoll.value().z());
        }
        else if (gnuplotData.dataIdentifier == "Mag uncomp X" && obs->magUncompXYZ.has_value())
        {
            gnuplotData.data.emplace_back(obs->magUncompXYZ.value().x());
        }
        else if (gnuplotData.dataIdentifier == "Mag uncomp Y" && obs->magUncompXYZ.has_value())
        {
            gnuplotData.data.emplace_back(obs->magUncompXYZ.value().y());
        }
        else if (gnuplotData.dataIdentifier == "Mag uncomp Z" && obs->magUncompXYZ.has_value())
        {
            gnuplotData.data.emplace_back(obs->magUncompXYZ.value().z());
        }
        else if (gnuplotData.dataIdentifier == "Accel uncomp X" && obs->accelUncompXYZ.has_value())
        {
            gnuplotData.data.emplace_back(obs->accelUncompXYZ.value().x());
        }
        else if (gnuplotData.dataIdentifier == "Accel uncomp Y" && obs->accelUncompXYZ.has_value())
        {
            gnuplotData.data.emplace_back(obs->accelUncompXYZ.value().y());
        }
        else if (gnuplotData.dataIdentifier == "Accel uncomp Z" && obs->accelUncompXYZ.has_value())
        {
            gnuplotData.data.emplace_back(obs->accelUncompXYZ.value().z());
        }
        else if (gnuplotData.dataIdentifier == "Gyro uncomp X" && obs->gyroUncompXYZ.has_value())
        {
            gnuplotData.data.emplace_back(obs->gyroUncompXYZ.value().x());
        }
        else if (gnuplotData.dataIdentifier == "Gyro uncomp Y" && obs->gyroUncompXYZ.has_value())
        {
            gnuplotData.data.emplace_back(obs->gyroUncompXYZ.value().y());
        }
        else if (gnuplotData.dataIdentifier == "Gyro uncomp Z" && obs->gyroUncompXYZ.has_value())
        {
            gnuplotData.data.emplace_back(obs->gyroUncompXYZ.value().z());
        }
        else if (gnuplotData.dataIdentifier == "Mag comp X" && obs->magCompXYZ.has_value())
        {
            gnuplotData.data.emplace_back(obs->magCompXYZ.value().x());
        }
        else if (gnuplotData.dataIdentifier == "Mag comp Y" && obs->magCompXYZ.has_value())
        {
            gnuplotData.data.emplace_back(obs->magCompXYZ.value().y());
        }
        else if (gnuplotData.dataIdentifier == "Mag comp Z" && obs->magCompXYZ.has_value())
        {
            gnuplotData.data.emplace_back(obs->magCompXYZ.value().z());
        }
        else if (gnuplotData.dataIdentifier == "Accel comp X" && obs->accelCompXYZ.has_value())
        {
            gnuplotData.data.emplace_back(obs->accelCompXYZ.value().x());
        }
        else if (gnuplotData.dataIdentifier == "Accel comp Y" && obs->accelCompXYZ.has_value())
        {
            gnuplotData.data.emplace_back(obs->accelCompXYZ.value().y());
        }
        else if (gnuplotData.dataIdentifier == "Accel comp Z" && obs->accelCompXYZ.has_value())
        {
            gnuplotData.data.emplace_back(obs->accelCompXYZ.value().z());
        }
        else if (gnuplotData.dataIdentifier == "Gyro comp X" && obs->gyroCompXYZ.has_value())
        {
            gnuplotData.data.emplace_back(obs->gyroCompXYZ.value().x());
        }
        else if (gnuplotData.dataIdentifier == "Gyro comp Y" && obs->gyroCompXYZ.has_value())
        {
            gnuplotData.data.emplace_back(obs->gyroCompXYZ.value().y());
        }
        else if (gnuplotData.dataIdentifier == "Gyro comp Z" && obs->gyroCompXYZ.has_value())
        {
            gnuplotData.data.emplace_back(obs->gyroCompXYZ.value().z());
        }
        else if (gnuplotData.dataIdentifier == "Sync In Count" && obs->syncInCnt.has_value())
        {
            gnuplotData.data.emplace_back(obs->syncInCnt.value());
        }
        else if (gnuplotData.dataIdentifier == "dtime" && obs->dtime.has_value())
        {
            gnuplotData.data.emplace_back(obs->dtime.value());
        }
        else if (gnuplotData.dataIdentifier == "dtheta X" && obs->dtheta.has_value())
        {
            gnuplotData.data.emplace_back(obs->dtheta.value().x());
        }
        else if (gnuplotData.dataIdentifier == "dtheta Y" && obs->dtheta.has_value())
        {
            gnuplotData.data.emplace_back(obs->dtheta.value().y());
        }
        else if (gnuplotData.dataIdentifier == "dtheta Z" && obs->dtheta.has_value())
        {
            gnuplotData.data.emplace_back(obs->dtheta.value().z());
        }
        else if (gnuplotData.dataIdentifier == "dvel X" && obs->dvel.has_value())
        {
            gnuplotData.data.emplace_back(obs->dvel.value().x());
        }
        else if (gnuplotData.dataIdentifier == "dvel Y" && obs->dvel.has_value())
        {
            gnuplotData.data.emplace_back(obs->dvel.value().y());
        }
        else if (gnuplotData.dataIdentifier == "dvel Z" && obs->dvel.has_value())
        {
            gnuplotData.data.emplace_back(obs->dvel.value().z());
        }
        else if (gnuplotData.dataIdentifier == "Vpe Status" && obs->vpeStatus.has_value())
        {
            gnuplotData.data.emplace_back(obs->vpeStatus.value().status);
        }
        else if (gnuplotData.dataIdentifier == "Temperature" && obs->temperature.has_value())
        {
            gnuplotData.data.emplace_back(obs->temperature.value());
        }
        else if (gnuplotData.dataIdentifier == "Pressure" && obs->pressure.has_value())
        {
            gnuplotData.data.emplace_back(obs->pressure.value());
        }
        else if (gnuplotData.dataIdentifier == "Mag comp N" && obs->magCompNED.has_value())
        {
            gnuplotData.data.emplace_back(obs->magCompNED.value().x());
        }
        else if (gnuplotData.dataIdentifier == "Mag comp E" && obs->magCompNED.has_value())
        {
            gnuplotData.data.emplace_back(obs->magCompNED.value().y());
        }
        else if (gnuplotData.dataIdentifier == "Mag comp D" && obs->magCompNED.has_value())
        {
            gnuplotData.data.emplace_back(obs->magCompNED.value().z());
        }
        else if (gnuplotData.dataIdentifier == "Accel comp N" && obs->accelCompNED.has_value())
        {
            gnuplotData.data.emplace_back(obs->accelCompNED.value().x());
        }
        else if (gnuplotData.dataIdentifier == "Accel comp E" && obs->accelCompNED.has_value())
        {
            gnuplotData.data.emplace_back(obs->accelCompNED.value().y());
        }
        else if (gnuplotData.dataIdentifier == "Accel comp D" && obs->accelCompNED.has_value())
        {
            gnuplotData.data.emplace_back(obs->accelCompNED.value().z());
        }
        else if (gnuplotData.dataIdentifier == "Gyro comp N" && obs->gyroCompNED.has_value())
        {
            gnuplotData.data.emplace_back(obs->gyroCompNED.value().x());
        }
        else if (gnuplotData.dataIdentifier == "Gyro comp E" && obs->gyroCompNED.has_value())
        {
            gnuplotData.data.emplace_back(obs->gyroCompNED.value().y());
        }
        else if (gnuplotData.dataIdentifier == "Gyro comp D" && obs->gyroCompNED.has_value())
        {
            gnuplotData.data.emplace_back(obs->gyroCompNED.value().z());
        }
        else if (gnuplotData.dataIdentifier == "Linear Accel X" && obs->linearAccelXYZ.has_value())
        {
            gnuplotData.data.emplace_back(obs->linearAccelXYZ.value().x());
        }
        else if (gnuplotData.dataIdentifier == "Linear Accel Y" && obs->linearAccelXYZ.has_value())
        {
            gnuplotData.data.emplace_back(obs->linearAccelXYZ.value().y());
        }
        else if (gnuplotData.dataIdentifier == "Linear Accel Z" && obs->linearAccelXYZ.has_value())
        {
            gnuplotData.data.emplace_back(obs->linearAccelXYZ.value().z());
        }
        else if (gnuplotData.dataIdentifier == "Linear Accel N" && obs->linearAccelNED.has_value())
        {
            gnuplotData.data.emplace_back(obs->linearAccelNED.value().x());
        }
        else if (gnuplotData.dataIdentifier == "Linear Accel E" && obs->linearAccelNED.has_value())
        {
            gnuplotData.data.emplace_back(obs->linearAccelNED.value().y());
        }
        else if (gnuplotData.dataIdentifier == "Linear Accel D" && obs->linearAccelNED.has_value())
        {
            gnuplotData.data.emplace_back(obs->linearAccelNED.value().z());
        }
        else if (gnuplotData.dataIdentifier == "Yaw Uncertainty" && obs->yawPitchRollUncertainty.has_value())
        {
            gnuplotData.data.emplace_back(obs->yawPitchRollUncertainty.value().x());
        }
        else if (gnuplotData.dataIdentifier == "Pitch Uncertainty" && obs->yawPitchRollUncertainty.has_value())
        {
            gnuplotData.data.emplace_back(obs->yawPitchRollUncertainty.value().y());
        }
        else if (gnuplotData.dataIdentifier == "Roll Uncertainty" && obs->yawPitchRollUncertainty.has_value())
        {
            gnuplotData.data.emplace_back(obs->yawPitchRollUncertainty.value().z());
        }

        // Delete old data
        if (NodeManager::appContext != NodeContext::POST_PROCESSING)
        {
            while (xDisplayScope != 0 && gnuplotData.data.size() > xDisplayScope)
            {
                gnuplotData.data.pop_front();
            }
        }
    }

    requestUpdate();

    invokeCallbacks(obs);
}

void NAV::GnuPlot::handleRtklibPosObs(std::shared_ptr<NAV::RtklibPosObs>& obs, size_t portIndex)
{
    for (auto& gnuplotData : plotData[portIndex])
    {
        if (gnuplotData.dataIdentifier == "GPS time of week" && obs->insTime.has_value())
        {
            gnuplotData.data.emplace_back(static_cast<double>(obs->insTime.value().GetGPSTime().tow));
        }
        else if (gnuplotData.dataIdentifier == "Latitude" && obs->positionLLH.has_value())
        {
            gnuplotData.data.emplace_back(obs->positionLLH.value()(1));
        }
        else if (gnuplotData.dataIdentifier == "Longitude" && obs->positionLLH.has_value())
        {
            gnuplotData.data.emplace_back(obs->positionLLH.value()(0));
        }
        else if (gnuplotData.dataIdentifier == "Height" && obs->positionLLH.has_value())
        {
            gnuplotData.data.emplace_back(obs->positionLLH.value()(2));
        }
        else if (gnuplotData.dataIdentifier == "X-ECEF" && obs->positionXYZ.has_value())
        {
            gnuplotData.data.emplace_back(obs->positionXYZ.value().x());
        }
        else if (gnuplotData.dataIdentifier == "Y-ECEF" && obs->positionXYZ.has_value())
        {
            gnuplotData.data.emplace_back(obs->positionXYZ.value().y());
        }
        else if (gnuplotData.dataIdentifier == "Z-ECEF" && obs->positionXYZ.has_value())
        {
            gnuplotData.data.emplace_back(obs->positionXYZ.value().z());
        }
        else if (gnuplotData.dataIdentifier == "Q" && obs->Q.has_value())
        {
            gnuplotData.data.emplace_back(obs->Q.value());
        }
        else if (gnuplotData.dataIdentifier == "ns" && obs->ns.has_value())
        {
            gnuplotData.data.emplace_back(obs->ns.value());
        }
        else if (gnuplotData.dataIdentifier == "sdn" && obs->sdNEU.has_value())
        {
            gnuplotData.data.emplace_back(obs->sdNEU.value()(0));
        }
        else if (gnuplotData.dataIdentifier == "sde" && obs->sdNEU.has_value())
        {
            gnuplotData.data.emplace_back(obs->sdNEU.value()(1));
        }
        else if (gnuplotData.dataIdentifier == "sdu" && obs->sdNEU.has_value())
        {
            gnuplotData.data.emplace_back(obs->sdNEU.value()(2));
        }
        else if (gnuplotData.dataIdentifier == "sdx" && obs->sdXYZ.has_value())
        {
            gnuplotData.data.emplace_back(obs->sdXYZ.value().x());
        }
        else if (gnuplotData.dataIdentifier == "sdy" && obs->sdXYZ.has_value())
        {
            gnuplotData.data.emplace_back(obs->sdXYZ.value().y());
        }
        else if (gnuplotData.dataIdentifier == "sdz" && obs->sdXYZ.has_value())
        {
            gnuplotData.data.emplace_back(obs->sdXYZ.value().z());
        }
        else if (gnuplotData.dataIdentifier == "sdne" && obs->sdne.has_value())
        {
            gnuplotData.data.emplace_back(obs->sdne.value());
        }
        else if (gnuplotData.dataIdentifier == "sdeu" && obs->sdeu.has_value())
        {
            gnuplotData.data.emplace_back(obs->sdeu.value());
        }
        else if (gnuplotData.dataIdentifier == "sdun" && obs->sdun.has_value())
        {
            gnuplotData.data.emplace_back(obs->sdun.value());
        }
        else if (gnuplotData.dataIdentifier == "sdxy" && obs->sdxy.has_value())
        {
            gnuplotData.data.emplace_back(obs->sdxy.value());
        }
        else if (gnuplotData.dataIdentifier == "sdyz" && obs->sdyz.has_value())
        {
            gnuplotData.data.emplace_back(obs->sdyz.value());
        }
        else if (gnuplotData.dataIdentifier == "sdzx" && obs->sdzx.has_value())
        {
            gnuplotData.data.emplace_back(obs->sdzx.value());
        }
        else if (gnuplotData.dataIdentifier == "Age" && obs->age.has_value())
        {
            gnuplotData.data.emplace_back(obs->age.value());
        }
        else if (gnuplotData.dataIdentifier == "Ratio" && obs->ratio.has_value())
        {
            gnuplotData.data.emplace_back(obs->ratio.value());
        }

        // Delete old data
        if (NodeManager::appContext != NodeContext::POST_PROCESSING)
        {
            while (xDisplayScope != 0 && gnuplotData.data.size() > xDisplayScope)
            {
                gnuplotData.data.pop_front();
            }
        }
    }

    requestUpdate();

    invokeCallbacks(obs);
}

void NAV::GnuPlot::handleKvhObs(std::shared_ptr<NAV::KvhObs>& obs, size_t portIndex)
{
    for (auto& gnuplotData : plotData[portIndex])
    {
        if (gnuplotData.dataIdentifier == "GPS time of week" && obs->insTime.has_value())
        {
            gnuplotData.data.emplace_back(static_cast<double>(obs->insTime.value().GetGPSTime().tow));
        }
        else if (gnuplotData.dataIdentifier == "Time since startup" && obs->timeSinceStartup.has_value())
        {
            gnuplotData.data.emplace_back(obs->timeSinceStartup.value());
        }
        else if (gnuplotData.dataIdentifier == "Temperature" && obs->temperature.has_value())
        {
            gnuplotData.data.emplace_back(obs->temperature.value());
        }
        else if (gnuplotData.dataIdentifier == "Sequence Number")
        {
            gnuplotData.data.emplace_back(obs->sequenceNumber);
        }
        else if (gnuplotData.dataIdentifier == "Mag uncomp X" && obs->magUncompXYZ.has_value())
        {
            gnuplotData.data.emplace_back(obs->magUncompXYZ.value().x());
        }
        else if (gnuplotData.dataIdentifier == "Mag uncomp Y" && obs->magUncompXYZ.has_value())
        {
            gnuplotData.data.emplace_back(obs->magUncompXYZ.value().y());
        }
        else if (gnuplotData.dataIdentifier == "Mag uncomp Z" && obs->magUncompXYZ.has_value())
        {
            gnuplotData.data.emplace_back(obs->magUncompXYZ.value().z());
        }
        else if (gnuplotData.dataIdentifier == "Accel uncomp X" && obs->accelUncompXYZ.has_value())
        {
            gnuplotData.data.emplace_back(obs->accelUncompXYZ.value().x());
        }
        else if (gnuplotData.dataIdentifier == "Accel uncomp Y" && obs->accelUncompXYZ.has_value())
        {
            gnuplotData.data.emplace_back(obs->accelUncompXYZ.value().y());
        }
        else if (gnuplotData.dataIdentifier == "Accel uncomp Z" && obs->accelUncompXYZ.has_value())
        {
            gnuplotData.data.emplace_back(obs->accelUncompXYZ.value().z());
        }
        else if (gnuplotData.dataIdentifier == "Gyro uncomp X" && obs->gyroUncompXYZ.has_value())
        {
            gnuplotData.data.emplace_back(obs->gyroUncompXYZ.value().x());
        }
        else if (gnuplotData.dataIdentifier == "Gyro uncomp Y" && obs->gyroUncompXYZ.has_value())
        {
            gnuplotData.data.emplace_back(obs->gyroUncompXYZ.value().y());
        }
        else if (gnuplotData.dataIdentifier == "Gyro uncomp Z" && obs->gyroUncompXYZ.has_value())
        {
            gnuplotData.data.emplace_back(obs->gyroUncompXYZ.value().z());
        }

        // Delete old data
        if (NodeManager::appContext != NodeContext::POST_PROCESSING)
        {
            while (xDisplayScope != 0 && gnuplotData.data.size() > xDisplayScope)
            {
                gnuplotData.data.pop_front();
            }
        }
    }

    requestUpdate();

    invokeCallbacks(obs);
}

void NAV::GnuPlot::handleImuObs(std::shared_ptr<NAV::ImuObs>& obs, size_t portIndex)
{
    for (auto& gnuplotData : plotData[portIndex])
    {
        if (gnuplotData.dataIdentifier == "GPS time of week" && obs->insTime.has_value())
        {
            gnuplotData.data.emplace_back(static_cast<double>(obs->insTime.value().GetGPSTime().tow));
        }
        else if (gnuplotData.dataIdentifier == "Time since startup" && obs->timeSinceStartup.has_value())
        {
            gnuplotData.data.emplace_back(obs->timeSinceStartup.value());
        }
        else if (gnuplotData.dataIdentifier == "Temperature" && obs->temperature.has_value())
        {
            gnuplotData.data.emplace_back(obs->temperature.value());
        }
        else if (gnuplotData.dataIdentifier == "Mag uncomp X" && obs->magUncompXYZ.has_value())
        {
            gnuplotData.data.emplace_back(obs->magUncompXYZ.value().x());
        }
        else if (gnuplotData.dataIdentifier == "Mag uncomp Y" && obs->magUncompXYZ.has_value())
        {
            gnuplotData.data.emplace_back(obs->magUncompXYZ.value().y());
        }
        else if (gnuplotData.dataIdentifier == "Mag uncomp Z" && obs->magUncompXYZ.has_value())
        {
            gnuplotData.data.emplace_back(obs->magUncompXYZ.value().z());
        }
        else if (gnuplotData.dataIdentifier == "Accel uncomp X" && obs->accelUncompXYZ.has_value())
        {
            gnuplotData.data.emplace_back(obs->accelUncompXYZ.value().x());
        }
        else if (gnuplotData.dataIdentifier == "Accel uncomp Y" && obs->accelUncompXYZ.has_value())
        {
            gnuplotData.data.emplace_back(obs->accelUncompXYZ.value().y());
        }
        else if (gnuplotData.dataIdentifier == "Accel uncomp Z" && obs->accelUncompXYZ.has_value())
        {
            gnuplotData.data.emplace_back(obs->accelUncompXYZ.value().z());
        }
        else if (gnuplotData.dataIdentifier == "Gyro uncomp X" && obs->gyroUncompXYZ.has_value())
        {
            gnuplotData.data.emplace_back(obs->gyroUncompXYZ.value().x());
        }
        else if (gnuplotData.dataIdentifier == "Gyro uncomp Y" && obs->gyroUncompXYZ.has_value())
        {
            gnuplotData.data.emplace_back(obs->gyroUncompXYZ.value().y());
        }
        else if (gnuplotData.dataIdentifier == "Gyro uncomp Z" && obs->gyroUncompXYZ.has_value())
        {
            gnuplotData.data.emplace_back(obs->gyroUncompXYZ.value().z());
        }

        // Delete old data
        if (NodeManager::appContext != NodeContext::POST_PROCESSING)
        {
            while (xDisplayScope != 0 && gnuplotData.data.size() > xDisplayScope)
            {
                gnuplotData.data.pop_front();
            }
        }
    }

    requestUpdate();

    invokeCallbacks(obs);
}