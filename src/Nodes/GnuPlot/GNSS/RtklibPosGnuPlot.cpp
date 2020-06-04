#include "RtklibPosGnuPlot.hpp"

#include "util/Logger.hpp"
#include "NodeData/IMU/VectorNavObs.hpp"
#include "Nodes/NodeManager.hpp"

#include <tuple>
#include <cmath>

NAV::RtklibPosGnuPlot::RtklibPosGnuPlot(const std::string& name, const std::map<std::string, std::string>& options)
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
            else if (xData == "latitude")
            {
                plotWindow->xLabel = "Latitude [deg]";
            }
            else if (xData == "longitude")
            {
                plotWindow->xLabel = "Longitude [deg]";
            }
            else if (xData == "height")
            {
                plotWindow->xLabel = "Height [m]";
            }
            else if (xData == "x-ecef")
            {
                plotWindow->xLabel = "X-ecef [m]";
            }
            else if (xData == "y-ecef")
            {
                plotWindow->xLabel = "Y-ecef [m]";
            }
            else if (xData == "z-ecef")
            {
                plotWindow->xLabel = "Z-ecef [m]";
            }
        }

        if (yData == "gpsToW")
        {
            dataIndices.push_back(plotWindow->addNewDataSet("GPS time of week [s]"));
        }
        else if (yData == "latitude")
        {
            dataIndices.push_back(plotWindow->addNewDataSet("Latitude [deg]"));
        }
        else if (yData == "longitude")
        {
            dataIndices.push_back(plotWindow->addNewDataSet("Longitude [deg]"));
        }
        else if (yData == "height")
        {
            dataIndices.push_back(plotWindow->addNewDataSet("Height [m]"));
        }
        else if (yData == "x-ecef")
        {
            dataIndices.push_back(plotWindow->addNewDataSet("X-ecef [m]"));
        }
        else if (yData == "y-ecef")
        {
            dataIndices.push_back(plotWindow->addNewDataSet("Y-ecef [m]"));
        }
        else if (yData == "z-ecef")
        {
            dataIndices.push_back(plotWindow->addNewDataSet("Z-ecef [m]"));
        }
        else if (yData == "Q")
        {
            dataIndices.push_back(plotWindow->addNewDataSet("Q = 1:fix, 2:float, 3:sbas, 4:dgps, 5:single, 6:ppp"));
        }
        else if (yData == "ns")
        {
            dataIndices.push_back(plotWindow->addNewDataSet("Number of satellites"));
        }
        else if (yData == "sdn")
        {
            dataIndices.push_back(plotWindow->addNewDataSet("sdn [m]"));
        }
        else if (yData == "sde")
        {
            dataIndices.push_back(plotWindow->addNewDataSet("sde [m]"));
        }
        else if (yData == "sdu")
        {
            dataIndices.push_back(plotWindow->addNewDataSet("sdu [m]"));
        }
        else if (yData == "sdx")
        {
            dataIndices.push_back(plotWindow->addNewDataSet("sdx [m]"));
        }
        else if (yData == "sdy")
        {
            dataIndices.push_back(plotWindow->addNewDataSet("sdy [m]"));
        }
        else if (yData == "sdz")
        {
            dataIndices.push_back(plotWindow->addNewDataSet("sdz [m]"));
        }
        else if (yData == "sdne")
        {
            dataIndices.push_back(plotWindow->addNewDataSet("sdne [m]"));
        }
        else if (yData == "sdeu")
        {
            dataIndices.push_back(plotWindow->addNewDataSet("sdeu [m]"));
        }
        else if (yData == "sdun")
        {
            dataIndices.push_back(plotWindow->addNewDataSet("sdun [m]"));
        }
        else if (yData == "sdxy")
        {
            dataIndices.push_back(plotWindow->addNewDataSet("sdxy [m]"));
        }
        else if (yData == "sdyz")
        {
            dataIndices.push_back(plotWindow->addNewDataSet("sdyz [m]"));
        }
        else if (yData == "sdzx")
        {
            dataIndices.push_back(plotWindow->addNewDataSet("sdzx [m]"));
        }
        else if (yData == "age")
        {
            dataIndices.push_back(plotWindow->addNewDataSet("Age [s]"));
        }
        else if (yData == "ratio")
        {
            dataIndices.push_back(plotWindow->addNewDataSet("Ratio"));
        }
    }
}

NAV::RtklibPosGnuPlot::~RtklibPosGnuPlot()
{
    LOG_TRACE("called for {}", name);
}

void NAV::RtklibPosGnuPlot::plotRtklibPosObs(std::shared_ptr<NAV::RtklibPosObs>& obs)
{
    for (auto& [xData, yData, wIndex, dataIndices] : dataToPlot)
    {
        auto plotWindow = plotWindows.at(wIndex);

        double plotX = 0.0;

        if (xData == "gpsToW" && obs->insTime.has_value())
        {
            plotX = static_cast<double>(obs->insTime.value().GetGPSTime().tow);
        }
        else if (xData == "latitude" && obs->positionLLH.has_value())
        {
            plotX = obs->positionLLH.value()(1);
        }
        else if (xData == "longitude" && obs->positionLLH.has_value())
        {
            plotX = obs->positionLLH.value()(0);
        }
        else if (xData == "height" && obs->positionLLH.has_value())
        {
            plotX = obs->positionLLH.value()(2);
        }
        else if (xData == "x-ecef" && obs->positionXYZ.has_value())
        {
            plotX = obs->positionXYZ.value().x();
        }
        else if (xData == "y-ecef" && obs->positionXYZ.has_value())
        {
            plotX = obs->positionXYZ.value().y();
        }
        else if (xData == "z-ecef" && obs->positionXYZ.has_value())
        {
            plotX = obs->positionXYZ.value().z();
        }
        else
        {
            continue;
        }

        if (yData == "gpsToW" && obs->insTime.has_value())
        {
            plotWindow->data.at(dataIndices.at(0)).xy.emplace_back(std::make_pair(plotX, static_cast<double>(obs->insTime.value().GetGPSTime().tow)));
        }
        else if (yData == "latitude" && obs->positionLLH.has_value())
        {
            plotWindow->data.at(dataIndices.at(0)).xy.emplace_back(std::make_pair(plotX, obs->positionLLH.value()(1)));
        }
        else if (yData == "longitude" && obs->positionLLH.has_value())
        {
            plotWindow->data.at(dataIndices.at(0)).xy.emplace_back(std::make_pair(plotX, obs->positionLLH.value()(0)));
        }
        else if (yData == "height" && obs->positionLLH.has_value())
        {
            plotWindow->data.at(dataIndices.at(0)).xy.emplace_back(std::make_pair(plotX, obs->positionLLH.value()(2)));
        }
        else if (yData == "x-ecef" && obs->positionXYZ.has_value())
        {
            plotWindow->data.at(dataIndices.at(0)).xy.emplace_back(std::make_pair(plotX, obs->positionXYZ.value().x()));
        }
        else if (yData == "y-ecef" && obs->positionXYZ.has_value())
        {
            plotWindow->data.at(dataIndices.at(0)).xy.emplace_back(std::make_pair(plotX, obs->positionXYZ.value().y()));
        }
        else if (yData == "z-ecef" && obs->positionXYZ.has_value())
        {
            plotWindow->data.at(dataIndices.at(0)).xy.emplace_back(std::make_pair(plotX, obs->positionXYZ.value().z()));
        }
        else if (yData == "Q" && obs->Q.has_value())
        {
            plotWindow->data.at(dataIndices.at(0)).xy.emplace_back(std::make_pair(plotX, obs->Q.value()));
        }
        else if (yData == "ns" && obs->ns.has_value())
        {
            plotWindow->data.at(dataIndices.at(0)).xy.emplace_back(std::make_pair(plotX, obs->ns.value()));
        }
        else if (yData == "sdn" && obs->sdNEU.has_value())
        {
            plotWindow->data.at(dataIndices.at(0)).xy.emplace_back(std::make_pair(plotX, obs->sdNEU.value()(0)));
        }
        else if (yData == "sde" && obs->sdNEU.has_value())
        {
            plotWindow->data.at(dataIndices.at(0)).xy.emplace_back(std::make_pair(plotX, obs->sdNEU.value()(1)));
        }
        else if (yData == "sdu" && obs->sdNEU.has_value())
        {
            plotWindow->data.at(dataIndices.at(0)).xy.emplace_back(std::make_pair(plotX, obs->sdNEU.value()(2)));
        }
        else if (yData == "sdx" && obs->sdXYZ.has_value())
        {
            plotWindow->data.at(dataIndices.at(0)).xy.emplace_back(std::make_pair(plotX, obs->sdXYZ.value().x()));
        }
        else if (yData == "sdy" && obs->sdXYZ.has_value())
        {
            plotWindow->data.at(dataIndices.at(0)).xy.emplace_back(std::make_pair(plotX, obs->sdXYZ.value().y()));
        }
        else if (yData == "sdz" && obs->sdXYZ.has_value())
        {
            plotWindow->data.at(dataIndices.at(0)).xy.emplace_back(std::make_pair(plotX, obs->sdXYZ.value().z()));
        }
        else if (yData == "sdne" && obs->sdne.has_value())
        {
            plotWindow->data.at(dataIndices.at(0)).xy.emplace_back(std::make_pair(plotX, obs->sdne.value()));
        }
        else if (yData == "sdeu" && obs->sdeu.has_value())
        {
            plotWindow->data.at(dataIndices.at(0)).xy.emplace_back(std::make_pair(plotX, obs->sdeu.value()));
        }
        else if (yData == "sdun" && obs->sdun.has_value())
        {
            plotWindow->data.at(dataIndices.at(0)).xy.emplace_back(std::make_pair(plotX, obs->sdun.value()));
        }
        else if (yData == "sdxy" && obs->sdxy.has_value())
        {
            plotWindow->data.at(dataIndices.at(0)).xy.emplace_back(std::make_pair(plotX, obs->sdxy.value()));
        }
        else if (yData == "sdyz" && obs->sdyz.has_value())
        {
            plotWindow->data.at(dataIndices.at(0)).xy.emplace_back(std::make_pair(plotX, obs->sdyz.value()));
        }
        else if (yData == "sdzx" && obs->sdzx.has_value())
        {
            plotWindow->data.at(dataIndices.at(0)).xy.emplace_back(std::make_pair(plotX, obs->sdzx.value()));
        }
        else if (yData == "age" && obs->age.has_value())
        {
            plotWindow->data.at(dataIndices.at(0)).xy.emplace_back(std::make_pair(plotX, obs->age.value()));
        }
        else if (yData == "ratio" && obs->ratio.has_value())
        {
            plotWindow->data.at(dataIndices.at(0)).xy.emplace_back(std::make_pair(plotX, obs->ratio.value()));
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