#include "VectorNavFile.hpp"

#include "util/Logger.hpp"
#include "util/InsTransformations.hpp"

#include "gui/widgets/FileDialog.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeData/IMU/VectorNavObs.hpp"

NAV::VectorNavFile::VectorNavFile()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    hasConfig = true;
    guiConfigDefaultWindowSize = { 395, 335 };

    nm::CreateOutputPin(this, "VectorNavObs", Pin::Type::Flow, NAV::VectorNavObs::type(), &VectorNavFile::pollData);
    nm::CreateOutputPin(this, "Header Columns", Pin::Type::Object, "std::vector<std::string>", &headerColumns);
}

NAV::VectorNavFile::~VectorNavFile()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::VectorNavFile::typeStatic()
{
    return "VectorNavFile";
}

std::string NAV::VectorNavFile::type() const
{
    return typeStatic();
}

std::string NAV::VectorNavFile::category()
{
    return "Data Provider";
}

void NAV::VectorNavFile::guiConfig()
{
    if (gui::widgets::FileDialogLoad(path, "Select File", ".csv", { ".csv" }, size_t(id), nameId()))
    {
        flow::ApplyChanges();
        initializeNode();
    }

    // Header info
    if (ImGui::BeginTable(fmt::format("##VectorNavHeaders ({})", id.AsPointer()).c_str(), 3,
                          ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg))
    {
        ImGui::TableSetupColumn("Time", ImGuiTableColumnFlags_WidthAutoResize);
        ImGui::TableSetupColumn("IMU", ImGuiTableColumnFlags_WidthAutoResize);
        ImGui::TableSetupColumn("Attitude", ImGuiTableColumnFlags_WidthAutoResize);
        ImGui::TableHeadersRow();

        auto TextColoredIfExists = [this](int index, const char* displayText, const char* searchText, bool alwaysNormal = false) {
            ImGui::TableSetColumnIndex(index);
            if (alwaysNormal || std::find(headerColumns.begin(), headerColumns.end(), searchText) != headerColumns.end())
            {
                ImGui::TextUnformatted(displayText);
            }
            else
            {
                ImGui::TextDisabled("%s", displayText);
            }
        };

        ImGui::TableNextRow();
        TextColoredIfExists(0, "GpsTime", "GpsToW");
        TextColoredIfExists(1, "VpeStatus", "AhrsStatus");
        TextColoredIfExists(2, "YawPitchRoll", "Yaw");
        ImGui::TableNextRow();
        TextColoredIfExists(0, "TimeStartup", "TimeStartup");
        TextColoredIfExists(1, "UncompMag", "UnCompMagX");
        TextColoredIfExists(2, "Quaternion", "Quat[0]");
        ImGui::TableNextRow();
        TextColoredIfExists(0, "TimeSyncIn", "TimeSyncIn");
        TextColoredIfExists(1, "UncompAccel", "UnCompAccX");
        TextColoredIfExists(2, "DCM", "C[0.0]");
        ImGui::TableNextRow();
        TextColoredIfExists(0, "SyncInCnt", "SyncInCnt");
        TextColoredIfExists(1, "UncompAngularRate", "UnCompGyroX");
        TextColoredIfExists(2, "MagNed", "MagN");
        ImGui::TableNextRow();
        TextColoredIfExists(0, "SyncOutCnt", "SyncOutCnt");
        TextColoredIfExists(1, "Temp", "Temperature");
        TextColoredIfExists(2, "AccelNed", "AccN");
        ImGui::TableNextRow();
        TextColoredIfExists(1, "Pres", "Pressure");
        TextColoredIfExists(2, "GyroNed", "YawRate");
        ImGui::TableNextRow();
        TextColoredIfExists(1, "DeltaTheta", "DeltaThetaX");
        TextColoredIfExists(2, "LinearAccelBody", "LinAccX");
        ImGui::TableNextRow();
        TextColoredIfExists(1, "DeltaVel", "DeltaVelX");
        TextColoredIfExists(2, "LinearAccelNed", "LinAccN");
        ImGui::TableNextRow();
        TextColoredIfExists(1, "Mag", "MagX");
        TextColoredIfExists(2, "YprU", "YawU");
        ImGui::TableNextRow();
        TextColoredIfExists(1, "Accel", "AccX");
        ImGui::TableNextRow();
        TextColoredIfExists(1, "AngularRate", "GyroX");

        ImGui::EndTable();
    }
}

[[nodiscard]] json NAV::VectorNavFile::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["FileReader"] = FileReader::save();

    return j;
}

void NAV::VectorNavFile::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("FileReader"))
    {
        FileReader::restore(j.at("FileReader"));
    }
}

bool NAV::VectorNavFile::initialize()
{
    LOG_TRACE("{}: called", nameId());

    return FileReader::initialize();
}

void NAV::VectorNavFile::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    FileReader::deinitialize();
}

bool NAV::VectorNavFile::resetNode()
{
    FileReader::resetReader();

    return true;
}

std::shared_ptr<NAV::NodeData> NAV::VectorNavFile::pollData(bool peek)
{
    auto obs = std::make_shared<VectorNavObs>(imuPos);

    if (fileType == FileType::BINARY)
    {
        // TODO: Implement VectorNavFile Binary reading
        LOG_ERROR("Binary VectorNavFile pollData is not implemented yet.");
        return nullptr;
    }
    // Ascii

    // Read line
    std::string line;
    // Get current position
    auto len = filestream.tellg();
    std::getline(filestream, line);
    if (peek)
    {
        // Return to position before "Read line".
        filestream.seekg(len, std::ios_base::beg);
    }
    // Remove any starting non text characters
    line.erase(line.begin(), std::find_if(line.begin(), line.end(), [](int ch) { return std::isgraph(ch); }));

    if (line.empty())
    {
        return nullptr;
    }

    // Convert line into stream
    std::stringstream lineStream(line);
    std::string cell;

    Eigen::Matrix3d dcm_np = Eigen::Matrix3d::Zero();

    std::optional<uint16_t> gpsCycle = 0;
    std::optional<uint16_t> gpsWeek;
    std::optional<long double> gpsToW;
    std::optional<double> magUncompX;
    std::optional<double> magUncompY;
    std::optional<double> magUncompZ;
    std::optional<double> accelUncompX;
    std::optional<double> accelUncompY;
    std::optional<double> accelUncompZ;
    std::optional<double> gyroUncompX;
    std::optional<double> gyroUncompY;
    std::optional<double> gyroUncompZ;
    std::optional<double> dthetaX;
    std::optional<double> dthetaY;
    std::optional<double> dthetaZ;
    std::optional<double> dvelX;
    std::optional<double> dvelY;
    std::optional<double> dvelZ;
    std::optional<double> magCompX;
    std::optional<double> magCompY;
    std::optional<double> magCompZ;
    std::optional<double> accelCompX;
    std::optional<double> accelCompY;
    std::optional<double> accelCompZ;
    std::optional<double> gyroCompX;
    std::optional<double> gyroCompY;
    std::optional<double> gyroCompZ;
    std::optional<double> yaw;
    std::optional<double> pitch;
    std::optional<double> roll;
    std::optional<double> quatX;
    std::optional<double> quatY;
    std::optional<double> quatZ;
    std::optional<double> quatW;
    std::optional<double> magCompN;
    std::optional<double> magCompE;
    std::optional<double> magCompD;
    std::optional<double> accelCompN;
    std::optional<double> accelCompE;
    std::optional<double> accelCompD;
    std::optional<double> linearAccelX;
    std::optional<double> linearAccelY;
    std::optional<double> linearAccelZ;
    std::optional<double> linearAccelN;
    std::optional<double> linearAccelE;
    std::optional<double> linearAccelD;
    std::optional<double> yawUncertainty;
    std::optional<double> pitchUncertainty;
    std::optional<double> rollUncertainty;
    std::optional<double> gyroCompN;
    std::optional<double> gyroCompE;
    std::optional<double> gyroCompD;

    // Split line at comma
    for (const auto& column : headerColumns)
    {
        if (std::getline(lineStream, cell, ','))
        {
            // Remove any trailing non text characters
            cell.erase(std::find_if(cell.begin(), cell.end(), [](int ch) { return std::iscntrl(ch); }), cell.end());
            if (cell.empty())
            {
                continue;
            }

            if (column == "GpsCycle")
            {
                gpsCycle = static_cast<uint16_t>(std::stoul(cell));
            }
            else if (column == "GpsWeek")
            {
                gpsWeek = static_cast<uint16_t>(std::stoul(cell));
            }
            else if (column == "GpsToW")
            {
                gpsToW = std::stold(cell);
            }
            else if (column == "TimeStartup")
            {
                obs->timeSinceStartup.emplace(std::stoull(cell));
            }
            else if (column == "TimeSyncIn")
            {
                obs->timeSinceSyncIn.emplace(std::stoull(cell));
            }
            else if (column == "SyncInCnt")
            {
                obs->syncInCnt.emplace(std::stoul(cell));
            }
            else if (column == "UnCompMagX")
            {
                magUncompX = std::stod(cell);
            }
            else if (column == "UnCompMagY")
            {
                magUncompY = std::stod(cell);
            }
            else if (column == "UnCompMagZ")
            {
                magUncompZ = std::stod(cell);
            }
            else if (column == "UnCompAccX")
            {
                accelUncompX = std::stod(cell);
            }
            else if (column == "UnCompAccY")
            {
                accelUncompY = std::stod(cell);
            }
            else if (column == "UnCompAccZ")
            {
                accelUncompZ = std::stod(cell);
            }
            else if (column == "UnCompGyroX")
            {
                gyroUncompX = std::stod(cell);
            }
            else if (column == "UnCompGyroY")
            {
                gyroUncompY = std::stod(cell);
            }
            else if (column == "UnCompGyroZ")
            {
                gyroUncompZ = std::stod(cell);
            }
            else if (column == "Temperature")
            {
                obs->temperature.emplace(std::stod(cell));
            }
            else if (column == "Pressure")
            {
                obs->pressure.emplace(std::stod(cell));
            }
            else if (column == "DeltaTime")
            {
                obs->dtime.emplace(std::stod(cell));
            }
            else if (column == "DeltaThetaX")
            {
                dthetaX = std::stod(cell);
            }
            else if (column == "DeltaThetaY")
            {
                dthetaY = std::stod(cell);
            }
            else if (column == "DeltaThetaZ")
            {
                dthetaZ = std::stod(cell);
            }
            else if (column == "DeltaVelX")
            {
                dvelX = std::stod(cell);
            }
            else if (column == "DeltaVelY")
            {
                dvelY = std::stod(cell);
            }
            else if (column == "DeltaVelZ")
            {
                dvelZ = std::stod(cell);
            }
            else if (column == "MagX")
            {
                magCompX = std::stod(cell);
            }
            else if (column == "MagY")
            {
                magCompY = std::stod(cell);
            }
            else if (column == "MagZ")
            {
                magCompZ = std::stod(cell);
            }
            else if (column == "AccX")
            {
                accelCompX = std::stod(cell);
            }
            else if (column == "AccY")
            {
                accelCompY = std::stod(cell);
            }
            else if (column == "AccZ")
            {
                accelCompZ = std::stod(cell);
            }
            else if (column == "GyroX")
            {
                gyroCompX = std::stod(cell);
            }
            else if (column == "GyroY")
            {
                gyroCompY = std::stod(cell);
            }
            else if (column == "GyroZ")
            {
                gyroCompZ = std::stod(cell);
            }
            else if (column == "AhrsStatus")
            {
                obs->vpeStatus.emplace(std::stoul(cell));
            }
            else if (column == "Yaw")
            {
                yaw = std::stod(cell);
            }
            else if (column == "Pitch")
            {
                pitch = std::stod(cell);
            }
            else if (column == "Roll")
            {
                roll = std::stod(cell);
            }
            else if (column == "Quat[0]")
            {
                quatW = std::stod(cell);
            }
            else if (column == "Quat[1]")
            {
                quatX = std::stod(cell);
            }
            else if (column == "Quat[2]")
            {
                quatY = std::stod(cell);
            }
            else if (column == "Quat[3]")
            {
                quatZ = std::stod(cell);
            }
            else if (column == "C[0.0]")
            {
                dcm_np(0, 0) = std::stod(cell);
            }
            else if (column == "C[0.1]")
            {
                dcm_np(0, 1) = std::stod(cell);
            }
            else if (column == "C[0.2]")
            {
                dcm_np(0, 2) = std::stod(cell);
            }
            else if (column == "C[1.0]")
            {
                dcm_np(1, 0) = std::stod(cell);
            }
            else if (column == "C[1.1]")
            {
                dcm_np(1, 1) = std::stod(cell);
            }
            else if (column == "C[1.2]")
            {
                dcm_np(1, 2) = std::stod(cell);
            }
            else if (column == "C[2.0]")
            {
                dcm_np(2, 0) = std::stod(cell);
            }
            else if (column == "C[2.1]")
            {
                dcm_np(2, 1) = std::stod(cell);
            }
            else if (column == "C[2.2]")
            {
                dcm_np(2, 2) = std::stod(cell);
            }
            else if (column == "MagN")
            {
                magCompN = std::stod(cell);
            }
            else if (column == "MagE")
            {
                magCompE = std::stod(cell);
            }
            else if (column == "MagD")
            {
                magCompD = std::stod(cell);
            }
            else if (column == "AccN")
            {
                accelCompN = std::stod(cell);
            }
            else if (column == "AccE")
            {
                accelCompE = std::stod(cell);
            }
            else if (column == "AccD")
            {
                accelCompD = std::stod(cell);
            }
            else if (column == "LinAccX")
            {
                linearAccelX = std::stod(cell);
            }
            else if (column == "LinAccY")
            {
                linearAccelY = std::stod(cell);
            }
            else if (column == "LinAccZ")
            {
                linearAccelZ = std::stod(cell);
            }
            else if (column == "LinAccN")
            {
                linearAccelN = std::stod(cell);
            }
            else if (column == "LinAccE")
            {
                linearAccelE = std::stod(cell);
            }
            else if (column == "LinAccD")
            {
                linearAccelD = std::stod(cell);
            }
            else if (column == "YawU")
            {
                yawUncertainty = std::stod(cell);
            }
            else if (column == "PitchU")
            {
                pitchUncertainty = std::stod(cell);
            }
            else if (column == "RollU")
            {
                rollUncertainty = std::stod(cell);
            }
            else if (column == "YawRate")
            {
                gyroCompN = std::stod(cell);
            }
            else if (column == "PitchRate")
            {
                gyroCompE = std::stod(cell);
            }
            else if (column == "RollRate")
            {
                gyroCompD = std::stod(cell);
            }
        }
    }

    if (gpsWeek.has_value() && gpsToW.has_value())
    {
        obs->insTime.emplace(gpsCycle.value(), gpsWeek.value(), gpsToW.value());
    }
    if (magUncompX.has_value() && magUncompY.has_value() && magUncompZ.has_value())
    {
        obs->magUncompXYZ.emplace(magUncompX.value(), magUncompY.value(), magUncompZ.value());
    }
    if (accelUncompX.has_value() && accelUncompY.has_value() && accelUncompZ.has_value())
    {
        obs->accelUncompXYZ.emplace(accelUncompX.value(), accelUncompY.value(), accelUncompZ.value());
    }
    if (gyroUncompX.has_value() && gyroUncompY.has_value() && gyroUncompZ.has_value())
    {
        obs->gyroUncompXYZ.emplace(gyroUncompX.value(), gyroUncompY.value(), gyroUncompZ.value());
    }
    if (dthetaX.has_value() && dthetaY.has_value() && dthetaZ.has_value())
    {
        obs->dtheta.emplace(dthetaX.value(), dthetaY.value(), dthetaZ.value());
    }
    if (dvelX.has_value() && dvelY.has_value() && dvelZ.has_value())
    {
        obs->dvel.emplace(dvelX.value(), dvelY.value(), dvelZ.value());
    }
    if (magCompX.has_value() && magCompY.has_value() && magCompZ.has_value())
    {
        obs->magCompXYZ.emplace(magCompX.value(), magCompY.value(), magCompZ.value());
    }
    if (accelCompX.has_value() && accelCompY.has_value() && accelCompZ.has_value())
    {
        obs->accelCompXYZ.emplace(accelCompX.value(), accelCompY.value(), accelCompZ.value());
    }
    if (gyroCompX.has_value() && gyroCompY.has_value() && gyroCompZ.has_value())
    {
        obs->gyroCompXYZ.emplace(gyroCompX.value(), gyroCompY.value(), gyroCompZ.value());
    }
    if (yaw.has_value() && pitch.has_value() && roll.has_value())
    {
        obs->yawPitchRoll.emplace(yaw.value(), pitch.value(), roll.value());
    }
    if (quatW.has_value() && quatX.has_value() && quatY.has_value() && quatZ.has_value())
    {
        obs->quaternion.emplace(quatW.value(), quatX.value(), quatY.value(), quatZ.value());
    }
    if (magCompN.has_value() && magCompE.has_value() && magCompD.has_value())
    {
        obs->magCompNED.emplace(magCompN.value(), magCompE.value(), magCompD.value());
    }
    if (accelCompN.has_value() && accelCompE.has_value() && accelCompD.has_value())
    {
        obs->accelCompNED.emplace(accelCompN.value(), accelCompE.value(), accelCompD.value());
    }
    if (linearAccelX.has_value() && linearAccelY.has_value() && linearAccelZ.has_value())
    {
        obs->linearAccelXYZ.emplace(linearAccelX.value(), linearAccelY.value(), linearAccelZ.value());
    }
    if (linearAccelN.has_value() && linearAccelE.has_value() && linearAccelD.has_value())
    {
        obs->linearAccelNED.emplace(linearAccelN.value(), linearAccelE.value(), linearAccelD.value());
    }
    if (yawUncertainty.has_value() && pitchUncertainty.has_value() && rollUncertainty.has_value())
    {
        obs->yawPitchRollUncertainty.emplace(yawUncertainty.value(), pitchUncertainty.value(), rollUncertainty.value());
    }
    if (gyroCompN.has_value() && gyroCompE.has_value() && gyroCompD.has_value())
    {
        obs->gyroCompNED.emplace(gyroCompN.value(), gyroCompE.value(), gyroCompD.value());
    }

    if (!obs->quaternion.has_value())
    {
        if (!dcm_np.isZero())
        {
            obs->quaternion.emplace(dcm_np);
        }
        else if (obs->yawPitchRoll.has_value() && !obs->yawPitchRoll.value().isZero())
        {
            obs->quaternion = trafo::quat_nb(trafo::deg2rad(obs->yawPitchRoll.value()(2)),
                                             trafo::deg2rad(obs->yawPitchRoll.value()(1)),
                                             trafo::deg2rad(obs->yawPitchRoll.value()(0)))
                              * Eigen::Quaterniond::Identity();
        }
    }

    LOG_DATA("DATA({}): {}, {}, {}, {}, {}", nameId(),
             obs->timeSinceStartup.has_value() ? std::to_string(obs->timeSinceStartup.value()) : "N/A",
             obs->syncInCnt.has_value() ? std::to_string(obs->syncInCnt.value()) : "N/A",
             obs->timeSinceSyncIn.has_value() ? std::to_string(obs->timeSinceSyncIn.value()) : "N/A",
             obs->vpeStatus.has_value() ? std::to_string(obs->vpeStatus.value().status) : "N/A",
             obs->temperature.has_value() ? std::to_string(obs->temperature.value()) : "N/A");

    if (obs->insTime.has_value())
    {
        // Has time value, but value should not be displayed
        if (obs->insTime.value() < lowerLimit)
        {
            // Resetting the value will make the read loop skip the message
            obs->insTime.reset();
            return obs;
        }
        if (obs->insTime.value() > upperLimit)
        {
            return nullptr;
        }
    }

    // Calls all the callbacks
    if (!peek)
    {
        invokeCallbacks(OutputPortIndex_VectorNavObs, obs);
    }

    return obs;
}