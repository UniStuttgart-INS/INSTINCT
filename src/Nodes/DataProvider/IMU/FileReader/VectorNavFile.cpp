#include "VectorNavFile.hpp"

#include "util/Logger.hpp"
#include "util/InsTransformations.hpp"

#include "gui/widgets/FileDialog.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeData/IMU/VectorNavImuObs.hpp"

NAV::VectorNavFile::VectorNavFile()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    hasConfig = true;
    guiConfigDefaultWindowSize = { 395, 335 };

    nm::CreateOutputPin(this, "VectorNavImuObs", Pin::Type::Flow, NAV::VectorNavImuObs::type(), &VectorNavFile::pollData);
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
    auto obs = std::make_shared<VectorNavImuObs>(imuPos);

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

    LOG_DATA("DATA({}): {}, {}, {}, {}, {}",
             name, obs->timeSinceStartup.value(), obs->temperature.value(),
             obs->accelUncompXYZ.value().x(), obs->accelUncompXYZ.value().y(), obs->accelUncompXYZ.value().z());

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