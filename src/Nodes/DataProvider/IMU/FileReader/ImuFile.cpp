#include "ImuFile.hpp"

#include "util/Logger.hpp"

#include "Navigation/Transformations/CoordinateFrames.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeData/IMU/ImuObs.hpp"

NAV::ImuFile::ImuFile()
    : Imu(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 377, 201 };

    nm::CreateOutputPin(this, "ImuObs", Pin::Type::Flow, { NAV::ImuObs::type() }, &ImuFile::pollData);
    nm::CreateOutputPin(this, "Header Columns", Pin::Type::Object, { "std::vector<std::string>" }, &_headerColumns);
}

NAV::ImuFile::~ImuFile()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::ImuFile::typeStatic()
{
    return "ImuFile";
}

std::string NAV::ImuFile::type() const
{
    return typeStatic();
}

std::string NAV::ImuFile::category()
{
    return "Data Provider";
}

void NAV::ImuFile::guiConfig()
{
    if (auto res = FileReader::guiConfig(".csv,.*", { ".csv" }, size_t(id), nameId()))
    {
        LOG_DEBUG("{}: Path changed to {}", nameId(), _path);
        flow::ApplyChanges();
        if (res == FileReader::PATH_CHANGED)
        {
            doInitialize();
        }
        else
        {
            doDeinitialize();
        }
    }

    Imu::guiConfig();

    // Header info
    if (ImGui::BeginTable(fmt::format("##ImuHeaders ({})", id.AsPointer()).c_str(), 3,
                          ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg))
    {
        ImGui::TableSetupColumn("Time", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("IMU", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("IMU", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableHeadersRow();

        auto TextColoredIfExists = [this](int index, const char* displayText, const char* searchText, bool alwaysNormal = false) {
            ImGui::TableSetColumnIndex(index);
            if (alwaysNormal || std::find(_headerColumns.begin(), _headerColumns.end(), searchText) != _headerColumns.end())
            {
                ImGui::TextUnformatted(displayText);
            }
            else
            {
                ImGui::TextDisabled("%s", displayText);
            }
        };

        ImGui::TableNextRow();
        TextColoredIfExists(0, "GpsCycle", "GpsCycle");
        TextColoredIfExists(1, "UnCompMag", "UnCompMagX");
        TextColoredIfExists(2, "CompMag", "MagX");
        ImGui::TableNextRow();
        TextColoredIfExists(0, "GpsWeek", "GpsWeek");
        TextColoredIfExists(1, "UnCompAcc", "UnCompAccX");
        TextColoredIfExists(2, "CompAcc", "AccX");
        ImGui::TableNextRow();
        TextColoredIfExists(0, "GpsToW", "GpsToW");
        TextColoredIfExists(1, "UnCompGyro", "UnCompGyroX");
        TextColoredIfExists(2, "CompGyro", "GyroX");
        ImGui::TableNextRow();
        TextColoredIfExists(0, "TimeStartup", "TimeStartup");
        TextColoredIfExists(1, "Temperature", "Temperature");

        ImGui::EndTable();
    }
}

[[nodiscard]] json NAV::ImuFile::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["FileReader"] = FileReader::save();
    j["Imu"] = Imu::save();

    return j;
}

void NAV::ImuFile::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("FileReader"))
    {
        FileReader::restore(j.at("FileReader"));
    }
    if (j.contains("Imu"))
    {
        Imu::restore(j.at("Imu"));
    }
}

bool NAV::ImuFile::initialize()
{
    LOG_TRACE("{}: called", nameId());

    return FileReader::initialize();
}

void NAV::ImuFile::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    FileReader::deinitialize();
}

bool NAV::ImuFile::resetNode()
{
    FileReader::resetReader();

    return true;
}

std::shared_ptr<const NAV::NodeData> NAV::ImuFile::pollData(bool peek)
{
    auto obs = std::make_shared<ImuObs>(_imuPos);

    // Read line
    std::string line;
    // Get current position
    auto len = _filestream.tellg();
    std::getline(_filestream, line);
    if (peek)
    {
        // Return to position before "Read line".
        _filestream.seekg(len, std::ios_base::beg);
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
    for (const auto& column : _headerColumns)
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

    // Calls all the callbacks
    if (!peek)
    {
        invokeCallbacks(OUTPUT_PORT_INDEX_IMU_OBS, obs);
    }

    return obs;
}