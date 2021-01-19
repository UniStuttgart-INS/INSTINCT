#include "ImuFile.hpp"

#include "util/Logger.hpp"

#include "util/InsTransformations.hpp"

#include "gui/widgets/FileDialog.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeData/IMU/ImuObs.hpp"

NAV::ImuFile::ImuFile()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    color = ImColor(255, 128, 128);
    hasConfig = true;

    nm::CreateOutputPin(this, "ImuObs", Pin::Type::Flow, NAV::ImuObs::type(), &ImuFile::pollData);
    nm::CreateOutputPin(this, "Header Columns", Pin::Type::Object, "std::vector<std::string>", &headerColumns);
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
    if (gui::widgets::FileDialogLoad(path, "Select File", ".csv", { ".csv" }, size_t(id), nameId()))
    {
        flow::ApplyChanges();
        deinitializeNode();
    }

    // Header info
    if (ImGui::BeginTable(fmt::format("##ImuHeaders ({})", id.AsPointer()).c_str(), 2,
                          ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg))
    {
        ImGui::TableSetupColumn("Time", ImGuiTableColumnFlags_WidthAutoResize);
        ImGui::TableSetupColumn("IMU", ImGuiTableColumnFlags_WidthAutoResize);
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
        TextColoredIfExists(0, "GpsCycle", "GpsCycle");
        TextColoredIfExists(1, "UnCompMag", "UnCompMagX");
        ImGui::TableNextRow();
        TextColoredIfExists(0, "GpsWeek", "GpsWeek");
        TextColoredIfExists(1, "UnCompAcc", "UnCompAccX");
        ImGui::TableNextRow();
        TextColoredIfExists(0, "GpsToW", "GpsToW");
        TextColoredIfExists(1, "UnCompGyro", "UnCompGyroX");
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

    return j;
}

void NAV::ImuFile::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("FileReader"))
    {
        FileReader::restore(j.at("FileReader"));
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

void NAV::ImuFile::resetNode()
{
    FileReader::resetReader();
}

std::shared_ptr<NAV::NodeData> NAV::ImuFile::pollData(bool peek)
{
    auto obs = std::make_shared<ImuObs>(imuPos);

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
        invokeCallbacks(OutputPortIndex_ImuObs, obs);
    }

    return obs;
}