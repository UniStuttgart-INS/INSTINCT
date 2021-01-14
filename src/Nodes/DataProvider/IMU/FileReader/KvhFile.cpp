#include "KvhFile.hpp"

#include "util/Logger.hpp"

#include "ImGuiFileDialog.h"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "util/UartSensors/KVH/KvhUtilities.hpp"

#include "NodeData/IMU/KvhObs.hpp"

NAV::KvhFile::KvhFile()
    : sensor(typeStatic())
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    color = ImColor(255, 128, 128);
    hasConfig = true;

    nm::CreateOutputPin(this, "", Pin::Type::Delegate, "KvhFile", this);

    nm::CreateOutputPin(this, "KvhObs", Pin::Type::Flow, NAV::KvhObs::type(), &KvhFile::pollData);
    nm::CreateOutputPin(this, "Header Columns", Pin::Type::Object, "std::vector<std::string>", &headerColumns);
}

NAV::KvhFile::~KvhFile()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::KvhFile::typeStatic()
{
    return "KvhFile";
}

std::string NAV::KvhFile::type() const
{
    return typeStatic();
}

std::string NAV::KvhFile::category()
{
    return "Data Provider";
}

void NAV::KvhFile::guiConfig()
{
    // Filepath
    if (ImGui::InputText("Filepath", &path))
    {
        LOG_DEBUG("{}: Filepath changed to {}", nameId(), path);
        flow::ApplyChanges();
        deinitialize();
    }
    ImGui::SameLine();
    std::string openFileDialogKey = fmt::format("Select File ({})", id.AsPointer());
    if (ImGui::Button("Open"))
    {
        igfd::ImGuiFileDialog::Instance()->OpenDialog(openFileDialogKey, "Select File", ".csv", "");
        igfd::ImGuiFileDialog::Instance()->SetExtentionInfos(".csv", ImVec4(0.0F, 1.0F, 0.0F, 0.9F));
    }

    if (igfd::ImGuiFileDialog::Instance()->FileDialog(openFileDialogKey, ImGuiWindowFlags_NoCollapse, ImVec2(600, 500)))
    {
        if (igfd::ImGuiFileDialog::Instance()->IsOk)
        {
            path = igfd::ImGuiFileDialog::Instance()->GetFilePathName();
            LOG_DEBUG("{}: Selected file: {}", nameId(), path);
            flow::ApplyChanges();
            initialize();
        }

        igfd::ImGuiFileDialog::Instance()->CloseDialog();
    }
    if (fileType == FileType::ASCII)
    {
        // Header info
        if (ImGui::BeginTable(fmt::format("##VectorNavHeaders ({})", id.AsPointer()).c_str(), 2,
                              ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg))
        {
            ImGui::TableSetupColumn("Basic", ImGuiTableColumnFlags_WidthAutoResize);
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
            TextColoredIfExists(0, "GpsTime", "GpsToW");
            TextColoredIfExists(1, "UnCompMag", "UnCompMagX");
            ImGui::TableNextRow();
            TextColoredIfExists(0, "TimeStartup", "TimeStartup");
            TextColoredIfExists(1, "UnCompAcc", "UnCompAccX");
            ImGui::TableNextRow();
            TextColoredIfExists(0, "Temperature", "Temperature");
            TextColoredIfExists(1, "UnCompGyro", "UnCompGyroX");
            ImGui::TableNextRow();
            TextColoredIfExists(0, "Status", "Status");
            ImGui::TableNextRow();
            TextColoredIfExists(0, "SequenceNumber", "SequenceNumber");

            ImGui::EndTable();
        }
    }
    else if (fileType == FileType::BINARY)
    {
        ImGui::TextUnformatted("Binary file");
    }
}

[[nodiscard]] json NAV::KvhFile::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["FileReader"] = FileReader::save();

    return j;
}

void NAV::KvhFile::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("FileReader"))
    {
        FileReader::restore(j.at("FileReader"));
    }
}

bool NAV::KvhFile::initialize()
{
    deinitialize();

    LOG_TRACE("{}: called", nameId());

    if (!Node::initialize()
        || !FileReader::initialize())
    {
        return false;
    }

    return isInitialized = true;
}

void NAV::KvhFile::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    FileReader::deinitialize();
    Node::deinitialize();
}

void NAV::KvhFile::resetNode()
{
    FileReader::resetReader();
}

std::shared_ptr<NAV::NodeData> NAV::KvhFile::pollData(bool peek)
{
    std::shared_ptr<KvhObs> obs = nullptr;

    // Get current position
    auto pos = filestream.tellg();

    if (fileType == FileType::BINARY)
    {
        uint8_t i = 0;
        std::unique_ptr<uart::protocol::Packet> packet = nullptr;
        while (filestream.readsome(reinterpret_cast<char*>(&i), 1))
        {
            packet = sensor.findPacket(i);

            if (packet != nullptr)
            {
                break;
            }
        }

        if (!packet)
        {
            return nullptr;
        }

        obs = std::make_shared<KvhObs>(imuPos, *packet);

        // Check if package is empty
        if (obs->raw.getRawDataLength() == 0)
        {
            return nullptr;
        }

        sensors::kvh::decryptKvhObs(obs);
    }
    else if (fileType == FileType::ASCII)
    {
        obs = std::make_shared<KvhObs>(imuPos);

        // Read line
        std::string line;
        std::getline(filestream, line);
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
                else if (column == "Status")
                {
                    obs->status = std::bitset<8>{ cell };
                }
                else if (column == "SequenceNumber")
                {
                    obs->sequenceNumber = static_cast<uint8_t>(std::stoul(cell));
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
    }

    LOG_DATA("DATA({}): {}, {}, {}, {}, {}",
             name, obs->sequenceNumber, obs->temperature.value(),
             obs->accelUncompXYZ.value().x(), obs->accelUncompXYZ.value().y(), obs->accelUncompXYZ.value().z());

    // Check if a packet was skipped
    if (!peek && callbacksEnabled)
    {
        if (prevSequenceNumber == UINT8_MAX)
        {
            prevSequenceNumber = obs->sequenceNumber;
        }
        if (obs->sequenceNumber != 0 && (obs->sequenceNumber < prevSequenceNumber || obs->sequenceNumber > prevSequenceNumber + 2))
        {
            LOG_WARN("{}: Sequence Number changed from {} to {}", name, prevSequenceNumber, obs->sequenceNumber);
        }
        prevSequenceNumber = obs->sequenceNumber;
    }

    if (peek)
    {
        // Return to position before "Read line".
        filestream.seekg(pos, std::ios_base::beg);
    }

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
        invokeCallbacks(OutputPortIndex_KvhObs, obs);
    }

    return obs;
}

NAV::FileReader::FileType NAV::KvhFile::determineFileType()
{
    LOG_TRACE("called for {}", name);

    auto filestream = std::ifstream(path);
    if (filestream.good())
    {
        union
        {
            std::array<char, 4> buffer;
            uint32_t ui32;
        } un{};

        if (filestream.readsome(un.buffer.data(), sizeof(uint32_t)) == sizeof(uint32_t))
        {
            un.ui32 = uart::stoh(un.ui32, sensors::kvh::KvhUartSensor::endianness);
            if (un.ui32 == sensors::kvh::KvhUartSensor::HEADER_FMT_A
                || un.ui32 == sensors::kvh::KvhUartSensor::HEADER_FMT_B
                || un.ui32 == sensors::kvh::KvhUartSensor::HEADER_FMT_C
                || un.ui32 == sensors::kvh::KvhUartSensor::HEADER_FMT_XBIT
                || un.ui32 == sensors::kvh::KvhUartSensor::HEADER_FMT_XBIT2)
            {
                return FileType::BINARY;
            }
        }

        filestream.seekg(0, std::ios_base::beg);
        std::string line;
        std::getline(filestream, line);
        filestream.close();

        auto n = std::count(line.begin(), line.end(), ',');

        if (n >= 3)
        {
            return FileType::ASCII;
        }

        LOG_CRITICAL("{} could not determine file type", name);
    }

    LOG_CRITICAL("{} could not open file {}", name, path);
    return FileType::NONE;
}