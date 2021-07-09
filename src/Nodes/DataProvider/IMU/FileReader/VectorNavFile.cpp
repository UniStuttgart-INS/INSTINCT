#include "VectorNavFile.hpp"

#include <exception>

#include "util/Logger.hpp"
#include "util/InsTransformations.hpp"

#include "gui/widgets/FileDialog.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeData/VectorNavBinaryOutput.hpp"
#include "Nodes/DataProvider/IMU/Sensors/VectorNavSensor.hpp"

NAV::VectorNavFile::VectorNavFile()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    hasConfig = true;
    guiConfigDefaultWindowSize = { 619, 444 };

    nm::CreateOutputPin(this, "Binary Output", Pin::Type::Flow, NAV::VectorNavBinaryOutput::type(), &VectorNavFile::pollData);
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
    if (gui::widgets::FileDialogLoad(path, "Select File", "Supported types (*.csv *.vnb){.csv,.vnb},.*", { ".csv", ".vnb" }, size_t(id), nameId()))
    {
        flow::ApplyChanges();
        initializeNode();
    }

    // Header info
    if (ImGui::BeginTable(fmt::format("##VectorNavHeaders ({})", id.AsPointer()).c_str(), 6,
                          ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg))
    {
        ImGui::TableSetupColumn("Time", ImGuiTableColumnFlags_WidthAutoResize);
        ImGui::TableSetupColumn("IMU", ImGuiTableColumnFlags_WidthAutoResize);
        ImGui::TableSetupColumn("GNSS1", ImGuiTableColumnFlags_WidthAutoResize);
        ImGui::TableSetupColumn("Attitude", ImGuiTableColumnFlags_WidthAutoResize);
        ImGui::TableSetupColumn("INS", ImGuiTableColumnFlags_WidthAutoResize);
        ImGui::TableSetupColumn("GNSS2", ImGuiTableColumnFlags_WidthAutoResize);
        ImGui::TableHeadersRow();

        auto TextColored = [](int index, const char* label, bool enabled) {
            ImGui::TableSetColumnIndex(index);
            if (enabled)
            {
                ImGui::TextUnformatted(label);
            }
            else
            {
                ImGui::TextDisabled("%s", label);
            }
        };

        for (size_t i = 0; i < 16; i++)
        {
            if (i < std::max({ VectorNavSensor::binaryGroupCommon.size(), VectorNavSensor::binaryGroupTime.size(), VectorNavSensor::binaryGroupIMU.size(),
                               VectorNavSensor::binaryGroupGNSS.size(), VectorNavSensor::binaryGroupAttitude.size(), VectorNavSensor::binaryGroupINS.size() }))
            {
                ImGui::TableNextRow();
            }
            if (i < VectorNavSensor::binaryGroupTime.size())
            {
                const auto& binaryGroupItem = VectorNavSensor::binaryGroupTime.at(i);
                TextColored(0, binaryGroupItem.name, binaryOutputRegister.timeField & binaryGroupItem.flagsValue);
                if (ImGui::IsItemHovered() && binaryGroupItem.tooltip != nullptr)
                {
                    ImGui::BeginTooltip();
                    binaryGroupItem.tooltip();
                    ImGui::EndTooltip();
                }
            }
            if (i < VectorNavSensor::binaryGroupIMU.size())
            {
                const auto& binaryGroupItem = VectorNavSensor::binaryGroupIMU.at(i);
                TextColored(1, binaryGroupItem.name, binaryOutputRegister.imuField & binaryGroupItem.flagsValue);
                if (ImGui::IsItemHovered() && binaryGroupItem.tooltip != nullptr)
                {
                    ImGui::BeginTooltip();
                    binaryGroupItem.tooltip();
                    ImGui::EndTooltip();
                }
            }
            if (i < VectorNavSensor::binaryGroupGNSS.size())
            {
                const auto& binaryGroupItem = VectorNavSensor::binaryGroupGNSS.at(i);
                TextColored(2, binaryGroupItem.name, binaryOutputRegister.gpsField & binaryGroupItem.flagsValue);
                if (ImGui::IsItemHovered() && binaryGroupItem.tooltip != nullptr)
                {
                    ImGui::BeginTooltip();
                    binaryGroupItem.tooltip();
                    ImGui::EndTooltip();
                }
            }
            if (i < VectorNavSensor::binaryGroupAttitude.size())
            {
                const auto& binaryGroupItem = VectorNavSensor::binaryGroupAttitude.at(i);
                TextColored(3, binaryGroupItem.name, binaryOutputRegister.attitudeField & binaryGroupItem.flagsValue);
                if (ImGui::IsItemHovered() && binaryGroupItem.tooltip != nullptr)
                {
                    ImGui::BeginTooltip();
                    binaryGroupItem.tooltip();
                    ImGui::EndTooltip();
                }
            }
            if (i < VectorNavSensor::binaryGroupINS.size())
            {
                const auto& binaryGroupItem = VectorNavSensor::binaryGroupINS.at(i);
                TextColored(4, binaryGroupItem.name, binaryOutputRegister.insField & binaryGroupItem.flagsValue);
                if (ImGui::IsItemHovered() && binaryGroupItem.tooltip != nullptr)
                {
                    ImGui::BeginTooltip();
                    binaryGroupItem.tooltip();
                    ImGui::EndTooltip();
                }
            }
            if (i < VectorNavSensor::binaryGroupGNSS.size())
            {
                const auto& binaryGroupItem = VectorNavSensor::binaryGroupGNSS.at(i);
                TextColored(5, binaryGroupItem.name, binaryOutputRegister.gps2Field & binaryGroupItem.flagsValue);
                if (ImGui::IsItemHovered() && binaryGroupItem.tooltip != nullptr)
                {
                    ImGui::BeginTooltip();
                    binaryGroupItem.tooltip();
                    ImGui::EndTooltip();
                }
            }
        }

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

    messageCount = 0;

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

NAV::FileReader::FileType NAV::VectorNavFile::determineFileType()
{
    LOG_TRACE("called");

    std::string filepath = path;
    if (!path.starts_with('/') && !path.starts_with('~'))
    {
        filepath = flow::GetProgramRootPath() + '/' + path;
    }

    auto filestreamHeader = std::ifstream(filepath);
    if (filestream.good())
    {
        std::array<char, std::string_view("GpsCycle").length()> buffer{};
        filestreamHeader.read(buffer.data(), buffer.size());
        filestreamHeader.close();

        if (std::string(buffer.data(), buffer.size()) == "GpsCycle")
        {
            return FileType::CSV;
        }

        return FileType::BINARY;
    }

    LOG_ERROR("Could not open file {}", filepath);
    return FileType::NONE;
}

void NAV::VectorNavFile::readHeader()
{
    if (fileType == FileType::CSV)
    {
        binaryOutputRegister.timeField = vn::protocol::uart::TimeGroup::TIMEGROUP_NONE;
        binaryOutputRegister.imuField = vn::protocol::uart::ImuGroup::IMUGROUP_NONE;
        binaryOutputRegister.gpsField = vn::protocol::uart::GpsGroup::GPSGROUP_NONE;
        binaryOutputRegister.attitudeField = vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_NONE;
        binaryOutputRegister.insField = vn::protocol::uart::InsGroup::INSGROUP_NONE;
        binaryOutputRegister.gps2Field = vn::protocol::uart::GpsGroup::GPSGROUP_NONE;

        // Read header line
        std::string line;
        std::getline(filestream, line);
        // Remove any starting non text characters
        line.erase(line.begin(), std::find_if(line.begin(), line.end(), [](int ch) { return std::isalnum(ch); }));
        // Convert line into stream
        std::stringstream lineStream(line);
        std::string cell;

        int column = 0;
        // Split line at comma
        while (std::getline(lineStream, cell, ','))
        {
            if (column++ > 2)
            {
                // Remove any trailing non text characters
                cell.erase(std::find_if(cell.begin(), cell.end(), [](int ch) { return std::iscntrl(ch); }), cell.end());

                std::string group = cell.substr(0, cell.find("::"));

                cell = cell.substr(cell.find("::") + 2);
                if (cell.find("::") != std::string::npos)
                {
                    cell = cell.substr(0, cell.find("::"));
                }

                bool identified = false;
                if (group == "Time")
                {
                    for (const auto& binaryGroupItem : VectorNavSensor::binaryGroupTime)
                    {
                        if (cell == binaryGroupItem.name)
                        {
                            binaryOutputRegister.timeField |= static_cast<vn::protocol::uart::TimeGroup>(binaryGroupItem.flagsValue);
                            identified = true;
                            break;
                        }
                    }
                }
                else if (group == "IMU")
                {
                    for (const auto& binaryGroupItem : VectorNavSensor::binaryGroupIMU)
                    {
                        if (cell == binaryGroupItem.name)
                        {
                            binaryOutputRegister.imuField |= static_cast<vn::protocol::uart::ImuGroup>(binaryGroupItem.flagsValue);
                            identified = true;
                            break;
                        }
                        if (cell == "DeltaTime")
                        {
                            binaryOutputRegister.imuField |= vn::protocol::uart::ImuGroup::IMUGROUP_DELTATHETA;
                            identified = true;
                            break;
                        }
                    }
                }
                else if (group == "GNSS1")
                {
                    for (const auto& binaryGroupItem : VectorNavSensor::binaryGroupGNSS)
                    {
                        if (cell == binaryGroupItem.name)
                        {
                            binaryOutputRegister.gpsField |= static_cast<vn::protocol::uart::GpsGroup>(binaryGroupItem.flagsValue);
                            identified = true;
                            break;
                        }
                    }
                }
                else if (group == "Att")
                {
                    for (const auto& binaryGroupItem : VectorNavSensor::binaryGroupAttitude)
                    {
                        if (cell == binaryGroupItem.name)
                        {
                            binaryOutputRegister.attitudeField |= static_cast<vn::protocol::uart::AttitudeGroup>(binaryGroupItem.flagsValue);
                            identified = true;
                            break;
                        }
                    }
                }
                else if (group == "INS")
                {
                    for (const auto& binaryGroupItem : VectorNavSensor::binaryGroupINS)
                    {
                        if (cell == binaryGroupItem.name)
                        {
                            binaryOutputRegister.insField |= static_cast<vn::protocol::uart::InsGroup>(binaryGroupItem.flagsValue);
                            identified = true;
                            break;
                        }
                    }
                }
                else if (group == "GNSS2")
                {
                    for (const auto& binaryGroupItem : VectorNavSensor::binaryGroupGNSS)
                    {
                        if (cell == binaryGroupItem.name)
                        {
                            binaryOutputRegister.gps2Field |= static_cast<vn::protocol::uart::GpsGroup>(binaryGroupItem.flagsValue);
                            identified = true;
                            break;
                        }
                    }
                }
                else
                {
                    LOG_ERROR("{}: Could not identify the group in CSV header - {}::{}", nameId(), group, cell);
                    deinitializeNode();
                    break;
                }

                if (!identified)
                {
                    LOG_ERROR("{}: Could not identify the field in CSV header - {}::{}", nameId(), group, cell);
                    deinitializeNode();
                    break;
                }
            }
        }
    }
    else // if (fileType == FileType::BINARY)
    {
        filestream.read(reinterpret_cast<char*>(&binaryOutputRegister.timeField), sizeof(vn::protocol::uart::TimeGroup));
        filestream.read(reinterpret_cast<char*>(&binaryOutputRegister.imuField), sizeof(vn::protocol::uart::ImuGroup));
        filestream.read(reinterpret_cast<char*>(&binaryOutputRegister.gpsField), sizeof(vn::protocol::uart::GpsGroup));
        filestream.read(reinterpret_cast<char*>(&binaryOutputRegister.attitudeField), sizeof(vn::protocol::uart::AttitudeGroup));
        filestream.read(reinterpret_cast<char*>(&binaryOutputRegister.insField), sizeof(vn::protocol::uart::InsGroup));
        filestream.read(reinterpret_cast<char*>(&binaryOutputRegister.gps2Field), sizeof(vn::protocol::uart::GpsGroup));
    }
}

std::shared_ptr<NAV::NodeData> NAV::VectorNavFile::pollData(bool peek)
{
    auto obs = std::make_shared<VectorNavBinaryOutput>();

    // Get current position
    auto len = filestream.tellg();

    if (fileType == FileType::CSV)
    {
        // Read line
        std::string line;
        std::getline(filestream, line);
        // Remove any starting non text characters
        line.erase(line.begin(), std::find_if(line.begin(), line.end(), [](int ch) { return std::isgraph(ch); }));

        if (line.empty())
        {
            LOG_DEBUG("{}: End of file reached after {} lines", nameId(), messageCount);
            return nullptr;
        }

        // Convert line into stream
        std::stringstream lineStream(line);

        auto extractCell = [&lineStream]() {
            if (std::string cell; std::getline(lineStream, cell, ','))
            {
                // Remove any trailing non text characters
                cell.erase(std::find_if(cell.begin(), cell.end(), [](int ch) { return std::iscntrl(ch); }), cell.end());

                if (!cell.empty())
                {
                    return cell;
                }
            }
            throw std::runtime_error("Cell is empty");
            return std::string("");
        };

        try
        {
            int32_t gpsCycle = std::stoi(extractCell());
            int32_t gpsWeek = std::stoi(extractCell());
            long double tow = std::stold(extractCell());
            obs->insTime = InsTime(gpsCycle, gpsWeek, tow);

            // Group 2 (Time)
            if (binaryOutputRegister.timeField != vn::protocol::uart::TimeGroup::TIMEGROUP_NONE)
            {
                if (!obs->timeOutputs)
                {
                    obs->timeOutputs = std::make_shared<NAV::sensors::vectornav::TimeOutputs>();
                    obs->timeOutputs->timeField |= binaryOutputRegister.timeField;
                }

                if (binaryOutputRegister.timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESTARTUP)
                {
                    obs->timeOutputs->timeStartup = std::stoul(extractCell());
                }
                if (binaryOutputRegister.timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEGPS)
                {
                    obs->timeOutputs->timeGps = std::stoul(extractCell());
                }
                if (binaryOutputRegister.timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_GPSTOW)
                {
                    obs->timeOutputs->gpsTow = std::stoul(extractCell());
                }
                if (binaryOutputRegister.timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_GPSWEEK)
                {
                    obs->timeOutputs->gpsWeek = static_cast<uint16_t>(std::stoul(extractCell()));
                }
                if (binaryOutputRegister.timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESYNCIN)
                {
                    obs->timeOutputs->timeSyncIn = std::stoul(extractCell());
                }
                if (binaryOutputRegister.timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEGPSPPS)
                {
                    obs->timeOutputs->timePPS = std::stoul(extractCell());
                }
                if (binaryOutputRegister.timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEUTC)
                {
                    obs->timeOutputs->timeUtc.year = static_cast<int8_t>(std::stoi(extractCell()));
                    obs->timeOutputs->timeUtc.month = static_cast<uint8_t>(std::stoul(extractCell()));
                    obs->timeOutputs->timeUtc.day = static_cast<uint8_t>(std::stoul(extractCell()));
                    obs->timeOutputs->timeUtc.hour = static_cast<uint8_t>(std::stoul(extractCell()));
                    obs->timeOutputs->timeUtc.min = static_cast<uint8_t>(std::stoul(extractCell()));
                    obs->timeOutputs->timeUtc.sec = static_cast<uint8_t>(std::stoul(extractCell()));
                    obs->timeOutputs->timeUtc.ms = static_cast<uint16_t>(std::stoul(extractCell()));
                }
                if (binaryOutputRegister.timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_SYNCINCNT)
                {
                    obs->timeOutputs->syncInCnt = static_cast<uint32_t>(std::stoul(extractCell()));
                }
                if (binaryOutputRegister.timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_SYNCOUTCNT)
                {
                    obs->timeOutputs->syncOutCnt = static_cast<uint32_t>(std::stoul(extractCell()));
                }
                if (binaryOutputRegister.timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESTATUS)
                {
                    auto timeOk = static_cast<uint8_t>(std::stoul(extractCell()));
                    auto dateOk = static_cast<uint8_t>(std::stoul(extractCell()));
                    auto utcTimeValid = static_cast<uint8_t>(std::stoul(extractCell()));
                    obs->timeOutputs->timeStatus = static_cast<uint8_t>(timeOk << 0U | dateOk << 1U | utcTimeValid << 2U);
                }
            }
            // Group 3 (IMU)
            if (binaryOutputRegister.imuField != vn::protocol::uart::ImuGroup::IMUGROUP_NONE)
            {
                if (!obs->imuOutputs)
                {
                    obs->imuOutputs = std::make_shared<NAV::sensors::vectornav::ImuOutputs>();
                    obs->imuOutputs->imuField |= binaryOutputRegister.imuField;
                }

                if (binaryOutputRegister.imuField & vn::protocol::uart::ImuGroup::IMUGROUP_IMUSTATUS)
                {
                    obs->imuOutputs->imuStatus = static_cast<uint16_t>(std::stoul(extractCell()));
                }
                if (binaryOutputRegister.imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPMAG)
                {
                    float vecX = std::stof(extractCell());
                    float vecY = std::stof(extractCell());
                    float vecZ = std::stof(extractCell());
                    obs->imuOutputs->uncompMag = { vecX, vecY, vecZ };
                }
                if (binaryOutputRegister.imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPACCEL)
                {
                    float vecX = std::stof(extractCell());
                    float vecY = std::stof(extractCell());
                    float vecZ = std::stof(extractCell());
                    obs->imuOutputs->uncompAccel = { vecX, vecY, vecZ };
                }
                if (binaryOutputRegister.imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPGYRO)
                {
                    float vecX = std::stof(extractCell());
                    float vecY = std::stof(extractCell());
                    float vecZ = std::stof(extractCell());
                    obs->imuOutputs->uncompGyro = { vecX, vecY, vecZ };
                }
                if (binaryOutputRegister.imuField & vn::protocol::uart::ImuGroup::IMUGROUP_TEMP)
                {
                    obs->imuOutputs->temp = std::stof(extractCell());
                }
                if (binaryOutputRegister.imuField & vn::protocol::uart::ImuGroup::IMUGROUP_PRES)
                {
                    obs->imuOutputs->pres = std::stof(extractCell());
                }
                if (binaryOutputRegister.imuField & vn::protocol::uart::ImuGroup::IMUGROUP_DELTATHETA)
                {
                    obs->imuOutputs->deltaTime = std::stof(extractCell());
                    float vecX = std::stof(extractCell());
                    float vecY = std::stof(extractCell());
                    float vecZ = std::stof(extractCell());
                    obs->imuOutputs->deltaTheta = { vecX, vecY, vecZ };
                }
                if (binaryOutputRegister.imuField & vn::protocol::uart::ImuGroup::IMUGROUP_DELTAVEL)
                {
                    float vecX = std::stof(extractCell());
                    float vecY = std::stof(extractCell());
                    float vecZ = std::stof(extractCell());
                    obs->imuOutputs->deltaV = { vecX, vecY, vecZ };
                }
                if (binaryOutputRegister.imuField & vn::protocol::uart::ImuGroup::IMUGROUP_MAG)
                {
                    float vecX = std::stof(extractCell());
                    float vecY = std::stof(extractCell());
                    float vecZ = std::stof(extractCell());
                    obs->imuOutputs->mag = { vecX, vecY, vecZ };
                }
                if (binaryOutputRegister.imuField & vn::protocol::uart::ImuGroup::IMUGROUP_ACCEL)
                {
                    float vecX = std::stof(extractCell());
                    float vecY = std::stof(extractCell());
                    float vecZ = std::stof(extractCell());
                    obs->imuOutputs->accel = { vecX, vecY, vecZ };
                }
                if (binaryOutputRegister.imuField & vn::protocol::uart::ImuGroup::IMUGROUP_ANGULARRATE)
                {
                    float vecX = std::stof(extractCell());
                    float vecY = std::stof(extractCell());
                    float vecZ = std::stof(extractCell());
                    obs->imuOutputs->angularRate = { vecX, vecY, vecZ };
                }
            }
            // Group 4 (GNSS1)
            if (binaryOutputRegister.gpsField != vn::protocol::uart::GpsGroup::GPSGROUP_NONE)
            {
                if (!obs->gnss1Outputs)
                {
                    obs->gnss1Outputs = std::make_shared<NAV::sensors::vectornav::GnssOutputs>();
                    obs->gnss1Outputs->gnssField |= binaryOutputRegister.gpsField;
                }

                if (binaryOutputRegister.gpsField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC)
                {
                    obs->gnss1Outputs->timeUtc.year = static_cast<int8_t>(std::stoi(extractCell()));
                    obs->gnss1Outputs->timeUtc.month = static_cast<uint8_t>(std::stoul(extractCell()));
                    obs->gnss1Outputs->timeUtc.day = static_cast<uint8_t>(std::stoul(extractCell()));
                    obs->gnss1Outputs->timeUtc.hour = static_cast<uint8_t>(std::stoul(extractCell()));
                    obs->gnss1Outputs->timeUtc.min = static_cast<uint8_t>(std::stoul(extractCell()));
                    obs->gnss1Outputs->timeUtc.sec = static_cast<uint8_t>(std::stoul(extractCell()));
                    obs->gnss1Outputs->timeUtc.ms = static_cast<uint16_t>(std::stoul(extractCell()));
                }
                if (binaryOutputRegister.gpsField & vn::protocol::uart::GpsGroup::GPSGROUP_TOW)
                {
                    obs->gnss1Outputs->tow = std::stoul(extractCell());
                }
                if (binaryOutputRegister.gpsField & vn::protocol::uart::GpsGroup::GPSGROUP_WEEK)
                {
                    obs->gnss1Outputs->week = static_cast<uint16_t>(std::stoul(extractCell()));
                }
                if (binaryOutputRegister.gpsField & vn::protocol::uart::GpsGroup::GPSGROUP_NUMSATS)
                {
                    obs->gnss1Outputs->numSats = static_cast<uint8_t>(std::stoul(extractCell()));
                }
                if (binaryOutputRegister.gpsField & vn::protocol::uart::GpsGroup::GPSGROUP_FIX)
                {
                    obs->gnss1Outputs->fix = static_cast<uint8_t>(std::stoul(extractCell()));
                }
                if (binaryOutputRegister.gpsField & vn::protocol::uart::GpsGroup::GPSGROUP_POSLLA)
                {
                    double vecX = std::stod(extractCell());
                    double vecY = std::stod(extractCell());
                    double vecZ = std::stod(extractCell());
                    obs->gnss1Outputs->posLla = { vecX, vecY, vecZ };
                }
                if (binaryOutputRegister.gpsField & vn::protocol::uart::GpsGroup::GPSGROUP_POSECEF)
                {
                    double vecX = std::stod(extractCell());
                    double vecY = std::stod(extractCell());
                    double vecZ = std::stod(extractCell());
                    obs->gnss1Outputs->posEcef = { vecX, vecY, vecZ };
                }
                if (binaryOutputRegister.gpsField & vn::protocol::uart::GpsGroup::GPSGROUP_VELNED)
                {
                    float vecX = std::stof(extractCell());
                    float vecY = std::stof(extractCell());
                    float vecZ = std::stof(extractCell());
                    obs->gnss1Outputs->velNed = { vecX, vecY, vecZ };
                }
                if (binaryOutputRegister.gpsField & vn::protocol::uart::GpsGroup::GPSGROUP_VELECEF)
                {
                    float vecX = std::stof(extractCell());
                    float vecY = std::stof(extractCell());
                    float vecZ = std::stof(extractCell());
                    obs->gnss1Outputs->velEcef = { vecX, vecY, vecZ };
                }
                if (binaryOutputRegister.gpsField & vn::protocol::uart::GpsGroup::GPSGROUP_POSU)
                {
                    float vecX = std::stof(extractCell());
                    float vecY = std::stof(extractCell());
                    float vecZ = std::stof(extractCell());
                    obs->gnss1Outputs->posU = { vecX, vecY, vecZ };
                }
                if (binaryOutputRegister.gpsField & vn::protocol::uart::GpsGroup::GPSGROUP_VELU)
                {
                    obs->gnss1Outputs->velU = std::stof(extractCell());
                }
                if (binaryOutputRegister.gpsField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEU)
                {
                    obs->gnss1Outputs->timeU = std::stof(extractCell());
                }
                if (binaryOutputRegister.gpsField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO)
                {
                    obs->gnss1Outputs->timeInfo.status = static_cast<uint8_t>(std::stoul(extractCell()));
                    obs->gnss1Outputs->timeInfo.leapSeconds = static_cast<int8_t>(std::stoi(extractCell()));
                }
                if (binaryOutputRegister.gpsField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP)
                {
                    obs->gnss1Outputs->dop.gDop = std::stof(extractCell());
                    obs->gnss1Outputs->dop.pDop = std::stof(extractCell());
                    obs->gnss1Outputs->dop.tDop = std::stof(extractCell());
                    obs->gnss1Outputs->dop.vDop = std::stof(extractCell());
                    obs->gnss1Outputs->dop.hDop = std::stof(extractCell());
                    obs->gnss1Outputs->dop.nDop = std::stof(extractCell());
                    obs->gnss1Outputs->dop.eDop = std::stof(extractCell());
                }
                if (binaryOutputRegister.gpsField & vn::protocol::uart::GpsGroup::GPSGROUP_SATINFO)
                {
                    obs->gnss1Outputs->satInfo.numSats = static_cast<uint8_t>(std::stoul(extractCell()));
                    for (size_t i = 0; i < obs->gnss1Outputs->satInfo.numSats; i++)
                    {
                        auto sys = static_cast<int8_t>(std::stoi(extractCell()));
                        auto svId = static_cast<uint8_t>(std::stoul(extractCell()));
                        auto flags = static_cast<uint8_t>(std::stoul(extractCell()));
                        auto cno = static_cast<uint8_t>(std::stoul(extractCell()));
                        auto qi = static_cast<uint8_t>(std::stoul(extractCell()));
                        auto el = static_cast<int8_t>(std::stoi(extractCell()));
                        auto az = static_cast<int16_t>(std::stoi(extractCell()));
                        obs->gnss1Outputs->satInfo.satellites.emplace_back(sys, svId, flags, cno, qi, el, az);
                    }
                }
                if (binaryOutputRegister.gpsField & vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS)
                {
                    obs->gnss1Outputs->raw.tow = std::stod(extractCell());
                    obs->gnss1Outputs->raw.week = static_cast<uint16_t>(std::stoul(extractCell()));
                    obs->gnss1Outputs->raw.numSats = static_cast<uint8_t>(std::stoul(extractCell()));
                    for (size_t i = 0; i < obs->gnss1Outputs->raw.numSats; i++)
                    {
                        auto sys = static_cast<uint8_t>(std::stoul(extractCell()));
                        auto svId = static_cast<uint8_t>(std::stoul(extractCell()));
                        auto freq = static_cast<uint8_t>(std::stoul(extractCell()));
                        auto chan = static_cast<uint8_t>(std::stoul(extractCell()));
                        auto slot = static_cast<int8_t>(std::stoi(extractCell()));
                        auto cno = static_cast<uint8_t>(std::stoul(extractCell()));
                        auto flags = static_cast<uint16_t>(std::stoul(extractCell()));
                        auto pr = std::stod(extractCell());
                        auto cp = std::stod(extractCell());
                        auto dp = std::stof(extractCell());
                        obs->gnss1Outputs->raw.satellites.emplace_back(sys, svId, freq, chan, slot, cno, flags, pr, cp, dp);
                    }
                }
            }
            // Group 5 (Attitude)
            if (binaryOutputRegister.attitudeField != vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_NONE)
            {
                if (!obs->attitudeOutputs)
                {
                    obs->attitudeOutputs = std::make_shared<NAV::sensors::vectornav::AttitudeOutputs>();
                    obs->attitudeOutputs->attitudeField |= binaryOutputRegister.attitudeField;
                }

                if (binaryOutputRegister.attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_VPESTATUS)
                {
                    obs->attitudeOutputs->vpeStatus = static_cast<uint16_t>(std::stoul(extractCell()));
                }
                if (binaryOutputRegister.attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YAWPITCHROLL)
                {
                    float vecX = std::stof(extractCell());
                    float vecY = std::stof(extractCell());
                    float vecZ = std::stof(extractCell());
                    obs->attitudeOutputs->ypr = { vecX, vecY, vecZ };
                }
                if (binaryOutputRegister.attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_QUATERNION)
                {
                    float vecW = std::stof(extractCell());
                    float vecX = std::stof(extractCell());
                    float vecY = std::stof(extractCell());
                    float vecZ = std::stof(extractCell());
                    obs->attitudeOutputs->qtn = { vecW, vecX, vecY, vecZ };
                }
                if (binaryOutputRegister.attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_DCM)
                {
                    float mat00 = std::stof(extractCell());
                    float mat01 = std::stof(extractCell());
                    float mat02 = std::stof(extractCell());
                    float mat10 = std::stof(extractCell());
                    float mat11 = std::stof(extractCell());
                    float mat12 = std::stof(extractCell());
                    float mat20 = std::stof(extractCell());
                    float mat21 = std::stof(extractCell());
                    float mat22 = std::stof(extractCell());
                    obs->attitudeOutputs->dcm << mat00, mat01, mat02,
                        mat10, mat11, mat12,
                        mat20, mat21, mat22;
                }
                if (binaryOutputRegister.attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_MAGNED)
                {
                    float vecX = std::stof(extractCell());
                    float vecY = std::stof(extractCell());
                    float vecZ = std::stof(extractCell());
                    obs->attitudeOutputs->magNed = { vecX, vecY, vecZ };
                }
                if (binaryOutputRegister.attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_ACCELNED)
                {
                    float vecX = std::stof(extractCell());
                    float vecY = std::stof(extractCell());
                    float vecZ = std::stof(extractCell());
                    obs->attitudeOutputs->accelNed = { vecX, vecY, vecZ };
                }
                if (binaryOutputRegister.attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_LINEARACCELBODY)
                {
                    float vecX = std::stof(extractCell());
                    float vecY = std::stof(extractCell());
                    float vecZ = std::stof(extractCell());
                    obs->attitudeOutputs->linearAccelBody = { vecX, vecY, vecZ };
                }
                if (binaryOutputRegister.attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_LINEARACCELNED)
                {
                    float vecX = std::stof(extractCell());
                    float vecY = std::stof(extractCell());
                    float vecZ = std::stof(extractCell());
                    obs->attitudeOutputs->linearAccelNed = { vecX, vecY, vecZ };
                }
                if (binaryOutputRegister.attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YPRU)
                {
                    float vecX = std::stof(extractCell());
                    float vecY = std::stof(extractCell());
                    float vecZ = std::stof(extractCell());
                    obs->attitudeOutputs->yprU = { vecX, vecY, vecZ };
                }
            }
            // Group 6 (INS)
            if (binaryOutputRegister.insField != vn::protocol::uart::InsGroup::INSGROUP_NONE)
            {
                if (!obs->insOutputs)
                {
                    obs->insOutputs = std::make_shared<NAV::sensors::vectornav::InsOutputs>();
                    obs->insOutputs->insField |= binaryOutputRegister.insField;
                }

                if (binaryOutputRegister.insField & vn::protocol::uart::InsGroup::INSGROUP_INSSTATUS)
                {
                    obs->insOutputs->insStatus = static_cast<uint16_t>(std::stoul(extractCell()));
                }
                if (binaryOutputRegister.insField & vn::protocol::uart::InsGroup::INSGROUP_POSLLA)
                {
                    double vecX = std::stod(extractCell());
                    double vecY = std::stod(extractCell());
                    double vecZ = std::stod(extractCell());
                    obs->insOutputs->posLla = { vecX, vecY, vecZ };
                }
                if (binaryOutputRegister.insField & vn::protocol::uart::InsGroup::INSGROUP_POSECEF)
                {
                    double vecX = std::stod(extractCell());
                    double vecY = std::stod(extractCell());
                    double vecZ = std::stod(extractCell());
                    obs->insOutputs->posEcef = { vecX, vecY, vecZ };
                }
                if (binaryOutputRegister.insField & vn::protocol::uart::InsGroup::INSGROUP_VELBODY)
                {
                    float vecX = std::stof(extractCell());
                    float vecY = std::stof(extractCell());
                    float vecZ = std::stof(extractCell());
                    obs->insOutputs->velBody = { vecX, vecY, vecZ };
                }
                if (binaryOutputRegister.insField & vn::protocol::uart::InsGroup::INSGROUP_VELNED)
                {
                    float vecX = std::stof(extractCell());
                    float vecY = std::stof(extractCell());
                    float vecZ = std::stof(extractCell());
                    obs->insOutputs->velNed = { vecX, vecY, vecZ };
                }
                if (binaryOutputRegister.insField & vn::protocol::uart::InsGroup::INSGROUP_VELECEF)
                {
                    float vecX = std::stof(extractCell());
                    float vecY = std::stof(extractCell());
                    float vecZ = std::stof(extractCell());
                    obs->insOutputs->velEcef = { vecX, vecY, vecZ };
                }
                if (binaryOutputRegister.insField & vn::protocol::uart::InsGroup::INSGROUP_MAGECEF)
                {
                    float vecX = std::stof(extractCell());
                    float vecY = std::stof(extractCell());
                    float vecZ = std::stof(extractCell());
                    obs->insOutputs->magEcef = { vecX, vecY, vecZ };
                }
                if (binaryOutputRegister.insField & vn::protocol::uart::InsGroup::INSGROUP_ACCELECEF)
                {
                    float vecX = std::stof(extractCell());
                    float vecY = std::stof(extractCell());
                    float vecZ = std::stof(extractCell());
                    obs->insOutputs->accelEcef = { vecX, vecY, vecZ };
                }
                if (binaryOutputRegister.insField & vn::protocol::uart::InsGroup::INSGROUP_LINEARACCELECEF)
                {
                    float vecX = std::stof(extractCell());
                    float vecY = std::stof(extractCell());
                    float vecZ = std::stof(extractCell());
                    obs->insOutputs->linearAccelEcef = { vecX, vecY, vecZ };
                }
                if (binaryOutputRegister.insField & vn::protocol::uart::InsGroup::INSGROUP_POSU)
                {
                    obs->insOutputs->posU = std::stof(extractCell());
                }
                if (binaryOutputRegister.insField & vn::protocol::uart::InsGroup::INSGROUP_VELU)
                {
                    obs->insOutputs->velU = std::stof(extractCell());
                }
            }
            // Group 7 (GNSS2)
            if (binaryOutputRegister.gps2Field != vn::protocol::uart::GpsGroup::GPSGROUP_NONE)
            {
                if (!obs->gnss2Outputs)
                {
                    obs->gnss2Outputs = std::make_shared<NAV::sensors::vectornav::GnssOutputs>();
                    obs->gnss2Outputs->gnssField |= binaryOutputRegister.gps2Field;
                }

                if (binaryOutputRegister.gps2Field & vn::protocol::uart::GpsGroup::GPSGROUP_UTC)
                {
                    obs->gnss2Outputs->timeUtc.year = static_cast<int8_t>(std::stoi(extractCell()));
                    obs->gnss2Outputs->timeUtc.month = static_cast<uint8_t>(std::stoul(extractCell()));
                    obs->gnss2Outputs->timeUtc.day = static_cast<uint8_t>(std::stoul(extractCell()));
                    obs->gnss2Outputs->timeUtc.hour = static_cast<uint8_t>(std::stoul(extractCell()));
                    obs->gnss2Outputs->timeUtc.min = static_cast<uint8_t>(std::stoul(extractCell()));
                    obs->gnss2Outputs->timeUtc.sec = static_cast<uint8_t>(std::stoul(extractCell()));
                    obs->gnss2Outputs->timeUtc.ms = static_cast<uint16_t>(std::stoul(extractCell()));
                }
                if (binaryOutputRegister.gps2Field & vn::protocol::uart::GpsGroup::GPSGROUP_TOW)
                {
                    obs->gnss2Outputs->tow = std::stoul(extractCell());
                }
                if (binaryOutputRegister.gps2Field & vn::protocol::uart::GpsGroup::GPSGROUP_WEEK)
                {
                    obs->gnss2Outputs->week = static_cast<uint16_t>(std::stoul(extractCell()));
                }
                if (binaryOutputRegister.gps2Field & vn::protocol::uart::GpsGroup::GPSGROUP_NUMSATS)
                {
                    obs->gnss2Outputs->numSats = static_cast<uint8_t>(std::stoul(extractCell()));
                }
                if (binaryOutputRegister.gps2Field & vn::protocol::uart::GpsGroup::GPSGROUP_FIX)
                {
                    obs->gnss2Outputs->fix = static_cast<uint8_t>(std::stoul(extractCell()));
                }
                if (binaryOutputRegister.gps2Field & vn::protocol::uart::GpsGroup::GPSGROUP_POSLLA)
                {
                    double vecX = std::stod(extractCell());
                    double vecY = std::stod(extractCell());
                    double vecZ = std::stod(extractCell());
                    obs->gnss2Outputs->posLla = { vecX, vecY, vecZ };
                }
                if (binaryOutputRegister.gps2Field & vn::protocol::uart::GpsGroup::GPSGROUP_POSECEF)
                {
                    double vecX = std::stod(extractCell());
                    double vecY = std::stod(extractCell());
                    double vecZ = std::stod(extractCell());
                    obs->gnss2Outputs->posEcef = { vecX, vecY, vecZ };
                }
                if (binaryOutputRegister.gps2Field & vn::protocol::uart::GpsGroup::GPSGROUP_VELNED)
                {
                    float vecX = std::stof(extractCell());
                    float vecY = std::stof(extractCell());
                    float vecZ = std::stof(extractCell());
                    obs->gnss2Outputs->velNed = { vecX, vecY, vecZ };
                }
                if (binaryOutputRegister.gps2Field & vn::protocol::uart::GpsGroup::GPSGROUP_VELECEF)
                {
                    float vecX = std::stof(extractCell());
                    float vecY = std::stof(extractCell());
                    float vecZ = std::stof(extractCell());
                    obs->gnss2Outputs->velEcef = { vecX, vecY, vecZ };
                }
                if (binaryOutputRegister.gps2Field & vn::protocol::uart::GpsGroup::GPSGROUP_POSU)
                {
                    float vecX = std::stof(extractCell());
                    float vecY = std::stof(extractCell());
                    float vecZ = std::stof(extractCell());
                    obs->gnss2Outputs->posU = { vecX, vecY, vecZ };
                }
                if (binaryOutputRegister.gps2Field & vn::protocol::uart::GpsGroup::GPSGROUP_VELU)
                {
                    obs->gnss2Outputs->velU = std::stof(extractCell());
                }
                if (binaryOutputRegister.gps2Field & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEU)
                {
                    obs->gnss2Outputs->timeU = std::stof(extractCell());
                }
                if (binaryOutputRegister.gps2Field & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO)
                {
                    obs->gnss2Outputs->timeInfo.status = static_cast<uint8_t>(std::stoul(extractCell()));
                    obs->gnss2Outputs->timeInfo.leapSeconds = static_cast<int8_t>(std::stoi(extractCell()));
                }
                if (binaryOutputRegister.gps2Field & vn::protocol::uart::GpsGroup::GPSGROUP_DOP)
                {
                    obs->gnss2Outputs->dop.gDop = std::stof(extractCell());
                    obs->gnss2Outputs->dop.pDop = std::stof(extractCell());
                    obs->gnss2Outputs->dop.tDop = std::stof(extractCell());
                    obs->gnss2Outputs->dop.vDop = std::stof(extractCell());
                    obs->gnss2Outputs->dop.hDop = std::stof(extractCell());
                    obs->gnss2Outputs->dop.nDop = std::stof(extractCell());
                    obs->gnss2Outputs->dop.eDop = std::stof(extractCell());
                }
                if (binaryOutputRegister.gps2Field & vn::protocol::uart::GpsGroup::GPSGROUP_SATINFO)
                {
                    obs->gnss2Outputs->satInfo.numSats = static_cast<uint8_t>(std::stoul(extractCell()));
                    for (size_t i = 0; i < obs->gnss2Outputs->satInfo.numSats; i++)
                    {
                        auto sys = static_cast<int8_t>(std::stoi(extractCell()));
                        auto svId = static_cast<uint8_t>(std::stoul(extractCell()));
                        auto flags = static_cast<uint8_t>(std::stoul(extractCell()));
                        auto cno = static_cast<uint8_t>(std::stoul(extractCell()));
                        auto qi = static_cast<uint8_t>(std::stoul(extractCell()));
                        auto el = static_cast<int8_t>(std::stoi(extractCell()));
                        auto az = static_cast<int16_t>(std::stoi(extractCell()));
                        obs->gnss2Outputs->satInfo.satellites.emplace_back(sys, svId, flags, cno, qi, el, az);
                    }
                }
                if (binaryOutputRegister.gps2Field & vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS)
                {
                    obs->gnss2Outputs->raw.tow = std::stod(extractCell());
                    obs->gnss2Outputs->raw.week = static_cast<uint16_t>(std::stoul(extractCell()));
                    obs->gnss2Outputs->raw.numSats = static_cast<uint8_t>(std::stoul(extractCell()));
                    for (size_t i = 0; i < obs->gnss2Outputs->raw.numSats; i++)
                    {
                        auto sys = static_cast<uint8_t>(std::stoul(extractCell()));
                        auto svId = static_cast<uint8_t>(std::stoul(extractCell()));
                        auto freq = static_cast<uint8_t>(std::stoul(extractCell()));
                        auto chan = static_cast<uint8_t>(std::stoul(extractCell()));
                        auto slot = static_cast<int8_t>(std::stoi(extractCell()));
                        auto cno = static_cast<uint8_t>(std::stoul(extractCell()));
                        auto flags = static_cast<uint16_t>(std::stoul(extractCell()));
                        auto pr = std::stod(extractCell());
                        auto cp = std::stod(extractCell());
                        auto dp = std::stof(extractCell());
                        obs->gnss2Outputs->raw.satellites.emplace_back(sys, svId, freq, chan, slot, cno, flags, pr, cp, dp);
                    }
                }
            }
        }
        catch (const std::exception& e)
        {
            LOG_ERROR("{}: Could not read line {} completely: {}", nameId(), messageCount + 2, e.what());
            return nullptr;
        }
    }
    else // if (fileType == FileType::BINARY)
    {
        auto readFromFilestream = [&, this](char* __s, std::streamsize __n) {
            filestream.read(__s, __n);
            if (!filestream.good())
            {
                throw std::runtime_error("End of file reached");
            }
        };

        try
        {
            int32_t gpsCycle = 0;
            int32_t gpsWeek = 0;
            long double tow = 0.0L;
            readFromFilestream(reinterpret_cast<char*>(&gpsCycle), sizeof(gpsCycle));
            readFromFilestream(reinterpret_cast<char*>(&gpsWeek), sizeof(gpsWeek));
            readFromFilestream(reinterpret_cast<char*>(&tow), sizeof(tow));
            obs->insTime = InsTime(gpsCycle, gpsWeek, tow);

            // Group 2 (Time)
            if (binaryOutputRegister.timeField != vn::protocol::uart::TimeGroup::TIMEGROUP_NONE)
            {
                if (!obs->timeOutputs)
                {
                    obs->timeOutputs = std::make_shared<NAV::sensors::vectornav::TimeOutputs>();
                    obs->timeOutputs->timeField |= binaryOutputRegister.timeField;
                }

                if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESTARTUP)
                {
                    filestream.read(reinterpret_cast<char*>(&obs->timeOutputs->timeStartup), sizeof(obs->timeOutputs->timeStartup));
                }
                if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEGPS)
                {
                    filestream.read(reinterpret_cast<char*>(&obs->timeOutputs->timeGps), sizeof(obs->timeOutputs->timeGps));
                }
                if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_GPSTOW)
                {
                    filestream.read(reinterpret_cast<char*>(&obs->timeOutputs->gpsTow), sizeof(obs->timeOutputs->gpsTow));
                }
                if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_GPSWEEK)
                {
                    filestream.read(reinterpret_cast<char*>(&obs->timeOutputs->gpsWeek), sizeof(obs->timeOutputs->gpsWeek));
                }
                if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESYNCIN)
                {
                    filestream.read(reinterpret_cast<char*>(&obs->timeOutputs->timeSyncIn), sizeof(obs->timeOutputs->timeSyncIn));
                }
                if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEGPSPPS)
                {
                    filestream.read(reinterpret_cast<char*>(&obs->timeOutputs->timePPS), sizeof(obs->timeOutputs->timePPS));
                }
                if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEUTC)
                {
                    filestream.read(reinterpret_cast<char*>(&obs->timeOutputs->timeUtc.year), sizeof(obs->timeOutputs->timeUtc.year));
                    filestream.read(reinterpret_cast<char*>(&obs->timeOutputs->timeUtc.month), sizeof(obs->timeOutputs->timeUtc.month));
                    filestream.read(reinterpret_cast<char*>(&obs->timeOutputs->timeUtc.day), sizeof(obs->timeOutputs->timeUtc.day));
                    filestream.read(reinterpret_cast<char*>(&obs->timeOutputs->timeUtc.hour), sizeof(obs->timeOutputs->timeUtc.hour));
                    filestream.read(reinterpret_cast<char*>(&obs->timeOutputs->timeUtc.min), sizeof(obs->timeOutputs->timeUtc.min));
                    filestream.read(reinterpret_cast<char*>(&obs->timeOutputs->timeUtc.sec), sizeof(obs->timeOutputs->timeUtc.sec));
                    filestream.read(reinterpret_cast<char*>(&obs->timeOutputs->timeUtc.ms), sizeof(obs->timeOutputs->timeUtc.ms));
                }
                if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_SYNCINCNT)
                {
                    filestream.read(reinterpret_cast<char*>(&obs->timeOutputs->syncInCnt), sizeof(obs->timeOutputs->syncInCnt));
                }
                if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_SYNCOUTCNT)
                {
                    filestream.read(reinterpret_cast<char*>(&obs->timeOutputs->syncOutCnt), sizeof(obs->timeOutputs->syncOutCnt));
                }
                if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESTATUS)
                {
                    filestream.read(reinterpret_cast<char*>(&obs->timeOutputs->timeStatus.status), sizeof(obs->timeOutputs->timeStatus.status));
                }
            }
            // Group 3 (IMU)
            if (binaryOutputRegister.imuField != vn::protocol::uart::ImuGroup::IMUGROUP_NONE)
            {
                if (!obs->imuOutputs)
                {
                    obs->imuOutputs = std::make_shared<NAV::sensors::vectornav::ImuOutputs>();
                    obs->imuOutputs->imuField |= binaryOutputRegister.imuField;
                }

                if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_IMUSTATUS)
                {
                    filestream.read(reinterpret_cast<char*>(&obs->imuOutputs->imuStatus), sizeof(obs->imuOutputs->imuStatus));
                }
                if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPMAG)
                {
                    filestream.read(reinterpret_cast<char*>(obs->imuOutputs->uncompMag.data()), sizeof(obs->imuOutputs->uncompMag));
                }
                if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPACCEL)
                {
                    filestream.read(reinterpret_cast<char*>(obs->imuOutputs->uncompAccel.data()), sizeof(obs->imuOutputs->uncompAccel));
                }
                if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPGYRO)
                {
                    filestream.read(reinterpret_cast<char*>(obs->imuOutputs->uncompGyro.data()), sizeof(obs->imuOutputs->uncompGyro));
                }
                if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_TEMP)
                {
                    filestream.read(reinterpret_cast<char*>(&obs->imuOutputs->temp), sizeof(obs->imuOutputs->temp));
                }
                if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_PRES)
                {
                    filestream.read(reinterpret_cast<char*>(&obs->imuOutputs->pres), sizeof(obs->imuOutputs->pres));
                }
                if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_DELTATHETA)
                {
                    filestream.read(reinterpret_cast<char*>(&obs->imuOutputs->deltaTime), sizeof(obs->imuOutputs->deltaTime));
                    filestream.read(reinterpret_cast<char*>(obs->imuOutputs->deltaTheta.data()), sizeof(obs->imuOutputs->deltaTheta));
                }
                if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_DELTAVEL)
                {
                    filestream.read(reinterpret_cast<char*>(obs->imuOutputs->deltaV.data()), sizeof(obs->imuOutputs->deltaV));
                }
                if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_MAG)
                {
                    filestream.read(reinterpret_cast<char*>(obs->imuOutputs->mag.data()), sizeof(obs->imuOutputs->mag));
                }
                if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_ACCEL)
                {
                    filestream.read(reinterpret_cast<char*>(obs->imuOutputs->accel.data()), sizeof(obs->imuOutputs->accel));
                }
                if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_ANGULARRATE)
                {
                    filestream.read(reinterpret_cast<char*>(obs->imuOutputs->angularRate.data()), sizeof(obs->imuOutputs->angularRate));
                }
            }
            // Group 4 (GNSS1)
            if (binaryOutputRegister.gpsField != vn::protocol::uart::GpsGroup::GPSGROUP_NONE)
            {
                if (!obs->gnss1Outputs)
                {
                    obs->gnss1Outputs = std::make_shared<NAV::sensors::vectornav::GnssOutputs>();
                    obs->gnss1Outputs->gnssField |= binaryOutputRegister.gpsField;
                }

                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC)
                {
                    filestream.read(reinterpret_cast<char*>(&obs->gnss1Outputs->timeUtc.year), sizeof(obs->gnss1Outputs->timeUtc.year));
                    filestream.read(reinterpret_cast<char*>(&obs->gnss1Outputs->timeUtc.month), sizeof(obs->gnss1Outputs->timeUtc.month));
                    filestream.read(reinterpret_cast<char*>(&obs->gnss1Outputs->timeUtc.day), sizeof(obs->gnss1Outputs->timeUtc.day));
                    filestream.read(reinterpret_cast<char*>(&obs->gnss1Outputs->timeUtc.hour), sizeof(obs->gnss1Outputs->timeUtc.hour));
                    filestream.read(reinterpret_cast<char*>(&obs->gnss1Outputs->timeUtc.min), sizeof(obs->gnss1Outputs->timeUtc.min));
                    filestream.read(reinterpret_cast<char*>(&obs->gnss1Outputs->timeUtc.sec), sizeof(obs->gnss1Outputs->timeUtc.sec));
                    filestream.read(reinterpret_cast<char*>(&obs->gnss1Outputs->timeUtc.ms), sizeof(obs->gnss1Outputs->timeUtc.ms));
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TOW)
                {
                    filestream.read(reinterpret_cast<char*>(&obs->gnss1Outputs->tow), sizeof(obs->gnss1Outputs->tow));
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_WEEK)
                {
                    filestream.read(reinterpret_cast<char*>(&obs->gnss1Outputs->week), sizeof(obs->gnss1Outputs->week));
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_NUMSATS)
                {
                    filestream.read(reinterpret_cast<char*>(&obs->gnss1Outputs->numSats), sizeof(obs->gnss1Outputs->numSats));
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_FIX)
                {
                    filestream.read(reinterpret_cast<char*>(&obs->gnss1Outputs->fix), sizeof(obs->gnss1Outputs->fix));
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSLLA)
                {
                    filestream.read(reinterpret_cast<char*>(obs->gnss1Outputs->posLla.data()), sizeof(obs->gnss1Outputs->posLla));
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSECEF)
                {
                    filestream.read(reinterpret_cast<char*>(obs->gnss1Outputs->posEcef.data()), sizeof(obs->gnss1Outputs->posEcef));
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELNED)
                {
                    filestream.read(reinterpret_cast<char*>(obs->gnss1Outputs->velNed.data()), sizeof(obs->gnss1Outputs->velNed));
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELECEF)
                {
                    filestream.read(reinterpret_cast<char*>(obs->gnss1Outputs->velEcef.data()), sizeof(obs->gnss1Outputs->velEcef));
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSU)
                {
                    filestream.read(reinterpret_cast<char*>(obs->gnss1Outputs->posU.data()), sizeof(obs->gnss1Outputs->posU));
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELU)
                {
                    filestream.read(reinterpret_cast<char*>(&obs->gnss1Outputs->velU), sizeof(obs->gnss1Outputs->velU));
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEU)
                {
                    filestream.read(reinterpret_cast<char*>(&obs->gnss1Outputs->timeU), sizeof(obs->gnss1Outputs->timeU));
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO)
                {
                    filestream.read(reinterpret_cast<char*>(&obs->gnss1Outputs->timeInfo.status.status), sizeof(obs->gnss1Outputs->timeInfo.status.status));
                    filestream.read(reinterpret_cast<char*>(&obs->gnss1Outputs->timeInfo.leapSeconds), sizeof(obs->gnss1Outputs->timeInfo.leapSeconds));
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP)
                {
                    filestream.read(reinterpret_cast<char*>(&obs->gnss1Outputs->dop.gDop), sizeof(obs->gnss1Outputs->dop.gDop));
                    filestream.read(reinterpret_cast<char*>(&obs->gnss1Outputs->dop.pDop), sizeof(obs->gnss1Outputs->dop.pDop));
                    filestream.read(reinterpret_cast<char*>(&obs->gnss1Outputs->dop.tDop), sizeof(obs->gnss1Outputs->dop.tDop));
                    filestream.read(reinterpret_cast<char*>(&obs->gnss1Outputs->dop.vDop), sizeof(obs->gnss1Outputs->dop.vDop));
                    filestream.read(reinterpret_cast<char*>(&obs->gnss1Outputs->dop.hDop), sizeof(obs->gnss1Outputs->dop.hDop));
                    filestream.read(reinterpret_cast<char*>(&obs->gnss1Outputs->dop.nDop), sizeof(obs->gnss1Outputs->dop.nDop));
                    filestream.read(reinterpret_cast<char*>(&obs->gnss1Outputs->dop.eDop), sizeof(obs->gnss1Outputs->dop.eDop));
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_SATINFO)
                {
                    filestream.read(reinterpret_cast<char*>(&obs->gnss1Outputs->satInfo.numSats), sizeof(obs->gnss1Outputs->satInfo.numSats));
                    obs->gnss1Outputs->satInfo.satellites.resize(obs->gnss1Outputs->satInfo.numSats);

                    for (auto& satellite : obs->gnss1Outputs->satInfo.satellites)
                    {
                        filestream.read(reinterpret_cast<char*>(&satellite.sys), sizeof(satellite.sys));
                        filestream.read(reinterpret_cast<char*>(&satellite.svId), sizeof(satellite.svId));
                        filestream.read(reinterpret_cast<char*>(&satellite.flags), sizeof(satellite.flags));
                        filestream.read(reinterpret_cast<char*>(&satellite.cno), sizeof(satellite.cno));
                        filestream.read(reinterpret_cast<char*>(&satellite.qi), sizeof(satellite.qi));
                        filestream.read(reinterpret_cast<char*>(&satellite.el), sizeof(satellite.el));
                        filestream.read(reinterpret_cast<char*>(&satellite.az), sizeof(satellite.az));
                    }
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS)
                {
                    filestream.read(reinterpret_cast<char*>(&obs->gnss1Outputs->raw.tow), sizeof(obs->gnss1Outputs->raw.tow));
                    filestream.read(reinterpret_cast<char*>(&obs->gnss1Outputs->raw.week), sizeof(obs->gnss1Outputs->raw.week));
                    filestream.read(reinterpret_cast<char*>(&obs->gnss1Outputs->raw.numSats), sizeof(obs->gnss1Outputs->raw.numSats));
                    obs->gnss1Outputs->raw.satellites.resize(obs->gnss1Outputs->raw.numSats);

                    for (auto& satellite : obs->gnss1Outputs->raw.satellites)
                    {
                        filestream.read(reinterpret_cast<char*>(&satellite.sys), sizeof(satellite.sys));
                        filestream.read(reinterpret_cast<char*>(&satellite.svId), sizeof(satellite.svId));
                        filestream.read(reinterpret_cast<char*>(&satellite.freq), sizeof(satellite.freq));
                        filestream.read(reinterpret_cast<char*>(&satellite.chan), sizeof(satellite.chan));
                        filestream.read(reinterpret_cast<char*>(&satellite.slot), sizeof(satellite.slot));
                        filestream.read(reinterpret_cast<char*>(&satellite.cno), sizeof(satellite.cno));
                        filestream.read(reinterpret_cast<char*>(&satellite.flags), sizeof(satellite.flags));
                        filestream.read(reinterpret_cast<char*>(&satellite.pr), sizeof(satellite.pr));
                        filestream.read(reinterpret_cast<char*>(&satellite.cp), sizeof(satellite.cp));
                        filestream.read(reinterpret_cast<char*>(&satellite.dp), sizeof(satellite.dp));
                    }
                }
            }
            // Group 5 (Attitude)
            if (binaryOutputRegister.attitudeField != vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_NONE)
            {
                if (!obs->attitudeOutputs)
                {
                    obs->attitudeOutputs = std::make_shared<NAV::sensors::vectornav::AttitudeOutputs>();
                    obs->attitudeOutputs->attitudeField |= binaryOutputRegister.attitudeField;
                }

                if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_VPESTATUS)
                {
                    filestream.read(reinterpret_cast<char*>(&obs->attitudeOutputs->vpeStatus.status), sizeof(obs->attitudeOutputs->vpeStatus.status));
                }
                if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YAWPITCHROLL)
                {
                    filestream.read(reinterpret_cast<char*>(obs->attitudeOutputs->ypr.data()), sizeof(obs->attitudeOutputs->ypr));
                }
                if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_QUATERNION)
                {
                    filestream.read(reinterpret_cast<char*>(obs->attitudeOutputs->qtn.coeffs().data()), sizeof(obs->attitudeOutputs->qtn));
                }
                if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_DCM)
                {
                    filestream.read(reinterpret_cast<char*>(obs->attitudeOutputs->dcm.data()), sizeof(obs->attitudeOutputs->dcm));
                }
                if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_MAGNED)
                {
                    filestream.read(reinterpret_cast<char*>(obs->attitudeOutputs->magNed.data()), sizeof(obs->attitudeOutputs->magNed));
                }
                if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_ACCELNED)
                {
                    filestream.read(reinterpret_cast<char*>(obs->attitudeOutputs->accelNed.data()), sizeof(obs->attitudeOutputs->accelNed));
                }
                if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_LINEARACCELBODY)
                {
                    filestream.read(reinterpret_cast<char*>(obs->attitudeOutputs->linearAccelBody.data()), sizeof(obs->attitudeOutputs->linearAccelBody));
                }
                if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_LINEARACCELNED)
                {
                    filestream.read(reinterpret_cast<char*>(obs->attitudeOutputs->linearAccelNed.data()), sizeof(obs->attitudeOutputs->linearAccelNed));
                }
                if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YPRU)
                {
                    filestream.read(reinterpret_cast<char*>(obs->attitudeOutputs->yprU.data()), sizeof(obs->attitudeOutputs->yprU));
                }
            }
            // Group 6 (INS)
            if (binaryOutputRegister.insField != vn::protocol::uart::InsGroup::INSGROUP_NONE)
            {
                if (!obs->insOutputs)
                {
                    obs->insOutputs = std::make_shared<NAV::sensors::vectornav::InsOutputs>();
                    obs->insOutputs->insField |= binaryOutputRegister.insField;
                }

                if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_INSSTATUS)
                {
                    filestream.read(reinterpret_cast<char*>(&obs->insOutputs->insStatus.status), sizeof(obs->insOutputs->insStatus.status));
                }
                if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSLLA)
                {
                    filestream.read(reinterpret_cast<char*>(obs->insOutputs->posLla.data()), sizeof(obs->insOutputs->posLla));
                }
                if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSECEF)
                {
                    filestream.read(reinterpret_cast<char*>(obs->insOutputs->posEcef.data()), sizeof(obs->insOutputs->posEcef));
                }
                if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELBODY)
                {
                    filestream.read(reinterpret_cast<char*>(obs->insOutputs->velBody.data()), sizeof(obs->insOutputs->velBody));
                }
                if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELNED)
                {
                    filestream.read(reinterpret_cast<char*>(obs->insOutputs->velNed.data()), sizeof(obs->insOutputs->velNed));
                }
                if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELECEF)
                {
                    filestream.read(reinterpret_cast<char*>(obs->insOutputs->velEcef.data()), sizeof(obs->insOutputs->velEcef));
                }
                if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_MAGECEF)
                {
                    filestream.read(reinterpret_cast<char*>(obs->insOutputs->magEcef.data()), sizeof(obs->insOutputs->magEcef));
                }
                if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_ACCELECEF)
                {
                    filestream.read(reinterpret_cast<char*>(obs->insOutputs->accelEcef.data()), sizeof(obs->insOutputs->accelEcef));
                }
                if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_LINEARACCELECEF)
                {
                    filestream.read(reinterpret_cast<char*>(obs->insOutputs->linearAccelEcef.data()), sizeof(obs->insOutputs->linearAccelEcef));
                }
                if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSU)
                {
                    filestream.read(reinterpret_cast<char*>(&obs->insOutputs->posU), sizeof(obs->insOutputs->posU));
                }
                if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELU)
                {
                    filestream.read(reinterpret_cast<char*>(&obs->insOutputs->velU), sizeof(obs->insOutputs->velU));
                }
            }
            // Group 7 (GNSS2)
            if (binaryOutputRegister.gps2Field != vn::protocol::uart::GpsGroup::GPSGROUP_NONE)
            {
                if (!obs->gnss2Outputs)
                {
                    obs->gnss2Outputs = std::make_shared<NAV::sensors::vectornav::GnssOutputs>();
                    obs->gnss2Outputs->gnssField |= binaryOutputRegister.gps2Field;
                }

                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC)
                {
                    filestream.read(reinterpret_cast<char*>(&obs->gnss2Outputs->timeUtc.year), sizeof(obs->gnss2Outputs->timeUtc.year));
                    filestream.read(reinterpret_cast<char*>(&obs->gnss2Outputs->timeUtc.month), sizeof(obs->gnss2Outputs->timeUtc.month));
                    filestream.read(reinterpret_cast<char*>(&obs->gnss2Outputs->timeUtc.day), sizeof(obs->gnss2Outputs->timeUtc.day));
                    filestream.read(reinterpret_cast<char*>(&obs->gnss2Outputs->timeUtc.hour), sizeof(obs->gnss2Outputs->timeUtc.hour));
                    filestream.read(reinterpret_cast<char*>(&obs->gnss2Outputs->timeUtc.min), sizeof(obs->gnss2Outputs->timeUtc.min));
                    filestream.read(reinterpret_cast<char*>(&obs->gnss2Outputs->timeUtc.sec), sizeof(obs->gnss2Outputs->timeUtc.sec));
                    filestream.read(reinterpret_cast<char*>(&obs->gnss2Outputs->timeUtc.ms), sizeof(obs->gnss2Outputs->timeUtc.ms));
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TOW)
                {
                    filestream.read(reinterpret_cast<char*>(&obs->gnss2Outputs->tow), sizeof(obs->gnss2Outputs->tow));
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_WEEK)
                {
                    filestream.read(reinterpret_cast<char*>(&obs->gnss2Outputs->week), sizeof(obs->gnss2Outputs->week));
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_NUMSATS)
                {
                    filestream.read(reinterpret_cast<char*>(&obs->gnss2Outputs->numSats), sizeof(obs->gnss2Outputs->numSats));
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_FIX)
                {
                    filestream.read(reinterpret_cast<char*>(&obs->gnss2Outputs->fix), sizeof(obs->gnss2Outputs->fix));
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSLLA)
                {
                    filestream.read(reinterpret_cast<char*>(obs->gnss2Outputs->posLla.data()), sizeof(obs->gnss2Outputs->posLla));
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSECEF)
                {
                    filestream.read(reinterpret_cast<char*>(obs->gnss2Outputs->posEcef.data()), sizeof(obs->gnss2Outputs->posEcef));
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELNED)
                {
                    filestream.read(reinterpret_cast<char*>(obs->gnss2Outputs->velNed.data()), sizeof(obs->gnss2Outputs->velNed));
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELECEF)
                {
                    filestream.read(reinterpret_cast<char*>(obs->gnss2Outputs->velEcef.data()), sizeof(obs->gnss2Outputs->velEcef));
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSU)
                {
                    filestream.read(reinterpret_cast<char*>(obs->gnss2Outputs->posU.data()), sizeof(obs->gnss2Outputs->posU));
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELU)
                {
                    filestream.read(reinterpret_cast<char*>(&obs->gnss2Outputs->velU), sizeof(obs->gnss2Outputs->velU));
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEU)
                {
                    filestream.read(reinterpret_cast<char*>(&obs->gnss2Outputs->timeU), sizeof(obs->gnss2Outputs->timeU));
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO)
                {
                    filestream.read(reinterpret_cast<char*>(&obs->gnss2Outputs->timeInfo.status.status), sizeof(obs->gnss2Outputs->timeInfo.status.status));
                    filestream.read(reinterpret_cast<char*>(&obs->gnss2Outputs->timeInfo.leapSeconds), sizeof(obs->gnss2Outputs->timeInfo.leapSeconds));
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP)
                {
                    filestream.read(reinterpret_cast<char*>(&obs->gnss2Outputs->dop.gDop), sizeof(obs->gnss2Outputs->dop.gDop));
                    filestream.read(reinterpret_cast<char*>(&obs->gnss2Outputs->dop.pDop), sizeof(obs->gnss2Outputs->dop.pDop));
                    filestream.read(reinterpret_cast<char*>(&obs->gnss2Outputs->dop.tDop), sizeof(obs->gnss2Outputs->dop.tDop));
                    filestream.read(reinterpret_cast<char*>(&obs->gnss2Outputs->dop.vDop), sizeof(obs->gnss2Outputs->dop.vDop));
                    filestream.read(reinterpret_cast<char*>(&obs->gnss2Outputs->dop.hDop), sizeof(obs->gnss2Outputs->dop.hDop));
                    filestream.read(reinterpret_cast<char*>(&obs->gnss2Outputs->dop.nDop), sizeof(obs->gnss2Outputs->dop.nDop));
                    filestream.read(reinterpret_cast<char*>(&obs->gnss2Outputs->dop.eDop), sizeof(obs->gnss2Outputs->dop.eDop));
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_SATINFO)
                {
                    filestream.read(reinterpret_cast<char*>(&obs->gnss2Outputs->satInfo.numSats), sizeof(obs->gnss2Outputs->satInfo.numSats));
                    obs->gnss2Outputs->satInfo.satellites.resize(obs->gnss2Outputs->satInfo.numSats);

                    for (auto& satellite : obs->gnss2Outputs->satInfo.satellites)
                    {
                        filestream.read(reinterpret_cast<char*>(&satellite.sys), sizeof(satellite.sys));
                        filestream.read(reinterpret_cast<char*>(&satellite.svId), sizeof(satellite.svId));
                        filestream.read(reinterpret_cast<char*>(&satellite.flags), sizeof(satellite.flags));
                        filestream.read(reinterpret_cast<char*>(&satellite.cno), sizeof(satellite.cno));
                        filestream.read(reinterpret_cast<char*>(&satellite.qi), sizeof(satellite.qi));
                        filestream.read(reinterpret_cast<char*>(&satellite.el), sizeof(satellite.el));
                        filestream.read(reinterpret_cast<char*>(&satellite.az), sizeof(satellite.az));
                    }
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS)
                {
                    filestream.read(reinterpret_cast<char*>(&obs->gnss2Outputs->raw.tow), sizeof(obs->gnss2Outputs->raw.tow));
                    filestream.read(reinterpret_cast<char*>(&obs->gnss2Outputs->raw.week), sizeof(obs->gnss2Outputs->raw.week));
                    filestream.read(reinterpret_cast<char*>(&obs->gnss2Outputs->raw.numSats), sizeof(obs->gnss2Outputs->raw.numSats));
                    obs->gnss1Outputs->raw.satellites.resize(obs->gnss1Outputs->raw.numSats);

                    for (auto& satellite : obs->gnss2Outputs->raw.satellites)
                    {
                        filestream.read(reinterpret_cast<char*>(&satellite.sys), sizeof(satellite.sys));
                        filestream.read(reinterpret_cast<char*>(&satellite.svId), sizeof(satellite.svId));
                        filestream.read(reinterpret_cast<char*>(&satellite.freq), sizeof(satellite.freq));
                        filestream.read(reinterpret_cast<char*>(&satellite.chan), sizeof(satellite.chan));
                        filestream.read(reinterpret_cast<char*>(&satellite.slot), sizeof(satellite.slot));
                        filestream.read(reinterpret_cast<char*>(&satellite.cno), sizeof(satellite.cno));
                        filestream.read(reinterpret_cast<char*>(&satellite.flags), sizeof(satellite.flags));
                        filestream.read(reinterpret_cast<char*>(&satellite.pr), sizeof(satellite.pr));
                        filestream.read(reinterpret_cast<char*>(&satellite.cp), sizeof(satellite.cp));
                        filestream.read(reinterpret_cast<char*>(&satellite.dp), sizeof(satellite.dp));
                    }
                }
            }
        }
        catch (const std::exception& e)
        {
            LOG_DEBUG("{}: {} after {} messages", nameId(), e.what(), messageCount);
            return nullptr;
        }
    }

    if (peek)
    {
        // Return to position before "Read line".
        filestream.seekg(len, std::ios_base::beg);
        messageCount++;
    }

    // Calls all the callbacks
    if (!peek)
    {
        invokeCallbacks(OutputPortIndex_VectorNavBinaryOutput, obs);
    }

    return obs;
}