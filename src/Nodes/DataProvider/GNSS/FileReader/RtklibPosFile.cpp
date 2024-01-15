// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "RtklibPosFile.hpp"

#include "util/Logger.hpp"
#include "Navigation/Transformations/CoordinateFrames.hpp"
#include "Navigation/Transformations/Units.hpp"
#include "util/Time/TimeBase.hpp"
#include "util/StringUtil.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeData/GNSS/RtklibPosObs.hpp"

NAV::RtklibPosFile::RtklibPosFile()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 380, 290 };

    nm::CreateOutputPin(this, "RtklibPosObs", Pin::Type::Flow, { NAV::RtklibPosObs::type() }, &RtklibPosFile::pollData);
}

NAV::RtklibPosFile::~RtklibPosFile()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::RtklibPosFile::typeStatic()
{
    return "RtklibPosFile";
}

std::string NAV::RtklibPosFile::type() const
{
    return typeStatic();
}

std::string NAV::RtklibPosFile::category()
{
    return "Data Provider";
}

void NAV::RtklibPosFile::guiConfig()
{
    if (auto res = FileReader::guiConfig(".pos,.*", { ".pos" }, size_t(id), nameId()))
    {
        LOG_DEBUG("{}: Path changed to {}", nameId(), _path);
        flow::ApplyChanges();
        if (res == FileReader::PATH_CHANGED)
        {
            doReinitialize();
        }
        else
        {
            doDeinitialize();
        }
    }

    ///  Header info
    if (ImGui::BeginTable(fmt::format("##RtklibPos ({})", id.AsPointer()).c_str(), 4,
                          ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_NoHostExtendX))
    {
        ImGui::TableSetupColumn("Basic", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("LLA", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("XYZ", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("Velocity", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableHeadersRow();

        auto TextColoredIfExists = [this](int index, const char* displayText, const char* searchText, bool alwaysNormal = false) {
            ImGui::TableSetColumnIndex(index);
            if (alwaysNormal || std::find_if(_headerColumns.begin(), _headerColumns.end(), [&searchText](const std::string& header) { return header.starts_with(searchText); }) != _headerColumns.end())
            {
                ImGui::TextUnformatted(displayText);
            }
            else
            {
                ImGui::TextDisabled("%s", displayText);
            }
        };

        ImGui::TableNextRow();
        TextColoredIfExists(0, "Date", "Date");
        TextColoredIfExists(1, "latitude(deg)", "latitude(deg)");
        TextColoredIfExists(2, "x-ecef(m)", "x-ecef(m)");
        TextColoredIfExists(3, "vn(m/s)", "vn(m/s)");
        ImGui::TableNextRow();
        TextColoredIfExists(0, "Time", "Time");
        TextColoredIfExists(1, "longitude(deg)", "longitude(deg)");
        TextColoredIfExists(2, "y-ecef(m)", "y-ecef(m)");
        TextColoredIfExists(3, "ve(m/s)", "ve(m/s)");
        ImGui::TableNextRow();
        TextColoredIfExists(0, "age(s)", "age(s)");
        TextColoredIfExists(1, "height(m)", "height(m)");
        TextColoredIfExists(2, "z-ecef(m)", "z-ecef(m)");
        TextColoredIfExists(3, "vu(m/s)", "vu(m/s)");
        ImGui::TableNextRow();
        TextColoredIfExists(0, "ratio", "ratio");
        TextColoredIfExists(1, "sdn(m)", "sdn(m)");
        TextColoredIfExists(2, "sdx(m)", "sdx(m)");
        TextColoredIfExists(3, "sdvn", "sdvn");
        ImGui::TableNextRow();
        TextColoredIfExists(0, "Q", "Q");
        TextColoredIfExists(1, "sde(m)", "sde(m)");
        TextColoredIfExists(2, "sdy(m)", "sdy(m)");
        TextColoredIfExists(3, "sdve", "sdve");
        ImGui::TableNextRow();
        TextColoredIfExists(0, "ns", "ns");
        TextColoredIfExists(1, "sdu(m)", "sdu(m)");
        TextColoredIfExists(2, "sdz(m)", "sdz(m)");
        TextColoredIfExists(3, "sdvu", "sdvu");
        ImGui::TableNextRow();
        TextColoredIfExists(1, "sdne(m)", "sdne(m)");
        TextColoredIfExists(2, "sdxy(m)", "sdxy(m)");
        TextColoredIfExists(3, "sdvne", "sdvne");
        ImGui::TableNextRow();
        TextColoredIfExists(1, "sdeu(m)", "sdeu(m)");
        TextColoredIfExists(2, "sdyz(m)", "sdyz(m)");
        TextColoredIfExists(3, "sdveu", "sdveu");
        ImGui::TableNextRow();
        TextColoredIfExists(1, "sdun(m)", "sdun(m)");
        TextColoredIfExists(2, "sdzx(m)", "sdzx(m)");
        TextColoredIfExists(3, "sdvun", "sdvun");

        ImGui::EndTable();
    }
}

[[nodiscard]] json NAV::RtklibPosFile::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["FileReader"] = FileReader::save();

    return j;
}

void NAV::RtklibPosFile::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("FileReader"))
    {
        FileReader::restore(j.at("FileReader"));
    }
}

bool NAV::RtklibPosFile::initialize()
{
    LOG_TRACE("{}: called", nameId());

    return FileReader::initialize();
}

void NAV::RtklibPosFile::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    FileReader::deinitialize();
}

bool NAV::RtklibPosFile::resetNode()
{
    FileReader::resetReader();

    return true;
}

std::shared_ptr<const NAV::NodeData> NAV::RtklibPosFile::pollData()
{
    auto obs = std::make_shared<RtklibPosObs>();

    // Read line
    std::string line;
    getline(line);
    // Remove any starting non text characters
    line.erase(line.begin(), std::find_if(line.begin(), line.end(), [](int ch) { return std::isgraph(ch); }));

    if (line.empty())
    {
        return nullptr;
    }

    std::istringstream lineStream(line);
    std::string cell;

    TimeSystem timeSystem = GPST;
    std::optional<uint16_t> year;
    std::optional<uint16_t> month;
    std::optional<uint16_t> day;
    std::optional<int32_t> hour;
    std::optional<uint16_t> minute;
    std::optional<long double> second = 0L;
    std::optional<uint16_t> gpsWeek;
    std::optional<long double> gpsToW;
    Eigen::Vector3d lla_pos{ std::nan(""), std::nan(""), std::nan("") };
    Eigen::Vector3d e_pos{ std::nan(""), std::nan(""), std::nan("") };
    Eigen::Vector3d n_vel{ std::nan(""), std::nan(""), std::nan("") };
    Eigen::Vector3d e_vel{ std::nan(""), std::nan(""), std::nan("") };

    std::optional<double> sdvN;
    std::optional<double> sdvE;
    std::optional<double> sdvD;
    std::optional<double> sdvX;
    std::optional<double> sdvY;
    std::optional<double> sdvZ;

    try
    {
        for (const auto& column : _headerColumns)
        {
            if (lineStream >> cell)
            {
                // Remove any trailing non text characters
                cell.erase(std::find_if(cell.begin(), cell.end(), [](int ch) { return std::iscntrl(ch); }), cell.end());
                if (cell.empty())
                {
                    continue;
                }

                // %  GPST          latitude(deg) longitude(deg)  ...
                // 2120 216180.000   XX.XXXXXXXXX    ...
                if (column == "GpsWeek")
                {
                    gpsWeek = static_cast<uint16_t>(std::stoul(cell));
                }
                else if (column == "GpsToW")
                {
                    gpsToW = std::stold(cell);
                }
                // %  GPST                  latitude(deg) longitude(deg)  ...
                // 2020/08/25 12:03:00.000   XX.XXXXXXXXX    ...
                // %  UTC                   latitude(deg) longitude(deg)  ...
                // 2020/08/25 12:02:42.000   XX.XXXXXXXXX    ...
                else if (column.starts_with("Date"))
                {
                    timeSystem = column.ends_with("-GPST") ? GPST : UTC;

                    auto ymd = str::split(cell, "/");
                    if (ymd.size() == 3)
                    {
                        year = static_cast<uint16_t>(std::stoi(ymd.at(0)));
                        month = static_cast<uint16_t>(std::stoi(ymd.at(1)));
                        day = static_cast<uint16_t>(std::stoi(ymd.at(2)));
                    }
                }
                else if (column.starts_with("Time"))
                {
                    auto hms = str::split(cell, ":");
                    if (hms.size() == 3)
                    {
                        hour = static_cast<uint16_t>(std::stoi(hms.at(0)));
                        if (column.ends_with("-JST")) { *hour -= 9; }
                        minute = static_cast<uint16_t>(std::stoi(hms.at(1)));
                        second = std::stold(hms.at(2));
                    }
                }
                else if (column == "x-ecef(m)")
                {
                    e_pos.x() = std::stod(cell);
                }
                else if (column == "y-ecef(m)")
                {
                    e_pos.y() = std::stod(cell);
                }
                else if (column == "z-ecef(m)")
                {
                    e_pos.z() = std::stod(cell);
                }
                else if (column == "latitude(deg)")
                {
                    lla_pos(0) = deg2rad(std::stod(cell));
                }
                else if (column == "longitude(deg)")
                {
                    lla_pos(1) = deg2rad(std::stod(cell));
                }
                else if (column == "height(m)")
                {
                    lla_pos(2) = std::stod(cell);
                }
                else if (column == "Q")
                {
                    obs->Q = static_cast<uint8_t>(std::stoul(cell));
                }
                else if (column == "ns")
                {
                    obs->ns = static_cast<uint8_t>(std::stoul(cell));
                }
                else if (column == "sdx(m)")
                {
                    obs->sdXYZ.x() = std::stod(cell);
                }
                else if (column == "sdy(m)")
                {
                    obs->sdXYZ.y() = std::stod(cell);
                }
                else if (column == "sdz(m)")
                {
                    obs->sdXYZ.z() = std::stod(cell);
                }
                else if (column == "sdn(m)")
                {
                    obs->sdNED(0) = std::stod(cell);
                }
                else if (column == "sde(m)")
                {
                    obs->sdNED(1) = std::stod(cell);
                }
                else if (column == "sdu(m)")
                {
                    obs->sdNED(2) = std::stod(cell);
                }
                else if (column == "sdxy(m)")
                {
                    obs->sdxy = std::stod(cell);
                }
                else if (column == "sdyz(m)")
                {
                    obs->sdyz = std::stod(cell);
                }
                else if (column == "sdzx(m)")
                {
                    obs->sdzx = std::stod(cell);
                }
                else if (column == "sdne(m)")
                {
                    obs->sdne = std::stod(cell);
                }
                else if (column == "sdeu(m)")
                {
                    obs->sded = std::stod(cell);
                }
                else if (column == "sdun(m)")
                {
                    obs->sddn = std::stod(cell);
                }
                else if (column == "age(s)")
                {
                    obs->age = std::stod(cell);
                }
                else if (column == "ratio")
                {
                    obs->ratio = std::stod(cell);
                }
                else if (column == "vn(m/s)")
                {
                    n_vel(0) = std::stod(cell);
                }
                else if (column == "ve(m/s)")
                {
                    n_vel(1) = std::stod(cell);
                }
                else if (column == "vu(m/s)")
                {
                    n_vel(2) = -std::stod(cell);
                }
                else if (column == "vx(m/s)")
                {
                    e_vel(0) = std::stod(cell);
                }
                else if (column == "vy(m/s)")
                {
                    e_vel(1) = std::stod(cell);
                }
                else if (column == "vz(m/s)")
                {
                    e_vel(2) = std::stod(cell);
                }
                else if (column == "sdvn")
                {
                    sdvN = std::stod(cell);
                }
                else if (column == "sdve")
                {
                    sdvE = std::stod(cell);
                }
                else if (column == "sdvu")
                {
                    sdvD = std::stod(cell);
                }
                else if (column == "sdvne")
                {
                    obs->sdvne = std::stod(cell);
                }
                else if (column == "sdveu")
                {
                    obs->sdved = std::stod(cell);
                }
                else if (column == "sdvun")
                {
                    obs->sdvdn = std::stod(cell);
                }
                else if (column == "sdvx")
                {
                    sdvX = std::stod(cell);
                }
                else if (column == "sdvy")
                {
                    sdvY = std::stod(cell);
                }
                else if (column == "sdvz")
                {
                    sdvZ = std::stod(cell);
                }
                else if (column == "sdvxy")
                {
                    obs->sdvxy = std::stod(cell);
                }
                else if (column == "sdvyz")
                {
                    obs->sdvyz = std::stod(cell);
                }
                else if (column == "sdvzx")
                {
                    obs->sdvzx = std::stod(cell);
                }
            }
        }
    }
    catch (...)
    {
        return nullptr;
    }

    if (gpsWeek.has_value() && gpsToW.has_value())
    {
        obs->insTime = InsTime(0, gpsWeek.value(), gpsToW.value());
    }
    else if (year.has_value() && month.has_value() && day.has_value()
             && hour.has_value() && minute.has_value() && second.has_value())
    {
        obs->insTime = InsTime(year.value(), month.value(), day.value(),
                               hour.value(), minute.value(), second.value(),
                               timeSystem);
    }

    if (!e_pos.hasNaN()) { obs->setPosition_e(e_pos); }
    else if (!lla_pos.hasNaN()) { obs->setPosition_lla(lla_pos); }

    if (!obs->sdXYZ.hasNaN()) { obs->sdNED = trafo::n_Quat_e(obs->latitude(), obs->longitude()) * obs->sdXYZ; }
    else if (!obs->sdNED.hasNaN()) { obs->sdXYZ = trafo::e_Quat_n(obs->latitude(), obs->longitude()) * obs->sdNED; }

    if (!n_vel.hasNaN()) { obs->setVelocity_n(n_vel); }
    else if (!e_vel.hasNaN()) { obs->setVelocity_e(e_vel); }

    if (sdvN && sdvE && sdvD) { obs->sdvNED = Eigen::Vector3d(sdvN.value(), sdvE.value(), sdvD.value()); }
    else if (sdvX && sdvY && sdvZ) { obs->sdvXYZ = Eigen::Vector3d(sdvX.value(), sdvY.value(), sdvZ.value()); }

    if (auto currentTime = util::time::GetCurrentInsTime();
        obs->insTime.empty() && !currentTime.empty())
    {
        obs->insTime = currentTime;
    }

    invokeCallbacks(OUTPUT_PORT_INDEX_RTKLIB_POS_OBS, obs);
    return obs;
}

NAV::FileReader::FileType NAV::RtklibPosFile::determineFileType()
{
    std::filesystem::path filepath = getFilepath();

    auto filestreamHeader = std::ifstream(filepath);
    if (!filestreamHeader.good())
    {
        return FileReader::FileType::NONE;
    }

    std::string line;
    do
    {
        if (filestreamHeader.eof())
        {
            return FileReader::FileType::NONE;
        }

        std::getline(filestreamHeader, line);
        // Remove any starting non text characters
        line.erase(line.begin(), std::find_if(line.begin(), line.end(),
                                              [](int ch) { return std::isgraph(ch); }));
    } while (!line.empty() && line.find("%  ") == std::string::npos);

    return FileReader::FileType::ASCII;
}

void NAV::RtklibPosFile::readHeader()
{
    // Read header line
    std::string line;
    do
    {
        getline(line);
        // Remove any starting non text characters
        line.erase(line.begin(), std::find_if(line.begin(), line.end(),
                                              [](int ch) { return std::isgraph(ch); }));
    } while (!line.empty() && line.find("%  ") == std::string::npos);

    // Convert line into stream
    std::istringstream lineStream(line);

    for (std::string cell; lineStream >> cell;) // split at 'space'
    {
        if (cell != "%")
        {
            if (cell == "GPST") // When RTKLIB selected 'ww ssss GPST' or 'hh:mm:ss GPST'
            {
                auto pos = tellg();
                getline(line);
                seekg(pos, std::ios::beg);

                if (line.substr(0, 7).find('/') == std::string::npos)
                {
                    // %  GPST          latitude(deg) longitude(deg)  ...
                    // 2120 216180.000   XX.XXXXXXXXX    ...
                    _headerColumns.emplace_back("GpsWeek");
                    _headerColumns.emplace_back("GpsToW");
                }
                else
                {
                    // %  GPST                  latitude(deg) longitude(deg)  ...
                    // 2020/08/25 12:03:00.000   XX.XXXXXXXXX    ...
                    _headerColumns.emplace_back("Date-GPST");
                    _headerColumns.emplace_back("Time-GPST");
                }
            }
            else if (cell == "UTC") // When RTKLIB selected 'hh:mm:ss UTC'
            {
                // %  UTC                   latitude(deg) longitude(deg)  ...
                // 2020/08/25 12:02:42.000   XX.XXXXXXXXX    ...
                _headerColumns.emplace_back("Date-UTC");
                _headerColumns.emplace_back("Time-UTC");
            }
            else if (cell == "JST") // When RTKLIB selected 'hh:mm:ss JST'
            {
                // %  JST                   latitude(deg) longitude(deg)  ...
                // 2020/08/25 21:02:42.000   XX.XXXXXXXXX    ...
                _headerColumns.emplace_back("Date-JST");
                _headerColumns.emplace_back("Time-JST");
            }
            else
            {
                _headerColumns.push_back(cell);
            }
        }
    }
}