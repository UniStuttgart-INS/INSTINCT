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
            if (alwaysNormal || std::ranges::find_if(_headerColumns, [&searchText](const std::string& header) { return header.starts_with(searchText); }) != _headerColumns.end())
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
    line.erase(line.begin(), std::ranges::find_if(line, [](int ch) { return std::isgraph(ch); }));

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
    Eigen::Vector3d lla_position{ std::nan(""), std::nan(""), std::nan("") };
    Eigen::Vector3d e_position{ std::nan(""), std::nan(""), std::nan("") };
    Eigen::Vector3d n_velocity{ std::nan(""), std::nan(""), std::nan("") };
    Eigen::Vector3d e_velocity{ std::nan(""), std::nan(""), std::nan("") };

    Eigen::Matrix3d e_posVar = Eigen::Matrix3d::Zero() * std::nan("");
    Eigen::Matrix3d e_velVar = Eigen::Matrix3d::Zero() * std::nan("");
    Eigen::Matrix3d n_posVar = Eigen::Matrix3d::Zero() * std::nan("");
    Eigen::Matrix3d n_velVar = Eigen::Matrix3d::Zero() * std::nan("");

    try
    {
        for (const auto& column : _headerColumns)
        {
            if (lineStream >> cell)
            {
                // Remove any trailing non text characters
                cell.erase(std::ranges::find_if(cell, [](int ch) { return std::iscntrl(ch); }), cell.end());
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
                else if (column == "x-ecef(m)" || column == "x-ecef")
                {
                    e_position.x() = std::stod(cell);
                }
                else if (column == "y-ecef(m)" || column == "y-ecef")
                {
                    e_position.y() = std::stod(cell);
                }
                else if (column == "z-ecef(m)" || column == "z-ecef")
                {
                    e_position.z() = std::stod(cell);
                }
                else if (column == "latitude(deg)" || column == "latitude")
                {
                    lla_position(0) = deg2rad(std::stod(cell));
                }
                else if (column == "longitude(deg)" || column == "longitude")
                {
                    lla_position(1) = deg2rad(std::stod(cell));
                }
                else if (column == "height(m)" || column == "height")
                {
                    lla_position(2) = std::stod(cell);
                }
                else if (column == "Q")
                {
                    obs->Q = static_cast<uint8_t>(std::stoul(cell));
                }
                else if (column == "ns")
                {
                    obs->ns = static_cast<uint8_t>(std::stoul(cell));
                }
                else if (column == "sdx(m)" || column == "sdx")
                {
                    e_posVar(0, 0) = std::pow(std::stod(cell), 2);
                }
                else if (column == "sdy(m)" || column == "sdy")
                {
                    e_posVar(1, 1) = std::pow(std::stod(cell), 2);
                }
                else if (column == "sdz(m)" || column == "sdz")
                {
                    e_posVar(2, 2) = std::pow(std::stod(cell), 2);
                }
                else if (column == "sdn(m)" || column == "sdn")
                {
                    n_posVar(0, 0) = std::pow(std::stod(cell), 2);
                }
                else if (column == "sde(m)" || column == "sde")
                {
                    n_posVar(1, 1) = std::pow(std::stod(cell), 2);
                }
                else if (column == "sdu(m)" || column == "sdu")
                {
                    n_posVar(2, 2) = std::pow(std::stod(cell), 2);
                }
                else if (column == "sdxy(m)" || column == "sdxy")
                {
                    e_posVar(0, 1) = std::stod(cell);
                    e_posVar(0, 1) = gcem::sgn(e_posVar(0, 1)) * std::pow(e_posVar(0, 1), 2);
                    e_posVar(1, 0) = -e_posVar(0, 1);
                }
                else if (column == "sdyz(m)" || column == "sdyz")
                {
                    e_posVar(1, 2) = std::stod(cell);
                    e_posVar(1, 2) = gcem::sgn(e_posVar(1, 2)) * std::pow(e_posVar(1, 2), 2);
                    e_posVar(2, 1) = -e_posVar(1, 2);
                }
                else if (column == "sdzx(m)" || column == "sdzx")
                {
                    e_posVar(2, 0) = std::stod(cell);
                    e_posVar(2, 0) = gcem::sgn(e_posVar(2, 0)) * std::pow(e_posVar(2, 0), 2);
                    e_posVar(0, 2) = -e_posVar(2, 0);
                }
                else if (column == "sdne(m)" || column == "sdne")
                {
                    n_posVar(0, 1) = std::stod(cell);
                    n_posVar(0, 1) = gcem::sgn(n_posVar(0, 1)) * std::pow(n_posVar(0, 1), 2);
                    n_posVar(1, 0) = -n_posVar(0, 1);
                }
                else if (column == "sdeu(m)" || column == "sdeu")
                {
                    n_posVar(1, 2) = std::stod(cell);
                    n_posVar(1, 2) = gcem::sgn(n_posVar(1, 2)) * std::pow(n_posVar(1, 2), 2);
                    n_posVar(2, 1) = -n_posVar(1, 2);
                }
                else if (column == "sdun(m)" || column == "sdun")
                {
                    n_posVar(2, 0) = std::stod(cell);
                    n_posVar(2, 0) = gcem::sgn(n_posVar(2, 0)) * std::pow(n_posVar(2, 0), 2);
                    n_posVar(0, 2) = -n_posVar(2, 0);
                }
                else if (column == "age(s)" || column == "age")
                {
                    obs->age = std::stod(cell);
                }
                else if (column == "ratio")
                {
                    obs->ratio = std::stod(cell);
                }
                else if (column == "vn(m/s)" || column == "vn")
                {
                    n_velocity(0) = std::stod(cell);
                }
                else if (column == "ve(m/s)" || column == "ve")
                {
                    n_velocity(1) = std::stod(cell);
                }
                else if (column == "vu(m/s)" || column == "vu")
                {
                    n_velocity(2) = -std::stod(cell);
                }
                else if (column == "vx(m/s)" || column == "vx")
                {
                    e_velocity(0) = std::stod(cell);
                }
                else if (column == "vy(m/s)" || column == "vy")
                {
                    e_velocity(1) = std::stod(cell);
                }
                else if (column == "vz(m/s)" || column == "vz")
                {
                    e_velocity(2) = std::stod(cell);
                }
                else if (column == "sdvn(m/s)" || column == "sdvn")
                {
                    n_velVar(0, 0) = std::pow(std::stod(cell), 2);
                }
                else if (column == "sdve(m/s)" || column == "sdve")
                {
                    n_velVar(1, 1) = std::pow(std::stod(cell), 2);
                }
                else if (column == "sdvu(m/s)" || column == "sdvu")
                {
                    n_velVar(2, 2) = std::pow(std::stod(cell), 2);
                }
                else if (column == "sdvne(m/s)" || column == "sdvne")
                {
                    n_velVar(0, 1) = std::stod(cell);
                    n_velVar(0, 1) = gcem::sgn(n_velVar(0, 1)) * std::pow(n_velVar(0, 1), 2);
                    n_velVar(1, 0) = -n_velVar(0, 1);
                }
                else if (column == "sdveu(m/s)" || column == "sdveu")
                {
                    n_velVar(1, 2) = std::stod(cell);
                    n_velVar(1, 2) = gcem::sgn(n_velVar(1, 2)) * std::pow(n_velVar(1, 2), 2);
                    n_velVar(2, 1) = -n_velVar(1, 2);
                }
                else if (column == "sdvun(m/s)" || column == "sdvun")
                {
                    n_velVar(2, 0) = std::stod(cell);
                    n_velVar(2, 0) = gcem::sgn(n_velVar(2, 0)) * std::pow(n_velVar(2, 0), 2);
                    n_velVar(0, 2) = -n_velVar(2, 0);
                }
                else if (column == "sdvx(m/s)" || column == "sdvx")
                {
                    e_velVar(0, 0) = std::pow(std::stod(cell), 2);
                }
                else if (column == "sdvy(m/s)" || column == "sdvy")
                {
                    e_velVar(1, 1) = std::pow(std::stod(cell), 2);
                }
                else if (column == "sdvz(m/s)" || column == "sdvz")
                {
                    e_velVar(2, 2) = std::pow(std::stod(cell), 2);
                }
                else if (column == "sdvxy(m/s)" || column == "sdvxy")
                {
                    e_velVar(0, 1) = std::stod(cell);
                    e_velVar(0, 1) = gcem::sgn(e_velVar(0, 1)) * std::pow(e_velVar(0, 1), 2);
                    e_velVar(1, 0) = -e_velVar(0, 1);
                }
                else if (column == "sdvyz(m/s)" || column == "sdvyz")
                {
                    e_velVar(1, 2) = std::stod(cell);
                    e_velVar(1, 2) = gcem::sgn(e_velVar(1, 2)) * std::pow(e_velVar(1, 2), 2);
                    e_velVar(2, 1) = -e_velVar(1, 2);
                }
                else if (column == "sdvzx(m/s)" || column == "sdvzx")
                {
                    e_velVar(2, 0) = std::stod(cell);
                    e_velVar(2, 0) = gcem::sgn(e_velVar(2, 0)) * std::pow(e_velVar(2, 0), 2);
                    e_velVar(0, 2) = -e_velVar(2, 0);
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

    if (!e_position.hasNaN() && !e_posVar.hasNaN()) { obs->setPositionAndStdDev_e(e_position, e_posVar); }
    else if (!lla_position.hasNaN() && !n_posVar.hasNaN()) { obs->setPositionAndStdDev_lla(lla_position, n_posVar); }
    else if (!e_position.hasNaN()) { obs->setPosition_e(e_position); }
    else if (!lla_position.hasNaN()) { obs->setPosition_lla(lla_position); }

    if (!e_velocity.hasNaN() && !e_velVar.hasNaN()) { obs->setVelocityAndStdDev_e(e_velocity, e_velVar); }
    if (!n_velocity.hasNaN() && !n_velVar.hasNaN()) { obs->setVelocityAndStdDev_n(n_velocity, n_velVar); }
    else if (!e_velocity.hasNaN()) { obs->setVelocity_e(e_velocity); }
    else if (!n_velocity.hasNaN()) { obs->setVelocity_n(n_velocity); }

    if (!e_velVar.hasNaN() && !e_posVar.hasNaN())
    {
        Eigen::MatrixXd cov = Eigen::MatrixXd::Zero(6, 6);
        cov.block<3, 3>(0, 0) = e_posVar;
        cov.block<3, 3>(3, 3) = e_velVar;
        obs->setPosVelCovarianceMatrix_e(cov);
    }
    else if (!n_velVar.hasNaN() && !n_posVar.hasNaN())
    {
        Eigen::MatrixXd cov = Eigen::MatrixXd::Zero(6, 6);
        cov.block<3, 3>(0, 0) = n_posVar;
        cov.block<3, 3>(3, 3) = n_velVar;
        obs->setPosVelCovarianceMatrix_n(cov);
    }
    else if (!e_posVar.hasNaN())
    {
        obs->setPosCovarianceMatrix_e(e_posVar);
    }
    else if (!n_posVar.hasNaN())
    {
        obs->setPosCovarianceMatrix_n(n_posVar);
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
        line.erase(line.begin(), std::ranges::find_if(line, [](int ch) { return std::isgraph(ch); }));
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
        line.erase(line.begin(), std::ranges::find_if(line, [](int ch) { return std::isgraph(ch); }));
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