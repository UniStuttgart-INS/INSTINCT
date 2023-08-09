// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "MultiImuFile.hpp"

#include "util/Logger.hpp"

#include "Navigation/Transformations/CoordinateFrames.hpp"
#include "Navigation/Transformations/Units.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"
#include "internal/gui/widgets/EnumCombo.hpp"
#include "internal/gui/widgets/HelpMarker.hpp"
#include "internal/gui/NodeEditorApplication.hpp"

NAV::MultiImuFile::MultiImuFile()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 528, 379 };

    updateNumberOfOutputPins();
}

NAV::MultiImuFile::~MultiImuFile()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::MultiImuFile::typeStatic()
{
    return "MultiImuFile";
}

std::string NAV::MultiImuFile::type() const
{
    return typeStatic();
}

std::string NAV::MultiImuFile::category()
{
    return "Data Provider";
}

void NAV::MultiImuFile::guiConfig()
{
    float columnWidth = 130.0F * gui::NodeEditorApplication::windowFontRatio();

    if (FileReader::guiConfig(".txt", { ".txt" }, size_t(id), nameId()))
    {
        flow::ApplyChanges();
        deinitialize();
    }

    ImGui::SetNextItemWidth(columnWidth);

    if (gui::widgets::EnumCombo(fmt::format("NMEA message type##{}", size_t(id)).c_str(), _nmeaType))
    {
        LOG_DEBUG("{}: nmeaType changed to {}", nameId(), fmt::underlying(_nmeaType));

        flow::ApplyChanges();
        doDeinitialize();
    }
    ImGui::SameLine();
    gui::widgets::HelpMarker("Until June 2023, NMEA messages in the Multi-IMU file's header were of the 'GPGGA' type. Since this type does not provide an absolute time reference, it was changed to 'GPZDA'.\n\n");

    if (_nmeaType == NmeaType::GPGGA)
    {
        if (gui::widgets::TimeEdit(fmt::format("{}", size_t(id)).c_str(), _startTime, _startTimeEditFormat))
        {
            LOG_DEBUG("{}: startTime changed to {}", nameId(), _startTime);
            flow::ApplyChanges();
        }
    }

    ImGui::Separator();
    // Set Imu Position and Rotation (from 'Imu::guiConfig();')
    bool showRotation = true;
    for (size_t i = 0; i < _nSensors; ++i)
    {
        ImGui::SetNextItemOpen(showRotation, ImGuiCond_FirstUseEver);
        if (i == 0) { showRotation = false; }
        if (ImGui::TreeNode(fmt::format("Imu #{} Position & Rotation##{}", i + 1, size_t(id)).c_str()))
        {
            std::array<float, 3> imuPosAccel = { static_cast<float>(_imuPosAll[i].b_positionAccel().x()), static_cast<float>(_imuPosAll[i].b_positionAccel().y()), static_cast<float>(_imuPosAll[i].b_positionAccel().z()) };
            if (ImGui::InputFloat3(fmt::format("Lever Accel [m]##{}", size_t(id)).c_str(), imuPosAccel.data()))
            {
                flow::ApplyChanges();
                _imuPosAll[i]._b_positionAccel = Eigen::Vector3d(imuPosAccel.at(0), imuPosAccel.at(1), imuPosAccel.at(2));
            }
            ImGui::SameLine();
            gui::widgets::HelpMarker("Position of the accelerometer sensor relative to the vehicle center of mass in the body coordinate frame.");

            std::array<float, 3> imuPosGyro = { static_cast<float>(_imuPosAll[i].b_positionGyro().x()), static_cast<float>(_imuPosAll[i].b_positionGyro().y()), static_cast<float>(_imuPosAll[i].b_positionGyro().z()) };
            if (ImGui::InputFloat3(fmt::format("Lever Gyro [m]##{}", size_t(id)).c_str(), imuPosGyro.data()))
            {
                flow::ApplyChanges();
                _imuPosAll[i]._b_positionGyro = Eigen::Vector3d(imuPosGyro.at(0), imuPosGyro.at(1), imuPosGyro.at(2));
            }
            ImGui::SameLine();
            gui::widgets::HelpMarker("Position of the gyroscope sensor relative to the vehicle center of mass in the body coordinate frame.");

            std::array<float, 3> imuPosMag = { static_cast<float>(_imuPosAll[i].b_positionMag().x()), static_cast<float>(_imuPosAll[i].b_positionMag().y()), static_cast<float>(_imuPosAll[i].b_positionMag().z()) };
            if (ImGui::InputFloat3(fmt::format("Lever Mag [m]##{}", size_t(id)).c_str(), imuPosMag.data()))
            {
                flow::ApplyChanges();
                _imuPosAll[i]._b_positionMag = Eigen::Vector3d(imuPosMag.at(0), imuPosMag.at(1), imuPosMag.at(2));
            }
            ImGui::SameLine();
            gui::widgets::HelpMarker("Position of the magnetometer sensor relative to the vehicle center of mass in the body coordinate frame.");

            Eigen::Vector3d eulerAccel = rad2deg(trafo::quat2eulerZYX(_imuPosAll[i].p_quatAccel_b()));
            std::array<float, 3> imuRotAccel = { static_cast<float>(eulerAccel.x()), static_cast<float>(eulerAccel.y()), static_cast<float>(eulerAccel.z()) };
            if (ImGui::InputFloat3(fmt::format("Rotation Accel [deg]##{}", size_t(id)).c_str(), imuRotAccel.data()))
            {
                // (-180:180] x (-90:90] x (-180:180]
                if (imuRotAccel.at(0) < -179.9999F)
                {
                    imuRotAccel.at(0) = -179.9999F;
                }
                if (imuRotAccel.at(0) > 180)
                {
                    imuRotAccel.at(0) = 180;
                }
                if (imuRotAccel.at(1) < -89.9999F)
                {
                    imuRotAccel.at(1) = -89.9999F;
                }
                if (imuRotAccel.at(1) > 90)
                {
                    imuRotAccel.at(1) = 90;
                }
                if (imuRotAccel.at(2) < -179.9999F)
                {
                    imuRotAccel.at(2) = -179.9999F;
                }
                if (imuRotAccel.at(2) > 180)
                {
                    imuRotAccel.at(2) = 180;
                }

                flow::ApplyChanges();
                _imuPosAll[i]._b_quatAccel_p = trafo::b_Quat_p(deg2rad(imuRotAccel.at(0)), deg2rad(imuRotAccel.at(1)), deg2rad(imuRotAccel.at(2)));
            }

            Eigen::Vector3d eulerGyro = rad2deg(trafo::quat2eulerZYX(_imuPosAll[i].p_quatGyro_b()));
            std::array<float, 3> imuRotGyro = { static_cast<float>(eulerGyro.x()), static_cast<float>(eulerGyro.y()), static_cast<float>(eulerGyro.z()) };
            if (ImGui::InputFloat3(fmt::format("Rotation Gyro [deg]##{}", size_t(id)).c_str(), imuRotGyro.data()))
            {
                // (-180:180] x (-90:90] x (-180:180]
                if (imuRotGyro.at(0) < -179.9999F)
                {
                    imuRotGyro.at(0) = -179.9999F;
                }
                if (imuRotGyro.at(0) > 180)
                {
                    imuRotGyro.at(0) = 180;
                }
                if (imuRotGyro.at(1) < -89.9999F)
                {
                    imuRotGyro.at(1) = -89.9999F;
                }
                if (imuRotGyro.at(1) > 90)
                {
                    imuRotGyro.at(1) = 90;
                }
                if (imuRotGyro.at(2) < -179.9999F)
                {
                    imuRotGyro.at(2) = -179.9999F;
                }
                if (imuRotGyro.at(2) > 180)
                {
                    imuRotGyro.at(2) = 180;
                }

                flow::ApplyChanges();
                _imuPosAll[i]._b_quatGyro_p = trafo::b_Quat_p(deg2rad(imuRotGyro.at(0)), deg2rad(imuRotGyro.at(1)), deg2rad(imuRotGyro.at(2)));
            }

            Eigen::Vector3d eulerMag = rad2deg(trafo::quat2eulerZYX(_imuPosAll[i].p_quatMag_b()));
            std::array<float, 3> imuRotMag = { static_cast<float>(eulerMag.x()), static_cast<float>(eulerMag.y()), static_cast<float>(eulerMag.z()) };
            if (ImGui::InputFloat3(fmt::format("Rotation Mag [deg]##{}", size_t(id)).c_str(), imuRotMag.data()))
            {
                // (-180:180] x (-90:90] x (-180:180]
                if (imuRotMag.at(0) < -179.9999F)
                {
                    imuRotMag.at(0) = -179.9999F;
                }
                if (imuRotMag.at(0) > 180)
                {
                    imuRotMag.at(0) = 180;
                }
                if (imuRotMag.at(1) < -89.9999F)
                {
                    imuRotMag.at(1) = -89.9999F;
                }
                if (imuRotMag.at(1) > 90)
                {
                    imuRotMag.at(1) = 90;
                }
                if (imuRotMag.at(2) < -179.9999F)
                {
                    imuRotMag.at(2) = -179.9999F;
                }
                if (imuRotMag.at(2) > 180)
                {
                    imuRotMag.at(2) = 180;
                }

                flow::ApplyChanges();
                _imuPosAll[i]._b_quatMag_p = trafo::b_Quat_p(deg2rad(imuRotMag.at(0)), deg2rad(imuRotMag.at(1)), deg2rad(imuRotMag.at(2)));
            }

            ImGui::TreePop();
        }
    }
}

[[nodiscard]] json NAV::MultiImuFile::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["FileReader"] = FileReader::save();
    j["imuPos"] = _imuPosAll;
    j["nmeaType"] = _nmeaType;
    j["startTime"] = _startTime;

    return j;
}

void NAV::MultiImuFile::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("FileReader"))
    {
        FileReader::restore(j.at("FileReader"));
    }
    if (j.contains("imuPos"))
    {
        j.at("imuPos").get_to(_imuPosAll);
    }
    if (j.contains("nmeaType"))
    {
        j.at("nmeaType").get_to(_nmeaType);
    }
    if (j.contains("startTime"))
    {
        j.at("startTime").get_to(_startTime);
    }
}

bool NAV::MultiImuFile::initialize()
{
    LOG_TRACE("{}: called", nameId());

    _lastFiltObs.reset();

    return FileReader::initialize();
}

void NAV::MultiImuFile::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    FileReader::deinitialize();
}

bool NAV::MultiImuFile::resetNode()
{
    FileReader::resetReader();

    for (auto& sensor : _messages)
    {
        sensor.clear();
    }
    for (auto& cnt : _messageCnt)
    {
        cnt = 0;
    }

    return true;
}

void NAV::MultiImuFile::updateNumberOfOutputPins()
{
    while (outputPins.size() < _nSensors)
    {
        nm::CreateOutputPin(this, fmt::format("ImuObs {}", outputPins.size() + 1).c_str(), Pin::Type::Flow, { NAV::ImuObs::type() }, &MultiImuFile::pollData);
        _imuPosAll.resize(_nSensors);

        _messages.resize(_nSensors);
        _messageCnt.resize(_nSensors);
    }
}

NAV::FileReader::FileType NAV::MultiImuFile::determineFileType()
{
    LOG_TRACE("called");

    auto filepath = getFilepath();

    if (good())
    {
        return FileType::ASCII;
    }

    LOG_ERROR("Could not open file {}", filepath.string());
    return FileType::NONE;
}

void NAV::MultiImuFile::readHeader()
{
    LOG_TRACE("called");

    bool gpzdaFound = false;
    bool gpggaFound = false;
    const char* gpzda = "GPZDA";
    const char* gpgga = "GPGGA";

    std::string line;
    auto len = tellg();

    // Find first line of data
    while (getline(line))
    {
        // Remove any trailing non text characters
        line.erase(std::find_if(line.begin(), line.end(), [](int ch) { return std::iscntrl(ch); }), line.end());

        if ((line.find(gpzda)) != std::string::npos)
        {
            gpzdaFound = true;

            int32_t year{};
            int32_t month{};
            int32_t day{};
            int32_t hour{};
            int32_t min{};
            long double sec{};

            // Convert line into stream
            std::stringstream lineStream(line);
            std::string cell;

            // Split line at comma
            for (const auto& col : _headerColumns)
            {
                if (std::getline(lineStream, cell, ','))
                {
                    // Remove any trailing non text characters
                    cell.erase(std::find_if(cell.begin(), cell.end(), [](int ch) { return std::iscntrl(ch); }), cell.end());
                    while (cell.empty())
                    {
                        std::getline(lineStream, cell, ',');
                    }

                    if (col == "nmeaMsgType")
                    {
                        LOG_DEBUG("{}: nmeaMsgType read.", nameId());
                        continue;
                    }
                    if (col == "UTC_HMS")
                    {
                        hour = std::stoi(cell.substr(0, 2));
                        min = std::stoi(cell.substr(2, 2));
                        sec = std::stold(cell.substr(4, 5));
                        continue;
                    }
                    if (col == "day")
                    {
                        day = std::stoi(cell);
                        continue;
                    }
                    if (col == "month")
                    {
                        month = std::stoi(cell);
                        continue;
                    }
                    if (col == "year")
                    {
                        year = std::stoi(cell);
                    }

                    _startTime = InsTime(year, month, day, hour, min, sec, UTC);

                    len = tellg();
                    continue;
                }
            }
        }
        if (line.find(gpgga) != std::string::npos)
        {
            gpggaFound = true;
            LOG_INFO("{}: Multi-IMU has no absolute time reference due to NMEA message 'GPGGA' instead of 'GPZDA' (used in old version of the Multi-IMU).", nameId());
            len = tellg();
            continue;
        }
        if ((std::find_if(line.begin(), line.begin() + 1, [](int ch) { return std::isdigit(ch); }) != (std::begin(line) + 1)) && (gpggaFound || gpzdaFound))
        {
            LOG_DEBUG("{}: Found first line of data: {}", nameId(), line);
            seekg(len, std::ios_base::beg); // Reser the read cursor, otherwise we skip the first message
            break;
        }
        len = tellg();
    }
    if (gpggaFound && _nmeaType == NmeaType::GPZDA)
    {
        LOG_WARN("{}: NMEA message type was set to 'GPZDA', but the file contains 'GPGGA'. Make sure that the absolute time reference is set correctly.", nameId());
    }
    if (gpzdaFound && _nmeaType == NmeaType::GPGGA)
    {
        LOG_WARN("{}: NMEA message type was set to 'GPGGA', but the file contains 'GPZDA'. The absolute time reference was read from the header, not the GUI input.", nameId());
    }
}

std::shared_ptr<const NAV::NodeData> NAV::MultiImuFile::pollData(size_t pinIdx, bool peek)
{
    std::shared_ptr<ImuObs> obs = nullptr;

    if (!_messages.at(pinIdx).empty()) // Another pin was reading a message for this pin
    {
        obs = _messages.at(pinIdx).begin()->second;
        if (!peek) // When peeking, we leave the message in the buffer, so we can remove it when polling
        {
            _messages.at(pinIdx).erase(_messages.at(pinIdx).begin());
        }
    }
    else
    {
        // Read line
        std::string line;
        while (getline(line))
        {
            // Remove any starting non text characters
            line.erase(line.begin(), std::find_if(line.begin(), line.end(), [](int ch) { return std::isgraph(ch); }));

            if (line.empty())
            {
                continue;
            }

            // Convert line into stream
            std::stringstream lineStream(line);
            std::string cell;

            size_t sensorId{};
            double gpsSecond{};
            double timeNumerator{};
            double timeDenominator{};
            std::optional<double> accelX;
            std::optional<double> accelY;
            std::optional<double> accelZ;
            std::optional<double> gyroX;
            std::optional<double> gyroY;
            std::optional<double> gyroZ;

            // Split line at comma
            for (const auto& col : _columns)
            {
                if (std::getline(lineStream, cell, ' '))
                {
                    // Remove any trailing non text characters
                    cell.erase(std::find_if(cell.begin(), cell.end(), [](int ch) { return std::iscntrl(ch); }), cell.end());
                    while (cell.empty())
                    {
                        std::getline(lineStream, cell, ' ');
                    }

                    if (col == "sensorId")
                    {
                        sensorId = std::stoul(cell); // NOLINT(clang-diagnostic-implicit-int-conversion)
                    }
                    else if (col == "gpsSecond")
                    {
                        gpsSecond = std::stod(cell); // [s]
                        if (_startupGpsSecond == 0)
                        {
                            _startupGpsSecond = gpsSecond;
                        }
                    }
                    else if (col == "timeNumerator")
                    {
                        timeNumerator = std::stod(cell);
                    }
                    else if (col == "timeDenominator")
                    {
                        timeDenominator = std::stod(cell);
                    }
                    else if (col == "accelX")
                    {
                        accelX = 0.001 * std::stod(cell); // [m/s²]
                    }
                    else if (col == "accelY")
                    {
                        accelY = 0.001 * std::stod(cell); // [m/s²]
                    }
                    else if (col == "accelZ")
                    {
                        accelZ = 0.001 * std::stod(cell); // [m/s²]
                    }
                    else if (col == "gyroX")
                    {
                        gyroX = deg2rad(std::stod(cell) / 131); // [deg/s]
                    }
                    else if (col == "gyroY")
                    {
                        gyroY = deg2rad(std::stod(cell)) / 131; // [deg/s]
                    }
                    else if (col == "gyroZ")
                    {
                        gyroZ = deg2rad(std::stod(cell)) / 131; // [deg/s]
                    }
                }
            }

            auto timeStamp = gpsSecond + timeNumerator / timeDenominator - _startupGpsSecond;
            if (!peek)
            {
                LOG_DEBUG("line: {}", line);
                LOG_DEBUG("timeStamp: {}", timeStamp);
            }

            obs = std::make_shared<ImuObs>(_imuPosAll[sensorId - 1]);

            obs->insTime = _startTime + std::chrono::duration<double>(timeStamp);

            if (accelX.has_value() && accelY.has_value() && accelZ.has_value())
            {
                obs->accelUncompXYZ.emplace(accelX.value(), accelY.value(), accelZ.value());
            }
            if (gyroX.has_value() && gyroY.has_value() && gyroZ.has_value())
            {
                obs->gyroUncompXYZ.emplace(gyroX.value(), gyroY.value(), gyroZ.value());
            }

            if (sensorId - 1 != pinIdx)
            {
                // Write message into buffer to process later on correct pin
                _messages.at(sensorId - 1).insert(std::make_pair(obs->insTime, obs));

                continue; // Read next line in file and search for correct sensor
            }
            if (peek)
            {
                // Write message into buffer to process later in poll step
                _messages.at(pinIdx).insert(std::make_pair(obs->insTime, obs));
            }
            break;
        }
    }

    // Calls all the callbacks
    if (obs && !peek)
    {
        _messageCnt.at(pinIdx)++;

        // Detect jumps back in time
        if (pinIdx == 2)
        {
            if (obs->insTime < _lastFiltObs)
            {
                LOG_ERROR("{}: obs->insTime < _lastFiltObs --> {}", nameId(), (obs->insTime - _lastFiltObs).count());
            }
            _lastFiltObs = obs->insTime;
        }

        invokeCallbacks(pinIdx, obs);
    }
    if (obs == nullptr)
    {
        LOG_DEBUG("{}: Finished reading on pinIdx {}. Read a total of {} messages.", nameId(), pinIdx, _messageCnt.at(pinIdx));
    }
    return obs;
}