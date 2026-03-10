// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "SpirentSimSENSORFile.hpp"

#include "internal/FlowManager.hpp"
#include "internal/gui/NodeEditorApplication.hpp"
#include "util/Logger.hpp"

#include "Navigation/Time/InsTime.hpp"
#include "NodeData/IMU/ImuObsWDelta.hpp"

NAV::SpirentSimSENSORFile::SpirentSimSENSORFile()
    : Imu(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 479, 226 };

    CreateOutputPin("ImuObs", Pin::Type::Flow, { NAV::ImuObsWDelta::type() }, &SpirentSimSENSORFile::pollData);
}

NAV::SpirentSimSENSORFile::~SpirentSimSENSORFile()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::SpirentSimSENSORFile::typeStatic()
{
    return "SpirentSimSENSORFile";
}

std::string NAV::SpirentSimSENSORFile::type() const
{
    return typeStatic();
}

std::string NAV::SpirentSimSENSORFile::category()
{
    return "Data Provider";
}

void NAV::SpirentSimSENSORFile::guiConfig()
{
    if (auto res = FileReader::guiConfig(".bin,.*", { ".bin" }, size_t(id), nameId()))
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

    ImGui::SetNextItemOpen(true, ImGuiCond_Always);
    if (ImGui::TreeNode(fmt::format("Simulation start time##{}", size_t(id)).c_str()))
    {
        if (gui::widgets::TimeEdit(fmt::format("Simulation start##{}", size_t(id)).c_str(),
                                   _startTime, _startTimeFormat,
                                   130.0F * gui::NodeEditorApplication::windowFontRatio(), 2))
        {
            flow::ApplyChanges();
        }
        ImGui::TreePop();
    }

    Imu::guiConfig();
}

[[nodiscard]] json NAV::SpirentSimSENSORFile::save() const
{
    LOG_TRACE("{}: called", nameId());

    return {
        { "FileReader", FileReader::save() },
        { "Imu", Imu::save() },
        { "startTime", _startTime },
        { "startTimeFormat", _startTimeFormat }

    };
}

void NAV::SpirentSimSENSORFile::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("FileReader")) { FileReader::restore(j.at("FileReader")); }
    if (j.contains("Imu")) { Imu::restore(j.at("Imu")); }
    if (j.contains("startTime")) { j.at("startTime").get_to(_startTime); }
    if (j.contains("startTimeFormat")) { j.at("startTimeFormat").get_to(_startTimeFormat); }
}

bool NAV::SpirentSimSENSORFile::initialize()
{
    LOG_TRACE("{}: called", nameId());

    return FileReader::initialize();
}

void NAV::SpirentSimSENSORFile::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    FileReader::deinitialize();
}

bool NAV::SpirentSimSENSORFile::resetNode()
{
    FileReader::resetReader();

    _firstEpoch.reset();
    _lastEpoch.reset();
    _msgCount = 0;
    _dtMin = std::numeric_limits<double>::infinity();

    return true;
}

std::shared_ptr<const NAV::NodeData> NAV::SpirentSimSENSORFile::pollData()
{
    std::array<char, MESSAGE_SIZE> buffer{};

    while (!eof() && good() && read(buffer.data(), MESSAGE_SIZE))
    {
        Message message{};
        std::memcpy(&message, buffer.data(), sizeof(message));

        if (message.type != Message::Type::simsensor_data)
        {
            LOG_DATA("{}: [{}] Message type is {}, but need SimSENSOR ({}). Reading next message.", nameId(), _msgCount,
                     fmt::underlying(message.type), fmt::underlying(Message::Type::simsensor_data));
            // LOG_DEBUG(R"({}: [{}]
            //   time_into_run = {} [µs]
            //   time_of_validity = {} [µs]
            //   status = {}
            //   simulation_update_rate = {} [ms]
            //   UDP_output_rate_ms = {} [ms]
            //   serial_poll_byte = {})",
            //           nameId(), _msgCount,
            //           message.time_into_run,
            //           message.time_of_validity,
            //           fmt::underlying(message.data.status_info.status),
            //           message.data.status_info.simulation_update_rate,
            //           message.data.status_info.UDP_output_rate_ms,
            //           message.data.status_info.serial_poll_byte);
            continue;
        }
        if (message.version_major != ETHERNET_SHARE_VERSION_MAJOR || message.version_minor > ETHERNET_SHARE_VERSION_MINOR)
        {
            LOG_ERROR("{}: [{}] Message version must be {}.{}, but the file has {}.{}. Reading next message.", nameId(), _msgCount,
                      ETHERNET_SHARE_VERSION_MAJOR, ETHERNET_SHARE_VERSION_MINOR,
                      message.version_major, message.version_minor);
            continue;
        }
        _msgCount++;

        InsTime currentEpoch = _startTime + std::chrono::duration<double>(message.data.simsensor.time_of_validity_s);
        if (!_lastEpoch.empty())
        {
            auto dt = static_cast<double>((currentEpoch - _lastEpoch).count());
            _dtMin = std::min(dt, _dtMin);
        }
        _lastEpoch = currentEpoch;

        // LOG_DATA(R"({}: [{}]
        //       time_into_run = {} [µs]
        //       time_of_validity = {} [µs]
        //       vehicle: id = {}, type = {}
        //       model_number = {}
        //       spare = {}
        //       time_of_validity_s = {}
        //       avg_acceleration = {}
        //       avg_rate = {}
        //       magnetic_flux_density_uT = {}
        //       _dtMin = {}
        //       delta_theta = {}
        //       delta_velocity = {})",
        //           nameId(), _msgCount,
        //           message.time_into_run,
        //           message.time_of_validity,
        //           message.data.simsensor.vehicle_id.id,
        //           fmt::underlying(message.data.simsensor.vehicle_id.type),
        //           message.data.simsensor.model_number,
        //           message.data.simsensor.spare,
        //           message.data.simsensor.time_of_validity_s,
        //           Eigen::Map<Eigen::Vector3d>(message.data.simsensor.accelerometer.avg_acceleration.data()).transpose(),
        //           Eigen::Map<Eigen::Vector3d>(message.data.simsensor.gyro.avg_rate.data()).transpose(),
        //           Eigen::Map<Eigen::Vector3d>(message.data.simsensor.magnetometer.magnetic_flux_density_uT.data()).transpose() / 100.0,
        //           _dtMin,
        //           Eigen::Map<Eigen::Vector3d>(message.data.simsensor.gyro.delta_theta.data()).transpose(),
        //           Eigen::Map<Eigen::Vector3d>(message.data.simsensor.accelerometer.delta_velocity.data()).transpose());

        auto obs = std::make_shared<ImuObsWDelta>(_imuPos);
        obs->insTime = currentEpoch;

        obs->p_acceleration = Eigen::Map<Eigen::Vector3d>(message.data.simsensor.accelerometer.avg_acceleration.data());
        obs->p_angularRate = Eigen::Map<Eigen::Vector3d>(message.data.simsensor.gyro.avg_rate.data());
        obs->p_magneticField = Eigen::Map<Eigen::Vector3d>(message.data.simsensor.magnetometer.magnetic_flux_density_uT.data()) / 100.0; // [µT] -> [Gauss]
        if (!std::isinf(_dtMin))
        {
            obs->dtime = _dtMin;
        }
        obs->dtheta = Eigen::Map<Eigen::Vector3d>(message.data.simsensor.gyro.delta_theta.data());
        obs->dvel = Eigen::Map<Eigen::Vector3d>(message.data.simsensor.accelerometer.delta_velocity.data());

        if (message.data.simsensor.time_of_validity_s == 0.0
            && obs->p_acceleration.isZero() && obs->p_angularRate.isZero() && obs->p_magneticField->isZero()
            && obs->dtheta.isZero() && obs->dvel.isZero())
        {
            LOG_DEBUG("{}: Skipping first message, as empty", nameId());
            _firstEpoch = currentEpoch;
            continue;
        }

        if (!_firstEpoch.empty())
        {
            auto obsFirstEpoch = std::make_shared<ImuObsWDelta>(*obs);
            obsFirstEpoch->insTime = _firstEpoch;
            _firstEpoch.reset();
            LOG_DEBUG("{}: Duplicating second message and sending out at epoch of first message", nameId());
            invokeCallbacks(OUTPUT_PORT_INDEX_IMU_OBS, obsFirstEpoch);
        }

        invokeCallbacks(OUTPUT_PORT_INDEX_IMU_OBS, obs);
        return obs;
    }

    LOG_DEBUG("{}: File finished after {} SimSENSOR messages.", nameId(), _msgCount);
    return nullptr;
}

NAV::FileReader::FileType NAV::SpirentSimSENSORFile::determineFileType()
{
    LOG_TRACE("called for {}", nameId());

    auto filestream = std::ifstream(getFilepath(), std::ios_base::in | std::ios_base::binary);

    if (!filestream.good())
    {
        LOG_ERROR("{}: Could not open file {}", nameId(), getFilepath());
        return FileType::NONE;
    }

    std::array<char, MESSAGE_SIZE> buffer{};
    filestream.read(buffer.data(), MESSAGE_SIZE);
    Message message{};
    std::memcpy(&message, buffer.data(), sizeof(message));

    if (message.version_major != ETHERNET_SHARE_VERSION_MAJOR
        || message.version_minor != ETHERNET_SHARE_VERSION_MINOR)
    {
        LOG_ERROR("{}: Message version must be {}.{}, but the file has {}.{}", nameId(),
                  ETHERNET_SHARE_VERSION_MAJOR, ETHERNET_SHARE_VERSION_MINOR,
                  message.version_major, message.version_minor);
        return FileType::NONE;
    }
    if (message.type != Message::Type::status)
    {
        LOG_ERROR("{}: First message must be of type status({}), but got {}.", nameId(),
                  fmt::underlying(Message::Type::status), fmt::underlying(message.type));
        return FileType::NONE;
    }

    filestream.read(buffer.data(), MESSAGE_SIZE);
    std::memcpy(&message, buffer.data(), sizeof(message));
    if (message.type != Message::Type::simsensor_data)
    {
        LOG_ERROR("{}: Second message must be of type simsensor({}), but got {}.", nameId(),
                  fmt::underlying(Message::Type::simsensor_data), fmt::underlying(message.type));
        return FileType::NONE;
    }

    LOG_DEBUG("{}: File successfully opened.", nameId());
    return FileType::BINARY;
}
