// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Ln200Sensor.hpp"

#include "internal/gui/widgets/HelpMarker.hpp"
#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "util/Logger.hpp"
#include "util/Time/TimeBase.hpp"
#include "util/Vendor/Litton/Ln200UartSensor.hpp"

#include "NodeData/IMU/ImuObsWDelta.hpp"

NAV::Ln200Sensor::Ln200Sensor()
    : Imu(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _onlyRealTime = true;
    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 479, 146 };

    _selectedBaudrate = baudrate2Selection(Baudrate::BAUDRATE_2000000);

    nm::CreateOutputPin(this, "ImuObs", Pin::Type::Flow, { NAV::ImuObsWDelta::type() });
}

NAV::Ln200Sensor::~Ln200Sensor()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::Ln200Sensor::typeStatic()
{
    return "Ln200Sensor";
}

std::string NAV::Ln200Sensor::type() const
{
    return typeStatic();
}

std::string NAV::Ln200Sensor::category()
{
    return "Data Provider";
}

void NAV::Ln200Sensor::guiConfig()
{
    if (ImGui::InputTextWithHint("SensorPort", "/dev/ttyUSB0", &_sensorPort))
    {
        LOG_DEBUG("{}: SensorPort changed to {}", nameId(), _sensorPort);
        flow::ApplyChanges();
        doDeinitialize();
    }
    ImGui::SameLine();
    gui::widgets::HelpMarker("COM port where the sensor is attached to\n"
                             "- \"COM1\" (Windows format for physical and virtual (USB) serial port)\n"
                             "- \"/dev/ttyS1\" (Linux format for physical serial port)\n"
                             "- \"/dev/ttyUSB0\" (Linux format for virtual (USB) serial port)\n"
                             "- \"/dev/tty.usbserial-FTXXXXXX\" (Mac OS X format for virtual (USB) serial port)\n"
                             "- \"/dev/ttyS0\" (CYGWIN format. Usually the Windows COM port number minus 1. This would connect to COM1)");

    Imu::guiConfig();
}

[[nodiscard]] json NAV::Ln200Sensor::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["UartSensor"] = UartSensor::save();
    j["Imu"] = Imu::save();

    return j;
}

void NAV::Ln200Sensor::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("UartSensor"))
    {
        UartSensor::restore(j.at("UartSensor"));
    }
    if (j.contains("Imu"))
    {
        Imu::restore(j.at("Imu"));
    }
}

bool NAV::Ln200Sensor::resetNode()
{
    return true;
}

bool NAV::Ln200Sensor::initialize()
{
    LOG_TRACE("{}: called", nameId());

    // connect to the sensor
    try
    {
        _sensor->connect(_sensorPort, sensorBaudrate());

        LOG_DEBUG("{} connected on port {} with baudrate {}", nameId(), _sensorPort, sensorBaudrate());
    }
    catch (...)
    {
        LOG_ERROR("{} could not connect", nameId());
        return false;
    }

    _sensor->registerAsyncPacketReceivedHandler(this, binaryAsyncMessageReceived);

    return true;
}

void NAV::Ln200Sensor::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    if (!isInitialized())
    {
        return;
    }

    if (_sensor->isConnected())
    {
        try
        {
            _sensor->unregisterAsyncPacketReceivedHandler();
        }
        catch (...) // NOLINT(bugprone-empty-catch)
        {}
        LOG_TRACE("{}: Disconnecting...", nameId());
        _sensor->disconnect();
        LOG_TRACE("{}: Disconnected", nameId());
    }
}

void NAV::Ln200Sensor::binaryAsyncMessageReceived(void* userData, uart::protocol::Packet& p, size_t /* index */)
{
    auto* lnSensor = static_cast<Ln200Sensor*>(userData);

    if (p.type() == uart::protocol::Packet::Type::TYPE_BINARY)
    {
        auto obs = std::make_shared<ImuObsWDelta>(lnSensor->_imuPos);

        // Lambda for reversing bits in a uint16_t
        auto reverseBits = [](uint16_t& word) {
            word = static_cast<uint16_t>(((word & 0x5555) << 1) | ((word & 0xAAAA) >> 1)); // Swap adjacent bits
            word = static_cast<uint16_t>(((word & 0x3333) << 2) | ((word & 0xCCCC) >> 2)); // Swap pairs of bits
            word = static_cast<uint16_t>(((word & 0x0F0F) << 4) | ((word & 0xF0F0) >> 4)); // Swap nibbles
            word = static_cast<uint16_t>(((word & 0x00FF) << 8) | ((word & 0xFF00) >> 8)); // Swap bytes
        };

        const std::vector<uint8_t>& bytes = p.getRawData();
        std::array<uint16_t, 14> rawWords{};     // Temporary storage for raw words
        std::array<double, 12> processedWords{}; // Storage of processed data words

        // Extract exactly 14 words (each 16 bits in little-endian order)
        for (uint8_t i = 0; i < 14; ++i)
        {
            uint8_t index = i * 2;
            uint16_t word = static_cast<uint16_t>(bytes[index + 1]) | static_cast<uint16_t>(static_cast<uint16_t>(bytes[index]) << 8);
            reverseBits(word);
            rawWords.at(i) = word;
        }

        // TODO: Check internal validity
        //  The last word is a checksum:
        // uint16_t checksum = rawWords[12];

        // TODO: Process IMU status
        // uint16_t imuStatus = rawWords[6];

        // TODO: Implement the conversion of the mux word based on the mux id
        // The 8th word (index 7) is the multiplexer ID and the 9th word (index 8) is mux data
        // uint16_t muxId = rawWords[7];
        // uint16_t muxDataWordRaw = rawWords[8];

        // Convert the mux word based on the mux id
        // double muxDataWord = 0.0;
        // if (muxId == 8 || muxId == 13) // 8: Temperature conversion, 13: Voltage conversion
        // {
        //     muxDataWord = static_cast<double>(muxDataWordRaw) * 0.1;
        // }
        // else // 17: IMU failure and all other cases
        // {
        //     muxDataWord = muxDataWordRaw;
        // }

        // Processing velocity
        for (uint8_t i = 0; i < 3; ++i)
        {
            processedWords.at(i) = static_cast<int16_t>(rawWords.at(i)) * (1.0 / 16384.0);
        }
        // Processing angles
        for (uint8_t i = 3; i < 6; ++i)
        {
            processedWords.at(i) = static_cast<int16_t>(rawWords.at(i)) * (1.0 / 262144.0);
        }
        double dVelX = processedWords[0];
        double dVelY = processedWords[1];
        double dVelZ = processedWords[2];
        double dAngleX = processedWords[3];
        double dAngleY = processedWords[4];
        double dAngleZ = processedWords[5];
        double accelX = processedWords[0] * FREQ;
        double accelY = processedWords[1] * FREQ;
        double accelZ = processedWords[2] * FREQ;
        double angleRateX = processedWords[3] * FREQ;
        double angleRateY = processedWords[4] * FREQ;
        double angleRateZ = processedWords[5] * FREQ;

        obs->dtime = 1.0 / FREQ;
        obs->dvel = { dVelX, dVelY, dVelZ };
        obs->dtheta = { dAngleX, dAngleY, dAngleZ };
        obs->p_acceleration = { accelX, accelY, accelZ };
        obs->p_angularRate = { angleRateX, angleRateY, angleRateZ };

        LOG_DATA("DATA({}): {}", lnSensor->name, obs->temperature.value());

        // Calls all the callbacks
        if (InsTime currentTime = util::time::GetCurrentInsTime();
            !currentTime.empty())
        {
            obs->insTime = currentTime;
        }
        lnSensor->invokeCallbacks(OUTPUT_PORT_INDEX_LN_OBS, obs);
    }
    else if (p.type() == uart::protocol::Packet::Type::TYPE_ASCII)
    {
        LOG_WARN("{}: Received an ASCII Async message: {}", lnSensor->name, p.datastr());
    }
}