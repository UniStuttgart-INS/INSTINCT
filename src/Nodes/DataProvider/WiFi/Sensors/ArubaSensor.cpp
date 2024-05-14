// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ArubaSensor.hpp"

#include "util/Time/TimeBase.hpp"
#include "util/Logger.hpp"
#include <libssh/libssh.h>
#include <regex>
#include <sstream>

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeData/WiFi/WiFiObs.hpp"

/// Speed of light in air [m/s]
constexpr double cAir = 299702547.0;

NAV::ArubaSensor::ArubaSensor()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _onlyRealTime = true;
    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 710, 220 };

    _sshHost = "192.168.178.45";
    _sshUser = "admin";
    _sshPassword = "admin1";
    _sshHostkeys = "ssh-rsa";
    _sshKeyExchange = "ecdh-sha2-nistp256";
    _sshPublickeyAcceptedTypes = "ssh-rsa";
    _outputInterval = 3000;

    nm::CreateOutputPin(this, "WiFiObs", Pin::Type::Flow, { NAV::WiFiObs::type() });
}

NAV::ArubaSensor::~ArubaSensor()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::ArubaSensor::typeStatic()
{
    return "ArubaSensor";
}

std::string NAV::ArubaSensor::type() const
{
    return typeStatic();
}

std::string NAV::ArubaSensor::category()
{
    return "Data Provider";
}

void NAV::ArubaSensor::guiConfig()
{
    if (ImGui::InputTextWithHint("SSH Host", "192.168.0.0", &_sshHost))
    {
        LOG_DEBUG("{}: ssh host changed to {}", nameId(), _sshHost);
        flow::ApplyChanges();
        doDeinitialize();
    }
    if (ImGui::InputTextWithHint("SSH User", "admin", &_sshUser))
    {
        LOG_DEBUG("{}: ssh admin changed to {}", nameId(), _sshUser);
        flow::ApplyChanges();
        doDeinitialize();
    }
    if (ImGui::InputText("SSH Password", &_sshPassword))
    {
        LOG_DEBUG("{}: ssh password changed to {}", nameId(), _sshPassword);
        flow::ApplyChanges();
        doDeinitialize();
    }
    if (ImGui::InputText("SSH Host Keys", &_sshHostkeys))
    {
        LOG_DEBUG("{}: ssh host keys changed to {}", nameId(), _sshHostkeys);
        flow::ApplyChanges();
        doDeinitialize();
    }
    if (ImGui::InputText("SSH Key Exchange", &_sshKeyExchange))
    {
        LOG_DEBUG("{}: ssh key exchange changed to {}", nameId(), _sshKeyExchange);
        flow::ApplyChanges();
        doDeinitialize();
    }
    if (ImGui::InputText("SSH Publickey Accepted Types", &_sshPublickeyAcceptedTypes))
    {
        LOG_DEBUG("{}: ssh publickey accepted types changed to {}", nameId(), _sshPublickeyAcceptedTypes);
        flow::ApplyChanges();
        doDeinitialize();
    }
    ImGui::Spacing();   // Add spacing here
    ImGui::Separator(); // Add a horizontal line
    if (ImGui::InputInt("Output interval [ms]", &_outputInterval))
    {
        LOG_DEBUG("{}: output interval changed to {}", nameId(), _outputInterval);
        flow::ApplyChanges();
    }
}

[[nodiscard]] json NAV::ArubaSensor::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["sshHost"] = _sshHost;
    j["sshUser"] = _sshUser;
    j["sshPassword"] = _sshPassword;
    j["sshHostkeys"] = _sshHostkeys;
    j["sshKeyExchange"] = _sshKeyExchange;
    j["sshPublickeyAcceptedTypes"] = _sshPublickeyAcceptedTypes;
    j["outputInterval"] = _outputInterval;

    return j;
}

void NAV::ArubaSensor::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("sshHost"))
    {
        j.at("sshHost").get_to(_sshHost);
    }
    if (j.contains("sshUser"))
    {
        j.at("sshUser").get_to(_sshUser);
    }
    if (j.contains("sshPassword"))
    {
        j.at("sshPassword").get_to(_sshPassword);
    }
    if (j.contains("sshHostkeys"))
    {
        j.at("sshHostkeys").get_to(_sshHostkeys);
    }
    if (j.contains("sshKeyExchange"))
    {
        j.at("sshKeyExchange").get_to(_sshKeyExchange);
    }
    if (j.contains("sshPublickeyAcceptedTypes"))
    {
        j.at("sshPublickeyAcceptedTypes").get_to(_sshPublickeyAcceptedTypes);
    }
    if (j.contains("outputInterval"))
    {
        j.at("outputInterval").get_to(_outputInterval);
    }
}

bool NAV::ArubaSensor::resetNode()
{
    return true;
}

bool NAV::ArubaSensor::initialize()
{
    LOG_TRACE("{}: called", nameId());

    // SSH session
    _session = ssh_new();
    if (_session == NULL)
    {
        LOG_INFO("{}: Failed to create SSH _session", nameId());
        return false;
    }
    LOG_DEBUG("{}: Successfully created SSH _session", nameId());

    ssh_options_set(_session, SSH_OPTIONS_HOST, "192.168.178.45");
    ssh_options_set(_session, SSH_OPTIONS_USER, "admin");
    ssh_options_set(_session, SSH_OPTIONS_HOSTKEYS, "ssh-rsa");
    ssh_options_set(_session, SSH_OPTIONS_KEY_EXCHANGE, "ecdh-sha2-nistp256");
    ssh_options_set(_session, SSH_OPTIONS_PUBLICKEY_ACCEPTED_TYPES, "ssh-rsa");

    // connect
    if (ssh_connect(_session) != SSH_OK)
    {
        LOG_INFO("{}: Failed to connect to the router", nameId());
        ssh_free(_session);
        return false;
    }
    LOG_DEBUG("{}: Successfully connected to the router", nameId());

    // authenticate
    if (ssh_userauth_password(_session, NULL, _sshPassword.c_str()) != SSH_AUTH_SUCCESS)
    {
        LOG_INFO("{}: Authentication failed", nameId());
        ssh_disconnect(_session);
        ssh_free(_session);
        return false;
    }
    LOG_DEBUG("{}: Authentication succeeded", nameId());

    // channel
    _channel = ssh_channel_new(_session);
    if (_channel == NULL)
    {
        LOG_INFO("{}: Failed to create SSH channel", nameId());
        ssh_disconnect(_session);
        ssh_free(_session);
        return false;
    }
    LOG_DEBUG("{}: Successfully created SSH channel", nameId());

    // open _session
    if (ssh_channel_open_session(_channel) != SSH_OK)
    {
        LOG_INFO("{}: Failed to open an SSH _session", nameId());
        ssh_channel_free(_channel);
        ssh_disconnect(_session);
        ssh_free(_session);
        return false;
    }
    LOG_DEBUG("{}: Successfully opened an SSH _session", nameId());

    // pty
    if (ssh_channel_request_pty(_channel) != SSH_OK)
    {
        LOG_INFO("{}: Failed to open pty", nameId());
        ssh_channel_free(_channel);
        ssh_disconnect(_session);
        ssh_free(_session);
        return false;
    }
    LOG_DEBUG("{}: Successfully opened pty", nameId());

    // shell
    if (ssh_channel_request_shell(_channel) != SSH_OK)
    {
        LOG_INFO("{}: Failed to open shell", nameId());
        ssh_channel_free(_channel);
        ssh_disconnect(_session);
        ssh_free(_session);
        return false;
    }
    LOG_DEBUG("{}: Successfully opened shell", nameId());

    _timer.start(_outputInterval, readSensorDataThread, this);

    return true;
}

void NAV::ArubaSensor::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    if (!isInitialized())
    {
        return;
    }

    ssh_channel_write(_channel, "exit\n", strlen("exit\n"));
    ssh_channel_free(_channel);
    ssh_disconnect(_session);
    ssh_free(_session);

    if (_timer.is_running())
    {
        _timer.stop();
    }

    // To Show the Deinitialization in the GUI
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

void NAV::ArubaSensor::readSensorDataThread(void* userData)
{
    auto* node = static_cast<ArubaSensor*>(userData);
    auto obs = std::make_shared<WiFiObs>();

    char buffer[1024];
    std::string receivedData;
    size_t bytesRead;
    // Send command to access point
    ssh_channel_write(node->_channel, "show ap range scanning-results\n", strlen("show ap range scanning-results\n"));
    // Read output
    do {
        bytesRead = static_cast<size_t>(ssh_channel_read_timeout(node->_channel, buffer, sizeof(buffer), 0, 10)); // timeout in ms
        if (bytesRead > 0)
        {
            receivedData.append(buffer, bytesRead);
        }
    } while (bytesRead > 0);
    // Send command to clear output
    ssh_channel_write(node->_channel, "clear range-scanning-result\n", strlen("clear range-scanning-result\n"));

    // Parse the received data
    std::istringstream iss(receivedData);
    std::string line;

    while (std::getline(iss, line) && line.find("Peer-bssid") == std::string::npos); // Skip lines until the header "Peer-bssid" is found

    // Skip the header lines
    std::getline(iss, line);
    std::getline(iss, line);

    // MAC address validation
    std::regex macRegex("^([0-9A-Fa-f]{2}[:-]){5}([0-9A-Fa-f]{2})$");

    while (std::getline(iss, line) && !line.empty())
    {
        std::istringstream lineStream(line);
        std::string value;
        std::string macAddress;
        lineStream >> macAddress;

        int rtt, rssi, stdValue;
        lineStream >> rtt >> rssi >> stdValue;

        std::regex timeRegex("\\d{4}-\\d{2}-\\d{2}"); // Time format: YYYY-MM-DD
        std::string timeStamp1, timeStamp2;
        lineStream >> timeStamp1;
        lineStream >> timeStamp2;
        while (lineStream >> value)
        {
            if (std::regex_match(value, timeRegex)) // Check if the value is a time stamp
            {
                timeStamp1 = value;
                break;
            }
        }
        lineStream >> value;
        timeStamp2 = value;
        double measuredDistance = static_cast<double>(rtt) * 1e-9 / 2 * cAir;
        double measuredDistanceStd = static_cast<double>(stdValue) * 1e-9 / 2 * cAir;
        if (std::regex_match(macAddress, macRegex)) // Check if the MAC address is valid
        {
            InsTime_YMDHMS yearMonthDayHMS(std::stoi(timeStamp1.substr(0, 4)), std::stoi(timeStamp1.substr(5, 2)), std::stoi(timeStamp1.substr(8, 2)), std::stoi(timeStamp2.substr(0, 2)), std::stoi(timeStamp2.substr(3, 2)), std::stoi(timeStamp2.substr(6, 2)));
            InsTime timeOfMeasurement(yearMonthDayHMS, UTC);
            std::transform(macAddress.begin(), macAddress.end(), macAddress.begin(), ::toupper); // Convert to uppercase
            obs->distance = measuredDistance;
            obs->distanceStd = measuredDistanceStd;
            obs->macAddress = macAddress;
            obs->insTime = timeOfMeasurement;
            // obs->insTime = util::time::GetCurrentInsTime(); // if you want to use the instinct time instead
            node->invokeCallbacks(OUTPUT_PORT_INDEX_WIFI_OBS, obs);
        }
    }
}