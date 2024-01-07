// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file ArubaSensor.hpp
/// @brief Aruba Sensor Class
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-03-19

#pragma once

#include "internal/Node/Node.hpp"

#include "util/CallbackTimer.hpp"
#include <libssh/libssh.h>

namespace NAV
{
/// Aruba Sensor Class
class ArubaSensor : public Node
{
  public:
    /// @brief Default constructor
    ArubaSensor();
    /// @brief Destructor
    ~ArubaSensor() override;
    /// @brief Copy constructor
    ArubaSensor(const ArubaSensor&) = delete;
    /// @brief Move constructor
    ArubaSensor(ArubaSensor&&) = delete;
    /// @brief Copy assignment operator
    ArubaSensor& operator=(const ArubaSensor&) = delete;
    /// @brief Move assignment operator
    ArubaSensor& operator=(ArubaSensor&&) = delete;

    /// @brief String representation of the Class Type
    [[nodiscard]] static std::string typeStatic();

    /// @brief String representation of the Class Type
    [[nodiscard]] std::string type() const override;

    /// @brief String representation of the Class Category
    [[nodiscard]] static std::string category();

    /// @brief ImGui config window which is shown on double click
    /// @attention Don't forget to set _hasConfig to true in the constructor of the node
    void guiConfig() override;

    /// @brief Saves the node into a json object
    [[nodiscard]] json save() const override;

    /// @brief Restores the node from a json object
    /// @param[in] j Json object with the node state
    void restore(const json& j) override;

    /// @brief Resets the node. It is guaranteed that the node is initialized when this is called.
    bool resetNode() override;

  private:
    constexpr static size_t OUTPUT_PORT_INDEX_Aruba_OBS = 0; ///< @brief Flow (ArubaObs)

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Function which performs the async data reading
    /// @param[in] userData Pointer to the object
    static void readSensorDataThread(void* userData);

    ssh_channel _channel; ///< @brief SSH channel
    ssh_session _session; ///< @brief SSH session

    /// Timer object to handle async data requests
    CallbackTimer _timer;

    /// Ssh options
    std::string _sshHost;
    std::string _sshUser;
    std::string _sshHostkeys;
    std::string _sshKeyExchange;
    std::string _sshPublickeyAcceptedTypes;

    /// Output interval in ms
    int _outputInterval;
};

} // namespace NAV