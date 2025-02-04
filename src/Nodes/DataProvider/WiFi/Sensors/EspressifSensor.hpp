// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file EspressifSensor.hpp
/// @brief Espressif Sensor Class
/// @author R. Lintz (r-lintz@gmx.de) (master thesis)
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-01-08

#pragma once

#include "internal/Node/Node.hpp"
#include "Nodes/DataProvider/Protocol/UartSensor.hpp"
#include "util/Vendor/Espressif/EspressifUartSensor.hpp"

namespace NAV
{
/// Espressif Sensor Class
class EspressifSensor : public Node, public UartSensor
{
  public:
    /// @brief Default constructor
    EspressifSensor();
    /// @brief Destructor
    ~EspressifSensor() override;
    /// @brief Copy constructor
    EspressifSensor(const EspressifSensor&) = delete;
    /// @brief Move constructor
    EspressifSensor(EspressifSensor&&) = delete;
    /// @brief Copy assignment operator
    EspressifSensor& operator=(const EspressifSensor&) = delete;
    /// @brief Move assignment operator
    EspressifSensor& operator=(EspressifSensor&&) = delete;

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
    constexpr static size_t OUTPUT_PORT_INDEX_WIFI_OBS = 0; ///< @brief Flow (EspressifObs)

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Callback handler for notifications of new asynchronous data packets received
    /// @param[in, out] userData Pointer to the data we supplied when we called registerAsyncPacketReceivedHandler
    /// @param[in] p Encapsulation of the data packet. At this state, it has already been validated and identified as an asynchronous data message
    /// @param[in] index Advanced usage item and can be safely ignored for now
    static void binaryAsyncMessageReceived(void* userData, uart::protocol::Packet& p, size_t index);

    /// Sensor Object
    vendor::espressif::EspressifUartSensor _sensor;
};

} // namespace NAV