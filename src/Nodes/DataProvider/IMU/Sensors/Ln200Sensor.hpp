// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Ln200Sensor.hpp
/// @brief Litton LN-200 IMU Sensor
/// @author M. Senger (st166640@stud.uni-stuttgart.de)
/// @date 2025-03-18

#pragma once

#include "Nodes/DataProvider/IMU/Imu.hpp"
#include "Nodes/DataProvider/Protocol/UartSensor.hpp"
#include "util/Vendor/Litton/Ln200UartSensor.hpp"

namespace NAV
{
/// LN-200 Sensor Class
class Ln200Sensor : public Imu, public UartSensor
{
  public:
    /// @brief Default constructor
    Ln200Sensor();
    /// @brief Destructor
    ~Ln200Sensor() override;
    /// @brief Copy constructor
    Ln200Sensor(const Ln200Sensor&) = delete;
    /// @brief Move constructor
    Ln200Sensor(Ln200Sensor&&) = delete;
    /// @brief Copy assignment operator
    Ln200Sensor& operator=(const Ln200Sensor&) = delete;
    /// @brief Move assignment operator
    Ln200Sensor& operator=(Ln200Sensor&&) = delete;

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
    constexpr static size_t OUTPUT_PORT_INDEX_LN_OBS = 0; ///< @brief Flow (ImuObs)

    /// IMU data frequency in Hz.
    constexpr static uint8_t FREQ = 200;

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Callback handler for notifications of new asynchronous data packets received
    /// @param[in] userData Pointer to the data we supplied when we called registerAsyncPacketReceivedHandler
    /// @param[in] p Encapsulation of the data packet. At this state, it has already been validated and identified as an asynchronous data message
    /// @param[in] index Advanced usage item and can be safely ignored for now
    static void binaryAsyncMessageReceived(void* userData, uart::protocol::Packet& p, size_t index);

    /// Sensor Object
    vendor::ln::Ln200UartSensor _sensor;
};

} // namespace NAV