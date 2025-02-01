// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file UartPacketConverter.hpp
/// @brief Decrypts Uart packets
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @author R. Lintz (r-lintz@gmx.de) (master thesis)
/// @date 2022-06-13

#pragma once

#include <cstdint>
#include "internal/Node/Node.hpp"

namespace NAV
{
/// Decrypts Uart packets
class UartPacketConverter : public Node
{
  public:
    /// @brief Default constructor
    UartPacketConverter();
    /// @brief Destructor
    ~UartPacketConverter() override;
    /// @brief Copy constructor
    UartPacketConverter(const UartPacketConverter&) = delete;
    /// @brief Move constructor
    UartPacketConverter(UartPacketConverter&&) = delete;
    /// @brief Copy assignment operator
    UartPacketConverter& operator=(const UartPacketConverter&) = delete;
    /// @brief Move assignment operator
    UartPacketConverter& operator=(UartPacketConverter&&) = delete;

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

  private:
    constexpr static size_t OUTPUT_PORT_INDEX_CONVERTED = 0;  ///< @brief Flow
    constexpr static size_t INPUT_PORT_INDEX_UART_PACKET = 0; ///< @brief Flow (UartPacket)
    constexpr static size_t INPUT_PORT_INDEX_SYNC_IN = 1;     ///< @brief Flow (SyncIn)

    /// Enum specifying the type of the output message
    enum OutputType : uint8_t
    {
        OutputType_UbloxObs, ///< Extract UbloxObs data
        OutputType_EmlidObs, ///< Extract EmlidObs data
        OutputType_WiFiObs,  ///< Extract WiFiObs data
    };

    /// The selected output type in the GUI
    OutputType _outputType = OutputType_UbloxObs;

    /// Show the SyncIn Pin
    bool _syncInPin = false;

    /// Last received syncInCnt
    int64_t _lastSyncInCnt = 0;

    /// Last received syncOutCnt
    int64_t _lastSyncOutCnt = 0;

    /// Corrected SyncOut counter in case of a reset (initiator)
    int64_t _syncOutCntCorr = 0;

    /// Corrected SyncIn counter in case of a reset (target)
    int64_t _syncInCntCorr = 0;

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Converts the UartPacket to the selected message type
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void receiveObs(InputPin::NodeDataQueue& queue, size_t pinIdx);
};

} // namespace NAV
