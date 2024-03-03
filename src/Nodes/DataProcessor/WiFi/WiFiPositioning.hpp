// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file WiFiPositioning.hpp
/// @brief Single Point Positioning (SPP) / Code Phase Positioning // TODO
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-04-29

#pragma once

#include "util/Eigen.hpp"
#include "util/CallbackTimer.hpp"
#include <vector>

#include "internal/Node/Node.hpp"
#include "NodeData/WiFi/WiFiObs.hpp"
#include "NodeData/State/Pos.hpp"

namespace NAV
{
/// @brief Numerically integrates Imu data
class WiFiPositioning : public Node
{
  public:
    /// @brief Default constructor
    WiFiPositioning();
    /// @brief Destructor
    ~WiFiPositioning() override;
    /// @brief Copy constructor
    WiFiPositioning(const WiFiPositioning&) = delete;
    /// @brief Move constructor
    WiFiPositioning(WiFiPositioning&&) = delete;
    /// @brief Copy assignment operator
    WiFiPositioning& operator=(const WiFiPositioning&) = delete;
    /// @brief Move assignment operator
    WiFiPositioning& operator=(WiFiPositioning&&) = delete;

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
    constexpr static size_t INPUT_PORT_INDEX_WIFI_OBS = 0; ///< @brief WiFiObs
    constexpr static size_t OUTPUT_PORT_INDEX_POS = 0;     ///< @brief Pos

    // --------------------------------------------------------------- Gui -----------------------------------------------------------------

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// Amount of wifi input pins
    size_t _nWifiInputPins = 2;

    /// @brief Adds/Deletes Input Pins depending on the variable _nNavInfoPins
    void updateNumberOfInputPins();

    /// @brief Available Frames
    enum class Frame : int
    {
        ECEF, ///< Earth-Centered Earth-Fixed frame
        LLA,  ///< Latitude-Longitude-Altitude frame
        NED,  ///< North-East-Down frame
    };
    /// Frame to calculate the position in
    Frame _frame = Frame::ECEF;

    /// @brief Available Solution Modes
    enum class SolutionMode : int
    {
        LSQ2D, ///< Least Squares in 2D
        LSQ3D, ///< Least Squares in 3D
        KF,    ///< Kalman Filter
    };
    /// Solution Mode
    SolutionMode _solutionMode = SolutionMode::LSQ3D;

    /// Output interval in ms
    int _outputInterval;
    // ------------------------------------------------------------ Algorithm --------------------------------------------------------------

    /// Estimated position in local frame [m]
    Eigen::Vector3d _e_position = Eigen::Vector3d::Zero();

    std::vector<std::string> _deviceMacAddresses;
    std::vector<Eigen::Vector3d> _devicePositions;
    size_t _numOfDevices;

    struct Device
    {
        Eigen::Vector3d position;
        InsTime time;
        double distance;
    };
    std::vector<Device> _devices;

    /// @brief Receive Function for the WiFi Observations
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void recvWiFiObs(InputPin::NodeDataQueue& queue, size_t pinIdx);

    /// Timer object to handle async data requests
    CallbackTimer _timer;

    /// @brief Calculate the position
    static void lsqSolution2d(void* userData);

    /// @brief Calculate the position
    static void lsqSolution3d(void* userData);
};

} // namespace NAV