/// This file is part of INSTINCT, the INS Toolkit for Integrated
/// Navigation Concepts and Training by the Institute of Navigation of
/// the University of Stuttgart, Germany.
///
/// This Source Code Form is subject to the terms of the Mozilla Public
/// License, v. 2.0. If a copy of the MPL was not distributed with this
/// file, You can obtain one at https://mozilla.org/MPL/2.0/.
///
/// <-- KEEP ONE EMPTY LINE HERE AND MAKE LICENSE COMMENTS only 2 slashes '//'
///
/// @file TimeWindow.hpp
/// @brief Limits measurement data from any source to a user-defined timewindow
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2022-10-27

#pragma once

#include "internal/Node/Node.hpp"

#include "NodeData/IMU/ImuObs.hpp"
#include "NodeData/State/PosVelAtt.hpp"

#include "Navigation/Time/InsTime.hpp"

#include <Eigen/Core>

namespace NAV
{
/// Limits measurement data from any source to a user-defined timewindow
class TimeWindow : public Node
{
  public:
    /// @brief Default constructor
    TimeWindow();
    /// @brief Destructor
    ~TimeWindow() override;
    /// @brief Copy Constructor
    TimeWindow(const TimeWindow&) = delete;
    /// @brief Move Constructor
    TimeWindow(TimeWindow&&) = delete;
    /// @brief Copy assignment operator
    TimeWindow& operator=(const TimeWindow&) = delete;
    /// @brief Move assignment operator
    TimeWindow& operator=(TimeWindow&&) = delete;

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
    constexpr static size_t OUTPUT_PORT_INDEX_FLOW = 0; ///< @brief Flow
    constexpr static size_t INPUT_PORT_INDEX_FLOW = 0;  ///< @brief Flow

    /// @brief Resets the node. It is guaranteed that the node is initialized when this is called.
    bool resetNode() override;

    /// @brief Called when a new link was established
    /// @param[in] startPin Pin where the link starts
    /// @param[in] endPin Pin where the link ends
    void afterCreateLink(OutputPin& startPin, InputPin& endPin) override;

    /// @brief Called when a link was deleted
    /// @param[in] startPin Pin where the link starts
    /// @param[in] endPin Pin where the link ends
    void afterDeleteLink(OutputPin& startPin, InputPin& endPin) override;

    /// @brief Callback when receiving data on a port
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void receiveObs(InputPin::NodeDataQueue& queue, size_t pinIdx);

    /// @brief Callback when receiving an ImuObs
    /// @param[in] imuObs Copied data to modify and send out again
    void receiveImuObs(const std::shared_ptr<ImuObs>& imuObs);

    // TODO: Add 'GnssObs'?
    //  /// @brief Callback when receiving an ImuObs
    //  /// @param[in] gnssObs Copied data to modify and send out again
    //  void receiveGnssObs(const std::shared_ptr<GnssObs>& gnssObs);

    /// @brief Callback when receiving an ImuObs
    /// @param[in] posVelAtt Copied data to modify and send out again
    void receivePosVelAtt(const std::shared_ptr<PosVelAtt>& posVelAtt);

    /// @brief Beginning of time window
    InsTime _startTime;

    /// @brief End of time window
    InsTime _endTime;

    /// @brief Possible units to specify a time
    enum class TimeFormats : int
    {
        MJD,    ///< full days, decimal fractions
        JD,     ///< full days, decimal fractions
        GPST,   ///< gpsCycle, gpsWeek, tow
        YMDHMS, ///< year, month, day, hour, min, sec
    };

    /// Selected unit for the accelerometer bias in the GUI
    TimeFormats _timeFormat = TimeFormats::YMDHMS;

    int32_t _days{};
    double _decFrac{};

    int32_t _gpsCycle{};
    int32_t _gpsWeek{};
    double _gpsTow{};

    int32_t _year{};
    int32_t _month{};
    int32_t _day{};
    int32_t _hour{};
    int32_t _min{};
    double _sec{};

    InsTime_MJD _mjdStart{ 0, 0.0 };
    InsTime_MJD _mjdEnd{ 0, 0.0 };
    InsTime_JD _jdStart{ 0, 0.0 };
    InsTime_JD _jdEnd{ 0, 0.0 };
    InsTime_YMDHMS _ymdhmsStart{ 0, 0, 0, 0, 0, 0.0 };
    InsTime_GPSweekTow _gpsWeekTowStart{ 0, 0, 0.0 };
    InsTime_YMDHMS _ymdhmsEnd{ 0, 0, 0, 0, 0, 0.0 };
    InsTime_GPSweekTow _gpsWeekTowEnd{ 0, 0, 0.0 };
};
} // namespace NAV
