// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file TimeWindow.hpp
/// @brief Limits measurement data from any source to a user-defined timewindow
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2022-10-27

#pragma once

#include "internal/Node/Node.hpp"

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

    /// @brief Initialize the node
    bool initialize() override;

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

    /// Selected time format in GUI
    TimeFormats _timeFormat = TimeFormats::YMDHMS;

    // --------------------------------------- Beginning of time window ------------------------------------------

    /// @brief Number of days at beginning of time window (Julien Date and Modified Julien Date)
    int32_t _daysStart{};
    /// @brief Decimal fraction of a day at beginning of time window (Julien Date and Modified Julien Date)
    double _decFracStart{};

    /// @brief GPS cycle at beginning of time window
    int32_t _gpsCycleStart{};
    /// @brief GPS week at beginning of time window
    int32_t _gpsWeekStart{};
    /// @brief GPS Time of Week (in sec) at beginning of time window
    double _gpsTowStart{};

    /// @brief Year at beginning of time window
    int32_t _yearStart{};
    /// @brief Month at beginning of time window
    int32_t _monthStart{};
    /// @brief Day at beginning of time window
    int32_t _dayStart{};
    /// @brief Hour at beginning of time window
    int32_t _hourStart{};
    /// @brief Minute at beginning of time window
    int32_t _minStart{};
    /// @brief Second at beginning of time window
    double _secStart{};

    // ------------------------------------------ End of time window ---------------------------------------------

    /// @brief Number of days at end of time window (Julien Date and Modified Julien Date)
    int32_t _daysEnd{};
    /// @brief Decimal fraction of a day at end of time window (Julien Date and Modified Julien Date)
    double _decFracEnd{};

    /// @brief GPS cycle at end of time window
    int32_t _gpsCycleEnd{};
    /// @brief GPS week at end of time window
    int32_t _gpsWeekEnd{};
    /// @brief GPS Time of Week (in sec) at end of time window
    double _gpsTowEnd{};

    /// @brief Year at end of time window
    int32_t _yearEnd{};
    /// @brief Month at end of time window
    int32_t _monthEnd{};
    /// @brief Day at end of time window
    int32_t _dayEnd{};
    /// @brief Hour at end of time window
    int32_t _hourEnd{};
    /// @brief Minute at end of time window
    int32_t _minEnd{};
    /// @brief Second at end of time window
    double _secEnd{};
};
} // namespace NAV
