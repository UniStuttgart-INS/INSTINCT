// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file LowPassFilter.hpp
/// @brief Adds errors (biases and noise) to measurements
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-12-21

#pragma once

#include "internal/Node/Node.hpp"

#include "NodeData/GNSS/GnssObs.hpp"
#include "NodeData/IMU/ImuObs.hpp"
#include "NodeData/IMU/ImuObsWDelta.hpp"
#include "NodeData/State/PosVelAtt.hpp"
#include "Navigation/INS/Units.hpp"

#include "util/Random/RandomNumberGenerator.hpp"

#include "util/Eigen.hpp"
#include <random>
#include <map>

namespace NAV
{
/// Adds errors (biases and noise) to measurements
class LowPassFilter : public Node
{
  public:
    /// @brief Default constructor
    LowPassFilter();
    /// @brief Destructor
    ~LowPassFilter() override;
    /// @brief Copy constructor
    LowPassFilter(const LowPassFilter&) = delete;
    /// @brief Move constructor
    LowPassFilter(LowPassFilter&&) = delete;
    /// @brief Copy assignment operator
    LowPassFilter& operator=(const LowPassFilter&) = delete;
    /// @brief Move assignment operator
    LowPassFilter& operator=(LowPassFilter&&) = delete;

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

    /// Input type
    enum class InputType : uint8_t
    {
        None,        ///< None
        ImuObs,      ///< ImuObs
        ImuObsWDelta ///< ImuObsWDelta
    };

    /// Input type
    InputType _inputType = InputType::None;

    // ###########################################################################################################

    /// Types of available filters (to be extended)
    enum class FilterType : uint8_t
    {
        Linear, ///< Linear fit filter
        // Experimental,
        COUNT, ///< Amount of items in the enum
    };

    /// @brief Converts the enum to a string
    /// @param[in] value Enum value to convert into text
    /// @return String representation of the enum
    static const char* to_string(FilterType value);

    /// Selected filter type in the GUI
    FilterType _filterType = FilterType::Linear;

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
    [[nodiscard]] std::shared_ptr<ImuObs> receiveImuObs(const std::shared_ptr<ImuObs>& imuObs);

    /// @brief Callback when receiving an ImuObsWDelta
    [[nodiscard]] std::shared_ptr<ImuObsWDelta> receiveImuObsWDelta(const std::shared_ptr<ImuObsWDelta>& imuObs);

    // #########################################################################################################################################
    //                                                              LinearFitter
    // #########################################################################################################################################

    /// @brief Linear Trend Filter using ImuObs
    [[nodiscard]] std::shared_ptr<ImuObs> FitLinearTrend(const std::shared_ptr<ImuObs>& imuObs);

    /// @brief Linear Trend Filter using ImuObsWDelta
    [[nodiscard]] std::shared_ptr<ImuObsWDelta> FitLinearTrend(const std::shared_ptr<ImuObsWDelta>& imuObsWDelta);

    /// @brief Map which stores all last accelerometer data points which were used in the previous fit
    std::map<InsTime, Eigen::VectorXd> DataToFilter_Accel;

    /// @brief Map which stores all last gyro data points which were used in the previous fit
    std::map<InsTime, Eigen::VectorXd> DataToFilter_Gyro;

    /// @brief Cutoff frequency for accelerometer data, inverse of this parameter equals to fitting period
    double _linear_filter_cutoff_frequency_accel = 10.0;

    /// @brief Cutoff frequency for gyro data, inverse of this parameter equals to fitting period
    double _linear_filter_cutoff_frequency_gyro = 1.0;
};

} // namespace NAV
