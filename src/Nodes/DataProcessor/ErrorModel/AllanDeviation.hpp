// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file AllanDeviation.hpp
/// @brief Computes Allan Deviation
/// @author M. Seyfried (Master thesis student)
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-03-29

#pragma once

#include "internal/Node/Node.hpp"

#include "util/Eigen.hpp"
#include <array>
#include <cstdint>
#include <mutex>
#include <vector>

namespace NAV
{
/// @brief Computes Allan Deviation of IMU Observations
class AllanDeviation : public Node
{
  public:
    /// @brief Default constructor
    AllanDeviation();
    /// @brief Destructor
    ~AllanDeviation() override;
    /// @brief Copy constructor
    AllanDeviation(const AllanDeviation&) = delete;
    /// @brief Move constructor
    AllanDeviation(AllanDeviation&&) = delete;
    /// @brief Copy assignment operator
    AllanDeviation& operator=(const AllanDeviation&) = delete;
    /// @brief Move assignment operator
    AllanDeviation& operator=(AllanDeviation&&) = delete;

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
    constexpr static size_t INPUT_PORT_INDEX_IMU_OBS = 0; ///< @brief Flow (ImuObs)

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Receive Sensor Data
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void receiveImuObs(InputPin::NodeDataQueue& queue, size_t pinIdx);

    // ------------------------------------------------------------ Algorithm --------------------------------------------------------------

    /// Sensor types
    enum SensorType : uint8_t
    {
        Accel,            ///< Accelerometer
        Gyro,             ///< Gyroscope
        SensorType_COUNT, ///< Amount of sensors to use
    };

    /// Data for each sensor
    struct Sensor
    {
        /// Cumulative Sums
        std::vector<Eigen::Vector3d> cumSum{ Eigen::Vector3d::Zero() };

        /// Allan Variance precursor
        std::array<std::vector<double>, 3> allanSum{};

        /// Allan Variance
        std::array<std::vector<double>, 3> allanVariance{};

        /// Allan Deviation
        std::array<std::vector<double>, 3> allanDeviation{};

        /// Slope of Allan Variance
        std::array<std::vector<double>, 3> slope{};

        /// Confidence of Allan Deviation
        std::array<std::array<std::vector<double>, 2>, 3> allanDeviationConfidenceIntervals{};

        /// Mutex to lock plotting
        std::mutex mutex;
    };

    /// Sensor data
    std::array<Sensor, SensorType_COUNT> _sensors;

    /// sampling interval
    double _samplingInterval = 0.0;

    /// Time of first epoch received
    InsTime _startingInsTime;

    /// averaging factors (n) used for Allan Variance computation
    std::vector<double> _averagingFactors;

    /// averaging times (τ)
    std::vector<double> _averagingTimes;

    /// number of observations for each τ
    std::vector<double> _observationCount;

    /// number of IMU observations / length of cumulative sums
    unsigned int _imuObsCount{ 0 };

    /// number of averaging factors per decade
    double _averagingFactorsPerDecade{ 100 };

    /// next averaging factor to be appended to _averagingFactors
    unsigned int _nextAveragingFactor{ 1 };

    /// exponent of next averaging factor
    unsigned int _nextAveragingFactorExponent{ 1 };

    /// multiplication factor for simple confidence
    std::vector<double> _confidenceMultiplicationFactor;

    /// Flag wether to display confidence intervals
    bool _displayConfidence{ false };
    /// The alpha value for the shaded plot of the confidence intervals
    float _confidenceFillAlpha{ 0.4F };
    /// Flag wether to update the plot for each message or once at the end
    bool _updateLast{ false };

    /// @brief Returns a string representation of the type
    /// @param[in] sensorType Sensor Type
    static const char* to_string(SensorType sensorType);

    /// @brief Returns a string for the unit of the type
    /// @param[in] sensorType Sensor Type
    static const char* unitString(SensorType sensorType);
};

} // namespace NAV