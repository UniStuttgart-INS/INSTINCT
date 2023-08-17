// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file SinglePointPositioning.hpp
/// @brief Single Point Positioning (SPP) / Code Phase Positioning
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-04-29

#pragma once

#include "util/Eigen.hpp"
#include <vector>

#include "internal/Node/Node.hpp"
#include "Navigation/GNSS/Core/Frequency.hpp"
#include "Navigation/GNSS/Core/Code.hpp"
#include "Navigation/GNSS/Core/SatelliteIdentifier.hpp"
#include "Navigation/GNSS/Positioning/SppAlgorithm.hpp"
#include "Navigation/Atmosphere/Ionosphere/Ionosphere.hpp"
#include "Navigation/Atmosphere/Troposphere/Troposphere.hpp"

#include "Navigation/Transformations/Units.hpp"
#include "Navigation/Math/KalmanFilter.hpp"
#include "Navigation/Math/LeastSquares.hpp"
#include "Navigation/GNSS/Functions.hpp"
#include "Navigation/GNSS/Satellite/internal/SatNavData.hpp"

#include "NodeData/GNSS/GnssObs.hpp"
#include "NodeData/GNSS/GnssNavInfo.hpp"
#include "NodeData/GNSS/SppSolution.hpp"

namespace NAV
{
/// @brief Numerically integrates Imu data
class SinglePointPositioning : public Node
{
  public:
    /// @brief Default constructor
    SinglePointPositioning();
    /// @brief Destructor
    ~SinglePointPositioning() override;
    /// @brief Copy constructor
    SinglePointPositioning(const SinglePointPositioning&) = delete;
    /// @brief Move constructor
    SinglePointPositioning(SinglePointPositioning&&) = delete;
    /// @brief Copy assignment operator
    SinglePointPositioning& operator=(const SinglePointPositioning&) = delete;
    /// @brief Move assignment operator
    SinglePointPositioning& operator=(SinglePointPositioning&&) = delete;

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
    constexpr static size_t INPUT_PORT_INDEX_GNSS_OBS = 0;      ///< @brief GnssObs
    constexpr static size_t INPUT_PORT_INDEX_GNSS_NAV_INFO = 1; ///< @brief GnssNavInfo

    constexpr static size_t OUTPUT_PORT_INDEX_SPPSOL = 0; ///< @brief Flow (SppSol)

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// Index of the Pin currently being dragged
    int _dragAndDropPinIndex = -1;
    /// Amount of NavInfo input pins
    size_t _nNavInfoPins = 1;
    /// @brief Adds/Deletes Input Pins depending on the variable _nNavInfoPins
    void updateNumberOfInputPins();

    /// Frequencies used for calculation (GUI filter)
    Frequency _filterFreq = G01;
    /// Codes used for calculation (GUI filter)
    Code _filterCode = Code_Default;
    /// List of satellites to exclude
    std::vector<SatId> _excludedSatellites;
    /// Elevation cut-off angle for satellites in [rad]
    double _elevationMask = static_cast<double>(15.0_deg);

    /// Ionosphere Model used for the calculation
    IonosphereModel _ionosphereModel = IonosphereModel::Klobuchar;

    /// Troposphere Models used for the calculation
    TroposphereModelSelection _troposphereModels;

    /// GNSS measurement error model to use
    GnssMeasurementErrorModel _gnssMeasurementErrorModel;

    /// Estimator type
    GNSS::Positioning::SPP::EstimatorType _estimatorType = GNSS::Positioning::SPP::EstimatorType::WEIGHTED_LEAST_SQUARES;

    /// State estimated by the algorithm
    GNSS::Positioning::SPP::State _state;

    /// @brief Receive Function for the Gnss Observations
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void recvGnssObs(InputPin::NodeDataQueue& queue, size_t pinIdx);
};

} // namespace NAV