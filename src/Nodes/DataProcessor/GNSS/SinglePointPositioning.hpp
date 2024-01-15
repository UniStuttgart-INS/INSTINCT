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

#include "internal/Node/Node.hpp"
#include "internal/gui/widgets/DynamicInputPins.hpp"

#include "Navigation/Transformations/Units.hpp"
#include "Navigation/Math/KalmanFilter.hpp"
#include "Navigation/Math/LeastSquares.hpp"
#include "Navigation/GNSS/Functions.hpp"
#include "Navigation/GNSS/Positioning/SPP/Algorithm.hpp"
#include "Navigation/GNSS/Satellite/internal/SatNavData.hpp"

#include "util/Eigen.hpp"

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

    /// @brief Function to call to add a new pin
    /// @param[in, out] node Pointer to this node
    static void pinAddCallback(Node* node);
    /// @brief Function to call to delete a pin
    /// @param[in, out] node Pointer to this node
    /// @param[in] pinIdx Input pin index to delete
    static void pinDeleteCallback(Node* node, size_t pinIdx);

    /// @brief SPP algorithm
    SPP::Algorithm _algorithm;

    /// @brief Dynamic input pins
    /// @attention This should always be the last variable in the header, because it accesses others through the function callbacks
    gui::widgets::DynamicInputPins _dynamicInputPins{ INPUT_PORT_INDEX_GNSS_NAV_INFO, this, pinAddCallback, pinDeleteCallback };

    // /// Estimator type
    // GNSS::Positioning::SPP::EstimatorType _estimatorType = GNSS::Positioning::SPP::EstimatorType::WEIGHTED_LEAST_SQUARES;

    // /// @brief SPP Kalman filter
    // GNSS::Positioning::SPP::SppKalmanFilter _kalmanFilter;

    // /// State estimated by the algorithm
    // GNSS::Positioning::SPP::State _state;

    // /// @brief All Inter-system clock error keys
    // std::vector<GNSS::Positioning::SPP::States::StateKeyTypes> _interSysErrs{};
    // /// @brief All Inter-system clock drift keys
    // /// @note Groves2013 does not estimate inter-system drifts, but we do for all models.
    // std::vector<GNSS::Positioning::SPP::States::StateKeyTypes> _interSysDrifts{};

    /// @brief Receive Function for the Gnss Observations
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void recvGnssObs(InputPin::NodeDataQueue& queue, size_t pinIdx);
};

} // namespace NAV