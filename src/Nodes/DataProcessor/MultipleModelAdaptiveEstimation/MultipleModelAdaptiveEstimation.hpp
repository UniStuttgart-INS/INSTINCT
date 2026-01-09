// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file MultipleModelAdaptiveEstimation.hpp
/// @brief Multiple Model Adaptive Estimation node (Kalman filter bank)
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2025-07-02

#pragma once

#include "Navigation/Time/InsTime.hpp"
#include "NodeData/State/InsGnssLCKFSolution.hpp"
#include "Navigation/Math/KeyedKalmanFilter.hpp"

#include "internal/Node/Node.hpp"
#include "internal/gui/widgets/DynamicInputPins.hpp"
#include <Eigen/src/Core/Matrix.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace NAV
{
class MultipleModelAdaptiveEstimation : public Node
{
  public:
    /// @brief Default constructor
    MultipleModelAdaptiveEstimation();
    /// @brief Destructor
    ~MultipleModelAdaptiveEstimation() override;
    /// @brief Copy constructor
    MultipleModelAdaptiveEstimation(const MultipleModelAdaptiveEstimation&) = delete;
    /// @brief Move constructor
    MultipleModelAdaptiveEstimation(MultipleModelAdaptiveEstimation&&) = delete;
    /// @brief Copy assignment operator
    MultipleModelAdaptiveEstimation& operator=(const MultipleModelAdaptiveEstimation&) = delete;
    /// @brief Move assignment operator
    MultipleModelAdaptiveEstimation& operator=(MultipleModelAdaptiveEstimation&&) = delete;

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
    constexpr static size_t OUTPUT_PORT_INDEX_DYN_DATA = 0; ///< @brief Flow (DynamicData)

    /// @brief Function to call to add a new pin
    /// @param[in, out] node Pointer to this node
    static void pinAddCallback(Node* node);
    /// @brief Function to call to delete a pin
    /// @param[in, out] node Pointer to this node
    /// @param[in] pinIdx Input pin index to delete
    static void pinDeleteCallback(Node* node, size_t pinIdx);

    /// @brief Dynamic input pins
    /// @attention This should always be the last variable in the header, because it accesses others through the function callbacks
    gui::widgets::DynamicInputPins _dynamicInputPins{ 0, this, pinAddCallback, pinDeleteCallback };

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Receive Data Function
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void receiveData(InputPin::NodeDataQueue& queue, size_t pinIdx);

    /// @brief Calculate adaptive estimate x and covariance P
    void calcAdaptiveEstimate();

    /// @brief Sets the PosVelAtt solution and its covariance matrix P (and does the necessary unit conversions)
    /// @param[in] mmaeSolution Adaptive estimate
    /// @param[in] R_N Prime vertical radius of curvature (East/West) [m]
    /// @param[in] R_E Meridian radius of curvature in [m]
    void setSolutionPosVelAttAndCov(const std::shared_ptr<PosVelAtt>& mmaeSolution, double R_N, double R_E);

    /// Lower limit for the probabilities (weights) and conditional densities of the measurement. Also used to check whether probabilities have been initialized
    constexpr static double LOWER_LIMIT = 1e-15;
    /// Upper limit for the probabilities
    constexpr static double WEIGHTS_UPPER_LIMIT = 0.5;

    /// Number of elements in PosVel measurement
    constexpr static size_t NUM_MEASUREMENTS_POSVEL = 6;
    /// Number of elements in Baro measurement
    constexpr static size_t NUM_MEASUREMENTS_BARO = 1;

    /// Scalar penalty factor
    double _alpha = 0.5;

    /// Lower limit for probabilities (so that MMAE does not get stuck on one LCKF with nearly 100%)
    double _weightsLowerLimit = LOWER_LIMIT;

    /// Number of elemental Kalman filters
    size_t _nFilters{};

    /// Conditional density function of the measurement f(z(k) | a_i, Z^(k-1))
    std::vector<double> _condDensitiesMeas;

    /// Conditional probabilities of the various hypotheses
    std::vector<double> _probabilities;

    /// @brief Kalman filter object for the MMAE solution
    KeyedKalmanFilterD<LckfKeys::KFStates, LckfKeys::KFMeas> _mmaeKalmanFilter{ LckfKeys::States, LckfKeys::Meas };

    /// GUI option for the initial distribution of weights
    enum class InitDistributionWeights : uint8_t
    {
        Uniform, ///< Uniform distribution
        FirstKF, ///< First LCKF (Pin) has big weight, others close to zero (in case of a 'nominal' scenario)
    };
    /// GUI option for the initial distribution of weights
    InitDistributionWeights _initDistributionWeights = InitDistributionWeights::Uniform;

    /// Update types that are relevant for the weight update in MMAE
    enum class UpdateType : uint8_t
    {
        PredictionOnly,  ///< No update
        PosVel,          ///< PosVel update only
        BaroHgt,         ///< BaroHgt update only
        PosVelAndBaroHgt ///< PosVel and BaroHgt update at the same time
    };

    /// MMAE pin data
    struct PinData
    {
        /// Index of the i-th LCKF
        size_t lckfIdx;
        /// Solution of the i-th LCKF
        std::shared_ptr<const InsGnssLCKFSolution> lckfSol;
        /// Update type that is relevant for the weight update
        UpdateType updateType;
    };

    /// Data storage for each pin
    std::vector<PinData> _pinData;

    /// Stores indices of the single LCKFs that are fed into the MMAE
    std::vector<size_t> _lckfIndices;
};

} // namespace NAV