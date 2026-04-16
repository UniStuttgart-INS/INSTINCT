// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Algorithm.hpp
/// @brief Single Point Positioning Algorithm
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-12-20

#pragma once

#include "Navigation/GNSS/Core/SatelliteSystem.hpp"
#include "Navigation/GNSS/Positioning/ReceiverClock.hpp"
#include "util/Assert.h"
#include <fmt/format.h>
#include <algorithm>
#include <cstdint>
#include <set>

#include "Navigation/GNSS/Positioning/Observation.hpp"
#include "Navigation/GNSS/Positioning/ObservationEstimator.hpp"
#include "Navigation/GNSS/Positioning/ObservationFilter.hpp"
#include "Navigation/GNSS/Positioning/Receiver.hpp"
#include "Navigation/GNSS/Positioning/SPP/Keys.hpp"
#include "Navigation/GNSS/Positioning/SPP/KalmanFilter.hpp"

#include "NodeData/GNSS/GnssObs.hpp"
#include "NodeData/GNSS/SppSolution.hpp"

namespace NAV
{

namespace SPP
{

/// Single Point Positioning Algorithm
class Algorithm
{
  public:
    /// Possible SPP estimation algorithms
    enum class EstimatorType : uint8_t
    {
        LeastSquares,         ///< Linear Least Squares
        WeightedLeastSquares, ///< Weighted Linear Least Squares
        KalmanFilter,         ///< Kalman Filter
        COUNT,                ///< Amount of items in the enum
    };

    /// @brief Receiver Types
    enum ReceiverType : uint8_t
    {
        Rover,              ///< Rover
        ReceiverType_COUNT, ///< Amount of receiver types
    };

    /// @brief Shows the GUI input to select the options
    /// @param[in] id Unique id for ImGui.
    /// @param[in] itemWidth Width of the widgets
    /// @param[in] unitWidth Width on unit inputs
    bool ShowGuiWidgets(const char* id, float itemWidth, float unitWidth);

    /// Reset the algorithm
    void reset();

    /// @brief Get the Estimator Type
    [[nodiscard]] EstimatorType getEstimatorType() const { return _estimatorType; }

    /// @brief Get the last update time
    const InsTime& getLastUpdateTime() const { return _lastUpdate; }

    /// @brief Calculate the SPP solution
    /// @param[in] gnssObs GNSS observation
    /// @param[in] gnssNavInfos Collection of GNSS Nav information
    /// @param[in] nameId Name and id of the node calling this (only used for logging purposes)
    /// @return The SPP Solution if it could be calculated, otherwise nullptr
    std::shared_ptr<SppSolution> calcSppSolution(const std::shared_ptr<const GnssObs>& gnssObs,
                                                 const std::vector<const GnssNavInfo*>& gnssNavInfos,
                                                 const std::string& nameId);

    /// Observation Filter
    ObservationFilter _obsFilter{ ReceiverType_COUNT,
                                  /* available */ std::unordered_set{ GnssObs::Pseudorange, GnssObs::Doppler },
                                  /*   needed  */ std::unordered_set{ GnssObs::Pseudorange } };

    /// Observation Estimator
    ObservationEstimator _obsEstimator{ ReceiverType_COUNT };

    /// Estimate Inter-frequency biases
    bool _estimateInterFreqBiases = true;

    /// @brief Set the Position
    /// @param[in] e_position ECEF position in [m]
    void setPosition(const Eigen::Vector3d& e_position);

    /// @brief Calculates the new satellite systems after the change of the reference system
    /// @param[out] newRef New reference system
    /// @param[out] oldRef Old reference system
    /// @param[out] oldInterSysSatSystems Old inter-system bias satellite systems
    /// @return List with the new systems
    static std::vector<SatelliteSystem> calcInterSystemChangeNewInterSysSatSystem(const SatelliteSystem& newRef,
                                                                                  const SatelliteSystem& oldRef,
                                                                                  const std::vector<SatelliteSystem>& oldInterSysSatSystems)
    {
        bool higherPrio = newRef < oldRef;
        INS_ASSERT_USER_ERROR(!higherPrio || std::ranges::contains(oldInterSysSatSystems, newRef), "The new reference has to be estimated before as inter-system bias before doing a reference switch.");

        std::vector<SatelliteSystem> newInterSysSatSystems = oldInterSysSatSystems;
        if (higherPrio)
        {
            for (auto& s : newInterSysSatSystems)
            {
                if (s == newRef)
                {
                    s = oldRef;
                    break;
                }
            }
        }
        else
        {
            std::erase(newInterSysSatSystems, newRef);
        }
        return newInterSysSatSystems;
    }

    /// @brief Calculates the transformation matrix D for the reference system change
    /// @param[in] newRef New reference system
    /// @param[in] oldRef Old reference system
    /// @param[in] oldStateKeys Old inter-system bias satellite systems
    /// @param[in] nameId Name and id of the calling node for logging
    /// @return Transformation matrix D
    template<typename StateKeyType>
    static KeyedMatrixXd<StateKeyType> calcInterSystemChangeMatrix(const SatelliteSystem& newRef,
                                                                   const SatelliteSystem& oldRef,
                                                                   const std::vector<StateKeyType>& oldStateKeys,
                                                                   [[maybe_unused]] const std::string& nameId)
    {
        bool higherPrio = newRef < oldRef;
        INS_ASSERT_USER_ERROR(!higherPrio || std::ranges::contains(oldStateKeys, StateKeyType{ Keys::InterSysClkBias{ newRef } }), "The new reference has to be estimated before as inter-system bias before doing a reference switch.");

        std::vector<StateKeyType> newStateKeys = oldStateKeys;
        for (auto& s : newStateKeys)
        {
            if (auto* bias = std::get_if<Keys::RecvClkBias>(&s))
            {
                *bias = Keys::RecvClkBias{ newRef };
            }
            else if (auto* bias = std::get_if<Keys::InterSysClkBias>(&s))
            {
                if (higherPrio && bias->satSys == newRef)
                {
                    *bias = Keys::InterSysClkBias{ oldRef };
                }
            }
        }
        if (!higherPrio)
        {
            std::erase(newStateKeys, StateKeyType{ Keys::InterSysClkBias{ newRef } });
        }

        KeyedMatrixXd<StateKeyType> D(Eigen::MatrixXd::Zero(static_cast<int>(newStateKeys.size()), static_cast<int>(oldStateKeys.size())),
                                      newStateKeys, oldStateKeys);
        LOG_DATA("{}: [{}] -> [{}]: D = \n{}", nameId, oldRef, newRef, D);

        if (D.hasAnyRows(Keys::Pos<StateKeyType>))
        {
            D.template block<3>(Keys::Pos<StateKeyType>, Keys::Pos<StateKeyType>).setIdentity();
        }
        if (D.hasAnyRows(Keys::Vel<StateKeyType>))
        {
            D.template block<3>(Keys::Vel<StateKeyType>, Keys::Vel<StateKeyType>).setIdentity();
        }
        for (const auto& s : oldStateKeys)
        {
            if (const auto* bias = std::get_if<Keys::RecvClkBias>(&s))
            {
                D(Keys::RecvClkBias{ newRef }, *bias) = higherPrio ? -1.0 : 1.0;
                if (!higherPrio)
                {
                    D(Keys::RecvClkBias{ newRef }, Keys::InterSysClkBias{ newRef }) = 1.0;
                }
            }
            else if (const auto* bias = std::get_if<Keys::InterSysClkBias>(&s))
            {
                if (higherPrio)
                {
                    if (bias->satSys != newRef)
                    {
                        D(*bias, *bias) = 1.0;
                    }
                    D(Keys::InterSysClkBias{ oldRef }, *bias) = -1.0;
                }
                else
                {
                    if (bias->satSys != newRef)
                    {
                        D(*bias, *bias) = 1.0;
                        D(*bias, Keys::InterSysClkBias{ newRef }) = -1.0;
                    }
                }
            }
            else if (const auto* bias = std::get_if<Keys::InterFreqBias>(&s))
            {
                D(*bias, *bias) = 1.0;
            }
        }
        if (auto drift = Keys::RecvClkDrift{};
            D.hasRow(drift))
        {
            D(drift, drift) = 1.0;
        }

        return D;
    }

  private:
    using Receiver = NAV::Receiver<ReceiverType>; ///< Receiver

    /// @brief All position keys
    const std::array<SPP::States::StateKeyType, 3>& PosKey = Keys::Pos<SPP::States::StateKeyType>;
    /// @brief All velocity keys
    const std::array<SPP::States::StateKeyType, 3>& VelKey = Keys::Vel<SPP::States::StateKeyType>;
    /// @brief All position and velocity keys
    const std::array<SPP::States::StateKeyType, 6>& PosVelKey = Keys::PosVel<SPP::States::StateKeyType>;

    /// @brief Checks if the SPP algorithm can calculate the position (always true for Kalman filter)
    /// @param[in] nDoppMeas Amount of Doppler measurements
    [[nodiscard]] bool canCalculateVelocity(size_t nDoppMeas) const;

    /// @brief Checks if the SPP algorithm can estimate inter-frequency biases
    [[nodiscard]] bool canEstimateInterFrequencyBias() const;

    /// @brief Updates the inter frequency biases
    /// @param[in] observations List of GNSS observation data used for the calculation
    /// @param[in] nameId Name and id of the node calling this (only used for logging purposes)
    void updateInterFrequencyBiases(const Observations& observations, const std::string& nameId);

    /// @brief Returns a list of state keys
    /// @param[in] nDoppMeas Amount of Doppler measurements
    /// @param[in] nameId Name and id of the node calling this (only used for logging purposes)
    std::vector<States::StateKeyType> determineStateKeys(size_t nDoppMeas, const std::string& nameId) const;

    /// @brief Returns a list of measurement keys
    /// @param[in] observations List of GNSS observation data used for the calculation
    /// @param[in] nPsrMeas Amount of pseudo-range measurements
    /// @param[in] nDoppMeas Amount of doppler measurements
    /// @param[in] nameId Name and id of the node calling this (only used for logging purposes)
    std::vector<Meas::MeasKeyTypes> determineMeasKeys(const Observations& observations, size_t nPsrMeas, size_t nDoppMeas, const std::string& nameId) const;

    /// @brief Calculates the measurement sensitivity matrix 𝐇
    /// @param[in] stateKeys State Keys
    /// @param[in] measKeys Measurement Keys
    /// @param[in] observations List of GNSS observation data used for the calculation
    /// @param[in] nameId Name and id of the node calling this (only used for logging purposes)
    /// @return The 𝐇 matrix
    [[nodiscard]] KeyedMatrixXd<Meas::MeasKeyTypes, States::StateKeyType> calcMatrixH(const std::vector<States::StateKeyType>& stateKeys,
                                                                                      const std::vector<Meas::MeasKeyTypes>& measKeys,
                                                                                      const Observations& observations,
                                                                                      const std::string& nameId) const;

    /// @brief Calculates the measurement noise covariance matrix 𝐑
    /// @param[in] measKeys Measurement Keys
    /// @param[in] observations List of GNSS observation data used for the calculation
    /// @param[in] nameId Name and id of the node calling this (only used for logging purposes)
    /// @return The 𝐑 matrix
    [[nodiscard]] static KeyedMatrixXd<Meas::MeasKeyTypes, Meas::MeasKeyTypes> calcMatrixR(const std::vector<Meas::MeasKeyTypes>& measKeys,
                                                                                           const Observations& observations,
                                                                                           const std::string& nameId);

    /// @brief Calculates the measurement innovation vector 𝜹𝐳
    /// @param[in] measKeys Measurement Keys
    /// @param[in] observations List of GNSS observation data used for the calculation
    /// @param[in] nameId Name and id of the node calling this (only used for logging purposes)
    /// @return The measurement innovation 𝜹𝐳 vector
    [[nodiscard]] static KeyedVectorXd<Meas::MeasKeyTypes> calcMeasInnovation(const std::vector<Meas::MeasKeyTypes>& measKeys,
                                                                              const Observations& observations,
                                                                              const std::string& nameId);

    /// @brief Assigns the result to the receiver variable
    /// @param[in] state Delta state
    /// @param[in] variance Variance of the state
    /// @param[in] e_oldPos Old position in ECEF coordinates in [m]
    /// @param[in] nParams Number of parameters to estimate the position
    /// @param[in] nUniqueDopplerMeas Number of available doppler measurements (unique per satellite)
    /// @param[in] dt Time step size in [s]
    /// @param[in] nameId Name and id of the node calling this (only used for logging purposes)
    void assignLeastSquaresResult(const KeyedVectorXd<States::StateKeyType>& state,
                                  const KeyedMatrixXd<States::StateKeyType, States::StateKeyType>& variance,
                                  const Eigen::Vector3d& e_oldPos,
                                  size_t nParams, size_t nUniqueDopplerMeas, double dt, const std::string& nameId);

    /// @brief Assigns the result to the receiver variable
    /// @param state Total state
    /// @param variance Variance of the state
    /// @param[in] nameId Name and id of the node calling this (only used for logging purposes)
    void assignKalmanFilterResult(const KeyedVectorXd<States::StateKeyType>& state,
                                  const KeyedMatrixXd<States::StateKeyType, States::StateKeyType>& variance,
                                  const std::string& nameId);

    /// @brief Computes all DOP values (by reference)
    /// @param[in, out] sppSol SppSol to fill with DOP values
    /// @param[in] H Measurement sensitivity matrix 𝐇
    /// @param[in] nameId Name and id of the node calling this (only used for logging purposes)
    void computeDOPs(const std::shared_ptr<SppSolution>& sppSol,
                     const KeyedMatrixXd<Meas::MeasKeyTypes, States::StateKeyType>& H,
                     const std::string& nameId);

    /// @brief Transforms the Kalman filter and receiver clock object for a reference change
    /// @param[in] newRef New reference system
    /// @param[in] nDoppMeas Amount of Doppler measurements
    /// @param[in] nameId Name and id of the node calling this (only used for logging purposes)
    void switchInterSysClkBiasReference(const SatelliteSystem& newRef, size_t nDoppMeas, const std::string& nameId);

    /// Receiver
    Receiver _receiver{ Rover };

    /// Estimator type used for the calculations
    EstimatorType _estimatorType = EstimatorType::WeightedLeastSquares;

    /// SPP specific Kalman filter
    SPP::KalmanFilter _kalmanFilter;

    /// Time of last update
    InsTime _lastUpdate;

    friend void to_json(json& j, const Algorithm& obj);
    friend void from_json(const json& j, Algorithm& obj);
};

/// @brief Converts the provided object into json
/// @param[out] j Json object which gets filled with the info
/// @param[in] obj Object to convert into json
void to_json(json& j, const Algorithm& obj);
/// @brief Converts the provided json object into a node object
/// @param[in] j Json object with the needed values
/// @param[out] obj Object to fill from the json
void from_json(const json& j, Algorithm& obj);

} // namespace SPP

/// @brief Converts the enum to a string
/// @param[in] estimatorType Enum value to convert into text
/// @return String representation of the enum
[[nodiscard]] const char* to_string(SPP::Algorithm::EstimatorType estimatorType);

/// @brief Converts the enum to a string
/// @param[in] receiver Enum value to convert into text
/// @return String representation of the enum
[[nodiscard]] const char* to_string(SPP::Algorithm::ReceiverType receiver);

} // namespace NAV

/// @brief Stream insertion operator overload
/// @param[in, out] os Output stream object to stream the time into
/// @param[in] obj Object to print
/// @return Returns the output stream object in order to chain stream insertions
std::ostream& operator<<(std::ostream& os, const NAV::SPP::Algorithm::EstimatorType& obj);

/// @brief Stream insertion operator overload
/// @param[in, out] os Output stream object to stream the time into
/// @param[in] obj Object to print
/// @return Returns the output stream object in order to chain stream insertions
std::ostream& operator<<(std::ostream& os, const NAV::SPP::Algorithm::ReceiverType& obj);

#ifndef DOXYGEN_IGNORE

/// @brief Formatter
template<>
struct fmt::formatter<NAV::SPP::Algorithm::EstimatorType> : fmt::formatter<const char*>
{
    /// @brief Defines how to format structs
    /// @param[in] type Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::SPP::Algorithm::EstimatorType& type, FormatContext& ctx) const
    {
        return fmt::formatter<const char*>::format(NAV::to_string(type), ctx);
    }
};

/// @brief Formatter
template<>
struct fmt::formatter<NAV::SPP::Algorithm::ReceiverType> : fmt::formatter<const char*>
{
    /// @brief Defines how to format structs
    /// @param[in] type Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::SPP::Algorithm::ReceiverType& type, FormatContext& ctx) const
    {
        return fmt::formatter<const char*>::format(NAV::to_string(type), ctx);
    }
};

#endif
