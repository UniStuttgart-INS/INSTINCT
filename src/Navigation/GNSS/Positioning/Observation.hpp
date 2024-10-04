// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Observation.hpp
/// @brief Observation data used for calculations
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-12-20

#pragma once

#include <cstdint>
#include <memory>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "Navigation/GNSS/Core/SatelliteIdentifier.hpp"
#include "Navigation/GNSS/Functions.hpp"
#include "Navigation/GNSS/Satellite/internal/SatNavData.hpp"
#include "Navigation/Transformations/CoordinateFrames.hpp"
#include "Navigation/Transformations/Units.hpp"
#include "Navigation/Atmosphere/Troposphere/ZenithDelay.hpp"

#include "NodeData/GNSS/GnssObs.hpp"

#include "util/Container/Unordered_map.hpp"
#include "util/Eigen.hpp"
#include "util/Logger.hpp"

namespace NAV
{

/// Observation storage type
struct Observations
{
    /// @brief Receiver specific observation of the signal
    struct SignalObservation
    {
        /// Receiver specific data
        struct ReceiverSpecificData
        {
            /// Observations
            struct Observation
            {
                double estimate = 0.0;    ///< Estimate (psr [m], carrier [m], range-rate [m/s])
                double measurement = 0.0; ///< Measurement (psr [m], carrier [m], range-rate [m/s])
                double measVar = 0.0;     ///< Variance of the measurement (psr [m^2], carrier [m^2], range-rate [m^2/s^2])
            };

            /// @brief Constructor
            /// @param[in] gnssObs GNSS observation
            /// @param[in] obsIdx GNSS observation index for this measurement
            /// @param[in] e_satPos Satellite position in e frame
            /// @param[in] e_satVel Satellite velocity in e frame
            /// @param[in] satClock Satellite clock information
            ReceiverSpecificData(std::shared_ptr<const GnssObs> gnssObs,
                                 size_t obsIdx,
                                 Eigen::Vector3d e_satPos,
                                 Eigen::Vector3d e_satVel,
                                 Clock::Corrections satClock)
                : _gnssObs(std::move(gnssObs)),
                  _obsIdx(obsIdx),
                  _e_satPos(std::move(e_satPos)),
                  _e_satVel(std::move(e_satVel)),
                  _satClock(satClock) {}

            /// Receiver observation of the signal
            unordered_map<GnssObs::ObservationType, Observation> obs;

            /// @brief Returns the observation data
            [[nodiscard]] const GnssObs::ObservationData& gnssObsData() const { return _gnssObs->data.at(_obsIdx); }
            /// @brief Satellite position in ECEF frame coordinates [m]
            [[nodiscard]] const Eigen::Vector3d& e_satPos() const { return _e_satPos; }
            /// @brief Satellite velocity in ECEF frame coordinates [m/s]
            [[nodiscard]] const Eigen::Vector3d& e_satVel() const { return _e_satVel; }
            /// @brief Position Line-of-sight unit vector in ECEF frame coordinates
            /// @param[in] e_recPos Receiver position in e frame
            [[nodiscard]] Eigen::Vector3d e_pLOS(const Eigen::Vector3d& e_recPos) const
            {
                return e_calcLineOfSightUnitVector(e_recPos, _e_satPos);
            }
            /// @brief Velocity Line-of-sight unit vector in ECEF frame coordinates
            /// @param[in] e_recPos Receiver position in e frame
            /// @param[in] e_recVel Receiver velocity in e frame
            [[nodiscard]] Eigen::Vector3d e_vLOS(const Eigen::Vector3d& e_recPos,
                                                 const Eigen::Vector3d& e_recVel) const
            {
                return (_e_satVel - e_recVel) / (_e_satPos - e_recPos).norm();
            }
            /// @brief Satellite Elevation [rad]
            /// @param[in] e_recPos Receiver position in e frame
            /// @param[in] lla_recPos Receiver position in lla frame
            [[nodiscard]] double satElevation(const Eigen::Vector3d& e_recPos,
                                              const Eigen::Vector3d& lla_recPos) const
            {
                Eigen::Vector3d n_lineOfSightUnitVector = trafo::n_Quat_e(lla_recPos(0), lla_recPos(1))
                                                          * e_pLOS(e_recPos);
                return calcSatElevation(n_lineOfSightUnitVector);
            }
            /// @brief Satellite Azimuth [rad]
            /// @param[in] e_recPos Receiver position in e frame
            /// @param[in] lla_recPos Receiver position in lla frame
            [[nodiscard]] double satAzimuth(const Eigen::Vector3d& e_recPos,
                                            const Eigen::Vector3d& lla_recPos) const
            {
                Eigen::Vector3d n_lineOfSightUnitVector = trafo::n_Quat_e(lla_recPos(0), lla_recPos(1))
                                                          * e_pLOS(e_recPos);
                return calcSatAzimuth(n_lineOfSightUnitVector);
            }
            /// @brief Satellite clock information
            [[nodiscard]] const Clock::Corrections& satClock() const { return _satClock; }

            /// @brief Terms used in the calculation
            struct CalcTerms
            {
                double rho_r_s = 0.0;         ///< Receiver-Satellite Range [m]
                ZenithDelay tropoZenithDelay; ///< Troposphere delay
                double dpsr_T_r_s = 0.0;      ///< Estimated troposphere propagation error [m]
                double dpsr_I_r_s = 0.0;      ///< Estimated ionosphere propagation error [m]
                double dpsr_ie_r_s = 0.0;     ///< Sagnac correction [m]
            };
            CalcTerms terms; ///< Sub terms used in the calculation

          private:
            std::shared_ptr<const GnssObs> _gnssObs = nullptr; ///< GNSS observation
            size_t _obsIdx = 0;                                ///< Gnss observation data index
            Eigen::Vector3d _e_satPos;                         ///< Satellite position in ECEF frame coordinates [m] (has to be calculated per signal because of TGD)
            Eigen::Vector3d _e_satVel;                         ///< Satellite velocity in ECEF frame coordinates [m/s]
            Clock::Corrections _satClock;                      ///< Satellite clock information
        };

        /// @brief Constructor
        /// @param[in] navData Satellite Navigation data
        /// @param[in] freqNum Frequency number. Only used for GLONASS G1 and G2
        SignalObservation(std::shared_ptr<SatNavData> navData,
                          int8_t freqNum)
            : _navData(std::move(navData)),
              _freqNum(freqNum) {}

        /// @brief Receiver specific data
        ///
        /// This is a shared_ptr so that e.g. the FGO can hold on to it over multiple epochs
        std::unordered_map<size_t, std::shared_ptr<ReceiverSpecificData>> recvObs;

        /// @brief Satellite Navigation data
        [[nodiscard]] const std::shared_ptr<const SatNavData>& navData() const { return _navData; }
        /// @brief Frequency number. Only used for GLONASS G1 and G2
        [[nodiscard]] int8_t freqNum() const { return _freqNum; }

      private:
        std::shared_ptr<const SatNavData> _navData = nullptr; ///< Satellite Navigation data
        int8_t _freqNum = -128;                               ///< Frequency number. Only used for GLONASS G1 and G2
    };

    unordered_map<SatSigId, SignalObservation> signals; ///< Observations and calculated data for each signal

    std::unordered_set<size_t> receivers;                                             ///< Receivers included
    std::set<SatelliteSystem> systems;                                                ///< Satellite systems used
    std::unordered_set<SatId> satellites;                                             ///< Satellites used
    std::array<size_t, GnssObs::ObservationType_COUNT> nObservables{};                ///< Number of observables
    std::array<size_t, GnssObs::ObservationType_COUNT> nObservablesUniqueSatellite{}; ///< Number of observables (counted once for each satellite)

    /// @brief Calculates/Recalculates the number of observables
    void recalcObservableCounts();
};

} // namespace NAV
