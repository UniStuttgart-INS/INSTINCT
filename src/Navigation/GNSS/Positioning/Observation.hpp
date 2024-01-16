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

#include <memory>
#include <set>
#include <unordered_set>

#include "Navigation/GNSS/Core/SatelliteIdentifier.hpp"
#include "Navigation/GNSS/Functions.hpp"
#include "Navigation/GNSS/Satellite/internal/SatNavData.hpp"
#include "Navigation/Transformations/CoordinateFrames.hpp"
#include "Navigation/Transformations/Units.hpp"

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
            /// @param obsIdx GNSS observation index for this measurement
            /// @param[in] e_recPos Receiver position in e frame
            /// @param[in] lla_recPos Receiver position in lla frame
            /// @param[in] e_recVel Receiver velocity in e frame
            /// @param[in] e_satPos Satellite position in e frame
            /// @param[in] e_satVel Satellite velocity in e frame
            ReceiverSpecificData(std::shared_ptr<const GnssObs> gnssObs,
                                 size_t obsIdx,
                                 const Eigen::Vector3d& e_recPos,
                                 const Eigen::Vector3d& lla_recPos,
                                 const Eigen::Vector3d& e_recVel,
                                 const Eigen::Vector3d& e_satPos,
                                 const Eigen::Vector3d& e_satVel)
                : _gnssObs(std::move(gnssObs)), _obsIdx(obsIdx)
            {
                _e_pLOS = e_calcLineOfSightUnitVector(e_recPos, e_satPos);
                _e_vLOS = (e_recVel - e_satVel) / (e_recPos - e_satPos).norm();
                Eigen::Vector3d n_lineOfSightUnitVector = trafo::n_Quat_e(lla_recPos(0), lla_recPos(1)) * _e_pLOS;
                _satElevation = calcSatElevation(n_lineOfSightUnitVector);
                _satAzimuth = calcSatAzimuth(n_lineOfSightUnitVector);

                LOG_DATA("    e_lineOfSightUnitVector {}", _e_pLOS.transpose());
                LOG_DATA("    n_lineOfSightUnitVector {}", n_lineOfSightUnitVector.transpose());
                LOG_DATA("    satElevation {}°", rad2deg(_satElevation));
                LOG_DATA("    satAzimuth   {}°", rad2deg(_satAzimuth));
            }

            /// Receiver observation of the signal
            unordered_map<GnssObs::ObservationType, Observation> obs;

            /// @brief Returns the observation data
            [[nodiscard]] const GnssObs::ObservationData& gnssObsData() const { return _gnssObs->data.at(_obsIdx); }
            /// @brief Position Line-of-sight unit vector in ECEF frame coordinates
            [[nodiscard]] const Eigen::Vector3d& e_pLOS() const { return _e_pLOS; }
            /// @brief Velocity Line-of-sight unit vector in ECEF frame coordinates
            [[nodiscard]] const Eigen::Vector3d& e_vLOS() const { return _e_vLOS; }
            /// @brief Satellite Elevation [rad]
            [[nodiscard]] const double& satElevation() const { return _satElevation; }
            /// @brief Satellite Azimuth [rad]
            [[nodiscard]] const double& satAzimuth() const { return _satAzimuth; }

          private:
            std::shared_ptr<const GnssObs> _gnssObs = nullptr; ///< GNSS observation
            size_t _obsIdx = 0;                                ///< Gnss observation data index
            Eigen::Vector3d _e_pLOS;                           ///< Position Line-of-sight unit vector in ECEF frame coordinates
            Eigen::Vector3d _e_vLOS;                           ///< Velocity Line-of-sight unit vector in ECEF frame coordinates
            double _satElevation = 0.0;                        ///< Satellite Elevation [rad]
            double _satAzimuth = 0.0;                          ///< Satellite Azimuth [rad]
        };

        /// @brief Constructor
        /// @param[in] navData Satellite Navigation data
        /// @param[in] e_satPos Satellite position in e frame
        /// @param[in] e_satVel Satellite velocity in e frame
        /// @param[in] satClock Satellite clock information
        /// @param[in] freqNum Frequency number. Only used for GLONASS G1 and G2
        SignalObservation(std::shared_ptr<SatNavData> navData,
                          Eigen::Vector3d e_satPos,
                          Eigen::Vector3d e_satVel,
                          Clock::Corrections satClock,
                          int8_t freqNum)
            : _navData(std::move(navData)),
              _freqNum(freqNum),
              _e_satPos(std::move(e_satPos)),
              _e_satVel(std::move(e_satVel)),
              _satClock(satClock) {}

        /// @brief Receiver specific data
        std::vector<ReceiverSpecificData> recvObs;

        /// @brief Satellite Navigation data
        [[nodiscard]] const std::shared_ptr<const SatNavData>& navData() const { return _navData; }
        /// @brief Frequency number. Only used for GLONASS G1 and G2
        [[nodiscard]] int8_t freqNum() const { return _freqNum; }
        /// @brief Satellite position in ECEF frame coordinates [m]
        [[nodiscard]] const Eigen::Vector3d& e_satPos() const { return _e_satPos; }
        /// @brief Satellite velocity in ECEF frame coordinates [m/s]
        [[nodiscard]] const Eigen::Vector3d& e_satVel() const { return _e_satVel; }
        /// @brief Satellite clock information
        [[nodiscard]] const Clock::Corrections& satClock() const { return _satClock; }

      private:
        std::shared_ptr<const SatNavData> _navData = nullptr; ///< Satellite Navigation data
        int8_t _freqNum = -128;                               ///< Frequency number. Only used for GLONASS G1 and G2
        Eigen::Vector3d _e_satPos;                            ///< Satellite position in ECEF frame coordinates [m] (has to be calculated per signal because of TGD)
        Eigen::Vector3d _e_satVel;                            ///< Satellite velocity in ECEF frame coordinates [m/s]
        Clock::Corrections _satClock;                         ///< Satellite clock information
    };

    unordered_map<SatSigId, SignalObservation> signals;                               ///< Observations and calculated data for each signal
    std::set<SatelliteSystem> systems;                                                ///< Satellite systems used
    std::unordered_set<SatId> satellites;                                             ///< Satellites used
    std::array<size_t, GnssObs::ObservationType_COUNT> nObservables{};                ///< Number of observables
    std::array<size_t, GnssObs::ObservationType_COUNT> nObservablesUniqueSatellite{}; ///< Number of observables (counted once for each satellite)
};

} // namespace NAV
