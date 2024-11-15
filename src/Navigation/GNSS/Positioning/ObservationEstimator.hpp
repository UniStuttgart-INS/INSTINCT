// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file ObservationEstimator.hpp
/// @brief Calculates Observation estimates
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-12-21

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include <imgui.h>
#include <application.h>
#include <imgui_stdlib.h>
#include "Navigation/GNSS/Core/Frequency.hpp"
#include "Navigation/GNSS/Core/SatelliteIdentifier.hpp"
#include "Navigation/GNSS/Positioning/AntexReader.hpp"
#include "Navigation/Transformations/Antenna.hpp"
#include "Navigation/Transformations/CoordinateFrames.hpp"
#include "NodeData/GNSS/GnssObs.hpp"
#include "internal/gui/widgets/imgui_ex.hpp"
#include "internal/gui/widgets/HelpMarker.hpp"

#include "Navigation/Atmosphere/Ionosphere/Ionosphere.hpp"
#include "Navigation/Atmosphere/Troposphere/Troposphere.hpp"
#include "Navigation/GNSS/Errors/MeasurementErrors.hpp"
#include "Navigation/GNSS/Positioning/Observation.hpp"
#include "Navigation/GNSS/Positioning/Receiver.hpp"
#include "Navigation/GNSS/Satellite/Ephemeris/GLONASSEphemeris.hpp"

#include "util/Container/UncertainValue.hpp"
#include "util/Json.hpp"
#include <Eigen/src/Core/MatrixBase.h>
#include <fmt/core.h>
#include <fmt/format.h>

namespace NAV
{

/// @brief Calculates Observation estimates
class ObservationEstimator
{
  public:
    /// @brief Constructor
    /// @param[in] receiverCount Number of receivers
    explicit ObservationEstimator(size_t receiverCount)
        : _receiverCount(receiverCount)
    {
        for (size_t i = 0; i < receiverCount; i++)
        {
            _antenna.emplace(i, Antenna{});
        }
    }

    /// @brief How the observation gets used. Influenced the measurement variance
    enum ObservationDifference : uint8_t
    {
        NoDifference,     ///< Estimation is not differenced
        SingleDifference, ///< Single Difference
        DoubleDifference, ///< Double Difference
    };

    /// @brief Calculates the observation estimates
    /// @param[in, out] observations List of GNSS observation data used for the calculation of this epoch
    /// @param[in] ionosphericCorrections Ionospheric correction parameters collected from the Nav data
    /// @param[in] receiver Receiver
    /// @param[in] nameId Name and Id of the node used for log messages only
    /// @param[in] obsDiff Observation Difference type to estimate
    template<typename ReceiverType>
    void calcObservationEstimates(Observations& observations,
                                  const Receiver<ReceiverType>& receiver,
                                  const std::shared_ptr<const IonosphericCorrections>& ionosphericCorrections,
                                  [[maybe_unused]] const std::string& nameId,
                                  ObservationDifference obsDiff) const
    {
        LOG_DATA("{}: Calculating all observation estimates:", nameId);
        for (auto& [satSigId, observation] : observations.signals)
        {
            calcObservationEstimate(satSigId, observation, receiver, ionosphericCorrections, nameId, obsDiff);
        }
    }

    /// @brief Calculates the observation estimate for the given signal
    /// @param[in] satSigId Satellite signal identifier
    /// @param[in, out] observation GNSS observation data used for the calculation of this epoch
    /// @param[in] ionosphericCorrections Ionospheric correction parameters collected from the Nav data
    /// @param[in] receiver Receiver
    /// @param[in] nameId Name and Id of the node used for log messages only
    /// @param[in] obsDiff Observation Difference type to estimate
    template<typename ReceiverType>
    void calcObservationEstimate(const SatSigId& satSigId,
                                 Observations::SignalObservation& observation,
                                 const Receiver<ReceiverType>& receiver,
                                 const std::shared_ptr<const IonosphericCorrections>& ionosphericCorrections,
                                 [[maybe_unused]] const std::string& nameId,
                                 ObservationDifference obsDiff) const
    {
        LOG_DATA("{}: Calculating observation estimates for [{}]", nameId, satSigId);
        const Frequency freq = satSigId.freq();
        const SatelliteSystem satSys = freq.getSatSys();

        auto& recvObs = observation.recvObs.at(receiver.type);

        Eigen::Vector3d e_recvPosAPC = e_calcRecvPosAPC(freq,
                                                        receiver.type,
                                                        receiver.e_posMarker,
                                                        receiver.gnssObs, nameId);
        Eigen::Vector3d lla_recvPosAPC = trafo::ecef2lla_WGS84(e_recvPosAPC);

        double elevation = recvObs->satElevation(e_recvPosAPC, lla_recvPosAPC);
        double azimuth = recvObs->satAzimuth(e_recvPosAPC, lla_recvPosAPC);
        Eigen::Vector3d e_pLOS = recvObs->e_pLOS(e_recvPosAPC);

        // Receiver-Satellite Range [m]
        double rho_r_s = (recvObs->e_satPos() - e_recvPosAPC).norm();
        recvObs->terms.rho_r_s = rho_r_s;
        // Troposphere
        auto tropo_r_s = calcTroposphericDelayAndMapping(receiver.gnssObs->insTime, lla_recvPosAPC,
                                                         elevation, azimuth, _troposphereModels);
        recvObs->terms.tropoZenithDelay = tropo_r_s;
        // Estimated troposphere propagation error [m]
        double dpsr_T_r_s = tropo_r_s.ZHD * tropo_r_s.zhdMappingFactor + tropo_r_s.ZWD * tropo_r_s.zwdMappingFactor;
        recvObs->terms.dpsr_T_r_s = dpsr_T_r_s;
        // Estimated ionosphere propagation error [m]
        double dpsr_I_r_s = calcIonosphericDelay(static_cast<double>(receiver.gnssObs->insTime.toGPSweekTow().tow),
                                                 freq, observation.freqNum(), lla_recvPosAPC, elevation, azimuth,
                                                 _ionosphereModel, ionosphericCorrections.get());
        recvObs->terms.dpsr_I_r_s = dpsr_I_r_s;
        // Sagnac correction [m]
        double dpsr_ie_r_s = calcSagnacCorrection(e_recvPosAPC, recvObs->e_satPos());
        recvObs->terms.dpsr_ie_r_s = dpsr_ie_r_s;

        // Earth's gravitational field causes relativistic signal delay due to space-time curvature (Shapiro effect) [s]
        // double posNorm = recvObs->e_satPos().norm() + e_recvPosAPC.norm();
        // double dt_rel_stc = e_recvPosAPC.norm() > InsConst::WGS84::a / 2.0 ? 2.0 * InsConst::WGS84::MU / std::pow(InsConst::C, 3) * std::log((posNorm + rho_r_s) / (posNorm - rho_r_s))
        //                                                                        : 0.0;

        double cn0 = recvObs->gnssObsData().CN0.value_or(1.0);

        for (auto& [obsType, obsData] : recvObs->obs)
        {
            LOG_DATA("{}:   [{}][{:11}][{:5}] Observation estimate", nameId, satSigId, obsType, receiver.type);
            switch (obsType)
            {
            case GnssObs::Pseudorange:
                obsData.estimate = rho_r_s
                                   + dpsr_ie_r_s
                                   + dpsr_T_r_s
                                   + dpsr_I_r_s
                                   + InsConst::C
                                         * (*receiver.recvClk.biasFor(satSys) - recvObs->satClock().bias
                                            // + dt_rel_stc
                                            + (receiver.interFrequencyBias.contains(freq) ? receiver.interFrequencyBias.at(freq).value : 0.0));
                obsData.measVar = _gnssMeasurementErrorModel.psrMeasErrorVar(satSys, elevation, cn0);
                LOG_DATA("{}:   [{}][{:11}][{:5}]     {:.4f} [m] Geometrical range", nameId, satSigId, obsType, receiver.type, rho_r_s);
                LOG_DATA("{}:   [{}][{:11}][{:5}]   + {:.4f} [m] Sagnac correction", nameId, satSigId, obsType, receiver.type, dpsr_ie_r_s);
                if (dpsr_T_r_s != 0.0) { LOG_DATA("{}:   [{}][{:11}][{:5}]   + {:.4f} [m] Tropospheric delay", nameId, satSigId, obsType, receiver.type, dpsr_T_r_s); }
                if (dpsr_I_r_s != 0.0) { LOG_DATA("{}:   [{}][{:11}][{:5}]   + {:.4f} [m] Ionospheric delay", nameId, satSigId, obsType, receiver.type, dpsr_I_r_s); }
                if (*receiver.recvClk.biasFor(satSys) != 0.0) { LOG_DATA("{}:   [{}][{:11}][{:5}]   + {:.4f} [m] Receiver clock bias", nameId, satSigId, obsType, receiver.type, InsConst::C * *receiver.recvClk.biasFor(satSys)); }
                LOG_DATA("{}:   [{}][{:11}][{:5}]   - {:.4f} [m] Satellite clock bias", nameId, satSigId, obsType, receiver.type, InsConst::C * recvObs->satClock().bias);
                // LOG_DATA("{}:   [{}][{:11}][{:5}]   + {:.4f} [m] Shapiro effect", nameId, satSigId, obsType, receiver.type, InsConst::C * dt_rel_stc);
                if (receiver.interFrequencyBias.contains(freq)) { LOG_DATA("{}:   [{}][{:11}][{:5}]   + {:.4f} [m] Inter-frequency bias", nameId, satSigId, obsType, receiver.type, InsConst::C * receiver.interFrequencyBias.at(freq).value); }
                LOG_DATA("{}:   [{}][{:11}][{:5}]   = {:.4f} [m] Pseudorange estimate", nameId, satSigId, obsType, receiver.type, obsData.estimate);
                LOG_DATA("{}:   [{}][{:11}][{:5}]     {:.4f} [m] Pseudorange measurement", nameId, satSigId, obsType, receiver.type, obsData.measurement);
                LOG_DATA("{}:   [{}][{:11}][{:5}]       {:.4e} [m] Difference to measurement", nameId, satSigId, obsType, receiver.type, obsData.measurement - obsData.estimate);
                break;
            case GnssObs::Carrier:
                obsData.estimate = rho_r_s
                                   + dpsr_ie_r_s
                                   + dpsr_T_r_s
                                   - dpsr_I_r_s
                                   + InsConst::C
                                         * (*receiver.recvClk.biasFor(satSys) - recvObs->satClock().bias
                                            // + dt_rel_stc
                                         );
                obsData.measVar = _gnssMeasurementErrorModel.carrierMeasErrorVar(satSys, elevation, cn0);
                LOG_DATA("{}:   [{}][{:11}][{:5}]     {:.4f} [m] Geometrical range", nameId, satSigId, obsType, receiver.type, rho_r_s);
                LOG_DATA("{}:   [{}][{:11}][{:5}]   + {:.4f} [m] Sagnac correction", nameId, satSigId, obsType, receiver.type, dpsr_ie_r_s);
                if (dpsr_T_r_s != 0.0) { LOG_DATA("{}:   [{}][{:11}][{:5}]   + {:.4f} [m] Tropospheric delay", nameId, satSigId, obsType, receiver.type, dpsr_T_r_s); }
                if (dpsr_I_r_s != 0.0) { LOG_DATA("{}:   [{}][{:11}][{:5}]   - {:.4f} [m] Ionospheric delay", nameId, satSigId, obsType, receiver.type, dpsr_I_r_s); }
                if (*receiver.recvClk.biasFor(satSys) != 0.0) { LOG_DATA("{}:   [{}][{:11}][{:5}]   + {:.4f} [m] Receiver clock bias", nameId, satSigId, obsType, receiver.type, InsConst::C * *receiver.recvClk.biasFor(satSys)); }
                LOG_DATA("{}:   [{}][{:11}][{:5}]   - {:.4f} [m] Satellite clock bias", nameId, satSigId, obsType, receiver.type, InsConst::C * recvObs->satClock().bias);
                // LOG_DATA("{}:   [{}][{:11}][{:5}]   + {:.4f} [m] Shapiro effect", nameId, satSigId, obsType, receiver.type, InsConst::C * dt_rel_stc);
                LOG_DATA("{}:   [{}][{:11}][{:5}]   = {:.4f} [m] Carrier-phase estimate", nameId, satSigId, obsType, receiver.type, obsData.estimate);
                LOG_DATA("{}:   [{}][{:11}][{:5}]     {:.4f} [m] Carrier-phase measurement", nameId, satSigId, obsType, receiver.type, obsData.measurement);
                LOG_DATA("{}:   [{}][{:11}][{:5}]       {:.4e} [m] Difference to measurement", nameId, satSigId, obsType, receiver.type, obsData.measurement - obsData.estimate);
                break;
            case GnssObs::Doppler:
                obsData.estimate = e_pLOS.dot(recvObs->e_satVel() - receiver.e_vel)
                                   - calcSagnacRateCorrection(e_recvPosAPC, recvObs->e_satPos(), receiver.e_vel, recvObs->e_satVel())
                                   + InsConst::C
                                         * (*receiver.recvClk.driftFor(satSys)
                                            - recvObs->satClock().drift);
                obsData.measVar = _gnssMeasurementErrorModel.psrRateMeasErrorVar(freq, observation.freqNum(), elevation, cn0);
                LOG_DATA("{}:   [{}][{:11}][{:5}]     {:.4f} [m/s] ", nameId, satSigId, obsType, receiver.type, e_pLOS.dot(recvObs->e_satVel() - receiver.e_vel));
                LOG_DATA("{}:   [{}][{:11}][{:5}]   - {:.4f} [m/s] Sagnac rate correction", nameId, satSigId, obsType, receiver.type, calcSagnacRateCorrection(e_recvPosAPC, recvObs->e_satPos(), receiver.e_vel, recvObs->e_satVel()));
                if (*receiver.recvClk.driftFor(satSys) != 0.0) { LOG_DATA("{}:   [{}][{:11}][{:5}]   + {:.4f} [m/s] Receiver clock drift", nameId, satSigId, obsType, receiver.type, InsConst::C * *receiver.recvClk.driftFor(satSys)); }
                LOG_DATA("{}:   [{}][{:11}][{:5}]   - {:.4f} [m/s] Satellite clock drift", nameId, satSigId, obsType, receiver.type, InsConst::C * recvObs->satClock().drift);
                LOG_DATA("{}:   [{}][{:11}][{:5}]   = {:.4f} [m/s] Doppler estimate", nameId, satSigId, obsType, receiver.type, obsData.estimate);
                LOG_DATA("{}:   [{}][{:11}][{:5}]     {:.4f} [m/s] Doppler measurement", nameId, satSigId, obsType, receiver.type, obsData.measurement);
                LOG_DATA("{}:   [{}][{:11}][{:5}]       {:.4e} [m/s] Difference to measurement", nameId, satSigId, obsType, receiver.type, obsData.measurement - obsData.estimate);
                break;
            case GnssObs::ObservationType_COUNT:
                break;
            }
            LOG_DATA("{}:   [{}][{:11}][{:5}] Observation error variance", nameId, satSigId, obsType, receiver.type);
            LOG_DATA("{}:   [{}][{:11}][{:5}]     {:.4g} [{}] Measurement error variance", nameId, satSigId, obsType, receiver.type, obsData.measVar,
                     obsType == GnssObs::Doppler ? "m^2/s^2" : "m^2");

            if (obsDiff == NoDifference)
            {
                if (obsType == GnssObs::Pseudorange || obsType == GnssObs::Carrier)
                {
                    obsData.measVar += observation.navData()->calcSatellitePositionVariance()
                                       + ionoErrorVar(dpsr_I_r_s, freq, observation.freqNum())
                                       + tropoErrorVar(dpsr_T_r_s, elevation);
                    LOG_DATA("{}:   [{}][{:11}][{:5}]   + {:.4f} [m^2] Satellite position variance", nameId, satSigId, obsType, receiver.type, observation.navData()->calcSatellitePositionVariance());
                    if (dpsr_I_r_s != 0.0) { LOG_DATA("{}:   [{}][{:11}][{:5}]   + {:.4f} [m^2] Ionosphere variance", nameId, satSigId, obsType, receiver.type, ionoErrorVar(dpsr_I_r_s, freq, observation.freqNum())); }
                    if (dpsr_T_r_s != 0.0) { LOG_DATA("{}:   [{}][{:11}][{:5}]   + {:.4f} [m^2] Troposphere variance", nameId, satSigId, obsType, receiver.type, tropoErrorVar(dpsr_T_r_s, elevation)); }
                }
                if (obsType == GnssObs::Pseudorange)
                {
                    obsData.measVar += _gnssMeasurementErrorModel.codeBiasErrorVar();
                    LOG_DATA("{}:   [{}][{:11}][{:5}]   + {:.4f} [m^2] Code bias variance", nameId, satSigId, obsType, receiver.type, _gnssMeasurementErrorModel.codeBiasErrorVar());
                }
            }
            LOG_DATA("{}:   [{}][{:11}][{:5}]   = {:.4g} [{}] Observation error variance", nameId, satSigId, obsType, receiver.type, obsData.measVar,
                     obsType == GnssObs::Doppler ? "m^2/s^2" : "m^2");
        }
    }

    /// Satellite Info used for estimate calculation
    struct SatelliteInfo
    {
        /// @brief Constructor
        /// @param[in] e_satPos Satellite position in ECEF coordinates [m]
        /// @param[in] e_satVel Satellite velocity in ECEF frame [m/s]
        /// @param[in] satClockBias Satellite clock bias [s]
        /// @param[in] satClockDrift Satellite clock drift [s/s]
        SatelliteInfo(Eigen::Vector3d e_satPos, Eigen::Vector3d e_satVel, double satClockBias, double satClockDrift)
            : e_satPos(std::move(e_satPos)), e_satVel(std::move(e_satVel)), satClockBias(satClockBias), satClockDrift(satClockDrift) {}

        Eigen::Vector3d e_satPos; ///< Satellite position in ECEF coordinates [m]
        Eigen::Vector3d e_satVel; ///< Satellite velocity in ECEF frame [m/s]
        double satClockBias{};    ///< Satellite clock bias [s]
        double satClockDrift{};   ///< Satellite clock drift [s/s]
    };

    /// @brief Calculates the pseudorange estimate
    /// @param[in] e_recvPosAPC Receiver antenna phase center position in ECEF coordinates [m]
    /// @param[in] recvClockBias Receiver clock bias [s]
    /// @param[in] interFreqBias Receiver inter-frequency bias [s]
    /// @param[in] dpsr_T_r_s Estimated troposphere propagation error [m]
    /// @param[in] dpsr_I_r_s Estimated ionosphere propagation error [m]
    /// @param[in] satInfo Satellite Information (pos, vel, clock)
    /// @param[in] nameId Name and Id of the node used for log messages only
    /// @return The estimate [m] and the position line-of-sight unit vector (used for the Jacobian)
    template<typename Derived>
    auto calcPseudorangeEstimate(const Eigen::MatrixBase<Derived>& e_recvPosAPC,
                                 auto recvClockBias,
                                 auto interFreqBias,
                                 auto dpsr_T_r_s,
                                 auto dpsr_I_r_s,
                                 const std::shared_ptr<const SatelliteInfo>& satInfo,
                                 [[maybe_unused]] const std::string& nameId) const
    {
        // Receiver-Satellite Range [m]
        auto rho_r_s = (satInfo->e_satPos - e_recvPosAPC).norm();

        // Sagnac correction [m]
        auto dpsr_ie_r_s = calcSagnacCorrection(e_recvPosAPC, satInfo->e_satPos);

        // Earth's gravitational field causes relativistic signal delay due to space-time curvature (Shapiro effect) [s]
        // double posNorm = e_satPos.norm() + e_recvPosAPC.norm();
        // double dt_rel_stc = e_recvPosAPC.norm() > InsConst::WGS84::a / 2.0 ? 2.0 * InsConst::WGS84::MU / std::pow(InsConst::C, 3) * std::log((posNorm + rho_r_s) / (posNorm - rho_r_s))
        //                                                                        : 0.0;

        return rho_r_s
               + dpsr_ie_r_s
               + dpsr_T_r_s
               + dpsr_I_r_s
               + InsConst::C * (recvClockBias - satInfo->satClockBias /* + dt_rel_stc */ + interFreqBias);
    }

    /// @brief Calculates the carrier-phase estimate
    /// @param[in] freq Signal frequency
    /// @param[in] freqNum Frequency number. Only used for GLONASS G1 and G2
    /// @param[in] receiverType Receiver type to select the correct antenna
    /// @param[in] e_recvPosMarker Receiver marker position in ECEF coordinates [m]
    /// @param[in] recvClockBias Receiver clock bias [s]
    /// @param[in] satInfo Satellite Information (pos, vel, clock)
    /// @param[in] gnssObs GNSS observation
    /// @param[in] ionosphericCorrections Ionospheric correction parameters collected from the Nav data
    /// @param[in] nameId Name and Id of the node used for log messages only
    /// @return The estimate [m] and the position line-of-sight unit vector (used for the Jacobian)
    template<typename Derived>
    std::pair<double, Eigen::Vector3d>
        calcCarrierEstimate(Frequency freq,
                            int8_t freqNum,
                            size_t receiverType,
                            const Eigen::MatrixBase<Derived>& e_recvPosMarker,
                            double recvClockBias,
                            const std::shared_ptr<const SatelliteInfo>& satInfo,
                            const std::shared_ptr<const GnssObs>& gnssObs,
                            const std::shared_ptr<const IonosphericCorrections>& ionosphericCorrections,
                            [[maybe_unused]] const std::string& nameId) const
    {
        Eigen::Vector3d e_recvPosAPC = e_calcRecvPosAPC(freq, receiverType, e_recvPosMarker, gnssObs, nameId);
        Eigen::Vector3d lla_recvPosAPC = trafo::ecef2lla_WGS84(e_recvPosAPC);

        Eigen::Vector3d e_pLOS = e_calcLineOfSightUnitVector(e_recvPosAPC, satInfo->e_satPos);
        Eigen::Vector3d n_lineOfSightUnitVector = trafo::n_Quat_e(lla_recvPosAPC(0), lla_recvPosAPC(1)) * e_pLOS;
        double elevation = calcSatElevation(n_lineOfSightUnitVector);
        double azimuth = calcSatAzimuth(n_lineOfSightUnitVector);

        // Receiver-Satellite Range [m]
        double rho_r_s = (satInfo->e_satPos - e_recvPosAPC).norm();
        // Estimated troposphere propagation error [m]
        double dpsr_T_r_s = calculateTroposphericDelay(lla_recvPosAPC, gnssObs, elevation, azimuth);
        // Estimated ionosphere propagation error [m]
        double dpsr_I_r_s = calculateIonosphericDelay(freq, freqNum, lla_recvPosAPC, gnssObs, ionosphericCorrections, elevation, azimuth);
        // Sagnac correction [m]
        double dpsr_ie_r_s = calcSagnacCorrection(e_recvPosAPC, satInfo->e_satPos);

        // Earth's gravitational field causes relativistic signal delay due to space-time curvature (Shapiro effect) [s]
        // double posNorm = e_satPos.norm() + e_recvPosAPC.norm();
        // double dt_rel_stc = e_recvPosAPC.norm() > InsConst::WGS84::a / 2.0 ? 2.0 * InsConst::WGS84::MU / std::pow(InsConst::C, 3) * std::log((posNorm + rho_r_s) / (posNorm - rho_r_s))
        //                                                                        : 0.0;

        double estimate = rho_r_s
                          + dpsr_ie_r_s
                          + dpsr_T_r_s
                          - dpsr_I_r_s
                          + InsConst::C * (recvClockBias - satInfo->satClockBias /* + dt_rel_stc */);

        return { estimate, e_pLOS };
    }

    /// @brief Calculates the doppler estimate
    /// @param[in] freq Signal frequency
    /// @param[in] receiverType Receiver type to select the correct antenna
    /// @param[in] e_recvPosMarker Receiver marker position in ECEF coordinates [m]
    /// @param[in] e_recvVel Receiver velocity in ECEF frame [m/s]
    /// @param[in] recvClockDrift Receiver clock drift [s/s]
    /// @param[in] satInfo Satellite Information (pos, vel, clock)
    /// @param[in] gnssObs GNSS observation
    /// @param[in] nameId Name and Id of the node used for log messages only
    /// @return The estimate [m/s] and the position line-of-sight unit vector (used for the Jacobian)
    template<typename DerivedPos, typename DerivedVel>
    auto calcDopplerEstimate(Frequency freq,
                             size_t receiverType,
                             const Eigen::MatrixBase<DerivedPos>& e_recvPosMarker,
                             const Eigen::MatrixBase<DerivedVel>& e_recvVel,
                             auto recvClockDrift,
                             const std::shared_ptr<const SatelliteInfo>& satInfo,
                             const std::shared_ptr<const GnssObs>& gnssObs,
                             [[maybe_unused]] const std::string& nameId) const
    {
        auto e_recvPosAPC = e_calcRecvPosAPC(freq, receiverType, e_recvPosMarker, gnssObs, nameId);

        auto e_pLOS = e_calcLineOfSightUnitVector(e_recvPosAPC, satInfo->e_satPos);

        return e_pLOS.dot(satInfo->e_satVel - e_recvVel)
               - calcSagnacRateCorrection(e_recvPosAPC, satInfo->e_satPos, e_recvVel, satInfo->e_satVel)
               + InsConst::C * (recvClockDrift - satInfo->satClockDrift);
    }

    /// @brief Calculates the observation variance
    /// @param[in] freq Signal Frequency
    /// @param[in] freqNum Frequency number. Only used for GLONASS G1 and G2
    /// @param[in] receiverType Receiver type to select the correct antenna
    /// @param[in] obsType Observation type to calculate the variance for
    /// @param[in] e_recvPosMarker Receiver marker position in ECEF coordinates [m]
    /// @param[in] e_satPos Satellite position in ECEF coordinates [m]
    /// @param[in] cn0 Carrier-to-Noise density [dBHz]
    /// @param[in] gnssObs GNSS observation
    /// @param[in] navData Navigation data including this satellite
    /// @param[in] ionosphericCorrections Ionospheric correction parameters collected from the Nav data
    /// @param[in] nameId Name and Id of the node used for log messages only
    /// @param[in] obsDiff Observation Difference type to estimate
    /// @return Observation variance in [m] (pseudorange, carrier) or [m/s] (doppler)
    double calcObservationVariance(Frequency freq,
                                   int8_t freqNum,
                                   size_t receiverType,
                                   GnssObs::ObservationType obsType,
                                   const Eigen::Vector3d& e_recvPosMarker,
                                   const Eigen::Vector3d& e_satPos,
                                   double cn0,
                                   const std::shared_ptr<const GnssObs>& gnssObs,
                                   const std::shared_ptr<const SatNavData>& navData,
                                   const std::shared_ptr<const IonosphericCorrections>& ionosphericCorrections,
                                   [[maybe_unused]] const std::string& nameId,
                                   ObservationDifference obsDiff) const
    {
        const SatelliteSystem satSys = freq.getSatSys();

        Eigen::Vector3d e_recvPosAPC = e_calcRecvPosAPC(freq, receiverType, e_recvPosMarker, gnssObs, nameId);
        Eigen::Vector3d lla_recvPosAPC = trafo::ecef2lla_WGS84(e_recvPosAPC);

        Eigen::Vector3d e_pLOS = e_calcLineOfSightUnitVector(e_recvPosAPC, e_satPos);
        Eigen::Vector3d n_lineOfSightUnitVector = trafo::n_Quat_e(lla_recvPosAPC(0), lla_recvPosAPC(1)) * e_pLOS;
        double elevation = calcSatElevation(n_lineOfSightUnitVector);
        double azimuth = calcSatAzimuth(n_lineOfSightUnitVector);

        // Estimated troposphere propagation error [m]
        double dpsr_T_r_s = calculateTroposphericDelay(lla_recvPosAPC, gnssObs, elevation, azimuth);
        // Estimated ionosphere propagation error [m]
        double dpsr_I_r_s = calculateIonosphericDelay(freq, freqNum, lla_recvPosAPC, gnssObs, ionosphericCorrections, elevation, azimuth);

        double variance = 0.0;

        switch (obsType)
        {
        case GnssObs::Pseudorange:
            variance = _gnssMeasurementErrorModel.psrMeasErrorVar(satSys, elevation, cn0);
            break;
        case GnssObs::Carrier:
            variance = _gnssMeasurementErrorModel.carrierMeasErrorVar(satSys, elevation, cn0);
            break;
        case GnssObs::Doppler:
            variance = _gnssMeasurementErrorModel.psrRateMeasErrorVar(freq, freqNum, elevation, cn0);
            break;
        case GnssObs::ObservationType_COUNT:
            break;
        }

        if (obsDiff == NoDifference)
        {
            if (obsType == GnssObs::Pseudorange || obsType == GnssObs::Carrier)
            {
                variance += navData->calcSatellitePositionVariance()
                            + ionoErrorVar(dpsr_I_r_s, freq, freqNum)
                            + tropoErrorVar(dpsr_T_r_s, elevation);
            }
            if (obsType == GnssObs::Pseudorange)
            {
                variance += _gnssMeasurementErrorModel.codeBiasErrorVar();
            }
        }

        return variance;
    }

    /// @brief Calculates the antenna phase center position for the marker
    /// @param[in] freq Signal frequency
    /// @param[in] receiverType Receiver type to select the correct antenna
    /// @param[in] e_recvPosMarker Receiver marker position in ECEF coordinates [m]
    /// @param[in] gnssObs GNSS observation
    /// @param[in] nameId Name and Id of the node used for log messages only
    template<typename Derived>
    [[nodiscard]] Eigen::Vector3<typename Derived::Scalar> e_calcRecvPosAPC(const Frequency& freq,
                                                                            size_t receiverType,
                                                                            const Eigen::MatrixBase<Derived>& e_recvPosMarker,
                                                                            const std::shared_ptr<const GnssObs>& gnssObs,
                                                                            [[maybe_unused]] const std::string& nameId) const
    {
        const Antenna& antenna = _antenna.at(receiverType);

        auto e_recvPosAPC = trafo::e_posMarker2ARP(e_recvPosMarker,
                                                   gnssObs,
                                                   antenna.hen_delta);
        if (antenna.enabled)
        {
            std::string antennaType;
            if (antenna.autoDetermine && gnssObs->receiverInfo)
            {
                antennaType = gnssObs->receiverInfo->get().antennaType;
            }
            else if (!antenna.autoDetermine)
            {
                antennaType = antenna.name;
            }
            e_recvPosAPC = trafo::e_posARP2APC(e_recvPosAPC, gnssObs, freq, antennaType, nameId);
        }

        return e_recvPosAPC;
    }

    /// @brief Calculate the ionospheric delay with the model selected in the GUI
    /// @param[in] freq Frequency of the signal
    /// @param[in] freqNum Frequency number. Only used for GLONASS G1 and G2
    /// @param[in] lla_recvPosAPC Receiver antenna phase center position in LLA coordinates [rad, rad, m]
    /// @param[in] gnssObs GNSS observation
    /// @param[in] ionosphericCorrections Ionospheric correction parameters collected from the Nav data
    /// @param[in] elevation Satellite elevation [rad]
    /// @param[in] azimuth Satellite azimuth [rad]
    /// @return Estimated ionosphere propagation error [m]
    double calculateIonosphericDelay(const Frequency& freq,
                                     int8_t freqNum,
                                     const Eigen::Vector3d& lla_recvPosAPC,
                                     const std::shared_ptr<const GnssObs>& gnssObs,
                                     const std::shared_ptr<const IonosphericCorrections>& ionosphericCorrections,
                                     double elevation,
                                     double azimuth) const
    {
        return calcIonosphericDelay(static_cast<double>(gnssObs->insTime.toGPSweekTow().tow),
                                    freq, freqNum, lla_recvPosAPC,
                                    elevation, azimuth,
                                    _ionosphereModel, ionosphericCorrections.get());
    }

    /// @brief Calculate the tropospheric delay with the model selected in the GUI
    /// @param[in] lla_recvPosAPC Receiver antenna phase center position in LLA coordinates [rad, rad, m]
    /// @param[in] gnssObs GNSS observation
    /// @param[in] elevation Satellite elevation [rad]
    /// @param[in] azimuth Satellite azimuth [rad]
    /// @return Estimated troposphere propagation error [m]
    double calculateTroposphericDelay(const Eigen::Vector3d& lla_recvPosAPC,
                                      const std::shared_ptr<const GnssObs>& gnssObs,
                                      double elevation,
                                      double azimuth) const
    {
        auto tropo_r_s = calcTroposphericDelayAndMapping(gnssObs->insTime, lla_recvPosAPC,
                                                         elevation, azimuth, _troposphereModels);

        return tropo_r_s.ZHD * tropo_r_s.zhdMappingFactor + tropo_r_s.ZWD * tropo_r_s.zwdMappingFactor;
    }

    /// @brief Shows the GUI input to select the options
    /// @param[in] id Unique id for ImGui.
    /// @param[in] itemWidth Width of the widgets
    template<typename ReceiverType>
    bool ShowGuiWidgets(const char* id, float itemWidth)
    {
        bool changed = false;

        ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
        if (ImGui::TreeNode(fmt::format("Compensation models##{}", id).c_str()))
        {
            ImGui::SetNextItemWidth(itemWidth - ImGui::GetStyle().IndentSpacing);
            if (ComboIonosphereModel(fmt::format("Ionosphere Model##{}", id).c_str(), _ionosphereModel))
            {
                LOG_DEBUG("{}: Ionosphere Model changed to {}", id, NAV::to_string(_ionosphereModel));
                changed = true;
            }
            if (ComboTroposphereModel(fmt::format("Troposphere Model##{}", id).c_str(), _troposphereModels, itemWidth - ImGui::GetStyle().IndentSpacing))
            {
                changed = true;
            }
            ImGui::TreePop();
        }

        ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
        if (ImGui::TreeNode(fmt::format("GNSS Measurement Error##{}", id).c_str()))
        {
            if (_gnssMeasurementErrorModel.ShowGuiWidgets(id, itemWidth - ImGui::GetStyle().IndentSpacing))
            {
                LOG_DEBUG("{}: GNSS Measurement Error Model changed.", id);
                changed = true;
            }
            ImGui::TreePop();
        }

        if (ImGui::TreeNode(fmt::format("Antenna settings##{}", id).c_str()))
        {
            for (size_t i = 0; i < _receiverCount; ++i)
            {
                if (_receiverCount > 1)
                {
                    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
                    if (!ImGui::TreeNode(fmt::format("{}", static_cast<ReceiverType>(i)).c_str())) { continue; }
                }

                if (ImGui::Checkbox(fmt::format("Correct Phase-center offset##antenna {} {}", id, i).c_str(), &_antenna.at(i).enabled))
                {
                    LOG_DEBUG("{}: Antenna {} enabled: {}", id, i, _antenna.at(i).enabled);
                    changed = true;
                }
                ImGui::SameLine();
                if (!_antenna.at(i).enabled) { ImGui::BeginDisabled(); }
                if (ImGui::Checkbox(fmt::format("Auto determine##antenna {} {}", id, i).c_str(), &_antenna.at(i).autoDetermine))
                {
                    LOG_DEBUG("{}: Antenna {} auto: {}", id, i, _antenna.at(i).autoDetermine);
                    changed = true;
                }
                if (_antenna.at(i).autoDetermine) { ImGui::BeginDisabled(); }
                ImGui::SetNextItemWidth(itemWidth - (1.0F + static_cast<float>(_receiverCount > 1)) * ImGui::GetStyle().IndentSpacing);

                if (ImGui::BeginCombo(fmt::format("Type##Combo {} {}", id, i).c_str(), _antenna.at(i).name.c_str()))
                {
                    ImGui::PushFont(Application::MonoFont());
                    if (ImGui::Selectable(fmt::format("##Selectable {} {}", id, i).c_str(), _antenna.at(i).name.empty()))
                    {
                        _antenna.at(i).name = "";
                    }

                    for (const auto& iter : AntexReader::Get().antennas())
                    {
                        const bool is_selected = (_antenna.at(i).name == iter);
                        if (ImGui::Selectable(iter.c_str(), is_selected))
                        {
                            _antenna.at(i).name = iter;
                        }
                        if (is_selected) { ImGui::SetItemDefaultFocus(); }
                    }
                    ImGui::PopFont();
                    ImGui::EndCombo();
                }
                if (_antenna.at(i).autoDetermine) { ImGui::EndDisabled(); }
                if (!_antenna.at(i).enabled) { ImGui::EndDisabled(); }
                ImGui::SameLine();
                gui::widgets::HelpMarker("Used for lookup in ANTEX file.");

                ImGui::SetNextItemWidth(itemWidth - (1.0F + static_cast<float>(_receiverCount > 1)) * ImGui::GetStyle().IndentSpacing);
                if (ImGui::InputDouble3(fmt::format("Delta H/E/N [m]##{} {}", id, i).c_str(), _antenna.at(i).hen_delta.data(), "%.3f"))
                {
                    LOG_DEBUG("{}: Antenna {} delta: {}", id, i, _antenna.at(i).hen_delta.transpose());
                    changed = true;
                }
                ImGui::SameLine();
                gui::widgets::HelpMarker("- Antenna height: Height of the antenna reference point (ARP) above the marker\n"
                                         "- Horizontal eccentricity of ARP relative to the marker (east/north)");

                if (_receiverCount > 1) { ImGui::TreePop(); }
            }
            ImGui::TreePop();
        }

        return changed;
    }

    /// @brief Get the Ionosphere Model selected
    [[nodiscard]] const IonosphereModel& getIonosphereModel() const { return _ionosphereModel; }
    /// @brief Get the Troposphere Model selection
    [[nodiscard]] const TroposphereModelSelection& getTroposphereModels() const { return _troposphereModels; }
    /// @brief Get the GNSS measurement error model
    [[nodiscard]] const GnssMeasurementErrorModel& getGnssMeasurementErrorModel() const { return _gnssMeasurementErrorModel; }

  private:
    IonosphereModel _ionosphereModel = IonosphereModel::Klobuchar; ///< Ionosphere Model used for the calculation
    TroposphereModelSelection _troposphereModels;                  ///< Troposphere Models used for the calculation
    GnssMeasurementErrorModel _gnssMeasurementErrorModel;          ///< GNSS measurement error model to use

    size_t _receiverCount; ///< Number of receivers (used for GUI)

    /// Antenna information
    struct Antenna
    {
        bool enabled = true;                                 ///< Enabled
        bool autoDetermine = true;                           ///< Try determine antenna type from e.g. RINEX file
        std::string name;                                    ///< Type of the antenna for the Antex lookup
        Eigen::Vector3d hen_delta = Eigen::Vector3d::Zero(); ///< Delta values of marker to antenna base
    };

    std::unordered_map<size_t, Antenna> _antenna; ///< User antenna selection. Key is receiver type

    /// @brief Converts the provided object into json
    /// @param[out] j Json object which gets filled with the info
    /// @param[in] obj Object to convert into json
    friend void to_json(json& j, const ObservationEstimator::Antenna& obj)
    {
        j = json{
            { "enabled", obj.enabled },
            { "autoDetermine", obj.autoDetermine },
            { "name", obj.name },
            { "hen_delta", obj.hen_delta },
        };
    }
    /// @brief Converts the provided json object into a node object
    /// @param[in] j Json object with the needed values
    /// @param[out] obj Object to fill from the json
    friend void from_json(const json& j, ObservationEstimator::Antenna& obj)
    {
        if (j.contains("enabled")) { j.at("enabled").get_to(obj.enabled); }
        if (j.contains("autoDetermine")) { j.at("autoDetermine").get_to(obj.autoDetermine); }
        if (j.contains("name")) { j.at("name").get_to(obj.name); }
        if (j.contains("hen_delta")) { j.at("hen_delta").get_to(obj.hen_delta); }
    }

    /// @brief Converts the provided object into json
    /// @param[out] j Json object which gets filled with the info
    /// @param[in] obj Object to convert into json
    friend void to_json(json& j, const ObservationEstimator& obj)
    {
        j = json{
            { "ionosphereModel", Frequency_(obj._ionosphereModel) },
            { "troposphereModels", obj._troposphereModels },
            { "gnssMeasurementError", obj._gnssMeasurementErrorModel },
            { "antenna", obj._antenna },
        };
    }
    /// @brief Converts the provided json object into a node object
    /// @param[in] j Json object with the needed values
    /// @param[out] obj Object to fill from the json
    friend void from_json(const json& j, ObservationEstimator& obj)
    {
        if (j.contains("ionosphereModel")) { j.at("ionosphereModel").get_to(obj._ionosphereModel); }
        if (j.contains("troposphereModels")) { j.at("troposphereModels").get_to(obj._troposphereModels); }
        if (j.contains("gnssMeasurementError")) { j.at("gnssMeasurementError").get_to(obj._gnssMeasurementErrorModel); }
        if (j.contains("antenna")) { j.at("antenna").get_to(obj._antenna); }
    }
};

} // namespace NAV
