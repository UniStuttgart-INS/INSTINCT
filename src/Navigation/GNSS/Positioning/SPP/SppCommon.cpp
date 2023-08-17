// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file SppCommon.cpp
/// @brief Common Functions for the SPP algorithm
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-07-21

#include "SppCommon.hpp"

#include "Navigation/GNSS/Satellite/Ephemeris/GLONASSEphemeris.hpp"
#include "Navigation/GNSS/Functions.hpp"
#include "Navigation/GNSS/Errors.hpp"
#include "Navigation/Transformations/Units.hpp"

namespace NAV::GNSS::Positioning::SPP
{

std::vector<CalcData> selectObservations(const std::shared_ptr<SppSolution>& sppSol,
                                         const std::shared_ptr<const GnssObs>& gnssObs,
                                         const std::vector<const GnssNavInfo*>& gnssNavInfos,
                                         const Frequency& filterFreq,
                                         const Code& filterCode,
                                         const std::vector<SatId>& excludedSatellites)
{
    std::vector<CalcData> calcData;
    for (const auto& obsData : gnssObs->data)
    {
        auto satId = obsData.satSigId.toSatId();

        if ((obsData.satSigId.freq() & filterFreq)                                                                 // frequency is selected in GUI
            && (obsData.satSigId.code & filterCode)                                                                // code is selected in GUI
            && obsData.pseudorange                                                                                 // has a valid pseudorange
            && std::find(excludedSatellites.begin(), excludedSatellites.end(), satId) == excludedSatellites.end()) // is not excluded
        {
            for (const auto& gnssNavInfo : gnssNavInfos)
            {
                if (auto satNavData = gnssNavInfo->searchNavigationData(satId, gnssObs->insTime)) // can calculate satellite position
                {
                    if (!satNavData->isHealthy())
                    {
                        LOG_DATA("Satellite {} is skipped because the signal is not healthy.", satId);
                        (*sppSol)(obsData.satSigId).skipped = true;

                        continue;
                    }
                    LOG_DATA("Using observation from {}", obsData.satSigId);
                    calcData.emplace_back(obsData, satNavData);
                    calcData.back().pseudorangeEst = obsData.pseudorange.value().value;
                    break;
                }
            }
        }
    }

    return calcData;
}

size_t findDopplerMeasurements(std::vector<CalcData>& calcData)
{
    size_t nDopplerMeas = 0;
    for (auto& calc : calcData)
    {
        const auto& obsData = calc.obsData;
        if (obsData.doppler)
        {
            nDopplerMeas++;
            // TODO: Find out what this is used for and find a way to use it, after the GLONASS orbit calculation is working
            if (obsData.satSigId.freq() & (R01 | R02))
            {
                if (auto satNavData = std::dynamic_pointer_cast<GLONASSEphemeris>(calc.satNavData))
                {
                    calc.freqNum = satNavData->frequencyNumber;
                }
            }

            calc.pseudorangeRateMeas = doppler2psrRate(obsData.doppler.value(), obsData.satSigId.freq());
        }
    }
    return nDopplerMeas;
}

ValueWeight<double> calcPsrAndWeight(const std::shared_ptr<SppSolution>& sppSol,
                                     const CalcData& calc,
                                     const InsTime& insTime,
                                     State& state,
                                     const Eigen::Vector3d& lla_pos,
                                     const IonosphericCorrections& ionosphericCorrections,
                                     const IonosphereModel& ionosphereModel,
                                     const TroposphereModelSelection& troposphereModels,
                                     const GnssMeasurementErrorModel& gnssMeasurementErrorModel,
                                     const EstimatorType& estimatorType)
{
    const auto& obsData = calc.obsData;
    auto& solSatData = (*sppSol)(obsData.satSigId);
    auto satSys = obsData.satSigId.toSatId().satSys;

    // Estimated modulation ionosphere propagation error [m]
    double dpsr_I = calcIonosphericDelay(static_cast<double>(insTime.toGPSweekTow().tow), obsData.satSigId.freq(), calc.freqNum,
                                         lla_pos, calc.satElevation, calc.satAzimuth, ionosphereModel, &ionosphericCorrections);
    LOG_DATA("         dpsr_I {} [m] (Estimated modulation ionosphere propagation error)", dpsr_I);
    solSatData.dpsr_I = dpsr_I;

    auto tropo = calcTroposphericDelayAndMapping(insTime, lla_pos, calc.satElevation, calc.satAzimuth, troposphereModels);
    LOG_DATA("         ZHD {}", tropo.ZHD);
    LOG_DATA("         ZWD {}", tropo.ZWD);
    LOG_DATA("         zhdMappingFactor {}", tropo.zhdMappingFactor);
    LOG_DATA("         zwdMappingFactor {}", tropo.zwdMappingFactor);

    // Estimated modulation troposphere propagation error [m]
    double dpsr_T = tropo.ZHD * tropo.zhdMappingFactor + tropo.ZWD * tropo.zwdMappingFactor;
    LOG_DATA("         dpsr_T {} [m] (Estimated modulation troposphere propagation error)", dpsr_T);
    solSatData.dpsr_T = dpsr_T;

    // Sagnac correction [m]
    double dpsr_ie = calcSagnacCorrection(state.e_position, calc.e_satPos);
    LOG_DATA("         dpsr_ie {} [m]", dpsr_ie);
    solSatData.dpsr_ie = dpsr_ie;
    // Geometric distance [m]
    double geometricDist = (calc.e_satPos - state.e_position).norm();
    LOG_DATA("         geometricDist {} [m]", geometricDist);
    solSatData.geometricDist = geometricDist;
    // System time difference to GPS [s]
    double sysTimeDiff = satSys != state.recvClk.referenceTimeSatelliteSystem && state.recvClk.sysTimeDiff.contains(satSys)
                             ? state.recvClk.sysTimeDiff[satSys].value
                             : 0.0;
    solSatData.dpsr_clkISB = sysTimeDiff * InsConst::C;
    LOG_DATA("         recClkBias  {} [m]", state.recvClk.bias.value * InsConst::C);
    LOG_DATA("         satClkBias  {} [m]", calc.satClkBias * InsConst::C);
    LOG_DATA("         dpsr_clkISB {} [m]", sysTimeDiff * InsConst::C);

    // Pseudorange estimate [m]
    double psrEst = geometricDist
                    + state.recvClk.bias.value * InsConst::C
                    + sysTimeDiff * InsConst::C
                    - calc.satClkBias * InsConst::C
                    + dpsr_I
                    + dpsr_T
                    + dpsr_ie;
    LOG_DATA("         psrEst {} [m]", psrEst);

    double W_psr = 1.0;
    if (estimatorType == EstimatorType::WEIGHTED_LEAST_SQUARES || estimatorType == EstimatorType::KF)
    {
        // Weight matrix - RTKLIB eq. E6.23, p. 158

        double varPsrMeas = gnssMeasurementErrorModel.psrMeasErrorVar(obsData.satSigId.toSatId().satSys, obsData.satSigId.freq(), calc.satElevation);
        LOG_DATA("         varPsrMeas {}", varPsrMeas);
        double varEph = calc.satNavData->calcSatellitePositionVariance();
        LOG_DATA("         varEph {}", varEph);
        double varIono = ionoErrorVar(dpsr_I, obsData.satSigId.freq(), calc.freqNum);
        LOG_DATA("         varIono {}", varIono);
        double varTrop = tropoErrorVar(dpsr_T, calc.satElevation);
        LOG_DATA("         varTrop {}", varTrop);
        double varBias = gnssMeasurementErrorModel.codeBiasErrorVar();
        LOG_DATA("         varBias {}", varBias);
        double varErrors = varPsrMeas + varEph + varIono + varTrop + varBias;
        LOG_DATA("         varErrors {}", varErrors);

        W_psr = 1.0 / varErrors;
        LOG_DATA("         W_psr {}", W_psr);
    }

    return { psrEst, W_psr };
}

ValueWeight<double> calcPsrRateAndWeight(const CalcData& calc,
                                         State& state,
                                         const EstimatorType& estimatorType,
                                         const GnssMeasurementErrorModel& gnssMeasurementErrorModel)
{
    const auto& obsData = calc.obsData;
    auto satSys = obsData.satSigId.toSatId().satSys;

    // Range-rate Sagnac correction [m/s]
    double dpsr_dot_ie = calcSagnacRateCorrection(state.e_position, calc.e_satPos, state.e_velocity, calc.e_satVel);
    LOG_DATA("         dpsr_dot_ie {}", dpsr_dot_ie);
    // System time drift difference to GPS [s/s]
    double sysDriftDiff = satSys != state.recvClk.referenceTimeSatelliteSystem
                              ? state.recvClk.sysDriftDiff[satSys].value
                              : 0.0;

    // Pseudorange-rate estimate [m/s] - Groves ch. 9.4.1, eq. 9.142, p. 412 (Sagnac correction different sign)
    double psrRateEst = calc.e_lineOfSightUnitVector.transpose() * (calc.e_satVel - state.e_velocity)
                        + state.recvClk.drift.value * InsConst::C
                        + sysDriftDiff * InsConst::C
                        - calc.satClkDrift * InsConst::C
                        - dpsr_dot_ie;
    LOG_DATA("         psrRateEst {}", psrRateEst);

    // Weight matrix
    double W_psrRate = 1.0;
    if (estimatorType == EstimatorType::WEIGHTED_LEAST_SQUARES || estimatorType == EstimatorType::KF)
    {
        double varPsrRateMeas = gnssMeasurementErrorModel.psrRateErrorVar(obsData.satSigId.freq());
        LOG_DATA("         varPsrRateMeas {}", varPsrRateMeas);
        double varEph = calc.satNavData->calcSatellitePositionVariance();
        LOG_DATA("         varEph {}", varEph);
        double varErrors = varPsrRateMeas + varEph;
        LOG_DATA("         varErrors {}", varErrors);

        W_psrRate = 1.0 / varErrors;
        LOG_DATA("         W_psrRate {}", W_psrRate);
    }

    return { psrRateEst, W_psrRate };
}

EstWeightDesignMatrices calcMeasurementEstimatesAndDesignMatrix(const std::shared_ptr<SppSolution>& sppSol,
                                                                std::vector<CalcData>& calcData,
                                                                int nParam,
                                                                int nMeasPsr,
                                                                int nMeasDoppler,
                                                                const InsTime& insTime,
                                                                State& state,
                                                                const Eigen::Vector3d& lla_pos,
                                                                const std::vector<SatelliteSystem>& satelliteSystems,
                                                                const IonosphericCorrections& ionosphericCorrections,
                                                                const IonosphereModel& ionosphereModel,
                                                                const TroposphereModelSelection& troposphereModels,
                                                                const GnssMeasurementErrorModel& gnssMeasurementErrorModel,
                                                                const EstimatorType& estimatorType)
{
    EstWeightDesignMatrices retVal;

    retVal.e_H_psr = Eigen::MatrixXd::Zero(nMeasPsr, nParam);
    retVal.psrEst = Eigen::VectorXd::Zero(nMeasPsr);
    retVal.psrMeas = Eigen::VectorXd::Zero(nMeasPsr);
    retVal.W_psr = Eigen::MatrixXd::Zero(nMeasPsr, nMeasPsr);

    retVal.e_H_r = Eigen::MatrixXd::Zero(nMeasDoppler, nParam);
    retVal.psrRateEst = Eigen::VectorXd::Zero(nMeasDoppler);
    retVal.psrRateMeas = Eigen::VectorXd::Zero(nMeasDoppler);
    retVal.W_psrRate = Eigen::MatrixXd::Zero(nMeasDoppler, nMeasDoppler);

    int ix = 0;
    int iv = 0;

    for (auto& calc : calcData)
    {
        if (calc.skipped) { continue; }

        const auto& obsData = calc.obsData;
        LOG_DATA("     satellite {}", obsData.satSigId);
        auto satId = obsData.satSigId.toSatId();

        auto& solSatData = (*sppSol)(obsData.satSigId);

        // #############################################################################################################################
        //                                                    Position calculation
        // #############################################################################################################################

        // Pseudorange measurement [m] - Groves ch. 8.5.3, eq. 8.48, p. 342
        retVal.psrMeas(ix) = obsData.pseudorange.value().value; // + (multipath and/or NLOS errors) + (tracking errors)
        LOG_DATA("         psrMeas({}) {}", ix, retVal.psrMeas(ix));

        // Pseudorange estimate [m] and Weight of pseudorange measurement [1/m²]
        auto [psrEst_i, W_psr_i] = calcPsrAndWeight(sppSol, calc, insTime, state, lla_pos,
                                                    ionosphericCorrections, ionosphereModel, troposphereModels,
                                                    gnssMeasurementErrorModel, estimatorType);
        retVal.psrEst(ix) = psrEst_i;
        calc.pseudorangeEst = retVal.psrEst(ix);
        solSatData.psrEst = retVal.psrEst(ix);
        retVal.W_psr(ix, ix) = W_psr_i;

        // Measurement/Geometry matrix - Groves ch. 9.4.1, eq. 9.144, p. 412
        retVal.e_H_psr.block<1, 3>(ix, 0) = -calc.e_lineOfSightUnitVector;
        retVal.e_H_psr(ix, 3) = 1;
        for (size_t s = 0; s < satelliteSystems.size(); s++)
        {
            if (satId.satSys != state.recvClk.referenceTimeSatelliteSystem
                && satId.satSys == satelliteSystems.at(s))
            {
                retVal.e_H_psr(ix, 4 + static_cast<int>(s)) = 1;
            }
        }
        LOG_DATA("         e_H_psr.row({}) {}", ix, retVal.e_H_psr.row(ix));

        // #############################################################################################################################
        //                                                    Velocity calculation
        // #############################################################################################################################

        if (calc.pseudorangeRateMeas)
        {
            // Pseudorange-rate measurement [m/s] - Groves ch. 8.5.3, eq. 8.48, p. 342
            retVal.psrRateMeas(iv) = calc.pseudorangeRateMeas.value(); // + (multipath and/or NLOS errors) + (tracking errors)
            LOG_DATA("         psrRateMeas({}) {}", iv, retVal.psrRateMeas(iv));

            auto [psrRateEst_i, W_psrRate_i] = calcPsrRateAndWeight(calc, state, estimatorType, gnssMeasurementErrorModel);
            retVal.psrRateEst(iv) = psrRateEst_i;
            solSatData.psrRateEst = retVal.psrRateEst(iv);
            retVal.W_psrRate(iv, iv) = W_psrRate_i;

            // Measurement/Geometry matrix - Groves ch. 9.4.1, eq. 9.144, p. 412
            retVal.e_H_r.row(iv) = retVal.e_H_psr.row(ix);

            iv++;
        }

        ix++;
    }

    retVal.dpsr = retVal.psrMeas - retVal.psrEst;
    retVal.dpsr_dot = retVal.psrRateMeas - retVal.psrRateEst;

    return retVal;
}

bool calcDataBasedOnEstimates(const std::shared_ptr<SppSolution>& sppSol,
                              std::vector<SatelliteSystem>& satelliteSystems,
                              size_t& cntSkippedMeas,
                              std::vector<CalcData>& calcData,
                              State& state,
                              size_t nParam,
                              size_t nMeasPsr,
                              size_t nMeasDoppler,
                              const InsTime& insTime,
                              const Eigen::Vector3d& lla_pos,
                              double elevationMask)
{
    cntSkippedMeas = 0;
    SatelliteSystem_ usedSatelliteSystems = SatSys_None;
    for (size_t i = 0; i < calcData.size(); i++)
    {
        auto& calc = calcData[i];

        const auto& obsData = calc.obsData;
        LOG_DATA("     satellite {}", obsData.satSigId);
        auto satId = obsData.satSigId.toSatId();

        // Calculate satellite clock, position and velocity
        LOG_DATA("         pseudorangeEst {}", calc.pseudorangeEst);

        auto satClk = calc.satNavData->calcClockCorrections(insTime, calc.pseudorangeEst, obsData.satSigId.freq());
        calc.satClkBias = satClk.bias;
        calc.satClkDrift = satClk.drift;
        LOG_DATA("         satClkBias {}, satClkDrift {}", calc.satClkBias, calc.satClkDrift);

        auto satPosVel = calc.satNavData->calcSatellitePosVel(satClk.transmitTime);
        calc.e_satPos = satPosVel.e_pos;
        calc.e_satVel = satPosVel.e_vel;
        LOG_DATA("         e_satPos {}", calc.e_satPos.transpose());
        LOG_DATA("         e_satVel {}", calc.e_satVel.transpose());

        // Line-of-sight unit vector in ECEF frame coordinates - Groves ch. 8.5.3, eq. 8.41, p. 341
        calc.e_lineOfSightUnitVector = e_calcLineOfSightUnitVector(state.e_position, calc.e_satPos);
        LOG_DATA("         e_lineOfSightUnitVector {}", calc.e_lineOfSightUnitVector.transpose());
        // Line-of-sight unit vector in NED frame coordinates - Groves ch. 8.5.3, eq. 8.41, p. 341
        calc.n_lineOfSightUnitVector = trafo::n_Quat_e(lla_pos(0), lla_pos(1)) * calc.e_lineOfSightUnitVector;
        LOG_DATA("         n_lineOfSightUnitVector {}", calc.n_lineOfSightUnitVector.transpose());
        // Elevation [rad] - Groves ch. 8.5.4, eq. 8.57, p. 344
        calc.satElevation = calcSatElevation(calc.n_lineOfSightUnitVector);
        LOG_DATA("         satElevation {}°", rad2deg(calc.satElevation));
        // Azimuth [rad] - Groves ch. 8.5.4, eq. 8.57, p. 344
        calc.satAzimuth = calcSatAzimuth(calc.n_lineOfSightUnitVector);
        LOG_DATA("         satAzimuth {}°", rad2deg(calc.satAzimuth));

        auto& solSatData = (*sppSol)(obsData.satSigId);
        solSatData.transmitTime = satClk.transmitTime;
        solSatData.satClkBias = satClk.bias;
        solSatData.satClkDrift = satClk.drift;
        solSatData.e_satPos = calc.e_satPos;
        solSatData.e_satVel = calc.e_satVel;
        solSatData.satElevation = calc.satElevation;
        solSatData.satAzimuth = calc.satAzimuth;

        if (!state.e_position.isZero() && calc.satElevation < elevationMask) // Do not check elevation mask when not having a valid position
        {
            cntSkippedMeas++;
            calc.skipped = true;
            LOG_DATA("         [{}] Measurement is skipped because of elevation {:.1f}° and mask of {}° ({} valid measurements remaining)",
                     obsData.satSigId, rad2deg(calc.satElevation), rad2deg(elevationMask), nMeasPsr - cntSkippedMeas);

            if (!(usedSatelliteSystems & satId.satSys)
                && calcData.begin() + static_cast<int64_t>(i + 1) != calcData.end()                                         // This is the last satellite and the system did not appear before
                && std::none_of(calcData.begin() + static_cast<int64_t>(i + 1), calcData.end(), [&](const CalcData& data) { // The satellite system has no satellites available anymore
                       return data.obsData.satSigId.toSatId().satSys == satId.satSys;
                   }))
            {
                LOG_DEBUG("The satellite system {} won't be used this iteration because no satellite complies with the elevation mask.",
                          satId.satSys);
                nParam--;
                satelliteSystems.erase(std::find(satelliteSystems.begin(), satelliteSystems.end(), satId.satSys));
            }

            solSatData.elevationMaskTriggered = true;

            if (nMeasPsr - cntSkippedMeas < nParam)
            {
                LOG_ERROR(" Cannot calculate position because only {} valid measurements ({} needed). Try changing filter settings or reposition your antenna.",
                          insTime, nMeasPsr - cntSkippedMeas, nParam);
                sppSol->nSatellitesPosition = nMeasPsr - cntSkippedMeas;
                sppSol->nSatellitesVelocity = nMeasDoppler - cntSkippedMeas;
                return false;
            }
            continue;
        }
        solSatData.elevationMaskTriggered = false;

        usedSatelliteSystems |= satId.satSys;
    }
    // sort used satellite systems of this epoch (for better handling later) and use first one as reference time (or in KF: use the one from first epoch if still available)
    std::sort(satelliteSystems.begin(), satelliteSystems.end());

    // TODO std::find(satelliteSystems.begin(), satelliteSystems.end(), _kalmanFilterReferenceTimeSatelliteSystem) != satelliteSystems.end() ? _recvClk.referenceTimeSatelliteSystem = _kalmanFilterReferenceTimeSatelliteSystem : _recvClk.referenceTimeSatelliteSystem = satelliteSystems.front();

    // Choose reference time satellite system
    state.recvClk.referenceTimeSatelliteSystem = satelliteSystems.front();
    for (const auto& availSatSys : satelliteSystems)
    {
        if (SatelliteSystem_(availSatSys) < SatelliteSystem_(state.recvClk.referenceTimeSatelliteSystem))
        {
            state.recvClk.referenceTimeSatelliteSystem = availSatSys;
        }
    }
    satelliteSystems.erase(std::find(satelliteSystems.begin(), satelliteSystems.end(), state.recvClk.referenceTimeSatelliteSystem));
    LOG_DATA("     referenceTimeSatelliteSystem {} ({} other time systems)", state.recvClk.referenceTimeSatelliteSystem, satelliteSystems.size());
    sppSol->recvClk.referenceTimeSatelliteSystem = state.recvClk.referenceTimeSatelliteSystem;

    return true;
}

} // namespace NAV::GNSS::Positioning::SPP