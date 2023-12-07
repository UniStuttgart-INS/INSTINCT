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

std::vector<CalcData> selectObservations(const std::shared_ptr<const GnssObs>& gnssObs,
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
                        continue;
                    }
                    LOG_DATA("Using observation from {}", obsData.satSigId);
                    calcData.emplace_back(obsData, satNavData);
                    calcData.back().pseudorangeEst = obsData.pseudorange.value().value;
                    if (obsData.satSigId.freq() & (R01 | R02))
                    {
                        if (auto satNavData = std::dynamic_pointer_cast<GLONASSEphemeris>(calcData.back().satNavData))
                        {
                            calcData.back().freqNum = satNavData->frequencyNumber;
                        }
                    }
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
            // if (obsData.satSigId.freq() & (R01 | R02))
            // {
            //     if (auto satNavData = std::dynamic_pointer_cast<GLONASSEphemeris>(calc.satNavData))
            //     {
            //         // calc.freqNum = satNavData->frequencyNumber;
            //     }
            // }

            calc.pseudorangeRateMeas = doppler2rangeRate(obsData.doppler.value(), obsData.satSigId.freq(), calc.freqNum);
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
        if (gnssMeasurementErrorModel.model == GnssMeasurementErrorModel::RTKLIB)
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
        }
        else if (gnssMeasurementErrorModel.model == GnssMeasurementErrorModel::Groves2013)
        {
            // Weight matrix - Groves 2013, see ch. 9.4.2.4, eq. 9.168, p. 422 (range acceleration is neglected)
            W_psr = gnssMeasurementErrorModel.psrMeasErrorVar(calc.satElevation, std::pow(10, calc.obsData.CN0.value() / 10));
        }
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
        if (gnssMeasurementErrorModel.model == GnssMeasurementErrorModel::RTKLIB)
        {
            double varPsrRateMeas = gnssMeasurementErrorModel.psrRateErrorVar(obsData.satSigId.freq(), calc.freqNum);
            LOG_DATA("         varPsrRateMeas {}", varPsrRateMeas);
            double varEph = calc.satNavData->calcSatellitePositionVariance();
            LOG_DATA("         varEph {}", varEph);
            double varErrors = varPsrRateMeas + varEph;
            LOG_DATA("         varErrors {}", varErrors);

            W_psrRate = 1.0 / varErrors;
        }
        else if (gnssMeasurementErrorModel.model == GnssMeasurementErrorModel::Groves2013)
        {
            // Weight matrix - Groves 2013, see ch. 9.4.2.4, eq. 9.168, p. 422 (range acceleration is neglected)
            W_psrRate = gnssMeasurementErrorModel.psrMeasErrorVar(calc.satElevation, std::pow(10, calc.obsData.CN0.value() / 10));
        }
        LOG_DATA("         W_psrRate {}", W_psrRate);
    }

    return { psrRateEst, W_psrRate };
}

EstWeightDesignMatrices calcMeasurementEstimatesAndDesignMatrix(const std::shared_ptr<SppSolution>& sppSol,
                                                                std::vector<CalcData>& calcData,
                                                                const InsTime& insTime,
                                                                State& state,
                                                                const Eigen::Vector3d& lla_pos,
                                                                const IonosphericCorrections& ionosphericCorrections,
                                                                const IonosphereModel& ionosphereModel,
                                                                const TroposphereModelSelection& troposphereModels,
                                                                const GnssMeasurementErrorModel& gnssMeasurementErrorModel,
                                                                const EstimatorType& estimatorType,
                                                                bool useDoppler,
                                                                const std::vector<States::StateKeyTypes>& interSysErrs,
                                                                const std::vector<States::StateKeyTypes>& interSysDrifts)
{
    int nParamFix = 4;                                      // Number of fix LSE parameters (position and receiver clock error or velocity and receiver clock drift respectively)
    int nMeasPsr = static_cast<int>(sppSol->nMeasPsr);      // Number of pseudorange measurements
    int nMeasDoppler = static_cast<int>(sppSol->nMeasDopp); // Number of doppler/pseudorange rate measurements

    // Get Measurement keys
    std::vector<Meas::MeasKeyTypes> measKeysPsr;
    std::vector<Meas::MeasKeyTypes> measKeysPsrRate;
    measKeysPsr.reserve(calcData.size());
    measKeysPsrRate.reserve(calcData.size());

    [[maybe_unused]] std::string measurementsUsed;
    [[maybe_unused]] std::string measurementsSkipped;
    for (auto& calc : calcData)
    {
        if (calc.skipped)
        {
            measurementsSkipped += fmt::format("{}, ", calc.obsData.satSigId);
            continue;
        }
        measurementsUsed += fmt::format("{}, ", calc.obsData.satSigId);
        measKeysPsr.emplace_back(Meas::Psr{ calc.obsData.satSigId });
        if (calc.pseudorangeRateMeas)
        {
            measKeysPsrRate.emplace_back(Meas::Doppler{ calc.obsData.satSigId });
        }
    }
    LOG_DATA("Using   measurement: {}", measurementsUsed);
    LOG_DATA("Skipped measurement: {}", measurementsSkipped);

    EstWeightDesignMatrices retVal;

    retVal.e_H_psr = KeyedMatrixXd<Meas::MeasKeyTypes, States::StateKeyTypes>(Eigen::MatrixXd::Zero(nMeasPsr, nParamFix), measKeysPsr, States::PosRecvClkErr);
    retVal.e_H_psr.addCols(interSysErrs);
    retVal.psrEst = KeyedVectorXd<Meas::MeasKeyTypes>(Eigen::VectorXd::Zero(nMeasPsr), measKeysPsr);
    retVal.psrMeas = KeyedVectorXd<Meas::MeasKeyTypes>(Eigen::VectorXd::Zero(nMeasPsr), measKeysPsr);
    retVal.W_psr = KeyedMatrixXd<Meas::MeasKeyTypes, Meas::MeasKeyTypes>(Eigen::MatrixXd::Zero(nMeasPsr, nMeasPsr), measKeysPsr, measKeysPsr);
    retVal.dpsr = KeyedVectorXd<Meas::MeasKeyTypes>(Eigen::VectorXd::Zero(nMeasPsr), measKeysPsr);

    retVal.e_H_r = KeyedMatrixXd<Meas::MeasKeyTypes, States::StateKeyTypes>(Eigen::MatrixXd::Zero(nMeasDoppler, nParamFix), measKeysPsrRate, States::VelRecvClkDrift);
    retVal.e_H_r.addCols(interSysDrifts);
    retVal.psrRateEst = KeyedVectorXd<Meas::MeasKeyTypes>(Eigen::VectorXd::Zero(nMeasDoppler), measKeysPsrRate);
    retVal.psrRateMeas = KeyedVectorXd<Meas::MeasKeyTypes>(Eigen::VectorXd::Zero(nMeasDoppler), measKeysPsrRate);
    retVal.W_psrRate = KeyedMatrixXd<Meas::MeasKeyTypes, Meas::MeasKeyTypes>(Eigen::MatrixXd::Zero(nMeasDoppler, nMeasDoppler), measKeysPsrRate, measKeysPsrRate);
    retVal.dpsr_dot = KeyedVectorXd<Meas::MeasKeyTypes>(Eigen::VectorXd::Zero(nMeasDoppler), measKeysPsrRate);

    for (auto& calc : calcData)
    {
        if (calc.skipped) { continue; }

        const auto& obsData = calc.obsData;
        LOG_DATA("     satellite {}", obsData.satSigId);
        auto satSigId = obsData.satSigId;

        auto& solSatData = (*sppSol)(obsData.satSigId);

        // #############################################################################################################################
        //                                                    Position calculation
        // #############################################################################################################################

        // Pseudorange measurement [m] - Groves ch. 8.5.3, eq. 8.48, p. 342
        retVal.psrMeas(Meas::Psr{ satSigId }) = obsData.pseudorange.value().value; // + (multipath and/or NLOS errors) + (tracking errors)
        LOG_DATA("         psrMeas({}) = {}", Meas::Psr{ satSigId }, retVal.psrMeas(Meas::Psr{ satSigId }));

        // Pseudorange estimate [m] and Weight of pseudorange measurement [1/m²]
        auto [psrEst_i, W_psr_i] = calcPsrAndWeight(sppSol, calc, insTime, state, lla_pos,
                                                    ionosphericCorrections, ionosphereModel, troposphereModels,
                                                    gnssMeasurementErrorModel, estimatorType);
        retVal.psrEst(Meas::Psr{ satSigId }) = psrEst_i;
        LOG_DATA("         psrEst({}) = {}", Meas::Psr{ satSigId }, retVal.psrEst(Meas::Psr{ satSigId }));
        calc.pseudorangeEst = psrEst_i;
        solSatData.psrEst = psrEst_i;
        retVal.W_psr(Meas::Psr{ satSigId }, Meas::Psr{ satSigId }) = W_psr_i;
        LOG_DATA("         W_psr({},{}) = {}", Meas::Psr{ satSigId }, Meas::Psr{ satSigId }, retVal.W_psr(Meas::Psr{ satSigId }, Meas::Psr{ satSigId }));

        // Measurement/Geometry matrix - Groves ch. 9.4.1, eq. 9.144, p. 412
        retVal.e_H_psr(Meas::Psr{ satSigId }, States::Pos) = -calc.e_lineOfSightUnitVector.transpose();
        retVal.e_H_psr(Meas::Psr{ satSigId }, States::SppStates::RecvClkErr) = 1;
        if (sppSol->recvClk.referenceTimeSatelliteSystem != satSigId.toSatId().satSys)
        {
            for (const auto& interSysErr : interSysErrs)
            {
                if (std::get_if<States::InterSysErr>(&interSysErr)->satSys == satSigId.toSatId().satSys)
                {
                    retVal.e_H_psr(Meas::Psr{ satSigId }, interSysErr) = 1;
                }
            }
        }
        LOG_DATA("         e_H_psr.row({}) = {}", Meas::Psr{ satSigId }, retVal.e_H_psr(Meas::Psr{ satSigId }, all));

        retVal.dpsr(Meas::Psr{ satSigId }) = retVal.psrMeas(Meas::Psr{ satSigId }) - psrEst_i;
        LOG_DATA("         dpsr({}) = {}", Meas::Psr{ satSigId }, retVal.dpsr(Meas::Psr{ satSigId }));

        // #############################################################################################################################
        //                                                    Velocity calculation
        // #############################################################################################################################

        if (calc.pseudorangeRateMeas && useDoppler)
        {
            // Pseudorange-rate measurement [m/s] - Groves ch. 8.5.3, eq. 8.48, p. 342
            retVal.psrRateMeas(Meas::Doppler{ satSigId }) = calc.pseudorangeRateMeas.value(); // + (multipath and/or NLOS errors) + (tracking errors)
            LOG_DATA("         psrRateMeas({}) = {}", Meas::Doppler{ satSigId }, retVal.psrRateMeas(Meas::Doppler{ satSigId }));

            auto [psrRateEst_i, W_psrRate_i] = calcPsrRateAndWeight(calc, state, estimatorType, gnssMeasurementErrorModel);
            retVal.psrRateEst(Meas::Doppler{ satSigId }) = psrRateEst_i;
            LOG_DATA("         psrRateEst({}) = {}", Meas::Doppler{ satSigId }, retVal.psrRateEst(Meas::Doppler{ satSigId }));
            solSatData.psrRateEst = psrRateEst_i;
            retVal.W_psrRate(Meas::Doppler{ satSigId }, Meas::Doppler{ satSigId }) = W_psrRate_i;
            LOG_DATA("         W_psrRate({},{}) = {}", Meas::Doppler{ satSigId }, Meas::Doppler{ satSigId }, retVal.W_psrRate(Meas::Doppler{ satSigId }, Meas::Doppler{ satSigId }));

            // Measurement/Geometry matrix - Groves ch. 9.4.1, eq. 9.144, p. 412
            retVal.e_H_r(Meas::Doppler{ satSigId }, States::Vel) = retVal.e_H_psr(Meas::Psr{ satSigId }, States::Pos);
            retVal.e_H_r(Meas::Doppler{ satSigId }, States::RecvClkDrift) = retVal.e_H_psr(Meas::Psr{ satSigId }, States::RecvClkErr);
            if (sppSol->recvClk.referenceTimeSatelliteSystem != satSigId.toSatId().satSys)
            {
                for (const auto& interSysDrift : interSysDrifts)
                {
                    if (std::get_if<States::InterSysDrift>(&interSysDrift)->satSys == satSigId.toSatId().satSys)
                    {
                        retVal.e_H_r(Meas::Doppler{ satSigId }, interSysDrift) = 1;
                    }
                }
            }
            LOG_DATA("         e_H_r.row({}) = {}", Meas::Doppler{ satSigId }, retVal.e_H_r(Meas::Doppler{ satSigId }, all));

            retVal.dpsr_dot(Meas::Doppler{ satSigId }) = retVal.psrRateMeas(Meas::Doppler{ satSigId }) - psrRateEst_i;
            LOG_DATA("         dpsr_dot({}) {}", Meas::Doppler{ satSigId }, retVal.dpsr_dot(Meas::Doppler{ satSigId }));
        }
    }

    return retVal;
}

bool calcDataBasedOnEstimates(const std::shared_ptr<SppSolution>& sppSol,
                              std::vector<SatelliteSystem>& satelliteSystems,
                              std::vector<CalcData>& calcData,
                              State& state,
                              size_t nParam,
                              size_t nMeasPsr,
                              size_t nMeasDoppler,
                              const InsTime& insTime,
                              const Eigen::Vector3d& lla_pos,
                              double elevationMask,
                              EstimatorType estimatorType)
{
    size_t cntSkippedMeas = 0;
    size_t cntSkippedMeasDoppler = 0;
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
            if (calc.pseudorangeRateMeas) { cntSkippedMeasDoppler++; }
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

            if (nMeasPsr - cntSkippedMeas < nParam && (estimatorType == EstimatorType::LEAST_SQUARES || estimatorType == EstimatorType::WEIGHTED_LEAST_SQUARES))
            {
                LOG_ERROR(" SPP cannot calculate position because only {} valid measurements ({} needed). Try changing filter settings or reposition your antenna.",
                          insTime, nMeasPsr - cntSkippedMeas, nParam);
                return false;
            }
            continue;
        }
        calc.skipped = false;
        solSatData.elevationMaskTriggered = false;

        usedSatelliteSystems |= satId.satSys;
    }

    // Update the amount of measurements and parameter to take skipped measurements because of elevation mask and not available satellite systems into account
    sppSol->nMeasPsr = nMeasPsr - cntSkippedMeas;
    sppSol->nMeasDopp = nMeasDoppler - cntSkippedMeasDoppler;
    sppSol->nParam = nParam;
    LOG_DATA(" Skipping {} signals. Left psr {}, doppler {}", cntSkippedMeas, sppSol->nMeasPsr, sppSol->nMeasDopp);

    // Choose reference time satellite system
    state.recvClk.referenceTimeSatelliteSystem = satelliteSystems.front();
    satelliteSystems.erase(std::find(satelliteSystems.begin(), satelliteSystems.end(), state.recvClk.referenceTimeSatelliteSystem));
    LOG_DATA("     referenceTimeSatelliteSystem {} ({} other time systems)", state.recvClk.referenceTimeSatelliteSystem, satelliteSystems.size());
    sppSol->recvClk.referenceTimeSatelliteSystem = state.recvClk.referenceTimeSatelliteSystem;

    sppSol->usedSatSysExceptRef = satelliteSystems;

    return true;
}

void getInterSysKeys(const std::vector<SatelliteSystem>& satelliteSystems,
                     std::vector<States::StateKeyTypes>& interSysErrs,
                     std::vector<States::StateKeyTypes>& interSysDrifts)
{
    if (satelliteSystems.size() == interSysErrs.size() && satelliteSystems.size() == interSysDrifts.size())
    {
        // Check that the keys and available satellite systems are the same
        for (const auto& interSysErr : interSysErrs)
        {
            if (std::find(satelliteSystems.begin(), satelliteSystems.end(), std::get_if<States::InterSysErr>(&interSysErr)->satSys) == satelliteSystems.end())
            {
                interSysErrs.clear();
                interSysDrifts.clear();
                for (const auto& satSys : satelliteSystems)
                {
                    interSysErrs.emplace_back(States::InterSysErr{ satSys });
                    interSysDrifts.emplace_back(States::InterSysDrift{ satSys });
                }
            }
        }
    }
    else
    {
        interSysErrs.clear();
        interSysDrifts.clear();
        for (const auto& satSys : satelliteSystems)
        {
            interSysErrs.emplace_back(States::InterSysErr{ satSys });
            interSysDrifts.emplace_back(States::InterSysDrift{ satSys });
        }
    }
}

} // namespace NAV::GNSS::Positioning::SPP