#include "SppAlgorithm.hpp"

#include "Navigation/GNSS/Errors.hpp"
#include "Navigation/GNSS/Functions.hpp"
#include "Navigation/GNSS/Satellite/Ephemeris/GLONASSEphemeris.hpp"
#include "Navigation/Math/LeastSquares.hpp"

#include "Navigation/Transformations/Units.hpp"
#include "internal/gui/widgets/EnumCombo.hpp"

namespace NAV
{

const char* to_string(SppEstimator sppEstimator)
{
    switch (sppEstimator)
    {
    case SppEstimator::LEAST_SQUARES:
        return "Least Squares";
    case SppEstimator::WEIGHTED_LEAST_SQUARES:
        return "Weighted Least Squares";
    case SppEstimator::COUNT:
        break;
    }
    return "";
}

bool ComboSppEstimator(const char* label, SppEstimator& sppEstimator)
{
    return gui::widgets::EnumCombo(label, sppEstimator);
}

std::shared_ptr<const SppSolution> calcSppSolution(SppState state,
                                                   const std::shared_ptr<const GnssObs>& gnssObs,
                                                   const std::vector<const GnssNavInfo*>& gnssNavInfos,
                                                   const IonosphereModel& ionosphereModel,
                                                   const TroposphereModelSelection& troposphereModels,
                                                   const SppEstimator& sppEstimator,
                                                   const Frequency& filterFreq,
                                                   const Code& filterCode,
                                                   const std::vector<SatId>& excludedSatellites,
                                                   double elevationMask)
{
    // Collection of all connected Ionospheric Corrections
    IonosphericCorrections ionosphericCorrections;
    for (const auto* gnssNavInfo : gnssNavInfos)
    {
        for (const auto& correction : gnssNavInfo->ionosphericCorrections.data())
        {
            if (!ionosphericCorrections.contains(correction.satSys, correction.alphaBeta))
            {
                ionosphericCorrections.insert(correction.satSys, correction.alphaBeta, correction.data);
            }
        }
    }

    auto sppSol = std::make_shared<SppSolution>();
    sppSol->insTime = gnssObs->insTime;
    // Data calculated for each observation
    struct CalcData
    {
        // Constructor
        explicit CalcData(const NAV::GnssObs::ObservationData& obsData, std::shared_ptr<NAV::SatNavData> satNavData)
            : obsData(obsData), satNavData(std::move(satNavData)) {}

        const NAV::GnssObs::ObservationData& obsData;          // GNSS Observation data
        std::shared_ptr<NAV::SatNavData> satNavData = nullptr; // Satellite Navigation data

        double satClkBias{};                        // Satellite clock bias [s]
        double satClkDrift{};                       // Satellite clock drift [s/s]
        Eigen::Vector3d e_satPos;                   // Satellite position in ECEF frame coordinates [m]
        Eigen::Vector3d e_satVel;                   // Satellite velocity in ECEF frame coordinates [m/s]
        double pseudorangeEst{ std::nan("") };      // Estimated Pseudorange [m]
        double pseudorangeRateMeas{ std::nan("") }; // Measured Pseudorange rate [m/s]

        // Data recalculated each iteration

        bool skipped = false;                    // Whether to skip the measurement
        Eigen::Vector3d e_lineOfSightUnitVector; // Line-of-sight unit vector in ECEF frame coordinates
        Eigen::Vector3d n_lineOfSightUnitVector; // Line-of-sight unit vector in NED frame coordinates
        double satElevation = 0.0;               // Elevation [rad]
        double satAzimuth = 0.0;                 // Azimuth [rad]
    };

    // Data calculated for each satellite (only satellites filtered by GUI filter & NAV data available)
    std::vector<CalcData> calcData;
    std::vector<SatelliteSystem> availSatelliteSystems; // List of satellite systems
    for (const auto& obsData : gnssObs->data)
    {
        auto satId = obsData.satSigId.toSatId();

        if ((obsData.satSigId.freq & filterFreq)                                                                   // frequency is selected in GUI
            && (obsData.code & filterCode)                                                                         // code is selected in GUI
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
                        (*sppSol)(obsData.satSigId, obsData.code).skipped = true;

                        continue;
                    }
                    LOG_DATA("Using observation from {} {}", obsData.satSigId, obsData.code);
                    calcData.emplace_back(obsData, satNavData);
                    calcData.back().pseudorangeEst = obsData.pseudorange.value().value;
                    if (std::find(availSatelliteSystems.begin(), availSatelliteSystems.end(), satId.satSys) == availSatelliteSystems.end())
                    {
                        availSatelliteSystems.push_back(satId.satSys);
                    }
                    break;
                }
            }
        }
    }

    size_t nMeas = calcData.size();
    LOG_DATA("nMeas {}", nMeas);
    size_t nParam = 4 + availSatelliteSystems.size() - 1; // 3x pos, 1x clk, (N-1)x clkDiff

    // Frequency number (GLONASS only)
    int8_t freqNum = -128;

    // Find all observations providing a doppler measurement (for velocity calculation)
    size_t nDopplerMeas = 0;
    for (auto& calc : calcData)
    {
        const auto& obsData = calc.obsData;
        if (obsData.doppler)
        {
            nDopplerMeas++;
            // TODO: Find out what this is used for and find a way to use it, after the GLONASS orbit calculation is working
            if (obsData.satSigId.freq & (R01 | R02))
            {
                if (auto satNavData = std::dynamic_pointer_cast<GLONASSEphemeris>(calc.satNavData))
                {
                    freqNum = satNavData->frequencyNumber;
                }
            }

            calc.pseudorangeRateMeas = doppler2psrRate(obsData.doppler.value(), obsData.satSigId.freq, freqNum);
        }
    }

    // #####################################################################################################################################
    //                                                          Calculation
    // #####################################################################################################################################

    if (nMeas < nParam)
    {
        LOG_ERROR("[{}] Cannot calculate position because only {} valid measurements ({} needed). Try changing filter settings or reposition your antenna.",
                  (gnssObs->insTime + std::chrono::seconds(gnssObs->insTime.leapGps2UTC())), nMeas, nParam);
        sppSol->nSatellitesPosition = nMeas;
        sppSol->nSatellitesVelocity = nDopplerMeas;
        return sppSol;
    }

    // Measurement/Geometry matrix for the pseudorange
    Eigen::MatrixXd e_H_psr = Eigen::MatrixXd::Zero(static_cast<int>(nMeas), static_cast<int>(nParam));
    // Pseudorange estimates [m]
    Eigen::VectorXd psrEst = Eigen::VectorXd::Zero(static_cast<int>(nMeas));
    // Pseudorange measurements [m]
    Eigen::VectorXd psrMeas = Eigen::VectorXd::Zero(static_cast<int>(nMeas));
    // Pseudorange measurement error weight matrix
    Eigen::MatrixXd W_psr = Eigen::MatrixXd::Zero(static_cast<int>(nMeas), static_cast<int>(nMeas));

    // Measurement/Geometry matrix for the pseudorange-rate
    Eigen::MatrixXd e_H_r = Eigen::MatrixXd::Zero(static_cast<int>(nDopplerMeas), static_cast<int>(nParam));
    // Corrected pseudorange-rate estimates [m/s]
    Eigen::VectorXd psrRateEst = Eigen::VectorXd::Zero(static_cast<int>(nDopplerMeas));
    // Corrected pseudorange-rate measurements [m/s]
    Eigen::VectorXd psrRateMeas = Eigen::VectorXd::Zero(static_cast<int>(nDopplerMeas));
    // Pseudorange rate (doppler) measurement error weight matrix
    Eigen::MatrixXd W_psrRate = Eigen::MatrixXd::Zero(static_cast<int>(nDopplerMeas), static_cast<int>(nDopplerMeas));

    for (size_t o = 0; o < 10; o++)
    {
        // Keeps track of skipped meausrements (because of elevation mask, ...)
        size_t cntSkippedMeas = 0;

        LOG_DATA("Iteration {}", o);
        // Latitude, Longitude, Altitude of the receiver [rad, rad, m]
        Eigen::Vector3d lla_pos = trafo::ecef2lla_WGS84(state.e_position);
        LOG_DATA("    [{}] e_position {}, {}, {}", o, state.e_position.x(), state.e_position.y(), state.e_position.z());
        LOG_DATA("    [{}] lla_pos {}°, {}°, {}m", o, rad2deg(lla_pos.x()), rad2deg(lla_pos.y()), lla_pos.z());
        LOG_DATA("    [{}] recvClk.bias {}", o, state.recvClk.bias.value);
        LOG_DATA("    [{}] recvClk.drift {}", o, state.recvClk.drift.value);

        std::vector<SatelliteSystem> satelliteSystems = availSatelliteSystems; // List of satellite systems

        SatelliteSystem_ usedSatelliteSystems = SatSys_None;
        for (size_t i = 0; i < nMeas; i++)
        {
            const auto& obsData = calcData[i].obsData;
            LOG_DATA("    [{}] satellite {}", o, obsData.satSigId);
            auto satId = obsData.satSigId.toSatId();

            // Calculate satellite clock, position and velocity
            LOG_DATA("    [{}]     pseudorangeEst {}", o, calcData[i].pseudorangeEst);

            auto satClk = calcData[i].satNavData->calcClockCorrections(gnssObs->insTime, calcData[i].pseudorangeEst, obsData.satSigId.freq);
            calcData[i].satClkBias = satClk.bias;
            calcData[i].satClkDrift = satClk.drift;
            LOG_DATA("    [{}]     satClkBias {}, satClkDrift {}", o, calcData[i].satClkBias, calcData[i].satClkDrift);

            auto satPosVel = calcData[i].satNavData->calcSatellitePosVel(satClk.transmitTime);
            calcData[i].e_satPos = satPosVel.e_pos;
            calcData[i].e_satVel = satPosVel.e_vel;
            LOG_DATA("    [{}]     e_satPos {}", o, calcData[i].e_satPos.transpose());
            LOG_DATA("    [{}]     e_satVel {}", o, calcData[i].e_satVel.transpose());

            // Line-of-sight unit vector in ECEF frame coordinates - Groves ch. 8.5.3, eq. 8.41, p. 341
            calcData[i].e_lineOfSightUnitVector = e_calcLineOfSightUnitVector(state.e_position, calcData[i].e_satPos);
            LOG_DATA("    [{}]     e_lineOfSightUnitVector {}", o, calcData[i].e_lineOfSightUnitVector.transpose());
            // Line-of-sight unit vector in NED frame coordinates - Groves ch. 8.5.3, eq. 8.41, p. 341
            calcData[i].n_lineOfSightUnitVector = trafo::n_Quat_e(lla_pos(0), lla_pos(1)) * calcData[i].e_lineOfSightUnitVector;
            LOG_DATA("    [{}]     n_lineOfSightUnitVector {}", o, calcData[i].n_lineOfSightUnitVector.transpose());
            // Elevation [rad] - Groves ch. 8.5.4, eq. 8.57, p. 344
            calcData[i].satElevation = calcSatElevation(calcData[i].n_lineOfSightUnitVector);
            LOG_DATA("    [{}]     satElevation {}°", o, rad2deg(calcData[i].satElevation));
            // Azimuth [rad] - Groves ch. 8.5.4, eq. 8.57, p. 344
            calcData[i].satAzimuth = calcSatAzimuth(calcData[i].n_lineOfSightUnitVector);
            LOG_DATA("    [{}]     satAzimuth {}°", o, rad2deg(calcData[i].satAzimuth));

            auto& solSatData = (*sppSol)(obsData.satSigId, obsData.code);
            solSatData.transmitTime = satClk.transmitTime;
            solSatData.satClkBias = satClk.bias;
            solSatData.satClkDrift = satClk.drift;
            solSatData.e_satPos = calcData[i].e_satPos;
            solSatData.e_satVel = calcData[i].e_satVel;
            solSatData.satElevation = calcData[i].satElevation;
            solSatData.satAzimuth = calcData[i].satAzimuth;

            if (!state.e_position.isZero() && calcData[i].satElevation < elevationMask) // Do not check elevation mask when not having a valid position
            {
                cntSkippedMeas++;
                calcData[i].skipped = true;
                LOG_DATA("    [{}]     [{}] Measurement is skipped because of elevation {:.1f}° and mask of {}° ({} valid measurements remaining)",
                         o, obsData.satSigId, rad2deg(calcData[i].satElevation), rad2deg(elevationMask), nMeas - cntSkippedMeas);

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

                if (nMeas - cntSkippedMeas < nParam)
                {
                    LOG_ERROR("[{}] Cannot calculate position because only {} valid measurements ({} needed). Try changing filter settings or reposition your antenna.",
                              gnssObs->insTime, nMeas - cntSkippedMeas, nParam);
                    sppSol->nSatellitesPosition = nMeas - cntSkippedMeas;
                    sppSol->nSatellitesVelocity = nDopplerMeas - cntSkippedMeas;
                    return sppSol;
                }
                continue;
            }
            solSatData.elevationMaskTriggered = false;

            usedSatelliteSystems |= satId.satSys;
        }

        size_t ix = 0;
        size_t iv = 0;
        state.recvClk.referenceTimeSatelliteSystem = satelliteSystems.front();
        for (const auto& availSatSys : satelliteSystems)
        {
            if (SatelliteSystem_(availSatSys) < SatelliteSystem_(state.recvClk.referenceTimeSatelliteSystem))
            {
                state.recvClk.referenceTimeSatelliteSystem = availSatSys;
            }
        }
        satelliteSystems.erase(std::find(satelliteSystems.begin(), satelliteSystems.end(), state.recvClk.referenceTimeSatelliteSystem));
        LOG_DATA("    [{}] referenceTimeSatelliteSystem {} ({} other time systems)", o, state.recvClk.referenceTimeSatelliteSystem, satelliteSystems.size());

        for (auto& calc : calcData)
        {
            if (calc.skipped) { continue; }

            const auto& obsData = calc.obsData;
            LOG_DATA("    [{}] satellite {}", o, obsData.satSigId);
            auto satId = obsData.satSigId.toSatId();

            // #############################################################################################################################
            //                                                    Position calculation
            // #############################################################################################################################

            // Pseudorange measurement [m] - Groves ch. 8.5.3, eq. 8.48, p. 342
            psrMeas(static_cast<int>(ix)) = obsData.pseudorange.value().value /* + (multipath and/or NLOS errors) + (tracking errors) */;
            LOG_DATA("    [{}]     psrMeas({}) {}", o, ix, psrMeas(static_cast<int>(ix)));
            // Estimated modulation ionosphere propagation error [m]
            double dpsr_I = calcIonosphericTimeDelay(static_cast<double>(gnssObs->insTime.toGPSweekTow().tow), obsData.satSigId.freq, lla_pos,
                                                     calc.satElevation, calc.satAzimuth, ionosphereModel, &ionosphericCorrections)
                            * InsConst::C;
            LOG_DATA("    [{}]     dpsr_I {} [m] (Estimated modulation ionosphere propagation error)", o, dpsr_I);

            auto tropo = calcTroposphericDelayAndMapping(gnssObs->insTime, lla_pos, calc.satElevation, calc.satAzimuth, troposphereModels);
            LOG_DATA("    [{}]     ZHD {}", o, tropo.ZHD);
            LOG_DATA("    [{}]     ZWD {}", o, tropo.ZWD);
            LOG_DATA("    [{}]     zhdMappingFactor {}", o, tropo.zhdMappingFactor);
            LOG_DATA("    [{}]     zwdMappingFactor {}", o, tropo.zwdMappingFactor);

            // Estimated modulation troposphere propagation error [m]
            double dpsr_T = tropo.ZHD * tropo.zhdMappingFactor + tropo.ZWD * tropo.zwdMappingFactor;
            LOG_DATA("    [{}]     dpsr_T {} [m] (Estimated modulation troposphere propagation error)", o, dpsr_T);

            // Measurement/Geometry matrix - Groves ch. 9.4.1, eq. 9.144, p. 412
            e_H_psr.block<1, 3>(static_cast<int>(ix), 0) = -calc.e_lineOfSightUnitVector;
            e_H_psr(static_cast<int>(ix), 3) = 1;
            for (size_t s = 0; s < satelliteSystems.size(); s++)
            {
                if (satId.satSys != state.recvClk.referenceTimeSatelliteSystem
                    && satId.satSys == satelliteSystems.at(s))
                {
                    e_H_psr(static_cast<int>(ix), 4 + static_cast<int>(s)) = 1;
                }
            }
            LOG_DATA("    [{}]     e_H_psr.row({}) {}", o, ix, e_H_psr.row(static_cast<int>(ix)));

            // Sagnac correction - Springer Handbook ch. 19.1.1, eq. 19.7, p. 562
            double dpsr_ie = 1.0 / InsConst::C * (state.e_position - calc.e_satPos).dot(InsConst::e_omega_ie.cross(state.e_position));
            LOG_DATA("    [{}]     dpsr_ie {}", o, dpsr_ie);
            // Geometric distance [m]
            double geometricDist = (calc.e_satPos - state.e_position).norm();
            LOG_DATA("    [{}]     geometricDist {}", o, geometricDist);
            // System time difference to GPS [s]
            double sysTimeDiff = satId.satSys != state.recvClk.referenceTimeSatelliteSystem
                                     ? state.recvClk.sysTimeDiff.at(satId.satSys).value
                                     : 0.0;

            // Pseudorange estimate [m]
            psrEst(static_cast<int>(ix)) = geometricDist
                                           + state.recvClk.bias.value * InsConst::C
                                           + sysTimeDiff * InsConst::C
                                           - calc.satClkBias * InsConst::C
                                           + dpsr_I
                                           + dpsr_T
                                           + dpsr_ie;
            LOG_DATA("    [{}]     psrEst({}) {}", o, ix, psrEst(static_cast<int>(ix)));
            calc.pseudorangeEst = psrEst(static_cast<int>(ix));

            if (sppEstimator == SppEstimator::WEIGHTED_LEAST_SQUARES)
            {
                // Weight matrix - RTKLIB eq. E6.23, p. 158

                double varPsrMeas = psrMeasErrorVar(obsData.satSigId.toSatId().satSys, obsData.satSigId.freq, calc.satElevation);
                LOG_DATA("    [{}]     varPsrMeas {}", o, varPsrMeas);
                double varEph = calc.satNavData->calcSatellitePositionVariance();
                LOG_DATA("    [{}]     varEph {}", o, varEph);
                double varIono = ionoErrorVar(dpsr_I, obsData.satSigId.freq, freqNum);
                LOG_DATA("    [{}]     varIono {}", o, varIono);
                double varTrop = tropoErrorVar(dpsr_T, calc.satElevation);
                LOG_DATA("    [{}]     varTrop {}", o, varTrop);
                double varBias = codeBiasErrorVar();
                LOG_DATA("    [{}]     varBias {}", o, varBias);
                double varErrors = varPsrMeas + varEph + varIono + varTrop + varBias;
                LOG_DATA("    [{}]     varErrors {}", o, varErrors);

                W_psr(static_cast<int>(ix), static_cast<int>(ix)) = 1.0 / varErrors;
                LOG_DATA("    [{}]     W_psr({},{}) {}", o, ix, ix, W_psr(static_cast<int>(ix), static_cast<int>(ix)));
            }

            LOG_DATA("    [{}]     dpsr({}) {}", o, ix, psrMeas(static_cast<int>(ix)) - psrEst(static_cast<int>(ix)));

            // #############################################################################################################################
            //                                                    Velocity calculation
            // #############################################################################################################################

            if (nDopplerMeas - cntSkippedMeas >= nParam && !std::isnan(calc.pseudorangeRateMeas))
            {
                // Measurement/Geometry matrix - Groves ch. 9.4.1, eq. 9.144, p. 412
                e_H_r.row(static_cast<int>(iv)) = e_H_psr.row(static_cast<int>(ix));

                // Pseudorange-rate measurement [m/s] - Groves ch. 8.5.3, eq. 8.48, p. 342
                psrRateMeas(static_cast<int>(iv)) = calc.pseudorangeRateMeas /* + (multipath and/or NLOS errors) + (tracking errors) */;
                LOG_DATA("    [{}]     psrRateMeas({}) {}", o, iv, psrRateMeas(static_cast<int>(iv)));

                // Range-rate Sagnac correction - Groves ch. 8.5.3, eq. 8.46, p. 342
                double dpsr_dot_ie = InsConst::omega_ie / InsConst::C
                                     * (calc.e_satVel.y() * state.e_position.x() + calc.e_satPos.y() * state.e_velocity.x()
                                        - calc.e_satVel.x() * state.e_position.y() - calc.e_satPos.x() * state.e_velocity.y());
                LOG_DATA("    [{}]     dpsr_dot_ie {}", o, dpsr_dot_ie);
                // System time drift difference to GPS [s/s]
                double sysDriftDiff = satId.satSys != state.recvClk.referenceTimeSatelliteSystem
                                          ? state.recvClk.sysDriftDiff[satId.satSys].value
                                          : 0.0;

                // Pseudorange-rate estimate [m/s] - Groves ch. 9.4.1, eq. 9.142, p. 412 (Sagnac correction different sign)
                psrRateEst(static_cast<int>(iv)) = calc.e_lineOfSightUnitVector.transpose() * (calc.e_satVel - state.e_velocity)
                                                   + state.recvClk.drift.value * InsConst::C
                                                   + sysDriftDiff * InsConst::C
                                                   - calc.satClkDrift * InsConst::C
                                                   - dpsr_dot_ie;
                LOG_DATA("    [{}]     psrRateEst({}) {}", o, iv, psrRateEst(static_cast<int>(iv)));

                if (sppEstimator == SppEstimator::WEIGHTED_LEAST_SQUARES)
                {
                    // Weight matrix

                    double varDopMeas = dopplerErrorVar();
                    LOG_DATA("    [{}]     varDopMeas {}", o, varDopMeas);
                    double varEph = calc.satNavData->calcSatellitePositionVariance();
                    LOG_DATA("    [{}]     varEph {}", o, varEph);
                    double varErrors = varDopMeas + varEph;
                    LOG_DATA("    [{}]     varErrors {}", o, varErrors);

                    W_psrRate(static_cast<int>(iv), static_cast<int>(iv)) = 1.0 / varErrors;
                    LOG_DATA("    [{}]     W_psrRate({},{}) {}", o, iv, iv, W_psrRate(static_cast<int>(iv), static_cast<int>(iv)));
                }

                iv++;
            }
            auto& solSatData = (*sppSol)(obsData.satSigId, obsData.code);
            solSatData.pseudorangeRate = calc.pseudorangeRateMeas;
            solSatData.dpsr_I = dpsr_I;
            solSatData.dpsr_T = dpsr_T;
            solSatData.geometricDist = geometricDist;

            ix++;
        }

        // #################################################################################################################################
        //                                                     Least squares solution
        // #################################################################################################################################

        // ---------------------------------------------------------- Position -------------------------------------------------------------
        LOG_DATA("    [{}] e_H_psr \n{}", o, e_H_psr.topRows(ix));
        if (sppEstimator == SppEstimator::WEIGHTED_LEAST_SQUARES)
        {
            LOG_DATA("    [{}] W_psr \n{}", o, W_psr.topLeftCorner(ix, ix));
        }
        LOG_DATA("    [{}] psrMeas {}", o, psrMeas.topRows(ix).transpose());
        LOG_DATA("    [{}] psrEst {}", o, psrEst.topRows(ix).transpose());

        // Difference between measured and estimated pseudorange
        Eigen::VectorXd dpsr = psrMeas.topRows(ix) - psrEst.topRows(ix);
        LOG_DATA("    [{}] dpsr {}", o, dpsr.transpose());

        LeastSquaresResult<Eigen::VectorXd, Eigen::MatrixXd> lsq;

        // [x, y, z, clkBias, sysTimeDiff...] - Groves ch. 9.4.1, eq. 9.141, p. 412
        if (sppEstimator == SppEstimator::WEIGHTED_LEAST_SQUARES)
        {
            lsq = solveWeightedLinearLeastSquaresUncertainties(e_H_psr.topRows(ix), W_psr.topLeftCorner(ix, ix), dpsr);
            LOG_DATA("    [{}] dx (wlsq) {}, {}, {}, {}", o, lsq.solution(0), lsq.solution(1), lsq.solution(2), lsq.solution(3));
            LOG_DATA("    [{}] stdev_dx (wlsq)\n{}", o, lsq.variance.cwiseSqrt());
        }
        else
        {
            lsq = solveLinearLeastSquaresUncertainties(e_H_psr.topRows(ix), dpsr);
            LOG_DATA("    [{}] dx (lsq) {}, {}, {}, {}", o, lsq.solution(0), lsq.solution(1), lsq.solution(2), lsq.solution(3));
            LOG_DATA("    [{}] stdev_dx (lsq)\n{}", o, lsq.variance.cwiseSqrt());
        }

        state.e_position += lsq.solution.head<3>();
        state.recvClk.bias.value += lsq.solution(3) / InsConst::C;
        for (size_t s = 0; s < satelliteSystems.size(); s++)
        {
            int idx = 4 + static_cast<int>(s);
            state.recvClk.sysTimeDiff[satelliteSystems.at(s)].value += lsq.solution(idx) / InsConst::C;
            state.recvClk.sysTimeDiff[satelliteSystems.at(s)].stdDev = std::sqrt(lsq.variance(idx, idx)) / InsConst::C;
        }

        sppSol->nSatellitesPosition = ix;
        if (ix > nParam) // Standard deviation can only be calculated with more measurements than estimated parameters
        {
            sppSol->setPositionAndStdDev_e(state.e_position, lsq.variance.topLeftCorner<3, 3>().cwiseSqrt());
            state.recvClk.bias.stdDev = std::sqrt(lsq.variance(3, 3)) / InsConst::C;
        }
        else
        {
            sppSol->setPosition_e(state.e_position);
            state.recvClk.bias.stdDev = std::nan("");
        }
        sppSol->recvClk.bias = state.recvClk.bias;
        sppSol->recvClk.referenceTimeSatelliteSystem = state.recvClk.referenceTimeSatelliteSystem;
        sppSol->recvClk.sysTimeDiff = state.recvClk.sysTimeDiff;

        bool solInaccurate = lsq.solution.norm() > 1e-4;

        // ---------------------------------------------------------- Velocity -------------------------------------------------------------
        if (iv >= nParam)
        {
            LOG_DATA("    [{}] e_H_r \n{}", o, e_H_r.topRows(iv));
            if (sppEstimator == SppEstimator::WEIGHTED_LEAST_SQUARES)
            {
                LOG_DATA("    [{}] W_psrRate \n{}", o, W_psrRate.topLeftCorner(iv, iv));
            }
            LOG_DATA("    [{}] psrRateMeas {}", o, psrRateMeas.topRows(iv).transpose());
            LOG_DATA("    [{}] psrRateEst {}", o, psrRateEst.topRows(iv).transpose());

            // Difference between measured and estimated pseudorange rates
            Eigen::VectorXd dpsr_dot = psrRateMeas.topRows(iv) - psrRateEst.topRows(iv);
            LOG_DATA("    [{}] dpsr_dot {}", o, dpsr_dot.transpose());

            // [vx, vy, vz, clkDrift, sysDriftDiff...] - Groves ch. 9.4.1, eq. 9.141, p. 412
            if (sppEstimator == SppEstimator::WEIGHTED_LEAST_SQUARES)
            {
                lsq = solveWeightedLinearLeastSquaresUncertainties(e_H_r.topRows(iv), W_psrRate.topLeftCorner(iv, iv), dpsr_dot);
                LOG_DATA("    [{}] dv (wlsq) {}", o, lsq.solution.transpose());
                LOG_DATA("    [{}] stdev_dv (wlsq)\n{}", o, lsq.variance.cwiseSqrt());
            }
            else
            {
                lsq = solveLinearLeastSquaresUncertainties(e_H_r.topRows(iv), dpsr_dot);
                LOG_DATA("    [{}] dv (lsq) {}", o, lsq.solution.transpose());
                LOG_DATA("    [{}] stdev_dv (lsq)\n{}", o, lsq.variance.cwiseSqrt());
            }

            state.e_velocity += lsq.solution.head<3>();
            state.recvClk.drift.value += lsq.solution(3) / InsConst::C;
            for (size_t s = 0; s < satelliteSystems.size(); s++)
            {
                int idx = 4 + static_cast<int>(s);
                state.recvClk.sysDriftDiff[satelliteSystems.at(s)].value += lsq.solution(idx) / InsConst::C;
                state.recvClk.sysDriftDiff[satelliteSystems.at(s)].stdDev = std::sqrt(lsq.variance(idx, idx)) / InsConst::C;
            }

            sppSol->nSatellitesVelocity = iv;
            if (iv > nParam) // Standard deviation can only be calculated with more measurements than estimated parameters
            {
                sppSol->setVelocityAndStdDev_e(state.e_velocity, lsq.variance.topLeftCorner<3, 3>().cwiseSqrt());
                state.recvClk.drift.stdDev = std::sqrt(lsq.variance(3, 3)) / InsConst::C;
            }
            else
            {
                sppSol->setVelocity_e(state.e_velocity);
                state.recvClk.drift.stdDev = std::nan("");
            }
            sppSol->recvClk.drift = state.recvClk.drift;
            sppSol->recvClk.sysDriftDiff = state.recvClk.sysDriftDiff;

            solInaccurate |= lsq.solution.norm() > 1e-4;
        }
        else
        {
            LOG_WARN("[{}] Cannot calculate velocity because only {} valid doppler measurements ({} needed). Try changing filter settings or reposition your antenna.",
                     gnssObs->insTime, iv, nParam);
            continue;
        }

        if (!solInaccurate)
        {
            break;
        }
    }

    return sppSol;
}
} // namespace NAV
