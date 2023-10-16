#include "SppLeastSquares.hpp"

#include "util/Logger.hpp"
#include "Navigation/Math/KeyedLeastSquares.hpp"

namespace NAV::GNSS::Positioning::SPP
{

bool solveLeastSquaresAndAssignSolution(const KeyedVectorXd<Meas::MeasKeyTypes>& dy,
                                        const KeyedMatrixXd<Meas::MeasKeyTypes, States::StateKeyTypes>& e_H,
                                        const KeyedMatrixXd<Meas::MeasKeyTypes, Meas::MeasKeyTypes>& W,
                                        const EstimatorType& estimatorType,
                                        size_t nMeas,
                                        const std::vector<States::StateKeyTypes>& interSysErrOrDrifts,
                                        Eigen::Vector3d& posOrVel, const std::vector<States::StateKeyTypes>& statesPosOrVel,
                                        UncertainValue<double>& biasOrDrift, States::StateKeyTypes statesBiasOrDrift,
                                        std::unordered_map<NAV::SatelliteSystem, NAV::UncertainValue<double>>& sysTimeOrDriftDiff,
                                        const std::shared_ptr<SppSolution>& sppSol,
                                        UncertainValue<double>& sppSolBiasOrDrift,
                                        std::unordered_map<NAV::SatelliteSystem, NAV::UncertainValue<double>>& sppSolSysTimeOrDriftDiff,
                                        SppSolSetPosOrVelAndStdDev_e sppSolSetPosOrVelAndStdDev_e,
                                        SppSolSetPosOrVel_e sppSolSetPosOrVel_e,
                                        SppSolSetErrorCovarianceMatrix sppSolSetErrorCovarianceMatrix)
{
    LOG_DATA("     e_H\n{}", e_H);
    LOG_DATA("     dy {}", dy.transposed());
    if (estimatorType == EstimatorType::WEIGHTED_LEAST_SQUARES) { LOG_DATA("     W\n{}", W); }

    KeyedLeastSquaresResult<KeyedVectorXd<States::StateKeyTypes>, KeyedMatrixXd<States::StateKeyTypes, States::StateKeyTypes>> lsq;

    // [x, y, z, clkBias, sysTimeDiff...] - Groves ch. 9.4.1, eq. 9.141, p. 412
    // [vx, vy, vz, clkDrift, sysDriftDiff...] - Groves ch. 9.4.1, eq. 9.141, p. 412
    lsq = estimatorType == EstimatorType::WEIGHTED_LEAST_SQUARES ? solveWeightedLinearLeastSquaresUncertainties(e_H, W, dy)
                                                                 : solveLinearLeastSquaresUncertainties(e_H, dy);

    LOG_DATA("     sol ({}lsq) {}", estimatorType == EstimatorType::WEIGHTED_LEAST_SQUARES ? "w" : "", lsq.solution.transposed());
    LOG_DATA("     var ({}lsq)\n{}", estimatorType == EstimatorType::WEIGHTED_LEAST_SQUARES ? "w" : "", lsq.variance);

    posOrVel += lsq.solution(statesPosOrVel);
    biasOrDrift.value += lsq.solution(statesBiasOrDrift) / InsConst::C;

    for (const auto& errOrDrift : interSysErrOrDrifts)
    {
        if (const auto* const key = std::get_if<States::InterSysErr>(&errOrDrift)) // key with right data type to access satSys
        {
            sysTimeOrDriftDiff[key->satSys].value += lsq.solution(errOrDrift) / InsConst::C;
            sysTimeOrDriftDiff[key->satSys].stdDev = std::sqrt(lsq.variance(errOrDrift, errOrDrift)) / InsConst::C;
        }
        else if (const auto* const key = std::get_if<States::InterSysDrift>(&errOrDrift))
        {
            sysTimeOrDriftDiff[key->satSys].value += lsq.solution(errOrDrift) / InsConst::C;
            sysTimeOrDriftDiff[key->satSys].stdDev = std::sqrt(lsq.variance(errOrDrift, errOrDrift)) / InsConst::C;
        }
    }

    if (nMeas > sppSol->nParam) // Standard deviation can only be calculated with more measurements than estimated parameters
    {
        std::invoke(sppSolSetPosOrVelAndStdDev_e, sppSol, posOrVel, lsq.variance(statesPosOrVel, statesPosOrVel));
        biasOrDrift.stdDev = std::sqrt(lsq.variance(statesBiasOrDrift, statesBiasOrDrift)) / InsConst::C;
    }
    else
    {
        std::invoke(sppSolSetPosOrVel_e, sppSol, posOrVel);
        biasOrDrift.stdDev = std::nan("");
    }

    std::invoke(sppSolSetErrorCovarianceMatrix, sppSol, lsq.variance);

    sppSolBiasOrDrift = biasOrDrift;
    sppSolSysTimeOrDriftDiff = sysTimeOrDriftDiff;

    return lsq.solution(all).norm() < 1e-4;

    return true;
}

} // namespace NAV::GNSS::Positioning::SPP