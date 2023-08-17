#include "SppLeastSquares.hpp"

#include "util/Logger.hpp"
#include "Navigation/Math/LeastSquares.hpp"

namespace NAV::GNSS::Positioning::SPP
{

bool solveLeastSquaresAndAssignSolution(const Eigen::VectorXd& dy, const Eigen::MatrixXd& e_H, const Eigen::MatrixXd& W,
                                        const EstimatorType& estimatorType,
                                        const std::vector<SatelliteSystem>& satelliteSystems,
                                        size_t nMeas,
                                        size_t nParam,
                                        Eigen::Vector3d& posOrVel,
                                        UncertainValue<double>& biasOrDrift,
                                        std::unordered_map<NAV::SatelliteSystem, NAV::UncertainValue<double>>& sysTimeOrDriftDiff,
                                        const std::shared_ptr<SppSolution>& sppSol,
                                        size_t& sppSolSatelliteNum,
                                        UncertainValue<double>& sppSolBiasOrDrift,
                                        std::unordered_map<NAV::SatelliteSystem, NAV::UncertainValue<double>>& sppSolSysTimeOrDriftDiff,
                                        SppSolSetPosOrVelAndStdDev_e sppSolSetPosOrVelAndStdDev_e,
                                        SppSolSetPosOrVel_e sppSolSetPosOrVel_e,
                                        SppSolSetErrorCovarianceMatrix sppSolSetErrorCovarianceMatrix)
{
    LOG_DATA("     e_H\n{}", e_H);
    LOG_DATA("     dy {}", dy.transpose());
    if (estimatorType == EstimatorType::WEIGHTED_LEAST_SQUARES) { LOG_DATA("     W\n{}", W); }

    LeastSquaresResult<Eigen::VectorXd, Eigen::MatrixXd> lsq;

    // [x, y, z, clkBias, sysTimeDiff...] - Groves ch. 9.4.1, eq. 9.141, p. 412
    // [vx, vy, vz, clkDrift, sysDriftDiff...] - Groves ch. 9.4.1, eq. 9.141, p. 412
    lsq = estimatorType == EstimatorType::WEIGHTED_LEAST_SQUARES ? solveWeightedLinearLeastSquaresUncertainties(e_H, W, dy)
                                                                 : solveLinearLeastSquaresUncertainties(e_H, dy);

    LOG_DATA("     sol ({}lsq) {}", estimatorType == EstimatorType::WEIGHTED_LEAST_SQUARES ? "w" : "", lsq.solution.transpose());
    LOG_DATA("     var ({}lsq)\n{}", estimatorType == EstimatorType::WEIGHTED_LEAST_SQUARES ? "w" : "", lsq.variance);

    posOrVel += lsq.solution.head<3>();
    biasOrDrift.value += lsq.solution(3) / InsConst::C;
    for (size_t s = 0; s < satelliteSystems.size(); s++)
    {
        int idx = 4 + static_cast<int>(s);
        sysTimeOrDriftDiff[satelliteSystems.at(s)].value += lsq.solution(idx) / InsConst::C;
        sysTimeOrDriftDiff[satelliteSystems.at(s)].stdDev = std::sqrt(lsq.variance(idx, idx)) / InsConst::C;
    }

    if (nMeas > nParam) // Standard deviation can only be calculated with more measurements than estimated parameters
    {
        std::invoke(sppSolSetPosOrVelAndStdDev_e, sppSol, posOrVel, lsq.variance.topLeftCorner<3, 3>());
        biasOrDrift.stdDev = std::sqrt(lsq.variance(3, 3)) / InsConst::C;
    }
    else
    {
        std::invoke(sppSolSetPosOrVel_e, sppSol, posOrVel);
        biasOrDrift.stdDev = std::nan("");
    }

    std::invoke(sppSolSetErrorCovarianceMatrix, sppSol, lsq.variance);

    sppSolSatelliteNum = nMeas;
    sppSolBiasOrDrift = biasOrDrift;
    sppSolSysTimeOrDriftDiff = sysTimeOrDriftDiff;

    return lsq.solution.norm() < 1e-4;
}

} // namespace NAV::GNSS::Positioning::SPP