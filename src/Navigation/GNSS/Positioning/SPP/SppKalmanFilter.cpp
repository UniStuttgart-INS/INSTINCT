#include "SppKalmanFilter.hpp"

#include "Navigation/Math/VanLoan.hpp"

#include "Navigation/Atmosphere/Ionosphere/Ionosphere.hpp"
#include "Navigation/Atmosphere/Troposphere/Troposphere.hpp"

#include <algorithm>

namespace States = NAV::GNSS::Positioning::SPP::KF::States;
namespace Meas = NAV::GNSS::Positioning::SPP::KF::Meas;

namespace NAV::GNSS::Positioning::SPP
{

void SppKalmanFilter::initialize()
{
    _lastUpdate.reset();
    _kalmanFilter = KeyedKalmanFilterD<States::StateKeyTypes, Meas::MeasKeyTypes>{ States::PosVelRecvClk, {} };
    _kalmanFilterInitialized = false;
    _kalmanFilterReferenceTimeSatelliteSystem = SatSys_None;

    KF::States::InterSysErrs.clear();
    KF::States::InterSysDrifts.clear();

    // Initial Covariance of the acceleration 𝜎_a due to user motion in horizontal and vertical component [m²/s³]
    if (gui_covarianceAccelUnit == CovarianceAccelUnits::m_sqrts3)
    {
        gui_covarianceAccel = gui_covarianceAccel.array().pow(2);
        gui_covarianceAccelUnit = CovarianceAccelUnits::m2_s3;
    }

    // Initial Covariance of the clock phase drift [m²/s³]
    if (gui_covarianceClkPhaseDriftUnit == CovarianceClkPhaseDriftUnits::m_sqrts3)
    {
        gui_covarianceClkPhaseDrift = std::pow(gui_covarianceClkPhaseDrift, 2);
        gui_covarianceClkPhaseDriftUnit = CovarianceClkPhaseDriftUnits::m2_s3;
    }

    // Initial Covariance of the frequency phase drift [m²/s]
    if (gui_covarianceClkFrequencyDriftUnit == CovarianceClkFrequencyDriftUnits::m_sqrts)
    {
        gui_covarianceClkFrequencyDrift = std::pow(gui_covarianceClkFrequencyDrift, 2);
        gui_covarianceClkFrequencyDriftUnit = CovarianceClkFrequencyDriftUnits::m2_s;
    }

    // Initial Covariance of the inter-system clock phase drift [m²/s³]
    if (gui_covarianceInterSysClkPhaseDriftUnit == CovarianceClkPhaseDriftUnits::m_sqrts3)
    {
        gui_covarianceInterSysClkPhaseDrift = std::pow(gui_covarianceInterSysClkPhaseDrift, 2);
        gui_covarianceInterSysClkPhaseDriftUnit = CovarianceClkPhaseDriftUnits::m2_s3;
    }

    // Initial Covariance of the inter-system frequency phase drift [m²/s]
    if (gui_covarianceInterSysClkFrequencyDriftUnit == CovarianceClkFrequencyDriftUnits::m_sqrts)
    {
        gui_covarianceInterSysClkFrequencyDrift = std::pow(gui_covarianceInterSysClkFrequencyDrift, 2);
        gui_covarianceInterSysClkFrequencyDriftUnit = CovarianceClkFrequencyDriftUnits::m2_s;
    }

    // ###########################################################################################################
    // Convert initial covariance (are used or partly used if Least squares has not more measurements than parameters to calculate uncertainty)

    // Initial Covariance of the position in [m²]
    if (gui_initCovariancePositionUnit == InitCovariancePositionUnits::m)
    {
        gui_initCovariancePosition = gui_initCovariancePosition.array().pow(2);
        gui_initCovariancePositionUnit = InitCovariancePositionUnits::m2;
    }

    // Initial Covariance of the velocity in [m²/s²]
    if (gui_initCovarianceVelocityUnit == InitCovarianceVelocityUnits::m_s)
    {
        gui_initCovarianceVelocity = gui_initCovarianceVelocity.array().pow(2);
        gui_initCovarianceVelocityUnit = InitCovarianceVelocityUnits::m2_s2;
    }

    // Initial Covariance of the receiver clock error [m²]
    if (gui_initCovarianceRecvClkErrUnit == InitCovarianceRecvClkErrUnits::s2 && gui_initCovarianceRecvClkErr < 1)
    {
        gui_initCovarianceRecvClkErr = gui_initCovarianceRecvClkErr * std::pow(InsConst::C, 2);
    }
    else if (gui_initCovarianceRecvClkErrUnit == InitCovarianceRecvClkErrUnits::s && gui_initCovarianceRecvClkErr < 1)
    {
        gui_initCovarianceRecvClkErr = std::pow(gui_initCovarianceRecvClkErr * InsConst::C, 2);
        gui_initCovarianceRecvClkErrUnit = InitCovarianceRecvClkErrUnits::s2;
    }

    // Initial Covariance of the receiver clock drift [m²/s²]
    if (gui_initCovarianceRecvClkDriftUnit == InitCovarianceRecvClkDriftUnits::s2_s2 && gui_initCovarianceRecvClkDrift < 1)
    {
        gui_initCovarianceRecvClkDrift = gui_initCovarianceRecvClkDrift * std::pow(InsConst::C, 2);
    }
    else if (gui_initCovarianceRecvClkDriftUnit == InitCovarianceRecvClkDriftUnits::s_s && gui_initCovarianceRecvClkDrift < 1)
    {
        gui_initCovarianceRecvClkDrift = std::pow(gui_initCovarianceRecvClkDrift * InsConst::C, 2);
        gui_initCovarianceRecvClkDriftUnit = InitCovarianceRecvClkDriftUnits::s2_s2;
    }

    // Initial Covariance of the inter-system clock error [m²]
    if (gui_initCovarianceInterSysErrUnit == InitCovarianceRecvClkErrUnits::s2 && gui_initCovarianceInterSysErr < 1)
    {
        gui_initCovarianceInterSysErr = gui_initCovarianceInterSysErr * std::pow(InsConst::C, 2);
    }
    else if (gui_initCovarianceInterSysErrUnit == InitCovarianceRecvClkErrUnits::s && gui_initCovarianceInterSysErr < 1)
    {
        gui_initCovarianceInterSysErr = std::pow(gui_initCovarianceInterSysErr * InsConst::C, 2);
        gui_initCovarianceInterSysErrUnit = InitCovarianceRecvClkErrUnits::s2;
    }

    // Initial Covariance of the inter-system clock drift [m²/s²]
    if (gui_initCovarianceInterSysDriftUnit == InitCovarianceRecvClkDriftUnits::s2_s2 && gui_initCovarianceInterSysDrift < 1)
    {
        gui_initCovarianceInterSysDrift = gui_initCovarianceInterSysDrift * std::pow(InsConst::C, 2);
    }
    else if (gui_initCovarianceInterSysDriftUnit == InitCovarianceRecvClkDriftUnits::s_s && gui_initCovarianceInterSysDrift < 1)
    {
        gui_initCovarianceInterSysDrift = std::pow(gui_initCovarianceInterSysDrift * InsConst::C, 2);
        gui_initCovarianceInterSysDriftUnit = InitCovarianceRecvClkDriftUnits::s2_s2;
    }
}

bool SppKalmanFilter::isInitialized() const
{
    return _kalmanFilterInitialized;
}

// ###########################################################################################################
//                                             Getter and Setter
// ###########################################################################################################
void SppKalmanFilter::setAllOtherSelectedSatelliteSystems(const std::vector<SatelliteSystem>& allSatSys)
{
    _allOtherSelectedSatelliteSystems = allSatSys;
}

std::vector<SatelliteSystem> SppKalmanFilter::getAllOtherSelectedSatelliteSystems() const
{
    return _allOtherSelectedSatelliteSystems;
}

SatelliteSystem SppKalmanFilter::getKalmanFilterReferenceTimeSatelliteSystem() const
{
    return _kalmanFilterReferenceTimeSatelliteSystem;
}

// ###########################################################################################################
//                                          Initialize Kalman Filter
// ###########################################################################################################
void SppKalmanFilter::initializeKalmanFilter(const std::shared_ptr<const SppSolution>& sppSolLSE)
{
    auto refSys = sppSolLSE->recvClk.referenceTimeSatelliteSystem;
    _kalmanFilterReferenceTimeSatelliteSystem = refSys;

    // Initialize Kalman Filter state x and Covariance P

    _kalmanFilter.x.segment(States::Pos) = sppSolLSE->e_position();                       // _e_position
    _kalmanFilter.x(States::RecvClkErr) = sppSolLSE->recvClk.bias.value * InsConst::C;    // receiver clock error
    _kalmanFilter.x.segment(States::Vel) = sppSolLSE->e_velocity();                       // _e_velocity
    _kalmanFilter.x(States::RecvClkDrift) = sppSolLSE->recvClk.drift.value * InsConst::C; // receiver clock drift
    // inter-system clock error and drift
    for (const auto& otherSys : sppSolLSE->otherUsedSatelliteSystems)
    {
        _kalmanFilter.x(States::InterSysErr{ otherSys }) = sppSolLSE->recvClk.sysTimeDiff.at(otherSys).value * InsConst::C;
        _kalmanFilter.x(States::InterSysDrift{ otherSys }) = sppSolLSE->recvClk.sysDriftDiff.at(otherSys).value * InsConst::C;
    }

    // Find Satellite systems that are selected but not available in first epoch
    std::vector<SatelliteSystem> satSysDiff;
    std::set_difference(_allOtherSelectedSatelliteSystems.begin(), _allOtherSelectedSatelliteSystems.end(),
                        sppSolLSE->otherUsedSatelliteSystems.begin(), sppSolLSE->otherUsedSatelliteSystems.end(),
                        std::inserter(satSysDiff, satSysDiff.begin()));

    // Standard deviation can only be calculated in LSE with more measurements than estimated parameters
    if (std::none_of(sppSolLSE->e_positionStdev().begin(), sppSolLSE->e_positionStdev().end(), [](double d) { return std::isnan(d); }))
    {
        _kalmanFilter.P(States::Pos, States::Pos) = sppSolLSE->e_CovarianceMatrix().block<3, 3>(0, 0);
        _kalmanFilter.P(States::RecvClkErr, States::RecvClkErr) = sppSolLSE->e_CovarianceMatrix()(6, 6);
        _kalmanFilter.P(States::RecvClkErr, States::Pos) = sppSolLSE->e_CovarianceMatrix().block<1, 3>(6, 0);
        _kalmanFilter.P(States::Pos, States::RecvClkErr) = sppSolLSE->e_CovarianceMatrix().block<3, 1>(0, 6);

        Eigen::Index i = 0;
        Eigen::Index j = 0;
        for (const auto& otherSys_i : sppSolLSE->otherUsedSatelliteSystems)
        {
            _kalmanFilter.P(States::Pos, States::InterSysErr{ otherSys_i }) = sppSolLSE->e_CovarianceMatrix().block<3, 1>(0, 8 + i);
            _kalmanFilter.P(States::InterSysErr{ otherSys_i }, States::Pos) = sppSolLSE->e_CovarianceMatrix().block<1, 3>(8 + i, 0);
            _kalmanFilter.P(States::RecvClkErr, States::InterSysErr{ otherSys_i }) = sppSolLSE->e_CovarianceMatrix()(6, 8 + i);
            _kalmanFilter.P(States::InterSysErr{ otherSys_i }, States::RecvClkErr) = sppSolLSE->e_CovarianceMatrix()(8 + i, 6);
            for (const auto& otherSys_j : sppSolLSE->otherUsedSatelliteSystems)
            {
                _kalmanFilter.P(States::InterSysErr{ otherSys_i }, States::InterSysErr{ otherSys_j }) = sppSolLSE->e_CovarianceMatrix()(8 + i, 8 + j);

                j += 2;
            }

            i += 2;
            j = 0;
        }

        if (!satSysDiff.empty())
        {
            for (const auto& satSysDiff_i : satSysDiff)
            {
                _kalmanFilter.P(States::InterSysErr{ satSysDiff_i }, States::InterSysErr{ satSysDiff_i }) = gui_initCovarianceInterSysErr;
            }
        }
    }
    else
    {
        _kalmanFilter.P(States::Pos, States::Pos) = gui_initCovariancePosition.asDiagonal();
        _kalmanFilter.P(States::RecvClkErr, States::RecvClkErr) = gui_initCovarianceRecvClkErr;
        for (const auto& otherSys_i : _allOtherSelectedSatelliteSystems)
        {
            _kalmanFilter.P(States::InterSysErr{ otherSys_i }, States::InterSysErr{ otherSys_i }) = gui_initCovarianceInterSysErr;
        }
    }

    // Standard deviation can only be calculated in LSE with more measurements than estimated parameters
    if (std::none_of(sppSolLSE->e_positionStdev().begin(), sppSolLSE->e_positionStdev().end(), [](double d) { return std::isnan(d); }))
    {
        _kalmanFilter.P(States::Vel, States::Vel) = sppSolLSE->e_CovarianceMatrix().block<3, 3>(3, 3);
        _kalmanFilter.P(States::RecvClkDrift, States::RecvClkDrift) = sppSolLSE->e_CovarianceMatrix()(7, 7);
        _kalmanFilter.P(States::Vel, States::RecvClkDrift) = sppSolLSE->e_CovarianceMatrix().block<3, 1>(3, 7);
        _kalmanFilter.P(States::RecvClkDrift, States::Vel) = sppSolLSE->e_CovarianceMatrix().block<1, 3>(7, 3);

        Eigen::Index i = 0;
        Eigen::Index j = 0;
        for (const auto& otherSys_i : sppSolLSE->otherUsedSatelliteSystems)
        {
            _kalmanFilter.P(States::Vel, States::InterSysDrift{ otherSys_i }) = sppSolLSE->e_CovarianceMatrix().block<3, 1>(3, 9 + i);
            _kalmanFilter.P(States::InterSysDrift{ otherSys_i }, States::Vel) = sppSolLSE->e_CovarianceMatrix().block<1, 3>(9 + i, 3);
            _kalmanFilter.P(States::RecvClkDrift, States::InterSysDrift{ otherSys_i }) = sppSolLSE->e_CovarianceMatrix()(7, 9 + i);
            _kalmanFilter.P(States::InterSysDrift{ otherSys_i }, States::RecvClkDrift) = sppSolLSE->e_CovarianceMatrix()(9 + i, 7);

            for (const auto& otherSys_j : sppSolLSE->otherUsedSatelliteSystems)
            {
                _kalmanFilter.P(States::InterSysDrift{ otherSys_i }, States::InterSysDrift{ otherSys_j }) = sppSolLSE->e_CovarianceMatrix()(9 + i, 9 + j);

                j += 2;
            }

            i += 2;
            j = 0;
        }
        if (!satSysDiff.empty())
        {
            for (const auto& satSysDiff_i : satSysDiff)
            {
                _kalmanFilter.P(States::InterSysDrift{ satSysDiff_i }, States::InterSysDrift{ satSysDiff_i }) = gui_initCovarianceInterSysDrift;
            }
        }
    }
    else
    {
        _kalmanFilter.P(States::Vel, States::Vel) = gui_initCovarianceVelocity.asDiagonal();
        _kalmanFilter.P(States::RecvClkDrift, States::RecvClkDrift) = gui_initCovarianceRecvClkDrift;
        for (const auto& otherSys_i : _allOtherSelectedSatelliteSystems)
        {
            _kalmanFilter.P(States::InterSysDrift{ otherSys_i }, States::InterSysDrift{ otherSys_i }) = gui_initCovarianceInterSysDrift;
        }
    }

    // System matrix - Groves ch. 9.4.2.2, eq. 9.148, p. 415
    _kalmanFilter.F(States::Pos, States::Vel) = Eigen::Matrix3d::Identity();
    _kalmanFilter.F(States::RecvClkErr, States::RecvClkDrift) = 1;
    // inter-system system clock error and drift
    for (const auto& satSys : _allOtherSelectedSatelliteSystems)
    {
        _kalmanFilter.F(States::InterSysErr{ satSys }, States::InterSysDrift{ satSys }) = 1;
    }
    LOG_DATA("{}:   System matrix F =\n{} ", nameId(), _kalmanFilter.F);

    // Fix part of Noise input matrix G
    _kalmanFilter.G(States::RecvClkErr, States::RecvClkErr) = 1;
    _kalmanFilter.G(States::RecvClkDrift, States::RecvClkDrift) = 1;
    _kalmanFilter.G(States::InterSysErrs, States::InterSysErrs) = Eigen::MatrixXd::Identity(static_cast<Eigen::Index>(States::InterSysErrs.size()),
                                                                                            static_cast<Eigen::Index>(States::InterSysErrs.size()));
    _kalmanFilter.G(States::InterSysDrifts, States::InterSysDrifts) = Eigen::MatrixXd::Identity(static_cast<Eigen::Index>(States::InterSysDrifts.size()),
                                                                                                static_cast<Eigen::Index>(States::InterSysDrifts.size()));

    // Fix part of Noise scale matrix W
    _kalmanFilter.W(States::RecvClkErr, States::RecvClkErr) = gui_covarianceClkPhaseDrift;
    _kalmanFilter.W(States::RecvClkDrift, States::RecvClkDrift) = gui_covarianceClkFrequencyDrift;
    for (const auto& satSys : _allOtherSelectedSatelliteSystems)
    {
        _kalmanFilter.W(States::InterSysErr{ satSys }, States::InterSysErr{ satSys }) = gui_covarianceInterSysClkPhaseDrift;
        _kalmanFilter.W(States::InterSysDrift{ satSys }, States::InterSysDrift{ satSys }) = gui_covarianceInterSysClkFrequencyDrift;
    }

    _kalmanFilterInitialized = true;
    LOG_DATA("_kalmanFilterInitialized {}", _kalmanFilterInitialized);

    _lastUpdate = sppSolLSE->insTime;
    LOG_DATA("_lastTime {}", _lastUpdate);

    assignInternalSolution(_allOtherSelectedSatelliteSystems);
}

// ###########################################################################################################
//                                          Add InterSys States keys
// ###########################################################################################################
void SppKalmanFilter::addInterSysStateKeys([[maybe_unused]] const InsTime& insTime)
{
    // Add Kalman Filter States (inter-system clock errors and drifts)
    for (const auto& satelliteSystem : _allOtherSelectedSatelliteSystems)
    {
        auto keyErr = States::InterSysErr{ satelliteSystem };
        auto keyDrift = States::InterSysDrift{ satelliteSystem };

        if (!_kalmanFilter.x.hasRow(keyErr))
        {
            // LOG_DEBUG("{}: [{}] Adding state: {}", nameId(), insTime, keyErr);
            _kalmanFilter.addState(keyErr);
            // LOG_DEBUG("{}: [{}] Adding state: {}", nameId(), insTime, keyDrift);
            _kalmanFilter.addState(keyDrift);

            States::InterSysErrs.emplace_back(KF::States::InterSysErr{ satelliteSystem });
            States::InterSysDrifts.emplace_back(KF::States::InterSysDrift{ satelliteSystem });
        }
    }
}

// ###########################################################################################################
//                                         Process Noise matrix Q
// ###########################################################################################################
void SppKalmanFilter::processNoiseMatrixGroves(double dt)
{
    Eigen::Vector3d lla_pos = trafo::ecef2lla_WGS84(_e_position);
    Eigen::Quaterniond n_Quat_e = trafo::n_Quat_e(lla_pos(0), lla_pos(1));
    Eigen::Vector3d n_velocity = n_Quat_e * _e_velocity;
    Eigen::VectorXd x_new = _kalmanFilter.Phi(all, all) * _kalmanFilter.x(all);
    Eigen::VectorXd n_velocity_new = n_Quat_e * x_new.block<3, 1>(3, 0);

    // Groves ch. 9.4.2.2, eq. 9.156, p. 418
    double aH_S = std::pow(processNoiseStandardDeviation, 2) / dt * (n_velocity_new(0, 0) - n_velocity(0, 0));
    double aV_S = std::pow(processNoiseStandardDeviation, 2) / dt * (n_velocity_new(2, 0) - n_velocity(2, 0));

    Eigen::DiagonalMatrix<double, 3> a_S_n(aH_S, aH_S, aV_S);                                              // Scaling matrix in n-frame
    Eigen::Matrix3d a_S_e = n_Quat_e.toRotationMatrix().transpose() * a_S_n * n_Quat_e.toRotationMatrix(); // Scaling matrix in e-frame

    // Groves ch. 9.4.2.2, eq. 9.156, p. 418
    double cPhi_S_a = std::pow(processNoiseStandardDeviation, 2) / dt * (x_new(6, 0) - _recvClk.bias.value - _recvClk.drift.value * dt);
    double cf_S_a = std::pow(processNoiseStandardDeviation, 2) / dt * (x_new(7) - _recvClk.drift.value);

    // Groves ch. 9.4.2.2, eq. 9.152, p. 417
    _kalmanFilter.Q(States::Pos, States::Pos) = std::pow(dt, 3) / 3.0 * a_S_e;
    _kalmanFilter.Q(States::Pos, States::Vel) = std::pow(dt, 2) / 2.0 * a_S_e;
    _kalmanFilter.Q(States::Vel, States::Pos) = _kalmanFilter.Q(States::Pos, States::Vel).transpose();
    _kalmanFilter.Q(States::Vel, States::Vel) = dt * a_S_e;
    _kalmanFilter.Q(States::RecvClkErr, States::RecvClkErr) = cPhi_S_a * dt + cf_S_a * std::pow(dt, 3) / 3.0;
    _kalmanFilter.Q(States::RecvClkErr, States::RecvClkDrift) = cf_S_a * std::pow(dt, 2) / 2.0;
    _kalmanFilter.Q(States::RecvClkDrift, States::RecvClkErr) = _kalmanFilter.Q(States::RecvClkErr, States::RecvClkDrift);
    _kalmanFilter.Q(States::RecvClkDrift, States::RecvClkDrift) = cf_S_a * dt;

    // TODO add Q for inter-system clock errors and drifts

    LOG_DATA("{} _kalmanFilter.Q {}\n ", nameId(), _kalmanFilter.Q);
}

// ###########################################################################################################
//                                  Assign Solution of Kalman Filter estimation
// ###########################################################################################################
void SppKalmanFilter::assignSolution(std::shared_ptr<NAV::SppSolution>& sppSol, State& state,
                                     const std::vector<SatelliteSystem>& otherSatelliteSystems)
{
    assignInternalSolution(otherSatelliteSystems);

    state.e_position = _kalmanFilter.x(States::Pos);
    state.e_velocity = _kalmanFilter.x(States::Vel);
    state.recvClk = _recvClk;
    sppSol->setPositionAndStdDev_e(_e_position, _kalmanFilter.P(States::Pos, States::Pos));
    sppSol->setVelocityAndStdDev_e(_e_velocity, _kalmanFilter.P(States::Vel, States::Vel));
    sppSol->recvClk = _recvClk;
    sppSol->setCovarianceMatrix(_kalmanFilter.P(all, all));
}

void SppKalmanFilter::assignInternalSolution(const std::vector<SatelliteSystem>& otherSatelliteSystems)
{
    _e_position = _kalmanFilter.x(States::Pos);
    _e_velocity = _kalmanFilter.x(States::Vel);
    _recvClk.bias.value = _kalmanFilter.x(States::RecvClkErr) / InsConst::C;
    _recvClk.bias.stdDev = std::sqrt(_kalmanFilter.P(States::RecvClkErr, States::RecvClkErr)) / InsConst::C;
    _recvClk.drift.value = _kalmanFilter.x(States::RecvClkDrift) / InsConst::C;
    _recvClk.drift.stdDev = std::sqrt(_kalmanFilter.P(States::RecvClkDrift, States::RecvClkDrift)) / InsConst::C;
    _recvClk.referenceTimeSatelliteSystem = _kalmanFilterReferenceTimeSatelliteSystem;

    for (const auto& satelliteSystem : otherSatelliteSystems)
    {
        auto keyErr = States::InterSysErr{ satelliteSystem };
        auto keyDrift = States::InterSysDrift{ satelliteSystem };

        _recvClk.sysTimeDiff[satelliteSystem].value = _kalmanFilter.x(keyErr) / InsConst::C;
        _recvClk.sysTimeDiff[satelliteSystem].stdDev = std::sqrt(_kalmanFilter.P(keyErr, keyErr)) / InsConst::C;
        _recvClk.sysDriftDiff[satelliteSystem].value = _kalmanFilter.x(keyDrift) / InsConst::C;
        _recvClk.sysDriftDiff[satelliteSystem].stdDev = std::sqrt(_kalmanFilter.P(keyDrift, keyDrift)) / InsConst::C;
    }
}

// ###########################################################################################################
//                       Single Point Positioning by Kalman Filter estimation
// ###########################################################################################################
void SppKalmanFilter::estimate(const InsTime& insTime, State& state,
                               std::vector<CalcData>& calcData,
                               std::shared_ptr<NAV::SppSolution>& sppSol,
                               const IonosphericCorrections& ionosphericCorrections,
                               const IonosphereModel& ionosphereModel,
                               const TroposphereModelSelection& troposphereModels,
                               const GnssMeasurementErrorModel& gnssMeasurementErrorModel,
                               double elevationMask, const std::array<bool, 2>& usedObservations)
{
    kalmanFilterPrediction(sppSol->insTime);
    assignSolution(sppSol, state, _allOtherSelectedSatelliteSystems);

    if (!calcData.empty())
    {
        // Sorted list of satellite systems of this epoch
        std::set<SatelliteSystem> sortedAvailableSatelliteSystems;
        for (const auto& calc : calcData) { sortedAvailableSatelliteSystems.insert(calc.obsData.satSigId.toSatId().satSys); }

        // Get a vector of all available systems (sorted) of this epoch
        std::vector<SatelliteSystem> availableSatelliteSystems;
        availableSatelliteSystems.reserve(sortedAvailableSatelliteSystems.size());
        std::copy(sortedAvailableSatelliteSystems.begin(), sortedAvailableSatelliteSystems.end(), std::back_inserter(availableSatelliteSystems));

        // Amount of pseudorange measurements
        size_t nMeasPsr = calcData.size();
        LOG_DATA("nMeasPsr {}", nMeasPsr);

        // Find all observations providing a doppler measurement (for velocity calculation)
        size_t nDopplerMeas = findDopplerMeasurements(calcData);
        LOG_DATA("nDopplerMeas {}", nDopplerMeas);

        // Amount of estimated parameters
        size_t nParam = 4 + sortedAvailableSatelliteSystems.size() - 1; // 3x pos, 1x clk, (N-1)x clkDiff (= nDoppler Param -> 3x vel, 1x clkDrift, (N-1)x clkDiffDrift)
        LOG_DATA("nParam {}", nParam);

        // Latitude, Longitude, Altitude of the receiver [rad, rad, m]
        Eigen::Vector3d lla_pos = trafo::ecef2lla_WGS84(_kalmanFilter.x(States::Pos));

        calcDataBasedOnEstimates(sppSol, availableSatelliteSystems, calcData, state,
                                 nParam, nMeasPsr, nDopplerMeas, insTime, lla_pos,
                                 elevationMask, EstimatorType::KF);

        if (sppSol->nSatellitesPosition + sppSol->nSatellitesVelocity == 0)
        {
            return;
        }

        auto [e_H_psr,          // Measurement/Geometry matrix for the pseudorange
              psrEst,           // Pseudorange estimates [m]
              psrMeas,          // Pseudorange measurements [m]
              W_psr,            // Pseudorange measurement error weight matrix
              dpsr,             // Difference between Pseudorange measurements and estimates
              e_H_r,            // Measurement/Geometry matrix for the pseudorange-rate
              psrRateEst,       // Corrected pseudorange-rate estimates [m/s]
              psrRateMeas,      // Corrected pseudorange-rate measurements [m/s]
              W_psrRate,        // Pseudorange rate (doppler) measurement error weight matrix
              dpsr_dot,         // Difference between Pseudorange rate measurements and estimates
              keyedObservations // Observations for use with KeyedKalmanFilter (contains Innovations, Design/Geometry matrix, Weight matrix)
        ] = calcMeasurementEstimatesAndDesignMatrix(sppSol, calcData,
                                                    insTime,
                                                    state, lla_pos,
                                                    ionosphericCorrections, ionosphereModel,
                                                    troposphereModels, gnssMeasurementErrorModel,
                                                    EstimatorType::KF);

        kalmanFilterUpdate(keyedObservations, sppSol->recvClk.referenceTimeSatelliteSystem, sppSol->otherUsedSatelliteSystems, usedObservations);
        assignSolution(sppSol, state, availableSatelliteSystems);

        _lastUpdate = sppSol->insTime;
        LOG_DATA("{} _lastTime {}", nameId(), _lastUpdate);
    }
}

// ###########################################################################################################
//                                          Kalman Filter Prediction
// ###########################################################################################################
void SppKalmanFilter::kalmanFilterPrediction(const InsTime& insTime)
{
    // Time intervall between last Kalman Filter estimation and now
    double dt = static_cast<double>((insTime - _lastUpdate).count());
    LOG_DATA("dt {}s", dt);

    // Update the State transition matrix (𝚽) and the Process noise covariance matrix (𝐐)

    if (qCalculationAlgorithm == QCalculationAlgorithm::Taylor1)
    {
        _kalmanFilter.calcTransitionMatrix_Phi_Taylor(dt, 1);

        processNoiseMatrixGroves(dt);
    }
    else if (qCalculationAlgorithm == QCalculationAlgorithm::VanLoan)
    {
        Eigen::Vector3d lla_pos = trafo::ecef2lla_WGS84(_e_position);
        Eigen::Quaterniond n_Quat_e = trafo::n_Quat_e(lla_pos(0), lla_pos(1));

        // Position dependent part of Noise input matrix G
        _kalmanFilter.G(States::Vel, States::Vel) = n_Quat_e.toRotationMatrix().transpose();

        // Position dependent part of Noise scale matrix W
        Eigen::DiagonalMatrix<double, 3> a_S_n(gui_covarianceAccel(0), gui_covarianceAccel(0), gui_covarianceAccel(1));            // Scaling matrix in n-frame
        _kalmanFilter.W(States::Vel, States::Vel) = n_Quat_e.toRotationMatrix().transpose() * a_S_n * n_Quat_e.toRotationMatrix(); // Scaling matrix in e-frame

        _kalmanFilter.calcPhiAndQWithVanLoanMethod(dt);
    }
    LOG_DATA("{}: F =\n{}", nameId(), _kalmanFilter.F);
    LOG_DATA("{}: G =\n{}", nameId(), _kalmanFilter.G);
    LOG_DATA("{}: W =\n{}", nameId(), _kalmanFilter.W);
    LOG_DATA("{}: GWG^T =\n{}", nameId(),
             KeyedMatrixXd<States::StateKeyTypes>(_kalmanFilter.G(all, all)
                                                      * _kalmanFilter.W(all, all)
                                                      * _kalmanFilter.G(all, all).transpose(),
                                                  _kalmanFilter.G.rowKeys()));
    LOG_DATA("State transition matrix Phi {}", _kalmanFilter.Phi);
    LOG_DATA("Process noise matrix Q {}", _kalmanFilter.Q);

    LOG_DATA("{}: x (a posteriori, t-1 = {}) =\n{}", nameId(), _lastUpdate, _kalmanFilter.x);
    LOG_DATA("{}: P (a posteriori, t-1 = {}) =\n{}", nameId(), _lastUpdate, _kalmanFilter.P);
    _kalmanFilter.predict();
    LOG_DATA("{}: x (a priori    , t   = {}) =\n{}", nameId(), insTime, _kalmanFilter.x);
    LOG_DATA("{}: P (a priori    , t   = {}) =\n{}", nameId(), insTime, _kalmanFilter.P);

    LOG_DATA("{}: dx Prediction (ECEF) = {}", nameId(), (_kalmanFilter.x.segment<3>(States::Pos) - _e_position));
    LOG_DATA("{}: dv Prediction (ECEF) = {}", nameId(), (_kalmanFilter.x.segment<3>(States::Vel) - _e_velocity));
}

// ###########################################################################################################
//                                          Kalman Filter Update
// ###########################################################################################################
void SppKalmanFilter::kalmanFilterUpdate(const KeyedObservations& keyedObservations,
                                         SatelliteSystem sppSolReferenceTimeSatelliteSystem,
                                         const std::vector<SatelliteSystem>& otherSatelliteSystems,
                                         const std::array<bool, 2>& usedObservations)
{
    // Update the Measurement sensitivity Matrix (𝐇), the Measurement noise covariance matrix (𝐑) and the Measurement vector (𝐳)

    std::vector<Meas::MeasKeyTypes> measKeys;
    measKeys.reserve(keyedObservations.size()
                     * static_cast<size_t>(std::count(usedObservations.begin(), usedObservations.end(), true)));
    for (const auto& [satSigId, keyedObservation] : keyedObservations)
    {
        for (const auto& [obsType, obs_i] : keyedObservation)
        {
            switch (obsType)
            {
            case GnssObs::Pseudorange:
                measKeys.emplace_back(Meas::Psr{ satSigId });
                break;
            case GnssObs::Doppler:
                measKeys.emplace_back(Meas::Doppler{ satSigId });
                break;
            default:
                LOG_WARN("Single Point Positioning only uses pseudorange and doppler observations. Therefore this observation is neglected.");
                break;
            }
        }
    }
    _kalmanFilter.setMeasurements(measKeys);

    // check whether reference time satellite system has changed
    bool referenceTimeSatelliteSystemChanged = false;
    if (_kalmanFilterReferenceTimeSatelliteSystem != sppSolReferenceTimeSatelliteSystem)
    {
        referenceTimeSatelliteSystemChanged = true;
        LOG_WARN("Reference satellite system for clock estimation is not available in this epoch. Inter-system clock update cannot be performed.");
    }

    // Innovation - Groves ch. 9.4.2.3, eq. 9.159, p. 420
    // Design matrix - Groves ch. 9.4.2.3, eq. 9.163, p. 420
    Eigen::Index i = 0;
    for (const auto& [satSigId, keyedObservation] : keyedObservations)
    {
        for (const auto& [obsType, obs_i] : keyedObservation)
        {
            switch (obsType)
            {
            case GnssObs::Pseudorange:
                _kalmanFilter.z(Meas::Psr{ satSigId }) = obs_i.z_i;

                _kalmanFilter.H(Meas::Psr{ satSigId }, States::Pos) = obs_i.e_H_i.block<3, 1>(0, 0).transpose();
                _kalmanFilter.H(Meas::Psr{ satSigId }, States::RecvClkErr) = obs_i.e_H_i(3, 0);
                if (!referenceTimeSatelliteSystemChanged)
                {
                    i = 0;
                    for (const auto& satSys : otherSatelliteSystems)
                    {
                        _kalmanFilter.H(Meas::Psr{ satSigId }, States::InterSysErr{ satSys }) = obs_i.e_H_i(4 + i, 0);
                        i++;
                    }
                }

                _kalmanFilter.R(Meas::Psr{ satSigId }, Meas::Psr{ satSigId }) = obs_i.W_i;

                break;
            case GnssObs::Doppler:
                _kalmanFilter.z(Meas::Doppler{ satSigId }) = obs_i.z_i;

                _kalmanFilter.H(Meas::Doppler{ satSigId }, States::Vel) = obs_i.e_H_i.block<3, 1>(0, 0).transpose();
                _kalmanFilter.H(Meas::Doppler{ satSigId }, States::RecvClkDrift) = obs_i.e_H_i(3, 0);
                if (!referenceTimeSatelliteSystemChanged)
                {
                    i = 0;
                    for (const auto& satSys : otherSatelliteSystems)
                    {
                        _kalmanFilter.H(Meas::Doppler{ satSigId }, States::InterSysDrift{ satSys }) = obs_i.e_H_i(4 + i, 0);
                        i++;
                    }
                }

                _kalmanFilter.R(Meas::Doppler{ satSigId }, Meas::Doppler{ satSigId }) = obs_i.W_i;

                break;
            default:
                LOG_WARN("Single Point Positioning only uses pseudorange and doppler observations. Therefore this observation is neglected.");
                break;
            }
        }
    }

    LOG_DATA("{}: z =\n{}", nameId(), _kalmanFilter.z.transpose());
    LOG_DATA("{}: H =\n{}", nameId(), _kalmanFilter.H);
    LOG_DATA("{}: R =\n{}", nameId(), _kalmanFilter.R);

    _kalmanFilter.correctWithMeasurementInnovation();
    LOG_DATA("{}: x (a posteriori, t   = {}) =\n{}", nameId(), insTime, _kalmanFilter.x);
    LOG_DATA("{}: P (a posteriori, t   = {}) =\n{}", nameId(), insTime, _kalmanFilter.P);
    LOG_DATA("{}: dx Update (ECEF) = {}", nameId(), (_kalmanFilter.x.segment<3>(States::Pos) - _e_position));
    LOG_DATA("{}: dv Update (ECEF) = {}", nameId(), (_kalmanFilter.x.segment<3>(States::Vel) - _e_velocity));
}

void to_json(json& j, const SppKalmanFilter& data)
{
    j["qCalculationAlgorithm"] = data.qCalculationAlgorithm;

    j["covarianceAccelUnit"] = data.gui_covarianceAccelUnit;
    j["covarianceAccel"] = data.gui_covarianceAccel;

    j["gui_covarianceClkPhaseDriftUnit"] = data.gui_covarianceClkPhaseDriftUnit;
    j["gui_covarianceClkPhaseDrift"] = data.gui_covarianceClkPhaseDrift;
    j["gui_covarianceClkFrequencyDriftUnit"] = data.gui_covarianceClkFrequencyDriftUnit;
    j["gui_covarianceClkFrequencyDrift"] = data.gui_covarianceClkFrequencyDrift;
    j["gui_covarianceInterSysClkPhaseDriftUnit"] = data.gui_covarianceInterSysClkPhaseDriftUnit;
    j["gui_covarianceInterSysClkPhaseDrift"] = data.gui_covarianceInterSysClkPhaseDrift;
    j["gui_covarianceInterSysClkFrequencyDriftUnit"] = data.gui_covarianceInterSysClkFrequencyDriftUnit;
    j["gui_covarianceInterSysClkFrequencyDrift"] = data.gui_covarianceInterSysClkFrequencyDrift;

    j["processNoiseStandardDeviation"] = data.processNoiseStandardDeviation;

    j["gui_initCovariancePositionUnit"] = data.gui_initCovariancePositionUnit;
    j["gui_initCovariancePosition"] = data.gui_initCovariancePosition;
    j["gui_initCovarianceVelocityUnit"] = data.gui_initCovarianceVelocityUnit;
    j["gui_initCovarianceVelocity"] = data.gui_initCovarianceVelocity;
    j["gui_initCovarianceRecvClkErrUnit"] = data.gui_initCovarianceRecvClkErrUnit;
    j["gui_initCovarianceRecvClkErr"] = data.gui_initCovarianceRecvClkErr;
    j["gui_initCovarianceRecvClkDriftUnit"] = data.gui_initCovarianceRecvClkDriftUnit;
    j["gui_initCovarianceRecvClkDrift"] = data.gui_initCovarianceRecvClkDrift;
    j["gui_initCovarianceInterSysErrUnit"] = data.gui_initCovarianceInterSysErrUnit;
    j["gui_initCovarianceInterSysErr"] = data.gui_initCovarianceInterSysErr;
    j["gui_initCovarianceInterSysDriftUnit"] = data.gui_initCovarianceInterSysDriftUnit;
    j["gui_initCovarianceInterSysDrift"] = data.gui_initCovarianceInterSysDrift;
}

void from_json(const json& j, SppKalmanFilter& data)
{
    // ###########################################################################################################

    if (j.contains("qCalculationAlgorithm")) { j.at("qCalculationAlgorithm").get_to(data.qCalculationAlgorithm); }

    if (j.contains("covarianceAccelUnit")) { j.at("covarianceAccelUnit").get_to(data.gui_covarianceAccelUnit); }
    if (j.contains("covarianceAccel")) { j.at("covarianceAccel").get_to(data.gui_covarianceAccel); }

    if (j.contains("gui_covarianceClkPhaseDrift")) { j.at("gui_covarianceClkPhaseDrift").get_to(data.gui_covarianceClkPhaseDrift); }
    if (j.contains("gui_covarianceClkPhaseDriftUnit")) { j.at("gui_covarianceClkPhaseDriftUnit").get_to(data.gui_covarianceClkPhaseDriftUnit); }
    if (j.contains("gui_covarianceClkFrequencyDrift")) { j.at("gui_covarianceClkFrequencyDrift").get_to(data.gui_covarianceClkFrequencyDrift); }
    if (j.contains("gui_covarianceClkFrequencyDriftUnit")) { j.at("gui_covarianceClkFrequencyDriftUnit").get_to(data.gui_covarianceClkFrequencyDriftUnit); }
    if (j.contains("gui_covarianceInterSysClkPhaseDrift")) { j.at("gui_covarianceInterSysClkPhaseDrift").get_to(data.gui_covarianceInterSysClkPhaseDrift); }
    if (j.contains("gui_covarianceInterSysClkPhaseDriftUnit")) { j.at("gui_covarianceInterSysClkPhaseDriftUnit").get_to(data.gui_covarianceInterSysClkPhaseDriftUnit); }
    if (j.contains("gui_covarianceInterSysClkFrequencyDrift")) { j.at("gui_covarianceInterSysClkFrequencyDrift").get_to(data.gui_covarianceInterSysClkFrequencyDrift); }
    if (j.contains("gui_covarianceInterSysClkFrequencyDriftUnit")) { j.at("gui_covarianceInterSysClkFrequencyDriftUnit").get_to(data.gui_covarianceInterSysClkFrequencyDriftUnit); }

    if (j.contains("processNoiseStandardDeviation")) { j.at("processNoiseStandardDeviation").get_to(data.processNoiseStandardDeviation); }

    // ###########################################################################################################

    if (j.contains("gui_initCovariancePositionUnit")) { j.at("gui_initCovariancePositionUnit").get_to(data.gui_initCovariancePositionUnit); }
    if (j.contains("gui_initCovariancePosition")) { j.at("gui_initCovariancePosition").get_to(data.gui_initCovariancePosition); }

    if (j.contains("gui_initCovarianceVelocityUnit")) { j.at("gui_initCovarianceVelocityUnit").get_to(data.gui_initCovarianceVelocityUnit); }
    if (j.contains("gui_initCovarianceVelocity")) { j.at("gui_initCovarianceVelocity").get_to(data.gui_initCovarianceVelocity); }

    if (j.contains("gui_initCovarianceRecvClkErrUnit")) { j.at("gui_initCovarianceRecvClkErrUnit").get_to(data.gui_initCovarianceRecvClkErrUnit); }
    if (j.contains("gui_initCovarianceRecvClkErr")) { j.at("gui_initCovarianceRecvClkErr").get_to(data.gui_initCovarianceRecvClkErr); }

    if (j.contains("gui_initCovarianceRecvClkDriftUnit")) { j.at("gui_initCovarianceRecvClkDriftUnit").get_to(data.gui_initCovarianceRecvClkDriftUnit); }
    if (j.contains("gui_initCovarianceRecvClkDrift")) { j.at("gui_initCovarianceRecvClkDrift").get_to(data.gui_initCovarianceRecvClkDrift); }

    if (j.contains("gui_initCovarianceInterSysErrUnit")) { j.at("gui_initCovarianceInterSysErrUnit").get_to(data.gui_initCovarianceInterSysErrUnit); }
    if (j.contains("gui_initCovarianceInterSysErr")) { j.at("gui_initCovarianceInterSysErr").get_to(data.gui_initCovarianceInterSysErr); }

    if (j.contains("gui_initCovarianceInterSysDriftUnit")) { j.at("gui_initCovarianceInterSysDriftUnit").get_to(data.gui_initCovarianceInterSysDriftUnit); }
    if (j.contains("gui_initCovarianceInterSysDrift")) { j.at("gui_initCovarianceInterSysDrift").get_to(data.gui_initCovarianceInterSysDrift); }
}

} // namespace NAV::GNSS::Positioning::SPP
