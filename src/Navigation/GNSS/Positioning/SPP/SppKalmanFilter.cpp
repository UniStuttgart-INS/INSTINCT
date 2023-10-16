#include "SppKalmanFilter.hpp"

#include "Navigation/Math/VanLoan.hpp"

#include "Navigation/Atmosphere/Ionosphere/Ionosphere.hpp"
#include "Navigation/Atmosphere/Troposphere/Troposphere.hpp"

#include "internal/gui/widgets/InputWithUnit.hpp"
#include "internal/gui/NodeEditorApplication.hpp"
#include "internal/NodeManager.hpp"
#include "internal/FlowManager.hpp"

#include <algorithm>

namespace NAV::GNSS::Positioning::SPP
{

void SppKalmanFilter::gui()
{
    const float itemWidth = 250.0F * gui::NodeEditorApplication::windowFontRatio();
    const float configWidth = 380.0F * gui::NodeEditorApplication::windowFontRatio();
    const float unitWidth = 100.0F * gui::NodeEditorApplication::windowFontRatio();

    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    if (ImGui::TreeNode(fmt::format("System/Process noise##{}", "SppKalmanFilter").c_str()))
    {
        ImGui::SetNextItemWidth(itemWidth);
        if (ImGui::Combo(fmt::format("Q calculation algorithm##{}", "SppKalmanFilter").c_str(), reinterpret_cast<int*>(&qCalculationAlgorithm), "Van Loan\0Taylor 1st Order (Groves 2013)\0\0"))
        {
            LOG_DEBUG("{}: Q calculation algorithm changed to {}", "SppKalmanFilter", fmt::underlying(qCalculationAlgorithm));
            flow::ApplyChanges();
        }

        if (qCalculationAlgorithm == SppKalmanFilter::QCalculationAlgorithm::VanLoan)
        {
            if (gui::widgets::InputDouble2WithUnit(fmt::format("Acceleration due to user motion (Hor/Ver)##{}", "SppKalmanFilter").c_str(),
                                                   configWidth, unitWidth, gui_covarianceAccel.data(), reinterpret_cast<int*>(&gui_covarianceAccelUnit), "m/√(s^3)\0m^2/s^3\0\0",
                                                   "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: gui_covarianceAccel changed to {}", "SppKalmanFilter", gui_covarianceAccel);
                LOG_DEBUG("{}: gui_covarianceAccelUnit changed to {}", "SppKalmanFilter", fmt::underlying(gui_covarianceAccelUnit));
                flow::ApplyChanges();
            }
            if (gui::widgets::InputDoubleWithUnit(fmt::format("Standard deviation of the receiver clock phase drift (RW)##{}", "SppKalmanFilter").c_str(),
                                                  configWidth, unitWidth, &gui_covarianceClkPhaseDrift, reinterpret_cast<int*>(&gui_covarianceClkPhaseDriftUnit), "m/√(s^3)\0m^2/s^3\0\0",
                                                  0.0, 0.0, "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: gui_covarianceClkPhaseDrift changed to {}", "SppKalmanFilter", gui_covarianceClkPhaseDrift);
                LOG_DEBUG("{}: gui_covarianceClkPhaseDriftUnit changed to {}", "SppKalmanFilter", fmt::underlying(gui_covarianceClkPhaseDriftUnit));
                flow::ApplyChanges();
            }
            if (gui::widgets::InputDoubleWithUnit(fmt::format("Standard deviation of the receiver clock frequency drift (IRW)##{}", "SppKalmanFilter").c_str(),
                                                  configWidth, unitWidth, &gui_covarianceClkFrequencyDrift, reinterpret_cast<int*>(&gui_covarianceClkFrequencyDriftUnit), "m/√(s)\0m^2/s\0\0",
                                                  0.0, 0.0, "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: gui_covarianceClkFrequencyDrift changed to {}", "SppKalmanFilter", gui_covarianceClkFrequencyDrift);
                LOG_DEBUG("{}: gui_covarianceClkFrequencyDriftUnit changed to {}", "SppKalmanFilter", fmt::underlying(gui_covarianceClkFrequencyDriftUnit));
                flow::ApplyChanges();
            }
            if (gui::widgets::InputDoubleWithUnit(fmt::format("Standard deviation of the inter-system clock phase drift (RW)##{}", "SppKalmanFilter").c_str(),
                                                  configWidth, unitWidth, &gui_covarianceInterSysClkPhaseDrift, reinterpret_cast<int*>(&gui_covarianceInterSysClkPhaseDriftUnit), "m/√(s^3)\0m^2/s^3\0\0",
                                                  0.0, 0.0, "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: gui_covarianceInterSysClkPhaseDrift changed to {}", "SppKalmanFilter", gui_covarianceInterSysClkPhaseDrift);
                LOG_DEBUG("{}: gui_covarianceInterSysClkPhaseDriftUnit changed to {}", "SppKalmanFilter", fmt::underlying(gui_covarianceInterSysClkPhaseDriftUnit));
                flow::ApplyChanges();
            }
            if (gui::widgets::InputDoubleWithUnit(fmt::format("Standard deviation of the inter-system clock frequency drift (IRW)##{}", "SppKalmanFilter").c_str(),
                                                  configWidth, unitWidth, &gui_covarianceInterSysClkFrequencyDrift, reinterpret_cast<int*>(&gui_covarianceInterSysClkFrequencyDriftUnit), "m/√(s)\0m^2/s\0\0",
                                                  0.0, 0.0, "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: gui_covarianceInterSysClkFrequencyDrift changed to {}", "SppKalmanFilter", gui_covarianceInterSysClkFrequencyDrift);
                LOG_DEBUG("{}: gui_covarianceInterSysClkFrequencyDriftUnit changed to {}", "SppKalmanFilter", fmt::underlying(gui_covarianceInterSysClkFrequencyDriftUnit));
                flow::ApplyChanges();
            }
        }

        if (qCalculationAlgorithm == SppKalmanFilter::QCalculationAlgorithm::Taylor1)
        {
            ImGui::SetNextItemWidth(itemWidth);
            if (ImGui::InputFloat(fmt::format("Standard Deviation of Process Noise", "SppKalmanFilter").c_str(), &processNoiseStandardDeviation, 0.0, 0.0, "%.2e"))
            {
                LOG_DEBUG("{}: processNoiseStandardDeviation changed to {}", "SppKalmanFilter", processNoiseStandardDeviation);
                flow::ApplyChanges();
            }
        }
        ImGui::TreePop();
    }

    // ###########################################################################################################

    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    if (ImGui::TreeNode(fmt::format("P Error covariance matrix (init)##{}", "SppKalmanFilter").c_str()))
    {
        if (gui::widgets::InputDouble3WithUnit(fmt::format("ECEF Position Covariance ({})##{}",
                                                           gui_initCovariancePositionUnit == SppKalmanFilter::InitCovariancePositionUnits::m2
                                                               ? "Variance σ²"
                                                               : "Standard deviation σ",
                                                           "SppKalmanFilter")
                                                   .c_str(),
                                               configWidth, unitWidth, gui_initCovariancePosition.data(), reinterpret_cast<int*>(&gui_initCovariancePositionUnit), "m^2, m^2, m^2\0"
                                                                                                                                                                   "m, m, m\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: gui_initCovariancePosition changed to {}", "SppKalmanFilter", gui_initCovariancePosition);
            LOG_DEBUG("{}: gui_initCovariancePositionUnit changed to {}", "SppKalmanFilter", fmt::underlying(gui_initCovariancePositionUnit));
            flow::ApplyChanges();
        }

        if (gui::widgets::InputDouble3WithUnit(fmt::format("Velocity Covariance ({})##{}",
                                                           gui_initCovarianceVelocityUnit == SppKalmanFilter::InitCovarianceVelocityUnits::m2_s2
                                                               ? "Variance σ²"
                                                               : "Standard deviation σ",
                                                           "SppKalmanFilter")
                                                   .c_str(),
                                               configWidth, unitWidth, gui_initCovarianceVelocity.data(), reinterpret_cast<int*>(&gui_initCovarianceVelocityUnit), "m^2/s^2\0"
                                                                                                                                                                   "m/s\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: gui_initCovarianceVelocity changed to {}", "SppKalmanFilter", gui_initCovarianceVelocity);
            LOG_DEBUG("{}: gui_initCovarianceVelocityUnit changed to {}", "SppKalmanFilter", fmt::underlying(gui_initCovarianceVelocityUnit));
            flow::ApplyChanges();
        }

        if (gui::widgets::InputDoubleWithUnit(fmt::format("Receiver Clock Bias Covariance ({})##{}",
                                                          gui_initCovarianceRecvClkErrUnit == SppKalmanFilter::InitCovarianceRecvClkErrUnits::s2
                                                              ? "Variance σ²"
                                                              : "Standard deviation σ",
                                                          "SppKalmanFilter")
                                                  .c_str(),
                                              configWidth, unitWidth, &gui_initCovarianceRecvClkErr, reinterpret_cast<int*>(&gui_initCovarianceRecvClkErrUnit), "s^2\0"
                                                                                                                                                                "s\0\0",
                                              0.0, 0.0, "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: gui_initCovarianceRecvClkErr changed to {}", "SppKalmanFilter", gui_initCovarianceRecvClkErr);
            LOG_DEBUG("{}: gui_initCovarianceRecvClkErrUnit changed to {}", "SppKalmanFilter", fmt::underlying(gui_initCovarianceRecvClkErrUnit));
            flow::ApplyChanges();
        }

        if (gui::widgets::InputDoubleWithUnit(fmt::format("Receiver Clock Drift Covariance ({})##{}",
                                                          gui_initCovarianceRecvClkDriftUnit == SppKalmanFilter::InitCovarianceRecvClkDriftUnits::s2_s2
                                                              ? "Variance σ²"
                                                              : "Standard deviation σ",
                                                          "SppKalmanFilter")
                                                  .c_str(),
                                              configWidth, unitWidth, &gui_initCovarianceRecvClkDrift, reinterpret_cast<int*>(&gui_initCovarianceRecvClkDriftUnit), "s^2/s^2\0"
                                                                                                                                                                    "s/s\0\0",
                                              0.0, 0.0, "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: gui_initCovarianceRecvClkDrift changed to {}", "SppKalmanFilter", gui_initCovarianceRecvClkDrift);
            LOG_DEBUG("{}: gui_initCovarianceRecvClkDriftUnit changed to {}", "SppKalmanFilter", fmt::underlying(gui_initCovarianceRecvClkDriftUnit));
            flow::ApplyChanges();
        }

        if (gui::widgets::InputDoubleWithUnit(fmt::format("Inter-system Clock Offsets Covariances ({})##{}",
                                                          gui_initCovarianceInterSysErrUnit == SppKalmanFilter::InitCovarianceRecvClkErrUnits::s2
                                                              ? "Variance σ²"
                                                              : "Standard deviation σ",
                                                          "SppKalmanFilter")
                                                  .c_str(),
                                              configWidth, unitWidth, &gui_initCovarianceInterSysErr, reinterpret_cast<int*>(&gui_initCovarianceInterSysErrUnit), "s^2\0"
                                                                                                                                                                  "s\0\0",
                                              0.0, 0.0, "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: gui_initCovarianceInterSysErr changed to {}", "SppKalmanFilter", gui_initCovarianceInterSysErr);
            LOG_DEBUG("{}: gui_initCovarianceInterSysErrUnit changed to {}", "SppKalmanFilter", fmt::underlying(gui_initCovarianceInterSysErrUnit));
            flow::ApplyChanges();
        }

        if (gui::widgets::InputDoubleWithUnit(fmt::format("Inter-system Clock Offset Drift Covariances ({})##{}",
                                                          gui_initCovarianceInterSysDriftUnit == SppKalmanFilter::InitCovarianceRecvClkDriftUnits::s2_s2
                                                              ? "Variance σ²"
                                                              : "Standard deviation σ",
                                                          "SppKalmanFilter")
                                                  .c_str(),
                                              configWidth, unitWidth, &gui_initCovarianceInterSysDrift, reinterpret_cast<int*>(&gui_initCovarianceInterSysDriftUnit), "s^2\0"
                                                                                                                                                                      "s\0\0",
                                              0.0, 0.0, "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: gui_initCovarianceInterSysDrift changed to {}", "SppKalmanFilter", gui_initCovarianceInterSysDrift);
            LOG_DEBUG("{}: gui_initCovarianceInterSysDriftUnit changed to {}", "SppKalmanFilter", fmt::underlying(gui_initCovarianceInterSysDriftUnit));
            flow::ApplyChanges();
        }

        ImGui::TreePop();
    }
}

void SppKalmanFilter::initialize()
{
    _lastUpdate.reset();
    _kalmanFilter = KeyedKalmanFilterD<States::StateKeyTypes, Meas::MeasKeyTypes>{ States::PosVelRecvClk, {} };
    _initialized = false;
    _refTimeSatSys = SatSys_None;

    // Initial Covariance of the acceleration 𝜎_a due to user motion in horizontal and vertical component [m²/s³]
    if (gui_covarianceAccelUnit == CovarianceAccelUnits::m_sqrts3)
    {
        _covarianceAccel = gui_covarianceAccel.array().pow(2);
    }
    else if (gui_covarianceAccelUnit == CovarianceAccelUnits::m2_s3)
    {
        _covarianceAccel = gui_covarianceAccel;
    }

    // Initial Covariance of the clock phase drift [m²/s³]
    if (gui_covarianceClkPhaseDriftUnit == CovarianceClkPhaseDriftUnits::m_sqrts3)
    {
        _covarianceClkPhaseDrift = std::pow(gui_covarianceClkPhaseDrift, 2);
    }
    else if (gui_covarianceClkPhaseDriftUnit == CovarianceClkPhaseDriftUnits::m2_s3)
    {
        _covarianceClkPhaseDrift = gui_covarianceClkPhaseDrift;
    }

    // Initial Covariance of the frequency phase drift [m²/s]
    if (gui_covarianceClkFrequencyDriftUnit == CovarianceClkFrequencyDriftUnits::m_sqrts)
    {
        _covarianceClkFrequencyDrift = std::pow(gui_covarianceClkFrequencyDrift, 2);
    }
    else if (gui_covarianceClkFrequencyDriftUnit == CovarianceClkFrequencyDriftUnits::m2_s)
    {
        _covarianceClkFrequencyDrift = gui_covarianceClkFrequencyDrift;
    }

    // Initial Covariance of the inter-system clock phase drift [m²/s³]
    if (gui_covarianceInterSysClkPhaseDriftUnit == CovarianceClkPhaseDriftUnits::m_sqrts3)
    {
        _covarianceInterSysClkPhaseDrift = std::pow(gui_covarianceInterSysClkPhaseDrift, 2);
    }
    else if (gui_covarianceInterSysClkPhaseDriftUnit == CovarianceClkPhaseDriftUnits::m2_s3)
    {
        _covarianceInterSysClkPhaseDrift = gui_covarianceInterSysClkPhaseDrift;
    }

    // Initial Covariance of the inter-system frequency phase drift [m²/s]
    if (gui_covarianceInterSysClkFrequencyDriftUnit == CovarianceClkFrequencyDriftUnits::m_sqrts)
    {
        _covarianceInterSysClkFrequencyDrift = std::pow(gui_covarianceInterSysClkFrequencyDrift, 2);
    }
    else if (gui_covarianceInterSysClkFrequencyDriftUnit == CovarianceClkFrequencyDriftUnits::m2_s)
    {
        _covarianceInterSysClkFrequencyDrift = gui_covarianceInterSysClkFrequencyDrift;
    }

    // ###########################################################################################################
    // Convert initial covariance (are used or partly used if Least squares has not more measurements than parameters to calculate uncertainty)

    // Initial Covariance of the position in [m²]
    if (gui_initCovariancePositionUnit == InitCovariancePositionUnits::m)
    {
        _initCovariancePosition = gui_initCovariancePosition.array().pow(2);
    }
    else if (gui_initCovariancePositionUnit == InitCovariancePositionUnits::m2)
    {
        _initCovariancePosition = gui_initCovariancePosition;
    }

    // Initial Covariance of the velocity in [m²/s²]
    if (gui_initCovarianceVelocityUnit == InitCovarianceVelocityUnits::m_s)
    {
        _initCovarianceVelocity = gui_initCovarianceVelocity.array().pow(2);
    }
    else if (gui_initCovarianceVelocityUnit == InitCovarianceVelocityUnits::m2_s2)
    {
        _initCovarianceVelocity = gui_initCovarianceVelocity;
    }

    // Initial Covariance of the receiver clock error [m²]
    if (gui_initCovarianceRecvClkErrUnit == InitCovarianceRecvClkErrUnits::s2)
    {
        _initCovarianceRecvClkErr = gui_initCovarianceRecvClkErr * std::pow(InsConst::C, 2);
    }
    else if (gui_initCovarianceRecvClkErrUnit == InitCovarianceRecvClkErrUnits::s)
    {
        _initCovarianceRecvClkErr = std::pow(gui_initCovarianceRecvClkErr * InsConst::C, 2);
    }

    // Initial Covariance of the receiver clock drift [m²/s²]
    if (gui_initCovarianceRecvClkDriftUnit == InitCovarianceRecvClkDriftUnits::s2_s2)
    {
        _initCovarianceRecvClkDrift = gui_initCovarianceRecvClkDrift * std::pow(InsConst::C, 2);
    }
    else if (gui_initCovarianceRecvClkDriftUnit == InitCovarianceRecvClkDriftUnits::s_s)
    {
        _initCovarianceRecvClkDrift = std::pow(gui_initCovarianceRecvClkDrift * InsConst::C, 2);
    }

    // Initial Covariance of the inter-system clock error [m²]
    if (gui_initCovarianceInterSysErrUnit == InitCovarianceRecvClkErrUnits::s2)
    {
        _initCovarianceInterSysErr = gui_initCovarianceInterSysErr * std::pow(InsConst::C, 2);
    }
    else if (gui_initCovarianceInterSysErrUnit == InitCovarianceRecvClkErrUnits::s)
    {
        _initCovarianceInterSysErr = std::pow(gui_initCovarianceInterSysErr * InsConst::C, 2);
    }

    // Initial Covariance of the inter-system clock drift [m²/s²]
    if (gui_initCovarianceInterSysDriftUnit == InitCovarianceRecvClkDriftUnits::s2_s2)
    {
        _initCovarianceInterSysDrift = gui_initCovarianceInterSysDrift * std::pow(InsConst::C, 2);
    }
    else if (gui_initCovarianceInterSysDriftUnit == InitCovarianceRecvClkDriftUnits::s_s)
    {
        _initCovarianceInterSysDrift = std::pow(gui_initCovarianceInterSysDrift * InsConst::C, 2);
    }
}

bool SppKalmanFilter::isInitialized() const
{
    return _initialized;
}

// ###########################################################################################################
//                                             Getter and Setter
// ###########################################################################################################
void SppKalmanFilter::setAllSatSysExceptRef(const std::vector<SatelliteSystem>& allSatSys)
{
    _allSatSysExceptRef = allSatSys;
}

std::vector<SatelliteSystem> SppKalmanFilter::getAllSatSysExceptRef() const
{
    return _allSatSysExceptRef;
}

SatelliteSystem SppKalmanFilter::getRefTimeSatSys() const
{
    return _refTimeSatSys;
}

// ###########################################################################################################
//                                          Initialize Kalman Filter
// ###########################################################################################################
void SppKalmanFilter::initializeKalmanFilter(std::shared_ptr<SppSolution>& sppSolLSE,
                                             const std::vector<States::StateKeyTypes>& interSysErrs,
                                             const std::vector<States::StateKeyTypes>& interSysDrifts)
{
    auto refSys = sppSolLSE->recvClk.referenceTimeSatelliteSystem;
    _refTimeSatSys = refSys;

    // ----------------------------------------------------- Initialize Kalman Filter state x --------------------------------------------------------

    _kalmanFilter.x.segment(States::Pos) = sppSolLSE->e_position();                    // _e_position
    _kalmanFilter.x(States::RecvClkErr) = sppSolLSE->recvClk.bias.value * InsConst::C; // receiver clock error

    if (std::none_of(sppSolLSE->e_velocity().begin(), sppSolLSE->e_velocity().end(), [](double d) { return std::isnan(d); })) // in case doppler observations were available for LSE
    {
        _kalmanFilter.x.segment(States::Vel) = sppSolLSE->e_velocity();                       // _e_velocity
        _kalmanFilter.x(States::RecvClkDrift) = sppSolLSE->recvClk.drift.value * InsConst::C; // receiver clock drift
    }

    // Inter-system clock error and drift
    for (const auto& otherSys : sppSolLSE->usedSatSysExceptRef)
    {
        _kalmanFilter.x(States::InterSysErr{ otherSys }) = sppSolLSE->recvClk.sysTimeDiff.at(otherSys).value * InsConst::C;
        if (std::none_of(sppSolLSE->e_velocity().begin(), sppSolLSE->e_velocity().end(), [](double d) { return std::isnan(d); })) // in case doppler observations were available for LSE
        {
            _kalmanFilter.x(States::InterSysDrift{ otherSys }) = sppSolLSE->recvClk.sysDriftDiff.at(otherSys).value * InsConst::C;
        }
    }

    LOG_DATA("x =\n{}", _kalmanFilter.x);

    // -------------------------------------------------- Initialize Kalman Filter covariance P ------------------------------------------------------

    // Find Satellite systems that are selected but not available in first epoch
    std::vector<SatelliteSystem> satSysDiff;
    std::set_difference(_allSatSysExceptRef.begin(), _allSatSysExceptRef.end(),
                        sppSolLSE->usedSatSysExceptRef.begin(), sppSolLSE->usedSatSysExceptRef.end(),
                        std::inserter(satSysDiff, satSysDiff.begin()));

    // Check if covariances are available (can only be calculated in LSE with more measurements than estimated parameters)
    if (std::none_of(sppSolLSE->e_positionStdev().begin(), sppSolLSE->e_positionStdev().end(), [](double d) { return std::isnan(d); }))
    {
        _kalmanFilter.P(States::PosRecvClkErr, States::PosRecvClkErr) = sppSolLSE->e_CovarianceMatrix()(States::PosRecvClkErr, States::PosRecvClkErr);
        _kalmanFilter.P(interSysErrs, interSysErrs) = sppSolLSE->e_CovarianceMatrix()(interSysErrs, interSysErrs);
        _kalmanFilter.P(States::PosRecvClkErr, interSysErrs) = sppSolLSE->e_CovarianceMatrix()(States::PosRecvClkErr, interSysErrs);
        _kalmanFilter.P(interSysErrs, States::PosRecvClkErr) = sppSolLSE->e_CovarianceMatrix()(interSysErrs, States::PosRecvClkErr);
        if (!satSysDiff.empty())
        {
            for (const auto& satSysDiff_i : satSysDiff)
            {
                _kalmanFilter.P(States::InterSysErr{ satSysDiff_i }, States::InterSysErr{ satSysDiff_i }) = _initCovarianceInterSysErr;
            }
        }
    }
    else
    {
        _kalmanFilter.P(States::Pos, States::Pos) = _initCovariancePosition.asDiagonal();
        _kalmanFilter.P(States::RecvClkErr, States::RecvClkErr) = _initCovarianceRecvClkErr;
        for (const auto& otherSys_i : _allSatSysExceptRef)
        {
            _kalmanFilter.P(States::InterSysErr{ otherSys_i }, States::InterSysErr{ otherSys_i }) = _initCovarianceInterSysErr;
        }
    }
    // Check if covariances are available (can only be calculated in LSE with more measurements than estimated parameters)
    if (std::none_of(sppSolLSE->e_velocityStdev().begin(), sppSolLSE->e_velocityStdev().end(), [](double d) { return std::isnan(d); }))
    {
        _kalmanFilter.P(States::VelRecvClkDrift, States::VelRecvClkDrift) = sppSolLSE->e_CovarianceMatrix()(States::VelRecvClkDrift, States::VelRecvClkDrift);
        _kalmanFilter.P(interSysDrifts, interSysDrifts) = sppSolLSE->e_CovarianceMatrix()(interSysDrifts, interSysDrifts);
        _kalmanFilter.P(States::VelRecvClkDrift, interSysDrifts) = sppSolLSE->e_CovarianceMatrix()(States::VelRecvClkDrift, interSysDrifts);
        _kalmanFilter.P(interSysDrifts, States::VelRecvClkDrift) = sppSolLSE->e_CovarianceMatrix()(interSysDrifts, States::VelRecvClkDrift);
        if (!satSysDiff.empty())
        {
            for (const auto& satSysDiff_i : satSysDiff)
            {
                _kalmanFilter.P(States::InterSysDrift{ satSysDiff_i }, States::InterSysDrift{ satSysDiff_i }) = _initCovarianceInterSysDrift;
            }
        }
    }
    else
    {
        _kalmanFilter.P(States::Vel, States::Vel) = _initCovarianceVelocity.asDiagonal();
        _kalmanFilter.P(States::RecvClkDrift, States::RecvClkDrift) = _initCovarianceRecvClkDrift;
        for (const auto& otherSys_i : _allSatSysExceptRef)
        {
            _kalmanFilter.P(States::InterSysDrift{ otherSys_i }, States::InterSysDrift{ otherSys_i }) = _initCovarianceInterSysDrift;
        }
    }

    LOG_DATA("P =\n{}", _kalmanFilter.P);

    // ---------------------------------------------------- Initialize Kalman Filter matrices --------------------------------------------------------

    // System matrix - Groves ch. 9.4.2.2, eq. 9.148, p. 415
    _kalmanFilter.F(States::Pos, States::Vel) = Eigen::Matrix3d::Identity();
    _kalmanFilter.F(States::RecvClkErr, States::RecvClkDrift) = 1;
    // inter-system system clock error and drift
    for (const auto& satSys : _allSatSysExceptRef)
    {
        _kalmanFilter.F(States::InterSysErr{ satSys }, States::InterSysDrift{ satSys }) = 1;
    }
    LOG_DATA("F =\n{}", _kalmanFilter.F);

    // Fix part of Noise input matrix G and Noise scale matrix W
    _kalmanFilter.G(States::RecvClkErr, States::RecvClkErr) = 1;
    _kalmanFilter.G(States::RecvClkDrift, States::RecvClkDrift) = 1;
    _kalmanFilter.W(States::RecvClkErr, States::RecvClkErr) = _covarianceClkPhaseDrift;
    _kalmanFilter.W(States::RecvClkDrift, States::RecvClkDrift) = _covarianceClkFrequencyDrift;
    for (const auto& satSys : _allSatSysExceptRef)
    {
        _kalmanFilter.G(States::InterSysErr{ satSys }, States::InterSysErr{ satSys }) = 1;
        _kalmanFilter.G(States::InterSysDrift{ satSys }, States::InterSysDrift{ satSys }) = 1;

        _kalmanFilter.W(States::InterSysErr{ satSys }, States::InterSysErr{ satSys }) = _covarianceInterSysClkPhaseDrift;
        _kalmanFilter.W(States::InterSysDrift{ satSys }, States::InterSysDrift{ satSys }) = _covarianceInterSysClkFrequencyDrift;
    }
    LOG_DATA("G =\n{}", _kalmanFilter.G);
    LOG_DATA("W =\n{}", _kalmanFilter.W);

    // -----------------------------------------------------------------------------------------------------------------------------------------------

    _initialized = true;
    LOG_DATA("_initialized {}", _initialized);

    _lastUpdate = sppSolLSE->insTime;
    LOG_DATA("_lastTime {}", _lastUpdate);

    assignInternalSolution(_allSatSysExceptRef);

    sppSolLSE->setCovarianceMatrix(_kalmanFilter.P);
}

// ###########################################################################################################
//                                          Add InterSys States keys
// ###########################################################################################################
void SppKalmanFilter::addInterSysStateKeys([[maybe_unused]] const InsTime& insTime)
{
    // Add Kalman Filter States (inter-system clock errors and drifts)
    for (const auto& satelliteSystem : _allSatSysExceptRef)
    {
        auto keyErr = States::InterSysErr{ satelliteSystem };
        auto keyDrift = States::InterSysDrift{ satelliteSystem };

        if (!_kalmanFilter.x.hasRow(keyErr))
        {
            LOG_DEBUG("{}: [{}] Adding state: {}", "SppKalmanFilter", insTime.toYMDHMS(GPST), keyErr);
            _kalmanFilter.addState(keyErr);
            LOG_DEBUG("{}: [{}] Adding state: {}", "SppKalmanFilter", insTime.toYMDHMS(GPST), keyDrift);
            _kalmanFilter.addState(keyDrift);
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
}

// ###########################################################################################################
//                                  Assign Solution of Kalman Filter estimation
// ###########################################################################################################
void SppKalmanFilter::assignSolution(std::shared_ptr<NAV::SppSolution>& sppSol,
                                     const std::vector<SatelliteSystem>& otherSatelliteSystems)
{
    assignInternalSolution(otherSatelliteSystems);

    sppSol->setPositionAndStdDev_e(_e_position, _kalmanFilter.P(States::Pos, States::Pos));
    sppSol->setVelocityAndStdDev_e(_e_velocity, _kalmanFilter.P(States::Vel, States::Vel));
    sppSol->recvClk = _recvClk;
    sppSol->setCovarianceMatrix(_kalmanFilter.P);
}

void SppKalmanFilter::assignInternalSolution(const std::vector<SatelliteSystem>& otherSatelliteSystems)
{
    _e_position = _kalmanFilter.x(States::Pos);
    _e_velocity = _kalmanFilter.x(States::Vel);
    _recvClk.bias.value = _kalmanFilter.x(States::RecvClkErr) / InsConst::C;
    _recvClk.bias.stdDev = std::sqrt(_kalmanFilter.P(States::RecvClkErr, States::RecvClkErr)) / InsConst::C;
    _recvClk.drift.value = _kalmanFilter.x(States::RecvClkDrift) / InsConst::C;
    _recvClk.drift.stdDev = std::sqrt(_kalmanFilter.P(States::RecvClkDrift, States::RecvClkDrift)) / InsConst::C;
    _recvClk.referenceTimeSatelliteSystem = _refTimeSatSys;

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
std::shared_ptr<NAV::SppSolution> SppKalmanFilter::estimateSppSolution(const InsTime& insTime,
                                                                       std::vector<CalcData>& calcData,
                                                                       const IonosphericCorrections& ionosphericCorrections,
                                                                       const IonosphereModel& ionosphereModel,
                                                                       const TroposphereModelSelection& troposphereModels,
                                                                       const GnssMeasurementErrorModel& gnssMeasurementErrorModel,
                                                                       double elevationMask,
                                                                       bool useDoppler,
                                                                       std::vector<States::StateKeyTypes>& interSysErrs,
                                                                       std::vector<States::StateKeyTypes>& interSysDrifts)
{
    auto sppSol = std::make_shared<SppSolution>();
    sppSol->insTime = insTime;

    kalmanFilterPrediction(insTime);
    assignSolution(sppSol, _allSatSysExceptRef);

    if (!calcData.empty())
    {
        // Sorted list of satellite systems of this epoch
        std::set<SatelliteSystem> sortedAvailableSatelliteSystems;
        for (const auto& calc : calcData) { sortedAvailableSatelliteSystems.insert(calc.obsData.satSigId.toSatId().satSys); }

        // Get a vector of all available systems (sorted) of this epoch
        std::vector<SatelliteSystem> availableSatelliteSystems;
        availableSatelliteSystems.reserve(sortedAvailableSatelliteSystems.size());
        std::copy(sortedAvailableSatelliteSystems.begin(), sortedAvailableSatelliteSystems.end(), std::back_inserter(availableSatelliteSystems));

        // Amount of estimated parameters
        size_t nParam = 4 + sortedAvailableSatelliteSystems.size() - 1; // 3x pos, 1x clk, (N-1)x clkDiff (= nDoppler Param -> 3x vel, 1x clkDrift, (N-1)x clkDiffDrift)
        LOG_DATA("nParam {}", nParam);
        // Amount of pseudorange measurements
        size_t nMeasPsr = calcData.size();
        LOG_DATA("nMeasPsr {}", nMeasPsr);

        // Find all observations providing a doppler measurement (for velocity calculation)
        size_t nDopplerMeas = 0;
        if (useDoppler)
        {
            nDopplerMeas = findDopplerMeasurements(calcData);
        }
        LOG_DATA("nDopplerMeas {}", nDopplerMeas);

        // Latitude, Longitude, Altitude of the receiver [rad, rad, m]
        Eigen::Vector3d lla_pos = trafo::ecef2lla_WGS84(_kalmanFilter.x(States::Pos));

        State state = SPP::State{ .e_position = _e_position,
                                  .e_velocity = _e_velocity,
                                  .recvClk = _recvClk };

        calcDataBasedOnEstimates(sppSol, availableSatelliteSystems, calcData, state,
                                 nParam, nMeasPsr, nDopplerMeas, insTime, lla_pos,
                                 elevationMask, EstimatorType::KF);

        if (sppSol->nSatellitesPosition + sppSol->nSatellitesVelocity == 0)
        {
            return sppSol; // no update
        }

        getInterSysKeys(availableSatelliteSystems, interSysErrs, interSysDrifts);

        auto [e_H_psr,     // Measurement/Geometry matrix for the pseudorange
              psrEst,      // Pseudorange estimates [m]
              psrMeas,     // Pseudorange measurements [m]
              W_psr,       // Pseudorange measurement error weight matrix
              dpsr,        // Difference between Pseudorange measurements and estimates
              e_H_r,       // Measurement/Geometry matrix for the pseudorange-rate
              psrRateEst,  // Corrected pseudorange-rate estimates [m/s]
              psrRateMeas, // Corrected pseudorange-rate measurements [m/s]
              W_psrRate,   // Pseudorange rate (doppler) measurement error weight matrix
              dpsr_dot     // Difference between Pseudorange rate measurements and estimates
        ] = calcMeasurementEstimatesAndDesignMatrix(sppSol, calcData,
                                                    insTime,
                                                    state, lla_pos,
                                                    ionosphericCorrections, ionosphereModel,
                                                    troposphereModels, gnssMeasurementErrorModel,
                                                    EstimatorType::KF, useDoppler, interSysErrs, interSysDrifts);

        kalmanFilterUpdate(dpsr, e_H_psr, W_psr, dpsr_dot, e_H_r, W_psrRate,
                           sppSol->recvClk.referenceTimeSatelliteSystem, useDoppler, interSysErrs, interSysDrifts, insTime);
        assignSolution(sppSol, availableSatelliteSystems);

        _lastUpdate = sppSol->insTime;
        LOG_DATA("_lastTime {}", _lastUpdate);
    }

    return sppSol;
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

        // Position dependent part of Noise scale matrix W in n-frame
        _kalmanFilter.W(States::Vel, States::Vel) = Eigen::DiagonalMatrix<double, 3>(_covarianceAccel(0), _covarianceAccel(0), _covarianceAccel(1));

        _kalmanFilter.calcPhiAndQWithVanLoanMethod(dt);
    }
    LOG_DATA("G =\n{}", _kalmanFilter.G);
    LOG_DATA("W =\n{}", _kalmanFilter.W);
    LOG_DATA("GWG^T =\n{}",
             KeyedMatrixXd<States::StateKeyTypes>(_kalmanFilter.G(all, all)
                                                      * _kalmanFilter.W(all, all)
                                                      * _kalmanFilter.G(all, all).transpose(),
                                                  _kalmanFilter.G.rowKeys()));
    LOG_DATA("State transition matrix Phi {}", _kalmanFilter.Phi);
    LOG_DATA("Process noise matrix Q {}", _kalmanFilter.Q);

    LOG_DATA("x (a posteriori, t-1 = {}) =\n{}", _lastUpdate, _kalmanFilter.x);
    LOG_DATA("P (a posteriori, t-1 = {}) =\n{}", _lastUpdate, _kalmanFilter.P);
    _kalmanFilter.predict();
    LOG_DATA("x (a priori    , t   = {}) =\n{}", insTime, _kalmanFilter.x);
    LOG_DATA("P (a priori    , t   = {}) =\n{}", insTime, _kalmanFilter.P);

    LOG_DATA("dx Prediction (ECEF) = {}", (_kalmanFilter.x.segment<3>(States::Pos) - _e_position));
    LOG_DATA("dv Prediction (ECEF) = {}", (_kalmanFilter.x.segment<3>(States::Vel) - _e_velocity));
}

// ###########################################################################################################
//                                          Kalman Filter Update
// ###########################################################################################################
void SppKalmanFilter::kalmanFilterUpdate(const KeyedVectorXd<Meas::MeasKeyTypes>& dpsr,
                                         const KeyedMatrixXd<Meas::MeasKeyTypes, States::StateKeyTypes>& e_H_psr,
                                         const KeyedMatrixXd<Meas::MeasKeyTypes, Meas::MeasKeyTypes>& W_psr,
                                         const KeyedVectorXd<Meas::MeasKeyTypes>& dpsr_dot,
                                         const KeyedMatrixXd<Meas::MeasKeyTypes, States::StateKeyTypes>& e_H_r,
                                         const KeyedMatrixXd<Meas::MeasKeyTypes, Meas::MeasKeyTypes>& W_psrRate,
                                         SatelliteSystem sppSolReferenceTimeSatelliteSystem,
                                         bool useDoppler,
                                         const std::vector<States::StateKeyTypes>& interSysErrs,
                                         const std::vector<States::StateKeyTypes>& interSysDrifts,
                                         [[maybe_unused]] const InsTime& insTime)
{
    // Update the Measurement sensitivity Matrix (𝐇), the Measurement noise covariance matrix (𝐑) and the Measurement vector (𝐳)

    // Get measurment keys to set Kalman Filter matrices and vectors
    std::vector<Meas::MeasKeyTypes> measKeys;
    useDoppler ? measKeys.reserve(static_cast<size_t>(dpsr.rows() * 2)) : measKeys.reserve(static_cast<size_t>(dpsr.rows()));

    for (const auto& key : dpsr.rowKeys())
    {
        measKeys.emplace_back(key);
    }
    for (const auto& key : dpsr_dot.rowKeys())
    {
        measKeys.emplace_back(key);
    }
    _kalmanFilter.setMeasurements(measKeys);

    // check whether reference time satellite system has changed
    bool referenceTimeSatelliteSystemChanged = false;
    if (_refTimeSatSys != sppSolReferenceTimeSatelliteSystem)
    {
        referenceTimeSatelliteSystemChanged = true;
        LOG_WARN("Reference satellite system for clock estimation is not available in this epoch. Inter-system clock update cannot be performed.");
    }

    // Innovation - Groves ch. 9.4.2.3, eq. 9.159, p. 420
    // Design matrix - Groves ch. 9.4.2.3, eq. 9.163, p. 420
    for (const auto& key : measKeys)
    {
        if (const auto key_ = std::get_if<Meas::Psr>(&key))
        {
            _kalmanFilter.z(key) = dpsr(key);
            _kalmanFilter.H(key, States::PosRecvClkErr) = e_H_psr(key, States::PosRecvClkErr);
            if (!referenceTimeSatelliteSystemChanged)
            {
                _kalmanFilter.H(key, interSysErrs) = e_H_psr(key, interSysErrs);
            }
            _kalmanFilter.R(key, key) = W_psr(key, key);
        }
        else if (const auto* const key_ = std::get_if<Meas::Doppler>(&key))
        {
            _kalmanFilter.z(key) = dpsr_dot(key);
            _kalmanFilter.H(key, States::VelRecvClkDrift) = e_H_r(key, States::VelRecvClkDrift);
            if (!referenceTimeSatelliteSystemChanged)
            {
                _kalmanFilter.H(key, interSysDrifts) = e_H_r(key, interSysDrifts);
            }
            _kalmanFilter.R(key, key) = W_psrRate(key, key);
        }
    }

    LOG_DATA("z =\n{}", _kalmanFilter.z.transposed());
    LOG_DATA("H =\n{}", _kalmanFilter.H);
    LOG_DATA("R =\n{}", _kalmanFilter.R);

    _kalmanFilter.correctWithMeasurementInnovation();
    LOG_DATA("x (a posteriori, t   = {}) =\n{}", insTime, _kalmanFilter.x);
    LOG_DATA("P (a posteriori, t   = {}) =\n{}", insTime, _kalmanFilter.P);

    LOG_DATA("dx Update (ECEF) = {}", (_kalmanFilter.x.segment<3>(States::Pos) - _e_position));
    LOG_DATA("dv Update (ECEF) = {}", (_kalmanFilter.x.segment<3>(States::Vel) - _e_velocity));
}

void to_json(json& j, const SppKalmanFilter& data)
{
    j["qCalculationAlgorithm"] = data.qCalculationAlgorithm;

    j["gui_covarianceAccelUnit"] = data.gui_covarianceAccelUnit;
    j["gui_covarianceAccel"] = data.gui_covarianceAccel;
    j["_covarianceAccel"] = data._covarianceAccel;

    j["gui_covarianceClkPhaseDriftUnit"] = data.gui_covarianceClkPhaseDriftUnit;
    j["gui_covarianceClkPhaseDrift"] = data.gui_covarianceClkPhaseDrift;
    j["_covarianceClkPhaseDrift"] = data._covarianceClkPhaseDrift;
    j["gui_covarianceClkFrequencyDriftUnit"] = data.gui_covarianceClkFrequencyDriftUnit;
    j["gui_covarianceClkFrequencyDrift"] = data.gui_covarianceClkFrequencyDrift;
    j["_covarianceClkFrequencyDrift"] = data._covarianceClkFrequencyDrift;
    j["gui_covarianceInterSysClkPhaseDriftUnit"] = data.gui_covarianceInterSysClkPhaseDriftUnit;
    j["gui_covarianceInterSysClkPhaseDrift"] = data.gui_covarianceInterSysClkPhaseDrift;
    j["_covarianceInterSysClkPhaseDrift"] = data._covarianceInterSysClkPhaseDrift;
    j["gui_covarianceInterSysClkFrequencyDriftUnit"] = data.gui_covarianceInterSysClkFrequencyDriftUnit;
    j["gui_covarianceInterSysClkFrequencyDrift"] = data.gui_covarianceInterSysClkFrequencyDrift;
    j["_covarianceInterSysClkFrequencyDrift"] = data._covarianceInterSysClkFrequencyDrift;

    j["processNoiseStandardDeviation"] = data.processNoiseStandardDeviation;

    j["gui_initCovariancePositionUnit"] = data.gui_initCovariancePositionUnit;
    j["gui_initCovariancePosition"] = data.gui_initCovariancePosition;
    j["_initCovariancePosition"] = data._initCovariancePosition;
    j["gui_initCovarianceVelocityUnit"] = data.gui_initCovarianceVelocityUnit;
    j["gui_initCovarianceVelocity"] = data.gui_initCovarianceVelocity;
    j["_initCovarianceVelocity"] = data._initCovarianceVelocity;
    j["gui_initCovarianceRecvClkErrUnit"] = data.gui_initCovarianceRecvClkErrUnit;
    j["gui_initCovarianceRecvClkErr"] = data.gui_initCovarianceRecvClkErr;
    j["_initCovarianceRecvClkErr"] = data._initCovarianceRecvClkErr;
    j["gui_initCovarianceRecvClkDriftUnit"] = data.gui_initCovarianceRecvClkDriftUnit;
    j["gui_initCovarianceRecvClkDrift"] = data.gui_initCovarianceRecvClkDrift;
    j["_initCovarianceRecvClkDrift"] = data._initCovarianceRecvClkDrift;
    j["gui_initCovarianceInterSysErrUnit"] = data.gui_initCovarianceInterSysErrUnit;
    j["gui_initCovarianceInterSysErr"] = data.gui_initCovarianceInterSysErr;
    j["_initCovarianceInterSysErr"] = data._initCovarianceInterSysErr;
    j["gui_initCovarianceInterSysDriftUnit"] = data.gui_initCovarianceInterSysDriftUnit;
    j["gui_initCovarianceInterSysDrift"] = data.gui_initCovarianceInterSysDrift;
    j["_initCovarianceInterSysDrift"] = data._initCovarianceInterSysDrift;
}

void from_json(const json& j, SppKalmanFilter& data)
{
    // ###########################################################################################################

    if (j.contains("qCalculationAlgorithm")) { j.at("qCalculationAlgorithm").get_to(data.qCalculationAlgorithm); }

    if (j.contains("gui_covarianceAccelUnit")) { j.at("gui_covarianceAccelUnit").get_to(data.gui_covarianceAccelUnit); }
    if (j.contains("gui_covarianceAccel")) { j.at("gui_covarianceAccel").get_to(data.gui_covarianceAccel); }
    if (j.contains("_covarianceAccel")) { j.at("_covarianceAccel").get_to(data._covarianceAccel); }

    if (j.contains("_covarianceClkPhaseDrift")) { j.at("_covarianceClkPhaseDrift").get_to(data._covarianceClkPhaseDrift); }
    if (j.contains("gui_covarianceClkPhaseDrift")) { j.at("gui_covarianceClkPhaseDrift").get_to(data.gui_covarianceClkPhaseDrift); }
    if (j.contains("gui_covarianceClkPhaseDriftUnit")) { j.at("gui_covarianceClkPhaseDriftUnit").get_to(data.gui_covarianceClkPhaseDriftUnit); }
    if (j.contains("_covarianceClkFrequencyDrift")) { j.at("_covarianceClkFrequencyDrift").get_to(data._covarianceClkFrequencyDrift); }
    if (j.contains("gui_covarianceClkFrequencyDrift")) { j.at("gui_covarianceClkFrequencyDrift").get_to(data.gui_covarianceClkFrequencyDrift); }
    if (j.contains("gui_covarianceClkFrequencyDriftUnit")) { j.at("gui_covarianceClkFrequencyDriftUnit").get_to(data.gui_covarianceClkFrequencyDriftUnit); }
    if (j.contains("_covarianceInterSysClkPhaseDrift")) { j.at("_covarianceInterSysClkPhaseDrift").get_to(data._covarianceInterSysClkPhaseDrift); }
    if (j.contains("gui_covarianceInterSysClkPhaseDrift")) { j.at("gui_covarianceInterSysClkPhaseDrift").get_to(data.gui_covarianceInterSysClkPhaseDrift); }
    if (j.contains("gui_covarianceInterSysClkPhaseDriftUnit")) { j.at("gui_covarianceInterSysClkPhaseDriftUnit").get_to(data.gui_covarianceInterSysClkPhaseDriftUnit); }
    if (j.contains("_covarianceInterSysClkFrequencyDrift")) { j.at("_covarianceInterSysClkFrequencyDrift").get_to(data._covarianceInterSysClkFrequencyDrift); }
    if (j.contains("gui_covarianceInterSysClkFrequencyDrift")) { j.at("gui_covarianceInterSysClkFrequencyDrift").get_to(data.gui_covarianceInterSysClkFrequencyDrift); }
    if (j.contains("gui_covarianceInterSysClkFrequencyDriftUnit")) { j.at("gui_covarianceInterSysClkFrequencyDriftUnit").get_to(data.gui_covarianceInterSysClkFrequencyDriftUnit); }

    if (j.contains("processNoiseStandardDeviation")) { j.at("processNoiseStandardDeviation").get_to(data.processNoiseStandardDeviation); }

    // ###########################################################################################################

    if (j.contains("gui_initCovariancePositionUnit")) { j.at("gui_initCovariancePositionUnit").get_to(data.gui_initCovariancePositionUnit); }
    if (j.contains("gui_initCovariancePosition")) { j.at("gui_initCovariancePosition").get_to(data.gui_initCovariancePosition); }
    if (j.contains("_initCovariancePosition")) { j.at("_initCovariancePosition").get_to(data._initCovariancePosition); }

    if (j.contains("gui_initCovarianceVelocityUnit")) { j.at("gui_initCovarianceVelocityUnit").get_to(data.gui_initCovarianceVelocityUnit); }
    if (j.contains("gui_initCovarianceVelocity")) { j.at("gui_initCovarianceVelocity").get_to(data.gui_initCovarianceVelocity); }
    if (j.contains("_initCovarianceVelocity")) { j.at("_initCovarianceVelocity").get_to(data._initCovarianceVelocity); }

    if (j.contains("gui_initCovarianceRecvClkErrUnit")) { j.at("gui_initCovarianceRecvClkErrUnit").get_to(data.gui_initCovarianceRecvClkErrUnit); }
    if (j.contains("gui_initCovarianceRecvClkErr")) { j.at("gui_initCovarianceRecvClkErr").get_to(data.gui_initCovarianceRecvClkErr); }
    if (j.contains("_initCovarianceRecvClkErr")) { j.at("_initCovarianceRecvClkErr").get_to(data._initCovarianceRecvClkErr); }

    if (j.contains("gui_initCovarianceRecvClkDriftUnit")) { j.at("gui_initCovarianceRecvClkDriftUnit").get_to(data.gui_initCovarianceRecvClkDriftUnit); }
    if (j.contains("gui_initCovarianceRecvClkDrift")) { j.at("gui_initCovarianceRecvClkDrift").get_to(data.gui_initCovarianceRecvClkDrift); }
    if (j.contains("_initCovarianceRecvClkDrift")) { j.at("_initCovarianceRecvClkDrift").get_to(data._initCovarianceRecvClkDrift); }

    if (j.contains("gui_initCovarianceInterSysErrUnit")) { j.at("gui_initCovarianceInterSysErrUnit").get_to(data.gui_initCovarianceInterSysErrUnit); }
    if (j.contains("gui_initCovarianceInterSysErr")) { j.at("gui_initCovarianceInterSysErr").get_to(data.gui_initCovarianceInterSysErr); }
    if (j.contains("_initCovarianceInterSysErr")) { j.at("_initCovarianceInterSysErr").get_to(data._initCovarianceInterSysErr); }

    if (j.contains("gui_initCovarianceInterSysDriftUnit")) { j.at("gui_initCovarianceInterSysDriftUnit").get_to(data.gui_initCovarianceInterSysDriftUnit); }
    if (j.contains("gui_initCovarianceInterSysDrift")) { j.at("gui_initCovarianceInterSysDrift").get_to(data.gui_initCovarianceInterSysDrift); }
    if (j.contains("_initCovarianceInterSysDrift")) { j.at("_initCovarianceInterSysDrift").get_to(data._initCovarianceInterSysDrift); }
}

} // namespace NAV::GNSS::Positioning::SPP
