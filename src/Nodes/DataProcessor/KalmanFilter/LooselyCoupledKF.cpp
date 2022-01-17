#include "LooselyCoupledKF.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>

#include <imgui_internal.h>
#include "internal/gui/widgets/imgui_ex.hpp"
#include "internal/gui/widgets/InputWithUnit.hpp"

#include "NodeData/State/PVAError.hpp"

#include "internal/FlowManager.hpp"
#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "Navigation/Constants.hpp"
#include "Navigation/Ellipsoid/Ellipsoid.hpp"
#include "Navigation/INS/Functions.hpp"
#include "Navigation/INS/ErrorEquations/LocalNavFrame/ErrorEquations.hpp"
#include "Navigation/Math/Math.hpp"
#include "Navigation/Math/VanLoan.hpp"
#include "Navigation/Gravity/Gravity.hpp"
#include "util/Logger.hpp"

/// @brief Scale factor to convert the attitude error
constexpr double SCALE_FACTOR_ATTITUDE = 180. / M_PI;
/// @brief Scale factor to convert the latitude and longitude error
constexpr double SCALE_FACTOR_LAT_LON = NAV::InsConst::pseudometre;
/// @brief Scale factor to convert the acceleration error
constexpr double SCALE_FACTOR_ACCELERATION = 1e3 / NAV::InsConst::G_NORM;
/// @brief Scale factor to convert the angular rate error
constexpr double SCALE_FACTOR_ANGULAR_RATE = 1e3;

NAV::LooselyCoupledKF::LooselyCoupledKF()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 822, 556 };

    _kalmanFilter_Kz = Eigen::MatrixXd::Zero(15, 1);

    nm::CreateInputPin(this, "InertialNavSol", Pin::Type::Flow, { NAV::InertialNavSol::type() }, &LooselyCoupledKF::recvInertialNavigationSolution);
    nm::CreateInputPin(this, "GNSSNavigationSolution", Pin::Type::Flow, { NAV::PosVelAtt::type() }, &LooselyCoupledKF::recvGNSSNavigationSolution);
    nm::CreateOutputPin(this, "PVAError", Pin::Type::Flow, { NAV::PVAError::type() });
    nm::CreateOutputPin(this, "ImuBiases", Pin::Type::Flow, { NAV::ImuBiases::type() });
    nm::CreateOutputPin(this, "x", Pin::Type::Matrix, { "Eigen::MatrixXd" }, &_kalmanFilter.x);
    nm::CreateOutputPin(this, "P", Pin::Type::Matrix, { "Eigen::MatrixXd" }, &_kalmanFilter.P);
    nm::CreateOutputPin(this, "Phi", Pin::Type::Matrix, { "Eigen::MatrixXd" }, &_kalmanFilter.Phi);
    nm::CreateOutputPin(this, "Q", Pin::Type::Matrix, { "Eigen::MatrixXd" }, &_kalmanFilter.Q);
    nm::CreateOutputPin(this, "z", Pin::Type::Matrix, { "Eigen::MatrixXd" }, &_kalmanFilter.z);
    nm::CreateOutputPin(this, "H", Pin::Type::Matrix, { "Eigen::MatrixXd" }, &_kalmanFilter.H);
    nm::CreateOutputPin(this, "R", Pin::Type::Matrix, { "Eigen::MatrixXd" }, &_kalmanFilter.R);
    nm::CreateOutputPin(this, "K", Pin::Type::Matrix, { "Eigen::MatrixXd" }, &_kalmanFilter.K);
    nm::CreateOutputPin(this, "K*z", Pin::Type::Matrix, { "Eigen::MatrixXd" }, &_kalmanFilter_Kz);
}

NAV::LooselyCoupledKF::~LooselyCoupledKF()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::LooselyCoupledKF::typeStatic()
{
    return "LooselyCoupledKF";
}

std::string NAV::LooselyCoupledKF::type() const
{
    return typeStatic();
}

std::string NAV::LooselyCoupledKF::category()
{
    return "Data Processor";
}

void NAV::LooselyCoupledKF::guiConfig()
{
    constexpr float configWidth = 380.0F;
    constexpr float unitWidth = 150.0F;

    ImGui::SetNextItemWidth(configWidth + ImGui::GetStyle().ItemSpacing.x);
    if (ImGui::Combo(fmt::format("Phi calculation algorithm##{}", size_t(id)).c_str(), reinterpret_cast<int*>(&_phiCalculationAlgorithm), "Taylor 1st Order\0Van Loan\0\0"))
    {
        LOG_DEBUG("{}: Phi calculation algorithm changed to {}", nameId(), _phiCalculationAlgorithm);

        if (_phiCalculationAlgorithm != PhiCalculationAlgorithm::VanLoan)
        {
            _qCalculationAlgorithm = QCalculationAlgorithm::Groves;
        }

        flow::ApplyChanges();
    }
    if (_phiCalculationAlgorithm != PhiCalculationAlgorithm::VanLoan)
    {
        ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
        ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5F);
    }

    ImGui::SetNextItemWidth(configWidth + ImGui::GetStyle().ItemSpacing.x);
    if (ImGui::Combo(fmt::format("Q calculation algorithm##{}", size_t(id)).c_str(), reinterpret_cast<int*>(&_qCalculationAlgorithm), "Groves\0Van Loan\0\0"))
    {
        LOG_DEBUG("{}: Q calculation algorithm changed to {}", nameId(), _qCalculationAlgorithm);
        flow::ApplyChanges();
    }

    if (_phiCalculationAlgorithm != PhiCalculationAlgorithm::VanLoan)
    {
        ImGui::PopItemFlag();
        ImGui::PopStyleVar();
    }

    ImGui::Separator();

    // ###########################################################################################################
    //                                Q - System/Process noise covariance matrix
    // ###########################################################################################################

    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    if (ImGui::TreeNode(fmt::format("Q - System/Process noise covariance matrix##{}", size_t(id)).c_str()))
    {
        // --------------------------------------------- Accelerometer -----------------------------------------------

        ImGui::SetNextItemWidth(configWidth + ImGui::GetStyle().ItemSpacing.x);
        if (ImGui::Combo(fmt::format("Random Process Accelerometer##{}", size_t(id)).c_str(), reinterpret_cast<int*>(&_randomProcessAccel), "White Noise\0"
                                                                                                                                            "Random Constant\0"
                                                                                                                                            "Random Walk\0"
                                                                                                                                            "Gauss-Markov 1st Order\0"
                                                                                                                                            "Gauss-Markov 2nd Order\0"
                                                                                                                                            "Gauss-Markov 3rd Order\0\0"))
        {
            if (_randomProcessAccel != RandomProcess::RandomWalk && _randomProcessAccel != RandomProcess::GaussMarkov1)
            {
                LOG_ERROR("Currently only 'Random Walk' and 'Gauss-Markov 1st Order' is supported");
                _randomProcessAccel = RandomProcess::RandomWalk;
            }

            LOG_DEBUG("{}: randomProcessAccel changed to {}", nameId(), _randomProcessAccel);
            flow::ApplyChanges();
        }

        if (_randomProcessAccel == RandomProcess::GaussMarkov1)
        {
            ImGui::SetNextItemWidth(configWidth + ImGui::GetStyle().ItemSpacing.x);
            if (ImGui::InputDouble3(fmt::format("Gauss-Markov β Accelerometer##{}", size_t(id)).c_str(), _beta_accel.data(), "%.4e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: beta_accel changed to {}", nameId(), _beta_accel.transpose());
                flow::ApplyChanges();
            }
        }

        if (gui::widgets::InputDouble3WithUnit(fmt::format("{} of the noise on the\naccelerometer specific-force measurements##{}", "Standard deviation", size_t(id)).c_str(),
                                               configWidth, unitWidth, _stdev_ra.data(), reinterpret_cast<int*>(&_stdevAccelNoiseUnits), "mg/√(Hz)\0m/s^2/√(Hz)\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: variance_ra changed to {}", nameId(), _stdev_ra.transpose());
            LOG_DEBUG("{}: varianceAccelNoiseUnits changed to {}", nameId(), _stdevAccelNoiseUnits);
            flow::ApplyChanges();
        }

        if (_qCalculationAlgorithm == QCalculationAlgorithm::Groves)
        {
            if (gui::widgets::InputDoubleWithUnit(fmt::format("{} of the accelerometer dynamic bias##{}", "Standard deviation", size_t(id)).c_str(),
                                                  configWidth, unitWidth, &_variance_bad, reinterpret_cast<int*>(&_varianceAccelBiasUnits), "µg\0\0",
                                                  0.0, 0.0, "%.4e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: variance_bad changed to {}", nameId(), _variance_bad);
                LOG_DEBUG("{}: varianceAccelBiasUnits changed to {}", nameId(), _varianceAccelBiasUnits);
                flow::ApplyChanges();
            }
        }

        // ----------------------------------------------- Gyroscope -------------------------------------------------

        ImGui::SetNextItemWidth(configWidth + ImGui::GetStyle().ItemSpacing.x);
        if (ImGui::Combo(fmt::format("Random Process Gyroscope##{}", size_t(id)).c_str(), reinterpret_cast<int*>(&_randomProcessGyro), "White Noise\0"
                                                                                                                                       "Random Constant\0"
                                                                                                                                       "Random Walk\0"
                                                                                                                                       "Gauss-Markov 1st Order\0"
                                                                                                                                       "Gauss-Markov 2nd Order\0"
                                                                                                                                       "Gauss-Markov 3rd Order\0\0"))
        {
            // TODO: Implement different Random processes
            if (_randomProcessGyro != RandomProcess::RandomWalk && _randomProcessGyro != RandomProcess::GaussMarkov1)
            {
                LOG_ERROR("Currently only 'Random Walk' and 'Gauss-Markov 1st Order' is supported");
                _randomProcessGyro = RandomProcess::RandomWalk;
            }

            LOG_DEBUG("{}: randomProcessGyro changed to {}", nameId(), _randomProcessGyro);
            flow::ApplyChanges();
        }
        if (_randomProcessGyro == RandomProcess::GaussMarkov1)
        {
            ImGui::SetNextItemWidth(configWidth + ImGui::GetStyle().ItemSpacing.x);
            if (ImGui::InputDouble3(fmt::format("Gauss-Markov β Gyroscope##{}", size_t(id)).c_str(), _beta_gyro.data(), "%.4e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: beta_gyro changed to {}", nameId(), _beta_gyro.transpose());
                flow::ApplyChanges();
            }
        }

        if (gui::widgets::InputDouble3WithUnit(fmt::format("{} of the noise on\nthe gyro angular-rate measurements##{}",
                                                           _stdevGyroNoiseUnits == StdevGyroNoiseUnits::deg_hr_sqrtHz ? "Standard deviation" : "Variance", size_t(id))
                                                   .c_str(),
                                               configWidth, unitWidth, _stdev_rg.data(), reinterpret_cast<int*>(&_stdevGyroNoiseUnits), "deg/hr/√(Hz)\0rad/s/√(Hz)\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: variance_rg changed to {}", nameId(), _stdev_rg.transpose());
            LOG_DEBUG("{}: varianceGyroNoiseUnits changed to {}", nameId(), _stdevGyroNoiseUnits);
            flow::ApplyChanges();
        }

        if (_qCalculationAlgorithm == QCalculationAlgorithm::Groves)
        {
            if (gui::widgets::InputDoubleWithUnit(fmt::format("{} of the gyro dynamic bias##{}",
                                                              _varianceGyroBiasUnits == VarianceGyroBiasUnits::deg_h ? "Standard deviation" : "Variance", size_t(id))
                                                      .c_str(),
                                                  configWidth, unitWidth, &_variance_bgd, reinterpret_cast<int*>(&_varianceGyroBiasUnits), "°/h\0\0",
                                                  0.0, 0.0, "%.4e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: variance_bgd changed to {}", nameId(), _variance_bgd);
                LOG_DEBUG("{}: varianceGyroBiasUnits changed to {}", nameId(), _varianceGyroBiasUnits);
                flow::ApplyChanges();
            }
        }

        ImGui::TreePop();
    }

    // ###########################################################################################################
    //                                        Measurement Uncertainties 𝐑
    // ###########################################################################################################

    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    if (ImGui::TreeNode(fmt::format("R - Measurement noise covariance matrix##{}", size_t(id)).c_str()))
    {
        if (gui::widgets::InputDouble3WithUnit(fmt::format("{} of the GNSS position measurements##{}",
                                                           _gnssMeasurementUncertaintyPositionUnit == GnssMeasurementUncertaintyPositionUnit::rad2_rad2_m2
                                                                   || _gnssMeasurementUncertaintyPositionUnit == GnssMeasurementUncertaintyPositionUnit::meter2
                                                               ? "Variance"
                                                               : "Standard deviation",
                                                           size_t(id))
                                                   .c_str(),
                                               configWidth, unitWidth, _gnssMeasurementUncertaintyPosition.data(), reinterpret_cast<int*>(&_gnssMeasurementUncertaintyPositionUnit), "rad^2, rad^2, m^2\0"
                                                                                                                                                                                     "rad, rad, m\0"
                                                                                                                                                                                     "m^2, m^2, m^2\0"
                                                                                                                                                                                     "m, m, m\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: gnssMeasurementUncertaintyPosition changed to {}", nameId(), _gnssMeasurementUncertaintyPosition.transpose());
            LOG_DEBUG("{}: gnssMeasurementUncertaintyPositionUnit changed to {}", nameId(), _gnssMeasurementUncertaintyPositionUnit);
            flow::ApplyChanges();
        }

        if (gui::widgets::InputDouble3WithUnit(fmt::format("{} of the GNSS velocity measurements##{}", _gnssMeasurementUncertaintyVelocityUnit == GnssMeasurementUncertaintyVelocityUnit::m2_s2 ? "Variance" : "Standard deviation",
                                                           size_t(id))
                                                   .c_str(),
                                               configWidth, unitWidth, _gnssMeasurementUncertaintyVelocity.data(), reinterpret_cast<int*>(&_gnssMeasurementUncertaintyVelocityUnit), "m^2/s^2\0"
                                                                                                                                                                                     "m/s\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: gnssMeasurementUncertaintyVelocity changed to {}", nameId(), _gnssMeasurementUncertaintyVelocity);
            LOG_DEBUG("{}: gnssMeasurementUncertaintyVelocityUnit changed to {}", nameId(), _gnssMeasurementUncertaintyVelocityUnit);
            flow::ApplyChanges();
        }

        ImGui::TreePop();
    }

    // ###########################################################################################################
    //                                        𝐏 Error covariance matrix
    // ###########################################################################################################

    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    if (ImGui::TreeNode(fmt::format("P Error covariance matrix (init)##{}", size_t(id)).c_str()))
    {
        if (gui::widgets::InputDouble3WithUnit(fmt::format("Position covariance ({})##{}",
                                                           _initCovariancePositionUnit == InitCovariancePositionUnit::rad2_rad2_m2
                                                                   || _initCovariancePositionUnit == InitCovariancePositionUnit::meter2
                                                               ? "Variance σ²"
                                                               : "Standard deviation σ",
                                                           size_t(id))
                                                   .c_str(),
                                               configWidth, unitWidth, _initCovariancePosition.data(), reinterpret_cast<int*>(&_initCovariancePositionUnit), "rad^2, rad^2, m^2\0"
                                                                                                                                                             "rad, rad, m\0"
                                                                                                                                                             "m^2, m^2, m^2\0"
                                                                                                                                                             "m, m, m\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: initCovariancePosition changed to {}", nameId(), _initCovariancePosition);
            LOG_DEBUG("{}: initCovariancePositionUnit changed to {}", nameId(), _initCovariancePositionUnit);
            flow::ApplyChanges();
        }

        if (gui::widgets::InputDouble3WithUnit(fmt::format("Velocity covariance ({})##{}",
                                                           _initCovarianceVelocityUnit == InitCovarianceVelocityUnit::m2_s2
                                                               ? "Variance σ²"
                                                               : "Standard deviation σ",
                                                           size_t(id))
                                                   .c_str(),
                                               configWidth, unitWidth, _initCovarianceVelocity.data(), reinterpret_cast<int*>(&_initCovarianceVelocityUnit), "m^2/s^2\0"
                                                                                                                                                             "m/s\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: initCovarianceVelocity changed to {}", nameId(), _initCovarianceVelocity);
            LOG_DEBUG("{}: initCovarianceVelocityUnit changed to {}", nameId(), _initCovarianceVelocityUnit);
            flow::ApplyChanges();
        }

        if (gui::widgets::InputDouble3WithUnit(fmt::format("Flight Angles covariance ({})##{}",
                                                           _initCovarianceAttitudeAnglesUnit == InitCovarianceAttitudeAnglesUnit::rad2
                                                                   || _initCovarianceAttitudeAnglesUnit == InitCovarianceAttitudeAnglesUnit::deg2
                                                               ? "Variance σ²"
                                                               : "Standard deviation σ",
                                                           size_t(id))
                                                   .c_str(),
                                               configWidth, unitWidth, _initCovarianceAttitudeAngles.data(), reinterpret_cast<int*>(&_initCovarianceAttitudeAnglesUnit), "rad^2\0"
                                                                                                                                                                         "deg^2\0"
                                                                                                                                                                         "rad\0"
                                                                                                                                                                         "deg\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: initCovarianceAttitudeAngles changed to {}", nameId(), _initCovarianceAttitudeAngles);
            LOG_DEBUG("{}: initCovarianceAttitudeAnglesUnit changed to {}", nameId(), _initCovarianceAttitudeAnglesUnit);
            flow::ApplyChanges();
        }

        if (gui::widgets::InputDouble3WithUnit(fmt::format("Accelerometer Bias covariance ({})##{}",
                                                           _initCovarianceBiasAccelUnit == InitCovarianceBiasAccelUnit::m2_s4
                                                               ? "Variance σ²"
                                                               : "Standard deviation σ",
                                                           size_t(id))
                                                   .c_str(),
                                               configWidth, unitWidth, _initCovarianceBiasAccel.data(), reinterpret_cast<int*>(&_initCovarianceBiasAccelUnit), "m^2/s^4\0"
                                                                                                                                                               "m/s^2\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: initCovarianceBiasAccel changed to {}", nameId(), _initCovarianceBiasAccel);
            LOG_DEBUG("{}: initCovarianceBiasAccelUnit changed to {}", nameId(), _initCovarianceBiasAccelUnit);
            flow::ApplyChanges();
        }

        if (gui::widgets::InputDouble3WithUnit(fmt::format("Gyroscope Bias covariance ({})##{}",
                                                           _initCovarianceBiasGyroUnit == InitCovarianceBiasGyroUnit::rad2_s2
                                                                   || _initCovarianceBiasGyroUnit == InitCovarianceBiasGyroUnit::deg2_s2
                                                               ? "Variance σ²"
                                                               : "Standard deviation σ",
                                                           size_t(id))
                                                   .c_str(),
                                               configWidth, unitWidth, _initCovarianceBiasGyro.data(), reinterpret_cast<int*>(&_initCovarianceBiasGyroUnit), "rad^2/s^2\0"
                                                                                                                                                             "deg^2/s^2\0"
                                                                                                                                                             "rad/s\0"
                                                                                                                                                             "deg/s\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: initCovarianceBiasGyro changed to {}", nameId(), _initCovarianceBiasGyro);
            LOG_DEBUG("{}: initCovarianceBiasGyroUnit changed to {}", nameId(), _initCovarianceBiasGyroUnit);
            flow::ApplyChanges();
        }

        ImGui::TreePop();
    }
}

[[nodiscard]] json NAV::LooselyCoupledKF::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["phiCalculation"] = _phiCalculationAlgorithm;
    j["qCalculation"] = _qCalculationAlgorithm;

    j["randomProcessAccel"] = _randomProcessAccel;
    j["beta_accel"] = _beta_accel;
    j["randomProcessGyro"] = _randomProcessGyro;
    j["beta_gyro"] = _beta_gyro;
    j["stdev_ra"] = _stdev_ra;
    j["stdevAccelNoiseUnits"] = _stdevAccelNoiseUnits;
    j["stdev_rg"] = _stdev_rg;
    j["stdevGyroNoiseUnits"] = _stdevGyroNoiseUnits;
    j["variance_bad"] = _variance_bad;
    j["varianceAccelBiasUnits"] = _varianceAccelBiasUnits;
    j["variance_bgd"] = _variance_bgd;
    j["varianceGyroBiasUnits"] = _varianceGyroBiasUnits;

    j["gnssMeasurementUncertaintyPositionUnit"] = _gnssMeasurementUncertaintyPositionUnit;
    j["gnssMeasurementUncertaintyPosition"] = _gnssMeasurementUncertaintyPosition;
    j["gnssMeasurementUncertaintyVelocityUnit"] = _gnssMeasurementUncertaintyVelocityUnit;
    j["gnssMeasurementUncertaintyVelocity"] = _gnssMeasurementUncertaintyVelocity;

    j["initCovariancePositionUnit"] = _initCovariancePositionUnit;
    j["initCovariancePosition"] = _initCovariancePosition;
    j["initCovarianceVelocityUnit"] = _initCovarianceVelocityUnit;
    j["initCovarianceVelocity"] = _initCovarianceVelocity;
    j["initCovarianceAttitudeAnglesUnit"] = _initCovarianceAttitudeAnglesUnit;
    j["initCovarianceAttitudeAngles"] = _initCovarianceAttitudeAngles;
    j["initCovarianceBiasAccelUnit"] = _initCovarianceBiasAccelUnit;
    j["initCovarianceBiasAccel"] = _initCovarianceBiasAccel;
    j["initCovarianceBiasGyroUnit"] = _initCovarianceBiasGyroUnit;
    j["initCovarianceBiasGyro"] = _initCovarianceBiasGyro;

    return j;
}

void NAV::LooselyCoupledKF::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());
    if (j.contains("phiCalculation"))
    {
        j.at("phiCalculation").get_to(_phiCalculationAlgorithm);
    }
    if (j.contains("qCalculation"))
    {
        j.at("qCalculation").get_to(_qCalculationAlgorithm);
    }
    // ------------------------------- 𝐐 System/Process noise covariance matrix ---------------------------------
    if (j.contains("randomProcessAccel"))
    {
        j.at("randomProcessAccel").get_to(_randomProcessAccel);
    }
    if (j.contains("beta_accel"))
    {
        _beta_accel = j.at("beta_accel");
    }
    if (j.contains("randomProcessGyro"))
    {
        j.at("randomProcessGyro").get_to(_randomProcessGyro);
    }
    if (j.contains("beta_gyro"))
    {
        _beta_gyro = j.at("beta_gyro");
    }
    if (j.contains("stdev_ra"))
    {
        _stdev_ra = j.at("stdev_ra");
    }
    if (j.contains("stdevAccelNoiseUnits"))
    {
        j.at("stdevAccelNoiseUnits").get_to(_stdevAccelNoiseUnits);
    }
    if (j.contains("stdev_rg"))
    {
        _stdev_rg = j.at("stdev_rg");
    }
    if (j.contains("stdevGyroNoiseUnits"))
    {
        j.at("stdevGyroNoiseUnits").get_to(_stdevGyroNoiseUnits);
    }
    if (j.contains("variance_bad"))
    {
        _variance_bad = j.at("variance_bad");
    }
    if (j.contains("varianceAccelBiasUnits"))
    {
        j.at("varianceAccelBiasUnits").get_to(_varianceAccelBiasUnits);
    }
    if (j.contains("variance_bgd"))
    {
        _variance_bgd = j.at("variance_bgd");
    }
    if (j.contains("varianceGyroBiasUnits"))
    {
        j.at("varianceGyroBiasUnits").get_to(_varianceGyroBiasUnits);
    }
    // -------------------------------- 𝐑 Measurement noise covariance matrix -----------------------------------
    if (j.contains("gnssMeasurementUncertaintyPositionUnit"))
    {
        j.at("gnssMeasurementUncertaintyPositionUnit").get_to(_gnssMeasurementUncertaintyPositionUnit);
    }
    if (j.contains("gnssMeasurementUncertaintyPosition"))
    {
        _gnssMeasurementUncertaintyPosition = j.at("gnssMeasurementUncertaintyPosition");
    }
    if (j.contains("gnssMeasurementUncertaintyVelocityUnit"))
    {
        j.at("gnssMeasurementUncertaintyVelocityUnit").get_to(_gnssMeasurementUncertaintyVelocityUnit);
    }
    if (j.contains("gnssMeasurementUncertaintyVelocity"))
    {
        _gnssMeasurementUncertaintyVelocity = j.at("gnssMeasurementUncertaintyVelocity");
    }
    // -------------------------------------- 𝐏 Error covariance matrix -----------------------------------------
    if (j.contains("initCovariancePositionUnit"))
    {
        j.at("initCovariancePositionUnit").get_to(_initCovariancePositionUnit);
    }
    if (j.contains("initCovariancePosition"))
    {
        _initCovariancePosition = j.at("initCovariancePosition");
    }
    if (j.contains("initCovarianceVelocityUnit"))
    {
        j.at("initCovarianceVelocityUnit").get_to(_initCovarianceVelocityUnit);
    }
    if (j.contains("initCovarianceVelocity"))
    {
        _initCovarianceVelocity = j.at("initCovarianceVelocity");
    }
    if (j.contains("initCovarianceAttitudeAnglesUnit"))
    {
        j.at("initCovarianceAttitudeAnglesUnit").get_to(_initCovarianceAttitudeAnglesUnit);
    }
    if (j.contains("initCovarianceAttitudeAngles"))
    {
        _initCovarianceAttitudeAngles = j.at("initCovarianceAttitudeAngles");
    }
    if (j.contains("initCovarianceBiasAccelUnit"))
    {
        j.at("initCovarianceBiasAccelUnit").get_to(_initCovarianceBiasAccelUnit);
    }
    if (j.contains("initCovarianceBiasAccel"))
    {
        _initCovarianceBiasAccel = j.at("initCovarianceBiasAccel");
    }
    if (j.contains("initCovarianceBiasGyroUnit"))
    {
        j.at("initCovarianceBiasGyroUnit").get_to(_initCovarianceBiasGyroUnit);
    }
    if (j.contains("initCovarianceBiasGyro"))
    {
        _initCovarianceBiasGyro = j.at("initCovarianceBiasGyro");
    }
}

bool NAV::LooselyCoupledKF::initialize()
{
    LOG_TRACE("{}: called", nameId());

    _kalmanFilter = KalmanFilter{ 15, 6 };

    _kalmanFilter_Kz = Eigen::MatrixXd::Zero(15, 1);

    _latestInertialNavSol = nullptr;
    _accumulatedImuBiases.biasAccel_b.setZero();
    _accumulatedImuBiases.biasGyro_b.setZero();

    // Initial Covariance of the attitude angles in [rad²]
    Eigen::Vector3d variance_angles = Eigen::Vector3d::Zero();
    if (_initCovarianceAttitudeAnglesUnit == InitCovarianceAttitudeAnglesUnit::rad2)
    {
        variance_angles = _initCovarianceAttitudeAngles;
    }
    else if (_initCovarianceAttitudeAnglesUnit == InitCovarianceAttitudeAnglesUnit::deg2)
    {
        variance_angles = trafo::deg2rad(_initCovarianceAttitudeAngles);
    }
    else if (_initCovarianceAttitudeAnglesUnit == InitCovarianceAttitudeAnglesUnit::rad)
    {
        variance_angles = _initCovarianceAttitudeAngles.array().pow(2);
    }
    else if (_initCovarianceAttitudeAnglesUnit == InitCovarianceAttitudeAnglesUnit::deg)
    {
        variance_angles = trafo::deg2rad(_initCovarianceAttitudeAngles).array().pow(2);
    }

    // Initial Covariance of the velocity in [m²/s²]
    Eigen::Vector3d variance_vel = Eigen::Vector3d::Zero();
    if (_initCovarianceVelocityUnit == InitCovarianceVelocityUnit::m2_s2)
    {
        variance_vel = _initCovarianceVelocity;
    }
    else if (_initCovarianceVelocityUnit == InitCovarianceVelocityUnit::m_s)
    {
        variance_vel = _initCovarianceVelocity.array().pow(2);
    }

    // Initial Covariance of the position in [rad² rad² m²]
    Eigen::Vector3d variance_lla = Eigen::Vector3d::Zero();
    if (_initCovariancePositionUnit == InitCovariancePositionUnit::rad2_rad2_m2)
    {
        variance_lla = _initCovariancePosition;
    }
    else if (_initCovariancePositionUnit == InitCovariancePositionUnit::rad_rad_m)
    {
        variance_lla = _initCovariancePosition.array().pow(2);
    }
    else if (_initCovariancePositionUnit == InitCovariancePositionUnit::meter)
    {
        variance_lla = (trafo::ecef2lla_WGS84(trafo::ned2ecef(_initCovariancePosition, { 0, 0, 0 }))).array().pow(2);
    }
    else if (_initCovariancePositionUnit == InitCovariancePositionUnit::meter2)
    {
        variance_lla = (trafo::ecef2lla_WGS84(trafo::ned2ecef(_initCovariancePosition.cwiseSqrt(), { 0, 0, 0 }))).array().pow(2);
    }

    // Initial Covariance of the accelerometer biases in [m^2/s^4]
    Eigen::Vector3d variance_accelBias = Eigen::Vector3d::Zero();
    if (_initCovarianceBiasAccelUnit == InitCovarianceBiasAccelUnit::m2_s4)
    {
        variance_accelBias = _initCovarianceBiasAccel;
    }
    else if (_initCovarianceBiasAccelUnit == InitCovarianceBiasAccelUnit::m_s2)
    {
        variance_accelBias = _initCovarianceBiasAccel.array().pow(2);
    }

    // Initial Covariance of the gyroscope biases in [rad^2/s^2]
    Eigen::Vector3d variance_gyroBias = Eigen::Vector3d::Zero();
    if (_initCovarianceBiasGyroUnit == InitCovarianceBiasGyroUnit::rad2_s2)
    {
        variance_gyroBias = _initCovarianceBiasGyro;
    }
    else if (_initCovarianceBiasGyroUnit == InitCovarianceBiasGyroUnit::deg2_s2)
    {
        variance_gyroBias = trafo::deg2rad(_initCovarianceBiasGyro.array().sqrt()).array().pow(2);
    }
    else if (_initCovarianceBiasGyroUnit == InitCovarianceBiasGyroUnit::rad_s)
    {
        variance_gyroBias = _initCovarianceBiasGyro.array().pow(2);
    }
    else if (_initCovarianceBiasGyroUnit == InitCovarianceBiasGyroUnit::deg_s)
    {
        variance_gyroBias = trafo::deg2rad(_initCovarianceBiasGyro).array().pow(2);
    }

    // 𝐏 Error covariance matrix
    _kalmanFilter.P = initialErrorCovarianceMatrixP0(variance_angles,    // Flight Angles covariance
                                                     variance_vel,       // Velocity covariance
                                                     variance_lla,       // Position (Lat, Lon, Alt) covariance
                                                     variance_accelBias, // Accelerometer Bias covariance
                                                     variance_gyroBias); // Gyroscope Bias covariance

    LOG_DEBUG("{}: initialized", nameId());
    LOG_DATA("{}: P_0 =\n{}", nameId(), _kalmanFilter.P);

    return true;
}

void NAV::LooselyCoupledKF::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

void NAV::LooselyCoupledKF::recvInertialNavigationSolution(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::LinkId /*linkId*/) // NOLINT(readability-convert-member-functions-to-static)
{
    auto inertialNavSol = std::static_pointer_cast<const InertialNavSol>(nodeData);

    if (_latestInertialNavSol)
    {
        _tau_KF = static_cast<double>((inertialNavSol->insTime.value() - _latestInertialNavSol->insTime.value()).count());
    }

    _latestInertialNavSol = inertialNavSol;

    looselyCoupledPrediction(inertialNavSol);
}

void NAV::LooselyCoupledKF::recvGNSSNavigationSolution(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::LinkId /*linkId*/)
{
    auto gnssMeasurement = std::static_pointer_cast<const PosVelAtt>(nodeData);

    if (_latestInertialNavSol)
    {
        looselyCoupledUpdate(gnssMeasurement);
    }
}

// ###########################################################################################################
//                                               Kalman Filter
// ###########################################################################################################

void NAV::LooselyCoupledKF::looselyCoupledPrediction(const std::shared_ptr<const InertialNavSol>& inertialNavSol)
{
    // ------------------------------------------- Data preparation ----------------------------------------------
    // v_n (tₖ₋₁) Velocity in [m/s], in navigation coordinates, at the time tₖ₋₁
    const Eigen::Vector3d& velocity_n__t1 = inertialNavSol->velocity_n();
    LOG_DATA("{}: velocity_n__t1 = {} [m / s]", nameId(), velocity_n__t1.transpose());
    // Latitude 𝜙, longitude λ and altitude (height above ground) in [rad, rad, m] at the time tₖ₋₁
    const Eigen::Vector3d& position_lla__t1 = inertialNavSol->latLonAlt();
    LOG_DATA("{}: position_lla__t1 = {} [rad, rad, m]", nameId(), position_lla__t1.transpose());
    // q (tₖ₋₁) Quaternion, from body to navigation coordinates, at the time tₖ₋₁
    const Eigen::Quaterniond& quaternion_nb__t1 = inertialNavSol->quaternion_nb();
    LOG_DATA("{}: quaternion_nb__t1 --> Roll, Pitch, Yaw = {} [deg]", nameId(), trafo::deg2rad(trafo::quat2eulerZYX(quaternion_nb__t1).transpose()));

    // Prime vertical radius of curvature (East/West) [m]
    const double R_E = calcEarthRadius_E(position_lla__t1(0));
    LOG_DATA("{}: R_E = {} [m]", nameId(), R_E);
    // Meridian radius of curvature in [m]
    const double R_N = calcEarthRadius_N(position_lla__t1(0));
    LOG_DATA("{}: R_N = {} [m]", nameId(), R_N);

    // Direction Cosine Matrix from body to navigation coordinates, at the time tₖ₋₁
    Eigen::Matrix3d DCM_nb = quaternion_nb__t1.toRotationMatrix();
    LOG_DATA("{}: DCM_nb =\n{}", nameId(), DCM_nb);

    // Conversion matrix between cartesian and curvilinear perturbations to the position
    Eigen::Matrix3d T_rn_p = conversionMatrixCartesianCurvilinear(position_lla__t1, R_N, R_E);
    LOG_DATA("{}: T_rn_p =\n{}", nameId(), T_rn_p);

    // a_p Acceleration in [m/s^2], in body coordinates
    const Eigen::Vector3d acceleration_b = inertialNavSol->imuObs->imuPos.quatAccel_bp() * inertialNavSol->imuObs->accelUncompXYZ.value()
                                           - _accumulatedImuBiases.biasAccel_b;
    LOG_DATA("{}: acceleration_b = {} [m/s^2]", nameId(), acceleration_b.transpose());

    // omega_in^n = omega_ie^n + omega_en^n
    Eigen::Vector3d angularRate_in_n = inertialNavSol->quaternion_ne() * InsConst::omega_ie_e
                                       + calcTransportRate_n(position_lla__t1, velocity_n__t1, R_N, R_E);
    LOG_DATA("{}: angularRate_in_n = {} [rad/s]", nameId(), angularRate_in_n.transpose());

    // Gauss-Markov constant for the accelerometer 𝛽 = 1 / 𝜏 (𝜏 correlation length) - Value from Jekeli (p. 183)
    Eigen::Vector3d beta_a = Eigen::Vector3d::Zero();
    if (_randomProcessAccel == RandomProcess::RandomWalk)
    {
        beta_a = Eigen::Vector3d::Zero();
    }
    else if (_randomProcessAccel == RandomProcess::GaussMarkov1)
    {
        beta_a = _beta_accel;
    }
    LOG_DATA("{}: beta_a = {} [1/s]", nameId(), beta_a.transpose());
    // Gauss-Markov constant for the gyroscope 𝛽 = 1 / 𝜏 (𝜏 correlation length) - Value from Jekeli (p. 183)
    Eigen::Vector3d beta_omega = Eigen::Vector3d::Zero();
    if (_randomProcessGyro == RandomProcess::RandomWalk)
    {
        beta_omega = Eigen::Vector3d::Zero();
    }
    else if (_randomProcessGyro == RandomProcess::GaussMarkov1)
    {
        beta_omega = _beta_gyro;
    }
    LOG_DATA("{}: beta_omega = {} [1/s]", nameId(), beta_omega.transpose());

    // ------------------------------------------- GUI Parameters ----------------------------------------------

    // 𝜎_ra Standard deviation of the noise on the accelerometer specific-force state [m / (s^2 · √(s))]
    Eigen::Vector3d sigma_ra = Eigen::Vector3d::Zero();
    switch (_stdevAccelNoiseUnits)
    {
    case StdevAccelNoiseUnits::mg_sqrtHz: // [mg / √(Hz)]
        sigma_ra = _stdev_ra * 1e-3;      // [g / √(Hz)]
        sigma_ra *= InsConst::G_NORM;     // [m / (s^2 · √(Hz))] = [m / (s · √(s))]
        // sigma_ra /= 1.;                // [m / (s^2 · √(s))]
        break;
    case StdevAccelNoiseUnits::m_s2_sqrtHz: // [m / (s^2 · √(Hz))] = [m / (s · √(s))]
        sigma_ra = _stdev_ra;
        // sigma_ra /= 1.;                  // [m / (s^2 · √(s))]
        break;
    }
    LOG_DATA("{}: sigma_ra = {} [m / (s^2 · √(s))]", nameId(), sigma_ra.transpose());

    // 𝜎_rg Standard deviation of the noise on the gyro angular-rate state [rad / (s · √(s))]
    Eigen::Vector3d sigma_rg = Eigen::Vector3d::Zero();
    switch (_stdevGyroNoiseUnits)
    {
    case StdevGyroNoiseUnits::deg_hr_sqrtHz:  // [deg / hr / √(Hz)] (see Woodman (2007) Chp. 3.2.2 - eq. 7 with seconds instead of hours)
        sigma_rg = trafo::deg2rad(_stdev_rg); // [rad / hr / √(Hz)]
        sigma_rg /= 60.;                      // [rad / √(hr)]
        sigma_rg /= 60.;                      // [rad / √(s)]
        // sigma_rg /= 1.;                    // [rad / (s · √(s))]
        break;
    case StdevGyroNoiseUnits::rad_s_sqrtHz: // [rad / (s · √(Hz))] = [rad / √(s)]
        sigma_rg = _stdev_rg;
        // sigma_rg /= 1.;                  // [rad / (s · √(s))]
        break;
    }
    LOG_DATA("{}: sigma_rg = {} [rad / (s · √(s))]", nameId(), sigma_rg.transpose());

    // ---------------------------------------------- Prediction -------------------------------------------------

    // System Matrix
    Eigen::Matrix<double, 15, 15> F = systemMatrixF(quaternion_nb__t1, acceleration_b, angularRate_in_n, velocity_n__t1, position_lla__t1, beta_a, beta_omega, R_N, R_E);
    LOG_DATA("{}: F =\n{}", nameId(), F);

    if (_phiCalculationAlgorithm == PhiCalculationAlgorithm::VanLoan)
    {
        // Noise Input Matrix
        Eigen::Matrix<double, 15, 6> G = noiseInputMatrixG(sigma_ra.array().square(), sigma_rg.array().square(), beta_a, beta_omega);
        LOG_DATA("{}: G =\n{}", nameId(), G);

        auto [Phi, Q] = calcPhiAndQWithVanLoanMethod<double, 15, 6>(F, G, _tau_KF);

        // 1. Calculate the transition matrix 𝚽_{k-1}
        _kalmanFilter.Phi = Phi;

        // 2. Calculate the system noise covariance matrix Q_{k-1}
        if (_qCalculationAlgorithm == QCalculationAlgorithm::VanLoan)
        {
            _kalmanFilter.Q = Q;
        }
    }
    else if (_phiCalculationAlgorithm == PhiCalculationAlgorithm::Taylor1)
    {
        _kalmanFilter.Phi = KalmanFilter::transitionMatrix(F, _tau_KF);
    }
    else
    {
        LOG_CRITICAL("{}: Calculation algorithm '{}' for the system matrix Phi is not supported.", nameId(), _phiCalculationAlgorithm);
    }
    notifyOutputValueChanged(OUTPUT_PORT_INDEX_Phi);
    LOG_DATA("{}: KF.Phi =\n{}", nameId(), _kalmanFilter.Phi);

    // 2. Calculate the system noise covariance matrix Q_{k-1}
    if (_qCalculationAlgorithm == QCalculationAlgorithm::Groves)
    {
        // 𝜎²_bad Variance of the accelerometer dynamic bias
        double sigma2_bad{};
        switch (_varianceAccelBiasUnits)
        {
        case VarianceAccelBiasUnits::microg:   // [µg]
            sigma2_bad = _variance_bad * 1e-6; // [g]
            sigma2_bad *= InsConst::G_NORM;    // [m / s^2]
            sigma2_bad = std::pow(sigma2_bad, 2);
            break;
        }
        LOG_DATA("{}: sigma2_bad = {} [µg]", nameId(), sigma2_bad);
        // 𝜎²_bgd Variance of the gyro dynamic bias
        double sigma2_bgd{};
        switch (_varianceGyroBiasUnits)
        {
        case VarianceGyroBiasUnits::deg_h:           // [° / h]
            sigma2_bgd = _variance_bgd / 3600.0;     // [° / s];
            sigma2_bgd = trafo::deg2rad(sigma2_bgd); // [rad / s];
            sigma2_bgd = std::pow(sigma2_bgd, 2);
            break;
        }
        LOG_DATA("{}: sigma2_bgd = {} [° / h]", nameId(), sigma2_bgd);

        _kalmanFilter.Q = systemNoiseCovarianceMatrix(sigma_ra.array().square(), sigma_rg.array().square(), sigma2_bad, sigma2_bgd,
                                                      F.block<3, 3>(3, 0),
                                                      T_rn_p,
                                                      DCM_nb, _tau_KF);
    }
    notifyOutputValueChanged(OUTPUT_PORT_INDEX_Q);
    LOG_DATA("{}: KF.Q =\n{}", nameId(), _kalmanFilter.Q);

    // 3. Propagate the state vector estimate from x(+) and x(-)
    // 4. Propagate the error covariance matrix from P(+) and P(-)
    _kalmanFilter.predict();
    notifyOutputValueChanged(OUTPUT_PORT_INDEX_x);
    notifyOutputValueChanged(OUTPUT_PORT_INDEX_P);
    LOG_DATA("{}: KF.x =\n{}", nameId(), _kalmanFilter.x);
    LOG_DATA("{}: KF.P =\n{}", nameId(), _kalmanFilter.P);

    // Averaging of P to avoid numerical problems with symmetry (did not work)
    // _kalmanFilter.P = ((_kalmanFilter.P + _kalmanFilter.P.transpose()) / 2.0);

    // LOG_DEBUG("{}: F\n{}\n", nameId(), F);
    // LOG_DEBUG("{}: Phi\n{}\n", nameId(), _kalmanFilter.Phi);

    // LOG_DEBUG("{}: Q\n{}\n", nameId(), _kalmanFilter.Q);
    // LOG_DEBUG("{}: Q - Q^T\n{}\n", nameId(), _kalmanFilter.Q - _kalmanFilter.Q.transpose());

    // LOG_DEBUG("{}: x\n{}\n", nameId(), _kalmanFilter.x);

    // LOG_DEBUG("{}: P\n{}\n", nameId(), _kalmanFilter.P);
    // LOG_DEBUG("{}: P - P^T\n{}\n", nameId(), _kalmanFilter.P - _kalmanFilter.P.transpose());

    Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(_kalmanFilter.P);
    auto rank = lu_decomp.rank();
    // LOG_DEBUG("{}: P.rank = {}", nameId(), rank);
    if (rank != 15)
    {
        LOG_WARN("{}: P.rank = {}", nameId(), rank);
    }
}

void NAV::LooselyCoupledKF::looselyCoupledUpdate(const std::shared_ptr<const PosVelAtt>& gnssMeasurement)
{
    // ------------------------------------------- Data preparation ----------------------------------------------
    // Latitude 𝜙, longitude λ and altitude (height above ground) in [rad, rad, m] at the time tₖ₋₁
    const Eigen::Vector3d& position_lla__t1 = _latestInertialNavSol->latLonAlt();
    LOG_DATA("{}: position_lla__t1 = {} [rad, rad, m]", nameId(), position_lla__t1.transpose());

    // Prime vertical radius of curvature (East/West) [m]
    const double R_E = calcEarthRadius_E(position_lla__t1(0));
    LOG_DATA("{}: R_E = {} [m]", nameId(), R_E);
    // Meridian radius of curvature in [m]
    const double R_N = calcEarthRadius_N(position_lla__t1(0));
    LOG_DATA("{}: R_N = {} [m]", nameId(), R_N);

    // Direction Cosine Matrix from body to navigation coordinates, at the time tₖ₋₁
    Eigen::Matrix3d DCM_nb = _latestInertialNavSol->quaternion_nb().toRotationMatrix();
    LOG_DATA("{}: DCM_nb =\n{}", nameId(), DCM_nb);

    // Conversion matrix between cartesian and curvilinear perturbations to the position
    Eigen::Matrix3d T_rn_p = conversionMatrixCartesianCurvilinear(position_lla__t1, R_N, R_E);
    LOG_DATA("{}: T_rn_p =\n{}", nameId(), T_rn_p);

    // Angular rate measured in units of [rad/s], and given in the body frame
    const Eigen::Vector3d angularRate_b = _latestInertialNavSol->imuObs->imuPos.quatGyro_bp()
                                              * (_latestInertialNavSol->imuObs->gyroCompXYZ.has_value()
                                                     ? _latestInertialNavSol->imuObs->gyroCompXYZ.value()
                                                     : _latestInertialNavSol->imuObs->gyroUncompXYZ.value())
                                          - _accumulatedImuBiases.biasGyro_b;
    LOG_DATA("{}: angularRate_b = {} [rad/s]", nameId(), angularRate_b.transpose());

    // Skew-symmetric matrix of the Earth-rotation vector in local navigation frame axes
    Eigen::Matrix3d Omega_ie_n = skewSymmetricMatrix(_latestInertialNavSol->quaternion_ne() * InsConst::omega_ie_e);
    LOG_DATA("{}: Omega_ie_n =\n{}", nameId(), Omega_ie_n);

    // -------------------------------------------- GUI Parameters -----------------------------------------------

    // GNSS measurement uncertainty for the position (Variance σ²) in [rad^2, rad^2, m^2]
    Eigen::Vector3d gnssSigmaSquaredLatLonAlt = Eigen::Vector3d::Zero();
    switch (_gnssMeasurementUncertaintyPositionUnit)
    {
    case GnssMeasurementUncertaintyPositionUnit::meter:
        gnssSigmaSquaredLatLonAlt = (trafo::ecef2lla_WGS84(trafo::ned2ecef(_gnssMeasurementUncertaintyPosition, position_lla__t1)) - position_lla__t1).array().pow(2);
        break;
    case GnssMeasurementUncertaintyPositionUnit::meter2:
        gnssSigmaSquaredLatLonAlt = (trafo::ecef2lla_WGS84(trafo::ned2ecef(_gnssMeasurementUncertaintyPosition.cwiseSqrt(), position_lla__t1)) - position_lla__t1).array().pow(2);
        break;
    case GnssMeasurementUncertaintyPositionUnit::rad_rad_m:
        gnssSigmaSquaredLatLonAlt = _gnssMeasurementUncertaintyPosition.array().pow(2);
        break;
    case GnssMeasurementUncertaintyPositionUnit::rad2_rad2_m2:
        gnssSigmaSquaredLatLonAlt = _gnssMeasurementUncertaintyPosition;
        break;
    }

    // GNSS measurement uncertainty for the velocity (Variance σ²) in [m^2/s^2]
    Eigen::Vector3d gnssSigmaSquaredVelocity = Eigen::Vector3d::Zero();
    switch (_gnssMeasurementUncertaintyVelocityUnit)
    {
    case GnssMeasurementUncertaintyVelocityUnit::m_s:
        gnssSigmaSquaredVelocity = _gnssMeasurementUncertaintyVelocity.array().pow(2);
        break;
    case GnssMeasurementUncertaintyVelocityUnit::m2_s2:
        gnssSigmaSquaredVelocity = _gnssMeasurementUncertaintyVelocity;
        break;
    }

    // ---------------------------------------------- Correction -------------------------------------------------
    // 5. Calculate the measurement matrix H_k
    _kalmanFilter.H = measurementMatrix(T_rn_p, DCM_nb, angularRate_b, _leverArm_InsGnss_b, Omega_ie_n);
    notifyOutputValueChanged(OUTPUT_PORT_INDEX_H);
    LOG_DATA("{}: KF.H =\n{}", nameId(), _kalmanFilter.H);

    // 6. Calculate the measurement noise covariance matrix R_k
    _kalmanFilter.R = measurementNoiseCovariance(gnssSigmaSquaredLatLonAlt, gnssSigmaSquaredVelocity);
    notifyOutputValueChanged(OUTPUT_PORT_INDEX_R);
    LOG_DATA("{}: KF.R =\n{}", nameId(), _kalmanFilter.R);

    // 8. Formulate the measurement z_k
    _kalmanFilter.z = measurementInnovation(gnssMeasurement->latLonAlt(), _latestInertialNavSol->latLonAlt(),
                                            gnssMeasurement->velocity_n(), _latestInertialNavSol->velocity_n(),
                                            T_rn_p, _latestInertialNavSol->quaternion_nb(), _leverArm_InsGnss_b, angularRate_b, Omega_ie_n);
    notifyOutputValueChanged(OUTPUT_PORT_INDEX_z);
    LOG_DATA("{}: KF.z =\n{}", nameId(), _kalmanFilter.z);

    Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp3(_kalmanFilter.H * _kalmanFilter.P * _kalmanFilter.H.transpose() + _kalmanFilter.R);
    auto rank3 = lu_decomp3.rank();
    if (rank3 != 6)
    {
        LOG_WARN("{}: (HPH^T + R).rank = {}", nameId(), rank3);
    }

    // 7. Calculate the Kalman gain matrix K_k
    // 9. Update the state vector estimate from x(-) to x(+)
    // 10. Update the error covariance matrix from P(-) to P(+)
    _kalmanFilter.correctWithMeasurementInnovation();
    notifyOutputValueChanged(OUTPUT_PORT_INDEX_K);
    notifyOutputValueChanged(OUTPUT_PORT_INDEX_x);
    notifyOutputValueChanged(OUTPUT_PORT_INDEX_P);
    LOG_DATA("{}: KF.K =\n{}", nameId(), _kalmanFilter.K);
    LOG_DATA("{}: KF.x =\n{}", nameId(), _kalmanFilter.x);
    LOG_DATA("{}: KF.P =\n{}", nameId(), _kalmanFilter.P);

    _kalmanFilter_Kz = _kalmanFilter.K * _kalmanFilter.z;
    notifyOutputValueChanged(OUTPUT_PORT_INDEX_Kz);

    // Averaging of P to avoid numerical problems with symmetry (did not work)
    // _kalmanFilter.P = ((_kalmanFilter.P + _kalmanFilter.P.transpose()) / 2.0);

    Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp1(_kalmanFilter.H * _kalmanFilter.P * _kalmanFilter.H.transpose() + _kalmanFilter.R);
    auto rank1 = lu_decomp1.rank();
    if (rank1 != 6)
    {
        LOG_WARN("{}: (HPH^T + R).rank = {}", nameId(), rank1);
    }

    Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp2(_kalmanFilter.K);
    auto rank2 = lu_decomp2.rank();
    if (rank2 != 6)
    {
        LOG_WARN("{}: K.rank = {}", nameId(), rank2);
    }

    // LOG_DEBUG("{}: H\n{}\n", nameId(), _kalmanFilter.H);
    // LOG_DEBUG("{}: R\n{}\n", nameId(), _kalmanFilter.R);
    // LOG_DEBUG("{}: z\n{}\n", nameId(), _kalmanFilter.z);

    // LOG_DEBUG("{}: K\n{}\n", nameId(), _kalmanFilter.K);
    // LOG_DEBUG("{}: x\n{}\n", nameId(), _kalmanFilter.x);
    // LOG_DEBUG("{}: P\n{}\n", nameId(), _kalmanFilter.P);

    // LOG_DEBUG("{}: K * z\n{}\n", nameId(), _kalmanFilter.K * _kalmanFilter.z);

    // LOG_DEBUG("{}: P - P^T\n{}\n", nameId(), _kalmanFilter.P - _kalmanFilter.P.transpose());

    Eigen::FullPivLU<Eigen::MatrixXd> decomp(_kalmanFilter.P);
    auto rank = decomp.rank();
    if (rank != 15)
    {
        LOG_WARN("{}: P.rank = {}", nameId(), rank);
    }

    // Push out the new data
    auto pvaError = std::make_shared<PVAError>();
    pvaError->insTime = gnssMeasurement->insTime;
    pvaError->positionError_lla() = _kalmanFilter.x.block<3, 1>(6, 0).array() * Eigen::Array3d(1. / SCALE_FACTOR_LAT_LON, 1. / SCALE_FACTOR_LAT_LON, 1);
    pvaError->velocityError_n() = _kalmanFilter.x.block<3, 1>(3, 0);
    pvaError->attitudeError_n() = _kalmanFilter.x.block<3, 1>(0, 0) * (1. / SCALE_FACTOR_ATTITUDE);

    _accumulatedImuBiases.biasAccel_b += _kalmanFilter.x.block<3, 1>(9, 0) * (1. / SCALE_FACTOR_ACCELERATION);
    _accumulatedImuBiases.biasGyro_b += _kalmanFilter.x.block<3, 1>(12, 0) * (1. / SCALE_FACTOR_ANGULAR_RATE);

    auto imuBiases = std::make_shared<ImuBiases>();
    imuBiases->insTime = gnssMeasurement->insTime;
    imuBiases->biasAccel_b = _accumulatedImuBiases.biasAccel_b;
    imuBiases->biasGyro_b = _accumulatedImuBiases.biasGyro_b;

    // Closed loop
    // _kalmanFilter.x.block<9, 1>(0, 0).setZero();
    _kalmanFilter.x.setZero();

    invokeCallbacks(OUTPUT_PORT_INDEX_PVA_ERROR, pvaError);
    invokeCallbacks(OUTPUT_PORT_INDEX_IMU_BIASES, imuBiases);
}

// ###########################################################################################################
//                                             System matrix 𝐅
// ###########################################################################################################

Eigen::Matrix<double, 15, 15> NAV::LooselyCoupledKF::systemMatrixF(const Eigen::Quaterniond& quaternion_nb,
                                                                   const Eigen::Vector3d& specForce_ib_b,
                                                                   const Eigen::Vector3d& angularRate_in_n,
                                                                   const Eigen::Vector3d& velocity_n,
                                                                   const Eigen::Vector3d& position_lla,
                                                                   const Eigen::Vector3d& beta_a,
                                                                   const Eigen::Vector3d& beta_omega,
                                                                   double R_N,
                                                                   double R_E)
{
    const double& latitude = position_lla(0); // Geodetic latitude of the body in [rad]
    const double& altitude = position_lla(2); // Geodetic height of the body in [m]

    // System matrix 𝐅
    // Math: \mathbf{F}^n = \begin{pmatrix} \mathbf{F}_{\dot{\psi},\psi}^n & \mathbf{F}_{\dot{\psi},\delta v}^n & \mathbf{F}_{\dot{\psi},\delta r}^n & \mathbf{0}_3 & -\mathbf{C}_b^n \\ \mathbf{F}_{\delta \dot{v},\psi}^n & \mathbf{F}_{\delta \dot{v},\delta v}^n & \mathbf{F}_{\delta \dot{v},\delta r}^n & \mathbf{C}_b^n & \mathbf{0}_3 \\ \mathbf{0}_3 & \mathbf{F}_{\delta \dot{r},\delta v}^n & \mathbf{F}_{\delta \dot{r},\delta r}^n & \mathbf{0}_3 & \mathbf{0}_3 \\ \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 \\ \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 \end{pmatrix} \qquad \text{T. Hobiger, Inertialnavigation V06 - V09 }
    Eigen::MatrixXd F = Eigen::MatrixXd::Zero(15, 15);

    F.block<3, 3>(0, 0) = F_dotpsi_psi_n(angularRate_in_n);
    F.block<3, 3>(0, 3) = F_dotpsi_dv_n(latitude, altitude, R_N, R_E);
    F.block<3, 3>(0, 6) = F_dotpsi_dr_n(latitude, altitude, velocity_n, R_N, R_E);
    F.block<3, 3>(0, 12) = F_dotpsi_dw_n(quaternion_nb.toRotationMatrix());
    F.block<3, 3>(3, 0) = F_dotdv_psi_n(quaternion_nb * specForce_ib_b);
    F.block<3, 3>(3, 3) = F_dotdv_dv_n(velocity_n, latitude, altitude, R_N, R_E);
    F.block<3, 3>(3, 6) = F_dotdv_dr_n(velocity_n, latitude, altitude, R_N, R_E);
    F.block<3, 3>(3, 9) = F_dotdv_df_n(quaternion_nb.toRotationMatrix());
    F.block<3, 3>(6, 3) = F_dotdr_dv_n(latitude, altitude, R_N, R_E);
    F.block<3, 3>(6, 6) = F_dotdr_dr_n(velocity_n, latitude, altitude, R_N, R_E);
    F.block<3, 3>(9, 9) = F_dotdf_df_n(beta_a);
    F.block<3, 3>(12, 12) = F_dotdw_dw_n(beta_omega);

    F.middleRows<3>(0) *= SCALE_FACTOR_ATTITUDE; // 𝜓' [deg / s] = 180/π * ... [rad / s]
    F.middleCols<3>(0) *= 1. / SCALE_FACTOR_ATTITUDE;

    // F.middleRows<3>(3) *= 1.; // 𝛿v' [m / s^2] = 1 * [m / s^2]
    // F.middleCols<3>(3) *= 1. / 1.;

    F.middleRows<2>(6) *= SCALE_FACTOR_LAT_LON; // 𝛿ϕ' [pseudometre / s] = R0 * [rad / s]
    F.middleCols<2>(6) *= 1. / SCALE_FACTOR_LAT_LON;
    // F.middleRows<1>(8) *= 1.; // 𝛿h' [m / s] = 1 * [m / s]
    // F.middleCols<1>(8) *= 1. / 1.;

    F.middleRows<3>(9) *= SCALE_FACTOR_ACCELERATION; // 𝛿f' [mg / s] = 1e3 / g * [m / s^3]
    F.middleCols<3>(9) *= 1. / SCALE_FACTOR_ACCELERATION;

    F.middleRows<3>(12) *= SCALE_FACTOR_ANGULAR_RATE; // 𝛿ω' [mrad / s^2] = 1e3 * [rad / s^2]
    F.middleCols<3>(12) *= 1. / SCALE_FACTOR_ANGULAR_RATE;

    return F;
}

// ###########################################################################################################
//                                           Noise input matrix 𝐆
// ###########################################################################################################

Eigen::Matrix<double, 15, 6> NAV::LooselyCoupledKF::noiseInputMatrixG(const Eigen::Vector3d& sigma2_ra, const Eigen::Vector3d& sigma2_rg, const Eigen::Vector3d& beta_a, const Eigen::Vector3d& beta_omega)
{
    // Math: \mathbf{G}_{a} = \begin{bmatrix} 0 & 0 \\ 0 & 0 \\ 0 & 0 \\ \mathbf{G}_{a} & 0 \\ 0 & \mathbf{G}_{\omega} \end{bmatrix} \quad \text{T. Hobiger}\,(6.5)
    Eigen::Matrix<double, 15, 6> G = Eigen::Matrix<double, 15, 6>::Zero();

    G.block<3, 3>(9, 0) = SCALE_FACTOR_ACCELERATION * noiseInputMatrixG_a(sigma2_ra, beta_a);
    G.block<3, 3>(12, 3) = SCALE_FACTOR_ANGULAR_RATE * noiseInputMatrixG_omega(sigma2_rg, beta_omega);

    return G;
}

Eigen::Matrix3d NAV::LooselyCoupledKF::noiseInputMatrixG_a(const Eigen::Vector3d& sigma2_ra, const Eigen::Vector3d& beta_a)
{
    if (_randomProcessAccel == RandomProcess::RandomWalk) // Random walk (beta = 0)
    {
        // Math: \mathbf{G}_{a} = \begin{bmatrix} \sqrt{\sigma_{a,1}^2} & 0 & 0 \\ 0 & \sqrt{\sigma_{a,2}^2} & 0 \\ 0 & 0 & \sqrt{\sigma_{a,3}^2} \end{bmatrix} \quad \text{T. Hobiger}\,(6.3)
        return Eigen::DiagonalMatrix<double, 3>{ sigma2_ra.cwiseSqrt() };
    }
    // else if (randomProcessAccel == RandomProcess::GaussMarkov1)

    // Math: \mathbf{G}_{a} = \begin{bmatrix} \sqrt{2 \sigma_{a,1}^2 \beta_{a,1}} & 0 & 0 \\ 0 & \sqrt{2 \sigma_{a,2}^2 \beta_{a,2}} & 0 \\ 0 & 0 & \sqrt{2 \sigma_{a,3}^2 \beta_{a,3}} \end{bmatrix} \quad \text{T. Hobiger}\,(6.3)
    return Eigen::DiagonalMatrix<double, 3>{ (2.0 * beta_a.cwiseProduct(sigma2_ra)).cwiseSqrt() };
}

Eigen::Matrix3d NAV::LooselyCoupledKF::noiseInputMatrixG_omega(const Eigen::Vector3d& sigma2_rg, const Eigen::Vector3d& beta_omega)
{
    if (_randomProcessGyro == RandomProcess::RandomWalk) // Random walk (beta = 0)
    {
        // Math: \mathbf{G}_{\omega} = \begin{bmatrix} \sqrt{\sigma_{\omega,1}^2} & 0 & 0 \\ 0 & \sqrt{\sigma_{\omega,2}^2} & 0 \\ 0 & 0 & \sqrt{\sigma_{\omega,3}^2} \end{bmatrix} \quad \text{T. Hobiger}\,(6.3)

        return Eigen::DiagonalMatrix<double, 3>{ sigma2_rg.cwiseSqrt() };
    }
    // else if (randomProcessGyro == RandomProcess::GaussMarkov1)

    // Math: \mathbf{G}_{\omega} = \begin{bmatrix} \sqrt{2 \sigma_{\omega,1}^2 \beta_{\omega,1}} & 0 & 0 \\ 0 & \sqrt{2 \sigma_{\omega,2}^2 \beta_{\omega,2}} & 0 \\ 0 & 0 & \sqrt{2 \sigma_{\omega,3}^2 \beta_{\omega,3}} \end{bmatrix} \quad \text{T. Hobiger}\,(6.3)
    return Eigen::DiagonalMatrix<double, 3>{ (2.0 * beta_omega.cwiseProduct(sigma2_rg)).cwiseSqrt() };
}

// ###########################################################################################################
//                                         Error covariance matrix P
// ###########################################################################################################

Eigen::Matrix<double, 15, 15> NAV::LooselyCoupledKF::initialErrorCovarianceMatrixP0(const Eigen::Vector3d& variance_angles,
                                                                                    const Eigen::Vector3d& variance_vel,
                                                                                    const Eigen::Vector3d& variance_lla,
                                                                                    const Eigen::Vector3d& variance_accelBias,
                                                                                    const Eigen::Vector3d& variance_gyroBias)
{
    // 𝐏 Error covariance matrix
    Eigen::Matrix<double, 15, 15> P = Eigen::Matrix<double, 15, 15>::Zero();

    P.diagonal() << std::pow(SCALE_FACTOR_ATTITUDE, 2) * variance_angles, // Flight Angles covariance
        variance_vel,                                                     // Velocity covariance
        std::pow(SCALE_FACTOR_LAT_LON, 2) * variance_lla(0),              // Latitude covariance
        std::pow(SCALE_FACTOR_LAT_LON, 2) * variance_lla(1),              // Longitude covariance
        variance_lla(2),                                                  // Altitude covariance
        std::pow(SCALE_FACTOR_ACCELERATION, 2) * variance_accelBias,      // Accelerometer Bias covariance
        std::pow(SCALE_FACTOR_ANGULAR_RATE, 2) * variance_gyroBias;       // Gyroscope Bias covariance

    return P;
}

// ###########################################################################################################
//                                                Correction
// ###########################################################################################################

Eigen::Matrix<double, 6, 15> NAV::LooselyCoupledKF::measurementMatrix(const Eigen::Matrix3d& T_rn_p, const Eigen::Matrix3d& DCM_nb, const Eigen::Vector3d& angularRate_ib_b, const Eigen::Vector3d& leverArm_InsGnss_b, const Eigen::Matrix3d& Omega_ie_n)
{
    // Math: \mathbf{H}_{G,k}^n = \begin{pmatrix} \mathbf{H}_{r1}^n & \mathbf{0}_3 & \mathbf{I}_3 & \mathbf{0}_3 & \mathbf{0}_3 \\ \mathbf{H}_{v1}^n & \mathbf{I}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{H}_{v5}^n \end{pmatrix}_k \qquad \text{P. Groves}\,(14.113)
    // G denotes GNSS indicated
    Eigen::Matrix<double, 6, 15> H = Eigen::Matrix<double, 6, 15>::Zero();
    H.block<3, 3>(0, 0) = measurementMatrix_r1_n(T_rn_p, DCM_nb, leverArm_InsGnss_b);
    H.block<3, 3>(0, 6) = -Eigen::Matrix3d::Identity();
    H.block<3, 3>(3, 0) = measurementMatrix_v1_n(DCM_nb, angularRate_ib_b, leverArm_InsGnss_b, Omega_ie_n);
    H.block<3, 3>(3, 3) = -Eigen::Matrix3d::Identity();
    H.block<3, 3>(3, 12) = measurementMatrix_v5_n(DCM_nb, leverArm_InsGnss_b);

    H.middleRows<2>(0) *= SCALE_FACTOR_LAT_LON;

    H.middleCols<3>(0) *= 1. / SCALE_FACTOR_ATTITUDE;
    H.middleCols<2>(6) *= 1. / SCALE_FACTOR_LAT_LON;
    // H.middleCols<3>(9) *= 1. / SCALE_FACTOR_ACCELERATION; // Only zero elements
    H.middleCols<3>(12) *= 1. / SCALE_FACTOR_ANGULAR_RATE;

    return H;
}

Eigen::Matrix3d NAV::LooselyCoupledKF::measurementMatrix_r1_n(const Eigen::Matrix3d& T_rn_p, const Eigen::Matrix3d& DCM_nb, const Eigen::Vector3d& leverArm_InsGnss_b)
{
    // Math: \mathbf{H}_{r1}^n \approx \mathbf{\hat{T}}_{r(n)}^p \begin{bmatrix} \begin{pmatrix} \mathbf{C}_b^n \mathbf{l}_{ba}^p \end{pmatrix} \wedge \end{bmatrix} \qquad \text{P. Groves}\,(14.114)
    Eigen::Vector3d product = DCM_nb * leverArm_InsGnss_b;
    return T_rn_p * skewSymmetricMatrix(product);
}

Eigen::Matrix3d NAV::LooselyCoupledKF::measurementMatrix_v1_n(const Eigen::Matrix3d& DCM_nb, const Eigen::Vector3d& angularRate_ib_b, const Eigen::Vector3d& leverArm_InsGnss_b, const Eigen::Matrix3d& Omega_ie_n)
{
    // Math: \mathbf{H}_{v1}^n \approx \begin{bmatrix} \begin{Bmatrix} \mathbf{C}_b^n (\mathbf{\hat{\omega}}_{ib}^b \wedge \mathbf{l}_{ba}^b) - \mathbf{\hat{\Omega}}_{ie}^n \mathbf{C}_b^n \mathbf{l}_{ba}^b \end{Bmatrix} \wedge \end{bmatrix} \qquad \text{P. Groves}\,(14.114)
    Eigen::Vector3d product = DCM_nb * (angularRate_ib_b.cross(leverArm_InsGnss_b)) - Omega_ie_n * DCM_nb * leverArm_InsGnss_b;

    return skewSymmetricMatrix(product);
}

Eigen::Matrix3d NAV::LooselyCoupledKF::measurementMatrix_v5_n(const Eigen::Matrix3d& DCM_nb, const Eigen::Vector3d& leverArm_InsGnss_b)
{
    // Math: \mathbf{H}_{v5}^n = \mathbf{C}_b^n \begin{bmatrix} \mathbf{l}_{ba}^b \wedge \end{bmatrix} \qquad \text{P. Groves}\,(14.114)
    return DCM_nb * skewSymmetricMatrix(leverArm_InsGnss_b);
}

Eigen::Matrix<double, 6, 6> NAV::LooselyCoupledKF::measurementNoiseCovariance(const Eigen::Vector3d& gnssVarianceLatLonAlt, const Eigen::Vector3d& gnssVarianceVelocity)
{
    // Math: \mathbf{R} = \begin{pmatrix} \sigma^2_\phi & 0 & 0 & 0 & 0 & 0 \\ 0 & \sigma^2_\lambda & 0 & 0 & 0 & 0 \\ 0 & 0 & \sigma^2_h & 0 & 0 & 0 \\ 0 & 0 & 0 & \sigma^2_{v_N} & 0 & 0 \\ 0 & 0 & 0 & 0 & \sigma^2_{v_E} & 0 \\ 0 & 0 & 0 & 0 & 0 & \sigma^2_{v_D} \end{pmatrix}
    Eigen::Matrix<double, 6, 6> R = Eigen::Matrix<double, 6, 6>::Zero();
    R.block<3, 3>(0, 0).diagonal() = gnssVarianceLatLonAlt;
    R.block<3, 3>(3, 3).diagonal() = gnssVarianceVelocity;

    R.block<2, 2>(0, 0).diagonal() *= std::pow(SCALE_FACTOR_LAT_LON, 2);

    return R;
}

Eigen::Matrix<double, 6, 1> NAV::LooselyCoupledKF::measurementInnovation(const Eigen::Vector3d& positionMeasurement_lla, const Eigen::Vector3d& positionEstimate_lla,
                                                                         const Eigen::Vector3d& velocityMeasurement_n, const Eigen::Vector3d& velocityEstimate_n,
                                                                         const Eigen::Matrix3d& T_rn_p, const Eigen::Quaterniond& q_nb, const Eigen::Vector3d& leverArm_InsGnss_b,
                                                                         const Eigen::Vector3d& angularRate_ib_b, const Eigen::Matrix3d& Omega_ie_n)
{
    // Math: \delta\mathbf{z}_{G,k}^{n-} = \begin{pmatrix} \mathbf{\hat{p}}_{aG} - \mathbf{\hat{p}}_b - \mathbf{\hat{T}}_{r(n)}^p \mathbf{C}_b^n \mathbf{l}_{ba}^b \\ \mathbf{\hat{v}}_{eaG}^n - \mathbf{\hat{v}}_{eb}^n - \mathbf{C}_b^n (\mathbf{\hat{\omega}}_{ib}^b \wedge \mathbf{l}_{ba}^b) + \mathbf{\hat{\Omega}}_{ie}^n \mathbf{C}_b^n \mathbf{l}_{ba}^b \end{pmatrix} \qquad \text{P. Groves}\,(14.116)
    Eigen::Vector3d deltaLLA = positionMeasurement_lla - positionEstimate_lla - T_rn_p * (q_nb * leverArm_InsGnss_b);
    Eigen::Vector3d deltaVel = velocityMeasurement_n - velocityEstimate_n - q_nb * (angularRate_ib_b.cross(leverArm_InsGnss_b)) + Omega_ie_n * (q_nb * leverArm_InsGnss_b);

    deltaLLA.topRows<2>() *= SCALE_FACTOR_LAT_LON;

    Eigen::Matrix<double, 6, 1> innovation;
    innovation << deltaLLA, deltaVel;

    return innovation;
}

// #########################################################################################################################################
//                                                               Deprecated
// #########################################################################################################################################

// ###########################################################################################################
//                                     System noise covariance matrix 𝐐
// ###########################################################################################################

Eigen::Matrix<double, 15, 15> NAV::LooselyCoupledKF::systemNoiseCovarianceMatrix(const Eigen::Vector3d& sigma2_ra, const Eigen::Vector3d& sigma2_rg, const double& sigma2_bad, const double& sigma2_bgd, const Eigen::Matrix3d& F_21_n, const Eigen::Matrix3d& T_rn_p, const Eigen::Matrix3d& DCM_nb, const double& tau_s)
{
    // Math: \mathbf{Q}_{INS}^n = \begin{pmatrix} \mathbf{Q}_{11} & {\mathbf{Q}_{21}^n}^T & {\mathbf{Q}_{31}^n}^T & \mathbf{0}_3 & {\mathbf{Q}_{51}^n}^T \\ \mathbf{Q}_{21}^n & \mathbf{Q}_{22}^n & {\mathbf{Q}_{32}^n}^T & {\mathbf{Q}_{42}^n}^T & \mathbf{Q}_{25}^n \\ \mathbf{Q}_{31}^n & \mathbf{Q}_{32}^n & \mathbf{Q}_{33}^n & \mathbf{Q}_{34}^n & \mathbf{Q}_{35}^n \\ \mathbf{0}_3 & \mathbf{Q}_{42}^n & {\mathbf{Q}_{34}^n}^T & S_{bad}\tau_s\mathbf{I}_3 & \mathbf{0}_3 \\ \mathbf{Q}_{51}^n & \mathbf{Q}_{52}^n & {\mathbf{Q}_{35}^n}^T & \mathbf{0}_3 & S_{bgd}\tau_s\mathbf{I}_3 \end{pmatrix} \qquad \text{P. Groves}\,(14.80)
    const double S_ra = psdGyroNoise(sigma2_ra, tau_s);
    const double S_rg = psdAccelNoise(sigma2_rg, tau_s);
    const double S_bad = psdAccelBiasVariation(sigma2_bad, tau_s);
    const double S_bgd = psdGyroBiasVariation(sigma2_bgd, tau_s);

    Eigen::Matrix<double, 15, 15> Q = Eigen::Matrix<double, 15, 15>::Zero();
    Q.block<3, 3>(0, 0) = systemNoiseCovariance_11(S_rg, S_bgd, tau_s);
    Q.block<3, 3>(3, 0) = systemNoiseCovariance_21(S_rg, S_bgd, F_21_n, tau_s);
    Q.block<3, 3>(3, 3) = systemNoiseCovariance_22(S_ra, S_bad, S_rg, S_bgd, F_21_n, tau_s);
    Q.block<3, 3>(3, 12) = systemNoiseCovariance_25(S_bgd, F_21_n, DCM_nb, tau_s);
    Q.block<3, 3>(6, 0) = systemNoiseCovariance_31(S_rg, S_bgd, F_21_n, T_rn_p, tau_s);
    Q.block<3, 3>(6, 3) = systemNoiseCovariance_32(S_ra, S_bad, S_rg, S_bgd, F_21_n, T_rn_p, tau_s);
    Q.block<3, 3>(6, 6) = systemNoiseCovariance_33(S_ra, S_bad, S_rg, S_bgd, T_rn_p, F_21_n, tau_s);
    Q.block<3, 3>(6, 9) = systemNoiseCovariance_34(S_bgd, T_rn_p, DCM_nb, tau_s);
    Q.block<3, 3>(6, 12) = systemNoiseCovariance_35(S_bgd, F_21_n, T_rn_p, DCM_nb, tau_s);
    Q.block<3, 3>(9, 3) = systemNoiseCovariance_42(S_bad, DCM_nb, tau_s);
    Q.block<3, 3>(9, 9) = systemNoiseCovariance_44(S_bad, tau_s);
    Q.block<3, 3>(12, 0) = systemNoiseCovariance_51(S_bgd, DCM_nb, tau_s);
    Q.block<3, 3>(12, 12) = systemNoiseCovariance_55(S_bad, tau_s);

    Q.block<3, 3>(0, 3) = Q.block<3, 3>(3, 0).transpose();   // Q_21^T
    Q.block<3, 3>(0, 6) = Q.block<3, 3>(6, 0).transpose();   // Q_31^T
    Q.block<3, 3>(3, 6) = Q.block<3, 3>(6, 3).transpose();   // Q_32^T
    Q.block<3, 3>(9, 6) = Q.block<3, 3>(6, 9).transpose();   // Q_34^T
    Q.block<3, 3>(12, 3) = Q.block<3, 3>(3, 12).transpose(); // Q_25^T
    Q.block<3, 3>(12, 6) = Q.block<3, 3>(6, 12).transpose(); // Q_35^T
    Q.block<3, 3>(3, 9) = Q.block<3, 3>(9, 3).transpose();   // Q_42^T
    Q.block<3, 3>(0, 12) = Q.block<3, 3>(12, 0).transpose(); // Q_51^T

    // TODO: Scale the Q matrix

    return Q;
}

double NAV::LooselyCoupledKF::psdGyroNoise(const Eigen::Vector3d& sigma2_ra, const double& tau_i)
{
    // Math: S_{ra} = \sigma_{ra}^2\tau_i \qquad \text{P. Groves}\,(14.83)
    return sigma2_ra.mean() * tau_i; // TODO: This is only a temporary fix. Here the values should be accounted for, for each axis separately.
}

double NAV::LooselyCoupledKF::psdAccelNoise(const Eigen::Vector3d& sigma2_rg, const double& tau_i)
{
    // Math: S_{rg} = \sigma_{rg}^2\tau_i \qquad \text{P. Groves}\,(14.83)
    return sigma2_rg.mean() * tau_i; // TODO: This is only a temporary fix. Here the values should be accounted for, for each axis separately.
}

double NAV::LooselyCoupledKF::psdAccelBiasVariation(const double& sigma2_bad, const double& tau_i)
{
    // Math: S_{bad} = \frac{\sigma_{bad}^2}{\tau_i} \qquad \text{P. Groves}\,(14.84)
    return sigma2_bad / tau_i;
}

double NAV::LooselyCoupledKF::psdGyroBiasVariation(const double& sigma2_bgd, const double& tau_i)
{
    // Math: S_{bgd} = \frac{\sigma_{bgd}^2}{\tau_i} \qquad \text{P. Groves}\,(14.84)
    return sigma2_bgd / tau_i;
}

Eigen::Matrix3d NAV::LooselyCoupledKF::conversionMatrixCartesianCurvilinear(const Eigen::Vector3d& position_lla, const double& R_N, const double& R_E)
{
    return Eigen::DiagonalMatrix<double, 3>{ 1.0 / (R_N + position_lla(2)),
                                             1.0 / ((R_E + position_lla(2)) * std::cos(position_lla(0))),
                                             -1.0 };
}

Eigen::Matrix3d NAV::LooselyCoupledKF::systemNoiseCovariance_11(const double& S_rg, const double& S_bgd, const double& tau_s)
{
    return (S_rg * tau_s + 1.0 / 3.0 * S_bgd * std::pow(tau_s, 3)) * Eigen::Matrix3d::Identity();
}

Eigen::Matrix3d NAV::LooselyCoupledKF::systemNoiseCovariance_21(const double& S_rg, const double& S_bgd, const Eigen::Matrix3d& F_21_n, const double& tau_s)
{
    return (0.5 * S_rg * std::pow(tau_s, 2) + 0.25 * S_bgd * std::pow(tau_s, 4)) * F_21_n;
}

Eigen::Matrix3d NAV::LooselyCoupledKF::systemNoiseCovariance_25(const double& S_bgd, const Eigen::Matrix3d& F_21_n, const Eigen::Matrix3d& DCM_nb, const double& tau_s)
{
    return 1.0 / 3.0 * S_bgd * std::pow(tau_s, 3) * F_21_n * DCM_nb;
}

Eigen::Matrix3d NAV::LooselyCoupledKF::systemNoiseCovariance_22(const double& S_ra, const double& S_bad, const double& S_rg, const double& S_bgd, const Eigen::Matrix3d& F_21_n, const double& tau_s)
{
    return (S_ra * tau_s + 1.0 / 3.0 * S_bad * std::pow(tau_s, 3)) * Eigen::Matrix3d::Identity()
           + (1.0 / 3.0 * S_rg * std::pow(tau_s, 3) + 0.2 * S_bgd * std::pow(tau_s, 5)) * F_21_n * F_21_n.transpose();
}

Eigen::Matrix3d NAV::LooselyCoupledKF::systemNoiseCovariance_31(const double& S_rg, const double& S_bgd, const Eigen::Matrix3d& F_21_n, const Eigen::Matrix3d& T_rn_p, const double& tau_s)
{
    return (1.0 / 3.0 * S_rg * std::pow(tau_s, 3) + 0.2 * S_bgd * std::pow(tau_s, 5)) * T_rn_p * F_21_n;
}

Eigen::Matrix3d NAV::LooselyCoupledKF::systemNoiseCovariance_32(const double& S_ra, const double& S_bad, const double& S_rg, const double& S_bgd, const Eigen::Matrix3d& F_21_n, const Eigen::Matrix3d& T_rn_p, const double& tau_s)
{
    return (0.5 * S_ra * std::pow(tau_s, 2) + 0.25 * S_bad * std::pow(tau_s, 4)) * T_rn_p
           + (0.25 * S_rg * std::pow(tau_s, 4) + 1.0 / 6.0 * S_bgd * std::pow(tau_s, 6)) * T_rn_p * F_21_n * F_21_n.transpose();
}

Eigen::Matrix3d NAV::LooselyCoupledKF::systemNoiseCovariance_33(const double& S_ra, const double& S_bad, const double& S_rg, const double& S_bgd, const Eigen::Matrix3d& T_rn_p, const Eigen::Matrix3d& F_21_n, const double& tau_s)
{
    return (1.0 / 3.0 * S_ra * std::pow(tau_s, 3) + 0.2 * S_bad * std::pow(tau_s, 5)) * T_rn_p * T_rn_p
           + (0.2 * S_rg * std::pow(tau_s, 5) + 1.0 / 7.0 * S_bgd * std::pow(tau_s, 7)) * T_rn_p * F_21_n * F_21_n.transpose() * T_rn_p;
}

Eigen::Matrix3d NAV::LooselyCoupledKF::systemNoiseCovariance_34(const double& S_bad, const Eigen::Matrix3d& T_rn_p, const Eigen::Matrix3d& DCM_nb, const double& tau_s)
{
    return 1.0 / 3.0 * S_bad * std::pow(tau_s, 3) * T_rn_p * DCM_nb.transpose();
}

Eigen::Matrix3d NAV::LooselyCoupledKF::systemNoiseCovariance_35(const double& S_bgd, const Eigen::Matrix3d& F_21_n, const Eigen::Matrix3d& T_rn_p, const Eigen::Matrix3d& DCM_nb, const double& tau_s)
{
    return 0.25 * S_bgd * std::pow(tau_s, 4) * T_rn_p * F_21_n * DCM_nb.transpose();
}

Eigen::Matrix3d NAV::LooselyCoupledKF::systemNoiseCovariance_42(const double& S_bad, const Eigen::Matrix3d& DCM_nb, const double& tau_s)
{
    return 0.5 * S_bad * std::pow(tau_s, 2) * DCM_nb;
}

Eigen::Matrix3d NAV::LooselyCoupledKF::systemNoiseCovariance_44(const double& S_bad, const double& tau_s)
{
    return S_bad * tau_s * Eigen::Matrix3d::Identity();
}

Eigen::Matrix3d NAV::LooselyCoupledKF::systemNoiseCovariance_51(const double& S_bgd, const Eigen::Matrix3d& DCM_nb, const double& tau_s)
{
    return 0.5 * S_bgd * std::pow(tau_s, 2) * DCM_nb;
}

Eigen::Matrix3d NAV::LooselyCoupledKF::systemNoiseCovariance_55(const double& S_bgd, const double& tau_s)
{
    return S_bgd * tau_s * Eigen::Matrix3d::Identity();
}