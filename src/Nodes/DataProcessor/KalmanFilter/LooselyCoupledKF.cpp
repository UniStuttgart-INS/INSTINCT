#include "LooselyCoupledKF.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <cmath>

#include <imgui_internal.h>
#include "internal/gui/widgets/imgui_ex.hpp"

#include "NodeData/State/PVAError.hpp"
#include "NodeData/State/ImuBiases.hpp"

#include "internal/FlowManager.hpp"
#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "util/InsMechanization.hpp"
#include "util/InsConstants.hpp"
#include "util/InsMath.hpp"
#include "util/InsGravity.hpp"
#include "util/Logger.hpp"

NAV::LooselyCoupledKF::LooselyCoupledKF()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    hasConfig = true;
    guiConfigDefaultWindowSize = { 650, 360 };

    nm::CreateInputPin(this, "InertialNavSol", Pin::Type::Flow, { NAV::InertialNavSol::type() }, &LooselyCoupledKF::recvInertialNavigationSolution);
    nm::CreateInputPin(this, "GNSSNavigationSolution", Pin::Type::Flow, { NAV::PosVelAtt::type() }, &LooselyCoupledKF::recvGNSSNavigationSolution);
    nm::CreateOutputPin(this, "PVAError", Pin::Type::Flow, { NAV::PVAError::type() });
    nm::CreateOutputPin(this, "ImuBiases", Pin::Type::Flow, { NAV::ImuBiases::type() });
    nm::CreateOutputPin(this, "x", Pin::Type::Matrix, { "Eigen::MatrixXd" }, &kalmanFilter.x);
    nm::CreateOutputPin(this, "P", Pin::Type::Matrix, { "Eigen::MatrixXd" }, &kalmanFilter.P);
    nm::CreateOutputPin(this, "Phi", Pin::Type::Matrix, { "Eigen::MatrixXd" }, &kalmanFilter.Phi);
    nm::CreateOutputPin(this, "Q", Pin::Type::Matrix, { "Eigen::MatrixXd" }, &kalmanFilter.Q);
    nm::CreateOutputPin(this, "z", Pin::Type::Matrix, { "Eigen::MatrixXd" }, &kalmanFilter.z);
    nm::CreateOutputPin(this, "H", Pin::Type::Matrix, { "Eigen::MatrixXd" }, &kalmanFilter.H);
    nm::CreateOutputPin(this, "R", Pin::Type::Matrix, { "Eigen::MatrixXd" }, &kalmanFilter.R);
    nm::CreateOutputPin(this, "K", Pin::Type::Matrix, { "Eigen::MatrixXd" }, &kalmanFilter.K);
    nm::CreateOutputPin(this, "K*z", Pin::Type::Matrix, { "Eigen::MatrixXd" }, &kalmanFilter_Kz);

    // SPP accuracy approx. 3m in horizontal direction and 3 times worse in vertical direction
    gnssSigmaSquaredLatLonAlt = trafo::ecef2lla_WGS84(trafo::ned2ecef({ 0.03, 0.03, 0.03 * 3 }, { 0, 0, 0 })).array().pow(2);
    gnssSigmaSquaredLatLonAlt(0) *= 1e6;
    gnssSigmaSquaredLatLonAlt(1) *= 1e6;
    gnssSigmaSquaredVelocity = Eigen::Array3d(0.5, 0.5, 0.5).pow(2);
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
    ImGui::SetNextItemWidth(300 + ImGui::GetStyle().ItemSpacing.x);
    if (ImGui::Combo(fmt::format("Phi calculation algorithm##{}", size_t(id)).c_str(), reinterpret_cast<int*>(&phiCalculation), "Taylor 1st Order\0Van Loan\0\0"))
    {
        LOG_DEBUG("{}: Phi calculation algorithm changed to {}", nameId(), phiCalculation);

        if (phiCalculation != PhiCalculation::VanLoan)
        {
            qCalculation = QCalculation::Groves;
        }

        flow::ApplyChanges();
    }
    if (phiCalculation != PhiCalculation::VanLoan)
    {
        ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
        ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5F);
    }

    ImGui::SetNextItemWidth(300 + ImGui::GetStyle().ItemSpacing.x);
    if (ImGui::Combo(fmt::format("Q calculation algorithm##{}", size_t(id)).c_str(), reinterpret_cast<int*>(&qCalculation), "Groves\0Van Loan\0\0"))
    {
        LOG_DEBUG("{}: Q calculation algorithm changed to {}", nameId(), qCalculation);
        flow::ApplyChanges();
    }

    if (phiCalculation != PhiCalculation::VanLoan)
    {
        ImGui::PopItemFlag();
        ImGui::PopStyleVar();
    }

    // ###########################################################################################################
    //                                       Random Process Accelerometer
    // ###########################################################################################################

    if (randomProcessAccel == RandomProcess::GaussMarkov1)
    {
        ImGui::Separator();
    }
    ImGui::SetNextItemWidth(300 + ImGui::GetStyle().ItemSpacing.x);
    if (ImGui::Combo(fmt::format("Random Process Accelerometer##{}", size_t(id)).c_str(), reinterpret_cast<int*>(&randomProcessAccel), "White Noise\0"
                                                                                                                                       "Random Constant\0"
                                                                                                                                       "Random Walk\0"
                                                                                                                                       "Gauss-Markov 1st Order\0"
                                                                                                                                       "Gauss-Markov 2nd Order\0"
                                                                                                                                       "Gauss-Markov 3rd Order\0\0"))
    {
        if (randomProcessAccel != RandomProcess::RandomWalk && randomProcessAccel != RandomProcess::GaussMarkov1)
        {
            LOG_ERROR("Currently only 'Random Walk' and 'Gauss-Markov 1st Order' is supported");
            randomProcessAccel = RandomProcess::RandomWalk;
        }

        LOG_DEBUG("{}: randomProcessAccel changed to {}", nameId(), randomProcessAccel);
        flow::ApplyChanges();
    }
    if (randomProcessAccel == RandomProcess::GaussMarkov1)
    {
        ImGui::SetNextItemWidth(300 + ImGui::GetStyle().ItemSpacing.x);
        if (ImGui::InputDouble3(fmt::format("Gauss-Markov β Accelerometer##{}", size_t(id)).c_str(), beta_accel.data(), "%.3e"))
        {
            LOG_DEBUG("{}: beta_accel changed to {}", nameId(), beta_accel.transpose());
            flow::ApplyChanges();
        }
        ImGui::Separator();
    }

    // ###########################################################################################################
    //                                         Random Process Gyroscope
    // ###########################################################################################################

    if (randomProcessAccel != RandomProcess::GaussMarkov1 && randomProcessGyro == RandomProcess::GaussMarkov1)
    {
        ImGui::Separator();
    }
    ImGui::SetNextItemWidth(300 + ImGui::GetStyle().ItemSpacing.x);
    if (ImGui::Combo(fmt::format("Random Process Gyroscope##{}", size_t(id)).c_str(), reinterpret_cast<int*>(&randomProcessGyro), "White Noise\0"
                                                                                                                                  "Random Constant\0"
                                                                                                                                  "Random Walk\0"
                                                                                                                                  "Gauss-Markov 1st Order\0"
                                                                                                                                  "Gauss-Markov 2nd Order\0"
                                                                                                                                  "Gauss-Markov 3rd Order\0\0"))
    {
        if (randomProcessGyro != RandomProcess::RandomWalk && randomProcessGyro != RandomProcess::GaussMarkov1)
        {
            LOG_ERROR("Currently only 'Random Walk' and 'Gauss-Markov 1st Order' is supported");
            randomProcessGyro = RandomProcess::RandomWalk;
        }

        LOG_DEBUG("{}: randomProcessGyro changed to {}", nameId(), randomProcessGyro);
        flow::ApplyChanges();
    }
    if (randomProcessGyro == RandomProcess::GaussMarkov1)
    {
        ImGui::SetNextItemWidth(300 + ImGui::GetStyle().ItemSpacing.x);
        if (ImGui::InputDouble3(fmt::format("Gauss-Markov β Gyroscope##{}", size_t(id)).c_str(), beta_gyro.data(), "%.3e"))
        {
            LOG_DEBUG("{}: beta_gyro changed to {}", nameId(), beta_gyro.transpose());
            flow::ApplyChanges();
        }
        ImGui::Separator();
    }

    // ###########################################################################################################
    //                                          Process Noise Variances
    // ###########################################################################################################

    ImGui::SetNextItemWidth(160);
    if (ImGui::InputDouble(fmt::format("##variance_ra {}", size_t(id)).c_str(), &variance_ra, 0.0, 0.0, "%.10e"))
    {
        LOG_DEBUG("{}: variance_ra changed to {}", nameId(), variance_ra);
        flow::ApplyChanges();
    }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(140);
    if (ImGui::Combo(fmt::format("##varianceAccelNoiseUnits {}", size_t(id)).c_str(), reinterpret_cast<int*>(&varianceAccelNoiseUnits), "mg/√(Hz)\0\0"))
    {
        LOG_DEBUG("{}: varianceAccelNoiseUnits changed to {}", nameId(), varianceAccelNoiseUnits);
        flow::ApplyChanges();
    }
    ImGui::SameLine();
    if (varianceAccelNoiseUnits == VarianceAccelNoiseUnits::mg_sqrtHz)
    {
        ImGui::TextUnformatted("Standard deviation of the noise on the\n"
                               "accelerometer specific-force measurements");
    }
    else
    {
        ImGui::TextUnformatted("Variance of the noise on the\n"
                               "accelerometer specific-force measurements");
    }

    ImGui::SetNextItemWidth(160);
    if (ImGui::InputDouble(fmt::format("##variance_rg {}", size_t(id)).c_str(), &variance_rg, 0.0, 0.0, "%.10e"))
    {
        LOG_DEBUG("{}: variance_rg changed to {}", nameId(), variance_rg);
        flow::ApplyChanges();
    }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(140);
    if (ImGui::Combo(fmt::format("##varianceGyroNoiseUnits {}", size_t(id)).c_str(), reinterpret_cast<int*>(&varianceGyroNoiseUnits), "deg/hr/√(Hz)\0\0"))
    {
        LOG_DEBUG("{}: varianceGyroNoiseUnits changed to {}", nameId(), varianceGyroNoiseUnits);
        flow::ApplyChanges();
    }
    ImGui::SameLine();
    if (varianceGyroNoiseUnits == VarianceGyroNoiseUnits::deg_hr_sqrtHz)
    {
        ImGui::TextUnformatted("Standard deviation of the noise on\n"
                               "the gyro angular-rate measurements");
    }
    else
    {
        ImGui::TextUnformatted("Variance of the noise on\n"
                               "the gyro angular-rate measurements");
    }

    if (qCalculation == QCalculation::Groves)
    {
        ImGui::SetNextItemWidth(160);
        if (ImGui::InputDouble(fmt::format("##variance_bad {}", size_t(id)).c_str(), &variance_bad, 0.0, 0.0, "%.10e"))
        {
            LOG_DEBUG("{}: variance_bad changed to {}", nameId(), variance_bad);
            flow::ApplyChanges();
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(140);
        if (ImGui::Combo(fmt::format("##varianceAccelNoiseUnits {}", size_t(id)).c_str(), reinterpret_cast<int*>(&varianceAccelNoiseUnits), "µg\0\0"))
        {
            LOG_DEBUG("{}: varianceAccelNoiseUnits changed to {}", nameId(), varianceAccelNoiseUnits);
            flow::ApplyChanges();
        }
        ImGui::SameLine();
        if (varianceAccelBiasUnits == VarianceAccelBiasUnits::microg)
        {
            ImGui::TextUnformatted("Variance of the accelerometer dynamic bias");
        }
        else
        {
            ImGui::TextUnformatted("Standard deviation of the accelerometer dynamic bias");
        }

        ImGui::SetNextItemWidth(160);
        if (ImGui::InputDouble(fmt::format("##variance_bgd {}", size_t(id)).c_str(), &variance_bgd, 0.0, 0.0, "%.10e"))
        {
            LOG_DEBUG("{}: variance_rg changed to {}", nameId(), variance_bgd);
            flow::ApplyChanges();
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(140);
        if (ImGui::Combo(fmt::format("##varianceGyroNoiseUnits {}", size_t(id)).c_str(), reinterpret_cast<int*>(&varianceGyroNoiseUnits), "°/h\0\0"))
        {
            LOG_DEBUG("{}: varianceGyroNoiseUnits changed to {}", nameId(), varianceGyroNoiseUnits);
            flow::ApplyChanges();
        }
        ImGui::SameLine();
        if (varianceGyroBiasUnits == VarianceGyroBiasUnits::deg_h)
        {
            ImGui::TextUnformatted("Variance of the gyro dynamic bias");
        }
        else
        {
            ImGui::TextUnformatted("Standard deviation of the gyro dynamic bias");
        }
    }
}

[[nodiscard]] json NAV::LooselyCoupledKF::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["phiCalculation"] = phiCalculation;
    j["qCalculation"] = qCalculation;
    j["randomProcessAccel"] = randomProcessAccel;
    j["beta_accel"] = beta_accel;
    j["randomProcessGyro"] = randomProcessGyro;
    j["beta_gyro"] = beta_gyro;
    j["variance_ra"] = variance_ra;
    j["varianceAccelNoiseUnits"] = varianceAccelNoiseUnits;
    j["variance_rg"] = variance_rg;
    j["varianceGyroNoiseUnits"] = varianceGyroNoiseUnits;
    j["variance_bad"] = variance_bad;
    j["varianceAccelBiasUnits"] = varianceAccelBiasUnits;
    j["variance_bgd"] = variance_bgd;
    j["varianceGyroBiasUnits"] = varianceGyroBiasUnits;

    return j;
}

void NAV::LooselyCoupledKF::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());
    if (j.contains("phiCalculation"))
    {
        phiCalculation = static_cast<PhiCalculation>(j.at("phiCalculation").get<int>());
    }
    if (j.contains("qCalculation"))
    {
        qCalculation = static_cast<QCalculation>(j.at("qCalculation").get<int>());
    }
    if (j.contains("randomProcessAccel"))
    {
        randomProcessAccel = static_cast<RandomProcess>(j.at("randomProcessAccel").get<int>());
    }
    if (j.contains("beta_accel"))
    {
        beta_accel = j.at("beta_accel");
    }
    if (j.contains("randomProcessGyro"))
    {
        randomProcessGyro = static_cast<RandomProcess>(j.at("randomProcessGyro").get<int>());
    }
    if (j.contains("beta_gyro"))
    {
        beta_gyro = j.at("beta_gyro");
    }
    if (j.contains("variance_ra"))
    {
        variance_ra = j.at("variance_ra");
    }
    if (j.contains("varianceAccelNoiseUnits"))
    {
        varianceAccelNoiseUnits = static_cast<VarianceAccelNoiseUnits>(j.at("varianceAccelNoiseUnits").get<int>());
    }
    if (j.contains("variance_rg"))
    {
        variance_rg = j.at("variance_rg");
    }
    if (j.contains("varianceGyroNoiseUnits"))
    {
        varianceGyroNoiseUnits = static_cast<VarianceGyroNoiseUnits>(j.at("varianceGyroNoiseUnits").get<int>());
    }
    if (j.contains("variance_bad"))
    {
        variance_bad = j.at("variance_bad");
    }
    if (j.contains("varianceAccelBiasUnits"))
    {
        varianceAccelBiasUnits = static_cast<VarianceAccelBiasUnits>(j.at("varianceAccelBiasUnits").get<int>());
    }
    if (j.contains("variance_bgd"))
    {
        variance_bgd = j.at("variance_bgd");
    }
    if (j.contains("varianceGyroBiasUnits"))
    {
        varianceGyroBiasUnits = static_cast<VarianceGyroBiasUnits>(j.at("varianceGyroBiasUnits").get<int>());
    }
}

bool NAV::LooselyCoupledKF::initialize()
{
    LOG_TRACE("{}: called", nameId());

    kalmanFilter = KalmanFilter{ 15, 6 };

    kalmanFilter_Kz = Eigen::MatrixXd::Zero(15, 1);

    latestInertialNavSol = nullptr;

    double variance_angles = std::pow(trafo::deg2rad(10), 2); // [rad²]
    double variance_vel = std::pow(10, 2);                    // [m²/s²]

    double std_pos = 100;                                        // [m]
    double earthRadius = 6371e3;                                 // [m]
    double variance_LatLon = std::pow(std_pos / earthRadius, 2); // [rad²]

    // 𝐏 Error covariance matrix
    kalmanFilter.P.diagonal() << variance_angles, variance_angles, variance_angles, // Flight Angles covariance
        variance_vel, variance_vel, variance_vel,                                   // Velocity covariance
        variance_LatLon * 1e6, variance_LatLon * 1e6, std::pow(std_pos, 2),         // Position (Lat, Lon, Alt) covariance
        1e0, 1e0, 1e0,                                                              // Accelerometer Bias covariance
        1e-4, 1e-4, 1e-4;                                                           // Gyroscope Bias covariance

    LOG_DEBUG("P: \n{}", kalmanFilter.P);

    LOG_DEBUG("LooselyCoupledKF initialized");

    return true;
}

void NAV::LooselyCoupledKF::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

void NAV::LooselyCoupledKF::recvInertialNavigationSolution(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId /*linkId*/) // NOLINT(readability-convert-member-functions-to-static)
{
    auto inertialNavSol = std::dynamic_pointer_cast<InertialNavSol>(nodeData);

    if (latestInertialNavSol)
    {
        tau_KF = static_cast<double>((inertialNavSol->insTime.value() - latestInertialNavSol->insTime.value()).count());
    }

    latestInertialNavSol = inertialNavSol;

    looselyCoupledPrediction(inertialNavSol);
}

void NAV::LooselyCoupledKF::recvGNSSNavigationSolution(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId /*linkId*/)
{
    [[maybe_unused]] auto gnssMeasurement = std::dynamic_pointer_cast<PosVelAtt>(nodeData);

    if (latestInertialNavSol)
    {
        looselyCoupledUpdate(gnssMeasurement);
    }
}

// ###########################################################################################################
//                                               Kalman Filter
// ###########################################################################################################

void NAV::LooselyCoupledKF::looselyCoupledPrediction(const std::shared_ptr<InertialNavSol>& inertialNavSol)
{
    // ------------------------------------------- Data preparation ----------------------------------------------
    /// v_n (tₖ₋₁) Velocity in [m/s], in navigation coordinates, at the time tₖ₋₁
    const Eigen::Vector3d& velocity_n__t1 = inertialNavSol->velocity_n();
    /// Latitude 𝜙, longitude λ and altitude (height above ground) in [rad, rad, m] at the time tₖ₋₁
    const Eigen::Vector3d position_lla__t1 = inertialNavSol->latLonAlt();
    /// q (tₖ₋₁) Quaternion, from body to navigation coordinates, at the time tₖ₋₁
    const Eigen::Quaterniond& quaternion_nb__t1 = inertialNavSol->quaternion_nb();

    // Prime vertical radius of curvature (East/West) [m]
    const double R_E = NAV::earthRadius_E(position_lla__t1(0));
    // Meridian radius of curvature in [m]
    const double R_N = NAV::earthRadius_N(position_lla__t1(0));

    // Direction Cosine Matrix from body to navigation coordinates
    Eigen::Matrix3d DCM_nb = quaternion_nb__t1.toRotationMatrix();

    // Conversion matrix between cartesian and curvilinear perturbations to the position
    Eigen::Matrix3d T_rn_p = conversionMatrixCartesianCurvilinear(position_lla__t1, R_N, R_E);

    // Position and rotation information for conversion of IMU data from platform to body frame
    const auto& imuPosition = inertialNavSol->imuObs->imuPos;

    // a_p Acceleration in [m/s^2], in platform coordinates
    const Eigen::Vector3d& acceleration_p = inertialNavSol->imuObs->accelCompXYZ.has_value()
                                                ? inertialNavSol->imuObs->accelCompXYZ.value()
                                                : inertialNavSol->imuObs->accelUncompXYZ.value();
    Eigen::Vector3d acceleration_b = imuPosition.quatAccel_bp() * acceleration_p;

    // omega_in^n = omega_ie^n + omega_en^n
    Eigen::Vector3d angularRate_in_n = inertialNavSol->quaternion_ne() * InsConst::angularVelocity_ie_e
                                       + transportRate(position_lla__t1, velocity_n__t1, R_N, R_E);

    // Gauss-Markov constant for the accelerometer 𝛽 = 1 / 𝜏 (𝜏 correlation length) - Value from Jekeli (p. 183)
    Eigen::Vector3d beta_a;
    if (randomProcessAccel == RandomProcess::RandomWalk)
    {
        beta_a = Eigen::Vector3d::Zero();
    }
    else if (randomProcessAccel == RandomProcess::GaussMarkov1)
    {
        beta_a = beta_accel;
    }
    // Gauss-Markov constant for the gyroscope 𝛽 = 1 / 𝜏 (𝜏 correlation length) - Value from Jekeli (p. 183)
    Eigen::Vector3d beta_omega;
    if (randomProcessGyro == RandomProcess::RandomWalk)
    {
        beta_omega = Eigen::Vector3d::Zero();
    }
    else if (randomProcessGyro == RandomProcess::GaussMarkov1)
    {
        beta_omega = beta_gyro;
    }

    // ------------------------------------------- GUI Parameters ----------------------------------------------

    // 𝜎²_ra Variance of the noise on the accelerometer specific-force measurements [m²/s³]
    double sigma2_ra{};
    if (varianceAccelNoiseUnits == VarianceAccelNoiseUnits::mg_sqrtHz)
    {
        sigma2_ra = std::pow((variance_ra /* [mg/√(Hz)] */) * 1e-3 * InsConst::G_NORM, 2);
    }
    // 𝜎²_rg Variance of the noise on the gyro angular-rate measurements [deg²/s]
    double sigma2_rg{};
    if (varianceGyroNoiseUnits == VarianceGyroNoiseUnits::deg_hr_sqrtHz)
    {
        // See Woodman (2007) Chp. 3.2.2 - eq. 7 with seconds instead of hours.
        sigma2_rg = std::pow(1 / 3600.0 * (trafo::deg2rad(variance_rg /* [deg/hr/√(Hz)] */)), 2);
    }

    // ---------------------------------------------- Prediction -------------------------------------------------

    // System Matrix
    Eigen::Matrix<double, 15, 15> F = systemMatrixF(quaternion_nb__t1, acceleration_b, angularRate_in_n, velocity_n__t1, position_lla__t1, beta_a, beta_omega);

    if (phiCalculation == PhiCalculation::VanLoan)
    {
        // Noise Input Matrix
        Eigen::Matrix<double, 15, 6> G = noiseInputMatrixG(sigma2_ra, sigma2_rg, beta_a, beta_omega);

        // Power Spectral Density of u (See Brown & Hwang (2012) chapter 3.9, p. 126 - footnote)
        Eigen::Matrix<double, 6, 6> W = Eigen::Matrix<double, 6, 6>::Identity();

        // C.F. van Loan (1978) - Computing Integrals Involving the Matrix Exponential
        Eigen::Matrix<double, 30, 30> A = Eigen::Matrix<double, 30, 30>::Zero();
        A.block<15, 15>(0, 0) = -F;
        A.block<15, 15>(0, 15) = G * W * G.transpose();
        A.block<15, 15>(15, 15) = F.transpose();
        A *= tau_KF;

        // Exponential Matrix of A (https://eigen.tuxfamily.org/dox/unsupported/group__MatrixFunctions__Module.html#matrixbase_exp)
        Eigen::Matrix<double, 30, 30> B = A.exp();

        // 1. Calculate the transition matrix 𝚽_{k-1}
        kalmanFilter.Phi = B.block<15, 15>(15, 15).transpose();

        // 2. Calculate the system noise covariance matrix Q_{k-1}
        if (qCalculation == QCalculation::VanLoan)
        {
            kalmanFilter.Q = kalmanFilter.Phi * B.block<15, 15>(0, 15);
        }

        // LOG_DEBUG("A: \n{}", A);
        // LOG_DEBUG("B: \n{}", B);
        // LOG_DEBUG("G: \n{}", G);
    }
    else if (phiCalculation == PhiCalculation::Taylor1)
    {
        kalmanFilter.Phi = transitionMatrix(F, tau_KF);
    }
    else
    {
        LOG_CRITICAL("{}: Calculation algorithm '{}' for the system matrix Phi is not supported.", nameId(), phiCalculation);
    }
    notifyOutputValueChanged(OutputPortIndex_Phi);

    // 2. Calculate the system noise covariance matrix Q_{k-1}
    if (qCalculation == QCalculation::Groves)
    {
        // 𝜎²_bad Variance of the accelerometer dynamic bias
        double sigma2_bad{};
        if (varianceAccelBiasUnits == VarianceAccelBiasUnits::microg)
        {
            sigma2_bad = std::pow((variance_bad /* [µg] */) * 1e-6 * InsConst::G_NORM, 2);
        }
        // 𝜎²_bgd Variance of the gyro dynamic bias
        double sigma2_bgd{};
        if (varianceGyroBiasUnits == VarianceGyroBiasUnits::deg_h)
        {
            sigma2_bgd = std::pow((variance_bgd /* [°/h] */) / 3600.0, 2);
        }

        kalmanFilter.Q = systemNoiseCovarianceMatrix(sigma2_ra, sigma2_rg, sigma2_bad, sigma2_bgd,
                                                     systemMatrixF_21_n(quaternion_nb__t1, acceleration_b),
                                                     T_rn_p,
                                                     DCM_nb, tau_KF);
    }
    notifyOutputValueChanged(OutputPortIndex_Q);

    // 3. Propagate the state vector estimate from x(+) and x(-)
    // 4. Propagate the error covariance matrix from P(+) and P(-)
    kalmanFilter.predict();
    notifyOutputValueChanged(OutputPortIndex_x);
    notifyOutputValueChanged(OutputPortIndex_P);

    // Averaging of P to avoid numerical problems with symmetry (did not work)
    kalmanFilter.P = ((kalmanFilter.P + kalmanFilter.P.transpose()) / 2.0);

    // LOG_DEBUG("F: \n{}", F);
    // LOG_DEBUG("Phi: \n{}", kalmanFilter.Phi);

    // LOG_DEBUG("Q: \n{}", kalmanFilter.Q);
    // LOG_DEBUG("Q - Q^T: \n{}", kalmanFilter.Q - kalmanFilter.Q.transpose());

    // LOG_DEBUG("x: \n{}", kalmanFilter.x);

    // LOG_DEBUG("P: \n{}", kalmanFilter.P);
    // LOG_DEBUG("P - P^T: \n{}", kalmanFilter.P - kalmanFilter.P.transpose());

    // Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(kalmanFilter.P);
    // auto rank = lu_decomp.rank();
    // LOG_DEBUG("P.rank: {}", rank);
    // static int counter = 0;
    // counter++;
    // if (rank != 15)
    // {
    //     LOG_WARN("{}: P has rank {} now after predicting {} times.", nameId(), rank, counter);
    // }
}

void NAV::LooselyCoupledKF::looselyCoupledUpdate(const std::shared_ptr<PosVelAtt>& gnssMeasurement)
{
    // ------------------------------------------- Data preparation ----------------------------------------------
    // Latitude 𝜙, longitude λ and altitude (height above ground) in [rad, rad, m] at the time tₖ₋₁
    const Eigen::Vector3d position_lla__t1 = latestInertialNavSol->latLonAlt();
    // q (tₖ₋₁) Quaternion, from body to navigation coordinates, at the time tₖ₋₁
    const Eigen::Quaterniond& quaternion_nb__t1 = latestInertialNavSol->quaternion_nb();

    // Prime vertical radius of curvature (East/West) [m]
    const double R_E = NAV::earthRadius_E(position_lla__t1(0));
    // Meridian radius of curvature in [m]
    const double R_N = NAV::earthRadius_N(position_lla__t1(0));

    // Direction Cosine Matrix from body to navigation coordinates
    Eigen::Matrix3d DCM_nb = quaternion_nb__t1.toRotationMatrix();

    // Conversion matrix between cartesian and curvilinear perturbations to the position
    Eigen::Matrix3d T_rn_p = conversionMatrixCartesianCurvilinear(position_lla__t1, R_N, R_E);

    // Position and rotation information for conversion of IMU data from platform to body frame
    const auto& imuPosition = latestInertialNavSol->imuObs->imuPos;

    // Angular rate measured in units of [rad/s], and given in the platform frame
    const Eigen::Vector3d& angularRate_p = latestInertialNavSol->imuObs->gyroCompXYZ.has_value()
                                               ? latestInertialNavSol->imuObs->gyroCompXYZ.value()
                                               : latestInertialNavSol->imuObs->gyroUncompXYZ.value();
    // Angular rate measured in units of [rad/s], and given in the body frame
    auto angularRate_b = imuPosition.quatGyro_bp() * angularRate_p;

    // Skew-symmetric matrix of the Earth-rotation vector in local navigation frame axes
    Eigen::Matrix3d Omega_ie_n = AngularVelocityEarthSkew_ie_n(latestInertialNavSol->latitude());

    // ---------------------------------------------- Correction -------------------------------------------------
    // 5. Calculate the measurement matrix H_k
    kalmanFilter.H = measurementMatrix(T_rn_p, DCM_nb, angularRate_b, leverArm_InsGnss, Omega_ie_n);
    notifyOutputValueChanged(OutputPortIndex_H);

    // 6. Calculate the measurement noise covariance matrix R_k
    kalmanFilter.R = measurementNoiseCovariance(gnssSigmaSquaredLatLonAlt, gnssSigmaSquaredVelocity);
    notifyOutputValueChanged(OutputPortIndex_R);

    // 8. Formulate the measurement z_k
    kalmanFilter.z = measurementInnovation(gnssMeasurement->latLonAlt(), latestInertialNavSol->latLonAlt(),
                                           gnssMeasurement->velocity_n(), latestInertialNavSol->velocity_n(),
                                           T_rn_p, quaternion_nb__t1, leverArm_InsGnss, angularRate_b, Omega_ie_n);
    notifyOutputValueChanged(OutputPortIndex_z);

    // Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp3(kalmanFilter.H * kalmanFilter.P * kalmanFilter.H.transpose() + kalmanFilter.R);
    // auto rank3 = lu_decomp3.rank();
    // LOG_DEBUG("HPH^T + R.rank: {}", rank3);

    // 7. Calculate the Kalman gain matrix K_k
    // 9. Update the state vector estimate from x(-) to x(+)
    // 10. Update the error covariance matrix from P(-) to P(+)
    kalmanFilter.correctWithMeasurementInnovation();
    notifyOutputValueChanged(OutputPortIndex_K);
    notifyOutputValueChanged(OutputPortIndex_x);
    notifyOutputValueChanged(OutputPortIndex_P);

    kalmanFilter_Kz = kalmanFilter.K * kalmanFilter.z;
    notifyOutputValueChanged(OutputPortIndex_Kz);

    // Averaging of P to avoid numerical problems with symmetry (did not work)
    kalmanFilter.P = ((kalmanFilter.P + kalmanFilter.P.transpose()) / 2.0);

    // Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp1(kalmanFilter.H * kalmanFilter.P * kalmanFilter.H.transpose() + kalmanFilter.R);
    // auto rank1 = lu_decomp1.rank();
    // LOG_DEBUG("HPH^T + R.rank: {}", rank1);

    // Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp2(kalmanFilter.K);
    // auto rank2 = lu_decomp2.rank();
    // LOG_DEBUG("K.rank: {}", rank2);

    // LOG_DEBUG("H: \n{}", kalmanFilter.H);
    // LOG_DEBUG("R: \n{}", kalmanFilter.R);
    // LOG_DEBUG("z: \n{}", kalmanFilter.z);

    // LOG_DEBUG("K: \n{}", kalmanFilter.K);
    // LOG_DEBUG("x: \n{}", kalmanFilter.x);
    // LOG_DEBUG("P: \n{}", kalmanFilter.P);

    // LOG_DEBUG("K * z: \n{}", kalmanFilter.K * kalmanFilter.z);

    // LOG_DEBUG("P - P^T: \n{}", kalmanFilter.P - kalmanFilter.P.transpose());

    // Eigen::FullPivLU<Eigen::MatrixXd> decomp(kalmanFilter.P);
    // auto rank = decomp.rank();
    // LOG_DEBUG("P.rank: {}", rank);

    // Push out the new data
    auto pvaError = std::make_shared<PVAError>();
    pvaError->insTime = gnssMeasurement->insTime;
    pvaError->positionError_lla() = kalmanFilter.x.block<3, 1>(6, 0);
    pvaError->velocityError_n() = kalmanFilter.x.block<3, 1>(3, 0);
    pvaError->attitudeError_n() = kalmanFilter.x.block<3, 1>(0, 0);

    auto imuBiases = std::make_shared<ImuBiases>();
    imuBiases->insTime = gnssMeasurement->insTime;
    imuBiases->biasAccel_b = kalmanFilter.x.block<3, 1>(9, 0);
    imuBiases->biasGyro_b = kalmanFilter.x.block<3, 1>(12, 0);

    // Closed loop
    // kalmanFilter.x.block<9, 1>(0, 0).setZero();
    kalmanFilter.x.setZero();

    invokeCallbacks(OutputPortIndex_PVAError, pvaError);
    invokeCallbacks(OutputPortIndex_ImuBiases, imuBiases);
}

// ###########################################################################################################
//                                           Transition matrix 𝚽
// ###########################################################################################################

Eigen::MatrixXd NAV::LooselyCoupledKF::transitionMatrix(const Eigen::MatrixXd& F, double tau_s)
{
    // Transition matrix 𝚽
    // Math: \mathbf{\Phi}_{INS}^n \approx \begin{bmatrix} \mathbf{I}_3 + \mathbf{F}_{11}^n \mathbf{\tau}_s & \mathbf{F}_{12}^n \mathbf{\tau}_s & \mathbf{F}_{13}^n \mathbf{\tau}_s & \mathbf{0}_3 & \mathbf{\hat{C}}_b^n \mathbf{\tau}_s \\ \mathbf{F}_{21}^n \mathbf{\tau}_s & \mathbf{I}_3 + \mathbf{F}_{22}^n \mathbf{\tau}_s & \mathbf{F}_{23}'^n \mathbf{\tau}_s & \mathbf{\hat{C}}_b^n \mathbf{\tau}_s & \mathbf{0}_3 \\ \mathbf{0}_3 & \mathbf{F}_{32}^n \mathbf{\tau}_s & \mathbf{I}_3 + \mathbf{F}_{33}^n \mathbf{\tau}_s & \mathbf{0}_3 & \mathbf{0}_3 \\ \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{I}_3 & \mathbf{0}_3 \\ \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{I}_3 \end{bmatrix} \qquad \text{P. Groves}\,(14.72)
    return Eigen::MatrixXd::Identity(15, 15) + F * tau_s;
}

Eigen::Matrix<double, 15, 15> NAV::LooselyCoupledKF::systemMatrixF(const Eigen::Quaterniond& quaternion_nb, const Eigen::Vector3d& specForce_ib_b, const Eigen::Vector3d& angularRate_in_n, const Eigen::Vector3d& velocity_n, const Eigen::Vector3d& position_lla, const Eigen::Vector3d& beta_a, const Eigen::Vector3d& beta_omega)
{
    // System matrix 𝐅
    // Math: \mathbf{F}_{INS}^n = \begin{pmatrix} \mathbf{F}_{11}^n & \mathbf{F}_{12}^n & \mathbf{F}_{13}^n & \mathbf{0}_3 & \mathbf{\hat{C}}_b^n \\ \mathbf{F}_{21}^n & \mathbf{F}_{22}^n & \mathbf{F}_{23}^n & \mathbf{\hat{C}}_b^n & \mathbf{0}_3 \\ \mathbf{0}_3 & \mathbf{F}_{32}^n & \mathbf{F}_{33}^n & \mathbf{0}_3 & \mathbf{0}_3 \\ \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 \\ \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 \end{pmatrix} \qquad \text{P. Groves}\,(14.63)
    Eigen::MatrixXd F = Eigen::MatrixXd::Zero(15, 15);

    F.block<3, 3>(0, 0) = systemMatrixF_11_n(angularRate_in_n);
    F.block<3, 3>(0, 3) = systemMatrixF_12_n(position_lla(0), position_lla(2));
    F.block<3, 3>(0, 6) = systemMatrixF_13_n(position_lla(0), position_lla(2), velocity_n);
    F.block<3, 3>(0, 12) = -quaternion_nb.toRotationMatrix(); // Sign from T. Hobiger (2021) Inertialnavigation V07 - equation (7.22)
    F.block<3, 3>(3, 0) = systemMatrixF_21_n(quaternion_nb, specForce_ib_b);
    F.block<3, 3>(3, 3) = systemMatrixF_22_n(velocity_n, position_lla(0), position_lla(2));
    F.block<3, 3>(3, 6) = systemMatrixF_23_n(velocity_n, position_lla(0), position_lla(2));
    F.block<3, 3>(3, 9) = quaternion_nb.toRotationMatrix();
    F.block<3, 3>(6, 3) = systemMatrixF_32_n(position_lla(0), position_lla(2));
    F.block<3, 3>(6, 6) = systemMatrixF_33_n(velocity_n, position_lla(0), position_lla(2));
    F.block<3, 3>(9, 9) = systemMatrixF_44_n(beta_a);
    F.block<3, 3>(12, 12) = systemMatrixF_55_n(beta_omega);

    // Conversion because state vector has milliradians for latitude and longitude
    constexpr double mrad2rad = 1e-3;
    // Conversion because state vector has milliradians for latitude and longitude
    constexpr double rad2mrad = 1e3;

    F.block<3, 1>(0, 6) *= mrad2rad; // F_13 first column gets multiplied by δϕ (Lat errors) [mrad] which needs to be scaled into rad
    // F.block<3, 1>(0, 7) *= mrad2rad; // F_13 second column gets multiplied by δλ (Lon errors) [mrad] - column contains only zeroes

    F.block<3, 1>(3, 6) *= mrad2rad; // F_23 first column gets multiplied by δϕ (Lat errors) [mrad] which needs to be scaled into rad
    // F.block<3, 1>(3, 7) *= mrad2rad; // F_23 second column gets multiplied by δλ (Lon errors) [mrad] - column contains only zeroes

    F.block<1, 3>(6, 3) *= rad2mrad; // F_32 first row δϕ' (Lat errors time derivative) gets multiplied with velocity and therefore needs to be scaled into mrad
    F.block<1, 3>(7, 3) *= rad2mrad; // F_32 second row δλ' (Lon errors time derivative) gets multiplied with velocity and therefore needs to be scaled into mrad

    F.block<2, 1>(6, 8) *= rad2mrad; // F33 third column δh, first and second row δϕ', δλ' - Altitude error needs to be scaled into mrad

    return F;
}

Eigen::Matrix3d NAV::LooselyCoupledKF::systemMatrixF_11_n(const Eigen::Vector3d& angularRate_in_n)
{
    // Math: \mathbf{F}_{11}^n = -[\mathbf{\hat{\omega}}_{in}^n \land] \qquad \text{P. Groves}\,(14.64)
    return -skewSymmetricMatrix(angularRate_in_n);
}

Eigen::Matrix3d NAV::LooselyCoupledKF::systemMatrixF_12_n(double latitude_b, double height_b)
{
    // Math: \mathbf{F}_{12}^n = \begin{bmatrix} 0 & \frac{-1}{R_E(\hat{L}_b) + \hat{h}_b} & 0 \\ \frac{1}{R_N(\hat{L}_b) + \hat{h}_b} & 0 & 0 \\ 0 & \frac{\tan{\hat{L}_b}}{R_E(\hat{L}_b) + \hat{h}_b} & 0 \end{bmatrix} \qquad \text{P. Groves}\,(14.65)
    Eigen::Matrix3d F_12_n = Eigen::Matrix3d::Zero(3, 3);
    double R_E = NAV::earthRadius_E(latitude_b);
    double R_N = NAV::earthRadius_N(latitude_b);

    F_12_n(0, 1) = 1 / (R_E + height_b);
    F_12_n(1, 0) = -1 / (R_N + height_b);
    F_12_n(2, 1) = -std::tan(latitude_b) / (R_E + height_b);

    return F_12_n;
}

Eigen::Matrix3d NAV::LooselyCoupledKF::systemMatrixF_13_n(double latitude_b, double height_b, const Eigen::Vector3d& v_eb_n)
{
    // Math: \mathbf{F}_{13}^n = \begin{bmatrix} \omega_{ie}\sin{\hat{L}_b} & 0 & \frac{\hat{v}_{eb,E}^n}{(R_E(\hat{L}_b) + \hat{h}_b)^2} \\ 0 & 0 & \frac{-\hat{v}_{eb,N}^n}{(R_N(\hat{L}_b) + \hat{h}_b)^2} \\ \omega_{ie}\cos{\hat{L}_b} + \frac{\hat{v}_{eb,E}^n}{(R_E(\hat{L}_b) + \hat{h}_b)\cos^2{\hat{L}_b}} & 0 & \frac{-\hat{v}_{eb,E}^n\tan{\hat{L}_b}}{(R_E(\hat{L}_b) + \hat{h}_b)^2} \end{bmatrix} \qquad \text{P. Groves}\,(14.66)
    Eigen::Matrix3d F_13_n = Eigen::Matrix3d::Zero(3, 3);
    double R_E = NAV::earthRadius_E(latitude_b);
    double R_N = NAV::earthRadius_N(latitude_b);

    F_13_n(0, 0) = -InsConst::angularVelocity_ie * std::sin(latitude_b);
    F_13_n(0, 2) = -v_eb_n(1) / std::pow((R_E + height_b), 2.0);
    F_13_n(1, 2) = v_eb_n(0) / std::pow((R_N + height_b), 2.0);
    F_13_n(2, 0) = -InsConst::angularVelocity_ie * std::cos(latitude_b) - v_eb_n(1) / ((R_E + height_b) * std::cos(latitude_b) * std::cos(latitude_b));
    F_13_n(2, 2) = v_eb_n(1) * std::tan(latitude_b) / std::pow((R_E + height_b), 2.0);

    return F_13_n;
}

Eigen::Matrix3d NAV::LooselyCoupledKF::systemMatrixF_21_n(const Eigen::Quaterniond& quaternion_nb, const Eigen::Vector3d& specForce_ib_b)
{
    // Math: \mathbf{F}_{21}^n = -\begin{bmatrix} (\mathbf{\hat{C}}_{b}^n \hat{f}_{ib}^b) \land \end{bmatrix} \qquad \text{P. Groves}\,(14.67)
    return skewSymmetricMatrix(quaternion_nb * specForce_ib_b);
}

Eigen::Matrix3d NAV::LooselyCoupledKF::systemMatrixF_22_n(const Eigen::Vector3d& v_eb_n, double latitude_b, double height_b)
{
    // Math: \mathbf{F}_{22}^n = \begin{bmatrix} \frac{\hat{v}_{eb,D}^n}{R_N(\hat{L}_b)+\hat{h}_b} & -\frac{2\hat{v}_{eb,E}^n\tan{\hat{L}}_b}{R_E(\hat{L}_b)+\hat{h}_b}-2\omega_{ie}\sin{\hat{L}_b} & \frac{\hat{v}_{eb,N}^n}{R_N(\hat{L}_b)+\hat{h}_b} \\ \frac{\hat{v}_{eb,E}^n\tan{\hat{L}}_b}{R_E(\hat{L}_b)+\hat{h}_b}+2\omega_{ie}\sin{\hat{L}_b} & \frac{\hat{v}_{eb,N}^n\tan{\hat{L}}_b+\hat{v}_{eb,D}^n}{R_E(\hat{L}_b)+\hat{h}_b} & \frac{\hat{v}_{eb,E}^n}{R_E(\hat{L}_b)+\hat{h}_b}+2\omega_{ie}\cos{\hat{L}_b} \\ -\frac{2\hat{v}_{eb,N}^n}{R_N(\hat{L}_b)+\hat{h}_b} & -\frac{2\hat{v}_{eb,E}^n}{R_E(\hat{L}_b)+\hat{h}_b}-2\omega_{ie}\cos{\hat{L}_b} & 0 \end{bmatrix} \qquad \text{P. Groves}\,(14.68)
    Eigen::Matrix3d F_22_n = Eigen::Matrix3d::Zero(3, 3);
    double R_E = NAV::earthRadius_E(latitude_b);
    double R_N = NAV::earthRadius_N(latitude_b);

    F_22_n(0, 0) = v_eb_n(2) / (R_N + height_b);
    F_22_n(0, 1) = -2 * v_eb_n(1) * std::tan(latitude_b) / (R_E + height_b) - 2.0 * InsConst::angularVelocity_ie * std::sin(latitude_b);
    F_22_n(0, 2) = v_eb_n(0) / (R_N + height_b);
    F_22_n(1, 0) = v_eb_n(1) * std::tan(latitude_b) / (R_E + height_b) + 2.0 * InsConst::angularVelocity_ie * std::sin(latitude_b);
    F_22_n(1, 1) = (v_eb_n(0) * std::tan(latitude_b) + v_eb_n(2)) / (R_E + height_b);
    F_22_n(1, 2) = v_eb_n(1) / (R_E + height_b) + 2.0 * InsConst::angularVelocity_ie * std::cos(latitude_b);
    F_22_n(2, 0) = -2.0 * v_eb_n(0) / (R_N + height_b);
    F_22_n(2, 1) = -2.0 * v_eb_n(1) / (R_E + height_b) - 2.0 * InsConst::angularVelocity_ie * std::cos(latitude_b);

    return F_22_n;
}

Eigen::Matrix3d NAV::LooselyCoupledKF::systemMatrixF_23_n(const Eigen::Vector3d& v_eb_n, double latitude_b, double height_b)
{
    // Math: \mathbf{F}_{23}^n = \begin{bmatrix} -\frac{(\hat{v}_{eb,E}^n)^2\sec^2{\hat{L}_b}}{R_E(\hat{L}_b)+\hat{h}_b}-2\hat{v}_{eb,E}^n\omega_{ie}\cos{\hat{L}_b} & 0 & \frac{(\hat{v}_{eb,E}^n)^2\tan{\hat{L}_b}}{(R_E(\hat{L}_b)+\hat{h}_b)^2}-\frac{\hat{v}_{eb,N}^n\hat{v}_{eb,D}^n}{(R_N(\hat{L}_b)+\hat{h}_b)^2} \\ \frac{\hat{v}_{eb,N}^n\hat{v}_{eb,E}^n\sec^2{\hat{L}_b}}{R_E(\hat{L}_b)+\hat{h}_b}+2\hat{v}_{eb,N}^n\omega_{ie}\cos{\hat{L}_b}-2\hat{v}_{eb,D}^n\omega_{ie}\sin{\hat{L}_b} & 0 & -\frac{\hat{v}_{eb,N}^n\hat{v}_{eb,E}^n\tan{\hat{L}_b}+\hat{v}_{eb,E}^n\hat{v}_{eb,D}^n}{(R_E(\hat{L}_b)+\hat{h}_b)^2} \\ 2\hat{v}_{eb,E}^n\omega_{ie}\sin{\hat{L}_b} & 0 & \frac{(\hat{v}_{eb,E}^n)^2}{(R_E(\hat{L}_b)+\hat{h}_b)^2}+\frac{(\hat{v}_{eb,N}^n)^2}{(R_N(\hat{L}_b)+\hat{h}_b)^2}-\frac{2g_0(\hat{L}_b)}{r_{eS}^e(\hat{L}_b)} \end{bmatrix} \qquad \text{P. Groves}\,(14.69)
    Eigen::Matrix3d F_23_n = Eigen::Matrix3d::Zero(3, 3);
    double R_E = NAV::earthRadius_E(latitude_b);
    double R_N = NAV::earthRadius_N(latitude_b);
    // Magnitude of gravity vector at ellipsoid height in [m / s^2]
    double g_0 = NAV::gravity::gravityMagnitude_SomiglianaAltitude(latitude_b, 0); //TODO: Split calculation into latitude and altitude part to save time (--> InsGravity)
    // Geocentric Radius in [m]
    double r_eS_e = geocentricRadius(latitude_b, R_E, InsConst::WGS84_e_squared);

    F_23_n(0, 0) = -(v_eb_n(1) * v_eb_n(1) * std::pow(secant(latitude_b), 2.0) / (R_E + height_b))
                   - 2.0 * v_eb_n(1) * InsConst::angularVelocity_ie * std::cos(latitude_b);

    F_23_n(0, 2) = (v_eb_n(1) * v_eb_n(1) * std::tan(latitude_b)) / std::pow(R_E + height_b, 2.0)
                   - (v_eb_n(0) * v_eb_n(2)) / std::pow(R_N + height_b, 2.0);

    F_23_n(1, 0) = v_eb_n(0) * v_eb_n(1) * std::pow(secant(latitude_b), 2.0) / (R_E + height_b)
                   + 2.0 * v_eb_n(0) * InsConst::angularVelocity_ie * std::cos(latitude_b)
                   - 2.0 * v_eb_n(2) * InsConst::angularVelocity_ie * std::sin(latitude_b);

    F_23_n(1, 2) = -(v_eb_n(0) * v_eb_n(1) * std::tan(latitude_b) + v_eb_n(1) * v_eb_n(2)) / std::pow(R_E + height_b, 2.0);

    F_23_n(2, 0) = 2.0 * v_eb_n(1) * InsConst::angularVelocity_ie * std::sin(latitude_b);

    F_23_n(2, 2) = (v_eb_n(1) * v_eb_n(1)) / std::pow(R_E + height_b, 2.0)
                   + (v_eb_n(0) * v_eb_n(0)) / std::pow(R_N + height_b, 2.0)
                   - 2.0 * g_0 / r_eS_e;

    return F_23_n;
}

Eigen::Matrix3d NAV::LooselyCoupledKF::systemMatrixF_32_n(double latitude_b, double height_b)
{
    // Math: \mathbf{F}_{32}^n = \begin{bmatrix} \frac{1}{R_N(\hat{L}_b) + \hat{h}_b} & 0 & 0 \\ 0 & \frac{1}{(R_E(\hat{L}_b) + \hat{h}_b)\cos{\hat{L}_b}} & 0 \\ 0 & 0 & -1 \end{bmatrix} \quad \text{P. Groves}\,(14.70)
    double R_E = NAV::earthRadius_E(latitude_b);
    double R_N = NAV::earthRadius_N(latitude_b);

    Eigen::DiagonalMatrix<double, 3> m(1.0 / (R_N + height_b),
                                       1.0 / ((R_E + height_b) * std::cos(latitude_b)),
                                       -1);
    return m;
}

Eigen::Matrix3d NAV::LooselyCoupledKF::systemMatrixF_33_n(const Eigen::Vector3d& v_eb_n, double latitude_b, double height_b)
{
    // Math: \mathbf{F}_{33}^n = \begin{bmatrix} 0 & 0 & -\frac{\hat{v}_{eb,N}^n}{(R_N(\hat{L}_b) + \hat{h}_b)^2} \\ \frac{\hat{v}_{eb,E}^n \sin{\hat{L}_b}}{(R_E(\hat{L}_b) + \hat{h}_b) \cos^2{\hat{L}_b}} & 0 & -\frac{\hat{v}_{eb,E}^n}{(R_N(\hat{L}_b) + \hat{h}_b)^2 \cos^2{\hat{L}_b}} \\ 0 & 0 & 0 \end{bmatrix} \quad \text{P. Groves}\,(14.71)
    Eigen::Matrix3d F_33_n = Eigen::Matrix3d::Zero(3, 3);
    double R_E = NAV::earthRadius_E(latitude_b);
    double R_N = NAV::earthRadius_N(latitude_b);

    F_33_n(0, 2) = -v_eb_n(0) / std::pow(R_N + height_b, 2.0);
    F_33_n(1, 0) = v_eb_n(1) * std::sin(latitude_b) / ((R_E + height_b) * std::pow(std::cos(latitude_b), 2.0));
    F_33_n(1, 2) = -v_eb_n(1) / (std::pow(R_E + height_b, 2.0) * std::cos(latitude_b));

    return F_33_n;
}

Eigen::Matrix3d NAV::LooselyCoupledKF::systemMatrixF_44_n(const Eigen::Vector3d& beta_a)
{
    // TODO: Math
    return -1 * Eigen::DiagonalMatrix<double, 3>{ beta_a };
}

Eigen::Matrix3d NAV::LooselyCoupledKF::systemMatrixF_55_n(const Eigen::Vector3d& beta_omega)
{
    // TODO: Math
    return -1 * Eigen::DiagonalMatrix<double, 3>{ beta_omega };
}

// ###########################################################################################################
//                                           Noise input matrix 𝐆
// ###########################################################################################################

Eigen::Matrix<double, 15, 6> NAV::LooselyCoupledKF::noiseInputMatrixG(const double& sigma2_ra, const double& sigma2_rg, const Eigen::Vector3d& beta_a, const Eigen::Vector3d& beta_omega)
{
    Eigen::Matrix<double, 15, 6> G = Eigen::Matrix<double, 15, 6>::Zero();

    G.block<3, 3>(9, 0) = noiseInputMatrixG_a(sigma2_ra, beta_a);
    G.block<3, 3>(12, 3) = noiseInputMatrixG_omega(sigma2_rg, beta_omega);

    return G;
}

Eigen::Matrix3d NAV::LooselyCoupledKF::noiseInputMatrixG_a(const double& sigma2_ra, const Eigen::Vector3d& beta_a)
{
    if (randomProcessAccel == RandomProcess::RandomWalk) // Random walk (beta = 0)
    {
        return Eigen::DiagonalMatrix<double, 3>{ std::sqrt(sigma2_ra), std::sqrt(sigma2_ra), std::sqrt(sigma2_ra) };
    }
    else // if (randomProcessAccel == RandomProcess::GaussMarkov1)
    {
        return Eigen::DiagonalMatrix<double, 3>{ (beta_a.array() * sigma2_ra).sqrt().matrix() };
    }
}

Eigen::Matrix3d NAV::LooselyCoupledKF::noiseInputMatrixG_omega(const double& sigma2_rg, const Eigen::Vector3d& beta_omega)
{
    if (randomProcessGyro == RandomProcess::RandomWalk) // Random walk (beta = 0)
    {
        return Eigen::DiagonalMatrix<double, 3>{ std::sqrt(sigma2_rg), std::sqrt(sigma2_rg), std::sqrt(sigma2_rg) };
    }
    else // if (randomProcessGyro == RandomProcess::GaussMarkov1)
    {
        return Eigen::DiagonalMatrix<double, 3>{ (beta_omega.array() * sigma2_rg).sqrt().matrix() };
    }
}

// ###########################################################################################################
//                                     System noise covariance matrix 𝐐
// ###########################################################################################################

Eigen::Matrix<double, 15, 15> NAV::LooselyCoupledKF::systemNoiseCovarianceMatrix(const double& sigma2_ra, const double& sigma2_rg, const double& sigma2_bad, const double& sigma2_bgd, const Eigen::Matrix3d& F_21_n, const Eigen::Matrix3d& T_rn_p, const Eigen::Matrix3d& DCM_nb, const double& tau_s)
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

    return Q;
}

double NAV::LooselyCoupledKF::psdGyroNoise(const double& sigma2_ra, const double& tau_i)
{
    // Math: S_{ra} = \sigma_{ra}^2\tau_i \qquad \text{P. Groves}\,(14.83)
    return sigma2_ra * tau_i;
}

double NAV::LooselyCoupledKF::psdAccelNoise(const double& sigma2_rg, const double& tau_i)
{
    // Math: S_{rg} = \sigma_{rg}^2\tau_i \qquad \text{P. Groves}\,(14.83)
    return sigma2_rg * tau_i;
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

// ###########################################################################################################
//                                                Correction
// ###########################################################################################################

Eigen::Matrix<double, 6, 15> NAV::LooselyCoupledKF::measurementMatrix(const Eigen::Matrix3d& T_rn_p, const Eigen::Matrix3d& DCM_nb, const Eigen::Vector3d& angularRate_ib_b, const Eigen::Vector3d& leverArm_InsGnss, const Eigen::Matrix3d& Omega_ie_n)
{
    // Scale factor to scale rad to milliradians
    Eigen::Matrix3d S_p = Eigen::DiagonalMatrix<double, 3>{ 1e3, 1e3, 1 };

    // Math: \mathbf{H}_{G,k}^n = \begin{pmatrix} \mathbf{H}_{r1}^n & \mathbf{0}_3 & -\mathbf{I}_3 & \mathbf{0}_3 & \mathbf{0}_3 \\ \mathbf{H}_{v1}^n & -\mathbf{I}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{H}_{v5}^n \end{pmatrix}_k \qquad \text{P. Groves}\,(14.113)
    // G denotes GNSS indicated
    Eigen::Matrix<double, 6, 15> H = Eigen::Matrix<double, 6, 15>::Zero();
    H.block<3, 3>(0, 0) = S_p * measurementMatrix_r1_n(T_rn_p, DCM_nb, leverArm_InsGnss);
    H.block<3, 3>(0, 6) = S_p * -Eigen::Matrix3d::Identity();
    H.block<3, 3>(3, 0) = measurementMatrix_v1_n(DCM_nb, angularRate_ib_b, leverArm_InsGnss, Omega_ie_n);
    H.block<3, 3>(3, 3) = -Eigen::Matrix3d::Identity();
    H.block<3, 3>(3, 12) = measurementMatrix_v5_n(DCM_nb, leverArm_InsGnss);

    return H;
}

Eigen::Matrix3d NAV::LooselyCoupledKF::measurementMatrix_r1_n(const Eigen::Matrix3d& T_rn_p, const Eigen::Matrix3d& DCM_nb, const Eigen::Vector3d& leverArm_InsGnss)
{
    // Math: \mathbf{H}_{r1}^n \approx \mathbf{\hat{T}}_{r(n)}^p \begin{bmatrix} \begin{pmatrix} \mathbf{\hat{C}}_b^n \mathbf{l}_{ba}^b \end{pmatrix} \wedge \end{bmatrix} \qquad \text{P. Groves}\,(14.114)
    Eigen::Vector3d product = DCM_nb * leverArm_InsGnss;
    return T_rn_p * skewSymmetricMatrix(product);
}

Eigen::Matrix3d NAV::LooselyCoupledKF::measurementMatrix_v1_n(const Eigen::Matrix3d& DCM_nb, const Eigen::Vector3d& angularRate_ib_b, const Eigen::Vector3d& leverArm_InsGnss, const Eigen::Matrix3d& Omega_ie_n)
{
    // Math: \mathbf{H}_{v1}^n \approx \begin{bmatrix} \begin{Bmatrix} \mathbf{\hat{C}}_b^n (\mathbf{\hat{\omega}}_{ib}^b \wedge \mathbf{l}_{ba}^b) - \mathbf{\hat{\Omega}}_{ie}^n\mathbf{\hat{C}}_b^n \mathbf{l}_{ba}^b \end{Bmatrix} \wedge \end{bmatrix} \qquad \text{P. Groves}\,(14.114)
    Eigen::Vector3d product = DCM_nb * (angularRate_ib_b.cross(leverArm_InsGnss)) - Omega_ie_n * DCM_nb * leverArm_InsGnss;

    return skewSymmetricMatrix(product);
}

Eigen::Matrix3d NAV::LooselyCoupledKF::measurementMatrix_v5_n(const Eigen::Matrix3d& DCM_nb, const Eigen::Vector3d& leverArm_InsGnss)
{
    // Math: \mathbf{\hat{C}}_b^n \begin{bmatrix} \mathbf{l}_{ba}^b \wedge \end{bmatrix}
    return DCM_nb * skewSymmetricMatrix(leverArm_InsGnss);
}

Eigen::Matrix<double, 6, 6> NAV::LooselyCoupledKF::measurementNoiseCovariance(const Eigen::Vector3d& gnssVarianceLatLonAlt, const Eigen::Vector3d& gnssVarianceVelocity)
{
    Eigen::Matrix<double, 6, 6> R = Eigen::Matrix<double, 6, 6>::Zero();
    R.diagonal().block<3, 1>(0, 0) = gnssVarianceLatLonAlt;
    R.diagonal().block<3, 1>(3, 0) = gnssVarianceVelocity;

    return R;
}

Eigen::Matrix<double, 6, 1> NAV::LooselyCoupledKF::measurementInnovation(const Eigen::Vector3d& positionMeasurement_lla, const Eigen::Vector3d& positionEstimate_n,
                                                                         const Eigen::Vector3d& velocityMeasurement_n, const Eigen::Vector3d& velocityEstimate_n,
                                                                         const Eigen::Matrix3d& T_rn_p, const Eigen::Quaterniond& q_nb, const Eigen::Vector3d& leverArm_InsGnss,
                                                                         const Eigen::Vector3d& angularRate_ib_b, const Eigen::Matrix3d& Omega_ie_n)
{
    // Scale factor to scale rad to milliradians
    Eigen::Matrix3d S_p = Eigen::DiagonalMatrix<double, 3>{ 1e3, 1e3, 1 };

    Eigen::Vector3d deltaLLA = S_p * (positionMeasurement_lla - positionEstimate_n - T_rn_p * (q_nb * leverArm_InsGnss));
    Eigen::Vector3d deltaVel = velocityMeasurement_n - velocityEstimate_n - q_nb * (angularRate_ib_b.cross(leverArm_InsGnss)) + Omega_ie_n * (q_nb * leverArm_InsGnss);

    Eigen::Matrix<double, 6, 1> innovation;
    innovation << deltaLLA, deltaVel;

    return innovation;
}