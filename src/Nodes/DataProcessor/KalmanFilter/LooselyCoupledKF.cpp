#include "LooselyCoupledKF.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>

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

    hasConfig = false;

    nm::CreateInputPin(this, "InertialNavigationSolution", Pin::Type::Flow, { NAV::PosVelAtt::type() }, &LooselyCoupledKF::recvInertialNavigationSolution);
    nm::CreateInputPin(this, "GNSSNavigationSolution", Pin::Type::Flow, { NAV::PosVelAtt::type() }, &LooselyCoupledKF::recvGNSSNavigationSolution);
    nm::CreateOutputPin(this, "PosVelAtt", Pin::Type::Flow, NAV::PosVelAtt::type());
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
}

[[nodiscard]] json NAV::LooselyCoupledKF::save() const
{
    LOG_TRACE("{}: called", nameId());

    // TODO: save, once there is something to save

    json j;

    j["LC KF somthing to save"] = "something to save";

    return j;
}

void NAV::LooselyCoupledKF::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    // TODO: restore, once there is something to restore

    LOG_DATA("restored j is {}", j);
}

bool NAV::LooselyCoupledKF::initialize()
{
    LOG_TRACE("{}: called", nameId());

    // posVelAtt__t1 = nullptr;

    LOG_DEBUG("LooselyCoupledKF initialized");

    return true;
}

void NAV::LooselyCoupledKF::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

void NAV::LooselyCoupledKF::recvInertialNavigationSolution(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId /*linkId*/) // NOLINT(readability-convert-member-functions-to-static)
{
    // TODO: cast nodeData as a dynamic pointer to state observation
    LOG_DATA("NodeData received: {}", nodeData);

    auto posVelAttMeasurement = std::dynamic_pointer_cast<PosVelAtt>(nodeData);

    if (!posVelAtt.has_value())
    {
        posVelAtt = posVelAttMeasurement;
    }

    // TODO: Use posVelAttMeasurement in filterObservation for the Update

    filterObservation();
}

void NAV::LooselyCoupledKF::recvGNSSNavigationSolution(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId /*linkId*/)
{
    [[maybe_unused]] auto posVelMeasurement = std::dynamic_pointer_cast<PosVelAtt>(nodeData);

    // TODO: Can we initialize this from GNSS only (without attitude)?
    // if (!posVelAtt.has_value())
    // {
    //     posVelAtt = posVelMeasurement;
    // }

    if (posVelAtt.has_value())
    {
        filterObservation();
    }

    // TODO: Use posVelAttMeasurement in filterObservation for the Update
}

void NAV::LooselyCoupledKF::filterObservation()
{
    // Data preparation ----------------------------------------------------------------------------------------
    /// v_n (tₖ₋₁) Velocity in [m/s], in navigation coordinates, at the time tₖ₋₁
    const Eigen::Vector3d& velocity_n__t1 = posVelAtt->velocity_n();
    /// Latitude 𝜙, longitude λ and altitude (height above ground) in [rad, rad, m] at the time tₖ₋₁
    const Eigen::Vector3d position_lla__t1 = posVelAtt->latLonAlt();
    /// q (tₖ₋₁) Quaternion, from body to navigation coordinates, at the time tₖ₋₁
    const Eigen::Quaterniond& quaternion_nb__t1 = posVelAtt->quaternion_nb();

    // Prediction ----------------------------------------------------------------------------------------------
    // 1. Calculate the transition matrix 𝚽_{k-1}
    Eigen::MatrixXd Phi = transitionMatrix(quaternion_nb__t1, specForce_ib_b, velocity_n__t1, position_lla__t1, 0.01); //TODO: Make dynamically adaptable update rate for KF

    // 2. Calculate the system noise covariance matrix Q_{k-1}
    // 3. Propagate the state vector estimate from x(+) and x(-)
    // 4. Propagate the error covariance matrix from P(+) and P(-)
    // Correction ----------------------------------------------------------------------------------------------
    // 5. Calculate the measurement matrix H_k
    // 6. Calculate the measurement noise covariance matrix R_k
    // 7. Calculate the Kalman gain matrix K_k
    // 8. Formulate the measurement z_k
    // 9. Update the state vector estimate from x(-) to x(+)
    // 10. Update the error covariance matrix from P(-) to P(+)

    // TODO: Reset the data ports

    // Push out the new data
    // invokeCallbacks(OutputPortIndex_PosVelAtt__t0, posVelAtt__t1);
}

Eigen::MatrixXd NAV::LooselyCoupledKF::transitionMatrix(const Eigen::Quaterniond& quaternion_nb, const Eigen::Vector3d& acceleration_ib_b, const Eigen::Vector3d& velocity_n, const Eigen::Vector3d& position_lla, double tau_s)
{
    Eigen::Vector3d angularRate_in_n(InsConst::angularVelocity_ie * std::cos(position_lla(0)),
                                     0,
                                     -InsConst::angularVelocity_ie * std::sin(position_lla(0)));

    // System matrix 𝐅
    // Math: \mathbf{F}_{INS}^n = \begin{pmatrix} \mathbf{F}_{11}^n & \mathbf{F}_{12}^n & \mathbf{F}_{13}^n & \mathbf{0}_3 & \mathbf{\hat{C}}_b^n \\ \mathbf{F}_{21}^n & \mathbf{F}_{22}^n & \mathbf{F}_{23}^n & \mathbf{\hat{C}}_b^n & \mathbf{0}_3 \\ \mathbf{0}_3 & \mathbf{F}_{32}^n & \mathbf{F}_{33}^n & \mathbf{0}_3 & \mathbf{0}_3 \\ \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 \\ \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 \end{pmatrix} \qquad \text{P. Groves}\,(14.63)
    Eigen::MatrixXd F = Eigen::MatrixXd::Zero(15, 15);

    F.block<3, 3>(0, 0) = systemMatrixF_11_n(angularRate_in_n);
    F.block<3, 3>(0, 3) = systemMatrixF_12_n(position_lla(0), position_lla(2));
    F.block<3, 3>(0, 6) = systemMatrixF_13_n(position_lla(0), position_lla(2), velocity_n);
    F.block<3, 3>(0, 12) = quaternion_nb.toRotationMatrix();
    F.block<3, 3>(3, 0) = systemMatrixF_21_n(quaternion_nb, acceleration_ib_b);
    F.block<3, 3>(3, 3) = systemMatrixF_22_n(velocity_n, position_lla(0), position_lla(2));
    F.block<3, 3>(3, 6) = systemMatrixF_23_n(velocity_n, position_lla(0), position_lla(2));
    F.block<3, 3>(3, 9) = quaternion_nb.toRotationMatrix();
    F.block<3, 3>(6, 3) = systemMatrixF_32_n(position_lla(0), position_lla(2));
    F.block<3, 3>(6, 6) = systemMatrixF_33_n(velocity_n, position_lla(0), position_lla(2));

    // Transition matrix 𝚽
    // Math: \mathbf{\Phi}_{INS}^n \approx \begin{bmatrix} \mathbf{I}_3 + \mathbf{F}_{11}^n \mathbf{\tau}_s & \mathbf{F}_{12}^n \mathbf{\tau}_s & \mathbf{F}_{13}^n \mathbf{\tau}_s & \mathbf{0}_3 & \mathbf{\hat{C}}_b^n \mathbf{\tau}_s \\ \mathbf{F}_{21}^n \mathbf{\tau}_s & \mathbf{I}_3 + \mathbf{F}_{22}^n \mathbf{\tau}_s & \mathbf{F}_{23}'^n \mathbf{\tau}_s & \mathbf{\hat{C}}_b^n \mathbf{\tau}_s & \mathbf{0}_3 \\ \mathbf{0}_3 & \mathbf{F}_{32}^n \mathbf{\tau}_s & \mathbf{I}_3 + \mathbf{F}_{33}^n \mathbf{\tau}_s & \mathbf{0}_3 & \mathbf{0}_3 \\ \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{I}_3 & \mathbf{0}_3 \\ \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{I}_3 \end{bmatrix} \qquad \text{P. Groves}\,(14.72)
    Eigen::MatrixXd Phi = Eigen::MatrixXd::Identity(15, 15);

    Phi += F * tau_s;

    return Phi;
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
    double R_E = NAV::earthRadius_E(NAV::InsConst::WGS84_a, NAV::InsConst::WGS84_e_squared, latitude_b);
    double R_N = NAV::earthRadius_N(NAV::InsConst::WGS84_a, NAV::InsConst::WGS84_e_squared, latitude_b);

    F_12_n(0, 1) = -1 / (R_E + height_b);
    F_12_n(1, 0) = 1 / (R_N + height_b);
    F_12_n(2, 1) = std::tan(latitude_b) / (R_E + height_b);

    return F_12_n;
}

Eigen::Matrix3d NAV::LooselyCoupledKF::systemMatrixF_13_n(double latitude_b, double height_b, const Eigen::Vector3d& v_eb_n)
{
    // Math: \mathbf{F}_{13}^n = \begin{bmatrix} \omega_{ie}\sin{\hat{L}_b} & 0 & \frac{\hat{v}_{eb,E}^n}{(R_E(\hat{L}_b) + \hat{h}_b)^2} \\ 0 & 0 & \frac{-\hat{v}_{eb,N}^n}{(R_N(\hat{L}_b) + \hat{h}_b)^2} \\ \omega_{ie}\cos{\hat{L}_b} + \frac{\hat{v}_{eb,E}^n}{(R_E(\hat{L}_b) + \hat{h}_b)\cos^2{\hat{L}_b}} & 0 & \frac{-\hat{v}_{eb,E}^n\tan{\hat{L}_b}}{(R_E(\hat{L}_b) + \hat{h}_b)^2} \end{bmatrix} \qquad \text{P. Groves}\,(14.66)
    Eigen::Matrix3d F_13_n = Eigen::Matrix3d::Zero(3, 3);
    double R_E = NAV::earthRadius_E(NAV::InsConst::WGS84_a, NAV::InsConst::WGS84_e_squared, latitude_b);
    double R_N = NAV::earthRadius_N(NAV::InsConst::WGS84_a, NAV::InsConst::WGS84_e_squared, latitude_b);

    F_13_n(0, 0) = InsConst::angularVelocity_ie * std::sin(latitude_b);
    F_13_n(0, 2) = v_eb_n(1) / std::pow((R_E + height_b), 2.0);
    F_13_n(1, 2) = -v_eb_n(0) / std::pow((R_N + height_b), 2.0);
    F_13_n(2, 0) = InsConst::angularVelocity_ie * std::cos(latitude_b) + v_eb_n(1) / ((R_E + height_b) * cos(latitude_b) * cos(latitude_b));
    F_13_n(2, 2) = -v_eb_n(1) * std::tan(latitude_b) / std::pow((R_E + height_b), 2.0);

    return F_13_n;
}

Eigen::Matrix3d NAV::LooselyCoupledKF::systemMatrixF_21_n(const Eigen::Quaterniond& quaternion_nb, const Eigen::Vector3d& specForce_ib_b)
{
    // Math: \mathbf{F}_{21}^n = -\begin{bmatrix} (\mathbf{\hat{C}}_{b}^n \hat{f}_{ib}^b) \land \end{bmatrix} \qquad \text{P. Groves}\,(14.67)
    return -skewSymmetricMatrix(quaternion_nb * specForce_ib_b);
}

Eigen::Matrix3d NAV::LooselyCoupledKF::systemMatrixF_22_n(const Eigen::Vector3d& v_eb_n, double latitude_b, double height_b)
{
    // Math: \mathbf{F}_{22}^n = \begin{bmatrix} \frac{\hat{v}_{eb,D}^n}{R_N(\hat{L}_b)+\hat{h}_b} & -\frac{2\hat{v}_{eb,E}^n\tan{\hat{L}}_b}{R_E(\hat{L}_b)+\hat{h}_b}-2\omega_{ie}\sin{\hat{L}_b} & \frac{\hat{v}_{eb,N}^n}{R_N(\hat{L}_b)+\hat{h}_b} \\ \frac{\hat{v}_{eb,E}^n\tan{\hat{L}}_b}{R_E(\hat{L}_b)+\hat{h}_b}+2\omega_{ie}\sin{\hat{L}_b} & \frac{\hat{v}_{eb,N}^n\tan{\hat{L}}_b+\hat{v}_{eb,D}^n}{R_E(\hat{L}_b)+\hat{h}_b} & \frac{\hat{v}_{eb,E}^n}{R_E(\hat{L}_b)+\hat{h}_b}+2\omega_{ie}\cos{\hat{L}_b} \\ -\frac{2\hat{v}_{eb,N}^n}{R_N(\hat{L}_b)+\hat{h}_b} & -\frac{2\hat{v}_{eb,E}^n}{R_E(\hat{L}_b)+\hat{h}_b}-2\omega_{ie}\cos{\hat{L}_b} & 0 \end{bmatrix} \qquad \text{P. Groves}\,(14.68)
    Eigen::Matrix3d F_22_n = Eigen::Matrix3d::Zero(3, 3);
    double R_E = NAV::earthRadius_E(NAV::InsConst::WGS84_a, NAV::InsConst::WGS84_e_squared, latitude_b);
    double R_N = NAV::earthRadius_N(NAV::InsConst::WGS84_a, NAV::InsConst::WGS84_e_squared, latitude_b);

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
    double R_E = NAV::earthRadius_E(NAV::InsConst::WGS84_a, NAV::InsConst::WGS84_e_squared, latitude_b);
    double R_N = NAV::earthRadius_N(NAV::InsConst::WGS84_a, NAV::InsConst::WGS84_e_squared, latitude_b);
    // Magnitude of gravity vector at ellipsoid height in [m / s^2]
    double g_0 = NAV::gravity::gravityMagnitude_SomiglianaAltitude(latitude_b, 0); //TODO: Split calculation into latitude and altitude part to save time (--> InsGravity)
    // Geocentric Radius in [m]
    double r_eS_e = geocentricRadius(latitude_b, R_E, InsConst::WGS84_e_squared);

    F_23_n(0, 0) = -(v_eb_n(0) * v_eb_n(0) * std::pow(secant(latitude_b), 2.0) / (R_E + height_b)) - 2.0 * v_eb_n(1) * InsConst::angularVelocity_ie * std::cos(latitude_b);
    F_23_n(0, 2) = (v_eb_n(1) * v_eb_n(1) * std::tan(latitude_b)) / std::pow(R_E + height_b, 2.0) - (v_eb_n(0) * v_eb_n(2)) / std::pow(R_N + height_b, 2.0);
    F_23_n(1, 0) = (v_eb_n(0) * v_eb_n(1) * std::pow(secant(latitude_b), 2.0) / (R_E + height_b)) + 2.0 * v_eb_n(0) * InsConst::angularVelocity_ie * std::cos(latitude_b) - 2.0 * v_eb_n(2) * InsConst::angularVelocity_ie * std::sin(latitude_b);
    F_23_n(1, 2) = -(v_eb_n(0) * v_eb_n(1) * std::tan(latitude_b) + v_eb_n(1) * v_eb_n(2)) / std::pow(R_E + height_b, 2.0);
    F_23_n(2, 0) = 2.0 * v_eb_n(1) * InsConst::angularVelocity_ie * std::sin(latitude_b);
    F_23_n(2, 2) = (v_eb_n(1) * v_eb_n(1)) / std::pow(R_E + height_b, 2.0) + (v_eb_n(0) * v_eb_n(0)) / std::pow(R_N + height_b, 2.0) - 2.0 * g_0 / r_eS_e;

    return F_23_n;
}

Eigen::Matrix3d NAV::LooselyCoupledKF::systemMatrixF_32_n(double latitude_b, double height_b)
{
    // Math: \mathbf{F}_{32}^n = \begin{bmatrix} \frac{1}{R_N(\hat{L}_b) + \hat{h}_b} & 0 & 0 \\ 0 & \frac{1}{(R_E(\hat{L}_b) + \hat{h}_b)\cos{\hat{L}_b}} & 0 \\ 0 & 0 & -1 \end{bmatrix} \quad \text{P. Groves}\,(14.70)
    double R_E = NAV::earthRadius_E(NAV::InsConst::WGS84_a, NAV::InsConst::WGS84_e_squared, latitude_b);
    double R_N = NAV::earthRadius_N(NAV::InsConst::WGS84_a, NAV::InsConst::WGS84_e_squared, latitude_b);

    Eigen::DiagonalMatrix<double, 3> m(1.0 / (R_N + height_b),
                                       1.0 / (R_E + height_b) * std::cos(latitude_b),
                                       -1);
    return m;
}

Eigen::Matrix3d NAV::LooselyCoupledKF::systemMatrixF_33_n(const Eigen::Vector3d& v_eb_n, double latitude_b, double height_b)
{
    // Math: \mathbf{F}_{33}^n = \begin{bmatrix} 0 & 0 & -\frac{\hat{v}_{eb,N}^n}{(R_N(\hat{L}_b) + \hat{h}_b)^2} \\ \frac{\hat{v}_{eb,E}^n \sin{\hat{L}_b}}{(R_E(\hat{L}_b) + \hat{h}_b) \cos^2{\hat{L}_b}} & 0 & -\frac{\hat{v}_{eb,E}^n}{(R_N(\hat{L}_b) + \hat{h}_b)^2 \cos^2{\hat{L}_b}} \\ 0 & 0 & 0 \end{bmatrix} \quad \text{P. Groves}\,(14.71)
    Eigen::Matrix3d F_33_n = Eigen::Matrix3d::Zero(3, 3);
    double R_E = NAV::earthRadius_E(NAV::InsConst::WGS84_a, NAV::InsConst::WGS84_e_squared, latitude_b);
    double R_N = NAV::earthRadius_N(NAV::InsConst::WGS84_a, NAV::InsConst::WGS84_e_squared, latitude_b);

    F_33_n(0, 2) = -v_eb_n(0) / std::pow(R_N + height_b, 2.0);
    F_33_n(1, 0) = v_eb_n(1) * std::sin(latitude_b) / ((R_E + height_b) * std::pow(std::cos(latitude_b), 2.0));
    F_33_n(1, 2) = -v_eb_n(1) / (std::pow(R_E + height_b, 2.0) * std::cos(latitude_b));
    return F_33_n;
}

// ###########################################################################################################
//                                     System noise covariance matrix 𝐐
// ###########################################################################################################

Eigen::MatrixXd NAV::LooselyCoupledKF::systemNoiseCovarianceMatrix(const double& sigma2_ra, const double& sigma2_rg, const double& sigma2_bad, const double& sigma2_bgd, const Eigen::Matrix3d& F_21_n, const Eigen::Matrix3d& T_rn_p, const Eigen::Matrix3d& DCM_nb, const double& tau_s)
{
    return;
}

double NAV::LooselyCoupledKF::S_ra(const double& sigma2_ra, const double& tau_i)
{
    // Math: S_{ra} = \sigma_{ra}^2\tau_i \qquad \text{P. Groves}\,(14.83)
    return sigma2_ra * tau_i;
}

double NAV::LooselyCoupledKF::S_rg(const double& sigma2_rg, const double& tau_i)
{
    // Math: S_{rg} = \sigma_{rg}^2\tau_i \qquad \text{P. Groves}\,(14.83)
    return sigma2_rg * tau_i;
}

double NAV::LooselyCoupledKF::S_bad(const double& sigma2_bad, const double& tau_i)
{
    // Math: S_{bad} = \frac{\sigma_{bad}^2}{\tau_i} \qquad \text{P. Groves}\,(14.84)
    return sigma2_bad / tau_i;
}

double NAV::LooselyCoupledKF::S_bgd(const double& sigma2_bgd, const double& tau_i)
{
    // Math: S_{bgd} = \frac{\sigma_{bgd}^2}{\tau_i} \qquad \text{P. Groves}\,(14.84)
    return sigma2_bgd / tau_i;
}

Eigen::Matrix3d NAV::LooselyCoupledKF::T_rn_p(const Eigen::Vector3d& position_lla, const double& R_N, const double& R_E)
{
    return;
}

Eigen::Matrix3d NAV::LooselyCoupledKF::systemNoiseCovariance_11(const double& S_rg, const double& S_bgd, const double& tau_s)
{
    return;
}

Eigen::Matrix3d NAV::LooselyCoupledKF::systemNoiseCovariance_21(const double& S_rg, const double& S_bgd, const Eigen::Matrix3d& F_21_n, const double& tau_s)
{
    return;
}

Eigen::Matrix3d NAV::LooselyCoupledKF::systemNoiseCovariance_22(const double& S_ra, const double& S_bad, const double& S_rg, const double& S_bgd, const Eigen::Matrix3d& F_21_n, const double& tau_s)
{
    return;
}

Eigen::Matrix3d NAV::LooselyCoupledKF::systemNoiseCovariance_31(const double& S_rg, const double& S_bgd, const Eigen::Matrix3d& F_21_n, const Eigen::Matrix3d& T_rn_p, const double& tau_s)
{
    return;
}

Eigen::Matrix3d NAV::LooselyCoupledKF::systemNoiseCovariance_32(const double& S_ra, const double& S_bad, const double& S_rg, const double& S_bgd, const Eigen::Matrix3d& F_21_n, const Eigen::Matrix3d& T_rn_p, const double& tau_s)
{
    return;
}

Eigen::Matrix3d NAV::LooselyCoupledKF::systemNoiseCovariance_33(const double& S_ra, const double& S_bad, const double& S_rg, const double& S_bgd, const Eigen::Matrix3d& T_rn_p, const Eigen::Matrix3d& F_21_n, const double& tau_s)
{
    return;
}

Eigen::Matrix3d NAV::LooselyCoupledKF::systemNoiseCovariance_34(const double& S_bgd, const Eigen::Matrix3d& T_rn_p, const Eigen::Matrix3d& DCM_nb, const double& tau_s)
{
    return;
}

Eigen::Matrix3d NAV::LooselyCoupledKF::systemNoiseCovariance_35(const double& S_bgd, const Eigen::Matrix3d& F_21_n, const Eigen::Matrix3d& T_rn_p, const Eigen::Matrix3d& DCM_nb, const double& tau_s)
{
    return;
}

Eigen::Matrix3d NAV::LooselyCoupledKF::systemNoiseCovariance_42(const double& S_bad, const Eigen::Matrix3d& DCM_nb, const double& tau_s)
{
    return;
}

Eigen::Matrix3d NAV::LooselyCoupledKF::systemNoiseCovariance_44(const double& S_bad, const double& tau_s)
{
    return;
}

Eigen::Matrix3d NAV::LooselyCoupledKF::systemNoiseCovariance_51(const double& S_bgd, const Eigen::Matrix3d& DCM_nb, const double& tau_s)
{
    return;
}

Eigen::Matrix3d NAV::LooselyCoupledKF::systemNoiseCovariance_52(const double& S_bgd, const Eigen::Matrix3d& F_21_n, const Eigen::Matrix3d& DCM_nb, const double& tau_s)
{
    return;
}

Eigen::Matrix3d NAV::LooselyCoupledKF::systemNoiseCovariance_55(const double& S_bgd, const double& tau_s)
{
    return;
}