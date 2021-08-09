#include "LooselyCoupledKF.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>

#include "util/Logger.hpp"
#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"
#include "util/InsMechanization.hpp"
#include "util/InsConstants.hpp"
#include "util/InsMath.hpp"
#include "util/InsGravity.hpp"

NAV::LooselyCoupledKF::LooselyCoupledKF()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    Phi = Eigen::MatrixXd::Identity(15, 15);

    hasConfig = true;
    // guiConfigDefaultWindowSize = {};

    nm::CreateInputPin(this, "PosVelAtt (t0)", Pin::Type::Flow, { NAV::PosVelAtt::type() }, &LooselyCoupledKF::recvState__t0);

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

    posVelAtt__t0 = nullptr;

    LOG_DEBUG("LooselyCoupledKF initialized");

    return true;
}

void NAV::LooselyCoupledKF::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

void NAV::LooselyCoupledKF::recvState__t0(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId /*linkId*/) // NOLINT(readability-convert-member-functions-to-static)
{
    // TODO: cast nodeData as a dynamic pointer to state observation
    LOG_DATA("NodeData received: {}", nodeData);

    auto posVelAtt = std::dynamic_pointer_cast<PosVelAtt>(nodeData);

    posVelAtt__t0 = posVelAtt;

    filterObservation();
}

void NAV::LooselyCoupledKF::filterObservation()
{
    // Prediction ----------------------------------------------------------------------------------------------
    // 1. Calculate the transition matrix Phi_{k-1}

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
    invokeCallbacks(OutputPortIndex_PosVelAtt__t0, posVelAtt__t0);
}

// void NAV::LooselyCoupledKF::updatePhi(const Eigen::Quaterniond& quaternion_nb, const Eigen::Vector3d& acceleration_ib_b, const Eigen::Vector3d& position_lla, double tau_s)
// {
//     LOG_DATA("Values: {}\n{}\n{}\n{}", quaternion_nb, acceleration_ib_b, position_lla, tau_s);

//     Phi = Eigen::MatrixXd::Identity(15, 15);
// }

void NAV::LooselyCoupledKF::systemMatrix(const Eigen::Quaterniond& quaternion_nb, const Eigen::Vector3d& acceleration_ib_b, const Eigen::Vector3d& velocity_n, const Eigen::Vector3d& position_lla)
{
    Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Zero();

    Eigen::Vector3d angularRate_in_n(InsConst::angularVelocity_ie * std::cos(position_lla(0)),
                                     0,
                                     -InsConst::angularVelocity_ie * std::sin(position_lla(0)));

    F.block<3, 3>(0, 0) = systemMatrixF_11_n(angularRate_in_n);
    F.block<3, 3>(0, 3) = systemMatrixF_12_n(position_lla(0), position_lla(2));
    F.block<3, 3>(0, 6) = systemMatrixF_13_n(position_lla(0), position_lla(2), velocity_n);
    F.block<3, 3>(0, 12) = quaternion_nb.toRotationMatrix();
    F.block<3, 3>(3, 0) = systemMatrixF_21_n(quaternion_nb, acceleration_ib_b);
    F.block<3, 3>(3, 3) = systemMatrixF_22_n(velocity_n, position_lla(0), position_lla(2));
    F.block<3, 3>(3, 6) = systemMatrixF_23_n(velocity_n, position_lla(0), position_lla(2));
    F.block<3, 3>(3, 9) = quaternion_nb.toRotationMatrix();
    F.block<3, 3>(6, 3) = systemMatrixF_32_n(position_lla(0), position_lla(2));
    F.block<3, 3>(6, 6) = systemMatrixF_33_n(position_lla(0), position_lla(2), velocity_n);
}

Eigen::Matrix3d NAV::LooselyCoupledKF::systemMatrixF_11_n(const Eigen::Vector3d& angularRate_in_n)
{
    Eigen::Matrix3d F_11_n = Eigen::Matrix3d::Zero(3, 3);
    F_11_n(0, 1) = angularRate_in_n(2);
    F_11_n(0, 2) = -angularRate_in_n(1);
    F_11_n(1, 2) = angularRate_in_n(0);
    F_11_n(1, 0) = -angularRate_in_n(2);
    F_11_n(2, 0) = angularRate_in_n(1);
    F_11_n(2, 1) = -angularRate_in_n(0);

    return F_11_n;
}

Eigen::Matrix3d NAV::LooselyCoupledKF::systemMatrixF_12_n(double latitude_b, double height_b)
{
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
    return -skewSymmetricMatrix(quaternion_nb * specForce_ib_b);
}

Eigen::Matrix3d NAV::LooselyCoupledKF::systemMatrixF_22_n(const Eigen::Vector3d& v_eb_n, double latitude_b, double height_b)
{
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

Eigen::Matrix3d NAV::LooselyCoupledKF::systemMatrixF_33_n(double latitude_b, double height_b, const Eigen::Vector3d& v_eb_n)
{
    Eigen::Matrix3d F_33_n = Eigen::Matrix3d::Zero(3, 3);
    double R_E = NAV::earthRadius_E(NAV::InsConst::WGS84_a, NAV::InsConst::WGS84_e_squared, latitude_b);
    double R_N = NAV::earthRadius_N(NAV::InsConst::WGS84_a, NAV::InsConst::WGS84_e_squared, latitude_b);

    F_33_n(0, 2) = -v_eb_n(0) / std::pow(R_N + height_b, 2.0);
    F_33_n(1, 0) = v_eb_n(1) * std::sin(latitude_b) / ((R_E + height_b) * std::pow(std::cos(latitude_b), 2.0));
    F_33_n(1, 2) = -v_eb_n(1) / (std::pow(R_E + height_b, 2.0) * std::cos(latitude_b));
    return F_33_n;
}