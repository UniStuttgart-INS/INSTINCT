/// @file LooselyCoupledKF.hpp
/// @brief Kalman Filter class for the loosely coupled INS/GNSS integration
/// @author T. Topp (topp@ins.uni-stuttgart.de) and M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2021-08-04

#pragma once

#include "internal/Node/Node.hpp"
#include "NodeData/State/InertialNavSol.hpp"

#include "KalmanFilter.hpp"

namespace NAV
{
class LooselyCoupledKF : public Node
{
  public:
    /// @brief Default constructor
    LooselyCoupledKF();
    /// @brief Destructor
    ~LooselyCoupledKF() override;
    /// @brief Copy constructor
    LooselyCoupledKF(const LooselyCoupledKF&) = delete;
    /// @brief Move constructor
    LooselyCoupledKF(LooselyCoupledKF&&) = delete;
    /// @brief Copy assignment operator
    LooselyCoupledKF& operator=(const LooselyCoupledKF&) = delete;
    /// @brief Move assignment operator
    LooselyCoupledKF& operator=(LooselyCoupledKF&&) = delete;
    /// @brief String representation of the class type
    [[nodiscard]] static std::string typeStatic();

    /// @brief String representation of the class type
    [[nodiscard]] std::string type() const override;

    /// @brief String representation of the class category
    [[nodiscard]] static std::string category();

    /// @brief ImGui config window which is shown on double click
    /// @attention Don't forget to set hasConfig to true in the constructor of the node
    void guiConfig() override;

    /// @brief Saves the node into a json object
    [[nodiscard]] json save() const override;

    /// @brief Restores the node from a json object
    /// @param[in] j Json object with the node state
    void restore(const json& j) override;

  private:
    constexpr static size_t OutputPortIndex_PVAError = 0;  ///< @brief Flow (PVAError)
    constexpr static size_t OutputPortIndex_ImuBiases = 1; ///< @brief Flow (ImuBiases)
    constexpr static size_t OutputPortIndex_x = 2;         ///< @brief x̂ State vector
    constexpr static size_t OutputPortIndex_P = 3;         ///< @brief 𝐏 Error covariance matrix
    constexpr static size_t OutputPortIndex_Phi = 4;       ///< @brief 𝚽 State transition matrix
    constexpr static size_t OutputPortIndex_Q = 5;         ///< @brief 𝐐 System/Process noise covariance matrix
    constexpr static size_t OutputPortIndex_z = 6;         ///< @brief 𝐳 Measurement vector
    constexpr static size_t OutputPortIndex_H = 7;         ///< @brief 𝐇 Measurement sensitivity Matrix
    constexpr static size_t OutputPortIndex_R = 8;         ///< @brief 𝐑 = 𝐸{𝐰ₘ𝐰ₘᵀ} Measurement noise covariance matrix
    constexpr static size_t OutputPortIndex_K = 9;         ///< @brief 𝐊 Kalman gain matrix
    constexpr static size_t OutputPortIndex_Kz = 10;       ///< @brief 𝐊*𝐳 Kalman gain matrix * 𝐳 Measurement vector

    /// 𝐊*𝐳 Kalman gain matrix * 𝐳 Measurement vector
    Eigen::MatrixXd kalmanFilter_Kz;

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Receive Function for the intertial navigation solution
    /// @param[in] nodeData State vector (PosVelAtt)
    /// @param[in] linkId Id of the link over which the data is received
    void recvInertialNavigationSolution(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId linkId);

    /// @brief Receive Function for the GNSS navigation solution
    /// @param[in] nodeData State vector (PosVel)
    /// @param[in] linkId Id of the link over which the data is received
    void recvGNSSNavigationSolution(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId linkId);

    /// @brief Predicts the state from the InertialNavSol
    void looselyCoupledPrediction(const std::shared_ptr<InertialNavSol>& inertialNavSol);

    /// @brief Updates the predicted state from the InertialNavSol with the GNSS measurement
    void looselyCoupledUpdate(const std::shared_ptr<PosVelAtt>& gnssMeasurement);

    /// Latest Position, Velocity, Attitude and Imu observation
    std::shared_ptr<InertialNavSol> latestInertialNavSol = nullptr;

    /// Timestamp of the KF
    double tau_KF = 0.01;

    // TODO: Make Variance choosable from the GUI and adapt default values

    /// @brief 𝜎²_ra Variance of the noise on the accelerometer specific-force measurements [m²/s³]
    // double variance_ra = std::pow((0.04 /* [mg/√(Hz)] */) * 1e-3 * InsConst::G_NORM, 2);
    double variance_ra = std::pow((0.4 /* [mg/√(Hz)] */) * 1e-3 * InsConst::G_NORM, 2);

    /// @brief 𝜎²_rg Variance of the noise on the gyro angular-rate measurements [deg²/s]
    /// @note See Woodman (2007) Chp. 3.2.2 - eq. 7 with seconds instead of hours.
    ///       Value from Brown (2012) table 9.3 for 'High quality'
    // double variance_rg = std::pow(1 / 3600.0 * (1e-3 /* [deg/s/√(Hz)] */), 2);
    double variance_rg = std::pow(10 / 3600.0 * (1e-3 /* [deg/s/√(Hz)] */), 2);

    /// @brief 𝜎²_bad Variance of the accelerometer dynamic bias
    /// @note Value from VN-310 Datasheet (In-Run Bias Stability (Allan Variance))
    // double variance_bad = std::pow((10 /* [µg] */) * 1e-6 * InsConst::G_NORM, 2);
    double variance_bad = std::pow((100 /* [µg] */) * 1e-6 * InsConst::G_NORM, 2);

    /// @brief 𝜎²_bgd Variance of the gyro dynamic bias
    /// @note Value from VN-310 Datasheet (In-Run Bias Stability (Allan Variance))
    // double variance_bgd = std::pow((1 /* [°/h] */) / 3600.0, 2);
    double variance_bgd = std::pow((10 /* [°/h] */) / 3600.0, 2);

    /// Lever arm between INS and GNSS in [m, m, m]
    Eigen::Vector3d leverArm_InsGnss{ 0.0, 0.0, 0.0 };

    /// Kalman Filter representation
    KalmanFilter kalmanFilter{ 15, 6 };

    /// σ² Standard deviation of the GPS LatLonAlt position [rad^2, rad^2, m^2]
    Eigen::Vector3d gnssSigmaSquaredLatLonAlt;
    /// σ² Standard deviation of the GPS NED velocity [m^2/s^2]
    Eigen::Vector3d gnssSigmaSquaredVelocity;

    // ###########################################################################################################
    //                                                Prediction
    // ###########################################################################################################

    // ###########################################################################################################
    //                                           Transition matrix 𝚽
    // ###########################################################################################################

    // Transition matrix 𝚽 and system-submatrices 𝐅_ij --------------------------------------------------------
    /// @brief Updates the state transition matrix 𝚽 limited to first order in 𝐅𝜏ₛ
    /// @param[in] quaternion_nb Attitude of the body with respect to n-system
    /// @param[in] specForce_ib_b Specific force of the body with respect to inertial frame in [m / s^2], resolved in body coord.
    /// @param[in] velocity_n Velocity in n-system in [m / s]
    /// @param[in] position_lla Position as Lat Lon Alt in [rad rad m]
    /// @param[in] tau_s time interval in [s]
    /// @note See Groves (2013) chapter 14.2.4, equations (14.63) and (14.72)
    static Eigen::MatrixXd transitionMatrix(const Eigen::Quaterniond& quaternion_nb, const Eigen::Vector3d& specForce_ib_b, const Eigen::Vector3d& velocity_n, const Eigen::Vector3d& position_lla, double tau_s);

    /// @brief Submatrix 𝐅_11 of the system matrix 𝐅
    /// @param[in] angularRate_in_n Angular rate vector of the n-system with respect to the i-system in [rad / s], resolved in the n-system
    /// @return The 3x3 matrix 𝐅_11
    /// @note See Groves (2013) equation (14.64)
    static Eigen::Matrix3d systemMatrixF_11_n(const Eigen::Vector3d& angularRate_in_n);

    /// @brief Submatrix 𝐅_12 of the system matrix 𝐅
    /// @param[in] latitude_b Geodetic latitude of the body in [rad]
    /// @param[in] height_b Geodetic height of the body in [m]
    /// @return The 3x3 matrix 𝐅_12
    /// @note See Groves (2013) equation (14.65)
    static Eigen::Matrix3d systemMatrixF_12_n(double latitude_b, double height_b);

    /// @brief Submatrix 𝐅_13 of the system matrix 𝐅
    /// @param[in] latitude_b Geodetic latitude of the body in [rad]
    /// @param[in] height_b Geodetic height of the body in [m]
    /// @param[in] v_eb_n Velocity of the body with respect to the e-system in [m / s], resolved in the n-system
    /// @return The 3x3 matrix 𝐅_13
    /// @note See Groves (2013) equation (14.66)
    static Eigen::Matrix3d systemMatrixF_13_n(double latitude_b, double height_b, const Eigen::Vector3d& v_eb_n);

    /// @brief Submatrix 𝐅_21 of the system matrix 𝐅
    /// @param[in] quaternion_nb Attitude of the body with respect to n-system
    /// @param[in] specForce_ib_b Specific force of the body with respect to inertial frame in [m / s^2], resolved in body coord.
    /// @return The 3x3 matrix 𝐅_21
    /// @note See Groves (2013) equation (14.67)
    static Eigen::Matrix3d systemMatrixF_21_n(const Eigen::Quaterniond& quaternion_nb, const Eigen::Vector3d& specForce_ib_b);

    /// @brief Submatrix 𝐅_22 of the system matrix 𝐅
    /// @param[in] v_eb_n Velocity of the body with respect to the e-system in [m / s], resolved in the n-system
    /// @param[in] latitude_b Geodetic latitude of the body in [rad]
    /// @param[in] height_b Geodetic height of the body in [m]
    /// @return The 3x3 matrix 𝐅_22
    /// @note See Groves (2013) equation (14.68)
    static Eigen::Matrix3d systemMatrixF_22_n(const Eigen::Vector3d& v_eb_n, double latitude_b, double height_b);

    /// @brief Submatrix 𝐅_23 of the system matrix 𝐅
    /// @param[in] v_eb_n Velocity of the body with respect to the e-system in [m / s], resolved in the n-system
    /// @param[in] latitude_b Geodetic latitude of the body in [rad]
    /// @param[in] height_b Geodetic height of the body in [m]
    /// @return The 3x3 matrix 𝐅_23
    /// @note See Groves (2013) equation (14.69)
    static Eigen::Matrix3d systemMatrixF_23_n(const Eigen::Vector3d& v_eb_n, double latitude_b, double height_b);

    /// @brief Submatrix 𝐅_32 of the system matrix 𝐅
    /// @param[in] latitude_b Geodetic latitude of the body in [rad]
    /// @param[in] height_b Geodetic height of the body in [m]
    /// @return The 3x3 matrix 𝐅_32
    /// @note See Groves (2013) equation (14.70)
    static Eigen::Matrix3d systemMatrixF_32_n(double latitude_b, double height_b);

    /// @brief Submatrix 𝐅_33 of the system matrix 𝐅
    /// @param[in] v_eb_n Velocity of the body with respect to the e-system in [m / s], resolved in the n-system
    /// @param[in] latitude_b Geodetic latitude of the body in [rad]
    /// @param[in] height_b Geodetic height of the body in [m]
    /// @return The 3x3 matrix 𝐅_33
    /// @note See Groves (2013) equation (14.71)
    static Eigen::Matrix3d systemMatrixF_33_n(const Eigen::Vector3d& v_eb_n, double latitude_b, double height_b);

    // ###########################################################################################################
    //                                     System noise covariance matrix 𝐐
    // ###########################################################################################################

    /// @brief System noise covariance matrix 𝐐_{k-1}
    /// @param[in] sigma2_ra Variance of the noise on the accelerometer specific-force measurements
    /// @param[in] sigma2_rg Variance of the noise on the gyro angular-rate measurements
    /// @param[in] sigma2_bad Variance of the accelerometer dynamic bias
    /// @param[in] sigma2_bgd Variance of the gyro dynamic bias
    /// @param[in] F_21_n Submatrix 𝐅_21 of the system matrix 𝐅
    /// @param[in] T_rn_p Conversion matrix between cartesian and curvilinear perturbations to the position
    /// @param[in] DCM_nb Direction Cosine Matrix from body to navigation coordinates
    /// @param[in] tau_s Time interval in [s]
    /// @return The 15x15 matrix of system noise covariances
    static Eigen::Matrix<double, 15, 15> systemNoiseCovarianceMatrix(const double& sigma2_ra, const double& sigma2_rg, const double& sigma2_bad, const double& sigma2_bgd, const Eigen::Matrix3d& F_21_n, const Eigen::Matrix3d& T_rn_p, const Eigen::Matrix3d& DCM_nb, const double& tau_s);

    /// @brief S_ra Power Spectral Density of the accelerometer random noise
    /// @param[in] sigma2_ra 𝜎²_ra standard deviation of the noise on the accelerometer specific-force measurements in [m/s^2]
    /// @param[in] tau_i 𝜏ᵢ interval between the input of successive accelerometer outputs to the inertial navigation equations in [s]
    /// @note See P. Groves (2013) - Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems (ch. 14.2.6)
    [[nodiscard]] static double psdGyroNoise(const double& sigma2_ra, const double& tau_i);

    /// @brief S_rg Power Spectral Density of the gyroscope random noise
    /// @param[in] sigma2_rg 𝜎²_rg standard deviation of the noise on the gyroscope angular-rate measurements in [rad/s]
    /// @param[in] tau_i 𝜏ᵢ interval between the input of successive gyroscope outputs to the inertial navigation equations in [s]
    /// @note See P. Groves (2013) - Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems (ch. 14.2.6)
    [[nodiscard]] static double psdAccelNoise(const double& sigma2_rg, const double& tau_i);

    /// @brief S_bad Power Spectral Density of the accelerometer bias variation
    /// @param[in] sigma2_bad 𝜎²_bad standard deviation of the accelerometer dynamic bias [m/s^2]
    /// @param[in] tau_i 𝜏ᵢ interval between the input of successive accelerometer outputs to the inertial navigation equations in [s]
    /// @note See P. Groves (2013) - Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems (ch. 14.2.6)
    [[nodiscard]] static double psdAccelBiasVariation(const double& sigma2_bad, const double& tau_i);

    /// @brief S_bgd Power Spectral Density of the gyroscope bias variation
    /// @param[in] sigma2_bgd 𝜎²_bgd standard deviation of the gyroscope dynamic bias [rad/s]
    /// @param[in] tau_i 𝜏ᵢ interval between the input of successive gyroscope outputs to the inertial navigation equations in [s]
    /// @note See P. Groves (2013) - Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems (ch. 14.2.6)
    [[nodiscard]] static double psdGyroBiasVariation(const double& sigma2_bgd, const double& tau_i);

    /// @brief Conversion matrix between cartesian and curvilinear perturbations to the position
    /// @param[in] position_lla Position as Lat Lon Alt in [rad rad m]
    /// @param[in] R_N Meridian radius of curvature in [m]
    /// @param[in] R_E Prime vertical radius of curvature (East/West) [m]
    /// @return A 3x3 matrix
    static Eigen::Matrix3d conversionMatrixCartesianCurvilinear(const Eigen::Vector3d& position_lla, const double& R_N, const double& R_E);

    /// @brief Submatrix 𝐐_11 of the system noise covariance matrix 𝐐
    /// @param[in] S_rg Power Spectral Density of the gyroscope random noise
    /// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
    /// @param[in] tau_s Time interval in [s]
    /// @return The 3x3 matrix 𝐐_11
    /// @note See Groves (2013) equation (14.81)
    static Eigen::Matrix3d systemNoiseCovariance_11(const double& S_rg, const double& S_bgd, const double& tau_s);

    /// @brief Submatrix 𝐐_21 of the system noise covariance matrix 𝐐
    /// @param[in] S_rg Power Spectral Density of the gyroscope random noise
    /// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
    /// @param[in] F_21_n Submatrix 𝐅_21 of the system matrix 𝐅
    /// @param[in] tau_s Time interval in [s]
    /// @return The 3x3 matrix 𝐐_21
    /// @note See Groves (2013) equation (14.81)
    static Eigen::Matrix3d systemNoiseCovariance_21(const double& S_rg, const double& S_bgd, const Eigen::Matrix3d& F_21_n, const double& tau_s);

    /// @brief Submatrix 𝐐_22 of the system noise covariance matrix 𝐐
    /// @param[in] S_ra Power Spectral Density of the accelerometer random noise
    /// @param[in] S_bad Power Spectral Density of the accelerometer bias variation
    /// @param[in] S_rg Power Spectral Density of the gyroscope random noise
    /// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
    /// @param[in] F_21_n Submatrix 𝐅_21 of the system matrix 𝐅
    /// @param[in] tau_s Time interval in [s]
    /// @return The 3x3 matrix 𝐐_22
    /// @note See Groves (2013) equation (14.81)
    static Eigen::Matrix3d systemNoiseCovariance_22(const double& S_ra, const double& S_bad, const double& S_rg, const double& S_bgd, const Eigen::Matrix3d& F_21_n, const double& tau_s);

    /// @brief Submatrix 𝐐_25 of the system noise covariance matrix 𝐐
    /// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
    /// @param[in] F_21_n Submatrix 𝐅_21 of the system matrix 𝐅
    /// @param[in] DCM_nb Direction Cosine Matrix from body to navigation coordinates
    /// @param[in] tau_s Time interval in [s]
    /// @return The 3x3 matrix 𝐐_25
    /// @note See Groves (2013) equation (14.80)
    static Eigen::Matrix3d systemNoiseCovariance_25(const double& S_bgd, const Eigen::Matrix3d& F_21_n, const Eigen::Matrix3d& DCM_nb, const double& tau_s);

    /// @brief Submatrix 𝐐_31 of the system noise covariance matrix 𝐐
    /// @param[in] S_rg Power Spectral Density of the gyroscope random noise
    /// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
    /// @param[in] F_21_n Submatrix 𝐅_21 of the system matrix 𝐅
    /// @param[in] T_rn_p Conversion matrix between cartesian and curvilinear perturbations to the position
    /// @param[in] tau_s Time interval in [s]
    /// @return The 3x3 matrix 𝐐_31
    /// @note See Groves (2013) equation (14.81)
    static Eigen::Matrix3d systemNoiseCovariance_31(const double& S_rg, const double& S_bgd, const Eigen::Matrix3d& F_21_n, const Eigen::Matrix3d& T_rn_p, const double& tau_s);

    /// @brief Submatrix 𝐐_32 of the system noise covariance matrix 𝐐
    /// @param[in] S_ra Power Spectral Density of the accelerometer random noise
    /// @param[in] S_bad Power Spectral Density of the accelerometer bias variation
    /// @param[in] S_rg Power Spectral Density of the gyroscope random noise
    /// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
    /// @param[in] F_21_n Submatrix 𝐅_21 of the system matrix 𝐅
    /// @param[in] T_rn_p Conversion matrix between cartesian and curvilinear perturbations to the position
    /// @param[in] tau_s Time interval in [s]
    /// @return The 3x3 matrix 𝐐_32
    /// @note See Groves (2013) equation (14.81)
    static Eigen::Matrix3d systemNoiseCovariance_32(const double& S_ra, const double& S_bad, const double& S_rg, const double& S_bgd, const Eigen::Matrix3d& F_21_n, const Eigen::Matrix3d& T_rn_p, const double& tau_s);

    /// @brief Submatrix 𝐐_33 of the system noise covariance matrix 𝐐
    /// @param[in] S_ra Power Spectral Density of the accelerometer random noise
    /// @param[in] S_bad Power Spectral Density of the accelerometer bias variation
    /// @param[in] S_rg Power Spectral Density of the gyroscope random noise
    /// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
    /// @param[in] T_rn_p Conversion matrix between cartesian and curvilinear perturbations to the position
    /// @param[in] F_21_n Submatrix 𝐅_21 of the system matrix 𝐅
    /// @param[in] tau_s Time interval in [s]
    /// @return The 3x3 matrix 𝐐_33
    /// @note See Groves (2013) equation (14.81)
    static Eigen::Matrix3d systemNoiseCovariance_33(const double& S_ra, const double& S_bad, const double& S_rg, const double& S_bgd, const Eigen::Matrix3d& T_rn_p, const Eigen::Matrix3d& F_21_n, const double& tau_s);

    /// @brief Submatrix 𝐐_34 of the system noise covariance matrix 𝐐
    /// @param[in] S_bad Power Spectral Density of the accelerometer bias variation
    /// @param[in] T_rn_p Conversion matrix between cartesian and curvilinear perturbations to the position
    /// @param[in] DCM_nb Direction Cosine Matrix from body to navigation coordinates
    /// @param[in] tau_s Time interval in [s]
    /// @return The 3x3 matrix 𝐐_34
    /// @note See Groves (2013) equation (14.81)
    static Eigen::Matrix3d systemNoiseCovariance_34(const double& S_bad, const Eigen::Matrix3d& T_rn_p, const Eigen::Matrix3d& DCM_nb, const double& tau_s);

    /// @brief Submatrix 𝐐_35 of the system noise covariance matrix 𝐐
    /// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
    /// @param[in] F_21_n Submatrix 𝐅_21 of the system matrix 𝐅
    /// @param[in] T_rn_p Conversion matrix between cartesian and curvilinear perturbations to the position
    /// @param[in] DCM_nb Direction Cosine Matrix from body to navigation coordinates
    /// @param[in] tau_s Time interval in [s]
    /// @return The 3x3 matrix 𝐐_35
    /// @note See Groves (2013) equation (14.81)
    static Eigen::Matrix3d systemNoiseCovariance_35(const double& S_bgd, const Eigen::Matrix3d& F_21_n, const Eigen::Matrix3d& T_rn_p, const Eigen::Matrix3d& DCM_nb, const double& tau_s);

    /// @brief Submatrix 𝐐_42 of the system noise covariance matrix 𝐐
    /// @param[in] S_bad Power Spectral Density of the accelerometer bias variation
    /// @param[in] DCM_nb Direction Cosine Matrix from body to navigation coordinates
    /// @param[in] tau_s Time interval in [s]
    /// @return The 3x3 matrix 𝐐_42
    /// @note See Groves (2013) equation (14.80)
    static Eigen::Matrix3d systemNoiseCovariance_42(const double& S_bad, const Eigen::Matrix3d& DCM_nb, const double& tau_s);

    /// @brief Submatrix 𝐐_44 of the system noise covariance matrix 𝐐
    /// @param[in] S_bad Power Spectral Density of the accelerometer bias variation
    /// @param[in] tau_s Time interval in [s]
    /// @return The 3x3 matrix 𝐐_44
    /// @note See Groves (2013) equation (14.80)
    static Eigen::Matrix3d systemNoiseCovariance_44(const double& S_bad, const double& tau_s);

    /// @brief Submatrix 𝐐_51 of the system noise covariance matrix 𝐐
    /// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
    /// @param[in] DCM_nb Direction Cosine Matrix from body to navigation coordinates
    /// @param[in] tau_s Time interval in [s]
    /// @return The 3x3 matrix 𝐐_51
    /// @note See Groves (2013) equation (14.80)
    static Eigen::Matrix3d systemNoiseCovariance_51(const double& S_bgd, const Eigen::Matrix3d& DCM_nb, const double& tau_s);

    /// @brief Submatrix 𝐐_55 of the system noise covariance matrix 𝐐
    /// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
    /// @param[in] tau_s Time interval in [s]
    /// @return The 3x3 matrix 𝐐_55
    /// @note See Groves (2013) equation (14.80)
    static Eigen::Matrix3d systemNoiseCovariance_55(const double& S_bgd, const double& tau_s);

    // ###########################################################################################################
    //                                                Correction
    // ###########################################################################################################

    /// @brief Measurement matrix for GNSS measurements at timestep k, represented in navigation coordinates
    /// @param[in] T_rn_p Conversion matrix between cartesian and curvilinear perturbations to the position
    /// @param[in] DCM_nb Direction Cosine Matrix from body to navigation coordinates
    /// @param[in] angularRate_ib_b Angular rate of body with respect to inertial system in body coordinates in [rad/s]
    /// @param[in] leverArm_InsGnss l_{ba}^b lever arm from the INS to the GNSS antenna [m]
    /// @param[in] Omega_ie_n Skew-symmetric matrix of the Earth-rotation vector in local navigation frame axes
    /// @return The 6x15 measurement matrix 𝐇
    static Eigen::Matrix<double, 6, 15> measurementMatrix(const Eigen::Matrix3d& T_rn_p, const Eigen::Matrix3d& DCM_nb, const Eigen::Vector3d& angularRate_ib_b, const Eigen::Vector3d& leverArm_InsGnss, const Eigen::Matrix3d& Omega_ie_n);

    /// @brief Submatrix 𝐇_r1 of the measurement sensitivity matrix 𝐇
    /// @param[in] T_rn_p Conversion matrix between cartesian and curvilinear perturbations to the position
    /// @param[in] DCM_nb Direction Cosine Matrix from body to navigation coordinates
    /// @param[in] leverArm_InsGnss l_{ba}^b lever arm from the INS to the GNSS antenna [m]
    /// @return The 3x3 matrix 𝐇_r1
    static Eigen::Matrix3d measurementMatrix_r1_n(const Eigen::Matrix3d& T_rn_p, const Eigen::Matrix3d& DCM_nb, const Eigen::Vector3d& leverArm_InsGnss);

    /// @brief Submatrix 𝐇_v1 of the measurement sensitivity matrix 𝐇
    /// @param[in] DCM_nb Direction Cosine Matrix from body to navigation coordinates
    /// @param[in] angularRate_ib_b Angular rate of body with respect to inertial system in body coordinates in [rad/s]
    /// @param[in] leverArm_InsGnss l_{ba}^b lever arm from the INS to the GNSS antenna [m]
    /// @param[in] Omega_ie_n Skew-symmetric matrix of the Earth-rotation vector in local navigation frame axes
    /// @return The 3x3 matrix 𝐇_v1
    static Eigen::Matrix3d measurementMatrix_v1_n(const Eigen::Matrix3d& DCM_nb, const Eigen::Vector3d& angularRate_ib_b, const Eigen::Vector3d& leverArm_InsGnss, const Eigen::Matrix3d& Omega_ie_n);

    /// @brief Submatrix 𝐇_v5 of the measurement sensitivity matrix 𝐇
    /// @param[in] DCM_nb Direction Cosine Matrix from body to navigation coordinates
    /// @param[in] leverArm_InsGnss l_{ba}^b lever arm from the INS to the GNSS antenna [m]
    /// @return The 3x3 matrix 𝐇_v5
    static Eigen::Matrix3d measurementMatrix_v5_n(const Eigen::Matrix3d& DCM_nb, const Eigen::Vector3d& leverArm_InsGnss);

    /// @brief Measurement noise covariance matrix 𝐑
    /// @param[in] gnssVarianceLatLonAlt Variances of the position LLA in [rad² rad² m²]
    /// @param[in] gnssVarianceVelocity Variances of the velocity in [m² m² m²]
    /// @return The 6x6 measurement covariance matrix 𝐑
    static Eigen::Matrix<double, 6, 6> measurementNoiseCovariance(const Eigen::Vector3d& gnssVarianceLatLonAlt, const Eigen::Vector3d& gnssVarianceVelocity);

    /// @brief Measurement innovation vector 𝜹𝐳
    /// @param[in] positionMeasurement_lla Position measurement as Lat Lon Alt in [rad rad m]
    /// @param[in] positionEstimate_n Position estimate as Lat Lon Alt in [rad rad m]
    /// @param[in] velocityMeasurement_n Velocity measurement in the n frame in [m/s]
    /// @param[in] velocityEstimate_n Velocity estimate in the n frame in [m/s]
    /// @param[in] T_rn_p Conversion matrix between cartesian and curvilinear perturbations to the position
    /// @param[in] q_nb Rotation quaternion from body to navigation coordinates
    /// @param[in] leverArm_InsGnss l_{ba}^b lever arm from the INS to the GNSS antenna [m]
    /// @param[in] angularRate_ib_b Angular rate of body with respect to inertial system in body coordinates in [rad/s]
    /// @param[in] Omega_ie_n Skew-symmetric matrix of the Earth-rotation vector in local navigation frame axes
    /// @return The 6x1 measurement innovation vector 𝜹𝐳
    static Eigen::Matrix<double, 6, 1> measurementInnovation(const Eigen::Vector3d& positionMeasurement_lla, const Eigen::Vector3d& positionEstimate_n,
                                                             const Eigen::Vector3d& velocityMeasurement_n, const Eigen::Vector3d& velocityEstimate_n,
                                                             const Eigen::Matrix3d& T_rn_p, const Eigen::Quaterniond& q_nb, const Eigen::Vector3d& leverArm_InsGnss,
                                                             const Eigen::Vector3d& angularRate_ib_b, const Eigen::Matrix3d& Omega_ie_n);
};
} // namespace NAV